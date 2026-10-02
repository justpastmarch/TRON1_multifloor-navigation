"""Causal, bounded adapter around the frozen gyro/GICP baseline. No ROS or socket.

Only the worker mutates the estimator. Sensor callbacks enqueue immutable inputs;
reset epochs invalidate in-flight results as well as previously published poses.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from concurrent.futures import Future
import hashlib
import math
from pathlib import Path
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .lidar_tracking_core import (
    ImuRotation, RegistrationConfig, register_scan_to_map, voxel_downsample_points,
)


@dataclass(frozen=True)
class TrackingSettings:
    frame_step: int = 2
    local_map_scans: int = 15
    max_range_m: float = 20.0
    imu_max_gap_s: float = 0.03
    gravity_half_window_s: float = 0.5
    imu_buffer_sec: float = 8.0
    imu_capacity: int = 4096
    queue_capacity: int = 8
    imu_wait_sec: float = 0.15
    registration: RegistrationConfig = RegistrationConfig(0.2, 0.8, 30, 0.35, 0.5, 0.35)
    rotational_deskew: bool = False
    control_geometry_points: bool = False


@dataclass(frozen=True)
class TrackingSample:
    epoch: int
    sequence: int
    stamp: float
    received_at: float
    measured_at: float
    completed_at: float
    last_geometry_at: float | None
    transform: tuple | None
    geometry_valid: bool
    state: str
    reason: str
    fitness: float | None = None
    rmse: float | None = None
    translation_innovation: float | None = None
    rotation_innovation: float | None = None
    prediction_source: str = "none"
    compute_sec: float = 0.0
    calibration_hash: str = ""
    processing_started_at: float | None = None
    scan_span_sec: float = 0.
    deskew_state: str = "disabled"
    deskew_compute_sec: float = 0.
    display_points: tuple | None = None


def load_imu_rotation(path):
    content = Path(path).read_bytes()
    matrix = np.asarray(yaml.safe_load(content)["mapping"]["extrinsic_R"], dtype=float).reshape(3, 3)
    if (not np.isfinite(matrix).all() or
            not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-3) or
            np.linalg.det(matrix) <= 0):
        raise ValueError("invalid calibrated LiDAR-to-IMU rotation")
    return Rotation.from_matrix(matrix).as_matrix().T, hashlib.sha256(content).hexdigest()


def points_from_livox(message):
    return np.fromiter((v for p in message.points for v in (p.x, p.y, p.z)),
                       dtype=float, count=len(message.points) * 3).reshape(-1, 3)


class TrackingEngine:
    """Single-consumer estimator also used by the sequential bag regression tool."""

    def __init__(self, settings, lidar_from_imu, calibration_hash="", epoch=0):
        self.settings = settings
        self.rotation = lidar_from_imu
        self.calibration_hash = calibration_hash
        self.epoch = epoch
        self.scans = deque(maxlen=settings.local_map_scans)
        self.pose = np.eye(4)
        self.last_stamp = None
        self.last_geometry_at = None
        self.sequence = 0

    def process(self, stamp, points, imu_rows, received_at, measured_at, now=time.monotonic):
        started = now()
        settings = self.settings
        incoming = np.asarray(points, dtype=float)
        if incoming.ndim != 2 or incoming.shape[1] not in (3, 4):
            raise ValueError('scan needs XYZ or XYZ with acquisition offset')
        xyz = incoming[:, :3]
        offsets = incoming[:, 3] if incoming.shape[1] == 4 else None
        distance = np.einsum("ij,ij->i", xyz, xyz)
        offset_mask = np.isfinite(xyz).all(axis=1) & (distance >= .25) & (distance <= settings.max_range_m ** 2)
        xyz = xyz[offset_mask]
        bootstrap = self.last_stamp is None
        prediction = self.pose.copy()
        imu_error = ""
        imu = None
        try:
            imu = ImuRotation(np.asarray(imu_rows, dtype=float), self.rotation, settings.imu_max_gap_s)
        except ValueError as error:
            imu_error = str(error)
        deskew_state, deskew_sec, scan_span = 'disabled', 0., 0.
        if offsets is not None and np.isfinite(offsets).all() and np.all(offsets >= 0):
            scan_span = float(offsets.max(initial=0.))
        if settings.rotational_deskew:
            began = now()
            try:
                if imu is None or offsets is None:
                    raise ValueError(imu_error or 'point acquisition offsets missing')
                if not np.isfinite(offsets).all() or np.any(offsets < 0):
                    raise ValueError('invalid point acquisition offsets')
                scan_span = float(offsets.max(initial=0.))
                imu._support(stamp, stamp + scan_span)
                xyz = imu.deskew_to_start(xyz, offsets[offset_mask], stamp)
                deskew_state = 'rotation_to_scan_start'
            except ValueError as error:
                self.sequence += 1
                finished = now()
                return TrackingSample(self.epoch, self.sequence, stamp, received_at, measured_at,
                    finished, self.last_geometry_at,
                    tuple(self.pose.flat) if self.last_stamp is not None else None,
                    False, 'DEGRADED', 'deskew unavailable: '+str(error),
                    compute_sec=finished-started, calibration_hash=self.calibration_hash,
                    processing_started_at=started, scan_span_sec=scan_span,
                    deskew_state='unavailable', deskew_compute_sec=finished-began)
            deskew_sec = now()-began
        scan = voxel_downsample_points(xyz, settings.registration.voxel_size_m)
        source = "none" if bootstrap else "gyro"
        if not bootstrap:
            try:
                if imu is None:
                    raise ValueError(imu_error)
                prediction[:3, :3] = self.pose[:3, :3] @ imu.relative(self.last_stamp, stamp)
            except ValueError as error:
                source, imu_error = "last_observation", str(error)
        accepted = False
        reason = "insufficient lidar points"
        fitness = rmse = translation = rotation = None
        if len(scan) >= 3:
            if bootstrap:
                try:
                    if imu is None:
                        raise ValueError(imu_error)
                    if (imu.times[0] > stamp - settings.gravity_half_window_s or
                            imu.times[-1] < stamp + settings.gravity_half_window_s):
                        raise ValueError("waiting for complete causal gravity window")
                    candidate = imu.initial_transform(stamp, settings.gravity_half_window_s)
                    accepted, source, reason = True, "gravity", "gravity initialized; no scan match yet"
                except ValueError as error:
                    reason = str(error)
            else:
                result = register_scan_to_map(scan, np.vstack(self.scans), prediction, settings.registration)
                candidate = result.transform.copy()
                fitness, rmse = result.fitness, result.inlier_rmse_m
                translation, rotation = result.translation_step_m, result.rotation_step_rad
                accepted = result.accepted and np.isfinite(candidate).all()
                reason = ("geometry recovered without IMU: " + imu_error if accepted and imu_error
                          else "geometry accepted" if accepted else "registration rejected: " + imu_error)
        if accepted:
            self.pose = candidate
            self.last_stamp = stamp
            # BOOTSTRAP initializes the frame; it is not scan-match evidence.
            if not bootstrap:
                self.last_geometry_at = measured_at
            homogeneous = np.column_stack((scan, np.ones(len(scan))))
            self.scans.append((candidate @ homogeneous.T).T[:, :3])
        self.sequence += 1
        finished = now()
        return TrackingSample(
            self.epoch, self.sequence, stamp, received_at, measured_at, finished,
            self.last_geometry_at,
            tuple(self.pose.flat) if self.last_stamp is not None else None,
            accepted and not bootstrap,
            "BOOTSTRAP" if accepted and bootstrap else "TRACKED" if accepted else "DEGRADED",
            reason, fitness, rmse, translation, rotation, source,
            finished - started, self.calibration_hash,
            processing_started_at=started, scan_span_sec=scan_span,
            deskew_state=deskew_state, deskew_compute_sec=deskew_sec,
            display_points=(tuple(scan[::max(1, (len(scan)+4999)//5000)].flat)
                            if accepted and (settings.rotational_deskew or settings.control_geometry_points) else None),
        )


class TrackingWorker:
    """Bounded callbacks; GICP runs outside all locks used by command readers."""

    def __init__(self, settings, lidar_from_imu, calibration_hash="", clock=time.monotonic,
                 on_sample=None, engine_factory=TrackingEngine):
        self.settings, self.rotation, self.calibration_hash = settings, lidar_from_imu, calibration_hash
        self.clock, self.on_sample, self.engine_factory = clock, on_sample, engine_factory
        self._condition = threading.Condition()
        self._imu = deque(maxlen=settings.imu_capacity)
        self._pending = deque()
        self._epoch = 0
        self._latest = None
        self._last_lidar = None
        self._source_index = 0
        self._stop = False
        self._thread = None
        self.dropped_scans = self.rejected_imu = 0
        self.error = ""
        self._entry_jobs = deque(maxlen=1)

    @property
    def epoch(self):
        with self._condition:
            return self._epoch

    def snapshot(self):
        with self._condition:
            return self._latest

    def diagnostics(self):
        with self._condition:
            return dict(epoch=self._epoch, queue_length=len(self._pending),
                        dropped_scans=self.dropped_scans, rejected_imu=self.rejected_imu,
                        worker_error=self.error)

    def reset(self, reason="explicit reset"):
        with self._condition:
            self._epoch += 1
            self._latest = None
            self._pending.clear()
            while self._entry_jobs:
                self._entry_jobs.popleft()[1].cancel()
            self._imu.clear()
            self._last_lidar = None
            self._source_index = 0
            self.error = reason
            self._condition.notify_all()

    def push_imu(self, row):
        row = tuple(float(x) for x in row)
        with self._condition:
            if len(row) != 7 or not all(map(math.isfinite, row)):
                self.rejected_imu += 1
                return False
            if self._imu and row[0] <= self._imu[-1][0]:
                self.rejected_imu += 1
                return False
            self._imu.append(row)
            cutoff = row[0] - self.settings.imu_buffer_sec
            while len(self._imu) > 2 and self._imu[1][0] < cutoff:
                self._imu.popleft()
            self._condition.notify_all()
            return True

    def push_scan(self, stamp, points, received_at=None, measured_at=None):
        received_at = self.clock() if received_at is None else received_at
        measured_at = received_at if measured_at is None else measured_at
        if not all(map(math.isfinite, (stamp, received_at, measured_at))) or stamp <= 0:
            return False
        with self._condition:
            if self._last_lidar is not None and stamp <= self._last_lidar:
                if stamp < self._last_lidar:
                    self.reset("LiDAR clock reversed; anchor invalidated")
                else:
                    self.dropped_scans += 1
                    return False
            self._last_lidar = stamp
            selected = self._source_index % self.settings.frame_step == 0
            self._source_index += 1
            if not selected:
                return False
            if len(self._pending) >= self.settings.queue_capacity:
                self._pending.popleft()
                self.dropped_scans += 1
            points = np.array(points, dtype=float, copy=True)
            points.setflags(write=False)
            self._pending.append((self._epoch, stamp, points, received_at, measured_at))
            self._condition.notify_all()
            return True

    def start(self):
        with self._condition:
            if self._thread is not None:
                return
            self._thread = threading.Thread(target=self._run, name="stair-lidar", daemon=True)
            self._thread.start()

    def entry_job(self, operation):
        """Run entry registration on the estimator worker, never the command timer."""
        future=Future()
        with self._condition:
            if self._entry_jobs or self._stop:
                raise ValueError("entry registration already pending or worker stopped")
            self._entry_jobs.append((operation,future,self._epoch))
            self._condition.notify_all()
        return future

    def _run(self):
        engine = None
        while True:
            job=None
            with self._condition:
                if self._stop:
                    return
                if self._entry_jobs:
                    job=self._entry_jobs.popleft()
                if job is not None:
                    pass
                elif not self._pending:
                    self._condition.wait(0.05)
                    continue
            if job is not None:
                operation,future,epoch=job
                if not future.set_running_or_notify_cancel():
                    continue
                try:
                    with self._condition:
                        latest=self._latest
                    if (engine is None or engine.epoch != epoch or latest is None or
                            not latest.geometry_valid or not engine.scans or
                            engine.last_stamp != latest.stamp):
                        raise ValueError("entry registration has no accepted cloud")
                    value=operation(engine.scans[-1],latest)
                    with self._condition:
                        if self._epoch!=epoch or self._stop:
                            raise ValueError("entry epoch invalidated during registration")
                    future.set_result(value)
                except Exception as error:
                    future.set_exception(error)
                continue
            with self._condition:
                if self._stop:
                    return
                if not self._pending:
                    self._condition.wait(0.05)
                    continue
                item = self._pending[0]
                epoch, stamp, points, received_at, measured_at = item
                if engine is None or engine.epoch != epoch:
                    engine = self.engine_factory(self.settings, self.rotation, self.calibration_hash, epoch)
                acquisition_span = (float(np.max(points[:, 3], initial=0.))
                                    if self.settings.rotational_deskew and points.shape[1] == 4 else 0.)
                required = stamp + max(acquisition_span, self.settings.gravity_half_window_s if engine.last_stamp is None else 0)
                wait_budget = self.settings.imu_wait_sec + (self.settings.gravity_half_window_s if engine.last_stamp is None else 0)
                if (not self._imu or self._imu[-1][0] < required) and self.clock() - received_at < wait_budget:
                    self._condition.wait(0.01)
                    continue
                imu_rows = tuple(self._imu)
                self._pending.popleft()
            try:
                sample = engine.process(stamp, points, imu_rows, received_at, measured_at, self.clock)
            except Exception as error:
                with self._condition:
                    if epoch == self._epoch:
                        self.error = "%s: %s" % (type(error).__name__, error)
                        self._latest = None
                continue
            with self._condition:
                if epoch != self._epoch or self._stop:
                    continue
                self._latest = sample
                self.error = ""
            if self.on_sample is not None:
                try:
                    self.on_sample(sample)
                except Exception as error:
                    with self._condition:
                        self.error = "diagnostic callback: %s" % error

    def shutdown(self, timeout=5.0):
        with self._condition:
            self._stop = True
            self._pending.clear()
            while self._entry_jobs:
                self._entry_jobs.popleft()[1].cancel()
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise RuntimeError("LiDAR worker did not stop within shutdown budget")
