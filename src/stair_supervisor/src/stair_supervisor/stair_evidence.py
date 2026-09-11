"""Pure odometry evidence tracking for one stair traversal."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import threading
from typing import Final

from .configuration import StairProfile


class Phase(str, Enum):
    VERIFY_ENTRY = "VERIFY_ENTRY"
    ALIGN = "ALIGN"
    FORWARD_SEGMENT_1 = "FORWARD_SEGMENT_1"
    LANDING = "LANDING"
    TURN_TO_NEXT_FLIGHT = "TURN_TO_NEXT_FLIGHT"
    FORWARD_SEGMENT_2 = "FORWARD_SEGMENT_2"
    EXIT_CONFIRM = "EXIT_CONFIRM"


@dataclass(frozen=True)
class OdometrySample:
    stamp: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class EvidenceReport:
    phase: Phase
    complete: bool
    faulted: bool
    detail: str
    progress: float
    threshold: float
    freshness_sec: float


_DWELL_PHASES: Final = frozenset((Phase.LANDING, Phase.EXIT_CONFIRM))


def _angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class StairEvidenceTracker:
    """Mutable state machine that faults closed on unsafe sensor observations."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._profile: StairProfile | None = None
        self._started_at = 0.0
        self._phase = Phase.VERIFY_ENTRY
        self._odom: OdometrySample | None = None
        self._odom_seq = 0
        self._phase_odom_seq = 0
        self._baseline_x = 0.0
        self._baseline_y = 0.0
        self._baseline_yaw = 0.0
        self._cumulative_yaw = 0.0
        self._last_step_m = 0.0
        self._last_yaw_step_rad = 0.0
        self._dwell_started_at: float | None = None
        self._fault_reason: str | None = None

    def arm(self, profile: StairProfile, started_at: float) -> None:
        with self._lock:
            self._profile = profile
            self._started_at = started_at
            self._fault_reason = None
            self._cumulative_yaw = 0.0
            self._last_step_m = 0.0
            self._last_yaw_step_rad = 0.0
            self._dwell_started_at = None
            self._slope_entered = False
            self._slope_exited = False
            self._begin_phase_locked(Phase.VERIFY_ENTRY)

    def begin_phase(self, phase: Phase, _now: float) -> None:
        with self._lock:
            self._begin_phase_locked(phase)

    def _begin_phase_locked(self, phase: Phase) -> None:
        self._phase = phase
        self._phase_odom_seq = self._odom_seq
        self._dwell_started_at = None
        if self._odom is not None:
            self._baseline_x = self._odom.x_m
            self._baseline_y = self._odom.y_m
            self._baseline_yaw = self._cumulative_yaw

    def update_odometry(self, sample: OdometrySample) -> None:
        with self._lock:
            if not all(math.isfinite(value) for value in (sample.stamp, sample.x_m, sample.y_m, sample.yaw_rad)):
                self._fault("nonfinite odometry")
                return
            previous = self._odom
            if previous is not None:
                gap = sample.stamp - previous.stamp
                if gap <= 0.0:
                    self._fault("non-increasing odometry timestamp")
                    return
                step = math.hypot(sample.x_m - previous.x_m, sample.y_m - previous.y_m)
                yaw_step = _angle_delta(sample.yaw_rad, previous.yaw_rad)
                profile = self._profile
                if profile is not None and gap > profile.max_sample_gap_sec:
                    self._fault("odometry sample gap")
                    return
                if profile is not None and step > profile.max_odom_step_m:
                    self._fault("odometry step")
                    return
                if profile is not None and abs(yaw_step) > profile.max_yaw_step_rad:
                    self._fault("yaw step")
                    return
                self._last_step_m = step
                self._last_yaw_step_rad = abs(yaw_step)
                self._cumulative_yaw += yaw_step
                if (
                    profile is not None
                    and self._phase in _DWELL_PHASES
                    and (step > profile.distance_tolerance_m or abs(yaw_step) > profile.yaw_tolerance_rad)
                ):
                    self._dwell_started_at = None
            self._odom = sample
            self._odom_seq += 1

    def evaluate(self, phase: Phase, now: float) -> EvidenceReport:
        with self._lock:
            return self._evaluate_locked(phase, now)

    def _evaluate_locked(self, phase: Phase, now: float) -> EvidenceReport:
        profile = self._profile
        if profile is None:
            return self._report(phase, False, True, "tracker not armed", 0.0, 0.0, math.inf)
        if phase is not self._phase:
            self._fault("phase mismatch")
        if now - self._started_at > profile.timeout_sec:
            self._fault("profile timeout")
        if self._fault_reason is not None:
            return self._report(phase, False, True, self._fault_reason, 0.0, 0.0, math.inf)
        if self._odom is None:
            return self._report(phase, False, False, "waiting for odometry", 0.0, 0.0, math.inf)
        odom_age = now - self._odom.stamp
        freshness = odom_age
        if odom_age < 0.0:
            self._fault("future-dated sensor sample")
        elif freshness > profile.sensor_freshness_sec:
            self._fault("stale sensor sample")
        elif freshness > profile.max_sample_gap_sec:
            self._fault("sensor sample gap")
        if self._fault_reason is not None:
            return self._report(phase, False, True, self._fault_reason, 0.0, 0.0, freshness)

        handlers = {
            Phase.VERIFY_ENTRY: lambda: self._entry_report(phase, freshness),
            Phase.ALIGN: lambda: self._yaw_report(phase, profile.alignment_yaw_rad, freshness),
            Phase.FORWARD_SEGMENT_1: lambda: self._distance_report(phase, profile.flight_1_distance_m, freshness),
            Phase.LANDING: lambda: self._dwell_report(phase, profile.landing_dwell_sec, now, freshness),
            Phase.TURN_TO_NEXT_FLIGHT: lambda: self._yaw_report(phase, profile.landing_turn_yaw_rad, freshness),
            Phase.FORWARD_SEGMENT_2: lambda: self._distance_report(phase, profile.flight_2_distance_m, freshness),
            Phase.EXIT_CONFIRM: lambda: self._dwell_report(phase, profile.exit_dwell_sec, now, freshness),
        }
        return handlers[phase]()

    def _entry_report(self, phase: Phase, freshness: float) -> EvidenceReport:
        profile = self._profile
        assert profile is not None
        complete = self._new_odom() and self._stationary(profile)
        detail = "stationary entry" if complete else "waiting for stationary entry"
        return self._report(phase, complete, False, detail, 0.0, 0.0, freshness)

    def _new_odom(self) -> bool:
        return self._odom_seq > self._phase_odom_seq

    def _stationary(self, profile: StairProfile) -> bool:
        return self._last_step_m <= profile.distance_tolerance_m and self._last_yaw_step_rad <= profile.yaw_tolerance_rad

    def _yaw_report(self, phase: Phase, target: float, freshness: float) -> EvidenceReport:
        progress = math.copysign(1.0, target) * (self._cumulative_yaw - self._baseline_yaw)
        threshold = abs(target)
        profile = self._profile
        assert profile is not None
        if progress < -profile.yaw_tolerance_rad:
            self._fault("reverse yaw progress")
        complete = self._new_odom() and progress + profile.yaw_tolerance_rad >= threshold
        return self._report(phase, complete, self._fault_reason is not None, self._fault_reason or "yaw progress", progress, threshold, freshness)

    def _distance_report(self, phase: Phase, target: float, freshness: float) -> EvidenceReport:
        odom = self._odom
        profile = self._profile
        assert odom is not None and profile is not None
        heading = odom.yaw_rad - (self._cumulative_yaw - self._baseline_yaw)
        signed_distance = (odom.x_m - self._baseline_x) * math.cos(heading) + (odom.y_m - self._baseline_y) * math.sin(heading)
        progress = math.copysign(1.0, target) * signed_distance
        threshold = abs(target)
        if progress < -profile.distance_tolerance_m:
            self._fault("reverse distance progress")
        complete = self._new_odom() and progress + profile.distance_tolerance_m >= threshold
        detail = self._fault_reason or ("distance progress" if complete else "waiting for distance progress")
        return self._report(phase, complete, self._fault_reason is not None, detail, progress, threshold, freshness)

    def _dwell_report(self, phase: Phase, dwell: float, _now: float, freshness: float) -> EvidenceReport:
        profile = self._profile
        assert profile is not None
        assert self._odom is not None
        evidence_stamp = self._odom.stamp
        valid = self._new_odom() and self._stationary(profile)
        if not valid:
            self._dwell_started_at = None
            return self._report(phase, False, False, "waiting for stationary dwell", 0.0, dwell, freshness)
        if self._dwell_started_at is None:
            self._dwell_started_at = evidence_stamp
        progress = evidence_stamp - self._dwell_started_at
        complete = progress >= dwell
        return self._report(phase, complete, False, "stationary dwell", progress, dwell, freshness)

    def _fault(self, reason: str) -> None:
        if self._profile is not None and self._fault_reason is None:
            self._fault_reason = reason

    @staticmethod
    def _report(phase: Phase, complete: bool, faulted: bool, reason: str, progress: float, threshold: float, freshness: float) -> EvidenceReport:
        detail = f"progress={progress:.6f} threshold={threshold:.6f} freshness={freshness:.6f} reason={reason}"
        return EvidenceReport(phase, complete, faulted, detail, progress, threshold, freshness)
