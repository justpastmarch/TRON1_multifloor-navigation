"""Re-level LiDAR tracking at stair entry when it disagrees with IMU gravity.

Field 2026-10-16 10:50 (RF descent entry): after a long session the tracked
local frame reported 20 deg roll and drifted ~1.8 m in height on a flat roof,
while the camera and the IMU accelerometer showed the body level (|roll| < 2
deg). The tracker still reported TRACKED, so an entry anchor built on it would
inherit a tilted frame for every later height/support decision.

This adds no stop condition. Only when the robot is quasi-static at entry and
the tracked tilt disagrees with the IMU gravity tilt by more than a commissioned
gap is the tracker re-bootstrapped (new epoch, gravity window), before the
anchor is built. Otherwise nothing changes. Attitude limits are untouched.
"""
from __future__ import annotations

import math

import numpy as np

STANDARD_GRAVITY = 9.80665
# Healthy quasi-static field gap (10/16-11/06 bags): roll <= 0.10, pitch p99 0.17 rad;
# the corrupted 10:50 frame was 0.39 rad.
DEFAULT_GAP_RAD = 0.2
DEFAULT_WINDOW_SEC = 0.5
DEFAULT_NORM_TOLERANCE = 0.02   # |mean accel| / g - 1: averages out balancing, rejects sustained acceleration
DEFAULT_TIMEOUT_SEC = 4.0


def _roll_pitch(rotation):
    rotation = np.asarray(rotation, dtype=float)
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    pitch = math.asin(max(-1., min(1., -rotation[2, 0])))
    return roll, pitch


def tracked_body_tilt(sample_transform, base_from_lidar):
    """Body roll/pitch of a tracking sample in its gravity-initialized local frame."""
    local_lidar = np.asarray(sample_transform, dtype=float).reshape(4, 4)
    body = local_lidar @ np.linalg.inv(np.asarray(base_from_lidar, dtype=float))
    return _roll_pitch(body[:3, :3])


def gravity_body_tilt(rows, lidar_from_imu, base_from_lidar, *, window_sec=DEFAULT_WINDOW_SEC,
                      norm_tolerance=DEFAULT_NORM_TOLERANCE):
    """Body roll/pitch from the mean IMU specific force over the latest window.

    rows: (stamp, gx, gy, gz, ax, ay, az). Returns None unless the window is
    populated and quasi-static (mean |a| within norm_tolerance of 1 g).
    """
    rows = np.asarray(rows, dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 7 or len(rows) < 10:
        return None
    recent = rows[rows[:, 0] >= rows[-1, 0] - window_sec]
    if len(recent) < 10:
        return None
    mean = recent[:, 4:7].mean(axis=0)
    gravity = STANDARD_GRAVITY if float(np.median(np.linalg.norm(rows[:, 4:7], axis=1))) > 5. else 1.
    if abs(float(np.linalg.norm(mean)) / gravity - 1.) > norm_tolerance:
        return None
    base_rotation = np.asarray(base_from_lidar, dtype=float)[:3, :3]
    g = base_rotation @ np.asarray(lidar_from_imu, dtype=float) @ mean
    return math.atan2(g[1], g[2]), math.atan2(-g[0], math.hypot(g[1], g[2]))


def attitude_gap(tracked, gravity):
    roll = abs(math.atan2(math.sin(tracked[0] - gravity[0]), math.cos(tracked[0] - gravity[0])))
    return max(roll, abs(tracked[1] - gravity[1]))


def relevel_before_entry(worker, imu_context, base_from_lidar, *, clock, sleep,
                         gap_rad=DEFAULT_GAP_RAD, timeout_sec=DEFAULT_TIMEOUT_SEC):
    """Compare tracked and IMU tilt; re-bootstrap tracking only on a large gap.

    Returns a diagnostic dict. Raises ValueError only if a re-level was needed
    and no fresh same-epoch geometry arrived within timeout_sec.
    """
    sample = worker.snapshot()
    result = dict(action='none', gap_threshold_rad=gap_rad)
    if sample is None or not sample.geometry_valid or sample.transform is None:
        result['reason'] = 'no valid tracking sample'
        return result
    if (imu_context is None or imu_context.get('epoch') != sample.epoch
            or imu_context.get('lidar_from_imu') is None):
        result['reason'] = 'no IMU window for this tracking epoch'
        return result
    gravity = gravity_body_tilt(imu_context['rows'], imu_context['lidar_from_imu'], base_from_lidar)
    if gravity is None:
        result['reason'] = 'IMU not quasi-static; tracked attitude kept'
        return result
    tracked = tracked_body_tilt(sample.transform, base_from_lidar)
    gap = attitude_gap(tracked, gravity)
    result.update(tracked_roll=tracked[0], tracked_pitch=tracked[1], imu_roll=gravity[0],
                  imu_pitch=gravity[1], gap_rad=gap, epoch=sample.epoch)
    if gap <= gap_rad:
        result['reason'] = 'tracked attitude consistent with IMU gravity'
        return result
    previous_epoch = worker.epoch
    worker.reset('tracked attitude disagrees with IMU gravity by %.2f rad; re-leveling before stair entry' % gap)
    result.update(action='relevel', previous_epoch=previous_epoch)
    started = clock()
    deadline = started + timeout_sec
    while True:
        fresh = worker.snapshot()
        now = clock()
        if (fresh is not None and fresh.epoch > previous_epoch and fresh.epoch == worker.epoch
                and fresh.geometry_valid and not worker.diagnostics().get('worker_error')):
            result.update(new_epoch=fresh.epoch, relevel_sec=now - started)
            return result
        if now >= deadline:
            raise ValueError('tracking re-level after IMU attitude mismatch (%.2f rad) did not '
                             'produce fresh geometry within %.1fs' % (gap, timeout_sec))
        sleep(min(.05, max(0., deadline - now)))
