"""Entry re-level: only on a large quasi-static tracked-vs-IMU tilt gap; never a new stop."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stair_supervisor.attitude_consistency import (  # noqa: E402
    attitude_gap, gravity_body_tilt, relevel_before_entry, tracked_body_tilt)

# Production calibration: 5 deg LiDAR pitch, IMU extrinsic from lidar_imu_baseline.yaml.
BASE_FROM_LIDAR = np.array([[0.996194698091746, 0, 0.087155742747658, 0], [0, 1, 0, 0],
                            [-0.087155742747658, 0, 0.996194698091746, .18], [0, 0, 0, 1]])
LIDAR_FROM_IMU = np.array([[0.989895, -0.135195, -0.042775], [0.136290, 0.990384, 0.023781],
                           [0.039149, -0.029370, 0.998802]])


def body_rotation(roll, pitch, yaw=0.3):
    return Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()


def imu_rows(roll, pitch, *, g=1., scale=1., n=200, t0=100., wobble=.0):
    """Specific force seen by the IMU for a static body at roll/pitch (plus balancing wobble)."""
    world_up = np.array([0., 0., g])
    body = body_rotation(roll, pitch)
    imu_from_base = (BASE_FROM_LIDAR[:3, :3] @ LIDAR_FROM_IMU).T
    a = imu_from_base @ body.T @ world_up
    rows = []
    for i in range(n):
        jitter = wobble * math.sin(i * .7) * np.array([1., -1., 0.])
        rows.append((t0 + i * .005, .01, -.01, .02, *(a * scale + jitter)))
    return rows


def tracking_transform(roll, pitch):
    local_base = np.eye(4)
    local_base[:3, :3] = body_rotation(roll, pitch)
    return tuple((local_base @ BASE_FROM_LIDAR).flat)


class FakeWorker:
    def __init__(self, roll, pitch, fresh_after_reset=True):
        self.epoch = 0
        self.resets = []
        self.fresh_after_reset = fresh_after_reset
        self.sample = SimpleNamespace(epoch=0, geometry_valid=True, transform=tracking_transform(roll, pitch))
        self.error = ''

    def snapshot(self):
        return self.sample

    def diagnostics(self):
        return dict(worker_error=self.error)

    def reset(self, reason):
        self.resets.append(reason)
        self.epoch += 1
        self.sample = None
        self.error = reason


class Clock:
    def __init__(self, worker):
        self.now, self.worker = 10., worker

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(seconds, .05)
        w = self.worker
        if w.resets and w.fresh_after_reset and self.now > 11.:  # ~1 s gravity bootstrap
            w.sample = SimpleNamespace(epoch=w.epoch, geometry_valid=True, transform=tracking_transform(0., 0.))
            w.error = ''


def context(rows, epoch=0):
    return dict(epoch=epoch, rows=tuple(rows), lidar_from_imu=LIDAR_FROM_IMU.tolist(), max_gap=.03)


class GravityTiltTest(unittest.TestCase):
    def test_recovers_body_tilt_through_production_calibration(self):
        for roll, pitch in ((0., 0.), (.3, -.1), (-.2, .25)):
            for scale, g in ((1., 1.), (9.80665, 9.80665)):
                tilt = gravity_body_tilt(imu_rows(roll, pitch, g=g), LIDAR_FROM_IMU, BASE_FROM_LIDAR)
                self.assertAlmostEqual(tilt[0], roll, places=4)  # 6-digit extrinsic is not exactly orthonormal
                self.assertAlmostEqual(tilt[1], pitch, places=4)

    def test_balancing_wobble_averages_out(self):
        tilt = gravity_body_tilt(imu_rows(0., 0., wobble=.05), LIDAR_FROM_IMU, BASE_FROM_LIDAR)
        self.assertLess(max(abs(tilt[0]), abs(tilt[1])), .01)

    def test_sustained_acceleration_is_not_quasi_static(self):
        self.assertIsNone(gravity_body_tilt(imu_rows(0., 0., g=1.05), LIDAR_FROM_IMU, BASE_FROM_LIDAR))
        self.assertIsNone(gravity_body_tilt(imu_rows(0., 0.)[:5], LIDAR_FROM_IMU, BASE_FROM_LIDAR))

    def test_tracked_tilt_matches_definition(self):
        roll, pitch = tracked_body_tilt(tracking_transform(.35, -.12), BASE_FROM_LIDAR)
        self.assertAlmostEqual(roll, .35)
        self.assertAlmostEqual(pitch, -.12)
        self.assertAlmostEqual(attitude_gap((.348, -.124), (-.039, .058)), .387, places=3)


class RelevelTest(unittest.TestCase):
    def run_check(self, tracked, imu, **worker_options):
        worker = FakeWorker(*tracked, **worker_options)
        clock = Clock(worker)
        result = relevel_before_entry(worker, context(imu_rows(*imu)), BASE_FROM_LIDAR,
                                      clock=clock.monotonic, sleep=clock.sleep)
        return result, worker, clock

    def test_field_10_50_corrupted_frame_is_releveled_not_stopped(self):
        # Tracked +0.348 roll vs IMU -0.039 roll at the RF descent entry.
        result, worker, clock = self.run_check((.348, -.124), (-.039, .058))
        self.assertEqual(result['action'], 'relevel')
        self.assertEqual(len(worker.resets), 1)
        self.assertEqual(result['new_epoch'], 1)
        self.assertLess(result['relevel_sec'], 4.)

    def test_consistent_tracking_changes_nothing(self):
        # Healthy quasi-static gaps reach ~0.1 roll / ~0.17 pitch in the field.
        for tracked, imu in (((-.03, .04), (.02, .10)), ((.0, .2), (.0, .04)), ((.1, 0.), (0., 0.))):
            result, worker, _ = self.run_check(tracked, imu)
            self.assertEqual(result['action'], 'none', result)
            self.assertEqual(worker.resets, [])

    def test_moving_imu_keeps_the_tracked_frame(self):
        worker = FakeWorker(.35, 0.)
        clock = Clock(worker)
        result = relevel_before_entry(worker, context(imu_rows(0., 0., g=1.1)), BASE_FROM_LIDAR,
                                      clock=clock.monotonic, sleep=clock.sleep)
        self.assertEqual(result['action'], 'none')
        self.assertEqual(worker.resets, [])

    def test_missing_or_other_epoch_imu_keeps_the_tracked_frame(self):
        for imu in (None, context(imu_rows(0., 0.), epoch=5)):
            worker = FakeWorker(.35, 0.)
            clock = Clock(worker)
            result = relevel_before_entry(worker, imu, BASE_FROM_LIDAR, clock=clock.monotonic, sleep=clock.sleep)
            self.assertEqual(result['action'], 'none')
            self.assertEqual(worker.resets, [])

    def test_relevel_without_fresh_geometry_is_reported(self):
        with self.assertRaises(ValueError):
            self.run_check((.35, 0.), (0., 0.), fresh_after_reset=False)


if __name__ == '__main__':
    unittest.main()
