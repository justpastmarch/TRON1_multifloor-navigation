"""Actual child-process reset/timeout lifecycle, with synthetic stationary input."""
import io
from functools import partial
from pathlib import Path
import tempfile
import threading
import time
import unittest

import numpy as np
import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from stair_supervisor.lidar_process import ProcessTrackingWorker
from stair_supervisor.lidar_tracking import TrackingSettings


def stalled_entry(_cloud, _sample, *, started_path):
    Path(started_path).write_text("started")
    time.sleep(60.)


class ProcessTest(unittest.TestCase):
    def setUp(self):
        self.worker = ProcessTrackingWorker(TrackingSettings(frame_step=1, gravity_half_window_s=.01), np.eye(3))
        self.worker.start()
        self.addCleanup(self.worker.shutdown)
        self.points = np.random.RandomState(7).uniform(.5, 3., (300, 3))

    def wait_for(self, predicate, timeout=4.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(predicate(), self.worker.diagnostics())

    def feed(self, stamp):
        for t in np.arange(stamp-.02, stamp+.065, .005):
            self.worker.push_imu((t, 0, 0, 0, 0, 0, 9.8))
        for n in range(2):
            message = CustomMsg()
            message.header.stamp = rospy.Time.from_sec(stamp+n*.02)
            message.points = [CustomPoint(x=x, y=y, z=z) for x, y, z in self.points]
            message.point_num = len(message.points)
            buffer = io.BytesIO();message.serialize(buffer)
            self.worker.push_raw_scan(stamp+n*.02, buffer.getvalue())
        self.wait_for(lambda: self.worker.snapshot() is not None and self.worker.snapshot().geometry_valid)

    def test_reset_rejects_old_frame_and_recovers_new_frame(self):
        self.feed(100.)
        self.worker.reset("test reset")
        self.assertIsNone(self.worker.snapshot())
        self.feed(200.)
        self.assertEqual(self.worker.snapshot().epoch, 1)
        self.assertGreater(self.worker.snapshot().stamp, 200.)

    def test_deskew_crosses_actual_process_boundary_with_preview(self):
        self.worker.shutdown()
        self.worker=ProcessTrackingWorker(TrackingSettings(frame_step=1,
            gravity_half_window_s=.01, rotational_deskew=True), np.eye(3))
        self.worker.start()
        self.addCleanup(self.worker.shutdown)
        self.feed(100.)
        sample=self.worker.snapshot()
        self.assertEqual(sample.deskew_state, 'rotation_to_scan_start')
        self.assertIsNotNone(sample.display_points)
        self.assertIsNotNone(sample.processing_started_at)

    def test_entry_timeout_terminates_native_owner_and_reacquires(self):
        self.feed(100.)
        old_process = self.worker._process
        with tempfile.TemporaryDirectory() as directory:
            started = Path(directory) / "entry-started"
            future = self.worker.entry_job(partial(stalled_entry, started_path=str(started)))
            self.wait_for(started.exists)
            self.worker.cancel_entry(future)
        self.assertTrue(future.cancelled())
        self.assertFalse(old_process.is_alive())
        self.assertNotEqual(self.worker._process.pid, old_process.pid)
        self.assertEqual(self.worker.epoch, 1)
        self.assertIsNone(self.worker.snapshot())
        self.feed(200.)
        self.assertEqual(self.worker.snapshot().epoch, 1)

    def test_shutdown_waits_for_inflight_sensor_enqueue(self):
        entered, release, closing = threading.Event(), threading.Event(), threading.Event()
        original = self.worker._scans
        errors = []

        class PausedQueue:
            def put_nowait(self, value):
                entered.set()
                if not release.wait(3.):
                    raise RuntimeError("test enqueue was not released")
                return original.put_nowait(value)

            def __getattr__(self, name):
                return getattr(original, name)

        self.worker._scans = PausedQueue()
        message = CustomMsg()
        message.header.stamp = rospy.Time.from_sec(100.)
        buffer = io.BytesIO()
        message.serialize(buffer)

        def enqueue():
            try:
                self.worker.push_raw_scan(100., buffer.getvalue())
            except Exception as error:
                errors.append(error)

        def close():
            closing.set()
            self.worker.shutdown()

        sender = threading.Thread(target=enqueue)
        closer = threading.Thread(target=close)
        sender.start()
        self.assertTrue(entered.wait(2.))
        closer.start()
        try:
            self.assertTrue(closing.wait(2.))
            self.assertFalse(self.worker._stopping.wait(.05))
        finally:
            release.set()
            sender.join(3.)
            closer.join(6.)
        self.assertFalse(sender.is_alive())
        self.assertFalse(closer.is_alive())
        self.assertEqual(errors, [])

    def test_restart_blocks_reset_and_entry_on_retired_queue(self):
        with self.worker._lock:
            self.worker._restarting = True
        old_epoch = self.worker.epoch
        try:
            self.worker.reset("concurrent callback")
            self.assertEqual(self.worker.epoch, old_epoch)
            with self.assertRaises(ValueError):
                self.worker.entry_job(lambda *_: None)
        finally:
            with self.worker._lock:
                self.worker._restarting = False

    def test_worker_exit_removes_old_valid_sample(self):
        self.feed(100.)
        self.worker._process.terminate()
        self.worker._process.join(2.)
        self.wait_for(lambda: self.worker.snapshot() is None and bool(self.worker.diagnostics()["worker_error"]))


if __name__ == "__main__":
    unittest.main()
