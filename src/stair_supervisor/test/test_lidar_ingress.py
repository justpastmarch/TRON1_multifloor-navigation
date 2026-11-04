"""No ROS node or estimator process: exercise the actual raw IPC ingress."""
import queue
import unittest
import numpy as np
from stair_supervisor.lidar_process import ProcessTrackingWorker
from stair_supervisor.lidar_tracking import TrackingSettings


class IngressTest(unittest.TestCase):
    def setUp(self):
        self.worker = ProcessTrackingWorker(TrackingSettings(queue_capacity=8), np.eye(3))
        self.addCleanup(self.worker.shutdown)

    def test_actual_multiprocessing_queue_retains_newest_eight_frames(self):
        for stamp in range(1, 11):
            self.assertTrue(self.worker.push_raw_scan(float(stamp), b'raw',
                            received_at=stamp + .1, measured_at=stamp - .1))
        frames = [self.worker._scans.get(timeout=1.) for _ in range(8)]
        self.assertEqual([frame[1] for frame in frames], list(range(3, 11)))
        self.assertTrue(all(frame[0] == 0 and frame[2] == b'raw' for frame in frames))
        self.assertEqual(frames[-1][3:], (10.1, 9.9))
        self.assertEqual(self.worker.diagnostics()['ingress_dropped_old_scans'], 2)
        self.assertEqual(self.worker.diagnostics()['ingress_dropped_new_scans'], 0)

    def test_temporarily_unreadable_feeder_is_waited_for_before_eviction(self):
        original = self.worker._scans
        class DelayedFeeder:
            def __init__(self):self.value = 'old'; self.waits = []
            def put_nowait(self, value):
                if self.value is not None:raise queue.Full
                self.value = value
            def get(self, timeout):
                self.waits.append(timeout)
                value, self.value = self.value, None
                return value
        channel = DelayedFeeder()
        self.worker._scans = channel
        try:
            self.assertTrue(self.worker.push_raw_scan(2., b'latest'))
            self.assertEqual(channel.value[1], 2.)
            self.assertEqual(channel.waits, [.005])
        finally:self.worker._scans = original

    def test_consumer_can_free_slot_during_feeder_empty_race(self):
        original = self.worker._scans
        class ConcurrentConsumer:
            def __init__(self):self.puts = 0;self.value = None
            def put_nowait(self, value):
                self.puts += 1
                if self.puts == 1:raise queue.Full
                self.value = value
            def get(self, timeout):raise queue.Empty
        channel = ConcurrentConsumer();self.worker._scans = channel
        try:
            self.assertTrue(self.worker.push_raw_scan(3., b'latest'))
            self.assertEqual(channel.value[1], 3.)
            self.assertEqual(self.worker._ingress_drops, 0)
        finally:self.worker._scans = original

    def test_stalled_feeder_drops_current_frame_without_unbounded_wait(self):
        original = self.worker._scans
        class StalledFeeder:
            def put_nowait(self, value):raise queue.Full
            def get(self, timeout):
                self.timeout = timeout
                raise queue.Empty
        channel = StalledFeeder();self.worker._scans = channel
        try:
            self.assertFalse(self.worker.push_raw_scan(3., b'latest'))
            self.assertEqual(channel.timeout, .005)
            self.assertEqual(self.worker._ingress_drops, 1)
            self.assertEqual(self.worker._ingress_new_drops, 1)
            self.assertEqual(self.worker._ingress_old_drops, 0)
        finally:self.worker._scans = original

    def test_queue_overload_preserves_reset_epoch_and_imu_queue(self):
        self.assertTrue(self.worker.push_imu((1.,0,0,0,0,0,9.8)))
        for stamp in range(1, 9):self.worker.push_raw_scan(float(stamp), b'old')
        self.worker.reset('new acquisition')
        self.assertTrue(self.worker.push_raw_scan(20., b'new'))
        frames = [self.worker._scans.get(timeout=1.) for _ in range(8)]
        self.assertEqual(frames[-1][0:3], (1, 20., b'new'))
        self.assertEqual(self.worker._imus.get(timeout=1.), (0, (1.,0,0,0,0,0,9.8)))
        self.assertEqual(self.worker._commands.get(timeout=1.), ('reset',1,'new acquisition'))


if __name__ == '__main__':unittest.main()
