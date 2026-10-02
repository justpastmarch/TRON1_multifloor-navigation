"""Analytic geometry and causal coverage tests; no robot or ROS master."""
import io
import math
import unittest
import threading
import time
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from stair_supervisor.lidar_tracking_core import ImuRotation
from stair_supervisor.lidar_tracking import TrackingEngine, TrackingSettings, TrackingWorker
from stair_supervisor.lidar_process import points_from_raw_livox


def rows(rate=2., end=100.6):
    return np.array([[t, 0., 0., rate, 0., 0., 9.81]
                     for t in np.arange(99.4, end+.0001, .005)])


class DeskewTest(unittest.TestCase):
    def test_worker_waits_for_acquisition_tail_imu(self):
        called=threading.Event()
        class Engine:
            def __init__(self, settings, rotation, checksum, epoch):
                self.last_stamp=99.;self.epoch=epoch
            def process(self, stamp, points, imu, received, measured, clock):
                called.set()
        worker=TrackingWorker(TrackingSettings(frame_step=1, rotational_deskew=True,
            imu_wait_sec=.3), np.eye(3), engine_factory=Engine)
        worker.start()
        try:
            for t in np.arange(99.98,100.051,.005):worker.push_imu((t,0,0,0,0,0,9.81))
            worker.push_scan(100.,np.array([[1.,1.,1.,.1]]))
            self.assertFalse(called.wait(.025))
            for t in np.arange(100.055,100.111,.005):worker.push_imu((t,0,0,0,0,0,9.81))
            self.assertTrue(called.wait(1.))
        finally:
            worker.shutdown()

    def test_configuration_accepts_only_a_boolean_deskew_switch(self):
        import yaml
        from stair_supervisor.configuration import load_lidar_configuration, StairConfigurationError
        source=Path(__file__).resolve().parents[1]/'config/stair_lidar_3f_4f_test.yaml'
        document=yaml.safe_load(source.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.yaml'
            for value in (True, False):
                document['tracking']['rotational_deskew']=value;path.write_text(yaml.safe_dump(document))
                self.assertEqual(load_lidar_configuration(path,'observe',True).document['tracking']['rotational_deskew'],value)
            document['tracking']['rotational_deskew']='true';path.write_text(yaml.safe_dump(document))
            with self.assertRaises(StairConfigurationError):load_lidar_configuration(path,'observe',True)

    def test_known_rotating_sensor_recovers_stationary_wall(self):
        offsets = np.linspace(0., .1, 101)
        world = np.column_stack((np.full(101, 2.), np.linspace(-1, 1, 101), np.full(101, .5)))
        angle = 2.*offsets
        raw = world.copy()
        raw[:, 0] = np.cos(angle)*world[:, 0]+np.sin(angle)*world[:, 1]
        raw[:, 1] = -np.sin(angle)*world[:, 0]+np.cos(angle)*world[:, 1]
        corrected = ImuRotation(rows(), np.eye(3), .03).deskew_to_start(raw, offsets, 100.)
        np.testing.assert_allclose(corrected, world, atol=1e-10)
        self.assertGreater(np.abs(raw-world).max(), .3)

    def test_static_cloud_and_unsorted_duplicate_offsets(self):
        points = np.random.RandomState(43).normal(size=(5, 3))
        offsets = np.array([.1, 0., .05, .05, .02])
        corrected = ImuRotation(rows(0.), np.eye(3), .03).deskew_to_start(points, offsets, 100.)
        np.testing.assert_allclose(corrected, points, atol=1e-12)

    def test_missing_tail_or_gap_cannot_be_extrapolated(self):
        for data in (rows(end=100.05), rows()[~((rows()[:, 0]>100.02)&(rows()[:, 0]<100.07))]):
            with self.assertRaises(ValueError):
                ImuRotation(data, np.eye(3), .03).deskew_to_start(np.ones((2, 3)), [0., .1], 100.)

    def test_raw_decoder_preserves_xyz_and_nanosecond_offsets(self):
        m=CustomMsg();m.header.stamp=rospy.Time.from_sec(100.)
        m.points=[CustomPoint(x=1., y=2., z=3., offset_time=0),
                  CustomPoint(x=4., y=5., z=6., offset_time=100000000)]
        m.point_num=2;s=io.BytesIO();m.serialize(s)
        plain=points_from_raw_livox(s.getvalue())
        timed=points_from_raw_livox(s.getvalue(), with_offsets=True)
        np.testing.assert_array_equal(plain, timed[:, :3])
        np.testing.assert_allclose(timed[:, 3], [0., .1])

    def test_missing_offsets_never_seed_a_mixed_map(self):
        engine=TrackingEngine(TrackingSettings(rotational_deskew=True), np.eye(3))
        sample=engine.process(100., np.ones((4, 3)), rows(), 100.6, 100.)
        self.assertFalse(sample.geometry_valid)
        self.assertEqual(sample.deskew_state, 'unavailable')
        self.assertEqual(len(engine.scans), 0)
        self.assertIsNone(engine.last_stamp)

    def test_filtered_points_keep_their_original_offsets_and_stamp(self):
        engine=TrackingEngine(TrackingSettings(rotational_deskew=True), np.eye(3))
        cloud=np.array([[.01,0,0,0], [2,0,0,.02], [2,1,0,.04], [2,-1,1,.08], [float('nan'),0,0,.1]])
        with patch('stair_supervisor.lidar_tracking.voxel_downsample_points', side_effect=lambda p,v:p.copy()) as voxel:
            sample=engine.process(100., cloud, rows(), 100.6, 100.)
        self.assertEqual(voxel.call_args.args[0].shape, (3,3))
        self.assertEqual(sample.stamp, 100.)
        self.assertEqual(sample.measured_at, 100.)
        self.assertEqual(sample.deskew_state, 'rotation_to_scan_start')
        self.assertAlmostEqual(sample.scan_span_sec, .1)
        self.assertEqual(len(sample.display_points), 9)

    def test_invalid_offsets_are_rejected_even_on_filtered_points(self):
        for offset in (-.01, float('nan')):
            engine=TrackingEngine(TrackingSettings(rotational_deskew=True), np.eye(3))
            sample=engine.process(100., np.array([[.01,0,0,offset], [2,0,0,0]]), rows(), 100.6, 100.)
            self.assertFalse(sample.geometry_valid)
            self.assertEqual(sample.deskew_state, 'unavailable')


if __name__ == '__main__':
    unittest.main()
