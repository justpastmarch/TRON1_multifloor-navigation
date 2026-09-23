"""Real ROS message types with isolated boundaries; no master or robot socket."""
import hashlib
import io
import json
from functools import partial
import pickle
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import struct
import unittest
import threading
from collections import deque
from unittest.mock import patch, Mock

import numpy as np
import rospy
from stair_supervisor import ros_lidar, ros_entrypoint
from stair_supervisor.stair_feedback import StairFeedback
from test_lidar_control import Worker, route, sample


class AdapterTest(unittest.TestCase):
    def test_diagnostics_normalize_numpy_scalars_and_nonfinite_values(self):
        value = {'checks': [np.bool_(True), np.bool_(False)],
                 'sequence': np.int64(7),
                 'values': np.array([np.float32(.5), np.nan, np.inf]),
                 'age': np.float32(np.nan)}
        encoded = json.dumps(ros_lidar.json_finite(value), allow_nan=False)
        self.assertEqual(json.loads(encoded),
                         {'checks': [True, False], 'sequence': 7,
                          'values': [.5, None, None], 'age': None})

    def test_settled_entry_debug_does_not_stop_lidar_publication(self):
        adapter = self.interface()
        from test_supervisor import make_configuration
        from stair_supervisor.stair_evidence import Phase
        adapter.control.capture_entry('test_up', np.eye(4), .01, 'synthetic survey', 100.)
        adapter.control.arm(make_configuration().profiles[0], 100.)
        adapter.control.begin_phase(Phase.VERIFY_ENTRY, 100.)
        complete = False
        for i in range(1, 8):
            stamp = 100. + i*.2
            adapter.worker.value = sample(i+1, stamp)
            adapter.clock.monotonic = lambda stamp=stamp: stamp+.01
            report = adapter.control.evaluate(Phase.VERIFY_ENTRY, stamp+.01)
            complete = complete or report.complete
            adapter._publish(None)
        self.assertTrue(complete)
        self.assertEqual(adapter.status.publish.call_count, 7)
        self.assertEqual(adapter.control_debug.publish.call_count, 7)
        debug = json.loads(adapter.control_debug.publish.call_args[0][0].data)
        self.assertIs(debug['phase_complete'], True)
        self.assertIs(debug['completion_checks']['settled'], True)

    def test_unsettled_entry_debug_does_not_stop_lidar_publication(self):
        adapter = self.interface()
        from test_supervisor import make_configuration
        from stair_supervisor.stair_evidence import Phase
        adapter.control.capture_entry('test_up', np.eye(4), .01, 'synthetic survey', 100.)
        adapter.control.arm(make_configuration().profiles[0], 100.)
        adapter.control.begin_phase(Phase.VERIFY_ENTRY, 100.)
        for i in range(1, 8):
            stamp = 100. + i*.2
            # Stay within target tolerance but exceed settling excursion.
            # The old false comparison was a NumPy bool_.
            adapter.worker.value = sample(i+1, stamp, x=.03*(-1)**i)
            adapter.clock.monotonic = lambda stamp=stamp: stamp+.01
            report = adapter.control.evaluate(Phase.VERIFY_ENTRY, stamp+.01)
            self.assertFalse(report.complete)
            adapter._publish(None)
        self.assertEqual(adapter.status.publish.call_count, 7)
        self.assertEqual(adapter.control_debug.publish.call_count, 7)
        debug = json.loads(adapter.control_debug.publish.call_args[0][0].data)
        self.assertIs(debug['completion_checks']['settled'], False)

    def interface(self):
        adapter = ros_lidar.RosLidarInterface.__new__(ros_lidar.RosLidarInterface)
        adapter.configuration = SimpleNamespace(mode="observe", observe_only=True,
            source_path='/synthetic/config.yaml', source_sha256='synthetic-hash', document={'routes':[route()]})
        adapter.worker = Worker()
        adapter.control = StairFeedback(adapter.worker, np.eye(4), [route()])
        adapter.clock = SimpleNamespace(monotonic=lambda: 100.)
        adapter.odom, adapter.status, adapter.control_debug, adapter.markers = Mock(), Mock(), Mock(), Mock()
        adapter.cloud=Mock();adapter._raw_scans=deque(maxlen=32);adapter._raw_lock=threading.Lock()
        adapter._published, adapter._observing = None, False
        adapter.worker.diagnostics = lambda: {"worker_error": ""}
        return adapter

    def test_raw_decoder_is_identical_to_generated_message_xyz(self):
        from livox_ros_driver2.msg import CustomMsg, CustomPoint
        from stair_supervisor.lidar_process import points_from_raw_livox, raw_header_stamp
        from stair_supervisor.lidar_tracking import points_from_livox
        message = CustomMsg()
        message.header.frame_id = "livox_frame"
        message.header.stamp = rospy.Time.from_sec(123.4)
        message.points = [CustomPoint(offset_time=i, x=float(i)/3, y=-float(i)/7,
                                      z=float(i)*.001, reflectivity=7, tag=1, line=2) for i in range(200)]
        message.point_num = len(message.points)
        stream = io.BytesIO();message.serialize(stream)
        np.testing.assert_array_equal(points_from_raw_livox(stream.getvalue()), points_from_livox(CustomMsg().deserialize(stream.getvalue())))
        self.assertAlmostEqual(raw_header_stamp(stream.getvalue()), 123.4)
        with self.assertRaises(ValueError):
            points_from_raw_livox(stream.getvalue()[:-1])

    def test_entry_job_is_serializable_for_compute_process(self):
        operation = partial(ros_lidar.fit_entry_template, route=route(), target=np.ones((4,3)),
                            reference={}, base_from_lidar=np.eye(4), registration=None)
        self.assertEqual(pickle.loads(pickle.dumps(operation)).func, ros_lidar.fit_entry_template)

    def test_observe_only_main_never_constructs_transport_or_command_node(self):
        config = SimpleNamespace(mode="observe", observe_only=True)
        with patch.object(ros_entrypoint.rospy, "get_param", side_effect=lambda _k, default=None: default), \
             patch.object(ros_entrypoint, "load_lidar_configuration", return_value=config), \
             patch.object(ros_lidar, "RosLidarInterface") as adapter, \
             patch.object(ros_entrypoint, "RobotTransport") as transport, \
             patch.object(ros_entrypoint, "RosStairSupervisorNode") as node, \
             patch.object(ros_entrypoint.rospy, "on_shutdown"), \
             patch.object(ros_entrypoint.rospy, "loginfo"), \
             patch.object(ros_entrypoint.rospy, "spin"):
            self.assertEqual(ros_entrypoint.main(), 0)
        adapter.assert_called_once()
        transport.assert_not_called()
        node.assert_not_called()

    def test_uncommissioned_route_can_be_observed_without_enabling_a_mission(self):
        adapter=self.interface()
        adapter.control.routes['test_up']['commissioned']=False
        adapter.control.capture_entry('test_up',np.eye(4),.01,'synthetic survey',100.)
        adapter.ensure_entry=Mock()
        from test_supervisor import make_configuration
        config=make_configuration()
        with patch.object(ros_lidar.rospy,'get_param',side_effect=lambda k,default=None:'test_up' if k.endswith('observe_route_id') else default), \
             patch.object(ros_lidar,'load_stair_configuration',return_value=config):
            result=adapter._start_observation(None)
        self.assertTrue(result.success,result.message)
        self.assertTrue(adapter._observing)
        self.assertFalse(adapter.control.route['commissioned'])

    def test_rejected_geometry_never_publishes_retimestamped_odometry(self):
        adapter = self.interface()
        adapter._publish(None)
        self.assertEqual(adapter.odom.publish.call_count, 1)
        original = adapter.odom.publish.call_args[0][0]
        self.assertEqual(original.header.stamp.to_sec(), 100.)
        adapter.worker.value = sample(2, 100.2, valid=False)
        adapter._publish(None)
        self.assertEqual(adapter.odom.publish.call_count, 1)

    def test_markers_use_route_frame_and_expire_without_hiding_stale_pose(self):
        adapter = self.interface()
        adapter.control.capture_entry("test_up", np.eye(4), .01, "survey", 100.)
        from test_supervisor import make_configuration
        adapter.control.arm(make_configuration().profiles[0], 100.)
        adapter._publish_geometry(adapter.worker.value)
        markers = adapter.markers.publish.call_args[0][0].markers
        self.assertEqual(len(markers), 7)
        self.assertTrue(all(m.header.frame_id == "stair_local_0" for m in markers))
        self.assertTrue(all(m.lifetime.to_sec() == .5 for m in markers))
        adapter.clock.monotonic = lambda: 101.
        adapter._publish_geometry(adapter.worker.value)
        body = next(m for m in adapter.markers.publish.call_args[0][0].markers if m.ns == "body")
        self.assertEqual((body.color.r, body.color.g), (1., 0.))

    def test_auto_entry_requires_matching_template_and_independent_error_budget(self):
        adapter = self.interface()
        adapter.clock = SimpleNamespace(monotonic=time.monotonic)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "survey.npy"
            np.save(str(path), np.array([[1., 0, 0], [0, 2, 0], [2, 2, 1]]))
            reference = dict(path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                yaw_candidates=[0.], min_fitness=.5, max_rmse_m=.1, score_gap=.1,
                validated_anchor_error_m=.02, timeout_sec=1., unique_geometry_verified=True)
            adapter.control.routes["test_up"]["entry_reference"] = reference
            adapter.configuration = SimpleNamespace(root=root)
            adapter.worker.settings = SimpleNamespace(registration=None)
            def entry_job(operation):
                future = Future()
                try:
                    future.set_result(operation(np.ones((3, 3)), sample()))
                except Exception as error:
                    future.set_exception(error)
                return future
            adapter.worker.entry_job = entry_job
            result = SimpleNamespace(accepted=True, fitness=.9, inlier_rmse_m=.01, transform=np.eye(4))
            with patch.object(ros_lidar, "register_scan_to_map", return_value=result):
                adapter.ensure_entry("test_up")
                self.assertEqual(adapter.control.anchors["test_up"].uncertainty_m, .02)
                reference["validated_anchor_error_m"] = .2
                with self.assertRaisesRegex(ValueError, "uncertainty"):
                    adapter.ensure_entry("test_up")
            reference["sha256"] = "wrong"
            with self.assertRaisesRegex(ValueError, "fingerprint"):
                adapter.ensure_entry("test_up")

    def test_clock_reversal_invalidates_epoch_and_does_not_emit_future_pose(self):
        adapter = self.interface()
        adapter._last_clock_offset = None
        adapter.worker.reset, adapter.worker.push_raw_scan = Mock(), Mock()
        message = SimpleNamespace(_buff=struct.pack('<III', 1, 100, 0))
        with patch.object(ros_lidar.rospy.Time, "now", return_value=rospy.Time.from_sec(100.)):
            adapter._scan(message)
        with patch.object(ros_lidar.rospy.Time, "now", return_value=rospy.Time.from_sec(99.)):
            adapter._scan(message)
        self.assertEqual(adapter.worker.push_raw_scan.call_count, 1)
        self.assertGreaterEqual(adapter.worker.reset.call_count, 1)


if __name__ == "__main__":
    unittest.main()
