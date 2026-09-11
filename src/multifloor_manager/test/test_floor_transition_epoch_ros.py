#!/usr/bin/env python3
"""Prove parsed tag evidence cannot commit across a terminal epoch."""

from __future__ import annotations

import threading
import unittest

from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rospy
import rostest

from multifloor_manager.map_evidence import (
    MapFingerprint,
    MapGeneration,
    MapIdentity,
    MapOrigin,
    Nanoseconds,
)
from multifloor_manager.msg import FloorState
import multifloor_manager.ros_callbacks as callback_module
from multifloor_manager.ros_runtime import RosEvidenceRuntime
from multifloor_manager.tag_evidence import FloorId, FloorTagSet, TagId
from multifloor_manager.transitions import (
    InvalidPredicateObservationError,
    PredicateObservation,
    TransitionCondition,
    TransitionPolicy,
)


class FakeTfBuffer:
    def can_transform(self, target: str, source: str, stamp, timeout) -> bool:
        return True


class CallbackEpochRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("floor_transition_epoch_test", anonymous=True)

    @staticmethod
    def _runtime() -> tuple:
        identity = MapIdentity(
            "map",
            1,
            1,
            0.1,
            MapOrigin(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            "fixture",
        )
        runtime = RosEvidenceRuntime(
            "3F",
            identity,
            (FloorTagSet(FloorId("4F"), (TagId(101),)),),
            FakeTfBuffer(),
            TransitionPolicy(
                "floor_transition_ready",
                True,
                (
                    TransitionCondition("T_FLOOR_CONFIRMED", True, True),
                    TransitionCondition("T_LOCALIZED", True, True),
                    TransitionCondition("T_COSTMAP_READY", True, True),
                ),
                0,
                10.0,
                0.2,
                8.0,
            ),
        )
        runtime.arm_transition("4F", (101,), identity)
        return runtime, identity

    def test_policy_token_starts_timeout_only_at_begin_and_rejects_stale_epoch(self) -> None:
        # Given: an armed policy with a manual monotonic clock.
        now = [10.0]
        identity = MapIdentity("map", 1, 1, 0.1, MapOrigin(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), "fixture")
        runtime = RosEvidenceRuntime("3F", identity, (), FakeTfBuffer(), TransitionPolicy(
            "floor_transition_ready", True,
            (TransitionCondition("ready", True, True),), 0, 10.0, 0.0, 2.0,
        ), lambda: now[0])
        token = runtime.arm_transition("4F", (), identity)

        # When: observations are recorded before the final gate begins.
        runtime.record_policy_observation(token, "ready", PredicateObservation(True, 10.0, token.epoch))
        now[0] = 11.0

        # Then: an armed policy cannot commit before begin or evaluation.
        self.assertFalse(runtime.commit_ready(token))
        runtime.begin_policy(token)
        self.assertFalse(runtime.commit_ready(token))
        self.assertTrue(runtime.evaluate_policy(token)[0])
        self.assertTrue(runtime.commit_ready(token))
        runtime.finish_transition(FloorState.FAULT, "finish")
        self.assertIsNone(runtime.policy_manager)
        self.assertEqual(runtime.policy_observations, {})
        with self.assertRaises(InvalidPredicateObservationError):
            runtime.begin_policy(token)
        with self.assertRaises(InvalidPredicateObservationError):
            runtime.evaluate_policy(token)
        with self.assertRaises(InvalidPredicateObservationError):
            runtime.commit_ready(token)

    def test_late_epoch_writer_cannot_change_rearmed_policy(self) -> None:
        # Given: a writer is blocked at the existing reentrant runtime condition.
        runtime, identity = self._runtime()
        token_n = runtime.arm_transition("4F", (101,), identity)
        writer_attempting, release_writer = threading.Event(), threading.Event()
        writer_done, writer_error = threading.Event(), []

        def blocked_writer() -> None:
            writer_attempting.set()
            try:
                runtime.record_policy_observation(token_n, "T_FLOOR_CONFIRMED", PredicateObservation(True, 0.0, token_n.epoch))
            except InvalidPredicateObservationError as error:
                writer_error.append(error)
            finally:
                writer_done.set()

        writer = threading.Thread(target=blocked_writer, daemon=True)
        with runtime.condition:
            writer.start()
            self.assertTrue(writer_attempting.wait(timeout=2.0))
            runtime.finish_transition(FloorState.FAULT, "finish N")
            token_n1 = runtime.arm_transition("4F", (101,), identity)
            release_writer.set()

        # When: the blocked N writer acquires the condition after N+1 is armed.
        self.assertTrue(writer_done.wait(timeout=2.0))

        # Then: N+1 remains empty and accepts only its own token.
        self.assertEqual(len(writer_error), 1)
        self.assertEqual(runtime.policy_observations, {})
        runtime.record_policy_observation(token_n1, "T_FLOOR_CONFIRMED", PredicateObservation(True, 0.0, token_n1.epoch))

    def test_tag_parse_finishing_after_fault_cannot_restore_state(self) -> None:
        # Given: an active transition whose tag parser is blocked outside the runtime lock.
        runtime, _identity = self._runtime()
        message = AprilTagDetectionArray()
        message.header.frame_id = "camera_link"
        message.header.stamp = rospy.Time.now()
        message.detections = (AprilTagDetection(id=(101,)),)
        parser_entered = threading.Event()
        release_parser = threading.Event()
        original_parser = callback_module.parse_apriltag_detection_array

        def blocked_parser(value, received_at_ns):
            parser_entered.set()
            self.assertTrue(release_parser.wait(timeout=2.0))
            return original_parser(value, received_at_ns)

        callback_module.parse_apriltag_detection_array = blocked_parser
        callback = threading.Thread(target=runtime.tag_callback, args=(message,), daemon=True)
        try:
            callback.start()
            self.assertTrue(parser_entered.wait(timeout=2.0))

            # When: the action faults before the parsed observation can commit.
            runtime.finish_transition(FloorState.FAULT, "injected terminal fault")
            release_parser.set()
            callback.join(timeout=2.0)

            # Then: stale callback state cannot cross the terminal epoch.
            self.assertFalse(callback.is_alive())
            self.assertIsNone(runtime.tag_state)
            self.assertIsNone(runtime.tag_context)
            self.assertFalse(runtime.tag_accepted)
            self.assertFalse(runtime.wrong_floor_tag)
            self.assertEqual(runtime.state, FloorState.FAULT)
        finally:
            release_parser.set()
            callback.join(timeout=2.0)
            callback_module.parse_apriltag_detection_array = original_parser

    def test_map_parse_finishing_after_fault_cannot_advance_generation(self) -> None:
        # Given: a target map fingerprint parser blocked during an active transition.
        runtime, identity = self._runtime()
        entered = threading.Event()
        release = threading.Event()
        original = callback_module.fingerprint_occupancy_grid
        fingerprint = MapFingerprint(
            identity.frame_id,
            identity.width,
            identity.height,
            identity.resolution,
            identity.origin,
            identity.data_hash,
            Nanoseconds(rospy.Time.now().to_nsec() + 1),
        )

        def blocked_fingerprint(_message, _received_at_ns):
            entered.set()
            self.assertTrue(release.wait(timeout=2.0))
            return fingerprint

        callback_module.fingerprint_occupancy_grid = blocked_fingerprint
        callback = threading.Thread(
            target=runtime.map_callback,
            args=(OccupancyGrid(),),
            daemon=True,
        )
        try:
            callback.start()
            self.assertTrue(entered.wait(timeout=2.0))
            runtime.finish_transition(FloorState.FAULT, "map race fault")
            release.set()
            callback.join(timeout=2.0)
            self.assertIsNone(runtime.map_guard)
            self.assertEqual(runtime.map_state.generation, MapGeneration(0))
            self.assertEqual(runtime.state, FloorState.FAULT)
        finally:
            release.set()
            callback.join(timeout=2.0)
            callback_module.fingerprint_occupancy_grid = original

    def test_pose_reduce_finishing_after_fault_cannot_restore_localization(self) -> None:
        # Given: a pose reducer blocked after capturing one active localization epoch.
        runtime, _identity = self._runtime()
        armed_at = rospy.Time.now().to_nsec() - 1
        runtime.arm_localization(armed_at)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        entered = threading.Event()
        release = threading.Event()
        original = callback_module.observe_pose

        def blocked_observe(state, stamp_ns, received_at_ns, covariance):
            entered.set()
            self.assertTrue(release.wait(timeout=2.0))
            return original(state, stamp_ns, received_at_ns, covariance)

        callback_module.observe_pose = blocked_observe
        callback = threading.Thread(target=runtime.pose_callback, args=(pose,), daemon=True)
        try:
            callback.start()
            self.assertTrue(entered.wait(timeout=2.0))
            runtime.finish_transition(FloorState.FAULT, "pose race fault")
            release.set()
            callback.join(timeout=2.0)
            self.assertIsNone(runtime.localization)
            self.assertEqual(runtime.state, FloorState.FAULT)
        finally:
            release.set()
            callback.join(timeout=2.0)
            callback_module.observe_pose = original

    def test_costmap_identity_finishing_after_fault_cannot_advance_sequence(self) -> None:
        # Given: costmap identity construction blocked after capturing an active epoch.
        runtime, identity = self._runtime()
        costmap = OccupancyGrid()
        costmap.header.frame_id = identity.frame_id
        costmap.info.width = identity.width
        costmap.info.height = identity.height
        costmap.info.resolution = identity.resolution
        costmap.info.origin.position.x = identity.origin.x
        costmap.info.origin.position.y = identity.origin.y
        costmap.info.origin.position.z = identity.origin.z
        costmap.info.origin.orientation.x = identity.origin.qx
        costmap.info.origin.orientation.y = identity.origin.qy
        costmap.info.origin.orientation.z = identity.origin.qz
        costmap.info.origin.orientation.w = identity.origin.qw
        entered = threading.Event()
        release = threading.Event()
        original = callback_module.MapIdentity

        def blocked_identity(*components):
            entered.set()
            self.assertTrue(release.wait(timeout=2.0))
            return original(*components)

        callback_module.MapIdentity = blocked_identity
        callback = threading.Thread(
            target=runtime.costmap_callback,
            args=(costmap,),
            daemon=True,
        )
        try:
            callback.start()
            self.assertTrue(entered.wait(timeout=2.0))

            # When: terminal FAULT invalidates the epoch before identity commit.
            runtime.finish_transition(FloorState.FAULT, "costmap race fault")
            release.set()
            callback.join(timeout=2.0)

            # Then: the stale costmap cannot advance readiness state.
            self.assertFalse(callback.is_alive())
            self.assertEqual(runtime.costmap_sequence, 0)
            self.assertIsNone(runtime.costmap_identity)
            self.assertEqual(runtime.state, FloorState.FAULT)
        finally:
            release.set()
            callback.join(timeout=2.0)
            callback_module.MapIdentity = original


if __name__ == "__main__":
    rostest.rosrun("multifloor_manager", "floor_transition_epoch_ros", CallbackEpochRosTest)
