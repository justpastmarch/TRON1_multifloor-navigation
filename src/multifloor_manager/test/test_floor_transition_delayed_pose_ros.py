#!/usr/bin/env python3
"""Reproduce AMCL callbacks queued before a no-motion service response."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import threading
import unittest

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import LoadMapResponse
import rospy
import rostest
from std_srvs.srv import EmptyResponse

from multifloor_manager.configuration import LandingPose, Quaternion, Stair, StairEndpoint
from multifloor_manager.map_evidence import MapIdentity, MapOrigin
from multifloor_manager.msg import FloorState, FloorTransitionResult
from multifloor_manager.ros_node import MultifloorManagerNode
from multifloor_manager.ros_runtime import RosEvidenceRuntime
from multifloor_manager.tag_evidence import FloorId, FloorTagSet, TagId
from multifloor_manager.transitions import TransitionCondition, TransitionPolicy


class FakeTfBuffer:
    def can_transform(self, target: str, source: str, stamp, timeout) -> bool:
        return True


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class RecordingActionServer:
    def __init__(self) -> None:
        self.succeeded = []

    @staticmethod
    def is_preempt_requested() -> bool:
        return False

    def set_succeeded(self, result: FloorTransitionResult) -> None:
        self.succeeded.append(result)

    def set_aborted(self, result: FloorTransitionResult, reason: str = "") -> None:
        self.fail("unexpected abort: {} {!r}".format(reason, result))

    @staticmethod
    def fail(message: str) -> None:
        raise AssertionError(message)


class EventDrivenServices:
    def __init__(self, runtime: RosEvidenceRuntime) -> None:
        self.runtime = runtime
        self.pending_pose = None
        self.pending = threading.Event()
        self.nomotion_calls = 0

    def call(self, name: str, proxy, *arguments):
        if name == "/change_map":
            return LoadMapResponse(result=LoadMapResponse.RESULT_SUCCESS)
        if name == "/move_base/clear_costmaps":
            return EmptyResponse()
        if name != "/request_nomotion_update":
            raise AssertionError("unexpected service: {}".format(name))

        prior_stamp = self.runtime.pose_high_water_ns(0)
        stamp_ns = rospy.Time.now().to_nsec()
        while stamp_ns <= prior_stamp:
            stamp_ns = rospy.Time.now().to_nsec()
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time(
            secs=stamp_ns // 1_000_000_000,
            nsecs=stamp_ns % 1_000_000_000,
        )
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.01
        self.pending_pose = message
        self.nomotion_calls += 1
        self.pending.set()
        return EmptyResponse()


class DelayedPoseRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("floor_transition_delayed_pose_test", anonymous=True)

    def test_pre_response_pose_committing_after_response_cannot_advance_handshake(self) -> None:
        # Given: real pose reduction with each old-stamped callback withheld until the wait begins.
        target = MapIdentity(
            "map",
            3,
            2,
            0.1,
            MapOrigin(-2.0, 1.0, 0.0, 0.0, 0.0, 0.12467473338522769, 0.992197667229329),
            "fixture",
        )
        runtime = RosEvidenceRuntime(
            "3F",
            target,
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
        runtime.state = FloorState.READY
        runtime.current_floor = "3F"
        services = EventDrivenServices(runtime)
        server = RecordingActionServer()
        node = MultifloorManagerNode.__new__(MultifloorManagerNode)
        node.runtime = runtime
        node.server = server
        node.services = services
        node.initialpose_publisher = RecordingPublisher()
        node.config_root = Path("/tmp")
        node.change_map_name = "/change_map"
        node.nomotion_name = "/request_nomotion_update"
        node.clear_costmaps_name = "/move_base/clear_costmaps"
        node.change_map = node.nomotion_update = node.clear_costmaps = None
        node.tag_timeout = node.transaction_timeout = 2.0
        node.identities = {"4F": target}
        node.floors = {"4F": SimpleNamespace(map_yaml=Path("4F.yaml"))}
        covariance = (0.0,) * 36
        node.stairs = {
            "stair_a": Stair(
                "stair_a",
                "3F",
                "4F",
                (
                    StairEndpoint("3F", "entry_3f", LandingPose(0.0, 0.0, Quaternion(0.0, 0.0, 0.0, 1.0)), covariance, (100,)),
                    StairEndpoint("4F", "entry_4f", LandingPose(1.0, 2.0, Quaternion(0.0, 0.0, 0.0, 1.0)), covariance, (101,)),
                ),
                "up",
                "down",
            )
        }
        feedback = []
        valid_pose_commits = 0

        def record_feedback(phase: str, detail: str) -> None:
            feedback.append((phase, valid_pose_commits))

        node._feedback = record_feedback
        observed_fences = []
        original_has_pose = runtime.has_pose_newer_than

        def record_fence(stamp_ns: int) -> bool:
            observed_fences.append(stamp_ns)
            return original_has_pose(stamp_ns)

        runtime.has_pose_newer_than = record_fence
        wait_calls = 0
        delayed_pose_accepted = []

        def event_wait(predicate, deadline: float):
            nonlocal valid_pose_commits, wait_calls
            wait_calls += 1
            if wait_calls not in (3, 4, 5):
                return True, ""
            self.assertTrue(services.pending.wait(timeout=1.0))
            self.assertIsNotNone(services.pending_pose)
            runtime.pose_callback(services.pending_pose)
            delayed_pose_accepted.append(predicate())
            accepted_fence = observed_fences[-1]
            valid_stamp = max(rospy.Time.now().to_nsec(), accepted_fence + 1)
            valid_pose = PoseWithCovarianceStamped()
            valid_pose.header.frame_id = "map"
            valid_pose.header.stamp = rospy.Time(
                secs=valid_stamp // 1_000_000_000,
                nsecs=valid_stamp % 1_000_000_000,
            )
            valid_pose.pose.covariance[0] = 0.01
            valid_pose.pose.covariance[7] = 0.01
            valid_pose.pose.covariance[35] = 0.01
            runtime.pose_callback(valid_pose)
            valid_pose_commits += 1
            services.pending.clear()
            return predicate(), ""

        node._wait = event_wait
        node._wait_for_policy_ready = lambda _token, _target, _marker: (True, "")

        # When: all three service responses precede release of their queued old-stamped pose.
        node._run_transaction("stair_a", "4F")

        # Then: none can satisfy an iteration or cause a premature AMCL_READY transition.
        self.assertEqual(delayed_pose_accepted, [False, False, False])
        self.assertEqual(services.nomotion_calls, 3)
        self.assertEqual(
            [commit_count for phase, commit_count in feedback if phase == "AMCL_READY"],
            [3],
        )
        self.assertEqual(len(node.initialpose_publisher.messages), 1)
        self.assertEqual(
            (
                node.initialpose_publisher.messages[0].pose.pose.position.x,
                node.initialpose_publisher.messages[0].pose.pose.position.y,
            ),
            (1.0, 2.0),
        )
        self.assertEqual(len(server.succeeded), 1)


if __name__ == "__main__":
    rostest.rosrun("multifloor_manager", "floor_transition_delayed_pose_ros", DelayedPoseRosTest)
