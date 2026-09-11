#!/usr/bin/env python3
"""Synthetic no-motion 5F/RF route and failure tests through Mission.action."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from mission_manager.msg import MissionAction, MissionFeedback, MissionGoal, MissionResult
from multifloor_manager.msg import FloorState
import rospy
from std_srvs.srv import Trigger


@dataclass(frozen=True)  # noqa: SLOTS_OK - ROS Noetic uses Python 3.8.
class FailureCase:
    __slots__ = ("scenario", "result_code", "reason")
    scenario: str
    result_code: int
    reason: str


class Mission5fRfSyntheticRosTest(unittest.TestCase):
    """Drive invented 5F/RF data only through the public mission action."""

    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("mission_5f_rf_synthetic_test", anonymous=True)
        cls.client = actionlib.SimpleActionClient("/mission", MissionAction)
        if not cls.client.wait_for_server(rospy.Duration(10.0)):
            raise AssertionError("synthetic mission action server did not start")
        rospy.wait_for_service("/mission_test/synchronize", timeout=10.0)
        cls.synchronize = rospy.ServiceProxy("/mission_test/synchronize", Trigger)
        cls.state_condition = threading.Condition()
        cls.floor_updates = 0
        cls.floor_id = ""

        def floor_state_callback(message: FloorState) -> None:
            with cls.state_condition:
                cls.floor_updates += 1
                cls.floor_id = message.floor_id
                cls.state_condition.notify_all()

        cls.floor_state_subscriber = rospy.Subscriber(
            "/multifloor/floor_state", FloorState, floor_state_callback, queue_size=10
        )

    def setUp(self) -> None:
        self._set_scenario("default")

    def _set_scenario(self, name: str) -> None:
        with self.state_condition:
            baseline_updates = self.floor_updates
        rospy.set_param("/mission_test/scenario", name)
        response = self.synchronize()
        self.assertTrue(response.success, response.message)
        expected_floor = response.message.rsplit("floor=", 1)[1]
        deadline = time.monotonic() + 5.0
        with self.state_condition:
            while self.floor_updates <= baseline_updates or self.floor_id != expected_floor:
                remaining = deadline - time.monotonic()
                self.assertGreater(remaining, 0.0, response.message)
                self.state_condition.wait(remaining)

    def _run(self, mission_type: str = "record_route") -> tuple[int, MissionResult, list[MissionFeedback]]:
        feedback: list[MissionFeedback] = []
        self.client.send_goal(
            MissionGoal(
                destination_id="roof_loop_complete",
                mission_type=mission_type,
                return_after_task=True,
            ),
            feedback_cb=feedback.append,
        )
        if not self.client.wait_for_result(rospy.Duration(20.0)):
            self.client.cancel_goal()
            raise AssertionError("synthetic mission did not terminate")
        return self.client.get_state(), self.client.get_result(), feedback

    def _restore_home(self) -> None:
        self._set_scenario("default")
        self.client.send_goal(MissionGoal(destination_id="home_5f", mission_type="navigate"))
        self.assertTrue(self.client.wait_for_result(rospy.Duration(20.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.SUCCEEDED)
        self.assertEqual(self.client.get_result().result_code, MissionResult.OK)

    def _assert_child_failure(self, case: FailureCase) -> None:
        self._set_scenario(case.scenario)
        state, result, _feedback = self._run()
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(result.result_code, case.result_code)
        self.assertIn(case.reason, result.reason)
        self._restore_home()

    def test_exact_route_uses_fresh_return_after_roof_scan(self) -> None:
        # Given/When: the synthetic roof-loop recording requests a return home.
        state, result, feedback = self._run()

        # Then: every expanded segment crosses Mission.action in exact route order.
        expected = (
            ("NAVIGATION", "stair_entry_5f"),
            ("STAIR", "roof_landing"),
            ("FLOOR_TRANSITION", "roof_landing"),
            ("NAVIGATION", "roof_loop_north"),
            ("NAVIGATION", "roof_loop_east"),
            ("NAVIGATION", "roof_loop_south"),
            ("NAVIGATION", "roof_loop_complete"),
            ("NAVIGATION", "roof_return_entry"),
            ("STAIR", "stair_return_5f"),
            ("FLOOR_TRANSITION", "stair_return_5f"),
            ("NAVIGATION", "home_5f"),
        )
        actual = tuple((item.segment_type, item.target_id) for item in feedback)
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)
        self.assertTrue(result.artifact_path.endswith(".bag"))
        self.assertEqual(actual, expected)
        self.assertEqual(tuple(item.segment_count for item in feedback), (11,) * 11)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 7)
        self.assertEqual(rospy.get_param("/mission_test/stair_count"), 2)
        self.assertEqual(rospy.get_param("/mission_test/floor_count"), 2)

    def test_navigation_abort_is_injected_alone(self) -> None:
        self._assert_child_failure(
            FailureCase("nav_failure", MissionResult.NAVIGATION_FAILED, "navigation failed")
        )

    def test_stale_localization_is_injected_alone(self) -> None:
        self._assert_child_failure(
            FailureCase("stale_localization", MissionResult.LOCALIZATION_FAILED, "stale localization")
        )

    def test_wrong_floor_tag_is_injected_alone(self) -> None:
        self._assert_child_failure(
            FailureCase("wrong_floor_tag", MissionResult.LOCALIZATION_FAILED, "wrong-floor tag")
        )

    def test_odometry_jump_is_injected_alone(self) -> None:
        self._assert_child_failure(
            FailureCase("odometry_jump", MissionResult.STAIR_FAILED, "odometry jump")
        )

    def test_websocket_loss_is_injected_alone(self) -> None:
        self._assert_child_failure(
            FailureCase("websocket_loss", MissionResult.MISSION_ABORT, "WebSocket loss")
        )

    def test_scan_record_failure_is_injected_alone(self) -> None:
        self._set_scenario("scan_failure")
        state, result, feedback = self._run(mission_type="inspect")
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(result.result_code, MissionResult.SCAN_FAILED)
        self.assertEqual(feedback[-1].segment_type, "SCAN")
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 5)
        self._restore_home()

    def test_cancellation_is_injected_alone(self) -> None:
        self._set_scenario("hold")
        dispatched = threading.Event()
        self.client.send_goal(
            MissionGoal(
                destination_id="roof_loop_complete",
                mission_type="inspect",
                return_after_task=True,
            ),
            feedback_cb=lambda _message: dispatched.set(),
        )
        self.assertTrue(dispatched.wait(5.0))
        self.client.cancel_goal()
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)
        self.assertEqual(self.client.get_result().result_code, MissionResult.MISSION_ABORT)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(
        "mission_manager",
        "mission_5f_rf_synthetic",
        Mission5fRfSyntheticRosTest,
    )
