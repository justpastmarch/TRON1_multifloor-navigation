#!/usr/bin/env python3
"""Drive the operational mission manager through its real Mission.action surface."""

from __future__ import annotations

import threading
import time
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
import rostest

from mission_manager.msg import MissionAction, MissionFeedback, MissionGoal, MissionResult
from multifloor_manager.msg import FloorState
from std_srvs.srv import Trigger


class MissionActionRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("mission_action_ros_test", anonymous=True)
        cls.client = actionlib.SimpleActionClient("/mission", MissionAction)
        if not cls.client.wait_for_server(rospy.Duration(10.0)):
            raise AssertionError("mission action server did not start")
        cls.secondary = actionlib.SimpleActionClient("/mission", MissionAction)
        if not cls.secondary.wait_for_server(rospy.Duration(10.0)):
            raise AssertionError("secondary mission action client did not connect")
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
            "/multifloor/floor_state",
            FloorState,
            floor_state_callback,
            queue_size=10,
        )

    def setUp(self) -> None:
        self._restore_home()
        # Returning to a logical home now performs real NAV even if its ID was
        # already the anchor. Exclude fixture setup from each scenario's count.
        self._set_scenario("default")

    def test_same_named_anchor_still_dispatches_navigation(self) -> None:
        state, result, _ = self._run("home_3f")
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 1)

    def _set_scenario(self, name: str) -> None:
        with self.state_condition:
            baseline_updates = self.floor_updates
        rospy.set_param("/mission_test/scenario", name)
        response = self.synchronize()
        self.assertTrue(response.success, response.message)
        self.assertIn("scenario={}".format(name), response.message)
        expected_floor = response.message.rsplit("floor=", 1)[1]
        deadline = time.monotonic() + 5.0
        with self.state_condition:
            while self.floor_updates <= baseline_updates or self.floor_id != expected_floor:
                remaining = deadline - time.monotonic()
                self.assertGreater(remaining, 0.0, response.message)
                self.state_condition.wait(remaining)

    def _run(
        self,
        destination: str,
        mission_type: str = "navigate",
        return_after_task: bool = False,
    ) -> tuple[int, MissionResult, list[MissionFeedback]]:
        feedback: list[MissionFeedback] = []
        self.client.send_goal(
            MissionGoal(
                destination_id=destination,
                mission_type=mission_type,
                return_after_task=return_after_task,
            ),
            feedback_cb=feedback.append,
        )
        if not self.client.wait_for_result(rospy.Duration(20.0)):
            self.client.cancel_goal()
            raise AssertionError("mission did not terminate")
        return self.client.get_state(), self.client.get_result(), feedback

    def _restore_home(self) -> None:
        self._set_scenario("default")
        state, result, _ = self._run("home_3f")
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)

    def test_busy_goal_is_rejected_without_preempting_active_goal(self) -> None:
        # Given: a mission blocked in its first real move_base child action.
        self._set_scenario("hold")
        dispatched = threading.Event()
        self.client.send_goal(
            MissionGoal(destination_id="stair_a_entry_3f", mission_type="navigate"),
            feedback_cb=lambda _message: dispatched.set(),
        )
        self.assertTrue(dispatched.wait(5.0))

        # When: a second client submits a well-formed goal.
        self.secondary.send_goal(
            MissionGoal(destination_id="home_3f", mission_type="navigate")
        )
        self.assertTrue(self.secondary.wait_for_result(rospy.Duration(5.0)))

        # Then: BUSY is rejected and the original remains cancellable, not superseded.
        self.assertEqual(self.secondary.get_state(), GoalStatus.REJECTED)
        self.assertEqual(self.secondary.get_result().result_code, MissionResult.BUSY)
        self.client.cancel_goal()
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)

    def test_cancellation_then_resume_uses_confirmed_anchor(self) -> None:
        # Given: navigation is active but has not confirmed its target.
        self._set_scenario("hold")
        dispatched = threading.Event()
        self.client.send_goal(
            MissionGoal(destination_id="stair_a_entry_3f", mission_type="navigate"),
            feedback_cb=lambda _message: dispatched.set(),
        )
        self.assertTrue(dispatched.wait(5.0))

        # When: the action is cancelled and a replacement mission is submitted.
        self.client.cancel_goal()
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))
        cancelled_state = self.client.get_state()
        self._set_scenario("default")
        state, result, _ = self._run("stair_a_entry_3f", return_after_task=True)

        # Then: cancellation is coherent and the replacement starts from home.
        self.assertEqual(cancelled_state, GoalStatus.PREEMPTED)
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 1)

    def test_communication_failure_aborts_before_floor_transition(self) -> None:
        # Given: the stair child reports transport communication loss.
        self._set_scenario("communication_failure")

        # When: a multi-floor mission reaches that child.
        state, result, _ = self._run("stair_a_landing_4f")

        # Then: mission abort is coherent and no localization action starts.
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(result.result_code, MissionResult.MISSION_ABORT)
        self.assertEqual(rospy.get_param("/mission_test/floor_count"), 0)
        self._restore_home()

    def test_dirty_scan_artifact_is_rejected(self) -> None:
        # Given: rosbag metadata omits one mandatory fixture topic.
        self._set_scenario("dirty_artifact")

        # When: the inspect mission validates its finalized artifact.
        state, result, _ = self._run("roof_scan", "inspect", True)

        # Then: it fails as SCAN_FAILED without executing return navigation.
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(result.result_code, MissionResult.SCAN_FAILED)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 3)
        self._restore_home()

    def test_fresh_return_completes_with_validated_scan_artifact(self) -> None:
        # Given: the asymmetric configured inspect mission.
        # When: it runs outbound, records, and requests return.
        state, result, feedback = self._run("roof_scan", "inspect", True)

        # Then: return includes its graph-only return_4f anchor and a real artifact path.
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)
        self.assertTrue(result.artifact_path.endswith(".bag"))
        self.assertIn("return_4f", [item.target_id for item in feedback])
        self.assertEqual(feedback[-1].segment_index, feedback[-1].segment_count - 1)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 5)

    def test_localization_failure_stops_before_next_navigation(self) -> None:
        # Given: floor transition cannot establish localization.
        self._set_scenario("localization_failure")

        # When: the mission reaches the transition action.
        state, result, _ = self._run("stair_a_landing_4f")

        # Then: the precise code propagates and no later NAV starts.
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(result.result_code, MissionResult.LOCALIZATION_FAILED)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 1)
        self._restore_home()

    def test_misleading_child_success_is_mission_abort(self) -> None:
        # Given: floor action status says SUCCEEDED while its result says failure.
        self._set_scenario("incoherent_floor")

        # When: the mission validates the child's terminal pair.
        state, result, _ = self._run("stair_a_landing_4f")

        # Then: incoherence is fail-closed and no subsequent segment executes.
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(result.result_code, MissionResult.MISSION_ABORT)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 1)
        self._restore_home()

    def test_malformed_and_unknown_goals_are_rejected_before_children(self) -> None:
        # Given: malformed destination and unknown mission type goals.
        goals = (
            MissionGoal(destination_id="." * 2 + "/roof", mission_type="navigate"),
            MissionGoal(destination_id="home_3f", mission_type="unknown"),
            MissionGoal(destination_id="missing", mission_type="navigate"),
        )

        # When/Then: each is rejected as INVALID_GOAL without a child goal.
        for goal in goals:
            self.client.send_goal(goal)
            self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))
            self.assertEqual(self.client.get_state(), GoalStatus.REJECTED)
            self.assertEqual(self.client.get_result().result_code, MissionResult.INVALID_GOAL)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 0)

    def test_multi_floor_mission_executes_each_child_once(self) -> None:
        # Given/When: a 3F-to-4F mission runs and freshly returns home.
        state, result, feedback = self._run(
            "stair_a_landing_4f", return_after_task=True
        )

        # Then: NAV, STAIR, and FLOOR_TRANSITION all cross ROS action boundaries.
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 1)
        self.assertEqual(rospy.get_param("/mission_test/stair_count"), 2)
        self.assertEqual(rospy.get_param("/mission_test/floor_count"), 2)
        self.assertIn("FLOOR_TRANSITION", [item.segment_type for item in feedback])

    def test_navigation_retry_is_bounded_and_succeeds(self) -> None:
        # Given: move_base aborts the first attempt only.
        self._set_scenario("nav_retry")

        # When: a same-floor mission executes.
        state, result, feedback = self._run("stair_a_entry_3f", return_after_task=True)

        # Then: one clear/retry occurs and action/result remain successful.
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 2)
        self.assertEqual(rospy.get_param("/mission_test/clear_count"), 1)
        self.assertEqual(feedback[0].segment_index, 0)

    def test_same_floor_feedback_has_coherent_index_count_and_floor(self) -> None:
        # Given/When: one same-floor outbound segment executes.
        state, result, feedback = self._run("stair_a_entry_3f")

        # Then: the Mission.action feedback describes that exact segment.
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)
        self.assertEqual(len(feedback), 1)
        self.assertEqual(feedback[0].current_floor, "3F")
        self.assertEqual(feedback[0].segment_index, 0)
        self.assertEqual(feedback[0].segment_count, 1)
        self._restore_home()

    def test_stale_state_aborts_before_child_goal(self) -> None:
        # Given: state publishers stop beyond the mission freshness bound.
        self._set_scenario("stale_state")
        rospy.sleep(0.7)

        # When: a valid mission is submitted with stale cached state.
        state, result, _ = self._run("stair_a_entry_3f")

        # Then: the mission aborts before move_base receives a goal.
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(result.result_code, MissionResult.MISSION_ABORT)
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 0)

    def test_repeated_interruptions_do_not_corrupt_next_mission(self) -> None:
        # Given: three independent missions are interrupted before NAV success.
        self._set_scenario("hold")
        for _ in range(3):
            dispatched = threading.Event()
            self.client.send_goal(
                MissionGoal(destination_id="stair_a_entry_3f", mission_type="navigate"),
                feedback_cb=lambda _message: dispatched.set(),
            )
            self.assertTrue(dispatched.wait(5.0))
            self.client.cancel_goal()
            self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))
            self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)

        # When: a later mission runs normally.
        self._set_scenario("default")
        state, result, _ = self._run("stair_a_entry_3f", return_after_task=True)

        # Then: no stale terminal state or cancellation poisons the new goal.
        self.assertEqual(state, GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)

    def test_scan_failure_never_launches_fresh_return(self) -> None:
        # Given: rosbag exits before its configured duration.
        self._set_scenario("scan_failure")

        # When: a return-capable inspect mission reaches SCAN.
        state, result, feedback = self._run("roof_scan", "inspect", True)

        # Then: SCAN_FAILED terminates at outbound count with no return targets.
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(
            result.result_code,
            MissionResult.SCAN_FAILED,
            "reason={!r} nav_count={} feedback={}".format(
                result.reason,
                rospy.get_param("/mission_test/nav_count"),
                [(item.segment_type, item.target_id) for item in feedback],
            ),
        )
        self.assertNotIn("return_4f", [item.target_id for item in feedback])
        self.assertEqual(rospy.get_param("/mission_test/nav_count"), 3)
        self._restore_home()

    def test_stair_failure_propagates_and_does_not_start_localization(self) -> None:
        # Given: stair traversal reaches a terminal hardware failure.
        self._set_scenario("stair_failure")

        # When: a multi-floor mission runs.
        state, result, _ = self._run("stair_a_landing_4f")

        # Then: STAIR_FAILED propagates and floor transition never starts.
        self.assertEqual(state, GoalStatus.ABORTED)
        self.assertEqual(result.result_code, MissionResult.STAIR_FAILED)
        self.assertEqual(rospy.get_param("/mission_test/floor_count"), 0)
        self._restore_home()


if __name__ == "__main__":
    rostest.rosrun("mission_manager", "mission_action_ros", MissionActionRosTest)
