#!/usr/bin/env python3
# --- How to run ---
# rostest mission_manager full_system_synthetic.test
"""Drive a production-configured cross-floor mission with synthetic hardware."""

from __future__ import annotations

import json
from pathlib import Path
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from mission_manager.msg import MissionAction, MissionFeedback, MissionGoal, MissionResult
from multifloor_manager.msg import FloorState, FloorTransitionActionFeedback
import rospy
import rostest
from stair_supervisor.msg import SupervisorState
from std_msgs.msg import Bool

from synthetic_hardware_peers import SyntheticHardwarePeers, require_loopback_ros


class FullSystemSyntheticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        require_loopback_ros()
        rospy.init_node("full_system_synthetic_test", anonymous=True)
        fixture_root = Path(__file__).resolve().parents[2] / "test" / "fixtures" / "stair_synthetic"
        cls.hardware = SyntheticHardwarePeers(stair_config_dir=fixture_root)
        cls.floor_states: list[FloorState] = []
        cls.floor_phases: list[str] = []
        cls.debug_values: dict[str, bool] = {}
        cls.supervisor_states: list[SupervisorState] = []
        cls.floor_subscriber = rospy.Subscriber(
            "/multifloor/floor_state",
            FloorState,
            cls.floor_states.append,
            queue_size=20,
        )
        cls.floor_feedback_subscriber = rospy.Subscriber(
            "/multifloor/floor_transition/feedback",
            FloorTransitionActionFeedback,
            lambda message: cls.floor_phases.append(message.feedback.phase),
            queue_size=20,
        )
        cls.supervisor_subscriber = rospy.Subscriber(
            "/stair_supervisor/state",
            SupervisorState,
            cls.supervisor_states.append,
            queue_size=100,
        )
        cls.debug_subscribers = tuple(
            rospy.Subscriber(
                "/multifloor/debug/{}".format(name),
                Bool,
                lambda message, key=name: cls.debug_values.__setitem__(
                    key, message.data
                ),
                queue_size=1,
            )
            for name in (
                "amcl_pose_samples",
                "amcl_pose_fresh",
                "scan_fresh",
                "scan_tf_valid",
                "odom_stationary",
                "odom_fresh",
                "localization_ready",
            )
        )
        cls.client = actionlib.SimpleActionClient("/mission", MissionAction)
        if not cls.client.wait_for_server(rospy.Duration(15.0)):
            raise AssertionError("mission action server did not start")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.hardware.shutdown()

    def test_production_route_completes_after_state_freshness_window(self) -> None:
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while rospy.Time.now() < deadline:
            floor_ready = (
                self.floor_states
                and self.floor_states[-1].state == FloorState.READY
            )
            supervisor_ready = (
                self.supervisor_states
                and self.supervisor_states[-1].state == SupervisorState.NAV
            )
            if floor_ready and supervisor_ready:
                break
            rospy.sleep(0.02)
        self.assertTrue(floor_ready and supervisor_ready)
        floor_count = len(self.floor_states)
        supervisor_count = len(self.supervisor_states)
        rospy.sleep(2.2)
        feedback: list[MissionFeedback] = []
        self.client.send_goal(
            MissionGoal(
                destination_id="stair_4f_from_3f", mission_type="navigate"
            ),
            feedback_cb=feedback.append,
        )
        self.assertTrue(self.client.wait_for_result(rospy.Duration(45.0)))
        result: MissionResult = self.client.get_result()
        self.assertEqual(
            self.client.get_state(),
            GoalStatus.SUCCEEDED,
            "{}; floor phases={!r}; debug={!r}".format(
                result.reason,
                self.floor_phases,
                self.debug_values,
            ),
        )
        self.assertEqual(result.result_code, MissionResult.OK)
        self.assertEqual(
            tuple((item.segment_type, item.target_id) for item in feedback),
            (
                ("NAVIGATION", "stair_3f_up_entry"),
                ("STAIR", "stair_4f_from_3f"),
                ("FLOOR_TRANSITION", "stair_4f_from_3f"),
            ),
        )
        self.assertGreater(len(self.floor_states), floor_count)
        self.assertGreater(len(self.supervisor_states), supervisor_count)
        self.assertEqual(self.floor_states[-1].floor_id, "4F")
        frames = [json.loads(frame) for frame in self.hardware.websocket_frames]
        twists = [
            frame["data"] for frame in frames if frame["title"] == "request_twist"
        ]
        self.assertTrue(
            any(item["x"] != 0.0 or item["z"] != 0.0 for item in twists)
        )
        stair_mode_index = next(
            index
            for index, frame in enumerate(frames)
            if frame["title"] == "request_stair_mode" and frame["data"] == {"enable": True}
        )
        first_stair_motion = next(
            frame["data"]
            for frame in frames[stair_mode_index + 1:]
            if frame["title"] == "request_twist"
            and (frame["data"]["x"] != 0.0 or frame["data"]["z"] != 0.0)
        )
        self.assertEqual(first_stair_motion["z"], 0.0)
        self.assertEqual(len(self.hardware.navigation_goals), 1)


if __name__ == "__main__":
    rostest.rosrun(
        "mission_manager", "full_system_synthetic", FullSystemSyntheticTest
    )
