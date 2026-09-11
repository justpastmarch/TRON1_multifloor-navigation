#!/usr/bin/env python3
"""Manual QA client that records one Mission.action transcript as JSON evidence."""

from __future__ import annotations

import json
from pathlib import Path
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
import rostest

from mission_manager.msg import MissionAction, MissionFeedback, MissionGoal, MissionResult


class ManualMissionClient(unittest.TestCase):
    def test_outbound_scan_and_fresh_return(self) -> None:
        # Given: the operational node and process-owned fixture children.
        client = actionlib.SimpleActionClient("/mission", MissionAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(10.0)))
        feedback: list[MissionFeedback] = []

        # When: a valid fixture inspect goal crosses Mission.action.
        client.send_goal(
            MissionGoal(
                destination_id="roof_scan",
                mission_type="inspect",
                return_after_task=True,
            ),
            feedback_cb=feedback.append,
        )
        self.assertTrue(client.wait_for_result(rospy.Duration(20.0)))
        result: MissionResult = client.get_result()

        # Then: write machine-readable proof of outbound, scan, and fresh return.
        self.assertEqual(client.get_state(), GoalStatus.SUCCEEDED)
        self.assertEqual(result.result_code, MissionResult.OK)
        targets = [item.target_id for item in feedback]
        self.assertIn("roof_scan", targets)
        self.assertIn("return_4f", targets)
        output = Path(rospy.get_param("~artifact_output"))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(
                {
                    "action": "/mission",
                    "goal": {
                        "destination_id": "roof_scan",
                        "mission_type": "inspect",
                        "return_after_task": True,
                    },
                    "terminal_status": int(client.get_state()),
                    "result_code": int(result.result_code),
                    "mission_id": result.mission_id,
                    "scan_artifact": result.artifact_path,
                    "fresh_return_proved": "return_4f" in targets,
                    "feedback": [
                        {
                            "state": item.state,
                            "floor": item.current_floor,
                            "segment_type": item.segment_type,
                            "segment_index": int(item.segment_index),
                            "segment_count": int(item.segment_count),
                            "target_id": item.target_id,
                        }
                        for item in feedback
                    ],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )


if __name__ == "__main__":
    rospy.init_node("manual_mission_client", anonymous=True)
    rostest.rosrun("mission_manager", "manual_mission_client", ManualMissionClient)
