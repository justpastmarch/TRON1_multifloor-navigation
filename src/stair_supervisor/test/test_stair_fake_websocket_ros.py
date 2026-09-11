#!/usr/bin/env python3
# --- How to run ---
# rostest stair_supervisor stair_supervisor_node.test
"""Drive the real stair action over synthetic sensors and a fake WebSocket."""

from __future__ import annotations

import json
from typing import List
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
import rostest

from stair_supervisor.configuration import Direction, StairProfile
from stair_supervisor.msg import (
    StairTraversalAction,
    StairTraversalFeedback,
    StairTraversalGoal,
    StairTraversalResult,
    SupervisorState,
)
from stair_supervisor.stair_evidence import Phase

from stair_sensor_fixture import SyntheticStairSensors
from test_websocket_tx_ros import make_node


def synthetic_profile() -> StairProfile:
    return StairProfile(
        "fake_ws_up", Direction.UP, True, 0.12, 0.20,
        0.40, 1.00, 0.20, 0.50, 0.80, 0.20,
        0.02, 0.02, 0.20, 0.05, 0.20, 0.15, 0.40, 0.30, 4.0,
    )


class StairFakeWebSocketRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("stair_fake_websocket_qa", anonymous=True)

    def test_action_feedback_state_and_tx_complete_without_robot_motion(self) -> None:
        # Given: the real ROS/action/transport boundaries over an in-memory socket.
        action_name = "/test/fake_websocket/stair_traversal"
        node, socket, factory, _clock = make_node(action_name, synthetic_profile())
        self.addCleanup(node.shutdown)
        sensors = SyntheticStairSensors(
            "/test/websocket_tx/odom",
            "/test/websocket_tx/imu",
        )
        sensors.wait_for_connections()
        feedback: List[StairTraversalFeedback] = []
        states: List[SupervisorState] = []
        state_subscriber = rospy.Subscriber(
            rospy.resolve_name("~state"),
            SupervisorState,
            states.append,
            queue_size=100,
        )
        self.addCleanup(state_subscriber.unregister)
        client = actionlib.SimpleActionClient(action_name, StairTraversalAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(2.0)))
        published = set()

        def supply_evidence(message: StairTraversalFeedback) -> None:
            feedback.append(message)
            phase = Phase(message.phase)
            if phase not in published:
                published.add(phase)
                sensors.publish_phase(phase, Direction.UP)

        # When: a complete UP traversal is submitted through the generated action.
        client.send_goal(
            StairTraversalGoal(stair_id="fake_ws_up", direction=StairTraversalGoal.UP),
            feedback_cb=supply_evidence,
        )
        self.assertTrue(client.wait_for_result(rospy.Duration(8.0)))
        result: StairTraversalResult = client.get_result()
        rospy.sleep(0.05)

        # Then: all phases, structured evidence, mode state, and socket TX are observable.
        self.assertEqual(client.get_state(), GoalStatus.SUCCEEDED, result.reason)
        self.assertEqual(result.result_code, StairTraversalResult.OK)
        self.assertEqual(list(dict.fromkeys(item.phase for item in feedback)), [phase.value for phase in Phase])
        self.assertTrue(all("progress=" in item.detail and "freshness=" in item.detail for item in feedback))
        self.assertIn(SupervisorState.STAIR, [item.state for item in states])
        self.assertEqual(node.state, SupervisorState.NAV)
        requests = [json.loads(payload) for payload in socket.sent_payloads]
        twists = [request["data"] for request in requests if request["title"] == "request_twist"]
        self.assertTrue(any(data["x"] != 0.0 or data["z"] != 0.0 for data in twists))
        self.assertEqual(twists[-1], {"x": 0.0, "y": 0.0, "z": 0.0})
        self.assertEqual(factory.calls, 1)

if __name__ == "__main__":
    rostest.rosrun(
        "stair_supervisor",
        "stair_fake_websocket_qa",
        StairFakeWebSocketRosTest,
    )
