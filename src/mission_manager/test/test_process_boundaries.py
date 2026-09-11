#!/usr/bin/env python3
# --- How to run ---
# rostest mission_manager process_boundaries.test
"""Drive test-only ROS and WebSocket peers through real process boundaries."""

from __future__ import annotations

from contextlib import closing
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from typing import NamedTuple
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
import rostest
from std_srvs.srv import Trigger
from websocket import create_connection

from mission_manager.msg import MissionAction, MissionGoal, MissionResult


class SocketReply(NamedTuple):
    ok: bool
    error: str
    response: str


def _parse_reply(text: str) -> SocketReply:
    document = json.loads(text)
    if not isinstance(document, dict):
        raise AssertionError("WebSocket reply must be an object")
    ok = document.get("ok")
    error = document.get("error", "")
    response = document.get("response", "")
    if not isinstance(ok, bool) or not isinstance(error, str) or not isinstance(response, str):
        raise AssertionError("WebSocket reply fields have invalid types")
    return SocketReply(ok, error, response)


class ProcessBoundaryTest(unittest.TestCase):
    server_process: subprocess.Popen[str]
    action_client: actionlib.SimpleActionClient
    websocket_url: str

    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("todo4_process_boundary_test", anonymous=True)
        server_path = Path(__file__).resolve().parent / "mock_peer_server.py"
        environment = dict(os.environ)
        environment["NO_PROXY"] = "127.0.0.1,localhost"
        environment["no_proxy"] = "127.0.0.1,localhost"
        cls.server_process = subprocess.Popen(
            [sys.executable, str(server_path)],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        rospy.wait_for_service("/todo4/mock_change_map", timeout=10.0)
        cls.action_client = actionlib.SimpleActionClient("/todo4/mock_mission", MissionAction)
        if not cls.action_client.wait_for_server(rospy.Duration(10.0)):
            raise AssertionError("mock action server did not start")
        port = rospy.get_param("/todo4_mock_peer_server/websocket_port")
        cls.websocket_url = f"ws://127.0.0.1:{port}/"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server_process.send_signal(signal.SIGINT)
        try:
            cls.server_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            cls.server_process.kill()
            cls.server_process.wait(timeout=5.0)

    def _request(self, payload: str) -> SocketReply:
        with closing(create_connection(self.websocket_url, timeout=3.0)) as connection:
            connection.send(payload)
            return _parse_reply(connection.recv())

    def test_ros_action_round_trip_across_server_process(self) -> None:
        # Given: a generated Mission action client connected to the mock process.
        goal = MissionGoal(destination_id="roof", mission_type="inspect", return_after_task=False)

        # When: the goal crosses ROS transport and completes.
        self.action_client.send_goal(goal)
        completed = self.action_client.wait_for_result(rospy.Duration(5.0))
        result: MissionResult = self.action_client.get_result()

        # Then: the real action endpoint returns the requested mission identity.
        self.assertTrue(completed)
        self.assertEqual(self.action_client.get_state(), GoalStatus.SUCCEEDED)
        self.assertEqual(result.mission_id, "roof")

    def test_ros_service_round_trip_across_server_process(self) -> None:
        # Given: a proxy connected to the process-owned Trigger service.
        service = rospy.ServiceProxy("/todo4/mock_change_map", Trigger)

        # When: the service request crosses ROS transport.
        response = service()

        # Then: the mock process acknowledges the request.
        self.assertTrue(response.success)
        self.assertEqual(response.message, "accepted")

    def test_fresh_websocket_round_trip_over_loopback_socket(self) -> None:
        # Given: a request inside the server's deterministic freshness window.
        # When: the request crosses an RFC6455 loopback connection.
        reply = self._request('{"request":"ping","stamp_ns":1700000009000000000}')

        # Then: the socket server returns a parsed success response.
        self.assertTrue(reply.ok)
        self.assertEqual(reply.response, "pong")

    def test_malformed_input_is_rejected_over_loopback_socket(self) -> None:
        # Given/When: malformed JSON crosses the WebSocket process boundary.
        reply = self._request("{")

        # Then: the server identifies malformed input without crashing.
        self.assertFalse(reply.ok)
        self.assertEqual(reply.error, "malformed_input")

    def test_stale_state_is_rejected_over_loopback_socket(self) -> None:
        # Given/When: an old timestamp crosses the WebSocket process boundary.
        reply = self._request('{"request":"ping","stamp_ns":1699999990000000000}')

        # Then: the deterministic server clock rejects stale state.
        self.assertFalse(reply.ok)
        self.assertEqual(reply.error, "stale_state")

    def test_repeated_interruptions_do_not_fake_success(self) -> None:
        # Given: three clients that disconnect immediately after upgrading.
        for _ in range(3):
            connection = create_connection(self.websocket_url, timeout=3.0)
            connection.close()

        # When: a later client sends a complete request.
        reply = self._request('{"request":"ping","stamp_ns":1700000009000000000}')

        # Then: only the complete request receives success.
        self.assertTrue(reply.ok)
        self.assertEqual(reply.response, "pong")


if __name__ == "__main__":
    rostest.rosrun("mission_manager", "todo4_process_boundaries", ProcessBoundaryTest)
