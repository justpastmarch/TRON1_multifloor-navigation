#!/usr/bin/env python3
# --- How to run ---
# rostest stair_supervisor stair_supervisor_node.test
"""Prove exact successful WebSocket frames on the real ROS topic boundary."""

from __future__ import annotations

import json
import threading
from typing import List
import unittest

import rospy
import rostest
from std_msgs.msg import String

from stair_supervisor.configuration import (
    MoveBaseLimits,
    RobotConfiguration,
    StairProfile,
    StairSupervisorConfiguration,
    WebSocketCalibration,
)
from stair_supervisor.msg import SupervisorState
from stair_supervisor.robot_config import (
    CommandStreamConfig,
    RobotConnectionConfig,
    RobotTransportConfig,
)
from stair_supervisor.robot_transport import RobotTransport
from stair_supervisor.ros_node import RosNodeSettings, RosStairSupervisorNode


class FakeClock:
    """Advance only when the transport asks for deterministic time."""

    def __init__(self) -> None:
        self.milliseconds = 1_700_000_000_000

    def now_ms(self) -> int:
        return self.milliseconds

    def monotonic(self) -> float:
        current = self.milliseconds / 1000.0
        self.milliseconds += 10
        return current

    def sleep(self, seconds: float) -> None:
        self.milliseconds += round(seconds * 1000.0)


class FakeWebSocket:
    """Capture raw frames and supply the documented mode handshake."""

    def __init__(self) -> None:
        self.sent_payloads: List[str] = []
        self.inbound: List[str] = []
        self.closed = False
        self.fail_sends = False

    def send(self, payload: str) -> None:
        self.sent_payloads.append(payload)
        if self.fail_sends:
            raise OSError("scripted send interruption")
        request = json.loads(payload)
        title = request["title"]
        if title == "request_twist":
            return
        self.inbound.append(
            json.dumps(
                {
                    "accid": request["accid"],
                    "title": title.replace("request_", "response_", 1),
                    "timestamp": request["timestamp"],
                    "guid": request["guid"],
                    "data": {"result": "success"},
                },
                separators=(",", ":"),
            )
        )
        if title in ("request_walk_mode", "request_stair_mode"):
            status = "WALK"
            if title == "request_stair_mode" and request["data"]["enable"]:
                status = "STAIR"
            self.inbound.append(
                json.dumps(
                    {
                        "accid": request["accid"],
                        "title": "notify_robot_info",
                        "timestamp": request["timestamp"],
                        "guid": "status-guid",
                        "data": {"status": status},
                    },
                    separators=(",", ":"),
                )
            )

    def recv(self) -> str:
        if not self.inbound:
            raise TimeoutError()
        return self.inbound.pop(0)

    def settimeout(self, _timeout: float) -> None:
        return

    def close(self) -> None:
        self.closed = True


class FakeFactory:
    """Expose one fake socket and count prohibited reconnect attempts."""

    def __init__(self, socket: FakeWebSocket) -> None:
        self.socket = socket
        self.calls = 0

    def __call__(self, _url: str, _timeout: float) -> FakeWebSocket:
        self.calls += 1
        return self.socket


def make_node(action_name: str, profile: StairProfile | None = None):
    socket = FakeWebSocket()
    factory = FakeFactory(socket)
    clock = FakeClock()
    transport = RobotTransport(
        RobotTransportConfig(
            connection=RobotConnectionConfig(
                accid="WF_QA_ZERO",
                url="ws://fake.invalid:5000",
                receive_timeout=0.01,
                request_timeout=0.1,
            ),
            stream=CommandStreamConfig(
                rate_hz=40.0,
                watchdog_sec=0.25,
                startup_zero_repeats=2,
                close_zero_repeats=4,
            ),
        ),
        WebSocketCalibration(0.5, 2.0),
        factory,
        clock,
    )
    configuration = StairSupervisorConfiguration(
        () if profile is None else (profile,),
        RobotConfiguration(
            MoveBaseLimits(0.35, 1.0, 0.2, 0.4, 2.0),
            WebSocketCalibration(0.5, 2.0),
            ("/navigation/cmd_vel",),
        ),
    )
    node = RosStairSupervisorNode(
        configuration,
        transport,
        RosNodeSettings(
            action_name,
            0.25,
            0.1,
            0.0,
            "/test/websocket_tx/odom",
        ),
    )
    return node, socket, factory, clock


class WebSocketTxRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("websocket_tx_surface_test", anonymous=True)

    def setUp(self) -> None:
        self.messages: List[str] = []
        self.lock = threading.Lock()
        self.subscriber = rospy.Subscriber(
            "/stair_supervisor/websocket_tx",
            String,
            self._receive,
            queue_size=100,
        )

    def tearDown(self) -> None:
        self.subscriber.unregister()

    def _receive(self, message: String) -> None:
        with self.lock:
            self.messages.append(message.data)

    def _snapshot(self) -> List[str]:
        with self.lock:
            return list(self.messages)

    def _await_count(self, count: int) -> None:
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while len(self._snapshot()) < count and rospy.Time.now() < deadline:
            rospy.sleep(0.01)
        self.assertGreaterEqual(len(self._snapshot()), count)

    def test_watchdog_and_close_success_frames_match_the_socket_bytes(self) -> None:
        # Given: the real ROS node boundary over a no-motion fake WebSocket.
        node, socket, factory, clock = make_node("/test/tx_success")
        while node._websocket_tx_publisher.get_num_connections() < 1:
            rospy.sleep(0.01)
        socket_start = len(socket.sent_payloads)
        topic_start = len(self._snapshot())

        # When: stale navigation emits watchdog zero and shutdown emits close zeros.
        clock.sleep(0.3)
        node.stream_once()
        node.shutdown()
        sent = socket.sent_payloads[socket_start:]
        self._await_count(topic_start + len(sent))
        observed = self._snapshot()[topic_start:topic_start + len(sent)]

        # Then: every successful frame is byte-equivalent and every twist is zero.
        self.assertEqual(observed, sent)
        twist_data = [
            json.loads(frame)["data"]
            for frame in observed
            if json.loads(frame)["title"] == "request_twist"
        ]
        self.assertTrue(twist_data)
        self.assertEqual(twist_data, [{"x": 0.0, "y": 0.0, "z": 0.0}] * len(twist_data))
        self.assertEqual(factory.calls, 1)
        self.assertTrue(socket.closed)

    def test_failed_send_is_not_published_and_latches_fault(self) -> None:
        # Given: a ready no-motion node whose next fake socket send will fail.
        node, socket, factory, _clock = make_node("/test/tx_failure")
        while node._websocket_tx_publisher.get_num_connections() < 1:
            rospy.sleep(0.01)
        published_before = len(self._snapshot())
        socket.fail_sends = True

        # When: the next zero stream tick is interrupted during send.
        node.stream_once()
        rospy.sleep(0.05)

        # Then: no false-success frame appears and the session cannot reconnect.
        self.assertEqual(len(self._snapshot()), published_before)
        self.assertEqual(node.state, SupervisorState.FAULT)
        self.assertEqual(factory.calls, 1)
        self.assertTrue(socket.closed)


if __name__ == "__main__":
    rostest.rosrun(
        "stair_supervisor",
        "websocket_tx_surface",
        WebSocketTxRosTest,
    )
