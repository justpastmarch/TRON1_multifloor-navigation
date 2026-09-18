#!/usr/bin/env python3
# --- How to run ---
# rostest stair_supervisor stair_supervisor_node.test
"""Exercise the real ROS action/topic boundary with a mocked robot transport."""

from __future__ import annotations

from dataclasses import replace
import threading
from typing import List
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
import rospy
import rostest
from std_msgs.msg import String

from stair_supervisor.configuration import (
    Direction,
    MoveBaseLimits,
    RobotConfiguration,
    StairProfile,
    StairSupervisorConfiguration,
    WebSocketCalibration,
)
from stair_supervisor.msg import (
    StairTraversalAction,
    StairTraversalFeedback,
    StairTraversalGoal,
    StairTraversalResult,
    SupervisorState,
)
from stair_supervisor.ros_node import RosNodeSettings, RosStairSupervisorNode
from stair_supervisor.stair_admission import AllowStairAdmissionValidator
from stair_supervisor.stair_evidence import Phase

from stair_sensor_fixture import SyntheticStairSensors


class MockTransport:
    """Record the sole node-owned command session without opening a socket."""

    def __init__(self) -> None:
        self.events: List[tuple] = []
        self.stream_period_sec = 0.01
        self._lock = threading.Lock()
        self._sent_observer = lambda _frame: None

    def _record(self, event: tuple) -> None:
        with self._lock:
            self.events.append(event)

    def start(self) -> None:
        self._record(("start",))

    def update_twist(self, linear_mps: float, angular_radps: float) -> None:
        self._record(("update", linear_mps, angular_radps))

    def send_current(self) -> None:
        self._record(("send",))

    def observe_sent_frames(self, observer) -> None:
        self._sent_observer = observer

    def emit_success(self, frame: str) -> None:
        self._sent_observer(frame)

    def request_stair_mode(self, enabled: bool) -> None:
        self._record(("stair", enabled))

    def close(self) -> None:
        self._record(("close",))

    def snapshot(self) -> List[tuple]:
        with self._lock:
            return list(self.events)


def configuration() -> StairSupervisorConfiguration:
    up = StairProfile(
        "up", Direction.UP, True, 0.12, 0.25,
        0.40, 1.00, 0.20, 0.50, 0.80, 0.20,
        0.02, 0.02, 1.00, 1.00, 0.40, 0.30, 2.0,
    )
    profiles = (
        up,
        replace(
            up, id="down", direction=Direction.DOWN, linear_speed=0.10,
            angular_speed=0.20,             alignment_yaw_rad=0.0,
            flight_1_distance_m=-1.00, landing_turn_yaw_rad=-0.50,
            flight_2_distance_m=-0.80,
        ),
        replace(up, id="disabled", direction=Direction.DOWN, enabled=False),
        replace(up, id="timeout", timeout_sec=0.08),
    )
    robot = RobotConfiguration(
        MoveBaseLimits(0.35, 1.0, 0.2, 0.4, 2.0),
        WebSocketCalibration(0.55, 1.8),
        ("/navigation/cmd_vel",),
    )
    return StairSupervisorConfiguration(profiles, robot)


class StairSupervisorNodeBoundaryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("stair_supervisor_node_boundary_test", anonymous=True)
        cls.transport = MockTransport()
        cls.tx_messages = []
        cls.tx_subscriber = rospy.Subscriber(
            "/stair_supervisor/websocket_tx",
            String,
            lambda message: cls.tx_messages.append(message.data),
            queue_size=20,
        )
        cls.odom_topic = "/test/stair_evidence/odom"
        cls.node = RosStairSupervisorNode(
            configuration(),
            cls.transport,
            RosNodeSettings(
                "/test/stair_traversal",
                0.25,
                100.0,
                0.0,
                cls.odom_topic,
            ),
            AllowStairAdmissionValidator(),
        )
        cls.client = actionlib.SimpleActionClient(
            "/test/stair_traversal",
            StairTraversalAction,
        )
        if not cls.client.wait_for_server(rospy.Duration(5.0)):
            raise AssertionError("stair action server did not start")
        cls.nav = rospy.Publisher("/navigation/cmd_vel", Twist, queue_size=1)
        cls.sensors = SyntheticStairSensors(cls.odom_topic)
        cls.sensors.wait_for_connections()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.shutdown()

    def _publish_evidence(self, phase: Phase, direction: Direction) -> None:
        self.sensors.publish_phase(phase, direction)

    def test_successful_transport_frame_is_published_byte_for_byte(self) -> None:
        # Given: an exact serialized frame reported by the node-owned transport.
        frame = '{"guid":"qa-zero","title":"request_twist","data":{"x":0.0,"y":0.0,"z":0.0}}'

        # When: the transport reports that the socket send succeeded.
        self.transport.emit_success(frame)

        # Then: the existing node publishes the same bytes as std_msgs/String data.
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while frame not in self.tx_messages and rospy.Time.now() < deadline:
            rospy.sleep(0.01)
        self.assertIn(frame, self.tx_messages)

    def test_action_transcript_covers_ownership_and_fail_closed_paths(self) -> None:
        # Given: one node-owned transport and an enabled profile.
        feedback: List[str] = []
        feedback_details: List[str] = []
        completed = set()

        def complete_phase(message: StairTraversalFeedback) -> None:
            feedback.append(message.phase)
            feedback_details.append(message.detail)
            command = Twist()
            command.linear.x = 0.31
            self.nav.publish(command)
            phase = Phase(message.phase)
            if phase not in completed:
                completed.add(phase)
                self._publish_evidence(phase, Direction.UP)

        # When: the generated action crosses ROS and completes all phases.
        self.client.send_goal(
            StairTraversalGoal(stair_id="up", direction=StairTraversalGoal.UP),
            feedback_cb=complete_phase,
        )
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))
        result: StairTraversalResult = self.client.get_result()

        # Then: profile order and command ownership are observable on the transcript.
        self.assertEqual(
            self.client.get_state(),
            GoalStatus.SUCCEEDED,
            f"{result.reason}; feedback={feedback}; details={feedback_details[-8:]}",
        )
        self.assertEqual(result.result_code, StairTraversalResult.OK)
        self.assertEqual(list(dict.fromkeys(feedback)), [phase.value for phase in Phase])
        events = self.transport.snapshot()
        start = events.index(("stair", True))
        end = events.index(("stair", False))
        self.assertNotIn(("update", 0.31, 0.0), events[start:end])
        self.assertEqual(events[start - 2:start], [("update", 0.0, 0.0), ("send",)])
        self.assertEqual(events[end - 2:end], [("update", 0.0, 0.0), ("send",)])
        self.node.stream_once()
        self.assertEqual(self.transport.snapshot()[-2], ("update", 0.0, 0.0))

        # Given/When: an enabled DOWN profile crosses the same generated action.
        down_feedback: List[str] = []
        down_completed = set()

        def complete_down(message: StairTraversalFeedback) -> None:
            down_feedback.append(message.phase)
            phase = Phase(message.phase)
            if phase not in down_completed:
                down_completed.add(phase)
                self._publish_evidence(phase, Direction.DOWN)

        self.client.send_goal(
            StairTraversalGoal(stair_id="down", direction=StairTraversalGoal.DOWN),
            feedback_cb=complete_down,
        )
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))

        # Then: DOWN reaches the same successful ordered boundary as UP.
        down_result: StairTraversalResult = self.client.get_result()
        self.assertEqual(
            self.client.get_state(),
            GoalStatus.SUCCEEDED,
            f"DOWN result: {down_result.reason}; code={down_result.result_code}; feedback={down_feedback}",
        )
        self.assertEqual(down_result.result_code, StairTraversalResult.OK)
        self.assertEqual(list(dict.fromkeys(down_feedback)), [phase.value for phase in Phase])

        # Given/When: a disabled profile is requested.
        before_disabled = len(self.transport.snapshot())
        self.client.send_goal(
            StairTraversalGoal(
                stair_id="disabled",
                direction=StairTraversalGoal.DOWN,
            )
        )
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))

        # Then: it aborts before mode or nonzero command output.
        disabled_result: StairTraversalResult = self.client.get_result()
        self.assertEqual(disabled_result.result_code, StairTraversalResult.CAPABILITY_DISABLED)
        disabled_events = self.transport.snapshot()[before_disabled:]
        self.assertFalse(any(event[0] == "stair" for event in disabled_events))
        self.assertFalse(
            any(
                event[0] == "update" and event[1:] != (0.0, 0.0)
                for event in disabled_events
            )
        )

        # Given: cancellation requested while the first moving segment runs.
        cancel_feedback: List[str] = []
        cancel_completed = set()

        def cancel_during_motion(message: StairTraversalFeedback) -> None:
            cancel_feedback.append(message.phase)
            phase = Phase(message.phase)
            if phase not in cancel_completed:
                cancel_completed.add(phase)
                self._publish_evidence(phase, Direction.UP)
            if phase is Phase.FORWARD_SEGMENT_1:
                self.client.cancel_goal()

        # When: the next safe checkpoint is reached.
        self.client.send_goal(
            StairTraversalGoal(stair_id="up", direction=StairTraversalGoal.UP),
            feedback_cb=cancel_during_motion,
        )
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))

        # Then: cancellation occurs at LANDING after returning to WALK.
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)
        self.assertNotIn(Phase.TURN_TO_NEXT_FLIGHT.value, cancel_feedback)
        self.assertIn(("stair", False), self.transport.snapshot())

        # Given/When: configured entry evidence never arrives before timeout.
        self.client.send_goal(
            StairTraversalGoal(stair_id="timeout", direction=StairTraversalGoal.UP)
        )
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))

        # Then: timeout aborts, latches FAULT, closes once, and never retries.
        timeout_result: StairTraversalResult = self.client.get_result()
        self.assertEqual(self.client.get_state(), GoalStatus.ABORTED)
        self.assertEqual(timeout_result.result_code, StairTraversalResult.STAIR_FAILED)
        events = self.transport.snapshot()
        self.assertEqual(events.count(("start",)), 1)
        self.assertEqual(events.count(("close",)), 1)
        self.assertEqual(self.node.state, SupervisorState.FAULT)


if __name__ == "__main__":
    rostest.rosrun(
        "stair_supervisor",
        "stair_supervisor_node_boundary",
        StairSupervisorNodeBoundaryTest,
    )
