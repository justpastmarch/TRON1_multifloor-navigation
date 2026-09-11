#!/usr/bin/env python3
"""Prove cancellation after stair acquisition remains fail closed."""

from __future__ import annotations

import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from multifloor_manager.msg import FloorState, FloorTransitionAction, FloorTransitionGoal
from nav_msgs.msg import OccupancyGrid
import rospy
import rostest

from floor_transition_test_messages import wait_for_state


class FloorTransitionCancelRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("floor_transition_cancel_ros_test", anonymous=True)

    def test_cancelled_transition_is_preempted_and_faulted(self) -> None:
        # Given: a manager made READY by its configured initial map.
        states = []
        feedback = []
        state_subscriber = rospy.Subscriber(
            "/multifloor/floor_state", FloorState, states.append, queue_size=10
        )
        map_publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        client = actionlib.SimpleActionClient(
            "/multifloor/floor_transition", FloorTransitionAction
        )
        self.assertTrue(client.wait_for_server(rospy.Duration(5.0)))
        initial = OccupancyGrid()
        initial.header.frame_id = "map"
        initial.info.width = 2
        initial.info.height = 2
        initial.info.resolution = 0.05
        initial.info.origin.orientation.w = 1.0
        initial.data = (0, 100, 100, 0)
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while rospy.Time.now() < deadline and not any(
            state.state == FloorState.READY for state in states
        ):
            initial.header.stamp = rospy.Time.now()
            initial.info.map_load_time = initial.header.stamp
            map_publisher.publish(initial)
            rospy.sleep(0.05)
        self.assertTrue(any(state.state == FloorState.READY for state in states))

        # When: the accepted action is cancelled while waiting for its post-stair tag vote.
        client.send_goal(
            FloorTransitionGoal(transition_id="stair_a", target_floor="4F"),
            feedback_cb=lambda message: feedback.append(message.phase),
        )
        phase_deadline = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < phase_deadline and "ARM_TARGET" not in feedback:
            rospy.sleep(0.02)
        client.cancel_goal()
        self.assertTrue(client.wait_for_result(rospy.Duration(3.0)))

        # Then: no rollback or READY resume occurs after the post-stair cancellation.
        self.assertEqual(client.get_state(), GoalStatus.PREEMPTED)
        self.assertTrue(wait_for_state(states, FloorState.FAULT, 1.0))
        self.assertEqual(states[-1].state, FloorState.FAULT)
        self.assertEqual(states[-1].floor_id, "")
        state_subscriber.unregister()


if __name__ == "__main__":
    rostest.rosrun(
        "multifloor_manager",
        "floor_transition_cancel_ros",
        FloorTransitionCancelRosTest,
    )
