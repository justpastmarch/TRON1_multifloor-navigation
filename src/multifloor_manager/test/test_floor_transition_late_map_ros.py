#!/usr/bin/env python3
"""Reject map callbacks arriving after a transition has faulted."""

from __future__ import annotations

import threading
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from multifloor_manager.msg import FloorState, FloorTransitionAction, FloorTransitionGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import LoadMap, LoadMapResponse
import rospy
import rostest


def map_message(floor_id: str) -> OccupancyGrid:
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.info.map_load_time = message.header.stamp
    message.info.origin.orientation.w = 1.0
    if floor_id == "3F":
        message.info.width, message.info.height, message.info.resolution = 2, 2, 0.05
        message.data = (0, 100, 100, 0)
    else:
        message.info.width, message.info.height, message.info.resolution = 3, 2, 0.10
        message.info.origin.position.x = -2.0
        message.info.origin.position.y = 1.0
        message.info.origin.orientation.z = 0.12467473338522769
        message.info.origin.orientation.w = 0.992197667229329
        message.data = (0, -1, 100, 100, -1, 0)
    return message


class LateMapRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("floor_transition_late_map_test", anonymous=True)

    def test_hanging_service_and_late_map_cannot_mutate_fault(self) -> None:
        # Given: READY 3F and a change_map peer that responds after the service timeout.
        states = []
        map_publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=2, latch=True)
        tag_publisher = rospy.Publisher(
            "/tag_detections", AprilTagDetectionArray, queue_size=5
        )
        state_subscriber = rospy.Subscriber(
            "/multifloor/floor_state", FloorState, states.append, queue_size=10
        )
        def change_map(_request):
            rospy.sleep(0.8)
            target = map_message("4F")
            map_publisher.publish(target)
            return LoadMapResponse(map=target, result=LoadMapResponse.RESULT_SUCCESS)

        service = rospy.Service("/change_map", LoadMap, change_map)
        client = actionlib.SimpleActionClient(
            "/multifloor/floor_transition", FloorTransitionAction
        )
        self.assertTrue(client.wait_for_server(rospy.Duration(5.0)))
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < deadline and not any(
            state.state == FloorState.READY for state in states
        ):
            map_publisher.publish(map_message("3F"))
            rospy.sleep(0.05)
        self.assertTrue(any(state.state == FloorState.READY for state in states))

        def publish_tags() -> None:
            rospy.sleep(0.1)
            for _ in range(3):
                message = AprilTagDetectionArray()
                message.header.frame_id = "camera_link"
                message.header.stamp = rospy.Time.now()
                message.detections = (AprilTagDetection(id=(101,)),)
                tag_publisher.publish(message)
                rospy.sleep(0.05)

        # When: the action faults, then its expected target map arrives late.
        thread = threading.Thread(target=publish_tags, daemon=True)
        thread.start()
        client.send_goal(FloorTransitionGoal(transition_id="stair_a", target_floor="4F"))
        self.assertTrue(client.wait_for_result(rospy.Duration(3.0)))
        self.assertEqual(client.get_state(), GoalStatus.ABORTED)
        fault_generation = client.get_result().map_generation
        rospy.sleep(1.0)
        client.send_goal(FloorTransitionGoal(transition_id="stair_a", target_floor="4F"))
        self.assertTrue(client.wait_for_result(rospy.Duration(2.0)))

        # Then: the terminal generation and latched FAULT remain unchanged.
        self.assertEqual(client.get_result().map_generation, fault_generation)
        self.assertEqual(states[-1].state, FloorState.FAULT)
        self.assertEqual(states[-1].map_generation, fault_generation)
        thread.join(timeout=1.0)
        service.shutdown()
        state_subscriber.unregister()


if __name__ == "__main__":
    rostest.rosrun("multifloor_manager", "floor_transition_late_map_ros", LateMapRosTest)
