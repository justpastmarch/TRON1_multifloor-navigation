#!/usr/bin/env python3
"""Drive the operational FloorTransition action through mocked ROS peers."""

from __future__ import annotations

import threading
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from multifloor_manager.msg import FloorState, FloorTransitionAction, FloorTransitionGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import LoadMap, LoadMapResponse
import rospy
import rostest
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyResponse
from tf2_msgs.msg import TFMessage

from floor_transition_test_messages import (
    localization_messages,
    map_message,
    publish_tags,
    subscribe_debug,
    wait_for_state,
)


class FloorTransitionRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("floor_transition_ros_test", anonymous=True)

    def setUp(self) -> None:
        self.map_publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=2, latch=True)
        self.costmap_publisher = rospy.Publisher(
            "/move_base/global_costmap/costmap", OccupancyGrid, queue_size=2, latch=True
        )
        self.tag_publisher = rospy.Publisher("/tag_detections", AprilTagDetectionArray, queue_size=10)
        self.pose_publisher = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped, queue_size=10)
        self.scan_publisher = rospy.Publisher("/scan", LaserScan, queue_size=10)
        self.odom_publisher = rospy.Publisher("/tron/wheel_odom_raw", Odometry, queue_size=10)
        self.tf_publisher = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.states = []
        self.initialposes = []
        self.feedback = []
        self.localization_threads = []
        self.sensor_stop_events = tuple(threading.Event() for _ in range(3))
        self.nomotion_calls = 0
        self.pose_updates_published = 0
        self.causal_violation = False
        self.unsolicited_pose_stamps = []
        self.causal_pose_stamps = []
        self.change_result = LoadMapResponse.RESULT_SUCCESS
        self.map_during_change = self.clear_called = False
        self.wrong_map_published = self.wrong_tf_scan_published = False
        self.wrong_costmap_published = False
        self.premature_clear = False
        self.succeeded_before_target_costmap = False
        self.debug_values = {}
        self.state_subscriber = rospy.Subscriber(
            "/multifloor/floor_state", FloorState, self.states.append, queue_size=20
        )
        self.initialpose_subscriber = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initialposes.append, queue_size=2
        )
        self.debug_subscribers = subscribe_debug(self.debug_values)
        self.change_service = rospy.Service("/change_map", LoadMap, self._change_map)
        self.nomotion_service = rospy.Service(
            "/request_nomotion_update", Empty, self._nomotion_update
        )
        self.clear_service = rospy.Service(
            "/move_base/clear_costmaps", Empty, self._clear_costmaps
        )
        self.client = actionlib.SimpleActionClient(
            "/multifloor/floor_transition", FloorTransitionAction
        )
        self.assertTrue(self.client.wait_for_server(rospy.Duration(5.0)))
        self._publish_until_ready()

    def tearDown(self) -> None:
        self.client.cancel_all_goals()
        for event in self.sensor_stop_events:
            event.set()
        for thread in self.localization_threads:
            thread.join(timeout=2.0)
        self.change_service.shutdown()
        self.nomotion_service.shutdown()
        self.clear_service.shutdown()
        self.state_subscriber.unregister()
        self.initialpose_subscriber.unregister()
        for subscriber in self.debug_subscribers:
            subscriber.unregister()

    def _publish_until_ready(self) -> None:
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while rospy.Time.now() < deadline:
            self.map_publisher.publish(map_message("3F"))
            if any(state.state == FloorState.READY and state.floor_id == "3F" for state in self.states):
                return
            rospy.sleep(0.05)
        self.fail("manager did not establish initial READY state: {!r}".format(self.states))

    def _change_map(self, request):
        if self.change_result == LoadMapResponse.RESULT_SUCCESS:
            rospy.sleep(2.1)
            stale = map_message("4F", rospy.Time(1))
            self.map_publisher.publish(stale)
            wrong = map_message("4F", rospy.Time.now())
            wrong.data = (100, -1, 100, 100, -1, 0)
            self.map_publisher.publish(wrong)
            self.wrong_map_published = True
            target = map_message("4F", rospy.Time.now())
            self.map_publisher.publish(target)
            rospy.sleep(0.1)
            self.map_during_change = True
            return LoadMapResponse(map=target, result=self.change_result)
        return LoadMapResponse(result=self.change_result)

    def _nomotion_update(self, _request):
        if self.nomotion_calls > self.pose_updates_published:
            self.causal_violation = True
        if self.nomotion_calls > 0:
            self.sensor_stop_events[self.nomotion_calls - 1].set()
        self.nomotion_calls += 1
        unsolicited = PoseWithCovarianceStamped()
        unsolicited.header.stamp = rospy.Time.now()
        unsolicited.header.frame_id = "map"
        unsolicited.pose.covariance[0] = 0.04
        unsolicited.pose.covariance[7] = 0.04
        unsolicited.pose.covariance[35] = 0.09
        self.unsolicited_pose_stamps.append(unsolicited.header.stamp.to_nsec())
        self.pose_publisher.publish(unsolicited)
        rospy.sleep(0.05)
        thread = threading.Thread(
            target=self._publish_localization_update,
            args=(self.nomotion_calls,),
            daemon=True,
        )
        self.localization_threads.append(thread)
        thread.start()
        return EmptyResponse()

    def _clear_costmaps(self, _request):
        self.clear_called = True
        wrong = map_message("4F")
        wrong.info.resolution = 0.20
        self.costmap_publisher.publish(wrong)
        self.wrong_costmap_published = True
        def publish_target() -> None:
            self.succeeded_before_target_costmap = self.client.get_state() == GoalStatus.SUCCEEDED
            self.costmap_publisher.publish(map_message("4F"))

        threading.Timer(0.05, publish_target).start()
        return EmptyResponse()

    def _publish_localization_update(self, call_number: int) -> None:
        rospy.sleep(0.1)
        stamp = rospy.Time.now()
        messages = localization_messages(stamp, self.initialposes[-1].header.stamp)
        if call_number == 1:
            self.pose_publisher.publish(messages.stale_pose)
        self.scan_publisher.publish(messages.scan)
        self.wrong_tf_scan_published = True
        self.premature_clear = self.premature_clear or self.clear_called
        self.pose_updates_published = call_number
        self.causal_pose_stamps.append(stamp.to_nsec())
        self.pose_publisher.publish(messages.pose)
        stop = self.sensor_stop_events[call_number - 1]
        while not stop.is_set() and not rospy.is_shutdown():
            fresh = localization_messages(rospy.Time.now(), self.initialposes[-1].header.stamp)
            self.tf_publisher.publish(fresh.transforms)
            self.odom_publisher.publish(fresh.odometry)
            stop.wait(timeout=0.02)
            self.scan_publisher.publish(fresh.scan)

    def test_inflight_map_and_costmap_barrier_complete_ordered_transition(self) -> None:
        # Given: a READY 3F manager and expected post-stair tag vote.
        expected_policy_outcome = rospy.get_param("~expected_policy_outcome", "success")
        tag_thread = threading.Thread(
            target=publish_tags, args=(self.tag_publisher, 101), daemon=True
        )
        tag_thread.start()

        # When: the directed 3F-to-4F action runs through real ROS transport.
        self.client.send_goal(
            FloorTransitionGoal(transition_id="stair_a", target_floor="4F"),
            feedback_cb=lambda feedback: self.feedback.append(feedback.phase),
        )
        self.assertTrue(self.client.wait_for_result(rospy.Duration(12.0)))
        tag_thread.join(timeout=1.0)

        if expected_policy_outcome == "blocked":
            self.assertEqual(self.client.get_state(), GoalStatus.ABORTED)
            self.assertTrue(
                wait_for_state(self.states, FloorState.FAULT, 1.0),
                "action aborted before terminal FAULT state arrived: {!r}".format(self.states),
            )
            self.assertEqual(self.states[-1].state, FloorState.FAULT)
            self.assertFalse(
                any(state.state == FloorState.READY and state.floor_id == "4F" for state in self.states)
            )
            self.assertIn("POLICY_READY", self.feedback)
            return

        self.assertFalse(
            self.causal_violation,
            "a no-motion call started before the prior causal pose update",
        )
        self.assertTrue(
            wait_for_state(self.states, FloorState.READY, 1.0),
            "action state={} result={!r} floor states={!r} debug={!r}".format(
                self.client.get_state(), self.client.get_result(), self.states, self.debug_values
            ),
        )

        # Then: the in-service map was captured and READY followed the post-clear costmap.
        self.assertEqual(self.client.get_state(), GoalStatus.SUCCEEDED)
        self.assertEqual(self.client.get_result().floor_id, "4F")
        self.assertEqual(self.client.get_result().map_generation, 1)
        self.assertTrue(self.map_during_change)
        self.assertTrue(self.wrong_map_published)
        self.assertTrue(self.wrong_tf_scan_published)
        self.assertFalse(self.premature_clear)
        self.assertTrue(self.clear_called)
        self.assertTrue(self.wrong_costmap_published)
        self.assertFalse(self.succeeded_before_target_costmap)
        self.assertEqual(self.nomotion_calls, 3)
        self.assertEqual(self.pose_updates_published, 3)
        self.assertEqual(len(set(self.causal_pose_stamps)), 3)
        self.assertTrue(
            all(stamp > self.initialposes[0].header.stamp.to_nsec() for stamp in self.causal_pose_stamps)
        )
        self.assertTrue(
            all(
                causal > unsolicited
                for causal, unsolicited in zip(
                    self.causal_pose_stamps,
                    self.unsolicited_pose_stamps,
                )
            )
        )
        self.assertEqual(len(self.initialposes), 1)
        self.assertEqual(
            self.feedback,
            [
                "ARM_TARGET",
                "CHANGE_MAP",
                "MAP_CONFIRM",
                "INITIALPOSE",
                 "AMCL_READY",
                 "CLEAR_COSTMAPS",
                 "COSTMAP_READY",
                 "POLICY_READY",
             ],
         )
        transitioning = next(state for state in self.states if state.state == FloorState.TRANSITIONING)
        self.assertEqual(transitioning.floor_id, "")
        self.assertEqual(self.states[-1].state, FloorState.READY)

        # Given: the next directed stair but a failing change_map result.
        self.change_result = LoadMapResponse.RESULT_INVALID_MAP_DATA
        tag_thread = threading.Thread(
            target=publish_tags, args=(self.tag_publisher, 202), daemon=True
        )
        tag_thread.start()

        # When: that post-stair transaction reaches the failed service result.
        self.client.send_goal(FloorTransitionGoal(transition_id="stair_b", target_floor="RF"))
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))
        tag_thread.join(timeout=1.0)
        self.assertTrue(wait_for_state(self.states, FloorState.FAULT, 1.0))

        # Then: action and floor state fail closed without rollback or extra AMCL updates.
        self.assertEqual(self.client.get_state(), GoalStatus.ABORTED)
        self.assertEqual(self.states[-1].state, FloorState.FAULT)
        self.assertEqual(self.states[-1].floor_id, "")
        self.assertEqual(self.nomotion_calls, 3)
        fault_generation = self.states[-1].map_generation

        # Given: the expected RF map arrives after the transaction is already terminal.
        self.map_publisher.publish(map_message("RF", rospy.Time.now()))
        rospy.sleep(0.1)

        # When: an invalid follow-up goal exposes the manager's internal generation.
        self.client.send_goal(FloorTransitionGoal(transition_id="stair_b", target_floor="RF"))
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2.0)))

        # Then: late callbacks cannot mutate the latched FAULT or its generation.
        self.assertEqual(self.client.get_result().map_generation, fault_generation)
        self.assertEqual(self.states[-1].state, FloorState.FAULT)
        self.assertEqual(self.states[-1].map_generation, fault_generation)

if __name__ == "__main__":
    rostest.rosrun("multifloor_manager", "floor_transition_ros", FloorTransitionRosTest)
