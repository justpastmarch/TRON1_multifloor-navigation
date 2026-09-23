#!/usr/bin/env python3
"""Actual observe-node boundary and compute child on an isolated test master."""
import random
import unittest

import rospy
import rostest
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ObservationTest(unittest.TestCase):
    def test_sensor_tracking_without_any_robot_command_surface(self):
        rospy.init_node("lidar_observation_boundary", anonymous=True)
        observations = []
        subscriber = rospy.Subscriber("/stair_supervisor/lidar_odom", Odometry, observations.append, queue_size=50)
        lidar = rospy.Publisher("/test/lidar", CustomMsg, queue_size=10)
        imu = rospy.Publisher("/test/imu", Imu, queue_size=100)
        # A recorder is also a subscriber. Its connection alone does not mean
        # the tested adapter has started; wait for that adapter's heartbeat.
        rospy.wait_for_message("/stair_supervisor/tracking_status", String, timeout=15.)
        deadline = rospy.Time.now() + rospy.Duration(15.)
        while (not lidar.get_num_connections() or not imu.get_num_connections()) and rospy.Time.now() < deadline:
            rospy.sleep(.02)
        self.assertGreater(lidar.get_num_connections(), 0)
        random.seed(17)
        points = [CustomPoint(x=random.uniform(1,4), y=random.uniform(-2,2), z=random.uniform(-1,2)) for _ in range(500)]
        stamps = []
        for index in range(150):
            now = rospy.Time.now()
            message = Imu();message.header.stamp = now;message.linear_acceleration.z = 9.81
            imu.publish(message)
            if index % 5 == 0:
                cloud = CustomMsg();cloud.header.stamp = now;cloud.header.frame_id = "livox"
                cloud.points = points;cloud.point_num = len(points)
                lidar.publish(cloud);stamps.append(now.to_nsec())
            rospy.sleep(.02)
        deadline = rospy.Time.now() + rospy.Duration(3.)
        while len(observations) < 3 and rospy.Time.now() < deadline:
            rospy.sleep(.02)
        self.assertGreaterEqual(len(observations), 3)
        self.assertTrue(all(o.header.stamp.to_nsec() in stamps for o in observations))
        self.assertTrue(all(o.header.frame_id.startswith("stair_local_") for o in observations))
        topics = dict(rospy.get_published_topics())
        self.assertNotIn("/stair_supervisor/websocket_tx", topics)
        self.assertNotIn("/navigation/cmd_vel", topics)
        self.assertNotIn("/stair_supervisor/state", topics)
        subscriber.unregister();lidar.unregister();imu.unregister()


if __name__ == "__main__":
    rostest.rosrun("stair_supervisor", "lidar_observation_only", ObservationTest)
