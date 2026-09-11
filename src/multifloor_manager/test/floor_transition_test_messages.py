"""ROS message fixtures shared by FloorTransition process tests."""

from __future__ import annotations

from dataclasses import dataclass
import math

from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


@dataclass(frozen=True)
class LocalizationMessages:
    __slots__ = ("transforms", "pose", "odometry", "scan", "stale_pose")
    transforms: TFMessage
    pose: PoseWithCovarianceStamped
    odometry: Odometry
    scan: LaserScan
    stale_pose: PoseWithCovarianceStamped


def subscribe_debug(values):
    names = (
        "amcl_pose_samples",
        "amcl_pose_fresh",
        "scan_fresh",
        "scan_tf_valid",
        "odom_stationary",
        "odom_fresh",
        "localization_ready",
    )
    return tuple(
        rospy.Subscriber(
            "/multifloor/debug/{}".format(name),
            Bool,
            lambda message, predicate=name: values.__setitem__(predicate, message.data),
            queue_size=2,
        )
        for name in names
    )


def wait_for_state(states, expected: int, timeout: float) -> bool:
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while rospy.Time.now() < deadline:
        if states and states[-1].state == expected:
            return True
        rospy.sleep(0.02)
    return False


def publish_tags(publisher, tag_id: int) -> None:
    rospy.sleep(0.1)
    for _ in range(3):
        message = AprilTagDetectionArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "camera_link"
        message.detections = (AprilTagDetection(id=(tag_id,)),)
        publisher.publish(message)
        rospy.sleep(0.05)


def map_message(floor_id: str, stamp: rospy.Time | None = None) -> OccupancyGrid:
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.header.stamp = stamp or rospy.Time.now()
    message.info.map_load_time = stamp or rospy.Time.now()
    message.info.origin.orientation.w = 1.0
    if floor_id == "3F":
        message.info.width = 2
        message.info.height = 2
        message.info.resolution = 0.05
        message.data = (0, 100, 100, 0)
    elif floor_id == "4F":
        message.info.width = 3
        message.info.height = 2
        message.info.resolution = 0.10
        message.info.origin.position.x = -2.0
        message.info.origin.position.y = 1.0
        message.info.origin.orientation.z = math.sin(0.125)
        message.info.origin.orientation.w = math.cos(0.125)
        message.data = (0, -1, 100, 100, -1, 0)
    else:
        message.info.width = 2
        message.info.height = 3
        message.info.resolution = 0.20
        message.info.origin.position.x = 3.0
        message.info.origin.position.y = -1.0
        message.info.origin.orientation.z = math.sin(-0.25)
        message.info.origin.orientation.w = math.cos(-0.25)
        message.data = (0, 100, -1, 0, 100, -1)
    return message


def localization_messages(
    stamp: rospy.Time,
    initialpose_stamp: rospy.Time,
) -> LocalizationMessages:
    map_to_odom = TransformStamped()
    map_to_odom.header.stamp = stamp
    map_to_odom.header.frame_id = "map"
    map_to_odom.child_frame_id = "odom"
    map_to_odom.transform.rotation.w = 1.0
    odom_to_base = TransformStamped()
    odom_to_base.header.stamp = stamp
    odom_to_base.header.frame_id = "odom"
    odom_to_base.child_frame_id = "base_Link"
    odom_to_base.transform.rotation.w = 1.0
    pose = PoseWithCovarianceStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = "map"
    pose.pose.covariance[0] = 0.04
    pose.pose.covariance[7] = 0.04
    pose.pose.covariance[35] = 0.09
    odometry = Odometry()
    odometry.header.stamp = stamp
    odometry.header.frame_id = "odom"
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = "base_Link"
    stale_pose = PoseWithCovarianceStamped()
    stale_pose.header.stamp = initialpose_stamp
    stale_pose.header.frame_id = "map"
    return LocalizationMessages(
        TFMessage((map_to_odom, odom_to_base)),
        pose,
        odometry,
        scan,
        stale_pose,
    )
