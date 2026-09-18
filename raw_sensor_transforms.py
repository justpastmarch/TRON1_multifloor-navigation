"""Build the replay TF tree exclusively from wheel odometry and LiDAR extrinsics."""

from __future__ import annotations

from pathlib import Path

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rosbag

from raw_sensor_display import BagMessageError


ODOM_FRAME = "odom"
BASE_FRAME = "base_Link"
LIDAR_FRAME = "mid360_link"


def wheel_odometry_transform(odometry: Odometry) -> TransformStamped:
    """Convert one wheel pose into the replay-only dynamic display transform."""
    transform = TransformStamped()
    transform.header.stamp = odometry.header.stamp
    transform.header.frame_id = ODOM_FRAME
    transform.child_frame_id = BASE_FRAME
    position = odometry.pose.pose.position
    orientation = odometry.pose.pose.orientation
    transform.transform.translation.x = position.x
    transform.transform.translation.y = position.y
    transform.transform.translation.z = position.z
    transform.transform.rotation = orientation
    return transform


def load_lidar_extrinsic(bag_path: Path) -> TransformStamped:
    """Read only the recorded base-to-LiDAR static calibration from a bag."""
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _topic, message, _stamp in bag.read_messages(topics=["/tf_static"]):
            for transform in message.transforms:
                if (
                    transform.header.frame_id == BASE_FRAME
                    and transform.child_frame_id == LIDAR_FRAME
                ):
                    output = TransformStamped()
                    output.header.frame_id = BASE_FRAME
                    output.child_frame_id = LIDAR_FRAME
                    output.transform = transform.transform
                    return output
    raise BagMessageError(f"bag has no {BASE_FRAME} -> {LIDAR_FRAME} static calibration")
