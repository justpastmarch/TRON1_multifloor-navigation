"""Recorded odometry boundary for the live stair-supervisor replay harness."""

from __future__ import annotations

from copy import deepcopy
import math

from nav_msgs.msg import Odometry
import rospy

from .stair_evidence import OdometrySample


def restamp_odometry(source: Odometry, stamp: rospy.Time) -> Odometry:
    """Copy a recorded message and make only its freshness timestamp current."""
    replayed = deepcopy(source)
    replayed.header.stamp = stamp
    return replayed


def odometry_sample(message: Odometry, stamp_sec: float) -> OdometrySample:
    """Project recorded planar pose into the production evidence contract."""
    orientation = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    position = message.pose.pose.position
    return OdometrySample(
        stamp_sec,
        position.x,
        position.y,
        math.atan2(sin_yaw, cos_yaw),
    )
