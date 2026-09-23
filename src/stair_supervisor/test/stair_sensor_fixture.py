"""Synthetic no-motion ROS sensor publisher for stair action tests."""

from __future__ import annotations

from dataclasses import dataclass
import math

from nav_msgs.msg import Odometry
import rospy

from stair_supervisor.configuration import Direction
from stair_supervisor.stair_evidence import Phase


@dataclass(frozen=True)
class MotionSample:
    dx_m: float
    dyaw_rad: float


class SyntheticStairSensors:
    """Publish coherent odometry profiles without commanding hardware."""

    def __init__(self, odom_topic: str) -> None:
        self.odom = rospy.Publisher(odom_topic, Odometry, queue_size=10)
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0

    def wait_for_connections(self) -> None:
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while (
            self.odom.get_num_connections() < 1
            and rospy.Time.now() < deadline
        ):
            rospy.sleep(0.01)
        if self.odom.get_num_connections() < 1:
            raise AssertionError("odometry publisher did not connect")

    def publish_phase(self, phase: Phase, direction: Direction) -> None:
        sign = 1.0 if direction is Direction.UP else -1.0
        stationary = MotionSample(0.0, 0.0)
        sequences = {
            Phase.VERIFY_ENTRY: (stationary,),
            # ALIGN accepts the NAV-aligned entry sample. Inventing a second
            # turn here can arrive after the next phase captures its baseline.
            Phase.ALIGN: (stationary,),
            Phase.FORWARD_SEGMENT_1: (
                MotionSample(sign * 0.35, 0.0),
                MotionSample(sign * 0.35, 0.0),
                MotionSample(sign * 0.30, 0.0),
            ),
            Phase.LANDING: (stationary,) * 6,
            Phase.TURN_TO_NEXT_FLIGHT: (
                MotionSample(0.0, sign * 0.25),
            ) * 2,
            Phase.FORWARD_SEGMENT_2: (
                MotionSample(sign * 0.30, 0.0),
                MotionSample(sign * 0.30, 0.0),
                MotionSample(sign * 0.20, 0.0),
            ),
            Phase.EXIT_CONFIRM: (stationary,) * 6,
        }
        for sample in sequences[phase]:
            self._publish(sample)

    def _publish(self, sample: MotionSample) -> None:
        self.x_m += sample.dx_m * math.cos(self.yaw_rad)
        self.y_m += sample.dx_m * math.sin(self.yaw_rad)
        self.yaw_rad += sample.dyaw_rad
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.pose.pose.position.x = self.x_m
        odom.pose.pose.position.y = self.y_m
        odom.pose.pose.orientation.z = math.sin(self.yaw_rad / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw_rad / 2.0)
        self.odom.publish(odom)
        rospy.sleep(0.05)
