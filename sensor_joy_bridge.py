#!/usr/bin/env python3
"""Publish TRON1 SensorJoy samples to ROS without issuing robot commands."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from typing import Final, Protocol, Sequence


DEFAULT_ROBOT_IP: Final = "10.192.1.2"
DEFAULT_TOPIC: Final = "/tron/sensor_joy"
NANOSECONDS_PER_SECOND: Final = 1_000_000_000


class SensorJoyData(Protocol):
    stamp: int
    axes: Sequence[float]
    buttons: Sequence[int]


@dataclass(frozen=True)  # noqa: SLOTS_OK - Python 3.8 runtime; explicit slots below.
class SensorJoySample:
    __slots__ = ("stamp_ns", "axes", "buttons")
    stamp_ns: int
    axes: tuple[float, ...]
    buttons: tuple[int, ...]


def sample_sensor_joy(sensor_joy: SensorJoyData) -> SensorJoySample:
    """Copy one SDK callback without axis mapping, scaling, or clamping."""
    return SensorJoySample(
        stamp_ns=int(sensor_joy.stamp),
        axes=tuple(float(axis) for axis in sensor_joy.axes),
        buttons=tuple(int(button) for button in sensor_joy.buttons),
    )


def ros_stamp_parts(stamp_ns: int) -> tuple[int, int]:
    """Split the SDK's documented nanosecond timestamp for rospy.Time."""
    return divmod(stamp_ns, NANOSECONDS_PER_SECOND)


def run(robot_ip: str, topic: str) -> int:
    """Initialize the receiver and register only the SensorJoy subscription."""
    import rospy
    from sensor_msgs.msg import Joy
    import limxsdk.robot.Robot as Robot
    import limxsdk.robot.RobotType as RobotType

    getattr(rospy, "init_node")("tron1_sensor_joy_bridge", anonymous=False)
    publisher = rospy.Publisher(topic, Joy, queue_size=20)

    def publish(sensor_joy: SensorJoyData) -> None:
        sample = sample_sensor_joy(sensor_joy)
        seconds, nanoseconds = ros_stamp_parts(sample.stamp_ns)
        message = Joy()
        message.header.stamp = rospy.Time(seconds, nanoseconds)
        message.axes = sample.axes
        message.buttons = sample.buttons
        publisher.publish(message)

    robot = Robot(RobotType.PointFoot)
    if not robot.init(robot_ip):
        rospy.logerr("SensorJoy receiver initialization failed")
        return 1
    robot.subscribeSensorJoy(publish)
    rospy.loginfo("SensorJoy receiver subscribed topic=%s", topic)
    rospy.spin()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Passive TRON1 SensorJoy ROS bridge")
    parser.add_argument("--robot-ip", default=os.environ.get("ROBOT_HOST", DEFAULT_ROBOT_IP))
    parser.add_argument("--topic", default=os.environ.get("SENSOR_JOY_TOPIC", DEFAULT_TOPIC))
    arguments = parser.parse_args(argv)
    return run(arguments.robot_ip, arguments.topic)


if __name__ == "__main__":
    raise SystemExit(main())
