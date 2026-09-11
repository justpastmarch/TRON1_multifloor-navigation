"""Production configuration and disabled-profile startup boundary."""

from __future__ import annotations

from pathlib import Path

import actionlib
from geometry_msgs.msg import Twist
import rospy

from stair_supervisor.msg import (
    StairTraversalAction,
    StairTraversalGoal,
    StairTraversalResult,
    SupervisorState,
)

from .configuration import StairConfigurationError, load_stair_configuration
from .robot_config import CommandStreamConfig, RobotConnectionConfig, RobotTransportConfig
from .robot_transport import RobotTransport
from .ros_node import RosNodeSettings, RosStairSupervisorNode


class DisabledRosStairSupervisorNode:
    """Expose fail-closed ROS interfaces while deployment profiles are disabled."""

    def __init__(self, reason: str, action_name: str) -> None:
        self._reason = reason
        self._state_publisher = rospy.Publisher(
            "~state",
            SupervisorState,
            queue_size=1,
            latch=True,
        )
        self._nav_subscriber = rospy.Subscriber(
            "/navigation/cmd_vel",
            Twist,
            lambda _message: None,
            queue_size=1,
        )
        self._action = actionlib.SimpleActionServer(
            action_name,
            StairTraversalAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._state_publisher.publish(
            SupervisorState(
                state=SupervisorState.DISARMED,
                connected=False,
                detail=reason,
                ownership_epoch=0,
            )
        )
        self._action.start()

    def _execute(self, _goal: StairTraversalGoal) -> None:
        self._action.set_aborted(
            StairTraversalResult(
                result_code=StairTraversalResult.CAPABILITY_DISABLED,
                reason=self._reason,
            )
        )

    def shutdown(self) -> None:
        return


def _settings() -> RosNodeSettings:
    return RosNodeSettings(
        action_name=rospy.get_param("~action_name", "/stair_traversal"),
        nav_freshness_sec=float(rospy.get_param("~nav_freshness_sec", 0.25)),
        stream_rate_hz=float(rospy.get_param("~stream_rate_hz", 40.0)),
        turn_linear_mps=float(rospy.get_param("~turn_linear_mps", 0.0)),
        odom_topic=rospy.get_param("~odom_topic", "/tron/wheel_odom_raw"),
    )


def main() -> int:
    """Construct the production transport exactly once and serve until shutdown."""
    settings = _settings()
    config_root = Path(
        rospy.get_param(
            "~config_dir",
            str(Path(__file__).resolve().parents[2] / "config"),
        )
    )
    try:
        configuration = load_stair_configuration(config_root)
    except StairConfigurationError as error:
        node = DisabledRosStairSupervisorNode(str(error), settings.action_name)
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
        return 0
    transport = RobotTransport(
        RobotTransportConfig(
            connection=RobotConnectionConfig(
                accid=rospy.get_param("~accid"),
                url=rospy.get_param("~websocket_url"),
                connect_timeout=float(rospy.get_param("~connect_timeout_sec", 3.0)),
                receive_timeout=float(rospy.get_param("~receive_timeout_sec", 0.1)),
                request_timeout=float(rospy.get_param("~request_timeout_sec", 8.0)),
            ),
            stream=CommandStreamConfig(
                rate_hz=settings.stream_rate_hz,
                watchdog_sec=settings.nav_freshness_sec,
                startup_zero_repeats=int(rospy.get_param("~startup_zero_repeats", 3)),
                close_zero_repeats=int(rospy.get_param("~shutdown_zero_repeats", 8)),
            ),
        ),
        configuration.robot.websocket_full_scale,
    )
    node = RosStairSupervisorNode(configuration, transport, settings)
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
    return 0
