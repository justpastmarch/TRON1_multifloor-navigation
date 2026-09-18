#!/usr/bin/env python3
# --- How to run ---
# Launched only by mission_manager/full_system_synthetic.test.
"""Run the real stair ROS node with an in-memory robot transport."""

from __future__ import annotations

import ipaddress
import os
from pathlib import Path
from urllib.parse import urlparse

import rospy

from stair_supervisor.configuration import load_stair_configuration
from stair_supervisor.robot_config import (
    CommandStreamConfig,
    RobotConnectionConfig,
    RobotTransportConfig,
)
from stair_supervisor.robot_clock import SystemClock
from stair_supervisor.robot_transport import RobotTransport
from stair_supervisor.ros_node import RosNodeSettings, RosStairSupervisorNode
from stair_supervisor.stair_admission import AllowStairAdmissionValidator

from test_websocket_tx_ros import FakeClock, FakeFactory, FakeWebSocket


class SyntheticIsolationError(RuntimeError):
    """Report a synthetic transport that could escape the local ROS graph."""


def require_loopback_master() -> None:
    """Refuse to expose synthetic robot transport on a non-local ROS graph."""
    hosts = (
        urlparse(os.environ.get("ROS_MASTER_URI", "")).hostname,
        os.environ.get("ROS_IP"),
        os.environ.get("ROS_HOSTNAME"),
    )
    for host in (value for value in hosts if value):
        if host == "localhost":
            continue
        try:
            loopback = ipaddress.ip_address(host).is_loopback
        except ValueError:
            loopback = False
        if not loopback:
            raise SyntheticIsolationError(
                "synthetic stair transport requires loopback ROS addressing"
            )


def main() -> int:
    require_loopback_master()
    rospy.init_node("stair_supervisor")
    config_root = Path(rospy.get_param("~config_dir")).resolve()
    configuration = load_stair_configuration(config_root)
    socket = FakeWebSocket()
    clock = SystemClock()
    transport = RobotTransport(
        RobotTransportConfig(
            connection=RobotConnectionConfig(
                accid="SYNTHETIC_LOOPBACK_ONLY",
                url="ws://127.0.0.1.invalid:5000",
                receive_timeout=0.01,
                request_timeout=0.2,
            ),
            stream=CommandStreamConfig(
                rate_hz=40.0,
                watchdog_sec=0.25,
                startup_zero_repeats=2,
                close_zero_repeats=4,
            ),
        ),
        configuration.robot.websocket_full_scale,
        FakeFactory(socket),
        clock,
    )
    replay_admission = (
        AllowStairAdmissionValidator()
        if os.environ.get("STAIR_REPLAY_ALLOW_ADMISSION") == "1"
        else None
    )
    node = RosStairSupervisorNode(
        configuration,
        transport,
        RosNodeSettings(
            "/stair_traversal",
            0.25,
            40.0,
            0.0,
            "/tron/wheel_odom_raw",
        ),
        replay_admission,
    )
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
