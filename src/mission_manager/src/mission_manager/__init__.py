"""Operational mission manager package entrypoint."""

from __future__ import annotations

import rospy


def main() -> int:
    """Host the Mission action until ROS shutdown."""
    if rospy.get_param('~mode', 'navigation') == 'mapping':
        from mission_manager.mapping_console import main as mapping_main
        return mapping_main()
    from mission_manager.ros_runtime import create_mission_action_server

    server = create_mission_action_server()
    rospy.loginfo("mission manager action server ready")
    rospy.spin()
    del server
    return 0
