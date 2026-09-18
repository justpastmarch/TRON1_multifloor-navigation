#!/usr/bin/env python3
"""Verify complete actionlib connectivity without submitting robot goals."""

from __future__ import annotations

import actionlib
import rospy

from mission_manager.msg import MissionAction
from multifloor_manager.msg import FloorTransitionAction
from stair_supervisor.msg import StairTraversalAction


def main() -> int:
    """Connect a real client to every operational action server."""
    getattr(rospy, "init_node")("action_server_readiness_probe", anonymous=True)
    clients = (
        ("/mission", actionlib.SimpleActionClient("/mission", MissionAction)),
        (
            "/multifloor/floor_transition",
            actionlib.SimpleActionClient(
                "/multifloor/floor_transition", FloorTransitionAction
            ),
        ),
        (
            "/stair_traversal",
            actionlib.SimpleActionClient("/stair_traversal", StairTraversalAction),
        ),
    )
    for name, client in clients:
        if not client.wait_for_server(rospy.Duration(15.0)):
            rospy.logerr("action server connection failed: %s", name)
            return 1
        print("[CHECK] actionlib connection: {}".format(name), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
