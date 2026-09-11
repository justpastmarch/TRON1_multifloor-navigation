"""Multifloor manager operational entrypoint."""

from __future__ import annotations

def main() -> int:
    """Run the node after scripts/multifloor_manager_node.py initializes rospy once."""
    from multifloor_manager.ros_node import run

    return run()
