"""Operational stair supervisor ROS package."""

from __future__ import annotations

def main() -> int:
    """Load the ROS boundary lazily so library imports remain ROS-independent."""
    from .ros_entrypoint import main as run_node

    return run_node()
