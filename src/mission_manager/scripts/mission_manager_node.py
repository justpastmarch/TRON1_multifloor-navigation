#!/usr/bin/env python3
"""Import the mission manager package entrypoint."""

import rospy

from mission_manager import main


if __name__ == "__main__":
    rospy.init_node("mission_manager", anonymous=False)
    raise SystemExit(main())
