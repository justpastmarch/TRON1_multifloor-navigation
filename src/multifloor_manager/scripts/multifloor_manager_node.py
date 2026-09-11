#!/usr/bin/env python3
"""Import the multifloor manager package entrypoint."""

import rospy

from multifloor_manager import main


if __name__ == "__main__":
    rospy.init_node("multifloor_manager", anonymous=False)
    raise SystemExit(main())
