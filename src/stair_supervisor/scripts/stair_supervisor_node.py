#!/usr/bin/env python3
"""Import the stair supervisor package entrypoint."""

import rospy

from stair_supervisor import main


if __name__ == "__main__":
    rospy.init_node("stair_supervisor", anonymous=False)
    raise SystemExit(main())
