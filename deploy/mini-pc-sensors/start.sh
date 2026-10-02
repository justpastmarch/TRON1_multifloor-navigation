#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source "$MINI_PC_WORKSPACE/devel/setup.bash"
unset ROS_HOSTNAME
exec /usr/bin/python3 -u "$HOME/.local/lib/tron1-sensors/watch_master.py"
