#!/usr/bin/env bash
# Runs the existing Stair Supervisor on the MiniPC; no new ROS controller.
set -eo pipefail
base="${HOME}/.local/share/tron1-stair-supervisor"
set +u
source /opt/ros/noetic/setup.bash
source "${HOME}/catkin_ws/devel/setup.bash"
source "${base}/ws/devel/setup.bash"
set -u
# runtime.env is installed with mode 0600 by deploy_remote_stair.sh.
source "${base}/runtime.env"
export ROS_MASTER_URI ROS_IP STAIR_NUMERICAL_PYTHON STAIR_SENSOR_PYTHONPATH
unset ROS_HOSTNAME
exec roslaunch stair_supervisor remote_stair.launch \
    "accid:=${ACCID}" "websocket_url:=${ROBOT_WS_URL}" \
    "lidar_mode:=${STAIR_LIDAR_MODE}" \
    "lidar_observe_only:=${STAIR_LIDAR_OBSERVE_ONLY}"
