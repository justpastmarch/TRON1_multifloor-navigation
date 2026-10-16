#!/usr/bin/env bash
# Record the high-bandwidth LiDAR and its IMU beside the sensor publisher.
set -eo pipefail
[[ $# == 1 && "$1" =~ ^[A-Za-z0-9_]+$ ]] || exit 2
base="${HOME}/.local/share/tron1-stair-supervisor"
set +u
source /opt/ros/noetic/setup.bash
source "${HOME}/catkin_ws/devel/setup.bash"
source "${base}/ws/devel/setup.bash"
set -u
source "${base}/runtime.env"
export ROS_MASTER_URI ROS_IP
unset ROS_HOSTNAME
mkdir -p "${base}/bags"
exec rosbag record -O "${base}/bags/$1.bag" /livox/lidar /livox/imu "__name:=$1"
