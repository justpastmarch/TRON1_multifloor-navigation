#!/usr/bin/env bash
# Mapping-session work beside the sensors on the mini PC. Only small results cross the link.
#   record NAME                 record raw LiDAR/IMU and the 2D inputs (/scan, odom, TF) into one mini PC BAG
#                               (unit tron1-rawbag-NAME, sealed with remote_stair_record.sh seal NAME)
#   fetch2d NAME DEST           copy only /scan, /tron/wheel_odom_raw, /tf, /tf_static of that BAG to DEST
#   build NAME TRAJ PARAMS OUT  build the 3D cloud beside the BAG; copy back cloud.npz + result.json
set -Eeuo pipefail
[[ $# -ge 2 && "$2" =~ ^[A-Za-z0-9_]+$ ]] || { echo 'Usage: remote_mapping_cloud.sh record NAME | fetch2d NAME DEST | build NAME TRAJECTORY PARAMS OUTPUT' >&2; exit 2; }
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${root}/config.env"
target="${MINI_PC_USER}@${MINI_PC_HOST}"
ssh_opts=(-o BatchMode=yes -o ConnectTimeout=10)
base="/home/${MINI_PC_USER}/.local/share/tron1-stair-supervisor"
raw="${base}/bags/$2.bag"
ros_env="set +u; source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash"
case "$1" in
    record)
        [[ $# == 2 ]] || exit 2
        ssh "${ssh_opts[@]}" "$target" "systemd-run --user --collect --unit=tron1-rawbag-$2 --property=KillSignal=SIGINT --property=TimeoutStopSec=60s \
            bash -c '$ros_env; source ${base}/runtime.env; export ROS_MASTER_URI ROS_IP; unset ROS_HOSTNAME; mkdir -p ${base}/bags; \
            exec rosbag record -O ${raw} /livox/lidar /livox/imu /scan /tron/wheel_odom_raw /tf /tf_static __name:=$2' >/dev/null"
        if ! ssh "${ssh_opts[@]}" "$target" "for i in \$(seq 1 80); do test -f '${raw}.active' && exit 0; sleep .1; done; exit 1"; then
            ssh "${ssh_opts[@]}" "$target" "systemctl --user stop 'tron1-rawbag-$2.service'" || true
            echo 'MiniPC mapping recording did not become ready.' >&2; exit 1
        fi
        ;;
    fetch2d)
        [[ $# == 3 ]] || exit 2
        remote="$(ssh "${ssh_opts[@]}" "$target" "test -f '$raw' && test ! -e '$raw.active' && mktemp -d /tmp/tron1-map2d.XXXXXX")" || {
            echo "MiniPC BAG is missing or still recording: $raw" >&2; exit 1; }
        trap 'ssh "${ssh_opts[@]}" "$target" "rm -rf '"'"'$remote'"'"'" >/dev/null 2>&1 || true' EXIT
        ssh "${ssh_opts[@]}" "$target" "$ros_env; python3 - '$raw' '$remote/input.bag'" <<'PY'
import sys, rosbag
topics = ['/scan', '/tron/wheel_odom_raw', '/tf', '/tf_static']
count = 0
with rosbag.Bag(sys.argv[1]) as source, rosbag.Bag(sys.argv[2], 'w', compression='lz4') as out:
    for topic, msg, t, header in source.read_messages(topics=topics, return_connection_header=True):
        out.write(topic, msg, t, connection_header=header)
        count += topic == '/scan'
if count < 2:
    sys.exit('mini PC BAG has no /scan')
PY
        mkdir -p "$(dirname "$3")"
        scp -q "${ssh_opts[@]}" "${target}:${remote}/input.bag" "$3"
        ;;
    build)
        [[ $# == 5 ]] || exit 2
        remote="$(ssh "${ssh_opts[@]}" "$target" "test -f '$raw' && test ! -e '$raw.active' && mktemp -d /tmp/tron1-cloud.XXXXXX")" || {
            echo "MiniPC raw BAG is missing or still recording: $raw" >&2; exit 1; }
        trap 'ssh "${ssh_opts[@]}" "$target" "rm -rf '"'"'$remote'"'"'" >/dev/null 2>&1 || true' EXIT
        builder="${root}/src/mission_manager/src/mission_manager/mapping_cloud.py"
        scp -q "${ssh_opts[@]}" "$builder" "$3" "$4" "${target}:${remote}/"
        ssh "${ssh_opts[@]}" "$target" "$ros_env; \
            nice -n 10 python3 -u '$remote/mapping_cloud.py' '$raw' '$remote/$(basename "$3")' '$remote/$(basename "$4")' '$remote/out'"
        mkdir -p "$5"
        scp -q "${ssh_opts[@]}" "${target}:${remote}/out/cloud.npz" "${target}:${remote}/out/result.json" "$5/"
        ;;
    *) echo 'Expected record, fetch2d or build.' >&2; exit 2 ;;
esac
