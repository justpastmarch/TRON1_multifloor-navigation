#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${1:-${ROOT}/manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid}"
RATE="${2:-1}"
RVIZ_CONFIG="${ROOT}/src/multifloor_manager/rviz/wf_bag_replay.rviz"

if [[ ! -r "$BAG" ]]; then
    printf 'BAG is not readable: %s\n' "$BAG" >&2
    exit 2
fi
if [[ ! "$RATE" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || ! awk -v r="$RATE" 'BEGIN { exit !(r > 0 && r <= 4) }'; then
    printf 'RATE must be greater than 0 and no more than 4.\n' >&2
    exit 2
fi
if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
    printf 'ROS Noetic is not installed: /opt/ros/noetic/setup.bash\n' >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [[ -r "${ROOT}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT}/devel/setup.bash"
fi
# BAG replay is isolated from the robot/remote master configured in config.env.
export ROS_MASTER_URI="${ROS_MASTER_URI_REPLAY:-http://127.0.0.1:11311}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME

roscore_pid=''
rviz_pid=''
master_ready() {
    timeout 1s rosparam get /run_id >/dev/null 2>&1
}
cleanup() {
    [[ -n "$rviz_pid" ]] && kill "$rviz_pid" 2>/dev/null || true
    [[ -n "$roscore_pid" ]] && kill "$roscore_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if ! master_ready; then
    roscore >/tmp/tron1-bag-replay-roscore.log 2>&1 &
    roscore_pid=$!
    for _ in {1..50}; do
        master_ready && break
        sleep 0.1
    done
fi
if ! master_ready; then
    printf 'ROS master did not become ready. See /tmp/tron1-bag-replay-roscore.log\n' >&2
    exit 1
fi

rosparam set use_sim_time true
rviz -d "$RVIZ_CONFIG" >/tmp/tron1-bag-replay-rviz.log 2>&1 &
rviz_pid=$!
sleep 2
printf '[BAG RVIZ] fixed_frame=odom rate=%sx\n[BAG RVIZ] bag=%s\n' "$RATE" "$BAG"
exec rosbag play --clock --rate="$RATE" "$BAG"
