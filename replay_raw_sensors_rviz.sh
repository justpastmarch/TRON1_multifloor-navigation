#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${1:-}"
RATE="${2:-1}"
START_SEC="${3:-0}"
RVIZ_CONFIG="${ROOT}/src/multifloor_manager/rviz/wf_raw_sensor_compare.rviz"

if [[ -z "$BAG" ]]; then
    printf 'Usage: ./replay_raw_sensors_rviz.sh BAG [RATE [START_SEC]]\n' >&2
    exit 2
fi
if [[ ! -r "$BAG" ]]; then
    printf 'BAG is not readable: %s\n' "$BAG" >&2
    exit 2
fi
if [[ ! "$RATE" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || ! awk -v r="$RATE" 'BEGIN { exit !(r > 0 && r <= 4) }'; then
    printf 'RATE must be greater than 0 and no more than 4.\n' >&2
    exit 2
fi
if [[ ! "$START_SEC" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    printf 'START_SEC must be zero or a positive number.\n' >&2
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

export ROS_MASTER_URI="${ROS_MASTER_URI_REPLAY:-http://127.0.0.1:11319}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
master_port="${ROS_MASTER_URI##*:}"
master_port="${master_port%/}"
if [[ ! "$master_port" =~ ^[0-9]+$ ]]; then
    printf 'ROS_MASTER_URI_REPLAY must end with a numeric port.\n' >&2
    exit 2
fi

roscore_pid=''
visualizer_pid=''
rviz_pid=''
master_ready() {
    timeout 1s rosparam get /run_id >/dev/null 2>&1
}
cleanup() {
    [[ -n "$rviz_pid" ]] && kill "$rviz_pid" 2>/dev/null || true
    [[ -n "$visualizer_pid" ]] && kill "$visualizer_pid" 2>/dev/null || true
    [[ -n "$roscore_pid" ]] && kill "$roscore_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if master_ready && [[ "${ALLOW_REPLAY_MASTER_REUSE:-0}" != "1" ]]; then
    printf 'Replay master already exists at %s; refusing to mix recorded sensors with it.\n' "$ROS_MASTER_URI" >&2
    printf 'Use another ROS_MASTER_URI_REPLAY or set ALLOW_REPLAY_MASTER_REUSE=1 explicitly.\n' >&2
    exit 2
fi
if ! master_ready; then
    roscore -p "$master_port" >/tmp/tron1-raw-replay-roscore.log 2>&1 &
    roscore_pid=$!
    for _ in {1..50}; do
        master_ready && break
        sleep 0.1
    done
fi
if ! master_ready; then
    printf 'ROS master did not become ready. See /tmp/tron1-raw-replay-roscore.log\n' >&2
    exit 1
fi

rosparam set use_sim_time true
python3 "${ROOT}/bag_sensor_visualizer.py" "$BAG" >/tmp/tron1-raw-replay-visualizer.log 2>&1 &
visualizer_pid=$!
export DISABLE_ROS1_EOL_WARNINGS=1
rviz -d "$RVIZ_CONFIG" >/tmp/tron1-raw-replay-rviz.log 2>&1 &
rviz_pid=$!
sleep 2
if ! kill -0 "$visualizer_pid" 2>/dev/null; then
    printf 'Raw sensor converter failed. See /tmp/tron1-raw-replay-visualizer.log\n' >&2
    exit 1
fi

printf '[RAW SENSOR RVIZ] fixed_frame=odom rate=%sx\n' "$RATE"
printf '[RAW SENSOR RVIZ] gray=LiDAR transformed only by recorded wheel-odom TF\n'
printf '[RAW SENSOR RVIZ] green=wheel odometry blue=acceleration magenta=angular velocity\n'
printf '[RAW SENSOR RVIZ] no FAST-LIO, scan matching, or IMU integration\n'
printf '[RAW SENSOR RVIZ] bag=%s start=%ss\n' "$BAG" "$START_SEC"

rosbag play --clock --rate="$RATE" --start="$START_SEC" --delay=1 "$BAG" --topics \
    /livox/lidar \
    /livox/imu \
    /tron/wheel_odom_raw \
    /camera1/color/image_raw/compressed
