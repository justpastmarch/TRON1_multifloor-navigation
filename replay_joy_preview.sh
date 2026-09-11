#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bag="${1:-}"
start_sec="${2:-0}"
duration_sec="${3:-5}"
rate="${4:-1}"
number_pattern='^([0-9]+([.][0-9]*)?|[.][0-9]+)$'

if (( $# > 4 )) || [[ ! -r "$bag" || "$bag" != *.bag ]]; then
    printf 'Replay requires one readable finalized .bag file and at most START_SEC, DURATION_SEC, RATE.\n' >&2
    exit 2
fi
if [[ ! "$start_sec" =~ $number_pattern ]] \
    || [[ ! "$duration_sec" =~ $number_pattern ]] \
    || [[ ! "$rate" =~ $number_pattern ]] \
    || ! awk -v value="$duration_sec" 'BEGIN { exit !(value > 0 && value <= 30) }' \
    || ! awk -v value="$rate" 'BEGIN { exit !(value > 0 && value <= 1) }'; then
    printf 'Replay bounds: START_SEC >= 0, 0 < DURATION_SEC <= 30, 0 < RATE <= 1.\n' >&2
    exit 2
fi

rosbag_bin="$(command -v rosbag || true)"
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
if [[ -z "$rosbag_bin" ]]; then
    rosbag_bin="$(command -v rosbag || true)"
fi
if [[ -z "$rosbag_bin" ]]; then
    printf 'Required command missing: rosbag\n' >&2
    exit 1
fi

joystick_topic="/tron/sensor_joy"
printf '[REPLAY PREVIEW] source=%s output=/replay%s start=%ss duration=%ss rate=%sx\n' \
    "$joystick_topic" "$joystick_topic" "$start_sec" "$duration_sec" "$rate"
exec "$rosbag_bin" play "$bag" --quiet --prefix=/replay --delay=0.5 \
    --start="$start_sec" --duration="$duration_sec" --rate="$rate" \
    --topics "$joystick_topic"
