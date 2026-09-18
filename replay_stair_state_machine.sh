#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${1:-${ROOT}/stair_captures/stair_3F_to_4F_UP_CLEAN_REPEAT_20260821_160211.bag}"
PROFILE="${2:-stair_3f_4f_up}"
RATE="${3:-1}"
START_SEC="${4:-0}"
OUTPUT_DIR="${5:-${ROOT}/logs/stair-state-replay-$(date +%Y%m%d-%H%M%S)}"
CONFIG_DIR="${6:-${ROOT}/src/stair_supervisor/config}"
IMAGE_TOPIC="/replay/stair_state_machine/image"

if [[ ! -r "$BAG" ]]; then
    printf 'BAG is not readable: %s\n' "$BAG" >&2
    exit 2
fi
if [[ ! "$RATE" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    printf 'RATE must be greater than zero.\n' >&2
    exit 2
fi
if ! awk -v rate="$RATE" 'BEGIN { exit !(rate > 0 && rate <= 4) }'; then
    printf 'RATE must be greater than zero and no more than four.\n' >&2
    exit 2
fi
if [[ ! "$START_SEC" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    printf 'START_SEC must be zero or positive.\n' >&2
    exit 2
fi
if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
    printf 'ROS Noetic is not installed.\n' >&2
    exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [[ -r "${ROOT}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT}/devel/setup.bash"
fi

export ROS_MASTER_URI="${ROS_MASTER_URI_STAIR_REPLAY:-http://127.0.0.1:11320}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
master_port="${ROS_MASTER_URI##*:}"
master_port="${master_port%/}"
if [[ ! "$master_port" =~ ^[0-9]+$ ]]; then
    printf 'ROS_MASTER_URI_STAIR_REPLAY must end with a numeric port.\n' >&2
    exit 2
fi

roscore_pid=''
supervisor_pid=''
viewer_pid=''
master_ready() {
    timeout 1s rosparam get /run_id >/dev/null 2>&1
}
cleanup() {
    [[ -n "$viewer_pid" ]] && kill "$viewer_pid" 2>/dev/null || true
    [[ -n "$supervisor_pid" ]] && kill "$supervisor_pid" 2>/dev/null || true
    [[ -n "$roscore_pid" ]] && kill "$roscore_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if master_ready; then
    printf 'Replay master already exists at %s; refusing to mix sessions.\n' "$ROS_MASTER_URI" >&2
    exit 2
fi
roscore -p "$master_port" >"/tmp/tron1-stair-replay-roscore.log" 2>&1 &
roscore_pid=$!
for _ in {1..50}; do
    master_ready && break
    sleep 0.1
done
if ! master_ready; then
    printf 'ROS master did not become ready. See /tmp/tron1-stair-replay-roscore.log\n' >&2
    exit 1
fi
rosparam set use_sim_time false

mkdir -p "$OUTPUT_DIR"
STAIR_REPLAY_ALLOW_ADMISSION=1 python3 "${ROOT}/src/stair_supervisor/test/synthetic_stair_supervisor_node.py" \
    "_config_dir:=${CONFIG_DIR}" \
    >"${OUTPUT_DIR}/supervisor.log" 2>&1 &
supervisor_pid=$!

if [[ "${SHOW_UI:-1}" == "1" ]]; then
    if command -v rqt_image_view >/dev/null 2>&1; then
        rqt_image_view "$IMAGE_TOPIC" >"${OUTPUT_DIR}/viewer.log" 2>&1 &
        viewer_pid=$!
    else
        printf 'rqt_image_view is unavailable; inspect %s/final-state.png instead.\n' "$OUTPUT_DIR"
    fi
fi

printf '[STAIR FSM REPLAY] isolated_master=%s\n' "$ROS_MASTER_URI"
printf '[STAIR FSM REPLAY] bag=%s profile=%s rate=%sx start=%ss\n' \
    "$BAG" "$PROFILE" "$RATE" "$START_SEC"
printf '[STAIR FSM REPLAY] image_topic=%s output=%s\n' "$IMAGE_TOPIC" "$OUTPUT_DIR"

hold=()
if [[ "${SHOW_UI:-1}" == "1" ]]; then
    hold=(--hold)
fi
set +e
python3 "${ROOT}/src/stair_supervisor/test/replay_stair_state_machine.py" \
    "$BAG" \
    --profile "$PROFILE" \
    --rate "$RATE" \
    --start "$START_SEC" \
    --config-dir "${CONFIG_DIR}" \
    --output-dir "$OUTPUT_DIR" \
    "${hold[@]}"
replay_status=$?
set -e
exit "$replay_status"
