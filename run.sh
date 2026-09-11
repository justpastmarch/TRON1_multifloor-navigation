#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${ROOT}/config.env"
FIXTURE_ROOT="${ROOT}/test/fixtures/building_valid"

usage() {
    printf '%s\n' \
        "Usage: ./run.sh [--check|--preflight|--record-manual|--replay-joy BAG [START_SEC [DURATION_SEC [RATE]]]|--help]" \
        "  no option    Start the validated three-node mission system" \
        "  --check      Validate the bundle, fixture, and local software" \
        "  --preflight  Also require production config and hardware connectivity" \
        "  --record-manual  Record sensors and physical SensorJoy input; starts no robot software" \
        "  --replay-joy  Preview up to 30s of recorded SensorJoy on /replay/tron/sensor_joy" \
        "  --help       Show this help"
}

case "${1:-}" in
    --help|-h) usage; exit 0 ;;
    --check|--preflight|--record-manual|--replay-joy|"") ;;
    *) usage >&2; exit 2 ;;
esac

if [[ ! -r "$CONFIG_FILE" ]]; then
    printf 'Missing configuration: %s\n' "$CONFIG_FILE" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$CONFIG_FILE"

if [[ "${1:-}" == "--replay-joy" ]]; then
    exec "${ROOT}/replay_joy_preview.sh" "${@:2}"
fi

if [[ "${1:-}" == "--record-manual" ]]; then
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
    if [[ "$SENSOR_JOY_RECEIVER_AUTOSTART" == "1" ]]; then
        receiver_status="$(ssh -o BatchMode=yes -o ConnectTimeout=5 \
            "${MINI_PC_USER}@${MINI_PC_HOST}" \
            "root='\${HOME}/.local/share/tron1-sensor-joy'; pattern='^/home/${MINI_PC_USER}/.local/share/tron1-sensor-joy/venv/bin/python /home/${MINI_PC_USER}/.local/share/tron1-sensor-joy/sensor_joy_bridge.py'; existing=\$(pgrep -f \"\$pattern\" || true); if [ -n \"\$existing\" ]; then printf 'reused pid=%s' \"\$existing\"; else test -x \"\$root/venv/bin/python\" && test -x \"\$root/sensor_joy_bridge.py\"; nohup setsid bash -lc 'source /opt/ros/noetic/setup.bash; source ${MINI_PC_WORKSPACE}/devel/setup.bash; export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}; export ROS_IP=${MINI_PC_ROS_IP}; export ROBOT_TYPE=${ROBOT_TYPE}; unset ROS_HOSTNAME; exec /home/${MINI_PC_USER}/.local/share/tron1-sensor-joy/venv/bin/python /home/${MINI_PC_USER}/.local/share/tron1-sensor-joy/sensor_joy_bridge.py --robot-ip ${ROBOT_HOST} --topic ${SENSOR_JOY_TOPIC}' >\"\$root/receiver.log\" 2>&1 < /dev/null & pid=\$!; printf '%s\\n' \"\$pid\" >\"\$root/receiver.pid\"; printf 'started pid=%s' \"\$pid\"; fi")"
        printf '[SSH] SensorJoy receiver: %s\n' "$receiver_status"
    fi
    if [[ "$MANUAL_CAPTURE_OUTPUT_BASE" != /* ]]; then
        MANUAL_CAPTURE_OUTPUT_BASE="${ROOT}/${MANUAL_CAPTURE_OUTPUT_BASE}"
    fi
    export CAMERA_IMAGE_TOPIC CAMERA_INFO_TOPIC CAPTURE_CAMERA_TOPIC RAW_LIVOX_TOPIC
    export SCAN_TOPIC WHEEL_ODOM_TOPIC IMU_TOPIC SENSOR_JOY_TOPIC MANUAL_CAPTURE_OUTPUT_BASE
    exec python3 "${ROOT}/manual_mission_capture.py"
fi

require_command() {
    command -v "$1" >/dev/null 2>&1 || {
        printf 'Missing command: %s\n' "$1" >&2
        return 1
    }
}

require_configuration() {
    local name
    for name in MINI_PC_HOST MINI_PC_USER MINI_PC_WORKSPACE ROS_MASTER_PORT \
        ROS_MASTER_HOST MINI_PC_ROS_IP D435F_SERIAL ROBOT_HOST ROBOT_WS_PORT LOCAL_WS_PORT ACCID POINTCLOUD_TOPIC \
        CAMERA_IMAGE_TOPIC CAMERA_INFO_TOPIC TAG_DETECTIONS_TOPIC INITIAL_FLOOR \
        HOME_LOCATION_ID INITIAL_LOCATION_ID INSPECT_PROFILE_ID SCAN_OUTPUT_BASE; do
        if [[ -z "${!name:-}" ]]; then
            printf 'Missing deployment value in config.env: %s\n' "$name" >&2
            return 1
        fi
    done
}

source_workspace() {
    if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
        printf 'ROS Noetic is not installed: /opt/ros/noetic/setup.bash\n' >&2
        return 1
    fi
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    if [[ ! -r "${ROOT}/devel/setup.bash" ]]; then
        printf 'Workspace is not built; run catkin_make in %s\n' "$ROOT" >&2
        return 1
    fi
    # shellcheck disable=SC1091
    source "${ROOT}/devel/setup.bash"
}

local_check() {
    require_configuration
    python3 "${ROOT}/validate_bundle.py" --site-config-root "$FIXTURE_ROOT"
    local command_name
    for command_name in python3 ssh timeout flock ip awk pgrep roslaunch rostopic rospack rosnode; do
        require_command "$command_name"
    done
    source_workspace
    local package_name
    for package_name in mission_manager multifloor_manager stair_supervisor rviz \
        map_server amcl move_base pointcloud_to_laserscan apriltag_ros; do
        rospack find "$package_name" >/dev/null
    done
    python3 -c 'import rospy, websocket, yaml'
    if command -v timedatectl >/dev/null 2>&1 \
        && [[ "$(timedatectl show -p NTPSynchronized --value)" != "yes" ]]; then
        printf 'Workstation clock is not NTP-synchronized.\n' >&2
        return 1
    fi
    printf '[CHECK] bundle, fixture, packages, and local software: OK\n'
}

production_check() {
    python3 "${ROOT}/validate_bundle.py" --validate-production-config
    printf '[CHECK] production site profile: OK\n'
}

remote_check() {
    local target="${MINI_PC_USER}@${MINI_PC_HOST}"
    local remote_epoch local_epoch delta
    printf '[SSH] connecting to mini PC: %s\n' "$target"
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" \
        "test -r /opt/ros/noetic/setup.bash && test -r '${MINI_PC_WORKSPACE}/devel/setup.bash' && test -r '${MINI_PC_WORKSPACE}/src/sensor_integration/launch/wf_mapping.launch'"
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" \
        "source /opt/ros/noetic/setup.bash; source '${MINI_PC_WORKSPACE}/devel/setup.bash'; export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}; export ROS_IP=${MINI_PC_ROS_IP}; export D435F_SERIAL=${D435F_SERIAL}; unset ROS_HOSTNAME; roslaunch --files sensor_integration wf_mapping.launch >/dev/null; roslaunch --files sensor_integration d435f.launch >/dev/null"
    if [[ "$(ssh -o BatchMode=yes "$target" 'timedatectl show -p NTPSynchronized --value')" != "yes" ]]; then
        printf 'Mini PC clock is not NTP-synchronized.\n' >&2
        return 1
    fi
    remote_epoch="$(ssh -o BatchMode=yes "$target" 'date +%s')"
    local_epoch="$(date +%s)"
    delta=$((local_epoch - remote_epoch))
    (( delta < 0 )) && delta=$((-delta))
    if (( delta > 1 )); then
        printf 'Clock difference is too large: %ss\n' "$delta" >&2
        return 1
    fi
    ssh -o BatchMode=yes "$target" "ping -c 1 -W 2 '${ROBOT_HOST}' >/dev/null"
    printf '[CHECK] mini PC, clock, SSH, and robot route: OK\n'
}

if [[ "${1:-}" == "--check" ]]; then
    local_check
    exit 0
fi
if [[ "${1:-}" == "--preflight" ]]; then
    local_check
    production_check
    remote_check
    exit 0
fi

python3 "${ROOT}/validate_bundle.py" --normalize-map
local_check
production_check
remote_check

SSH_TARGET="${MINI_PC_USER}@${MINI_PC_HOST}"
ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}"
ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" | awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')"
if [[ -z "$ROS_IP" ]]; then
    printf 'Cannot determine the workstation ROS_IP.\n' >&2
    exit 1
fi
export ROS_MASTER_URI ROS_IP
unset ROS_HOSTNAME

SENSOR_STACK_ACTION="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$SSH_TARGET" \
     "mapping_status=; if pgrep -f '[r]oslaunch.*wf_mapping.launch' >/dev/null; then mapping_status=reused; else nohup setsid bash -lc 'source /opt/ros/noetic/setup.bash; source ${MINI_PC_WORKSPACE}/devel/setup.bash; export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}; export ROS_IP=${MINI_PC_ROS_IP}; export D435F_SERIAL=${D435F_SERIAL}; unset ROS_HOSTNAME; exec roslaunch sensor_integration wf_mapping.launch' >/tmp/tron1-system.log 2>&1 < /dev/null & mapping_status='start requested'; fi; printf 'mapping=%s' \"$mapping_status\"")"
printf '[SSH] mini PC sensor stack: %s\n' "$SENSOR_STACK_ACTION"

for _ in {1..30}; do
    timeout 3s rostopic list >/dev/null 2>&1 && break
    sleep 1
done
if ! timeout 3s rostopic list >/dev/null 2>&1; then
    printf 'ROS master is unavailable: %s\n' "$ROS_MASTER_URI" >&2
    exit 1
fi

exec 9>/tmp/tron1_system.lock
if ! flock -n 9; then
    printf 'TRON1 mission system is already running.\n' >&2
    exit 1
fi

if pgrep -f '[a]priltag_ros_continuous_node' >/dev/null 2>&1 || \
   timeout 3s rosnode list 2>/dev/null | awk '$1 == "/apriltag_ros_continuous_node" {found=1} END {exit !found}'; then
    printf '%s\n' \
        'AprilTag detector is already running.' \
        'Stop the existing detector launch first; system.launch owns exactly one detector.' >&2
    exit 1
fi

LOG_DIR="${ROOT}/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
pids=()
LAST_PID=""

start_component() {
    local name="$1"
    shift
    "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
    LAST_PID=$!
    pids+=("$LAST_PID")
    printf '[START] %s (PID %s)\n' "$name" "$LAST_PID"
}

cleanup() {
    trap - INT TERM EXIT
    local pid
    printf '\n[STOP] local mission system\n'
    for pid in "${pids[@]:-}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${pids[@]:-}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
}
trap cleanup INT TERM EXIT

start_component robot_tunnel ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes \
    -L "127.0.0.1:${LOCAL_WS_PORT}:${ROBOT_HOST}:${ROBOT_WS_PORT}" "$SSH_TARGET"
tunnel_pid="$LAST_PID"
sleep 2
if ! kill -0 "$tunnel_pid" 2>/dev/null; then
    printf 'Robot WebSocket SSH tunnel failed.\n' >&2
    exit 1
fi
printf '[SSH] robot WebSocket tunnel: OK\n'

start_component system roslaunch mission_manager system.launch \
    cloud_in:="$POINTCLOUD_TOPIC" image_rect:="$CAMERA_IMAGE_TOPIC" \
    camera_info:="$CAMERA_INFO_TOPIC" tag_topic:="$TAG_DETECTIONS_TOPIC" \
    initial_floor:="$INITIAL_FLOOR" home_location_id:="$HOME_LOCATION_ID" \
    initial_location_id:="$INITIAL_LOCATION_ID" inspect_profile_id:="$INSPECT_PROFILE_ID" \
    scan_output_base:="$SCAN_OUTPUT_BASE" accid:="$ACCID" \
    websocket_url:="ws://127.0.0.1:${LOCAL_WS_PORT}" \
    initial_x:="$NAV_START_X" initial_y:="$NAV_START_Y" initial_yaw:="$NAV_START_YAW" \
    max_vel_x:="$MAX_VEL_X" max_vel_theta:="$MAX_VEL_THETA" \
    min_in_place_vel_theta:="$MIN_IN_PLACE_VEL_THETA" \
    acc_lim_x:="$ACC_LIM_X" acc_lim_theta:="$ACC_LIM_THETA"
system_pid="$LAST_PID"

wait_for_stream() {
    local topic="$1"
    local output
    output="$(timeout 12s rostopic hz -w 2 "$topic" 2>/dev/null || true)"
    if [[ "$output" != *"average rate:"* ]]; then
        printf 'Required fresh topic is unavailable: %s\n' "$topic" >&2
        return 1
    fi
}

wait_for_value() {
    local topic="$1"
    local expected="$2"
    local actual
    actual="$(timeout 30s rostopic echo -n 1 "$topic" 2>/dev/null | tr -d '[:space:]')"
    if [[ "$actual" != "$expected" ]]; then
        printf 'Readiness state mismatch: %s expected=%s actual=%s\n' \
            "$topic" "$expected" "$actual" >&2
        return 1
    fi
}

wait_for_fresh_message() {
    local topic="$1"
    local message_type="$2"
    local actual_type
    actual_type="$(timeout 3s rostopic type "$topic" 2>/dev/null || true)"
    if [[ "$actual_type" != "$message_type" ]]; then
        printf 'Topic type mismatch: %s expected=%s actual=%s\n' \
            "$topic" "$message_type" "$actual_type" >&2
        return 1
    fi
    if ! timeout 12s rostopic echo -n 1 "$topic" 2>/dev/null | \
        python3 -c '
import re
import sys
import time

topic = sys.argv[1]
text = sys.stdin.read()
seconds = re.findall(r"^\s+secs:\s*(-?\d+)\s*$", text, re.MULTILINE)
nanos = re.findall(r"^\s+nsecs:\s*(\d+)\s*$", text, re.MULTILINE)
if not seconds or not nanos:
    raise SystemExit("message has no timestamp: " + topic)
stamp = int(seconds[0]) + int(nanos[0]) / 1_000_000_000
age = abs(time.time() - stamp)
if stamp == 0 or age > 2.0:
    raise SystemExit("stale message: {} age={:.3f}s".format(topic, age))
' "$topic"; then
        printf 'Fresh message check failed: %s\n' "$topic" >&2
        return 1
    fi
}

for topic in /mission/status /multifloor/floor_transition/status /stair_traversal/status; do
    wait_for_stream "$topic"
done
wait_for_value /multifloor/floor_state/state 2
wait_for_value /stair_supervisor/state/state 1
for topic in /scan /tron/wheel_odom_raw /tf "$TAG_DETECTIONS_TOPIC"; do
    wait_for_stream "$topic"
done
wait_for_fresh_message /scan sensor_msgs/LaserScan
wait_for_fresh_message /tron/wheel_odom_raw nav_msgs/Odometry
wait_for_fresh_message /tf tf2_msgs/TFMessage
wait_for_fresh_message "$TAG_DETECTIONS_TOPIC" apriltag_ros/AprilTagDetectionArray
for topic in /map/info /move_base/global_costmap/costmap/info /move_base/local_costmap/costmap/info; do
    timeout 25s rostopic echo -n 1 "$topic" >/dev/null
done

scan_publishers="$(rostopic info /scan | awk '/^Publishers:/{active=1; next} /^Subscribers:/{active=0} active && /^ \* /{count++} END{print count+0}')"
if [[ "$scan_publishers" != "1" ]]; then
    printf 'Expected exactly one /scan publisher; found %s.\n' "$scan_publishers" >&2
    exit 1
fi
if ! kill -0 "$system_pid" 2>/dev/null; then
    printf 'Top-level system launch exited before readiness.\n' >&2
    exit 1
fi

printf '%s\n' \
    '[READY] Mission, floor-transition, and stair action servers are ready.' \
    '[READY] FloorState=READY and SupervisorState=NAV.' \
    '[READY] Fresh scan, odometry, TF, and AprilTag input verified.' \
    '[READY] Submit goals only through /mission.' \
    '[READY] Stop with Ctrl+C. The mini-PC sensor stack remains running.' \
    "[LOG] ${LOG_DIR}"
wait "$system_pid"
