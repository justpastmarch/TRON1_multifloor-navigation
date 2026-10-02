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

# ROS/catkin setup hooks read optional variables before defining them.
# Keep strict checks in our code, but permit unset variables inside setup hooks.
source_ros_setup() {
    local setup_status=0
    set +u
    source "$1" || setup_status=$?
    set -u
    return "$setup_status"
}

if [[ "${1:-}" == "--record-manual" ]]; then
    if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
        printf 'ROS Noetic is not installed: /opt/ros/noetic/setup.bash\n' >&2
        exit 1
    fi
    # shellcheck disable=SC1091
    source_ros_setup /opt/ros/noetic/setup.bash
    if [[ -r "${ROOT}/devel/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source_ros_setup "${ROOT}/devel/setup.bash"
    fi
    if [[ "$SENSOR_JOY_RECEIVER_AUTOSTART" == "1" ]]; then
        receiver_status="$(ssh -o BatchMode=yes -o ConnectTimeout=5 \
            "${MINI_PC_USER}@${MINI_PC_HOST}" \
            "root=\"\${HOME}/.local/share/tron1-sensor-joy\"; pattern='^/home/${MINI_PC_USER}/.local/share/tron1-sensor-joy/venv/bin/python /home/${MINI_PC_USER}/.local/share/tron1-sensor-joy/sensor_joy_bridge.py'; existing=\$(pgrep -f \"\$pattern\" || true); if [ -n \"\$existing\" ]; then printf 'reused pid=%s' \"\$existing\"; else test -x \"\$root/venv/bin/python\" && test -x \"\$root/sensor_joy_bridge.py\" || exit 1; nohup setsid bash -lc 'source /opt/ros/noetic/setup.bash; source ${MINI_PC_WORKSPACE}/devel/setup.bash; export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}; export ROS_IP=${MINI_PC_ROS_IP}; export ROBOT_TYPE=${ROBOT_TYPE}; unset ROS_HOSTNAME; exec /home/${MINI_PC_USER}/.local/share/tron1-sensor-joy/venv/bin/python /home/${MINI_PC_USER}/.local/share/tron1-sensor-joy/sensor_joy_bridge.py --robot-ip ${ROBOT_HOST} --topic ${SENSOR_JOY_TOPIC}' >\"\$root/receiver.log\" 2>&1 < /dev/null & pid=\$!; printf '%s\\n' \"\$pid\" >\"\$root/receiver.pid\"; printf 'started pid=%s' \"\$pid\"; fi")"
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
    source_ros_setup /opt/ros/noetic/setup.bash
    if [[ ! -r "${ROOT}/devel/setup.bash" ]]; then
        printf 'Workspace is not built; run catkin_make in %s\n' "$ROOT" >&2
        return 1
    fi
    # shellcheck disable=SC1091
    source_ros_setup "${ROOT}/devel/setup.bash"
}

local_check() {
    require_configuration
    python3 "${ROOT}/validate_bundle.py" --site-config-root "$FIXTURE_ROOT"
    local command_name
    for command_name in python3 ssh timeout flock ip awk pgrep roscore roslaunch rostopic rospack rosnode; do
        require_command "$command_name"
    done
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
    case "${STAIR_LIDAR_MODE:-off}" in
        off) ;;
        control|observe)
            : "${STAIR_PYTHON:?Set the existing LiDAR Python runner}"
            : "${STAIR_LIDAR_CONFIG:?Set the LiDAR configuration}"
            "$STAIR_PYTHON" "$ROOT/src/stair_supervisor/scripts/check_lidar_config.py" \
                "$STAIR_LIDAR_CONFIG" --mode "$STAIR_LIDAR_MODE"
            ;;
        *) printf 'Invalid STAIR_LIDAR_MODE: %s\n' "$STAIR_LIDAR_MODE" >&2; return 1 ;;
    esac
    printf '[CHECK] bundle, fixture, packages, local software, and selected stair config: OK\n'
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
        "test -r /opt/ros/noetic/setup.bash && test -r '${MINI_PC_WORKSPACE}/devel/setup.bash' && test -r '${MINI_PC_WORKSPACE}/src/sensor_integration/launch/wf_mapping.launch' && command -v tmux >/dev/null"
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" \
        "source /opt/ros/noetic/setup.bash && source '${MINI_PC_WORKSPACE}/devel/setup.bash' && export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT} && export ROS_IP=${MINI_PC_ROS_IP} && export D435F_SERIAL=${D435F_SERIAL} && unset ROS_HOSTNAME && roslaunch --files sensor_integration wf_mapping.launch >/dev/null && roslaunch --files sensor_integration d435f.launch >/dev/null"
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

# Load ROS before validators import package initializers and before CLI checks.
source_workspace

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
ROS_IP="$(ip -4 route get "$MINI_PC_HOST" | awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')"
if [[ -z "$ROS_IP" ]]; then
    printf 'Cannot determine the workstation ROS_IP.\n' >&2
    exit 1
fi
if [[ "$ROS_MASTER_HOST" != "$ROS_IP" ]]; then
    printf 'ROS master must use the workstation LAN address: expected=%s actual=%s\n' \
        "$ROS_IP" "$ROS_MASTER_HOST" >&2
    exit 1
fi
export ROS_MASTER_URI ROS_IP
unset ROS_HOSTNAME

exec 9>/tmp/tron1_system.lock
if ! flock -n 9; then
    printf 'TRON1 mission system is already running.\n' >&2
    exit 1
fi

LOG_DIR="${ROOT}/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
pids=()
LAST_PID=""
stair_record_pid=""

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
        [[ "$pid" == "$stair_record_pid" ]] && continue
        kill -TERM "$pid" 2>/dev/null || true
    done
    # rosbag needs time to finish its index after SIGINT. Do not terminate it
    # after the generic two-second grace period used for other components.
    if [[ -n "$stair_record_pid" ]]; then
        for _ in {1..40}; do
            kill -0 "$stair_record_pid" 2>/dev/null || break
            sleep .25
        done
        if kill -0 "$stair_record_pid" 2>/dev/null; then
            printf '[BAG] recorder did not finish; preserve .active for recovery. PID=%s\n' "$stair_record_pid" >&2
            kill -TERM "$stair_record_pid" 2>/dev/null || true
        fi
        wait "$stair_record_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_component ros_master roscore -p "$ROS_MASTER_PORT"
master_pid="$LAST_PID"
for _ in {1..30}; do
    timeout 3s rostopic list >/dev/null 2>&1 && break
    sleep 1
done
if ! kill -0 "$master_pid" 2>/dev/null || ! timeout 3s rostopic list >/dev/null 2>&1; then
    printf 'Workstation ROS master failed to start: %s\n' "$ROS_MASTER_URI" >&2
    exit 1
fi
printf '[START] workstation ROS master: %s\n' "$ROS_MASTER_URI"

# A boot-managed sensor launch has one owner. Do not replace it with tmux.
if ssh -o BatchMode=yes -o ConnectTimeout=5 "$SSH_TARGET" \
    'systemctl --user cat tron1-sensors.service >/dev/null 2>&1'; then
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$SSH_TARGET" \
        'systemctl --user start tron1-sensors.service'
    SENSOR_STACK_ACTION='systemd managed (starting or already running)'
else
SENSOR_STACK_ACTION="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$SSH_TARGET" \
     "mapping_status=; sensor_ready=1; source /opt/ros/noetic/setup.bash && source ${MINI_PC_WORKSPACE}/devel/setup.bash && export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT} && export ROS_IP=${MINI_PC_ROS_IP} && unset ROS_HOSTNAME || sensor_ready=0; if [ \"\$sensor_ready\" = 1 ]; then for topic in /livox/lidar /tron/wheel_odom_raw /scan ${CAMERA_IMAGE_TOPIC} ${CAMERA_INFO_TOPIC}; do timeout 5s rostopic echo -n 1 \"\$topic\" >/dev/null 2>&1 || { sensor_ready=0; break; }; done; fi; if [ \"\$sensor_ready\" = 1 ]; then mapping_status=reused; else tmux kill-session -t wf_mapping 2>/dev/null || true; pkill -INT -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch sensor_integration wf_mapping.launch$' 2>/dev/null || true; sleep 5; pkill -TERM -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch sensor_integration wf_mapping.launch$' 2>/dev/null || true; tmux new-session -d -s wf_mapping 'source /opt/ros/noetic/setup.bash; source ${MINI_PC_WORKSPACE}/devel/setup.bash; export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}; export ROS_IP=${MINI_PC_ROS_IP}; export D435F_SERIAL=${D435F_SERIAL}; unset ROS_HOSTNAME; exec roslaunch sensor_integration wf_mapping.launch' || exit 1; mapping_status='restart requested'; fi; printf 'mapping=%s' \"\$mapping_status\"")"
fi
printf '[SSH] mini PC sensor stack: %s\n' "$SENSOR_STACK_ACTION"

if [[ "${SENSOR_JOY_RECEIVER_AUTOSTART:-0}" == "1" ]]; then
    bash "$ROOT/sensor_joy_session.sh" || printf '%s\n' \
        '[WARN] SensorJoy is not publishing; restore it before recording a field test.' >&2
fi

if pgrep -f '[a]priltag_ros_continuous_node' >/dev/null 2>&1 || \
   timeout 3s rosnode list 2>/dev/null | awk '$1 == "/apriltag_ros_continuous_node" {found=1} END {exit !found}'; then
    printf '%s\n' \
        'AprilTag detector is already running.' \
        'Stop the existing detector launch first; system.launch owns exactly one detector.' >&2
    exit 1
fi

start_component robot_tunnel ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes \
    -L "127.0.0.1:${LOCAL_WS_PORT}:${ROBOT_HOST}:${ROBOT_WS_PORT}" "$SSH_TARGET"
tunnel_pid="$LAST_PID"
sleep 2
if ! kill -0 "$tunnel_pid" 2>/dev/null; then
    printf 'Robot WebSocket SSH tunnel failed.\n' >&2
    exit 1
fi
printf '[SSH] robot WebSocket tunnel: OK\n'

lidar_launch_args=()
if [ -n "${STAIR_LIDAR_CONFIG:-}" ]; then
    lidar_launch_args+=("stair_lidar_config:=$STAIR_LIDAR_CONFIG")
fi
if [ -n "${ARRIVAL_HOLD_SETTINGS:-}" ]; then
    lidar_launch_args+=("arrival_hold_enabled:=true" "arrival_hold_settings:=$ARRIVAL_HOLD_SETTINGS")
fi
if [[ "${STAIR_RECORD:-0}" == "1" && "${MISSION_CONSOLE_PORT:-0}" == "0" ]]; then
    : "${STAIR_LIDAR_CONFIG:?STAIR_RECORD needs an explicit LiDAR config}"
    : "${STAIR_PYTHON:?STAIR_RECORD needs the LiDAR Python interpreter}"
    "$STAIR_PYTHON" "$ROOT/src/stair_supervisor/scripts/check_lidar_config.py" \
        "$STAIR_LIDAR_CONFIG" --mode "${STAIR_LIDAR_MODE:-control}" --snapshot-dir "$LOG_DIR"
    start_component stair_bag rosbag record --split --size=1024 -O "$LOG_DIR/stair" \
        /livox/lidar /livox/imu /tron/wheel_odom_raw /tron/sensor_joy /scan /tf /tf_static \
        /navigation/cmd_vel /stair_supervisor/websocket_tx /stair_supervisor/state \
        /stair_supervisor/tracking_status /stair_supervisor/control_debug /stair_supervisor/geometry_markers \
        /stair_supervisor/lidar_odom /stair_traversal/goal /stair_traversal/cancel \
        /stair_traversal/feedback /stair_traversal/result /mission/status /multifloor/floor_state
    stair_record_pid="$LAST_PID"
    printf '[BAG] automatic stair recording: %s\n' "$LOG_DIR"
fi
# Resolve the selected floor through the existing site configuration.
initial_map_yaml="$(python3 -c 'import pathlib,sys,yaml; root=pathlib.Path(sys.argv[1]); rows=yaml.safe_load((root/"floors.yaml").read_text())["floors"]; print(root/next(x["map_yaml"] for x in rows if x["id"]==sys.argv[2]))' "$ROOT/src/multifloor_manager/config" "$INITIAL_FLOOR")"
start_component system roslaunch mission_manager system.launch "map_yaml:=$initial_map_yaml" "${lidar_launch_args[@]}" "console_port:=${MISSION_CONSOLE_PORT:-0}" \
    startup_localization:="${STARTUP_LOCALIZATION:-auto}" \
    stair_lidar_mode:="${STAIR_LIDAR_MODE:-off}" \
    stair_lidar_observe_only:="${STAIR_LIDAR_OBSERVE_ONLY:-true}" \
    stair_python:="${STAIR_PYTHON:-}" \
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
    actual="$(timeout 30s rostopic echo -n 1 "$topic" 2>/dev/null | tr -cd '[:digit:]' || true)"
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
    if ! timeout 20s rostopic echo -n 1 "$topic" 2>/dev/null | \
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

wait_for_action_server() {
    local topic="$1"
    local actual_type publishers
    for _ in {1..60}; do
    actual_type="$(timeout 15s rostopic type "$topic" 2>/dev/null || true)"
        if [[ "$actual_type" == "actionlib_msgs/GoalStatusArray" ]]; then
            break
        fi
        sleep 1
    done
    if [[ "$actual_type" != "actionlib_msgs/GoalStatusArray" ]]; then
        printf 'Action status topic type mismatch: %s expected=actionlib_msgs/GoalStatusArray actual=%s\n' \
            "$topic" "$actual_type" >&2
        return 1
    fi
    local raw
    for _ in {1..30}; do
        raw="$(timeout 5s rostopic info "$topic" 2>/dev/null || true)"
        if [[ -n "$raw" ]]; then
            publishers="$(printf '%s' "$raw" | \
                awk '/^Publishers:/{active=1; next} /^Subscribers:/{active=0} active && /^ \* /{count++} END{print count+0}')"
        else
            publishers=0
        fi
        if (( publishers >= 1 )); then
            break
        fi
        sleep 1
    done
    if (( publishers < 1 )); then
        printf 'Action server has no publisher: %s\n' "$topic" >&2
        return 1
    fi
}

for topic in /mission/status /multifloor/floor_transition/status /stair_traversal/status; do
    wait_for_action_server "$topic"
done
python3 "${ROOT}/verify_action_servers.py"
printf '%s\n' '[LOCALIZATION] Finding the current pose. Use RViz 2D Pose Estimate to set position and direction.'
while true; do
    kill -0 "$system_pid" 2>/dev/null || { printf 'System launch exited.\n' >&2; exit 1; }
    if [[ -n "$stair_record_pid" ]] && ! kill -0 "$stair_record_pid" 2>/dev/null; then
        printf '[BAG] recording exited; see %s/stair_bag.log. Mission continues.\n' "$LOG_DIR" >&2
        stair_record_pid=""
    fi
    floor_state="$(timeout 3s rostopic echo -n 1 /multifloor/floor_state/state 2>/dev/null | tr -cd '[:digit:]' || true)"
    [[ "$floor_state" == "2" ]] && break
    [[ "$floor_state" == "3" ]] && { printf 'Floor manager fault.\n' >&2; exit 1; }
    sleep 1
done
wait_for_value /stair_supervisor/state/state 1
for topic in /scan /tron/wheel_odom_raw /tf; do
    wait_for_stream "$topic"
done
wait_for_fresh_message /scan sensor_msgs/LaserScan
wait_for_fresh_message /tron/wheel_odom_raw nav_msgs/Odometry
wait_for_fresh_message /tf tf2_msgs/TFMessage
# Tag input is required by floor-arrival verification, not flat NAV startup.
# The detector already respawns; its outage must not tear down mission/UI.
printf '%s\n' '[INFO] AprilTag input is monitored separately; floor arrival still requires valid tag evidence.'
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
    '[READY] Fresh scan, odometry and TF verified. AprilTag readiness is shown in the UI.' \
    '[READY] Normal missions use /mission; explicit stair trials use stair_entry_test.py.' \
    '[READY] Stop with Ctrl+C. The mini-PC sensor stack remains running.' \
    "[LOG] ${LOG_DIR}"
if [[ -n "$stair_record_pid" ]]; then
    # Recording failure is visible without making it a mobility stop gate.
    wait -n "$system_pid" "$stair_record_pid" || true
    if kill -0 "$system_pid" 2>/dev/null && ! kill -0 "$stair_record_pid" 2>/dev/null; then
        printf '[BAG] recording exited; mission continues. See %s/stair_bag.log\n' "$LOG_DIR" >&2
    fi
fi
wait "$system_pid"
