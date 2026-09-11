#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"; MODE="${1:-common}"
PASS_COUNT=0; WARN_COUNT=0; FAIL_COUNT=0

usage() {
  cat <<'EOF'
Usage: bash notebooks/check_environment.sh [MODE]

Read-only checks. This script never starts, stops, or reconfigures ROS nodes.

Modes:
  common      Local workspace, config.env, clock, ROS packages, and ROS master
  mapping     common + mini PC SSH/sensor-stack files and current sensor topics
  navigation  common + live manual navigation nodes, topics, and supervisor state
  recording   common + live mandatory recording topics, disk, and recorder ownership
EOF
}

pass() {
  PASS_COUNT=$((PASS_COUNT + 1)); printf '[PASS] %s\n' "$1"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1)); printf '[WARN] %s\n       -> %s\n' "$1" "$2"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1)); printf '[FAIL] %s\n       -> %s\n' "$1" "$2"
}

require_command() {
  if command -v "$1" >/dev/null 2>&1; then
    pass "command available: $1"
  else
    fail "command missing: $1" "$2"
  fi
}

require_key() {
  if [[ -n "${!1:-}" ]]; then
    pass "config.env key populated: $1"
  else
    fail "config.env key missing: $1" "set $1 in ${ROOT}/config.env"
  fi
}

require_package() {
  if rospack find "$1" >/dev/null 2>&1; then
    pass "ROS package available: $1"
  else
    fail "ROS package missing: $1" "install/build the package, then source devel/setup.bash"
  fi
}

require_topic_type() {
  local topic="$1" expected="$2" actual
  actual="$(timeout 3s rostopic type "$topic" 2>/dev/null || true)"
  if [[ "$actual" == "$expected" ]]; then
    pass "topic ready: ${topic} (${expected})"
  else
    fail "topic unavailable or wrong type: ${topic}" "expected ${expected}; actual ${actual:-<none>}"
  fi
}

check_common() {
  printf '\n== Common notebook prerequisites ==\n'

  if [[ -r /opt/ros/noetic/setup.bash ]]; then
    pass 'ROS Noetic setup exists'
  else
    fail 'ROS Noetic setup missing' 'install ROS Noetic desktop/navigation packages'
  fi
  if [[ -r "${ROOT}/devel/setup.bash" ]]; then
    pass 'workspace is built'
  else
    fail 'workspace setup missing' "run: cd ${ROOT} && catkin_make"
  fi
  if [[ -r "${ROOT}/config.env" ]]; then
    pass 'config.env exists'
  else
    fail 'config.env missing' "create ${ROOT}/config.env from the deployment values"
    return
  fi

  for command_name in python3 timeout ip awk ssh rostopic rospack rosnode roslaunch; do
    require_command "$command_name" 'complete the root README installation steps'
  done
  if command -v jupyter >/dev/null 2>&1; then
    pass 'Jupyter is installed'
  else
    warn 'Jupyter is not installed' 'run: sudo apt install jupyter-notebook python3-ipykernel'
  fi

  if [[ -r /opt/ros/noetic/setup.bash && -r "${ROOT}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    # shellcheck disable=SC1091
    source "${ROOT}/devel/setup.bash"
  fi
  set -a
  # shellcheck disable=SC1091
  source "${ROOT}/config.env"
  set +a

  for key in ROS_MASTER_HOST ROS_MASTER_PORT; do
    require_key "$key"
  done
  if [[ -z "${ROS_MASTER_HOST:-}" || -z "${ROS_MASTER_PORT:-}" ]]; then
    return
  fi

  export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}"
  local route
  route="$(ip -4 route get "$ROS_MASTER_HOST" 2>/dev/null || true)"
  if [[ "$route" =~ [[:space:]]src[[:space:]]([^[:space:]]+) ]]; then
    export ROS_IP="${BASH_REMATCH[1]}"
    unset ROS_HOSTNAME
    pass "ROS network identity: ROS_MASTER_URI=${ROS_MASTER_URI}, ROS_IP=${ROS_IP}"
  else
    fail 'no IPv4 route to ROS master' "check network access to ${ROS_MASTER_HOST}"
  fi

  for package_name in gmapping map_server move_base multifloor_manager stair_supervisor; do
    require_package "$package_name"
  done

  local ntp_status
  ntp_status="$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)"
  if [[ "$ntp_status" == 'yes' ]]; then
    pass 'local clock is NTP synchronized'
  else
    fail 'local clock is not NTP synchronized' 'enable time synchronization before multi-machine ROS use'
  fi

  if timeout 3s rostopic list >/dev/null 2>&1; then
    pass "ROS master reachable: ${ROS_MASTER_URI}"
  else
    fail 'ROS master is unreachable' 'start/locate the configured ROS master, then rerun this checker'
  fi
}

check_mapping() {
  printf '\n== Mapping prerequisites ==\n'
  for key in MINI_PC_USER MINI_PC_HOST MINI_PC_WORKSPACE MINI_PC_ROS_IP D435F_SERIAL; do
    require_key "$key"
  done
  if [[ -z "${MINI_PC_USER:-}" || -z "${MINI_PC_HOST:-}" || -z "${MINI_PC_WORKSPACE:-}" ]]; then
    return
  fi

  local target="${MINI_PC_USER}@${MINI_PC_HOST}"
  local remote_files
  remote_files="test -r /opt/ros/noetic/setup.bash && test -r '${MINI_PC_WORKSPACE}/devel/setup.bash' && test -r '${MINI_PC_WORKSPACE}/src/sensor_integration/launch/wf_mapping.launch'"
  if timeout 12s ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" "$remote_files"; then
    pass 'mini PC SSH key and sensor-stack files are ready'
  else
    fail 'mini PC SSH or sensor-stack files unavailable' "verify key login and ${MINI_PC_WORKSPACE}/src/sensor_integration/launch/wf_mapping.launch"
    return
  fi

  local remote_launch
  remote_launch="source /opt/ros/noetic/setup.bash; source '${MINI_PC_WORKSPACE}/devel/setup.bash'; export ROS_MASTER_URI='${ROS_MASTER_URI}'; export ROS_IP='${MINI_PC_ROS_IP}'; export D435F_SERIAL='${D435F_SERIAL}'; unset ROS_HOSTNAME; roslaunch --files sensor_integration wf_mapping.launch >/dev/null"
  if timeout 15s ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" "$remote_launch"; then
    pass 'remote wf_mapping.launch resolves without starting nodes'
  else
    fail 'remote wf_mapping.launch does not resolve' 'build the mini PC workspace and verify sensor_integration'
  fi

  local remote_ntp
  remote_ntp="$(timeout 8s ssh -o BatchMode=yes -o ConnectTimeout=5 "$target" 'timedatectl show -p NTPSynchronized --value' 2>/dev/null || true)"
  if [[ "$remote_ntp" == 'yes' ]]; then
    pass 'mini PC clock is NTP synchronized'
  else
    fail 'mini PC clock is not NTP synchronized' 'enable time synchronization on the mini PC'
  fi

  if [[ "$(timeout 3s rostopic type /scan 2>/dev/null || true)" == 'sensor_msgs/LaserScan' ]]; then
    pass 'current graph already has /scan'
  else
    warn '/scan is not running yet' 'Notebook 01 may start the verified remote sensor stack'
  fi
  if [[ "$(timeout 3s rostopic type /tron/wheel_odom_raw 2>/dev/null || true)" == 'nav_msgs/Odometry' ]]; then
    pass 'current graph already has wheel odometry'
  else
    warn 'wheel odometry is not running yet' 'Notebook 01 waits for it after starting sensors'
  fi
  warn 'mapping motion controller is external to this repository' 'do not start SLAM until the site-approved controller and physical E-stop are ready'
}

check_navigation() {
  printf '\n== Live manual navigation lane ==\n'
  for key in NAV_START_X NAV_START_Y NAV_START_YAW ROBOT_HOST ROBOT_WS_PORT LOCAL_WS_PORT ACCID; do
    require_key "$key"
  done

  local nodes
  nodes="$(timeout 3s rosnode list 2>/dev/null || true)"
  for node in /map_server /amcl /move_base /stair_supervisor; do
    if grep -Fxq "$node" <<<"$nodes"; then
      pass "node ready: ${node}"
    else
      fail "node missing: ${node}" 'start the numbered manual lane terminals in docs/ROSBAG-RECORD'
    fi
  done
  for conflict in /mission_manager /multifloor_manager /slam_gmapping; do
    if grep -Fxq "$conflict" <<<"$nodes"; then
      fail "conflicting node is running: ${conflict}" 'stop its owning terminal; do not mix managed, mapping, and direct navigation modes'
    else
      pass "conflicting node absent: ${conflict}"
    fi
  done

  require_topic_type /scan sensor_msgs/LaserScan
  require_topic_type /tron/wheel_odom_raw nav_msgs/Odometry
  require_topic_type /map nav_msgs/OccupancyGrid
  require_topic_type /amcl_pose geometry_msgs/PoseWithCovarianceStamped
  require_topic_type /stair_supervisor/state stair_supervisor/SupervisorState

  local supervisor
  supervisor="$(timeout 5s rostopic echo -n 1 /stair_supervisor/state 2>/dev/null || true)"
  if grep -Eq '^state: 1$' <<<"$supervisor" && grep -Eq '^connected: [Tt]rue$' <<<"$supervisor"; then
    pass 'stair supervisor is NAV(1) and connected'
  else
    fail 'stair supervisor is not ready for robot motion' 'require state: 1 and connected: True before opening Notebook 02'
  fi
}

check_recording() {
  printf '\n== Live recording inputs ==\n'
  for key in POINTCLOUD_TOPIC CAMERA_IMAGE_TOPIC CAMERA_INFO_TOPIC TAG_DETECTIONS_TOPIC; do
    require_key "$key"
  done
  if [[ -z "${POINTCLOUD_TOPIC:-}" || -z "${CAMERA_IMAGE_TOPIC:-}" || -z "${CAMERA_INFO_TOPIC:-}" ]]; then
    return
  fi

  require_topic_type "${CAMERA_IMAGE_TOPIC%/}/compressed" sensor_msgs/CompressedImage
  require_topic_type "$CAMERA_INFO_TOPIC" sensor_msgs/CameraInfo
  require_topic_type "$POINTCLOUD_TOPIC" livox_ros_driver2/CustomMsg
  require_topic_type /tron/wheel_odom_raw nav_msgs/Odometry
  require_topic_type /scan sensor_msgs/LaserScan
  require_topic_type /tf tf2_msgs/TFMessage
  require_topic_type /tf_static tf2_msgs/TFMessage

  if pgrep -af '[r]osbag[[:space:]]+record|/rosbag/[r]ecord' >/dev/null; then
    fail 'another recorder is already running' 'finish the recorder in its owning terminal or kernel'
  else
    pass 'no existing recorder process found'
  fi

  local disk_path="${ROOT}/stair_captures"
  [[ -d "$disk_path" ]] || disk_path="$ROOT"
  local free_kib
  free_kib="$(df -Pk "$disk_path" | awk 'NR == 2 {print $4}')"
  if [[ "$free_kib" =~ ^[0-9]+$ ]] && ((free_kib >= 10485760)); then
    pass 'recording destination has at least 10 GiB free'
  else
    fail 'recording destination has less than 10 GiB free' "free space under ${disk_path} before recording"
  fi
}

case "$MODE" in
  --help|-h)
    usage
    exit 0
    ;;
  common|mapping|navigation|recording)
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

printf 'TRON1 notebook environment check: mode=%s\nroot=%s\n' "$MODE" "$ROOT"
check_common
case "$MODE" in
  common) ;;
  mapping) check_mapping ;;
  navigation) check_navigation ;;
  recording) check_recording ;;
esac

printf '\nSummary: PASS=%d WARN=%d FAIL=%d\n' "$PASS_COUNT" "$WARN_COUNT" "$FAIL_COUNT"
if ((FAIL_COUNT > 0)); then
  exit 1
fi
