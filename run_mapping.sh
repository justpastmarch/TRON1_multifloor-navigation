#!/usr/bin/env bash
# GUI entry for independent new-site 2D mapping. No robot motion transport.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/noetic/setup.bash
source "$ROOT/devel/setup.bash"
source "$ROOT/config.env"
set -u
printf '[NET] %s path: mini PC %s, ROS master %s\n' "$TRON_NETWORK" "$MINI_PC_HOST" "$ROS_MASTER_HOST"
export ROS_MASTER_URI="${MAPPING_MASTER_URI:-http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}}"
export ROS_IP="${MAPPING_ROS_IP:-${ROS_MASTER_HOST}}"
unset ROS_HOSTNAME
if [[ "${1:-}" == "--check" ]]; then
    rospack find slam_toolbox
    rospack find gmapping
    rospack find mission_manager
    exit 0
fi
exec 9>/tmp/tron1_system.lock
if ! flock -n 9; then
    echo '기존 TRON 실행을 종료한 뒤 지도 작성 프로그램을 여세요. 주행과 매핑은 동시에 실행하지 않습니다.' >&2
    exit 1
fi
pids=()
cleanup() {
    trap - INT TERM EXIT
    local pid
    for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if ! timeout 3s rostopic list >/dev/null 2>&1; then
    roscore -p "${ROS_MASTER_URI##*:}" &
    pids+=("$!")
    ready=0
    for _ in {1..40}; do
        if timeout 2s rostopic list >/dev/null 2>&1; then ready=1; break; fi
        sleep .2
    done
    [[ "$ready" == 1 ]] || { echo 'ROS Master를 시작하지 못했습니다.' >&2; exit 1; }
fi
# Reuse the same systemd sensor owner. Replay/test masters never touch MiniPC.
if [[ -z "${MAPPING_MASTER_URI:-}" ]]; then
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${MINI_PC_USER}@${MINI_PC_HOST}" \
        'systemctl --user start tron1-sensors.service' || \
        echo '센서 서비스 연결 대기: GUI의 scan 상태를 확인하세요.' >&2
    # The mini PC raw LiDAR recorder (shared with stair runs) reads its master
    # from runtime.env; point it at this session's network path.
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${MINI_PC_USER}@${MINI_PC_HOST}" \
        "f=~/.local/share/tron1-stair-supervisor/runtime.env; test -f \$f && sed -i -e 's#^ROS_MASTER_URI=.*#ROS_MASTER_URI=${ROS_MASTER_URI}#' -e 's#^ROS_IP=.*#ROS_IP=${MINI_PC_ROS_IP}#' \$f" || \
        echo '미니 PC 원본 LiDAR 녹화 설정을 갱신하지 못했습니다. 3D 지도는 만들어지지 않을 수 있습니다.' >&2
    export MAPPING_RAW_RECORD_TOOL="${ROOT}/remote_stair_record.sh"
    export MAPPING_CLOUD_TOOL="${ROOT}/remote_mapping_cloud.sh"
fi
export STAIR_LIDAR_CONFIG SCAN_OUTPUT_BASE
PORT="${MAPPING_CONSOLE_PORT:-8766}"
rosrun mission_manager mission_manager_node.py _mode:=mapping _console_port:="$PORT" _scan_output_base:="$SCAN_OUTPUT_BASE" &
node_pid=$!
pids+=("$node_pid")
if [[ "${MAPPING_OPEN_BROWSER:-0}" == 1 ]]; then
    python3 - "$PORT" <<'PY' &
import sys,time,urllib.request,subprocess
url='http://127.0.0.1:'+sys.argv[1]
for _ in range(80):
    try:
        urllib.request.urlopen(url+'/api/state',timeout=1).close()
        subprocess.run(['xdg-open',url],check=False)
        break
    except OSError:time.sleep(.25)
PY
    pids+=("$!")
fi
printf '지도 관리 UI: http://127.0.0.1:%s/\n물리 조종기로 이동하세요. 이 프로그램은 이동 명령을 전송하지 않습니다.\n' "$PORT"
wait "$node_pid"
