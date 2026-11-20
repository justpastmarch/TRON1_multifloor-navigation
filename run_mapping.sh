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
# Where the mapping console runs. auto: on the mini PC when the link is
# Tailscale (phone hotspot), here on the robotnet LAN.
MAPPING_LOCATION="${MAPPING_LOCATION:-auto}"
if [[ "$MAPPING_LOCATION" == auto ]]; then
    [[ "$TRON_NETWORK" == tailscale && -z "${MAPPING_MASTER_URI:-}" ]] && MAPPING_LOCATION=minipc || MAPPING_LOCATION=workstation
fi
if [[ "$MAPPING_LOCATION" == minipc ]]; then
    PORT="${MAPPING_CONSOLE_PORT:-8766}"
    target="${MINI_PC_USER}@${MINI_PC_HOST}"
    base="/home/${MINI_PC_USER}/.local/share/tron1-stair-supervisor"
    ssh_opts=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
    retry() { local n; for n in 1 2 3 4 5 6 7 8 9 10; do "$@" && return 0; printf '[MINI PC] 연결 재시도 %d/10\n' "$n" >&2; sleep 3; done; return 1; }
    printf '[MINI PC] 지도 관리를 미니 PC에서 실행합니다 (%s). 연결이 끊겨도 녹화와 지도 작성은 계속됩니다.\n' "$MINI_PC_HOST"
    export RSYNC_RSH="ssh ${ssh_opts[*]}"
    retry rsync -a --checksum --delete --exclude=test --exclude=__pycache__ --exclude='*.pyc' \
        "$ROOT/src/mission_manager/" "${target}:${base}/ws/src/mission_manager/" || { echo '[MINI PC] 코드 동기화 실패' >&2; exit 1; }
    retry rsync -a "$ROOT/remote_minipc_mapping.sh" "${target}:${base}/mapping.sh" || exit 1
    # Sensor watcher that can attach to a master on the mini PC itself.
    retry rsync -a "$ROOT/deploy/mini-pc-sensors/watch_master.py" "${target}:/home/${MINI_PC_USER}/.local/lib/tron1-sensors/watch_master.py" || exit 1
    retry ssh "${ssh_opts[@]}" "$target" "grep -q '^ROS_MASTER_URI_LOCAL=' ~/.config/tron1-sensors.env || printf 'ROS_MASTER_URI_LOCAL=http://127.0.0.1:11311\\nROS_IP_LOCAL=127.0.0.1\\n' >> ~/.config/tron1-sensors.env" || exit 1
    retry ssh "${ssh_opts[@]}" "$target" "bash '${base}/mapping.sh' start" || { echo '[MINI PC] 지도 관리 시작 실패' >&2; exit 1; }
    remote_cleanup() {
        trap - INT TERM EXIT
        pkill -P "$tunnel_pid" 2>/dev/null || true; kill "$tunnel_pid" 2>/dev/null || true
        printf '[MINI PC] 지도 관리 종료 중 (녹화 마감)...\n'
        retry ssh "${ssh_opts[@]}" "$target" "bash '${base}/mapping.sh' stop" || \
            echo '[MINI PC] 종료 명령을 보내지 못했습니다. 미니 PC에서 계속 실행 중입니다. 다시 실행하면 이어서 볼 수 있습니다.' >&2
        local library=/mnt/ethan/tron-bags/mapping
        mountpoint -q /mnt/ethan || library="${HOME}/TRON1_maps/mapping"
        mkdir -p "$library"
        printf '[MINI PC] 결과 지도를 %s 로 복사합니다 (원본 LiDAR BAG은 미니 PC에 남김)...\n' "$library"
        retry rsync -a --partial --exclude=raw.bag --exclude='*.active' --exclude='replay.bag' \
            "${target}:/home/${MINI_PC_USER}/TRON1_maps/mapping/" "$library/" || \
            echo '[MINI PC] 결과 복사 실패. 연결이 좋을 때 다시 실행 후 종료하면 복사합니다.' >&2
    }
    tunnel_pid=0
    trap remote_cleanup EXIT
    # Screen only: forward the loopback console; reconnect whenever the link returns.
    ( while true; do
        ssh "${ssh_opts[@]}" -o ExitOnForwardFailure=yes -N -L "127.0.0.1:${PORT}:127.0.0.1:${PORT}" "$target" 2>/dev/null
        sleep 2
      done ) &
    tunnel_pid=$!
    if [[ "${MAPPING_OPEN_BROWSER:-0}" == 1 ]]; then
        for _ in $(seq 1 120); do
            if curl -s -m 2 -o /dev/null "http://127.0.0.1:${PORT}/api/state"; then xdg-open "http://127.0.0.1:${PORT}/" >/dev/null 2>&1 || true; break; fi
            sleep .5
        done
    fi
    printf '지도 관리 UI: http://127.0.0.1:%s/ (미니 PC 화면을 연결해 보여 줌)\n물리 조종기로 이동하세요. 끝낼 때는 화면에서 작성 종료 후 이 창에서 Ctrl+C.\n' "$PORT"
    wait "$tunnel_pid"
    exit 0
fi
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
