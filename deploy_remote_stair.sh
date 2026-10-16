#!/usr/bin/env bash
# Synchronize the three existing ROS packages into an isolated MiniPC overlay.
# The MiniPC's original catkin_ws and running sensor service are untouched.
set -Eeuo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${root}/config.env"
target="${MINI_PC_USER}@${MINI_PC_HOST}"
export RSYNC_RSH="ssh -o BatchMode=yes -o ConnectTimeout=5"
base="/home/${MINI_PC_USER}/.local/share/tron1-stair-supervisor"
workspace="${base}/ws"
selected="$(realpath "${STAIR_LIDAR_CONFIG}")"
expected="$(realpath "${root}/src/stair_supervisor/config/stair_lidar_3f_4f_test.yaml")"
if [[ "$selected" != "$expected" ]]; then
    echo 'MiniPC deployment requires the selected production LiDAR config in the stair_supervisor package.' >&2
    exit 1
fi
ssh -o BatchMode=yes "$target" \
    "if systemctl --user is-active --quiet tron1-stair-supervisor.service; then echo 'MiniPC Stair Supervisor is active; stop the current run before deployment.' >&2; exit 1; fi; mkdir -p '${workspace}/src'"
changes=""
for package in stair_supervisor multifloor_manager mission_manager; do
    output="$(rsync -ai --checksum --delete --exclude=test --exclude=__pycache__ --exclude='*.pyc' \
        "${root}/src/${package}/" "${target}:${workspace}/src/${package}/")"
    changes+="$output"
done
rsync -a "${root}/remote_stair_runtime.sh" "${target}:${base}/start.sh"
rsync -a "${root}/remote_stair_rawbag.sh" "${target}:${base}/record_raw.sh"
# Do not print the robot account ID; keep it only in a private remote env file.
tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT
chmod 600 "$tmp"
{
    printf 'ROS_MASTER_URI=%q\n' "http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}"
    printf 'ROS_IP=%q\n' "${MINI_PC_ROS_IP}"
    printf 'ROBOT_WS_URL=%q\n' "ws://${ROBOT_HOST}:${ROBOT_WS_PORT}"
    printf 'ACCID=%q\n' "${ACCID}"
    printf 'STAIR_LIDAR_MODE=%q\n' "${STAIR_LIDAR_MODE}"
    printf 'STAIR_LIDAR_OBSERVE_ONLY=%q\n' "${STAIR_LIDAR_OBSERVE_ONLY}"
    printf 'STAIR_NUMERICAL_PYTHON=%q\n' "${base}/venv/bin/python"
    printf 'STAIR_SENSOR_PYTHONPATH=%q\n' "${workspace}/devel/lib/python3/dist-packages:${MINI_PC_WORKSPACE}/devel/lib/python3/dist-packages"
} > "$tmp"
rsync -a --chmod=F600 "$tmp" "${target}:${base}/runtime.env"
ssh -o BatchMode=yes "$target" "chmod 700 '${base}/start.sh' '${base}/record_raw.sh'; chmod 600 '${base}/runtime.env'"
if [[ -n "$changes" ]] || ! ssh -o BatchMode=yes "$target" "test -r '${workspace}/devel/setup.bash'"; then
    echo '[DEPLOY] Building the isolated MiniPC ROS overlay (first run or source changed).'
    ssh -o BatchMode=yes "$target" \
        "set -e; source /opt/ros/noetic/setup.bash; source '${MINI_PC_WORKSPACE}/devel/setup.bash'; cd '${workspace}'; catkin_make -j2 -DCATKIN_ENABLE_TESTING=OFF >'${base}/build.log' 2>&1 || { tail -80 '${base}/build.log' >&2; exit 1; }"
fi
ssh -o BatchMode=yes "$target" \
    "test -x '${base}/venv/bin/python' && '${base}/venv/bin/python' -c 'import numpy,scipy,open3d,yaml'" || {
    echo 'MiniPC numerical runtime is missing; installing the pinned LiDAR dependencies.'
    ssh -o BatchMode=yes "$target" \
        "test -x '${base}/venv/bin/python' || python3 -m venv --system-site-packages '${base}/venv'; '${base}/venv/bin/python' -m pip install --no-input --disable-pip-version-check --upgrade 'pip<25.1' >'${base}/pip.log' 2>&1 && '${base}/venv/bin/python' -m pip install --no-input --disable-pip-version-check -r '${workspace}/src/stair_supervisor/requirements-lidar.txt' >>'${base}/pip.log' 2>&1 || { tail -60 '${base}/pip.log' >&2; exit 1; }"
}
ssh -o BatchMode=yes "$target" \
    "source /opt/ros/noetic/setup.bash; source '${MINI_PC_WORKSPACE}/devel/setup.bash'; source '${workspace}/devel/setup.bash'; roslaunch --files stair_supervisor remote_stair.launch accid:=offline_check websocket_url:=ws://127.0.0.1:1 >/dev/null; printf 'stair_action_md5='; rosmsg md5 stair_supervisor/StairTraversalActionGoal; printf 'floor_state_md5='; rosmsg md5 multifloor_manager/FloorState"
echo '[DEPLOY] MiniPC Stair Supervisor code, messages, launch, and numerical runtime are ready.'
