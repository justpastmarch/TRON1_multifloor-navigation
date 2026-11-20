#!/usr/bin/env bash
# Build the 3D map beside a mini PC raw LiDAR BAG; copy back only the result.
# Usage: remote_mapping_cloud.sh build NAME TRAJECTORY_JSON PARAMS_JSON LOCAL_OUTPUT_DIR
set -Eeuo pipefail
[[ $# == 5 && "$1" == build && "$2" =~ ^[A-Za-z0-9_]+$ ]] || { echo 'Usage: remote_mapping_cloud.sh build NAME TRAJECTORY PARAMS OUTPUT' >&2; exit 2; }
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${root}/config.env"
target="${MINI_PC_USER}@${MINI_PC_HOST}"
ssh_opts=(-o BatchMode=yes -o ConnectTimeout=10)
builder="${root}/src/mission_manager/src/mission_manager/mapping_cloud.py"
raw="/home/${MINI_PC_USER}/.local/share/tron1-stair-supervisor/bags/$2.bag"
remote="$(ssh "${ssh_opts[@]}" "$target" "test -f '$raw' && test ! -e '$raw.active' && mktemp -d /tmp/tron1-cloud.XXXXXX")" || {
    echo "MiniPC raw BAG is missing or still recording: $raw" >&2; exit 1; }
cleanup() { ssh "${ssh_opts[@]}" "$target" "rm -rf '$remote'" >/dev/null 2>&1 || true; }
trap cleanup EXIT
scp -q "${ssh_opts[@]}" "$builder" "$3" "$4" "${target}:${remote}/"
ssh "${ssh_opts[@]}" "$target" "set +u; source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; \
    nice -n 10 python3 -u '$remote/mapping_cloud.py' '$raw' '$remote/$(basename "$3")' '$remote/$(basename "$4")' '$remote/out'"
mkdir -p "$5"
scp -q "${ssh_opts[@]}" "${target}:${remote}/out/cloud.npz" "${target}:${remote}/out/result.json" "$5/"
