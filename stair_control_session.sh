#!/usr/bin/env bash
# Select one explicit control configuration; reuse run.sh and its command owner.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 || $# -gt 2 || ( "$1" != check && "$1" != start ) ]]; then
    printf '%s\n' 'Usage: ./stair_control_session.sh check|start [configuration.yaml]'
    exit 2
fi
# Use the same installed settings as run.sh, including the numerical runner.
# shellcheck disable=SC1091
source "$ROOT/config.env"
: "${STAIR_PYTHON:?Set the existing ROS + LiDAR Python interpreter explicitly}"
[[ -x "$STAIR_PYTHON" ]] || { printf 'Python is not executable: %s\n' "$STAIR_PYTHON" >&2; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [[ -r "$ROOT/devel/setup.bash" ]]; then source "$ROOT/devel/setup.bash"; fi
export STAIR_LIDAR_CONFIG
STAIR_LIDAR_CONFIG="$(realpath "${2:-$STAIR_LIDAR_CONFIG}")"
export STAIR_LIDAR_MODE=control STAIR_LIDAR_OBSERVE_ONLY=false STAIR_RECORD=1 STAIR_PYTHON
"$STAIR_PYTHON" "$ROOT/src/stair_supervisor/scripts/check_lidar_config.py" "$STAIR_LIDAR_CONFIG"
if [[ "$1" == start ]]; then exec "$ROOT/run.sh"; fi
