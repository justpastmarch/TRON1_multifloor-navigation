#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(dirname "${SCRIPT_DIR}")"
SOURCE_ROOT="$(dirname "${PACKAGE_ROOT}")"
WORKSPACE_ROOT="$(dirname "${SOURCE_ROOT}")"
if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/devel/setup.bash"
fi
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
exec /usr/bin/python3 "$@"
