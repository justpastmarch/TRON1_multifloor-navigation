#!/usr/bin/env bash
# Run a script with the frozen numerical environment plus ROS distro libraries.
# Append system paths after the numerical environment: never shadow its NumPy.
set -euo pipefail
: "${STAIR_NUMERICAL_PYTHON:?Set the existing Python with the pinned LiDAR numerical dependencies}"
[[ -x "$STAIR_NUMERICAL_PYTHON" && $# -ge 1 ]] || { echo 'Expected executable numerical Python and script path' >&2; exit 2; }
exec "$STAIR_NUMERICAL_PYTHON" -c '
import json, os, runpy, subprocess, sys
system_paths = json.loads(subprocess.check_output(["/usr/bin/python3", "-c", "import sys,json;print(json.dumps(sys.path))"], text=True))
sys.path.extend(p for p in system_paths if p and p not in sys.path)
sys.path.extend(p for p in os.environ.get("STAIR_SENSOR_PYTHONPATH", "").split(os.pathsep) if p and p not in sys.path)
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name="__main__")
' "$@"
