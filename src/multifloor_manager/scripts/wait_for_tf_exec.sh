#!/usr/bin/env bash
set -eu

if [[ "$#" -lt 4 || "$3" != "--" ]]; then
  printf 'usage: %s TARGET_FRAME SOURCE_FRAME -- COMMAND [ARGS...]\n' "$0" >&2
  exit 64
fi

target_frame=$1
source_frame=$2
shift 3
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

printf '[WAIT] waiting for TF %s -> %s before starting %s\n' \
  "$target_frame" "$source_frame" "$1" >&2

while true; do
  if timeout --foreground --kill-after=1s 8s \
      rostopic echo /tf 2>&1 \
      | grep -q "frame_id: \"$target_frame\""; then
    printf '[READY] TF %s -> %s\n' "$target_frame" "$source_frame" >&2
    if ! python3 "$script_dir/rviz_tf_reconnect.py"; then
      printf '[WARN] RViz TF resync skipped; navigation will continue\n' >&2
    fi
    exec "$@"
  fi
  sleep 1
done
