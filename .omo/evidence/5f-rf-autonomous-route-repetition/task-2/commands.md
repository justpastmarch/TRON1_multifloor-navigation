# Command ledger

Commands are listed from the two stated working directories. `source` prefixes are shown where ROS Python message types were required.

## Prerequisite and baseline

```bash
git status --short
uv --version
rosbag info --yaml "manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid"
UV_CACHE_DIR=/tmp/offline-lio-uv uv run --with numpy --python /usr/bin/python3 tools/offline_lio/test_registration_core.py
bash -n tools/offline_lio/run_fast_lio_replay.sh
bash -n tools/offline_lio/replay_rviz.sh
bash -n tools/offline_lio/run_all.sh
sha256sum tools/offline_lio/process_bag.py tools/offline_lio/build_comparison_bag.py tools/offline_lio/registration_core.py tools/offline_lio/comparison_trajectory.py tools/offline_lio/traversal_events.py tools/offline_lio/event_camera.py tools/offline_lio/comparison_markers.py tools/offline_lio/run_all.sh tools/offline_lio/run_fast_lio_replay.sh tools/offline_lio/replay_rviz.sh tools/offline_lio/README.md
sha256sum "manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid" "manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.INCOMPLETE.json"
```

## Existing registration and extraction

```bash
source /opt/ros/noetic/setup.bash && source "$HOME/catkin_ws/devel/setup.bash" && UV_CACHE_DIR=/tmp/offline-lio-uv uv run --with numpy --with pyarrow --python /usr/bin/python3 tools/offline_lio/process_bag.py build/offline_lio/5f-rf-task-2/lidar_spec.json
source /opt/ros/noetic/setup.bash && source "$HOME/catkin_ws/devel/setup.bash" && UV_CACHE_DIR=/tmp/offline-lio-uv uv run --python /usr/bin/python3 .omo/evidence/5f-rf-autonomous-route-repetition/task-2/extract_route_evidence.py "manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid" "/home/m3tron/Desktop/TRON1_Modular_Navigation/build/offline_lio/5f-rf-task-2/lidar_registration.csv" ".omo/evidence/5f-rf-autonomous-route-repetition/task-2"
UV_CACHE_DIR=/tmp/offline-lio-uv uv run --python /usr/bin/python3 .omo/evidence/5f-rf-autonomous-route-repetition/task-2/build_visuals.py .omo/evidence/5f-rf-autonomous-route-repetition/task-2
UV_CACHE_DIR=/tmp/offline-lio-uv uv run --python /usr/bin/python3 .omo/evidence/5f-rf-autonomous-route-repetition/task-2/inspect_package.py .omo/evidence/5f-rf-autonomous-route-repetition/task-2 "manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid" d28f40a9a4c0c30906ff835b643f4893f4f652642a0c97d27157a0d0c7651a02
rosbag info --yaml "/home/m3tron/Desktop/TRON1_Modular_Navigation/build/offline_lio/5f-rf-task-2/lidar_registration.bag"
```

The extractor was retried after two environment-compatibility failures were corrected: Python 3.8 lacks `match/case` and PEP-723 isolation initially lacked PyYAML/ROS bag dependencies. The visual builder was retried with the ROS-compatible Python 3.8 constraint after Python 3.14 could not use the pinned PyArrow wheel. The first inspector run exposed integer parsing of the sibling CSV's `accepted` field; the predicate was corrected to compare with `1` before the passing runs.

## Adversarial and verification

```bash
source /opt/ros/noetic/setup.bash && source "$HOME/catkin_ws/devel/setup.bash" && UV_CACHE_DIR=/tmp/offline-lio-uv uv run --with numpy --with pyarrow --python /usr/bin/python3 tools/offline_lio/process_bag.py "/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/.omo/evidence/5f-rf-autonomous-route-repetition/task-2/adversarial_fixtures/malformed_lidar_spec.json"
timeout --signal=INT --kill-after=5s 1s env UV_CACHE_DIR=/tmp/offline-lio-uv uv run --with numpy --with pyarrow --python /usr/bin/python3 tools/offline_lio/process_bag.py "/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/.omo/evidence/5f-rf-autonomous-route-repetition/task-2/adversarial_fixtures/interrupted_lidar_spec.json"
timeout --signal=TERM --kill-after=5s 1s env UV_CACHE_DIR=/tmp/offline-lio-uv uv run --with numpy --with pyarrow --python /usr/bin/python3 tools/offline_lio/process_bag.py "/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/.omo/evidence/5f-rf-autonomous-route-repetition/task-2/adversarial_fixtures/interrupted_lidar_spec.json"
pgrep -af "tools/offline_lio/process_bag.py"
UV_CACHE_DIR=/tmp/offline-lio-uv uv run --python /usr/bin/python3 .omo/evidence/5f-rf-autonomous-route-repetition/task-2/inspect_package.py /tmp/task2-stale "manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid" d28f40a9a4c0c30906ff835b643f4893f4f652642a0c97d27157a0d0c7651a02
uv run "/home/m3tron/.cache/opencode/packages/oh-my-openagent@latest/node_modules/oh-my-openagent/dist/skills/programming/scripts/python/check-no-excuse-rules.py" .omo/evidence/5f-rf-autonomous-route-repetition/task-2/extract_route_evidence.py .omo/evidence/5f-rf-autonomous-route-repetition/task-2/build_visuals.py .omo/evidence/5f-rf-autonomous-route-repetition/task-2/inspect_package.py
uv run --with ruff ruff check .omo/evidence/5f-rf-autonomous-route-repetition/task-2/extract_route_evidence.py .omo/evidence/5f-rf-autonomous-route-repetition/task-2/build_visuals.py .omo/evidence/5f-rf-autonomous-route-repetition/task-2/inspect_package.py
jq empty .omo/evidence/5f-rf-autonomous-route-repetition/task-2/manifest.json
UV_CACHE_DIR=/tmp/offline-lio-uv uv run --with numpy python -c 'import hashlib,json,pathlib; root=pathlib.Path(".omo/evidence/5f-rf-autonomous-route-repetition/task-2"); m=json.loads((root/"manifest.json").read_text()); bad=[]; [(bad.append(n) if hashlib.sha256((root/n).read_bytes()).hexdigest()!=h else None) for n,h in m["core_output_sha256"].items()]; print({"checked":len(m["core_output_sha256"]),"mismatches":bad}); raise SystemExit(bool(bad))'
pgrep -af "roscore|rosmaster|roslaunch|rosbag play|tools/offline_lio/process_bag.py|run_fast_lio_replay"
UV_CACHE_DIR=/tmp/offline-lio-uv uv run --with numpy python -c 'import json,pathlib; root=pathlib.Path(".omo/evidence/5f-rf-autonomous-route-repetition/task-2"); names=("manifest.json","DoneClaim.json","bag_metadata.json","inspection_result.json"); [json.loads((root/name).read_text()) for name in names]; print({"valid_json":list(names)})'
rm -rf "/tmp/offline-lio-uv"
test ! -e /tmp/task2-interrupt && test ! -e /tmp/task2-stale && test ! -e /tmp/offline-lio-uv
sha256sum "manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid" "manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.INCOMPLETE.json"
pgrep -af "tools/offline_lio/process_bag.py|run_fast_lio_replay|rosbag play|rviz"
```

`jq` was unavailable (`command not found`), so JSON parsing and all 15 manifest checksum comparisons were performed by the following `uv` command; it reported zero mismatches. The process inventory showed only pre-existing ROS masters, which Task 2 did not start or stop.

Temporary setup/cleanup commands were `mkdir -p`, `ln -s`, `cp`, `touch -d @0`, `rm -rf /tmp/task2-stale`, and `rmdir /tmp/task2-interrupt`. All cleanup completed.
