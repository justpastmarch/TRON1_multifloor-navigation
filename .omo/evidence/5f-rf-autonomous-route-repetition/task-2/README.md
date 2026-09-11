# Task 2 no-motion route-survey evidence

This package is diagnostic evidence for a later stationary site survey. It does not replay Joy, publish motion, activate production YAML, claim localization truth, or bridge missing data.

## Reproduce

From `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`:

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
UV_CACHE_DIR=/tmp/offline-lio-uv uv run --python /usr/bin/python3 \
  .omo/evidence/5f-rf-autonomous-route-repetition/task-2/extract_route_evidence.py \
  manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid \
  /home/m3tron/Desktop/TRON1_Modular_Navigation/build/offline_lio/5f-rf-task-2/lidar_registration.csv \
  .omo/evidence/5f-rf-autonomous-route-repetition/task-2
UV_CACHE_DIR=/tmp/offline-lio-uv uv run --python /usr/bin/python3 \
  .omo/evidence/5f-rf-autonomous-route-repetition/task-2/inspect_package.py \
  .omo/evidence/5f-rf-autonomous-route-repetition/task-2 \
  manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid \
  d28f40a9a4c0c30906ff835b643f4893f4f652642a0c97d27157a0d0c7651a02
```

The inspector must report `status: pass`, `motion_interfaces_invoked: false`, the exact common outage overlap, and the rejected wheel edge. Review `rgb_route_contact_sheet.jpg`, `route_paths_with_rejections.png`, `semantic_segments.csv`, and `map_alignment_block.md` manually.

## Provenance vocabulary

- **measured**: copied directly from bag records, message fields, file metadata, or checksums.
- **derived**: deterministic calculation from measured values, including normalization, thresholds, masks, and registration output.
- **inferred**: human visual interpretation, never an exact floor boundary or localization authority.
- **unavailable**: required evidence was not observed; the dependent output remains blank/blocked.

`controller_intent_samples.csv` is explicitly controller intent only. It is not a transmitted command record. Wheel odometry and the sibling LiDAR registration are relative diagnostic lanes. FAST-LIO output is unavailable for this bag and is not substituted or treated as truth.
