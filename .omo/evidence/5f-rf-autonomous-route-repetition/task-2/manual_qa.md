# Manual QA record

## No-motion package inspection

Command: see `commands.md`, final `inspect_package.py` invocation.

Result: **PASS**. The source checksum matched; every detected gap is marked rejected; no zero fill or interpolation is declared; no wheel or LiDAR path edge crosses its rejection boundary; Joy has only `controller_intent_only` provenance; no motion interface was invoked.

Generated ROS bag inspection: `lidar_registration.bag` is indexed/readable with 4,171 messages: 1,345 odometry, 1,345 path, 1,345 cloud, 135 local-map, and one global-map message. Its ROS timestamps are LiDAR sensor-header times and begin 2.793928 s before source bag record time; CSV exports retain bag record time separately.

## Visual inspection

- `route_paths_with_rejections.png`: PASS for rejection visibility. Wheel segments restart independently and the 236.838 s jump is not drawn. LiDAR shows only accepted, valid-edge fragments. Neither panel is a calibrated or ground-truth frame.
- `rgb_route_contact_sheet.jpg`: coarse visual classification only. Indoor 5F evidence is visible at 0.001-24.934 s and 399.983-494.403 s; stairs at 49.973-175.006 s; rooftop at 200.007-300.011 s. Exact transition times are not visible and remain bracketed in `semantic_boundary_brackets.csv`.
- Outage frames 216.000, 225.000, and 236.840 s show near-static terrace content; 239.000 s changes abruptly to a railing/city view. They do not provide a surveyed floor/stair correspondence.

## Map/waypoint review

Result: **BLOCKED, correctly fail-closed**. Both map images and metadata are present, but there are zero reviewed surveyed correspondence pairs per floor. The required minimum is two. No transform, residual, overlay, or coordinate proposal is emitted; all nine waypoint rows have blank `x_m`, `y_m`, and `yaw_rad`.

## Downstream disposition

Task 5 may consume only `waypoint_proposals.csv` after its blocked rows are replaced by reviewed stationary survey results. Tasks 8-10 may use route shape only as diagnostic resemblance, never acceptance truth.
