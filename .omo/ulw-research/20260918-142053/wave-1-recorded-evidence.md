# Wave 1: Recorded Evidence

Worker: `bg_a5f9a7f8`

## Findings

- Seventeen stair bags are readable, with mixed IMU, wheel odometry, LiDAR, scan, camera, TF, and tag coverage.
- The main 5F/RF artifact is a readable but incomplete 508.439 s bag with IMU about 188.8 Hz, wheel odometry about 93.9 Hz, LiDAR about 5.3 Hz, RGB about 20.5 Hz, and scan about 9.7 Hz.
- That artifact lacks tag, mission feedback/result, and floor-state evidence.
- Existing extracts record 232 gaps over 0.5 s and a simultaneous multi-sensor outage around 232.026 to 236.569 s.
- Relative LiDAR registration accepted 839 of 1,345 processed frames, about 62.4 percent, and is diagnostic rather than map truth.
- Existing data has no independent stair-relative pose, surveyed correspondence, or ground truth sufficient to identify metric lateral drift, heading drift, wheel slip, or IMU bias.
- All current stair-state replay outcomes are FAULTED or INCOMPLETE; they demonstrate rejection behavior, not successful physical control.

## Primary artifacts

- `stair_captures/`
- `manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid`
- `.omo/evidence/5f-rf-autonomous-route-repetition/task-2/bag_metadata.json`
- `.omo/evidence/5f-rf-autonomous-route-repetition/task-2/extraction_summary.json`
- `.omo/evidence/5f-rf-autonomous-route-repetition/task-2/inspection_result.json`
- `.omo/evidence/5f-rf-autonomous-route-repetition/task-2/normalized_wheel_path.csv`
- `.omo/evidence/5f-rf-autonomous-route-repetition/task-2/lidar_relative_path.csv`

## Expansion leads

- Stationary IMU calibration captures.
- Surveyed stair-relative landmarks and synchronized reference pose.
- Separate UP/DOWN captures with command and mission state topics.
- Controller comparison metrics and repeated-run evidence.
