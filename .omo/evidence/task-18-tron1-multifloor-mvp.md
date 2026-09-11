# Todo 18 site-transition gate: BLOCKED

Date: 2026-08-19

## Safe checks

- `python3 validate_bundle.py --site-config-root test/fixtures/building_valid`
  - PASS: `SITE_CONFIG_OK floors=3 locations=7 edges=9`, `BUNDLE_OK`
- `./run.sh --check`
  - PASS: `[CHECK] bundle, fixture, packages, and local software: OK`
- `python3 validate_bundle.py --validate-production-config`
  - Expected gate failure: `BUNDLE_ERROR: locations.yaml:configured: must be true before use`

No `--preflight`, production launch, SSH, remote ROS-master query, camera access, or robot contact was attempted.

## Local fixture transition evidence

- `catkin_make run_tests_multifloor_manager`
- `catkin_test_results --verbose build/test_results/multifloor_manager`
  - PASS: 37 tests, 0 errors, 0 failures, 0 skipped.

The suite exercised target-map fingerprint/generation, stored-pose handshake, three post-pose readiness samples, covariance/TF/scan/odometry freshness, costmap identity, expected/wrong/stale/replayed tag evidence, wrong/dirty/late maps, cancellation, delayed callbacks, and epoch isolation.

## Missing evidence blocking the real-site gate

- All nine production YAML groups are `configured: false`; production floors, locations, graph, stairs, tag IDs/sizes, transition policies, landing poses/covariances, scan profiles, stair profiles, and robot profile are not approved for use.
- Production map assets contain only `floor_3F.yaml` and `floor_3F.pgm`; actual 4F, 5F, and RF map YAML/image pairs are absent, and the disabled 3F asset is not site-validated evidence.
- Todo 17 (live MID-360 PointCloud2 to LaserScan to AMCL/move_base flat-floor gate) remains incomplete.
- There is no live ROS-master evidence for the configured sensor/tag topics, TF tree, `/initialpose` to AMCL causality, three stationary causal AMCL updates, covariance, scan, costmap readiness, or map generation.
- There is no physical camera/AprilTag evidence for stationary and approach-motion M-of-N/freshness confirmation, occlusion, wrong-floor tags, or every directed 3F/4F/5F/RF transition.

Fixture success is not promoted to a production PASS. Todo 18 remains unchecked.

## Cleanup

- No rosbag files were created.
- No task-18 files were created under `/tmp/opencode`.
- Final process checks found no `roscore`, `rosmaster`, `roslaunch`, `rostest`, `rosbag`, `catkin_make`, or `ssh` process.
- Existing `/tmp/tron1_rviz_navigation.lock` and prior task-16 artifacts were not created or modified by this run.
