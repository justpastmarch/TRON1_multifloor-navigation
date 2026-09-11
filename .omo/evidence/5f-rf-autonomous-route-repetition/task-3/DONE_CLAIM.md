# Task 3 DoneClaim

## Claim

Task 3 is complete at the software/configuration boundary. One stair now owns exactly two floor-keyed endpoint records. Route planning and floor-transition execution select the endpoint for the destination floor in both directions. Malformed endpoint, graph, tag, location, covariance, pose, and profile references fail closed. No runtime node or generated ROS interface was added or changed, and production configuration remains disabled pending surveyed values.

This workspace has no Git metadata, so the handoff is identified by files rather than a commit. Implementation files are `multifloor_manager/{configuration.py,transition_configuration.py,ros_node.py}`, `mission_manager/{site_config.py,route_planner.py}`, `src/multifloor_manager/CMakeLists.txt`, the two asymmetric fixture roots under `test/fixtures/`, and the disabled production `building_graph.yaml`/`stairs.yaml`. Coverage is in `test/{test_site_configuration.py,test_stair_endpoint_configuration.py,test_launch_contract.py}`, `mission_manager/test/test_route_planner.py`, and `multifloor_manager/test/test_floor_transition_delayed_pose_ros.py`.

## Acceptance mapping

- `StairEndpoint` owns `entry_location_id`, `target_landing`, a 36-value covariance, and expected landing tag IDs.
- `Stair.endpoints` is immutable and must be keyed by exactly `from_floor` and `to_floor`; the unreleased one-endpoint shape is rejected rather than shimmed.
- Site validation binds endpoint locations and tags to the same floor/stair, validates both direction-specific profiles, rejects cross-floor `NAV`, and requires each stair edge to start at that floor's endpoint entry.
- The asymmetric fixture plans both directions as `STAIR` followed by `FLOOR_TRANSITION` and selects the target-floor endpoint.
- `MultifloorManagerNode` arms target-floor tag evidence and publishes the target-floor landing pose/covariance.
- Production YAML contains no fabricated endpoint values and stays `configured: false`.

## Verification evidence

### Characterization before migration

```text
python3 -m unittest test.test_site_configuration.SiteConfigurationTest.test_current_stair_uses_one_endpoint_for_entry_landing_and_tags -v
Ran 1 test ... OK
```

The characterization passed before the schema migration and was then replaced by bidirectional endpoint tests.

### Focused unit tests

```text
python3 -m unittest test.test_site_configuration test.test_stair_endpoint_configuration src.mission_manager.test.test_route_planner -v
Ran 45 tests
OK
```

The final root run also passed every Task 3 test, including all 12 endpoint rejection/selection cases. `test_stair_endpoint_configuration.py` is the malformed-input matrix: endpoint keys/count, entry-floor binding, quaternion, tag existence/ownership, covariance length/content, cross-floor `NAV`, stair-edge origin, old schema, and direction-profile references. Its three unrelated failures are listed under `Known unrelated workspace failures`.

### Validator and route manual QA

```text
python3 validate_bundle.py --site-config-root test/fixtures/building_valid
SITE_CONFIG_OK floors=3 locations=7 edges=9
BUNDLE_OK

UP [('STAIR', '4F', 'cautious_up'), ('FLOOR_TRANSITION', '4F', None)]
DOWN [('STAIR', '3F', 'cautious_down'), ('FLOOR_TRANSITION', '3F', None)]
```

Malformed probes failed closed:

```text
missing endpoint, repeated attempts: exit 1
BUNDLE_ERROR: stairs.yaml:stairs[0].endpoints: keys must be ['3F', '4F']; got ['4F']

cross-floor NAV: exit 1
BUNDLE_ERROR: building_graph.yaml:edges[0]: NAV endpoints must share a floor

production validation (intentional while production is disabled): exit 1
BUNDLE_ERROR: locations.yaml:configured: must be true before use
```

The operator-facing aggregate check passed after the implementation:

```text
./run.sh --check
SITE_CONFIG_OK floors=3 locations=7 edges=9
BUNDLE_OK
[CHECK] bundle, fixture, packages, and local software: OK
```

### Build and registered tests

```text
catkin_make
exit 0

source devel/setup.bash && catkin_make run_tests -j1
245/246 registered tests passed

catkin_test_results --verbose build/test_results
Summary: 246 tests, 0 errors, 1 failures, 0 skipped
```

All Task 3 configuration and planner tests passed. `test_bidirectional_stair_expansion_targets_each_floor_endpoint` verifies both route directions and target endpoint IDs. `test_pre_response_pose_committing_after_response_cannot_advance_handshake` exercises `_run_transaction`, verifies the 4F target endpoint pose `(1.0, 2.0)` is published, and completes the transition. The sole registered failure is an independent storage-capacity gate documented below.

### Runtime failure classification

The failing `mission_system.test` case was reproduced in isolation through the repository's loopback wrapper. Temporary instrumentation captured the action result and was removed afterward:

```text
state=4 (ABORTED)
result_code=7 (SCAN_FAILED)
reason=scan recording failed: insufficient free space: /tmp/.../artifacts/scans/...bag
nav_count=3
```

This is outside Task 3's endpoint path: the route completed its three outbound navigation actions before the scan free-space check aborted the mission. The other 14 mission-system cases passed. No claim is made that this failure predates Task 3 because the workspace has no baseline revision metadata.

### Diagnostics and interface receipts

LSP diagnostics reported no issues in all 10 touched Python source/test files.

Current generated-interface source hashes:

```text
83157e5987e17a6e405c00890d4a25a297f7e180676b63aeeae8113371bc3ac8  src/mission_manager/action/Mission.action
4fb6c09c1656c393be39ddf306c9435814c015d90326d9e024d582acc10ac720  src/multifloor_manager/action/FloorTransition.action
54d7c397307e5d1f2fb6ef77c9938c4974ada3c6c6f71c31ed79939a54057e8e  src/stair_supervisor/action/StairTraversal.action
a21ba5f760a2317340d494b90e0e85dbd9c68579a82399cafd0c939f613e47f2  src/multifloor_manager/msg/FloorState.msg
0ce89dae8fdae7d08ab30c31b019ce300b87e0c84205cd8ae52b8f232d0845cf  src/stair_supervisor/msg/SupervisorState.msg
```

No action or message definition was edited during this task. With no Git baseline, the hashes are current receipts rather than before/after proof. `./run.sh --check` also confirmed the existing three-package/node inventory.

## Known unrelated workspace failures

The final root discovery ran 123 tests: all Task 3 tests passed, while three existing contract tests failed outside this task's files and behavior:

1. `test_mapping_notebook_owns_slam_and_refuses_localization_conflicts` expects literal `_base_frame:=base_Link`; the notebook constructs the equivalent argument as `f"_base_frame:={BASE_FRAME}"`.
2. `test_one_runtime_entrypoint_exists_when_each_package_is_inspected` sees `stair_supervisor/scripts/calibrate_yaw_rate.py` in addition to the node entrypoint.
3. The same test sees `multifloor_manager/scripts/rviz_tf_reconnect.py` in addition to the node entrypoint.

These files were not modified during Task 3. The more specific runtime inventory check in `./run.sh --check` passes; unrelated notebook/utility work was left intact. With no Git metadata, this claim is based on the task's edit inventory rather than revision history.

## Static quality note

The repository's no-excuse scanner reported Python-3.8-incompatible `slots=True` suggestions for dataclasses that already declare explicit `__slots__`, plus the pre-existing 296-pure-LOC `multifloor_manager/ros_node.py`. The new `StairEndpoint` follows the repository's Python 3.8-compatible explicit-slot pattern. No unrelated runtime refactor was introduced solely to silence those false-positive/inherited findings.

## Scope boundary

This DoneClaim is for Task 3 software/configuration behavior only. It does not claim physical robot, stair, localization, or Mission success; those remain later hardware gates in the plan.
