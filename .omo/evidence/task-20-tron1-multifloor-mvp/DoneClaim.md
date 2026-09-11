# Todo 20 DoneClaim: local software PASS, physical mission BLOCKED

Date: 2026-08-19

## Verdict

- **Local fixture `Mission.action`: PASS.** The operational `mission_manager` node was driven through `/mission` on localhost with process-owned fixture child action servers. The inspect goal completed with action status `SUCCEEDED (3)`, result `OK (0)`, every segment class, coherent indices, four generation changes, a finalized fixture scan artifact, and a freshly planned return route.
- **Real 3F office -> roof scan -> home: BLOCKED.** Todos 17-19 have no physical evidence, all nine production profile groups remain disabled and empty, and only the unvalidated 3F map pair exists. No production launch, SSH, remote ROS master, cloud, camera/tag, robot WebSocket, or hardware motion was attempted.
- Todo 20 remains unchecked because its acceptance criteria require actual graph/site/hardware evidence. Fixture success is not promoted to physical success.

## Exact localhost commands and results

Baseline gates:

```text
$ python3 validate_bundle.py --site-config-root test/fixtures/building_valid
SITE_CONFIG_OK floors=3 locations=7 edges=9
BUNDLE_OK

$ ./run.sh --check
SITE_CONFIG_OK floors=3 locations=7 edges=9
BUNDLE_OK
[CHECK] bundle, fixture, packages, and local software: OK
```

Manual action run at `ROS_MASTER_URI=http://127.0.0.1:11311`, with `ROS_HOSTNAME` and `ROS_IP` both `127.0.0.1`:

```text
$ roslaunch mission_manager manual_mission.test
$ rostopic echo /multifloor/floor_state
$ rosrun mission_manager manual_mission_client.py _artifact_output:=/tmp/opencode/todo20-manual-action.json
[ROSUNIT] ... test_outbound_scan_and_fresh_return ... ok
RESULT: SUCCESS; TESTS: 1; ERRORS: 0; FAILURES: 0
```

The exact submitted action goal was:

```json
{"destination_id":"roof_scan","mission_type":"inspect","return_after_task":true}
```

The terminal result was:

```json
{"terminal_status":3,"result_code":0,"mission_id":"mission_fe0e74aaedb246a192f0c5a98b76fd6c","fresh_return_proved":true}
```

## Manual action feedback and fresh return

The feedback stream contained all 14 executed segments with contiguous indices `0..13`:

```text
outbound count=8
  0 NAVIGATION       3F -> stair_a_entry_3f
  1 STAIR            3F -> stair_a_landing_4f
  2 FLOOR_TRANSITION 3F -> stair_a_landing_4f
  3 NAVIGATION       4F -> stair_b_entry_4f
  4 STAIR            4F -> roof_landing
  5 FLOOR_TRANSITION 4F -> roof_landing
  6 NAVIGATION       RF -> roof_scan
  7 SCAN             RF -> roof_scan
fresh return expands total count to 14
  8 NAVIGATION       RF -> roof_landing
  9 STAIR            RF -> return_4f
 10 FLOOR_TRANSITION RF -> return_4f
 11 NAVIGATION       4F -> stair_a_landing_4f
 12 STAIR            4F -> home_3f
 13 FLOOR_TRANSITION 4F -> home_3f
```

The outbound count changing from 8 to 14 only after SCAN, plus the return-only `return_4f` target, proves the home route was a new planner output rather than a reversed/cached outbound list.

Concurrent `/multifloor/floor_state` output remained `READY (2)` and showed strictly increasing generations:

```text
3F generation=1 -> 4F generation=2 -> RF generation=3
-> 4F generation=4 -> 3F generation=5
```

Each later NAV feedback appeared on the transitioned floor, so the action did not advance navigation before the fixture's matching READY generation was observed.

## Scan artifact

The full action finalized this test-only artifact before returning home:

```text
/tmp/tron1-manual_mission_m3tron_All_Series_361079_2870513680714391359/artifacts/scans/mission_fe0e74aaedb246a192f0c5a98b76fd6c_roof_scan_1787148876389065265.bag
size: 11 bytes
sha256: 2cb9ea9b24556fe79c39a36dc65ac5de4831fbf077c3ec6d47c98b0045301c87
```

The fixture rosbag adapter reported duration `0.2` and one message for every mandatory fixture topic: `/scan/points`, RGB image, depth image, `/imu/data`, `/tf`, and `/localization/pose`. This file is deliberately a fixture artifact (`fixture bag`), not a physical sensor bag.

The real rosbag adapter was separately exercised on localhost:

```text
$ rostest mission_manager scan_recorder_ros.test --text
test_six_required_topic_types_form_valid_managed_bag ... ok
RESULT: SUCCESS; TESTS: 1; ERRORS: 0; FAILURES: 0
```

That test recorded and reopened actual ROS message types for all six topics and removed its temporary bag. It validates the software adapter, not production sensor data.

## Controlled failure and interruption matrix

```text
$ rostest mission_manager mission_system.test --text
Ran 15 tests in 3.012s; OK
RESULT: SUCCESS; TESTS: 15; ERRORS: 0; FAILURES: 0
```

- Navigation: a manual `/mission/goal` with scenario `nav_failure` returned action `ABORTED (4)`, result `NAVIGATION_FAILED (4)`, reason `move_base navigation failed`, and no artifact. The matrix also proved exactly one retry/clear and no third attempt.
- Stair: `stair_failure` returned `STAIR_FAILED (5)` and `floor_count == 0`, proving no localization/next segment ran.
- Localization: `localization_failure` returned `LOCALIZATION_FAILED (6)` with `nav_count == 1`, proving no post-transition NAV ran.
- Scan: `scan_failure` returned `SCAN_FAILED (7)`, kept `nav_count == 3`, and emitted no `return_4f` feedback, proving return planning/execution never started.
- Cancellation/resume: an active NAV was cancelled to `PREEMPTED`; the replacement mission planned from the confirmed home anchor and succeeded.
- Repeated interruption: three independently cancelled NAV missions all reached `PREEMPTED`; a fourth normal mission then succeeded without stale cancellation state.
- Misleading success: a floor child reporting action `SUCCEEDED` with a failed result was rejected as `MISSION_ABORT (9)` with no later NAV.
- Generated/dirty artifact: omission of one mandatory topic returned `SCAN_FAILED (7)` and did not run the return route; early rosbag exit behaved the same way. No `.active` file was accepted.
- Stale state: stopped state publication returned `MISSION_ABORT (9)` before any child NAV (`nav_count == 0`).
- BUSY/malformed: concurrent BUSY and malformed/unknown goals were rejected without preempting or dispatching child work.
- Communication loss: stair transport loss returned `MISSION_ABORT (9)` before any floor transition.
- Dirty workspace: this directory has no Git metadata. No product/config file was edited, production flags stayed false, and unrelated pre-existing `/tmp/opencode` items were left untouched.

The fixture graph has no robot/WebSocket command owner, so it cannot substantiate a physical zero-velocity claim. Fail-closed evidence here is action status plus child counters/no-unintended-next-segment; the real zero-command gate remains part of blocked Todo 19.

## Physical gate prerequisites

Before the requested real mission can be attempted, all of the following evidence is required:

1. Provision and site-approve all nine production YAML groups (`locations`, graph, scan profiles, floors, stairs, tags, transitions, stair profiles, robot); only then change their explicit `configured` gates.
2. Survey the real 3F office/home, 4F/5F/RF landings, roof scan pose, directed stair edges, landing hypotheses/covariances, and writable scan output. Production locations, graph, floors, tags, and stair profiles are currently empty.
3. Add and fingerprint actual 4F, 5F, and RF map YAML/image pairs; only `floor_3F.yaml` and `.pgm` currently exist and are not site-validated evidence.
4. Complete Todo 17 with live MID-360 PointCloud2 -> LaserScan -> AMCL/move_base evidence: one `/scan` publisher, TF/timestamps/rates, covariance, costmaps, reachable flat-floor goal, parameter snapshot, and sensor bag.
5. Complete Todo 18 with installed tag IDs/sizes and live camera detections, M-of-N/freshness/occlusion/wrong-floor probes, stored landing pose causality, map fingerprints, AMCL readiness, costmap identity, and every directed transition.
6. Complete Todo 19 with the approved TRON1 model/firmware/robot profile, mini-PC and robot reachability/NTP evidence, sole WebSocket ownership, zero barriers, watchdog/e-stop/disconnect behavior, and supervised ascent/descent artifacts (raw transport, IMU, odom, tags, action feedback, and video/logs).
7. Re-run production preflight only after those local profile gates pass, then execute the physical mission under site safety controls and inspect the real sensor bag, action/topic status, command transcript, and final home pose.

`python3 validate_bundle.py --validate-production-config` currently exits nonzero with:

```text
BUNDLE_ERROR: locations.yaml:configured: must be true before use
```

## Cleanup receipt

- All locally started masters, launches, action servers, topic echoes, test clients, and rosbag processes terminated.
- All Todo 20 fixture bags, `.invalid` bags, scan directories, and `/tmp/opencode/todo20-*` logs/JSON were removed after this report captured their results.
- No `.active` files remained.
- No SSH command, production launch, remote connection, or hardware command was issued.
- Final process and temporary-artifact audits are recorded in the session/ledger entry.
