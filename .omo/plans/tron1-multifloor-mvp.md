# tron1-multifloor-mvp - Work Plan

## TL;DR (For humans)
<!-- Fill this LAST, after the detailed plan below is written, so it summarizes the REAL plan. -->
<!-- Plain English for a non-engineer: NO file paths, NO todo numbers, NO wave/agent/tool names. -->

**What you'll get:** A ROS1 Noetic multi-floor mission system in which a user selects a named destination and mission, the robot navigates each floor with AMCL and move_base, crosses stairs through a bounded supervisor, switches maps and re-localizes at each exit, records a scan mission, and plans a fresh route home.

**Why this approach:** It preserves the proven 2D navigation stack and adds only three custom nodes. The stair supervisor becomes the sole robot-command connection, eliminating the existing fourth bridge node and command collisions without adding a mux or arbiter framework.

**What it will NOT do:** It will not use FAST-LIO as navigation localization, maintain AMCL through stairs, add full 3D navigation, or introduce a mux, safety node, localization manager, sensor-fusion layer, map proxy, pose graph, or transition-scoring framework. It will not invent missing floor maps, tag placement, or stair motion profiles.

**Effort:** XL
**Risk:** High - software interfaces are standard, but physical ascent/descent behavior and site data must be proven on the deployed TRON1 and building.
**Decisions to sanity-check:** Exactly three catkin packages/nodes; custom actions for mission, floor transition, and stair traversal; stair_supervisor as sole WebSocket owner; covariance-plus-freshness AMCL readiness; fresh graph planning for return.

Your next move: execute this plan in a separate worker session, or request the optional dual high-accuracy review first. Full execution detail follows below.

---

> TL;DR (machine): XL/high-risk ROS1 Noetic implementation: three custom nodes, standard navigation/perception reuse, multi-floor actions and YAML DBs, deterministic mission/return planning, mock and hardware-gated verification.

## Scope
### Must have
- Convert the current flat portable bundle into a catkin workspace while keeping root `run.sh` as the operator wrapper.
- Create exactly three project packages and exactly one runtime node per package: `mission_manager`, `multifloor_manager`, and `stair_supervisor`.
- Keep the current deployment split for MVP: mini PC remains ROS master and sensor source; the workstation runs map/navigation/perception adapters, the three custom nodes, RViz, and optional UI client. Make all source topics/remaps configurable so standard perception nodes can later move to the mini PC without code changes.
- Reuse installed `map_server`, `amcl`, `move_base`, `pointcloud_to_laserscan`, `apriltag_ros`, Livox, RealSense, actionlib, TF2, and rosbag processes.
- Use `Mission.action`, `FloorTransition.action`, and `StairTraversal.action`; their definitions live in their owning package and no fourth interface package is created.
- Use `move_base` actionlib for navigation, `nav_msgs/LoadMap` for `/change_map`, `/initialpose` plus `/request_nomotion_update` for AMCL initialization, and `/move_base/clear_costmaps` after localization.
- Remap move_base output to `/navigation/cmd_vel`; only `stair_supervisor` may subscribe and send commands to the TRON1 WebSocket.
- Implement deterministic directed building-graph routing, named locations, directed stair transitions, expected AprilTag sets, stored landing hypotheses with covariance, transition predicates, scan profiles, fresh return planning, and mission-level results.
- Preserve clipping, `STAND -> WALK`, watchdog zero, repeated shutdown zero, response correlation, and WebSocket timeout behavior from `cmd_vel_bridge.py` and `tron1_robot_client.py`.
- Keep physical site data separate from automated test fixtures; hardware launch fails closed when configured floor maps, PointCloud2 source, tag IDs/sizes, or enabled stair profiles are missing.

### Must NOT have (guardrails, anti-slop, scope boundaries)
- No fourth project-owned ROS executable, helper node, custom bringup/interface package, standalone `cmd_vel_bridge`, or Python helper that calls `rospy.init_node` outside the three node entry points.
- No `twist_mux`, separate command arbiter, safety monitor node, localization manager, map proxy, sensor-fusion node, transition-confidence score, pose graph, full 3D navigation, or FAST-LIO navigation dependency.
- No low-level joint, gait, footstep, or balance controller; stair motion is limited to documented high-level mode and bounded twist requests.
- No AMCL continuity requirement while a stair action is active and no navigation goal while floor state is not `READY`.
- No automatic retry of stairs, map switching, tag confirmation, or localization. Navigation may retry once after cancel, terminal status, and costmap clear.
- No fabricated production maps, coordinates, camera topics, tag sizes, or stair profiles. Explicit test fixtures must be named and stored under `test/fixtures/`.
- No direct user/RViz publication to managed `/initialpose` or `/move_base_simple/goal` in production launch.

## Verification strategy
> Zero human intervention - all verification is agent-executed.
- Test decision: Hybrid TDD. Use Python `unittest`/`rostest` available on ROS Noetic: test-first for pure route/FSM/config/predicate/readiness logic; tests-after for action/service wiring, mocked WebSocket, rosbag lifecycle, and launch integration. Physical robot/site gates are separately recorded manual experiments and are never represented as automated software success.
- Required automated commands: `catkin_make`, `catkin_make run_tests`, `catkin_test_results --verbose`, `./run.sh --check`, package/node inventory inspection, and targeted rostests with mocked action/service/WebSocket peers.
- Required observable evidence: action goals/feedback/results, floor generation and map fingerprint, AMCL covariance/freshness samples, TF checks at scan stamps, WebSocket JSON transcript, rosbag metadata, runtime node/topic publisher counts, and logs for injected failures.
- Hardware release gates: flat-floor robot command test; MID-360 PointCloud2 to LaserScan to AMCL test; map switch/re-localization test; AprilTag motion test; move_base/stair handoff test; guarded ascent and descent tests; full mission/scan/fresh-return test. A failed hardware gate blocks only dependent physical stages, not pure-software completion.
- Evidence: <attemptDir>/task-<N>-tron1-multifloor-mvp.<ext> (attemptDir = currentAttemptDir from 'omo ulw-loop status --json', .omo/evidence/ulw/<session>/<goalId>/a<attempt>; outside ulw-loop use .omo/evidence/)

## Execution strategy
### Parallel execution waves
> Target 5-8 todos per wave. Fewer than 3 (except the final) means you under-split.
- Wave 1, contracts and workspace: Todos 1-5 in sequence where noted; lock package boundaries, interfaces, schemas, fixtures, and standard-node launch contracts.
- Wave 2, test-first core logic: Todos 6-10 may run in parallel after their listed prerequisites.
- Wave 3, ROS node integration: Todos 11-15 integrate the three runtime nodes, launch/validation, and operator surfaces.
- Wave 4, system and physical gates: Todos 16-20 execute automated end-to-end QA first, then progressively gated sensor, transition, stair, and mission tests.

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| 1 | none | 2, 3, 4, 5 | none |
| 2 | 1 | 6-15 | 3, 4 |
| 3 | 1 | 6, 7, 8, 10, 12, 14 | 2, 4 |
| 4 | 1 | 6-16 | 2, 3, 5 |
| 5 | 1 | 8, 12, 15, 17 | 4 |
| 6 | 2, 3, 4 | 14 | 7-10 |
| 7 | 2, 3, 4 | 14 | 6, 8-10 |
| 8 | 2, 3, 4, 5 | 12 | 6, 7, 9, 10 |
| 9 | 1, 2, 4 | 11 | 6-8, 10 |
| 10 | 2, 3, 4 | 14 | 6-9 |
| 11 | 2, 4, 9 | 14-16, 19 | 12, 13 |
| 12 | 2, 3, 4, 5, 8 | 14-16, 18 | 11, 13 |
| 13 | 2, 3, 4 | 14-16 | 11, 12 |
| 14 | 6, 7, 10-13 | 15, 16, 20 | none |
| 15 | 5, 11-14 | 16-20 | none |
| 16 | 11-15 | 17-20 | none |
| 17 | 5, 15, 16 | 18-20 | none |
| 18 | 12, 16, 17 | 19, 20 | none |
| 19 | 11, 16-18 | 20 | none |
| 20 | 14, 16-19 | final wave | none |

## Todos
> Implementation + Test = ONE todo. Never separate.
<!-- APPEND TASK BATCHES BELOW THIS LINE WITH edit/apply_patch - never rewrite the headers above. -->
- [x] 1. Convert the flat bundle into a three-package catkin workspace without losing the existing operator wrapper
  What to do / Must NOT do: Create workspace `src/CMakeLists.txt` and exactly `src/mission_manager`, `src/multifloor_manager`, `src/stair_supervisor`, each with `package.xml`, `CMakeLists.txt`, `setup.py`, Python package directory, `scripts/<node>.py`, `launch/`, `config/`, and `test/` only where owned. Move reusable navigation/maps/RViz assets into the owning packages, retain root `run.sh`, `README.md`, and compatibility `config.env`, and make `run.sh` source `/opt/ros/noetic/setup.bash` plus workspace `devel/setup.bash`. Do not create a fourth package or node.
  Parallelization: Wave 1 | Blocked by: none | Blocks: 2-5
  References (executor has NO interview context - be exhaustive): `run.sh:4-58`, `run.sh:101-194`, `validate_bundle.py:14-50`, `launch/navigation.launch:1-69`; catkin Python package guidance `https://wiki.ros.org/catkin/Tutorials/create_a_workspace`.
  Acceptance criteria (agent-executable): `catkin_make` exits 0; `rospack find mission_manager`, `multifloor_manager`, and `stair_supervisor` resolve inside this workspace; a workspace package listing contains exactly those three project packages; each node script imports but does not yet need full behavior.
  QA scenarios (name the exact tool + invocation): happy: run `catkin_make && rospack find ...` and save paths; failure: add a temporary fourth-package fixture outside the source tree to the validator input and assert rejection without changing production files. Evidence `<attemptDir>/task-1-tron1-multifloor-mvp.txt`.
  Commit: Y | `build(workspace): establish three-package catkin layout`

- [x] 2. Define and generate the complete ROS interface contract in the owning packages
  What to do / Must NOT do: Add `mission_manager/action/Mission.action`, `multifloor_manager/action/FloorTransition.action`, `multifloor_manager/msg/FloorState.msg`, `stair_supervisor/action/StairTraversal.action`, and `stair_supervisor/msg/SupervisorState.msg`. `Mission` goal fields: `destination_id`, `mission_type`, `return_after_task`; feedback: `mission_id`, `state`, `current_floor`, `segment_type`, `segment_index`, `segment_count`, `target_id`; result: `result_code`, `reason`, `mission_id`, `artifact_path`. `FloorTransition` goal: `transition_id`, `target_floor`; feedback: `phase`, `detail`; result: `result_code`, `floor_id`, `map_generation`, `reason`. `FloorState`: header, constants `UNKNOWN/TRANSITIONING/READY/FAULT`, `floor_id`, `map_generation`, `state`, `detail`. `StairTraversal` goal: `stair_id`, constants `UP/DOWN`, `direction`; feedback: `phase`, `detail`; result: `result_code`, `reason`. `SupervisorState`: header, constants `DISARMED/NAV/STAIR/FAULT`, `state`, `connected`, `detail`, `ownership_epoch`. Define `OK`, `BUSY`, `INVALID_GOAL`, `CAPABILITY_DISABLED`, `NAVIGATION_FAILED`, `STAIR_FAILED`, `LOCALIZATION_FAILED`, `SCAN_FAILED`, `COMMUNICATION_LOST`, `MISSION_ABORT` result constants in the owning actions. Use standard action PREEMPTED/ABORTED/SUCCEEDED semantics; do not add an interface package.
  Parallelization: Wave 1 | Blocked by: 1 | Blocks: 6-15
  References (executor has NO interview context - be exhaustive): approved `.omo/drafts/tron1-multifloor-mvp.md`; current simple goal limitation in official move_base source `https://github.com/ros-planning/navigation/blob/noetic-devel/move_base/src/move_base.cpp`; `cmd_vel_bridge.py:211-220`.
  Acceptance criteria (agent-executable): generated Python action/message modules import after `catkin_make`; `rosmsg show` and `rosmsg md5` succeed for all five interfaces; an interface contract test asserts every field and constant exactly.
  QA scenarios (name the exact tool + invocation): happy: run `rosmsg show`/Python imports; failure: feed an invalid mission type and assert `INVALID_GOAL` without starting a child action. Evidence `<attemptDir>/task-2-tron1-multifloor-mvp.txt`.
  Commit: Y | `feat(interfaces): define mission floor and stair contracts`

- [x] 3. Implement strict YAML schemas, typed loaders, and cross-reference validation for all site and policy data
  What to do / Must NOT do: Add `mission_manager/config/building_graph.yaml`, `locations.yaml`, `scan_profiles.yaml`; `multifloor_manager/config/floors.yaml`, `stairs.yaml`, `apriltags.yaml`, `transitions.yaml`; `stair_supervisor/config/stair_profiles.yaml`, `robot.yaml`. Lock schema: floor IDs map to YAML map path and frame `map`; locations contain unique ID, floor, type, x/y/yaw; directed graph edges are `NAV` or `STAIR_UP/STAIR_DOWN`; stairs contain from/to floor, entry location, target landing pose, 6x6 covariance diagonal, expected tag set, up/down profile IDs; tags contain family, physical size, semantic role, floor/stair; transition policy contains enabled/required booleans, temporal vote, freshness, dwell, and timeout; scan profile contains mandatory topic list, duration, output root, free-space minimum; robot config separates move_base limits from normalized WebSocket full-scale calibration. Use typed immutable dataclasses and parse-don't-validate at startup. No production placeholder may pass validation.
  Parallelization: Wave 1 | Blocked by: 1 | Blocks: 6-8, 10, 12, 14
  References (executor has NO interview context - be exhaustive): `config.env:2-21`, `maps/floor_3F.yaml:1-9`, `nav/base_local_planner_params.yaml:1-20`, `validate_bundle.py:77-95`, user-provided YAML examples in the approved draft.
  Acceptance criteria (agent-executable): test-first loader suite accepts a complete asymmetric test building and rejects duplicate IDs, missing map images, dangling graph endpoints, floor mismatch, invalid quaternion/yaw, nonpositive tag size, absent expected tags, incomplete covariance, missing ascent/descent profile, and nonexistent scan topic lists.
  QA scenarios (name the exact tool + invocation): happy: load `test/fixtures/building_valid/`; failure: parameterized invalid fixtures each emit a path-qualified deterministic error and nonzero validator exit. Evidence `<attemptDir>/task-3-tron1-multifloor-mvp.txt`.
  Commit: Y | `feat(config): add typed building and transition databases`

- [x] 4. Establish reusable automated test fixtures and non-node mock peers
  What to do / Must NOT do: Add pure Python fixtures for an asymmetric 3F→4F→roof graph, tiny generated occupancy maps, tag detections, AMCL poses/covariances, TF timestamps, and WebSocket transcripts. Add rostest-only mock action/service/WebSocket servers as test processes; they may be executable under tests but must not be installed or counted as deployed project nodes. Provide a deterministic fake clock seam for pure logic and a temporary rosbag output directory. Do not add production fallback nodes.
  Parallelization: Wave 1 | Blocked by: 1 | Blocks: 6-16
  References (executor has NO interview context - be exhaustive): no current tests; `validate_bundle.py:37-50`; `tron1_robot_client.py:43-111`; QA matrix in `.omo/drafts/tron1-multifloor-mvp.md` findings.
  Acceptance criteria (agent-executable): `catkin_make run_tests` executes at least one test target per package; test-only processes are absent from install rules and production launch; fixtures use obvious non-production IDs and maps.
  QA scenarios (name the exact tool + invocation): happy: run test discovery and inspect JUnit results; failure: run deployment inventory against test helpers and confirm they are ignored as non-installed fixtures rather than accepted runtime nodes. Evidence `<attemptDir>/task-4-tron1-multifloor-mvp.txt`.
  Commit: Y | `test(fixtures): add deterministic ROS and robot mocks`

- [x] 5. Build the standard navigation and perception launch contract around configurable source topics
  What to do / Must NOT do: Move `navigation.launch` and nav YAML to `multifloor_manager`; retain `map_server`, `amcl`, and `move_base`; set AMCL `use_map_topic=true`, `first_map_only=false`, correct `max_beams` to `laser_max_beams`, and remap move_base `cmd_vel` to `/navigation/cmd_vel`. Add standard `pointcloud_to_laserscan` launch using configurable `cloud_in`, `target_frame=base_Link`, height/angle/range limits, and output `/scan`. Add standard `apriltag_ros` launch with configurable rectified image/camera-info remaps and generated tag IDs/sizes config. Require exactly one `/scan` publisher and do not consume FAST-LIO output. Preserve planner tuning except removing parameters proven ignored; keep RViz visualization topics.
  Parallelization: Wave 1 | Blocked by: 1 | Blocks: 8, 12, 15, 17
  References (executor has NO interview context - be exhaustive): `launch/navigation.launch:13-68`, `nav/*.yaml`, `rviz/wf_navigation.rviz:46-271`, `logs/20260819_132949/navigation.log:1-9`; `https://github.com/ros-perception/pointcloud_to_laserscan/tree/ros1`; `https://github.com/AprilRobotics/apriltag_ros`.
  Acceptance criteria (agent-executable): `roslaunch --nodes` reports only the expected standard nodes plus later includes; launch XML validates; tests assert `/navigation/cmd_vel` remap, AMCL dynamic-map parameters, exact source-topic parameters, and no FAST-LIO reference.
  QA scenarios (name the exact tool + invocation): happy: launch against fixture PointCloud2/camera publishers and observe one `/scan` publisher plus tag detections; failure: wrong cloud type/frame or missing camera info keeps readiness false and produces a clear preflight error. Evidence `<attemptDir>/task-5-tron1-multifloor-mvp.txt`.
  Commit: Y | `feat(bringup): configure standard navigation and perception nodes`

- [x] 6. Implement deterministic BuildingPlanner and fresh return routing test-first inside mission_manager
  What to do / Must NOT do: Implement a pure directed-graph BFS with stable edge-ID ordering. Expand a route into generic `NAVIGATION`, `STAIR`, `FLOOR_TRANSITION`, and `SCAN` segments; validate segment floor continuity and commit the logical anchor only after success. Resolve named destination/home from Location DB. When return is requested, run a new BFS from confirmed current anchor to `home_location_id`; never reverse or reuse outbound segments. Reject routes that require disabled or directionally absent stairs.
  Parallelization: Wave 2 | Blocked by: 2, 3, 4 | Blocks: 14
  References (executor has NO interview context - be exhaustive): user route examples and explicit fresh-home requirement; no existing planner code; action contracts from Todo 2 and YAML schema from Todo 3.
  Acceptance criteria (agent-executable): tests cover same-floor, multi-floor, unreachable, disabled stair, tie ordering, invalid continuity, and asymmetric return where outbound A→B but return B→C→A; outputs are byte-for-byte deterministic.
  QA scenarios (name the exact tool + invocation): happy: plan 3F office→roof scan→fresh home; failure: remove one descending edge and assert return planning fails before motion with `NAVIGATION_FAILED`/route reason. Evidence `<attemptDir>/task-6-tron1-multifloor-mvp.txt`.
  Commit: Y | `feat(mission): add deterministic building route planner`

- [x] 7. Implement the generic Mission FSM and Boolean TransitionManager test-first
  What to do / Must NOT do: Implement `WAIT_GOAL`, `PLAN_MISSION`, `EXECUTE_SEGMENT`, `NEXT_SEGMENT`, `MISSION_COMPLETE`, `MISSION_ABORT`; segment handlers are `NAVIGATION`, `STAIR`, `FLOOR_TRANSITION`, `SCAN`. Keep sensor logic out of the FSM. TransitionManager consumes named booleans such as `T_GOAL_REACHED`, `T_STAIR_EXIT`, `T_FLOOR_CONFIRMED`, `T_LOCALIZED`; enabled/required/optional-count/dwell rules come from YAML and produce one Boolean plus an evidence dictionary for debug logs. Do not implement continuous scoring or one state per floor/service call.
  Parallelization: Wave 2 | Blocked by: 2, 3, 4 | Blocks: 14
  References (executor has NO interview context - be exhaustive): user sections 16-20; approved draft decisions; no current mission FSM.
  Acceptance criteria (agent-executable): transition-table tests cover every state/event pair, illegal events, cancellation, timeout, retry result, optional predicates, dwell reset, and evidence logging; no floor-specific state name exists.
  QA scenarios (name the exact tool + invocation): happy: replay a complete route event sequence; failure: stale/false required predicate prevents transition and records each evidence Boolean without advancing. Evidence `<attemptDir>/task-7-tron1-multifloor-mvp.txt`.
  Commit: Y | `feat(mission): add generic FSM and boolean transitions`

- [x] 8. Implement floor, map, and AprilTag evidence models inside multifloor_manager
  What to do / Must NOT do: Parse `AprilTagDetectionArray`, accept only expected IDs for the active transition, require configured fresh detections and deterministic temporal voting, and expose typed Boolean predicate/evidence values for the later multifloor node to publish under `/multifloor/debug/<predicate>`; actual ROS publishers and latched `FloorState` belong to Todo 12. Implement target map fingerprint `(frame_id,width,height,resolution,origin,data hash,map_load_time)` and monotonic `map_generation`. Default policy: three consecutive fresh expected detections within one second, message age ≤0.5s, ten-second timeout; an equally valid wrong-floor expected-set vote aborts. Never infer floor solely from IMU/z integration or tag pose magnitude.
  Parallelization: Wave 2 | Blocked by: 2-5 | Blocks: 12
  References (executor has NO interview context - be exhaustive): `rviz/wf_navigation.rviz:215-250`; apriltag contract `https://github.com/AprilRobotics/apriltag_ros/tree/master/apriltag_ros/msg`; official map_server source `https://github.com/ros-planning/navigation/blob/noetic-devel/map_server/src/main.cpp`.
  Acceptance criteria (agent-executable): tests reject unknown, stale, intermittent, wrong-route, and wrong-floor tags; map tests distinguish same metadata/different data and ignore all pre-armed generations.
  QA scenarios (name the exact tool + invocation): happy: expected tag vote and new target fingerprint become true; failure: replay a stale pre-transition tag/map and assert both remain false. Evidence `<attemptDir>/task-8-tron1-multifloor-mvp.txt`.
  Commit: Y | `feat(multifloor): model tag votes and map generations`

- [x] 9. Migrate and harden the TRON1 WebSocket transport as stair_supervisor library code
  What to do / Must NOT do: Move `DirectRobotClient` and bridge conversion logic into `stair_supervisor/src/stair_supervisor/` without a ROS node entry point. Preserve GUID correlation, `request_twist`, STAND/WALK requests, normalized clipping, timeouts, ≥30Hz stream, watchdog, and repeated zero on close. Add typed high-level requests for `request_stair_mode`, `request_emgy_stop`, odometry, and IMU only where documented; verify a post-request mode/status message rather than accepting any correlated success. Separate move_base velocity limits from robot full-scale calibration. Do not implement low-level SDK control or auto-reconnect/resume after connection loss.
  Parallelization: Wave 2 | Blocked by: 1, 2, 4 | Blocks: 11
  References (executor has NO interview context - be exhaustive): `cmd_vel_bridge.py:32-220`, `tron1_robot_client.py:15-117`, `config.env:17-21`, LIMX high-level guide `https://www.limxdynamics.com/en/documents/799664773997400064`.
  Acceptance criteria (agent-executable): mocked WebSocket tests prove protocol envelope, mode/status verification, clipping, nonfinite-to-zero, watchdog zero, close zeros, timeout cleanup, malformed response rejection, disconnect fault, and no silent reconnect.
  QA scenarios (name the exact tool + invocation): happy: STAND→WALK and bounded twist transcript; failure: wrong status, missing response, send failure, and disconnect each latch fault and never emit a later nonzero command. Evidence `<attemptDir>/task-9-tron1-multifloor-mvp.json`.
  Commit: Y | `refactor(stair): absorb robot transport into supervisor library`

- [x] 10. Implement a managed rosbag scan recorder test-first inside mission_manager
  What to do / Must NOT do: Implement a non-node `ScanRecorder` that prechecks profile, writable destination and configured minimum free space; creates `<mission_id>_<location_id>_<timestamp>.bag`; starts `rosbag record -O` with an explicit mandatory topic allow-list; records for `duration_sec`; stops with SIGINT; waits up to ten seconds; and validates `rosbag info --yaml` for nonzero duration and every mandatory topic/message count. Cancellation performs graceful finalization and returns PREEMPTED with artifact path; forced termination marks invalid. Do not use `-a`, shell interpolation, or a scan_manager node. Scan failure aborts and does not automatically start return.
  Parallelization: Wave 2 | Blocked by: 2-4 | Blocks: 14
  References (executor has NO interview context - be exhaustive): user Scan Mission requirements; ROS rosbag CLI `https://wiki.ros.org/rosbag/Commandline`; current project has no recorder.
  Acceptance criteria (agent-executable): tests with a temporary ROS master and fixture publishers produce a valid bag containing PointCloud2, RGB, depth, IMU, TF, and pose topics; missing topic, disk precheck, subprocess failure, cancellation, and SIGINT timeout produce deterministic result/error codes.
  QA scenarios (name the exact tool + invocation): happy: record and inspect a short fixture mission; failure: omit one mandatory topic and assert invalid artifact plus `SCAN_FAILED`. Evidence `<attemptDir>/task-10-tron1-multifloor-mvp.yaml`.
  Commit: Y | `feat(scan): manage deterministic rosbag mission capture`

- [x] 11. Implement stair_supervisor as the sole command owner and StairTraversal action server
  What to do / Must NOT do: Create the only `rospy.init_node("stair_supervisor")`. Subscribe `/navigation/cmd_vel`; own the only WebSocket; publish latched `SupervisorState`; serve `StairTraversalAction`. Ownership states: `DISARMED`, `NAV`, `STAIR`, `FAULT`. In NAV, forward only finite fresh commands newer than `ownership_epoch`; in STAIR, discard NAV input, emit zero barrier, enter verified stair mode, execute configured `VERIFY_ENTRY→ALIGN→FORWARD_SEGMENT_1→LANDING→TURN_TO_NEXT_FLIGHT→FORWARD_SEGMENT_2→EXIT_CONFIRM` profile using bounded high-level twists and configured Boolean evidence, emit zero barrier, exit stair mode to WALK, verify stationary, increment epoch, and return NAV. Disabled profiles return `CAPABILITY_DISABLED` before mode/nonzero commands. Communication loss latches FAULT and aborts; no automatic stair retry.
  Parallelization: Wave 3 | Blocked by: 2, 4, 9 | Blocks: 14-16, 19
  References (executor has NO interview context - be exhaustive): `cmd_vel_bridge.py:52-191`; user Stair Supervisor sections 5, 10-12, 21, 28-29; LIMX SDK guide; Todo 9 transport contract.
  Acceptance criteria (agent-executable): rostest transcript proves one WebSocket owner, zero barriers, no NAV command during STAIR, stale NAV rejection after handoff, correct FSM/profile order, timeout fault, disabled capability, cancellation at configured safe checkpoints, and zero on shutdown.
  QA scenarios (name the exact tool + invocation): happy: mocked up/down profile reaches NAV with new epoch; failure: tag timeout, status mismatch, WebSocket loss, or stale NAV command causes zero/FAULT/ABORTED and no resume. Evidence `<attemptDir>/task-11-tron1-multifloor-mvp.json`.
  Commit: Y | `feat(stair): add exclusive command and traversal supervisor`

- [x] 12. Implement multifloor_manager FloorTransition as one ordered, generation-safe action
  What to do / Must NOT do: Create the only `rospy.init_node("multifloor_manager")`. On accepted goal: require current state READY and matching directed stair; publish TRANSITIONING with blank current floor; arm target map fingerprint before service call; require expected AprilTag vote; call `/change_map` with resolved filesystem YAML path; require `RESULT_SUCCESS` and independently observe newer matching target map; publish stored landing `PoseWithCovarianceStamped` on managed `/initialpose`; call `/request_nomotion_update` for three distinct post-initialpose updates; require AMCL readiness conjunction: finite covariance diagonals x≤0.05m², y≤0.05m², yaw≤0.10rad², three post-initialpose samples, fresh `/scan`, stationary fresh odometry, valid `map→odom→base_Link` at scan stamps, and 30s timeout; call `/move_base/clear_costmaps`; require global costmap dimensions/origin/resolution match target; then publish READY with target floor and incremented generation. On any post-stair failure remain UNKNOWN/FAULT; do not roll back map or resume NAV.
  Parallelization: Wave 3 | Blocked by: 2-5, 8 | Blocks: 14-16, 18
  References (executor has NO interview context - be exhaustive): `launch/navigation.launch:18-43`, `maps/floor_3F.yaml:1-9`, `logs/20260819_132949/navigation.log:4-9`; official `map_server`, AMCL, and move_base Noetic sources cited above.
  Acceptance criteria (agent-executable): rostest proves publication arriving during `/change_map` is captured; stale map/pose/scan/TF is rejected; all ordered phases appear in feedback; success occurs only after matching costmap; each service timeout/result failure leads to ABORTED plus non-READY FloorState.
  QA scenarios (name the exact tool + invocation): happy: fixture 3F→4F transition increments generation and reaches READY; failure: wrong map hash or excessive covariance times out to FAULT without a navigation goal. Evidence `<attemptDir>/task-12-tron1-multifloor-mvp.yaml`.
  Commit: Y | `feat(multifloor): add transactional map switch and AMCL readiness`

- [x] 13. Implement the move_base navigation executor with explicit goal, cancel, retry, and readiness rules
  What to do / Must NOT do: Add a non-node `NavigationExecutor` inside mission_manager using `actionlib.SimpleActionClient` for `/move_base`; require `FloorState.READY`, matching floor/generation, and `SupervisorState.NAV`; send normalized map-frame goals from Location DB; correlate status and terminal result; on cancel wait PREEMPTED/RECALLED; on ABORTED/LOST permit exactly one retry after terminal status and `/move_base/clear_costmaps`; then fail. Before stair acquisition, cancel goal, await terminal status, and require zero/stationary evidence. Do not publish `/move_base_simple/goal` or consider goal publication success.
  Parallelization: Wave 3 | Blocked by: 2-4 | Blocks: 14-16
  References (executor has NO interview context - be exhaustive): `rviz/wf_navigation.rviz:207-213,270-271`; official move_base source/actionlib contracts; user sections 15, 21, 25, 27.
  Acceptance criteria (agent-executable): mocked action tests cover SUCCEEDED, rejected quaternion, ABORTED then one retry, LOST, cancellation, wrong floor generation, supervisor non-NAV, and zero handoff; no path can send a goal while floor is TRANSITIONING/FAULT.
  QA scenarios (name the exact tool + invocation): happy: named location reaches SUCCEEDED; failure: repeated ABORTED returns `NAVIGATION_FAILED` after one retry and leaves no active goal. Evidence `<attemptDir>/task-13-tron1-multifloor-mvp.txt`.
  Commit: Y | `feat(mission): manage move_base lifecycle and handoff`

- [x] 14. Implement mission_manager node, Mission action, generic segment orchestration, scan, and fresh return
  What to do / Must NOT do: Create the only `rospy.init_node("mission_manager")`; host `MissionActionServer`; reject invalid or BUSY goals; generate mission ID and outbound route; execute each NAV via NavigationExecutor, STAIR via StairTraversal action, FLOOR_TRANSITION via FloorTransition action, and SCAN via ScanRecorder; emit progress feedback and debug evidence; commit anchor only after successful segment. On `return_after_task=true`, plan a new route from confirmed current anchor to home after successful scan. Propagate SUCCESS/RETRY/FAILED mission-level semantics and precise reason codes. Cancellation immediately cancels safe child actions, finalizes active scan, and respects stair/floor action safe checkpoint semantics. Scan failure aborts without return. Critical diagnostics, communication loss, or child FAULT cause MISSION_ABORT and zero through supervisor behavior; do not add a safety node.
  Parallelization: Wave 3 | Blocked by: 6, 7, 10-13 | Blocks: 15, 16, 20
  References (executor has NO interview context - be exhaustive): user sections 15-17, 22, 24-28, 30; action interfaces Todo 2; pure logic Todos 6-7; current readiness text `run.sh:216-220`.
  Acceptance criteria (agent-executable): rostest executes same-floor, multi-floor, scan, fresh return, cancellation, BUSY, navigation retry, stair failure, localization failure, scan failure, and communication loss; feedback indices/state/floor are correct and terminal action status/result code agree.
  QA scenarios (name the exact tool + invocation): happy: asymmetric fixture completes outbound scan and independently planned return; failure: localization failure aborts before next NAV, and scan failure never launches return. Evidence `<attemptDir>/task-14-tron1-multifloor-mvp.yaml`.
  Commit: Y | `feat(mission): orchestrate complete multifloor mission lifecycle`

- [x] 15. Integrate one top-level launch, root preflight, validator, developer RViz, and operating documentation
  What to do / Must NOT do: Add `mission_manager/launch/system.launch` including navigation/perception and exactly the three custom nodes. Update `run.sh` to retain remote mini-PC/clock/route checks and SSH tunnel, source workspace, start one top-level roslaunch, and wait for three action servers, `FloorState.READY`, `SupervisorState.NAV`, fresh scan/odom/TF, configured tag topic, and one `/scan` publisher. Remove direct `cmd_vel_bridge.py` launch. Replace exact inventory validation with package/schema/map/action/launch/forbidden-node checks. Update RViz to visualization-only in managed launch, with no operational SetGoal/SetInitialPose bypass. Update README with UI action examples, config provisioning, scan artifacts, failure codes, and hardware gate commands. Do not add a UI node; UI communicates through `Mission.action`.
  Parallelization: Wave 3 | Blocked by: 5, 11-14 | Blocks: 16-20
  References (executor has NO interview context - be exhaustive): `run.sh:42-220`, `validate_bundle.py:14-145`, `rviz/wf_navigation.rviz:259-271`, `README.md`, all interfaces/configs from Todos 2-3.
  Acceptance criteria (agent-executable): `./run.sh --check` passes with valid fixture/deployment config and reports all required packages/topics/contracts; fails with a fourth custom node, standalone bridge, duplicate scan publisher, missing map/tag/profile/topic, or FAST-LIO navigation reference; `roslaunch --nodes` matches the expected graph.
  QA scenarios (name the exact tool + invocation): happy: launch mock system and submit a Mission action goal from CLI; failure: intentionally select an incomplete deployment profile and assert preflight stops before robot connection/motion. Evidence `<attemptDir>/task-15-tron1-multifloor-mvp.txt`.
  Commit: Y | `feat(bringup): integrate validated three-node system launch`

- [x] 16. Run the complete automated software acceptance suite and capture behavior-level evidence
  What to do / Must NOT do: Execute build, unit, rostest, validator, package/node count, action contract, mocked WebSocket, map generation, AMCL stale-data, route/return, rosbag, and end-to-end mission tests. Use the artifact through Mission.action, not direct class calls only. Confirm no installed fourth node/package and no prohibited dependency. Fix failures at their owning task rather than weakening tests.
  Parallelization: Wave 4 | Blocked by: 11-15 | Blocks: 17-20
  References (executor has NO interview context - be exhaustive): all previous tasks; current baseline `./run.sh --check`; final graph contract in Todo 15.
  Acceptance criteria (agent-executable): `catkin_make && catkin_make run_tests && catkin_test_results --verbose && ./run.sh --check` all exit 0; runtime mock mission completes scan and fresh return; all injected failures produce expected action status/result and zero command transcript.
  QA scenarios (name the exact tool + invocation): happy: run full mock mission via action CLI/client; failure: execute failure matrix for stale map, tag timeout, WebSocket loss, move_base abort, bag missing topic, and cancellation. Evidence `<attemptDir>/task-16-tron1-multifloor-mvp/`.
  Commit: N | verification-only

- [ ] 17. Validate MID-360 PointCloud2 to LaserScan to AMCL and move_base on the flat-floor robot
  What to do / Must NOT do: Confirm `livox_ros_driver2` produces a non-FAST-LIO `sensor_msgs/PointCloud2`; tune only pointcloud_to_laserscan height/range/angle parameters; verify one `/scan` publisher, frames/timestamps/rate, AMCL pose/covariance, TF freshness, costmaps, and a short move_base goal. Record a bag and parameter snapshot. If vendor PointCloud2 is unavailable, block the gate rather than add a custom converter.
  Parallelization: Wave 4 hardware gate | Blocked by: 5, 15, 16 | Blocks: 18-20
  References (executor has NO interview context - be exhaustive): user experiment 2; `nav/costmap_common_params.yaml:1-14`; `launch/navigation.launch:18-43`; Livox driver `https://github.com/Livox-SDK/livox_ros_driver2`.
  Acceptance criteria (agent-executable): `/scan` is stable at configured rate with `base_Link` transform; AMCL remains below configured covariance limits during representative flat-floor motion; move_base reaches one reachable goal; evidence bag contains cloud/scan/odom/TF/amcl/cmd_vel.
  QA scenarios (name the exact tool + invocation): happy: flat-floor goal succeeds; failure: stop PointCloud2 and verify move_base/stair supervisor emits/forwards zero and mission cannot continue. Evidence `<attemptDir>/task-17-tron1-multifloor-mvp/`.
  Commit: Y | `tune(perception): validate MID360 laser projection parameters`

- [ ] 18. Validate map switching, stored landing initialization, AMCL readiness, and AprilTag confirmation with real floor assets
  What to do / Must NOT do: Provision actual 3F/4F/5F/roof maps and verified map-relative locations, tag IDs/sizes, landing hypotheses/covariances; run repeated stationary and approach-motion tests for each directed transition; call FloorTransition and confirm map fingerprint, `/initialpose`, post-update AMCL covariance/TF/scan, matching costmap, and FloorState generation. Do not lower thresholds merely to pass without documenting evidence.
  Parallelization: Wave 4 site gate | Blocked by: 12, 16, 17 | Blocks: 19, 20
  References (executor has NO interview context - be exhaustive): user experiments 3-4; `maps/floor_3F.yaml`; Todo 12 transaction; AprilTag source/config links above.
  Acceptance criteria (agent-executable): each expected tag set meets configured M-of-N/freshness criteria under motion; each map transition reaches READY within timeout from stored hypothesis; wrong/stale/occluded tag and wrong map are rejected; no navigation goal is sent before READY.
  QA scenarios (name the exact tool + invocation): happy: repeated 3F→4F fixture/site transition succeeds; failure: present a wrong-floor tag and assert transition aborts to FAULT without rollback or motion. Evidence `<attemptDir>/task-18-tron1-multifloor-mvp/`.
  Commit: Y | `data(site): add validated floor maps tags and landing poses`

- [ ] 19. Validate move_base-to-stair ownership handoff and TRON1 ascent/descent profiles progressively
  What to do / Must NOT do: First run flat-floor STAND/WALK/twist/zero/watchdog/emergency-stop/connection-loss tests; verify `request_stair_mode` enter/exit response and status at zero velocity; then enable one prevalidated stair profile at a time and run entry alignment, ascent, landing turn, exit, and descent tests. Capture raw WebSocket, IMU, odom, AprilTag, action feedback, and video/log artifacts. Do not enable automatic retry or infer completion solely from elapsed time/mode response. The user's safe test environment removes the need for new software architecture, not the need to record pass/fail evidence.
  Parallelization: Wave 4 robot gate | Blocked by: 11, 16-18 | Blocks: 20
  References (executor has NO interview context - be exhaustive): user experiments 1 and 5; LIMX high-level SDK; Todos 9 and 11; existing bridge behavior `cmd_vel_bridge.py:138-191`.
  Acceptance criteria (agent-executable): only stair_supervisor sends commands; every handoff contains zero barriers; stale NAV never resumes; configured ascent and descent profiles each complete with expected exit evidence and return NAV, while tag timeout, WebSocket loss, emergency stop, and profile timeout produce zero/FAULT/ABORTED.
  QA scenarios (name the exact tool + invocation): happy: one ascent and one descent complete; failure: interrupt evidence/connection and verify fail-closed action result with no automatic restart. Evidence `<attemptDir>/task-19-tron1-multifloor-mvp/`.
  Commit: Y | `tune(stair): add robot-validated traversal profiles`

- [ ] 20. Execute and document the full destination, scan, and fresh-home-return mission
  What to do / Must NOT do: Through `Mission.action`, run the requested 3F office→roof scan→home mission using actual graph/site data. Observe every NAV, STAIR, FLOOR_TRANSITION, SCAN segment; verify progress indices, floor generations, map/AMCL barriers, scan artifact, and a newly planned return route. Repeat one controlled failure case for navigation, stair, localization, and scan. Do not declare complete from logs alone; compare actual action/status/topic/WebSocket/bag artifacts.
  Parallelization: Wave 4 final system gate | Blocked by: 14, 16-19 | Blocks: final verification wave
  References (executor has NO interview context - be exhaustive): full user example; Mission action Todo 2; orchestration Todo 14; all hardware/site gates.
  Acceptance criteria (agent-executable): final Mission action returns SUCCESS after reaching home; scan bag has all mandatory topics and nonzero data; return route is a fresh planner output; no direct RViz goal/initialpose or fourth custom node appears; failure runs return their specified codes and leave zero command/no active goal.
  QA scenarios (name the exact tool + invocation): happy: complete rooftop scan and fresh return; failure: inject one failure at each segment class and save terminal result, zero command, and no-unintended-next-segment evidence. Evidence `<attemptDir>/task-20-tron1-multifloor-mvp/`.
  Commit: Y | `docs(acceptance): record complete multifloor mission evidence`

## Final verification wave
> Runs in parallel after ALL todos. ALL must APPROVE. Surface results and wait for the user's explicit okay before declaring complete.
- [ ] F1. Plan compliance audit: independently map every Must have/Must NOT have and Todo acceptance criterion to current files and captured evidence; reject missing or self-reported-only claims. Evidence `<attemptDir>/final-F1-compliance.md`.
- [ ] F2. Code quality review: run Python/ROS diagnostics, inspect callback/thread/subprocess shutdown behavior, verify typed config boundaries and module size, and confirm no fourth node or hidden command source. Evidence `<attemptDir>/final-F2-quality.md`.
- [ ] F3. Real manual QA: drive the deployed surface through `Mission.action` for happy path, cancellation, one bad goal, one sensor/connection failure, scan artifact inspection, and fresh return; review RViz/topic/action/WebSocket evidence. Evidence `<attemptDir>/final-F3-manual-qa/`.
- [ ] F4. Scope fidelity: verify AMCL remains flat-floor localization, stairs intentionally break localization continuity, FAST-LIO is absent from navigation, standard ROS packages are reused, and forbidden frameworks/nodes were not added. Evidence `<attemptDir>/final-F4-scope.md`.

## Commit strategy
- Use one atomic commit per implementation todo marked `Commit: Y`; never commit evidence-only reruns separately.
- Preserve unrelated shared-worktree changes and stage only the files named by the active todo.
- Recommended sequence: workspace/interfaces/config/fixtures/bringup, then pure logic, then transport/three nodes, then system integration, then validated site/perception/stair data and acceptance docs.
- Do not commit production credentials, robot secrets, rosbag payloads, generated catkin build/devel directories, `.omo/evidence`, logs, or private camera imagery. Store large evidence outside Git and commit only metadata/checksums when required.

## Success criteria
- Exactly three deployed project-owned ROS nodes exist: `/mission_manager`, `/multifloor_manager`, `/stair_supervisor`; the standalone bridge is gone and only stair_supervisor owns the robot WebSocket.
- A named Mission action can perform same-floor and multi-floor navigation, bounded stair actions, generation-safe floor transitions, scan recording, and fresh graph-planned return with coherent feedback/result/cancellation.
- Each floor transition proves expected AprilTag evidence, target map fingerprint, stored-pose publication, post-generation AMCL covariance/scan/TF evidence, and target costmap readiness before NAV resumes.
- move_base and stair commands never overlap; every ownership change has zero barriers; stale commands cannot resume; timeout/disconnect/fault paths stop and abort without automatic motion restart.
- MID-360 navigation uses standard PointCloud2→pointcloud_to_laserscan→AMCL; FAST-LIO is not a navigation dependency.
- Scan output is gracefully finalized, contains all configured mandatory topics, and is validated before mission success.
- All automated build/tests/checks pass, required real site/robot gates have evidence, and the four final verification tasks approve.
- No prohibited fourth node, mux, safety node, localization manager, sensor-fusion node, score framework, map proxy, pose graph, full 3D navigation, or low-level gait/footstep controller is present.
