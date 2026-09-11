---
slug: tron1-multifloor-mvp
status: complete
intent: clear
review_required: false
pending-action: choose execution or optional high-accuracy review for .omo/plans/tron1-multifloor-mvp.md
approach: Preserve the existing ROS1 navigation stack, add exactly three catkin Python nodes, absorb the current robot WebSocket bridge into one permitted node, and gate hardware-only stair behavior behind explicit experiments.
---

# Draft: tron1-multifloor-mvp

## Components (topology ledger)
<!-- Lock the SHAPE before depth. One row per top-level component that can succeed or fail independently. -->
<!-- id | outcome (one line) | status: active|deferred | evidence path -->
mission-manager | Generic mission FSM, building route planner, move_base action lifecycle, scan subprocess, UI action server | active | user brief; launch/navigation.launch:45-68
multifloor-manager | Floor DB, change_map transaction, exit-pose initialisation, AMCL readiness, AprilTag floor confirmation | active | launch/navigation.launch:13-43; ROS navigation noetic source
stair-supervisor | Exclusive high-level TRON1 motion gateway and stair FSM with fail-closed zero command | active | cmd_vel_bridge.py:32-220; tron1_robot_client.py:25-117; LIMX high-level SDK
configuration-data | Building graph, locations, stairs, tags, transitions, scan topics, thresholds, and per-floor maps | active | config.env:2-21; nav/*.yaml; maps/floor_3F.yaml
existing-ros-stack | map_server, AMCL, move_base, pointcloud_to_laserscan, apriltag_ros and sensor drivers remain third-party processes | active | launch/navigation.launch:13-68; rviz/wf_navigation.rviz:46-271
verification-deployment | Offline logic tests, mocked ROS/action/WebSocket tests, bag replay, and gated robot/stair trials | active | validate_bundle.py:14-145; run.sh:42-220; logs/20260819_132949/navigation.log

## Open assumptions (announced defaults)
<!-- Record any default you adopt instead of asking, so the user can veto it at the gate. -->
<!-- assumption | adopted default | rationale | reversible? -->
ROS packaging | exactly three catkin packages under workspace src/, while root run.sh remains the operator wrapper | user explicitly requested package structures and ROS1 Noetic deployment; no fourth bringup/interface package | reversible
message ownership | custom action/service/message definitions live in the package that owns the server, avoiding a fourth interface package | satisfies exactly-three-package/node boundary | reversible with migration
floor transition | use /change_map, wait for a new /map identity, publish /initialpose, request no-motion updates, require repeated covariance+TF+scan evidence, then clear costmaps | ROS1 exposes no atomic floor-switch barrier or AMCL converged boolean | reversible thresholds
scan mission | mission_manager owns a rosbag record subprocess, stops it with SIGINT, waits, and validates with rosbag info | no standard rosbag start/stop service and user explicitly permits process/service invocation | reversible
return planning | resolve home as a fresh planner query, never reverse outbound segments | explicit user requirement | reversible
FAST-LIO | excluded from navigation and all MVP success criteria; only a future scan/mapping extension point | explicit user requirement | reversible
stair delivery | implement both ascent and descent route types, but physical enablement remains blocked until the installed WF_TRON1A firmware and request_stair_mode behavior pass hardware gates | automatic return requires descent; public SDK does not prove operational stair semantics | safety gate

## Findings (cited - path:lines)
- Current project is a flat portable bundle, not a catkin package; there is no package.xml, CMakeLists.txt, setup.py, test tree, message, service, or action definition.
- run.sh:167-194 launches RViz, SSH tunnel, navigation, and the existing fourth custom node cmd_vel_bridge.py.
- validate_bundle.py:14-50 enforces an exact file inventory and validate_bundle.py:66-74 enforces only map_server, amcl, and move_base in navigation.launch; both must be redesigned for package growth.
- cmd_vel_bridge.py:52-191 already provides clipping, watchdog zero, STAND to WALK, and shutdown zero behavior that must not be lost.
- Official Noetic map_server source advertises change_map as nav_msgs/LoadMap and republishes latched map/map_metadata, but provides no AMCL or costmap barrier.
- Official Noetic AMCL source accepts initialpose, subscribes to map when use_map_topic=true, rebuilds map-dependent state on every map when first_map_only=false, and has no converged Boolean.
- Official move_base source provides the move_base action server, publishes cmd_vel, and exposes clear_costmaps as std_srvs/Empty; action terminal status must drive mission outcomes.
- Official LIMX TRON1 guide documents request_twist, stand, walk, stair mode, emergency stop, odometry, IMU, and recovery high-level APIs, but not a complete safe stair-mode operational contract for this exact robot/firmware.
- apriltag_ros provides AprilTagDetectionArray and camera-relative tag TF; accepted floor meaning and temporal voting are application policy.
- rosbag record has no standard start/stop service; graceful SIGINT and rosbag info verification are required for a trustworthy artifact.
- Existing logs/20260819_132949/navigation.log:4 shows transform timeouts, so TF freshness must be an acceptance criterion rather than assumed.

## Decisions (with rationale)
- Keep map_server, AMCL, move_base, pointcloud_to_laserscan, apriltag_ros, Livox and RealSense drivers as existing ROS processes; they do not count as custom nodes.
- Keep BuildingPlanner, MissionFSM, TransitionManager and transition predicates as ordinary Python modules inside mission_manager.
- Keep FLOOR_TRANSITION as one mission state with an internal ordered transaction rather than one FSM state per service call.
- Use move_base actionlib, not /move_base_simple/goal, for mission execution so cancel, feedback, correlation, and terminal result are observable.
- Treat stored exit pose as an initial hypothesis with configured covariance, never as authoritative localization.
- Require a single process to own the TRON1 WebSocket connection; the current standalone cmd_vel_bridge node cannot remain if exactly three custom nodes is strict.
- Absorb cmd_vel_bridge and DirectRobotClient behavior as non-node Python modules inside stair_supervisor. Remap move_base output to /navigation/cmd_vel; stair_supervisor forwards it only in NAV ownership and emits its internal fixed stair commands only in STAIR ownership. This preserves one WebSocket owner, watchdog, clipping, and shutdown zeros without a mux or fourth node.
- Expose the user mission contract as a custom Mission.action owned by mission_manager, with destination_id, mission_type, and return_after_task goal fields plus progress feedback, terminal result, and standard action cancellation.
- Use hybrid TDD: test-first for route planning, FSM, YAML parsing, transition predicates, and AMCL readiness policy; tests-after for ROS action/service wiring, WebSocket integration, bag lifecycle, and hardware-gated trials.
- Do not add twist_mux, a safety node, localization manager, map proxy, sensor-fusion node, transition score framework, or FAST-LIO navigation dependency.

## Scope IN
- Three ROS1 Noetic catkin Python packages and exactly three custom runtime nodes: mission_manager, multifloor_manager, stair_supervisor.
- Building graph and location/stair/tag/transition/scan YAML schemas with validators.
- Mission action contract, status/debug topics, floor/stair actions or services, and launch integration.
- Multi-floor route planning, outbound scan mission, fresh home-route planning, cancellation, retry, and mission-level failure codes.
- change_map, initialpose, AMCL readiness, clear_costmaps, move_base goal/cancel, AprilTag temporal voting, TRON1 high-level commands, and rosbag lifecycle.
- Offline unit tests, ROS integration tests, mocked WebSocket tests, bag replay, and explicit hardware gates.

## Scope OUT (Must NOT have)
- Any fourth custom ROS node or separate custom interface/bringup node.
- Full 3D navigation, pose graph, continuous FAST-LIO localization, map proxy, command mux, command-arbiter node, localization manager, sensor-fusion node, safety-monitor node, or transition confidence score framework.
- Low-level joint/footstep planning, automatic coverage planning, semantic perception, or geometry-required stair detection.
- Claiming stair ascent/descent safe or supported before robot-specific guarded experiments pass.

## Open questions
- None. The user selected the custom Mission action and hybrid TDD; robot-command ownership is resolved in favor of stair_supervisor after comparing practical MVP tradeoffs.

## Approval gate
status: approved-and-planned
approach: Build exactly three catkin Python packages under src/. mission_manager owns the Mission action, generic FSM, route planner, move_base action client, and rosbag lifecycle. multifloor_manager owns floor identity, AprilTag voting, change_map, stored exit pose, initialpose publication, AMCL readiness, and costmap clearing. stair_supervisor owns the sole TRON1 WebSocket, navigation command forwarding, stair FSM, high-level stair/twist requests, watchdog, and fail-closed zeros. Existing ROS packages remain separate third-party processes. Hardware stair ascent and descent remain blocked behind explicit guarded capability gates.
pending-action: plan written at .omo/plans/tron1-multifloor-mvp.md; user chooses `$start-work tron1-multifloor-mvp` or optional dual high-accuracy review. Metis was unavailable in the environment and the mandatory gap analysis was completed by Oracle model openai/gpt-5.6-sol instead.
<!-- When exploration is exhausted and unknowns are answered, set status: awaiting-approval. -->
<!-- That durable record is the loop guard: on a later turn read it and resume at the gate instead of re-running exploration. -->
