---
slug: tron1-stair-simulation
status: handed-off
intent: clear
review_required: false
pending-action: user starts worker with `$start-work tron1-stair-simulation`
approach: Preserve the existing stair supervisor and WebSocket RobotProtocol as the common application boundary; select a simulation or physical deployment profile at startup. Simulation runs the official WF_TRON1A Gazebo model and official pretrained ONNX locomotion controller behind a protocol-compatible simulator peer. Physical mode preserves the commissioned vendor WebSocket path. The plan explicitly treats low-level locomotion as different providers and proves high-level contract parity without claiming physics identity.
---

# Draft: tron1-stair-simulation

## Components (topology ledger)
<!-- Lock the SHAPE before depth. One row per top-level component that can succeed or fail independently. -->
<!-- id | outcome (one line) | status: active|deferred | evidence path -->
| id | outcome | status | evidence path |
|---|---|---|---|
| C1 | Reproducible official WF_TRON1A simulation with a pinned model, Gazebo runtime, and official ONNX locomotion artifacts | active | `config.env:13`; LimX `tron1-gazebo-ros` and `tron1-rl-deploy-ros` at cited commits |
| C2 | Protocol-compatible simulation peer implements the existing WebSocket request/status contract and translates normalized twist to the official controller `/cmd_vel` input | active | `src/stair_supervisor/src/stair_supervisor/{robot_protocol.py,robot_transport.py,robot_conversion.py}` |
| C3 | Existing mission, stair state machine, action contracts, and stair profiles run unchanged in both modes | active | `src/stair_supervisor/src/stair_supervisor/supervisor.py`; `src/mission_manager/src/mission_manager/ros_segments.py` |
| C4 | One owner-facing startup selection resolves all transport, sensor, world, topic, and configuration inputs consistently | active | `run.sh`; `config.env`; `src/mission_manager/launch/system.launch` |
| C5 | Measured stair digital twin and simulated LiDAR/IMU/odometry expose the same units, frames, topics, timing, and fault boundaries consumed by production | active | `docs/stair-up-commissioning.md`; `stair_captures/`; launch/config paths in findings |
| C6 | Automated parity, perturbation, replay, deployment, rollback, and hardware-promotion gates prove what can be transferred with a config switch | active | existing transport, process-boundary, ROS-node, and transition tests listed below |

## Open assumptions (announced defaults)
<!-- Record any default you adopt instead of asking, so the user can veto it at the gate. -->
<!-- assumption | adopted default | rationale | reversible? -->
| assumption | adopted default | rationale | reversible? |
|---|---|---|---|
| Exact robot variant | `WF_TRON1A`; unit identity remains `WF_TRON1A_632` only for physical deployment | Local commissioned `ACCID=WF_TRON1A_632`; official model/policy assets are variant-level, not serial-specific | yes, profile-controlled |
| Common application boundary | Preserve the deployed WebSocket `RobotProtocol`; make simulation emulate it | Leaves `StairSupervisor`, mission logic, and physical deployment path unchanged | yes |
| Simulation locomotion | Official WF_TRON1A ONNX artifacts from `tron1-rl-deploy-ros`, initially `RL_TYPE=isaacgym`, selected from a pinned commit | Official repository contains matching `policy.onnx` and `encoder.onnx`; no custom policy training is needed to begin | yes, profile-controlled |
| Physical locomotion | Preserve the current vendor WebSocket firmware and `request_stair_mode`; do not run the ROS-side ONNX hardware controller concurrently | Official ROS-side controller writes low-level joint commands and has no documented arbitration with the existing firmware path | yes, but changing it requires a separate vendor-supported commissioning decision |
| Meaning of sim-to-real | Same high-level code, state machine, command semantics, profiles, and one-command launch; low-level policies need not be byte-identical | This is the only supported path that preserves the commissioned physical controller while using official simulation assets | no for this plan's architecture contract |
| Test strategy | TDD for all new adapters/config resolvers, followed by deterministic integration and scenario tests | Existing repository already has protocol fakes, loopback WebSocket, ROS-node, and launch contract seams | yes |
| Dedicated workspace | Execution creates `/home/m3tron/Desktop/TRON1_Stair_Simulation`; this source workspace remains an upstream dependency and is not copied ad hoc | Matches the user's requested isolation while avoiding divergence of the production state machine | yes |

## Findings (cited - path:lines)
- `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/config.env:13` identifies the commissioned unit as `WF_TRON1A_632`.
- `run.sh:37-47,127-217` currently owns deployment inputs, remote sensors, SSH WebSocket forwarding, and top-level launch; it is production-only today.
- `src/mission_manager/launch/system.launch:3-83` is the single runtime composition point but has no `sim|real` deployment-mode argument.
- `src/stair_supervisor/src/stair_supervisor/ros_entrypoint.py:70-127` constructs exactly one `RobotTransport` from `accid` and `websocket_url`.
- `src/stair_supervisor/src/stair_supervisor/supervisor.py:206,255` switches stair mode through the injected transport while keeping behavior logic transport-independent.
- `src/stair_supervisor/src/stair_supervisor/robot_transport.py:184-276` owns STAND/WALK startup, normalized twist, watchdog, stair mode, odometry/IMU requests, emergency stop, and shutdown barriers.
- `src/multifloor_manager/launch/navigation.launch:92-114` (covered by `test/test_launch_contract.py`) isolates move_base output on `/navigation/cmd_vel` and consumes `/tron/wheel_odom_raw`.
- Existing test seams include `src/stair_supervisor/test/robot_transport/`, `src/stair_supervisor/test/test_stair_supervisor_node.py`, `src/mission_manager/test/mock_peer_server.py`, and `src/mission_manager/test/test_process_boundaries.py`.
- LimX `tron1-gazebo-ros` commit `e318ceb2282d053c086866d333bdd99b4c2af77b` provides Gazebo integration and low-level examples, but not a ready locomotion policy.
- LimX `tron1-rl-deploy-ros` commit `14ac8d19db4ffaaeea2ac0915216728b223acf23` provides WF_TRON1A `isaacgym` and `isaaclab` ONNX policy/encoder artifacts and subscribes to `/cmd_vel`; this plan pins `isaacgym` as the default.
- The official ONNX runtime exposes only STAND/WALK and no explicit stair-mode, terrain selector, stair perception, or dedicated stair policy. Stair terrain in training is not evidence of a dedicated stair controller.
- The official physical ONNX launch writes low-level commands through the LimX SDK. Official sources do not establish safe coexistence or arbitration with the commissioned WebSocket firmware path.
- Eleven captured stair bags contain LiDAR and wheel odometry but no command/control trace; they support geometry/perception calibration and held-out replay, not control replay.

## Decisions (with rationale)
- Use `mode:=sim|real` as the sole normal deployment selector; reject partial mixed-mode startup.
- Keep the existing supervisor, phase machine, ROS actions, normalized command semantics, and physical WebSocket client unchanged.
- Implement a simulator-side WebSocket peer rather than a second supervisor transport API, so the exact production protocol is exercised in simulation.
- Map simulated `request_twist` to the official controller's `/cmd_vel`; emulate the full correlation/status/watchdog contract rather than acknowledging every request optimistically.
- Treat `request_stair_mode` in simulation as an application state and gating contract, not as evidence that the official ONNX controller has a distinct stair gait.
- Keep the vendor WebSocket locomotion path canonical for the physical robot; never start the direct low-level ONNX hardware controller at the same time.
- Pin official repositories and policy artifacts by commit and verify hashes; do not depend on floating `master`.
- Require the same stair profile schema and command limits in both modes, with only adapter-, endpoint-, sensor-, and physics-specific values split by profile.
- Make simulation success a promotion gate, not proof of real-world stair success; require a bounded hardware commissioning sequence before operational use.

## Scope IN
- New dedicated Desktop simulation workspace and reproducible build/runtime environment.
- Official WF_TRON1A model, official ONNX locomotion deployment, realistic measured stair world, and simulated production-equivalent sensors.
- Protocol-compatible simulation peer and one-command `sim|real` profile selection.
- Reuse of the existing mission/stair behavior code without forks or copied implementations.
- Contract parity, deterministic scenario tests, perturbation/fault injection, bag replay, sim-to-real configuration promotion, deployment bundle, and rollback.
- Explicit documentation of which properties are shared across modes and which remain low-level provider differences.

## Scope OUT (Must NOT have)
- No custom RL policy training unless the official WF_TRON1A policy fails a separately evidenced capability gate and the user approves a future plan.
- No replacement of the physical robot's vendor controller, no concurrent low-level controller ownership, and no undocumented firmware flashing.
- No duplicated or simulator-specific stair state machine.
- No claim that Gazebo success guarantees hardware success or that simulation reproduces proprietary onboard dynamics exactly.
- No manual source edits as the normal sim/real deployment mechanism; the normal operation must be a profile/launch selection.

## Open questions
- Resolved by user: simulation uses LimX's official WF_TRON1A ONNX controller while physical mode preserves the existing vendor WebSocket stair controller. This gives identical high-level code and a configuration-only switch, but not identical low-level locomotion implementation.
- Resolved by user: TDD plus automated integration/scenario verification.

## Approval gate
status: plan-written
approach: Preserve one high-level application and protocol contract. Use a protocol-compatible Gazebo/ONNX simulation backend for `mode:=sim`; preserve the commissioned vendor WebSocket backend for `mode:=real`; isolate deployment-specific values in validated profiles; prove parity and promotion through automated gates.
approval receipt: User selected `상위 코드 동일 (Recommended)` and `TDD 승인 (Recommended)`.
plan path: `.omo/plans/tron1-stair-simulation.md`
canonical execution truth: `.omo/plans/tron1-stair-simulation.md`; this draft is historical intent/research/approval provenance and must not override the completed plan.
plan shape: 19 implementation todos across 4 waves plus 4 final-verification tasks.
mandatory Metis receipt: `ses_f9f8f7f43ffeVWjUAM393bXe4g`; integrated pinning, scale conversion, clock-domain, sensor-contract, command-authority, watchdog, oracle-independence and hardware-promotion corrections.
structural self-check: 8 canonical headers in order; 19 column-zero implementation rows; 4 column-zero final-verifier rows; no task grammar violations.
SIP review:
  shower: `ses_f9f740549ffeKICrdx63EikkFs`; fixed dead-man ownership, exact evidence predicates, clone URLs, import allowlist, artifact schemas, scale axes, capability-blocked semantics, and hardware checkpoint ordering.
  factchk: `ses_f9f73f69affe3Mal3Uldz5hug8`; verified repository commits, ROS Noetic/WF_TRON1A deployment surface, local seven phases/three application nodes/watchdog/ACCID; recast compatibility, scale and policy capability as pinned-source gates or design thresholds rather than guarantees.
  mandela: fired `shared hallucination`, `tautology`, `verifier=designer`, and `shared-pool bias`; fixed with real motionless firmware cross-check, pre-run frozen scorer/scenario/site hashes, non-simulator labels, dated tuning/held-out split, and separate hardware receipts.
  ssotize: read-only audit found intentional plan/draft duplication plus a separate superseded localization plan; `.omo/plans/tron1-stair-simulation.md` is canonical execution truth, the draft remains provenance, and no consolidation mutation is needed.
  detool: skipped because this is an intentionally ROS/LimX/Gazebo-specific operational plan, not a portability claim.
  re0: removed patched-over ambiguity while preserving the approved architecture and canonical plan structure.
handoff receipt: User selected `바로 실행 (Recommended)` and declined the optional dual high-accuracy review.
next workflow action: User starts a separate worker session with `$start-work tron1-stair-simulation`. No implementation starts in this planning session.
