# tron1-stair-simulation - Work Plan

## TL;DR (For humans)

**What you'll get:** 바탕화면의 새 통합 workspace 하나에서 동일한 계단 상태머신과 센서 기반 계단 판단 코드를 사용하고, 실행 옵션만 `sim` 또는 `real`로 바꾸는 TRON1 계단 주행 시스템입니다. 시뮬레이션에서는 LimX의 공식 WF_TRON1A 모델과 공식 ONNX 보행정책을 사용하고, 실제 로봇에서는 현재 검증된 WebSocket firmware를 그대로 사용합니다.

**Why this approach:** 실제 firmware와 공식 시뮬레이션 정책 사이에 문서화된 저수준 동등성이나 동시 제어 계약이 없으므로, 안정된 WebSocket 명령 계약 위의 코드만 완전히 공통화합니다. 계단 판단과 우측 벽 보정도 동일한 LiDAR·odometry 입력으로 동작하게 만들어 simulator 전용 정답 신호에 의존하지 않습니다.

**What it will NOT do:** 시뮬레이션 성공을 실제 성공으로 과장하지 않습니다. 실제 firmware와 ROS ONNX 저수준 controller를 동시에 실행하지 않으며, 별도 승인 없는 custom RL 학습·firmware flashing·ROS 2 전환도 하지 않습니다.

**Effort:** XL
**Risk:** High - 공식 ONNX 정책에 별도 stair mode가 없고 실제 계단에서의 locomotion 성능이 공식적으로 보장되지 않습니다.
**Decisions to sanity-check:** 새 workspace를 sim/real의 단일 기준 소스로 삼고 기존 workspace는 해시가 기록된 import 원본으로만 보존합니다. SIM은 공식 ONNX, REAL은 현재 vendor WebSocket locomotion을 사용하며 공통 경계 위의 코드만 동일합니다. 테스트는 TDD입니다.

Your next move: `$start-work tron1-stair-simulation`으로 별도 worker 세션에서 실행합니다. Full execution detail follows below.

---

> TL;DR (machine): XL/high-risk unified ROS Noetic workspace; pinned WF_TRON1A Gazebo+ONNX backend, protocol-compatible simulator peer, shared sensor evidence/wall controller, mode-only sim/real launch, deterministic and hardware-promotion gates.

## Scope
### Must have
- 실행 시 `/home/m3tron/Desktop/TRON1_Stair_Simulation`을 생성하고 독립 Git repository이자 ROS Noetic Catkin workspace로 초기화한다.
- `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`의 논리적 source/config/test/docs만 한 번 해시 검증하여 새 workspace로 import하고, `build/`, `devel/`, `install/`, `log/`, `.omo/`와 생성 산출물은 import하지 않는다. 이후 새 workspace를 sim/real 공통 source of truth로 사용한다.
- 외부 LimX source를 repository에 복제해 재배포하지 않고 bootstrap 시 아래 HTTPS URL과 고정 commit으로 clone한다. 정상 실행에서 mirror나 floating fallback은 사용하지 않는다.
  - `https://github.com/limxdynamics/tron1-gazebo-ros.git@e318ceb2282d053c086866d333bdd99b4c2af77b`
  - `https://github.com/limxdynamics/tron1-rl-deploy-ros.git@14ac8d19db4ffaaeea2ac0915216728b223acf23`
  - `https://github.com/limxdynamics/tron1-robot-description.git@5b97add1f3b461c9ed26ff2ff2f5025cc6ee4316`
  - `https://github.com/limxdynamics/limxsdk-lowlevel.git@70ff83c22f5f54a07c2ddf7e32a2c6ffb1d2ebc7`
- Ubuntu 20.04, ROS Noetic, Gazebo Classic 11, Catkin, x86_64를 고정 실행 환경으로 삼고 다른 ROS/Gazebo 조합은 fail-closed한다.
- WF_TRON1A와 `RL_TYPE=isaacgym`의 `policy.onnx`·`encoder.onnx`를 기본 locomotion으로 사용한다. joint order, file size, Git blob ID와 실행 시 계산한 SHA-256을 lock manifest로 검증한다.
- 기존 `StairSupervisor`의 7개 phase 순서, action contract, command ownership, safe-checkpoint cancellation, WebSocket envelope/GUID/status/fault semantics를 유지한다.
- simulator는 기존 WebSocket `RobotProtocol`을 구현하는 server가 되어 기존 `RobotTransport`가 수정 없이 연결되도록 한다.
- `request_stair_mode`는 simulator protocol status를 `STAIR/WALK`로 전환하지만 ONNX에 존재하지 않는 stair gait를 호출했다고 가장하지 않는다.
- normalized WebSocket twist와 공식 ONNX `/cmd_vel`의 서로 다른 scale을 명시적으로 변환한다. simulator peer의 단일 `CommandLease`가 0.25 s dead-man timer와 상태를 소유하고, adapter는 peer가 요청한 변환·publish·idempotent zero만 수행한다.
- 계단 evidence와 우측 벽 0.30 m 보정은 simulator ground-truth phase flag가 아니라 `/scan`, `/tron/wheel_odom_raw`, TF를 소비하는 동일 in-process implementation으로 SIM과 REAL에서 실행한다.
- application node `mission_manager`, `multifloor_manager`, `stair_supervisor` 정확히 3개 운영 계약을 유지한다. Gazebo와 vendor controller 같은 infrastructure node는 이 숫자에 포함하지 않는다. 계단 perception/control은 `stair_supervisor` 내부 injectable component로 둔다.
- 9단, landing, 10단 실제 계단을 두 forward segment와 한 landing으로 나타내는 measured site profile을 만든다. 현 상태머신에는 per-step counter가 없으므로 9/10 step을 새 phase로 추가하지 않는다.
- 정상, timeout, disconnect, malformed response, stale status, sensor dropout, friction/noise/latency perturbation을 자동 검증하고 모든 결과를 evidence artifact로 남긴다.
- `./run.sh --mode sim`과 `./run.sh --mode real`만 정상 운영 진입점으로 제공하고 mixed mode 또는 중복 command owner를 거부한다.
- 실제 로봇 배포 전 read-only preflight, flat-motion canary, monitored stair promotion, rollback 순서를 제공하며 실제 firmware 경로 외의 저수준 writer는 시작하지 않는다.

### Must NOT have (guardrails, anti-slop, scope boundaries)
- 기존 source workspace를 수정하거나 build artifact를 복사하지 않는다. import 후에는 새 workspace 안에 두 번째 application copy를 만들지 않는다.
- floating branch, unverified vendor binary, `git pull` 기반 실행, 자동 dependency upgrade를 허용하지 않는다.
- `robot_visualization`과 PlotJuggler는 headless 필수 dependency가 아니므로 포함하지 않는다. 공식 launch의 `rqt_robot_steering`도 자동 `/cmd_vel` publisher 충돌을 막기 위해 실행하지 않는다.
- REAL mode에서 `pointfoot_hw.launch` 또는 LimX SDK direct joint writer를 시작하지 않는다. vendor WebSocket firmware와 ROS ONNX hardware controller를 동시에 실행하지 않는다.
- SIM phase feedback을 그대로 evidence `true`로 되돌리는 self-confirming publisher, Gazebo pose zone만으로 성공시키는 oracle, test가 스스로 만든 trajectory를 ground truth로 사용하는 검증을 금지한다.
- official ONNX policy가 실제 9+10 계단을 오를 수 있다는 사전 가정을 acceptance로 바꾸지 않는다. capability gate를 통과하지 못하면 실패 evidence를 남기고 완료로 선언하지 않는다.
- `isaacgym` nominal gate 실패 시에만 사전에 lock한 `isaaclab` profile을 동일한 동결 시나리오로 한 번 평가한다. 둘 다 실패하면 terminal status는 `CAPABILITY_BLOCKED_OFFICIAL_POLICY`이며 Tasks 17-19와 release tag를 시작하지 않는다.
- 별도 사용자 승인 없이 custom policy 학습, firmware flashing, proprietary controller reverse engineering, ROS 2 migration을 추가하지 않는다.
- 기존 7 phase를 per-step state machine으로 재작성하거나 Mission/Stair action schema를 변경하지 않는다.
- 물리 e-stop을 software e-stop으로 대체하지 않는다. 실제 motion promotion은 `--operator-present`와 explicit stage selection 없이는 시작하지 않는다.
- Python/ROS module은 순수 LOC 250줄을 넘기지 않고 giant bridge/controller module, duplicated parser, catch-all exception, silent fallback profile을 만들지 않는다.

## Verification strategy
> Zero human intervention - all software verification is agent-executed. 실제 motion 단계에서 사람은 물리 e-stop 담당자로만 존재하며 판정·명령·evidence 수집은 script가 수행한다.
- Test decision: TDD; Python `unittest`, `rostest`, Catkin tests, deterministic Gazebo scenario runner, rosbag replay, schema/pin validators를 사용한다.
- Regression oracles: `src/stair_supervisor/test/robot_transport/test_robot_transport_fakes.py`, `test_robot_transport.py`, `test_robot_transport_faults.py`, `test_robot_protocol.py`, `test_supervisor.py`, `test_stair_supervisor_node.py`. 기존 동작을 고정하지만 독립적인 firmware ground truth로 과장하지 않는다.
- Cross-mode contract: 같은 scripted sensor/command trace에서 WebSocket boundary의 request ordering와 normalized payload가 동일해야 하며 backend-specific `/cmd_vel` scale은 adapter 내부에만 존재해야 한다.
- Independent evidence: tuning corpus와 held-out bag을 기록 날짜 기준으로 먼저 동결한다. phase label과 geometry는 기존 commissioning 기록·camera visual review·site measurement에서 만들고 simulator/state-machine output으로 만들지 않는다. 독립 label이 없는 bag은 평가 점수에서 제외하되 누락을 manifest에 기록한다.
- Leakage controls: protocol regression vector는 pinned source와 Task 19 motionless hardware handshake에 양방향 대조한다. nominal/robustness scorer·case manifest·site-survey hash는 첫 run 전에 동결하고 scorer가 만든 output이나 runtime controller state는 정답 생성에 사용하지 않는다.
- Physics gate: deterministic seeds `0..19`에서 20/20 완주, zero fall/collision; calibrated perturbation matrix 100 trials에서 최소 95/100 완주, zero command-authority collision을 요구한다.
- Hardware claim gate: SIM 통과만으로 real capability를 선언하지 않는다. REAL은 별도 preflight·canary·stair promotion receipt가 있어야 `hardware_promoted=true`가 된다.
- Evidence: `<attemptDir>/task-<N>-tron1-stair-simulation.<ext>`; `attemptDir`는 `omo ulw-loop status --json`의 `currentAttemptDir`, loop 밖에서는 `.omo/evidence/`이다.

## Execution strategy
### Parallel execution waves
> 각 task는 implementation과 test를 함께 끝낸다. 같은 wave 안에서 파일 소유권이 겹치지 않는 task만 병렬 실행한다.

- **Wave 1 — reproducible contracts:** Tasks 1-5. workspace/import, vendor lock, physical capture contract, deployment schema, golden protocol vectors를 독립적으로 확정한다.
- **Wave 2 — isolated components:** Tasks 6-11. protocol peer, scale/watchdog adapter, official ONNX bringup, stair world/sensors, sensor evidence, wall-aware motion policy를 TDD로 병렬 구현한다.
- **Wave 3 — composition:** Tasks 12-15. mode/clock launch, peer-controller integration, common perception/control replay, automated quality pipeline을 결합한다.
- **Wave 4 — proof and promotion:** Tasks 16-19. nominal traversal, robustness matrix, one-command distribution/rollback, physical promotion을 순서대로 증명한다.
- **Final wave:** F1-F4는 모든 implementation task 뒤 병렬 실행하며 모두 APPROVE해야 한다.

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| 1 | — | 6-19 | 2, 3, 4, 5 |
| 2 | — | 8, 12, 15-19 | 1, 3, 4, 5 |
| 3 | — | 9-11, 14, 16-19 | 1, 2, 4, 5 |
| 4 | — | 6-7, 10-19 | 1, 2, 3, 5 |
| 5 | — | 6, 13, 15-19 | 1, 2, 3, 4 |
| 6 | 1, 4, 5 | 13-19 | 7-11 |
| 7 | 1, 4, 5 | 13, 16-19 | 6, 8-11 |
| 8 | 1, 2, 4 | 13, 16-19 | 6, 7, 9-11 |
| 9 | 1, 3, 4 | 12, 14, 16-19 | 6-8, 10-11 |
| 10 | 1, 3, 4 | 14, 16-19 | 6-9, 11 |
| 11 | 1, 3, 4 | 14, 16-19 | 6-10 |
| 12 | 1-4, 8-9 | 13, 15-19 | 14 |
| 13 | 5-8, 12 | 15-19 | 14 |
| 14 | 3-4, 9-11 | 15-19 | 12-13 |
| 15 | 1-14 | 16-19 | — |
| 16 | 8-15 | 17-19 | — |
| 17 | 16 | 18-19 | — |
| 18 | 15-17 | 19 | — |
| 19 | 16-18 | F1-F4 | — |

## Todos
> Implementation + Test = ONE todo. Never separate.

- [x] 1. Initialize the canonical unified sim/real workspace from a verified source import
  What to do / Must NOT do: Create `/home/m3tron/Desktop/TRON1_Stair_Simulation` with `src/`, `config/`, `worlds/`, `scripts/`, `tests/`, `vendor/`, `artifacts/`, `docs/`, `.omo/evidence/`; initialize Git. Import the positive allowlist `README.md`, `run.sh`, `validate_bundle.py`, root `test/`, `docs/`, and `src/{mission_manager,multifloor_manager,stair_supervisor}` from `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`. Exclude `config.env`, bags, `.omo/`, credentials, build/devel/install/log/cache and generated files; create a committed `.env.example` containing keys only and keep runtime `.env` ignored. Generate sorted `IMPORT_MANIFEST.json` with schema `{schema_version:1,source_root,imported_at_utc,entries:[{path,sha256,size_bytes}]}`. Do not alter the source workspace. Make the new repository the only editable sim/real source after import.
  Parallelization: Wave 1 | Blocked by: none | Blocks: 6-19
  References (executor has NO interview context - be exhaustive): source `README.md`; `run.sh`; `config.env`; `src/{mission_manager,multifloor_manager,stair_supervisor}`; root `test/`; `docs/`; exclude generated `build/`, `devel/`, `install/`; draft decision `.omo/drafts/tron1-stair-simulation.md`.
  Acceptance criteria (agent-executable): `python3 scripts/verify_import.py --manifest IMPORT_MANIFEST.json --source /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation` exits 0 and proves allowlist completeness and exclusions; generated directories are absent; `config.env` is absent; secret-pattern scan is empty; source tree checksum before/after is identical.
  QA scenarios (name the exact tool + invocation): happy — run import into an empty temporary directory and verify all manifest hashes; failure — add a generated `devel/` file and mutate one imported file, rerun validator, assert both are reported and exit is nonzero. Evidence `<attemptDir>/task-1-tron1-stair-simulation.json`.
  Commit: Y | `chore(workspace): initialize canonical sim-real source tree`

- [x] 2. Lock and verify the complete official LimX headless dependency set
  What to do / Must NOT do: Add `DEPENDENCIES.lock.yaml`, `scripts/bootstrap_vendor.sh`, and `scripts/verify_vendor.py` for the four approved HTTPS repositories/commits. Use schema `{schema_version:1,environment:{ubuntu,ros,gazebo,arch},repositories:[{name,url,commit,license_file,artifacts:[{path,git_blob,size_bytes,sha256}]}]}`. Record license/NOTICE status without assuming ONNX/prebuilt-binary terms. Clone exact commits into ignored `vendor/src/`; never fetch floating master/mirror during normal run and never commit vendor binaries. Explicitly omit optional `robot-visualization` and `plot:=true`.
  Parallelization: Wave 1 | Blocked by: none | Blocks: 8, 12, 15-19
  References: LimX `tron1-gazebo-ros@e318ceb2282d053c086866d333bdd99b4c2af77b` README lines 27-63; `tron1-rl-deploy-ros@14ac8d19db4ffaaeea2ac0915216728b223acf23` README lines 48-119 and `robot_hw/launch/pointfoot_hw_sim.launch`; `tron1-robot-description@5b97add1f3b461c9ed26ff2ff2f5025cc6ee4316`; `limxsdk-lowlevel@70ff83c22f5f54a07c2ddf7e32a2c6ffb1d2ebc7` README/CMake; optional visualization exclusion `pointfoot_hw_sim.launch:1-19`.
  Acceptance criteria: from an empty `vendor/src`, `./scripts/bootstrap_vendor.sh && python3 scripts/verify_vendor.py DEPENDENCIES.lock.yaml` exits 0; `git -C vendor/src/<repo> rev-parse HEAD` equals each exact SHA; all recorded binary hashes/sizes match; wrong OS/ROS/architecture exits before Catkin build; `git status --short` shows no vendor files tracked.
  QA scenarios: happy — clean clone and verify twice idempotently; failure — checkout one repository one commit away and alter one ONNX byte, assert verifier names both drift sources and exits nonzero. Evidence `<attemptDir>/task-2-tron1-stair-simulation.json`.
  Commit: Y | `build(vendor): pin official tron1 simulation dependencies`

- [x] 3. Derive immutable stair, odometry, frame, and corpus contracts from physical captures
  What to do / Must NOT do: Add `scripts/extract_capture_contract.py`, `config/site/stair_3f_4f.yaml`, `tests/fixtures/capture_contract.json`, and `tests/fixtures/corpus_split.yaml`. Inspect all bags plus commissioning/visual-review material to record topic types, frames, rates, timestamp behavior, scan geometry, odometry semantics, 9-step/landing/10-step geometry and right-wall reference. Freeze tuning/held-out sets by recording date before controller implementation. Corpus entries use `{path,sha256,recorded_at,split,label_source,labels}`; `label_source` is commissioning record, camera visual review or measured site survey, never simulator/state-machine output. Missing dimensions or independent labels become `measurement_required`/`label_required`, never invented defaults.
  Parallelization: Wave 1 | Blocked by: none | Blocks: 9-11, 14, 16-19
  References: `stair_captures/*.bag`; `docs/stair-up-commissioning.md:3-23`; `src/multifloor_manager/launch/navigation.launch` odom remap; `src/multifloor_manager/launch/pointcloud_to_laserscan.launch`; `test/test_launch_contract.py:92-164`; visual review images under `stair_captures/visual_review_*`.
  Acceptance criteria: `python3 scripts/extract_capture_contract.py --capture-root <source>/stair_captures --output tests/fixtures/capture_contract.json --site-config config/site/stair_3f_4f.yaml` is deterministic; config validates `flight_1_steps=9`, `flight_2_steps=10`, positive measured tread/riser/landing dimensions, `right_wall_target_m=0.30`; topic/frame/rate values come from bag messages; tuning and held-out file sets are nonempty/disjoint and cover every stair bag exactly once.
  QA scenarios: happy — regenerate twice and byte-compare outputs; failure — remove required odometry or geometry evidence from a temporary corpus and assert explicit `measurement_required` failure, not a default dimension. Evidence `<attemptDir>/task-3-tron1-stair-simulation.json`.
  Commit: Y | `test(site): lock physical stair and sensor contracts`

- [x] 4. Define strict deployment, stair-observation, and motion-policy schemas
  What to do / Must NOT do: Add typed loaders and strict schema-versioned YAML for `config/deployment/{sim,real}.yaml`, `config/stair/{observation,motion}.yaml`, and the measured site profile. Define `mode`, ACCID, endpoint, use_sim_time, topic/frame names, full-scale units, controller scale, watchdog, evidence source, right-wall target/tolerances, dwell/freshness, progress/heading thresholds, gains/limits and hardware promotion stage. Resolve to one immutable runtime bundle; reject unknown keys, mixed sim/real transports, duplicate command owners, invalid finite ranges and unmeasured site fields. Do not hide mode in multiple environment variables.
  Parallelization: Wave 1 | Blocked by: none | Blocks: 6-7, 10-19
  References: `src/stair_supervisor/src/stair_supervisor/configuration.py:37-77,146-203`; `robot_config.py:26-75`; `ros_entrypoint.py:70-127`; `config.env:1-31`; existing production `stair_supervisor/config/{robot,stair_profiles}.yaml`; user-approved `mode:=sim|real` boundary.
  Acceptance criteria: `docs/contracts/runtime-v1.md` and machine JSON Schemas define every field/unit in deployment/site/observation/motion/scenario/receipt artifacts; tests pass exhaustive valid/invalid fixtures; SIM resolves controller endpoint, `/clock`, simulated topics and no hardware tunnel; REAL resolves `WF_TRON1A_632` as the protocol robot identity, current WebSocket route and no SDK writer; malformed/unknown/mixed configuration exits before side effects.
  QA scenarios: happy — snapshot exact resolved JSON for both profiles; failure — combine real robot IP with sim low-level controller and assert `conflicting_command_owners` before side effects. Evidence `<attemptDir>/task-4-tron1-stair-simulation.json`.
  Commit: Y | `feat(config): define fail-closed sim-real runtime profiles`

- [x] 5. Freeze the production WebSocket protocol as reusable golden vectors
  What to do / Must NOT do: Extract reusable, implementation-neutral protocol fixtures from existing fake tests into `tests/protocol_vectors/` and a shared contract runner. Cover exact five-field envelope, ACCID echo, GUID correlation, response/notify ordering, STAND/WALK startup, stair enable/disable, odom/IMU enable ACKs, twist payloads, zero barriers, malformed/stale/timeout/disconnect and permanent fault/no-reconnect. Preserve existing tests; do not make the simulator implementation itself the oracle.
  Parallelization: Wave 1 | Blocked by: none | Blocks: 6, 13, 15-19
  References: `src/stair_supervisor/src/stair_supervisor/robot_protocol.py`; `robot_client.py`; `robot_transport.py:184-276`; `robot_conversion.py`; `src/stair_supervisor/test/robot_transport/{test_robot_transport_fakes,test_robot_transport,test_robot_transport_faults,test_robot_protocol}.py`; existing `src/mission_manager/test/test_process_boundaries.py`.
  Acceptance criteria: golden runner passes against `FakeWebSocket/FakeFactory`; vectors enumerate every `RequestTitle`; wrong ACCID/GUID/status/timestamp/order mutations fail exact assertions; pinned protocol/client source agrees with vector fields; vectors remain marked `regression`, not `firmware-ground-truth`, until Task 19's motionless handshake confirms them; existing tests stay green.
  QA scenarios: happy — `python3 -m unittest tests.test_protocol_vectors -v`; failure — run vector mutator for one field per envelope/status invariant and assert 100% mutations rejected. Evidence `<attemptDir>/task-5-tron1-stair-simulation.json`.
  Commit: Y | `test(protocol): publish reusable robot transport vectors`

- [x] 6. Implement the protocol-compatible simulator WebSocket peer with TDD
  What to do / Must NOT do: Create a small `tron1_sim_bridge` Catkin/Python package whose WebSocket server accepts one configured ACCID, emits correlated responses and fresh `notify_robot_info`, implements startup/mode/enable/twist/emergency-stop/close semantics, and exposes typed `VelocitySink` and `ControllerStateSource` ports. `request_stair_mode` changes emulated status only; it must not claim to select an ONNX stair policy. Maintain one session, deterministic timestamps, bounded queues and permanent fault behavior; no optimistic ACK before backend acceptance. The peer alone owns `CommandLease(last_valid_command_monotonic, timeout_sec=0.25)` and invokes `VelocitySink.zero()` on expiry, disconnect, fault, e-stop and mode exit.
  Parallelization: Wave 2 | Blocked by: 1, 4, 5 | Blocks: 13-19
  References: golden vectors from Task 5; `robot_protocol.py:16-36,67-131`; `robot_client.py:127-188`; `robot_transport.py:184-276`; `test_robot_transport_fakes.py`; pinned LimX controller source exposes STAND/WALK integration and no runtime stair selector was found.
  Acceptance criteria: peer passes the golden contract runner over a real loopback socket; startup reaches WALK only after backend ready; stair enable yields response then fresh STAIR status, disable yields WALK; exactly one `CommandLease` timer exists and expiry invokes one idempotent zero; every protocol fault closes and latches the session; module LOC limits pass.
  QA scenarios: happy — launch peer on an ephemeral port and run all golden traces; failure — inject delayed, malformed, mismatched-GUID and backend-rejected operations, assert the client enters FAULT and peer does not silently reconnect. Evidence `<attemptDir>/task-6-tron1-stair-simulation.log`.
  Commit: Y | `feat(sim-peer): emulate the production robot protocol`

- [x] 7. Implement normalized-twist conversion, rate limiting, and an idempotent zero sink
  What to do / Must NOT do: Add a pure `VelocitySink` adapter from production `NormalizedTwist` to the pinned controller's `/cmd_vel` semantics. Task 2 must first verify the expected pinned-source mapping: `linear.x` full scale 1.5, `linear.y` 1.0 and `angular.z` 0.5; production keeps `y=0`. Preserve physical full-scale SI meaning rather than raw pass-through, clamp finite values, publish at the configured rate and expose idempotent `zero()`. This adapter owns no timer or lease; Task 6 is the sole dead-man owner. No second `/cmd_vel` publisher is allowed.
  Parallelization: Wave 2 | Blocked by: 1, 4, 5 | Blocks: 13, 16-19
  References: `robot_conversion.py`; `configuration.py:58-70,186-198`; `robot_transport.py` watchdog/zero barriers; LimX `ControllerBase.cpp:68-74`; `PointfootController.cpp:435-446`; official scales verified by Metis; `config.env` and `robot.yaml` production limits.
  Acceptance criteria: table-driven tests prove SI command → normalized payload → controller `/cmd_vel` → effective SI round-trip within `1e-6` for zero, limits, over-range and signs; Task 2 source audit proves or rejects the expected axis mapping before tests accept it; repeated `zero()` remains zero; one publisher owns `/cmd_vel`; NaN/Inf or invalid scale fails closed.
  QA scenarios: happy — replay a ramp/turn command matrix and compare recorded ROS messages to expected JSON; failure — mutate the pinned scale mapping and call `zero()` repeatedly, asserting startup rejects scale drift and output never becomes nonzero. Evidence `<attemptDir>/task-7-tron1-stair-simulation.json`.
  Commit: Y | `feat(sim-bridge): preserve velocity semantics and zero output`

- [x] 8. Build a headless pinned WF_TRON1A Gazebo and ONNX locomotion bringup
  What to do / Must NOT do: Create a project-owned headless launch that composes the pinned WF_TRON1A description, Gazebo Classic and official RL controller with `ROBOT_TYPE=WF_TRON1A`, `RL_TYPE=isaacgym`, `plot=false`. Exclude `rqt_robot_steering`; validate the isaacgym joint order against URDF/controller params before loading; enforce one SDK low-level writer and one `/cmd_vel` source; expose controller readiness/status to the peer. Do not start official physical `pointfoot_hw.launch` in any mode.
  Parallelization: Wave 2 | Blocked by: 1, 2, 4 | Blocks: 13, 16-19
  References: pinned repositories Task 2; official `robot_hw/launch/pointfoot_hw_sim.launch:1-19`; `robot_controllers/config/controllers.yaml`; `PointfootController.cpp`; WF_TRON1A Xacro/transmission/Gazebo assets; official Catkin build commands.
  Acceptance criteria: `catkin_make install` succeeds from clean vendor/cache state; headless launch reaches controller-ready; commanded forward/turn/zero visibly changes simulated base state; runtime audit finds one SDK command writer and one `/cmd_vel` publisher; policy/encoder hashes and joint order are logged before enable.
  QA scenarios: happy — launch for 30 s, issue bounded twist then zero, assert motion then stop from Gazebo model state; failure — swap isaaclab joint order or alter policy hash and assert launch exits before command enable. Evidence `<attemptDir>/task-8-tron1-stair-simulation.log`.
  Commit: Y | `feat(gazebo): bring up pinned wf-tron1a locomotion`

- [x] 9. Build the measured 9-step/landing/10-step world and production-topic sensor contracts
  What to do / Must NOT do: Generate `worlds/stair_3f_4f.world` from Task 3's measured site config, including right wall and collision geometry. Add only consumer-contract sensors needed by shared code: `/scan`, `/tron/wheel_odom_raw`, IMU/TF and minimal fixture publishers required for full-system readiness. Match bag-derived frame IDs, rates, units and timestamp rules. Keep odom/IMU WebSocket enable requests as ACK-only protocol operations; ROS sensor publishers remain separate. Do not emulate Livox/D435 photorealism or use phase-triggered ground truth.
  Parallelization: Wave 2 | Blocked by: 1, 3, 4 | Blocks: 12, 14, 16-19
  References: Task 3 capture/site contracts; official WF_TRON1A IMU/Gazebo Xacro; `pointcloud_to_laserscan.launch:3-18`; `navigation.launch`; `multifloor_manager/ros_node.py:87-94`; `test/test_launch_contract.py:92-213`.
  Acceptance criteria: world generator output is deterministic; SDF collision dimensions equal site config; `/scan` has exactly one publisher; odom topic/type/frame/child/rate match capture contract; TF has no loops and connects required base/sensor frames; no evidence topic is published by world plugins.
  QA scenarios: happy — launch world, run topic/frame audit for 60 s and compare JSON to capture contract; failure — alter odom frame or start a second scan publisher and assert readiness/audit fails. Evidence `<attemptDir>/task-9-tron1-stair-simulation.json`.
  Commit: Y | `feat(world): model measured stair and sensor contracts`

- [x] 10. Add shared in-process sensor evidence without changing the seven-phase state machine
  What to do / Must NOT do: Inside `stair_supervisor`, introduce an injectable, thread-safe sensor snapshot/evidence implementation consuming `/scan`, `/tron/wheel_odom_raw` and TF. On action reset, establish an odometric origin and compute exactly: `VERIFY_ENTRY=fresh sensors + valid entrance corridor/right-wall fit`; `ALIGN=wall distance and heading within tolerance for dwell`; `FORWARD_SEGMENT_1=first-flight projected progress reached with fresh wall fit`; `LANDING=landing progress/level region reached and stopped for dwell`; `TURN_TO_NEXT_FLIGHT=configured yaw delta within tolerance for dwell`; `FORWARD_SEGMENT_2=second-flight projected progress reached with fresh wall fit`; `EXIT_CONFIRM=top exit progress and forward clearance within dwell`. Preserve `RosBooleanEvidence` only as test/manual fixture mode. Keep phase/action schema untouched and add no fourth application node. Predicates must not read supervisor feedback or Gazebo model state.
  Parallelization: Wave 2 | Blocked by: 1, 3, 4 | Blocks: 14, 16-19
  References: `supervisor.py:15-23,197-267`; `ros_node.py:28-60,62-179`; `ros_entrypoint.py:70-87`; `StairTraversal.action`; Task 3 capture contract; existing `test_supervisor.py`, `test_supervisor_safety.py`, `test_stair_supervisor_node.py`.
  Acceptance criteria: tests cover stale/missing/invalid scan, odom reset, wall fit, heading/progress thresholds, dwell and all seven named predicates; held-out replay produces independently labeled phases without action feedback; application node inventory remains the three named nodes; existing state-machine tests pass unchanged.
  QA scenarios: happy — replay held-out bag with simulated ROS time and assert ordered evidence transitions; failure — drop scan/odom, rewind timestamp and inject outlier wall points, assert evidence stays false and traversal times out safely. Evidence `<attemptDir>/task-10-tron1-stair-simulation.json`.
  Commit: Y | `feat(stair): derive phase evidence from shared sensors`

- [x] 11. Add a shared right-wall-aware phase command policy while preserving state transitions
  What to do / Must NOT do: Extract phase command generation behind an injected `StairMotionPolicy`. Preserve zero commands in `VERIFY_ENTRY`, `LANDING`, `EXIT_CONFIRM` and existing safe cancellation. For forward segments, combine profile linear speed with bounded right-wall distance/heading correction toward 0.30 m; for ALIGN/TURN use measured heading targets and bounded angular speed. Require fresh Task 10 observations; stale/invalid input commands zero, never open-loop fallback. Keep existing fixed policy available only to legacy unit fixtures.
  Parallelization: Wave 2 | Blocked by: 1, 3, 4 | Blocks: 14, 16-19
  References: `supervisor.py:229-267` current commands; `StairProfile` and `robot.yaml` speed/full-scale limits; prior right-wall objective in draft; Task 3 measured contract; Task 10 sensor snapshot; `test_supervisor.py` command-order assertions.
  Acceptance criteria: pure tests prove sign-correct correction, 0.30 m convergence, saturation, acceleration limits, phase-specific zero, turn heading and stale-input zero; original seven phases/results/ownership epochs remain byte-for-byte equivalent for equivalent evidence; no command exceeds configured physical or WebSocket limits.
  QA scenarios: happy — closed-loop synthetic wall offsets on both sides converge within configured tolerance/dwell; failure — remove wall fit during forward phase and assert immediate zero then timeout/FAULT rather than straight open-loop motion. Evidence `<attemptDir>/task-11-tron1-stair-simulation.json`.
  Commit: Y | `feat(stair): share wall-aware motion policy across modes`

- [x] 12. Compose one fail-closed `run.sh --mode sim|real` runtime with correct clock domains
  What to do / Must NOT do: Replace the imported top-level wrapper with one parser and immutable runtime bundle. SIM sources/builds the pinned overlay, starts `/clock`, measured world, official ONNX controller, sensor contracts, peer and the same three application nodes. REAL preserves existing remote sensor launch, SSH tunnel, production YAML and vendor WebSocket route. Propagate `use_sim_time`; make readiness compare message headers against ROS `/clock` in SIM and synchronized wall time in REAL; never start mixed dependencies or silently fall back to REAL.
  Parallelization: Wave 3 | Blocked by: 1-4, 8-9 | Blocks: 13, 15-19
  References: imported `run.sh:37-47,127-217,252-298`; `config.env`; `mission_manager/launch/system.launch:3-83`; `_configuration_roots()` in `mission_manager/ros_runtime.py`; existing `test/test_system_operator_contract.py`, `test/test_bundle_contract.py`, `test/test_launch_contract.py`.
  Acceptance criteria: `./run.sh --mode sim --check` and `./run.sh --mode real --check` exit 0 without motion; SIM reaches `[READY]` under nonzero `/clock`; REAL generated process/argument snapshot matches imported behavior except explicit new profile plumbing; unknown/missing mode and mixed profile exit before SSH/socket/ROS side effects.
  QA scenarios: happy — run both checks and compare process manifests; failure — freeze `/clock`, set future sensor stamps and request mixed mode, assert each produces a distinct fail-closed diagnostic. Evidence `<attemptDir>/task-12-tron1-stair-simulation.log`.
  Commit: Y | `feat(runtime): switch sim and real through one profile`

- [x] 13. Integrate the simulator peer with the official controller and prove protocol parity
  What to do / Must NOT do: Connect Task 6 peer backend to Task 7 adapter and Task 8 controller readiness. Exercise the real `RobotTransport` over an actual loopback WebSocket; map STAND/WALK/e-stop/shutdown deterministically; keep `request_stair_mode` as emulated application status while controller remains internally WALK. Audit publisher/SDK authority continuously and fault if ownership changes.
  Parallelization: Wave 3 | Blocked by: 5-8, 12 | Blocks: 15-19
  References: Tasks 5-8 outputs; `robot_transport.py`; `robot_client.py`; official controller STAND/WALK enum; `src/mission_manager/test/test_process_boundaries.py`; current three-node contract.
  Acceptance criteria: all golden vectors pass over the integrated peer; identical high-level command trace yields byte-identical WebSocket requests versus fake oracle; captured `/cmd_vel` reflects only documented adapter scaling; stair mode status order is exact while ONNX controller mode remains WALK; authority audit stays at one writer/publisher.
  QA scenarios: happy — complete startup, stair enable, twist sequence, disable and close trace; failure — kill controller, add second `/cmd_vel` publisher and reject one backend command, assert peer returns failure/FAULT and zeros output. Evidence `<attemptDir>/task-13-tron1-stair-simulation.json`.
  Commit: Y | `test(integration): prove simulator transport parity`

- [x] 14. Integrate shared perception/control in both profiles and validate with held-out replay
  What to do / Must NOT do: Wire Tasks 10-11 into the existing stair supervisor through profile-selected injectable components, defaulting to sensor mode for both SIM and REAL. Feed SIM topics from Task 9 and replay physical held-out bags through the same subscribers/clock. Keep Bool fixture mode test-only. Produce a cross-mode phase/evidence/command trace comparer; do not tune on held-out recordings.
  Parallelization: Wave 3 | Blocked by: 3-4, 9-11 | Blocks: 15-19
  References: Tasks 3, 9-11; `ros_entrypoint.py`; `ros_node.py`; `supervisor.py`; existing bag replay/topic tests; corpus split manifest.
  Acceptance criteria: tuning corpus calibrates only declared parameters; held-out corpus independently passes freshness/frame/evidence ordering thresholds; SIM and bag replay expose the same evidence schema and command limits; seven-phase state-machine tests, production launch contract and exact three-node inventory remain green.
  QA scenarios: happy — replay every held-out bag and save confusion/timing/phase trace; failure — attempt to load a held-out bag through the tuner or enable Bool fixture in production profile, assert validator refuses. Evidence `<attemptDir>/task-14-tron1-stair-simulation.json`.
  Commit: Y | `test(stair): validate shared perception and control contracts`

- [x] 15. Establish the clean-build and automated regression pipeline
  What to do / Must NOT do: Add one CI/local gate running import/vendor/schema verification, Python tests, Catkin build/tests, launch contracts, protocol loopback, frame/topic audit and short Gazebo smoke test from a clean state. Capture exact command, exit code, test counts and artifacts; reject grep-only success and stale build products. Ensure all Python modules meet strict typing/lint policy available in the Ubuntu 20.04 environment and pure LOC ceiling.
  Parallelization: Wave 3 | Blocked by: 1-14 | Blocks: 16-19
  References: root `validate_bundle.py`; imported README test commands; all prior task test entrypoints; `.omo` evidence convention; dependency lock.
  Acceptance criteria: `./scripts/verify_all.sh --clean` completes with no failures and emits schema-validated `artifacts/verification/summary.json` containing `{schema_version,release_hash,commands:[{argv,started_at,ended_at,exit_code,stdout_sha256,artifacts}],overall}`; a second clean run is reproducible; the summary proves each command executed and reports artifact hashes; generated/build files remain ignored.
  QA scenarios: happy — clean full run; failure — inject one protocol, schema, pin, TF and test failure independently and assert pipeline identifies the owning gate and exits nonzero. Evidence `<attemptDir>/task-15-tron1-stair-simulation.json`.
  Commit: Y | `ci(verification): gate clean sim-real builds`

- [x] 16. Demonstrate nominal closed-loop 9-step/landing/10-step simulation traversal
  What to do / Must NOT do: Add a deterministic scenario runner that places WF_TRON1A at the measured entry pose, launches the full common application stack, sends the real `/stair_traversal` UP action, records ROS bag/Gazebo state/phase/evidence/commands and evaluates progress against independently frozen site-survey acceptance zones. Freeze scorer code, scenario manifest and hashes before the first run. Tune only declared site/motion parameters within measured uncertainty. Run `isaacgym` first; if it misses the nominal gate, run the locked `isaaclab` profile exactly once against the unchanged scorer/seeds and record both receipts. Do not bypass sensor evidence or teleport after start.
  Parallelization: Wave 4 | Blocked by: 8-15 | Blocks: 17-19
  References: measured world Task 9; integrated stack Tasks 12-14; `StairTraversal.action`; phase order `supervisor.py:15-23`; official policy artifacts; verification strategy nominal gate.
  Acceptance criteria: seeds `0..19` each produce action result `OK`, exact seven-phase order, final pose in the independently measured exit region, no fall/contact violation, no watchdog/ownership fault and final WALK; every run has bag, trace and model-state evidence. If neither locked official profile passes, emit `CAPABILITY_BLOCKED_OFFICIAL_POLICY`, make no success commit/tag, stop Tasks 17-19 and hand back evidence for a separately approved custom-policy plan.
  QA scenarios: happy — execute `./scripts/run_scenarios.py --suite nominal --seeds 0:19`; failure — disable one evidence producer and lower friction outside allowed bounds, assert timeout/fall is detected and cannot be scored OK. Evidence `<attemptDir>/task-16-tron1-stair-simulation.json` plus bags.
  Commit: Y only when the nominal gate passes | `test(scenario): prove nominal stair ascent in simulation`; on `CAPABILITY_BLOCKED_OFFICIAL_POLICY`, commit nothing as success and retain only evidence artifacts for handoff.

- [x] 17. Prove robustness and fail-closed behavior across calibrated perturbations
  What to do / Must NOT do: Before any run, define and hash a fixed 100-case matrix spanning measured friction/geometry tolerance, mass/CoM uncertainty, scan/odom noise, timestamp jitter, WebSocket latency/loss, sensor dropout, controller death, malformed status, cancellation and e-stop. Derive ranges from Task 3 measurements or label them explicit stress-test bounds; treat 95/100 as a release threshold, not an empirical real-world reliability claim. Use seeded cases and an immutable scorer; do not retune or exclude failures after viewing results.
  Parallelization: Wave 4 | Blocked by: 16 | Blocks: 18-19
  References: Tasks 3, 5, 7, 9, 13-16; `test_robot_transport_faults.py`; `test_supervisor_safety.py`; measured capture distributions; verification strategy perturbation gate.
  Acceptance criteria: `./scripts/run_scenarios.py --suite robustness --manifest config/scenarios/robustness.yaml` executes exactly 100 declared cases; at least 95 complete successfully; all failure-injection cases stop/zero within their specified bound and none are scored successful; zero falls/collisions in successful cases; results are immutable JSON with seed/config/artifact hashes.
  QA scenarios: happy — run fixed matrix and independently rescore artifacts; failure — tamper with one result or remove one case, assert scorer rejects incomplete/hash-drifted suite. Evidence `<attemptDir>/task-17-tron1-stair-simulation.json` plus bags.
  Commit: Y | `test(robustness): qualify stair simulation perturbations`

- [x] 18. Package one-command SIM/REAL deployment, update, and rollback
  What to do / Must NOT do: Provide `./run.sh --mode sim|real`, `./scripts/install.sh`, `./scripts/package_release.py`, versioned profile bundle, checksums, operator docs and rollback. The release contains common application code/config only; SIM dependencies are recreated from locks and REAL starts only existing vendor WebSocket route. Add dry-run process manifests and an atomic install/previous-version switch; no firmware image or vendor binary redistribution.
  Parallelization: Wave 4 | Blocked by: 15-17 | Blocks: 19
  References: Tasks 1-4, 12, 15-17; imported `run.sh`; README preflight/production contract; LimX license findings; hardware route `config.env`.
  Acceptance criteria: from a clean release tarball, `install.sh --target <temp>` then `run.sh --mode sim --check` succeeds; `run.sh --mode real --check` generates the expected vendor-only manifest; changing one YAML selector is sufficient to switch modes; checksum corruption aborts; rollback atomically restores the prior verified release and REAL check.
  QA scenarios: happy — install, switch modes, upgrade, rollback in an isolated prefix; failure — corrupt artifact, omit profile and attempt mixed owner mode, assert no process starts. Evidence `<attemptDir>/task-18-tron1-stair-simulation.json`.
  Commit: Y | `feat(deploy): package one-command sim-real releases`

- [x] 19. Execute staged physical promotion through the unchanged vendor controller path
  What to do / Must NOT do: Add `scripts/promote_hardware.py` with strict ordered stages `contract`, `flat-canary`, `stair-up`. Each invocation first verifies release/profile hashes, REAL process manifest, synchronized clocks, ACCID `WF_TRON1A_632`, exclusive vendor WebSocket ownership, then requires `--operator-present` and a fresh typed `--physical-estop-ack <nonce>` only for motion stages. `contract` is motionless and captures a real firmware handshake to cross-check Task 5 vectors; `flat-canary` exercises bounded stand/walk/zero and sensor evidence; `stair-up` sends the same common action/profile and records all evidence. Any failed prerequisite or stage invalidates later receipts, sends the existing zero barrier where connected and invokes Task 18 rollback. Never start ROS ONNX hardware control.
  Parallelization: Wave 4 | Blocked by: 16-18 | Blocks: F1-F4
  References: imported README preflight/hardware gates; `run.sh`; `config.env:ACCID=WF_TRON1A_632`; `robot_transport.py`; `docs/stair-up-commissioning.md`; Task 18 release/rollback; user requirement for physical e-stop.
  Acceptance criteria: `contract` and `flat-canary` produce signed machine-readable receipts; with hardware available and explicit `--operator-present`, three consecutive monitored `stair-up` runs complete exact seven-phase order, action `OK`, final WALK, no communication/fault/e-stop event and matching release/profile hashes; without hardware/operator flag the script exits `PENDING_HARDWARE` and the project must not claim hardware promotion.
  QA scenarios: happy — run stages in order and independently verify receipts/bags; failure — wrong ACCID, second owner, stale scan, missing e-stop acknowledgment and forced disconnect each abort before or during motion with zero barrier and non-promoted status. Evidence `<attemptDir>/task-19-tron1-stair-simulation.json` plus hardware bags.
  Commit: Y | `feat(promotion): gate physical stair deployment`

## Final verification wave
> Runs in parallel after ALL todos. ALL must APPROVE. Surface results and wait for the user's explicit okay before declaring complete.

- [ ] F1. Plan compliance audit
  Verify every Must Have and Must NOT Have against files, process manifests, dependency hashes and Task 1-19 evidence. Reject completion if the original source was changed, a low-level owner is duplicated, a task lacks real command evidence, or hardware promotion is claimed without Task 19 receipts. Output `<attemptDir>/final-F1-plan-compliance.json` with APPROVE/REJECT and cited artifacts.

- [ ] F2. Code quality review
  Run clean unit/rostest/Catkin/lint/type/LOC checks, inspect adapter/perception/controller boundaries and prove no duplicate state machine, giant module, silent fallback, catch-all exception or generated dependency is committed. Re-run `./scripts/verify_all.sh --clean`; output `<attemptDir>/final-F2-code-quality.json`.

- [ ] F3. Real manual QA
  Use the system as an operator would: install a clean release, run SIM nominal and one fault scenario through the public `run.sh`, inspect recorded robot motion/phase/topic/TF evidence, switch to REAL check, and when hardware is available execute Task 19 stages. Do not substitute unit tests or grep. Output `<attemptDir>/final-F3-real-qa.json`; return INCONCLUSIVE rather than APPROVE if physical promotion is required but unavailable.

- [ ] F4. Scope fidelity
  Diff the final repository and runtime process graph against this plan. Confirm ROS1 Noetic/headless scope, exactly three application nodes, official pinned dependencies, no custom RL/ROS2/UI/firmware work, no simulator-only evidence shortcut and no physics-equivalence overclaim. Output `<attemptDir>/final-F4-scope-fidelity.json`.

## Commit strategy
- Task 1 initializes the new repository; Tasks 1-19 each land one atomic Conventional Commit shown in the task.
- Vendor repositories, ONNX files, prebuilt SDK libraries, bags, Catkin build/install output and runtime artifacts stay ignored; only lock manifests, source, small fixtures and checksums are committed.
- Changes to imported common application code and simulation packages live in the same canonical repository so SIM and REAL cannot drift by branch or copy.
- Before every commit run the task-specific test; before Waves 3 and 4 run `./scripts/verify_all.sh --clean`.
- Do not squash away the initial import manifest or dependency-lock commits; they are provenance and rollback boundaries.
- Release tags are created only after F1-F4 approve and Task 19 status is represented honestly as promoted or pending hardware.

## Success criteria
- `/home/m3tron/Desktop/TRON1_Stair_Simulation` is the sole editable sim/real codebase and its import/vendor provenance verifies from a clean checkout.
- `./run.sh --mode sim` and `./run.sh --mode real` resolve complete, mutually exclusive runtime profiles; switching requires no source edit.
- SIM runs WF_TRON1A with pinned official model and ONNX locomotion; REAL runs only the existing vendor WebSocket firmware path.
- The same `StairSupervisor`, sensor evidence, right-wall controller, stair/site profile and action contract execute in both modes; only backend/endpoint/clock/sensor-provider config differs.
- The simulator peer passes the production protocol golden vectors, preserves normalized SI semantics, zeros stale commands and never invents a real ONNX stair mode.
- The measured 9-step/landing/10-step world exposes bag-matched `/scan`, odometry and TF contracts without phase-triggered ground truth.
- All clean-build/unit/rostest/launch/protocol/replay gates pass; nominal simulation is 20/20 and robustness is at least 95/100 with no false-success failure case.
- A versioned release can be installed, switched, checked and rolled back through public commands only.
- Hardware usability is claimed only when Task 19 produces three valid physical stair receipts; otherwise the final status is explicitly `PENDING_HARDWARE`, never “complete on robot.”
- If Task 16 ends `CAPABILITY_BLOCKED_OFFICIAL_POLICY`, the plan is not successful even if all software contract tests pass; robustness, release and hardware promotion remain blocked pending a separately approved locomotion-policy plan.
