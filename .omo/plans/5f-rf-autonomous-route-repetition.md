# 5F-RF Recorded Route Autonomous Repetition - Work Plan

## TL;DR

**What you'll get:** The TRON1 will repeat the recorded 5F-to-rooftop loop and return through the existing `/mission` surface. Flat-floor motion remains closed-loop through map/AMCL/move_base, stair motion uses the robot's documented high-level stair/walk modes, and `stair_supervisor` remains the only WebSocket command sender.

**Why this approach:** Recorded Joy is useful evidence but is not a safe route controller. The source bag has a 225-239 s recorded-input/sensor gap and open-loop joystick replay would accumulate pose error. The route will therefore be represented as surveyed map-frame locations and directed stair transitions, with live localization and explicit stair evidence correcting execution.

**What it will NOT do:** It will not publish recorded Joy to a live command topic, replay a timed joystick trace, use low-level `publishRobotCmd`, use FAST-LIO as navigation localization, allow the physical remote and application to send motion simultaneously, or start the complete route before every smaller hardware gate passes.

**Effort:** Large
**Risk:** High. Network command transport exists and is tested against fakes, but production route YAML is disabled, bidirectional landing data is incomplete, stair phase evidence has no production publisher, and no physical end-to-end run has been proven.

**Relationship to the existing plan:** This plan builds on completed software work in `.omo/plans/tron1-multifloor-mvp.md`. For the 5F-RF objective it supersedes that plan's unchecked hardware/site Tasks 17-20 and corrects two assumptions that proved incomplete: one landing hypothesis cannot support a bidirectional stair, and production stair phase evidence currently has no owner.

## Fixed decisions

- The application is the sole motion-command sender. The physical remote remains powered, neutral, and available only for the manufacturer's emergency-stop procedure during commissioning.
- The user-facing execution surface remains `Mission.action`; no operator or RViz control may publish directly to `/navigation/cmd_vel` or the robot WebSocket.
- The recorded bag is commissioning evidence, not executable control input.
- The 5F and RF occupancy maps are the planar localization authorities. LiDAR-only registration is used only to compare route shape and help survey waypoints.
- FAST-LIO remains a comparison lane and is never consumed by move_base, AMCL, stair evidence, or mission success logic.
- Existing three-node topology is preserved. Any new stair evidence logic is library code inside the installed `stair_supervisor` node, not a fourth ROS node.
- Robot motion remains high-level: `request_stand_mode`, `request_walk_mode`, `request_stair_mode`, and continuous bounded `request_twist` only.
- No automatic retry after a stair, localization, command-transport, or evidence failure. A fault sends zero, aborts the mission, and requires a fresh operator start.

## 상세 설계 설명

### 1. 이 계획에서 말하는 “기록 경로 자율 반복”

이 계획은 리모컨 축 값을 시간순으로 다시 보내는 기능이 아니다. 이전 수동 주행이 지나간 장소를 5F와 RF 지도의 위치, 방향, 연결 관계로 다시 표현하고, 실제 주행 중에는 현재 센서로 위치 오차를 계속 수정한다.

최종 동작은 다음 순서다.

1. 로봇은 5F의 `home_5f`에서 시작한다.
2. `move_base`가 5F 지도와 AMCL 위치를 사용해 `stair_5f_to_rf`까지 이동한다.
3. `stair_supervisor`가 평면 주행 명령을 차단하고 zero command를 전송한 뒤 계단 모드로 전환한다.
4. 계단 상승은 정렬, 첫 번째 계단 구간, 중간참 정지, 회전, 두 번째 계단 구간, 옥상 착지 확인 순으로 진행한다.
5. `multifloor_manager`가 RF 태그, RF 지도, RF 착지 pose, AMCL, TF, scan, costmap을 검증한 뒤에만 RF를 `READY`로 만든다.
6. `move_base`가 RF 지도에서 `roof_loop_sw → roof_loop_nw → roof_loop_ne → roof_loop_se` 순으로 이동한다.
7. 목적지 도착 후 `BuildingPlanner.plan_return()`이 현재 확정 위치에서 `home_5f`까지 새 경로를 계산한다. 왕복 경로를 단순히 뒤집지 않는다.
8. RF 착지점으로 돌아와 계단 하강과 5F 재로컬라이제이션을 수행한다.
9. 5F가 다시 `READY`가 된 뒤 `move_base`가 `home_5f`까지 이동한다.
10. 전체 과정의 bag과 명령 transcript가 정상적으로 finalize된 뒤에만 Mission이 성공한다.

따라서 “같은 경로”의 의미는 같은 조이스틱 값이나 같은 시간표가 아니다. 5F 출발점, 계단, RF 루프 waypoint, 하강, 5F 복귀라는 동일한 공간 경로를 현재 위치 피드백으로 반복한다는 의미다.

### 2. 실제 네트워크 명령이 로봇까지 가는 경로

평면 구간의 명령 흐름은 다음과 같다.

```text
Mission.action goal
  → BuildingPlanner가 directed route 생성
  → RosSegmentExecutor가 NAVIGATION segment 실행
  → NavigationExecutor가 map-frame move_base goal 전송
  → move_base가 /navigation/cmd_vel 생성
  → stair_supervisor가 fresh Twist만 수신
  → RobotTransport가 물리 속도를 정규화
  → WebSocket request_twist 전송
  → TRON1 내장 보행 제어기가 실제 이동
```

현재 `robot.yaml`의 변환 기준은 `linear_mps=0.55`, `angular_radps=1.57`이다. 현재 구현은 전진 속도와 yaw 회전을 각각 정규화하고 lateral `y`는 0으로 유지한다. 이 5F-RF 경로는 우선 전진과 회전만으로 구성하며, 횡이동은 실주행에 필요하다는 현장 증거가 생기기 전에는 추가하지 않는다.

`stair_supervisor`는 이미 `/navigation/cmd_vel`이 오래됐거나 비정상 값이면 zero를 전송하고, 종료 시에도 zero를 반복한다. 이번 작업은 이 경로를 바꾸는 것이 아니라 실제 전송에 성공한 JSON을 `/stair_supervisor/websocket_tx`에 함께 기록해 “어떤 명령이 실제 socket으로 나갔는가”를 사후 검증 가능하게 만든다.

계단에서는 동일한 WebSocket owner가 명령권을 계속 가진다. `move_base` goal을 종료하고 로봇의 정지를 확인한 다음에만 `request_stair_mode`와 계단용 `request_twist`를 전송한다. 따라서 NAV와 STAIR가 동시에 로봇을 움직이는 경로는 존재하지 않는다.

### 3. 왜 SensorJoy를 그대로 보내지 않는가

`/tron/sensor_joy`는 제조사 리모컨에서 관측된 축과 버튼이다. 이것은 로봇이 실제로 받은 WebSocket packet이 아니며 다음 정보가 없다.

- 제조사 내부 deadzone과 gain
- 버튼 조합에 따른 모드 전환과 deadman 조건
- 같은 시점에 로봇 제어기가 적용한 제한값
- 실제 전송 성공 여부와 socket timestamp
- 기록 공백 동안의 명령

또한 이번 bag에는 225-239초 공통 공백과 236.838초 wheel odometry 불연속이 있다. 이 구간을 0으로 채우면 수동 주행과 다른 곳에서 멈추고, 이전 값으로 채우면 관측되지 않은 명령을 만들어낸다. 따라서 raw Joy는 다음 용도로만 사용한다.

- 수동 운전자가 어느 방향을 의도했는지 시각화
- 경로 구간과 회전 시점을 찾는 보조 증거
- 향후 실제 송신 command와 비교할 참고 자료

실제 자율 주행 입력은 `locations.yaml`, `building_graph.yaml`, stair profile, 현재 AMCL/odom/IMU/tag evidence다.

### 4. route와 configuration이 표현해야 하는 것

5F-RF route는 최소한 다음 location을 가져야 한다.

| Location ID | Floor | 의미 |
| --- | --- | --- |
| `home_5f` | 5F | Mission 시작점과 최종 복귀점 |
| `stair_5f_to_rf` | 5F | 상승 전 정렬을 끝내는 stair entry |
| `stair_5f_from_rf` | 5F | 하강 후 5F map에서 사용할 landing anchor |
| `stair_rf_landing` | RF | 상승 후 RF map에서 사용할 landing anchor |
| `stair_rf_to_5f` | RF | 하강을 시작하기 전 정렬을 끝내는 stair entry |
| `roof_loop_sw` | RF | 옥상 루프 남서 waypoint |
| `roof_loop_nw` | RF | 옥상 루프 북서 waypoint |
| `roof_loop_ne` | RF | 옥상 루프 북동 waypoint |
| `roof_loop_se` | RF | 옥상 루프 남동 목적지 |

각 location은 해당 floor map 기준의 `x`, `y`, `yaw`를 가진다. bag의 LiDAR-only trajectory에서 얻은 좌표를 그대로 복사하지 않는다. 최소 두 개의 분리된 현장 anchor로 bag trajectory와 floor map의 강체 변환을 구하고, live map에서 로봇을 정지시켜 최종 pose를 확인한다.

directed graph는 다음 두 방향을 모두 명시한다.

```text
UP:
home_5f
  → NAV stair_5f_to_rf
  → STAIR_UP stair_rf_landing
  → FLOOR_TRANSITION RF
  → NAV roof_loop_sw → roof_loop_nw → roof_loop_ne → roof_loop_se

DOWN:
roof_loop_se
  → NAV stair_rf_to_5f
  → STAIR_DOWN stair_5f_from_rf
  → FLOOR_TRANSITION 5F
  → NAV home_5f
```

서로 다른 floor의 location을 `NAV` edge로 직접 연결하면 안 된다. `STAIR_UP` 또는 `STAIR_DOWN` edge 하나는 runtime에서 `STAIR` segment와 `FLOOR_TRANSITION` segment 두 개로 확장된다.

### 5. 양방향 계단 endpoint schema

현재 stair 설정은 landing pose와 expected tag set이 하나뿐이라 상승과 하강의 서로 다른 도착 floor를 표현할 수 없다. 새 schema는 stair 하나에 5F endpoint와 RF endpoint를 각각 둔다.

각 endpoint는 다음 값을 가진다.

- 해당 floor의 entry location ID
- 계단 통과 직후 AMCL에 넣을 landing pose
- 6x6 pose covariance 36개 값
- 해당 floor landing에서 관측해야 할 AprilTag ID
- 연결된 direction-specific stair profile

`stair_5f_rf`의 RF endpoint는 tag 600, 5F endpoint는 tag 501을 사용하되 실제 detector가 floor, family, size를 확인하기 전까지 production을 `configured: true`로 바꾸지 않는다. 상승은 RF endpoint를 선택하고 하강은 5F endpoint를 선택한다.

### 6. 계단 phase evidence의 실제 판정

현재 `RosBooleanEvidence`가 기다리는 Bool topic에는 production publisher가 없다. 이 상태에서는 action이 실제 계단을 진행하지 못하고 timeout된다. 새 `stair_evidence.py`는 별도 node가 아니라 `stair_supervisor` 내부의 순수 상태 추적기다.

| Phase | 로봇 명령 | 다음 phase로 넘어가는 최소 evidence |
| --- | --- | --- |
| `VERIFY_ENTRY` | zero | odom과 IMU가 fresh이고 로봇이 정지 상태 |
| `ALIGN` | 설정된 방향으로 저속 회전 | phase 시작 yaw 대비 signed yaw 변화가 목표 범위에 도달 |
| `FORWARD_SEGMENT_1` | 저속 전진 | signed 이동거리 도달과 IMU pitch의 slope 진입 후 level 복귀가 모두 발생 |
| `LANDING` | zero | fresh odom 기준 정지와 IMU level 상태가 설정 dwell 동안 유지 |
| `TURN_TO_NEXT_FLIGHT` | 저속 회전 | 중간참 시작 yaw 대비 signed turn 목표 도달 |
| `FORWARD_SEGMENT_2` | 저속 전진 | 두 번째 signed 이동거리와 두 번째 slope cycle 확인 |
| `EXIT_CONFIRM` | zero | fresh sensor, level, stationary가 exit dwell 동안 유지 |

“slope cycle”은 pitch가 평지 범위를 벗어나 계단 경사를 관측한 뒤 다시 평지 범위로 돌아오는 순서다. 이동거리만으로 통과시키지 않는 이유는 wheel odometry jump가 목표 거리를 한 번에 넘겨 거짓 성공을 만들 수 있기 때문이다. IMU만으로 통과시키지 않는 이유는 로봇이 제자리에서 기울거나 흔들린 것을 계단 통과로 오인할 수 있기 때문이다.

다음 조건은 성공 evidence가 아니라 즉시 fault 사유다.

- NaN 또는 무한대 센서 값
- profile의 freshness보다 오래된 odom 또는 IMU
- 허용 sample gap 초과
- 한 sample에서 허용 범위를 넘는 odometry 이동 또는 yaw jump
- 명령 방향과 반대되는 진행이 tolerance를 초과
- 전체 phase/profile timeout
- WebSocket send/receive 실패

fault가 발생하면 현재 phase를 보존해 자동 재시작하지 않는다. zero 전송, transport close 또는 FAULT latch, Mission abort 순으로 끝낸다.

action cancellation은 transport fault와 구분한다. 현재 안전 checkpoint는 `VERIFY_ENTRY`, `LANDING`, `EXIT_CONFIRM`이다. cancellation은 즉시 latch하지만 `ALIGN` 또는 `FORWARD_SEGMENT_1` 중에는 `LANDING`까지, `TURN_TO_NEXT_FLIGHT` 또는 `FORWARD_SEGMENT_2` 중에는 `EXIT_CONFIRM`까지 진행한 뒤 zero barrier와 PREEMPTED로 끝낸다. 일반 cancel은 계단 flight 중간에서 즉시 zero를 보내지 않는다. 즉시 정지가 필요한 위험 상황에서는 물리 e-stop을 사용한다. 물리 e-stop은 현재 WebSocket status로 자동 감지된다고 가정하지 않으므로, operator는 앱 Mission을 취소하고 command process가 종료됐으며 TX가 zero임을 확인하기 전에는 e-stop을 해제하지 않는다.

### 7. floor transition과 재로컬라이제이션

계단 action이 끝났다고 바로 평면 주행을 재개하지 않는다. `FloorTransition.action`은 target floor에 대해 다음 순서를 모두 통과해야 한다.

1. target floor에 연결된 expected tag를 fresh detection으로 확인한다.
2. `/change_map`으로 target floor map을 로드한다.
3. 이전 map과 다른 fingerprint와 증가한 `map_generation`을 확인한다.
4. direction-specific landing pose와 covariance를 `/initialpose`로 보낸다.
5. `/request_nomotion_update` 이후의 새 AMCL sample만 받는다.
6. AMCL covariance, fresh `/scan`, stationary odom, scan timestamp의 `map→odom→base_Link` TF를 확인한다.
7. move_base costmap을 clear하고 target map과 dimensions, origin, resolution이 일치하는지 확인한다.
8. 모두 통과한 뒤에만 `FloorState.READY`와 새 generation을 publish한다.

tag가 보였다는 사실만으로 floor를 확정하지 않으며, map load 성공 응답만으로도 성공하지 않는다. AMCL과 costmap까지 target floor에 수렴해야 다음 NAV goal이 전송된다.

### 8. command와 주행 evidence 기록

최종 `record_route` artifact는 최소한 다음 질문에 답할 수 있어야 한다.

| 질문 | 필요한 evidence |
| --- | --- |
| 누가 로봇 명령을 보냈는가? | node inventory, WebSocket owner count, `/stair_supervisor/websocket_tx` |
| move_base가 무엇을 요청했는가? | `/navigation/cmd_vel`, move_base goal/status/plan |
| 실제 socket으로 무엇이 나갔는가? | 성공한 serialized JSON frame과 ROS bag timestamp |
| 계단에서 어느 phase였는가? | StairTraversal feedback, supervisor state, odom, IMU |
| 어느 floor map을 사용했는가? | FloorState, map generation, map fingerprint |
| 위치 추정이 유효했는가? | `/amcl_pose`, `/scan`, `/tf`, costmap metadata |
| 실제로 움직이고 멈췄는가? | wheel odom, IMU, external synchronized video |
| 오류 후 명령이 재개됐는가? | fault 이후 WebSocket TX transcript와 action terminal result |

WebSocket TX topic은 socket send가 성공한 frame만 기록한다. send 실패는 성공 frame처럼 기록하지 않고 transport fault로 남긴다. 따라서 source command와 TX frame 수가 다르면 그 차이는 조사 대상이며, 자동으로 누락을 보간하지 않는다.

### 9. fail-closed 동작표

| 실패 | 즉시 동작 | Mission 결과 | 자동 재개 |
| --- | --- | --- | --- |
| `/navigation/cmd_vel` stale | zero 전송 | 상황에 따라 NAV 실패 또는 대기 | 없음 |
| 비정상/nonfinite Twist | 전체 command zero | 입력이 회복돼도 새 fresh command 필요 | 과거 값 재사용 없음 |
| WebSocket disconnect/timeout | zero 시도 후 transport FAULT | `COMMUNICATION_LOST` 또는 Mission abort | 없음 |
| stair evidence stale/jump/timeout | stair command zero, STAIR FAULT | stair failure 또는 Mission abort | 없음 |
| wrong/stale landing tag | map 전환 금지 | localization failure | 없음 |
| AMCL/TF/scan/costmap 불일치 | FloorState를 READY로 만들지 않음 | localization failure | 없음 |
| Mission cancel | active child cancel, zero barrier, recorder finalize | PREEMPTED/cancelled | 없음 |
| 물리 remote emergency stop | 즉시 물리 정지 후 operator가 Mission 취소와 command process 종료 수행 | 앱 종료와 TX zero를 별도 확인 | 앱 세션 종료 확인 전 해제 금지 |

### 10. 단계별 구현 산출물

| Task | 코드 또는 설정 결과 | 자동 검증 결과 | 현장 결과 |
| --- | --- | --- | --- |
| 1 | TX observer와 recording topic, replay 격리 강화 | fake socket transcript와 forbidden-remap tests | 없음 |
| 2 | 원본을 변경하지 않은 route evidence package | gap/jump provenance 검증 | 지도 anchor 확인 자료 |
| 3 | 양방향 stair endpoint schema와 graph validation | up/down route expansion tests | 없음 |
| 4 | in-process stair evidence tracker | phase/fault/rosbag replay tests | 없음 |
| 5 | 검토된 5F/RF production YAML | validator와 preflight | 실제 pose/tag/profile 측정 |
| 6 | production-shaped mock mission | full happy/failure matrix | 없음 |
| 7 | 변경 없음, flat command commissioning | 사전 software gate 재확인 | 전진, 회전, zero, watchdog, e-stop 증거 |
| 8 | 검증된 map/nav tuning | localization failure injection | 5F와 RF waypoint 반복 성공 |
| 9 | 동결된 UP/DOWN stair profiles | profile schema와 evidence regression | 독립 ascent/descent validation |
| 10 | 완성된 route profile과 운영 문서 | 최종 software suite | 실제 왕복 Mission bag, video, result |

### 11. production 값 확정 원칙

테스트 fixture의 숫자는 알고리즘 경계를 검증하는 합성값이며 production에 복사하지 않는다. production 값은 다음 절차로만 확정한다.

1. 현장 tuning run에서 후보 값을 측정한다.
2. 후보 값을 YAML에 기록하고 profile version과 측정 artifact checksum을 남긴다.
3. tuning에 사용한 run과 분리된 validation run을 수행한다.
4. validation run이 합격하면 값을 동결한다.
5. 동결 후 값 변경은 해당 hardware gate부터 다시 수행한다.

안전한 초기 속도, distance tolerance, yaw tolerance, pitch threshold, dwell은 이 계획에서 임의 숫자로 만들지 않는다. 값이 비어 있거나 `configured: false`이면 preflight가 로봇 연결 전에 중단해야 한다.

## Scope

### Must have

- A directed 5F → RF roof-loop → 5F route with separately surveyed ascent and descent entry/landing poses.
- Closed-loop flat-floor waypoint execution through the current `move_base` action path.
- Sensor-derived stair phase evidence with freshness, continuity, distance, yaw, slope-cycle, stationary-dwell, and timeout gates.
- Direction-specific landing hypotheses, covariance, and expected AprilTag IDs for both 5F and RF.
- Exact recording of successful outbound WebSocket JSON frames alongside source `/navigation/cmd_vel`, state, localization, and sensor evidence.
- Progressive physical gates: zero/network, short flat motion, 5F navigation, RF navigation, stair ascent, stair descent, then the complete mission.
- A final `record_route` Mission goal to location ID `roof_loop_se` whose artifact proves route, return, command ownership, stop behavior, map-load generations, and localization readiness.

### Must not have

- No live use of `./run.sh --replay-joy`; it remains restricted to `/replay/tron/sensor_joy` and at most 30 seconds.
- No conversion of the existing 508 s Joy stream into an open-loop command script.
- No interpolation over the 216.221-236.647 RGB outage or the 225-239 s common sensor gap.
- No use of the 236.838 s wheel-odometry discontinuity as valid distance or heading evidence.
- No fabricated production coordinates, covariances, tag observations, stair distances, speed limits, or pass results.
- No command mux, reconnect-and-resume behavior, new runtime node, or alternate WebSocket owner.
- No claim of success from unit tests, RViz, or logs alone. The matching physical Mission surface must complete under observation.

## Current blockers this plan resolves

1. `src/mission_manager/config/locations.yaml`, `building_graph.yaml`, `src/multifloor_manager/config/{floors,stairs,transitions}.yaml`, and `src/stair_supervisor/config/stair_profiles.yaml` remain `configured: false`.
2. `building_graph.yaml` has no `STAIR_UP` edge from 5F to RF and incorrectly represents RF-to-5F movement as a `NAV` edge across floor IDs.
3. The current stair schema stores one `target_landing` and one expected-tag set, which cannot represent both ascent and descent landing hypotheses.
4. `RosBooleanEvidence` subscribes to phase Bool topics, but the production graph contains no publishers for those topics. A real stair action would wait until timeout.
5. The current bag has raw Joy but no exact WebSocket TX log. Actual command provenance cannot be reconstructed after the fact.
6. The existing packet/transport tests use fake WebSockets; physical STAND/WALK/twist/watchdog/emergency-stop behavior has not been demonstrated on this robot.

## Verification strategy

- Automated gates use focused `unittest`/`rostest`, `catkin_make`, `catkin_make run_tests`, `catkin_test_results --verbose`, `./run.sh --check`, and `./run.sh --preflight`.
- Pure tests use clearly synthetic thresholds chosen only to exercise boundary behavior. Production distance, yaw, pitch, dwell, and speed values come only from hardware commissioning and are frozen before independent validation runs.
- Hardware evidence is recorded as rosbag plus WebSocket TX transcript, action feedback/result, parameter snapshot, and synchronized video. A human safety operator must be physically present because the artifact is a moving robot; this requirement is not replaced by software automation.
- Every hardware gate starts from zero command with the remote joysticks neutral, confirms the emergency stop is reachable, runs one bounded behavior, returns to zero, and inspects evidence before enabling the next gate.
- A failed gate blocks all dependent gates. Thresholds are not relaxed merely to obtain a pass; the failing sensor, map, profile, or transport behavior is corrected and the same gate is repeated.

## Execution order

### 1. Lock the non-replay safety contract and outbound-command evidence

**Implementation**
- Keep `replay_joy_preview.sh` and `run.sh --replay-joy` isolated on `/replay/tron/sensor_joy`; strengthen contract tests so neither can remap to `/tron/sensor_joy`, `/navigation/cmd_vel`, `/stair/cmd_vel`, or a WebSocket surface.
- Add an observer seam to `src/stair_supervisor/src/stair_supervisor/robot_client.py`/`robot_transport.py` that receives the exact serialized request only after `send()` succeeds.
- In the existing `stair_supervisor` node, publish successful frames on `/stair_supervisor/websocket_tx` as `std_msgs/String`. Do not put ROS imports in the transport library.
- Ensure shutdown, watchdog, and send-failure paths cannot publish a nonzero frame after fault latch. Preserve GUID correlation and no-reconnect behavior.
- Add `/navigation/cmd_vel`, `/stair_supervisor/websocket_tx`, `/stair_supervisor/state`, `/mission/feedback`, `/mission/result`, `/multifloor/floor_state`, `/amcl_pose`, `/tf`, `/tf_static`, `/tron/wheel_odom_raw`, `/tron/imu`, `/scan`, raw LiDAR, and RGB to the final route recording profile.

**References**
- `run.sh:8`, `run.sh:32`
- `replay_joy_preview.sh`
- `src/stair_supervisor/src/stair_supervisor/robot_transport.py:61`
- `src/stair_supervisor/src/stair_supervisor/ros_node.py:63`
- `manual_mission_capture.py:56`

**Acceptance**
- Tests prove preview Joy cannot reach any live command topic.
- A fake-WebSocket rostest observes byte-equivalent JSON on `/stair_supervisor/websocket_tx` for every successful send, including zero frames, and observes no false success record for a failed send.
- Exactly one installed node owns the WebSocket and no fourth project node appears.

### 2. Produce a route-survey evidence package from the recorded drive

**Implementation**
- Treat `/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/` as an explicit prerequisite. At task start, verify the directory and required scripts exist, run the workflow's documented validation commands, and record checksums of the scripts used in the evidence manifest. If the prerequisite is absent or its validation fails, block Task 2; do not substitute a new registration stack inside this project.
- Reuse `/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/` to process `manual-5F-rooftop_route_1788920816079496577.bag.active.invalid` without modifying the original bag.
- Segment evidence into 5F planar, stair, and RF planar intervals. Preserve the common sensor gap and wheel-odometry discontinuity as rejected intervals rather than bridging them.
- Export the LiDAR-only relative path, normalized wheel path, controller-intent samples, gap markers, RGB event frames, and a manifest that labels every field as measured, derived, inferred, or unavailable.
- Overlay each planar segment on its occupancy map using surveyed correspondences. Require at least two spatially separated correspondences per floor and retain transform residuals; do not silently warp individual route segments.
- Use the overlay to propose, not automatically activate, map-frame poses for `home_5f`, 5F stair entry/down landing, RF stair landing/down entry, and roof-loop southwest/northwest/northeast/southeast waypoints.

**References**
- Source bag under `manual_captures/`
- `/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/process_bag.py`
- `/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/build_comparison_bag.py`
- `src/multifloor_manager/config/maps/floor_5F.yaml`
- `src/multifloor_manager/config/maps/floor_RF.yaml`

**Acceptance**
- The evidence manifest names exact valid/rejected intervals and does not present Joy intent as transmitted command.
- 5F and RF overlays render in RViz with residuals and surveyed anchor IDs.
- No production YAML changes until every proposed pose is checked on the live map with the robot stationary.

### 3. Correct the bidirectional stair and route configuration model

**Implementation**
- Replace the unreleased single `entry_location_id`/`target_landing`/`expected_tag_ids` stair shape with two endpoint records keyed by floor ID. Each endpoint contains its entry location ID, landing pose, 36-value covariance, and expected landing tag IDs. Require exactly the stair's `from_floor` and `to_floor` keys.
- Update typed loaders, cross-reference validation, fixtures, and route expansion so ascent selects the RF endpoint and descent selects the 5F endpoint.
- Reject a graph that represents a cross-floor `NAV` edge, a target floor without an endpoint hypothesis, a landing tag belonging to another floor/stair, or a missing direction-specific profile.
- Keep `FloorTransition.action` unchanged; resolve the target-floor endpoint before sending `/initialpose` and evaluating expected tags.

**References**
- `src/multifloor_manager/src/multifloor_manager/configuration.py:65`
- `src/mission_manager/src/mission_manager/site_config.py`
- `src/mission_manager/src/mission_manager/route_planner.py`
- `src/multifloor_manager/src/multifloor_manager/ros_node.py`
- `test/fixtures/building_valid/stairs.yaml`

**Acceptance**
- Unit tests cover valid up/down endpoint selection and reject every mismatched floor, pose, covariance, location, tag, and profile reference.
- An asymmetric fixture plans both 5F→RF and RF→5F with `STAIR` then `FLOOR_TRANSITION`; no cross-floor `NAV` segment is emitted.
- Existing generated action/message contracts remain unchanged.

### 4. Replace unowned Bool topics with in-process stair phase evidence

**Implementation**
- Add a pure `stair_evidence.py` library inside `stair_supervisor`; do not add a node. It consumes timestamped odometry and IMU snapshots supplied by `RosStairSupervisorNode`.
- Extend each direction-specific `StairProfile` with surveyed phase parameters: alignment yaw, flight-1 signed distance, landing level/stationary dwell, landing turn yaw, flight-2 signed distance, exit level/stationary dwell, distance/yaw tolerances, pitch enter/exit thresholds, sensor freshness, maximum sample gap, maximum odometry step, and overall timeout.
- Advance a flight only after both signed distance progress and a complete IMU slope cycle are observed. Advance landing/exit only after fresh level and stationary evidence holds for the configured dwell. Advance the landing turn only after signed yaw reaches its bound.
- Reset all relative baselines at action start and each phase boundary. Reject nonfinite data, stale samples, backwards progress beyond tolerance, excessive sample gaps, or odometry jumps by latching FAULT and sending zero.
- Remove production dependence on external phase Bool topics. Retain a fake evidence implementation only as a test seam for pure supervisor tests.
- Publish phase/detail evidence through the existing StairTraversal feedback and supervisor state, including current progress, threshold, freshness, and rejection reason.

**References**
- `src/stair_supervisor/src/stair_supervisor/ros_node.py:38`
- `src/stair_supervisor/src/stair_supervisor/supervisor.py:15`
- `src/stair_supervisor/src/stair_supervisor/configuration.py`
- `src/stair_supervisor/config/stair_profiles.yaml`
- Source bag's 236.838 s odometry discontinuity as a mandatory negative fixture

**Acceptance**
- Pure tests prove every phase, both directions, angle wraparound, slope-cycle requirement, dwell reset, stale/nonfinite input, sample gap, odometry jump, timeout, cancellation, and zero/fault behavior.
- A rosbag replay test advances on valid synthetic sensor sequences and aborts on the recorded discontinuity/gap sequence without sending a later nonzero command.
- Production launch has no unresolved `/stair/evidence/*` subscription.

### 5. Provision only the reviewed 5F-RF route data

**Implementation**
- Survey and add `home_5f`, `stair_5f_to_rf`, `stair_5f_from_rf`, `stair_rf_landing`, `stair_rf_to_5f`, and the four roof-loop waypoints to `locations.yaml`. Keep landing and descent-entry IDs separate even if the survey confirms identical poses because they have different route semantics.
- Replace the invalid RF-to-5F `NAV` edge with explicit `STAIR_UP` and `STAIR_DOWN` edges referencing `stair_5f_rf`; retain directed roof-loop edges and add only the NAV edges needed to reach/leave the stair endpoints.
- Populate `stairs.yaml` with both endpoint hypotheses and tags 501/600 only after detector evidence confirms their installed floor, family, and size.
- Bind tag 501 to the surveyed 5F landing endpoint and tag 600 to the surveyed RF landing endpoint, matching `apriltags.yaml`; detector evidence must still confirm both before activation.
- Populate separate `stair_5f_rf_up` and `stair_5f_rf_down` profiles from gated measurements. Fixture values are never copied into production.
- Enable the standard transition policy only after expected-tag, map fingerprint, AMCL covariance/freshness, TF, scan, stationary, and costmap gates pass on both target floors.
- Set the deployment start to `INITIAL_FLOOR=5F`, `HOME_LOCATION_ID=home_5f`, and `INITIAL_LOCATION_ID=home_5f` for this route profile. Keep unrelated 3F/4F routes out of the activated graph unless independently commissioned.

**References**
- `src/mission_manager/config/locations.yaml`
- `src/mission_manager/config/building_graph.yaml`
- `src/multifloor_manager/config/{floors,stairs,apriltags,apriltag_ros_tags,transitions}.yaml`
- `src/stair_supervisor/config/{robot,stair_profiles}.yaml`
- `config.env`

**Acceptance**
- `python3 validate_bundle.py --validate-production-config` and `./run.sh --preflight` pass with the reviewed 5F/RF profile.
- Planner tests produce this exact logical sequence: 5F NAV → stair up → RF transition → RF loop NAV segments → fresh return plan → stair down → 5F transition → home NAV.
- `roof_loop_se` is the destination location ID for that sequence. “Fresh return plan” means the planner runs a new graph search from the confirmed destination anchor rather than reversing or reusing the outbound segments.
- Removing either target-floor landing hypothesis, tag, reverse edge, map, or profile fails preflight before the SSH tunnel or robot connection starts.

### 6. Prove the complete behavior without robot motion

**Implementation**
- Extend fake WebSocket/action/service/sensor peers to execute the exact 5F-RF production-shaped route through `Mission.action` while using non-production fixture values.
- Record and inspect one mock `record_route` artifact. Assert it contains navigation commands, successful WebSocket frames, floor states/generations, stair phases, AMCL/TF/scan evidence, and the terminal Mission result.
- Inject navigation abort, stale localization, wrong-floor tag, odometry jump, IMU slope absence, WebSocket loss, cancellation, and scan-record failure one at a time.

**Acceptance**
- `catkin_make`, focused unit tests, `catkin_make run_tests`, `catkin_test_results --verbose`, and `./run.sh --check` exit successfully, apart from any explicitly documented pre-existing unrelated test inventory failures.
- The happy mock mission reaches home with a fresh return plan.
- Every injected failure leaves a zero command transcript, no active child goal, no unintended next segment, and a coherent action result/status.

### 7. Hardware gate A: prove network motion and stop on flat 5F

**Procedure**
- Clear the area, place a safety operator at the remote emergency stop, keep both joysticks neutral, and run `./run.sh --preflight` before opening the command session.
- At zero velocity, verify connect → repeated zero → STAND → WALK status and exactly one WebSocket sender.
- Through a bounded test-only Mission fixture, execute one short forward command, one short yaw command, and zero. Repeat with input stopped to trigger watchdog zero, then repeat with an intentional WebSocket interruption.
- Compare `/navigation/cmd_vel`, exact `/stair_supervisor/websocket_tx`, wheel odometry, IMU, video, and physical displacement/direction.

**Pass gate**
- Forward and yaw signs are correct, observed speed stays within commissioned limits, stop/watchdog/connection-loss each produce and maintain zero, and no nonzero frame appears after FAULT.
- Emergency stop physically halts the robot. Before release, cancel the Mission, stop the command process, and verify the TX stream is zero/closed; then confirm a fresh application start is required before any new motion.
- Any sign, scale, mode, or stop discrepancy blocks all later motion.

### 8. Hardware gate B: prove closed-loop navigation separately on 5F and RF

**Procedure**
- On 5F, initialize AMCL at surveyed `home_5f`, verify scan/TF/covariance/costmaps, and send one short named-location Mission goal that does not approach the stair.
- On RF, repeat from the surveyed landing with each roof-loop waypoint as a separate goal before attempting the complete directed loop.
- Record path tracking, planner commands, command frames, AMCL covariance, clearances, recovery behavior, and final pose error. Do not tune using the rejected bag intervals.

**Pass gate**
- Every waypoint succeeds repeatedly within the surveyed clearance and pose-error tolerance, localization remains fresh, and loss of scan/TF/localization stops or aborts before unsafe continuation.
- The directed RF loop succeeds as flat-floor navigation before any stair is included.

### 9. Hardware gate C: commission ascent and descent one phase at a time

**Procedure**
- Verify `request_stair_mode` enter/exit at zero before movement.
- Run ascent phases in order with conservative speeds: entry verification, alignment, first flight, landing dwell, turn, second flight, exit dwell. Inspect sensor evidence and stop after each phase during initial commissioning.
- Store measured distances, yaw, pitch-cycle bounds, dwell, sample continuity, and timeouts in the UP profile only after repeated agreement.
- Repeat independently for descent and its DOWN profile. Never derive DOWN by negating UP values.
- Freeze each profile after tuning runs, then require separate ascent and descent validation runs whose data were not used to choose the thresholds.
- After phase gates pass, execute one complete ascent action and one complete descent action. Verify target-floor tag, map switch, stored target landing pose, AMCL readiness, costmap match, and return to NAV ownership.

**Pass gate**
- Each direction completes repeatedly with expected phase evidence and target-floor localization.
- Missing slope cycle, stale IMU/odom, excessive odometry step, wrong/occluded tag, cancellation, timeout, WebSocket loss, or emergency stop produces zero and aborts without retry/resume.
- A single failed direction remains disabled in production even if the opposite direction passes.

### 10. Execute the recorded-route equivalent through Mission.action

**Procedure**
- Start at surveyed `home_5f` with FloorState READY and SupervisorState NAV.
- Submit a `record_route` Mission goal to `roof_loop_se` with `return_after_task=true`. The directed graph must force the RF loop before the destination and independently plan the return through the RF landing and down stair.
- Observe action feedback, RViz map/plan/pose, stair phases, floor generations, exact WebSocket frames, and the recording process. Use only the Mission cancel surface for a controlled cancellation run.
- Compare the completed path to the recorded route's LiDAR-only shape after aligning each floor independently. Report geometric difference; do not require packet or timestamp identity with the remote-controlled run.

**Final acceptance**
- The physical robot reaches `roof_loop_se`, returns to `home_5f`, and the Mission action terminates SUCCESS with no manual motion command during the run.
- The artifact contains nonzero data for every mandatory command/state/localization/sensor topic and finalizes without `.active`/`.invalid` status.
- Packet evidence shows `stair_supervisor` as the only sender, explicit zero-command barriers before and after every NAV/STAIR ownership or mode transition, and no command gap that is interpreted as continued motion.
- Floor transition evidence shows RF and then 5F target tags, new map generations, fresh AMCL/TF/scan, matching costmaps, and NAV resumption only after READY.
- A separate controlled cancellation stops, finalizes evidence, and does not launch the next route segment.

## TODOs

- [x] 1. Lock the non-replay safety contract and record successful outbound WebSocket commands.
- [x] 2. Produce a provenance-preserving route-survey evidence package from the recorded drive.
- [x] 3. Correct the bidirectional stair endpoint and directed-route configuration model.
- [x] 4. Replace unowned stair Bool topics with in-process odometry/IMU phase evidence.
- [ ] 5. Provision reviewed 5F-RF production route, map, tag, transition, and stair data.
- [ ] 6. Prove the complete 5F-RF mission and failure matrix without robot motion.
- [ ] 7. Hardware gate A: prove bounded network motion, watchdog, disconnect, and emergency-stop behavior on flat 5F.
- [ ] 8. Hardware gate B: prove closed-loop waypoint navigation independently on 5F and RF.
- [ ] 9. Hardware gate C: commission and independently validate stair ascent and descent.
- [ ] 10. Execute, record, and review the complete 5F-RF round-trip through Mission.action.

## Dependency order

| Task | Depends on | Blocks |
| --- | --- | --- |
| 1 | none | 6-10 |
| 2 | none | 5, 8-10 |
| 3 | none | 5, 6, 9, 10 |
| 4 | 3 | 6, 9, 10 |
| 5 | 2, 3, 4 | 6, 8-10 |
| 6 | 1, 3-5 | 7-10 |
| 7 | 1, 6 | 8-10 |
| 8 | 2, 5-7 | 10 |
| 9 | 3-8 | 10 |
| 10 | 1-9 | final review |

Tasks 1-3 may begin in parallel where file ownership does not overlap. Task 4 starts after Task 3; hardware gates 7-10 are strictly sequential.

## Final review

- Re-read the user's priority: actual autonomous repetition of the recorded 5F-rooftop route, not visualization alone and not live teleoperation.
- Audit the installed graph for exactly three project nodes and one WebSocket owner.
- Map every failure gate to observed zero-command evidence and a terminal action status.
- Verify production values came from recorded measurements or on-site survey, never fixtures or inference across outages.
- Review the final bag and synchronized video through the actual Mission surface. Automated tests and RViz-only replay cannot substitute for this gate.
- Treat resemblance to the source bag as diagnostic only because that bag also informed the waypoint proposal. Independent acceptance comes from surveyed map-frame endpoint error, live localization evidence, external video, and profile-validation runs recorded after tuning values were frozen.

## Success criteria

- The application, not the physical remote, owns all normal motion during execution.
- The robot autonomously repeats the route semantically: 5F start, stair ascent, RF directed loop, stair descent, and 5F home return.
- Flat-floor position error is corrected online by AMCL/move_base rather than replay timing.
- Stair progress is gated by fresh physical evidence and target-floor localization, not elapsed time alone.
- Every command sent to the robot is captured with its corresponding Mission, floor, supervisor, localization, and sensor evidence.
- All stop/fault paths are fail-closed and require a fresh start; no unintended motion resumes.
