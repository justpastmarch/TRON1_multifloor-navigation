# Phase C 재개 기록

기준은 현재 worktree이며 소스 경로는 coverage.json의 repository 기준이다. 이전 phase-C-partial.md를 대체하지 않고 추가 검증·정정을 기록한다. 전체 감사는 미완료다.

## 열람·검증

OBSERVED: configuration/site loader, scan recorder, robot conversion/protocol, production graph/location/floor/tag/stair/recording/navigation YAML, action/msg/srv를 전체 열람했다. 이번 재개 시점의 READ 91개에 이어 test_fsm.py와 test_stair_entry_gate.py도 전체 열람했다. 나머지 테스트의 실행 안전성은 아직 검토 중이다.

ROS에 접속하지 않는 순수 Python 로직 probe 결과는 pure-C-probe.json에 보존했다. production 설정 로드와 실제 planner/evidence/transition/FSM 코드를 사용했고 이동·녹화 peer는 fake였다. 테스트 oracle의 물리적 독립성은 없다. 따라서 논리 반례의 OBSERVED이지 hardware/E2E 성공·실패 관찰이 아니다. 감사자가 생성한 bag, ROS process, cache는 없다.

- C-04 보강: 0.05초마다 0.02m씩 이동하는 odometry를 입력하면 5초 동안 2.02m 이동하고도 LANDING이 stationary dwell로 완료된다. 실제 slip/추락 발생은 UNVERIFIED. Safety S4 / Mobility M2를 유지한다.
- C-05 보강: 층 확인 이후 11초에 다른 최종 조건은 새로 입력해도 T_FLOOR_CONFIRMED만 stale로 실패했다. Safety S1 / Mobility M2.
- C-06 보강: 성공 NAV 직후 recorder.assert_active 오류를 주입하면 NEXT_SEGMENT에서 SEGMENT_FAILED 예외가 발생하며 outcome을 반환하지 않는다. anchor 손실 경로를 뒷받침한다. Safety S1 / Mobility M2.
- 반증: ScanRecorder 생성자는 경로를 resolve하고 runtime을 저장한다. 저장소 쓰기·용량 검사는 start의 사전검사(scan_recorder.py:197-209)에 있다. 디스크 부족 자체가 일반 NAV의 startup을 차단한다는 주장은 하지 않는다. scan profile 설정 스키마의 전역 의존성과 구별한다.

## 추가 finding

### C-07 — 녹화 완료가 아직 발행할 수 없는 부모 결과를 요구

Safety: S1. Mobility: M4 (RECORD_ROUTE/동일 profile 녹화 기능 범위). OBSERVED 구성·호출순서, INFERRED 깨끗한 단일 mission에서의 실패.

src/mission_manager/config/scan_profiles.yaml:15는 /mission/result를 필수 녹화 topic으로 둔다. scan_recorder.py:211-236은 메시지 0개인 필수 topic이 하나라도 있으면 실패한다. record_route_runner.py:217-235는 녹화 finalize/검증 이후에만 결과를 반환한다. mission_action_server.py:123-140은 그 반환 뒤 부모 terminal을 발행한다. 설치 actionlib/action_server.py:140-141에서 result publisher는 latched가 아니다.

정상적인 단일 accepted mission의 자체 terminal은 검증하는 bag 안에 들어갈 수 없다. 다른 요청의 reject 결과 등이 우연히 녹화되면 topic count 조건을 만족할 수 있지만 mission identity 검증이 아니다. 실 bag 재현은 하지 않았다. 좁은 개선은 부모 terminal을 진행 중 녹화의 필수 sensor 집합에서 제외하고 결과/manifest로 결합하는 것이다. 새 node는 필요 없다. 검증: 다른 mission 결과가 전혀 없는 clean recorder와 unrelated result 주입을 구분해야 한다.

### C-08 — 정지 진입 gate가 새 AMCL pose 3개를 스스로 얻지 못함

Safety: S1. Mobility: M3. INFERRED; 실제 발생 빈도 UNVERIFIED. 판정 TUNE.

mission_manager/ros_state.py:191-203은 NAV 뒤 새 pose fence를 만들고 stair_entry_gate.py의 policy는 post-fence 복수 pose를 요구한다. 이 준비 경로에는 nomotion 요청이 없다. navigation.launch의 AMCL 이동 임계와 upstream amcl_node.cpp:1111-1121,1210-1400을 대조했다. AMCL의 정지 중 TF 재발행과 새 pose 표본 발행은 같은 계약이 아니다. nomotion callback은 강제 갱신 플래그를 둔다(:1023-1028). 따라서 gui_publish_rate만으로 정지 후 요구 표본이 주기적으로 생긴다고 볼 수 없다. 외부 nomotion 요청/충분한 이동·노이즈가 없다면 정상 정지 진입이 timeout될 수 있다.

최소 개선: 위치·방향·floor 검증을 보존하면서 기존 진입 경로에서 bounded pose acquisition을 연결한다. 검증: 정지 AMCL 실제 발행 계약을 재현하는 peer, 충분한 fresh pose, 잘못된 floor 및 covariance를 별도 검증. 공식 source URL/hash는 upstream manifest에 보존하며 설치 바이너리와 branch source의 완전한 일치는 UNVERIFIED다.

### C-09 — production directed graph에 복귀 연결이 없음

Safety: S1. Mobility: M2. OBSERVED production planner 반례.

src/mission_manager/config/building_graph.yaml의 4F·5F 평면 연결은 도착 landing에서 다음 상행 entry 방향이다. 반대 방향 NAV edge가 없다. 실제 BuildingPlanner와 production bundle로 stair_5f_to_rf→home_3f 및 stair_4f_to_5f→home_3f를 요청하면 no directed route다(pure-C-probe.json). 외부 목표 실행은 하지 않았다. return_after_task를 원하는 특정 위치 조합을 차단한다. 물리적으로 안전한 역방향 연결인지는 UNVERIFIED이므로 무조건 edge를 뒤집지 않는다. 최소 개선: 현장 확인된 복귀 연결만 기존 graph에 추가하고 outward/return을 둘 다 사전검사한다.

## runtime 관찰 추가

명령: SSH key 인증으로 date/hostname, systemctl --user is-active/is-enabled/show, ps/pgrep, sensor launch 및 bridge wrapper 원문 조회. 관찰 시각: 2026-09-18 02:52:52 및 02:53:32 UTC. 관찰 결과: Mini PC astra-web.service active/enabled, MainPID 904/python3; 해당 조회에서 ROS 관련 process는 보이지 않았다. 관련 코드: run.sh의 sensor reuse/start 경로 및 remote-sensor-observation.txt.

OBSERVED 외부 launch: wf_mapping은 LiDAR, odometry bridge, D435, scan converter를 포함한다. bridge wrapper 아래 실제 Python 구현은 미열람이다. astra-web의 active 상태만으로 command 충돌을 확정하지 않는다. 현재 ROS graph/실행 중 NAV behavior는 관찰하지 못했다. 암호를 저장하지 않았고 password auth를 실행하지 않았다.

## 단계 상태

Phase C의 production 상태머신 정적 재구성과 configuration 도달성 검토를 마쳤다. cancellation interleaving의 결정적 재현, late service completion, floor ownership epoch 악용 가능성, 모든 unit/contract/replay 및 과거 evidence는 E에서 이어서 확인한다. 이것은 해당 주장을 OBSERVED로 승격하거나 E2E를 완료했다는 뜻이 아니다.

확정 사실은 위 source/probe/제한된 runtime 관찰이다. 실제 계단참/정지 안정성, 펌웨어 watchdog, nominal communication jitter, AMCL 설치 binary 동작은 아직 UNVERIFIED다. 새 조사 대상은 C-07의 recording dependency cycle, C-08의 pose 생산 계약, 복귀 graph, scan loss와 costmap freshness의 연결이다. 남은 phase: D → E → F.
