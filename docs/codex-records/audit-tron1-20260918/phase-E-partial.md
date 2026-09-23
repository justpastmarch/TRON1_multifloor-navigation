# Phase E 중간 기록 — AUDIT_INCOMPLETE

기준 경로는 coverage.json의 repository다. E는 끝나지 않았다. 문서·테스트·보안·과거 evidence의 전체 교차검증과 F 15개 시나리오가 남아 있다.

## 읽은 범위와 실행 결과

README, DESIGN, manual capture/legacy viewer/commissioning 문서, SensorJoy bridge, 선택한 unit/contract/integration test source 및 test/fixtures 전체를 읽었다. .omo task-2 manifest와 extraction summary는 기존 주장으로 읽었으며 실제 bag 재분석 결과로 둔갑시키지 않는다. .omo/run-continuation 973개는 모두 세션 ID·시각·idle/stopped 값뿐임을 검사했다(session-metadata-review.json). source/plan/evidence 내용이 없는 관리 metadata이므로 METADATA_ONLY다. 계획·실험 증거 파일에는 이 처리를 적용하지 않았다.

OBSERVED 실행: selected-unit-results.json의 10개 unit test 통과, selected-contract-results.json의 6개 file/XML contract 중 5개 통과·1개 실패. 합계 16개 실행, 15 pass / 1 fail. 별도 첫 loader 시도는 생성된 stair_supervisor.srv import 부재로 중단됐고 테스트 0개 실행했다. runtime_gating, ROS integration, hardware 시험은 실행하지 않았다. subprocess mock 및 socket 금지를 둔 unit 경로와 read-only contract만 실행했다. test 결과는 전체 suite의 성적이 아니다.

확정 OBSERVED: 실패한 contract는 삭제된 local pointcloud_to_laserscan.launch include를 기대한다(test/test_system_operator_contract.py:34-54). 실제 system.launch:30-31은 Mini PC scan 단일 owner를 명시한다. 실패를 해결하려고 로컬 publisher를 되살리면 현 구조의 중복 방지와 충돌할 수 있다.

## E-01 — 수동 녹화 문서와 기본 원격 process 시작 동작 불일치

Safety: S2. Mobility: M2. OBSERVED 코드/문서 불일치; SDK ownership 효과 UNVERIFIED.

docs/manual-mission-capture-ko.md:3-4는 robot/sensor launch/SDK listener를 시작하지 않는다고 설명한다. run.sh:47-50은 SENSOR_JOY_RECEIVER_AUTOSTART=1이면 SSH로 listener 부재를 확인하고 nohup/setsid로 시작한다. config.env:30의 기본값이 1이다. sensor_joy_bridge.py:64-68은 Robot.init을 실행한다. 해당 문서는 SDK 초기화 ownership을 먼저 확인하도록 요구하지만 wrapper의 자동 시작 경로에는 그 확인 gate가 없다. 감사에서 이 명령이나 SDK init은 실행하지 않았다.

최소 개선: 기본 동작과 설명을 일치시키고 현재 listener 존재 여부·초기화 검증 상태를 명시한다. 기존 flag로 자동 시작을 명시적으로 선택하도록 범위를 좁힐 수 있다. 새 subsystem은 불필요하다. Required test: default capture path에서 원격 시작 호출 0회, opt-in일 때 정확히 한 listener만 생성, 단순 pgrep registration과 실제 callback health 구별. 실 SDK init이 명령 소유권을 바꾸는지는 외부 source/제조사 확인 없이는 확정하지 않는다.

## E-02 — production 계약 변경이 운영 예시와 시험에 반영되지 않음

Safety: S1. Mobility: M2. OBSERVED.

README.md:233-235의 inspect 예시는 destination_id=roof_scan인데 현재 locations.yaml:5-71의 ID 집합에 없다. goal admission이 이를 INVALID_GOAL로 거부하는 코드와 일치한다. README:87-88,137-140의 production configured=false 설명도 현재 true 설정과 다르다. src/mission_manager/test/test_runtime_gating.py:13-24는 현재 production loader가 ConfigurationError를 내야 한다고 가정하지만 production 구성은 순수 probe에서 정상 로드됐다. 이 테스트 자체는 미실행이다.

같은 root의 다른 증상: 위 scan launch contract 실패, full_system_synthetic test의 fixture 경로. test_full_system_synthetic_ros.py:29가 __file__.parents[2]/test/fixtures를 사용하면 이 source 배치에서는 src/test/fixtures로 해석된다. 실제 fixture는 repository root/test/fixtures다. catkin 실행 wrapper의 __file__ 계약은 추가 확인 대상이므로 전체 rostest 실패를 실행 관찰로 주장하지 않는다.

최소 개선: production ID와 실제 owner를 기준으로 예시·contract를 동기화하고 configuration loader 성공과 물리 commissioning 승인 상태를 분리한다. 문서 명령의 ID/경로 검사는 읽기 전용으로 수행 가능하다. runtime을 과거 assertion에 맞추는 변경은 피한다. Required test: 문서 목적지 모두 존재, 현재 scan owner가 정확히 하나, source/devel 실행 경로에서 fixture 존재, configured=true 자료로 startup contract 검증.

## E-03 — 합성 성공 oracle이 구현 phase/threshold를 따라감

Safety: S2. Mobility: M2 (검증 사각지대의 간접 영향). OBSERVED 시험 구조; 실 운용 영향은 관련 C finding으로 분리.

synthetic_hardware_peers.py:157-163은 supervisor feedback의 phase를 받아들이고 :195-219는 같은 profile의 목표거리/각도 및 허용 step 값으로 가짜 odom을 생성한다. :165-168의 move_base는 path/collision 검사 없이 목적지를 저장하고 즉시 성공한다. :251 이후 pose는 AMCL 대신 주기적으로 목표값을 발행한다. 따라서 이 시험은 action plumbing/상태 진행 검증에는 유용하지만 물리 계단참 도달, 정지, 경로 통행성 또는 정지 AMCL pose 생산 계약을 독립적으로 입증하지 못한다.

fixture parity의 구체적 차이도 있다. building_valid/scan_profiles.yaml과 building_5f_rf_synthetic/scan_profiles.yaml에는 production의 필수 /mission/result가 없어 C-07을 드러내지 않는다. building_valid graph에는 복귀 NAV edge가 있으나 production에는 C-09 연결이 없다. stair_synthetic의 상행 turn 부호·gap 값도 production과 같지 않다. test_command_provenance_contract.py:23-40은 production topic 목록에 /mission/result가 있는지만 확인하므로 C-07의 호출순서 결함을 오히려 보존할 수 있다.

mandela 점검: phase 피드백→fake sensor→동일 gate 성공은 ground-truth 독립성이 없다. 최소 보완은 기존 fixture에 실제 ROS 생산 계약을 반영하고, controller 출력/phase를 보지 않는 사전 고정 sensor trace와 물리적 관측을 별도 oracle로 사용하는 것이다. synthetic 시험을 버릴 필요는 없다. Required test: production scan profile clean terminal, 정지 AMCL, 실제 return topology, cumulative drift during dwell, phase와 무관하게 고정된 recorded samples.

## security 조사의 현재 경계

OBSERVED: recording artifact root는 resolve+relative_to로 제한하고 identity를 검사하며 goal의 mission_id는 서버 UUID다(scan_recorder.py:117-122,190; mission_action_server.py:95). 따라서 임의 goal 문자열만으로 path traversal 가능하다고 주장하지 않는다. shell/SSH quoting은 신뢰된 config의 실행과 원격 입력의 공격 가능성을 나눠야 한다. config.env가 shell source라는 사실만으로 인증 없는 원격 exploit을 확정하지 않는다. ROS network/API 접근제어, floor action epoch 검증, WebSocket 인증 및 shared path race의 전체 검토는 남아 있다.

## 새 조사 대상과 남은 phase

새 조사 대상: manual autostart의 실제 SDK provenance, fixture 경로의 catkin wrapper, 합성 feedback leakage, production profile 반영, prior task-2 raw gaps의 타임스탬프 정의. manifest가 보고한 1.919m wheel discontinuity와 여러 초 gap은 재분석 전 기존 문서 OBSERVED일 뿐 현재 runtime OBSERVED가 아니다. 이 기록만으로 0.12s gate의 최소성이나 완화를 확정하지 않는다.

남은 단계: D 측정 근거·constraint 누락 보강, E 잔여 source/docs/tests/security/evidence, F 15개 시나리오. 전체 Safety와 Operational mobility verdict는 아직 미판정이다.
