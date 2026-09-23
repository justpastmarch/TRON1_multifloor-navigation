# Constraint ledger — 부분 목록, 미완료

기준 repository: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation. 축약 package 경로는 src/<package>/src/<package>/ 기준이며 scripts는 별도 표시했다. D의 정적 검토를 바탕으로 E의 잔여 evidence를 읽고 보완해야 한다. 위치와 구현은 OBSERVED; hazard의 실물 현실성/임계 최소성은 아직 증명되지 않았다. KEEP 판정은 0개다. UNVERIFIED는 사용자의 추가 완료 규칙에 따른 보류 판정이다.

| Constraint | 위치 | 차단하려는 hazard | 적용 범위 | false positive/미검증점 | 복구 | 잠정 판정 |
|---|---|---|---|---|---|---|
| 전역 camera/tag/action readiness | run.sh:70-73,105-107,225,348-360 | 해당 기능 입력/서버 부재 | 모든 startup | camera 없이 flat NAV 차단 | 입력 복구 후 전체 재실행 | NARROW |
| Mini PC 센서 묶음 재시작 | run.sh:224-225 | 죽은 sensor stack 재사용 | lidar/scan/odom/camera 전체 | camera 단독 누락에도 다른 센서 재시작 | 전체 wf_mapping restart branch | NARROW |
| NTP와 clock 차이 | run.sh:110-114,131-141 | 서로 다른 시간 기준의 stale 판정 | 전체 startup | sync 플래그와 실제 clock 오차 불일치 가능 | clock 정상화 후 재시도 | UNVERIFIED |
| AprilTag 중복 detector | run.sh:228-234 | 중복 detector 입력 | 전체 startup | stale registration에도 종료 가능 | 문서는 기존 detector 중단 요구 | NARROW |
| 단일 scan publisher | run.sh:365-369 | 다중 scan authority | 전체 startup | 죽은 registration 집계 가능 | 구체 publisher 확인 후 재시도 | UNVERIFIED |
| TF launch 대기 | scripts/wait_for_tf_exec.sh:17-28 [multifloor_manager] | TF 부재 상태 NAV | move_base 시작 | 무한 대기/불완전 체인 판정 | 자동 탈출 deadline 없음 | NARROW |
| floor/supervisor health | mission_manager/ros_state.py:151-166 | 잘못된 floor/command owner | 모든 segment 시작 | floor FAULT가 관련 작업도 차단 | 상위 원인 복구 경로 미완 | NARROW |
| NAV floor/generation 일치 | mission_manager/navigation_executor.py:239-250 | 잘못된 지도 목적지 명령 | NAV 시작 | stale state와 실제 상태 구별 필요 | 최신 floor state 수신 | UNVERIFIED |
| NAV freshness/zero | stair_supervisor/supervisor.py:95-112 | 오래된 속도 명령 지속 | NAV streaming | 5Hz controller 대비 0.25s 여유 검토 필요 | fresh NAV 입력 | TUNE |
| transport fault latch | stair_supervisor/robot_transport.py:81-98 | 불확실 통신 명령 지속 | 전체 command transport | 일시 장애 후 session 회복 불가 | 현재 node/session 재생성 필요 | MAKE_RECOVERABLE |
| stair admission token | mission_manager/stair_admission.py:57-107 | mission 소유권 없는 stair 요청 | STAIR 진입 | 1s lifetime과 recheck 시간 예산 | 새 admission 발급 | UNVERIFIED |
| post-fence entry pose | mission_manager/stair_entry_gate.py:115-176 | 잘못된 진입 위치/방향 | STAIR 진입 | 정지 후 새 AMCL 3개 요구 충족 미검증 | 재시도; nomotion 연결 미확인 | TUNE |
| stair sample gap/freshness | stair_supervisor/stair_evidence.py:96-156 | 관측 상실 상태 stair 이동 | 계단 profile | 0.12s gap 1회로 fault | supervisor FAULT 복구 필요 | TUNE |
| stair dwell/progress | stair_supervisor/stair_evidence.py:169-224 | 계단참 도달 전 회전/이탈 | 계단 phase | 실측 threshold와 slip 관계 미검증 | FAULT 경로 | UNVERIFIED |
| checkpoint cancel | stair_supervisor/supervisor.py:27-29,168-185 | 불안정 계단 구간 임의 중단 | STAIR cancel | LANDING evidence 미완료/FAULT에도 mode 해제·NAV 복귀하는 순수 반례; 취소 지연도 존재 | 위험 정지와 검증된 ownership 복귀를 분리; 현 구현 복구 불충분 | TUNE |
| tag vote | multifloor_manager/tag_evidence.py:87-92,224-295 | 잘못된 floor 확정 | floor transition | 3회/1초/0.5초/10초 임계 실측 미검증 | 실패 시 floor FAULT | TUNE |
| map fingerprint | multifloor_manager/map_evidence.py:256-296 | 이전/다른 지도 전이 | floor transition | 동일 identity와 load-time 계약 검증 필요 | 실패 시 floor FAULT | UNVERIFIED |
| AMCL/scan/odom/TF conjunction | multifloor_manager/readiness.py:214-245 | localization 미확정 후 주행 | floor transition | pose 갱신과 stationary 필요의 충족 검토 | 실패 시 floor FAULT | TUNE |
| costmap metadata equality | multifloor_manager/ros_node.py:170-180 | 이전 지도 costmap 사용 | floor transition | origin/크기 동일성 필요성 미증명 | 실패 시 floor FAULT | UNVERIFIED |
| floor confirmed freshness | multifloor_manager/ros_node.py:220-224; transitions.py:214-218 | 오래된 floor 관측 사용 | 최종 transition gate | 후속 작업 10초 초과 시 관측 갱신 없음 | 실패 시 floor FAULT | NARROW |
| floor FAULT latch | multifloor_manager/ros_node.py:115-121,199-208 | 불확실 floor 상태 NAV | 후속 mission | 일시 실패도 READY 복구 경로 부재 | 안전한 runtime 재관측 경로 필요 | MAKE_RECOVERABLE |


## Phase D/E에서 추가 발견한 제약

위 21개와 함께 사용하는 추가 12개다. 주요 constraint의 전체 목록 확정은 아직 미완료다. source 근거는 phase-D.md, phase-C-continuation.md, phase-E-partial.md에서 연결한다. KEEP은 여전히 0개다.

| Constraint | 위치 | 차단하려는 hazard | 적용 범위 | false positive/미검증점 | 복구 | 잠정 판정 |
|---|---|---|---|---|---|---|
| 전체 site bundle validation | mission_manager/site_config.py:200-212 | 잘못된 지도/route/config | 모든 mission startup | 무관한 층·scan profile 결함도 NAV 차단 | bundle 수정 후 재실행 | NARROW |
| recording mandatory topic count | mission_manager/scan_recorder.py:211-236; config/scan_profiles.yaml:15 | 불완전 녹화 성공 처리 | INSPECT/RECORD_ROUTE | 부모 result가 finalize 이후 발생 | topic policy 좁힌 뒤 재시도 | NARROW |
| free space fraction | mission_manager/scan_recorder.py:197-209 | 기록 중 disk exhaustion | 녹화 시작 | 총 disk 비율과 실제 필요 용량 차이 | 저장공간 복구 후 재시도 | TUNE |
| artifact root/identity bound | mission_manager/scan_recorder.py:117-122,190 | root 이탈/기존 artifact 덮어쓰기 | 녹화 경로 | 동시 경로 변경 race 미검증 | 새 ID/정상 output root | UNVERIFIED |
| robot radius | nav/costmap_common_params.yaml:2 | body collision | global/local NAV | 실측 외곽과 일치 미확인 | 실측 후 config 검증 | UNVERIFIED |
| footprint padding | nav/costmap_common_params.yaml:3 | geometry/localization 오차 | global/local NAV | 2cm 근거 미확인 | 측정된 오차에 맞게 조정 | TUNE |
| inflation cost | nav/*costmap_params.yaml inflation_layer | 벽 근접 비용 | global/local NAV | soft cost와 금지 영역 혼동 금지 | profile 비용 검증 | TUNE |
| obstacle/raytrace range | nav/local_costmap_params.yaml:22-23 | 동적 장애물/잔상 | FLAT_NAV local map | 실센서 coverage·제동거리 미확인 | 새 관측 clearing | UNVERIFIED |
| TF age | nav/*costmap_params.yaml transform_tolerance | 오래된 위치로 trajectory 평가 | NAV costmap | 0.5초 근거/clock·TF gap | fresh TF 수신 | TUNE |
| goal tolerance/latch | nav/base_local_planner_params.yaml:14-16 | 목적지 자세 오차 | NAV 완료 및 stair entry에 재사용 | 일반 goal과 stair entry hazard 차이 | 재시도/pose acquisition | NARROW |
| oscillation/planner patience | navigation.launch:54-57 | 진전 없는 motion | NAV | 좁은 통로 정상 회전과 구별 필요 | mission bounded retry | TUNE |
| recovery disabled | navigation.launch:55 | 불확실 공간에서 recovery 회전 | 모든 NAV | 평면 복구도 일괄 차단 | 현재 새 goal/제한 재시도 | NARROW |

scan freshness expected_update_rate 누락은 gate 자체가 아니라 D-01의 보호 공백 후보로 별도 기록한다. 속도·가속도·sim horizon·unknown-space의 최소성은 phase-D.md에서 UNVERIFIED이며 추가 upstream/static-layer 검토 후 ledger를 확정한다.

## 추가 12개: 45개 제약 기록, 전체 목록 확정 전

구현/config와 공식 source는 OBSERVED, 물리적 최소 범위·임계는 별도 근거가 없으면 UNVERIFIED다. KEEP 0개. 보호 공백(미검증 floor handoff epoch, scan freshness 누락)은 실제 존재하는 gate처럼 세지 않고 findings C-10/D-01에 기록한다.

| Constraint | 위치 | 차단하려는 hazard | 적용 범위 | 정상 주행 false positive/미검증점 | 복구 | 잠정 판정 |
|---|---|---|---|---|---|---|
| 초기 지도 identity 일치 | multifloor_manager/ros_callbacks.py:36-59 | 지정 floor와 다른 지도 사용 | startup floor READY | floor만 바꾸고 map default가 남으면 UNKNOWN; pose·anchor 동시 검증 없음 | 동일 startup 입력으로 map/floor/pose/anchor 재설정 | TUNE |
| test-fixture opt-in | mission_manager/ros_runtime.py:38-53; system.launch:52-79 | synthetic 설정의 실장비 적용 | package config 선택 | mission profile gate와 다른 두 node의 allow_test_fixture 분기 일치 필요 | production profile로 일관되게 시작 | UNVERIFIED |
| workstation 전체 실행 lock | run.sh:178-182 | 중복 managed command owner | 같은 host의 /tmp/tron1_system.lock | 서로 독립 master라도 차단; fd 상속과 stale 소유권 확인 필요 | 실제 owner 종료 후 재실행 | NARROW |
| command cadence 30Hz 하한 | stair_supervisor/robot_config.py:56-65 | transport/firmware command 공백 | 전체 transport startup | 문서상 최소와 실제 firmware 계약 미검증 | 검증한 cadence 설정 | UNVERIFIED |
| twist finite/clipping | stair_supervisor/robot_conversion.py:25-37 | NaN/Inf·범위 초과 명령 | 전체 NAV/STAIR 전송 | full-scale calibration 실측 미검증; clipping이 요청과 실제 동작 차이를 숨김 | finite 입력·calibration 정상화 | UNVERIFIED |
| NAV 속도·가속도 envelope | nav/base_local_planner_params.yaml:2-9; navigation.launch:67-74; config.env | 급가속/과속·기체 제어 한계 | 평면 local planner | 실제 제동/회전·floor별 필요성 미측정; YAML과 launch 값을 구분 | commissioning 후 기존 config 조정 | TUNE |
| trajectory horizon/sampling | nav/base_local_planner_params.yaml:10-13 | 짧은 예측/이산 sample의 충돌 누락 | local NAV | sim_time1.2/granularity0.05/12×24 후보의 충분성 미검증 | 같은 planner의 측정 기반 조정 | UNVERIFIED |
| 일반 전진-only rollout | nav/base_local_planner_params.yaml:3,20 | 후방 coverage 없는 정상 후진 | 일반 sampled NAV | 좁은 구역 복구 제한; 별도 escape -0.1은 존재 | 후방 coverage 검증 후 제한 후보/복구 | NARROW |
| unknown-space 허용 | upstream static_layer.cpp:74,150-164 및 Navfn allow_unknown 기본; repo override 없음 | 관측 불완전 구역 경로 단절 완화 | global plan | free/unknown/실물 지지면 구별 미검증; 일괄 금지는 정상 경로 차단 가능 | floor별 지도·관측 coverage 재검증 | UNVERIFIED |
| 내부 backup escape | upstream trajectory_planner.cpp:849-900; trajectory_planner_ros.cpp 기본 escape_vel | 유효 trajectory 부재 시 탈출 | local planner fallback | -1 footprint collision cost 양수 변환; 실후방 여유 미검증, recovery_enabled=false와 별개 | 기존 planner 설정·좁은 recovery 검증 | UNVERIFIED |
| 모든 recorder process 중복 차단 | notebooks/03_rosbag_recording.ipynb:165-168; check_environment.sh:245 부근 | 중복 녹화·disk 과부하 | host의 rosbag record 전체 | 별도 경로·충분한 용량의 unrelated recorder도 차단 | 현재 recorder 정상 종료; 대상 경로/자원 단위로 좁히기 | NARROW |
| notebook 녹화10GiB 여유 | notebooks/03_rosbag_recording.ipynb:173-175 | 녹화 중 disk exhaustion | 모든 manual notebook recording | 기록 길이/rate와 무관한 일괄값 | 여유 공간 확보·예상 용량 기준으로 조정 | TUNE |

checkpoint cancel 행은 C-04의 순수 반례를 반영해 TUNE으로 갱신했다. 위험 정지는 즉시 처리하되 LANDING 이름만으로 mode 해제/NAV 복귀를 허용하지 않도록 기존 evidence와 fault 우선순위를 정리한다. read-only bag에서 관찰한 gap은 임계 삭제의 근거가 아니며 정확한 phase·source clock·실제 지지 상태와 함께 판단해야 한다. replay isolation은 운용 command boundary의 보호 공백(E-04)이고 평면 주행용 새 readiness 조건으로 제안하지 않는다.
