# Phase C 중간 기록 — AUDIT_INCOMPLETE

기준 경로: `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`. 이하 코드 근거는 이 경로 기준이다. 이 문서는 최종 감사나 실장비 승인서가 아니다.

## 열람 범위와 남은 범위

이번 단계에서 25개 파일을 추가로 전체 열람했다. 정확한 목록은 coverage.json의 READ 상태에 있다. mission FSM/action/orchestrator/route/navigation/segment/admission/record-route, floor callback/readiness/tag/map/policy/service, stair evidence/admission 및 production stair/robot/transition YAML을 읽었다. Phase B에서 읽은 supervisor와 floor action 구현도 대조했다.

C는 아직 완료하지 않았다. configuration loader의 실제 유효성, scan_recorder, production graph/location/floor/stair/tag config, action/message/service 원문, cancellation race의 반례 테스트, AMCL의 정지 중 pose 발행 계약, 과거 replay evidence를 추가 확인해야 한다. D/E/F는 미착수다.

## 확정된 사실과 반증

- OBSERVED: MissionFSM에 RESET 전이는 있지만 mission_orchestrator.py:69는 요청마다 새 FSM을 만든다. mission_action_server.py:134-154는 정상 반환/예외 시 active handle을 해제한다. 따라서 'RESET을 호출하지 않으므로 모든 mission이 영구 BUSY'라는 주장은 잘못이다. 무한 대기에 걸리는 경로와 구별해야 한다.
- OBSERVED: NAV 재시도는 navigation_executor.py:187-204의 최대 2회다. orchestrator의 일반 RETRY 루프에는 상한이 없지만, 현재 읽은 production ros_segments.py에는 RETRY 반환이 없다. 무한 retry를 현재 production 결함으로 확정하지 않는다.
- OBSERVED: planner는 disabled stair edge를 제외하지만 NAV edge는 유지한다(route_planner.py:223-230). 따라서 stair enabled=false만으로 평면 경로가 차단된다고 단정할 수 없다. stair config 로드 자체 실패가 transport DISARMED로 이어지는 문제와 구별한다.
- OBSERVED: robot.yaml과 stair_profiles.yaml은 configured=true이고 production 계단 profile들은 enabled=true다. '기본 production 계단은 비활성'이라는 주석이나 과거 문서만으로 현재 동작을 판단하면 안 된다.
- OBSERVED: 초기 FloorState READY는 예상 map fingerprint 관찰로 설정된다(ros_callbacks.py:48-53). 초기 위치의 물리적 일치를 증명하는 조건은 이 경로에 없다. 전이 후 localization 검증과 동일시하면 안 된다.
- OBSERVED: map_loader.py:120은 resolution을 float32로 정규화한다. 단순히 YAML double과 ROS float32가 다르다는 이유로 map identity 실패를 주장하면 안 된다.

## 잠정 root-cause findings

### C-01 — child 실행 수명에 상한과 지속 health 검사가 없음

Safety: S2. Mobility: M3.

OBSERVED: navigation_executor.py:290-298, ros_segments.py:175,202는 인자 없이 wait_for_result를 호출한다. NAV client 생성/전송 경로에는 wait_for_server가 없다(navigation_executor.py:155-171,271-288). ros_segments.py:84는 segment 시작 전에만 health를 확인한다. mission_action_server.py:82는 active handle이 남으면 BUSY를 반환한다.

OBSERVED upstream: 설치된 `/opt/ros/noetic/lib/python3/dist-packages/actionlib/simple_action_client.py:120-144`에서 기본 0 timeout은 무한 대기이며 DONE 또는 ROS shutdown을 기다린다. actionlib가 자동으로 이 애플리케이션의 deadline을 제공한다고 가정할 수 없다.

INFERRED: child가 terminal을 발행하지 않는 장애에서는 부모가 취소를 요청해도 결과를 기다리며 다음 mission을 막을 수 있다. 서버 종료가 실제 어떤 통신상태를 유발하는지는 fault-injection 미실행이다. stop 보장은 실제 transport 상태까지 검증되지 않았다.

최소 개선: 기존 segment executor 안에서 wall/monotonic deadline, health polling, bounded cancel acknowledgement를 연결한다. timeout 후에는 이전 child가 정지·격리됐다는 근거 없이 새 명령을 허용하지 않는다. 새 node는 필요하지 않다. 검증: 결과를 영원히 보내지 않는 fake child, cancel ACK 상실, 서버 미연결 최초 NAV, 복구 후 새 goal.

### C-02 — floor 전이 실패가 현재 floor와 복구 진입점을 함께 잃음

Safety: S2. Mobility: M3. 판정 MAKE_RECOVERABLE.

OBSERVED: multifloor ros_node.py:115-121은 실패/취소를 FAULT로 끝낸다. ros_runtime.py:221-235는 active evidence를 지우고 기본 빈 floor_id로 publish한다. 다음 전이는 ros_node.py:199-208의 READY 조건을 만족해야 한다. 일반 map callback은 UNKNOWN 초기화 때만 READY로 바꾼다(ros_callbacks.py:48-54). mission health는 FAULT를 차단한다(ros_state.py:162-165).

INFERRED: 일시적 tag/service/evidence timeout 후 센서가 회복돼도 이 node의 공개 action 경로로 전이를 재시도할 수 없다. map 변경 전·후 실패를 구별하는 복구 정보도 state에 남기지 않는다. 다른 외부 복구 도구의 존재는 미열람 범위에 남는다.

최소 개선: 실패 지점·확정 map/floor·진행 중 service를 보존하고 기존 transaction을 재관측하거나 재시도하는 제한된 경로를 둔다. 단순 FAULT 해제만으로 잘못된 floor에서 NAV를 허용하면 안 된다. 검증: map 변경 전 tag timeout, 변경 후 AMCL timeout, late service completion, cancel 이후 확인된 floor로 복구.

### C-03 — cancel 요청과 실제 child dispatch 사이의 간극

Safety: S3. Mobility: M2.

OBSERVED: ros_segments.py:121,140,193의 `_context`는 cancellation predicate를 사용하지 않는다. cancel_active:101-109는 현재 client에만 cancel을 보낸다. stair 경로는 entry wait와 server wait 후 새 goal을 전송한다(:147-175). NAV execute:181-185는 cancel flag를 초기화한다. floor 역시 wait_for_server 후 send_goal한다(:194-202).

INFERRED: 부모가 segment dispatch 전에 cancellation을 확인한 뒤, 아직 child goal이 없는 준비 대기 중에 cancel이 들어오면 그 cancel이 나중 child goal에 전달되지 않을 수 있다. 실제 actionlib 선후관계와 재현은 아직 검증하지 않았다. '항상 cancel이 유실된다'는 주장은 아니다.

최소 개선: 기존 context predicate를 준비 대기 및 전송 직전에 확인하고, send/cancel 경합을 같은 소유권 경계에서 처리한다. 검증: entry evidence 대기, server wait, NAV execute 진입 전 각각의 결정적 cancel interleaving.

### C-04 — 계단 phase 완료가 물리적 계단참/출구 확인을 대신함

Safety: S4 (잠재 추락/충돌 영향). Mobility: M2. 증거 수준: 코드 OBSERVED, 실제 위험 발생 UNVERIFIED.

OBSERVED: stair_evidence.py:176-179의 ALIGN은 새 odom sample만 요구한다. LANDING/EXIT_CONFIRM은 :211-224의 per-sample displacement/yaw tolerance와 odom stamp dwell이다. flight 진행은 :197-209의 odom 투영거리이며 별도 계단참/낭떠러지 관측 입력은 없다. 현재 production distance_tolerance=0.50m, yaw_tolerance=0.35rad, dwell=5s, profile enabled=true(stair_profiles.yaml:7,12-21).

반증/범위: entry의 절대 위치·방향은 mission stair_entry_gate.py:115-176이 먼저 검사한다. 그러므로 '계단 진입 정렬 검사가 전혀 없다'는 주장은 아니다. 여기서 미증명인 것은 주행 후 실제 계단참 도달과 제자리 turn의 물리적 여유다. calibration·bag·실측 검토 전 tolerance 축소/증가나 KEEP을 확정하지 않는다.

최소 개선 후보: 기존 LANDING/TURN gate에 실제 계단참과 정지에 대한 독립 관측을 연결하고, per-sample 작은 이동을 장시간 정지로 오인하지 않는 조건을 검증한다. 새 phase/node는 제안하지 않는다. 검증: wheel slip/drift, 천천히 누적 이동하는 dwell, landing 도달 전 odom 임계 초과의 안전한 replay/모형 반례.

### C-05 — 이미 확인한 floor 사실이 후속 작업 시간 때문에 만료됨

Safety: S1. Mobility: M2. 판정 NARROW 또는 TUNE, 최종 미확정.

OBSERVED: ros_node.py:220-224는 T_FLOOR_CONFIRMED timestamp를 한 번 기록한다. 이후 map load, 3회 nomotion/pose wait, costmap 작업을 수행한다. final gate :145-159는 T_LOCALIZED와 T_COSTMAP_READY만 갱신한다. transitions.py:214-218는 원래 observation age를 검사하며 production freshness는 10초다(transitions.yaml:20). tag callback은 policy observation을 새로 기록하지 않는다.

INFERRED: tag 승인 후 후속 작업이 10초를 넘으면 map/AMCL/costmap이 정상이어도 final gate의 floor 증거가 만료되어 실패할 수 있다. 이후 새 tag를 보더라도 해당 observation timestamp 갱신 경로가 없다. 실제 발생 빈도는 UNVERIFIED.

최소 개선: 같은 전이 epoch 동안 유지되는 floor 확인 사실과 연속 sensor freshness를 구별하되, 반대 floor 관측 시 무효화한다. 검증: tag 승인 후 9초/11초 map-load 사례와 반대 floor evidence.

### C-06 — RECORD_ROUTE 오류 경로의 FSM event가 현재 state와 불일치

Safety: S1. Mobility: M2.

OBSERVED: record_route_runner.py:169-176은 segment 성공 후 NEXT_SEGMENT로 바꾼 뒤 recording.assert_active 실패 시 SEGMENT_FAILED를 dispatch한다. fsm.py:65-80에는 NEXT_SEGMENT+SEGMENT_FAILED 전이가 없다. exception은 action boundary에서 abort되지만 정상 RecordRouteOutcome을 반환하지 않아 orchestrator.py:67의 anchor 갱신이 실행되지 않는다.

INFERRED: 실제 이동 segment가 성공한 뒤 recording process가 죽으면, 이번 run에서 갱신된 anchor가 부모에 반영되지 않을 수 있다. 물리적 목적지와 다음 계획의 출발점이 달라질 수 있다. 테스트 미실행.

최소 개선: recording failure를 NEXT_SEGMENT에서 허용하는 기존 오류 경로로 처리하고, 성공한 이동의 anchor를 오류 결과에도 반환한다. 검증: 첫 NAV 성공 직후 fake recorder 종료, 다음 goal의 source anchor 확인.

## 다음 조사 대상

1. production site configuration과 action/message 계약을 전체 읽고 위 경로가 실제 조합에서도 도달 가능한지 확인.
2. scan_recorder의 constructor가 단순 NAV startup에 storage를 요구하는지 확인.
3. AMCL upstream 소스로 정지 상태 pose publication/nomotion 및 post-fence 3 sample 요구의 충족 가능성을 검증. 아직 AMCL 동작을 확정하지 않음.
4. floor action의 stair_ownership_epoch 필드가 server에서 사용되지 않는 경로를 supervisor handoff 계약과 대조. 무단 floor transition의 실제 도달성은 E에서 통합.
5. bounded service timeout 뒤 daemon 호출이 실제 map 변경을 늦게 완료할 수 있는지, recovery 설계에서 어떻게 fence할지 검토.
6. 모든 관련 unit/contract/replay tests를 먼저 읽고 안전성을 확인한 후 실행. 실장비나 runtime state를 변경하는 테스트는 실행하지 않음.
