# Phase F — scenarios 8–15 adjudicated

코드 경로 판정이다. 실기체 E2E 실행 또는 성공률 추정이 아니다. CONDITIONAL은 명시한 상태가 충족될 때 가능한 경로, BLOCKED는 현재 공개 경로의 단절, UNVERIFIED는 실제 회복을 판정할 근거 부족을 뜻한다. PENDING은0개다. S/M은 해당 scenario의 관련 finding 중 가장 큰 영향이며 새로운 finding으로 중복 집계하지 않는다. minimum 항목은 필요한 설계 조건으로, KEEP에 요구되는 현장 최소성 증명은 아니다.

| ID | 시나리오 | 판정 | Safety / Mobility | 핵심 |
|---|---|---|---|---|
| 8 | mission cancel 후 새 goal | CONDITIONAL | S4 / M3 | 정상 child terminal과 상태 일관성이 돌아오면 새 goal을 받는 코드 경로는 존재한다. 모든 취소 시점에 안전한 새 임무가 가능하다는 판정은 거부한다. |
| 9 | 일시 scan gap | UNVERIFIED | S2 / M3 | 운용 NAV의 scan-only 공백을 안전하게 감지·일시 정지·정상 재개하는 E2E 계약은 입증되지 않는다. 코드상 scan 직접 freshness 보호가 부족하고 TF 만료와 floor 전이 실패는 별도 경로다. |
| 10 | 일시 odom gap | CONDITIONAL | S4 / M3 | 공백이 허용 경계 안에 있고 다른 상태가 정상인 경우와, 활성 STAIR에서0.12초를 넘겨 FAULT가 된 경우를 구분한다. 후자의 같은 session 내 회복은 BLOCKED다. NAV-only 경로의 실제 회복은 UNVERIFIED다. |
| 11 | WebSocket tunnel 단절 후 복구 | BLOCKED | S3 / M3 | 단절이 transport FAULT에 도달한 뒤 tunnel만 복원해 같은 owner/session에서 정상 주행을 재개하는 경로는 없다. budget 안의 transient send 복구는 별도 CONDITIONAL branch다. |
| 12 | stair supervisor FAULT 후 복구 | BLOCKED | S4 / M3 | 현재 공개 runtime interface로 latched stair/transport FAULT를 재검증한 뒤 정상 NAV/STAIR로 복귀하는 경로가 없다. cancelled 결과로 NAV에 돌아오는 경로를 FAULT recovery로 잘못 세지 않는다. |
| 13 | camera unavailable flat NAV | BLOCKED | S1 / M3 | 카메라가 없는 cold start에서 정상 LiDAR/odom/map을 가진 flat route도 managed ./run.sh는 시작 완료하지 못한다. 이미 정상 실행된 NAV 도중 camera만 사라진 경우의 즉시 정지와는 다르다. |
| 14 | AprilTag 미검출 flat NAV | CONDITIONAL | S1 / M3 | detector가 fresh한 빈 detection array를 계속 발행하는 미검출 상태는 flat NAV를 태그내용 때문에 차단하지 않는다. detector/stream 자체가 없거나stale인경우는startup BLOCKED다. |
| 15 | RViz 종료 autonomous NAV | CONDITIONAL | S1 / M2 | 정상 startup을 마친 뒤 RViz만 종료하면 이미 실행 중인 autonomous mission/NAV를 중단시키는 dependency는 없다. 다만 startup TF helper 경로의 boundedness는 별도 결함이다. |

## 8. mission cancel 후 새 goal

**CONDITIONAL — Safety S4 / Mobility M3**

- 최소 필요 조건: 같은 goal ID 취소, 이전 child의 정지/terminal 또는 확실한 격리, 확인된 현재 floor/map/pose와 anchor, 기존 command/token의 무효화, 새 mission ID. 이는 설계상 필요한 조건이며 실물 최소성 증명은 아니다.
- 현재 gate: active mission이 있으면 BUSY. child cancel 전송 뒤 terminal wait, 정상 run 반환/예외 시 active release. floor/supervisor health와 NAV floor/generation, stair entry/admission을 요구한다.
- 불필요하거나 범위를 좁힐 gate: BUSY 자체의 삭제 대상이 아니라 terminal 없는 child를 무기한 기다려 BUSY가 영구 유지되는 범위를 좁혀야 한다. floor cancellation 뒤 빈 floor FAULT의 재관측/retry 부재도 과도하다.
- 성공 가능성: 정상 terminal을 반환하는 NAV 취소에서는 코드상 허용되고 관련 ROS fixture 시험이 존재한다. 준비 대기 중 취소·cancel ACK 소실·floor FAULT·불완전 stair checkpoint에서는 실패 또는 안전 불충족 경로가 확인돼 일반 성공을 보증하지 못한다. 실기체 성공 확률은 UNVERIFIED다.
- false-stop: 정상 child가 죽거나 cancel ACK가 사라져도 active가 해제되지 않아 다음 goal이 BUSY로 거부될 수 있다. 정상 sensor가 회복돼도 floor FAULT는 새 전이를 거부한다.
- 복구: 현재 정상 terminal 뒤에는 새 goal 가능. hung child/terminal FAULT에는 공개된 scoped reset/rearm 명령이 없다. bounded cancel/health 확인과 실제 floor 재관측을 기존 owner 안에 추가하는 것이 최소 수정이다. 전체 재시작의 초기 pose/anchor 재적용은 검증된 resume이 아니다.
- OBSERVED: mission active/release와 cancel callback 코드가 존재한다. test_mission_action_ros.py는 취소 뒤 새 goal과 세 번 중단 뒤 성공을 시험한다. 이번 ROS 시험 실행은 아니다. 이번 감사의 logic-probe는 prepare/server 대기 중 취소 후 stair/floor child가 새로 전송됨을 확인했고 landing-cancel probe는 incomplete/faulted report에서 NAV 복귀함을 확인했다.
- INFERRED: 같은 interleaving이 live ROS에서 발생하면 취소 뒤 움직임 요청이 생길 수 있다. 무기한 child wait는 새 goal의 liveness를 막는다.
- UNVERIFIED: 실기체 cancel-to-stop 시간, 안전 checkpoint의 실제 자세, ROS 단절 interleaving 빈도와 새 goal의 실제 주행 성공.
- 관련 finding: C-01, C-02, C-03, C-04, C-06; constraint: K07, K15, K21, K46, K59, K60, K61.
- 근거: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/mission_action_server.py:82 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/mission_action_server.py:114 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/mission_action_server.py:149 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_segments.py:147 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_segments.py:193 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/navigation_executor.py:290 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/supervisor.py:171 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/test/test_mission_action_ros.py:116 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/test/test_mission_action_ros.py:274 [repository]; /home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/audit-tron1-20260918/logic-probe-resume2.json [audit]; /home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/audit-tron1-20260918/selected-tests-resume2.json [audit].

## 9. 일시 scan gap

**UNVERIFIED — Safety S2 / Mobility M3**

- 최소 필요 조건: 주행에 실제 사용하는 장애물 관측의 유효 기한, 유효한 TF/localization, 공백 중 새 위험을 고려한 zero 또는 감속, 신선한 scan과 일관된 localization이 회복된 뒤 같은 안전 목표를 재승인. 시간 임계값의 실물 최소성은 미입증.
- 현재 gate: startup은 /scan stream/header와 publisher 정확히1개를 확인한다. 운용 ordinary mission health는 floor/supervisor만 본다. costmap source expected_update_rate 누락은 공식 upstream default0을 사용해 observation buffer가 current를 유지한다. floor localization 단계는 scan age0.5초를 요구한다.
- 불필요하거나 범위를 좁힐 gate: 운용 scan-only 공백을 직접 차단하는 과도한 gate는 확인되지 않았다. 오히려 보호 공백이 있다. 전이 timeout이 빈 floor terminal FAULT로 이어져 회복 후에도 막는 범위는 좁혀야 한다.
- 성공 가능성: 짧은 공백 뒤 scan이 돌아오고 TF/localization/action이 유지되면 진행 가능한 추론 경로는 있다. 그러나 현재 binary와 sensor/TF 관계를 실시간 관찰하지 않았고 안전한 pause/resume의 보장은 없다. 실기체 성공 확률 UNVERIFIED.
- false-stop: TF age 초과, 전이 localization timeout 또는 stale 조건이 floor FAULT로 이어지면 scan 복원 후에도 새 전이를 거부할 수 있다. 정상 NAV에서 scan 공백만으로 항상 정지한다고는 말할 수 없다.
- 복구: 공백 자체에서 terminal fault가 없고 upstream가 fresh 관측을 다시 수용하면 재개 가능성이 있다. floor FAULT 후에는 공개 retry/rearm 경로가 없다. scan age를 실제 관측 source에 연결하고 회복 뒤 fresh evidence 재검증을 지원해야 한다.
- OBSERVED: startup scan gate와 운용 health 범위가 다르다. 공식 upstream observation_buffer default0 branch와 move_base noncurrent zero branch가 존재한다. repository는 expected_update_rate를 설정하지 않는다. readiness unit은 stale scan 거부를 시험하지만 회복 후 정상 mission 종료를 입증하지 않는다.
- INFERRED: TF가 여전히 fresh하면 scan 단절이 costmap-current 검사만으로 검출되지 않을 수 있다. TF 만료나 전이 timeout의 간접 정지는 scan watchdog과 동등하지 않다.
- UNVERIFIED: 실 설치 binary의 현재 동작, scan 공백과 TF age의 상관, 실제 장애물 위험·정지·자동 재개 및 성공 확률.
- 관련 finding: D-01, C-02, C-05; constraint: K05, K07, K18, K21, K30, K57, K58.
- 근거: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:354 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_state.py:151 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/config/nav/costmap_common_params.yaml:1 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/src/multifloor_manager/readiness.py:35 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/test/test_readiness.py:92 [repository]; /home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/audit-tron1-20260918/upstream/costmap_2d__plugins__obstacle_layer.cpp:97 [official-upstream-snapshot]; /home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/audit-tron1-20260918/upstream/costmap_2d__src__observation_buffer.cpp:231 [official-upstream-snapshot]; /home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/audit-tron1-20260918/upstream/move_base__src__move_base.cpp:829 [official-upstream-snapshot].

## 10. 일시 odom gap

**CONDITIONAL — Safety S4 / Mobility M3**

- 최소 필요 조건: 신선하고 단조로운 odom/TF, 주행 상태에 맞는 gap 허용값, 재개 시 odom 원점·pose 연속성·현재 floor와 ownership 확인. 계단 위 불확실한 자세에서 자동 NAV 반환 금지. 현장 허용값 최소성은 미입증.
- 현재 gate: production stair max_sample_gap0.12초, freshness0.20초, odom step0.10m, yaw step0.20rad, profile timeout300초. fault latch 뒤 후속 정상 sample은 progress를 회복시키지 않는다. NAV 일반 health는 odom을 직접 검사하지 않고 handoff/entry/floor localization에서 별도 freshodom을 요구한다.
- 불필요하거나 범위를 좁힐 gate: 검증된 짧은 공백도 terminal FAULT 뒤 sensor 회복만으로 끝없이 재거부하는 정책은 recoverable rearm으로 좁힐 대상이다. 0.12초 수치를 증거 없이 늘리는 것이 해법은 아니다.
- 성공 가능성: 0.12초 이하이고 실제 sample timestamp/step 경계를 지키는 활성 stair에는 그 공백 조건만의 차단이 없다. 0.12초 초과로 fault가 확정되면 정상 sample만으로 같은 session을 회복하지 못한다. 실기체 통과·정지·재개 성공 확률 UNVERIFIED.
- false-stop: 통신 jitter 또는 일시 생산 지연이 threshold를 넘으면 다른 센서와 실제 자세가 정상이어도 irreversible session fault가 될 수 있다. 계단 밖 NAV에도 같은 threshold가 전역 적용된다고 주장하지 않는다.
- 복구: 현재 활성 stair fault는 reset/rearm 인터페이스가 없어 owner/session 재생성이 필요하지만 검증된 현장 복구 절차는 없다. 실제 정지/landing/odom 재기준과 fresh admission을 확인하는 명시적 회복을 기존 owner에 추가해야 한다.
- OBSERVED: stair gap/freshness 임계값과 latch 구현을 직접 확인했다. recorded gap/jump fixture 뒤 후속 정상 sample도 faulted인 unit이 존재하고 이번 pure60개 실행에 포함됐다. current stair dwell과 cancel 우선순위에는 C-04 반례가 있어 단순 NAV 복귀를 안전 복구로 세지 않았다.
- INFERRED: STAIR에서 일시적 odom outage가 회복 불가 session 중단으로 커질 수 있다. NAV에서 odom/TF 복원으로 이어지는 경로는 외부 bridge와 upstream 상태에 의존한다.
- UNVERIFIED: 외부 odom bridge 복구, 물리적 odom 오차/슬립, gap 허용값의 현장 최소성, 계단상 안전 정지와 재승인.
- 관련 finding: C-04, B-02, C-02; constraint: K13, K14, K15, K18, K21, K54, K55.
- 근거: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/config/stair_profiles.yaml:18 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/stair_evidence.py:110 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/stair_evidence.py:141 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/supervisor.py:171 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/test/test_stair_evidence.py:195 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_state.py:151 [repository]; /home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/audit-tron1-20260918/selected-tests-resume2.json [audit].

## 11. WebSocket tunnel 단절 후 복구

**BLOCKED — Safety S3 / Mobility M3**

- 최소 필요 조건: 기존 명령/epoch 폐기, 실제 zero/정지와 독점 owner 확인, 새 session의 bounded 연결·mode/status 검증, 필요한 localization 재확인, 새 goal의 명시 재승인. reconnect 자체가 안전성의 증명은 아니다.
- 현재 gate: run.sh는 시작2초 뒤 tunnelPID만 확인하며 운용 후에는 systemPID만 wait한다. send 성공기준0.25초 outage budget을 넘긴 실패는 transport FAULT를 latch하고 socket을 닫는다. supervisor는 FAULT, stream timer는중지. transport.start는 NEW에서만 허용되고 reset/reconnect 없음.
- 불필요하거나 범위를 좁힐 gate: 불확실한 연결로 old motion을 자동 재전송하지 않는 보호는 필요하나, 확인된 새 session까지 영구 금지하고 전체 system 재기동으로만 밀어내는 범위는 좁혀야 한다.
- 성공 가능성: terminal FAULT 후 동일 session의 복구는 코드상 불가능하다. budget 안의 일시 send 실패 회복은 시험돼 있으나 실제 tunnel 재생성 및 펌웨어 rearm 성공률은 UNVERIFIED다.
- false-stop: 일시 네트워크 문제가 threshold를 넘으면 네트워크가 정상화돼도 주행은 계속 차단된다. 실제 단절 검출시간과 로봇 정지시간을0.25초로 보장하지 않는다.
- 복구: 문서에 legacy foreground tunnel 명령은 있으나 managed session을 reset하지 않는다. 공개 reset/rearm 명령은 없다. bounded transport 재생성 및 old epoch 폐기를 구현하고 기존 stop→tunnel→master 순서도 바로잡아야 한다.
- OBSERVED: FAULT latch/NEW-only start/timer shutdown과 run.sh post-ready wait를 확인했다. faults unit은 반복 start를 거부하고 connection factory 호출1회를 기대한다. 이번 pure suite에서 통과했다. cleanup은 tunnel에 system보다 먼저 signal을 보낸다.
- INFERRED: 일시 tunnel 상실이 오래 지속되는 mission 가용성 장애로 확산한다. cleanup 시 zero 전송과 tunnel 해제가 경합할 수 있다.
- UNVERIFIED: 실제 network disconnect 검출 지연, firmware watchdog, 물리정지, 새 session handshake의 실기체 안전성과 성공률.
- 관련 finding: B-02, B-03, C-01; constraint: K09, K10, K51, K52, K53, K59.
- 근거: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:236 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:382 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:202 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/robot_transport.py:81 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/robot_transport.py:177 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/robot_transport.py:185 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/ros_node.py:145 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/test/robot_transport/test_robot_transport_faults.py:167 [repository].

## 12. stair supervisor FAULT 후 복구

**BLOCKED — Safety S4 / Mobility M3**

- 최소 필요 조건: fault 원인의 분류·해소, 계단상 실제 자세/landing/정지 확인, fresh sensor와 독점 command ownership, 이전 goal/token/epoch 격리, fresh admission과 bounded 재승인. 계단 중 자동 재개를 전제하지 않는다.
- 현재 gate: supervisor FAULT에서 NAV명령무시, transportclose, faultedtransportstart거부. missionhealth가FAULT를거부한다. 정상finish는NAV로돌아가지만fault recoveryinterface는없다. checkpointcancel이report.faulted보다앞인경로는별도C-04위반이다.
- 불필요하거나 범위를 좁힐 gate: 확인된 recoverable fault와 현장 구조가 필요한 fault를 구분하지 않고 모든 경우 session 영구폐쇄에 의존한다. 반대로 unsafe checkpoint를 이름만으로 복구 승인하는 조건은 강화해야 한다.
- 성공 가능성: 동일 객체/session의 공개 recovery는 코드상 차단된다. 새 process 생성 뒤 실제 안전한 재개는 확인되지 않았으며 성공 확률을 추정하지 않는다.
- false-stop: 순간sensorgap/일시commfault로terminal에들어가면자료정상화뒤에도진행불가. SAFE_CHECKPOINTcancel이fault보다우선해NAV로돌아가는것은false-stop회복의증거가아니라안전결함이다.
- 복구: README는 안전 위치 점검/수동 복구를 말하지만 reset 서비스나 검증된 rearm 명령을 제공하지 않는다. 기존 owner에 fault 단계와 마지막 확정 상태를 보존하고 원인별 명시 재관측 절차를 넣어야 한다.
- OBSERVED: FAULT latch와close·nonNAV명령거부코드확인. incomplete/faulted LANDING report에서 cancel후NAV로돌아가는pureprobe가있다. README 결과코드표는점검을요구하지만복구API를정의하지않는다.
- INFERRED: 복구가능한일시장애도전체process재생성을요구한다. 계단상안전증거없이NAV복귀하면새명령의위험이남는다.
- UNVERIFIED: fault시실제기체자세,기계정지·계단탈출,조종기/펌웨어상태,현장수동복구성공.
- 관련 finding: B-02, C-04, C-01; constraint: K07, K10, K14, K15, K50, K51, K52, K55.
- 근거: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/supervisor.py:80 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/supervisor.py:171 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/supervisor.py:234 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/robot_transport.py:81 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_state.py:151 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/README.md:275 [repository]; /home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/audit-tron1-20260918/selected-tests-resume2.json [audit].

## 13. camera unavailable flat NAV

**BLOCKED — Safety S1 / Mobility M3**

- 최소 필요 조건: flat NAV에필요한map/pose/TF/scan/odom/commandowner/action연결과물리통로. 해당route가floor/tagtransition을쓰지않으면camera/tag는최소필수조건으로입증되지않는다.
- 현재 gate: 원격sensorreuse는cameraimage/info를요구하며부재시wf_mapping전체restart. wrapper는tagstream/headerfreshness및모든action을전역요구한다. ordinaryruntimehealth는camera를검사하지않는다.
- 불필요하거나 범위를 좁힐 gate: flat-onlyroute에서도camera/image-info/tag와무관stair/floorcapability를전체시작gate로묶는범위, camera실패로정상LiDAR/odom까지restart하는범위.
- 성공 가능성: camera가회복되지않는managedcoldstart는코드상차단된다. 이미ready인NAV는camera단독상실만으로즉시stop시키는코드가없어다른조건정상시계속할수있다는추론. 실기체성공확률UNVERIFIED.
- false-stop: 정상평지주행센서를모두갖췄어도시작차단되거나camera재시작시정상sensor가같이끊긴다. 이것은보이는tag가0개인상황과다르다.
- 복구: 현재는camera/tagstream을복구하고전역wrapper를다시시작하는경로뿐이며camera-onlyrestart명령은repository에없다. 요청capability별gate와실제cameraowner의좁은restart경계를지원해야한다.
- OBSERVED: run.sh:225의image/info5초probe및전체wf_mappingrestartbranch. run.sh:354–360의tagstream/freshheader요구. 운용health는floor/supervisor만검사한다.
- INFERRED: 카메라와무관한flatNAV의coldstart가차단된다. 운용후camera단독loss가그자체로현재NAV즉시정지를보장하지않는다.
- UNVERIFIED: 카메라driver별복구범위,현재실기체flatnavigation성공,외부sensorrestart가정상scan/odom에미치는실측영향.
- 관련 finding: B-01; constraint: K01, K02, K22, K65.
- 근거: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:225 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:348 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:354 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_state.py:151 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/test/test_configuration_transition.py:224 [repository].

## 14. AprilTag 미검출 flat NAV

**CONDITIONAL — Safety S1 / Mobility M3**

- 최소 필요 조건: 정상floor/map/pose/TF/scan/odom/owner와flatroute. 현재route가floor전이를요구하지않는다면보이는AprilTag를항상필수로둘근거는없다.
- 현재 gate: startup은tagmessage type/stream/headerage를검사하며detections배열의비어있음은검사하지않는다. ordinaryhealth도tag내용을검사하지않는다. tagvote는활성floortransitioncontext가있을때만동작한다.
- 불필요하거나 범위를 좁힐 gate: flat-only에서도detectorstream자체를전역필수로두는범위는과도하다. 그러나현재코드가모든flatNAV에서visible-tag vote를요구한다는주장은틀리다.
- 성공 가능성: fresh empty array와다른NAV조건이충족되면코드경로상미검출때문에막히지않는다. stream부재는차단된다. 실제평지주행확률은측정하지않았다.
- false-stop: 태그가단순히시야에없다는이유의false-stop은이flat경로에서확인되지않았다. detector죽음/메시지부재를카메라와무관한NAV까지확대하는startupfalse-stop은존재한다.
- 복구: 빈array는visible-tag복구가필요없다. detector사망은명시3초respawn대상이나camera/info/relay문제는별도다. floortransition이필요하면유효route-tag증거를다시획득해야하며vote삭제를제안하지않는다.
- OBSERVED: wait_for_fresh_message는첫headerstamp를읽고age2초를확인하지만배열길이를읽지않는다. floor tag_callback은epoch/state/context가없으면바로반환한다. detectorrespawn3초와relayrespawn없음을확인했다.
- INFERRED: 정상빈detectionstream은flatstartup내용검사를통과할수있다. 미검출과detector가없음을혼동하면불필요하게운용가능성을낮게평가한다.
- UNVERIFIED: 현재camera/detector가빈array를실제로발행하는주기,현장timestamp상태,실기체NAV성공.
- 관련 finding: B-01; constraint: K01, K02, K04, K16.
- 근거: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:259 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:281 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:354 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/src/multifloor_manager/ros_callbacks.py:61 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/launch/apriltag.launch:19 [repository].

## 15. RViz 종료 autonomous NAV

**CONDITIONAL — Safety S1 / Mobility M2**

- 최소 필요 조건: GUI와독립적인map/TF/localization/scan/odom/move_base/mission/transport건전성. RViz가없어도운영자가결과와정지상태를관찰할수있는기존action/state정보. GUI자체는제어최소조건아님.
- 현재 gate: system.launch는rviz=true기본이나false옵션지원, RViznode는required/respawn설정없음. wrapper는RVizready검사를하지않는다. move_basestartupwrapper는TF문자열검사뒤RVizXMLRPChelper를동기로호출하고helper가nonzero로끝나면warn후planner실행한다.
- 불필요하거나 범위를 좁힐 gate: planner시작을visualizationhelper완료에묶고해당RPC에timeout을두지않은범위. RViz종료후전체master/sensor/navigation을재시작할필요는코드상입증되지않는다.
- 성공 가능성: ready후RViz단독종료는제어code경로상허용된다. RViz부재로helper가즉시오류나면nonfatal이며historicallog도이를기록한다. reachable하지만응답없는helper가startup을막을가능성은있다. 실기체NAV완료확률UNVERIFIED.
- false-stop: 이미돌아가는NAV의RViz단독종료로인한직접false-stop은확인되지않았다. startup에서helperRPC무응답이면실제TF가정상이어도planner실행이대기할수있다.
- 복구: 제어component가정상이라면GUI만복구해야한다. 문서manualRViz명령은directgoal도구가있는별도lane이며managed복구와동일시하지않는다. helper는기존RViz연결갱신일뿐deadRViz기동명령이아니다.
- OBSERVED: system.launch:28,81에optionalRViz와required/respawn생략이있다. 설치roslaunch기본required=false와processmonitor동작확인. wait_for_tf_exec.sh:22–25는helper오류를warning으로처리하고exec한다. helperXMLRPC명시timeout은없다.
- INFERRED: 정상startup뒤RViz죽음이mission/NAVnode를같이죽이지않는다. startup의hangingRPC는move_baseexec를막을수있다.
- UNVERIFIED: 현재실ROSgraph에서RViz종료후완전한주행결과와오류상황,endpoint가실제로hang하는빈도.
- 관련 finding: B-04; constraint: K06, K65.
- 근거: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/launch/system.launch:28 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/launch/system.launch:81 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/scripts/wait_for_tf_exec.sh:17 [repository]; /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/scripts/rviz_tf_reconnect.py:25 [repository]; /opt/ros/noetic/lib/python3/dist-packages/roslaunch/core.py:432 [installed-upstream]; /opt/ros/noetic/lib/python3/dist-packages/roslaunch/pmon.py:560 [installed-upstream].

이번하위작업에서새시험/ROS/SSH/robotcommand0,target쓰기0,temp0. root scenario-status.json은수정하지않았다. JSON/Markdown은target밖retainedauditartifact다.
