# Phase E final — test and evidence coverage

## 이번 감사의 실행과 수집 실패

OBSERVED: 실제 실행은 **88 tests /85 pass/3 failure/0 error**이다. 70 pure-unit와18 read-only contract 실행의 합이며 아래 유형별 행들은 중첩돼 별도 합산하지 않는다. 전체 suite·hardware acceptance 수치가 아니다.

| 이번 실행 원장 | 실행/성공/실패 | 증거 |
|---|---:|---|
| selected-unit-results.json |10/10/0|:8–13, FSM·stair entry·subprocess seam3module|
| selected-contract-results.json |6/5/1|:3–11, 삭제된 pointcloud_to_laserscan.launch 기대값|
| resume2-unit-results.json |60/60/0|:2–5, stdout60 tests OK를 보존; 뒤 독립 probe import 실패와 분리|
| selected-contracts-resume4.json |12/10/2|:8–22, robot max_vel_x0.50 기대값과 admission_token/ENTRY_REJECTED 누락|

OBSERVED: **테스트 실행 전 로드 시도가 적어도2회 중단됐으며 각각 실행0개**다. selected-unit-results.json:12에는 generated stair_supervisor.srv 부재로 초기4module suite를 로드하지 못했다고 명시한다. phase-E-resume2.md:9에는 uuid의 platform 조회 subprocess를 guard가 막아0개 실행으로 중단한 뒤 표준 module을 미리 로드하고 동일guard하에60개를 실행했다고 명시한다. 후자의 임시root는 resume2-loader-attempt0-cleanup.json:17–18에서 root_removed=true와 remaining_threads=[]로 정리됐다. 따라서 CHECKPOINT의 “loader 중단은0개”는 **“로드 중단 시도는 실행0개여서88개에 합산하지 않았음”**으로 고쳐야 한다. 로드 실패가 없었다는 뜻이면 사실과 다르다. 이 문구 정정은 테스트 합계를 바꾸지 않는다.

E-02, Safety S2 / Mobility M2: 현재 실패3개는 test_system_operator_contract.py:46의 삭제된 scan launch, test_legacy_rviz_viewer_contract.py:103의0.50, test_interface_contract.py:120의옛wire계약이다. current code를 오래된 assertion에 맞춰 되돌릴 근거가 아니다. 현재 설치·runtime wire가 실제 mismatch라는 주장은 별도 검증 없이는 UNVERIFIED다.

## 유형별 적용 범위

아래 source 존재와 assertion 내용은 OBSERVED다. “미실행”은 이번 감사에서의 상태다. 이전 source 전독해 coverage에 더해 이번에는 root/test와3package test/transport의 test*.py71파일 및 모든372개 test명 정의를 AST 인덱스로 대조하고 핵심 본문을 재확인했다. 71에는 helper가 포함되며372는 parametrization/subTest를 펼친 실행 건수가 아니다. 이 인덱스를 새 시험 실행이나 새 READ 근거로 사용하지 않았다.

| 유형 | 대표 원문 및 검증하는 경계 | 이번 감사 실행/관찰과 남은 한계 |
|---|---|---|
| Unit | mission test_fsm.py:40,69,103; test_navigation_executor.py:107,133,211; stair test_supervisor_safety.py:71,149; transport test_robot_transport_faults.py:126 | 선택 pure70개 실행. 메모리 fake, 통신/실행 guard. 물리 이동·network timing 미검증 |
| Contract | test_interface_contract.py:104; test_process_boundary_contract.py:14; test_workspace_layout.py:28; test_command_provenance_contract.py:11 | 선택18개 실행,3실패. 문자열/XML/필드 존재는 실제 authorisation·owner exclusivity 증명과 다름 |
| Configuration | test_site_configuration.py:59,125,215; test_stair_endpoint_configuration.py:42,84,95; test_runtime_gating.py:14,26; test_configuration_transition.py:170 | parser/reference/fixture gate가 존재. map_loader의2개가pure60에포함. commissioned geometry·enabled profile 승인 증명아님. runtime_gating:22의configured:false 가정은 현재자료와 시점분리 |
| Launch | test_launch_contract.py:57,99,182; test_system_operator_contract.py:13,38;각CMakeLists등록 | XML/topology계약 실행. 실제launch는 실행하지 않음. mission CMakeLists:60–79에process/recording/mission/5F-RF/full-system synthetic 등록, multifloor:73–95에6rostest등록, stair:68–71에node rostest등록은 직접확인 |
| Integration | mission test_mission_action_ros.py:93,116,274; test_mission_5f_rf_synthetic_ros.py:99–161; test_full_system_synthetic_ros.py:80; floor test_floor_transition_epoch_ros.py:138–249 | source검토만. loopback/fake child/synthetic peers 성공은 실제 move_base path feasibility·sensor independence·site readiness를 증명하지 않음. test_full_system_synthetic_ros.py:29의fixture 경로는 기존 E-02 범위 유지 |
| ROS graph | test_process_boundaries.py:86–136, test_process_boundary_contract.py:14; phase-B.md의 ownership/runtime observation | 이번환경ps는 matchingrows없음,ss는권한거절(02:42:09UTC). 실제host/miniPC graph를조회·인증한것아님. actionlib upstream의connection gate근거와현재실행graph는별개 |
| Replay | test_bag_replay.py:11,36; test_ros_bag_replay.py:74; test_replay_stair_state_machine_contract.py:9; test_stair_evidence.py:195 | timestamp/pose 변환, terminal snapshot,wrapper문자열,recorded gap/jump축약fixture테스트. 이번bag21metadata와2payload를read-only분석했으나ROSreplay실행0. 실물outcome독립oracle없음 |
| Hardware | docs/stair-up-commissioning.md:19–22; .omo/start-work/ledger.jsonl:80,117–122 | 과거 manual joystick 및 STAND/WALK startup통신흔적은존재. 현재장비/현장commissioning검증0. sibling held-out0scored/nominal0success/robustness0success/PENDING_HARDWARE를합격으로세지않음 |
| Manual-only | README.md:108,252,279,322; docs/stair-up-commissioning.md:19–22;64원본JPEG와video metadata | 수동 commissioning·fault검토절차/관찰자료. 시험담당자·계단geometry·정지거리·슬립·외부원격controlownership은미검증. 프레임overlay/filename/time표시를독립측정으로인정하지않음 |

## 정상 주행과 복구의 coverage gap

다음은 “관련 시험이 전혀 없다”는 주장이 아니다. existing positive/fault tests가 아래 정상 운용 종료 조건을 입증하지 못한다는 범위다. source전독해,등록표,테스트본문과현재코드반례를함께대조했다.

| 질문 | 이미 있는 검증(OBSERVED) | 아직 증명되지 않은 종료 조건 / 관련 finding |
|---|---|---|
| 좁은 통로 통과 | test_launch_contract.py:57–97는costmapplugins와inflation설정존재검사 | 실측footprint·통로폭·localplanner동작으로성공조건검증없음. collision/clearance는UNVERIFIED, tuningconstraint를단순히완화해합격시킬수없음. Safety S2 / Mobility M2,기존Dconstraint범위 |
| scan잠깐중단뒤재개 | test_readiness.py:92–110는stale scan/odom독립거부;test_mission_5f_rf_synthetic_ros.py:132는stale localization주입 | 신선한scan복원후안전한same-floormission을자동또는명시rearm으로재개하는E2E미증명. C-02 Safety S2/M3, D-01 Safety S2/M1 |
| odom공백/점프뒤복구 | test_stair_evidence.py:195–212는gap/jump뒤후속sample도FAULT유지;:134–150는큰이동시dwellreset | 적절한zero·pose/epoch재동기화·freshadmission뒤회복하는workflow미증명. 반대로영구latch는테스트가명시하는현재정책. C-04 Safety S4/M2, C-02 Safety S2/M3 |
| tunnel/transport재접속 | test_robot_transport.py:228은budget내transient복구, faults.py:126–188은budget초과및disconnect후start재시도금지 | terminalFAULT이후새연결·기존명령폐기·zero/ownership확인·사용자rearm경로없음. 자동재접속의실물안전성도UNVERIFIED. B-02 Safety S1/M3 |
| cancel뒤새goal | test_mission_action_ros.py:116–137,274–294는cancel→새goal/repeatedinterruptions; navigation_executor.py:211–255는retryreadiness race | cancel준비단계와childstart원자성(C-03),child가terminal을영원히주지않는C-01,stationary검증전checkpoint반환C-04는별도경로다. existingcanceltests가없다고쓰면틀림. C-03 Safety S3/M2, C-01 Safety S2/M3 |
| floorFAULT복구 | epoch_ros.py:138–249는FAULT뒤lateinternalcallback차단,late_map_ros.py:43는늦은map/timeout;mission_action.py:139는통신실패후floor호출차단 | FAULT에서flooridentity복원·새map/localizationtransaction·missionrearm성공을증명하지않음. 외부service가이미수행한change_map부작용은callbackfence와별개. C-02 Safety S2/M3, C-10 Safety S3/M2 |
| 일부capability없을때평지운용 | test_configuration_transition.py:170–238는empty stair profiles로NAV허용,stairrequest만CAPABILITY_DISABLED;route_planner.py:88은disabled방향배제 | 실제wrapper가camera/tag/stairreadiness부재를격리한채평지navigation완료하는시험없음. 라이브러리partialcapability성공과wrapperglobalgate충돌. B-01 Safety S1/M3 |

E-03, Safety S2 / Mobility M2: feedback phase로센서를만드는 test_stair_supervisor_node.py:167–181 및 test_stair_fake_websocket_ros.py:79–89는제어출력과oracle독립성이없다. 이들은action/wire/state계약에유용하지만,계단참정지·flightdistance·각도·footcontact의현실성을증명하지못한다. production `/mission/result`필수topic과fixture누락(C-07),productionreturnedge누락(C-09),10초floor-confirmedevent만료(C-05)를fixturePASS로반박할수없다.

## 과거 결과·종료 상태

OBSERVED: 과거150tests/9errors/1failure는ledger:66–70 및phase-E-resume6,과거modular sourced30OK/registered25OK는phase-E-resume7에적용저장소와실패→수정순서가구분돼있다. 이를이번88개에합산하지않는다. 이번선택88개로전체372정의의비율을계산하지않으며실행coverage%를제공하지않는다.

INFERRED: negative rejection 시험에비해fault후다시정상임무를끝내는recovery acceptance가약하다. 현재반례를통해개별경로를수정한뒤각장애의복구후정상종료까지검증해야한다. 추가node나전면재설계를전제하지않는다. UNVERIFIED: 실물정지거리·하강·미끄럼·좁은통로·완전한sensorrecovery·외부commandownership.

이번하위작업은read-only재검토와보고서작성뿐이며새tests/ROS/SSH/process실행0,임시artifact0,target쓰기0이다. Phase E의testtaxonomy와한계를마감했고Phase F시나리오판정은parent가통합한다.
