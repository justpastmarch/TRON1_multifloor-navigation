# Phase F E2E 시나리오1–7 판정

7개 모두 판정했다. `CONDITIONAL`은 명시한 조건에서 코드 경로가 존재한다는 뜻이며 실행 PASS가 아니다. `BLOCKED`는 현재 안전 운용 승인 또는 요청 운영 경로가 막힌 경우이며 그 이유를 구별했다. 물리 성공은 모두 NOT_EXECUTED/UNVERIFIED다. 기존 시험88개에 추가 실행을 합산하지 않는다. 정확한 주장별 절대경로·줄·해시는 JSON에 있다.

| 번호·시나리오 | 판정 / 코드 경로 | 최소 필요 조건 | 현재 gate | 불필요하거나 좁힐 gate | 성공 가능성 | false-stop | 복구 | S/M · finding |
|---|---|---|---|---|---|---|---|---|
| 1 3F HOME에서 3F 계단 진입점까지 | CONDITIONAL / CONDITIONAL | 동일한3F map/floor/AMCL pose/home_3f anchor; fresh scan·wheel odom·TF,유효costmap·move_base,단일command transport; 목표stair_3f_up_entry까지실제통행가능한공간과NAV goal terminal | startup K01–K08·K22·K34–K36·K65; mission valid goal/단일active(K46–K48), NAV READY/generation/owner와stale-to-zero(K08–K09); planner/geometry/velocity K26–K33,K39–K43,K66 | 계단에진입하지않는NAVIGATE에는camera/tag·다른층stair profile·floor-transition action전체readiness가필요하지않다.; 일반NAVIGATE는recording을시작하지않으므로disk-free gate가항상NAV를막는다고주장하지않는다. | 코드경로상조건부가능. 성공률숫자없음;실제장애물·firmware·전체runtime은UNVERIFIED. | camera/topic공백또는무관site설정불일치로startup전체차단; TF문자열대기·floor/supervisor stale; child terminal소실시부모BUSY 지속 | 일반NAV실패는최대2attempt이며첫실패후costmap clear. fresh command는NAVzero에서회복한다. transport FAULT 또는hung child는현재session/process재생성이필요하다. | S2/M3 · B-01,B-04,C-01,C-11,D-01 |
| 2 좁은 통로를 포함한 flat navigation | UNVERIFIED / CONDITIONAL | 시나리오1의NAV필수조건; 기체외곽·부착물·localization오차를포함한실측통로여유; global path와local footprint trajectory가함께유효하며후방escape시실후방coverage존재 | radius0.28m,padding0.02m,5cmgrid,inflation0.33m/costscale4.0; local obstacle1.3m/raytrace1.4m,4×4m rolling map; sim_time1.2/12×24samples,goal0.25m/0.2rad,planner5s/oscillation10s,recovery disabled; 일반min_vel_x0과별도upstreambackup-0.1의차이 | 전역camera/tag조건은평면NAV에불필요하다.; padding/inflation/속도가과도하다는실측증거는없으므로임의축소또는REMOVE를권고하지않는다. | 실제통로성공가능성UNVERIFIED. 분석footprint폭약0.588–0.617m는연속기하모델값이며planner가보장하는최소통로폭이아니다. | global점비용계획은가능해도local footprint/샘플링은실패할수있음; 좁은구역회전/작은오차에서oscillation·planner patience 후abort; 정상TF/scan공급이불안정하면costmappose조건에서정지 | 물리외곽을유지하고기존padding/costmap/planner설정과제한된평면복구를측정으로조정한다. 넓은범위의blind recovery 활성화는최소개선으로입증되지않았다. | S2/M2 · B-01,D-01,E-02 |
| 3 3F→4F stair traversal | BLOCKED / CONDITIONAL | 3Fstair entry에서확인된정지·위치·각도와독립UP실측profile; 단일commandowner·검증된mode전환·freshodom와slip/landing구분가능한관측; 4Ftarget tag400,4Fmap과landingpose,AMCL/scan/odom/TF·costmap동기화 | 진입후새AMCL3개/2초준비와1초단일사용admission; VERIFY_ENTRY→ALIGN→flight1→LANDING→TURN→flight2→EXIT_CONFIRM; odomgap0.12s/freshness0.20s/step0.10m/yawstep0.20rad;profile전체300초; tag3회/1초/0.5초freshness→change_map→pose/nomotion→policy floor/localized/costmap | 정지후요구pose를스스로갱신시키지않는진입대기는C08 false-stop이다.; 같은epoch의이미확정floor를후속map/localization시간만으로10초후만료시키는C05조건은좁혀야한다.; tag와AMCL은층identity와localization을각각검사하므로중복이라는이유로둘다제거할수없다. | 코드경로는조건부이나안전운용BLOCKED;physicalsuccessUNVERIFIED. 과거bag·syntheticphase완료는현장독립검증을대체하지않는다. | 정지AMCL새표본부족→entrytimeout; 0.12sodomgap1회→stairFAULT; tag/map/AMCL지연→10초floorobservation만료→floorFAULT; cancel준비경쟁또는childterminal미수신 | 먼저기존stationary/LANDING/취소우선순위를수정하고실정지·착지근거를검증한다. 현재stair/floorFAULT는자동회복이없으므로공개재관측/epoch재진입경로가필요하다. | S4/M3 · C-01,C-02,C-03,C-04,C-05,C-08,C-10,E-03 |
| 4 4F landing에서 다음 stair entry까지 | CONDITIONAL / CONDITIONAL | 실제4Flanding과confirmedanchor일치; 현재4Fmap/generation,유효AMCL/scan/odom/TF/costmap; stairmode해제와NAVowner반환·freshcommandtransport | 시나리오1의NAVhealth/floor/generation/owner/geometrygate; 이전STAIR뒤FLOOR_TRANSITION성공에서만4Ftargetanchor확정; 선택return은confirmedanchor에서새directedBFS | 이평면segment자체에는추가tag재관측/camera가필요하지않으나managedstartup은전역으로요구한다.; 이전전이일시실패를floor영구FAULT로고정하면재관측후이평면이동도불가하다. | 단방향이동은코드상조건부가능;물리통로와4Fpose는UNVERIFIED. return_after_task가붙으면생산graph로HOME복귀는불가. | 이전floorFAULT/freshness만료로NAV시작거절; 다음entry도달후return BFS가nodirectedroute; NAVterminal소실로후속missionBUSY | 확정floor/pose/anchor를재관측해READY로복구하고별도확인된역방향NAVedge를추가해야한다. 임의anchor교체나미확인경로역주행은복구가아니다. | S2/M3 · C-01,C-02,C-05,C-09,C-11 |
| 5 4F→5F | BLOCKED / CONDITIONAL | 4Fstair_4f_to_5f의검증된정지/정렬/anchor; 4F5FUP독립실측profile·착지/정지관측·단일owner; 5Ftargettag500,5Fmap/landingpose와localization/costmap전이 | 3번과동일한admission/post-fence/phase/odom/timeouts; stair_4f_5f_up은stair_10_9_upYAMLanchor를상속; 5Ffloor전이후confirmedanchor=stair_5f_from_4f | C08정지pose공급없는복수표본대기와C05one-shotfloor만료; 4F/5F와무관한층bundle/readiness동시요구 | 코드상conditional;실장비안전운용BLOCKED. profile공유가틀렸다고확정하지않으며서로다른계단의동일성은UNVERIFIED. | entrypose새표본대기·짧은odomgap·finalfloorstale; 실제계단progress와profiledistance불일치시timeout/오인완료; floorFAULT후다음5FNAV거절 | 3번의기존gate수정·독립4F5FUP/지원되는DOWN검증·floor/transport재관측복구가선행돼야한다. 현재자율재시도만으로안전복구보장없음. | S4/M3 · C-01,C-02,C-03,C-04,C-05,C-08,E-03 |
| 6 5F에서 직접 시작해 RF 이동 | BLOCKED / BLOCKED_IN_DEFAULT_OPERATOR_PATH | 5Fmap·initial_floor5F·실제known5Flocationpose·같은initialanchor의원자적설정; 5Fentry까지NAV가필요하면그route와NAV최소입력; 5F→RF독립계단profile/owner/landing/targettag600 및RFfloortransition | systemmapdefault3F와initialflooridentity일치시에만startupREADY; initial_location_id/AMCLinitialpose는독립입력; 5Flanding→5FentryNAV 및5Fentry→RFSTAIR/FLOORedge는존재 | 5Fknownlocation을선택해도map/pose/anchor를중복수동입력해야해정상5F시작을쉽게차단한다.; 3FHOME default를물리적필수시작층으로오인할필요는없다. | 현재run.sh의floor/location변경만으로는BLOCKED. 기존system.launch map_yaml등을일관되게제공하면소프트웨어경로는가능하지만안전승인은3번과동일하게BLOCKED,physicalUNVERIFIED. | 3Fmap과5Fidentity불일치→floorUNKNOWN→startupREADY실패; 잘못된5Fanchor이면잘못된route또는빈route무이동성공; 계단C04/C08/floorC05와RF후HOMEreturn누락 | 기존설정loader에서knownlocation→map/floor/pose/anchor를원자적으로해결하고현pose확인후시작한다. 현재floorUNKNOWN에서일관된map관측은초기READY경로가있지만audit에서변경하지않았다. | S4/M3 · C-04,C-08,C-09,C-11,B-01 |
| 7 임의 4F pose에서 localization 후 navigation | BLOCKED / BLOCKED_IN_REQUESTED_AUTOMATED_WORKFLOW | 올바른4Fmap과flooridentity; 실측/불확실도를포함한initialpose 또는독립global-localization수렴검증; 대칭복도의오수렴을배제한현위치확인과도달가능한graphanchor연결; 그뒤NAV최소입력/transport와route | run.sh가fixed config.env에서독립pose/location을읽고알수없는start-option은거절; 초기floorREADY는mapidentity만검사하며actualpose-anchor대조없음; planner는등록된origin/destination ID만사용 | 현재floor와무관한전체camera/stair/site startup조건; knownanchor입력만으로위치획득을대체하는수동중복설정은불필요한운영부담이면서safety공백 | 요청한자동workflow는BLOCKED. 실측pose·map·anchor를외부에서일관되게확정하고기존launch입력을설정한known-locationNAV는조건부가능하나이번시나리오의자동획득완료가아니다. | 4Ffloor만변경하면3Fmapdefault와불일치; origin을임의좌표로주면named-ID계약에서거절; 등록anchor를임의로선택하면실제위치와다른route또는동일destination빈routefalse-success | 기존startup 설정/관측경로에map·floor·pose·anchor의원자적선택을추가하고localization/anchor확인실패는재관측으로돌린다. 새node나맹목적nearestanchor추정을필수로제안하지않는다. | S2/M2 · C-11,B-01 |

## Evidence 구분

**1. 3F HOME에서 3F 계단 진입점까지** — 등록된 HOME와실제pose가일치하고필수NAV입력/transport가정상일때단일NAV경로가존재한다. 실물주행성공은검증하지않았다.

OBSERVED: home_3f→stair_3f_up_entry NAV edge가있고두location은3F다. success이면anchor를confirmed target으로옮긴다.

INFERRED: 독립적인camera고장도이평면임무의managed startup을막는다.

UNVERIFIED: 현재route 물리통행,localization정확도,실제정지거리·장비명령수신.

근거: [run.sh:70](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:70), [run.sh:348](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:348), [ros_state.py:151](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_state.py:151), [navigation_executor.py:239](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/navigation_executor.py:239), [building_graph.yaml:8](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/config/building_graph.yaml:8). 전체 근거는 JSON.

**2. 좁은 통로를 포함한 flat navigation** — 현재지도에서격리planner를실행하거나실측통로폭을대조하지않았으므로좁은통로통과/차단을확정할수없다. inflation중첩만으로BLOCKED라고판정하지않는다.

OBSERVED: repository navigation 값과공식upstreamcost/footprint처리를확인했다. 모델상inflation은7cell이며외측softcost를모두금지영역으로볼수없다.

INFERRED: global/local다른충돌모델및복구정책이좁은통로false-stop을만들수있다.

UNVERIFIED: 실측몸체/통로폭,PGM pixel상특정경로,현재설치binary와공식source동등성,liveoverride.

근거: [costmap_common_params.yaml:1](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/config/nav/costmap_common_params.yaml:1), [local_costmap_params.yaml:1](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/config/nav/local_costmap_params.yaml:1), [global_costmap_params.yaml:1](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/config/nav/global_costmap_params.yaml:1), [base_local_planner_params.yaml:1](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/config/nav/base_local_planner_params.yaml:1), [navigation.launch:47](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/launch/navigation.launch:47). 전체 근거는 JSON.

**3. 3F→4F stair traversal** — STAIR/FLOOR코드경로는존재하지만C04정지·landing·cancel반례때문에현재설정의안전한실장비운용승인을할수없다. 소프트웨어가항상실행불가능하다는판정은아니다.

OBSERVED: 3F4FUP edge/profile enabled,4F도착tag400으로구성돼있다. 수용step상한이stationarytolerance보다작고checkpointcancel이fault/complete보다먼저평가된다.

INFERRED: 연속이동/불완전landing에서turn 또는NAV반환가능;정상stair에도falsefault가능.

UNVERIFIED: 실제계단치수·미끄러짐·독립UPcommissioning·firmwaremode/정지반응.

근거: [route_planner.py:204](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/route_planner.py:204), [ros_segments.py:140](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_segments.py:140), [stair_entry_gate.py:115](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/stair_entry_gate.py:115), [stair_evidence.py:96](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/stair_evidence.py:96), [supervisor.py:168](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/supervisor.py:168). 전체 근거는 JSON.

**4. 4F landing에서 다음 stair entry까지** — 4Ffloortransition이정상READY로완료되고anchor가stair_4f_from_3f이면다음entry까지NAV edge가있다. optionalHOMEreturn은역방향NAV누락때문에별도로BLOCKED다.

OBSERVED: stair_4f_from_3f→stair_4f_to_5f NAV edge가있다. 현재production에는4Fentry→4Flanding reverseNAV가없다. STAIRsegment의confirmed_location은source이고FLOOR_TRANSITION성공후target으로옮긴다.

INFERRED: return계획시점이outbound뒤라목적지도달뒤에야복귀불가가노출될수있다.

UNVERIFIED: 4F실제landing/entry사이통로·지도정확도·현재localization.

근거: [run.sh:70](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:70), [run.sh:348](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:348), [ros_state.py:151](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_state.py:151), [navigation_executor.py:239](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/navigation_executor.py:239), [building_graph.yaml:16](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/config/building_graph.yaml:16). 전체 근거는 JSON.

**5. 4F→5F** — 4Fentry→5Flanding경로는있지만3번과같은C04안전반례가적용된다. sharedprofile이실제4F5F치수와맞는지는독립검증이없다.

OBSERVED: 4F5FUP/DOWNedge와target500구성존재 4F5FUPprofile은3F4F와같은YAMLanchor를상속 동일stairtracker와floorstate경로사용

INFERRED: C04/C05/C08 rootcause가층만바뀐이경로에도그대로적용된다.

UNVERIFIED: 4F5F독립profile치수·traction·landing확인·physicalcompletion.

근거: [route_planner.py:204](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/route_planner.py:204), [ros_segments.py:140](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/ros_segments.py:140), [stair_entry_gate.py:115](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/stair_entry_gate.py:115), [stair_evidence.py:96](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/stair_evidence.py:96), [supervisor.py:168](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/supervisor.py:168). 전체 근거는 JSON.

**6. 5F에서 직접 시작해 RF 이동** — run.sh에서floor/location/pose만5F로바꾸면map_yaml이전달되지않아systemdefault3Fmap이남는다. 자동5F시작경로는없으며직접일관된launch구성을만들어도STAIR안전C04는남는다.

OBSERVED: run.sh가initial_floor/initial_location/initialpose는전달하지만map_yaml은전달하지않는다. system.launch 기본map은3F이며floorcallback은initialidentity와일치할때READY다. 5F/RFgraph와map설정은존재한다.

INFERRED: floor만5F로바꾼일반운영은identitymismatch로시작하지못한다.

UNVERIFIED: 수동으로정합설정한5F현장시작·RF계단실측·localization·실제물리성공.

근거: [run.sh:245](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:245), [system.launch:3](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/launch/system.launch:3), [ros_callbacks.py:36](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/src/multifloor_manager/ros_callbacks.py:36), [floors.yaml:5](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/config/floors.yaml:5), [building_graph.yaml:30](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/config/building_graph.yaml:30). 전체 근거는 JSON.

**7. 임의 4F pose에서 localization 후 navigation** — 현재운영표면에는start-floor auto-localize 또는임의pose에서검증된graphanchor로연결하는절차가없다. AMCL upstream기능가능성과repository의완성된운영경로를구분한다.

OBSERVED: 현재CLI에start-at/start-floor/start-pose/auto-localize옵션이없다. mission은settings.initial_location_id를즉시anchor로채택하고namedgraph에서만계획한다. 초기mapidentityREADY와poseconvergence/anchorconfirmation은연결돼있지않다.

INFERRED: 임의4Fpose를기존anchor로가정하면물리위치와logicalroute가달라질수있다.

UNVERIFIED: 대칭4F복도global-localization수렴,nearestanchor연결의실제통행·정답성,현재실장비초기pose.

근거: [run.sh:8](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:8), [run.sh:245](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/run.sh:245), [system.launch:6](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/launch/system.launch:6), [ros_callbacks.py:36](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/src/multifloor_manager/ros_callbacks.py:36), [mission_orchestrator.py:41](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/mission_orchestrator.py:41). 전체 근거는 JSON.

Artifact: 대상 쓰기0,새시험0,ROS/SSH/robot command0,backgroundprocess0,임시파일/cache0. 본 MD/JSON2개는감사산출물로보존한다. shared scenario-status와root보고서는변경하지않았다.
