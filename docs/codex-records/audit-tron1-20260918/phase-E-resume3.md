# Phase E 세 번째 재개 — 진행 중

범위는 현재 worktree이며 대상 파일을 수정하지 않았다. `resume-3-baseline.json`: 2026-09-18T03:56:32 UTC에 Git status/plain diff/staged diff 일치, baseline hash 변경 없음. A–D의 정적 분석을 유지하면서 E의 문서·녹화·테스트 근거를 보완한다. F는 아직 시작하지 않았다.

## E-04 보강: command-containing bag과 운용 master 재사용

Safety S4 / Mobility M3. OBSERVED: `replay_bag_rviz.sh:28,43-61`은 기본 localhost:11311의 기존 master를 재사용하고 use_sim_time을 설정한 뒤 전체 topic을 재생한다. `docs/ROSBAG-RECORD:162-184`의 녹화 목록에도 goal/속도가 포함된다.

관찰 명령: 설치된 `/opt/ros/noetic/lib/python3/dist-packages/rosbag/bag.py`의 `Bag(path, 'r', skip_index=True)`로 모든 21개 bag의 connection/chunk/topic metadata를 조회하고, 선택한 2개 bag의 메시지는 `Bag(path, 'r').read_messages(topics=[...])`로만 읽었다. publish/play/reindex는 실행하지 않았다. read/open/close 구현(:404-475,713-723,1445-1471)을 확인했다.

관찰 시각: metadata 2026-09-18T03:57:54.843237 UTC. 선택 메시지 관찰 시각은 `bag-selected-observations.json`.

관찰 결과: `stair_captures/stair_3F_to_4F_FULL_20260916_234018.bag`에 move_base goal 4개, navigation/cmd_vel 801개가 실제로 있다. linear.x 범위 -0.1~0.5000000000000002, angular.z 약 ±0.8이다. 기본 입력인 manual bag에는 goal/command topic이 없으나, 도구는 인자로 이 stair bag을 받는다. 실제 command 재유입과 clock/TF 오염은 INFERRED 위험이며, 실제 robot replay는 하지 않았다. bag의 command 기록은 물리 실행을 입증하지 않는다.

최소 조치는 기존 master 거부, 전용 master 소유·주소 확인, 관측 topic allowlist, 정상 child wait/cleanup이다. 신규 navigation gate는 필요 없다. `exec rosbag`이 EXIT trap을 대체하는 기존 근거도 유지한다.

## E-02 보강: 과거 운용 설명이 현재 구성·원본 bag과 불일치

Safety S2 / Mobility M2로 상향한다. 같은 문서·evidence 갱신 누락 root에 통합한다.

OBSERVED: `docs/navigation-perception-network-incident-rca-ko.md:449,461,632`는 지정한 실패 BAG에 음수 cmd_vel이 없고 escape fallback도 command로 나타나지 않았다고 단정한다. 원본 bag을 읽으면 총 74개의 음수 linear.x(-0.1) 메시지가 있다. goal ID별 음수 개수는 22/17/2/33이다. 문서가 분석한 23:42:45.204 시작 시도에도 2개가 있다. `bag-nav-attempts.json`에 각 시각·goal ID를 보존했다. 어느 planner 분기가 만들었는지는 이 topic만으로 확정하지 않는다. upstream fallback과 값이 맞는다는 것은 INFERRED이며, 실제 hardware 후진은 UNVERIFIED다.

이 bag의 전진 최고 기록은 0.5이고 현재 launch 기본값은 0.3이다. 과거 bag이 현재 설정과 같다는 가정을 배제한다. 문서의 원인 단정으로 production min_vel_x부터 바꾸기보다 해당 실행 parameter와 planner/costmap 근거를 대조해야 한다. 실제 plan/costmap topic은 이 bag에 없으므로 벽 접근 최초 원인은 여전히 UNVERIFIED다.

OBSERVED: `docs/transition-policy-state-machine.md:23-30,512,789` 및 `.omo/evidence/5f-rf-autonomous-route-repetition/COMMISSIONING_STATUS.md`는 production disabled를 현재 상태로 서술하지만 현재 YAML은 enabled다. 문서의 FloorTransition goal 설명은 ownership epoch 필드를 누락한다. `docs/ROSBAG-RECORD`는 과거 robot-side master 주소와 compressed detector 계약을 유지하지만 최신 run/launch는 workstation master와 raw relay다. 수동 mode 문서라는 차이는 인정하되, 운영자가 이 주소·정책을 현재 managed mode에 섞으면 실패할 수 있다(INFERRED).

README:375의 rate20 예시는 replay wrapper의 최대4와 불일치한다. replay의 timestamp 갱신·rate 변경은 실제 sensor clock/gap 검증을 대체하지 못한다. 해당 명령은 실행하지 않았다.

최소 조치: 실제 활성 구성과 각 문서의 적용 시점·mode를 명시하고, bag claim은 시간 구간/goal ID/추출값과 연결한다. 광범위한 재설계 없이 기존 문서와 검증 계약을 정리한다. Required test: 문서 예시의 옵션/구성 계약, 원본 bag 음수 count, 새 runtime와 historical data provenance 구분.

## 원시 evidence와 임계값의 구분

OBSERVED: latest manual `.active.invalid`의 metadata count는 보존 capture JSON과 모두 일치한다. 확장자는 recorder finalize 실패를 나타내며 파일은 read-only로 조회 가능했다. 이를 정상 acceptance로 승격하지 않는다. manual bag에 mission result/floor state/tag detections가 없다.

선택 bag에서 기록 시각과 header 시각 간격을 분리 계산했다. 20260916_234018 bag odom header 최대 gap 0.221392s, 0.12s 초과5회; 기록 간격 최대0.186899s, 초과6회. manual bag odom header 최대10.390351s, xy step 최대1.919271m(0.1m 초과2회); scan header 최대4.999356s, 기록 간격 최대13.322856s. 단순 평균 rate는 이런 gap을 설명하지 못한다. 정확한 onset/phase·기체 움직임·publisher clock·전송/녹화 지연은 이 집계만으로 분리되지 않는다.

따라서 gap 보호를 REMOVE 또는 무조건 크게 완화할 근거는 없다. 현재 0.12s 임계가 이 데이터에 비춰 여유가 작은 것은 OBSERVED 비교지만, 실제 supervisor callback에서 동일 fault가 발생했다거나 모든 초과가 무해한 jitter라는 주장은 하지 않는다. 좁은 recovery와 임계 측정이 필요한 기존 TUNE 판정을 유지한다.

## Coverage와 남은 조사

모든 bag은 METADATA_ONLY다. bulk binary payload 전체 제외는 사용자 허용 범위이며 일부 메시지 추출을 전체 READ로 세지 않는다. 현재까지 문서 전체, 도구 원문, 추가 package/launch 계약, capture report를 읽었다. 파일 목록과 상태의 기준은 coverage.json이다.

남은 E: 나머지 test, notebooks/RViz, .omo plan/evidence, 기존 log/image/CSV의 의미 검토. 신규 조사: 과거 commissioning 후보와 현재 enabled stair profile의 provenance, TF/network historical claim, startup map/floor/pose/anchor의 원자성, command boundary 보안. F: 15 E2E 모두 PENDING. Safety/Mobility 최종 판정은 아직 미확정이다.

## C-11: 시작 지도·층·pose·logical anchor가 독립 입력

Safety S2 / Mobility M2. OBSERVED 소스 계약, 실제 오위치 주행은 UNVERIFIED.

`run.sh:245-255`는 INITIAL_FLOOR, INITIAL_LOCATION_ID, NAV_START_*를 별도로 전달하지만 map_yaml은 전달하지 않는다. `src/mission_manager/launch/system.launch:6`은 floor_3F.yaml을 기본값으로 갖는다. `multifloor_manager/ros_callbacks.py:36-59`에서 초기 floor READY는 initial map identity 일치로 열리고, initial pose/현재 실제 floor의 독립 검증은 없다. `mission_manager/mission_orchestrator.py:43`은 설정 location을 즉시 LogicalAnchor로 신뢰한다. 같은 anchor로의 빈 route는 :88-89에서 health/pose 확인 없이 성공을 반환한다. NAV readiness는 `navigation_executor.py:239-250`의 floor/generation와 supervisor state 검사다.

따라서 3F HOME은 알고리즘의 구조적 필수 조건은 아니지만 현재 wrapper의 지도 default와 연결돼 있다. floor만 5F로 바꾸면 map은 3F인 채로 남아 초기 READY를 얻지 못할 수 있다(INFERRED; map identity는 실제 구별됨). known location ID를 바꾸어도 pose가 자동으로 따라오지 않는다. 동일 층 임의 위치에서 기존 anchor를 그대로 신뢰하면 잘못된 logical route 시작점 또는 이동 없는 성공을 낼 수 있다(INFERRED). arbitrary pose/global localization/nearest-anchor 선택은 이 시작 경로에 구현돼 있지 않다.

최소 개선: 기존 wrapper와 runtime 설정 로딩에서 location→floor→map→pose를 한 번에 결정하고 불일치를 시작 전에 거부한다. 임의 pose는 localization 확인 뒤 유효한 graph 연결점을 정해야 하며 nearest-distance만으로 같은 복도/접근 가능성을 보증하지 않는다. floor-only 자동 localization의 현장 수렴 기준은 UNVERIFIED로 남긴다. 새 node/state를 요구하지 않는다. Required test: 등록된 4F/5F 시작, floor만 변경한 입력, pose와 anchor 불일치, 빈 route의 성공 조건.

## Notebook·기존 로그의 추가 교차검증

OBSERVED: 3개 운영 notebook의 모든 cell/source/output/metadata를 읽었다. checkpoint 3개는 이미 읽은 원본과 byte-level 전체 diff를 검토했다. 03은 완전히 같고 01/02는 모든 차이를 읽었으므로 동일 본문을 다시 출력하지 않았다. notebook은 실행하지 않았다.

E-02(S2/M2)에 통합: `notebooks/02_pose_and_nav_goal.ipynb` cell7은 managed runtime과 같은 production readiness라고 서술하지만 cell8은 pose timestamp 증가와 covariance 상한만 검사한다. finite/음수 covariance, scan/odom freshness·stationarity, TF conjunction은 검사하지 않는다. cell4의 node/topic 검사는 등록 정보이며 supervisor message는 출력만 하고, 실제 supervisor NAV/connected 검사는 goal cell11에서 수행한다. manual mode의 별도 현장 확인 지시와 ARM_MOTION=false는 인정하되 이 경로를 managed readiness와 동등하다고 설명하면 안 된다. 초기 baseline pose는 nomotion service 호출보다 먼저 기다리므로 정지 AMCL의 새 message 부재 시 스스로 갱신을 요청하기 전에 timeout될 가능성도 있다(INFERRED). 기존 subscriber/cache와 검증 함수를 재사용하고 nomotion 요청과 관측 순서를 정리하는 것이 최소 개선이다.

B-01(S1/M3)에 통합: notebook common 검사도 gmapping/map_server/move_base/multifloor/stair를 mode와 관계없이 요구한다. 녹화만 하는 경우의 최소 의존성과 다르다. recording의 전역 rosbag process 차단 및 10GiB 일괄 임계는 별도로 ledger에 넣었다.

OBSERVED historical evidence: `logs/20260916_233206/system.log`는 max_vel_x=0.5, global/local inflation_radius=0.55를 기록하고 bag의 4회 oscillation abort 시각과 일치한다. 현재 YAML inflation 0.33과 구분한다. `logs/20260917_225917/system.log`도 inflation 0.55, max_vel_x=0.3을 기록한다. 두 로그의 RViz helper는 `/rviz_navigation` unknown-node 오류 뒤 navigation을 계속 시작했다. 현재 명명 불일치 B-04를 뒷받침하지만 모든 경우 hang했다고 주장하지 않는다.

두 robot_tunnel.log는 Broken pipe를 보존한다. 로그에는 시각이 없어 supervisor FAULT와 인과 순서를 확정하지 않는다. 과거 command_bridge.log의 close 중 timeout은 이미 삭제된 legacy source의 stack trace이므로 현재 transport 동작과 혼동하지 않는다.

12개 replay result.json과 대응 dashboard 12장을 전부 검토했다. 11개 FAULTED, 1개 INCOMPLETE이며 gap/step/reverse progress가 주된 이유다. 이는 historical replay 결과이며 당시 정확한 profile revision/rate/start 및 실물 안전성은 별도다. `.omo/.../stair-profile-analysis-2026-09-11.md`의 5F/RF UP 후보 거리4.771/3.619가 현재 다른 층 profile에도 동일하게 들어간 사실은 OBSERVED다. 당시 문서의 DOWN flight split은 unresolved이고 현재 DOWN은 역순 음수 거리다. 승인이 없었다고 단정하지 않으며, 층별 실측·승인 근거가 남은 evidence에 있는지 계속 검토한다(C-04 S4/M2).
