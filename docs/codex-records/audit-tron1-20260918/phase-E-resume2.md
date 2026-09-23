# Phase E 두 번째 재개 — AUDIT_INCOMPLETE

repository 기준 경로는 coverage.json과 같다. 시작 시 Git status/plain diff/staged diff 및 baseline hash 1,735개는 이전과 같았다(resume-2-baseline.json). E 전체와 F는 아직 끝나지 않았다.

## OBSERVED 검증과 범위

선택한 추가 pure unit 60개 모두 통과했다(selected-tests-resume2.json). 이전 16개와 합계 76개 실행, 75 pass/1 fail이다. source package entrypoint의 ROS import를 namespace로 우회했고 socket connect/bind, 실제 subprocess 실행을 금지했다. 모든 움직임·transport·서비스는 메모리 fake였다. test_scan_recorder의 파일은 전용 /home/m3tron/.local/state/codex-desktop/tmp/tron1-audit-resume2-* 내부에 생성한 후 제거했다. 개별 file 이벤트와 cleanup은 resume2-unit-run-cleanup.json에 기록됐다.

첫 로드 시도는 uuid의 표준 platform 조회 subprocess를 guard가 차단하여 테스트 0개로 중단했다. 이후 표준 module을 먼저 로드하고 guard를 유지해 60개를 실행했다. 추가 probe의 import 오타는 test 성공 뒤 발생했으며 probe만 재실행했다. 60개 suite를 재실행하거나 통과 수에 중복 계산하지 않았다. probe와 suite는 독립적으로 구분했다.

## C-03 보강 — 준비 중 취소 후에도 새 child 전송

Safety S3 / Mobility M2. OBSERVED 순수 source-body 반례, 실제 ROS action interleaving은 UNVERIFIED.

logic-probe-resume2.py는 ros_segments.py의 class body를 변경 없이 추출해 fake dependency로 실행한다. 공개 execute를 통해 floor wait_for_server 중 또는 stair entry wait 중 parent cancel→cancel_active를 호출했다. 두 경우 모두 존재하지 않는 child에 cancel을 호출한 뒤 새 goal을 전송했고, context cancellation predicate 호출은 0회였다. 기존 C-03의 추론을 이 범위에서 재현했다. actionlib 서버 실제 실행/명령은 발생하지 않았다.

기존 test_navigation_executor.py는 활성 goal cancel 및 retry readiness 경합을 다루지만 ros_segments 준비 대기의 이 간극은 커버하지 않는다. 이 구별 없이 'cancel 테스트가 있으니 안전하다' 또는 '모든 cancel이 실패한다'고 판단하면 안 된다.

## C-02 보강 — timeout이 외부 작업 취소를 의미하지 않음

Safety S2 / Mobility M3. OBSERVED BoundedServiceCaller 논리, 실제 map service 지연 효과 INFERRED.

ros_services.py:33-45의 worker는 timeout 후 중단되지 않는다. fake peer를 지연시키면 call은 response timeout을 반환하고 이후 peer effect가 완료된다(logic-probe-resume2.json). probe worker는 종료/join 확인했다. ros_node.py:227-234의 change_map 호출에 이 경계를 적용하므로 실제 server 작업은 FAULT 뒤 늦게 완료될 수 있다. runtime의 epoch가 stale callback을 무효화하는 것과 외부 map 변경을 취소하는 것은 다르다. 기존 floor recovery finding에 통합하며, 새 epoch를 여는 복구는 외부 map 결과 재관측까지 포함해야 한다.

## C-04 보강 — checkpoint 이름이 정지 증거를 대체하고 FAULT보다 우선함

Safety S4 / Mobility M2. OBSERVED 순수 supervisor 반례, 실제 기체 안정성 UNVERIFIED.

supervisor.py:171-179는 report를 얻은 뒤 safe-checkpoint cancel을 report.faulted/report.complete보다 먼저 처리한다. _finish:218-225는 stair_mode=false 후 NAV로 복귀한다. LANDING report가 complete=false인 경우 및 추가로 faulted=true인 경우 각각 cancel을 주입하면 둘 다 cancelled=true, NAV, stair mode disabled였다(selected-tests-resume2.json). 정지나 landing 확인을 통과하지 않아도 이름이 LANDING이라는 이유로 같은 복귀 경로를 탄다.

이는 기존 C-04의 evidence와 ownership 복귀를 연결하는 같은 root다. 단순히 cancel을 늦추는 조치만 제안하지 않는다. 위험 정지는 즉시 처리하되, 정지 확인·불확실한 자세에서의 mode 해제·NAV 재허용은 구분해야 한다. 기존 state/evidence 안에서 report fault의 우선순위와 복귀 조건을 정리할 수 있다. Required test: 각 checkpoint에서 불완전/FAULT report+cancel 동시 도착, 계단 진입 전 안전한 cancel, 정지 확인 후 정상 NAV 복귀.

## C-10 — floor action에 선언된 command ownership fence가 연결되지 않음

Safety S3 / Mobility M2. OBSERVED 계약/소스, INFERRED concurrent caller 위험.

FloorTransition.action:3에는 stair_ownership_epoch가 있지만 mission ros_segments.py:199-200은 이를 채우지 않고, multifloor ros_node.py:189는 transition_id/target_floor만 전달한다. server의 admission :194-208은 현재 READY, 설정된 endpoint와 target만 검사한다. supervisor state 구독이나 NAV child 정지 확인은 이 경로에 없다. map 변경(:225-231)은 localization의 stationary 검사보다 먼저다.

의도된 단일 mission 순서에서는 이전 NAV를 취소하고 stair 성공 뒤 floor action을 실행한다. 그러나 공개 child action을 별도로 호출하는 ROS peer가 있으면 이 순서가 server에서 강제되지 않는다. 실제 unauthorized peer/네트워크 침입을 관찰했다는 뜻은 아니다. 단순 전역 readiness를 추가하기보다 현재 소유자의 handoff/epoch를 기존 action boundary에서 검증하고 지도 변경 동안 NAV 명령을 허용하지 않는 최소 계약이 필요하다. Required test: NAV 활성 중 direct floor goal, stale/reused epoch, 정상 stair→floor handoff. 외부 ROS 접근 범위·인증은 남은 security 검토 대상이다.

## E-04 — 일부 replay 도구가 기존 운용 master와 섞일 수 있음

Safety S4 / Mobility M3. OBSERVED shell 구성, INFERRED 실제 graph 오염/동작 위험. replay는 실행하지 않았다.

replay_bag_rviz.sh:31 부근의 기본 master는 localhost:11311이고, 기존 master가 있으면 그대로 재사용한다. use_sim_time을 true로 바꾸고 rosbag 전체 topic을 재발행한다. run.sh가 같은 workstation의 11311 master를 소유하는 환경에서 loopback 주소는 별도의 master를 뜻하지 않는다. bag에 goal/command가 있으면 같은 graph로 유입될 수 있고 TF/clock/sensor만 있어도 운용 판단을 오염시킬 수 있다. 현재 bag에 그런 command가 있다는 주장은 metadata 확인 전 보류한다.

마지막 exec rosbag은 shell cleanup trap도 대체한다. 무해한 bash 반례에서 EXIT trap 설정 후 exec printf를 실행하자 child 출력만 나타나고 cleanup trap 출력은 없었다. 실제 background process를 생성하지 않았다. 이 경로에서는 시작한 viewer/master의 정리와 use_sim_time 복원도 보장되지 않는다.

다른 wrapper는 구별한다: raw replay는 기본11319와 기존 master 재사용 opt-in, stair replay는 기본11320과 기존 master 거부·fake transport를 둔다. joy preview는 /replay prefix로 topic을 제한한다. 따라서 모든 replay가 동일하게 위험하다고 묶지 않는다. 최소 개선은 기존 graph 재사용 거부, 전용 local master 소유 확인, topic allowlist, exec 없이 owned child 종료를 기다리는 기존 wrapper 정리다. 정상 NAV에 gate를 추가할 필요는 없다. Required test: master 이미 존재할 때 불변 거부, command-containing bag의 topic 차단, 종료 후 owned process/parameter 정리.

## D 추가 upstream 확인

공식 static_layer.cpp:74,150-164는 기본 track_unknown_space=true로 unknown을 보존하며, NavfnROS 기본 allow_unknown=true와 결합하면 global planner의 unknown은 무조건 차단이 아니다. current config에 명시 override는 없다. 실제 installed binary/parameter는 UNVERIFIED. 안전한 통행 영역의 측정 없이 이 설정을 전역 false로 바꾸면 정상 경로도 막힐 수 있으므로 계단·낭떠러지 구역의 지도/관측 coverage를 먼저 분리해야 한다.

공식 trajectory_planner.cpp:849-900에는 backup_vel fallback과 footprint collision cost(-1)를 양수로 바꾸는 escape 경로가 있다. 따라서 recovery_behavior_enabled=false 및 min_vel_x=0만으로 후진이 금지되거나 모든 backup trajectory가 같은 collision rejection을 받는다고 볼 수 없다. source URL/hash는 upstream/manifest-resume2.json. 실제 후방 공간·실행 여부는 UNVERIFIED이며, 현 단계에서는 D의 안전 가정 확인 대상으로 기록한다.

## 남은 조사

E2E 성공/실물 안전을 증명하지 못하는 mock을 명확히 분류했다. 남은 doc/launch/test/evidence/log/bag metadata를 계속 읽어 recovery·source/build drift·보안·fixture parity를 확인해야 한다. startup map/floor/pose/anchor의 원자성, recorder topic dependency, unknown-space·escape 경로의 물리 영향은 최종 시나리오 통합 전 보완한다. F 15개 모두 아직 PENDING이다.
