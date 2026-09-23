# TRON1 최소 변경 구현 계획

2026-09-21 · ACTIVE_PLAN / 구현 전 보고 · 실장비 검증 전

Understood as: 전체 요구 기능은 유지하면서 현재 노드·명령 경로·상태머신을 최대한 재사용하고, 작동에 필요한 결함 수정과 좁은 기능 확장만 계획한다. 이번 요청은 계획·보고이며 운용 코드 적용이나 장비 실행 요청이 아니다.

이 문서가 현재 구현 범위의 기준이다. [9월 20일 상세안](IMPLEMENTATION-SPEC-20260920.md)은 확장 후보와 참고 계약으로 남기며 전체를 구현하지 않는다. MotionCoordinator, 범용 lease API, 새 위치 추정기, raw/control odom 분리, TF 개편, SQLite, 범용 장치 엔진은 기본 범위에서 제외한다.

## 1. 완료해야 할 사용자 결과와 변경 원칙

초기 UI 선택은 3F다. 사용자가 실제 현재 층을 적용하고, 지도에서 실제 도달 가능한 어느 위치든 목적지로 지정한다. 3F→4F는 실제 현재 위치→계단 진입점 NAV→기존 supervisor의 계단·회전·출구 판정→4F 지도·위치 확인→알림·지정점 유지다. 5F→RF는 경로상의 문 요청과 여러 촬영점 이동·촬영·자동 메일을 포함하며 마지막 지점에서 유지한다. 복귀 승인이 있어야 원래 실제 출발점으로 돌아간다. RF는 UI에서 ‘6층·옥상’으로 표시한다.

- mission_manager는 기존의 단일 순차 실행기, multifloor_manager는 지도·층 적용자, stair_supervisor는 유일한 robot transport로 유지한다. 새 제어 노드는 기본안에 없다.
- 기존 move_base·AMCL·costmap·센서 topic/TF·방향별 stair graph/profile을 우선 재사용한다. 기존 정상 기능과 기록 기능을 일괄 재작성하지 않는다.
- 필요한 새 입력도 같은 접수 lock·취소·실행 경로를 공유한다. ROS 노드를 늘리지 않는다는 이유로 별도의 실행 상태나 제어 thread를 무분별하게 추가하지 않는다.
- 기능별 작은 변경 묶음으로 검증하고 되돌릴 수 있게 한다. 요구 동작을 못 하는 것이 확인된 때에만 확장 후보를 다시 보고한다.
- ‘작은 diff’ 자체를 성공 기준으로 삼지 않는다. 기존 defect를 우회하거나 정확하지 않은 위치·도착·정지를 정상이라고 표시하는 축소는 하지 않는다.

## 2. 현재 확인한 근거

대상은 `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`, HEAD `eb24e08c9220f454080bd672bd8ce3ec0aeb7159`다. 계획 전후 Git status와 staged/unstaged diff는 깨끗하며 source 변경 없음이다. 이번 확인은 필요한 경계의 읽기 전용 재확인으로, 새 전체 inventory 감사나 runtime 시험은 아니다. Mini PC의 실제 배포 상태는 구현 착수 때 따로 대조한다.

OBSERVED는 코드·설치 source·기존 감사에서 직접 확인, INFERRED는 그 근거를 연결한 해석, UNVERIFIED는 실장비·외부 계약이 필요하다는 뜻이다. 아래 변경은 모두 제안이며 현재 구현된 기능이라는 뜻이 아니다.

| OBSERVED 근거 | 작은 변경으로 해결할 지점 | 관련 영향 |
|---|---|---|
| [Mission 접수](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/mission_action_server.py:91)는 named ID와 한 active goal을 관리 | 기존 접수부를 공유하고 pose 입력만 추가 | C-01 S2/M3, C-03 S3/M2 |
| [planner](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/route_planner.py:141)는 named graph, 반환은 HOME | 실제 pose를 시작으로 접속 NAV 추가, snapshot 복귀 | C-11 S2/M2; 새 pose 기능은 사용자 요구 |
| [floor 요청](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/src/multifloor_manager/ros_node.py:194)은 현재 READY와 계단 연결을 요구 | 현재 층 초기화 입력을 분리하되 map/pose 적용 코드 재사용 | C-02 S2/M3, C-10 S3/M2 |
| [계단 정렬·정지](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/stair_evidence.py:191)는 새 표본·인접 증분에 의존 | 실제 방향 오차와 시간 구간의 이동/회전 범위로 교정 | C-04 S4/M2 |
| [NAV 설정](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/multifloor_manager/config/nav/base_local_planner_params.yaml:14)은 목표 오차 0.25m | 요구 HOLD 거리와 실제 정밀도를 먼저 비교 | H-01 S3/M2: 기존 설정으로 정밀 HOLD가 되는지는 UNVERIFIED |
| [센서 보고서](../sensor-recording-analysis-20260918/REPORT.md)의 관측과 source상 scan deskew 의존성 | raw odom만으로 밀림을 단정하지 않고 외부 실측 대조 | R1 S3/M2의 해석 INFERRED; 실제 밀림 원인 UNVERIFIED |

C/B 등급은 [기존 감사](../audit-tron1-20260918/FINAL-AUDIT.md), H-01/L-01은 이전 계획 검토의 영향 등급이다. Safety는 S0 확인된 영향 없음, S1 낮은 간접 영향, S2 제한 조건의 안전 여유 부족, S3 fault 중 위험 명령, S4 충돌·추락·명령 소유 상실 가능성이다. Mobility는 M0 확인된 영향 없음, M1 낮은 간접 영향, M2 특정 경로·기능 차단, M3 복구 고착·전체 재시작 요구, M4 해당 핵심 임무 불가능이다. 최소 수정으로 이 경계들을 재사용할 수 있다는 판단은 INFERRED이며 실제 호환성은 시험 전 UNVERIFIED다. 이전 검토 이력은 [결정 기록](decisions.json)에 보존한다.

## 3. 네 번의 구현 묶음

| 순서 | 사용자가 얻는 결과 | 수정하는 경계 | 완료 조건 |
|---|---|---|---|
| M1 현재 층·평지 이동 | 층 선택, 지도 클릭 이동, 실제 출발점 저장 | mission 접수/planner/NAV, floor 초기화, 필요한 startup gate | P01/P02/P11/P12/P14/P15 관련 항목. 등록 지점 강제 없이 왕복 NAV, 잘못된 층·pose를 허위 확정하지 않음 |
| M2 기존 계단 연결 | 3F 임의 위치→4F 도착·알림·다음 명령 | 기존 stair 증거·cancel, floor handoff/실패 복구 | P03/P12/P13/P15. 독립 관측으로 진입·착지·회전·출구 확인. active 해제 및 다음 요청 접수 |
| M3 지정점 유지·승인 복귀 | 실제 밀림 보정, 승인 후 출발 좌표 복귀 | 최소 HOLD 실험에서 선정한 한 경로, 기존 복귀 계획·snapshot | P04/P05/P13/P14. 노이즈에 불필요 보정 없음, 실제 이탈 복원, 새 명령 우선, HOME 대체 없음 |
| M4 옥상 작업 | 5F↔RF 문, 다지점 사진·메일·마지막 지점 유지 | 기존 executor에 문/사진 handler, 기존 앱의 메일 처리 | P06~P10. 실제 개방·통과·사진 확인, 부분 실패 재시도, 메일 장애 중 복귀 가능 |

M2는 첫 층간 이동의 검증 결과다. 위치 유지가 없는 M2만으로 사용자 전체 요구를 완료했다고 하지 않는다. M3의 관측 실험은 M1/M2 개발과 독립적으로 준비할 수 있다. 5F↔RF와 DOWN은 각각 검증하며 3F→4F 성공을 대체 증거로 쓰지 않는다. 코드 시험은 먼저 격리 환경에서, 실물 이동은 정한 현장 조건에서 수행한다. 이번에는 어느 시험도 새로 실행하지 않았다.

## 4. M1: 기존 실행기에 현재 pose와 좁은 새 입력만 추가

`mission_action_server.py`의 admission/release/cancel을 한곳에서 공유한다. 새 pose용 action(가칭 `PoseMission.action`)도 같은 `_active`와 orchestrator를 쓴다. 새 MissionCoordinator·별도 scheduler는 만들지 않는다. 기존 `Mission.action`의 named goal/result/cancel과 legacy HOME 자동 복귀는 유지하며 새 웹 작업의 승인 복귀와 구별한다.

최소 요청은 request_id, 작업 종류, floor/map revision이 붙은 목표 pose 또는 촬영점 목록, optional return_snapshot_id다. 중복 ID+같은 내용은 기존 결과를 조회하고 ID+다른 내용은 충돌로 반환한다. 기존 action에 필드를 덧붙여 호환이라고 주장하지 않는다. ROS 메시지 필드가 MD5 계산에 들어가는 사실은 [설치된 genmsg](/opt/ros/noetic/lib/python3/dist-packages/genmsg/gentools.py:58)에서 확인했다.

`RosStateMonitor`의 기존 floor/AMCL/odom 입력으로 현재 위치 문맥을 만든다. 시작에 `_anchor` 이름만 믿지 않는다. `BuildingPlanner.plan_from_pose()`를 좁게 추가한다. 같은 층은 현재 pose→목표를 직접 NAV하고, 다른 층은 기존 방향별 계단 연결을 선택한 뒤 시작→entry NAV와 최종 출구→목표 NAV를 붙인다. 연결 불가한 entry이면 기존의 다른 유효 entry 후보를 확인하며 무한 재시도하지 않는다. 거리가 가깝다는 이유로 벽 건너 named 지점에 이미 도착했다고 가정하지 않는다.

임시 시작/목표는 요청별 RouteContext에 둔다. `LogicalAnchor`의 성공 후 commit 규칙을 유지하고 executor가 context의 target pose를 우선 해석하게 한다. 공유 `_locations`를 계속 수정하거나 모든 클릭을 YAML에 저장하지 않는다. 기존 기록 임무는 원래 경로를 유지하며 pose 확장을 강제로 끌어들이지 않는다.

`NavigationExecutor._await_terminal()`과 segment child 대기는 취소·deadline·결과 대조를 추가한다. timeout 뒤 실제 child가 살아 있는지 모른 채 새 goal을 보내지 않는다. 그런 경우에는 ‘종료 확인 필요’를 표시하고 취소/조회로 복구한다. 준비 대기 이후 발행 직전에 다시 취소를 확인하며 성공·실패 terminal과 active 해제를 일관되게 처리한다.

orchestrator의 RETRY도 해당 단계의 횟수/전체 시간 예산을 공유하게 하여 child마다 timer를 초기화하는 무한 반복을 막는다. 예산은 실제 정상 소요 시간과 복구 가능성으로 정하며, timeout 하나를 모든 단계에 복사하지 않는다.

현재 층 적용은 가칭 `InitializeFloor.action`을 기존 floor 노드에 추가하고 기존 map loader·initialpose·관측·transaction lock을 재사용한다. mission의 같은 접수 경계에서 이동 정리→층 적용을 실행한다. 계단 종료 요청을 가짜 stair ID로 호출하지 않는다. 초기 기본 3F와 실제 pose 확정은 구분하고 유효한 같은 층 재선택은 불필요한 reset을 피한다. UI 지도 변환은 origin/resolution/yaw/Y축 방향을 함께 시험한다.

적용 도중 실패하면 마지막 확정 층, 실제 로드된 지도, 위치 미확정을 구분한다. 늦은 map 호출이 살아 있으면 후속 변경을 겹치지 않는다. 해당 지도에서 pose를 재확인하거나 이전 지도와 pose를 함께 다시 확인해 복구한다. 새 전역 상태 엔진을 만드는 대신 기존 runtime/detail와 작은 transaction 문맥을 확장한다.

## 5. M2: 기존 계단 경로에서 실제 반례만 수정

먼저 [기존 감사](../audit-tron1-20260918/FINAL-AUDIT.md)의 B-01/B-04, C-01~05/C-08~11 중 선택 경로에 해당하는 반례를 기존 test에 고정한다. 카메라·다른 층 profile·RViz의 필요 여부는 startup와 실행 gate를 각각 대조한다. 문서와 다른 설정을 무조건 켜거나 readiness 전체를 제거하지 않는다.

`stair_evidence.py`의 ALIGN은 실제 방향 오차를 확인하고, LANDING/EXIT는 관측이 이어지는 시간 구간의 위치·yaw 범위와 속도로 정지를 평가한다. 샘플 사이 공백은 정지 시간에 포함하지 않는다. `supervisor.py`의 취소는 phase 이름이 아니라 확인된 checkpoint 이후에 소유를 반환하도록 고친다. 정지 관측만으로 지지면·실제 착지를 확인했다고 표시하지 않는다.

checkpoint 계약은 profile별로 진행 위치/방향·시간 구간 정지에 더해, 기존 센서에서 얻을 수 있는 착지/지지 관측의 종류·유효 시각·허용 범위를 지정한다. 현재 어떤 입력이 이를 충족하는지는 UNVERIFIED다. 현장 영상은 시험의 정답이지 자동 판정 입력으로 몰래 대체하지 않는다. 기존 입력으로 구별이 안 되면 M2의 자율 checkpoint 인수를 미완료로 남기고 필요한 관측 추가만 보고한다. 정지 판정 코드를 고쳤다는 이유로 실계단 운용을 승인하지 않는다.

`ros_segments.py`에서 기존 `stair_ownership_epoch`를 전달하고 floor 노드에서 검증한다. epoch 숫자 비교 하나로 동시 실행을 막았다고 하지 않는다. 지도 변경 동안 기존 supervisor의 송신 경로에 제한된 대기 예약을 걸고 NAV 명령을 선택하지 않으며, 실패·취소의 늦은 map 부작용까지 정리한 뒤 해제한다. 직접 들어온 floor action도 이 예약을 확인해야 한다. 계단 중 새 지도 적용은 거부하고 이유를 반환한다.

이 예약은 기존 owner와 lock/epoch를 확장하는 좁은 변경이다. 노드 사이 요청이 필요하면 지도 대기·해제용 작은 서비스 하나를 추가하고, 이후 전용 HOLD가 필요할 때 같은 idle activity 경계를 사용한다. 범용 분산 lease 프로토콜이나 외부 제어 owner를 새로 만들지 않는다. 서비스 재전송은 request ID로 같은 예약을 조회하며 timeout이 자동 해제를 뜻하지 않는다. 재시작 시 과거 예약·속도를 자동 복원하지 않고 현재 상태를 대조한다.

STAIR firmware의 자세 보정 능력, 실제 3F↔4F/5F↔RF profile 정확도는 UNVERIFIED다. 실제 진행 오차가 거리 설정 문제인지 관측/제어 부족인지 구별하고 부족한 부분만 추가 보고한다. FAULT 복구가 필요한 경로는 닫힌 transport 객체 재사용 대신 기존 owner에서 새 세션을 만드는 명시적 복구로 한정하고, 오래된 desired command를 폐기한다.

## 6. AMCL·HOLD: 작은 실험으로 추가 개발 여부 결정

AMCL은 기본 pose가 임의 배치 후에도 강제되는지, 층/map/scan/TF·시각이 맞는지부터 교정한다. 원인 구분 없이 particle 수·covariance·주행 허용 오차를 일괄 조정하지 않는다. 유효한 연속 추적은 유지하고, 초기 획득·수동 이동·설명되지 않는 위치 jump 때만 재획득한다.

기존 태그를 활용하려면 map상의 실제 태그 pose와 camera extrinsic이 필요하다. 현재 ID 인식만으로 로봇 pose가 계산됐다고 취급하지 않는다. 기존 태그의 측량·pose 처리 또는 지도상 수동 위치/방향 지정 중 사용자 운용에 맞는 최소 수단을 선택한다. 수동 보조 허용·태그 설치 범위는 미답변이므로 아직 확정하지 않는다. 두 모서리를 구분할 관측이 없으면 자동 초기화를 완료했다고 표시하지 않는다.

HOLD는 다음 두 실험을 먼저 한다. 첫째 실제 정지·바퀴 이동·미끄럼을 외부 영상/실측과 기존 odom/AMCL/scan에 함께 기록해 구별 가능한지 본다. 둘째 지정 거리 이탈에 기존 NAV가 저속으로 원점까지 복원하는지, 특히 측면 이탈·좁은 공간·기체 deadband를 본다. 현 `/scan`도 raw odom으로 deskew하므로 완전히 독립된 정답으로 세지 않는다. 노이즈·AMCL jump·장시간 누적 변화도 반례로 포함한다.

| 실험 결과 | 선택할 최소 구현 | 확장 조건 |
|---|---|---|
| 관측이 실제 이탈을 구분하고 기존 NAV가 요구 허용 범위 충족 | mission 내부 저우선순위 HOLD 감시가 기존 NavigationExecutor를 직렬 재사용 | 사용자의 거리보다 실제 도착 오차가 크거나 보정이 과도하면 이 안 탈락 |
| 관측은 충분하지만 NAV의 작은 보정이 요구를 못 맞춤 | 기존 supervisor 안의 제한된 HoldController를 별도 변경 보고 후 구현 | 저속 기체 응답·충돌/정지 궤적·인계 검증 필요 |
| 현재 관측으로 실제 밀림을 구별하지 못함 | 시간/단위/초기화 결함부터 수정하고 기존 외부 기준 활용 | 이것으로도 부족할 때만 추가 국소 추정기·센서 변경 보고 |

기본안은 첫 행이며 충족이 증명된 경우에만 적용한다. 현재 25cm 설정을 모든 NAV에서 강제로 줄이거나 실행 중 rosparam 변경만으로 작은 HOLD 오차가 적용된다고 가정하지 않는다. 요구 거리 미답변 상태에서 어떤 안이 통과할지 확정하지 않는다. 기본안은 유효한 전역 위치와 기존 planner를 쓰므로 독립 local HOLD라고 부르지 않는다.

어느 안이든 목표는 지정점으로 고정한다. r_on 초과에서 보정, 더 작은 r_off 안에서 종료하며 관측 노이즈에 반응하지 않게 한다. r_on은 최대 허용 이탈과 다르므로 측정 오차·응답 지연을 반영한다. AMCL jump·odom reset으로 목표를 자동 이동시키지 않고, 좌표 의미가 달라지면 재확인을 요구한다. 전용 제어가 필요하면 footprint·장애물·정지 궤적과 관측 시각 검사를 포함한다. 계단 중·확인되지 않은 계단 끝에서 평지 HOLD를 실행하지 않는다.

새 유효 임무는 기존 admission lock 안에서 HOLD 보정을 취소하고 child 종료/명령 폐기를 확인한 뒤 시작한다. HOLD 감시는 사용자 mission의 active handle을 점유하지 않는다. 조회·잘못된 요청은 HOLD를 끄지 않는다. 명시적 정지는 HOLD도 해제하며, zero 송신을 물리적 위치 유지 성공으로 표시하지 않는다. HOLD 미지원은 명확히 표시하되 관련 없는 일반 NAV를 전부 차단하지 않는다.

현재 Twist에는 원래 생성 goal/stamp가 없어 수신 시 epoch만 붙여 임의로 지연된 과거 명령까지 구별할 수 없다. 입력 폐기·기존 child 종료는 재사용하되 엄밀한 발생원 구분은 미보장으로 남긴다. 인수 환경에서 필요한 지연 격리를 충족하지 못하면 생산 지점의 식별자 추가를 별도 보고하며, 적은 수정이라는 이유로 그 시험을 PASS로 바꾸지 않는다.

## 7. M3·M4: 작은 기록과 기존 순차 executor 확장

출발 직전에 snapshot(원 job ID, 실제 floor/map revision, x/y/yaw, 시각·위치 출처)을 저장한다. 이후 층 전환·사진 실패로 덮어쓰지 않는다. 복귀 승인은 snapshot을 목적지로 하는 새 pose 임무이며 현재 위치에서 계획한다. 장애물이나 지도 revision 변화로 의미가 달라졌으면 이유를 표시하고 HOME으로 대체하지 않는다. generation 증가만으로 정상 복귀를 막지 않는다.

별도 DB를 도입하지 않고 기존 artifact 저장 루트 아래 요청별 `mission.json`, 사진과 `manifest.json`, `mail.json`을 둔다. 위치 복귀 기록을 bag 성공 여부에 종속시키지 않는다. mission 단일 writer가 같은 디렉터리 임시 파일→완료 확인→원자적 교체로 갱신하고 접수 ACK/첫 이동 전에 필요한 기록을 확정한다. 전원 손실까지 보존할 범위는 파일/디렉터리 동기화와 재시작 시험으로 검증한다. 중복 요청 내용 hash와 결과를 저장한다. 파일 수·조회 병목이 확인되기 전 SQLite로 확장하지 않는다.

재시작 때 기록과 현재 ROS 실행을 대조하고 진행 중 작업을 자동 재실행하지 않는다. 일부 사진과 snapshot은 유지한다. 임시 사진 정리와 실패 기록도 같은 job에 남긴다. 보존 정책 확정 전 원본을 자동 삭제하지 않으며 공간 부족은 해당 기록/촬영 실패로 알린다. 출발점 저장이 필요한 새 작업은 저장 실패 상태에서 몰래 출발하지 않는다.

문은 executor의 좁은 `DOOR_OPEN` handler와 장치 설정(device ID→endpoint/인증 참조/timeout)을 추가한다. route에는 대기점·통과점·방향별 필요 여부만 둔다. 대기점→요청→실제 열림 확인→통과 순서이며 HTTP 성공을 물리 개방으로 취급하지 않는다. 응답 유실은 상태 확인 또는 장치의 중복 방지 계약으로 처리한다. 계단 내부에 문이 있다면 확인된 checkpoint hook이 필요한지 위치 답변 후 별도 보고한다. 임의 스크립트 실행기·장치별 노드는 만들지 않는다.

사진은 `PHOTO_CAPTURE` handler로 현재 RGB 경로를 재사용한다. NAV→방향/안정 확인→도착 이후 새 frame 저장→시각/위치/hash manifest 확정 순서다. 기존 INSPECT/SCAN의 rosbag 의미는 바꾸지 않는다. 사진 실패는 해당 지점만 실패로 남기고 active를 해제해 복귀를 받을 수 있게 한다. 재개 때 이미 다른 곳으로 이동했다면 필요한 NAV가 포함된 새 요청임을 표시한다.

사진 실패의 기본 정책은 촬영 임무를 PARTIAL_FAILED로 종료하고 남은 지점을 자동 진행하지 않는 것으로 제안한다. 유효한 HOLD가 있으면 현재 지정점에서 유지하고, 없으면 유지 불가를 표시한다. 성공 사진은 보존하며 실패점 재시도·남은 지점 재개·승인 복귀를 새 요청으로 받는다. 정상 완료 때는 마지막 지점에서 유지한다.

메일은 선택한 기존 앱/backend의 작업 처리 또는 기존 mission의 제한된 비제어 worker 중 실제 재사용 가능한 한곳에 둔다. 네트워크 대기는 motion/접수 lock을 잡지 않는다. manifest와 발송 상태를 읽고 결과만 기록하며 이동을 재실행하지 않는다. 응답 유실은 UNKNOWN_DELIVERY로 남기고 provider 조회/중복 방지 없이는 정확히 한 번 발송을 보장하지 않는다. 메일 장애와 승인 복귀는 분리한다.

worker의 결과는 mission의 단일 writer로 전달해 파일에 반영한다. 외부 backend가 같은 JSON을 동시에 수정하는 구현은 하지 않는다.

## 8. UI·파일·호환성의 변경 경계

UI는 현재 층, 지도 목표 선택, 3F→4F, 옥상 촬영, 취소, 승인 복귀와 실제 진행 상태에 집중한다. 기존 웹앱/ROS bridge가 있으면 재사용한다. 로컬 Qt 제어 도구를 웹앱이라고 간주하지 않는다. 실제 웹앱 경로는 미답변이므로 새 ROS client/node 여부를 아직 정하지 않았다. 필요하면 node·연결·종료·대안을 보고한다. 새 제어 노드 0개라는 기본 방향과 웹 연결 node도 0개로 보장한다는 주장은 다르다.

| 경계 | 기존 파일 중심의 수정 대상 | 추가 최소 요소 |
|---|---|---|
| 접수·좌표·복귀 | mission_action_server, mission_types, mission_orchestrator, route_planner, ros_segments, ros_state, navigation_executor, ros_runtime | pose 입력 타입/action, 요청별 문맥·파일 기록 helper |
| 현재 층·전이 | multifloor ros_node/ros_runtime/ros_services, 필요 callback·readiness | 초기화 action, 기존 transaction의 목적별 입력·복구 |
| 계단·지도 중 명령 | stair supervisor/ros_node/stair_evidence, 기존 admission | 제한된 idle 예약 서비스; 전용 HOLD 모듈은 실험 결과에 따라 |
| 촬영·문·메일 | 기존 segment_types/dispatch/FSM 표시, 실제 웹앱 연결부 | 좁은 photo/door handler·파일 상태; 범용 작업 엔진 없음 |
| 센서 | 우선 진단과 기존 설정 대조 | 결함이 확인된 시각/단위 처리만. 새 odom/TF 체계는 제외 |
| 패키징·시험 | 관련 CMakeLists/package.xml/launch와 기존 test | 필요한 타입 생성·설치, 실제 변경을 검증하는 회귀 |

파일 개수를 억지로 줄여 취소·설치·시험 경계를 빠뜨리지 않는다. 각 묶음의 diff에 사용자 기능 또는 확인한 defect와의 연결을 붙이고, 무관한 이름 변경·파일 이동·추상화는 제외한다. 필요한 새 action도 실행기는 하나다. 메시지 정의·기존 result/cancel·단일 transport/TF·source/devel/install 출처를 검증하고, 기능별 설정은 시작 전에 적용하며 동작 중 전환을 가정하지 않는다.

## 9. 최소 제약 ledger와 디버깅

기존 [67개 제약 ledger](../audit-tron1-20260918/constraint-ledger-final.md)는 보존한다. 다음은 이번 계획의 변경 대상/신규 조건이다. KEEP은 새로 부여하지 않는다. NARROW는 필요한 단계로 축소, TUNE은 근거에 따라 수치·관측 조정, UNVERIFIED는 실물/외부 확인 전 미확정이다. KEEP 변경이 필요하면 hazard·현실 근거·최소 범위·더 좁은 대안의 불충분성·오탐 복구를 모두 제시한다.

| 조건과 막을 문제 | 최소 적용·복구 | 판정 / 영향 |
|---|---|---|
| 입력 없는 기능 실행 | camera/tag/메일/타 층 profile/RViz는 사용하는 기능만; 정상 flat NAV 차단 0건을 시험 | NARROW · B-01 S1/M3 |
| 잘못된 층·좌표 | 초기 적용/목표 접수/map 변경에 일치 확인, 해당 pose만 재획득 | NARROW · C-11 S2/M2 |
| 대칭 위치 오인 | 초기 획득/불일치 때 외부 위치 근거, 유효한 추적 중 태그 필수 아님 | NARROW/UNVERIFIED · L-01 S3/M2 |
| 이동·map·HOLD 경쟁 | 기존 owner에서 필요한 인계만 직렬화, 종료/취소/늦은 map 정리 뒤 해제 | NARROW · C-03/C-10 각각 S3/M2 |
| child/통신 무응답 | 해당 실행만 timeout·대조·명시적 복구, 무조건 반복/영구 BUSY 제거 | TUNE · C-01 S2/M3, B-02 S1/M3 |
| 잘못된 정지·착지 | 해당 계단의 시간 구간 관측, 확인된 checkpoint 재평가 | TUNE/UNVERIFIED · C-04 S4/M2 |
| 노이즈 추종·잘못된 HOLD | HOLD만 deadband/관측·공간 검사, 새 명령 인계·목표 재확인 | TUNE/UNVERIFIED · H-01 S3/M2 |
| 닫힌 문 통과 | 해당 통과만 개방 확인·상태 조회/재요청/대체 경로 | UNVERIFIED · 예상 S3/M2 |
| 사진·메일 실패가 이동 차단 | 부분 결과 보존·해당 작업만 재시도·복귀 접수 해제 | NARROW · 예상 S1/M2 |
| snapshot 손실·임의 복귀 | 해당 요청의 저장/승인만 확인, 저장 복구 후 접수·다른 이동 계약과 구분 | NARROW · 예상 S2/M2; 승인은 사용자 요구 |

디버깅은 새 관제 시스템 대신 기존 로그/feedback에 request→mission→segment→child ID, floor/map generation/ownership epoch, 목표/실제 pose, 관측 시각·수신 시각, 차단 reason/측정값/기준을 연결한다. 최종 명령 선택·송신·기체 피드백을 구분한다. 사진/문/메일 ID도 같은 job으로 묶는다. UI에는 현재 단계·막힌 이유·가능한 다음 동작을 표시한다. stale timeout은 실측 지연 없이 일괄 늘리지 않는다.

## 10. 인수 기준: 기존 P01~P15 유지

아래는 앞으로의 시험이며 전부 아직 UNVERIFIED다. historical 감사의 15개 판정을 바꾸지 않는다. 시험 범위·거리·시간·허용 오차는 실행 전에 정하고 결과에 맞춰 사후 완화하지 않는다.

| ID | 시나리오 | 통과 기준 |
|---|---|---|
| P01 | 3F 기본/층 선택·초기 위치 | 실제 floor/map/pose 일치, 잘못된 저장 pose 미승인, 미리보기는 운용 map 불변 |
| P02 | 각 층 임의 출발·목표 | 독립 참 위치 대조, HOME/등록 anchor 우회 강제 없음, 유효한 추적 중 tag 비가시 허용 |
| P03 | 3F 임의 위치→4F | 실제 entry/계단참/회전/출구/층 확인·알림·다음 명령 |
| P04 | 지정점 유지 | 노이즈 보정 없음, 실제·측면 밀림 복원, 고정 목표, map jump 추격 없음, 새 명령 우선 |
| P05 | 승인 복귀 | 승인 전 출발 없음, HOME 대신 실제 snapshot, DOWN/층 전이 확인 |
| P06 | 5F↔RF 문 | 위치 trigger·실제 개방·통과·방향별 동작 |
| P07 | 문 거부·유실·닫힘·취소 | 허위 개방·맹목적 중복 요청 없음, 필요한 구간만 복구 |
| P08 | RF 다지점 촬영 | 실제 장소/방향/새 frame과 manifest 일치, 마지막 지점 유지 |
| P09 | 일부 사진 실패 | 성공 사진/위치 보존, active 해제·복귀 가능, 해당 단계만 재개 |
| P10 | 메일 장애·응답 유실 | 사진 보존, 불확실한 전송 표시, 이동 재실행 없음, 승인 복귀 가능 |
| P11 | 무관 기능 장애 | 정의한 정상 조건군에서 무관 camera/tag/RViz/profile/메일 gate의 flat NAV 차단 0건 |
| P12 | 준비·이동·전환 취소 | 이후 새 child 없음, 실제 checkpoint·소유·늦은 부작용 정리 |
| P13 | 관측/통신 결손·reset | stale 명령·허위 유지·영구 BUSY 없음, 확인 가능한 복구 |
| P14 | 중복 클릭·재접속·재시작·경쟁 | 한 번 접수, 기존 기록 조회, owner 하나, 재시작 자동 재출발 없음 |
| P15 | 지도 변경·출발점 장애물·legacy | 잘못된 복귀/허위 성공 없음, 기존 goal/result/cancel·배포 출처 일치 |

검증은 기존 순수 test 확장→격리 ROS 실제 설정 통합→장애 주입→현장 순서다. 합성 phase를 출력했다는 사실로 실제 착지를 채점하지 않는다. 위치·밀림은 외부 측량/영상, 문은 물리 개방 관측, 사진은 실제 장면과 대조한다. HOLD 최대 이탈·복원 시간·불필요 보정 횟수·명령 인계 지연을 기록한다. 허용 수치는 사용자 거리와 기체 측정 전 확정하지 않는다.

Safety verdict와 Mobility verdict는 별도로 기록한다. 목적지 도착만으로 명령 충돌을 통과시키지 않고, 충돌 없이 멈췄어도 다음 명령 불가이면 mobility 실패다. 인수 미통과 기능은 완료 표시하지 않으며 평지 등 독립적으로 검증된 기능의 사용까지 불필요하게 막지 않는다.

## 11. 확장 조건과 필요한 답변

| 보류 항목 | 다시 검토할 구체 조건 |
|---|---|
| 새 위치 추정기·odom/TF 변경 | 기존 시각/초기화/관측 교정 뒤에도 외부 실측상 요구 이탈을 구별하지 못함 |
| 전용 HOLD controller | 기존 NAV로 요구 오차/공간/인계 성능을 충족하지 못함 |
| 발행 지점 command 식별자 | 현행 Twist 경로로 요구 지연/명령 격리 인수 기준을 충족하지 못함 |
| 태그 pose 확장·추가 설치 | 기존 초기화/연속 추적으로 대칭 위치를 구분 못 하고 사용자 운용이 해당 방식을 허용 |
| 계단 내부 문 hook | 문이 기존 mission 단계 사이에 놓을 수 없는 실제 계단 phase에 있음 |
| DB·범용 작업/권한 시스템 | 현재 단일 실행기·파일 기록으로 해결할 구체 병목/경합이 재현됨 |

이미 질문한 항목 중 M1의 UI 연결 전에 필요한 것은 웹앱 경로/bridge, 위치 획득 확정 전에 수동 위치 지정 허용·기존 태그 활용 범위, M3 인수 전에 HOLD 시작 거리·방향 유지 및 가능하면 최대 허용 이탈, M4 통합 전에 문 위치/API/열림 확인·복귀 적용과 촬영점/메일 서비스다. 이 답변 없이도 baseline·공통 접수·현재 pose 문맥·취소 회귀는 준비할 수 있다. 기존 계단 왕복 성공 이력은 현장 시험 범위를 줄이는 자료지만 결과를 대신하지 않는다. 미답변 장치 주소·임계값·기체 기능을 추측하지 않는다.

다음 착수는 M1의 baseline 고정과 기존 test에 현재 위치/취소/무관 gate 반례를 넣는 것이다. 원본 worktree의 기존 변경은 보존하고 필요하면 별도 checkout에서 구현한다. 이번 계획 작업은 원본 recording·repository·ROS·기체·자동문·메일을 변경하거나 실행하지 않았다. 임시 검토 사본은 제거하며, 최종 문서 검증·정리 기록은 decisions.json의 minimum_change_plan_20260921에 남긴다.
