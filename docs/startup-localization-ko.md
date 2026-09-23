# 임의 위치에서 자동 측위와 수동 지정

`run.sh` / `mission_manager/system.launch`는 기본적으로 선택된 층의 지도 전체에서 시작 위치를 찾는다. 초기 층은 기존 설정대로 3F다. `home_3f`는 경로 그래프의 논리적 기준점이며 로봇이 실제 그곳에 있다는 뜻이 아니다. 시작할 때 그 좌표를 AMCL에 주입하지 않는다.

이번 적용 범위는 **3F 지도 안의 임의 위치에서 시작하기**다. 다른 층을 자동으로 판별하거나 층 선택 웹 화면을 추가한 변경은 아니다. 다른 층에서 새로 시작하는 설정은 지도 파일·initial_floor·논리적 initial_location_id가 함께 일치해야 하며 이번 현장 검증 범위에 포함하지 않는다.

## 사용

1. 지도와 센서가 들어오면 현재 Floor Manager가 자동 탐색한다. 로봇을 회전시키거나 이동시켜 위치를 찾는 명령은 보내지 않는다.
2. 자동 탐색 결과가 모호하거나 잘못 보이면 RViz 상단 **2D Pose Estimate**를 누른다. 지도에서 실제 로봇 위치를 누르고 로봇이 바라보는 방향으로 끌어 놓는다. 수동 입력은 진행 중인 자동 탐색보다 우선한다.
3. 위치 확인이 끝나면 기존 `/mission`으로 목적지를 보낸다. 3→4층 목적지는 `stair_4f_from_3f`이며 순서는 현재 위치 → `stair_3f_up_entry` NAV → 계단 → 4F 지도 전환이다. 이 문서의 측위 구현만으로 LiDAR 계단 제어의 현장 설정이 완료된 것은 아니다.

상태는 `rostopic echo /multifloor/localization_status`로 보고, `rostopic echo -n 1 /multifloor/floor_state`의 `state: 2`를 확인한다. 자동 재시도는 `rosservice call /multifloor/localize_auto`다. 미션 취소는 [기존 Mission.action UI 계약](../README.md#missionaction-cli와-ui-contract)에 따라 해당 goal ID를 취소하고 종료 결과를 확인한다. 물리적 정지는 현장에서 확인하며 0 명령만으로 정지했다고 판단하지 않는다.

주행 중 위치를 수정하려면 현재 미션을 취소하고 정지한 뒤 지정한다. 계단 주행·층 전환 중에는 위치 재설정을 받지 않는다. 위치 지정 자체가 복귀나 주행 명령을 만들지는 않는다. 사용자가 지정한 위치도 새로운 AMCL 결과와 스캔 일치를 확인하며, 수동 입력은 데이터 확인을 생략하는 기능이 아니다.

## 인터페이스와 소유권

새 ROS 노드는 없다. 기존 Floor Manager 안의 `StartupLocalization`이 시작 측위를 맡고, 지속적인 `map → odom` TF는 기존 AMCL이 맡는다. Stair Supervisor와 LiDAR 계단 추적 코어는 변경하지 않는다.

| 용도 | 인터페이스 |
|---|---|
| RViz/향후 웹 UI에서 위치·방향 지정 | `/initialpose`, `geometry_msgs/PoseWithCovarianceStamped`, frame `map` |
| 자동 탐색 다시 시작 | `/multifloor/localize_auto`, `std_srvs/Trigger` |
| 상태 표시 | `/multifloor/localization_status`, `std_msgs/String`의 JSON |
| AMCL로 승인된 초기 위치 전달 | `/multifloor/amcl_initialpose` — Floor Manager 소유 |
| 기존 미션의 주행 가능 판단 | `/multifloor/floor_state`의 `READY` |

상태 JSON의 `state`는 `SEARCHING`, `AMBIGUOUS`, `VERIFYING`, `READY`, `NEEDS_POSE`, `INPUT_REJECTED`다. `reason`, `source`, `request_id`로 현재 요청과 실패 원인을 구분한다. `INPUT_REJECTED`는 새 입력이 거부됐다는 뜻이며 이전에 확정된 위치를 자동으로 무효화하지 않는다. 웹 앱 화면은 이번 변경에 포함하지 않았으며 위 인터페이스를 그대로 연결할 수 있다.

`config.env`의 `STARTUP_LOCALIZATION=auto`가 기본이다. `manual`은 수동 지정을 기다린다. `disabled`는 기존 고정 좌표 방식을 명시적으로 선택하는 호환 모드다. Floor Manager를 단독 실행할 때는 `~startup_localization=auto`와 `~initialpose_topic=/multifloor/amcl_initialpose`, AMCL의 동일한 initialpose remap을 함께 지정해야 한다. 기본 단독 노드 모드는 기존 테스트와 호출자 호환을 위해 `disabled`다.

## 알고리즘과 한계

OBSERVED (소스와 실제 AMCL + 가상 센서 시험) — 저장된 지도와 현재 스캔을 비교하는 coarse-to-fine 탐색으로 후보를 만든다. 장애물 끝점 일치와 벽을 가로지르는 광선을 함께 평가하고, 서로 떨어진 후보의 점수 차이를 비교한다. 이후 후보를 AMCL에 주입하고 새로운 AMCL 표본, 공분산, 스캔/TF, 정지 상태 및 지도와의 일치를 확인한다. 정지 AMCL 갱신 요청의 응답 순서가 바뀌어도 재요청할 수 있다. 이 절차는 시작 또는 명시적인 재설정에만 적용하며 주행 중 반복적인 전역 탐색을 하지 않는다.

OBSERVED (소스·경로 단위시험·ROS 미션 시험) — 경로 계획은 첫 NAV를 실제 현재 pose에서 실행한다. 논리적 출발지와 목적지가 같아도 NAV를 생략하지 않으며, 논리적 위치가 계단 진입점이더라도 첫 단계가 계단이면 진입점 NAV를 먼저 수행한다. 기존에 NAV로 시작하는 3→4층 경로에는 추가 우회 지점을 넣지 않는다.

UNVERIFIED — 자동 탐색 점수와 후보 간 차이는 실제 위치 정답 확률이 아니다. 제한된 해상도의 탐색, 지도 변화, 가림과 반복 구조 때문에 틀린 후보가 남을 수 있다. 완전히 구별되지 않는 공간을 LiDAR 한 장으로 항상 구분한다고 보장하지 않는다. 실제 3F 여러 출발점에서 독립적으로 확인한 위치·방향과 대조하는 현장 검증이 필요하다. 가상 스캔 시험의 오차를 실로봇 정확도로 해석하면 안 된다.

| 제약 | 적용 범위·복구 | 판정 | Safety / Mobility |
|---|---|---|---|
| 후보 일치도·대안과의 차이 | 자동 시작 확정에만 적용. 모호하면 수동 위치·방향으로 후보를 좁힌다. 원점 이동이나 노드 재시작 불필요 | TUNE: 현장 false-positive율 미측정 | S2 / M2 |
| 새 스캔·AMCL·TF·정지 확인 | 시작/재설정에만 적용. 데이터가 돌아오면 대기 중 입력 처리, 실패 후 수동 재지정 또는 재탐색 가능 | NARROW: 기존 주행 중 게이트로 확대하지 않음 | S2 / M2 |
| 활성 미션·계단·층 전환 중 재설정 거부 | 현재 동작을 마치거나 취소한 뒤 다시 지정. 기존 FAULT의 층 정합성은 이 입력으로 우회하지 않음 | NARROW: 초기 위치 변경만 제한 | S2 / M2 |

TUNE은 현장 데이터로 문턱값 조정이 필요함, NARROW는 적용 범위를 시작/재설정으로 제한함을 뜻한다. S2는 제한 조건의 안전 여유 부족, M2는 특정 경로·기능 차단 영향이다. 센서가 부족해 입력이 대기 중이면 새 데이터가 돌아온 뒤 자동 처리하며, 이미 실패한 요청은 재지정하거나 재시도한다.

S/M은 이 변경의 잠재 영향 기록이며 전 시스템 감사 완료 판정이 아니다. 실제 계단 주행, 미끄러짐에 대한 유지 제어, 도착 판정은 별도의 현장 검증 대상이다.

## ROS upstream 확인

ROS Noetic [AMCL 소스](https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp)는 initial pose 입력으로 필터를 초기화하고, `request_nomotion_update`로 정지 상태의 스캔 갱신을 요청하는 동작을 정의한다. 지도 전체 후보 탐색과 READY 판단은 이 repository가 추가한 동작이다. [move_base 소스](https://github.com/ros-planning/navigation/blob/noetic-devel/move_base/src/move_base.cpp)의 `makePlan`은 현재 로봇 pose를 시작점으로 얻는다. 이 upstream 동작과 실로봇의 정상 주행을 동일시하지 않는다.
