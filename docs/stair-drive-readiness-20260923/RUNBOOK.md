# 지정 진입점에서 RViz를 보며 시험하기

> 2026-09-23 최신 수정·실주행 결과: [현재 개발 상태](../development-status-20260923.md). 아래 기존 설명과 시점별 관측 기록은 해당 당시 범위로 해석하세요.


대상은 **3→4층 계단 진입점에서 시작하는 명시적 단계 시험**이다. 3층 임의 위치 NAV부터 4층 정상 도착까지의 전체 미션 완료를 보증하지 않는다. 자동 진입 측위 대신 이미 허용한 위치 지정 방법을 사용한다. [구현·검증 범위](REPORT.md), [현재 실행 준비 상태](LIVE_READINESS.md).

## 시작 위치

기존 NAV 임무가 실행 중이지 않은 상태에서, 몸체 중심을 첫 단 수직면보다 **45cm 앞**, 계단 폭의 중앙에 놓고 계단 정면을 향한다. 45×45cm 몸체의 앞면은 첫 단에서 약 22.5cm 떨어진 배치다. 진입 바닥에 몸체가 있고, 기존에 확인한 RC 회수·자세 유지 방법을 사용한다. 이미 주신 계단 치수를 다시 입력할 필요는 없다.

## 공통 준비

새 터미널마다 아래 준비를 실행한다. 경로는 이번에 확인한 로컬 설치를 기준으로 적었다. 다른 장비에 경로를 추측해서 복사하지 않는다.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
source ./config.env
export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}"
export ROS_IP="$(ip -4 route get "$MINI_PC_HOST" | awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')"
unset ROS_HOSTNAME
export STAIR_TEST_CONFIG="$STAIR_LIDAR_CONFIG"
```

계산용 Python의 고정 NumPy/Open3D를 먼저 사용하고 ROS/system 라이브러리와 실제 LiDAR 메시지 경로를 뒤에 연결한다. 기존 가상환경을 변경하거나 새 패키지를 설치하지 않는다.

## 1. 설정 검사와 기존 스택 실행

```bash
./run.sh --check
./stair_control_session.sh start
```

`run.sh --check`는 로봇에 연결하지 않고 bundle·ROS 설치·시간 동기화·선택된 LiDAR 설정을 확인한다. 빠른 설정 검사만 필요하면 `./stair_control_session.sh check`를 사용한다. `config.env`가 기존 실행 경로의 기본값을 `control`/`observe_only=false`로 지정한다. route `stair_3f_4f_up`, `configured: true`, `commissioned: false`, 라이브러리 버전과 설정 hash를 출력한다. `start`는 기존 `run.sh`로 센서/미션/Supervisor를 실행하므로 실제 운영 실행이다. 별도 이동 목표는 자동 제출하지 않는다. 이미 스택이 실행 중이면 중복 실행하지 않고 현재 세션을 정리한 후 같은 설정으로 시작한다.

시작 터미널에 `[READY]`가 출력된 뒤 다음 단계로 진행한다. 위치를 찾는 메시지에서 기다리는 경우 NAV 시작 위치 인식이 아직 끝나지 않은 상태다. `run.sh`의 `/mission` 안내는 정상 미션용이며, 아래 `stair_entry_test.py`는 기존 명시적 시험 경로다.

이 실행 경로는 `logs/<시각>/`에 설정 사본·hash·센서·명령·상태 BAG를 자동 기록한다. 따로 BAG 명령을 입력하지 않아도 된다. 실시간 영상 스트리밍을 추가하지 않는다.

## 2. RViz 실행

공통 준비를 한 다른 터미널에서:

```bash
rosrun rviz rviz -d "$PWD/src/stair_supervisor/rviz/stair_lidar_control.rviz"
```

녹색 화살표는 기존 LiDAR 측위, 회색 점군은 같은 시각의 관측을 그 측위로 옮긴 결과다. 기준 프레임은 `stair_local_0`이며 tracker reset 뒤에는 `/stair_supervisor/lidar_odom`의 `header.frame_id`와 맞춘다. 점군이 없으면 다른 시각의 점군을 억지로 붙이지 않고 표시를 생략한다.

## 3. 이동 없는 진입 미리보기

시작 위치에 놓은 뒤 공통 준비를 한 터미널에서:

```bash
"$STAIR_PYTHON" ./src/stair_supervisor/scripts/stair_entry_test.py preview "$STAIR_TEST_CONFIG" --placed-at-entry
```

이 명령은 현재 위치를 **지정 배치로 선언**하는 것이며 자동 인식이 아니다. action 목표나 속도를 보내지 않는다. RViz에 `ENTRY PREVIEW / NO TEST COMMAND`와 몸체·경로 테두리가 나타난다. 위에서 본 XY 기준으로 첫 계단 중앙, 계단참 테두리, 돌아올 두 번째 계단 방향이 실제 구조와 맞는지 확인한다. Z는 몸체 기준이므로 테두리가 점군의 바닥 표면에 붙어야 하는 것은 아니다.

표시가 맞으면 같은 tracking epoch에서 120초 안에 다음 시험을 시작할 수 있다. 조금 밀려도 `run`이 원점을 현재 위치로 다시 옮기지는 않는다. 실제 시작 구역 밖이면 Supervisor가 거절한다. 위치를 아무렇게나 놓은 상태에서 표시를 맞추려고 preview를 반복하지 않는다.

## 4. 첫 계단부터 계단참까지

```bash
"$STAIR_PYTHON" ./src/stair_supervisor/scripts/stair_entry_test.py run "$STAIR_TEST_CONFIG"
```

기존 VERIFY_ENTRY → ALIGN → FORWARD_SEGMENT_1 순서다. 모드 응답 뒤 시험 시간이 시작되고 전체 상한은 65초다. 구간 목표가 먼저 충족되면 종료한다. **시험 종료는 정상 층 도착 성공이 아니며 0 명령과 STAIR 소유권을 유지한다. 0은 물리적 위치 유지 보장이 아니다.** RC로 자세를 유지하는 기존 방법을 따른다.

첫 구간 검증 후, 같은 진입 배치에서 계단참 회전까지 확인하는 명령:

```bash
"$STAIR_PYTHON" ./src/stair_supervisor/scripts/stair_entry_test.py run "$STAIR_TEST_CONFIG" --phase TURN_TO_NEXT_FLIGHT --seconds 195
```

실기 단계 결과를 확인한 뒤 진입부터 상부 진출까지 연결하는 시험 명령:

```bash
"$STAIR_PYTHON" ./src/stair_supervisor/scripts/stair_entry_test.py run "$STAIR_TEST_CONFIG" --phase EXIT_CONFIRM --seconds 255
```

각 실행은 새 preview를 소비한다. 위 명령은 모두 진입점부터 실행하며 계단참에서 첫 명령의 뒤를 자동 재개하는 명령은 아니다. 중간 시작은 기존 개별 `phase_test`와 그 위치의 좌표 증거가 필요하다. 이 문서의 연결 시험은 floor manager에 정상 층 도착을 발행하지 않는다.

## 상태·취소·정리

```bash
rostopic echo /stair_supervisor/control_debug
```

`phase`, `command`, `boundary_adjusted`, `below_flight_command_floor`, `clearance_m`, `effective_margin_m`, `incomplete_conditions`, `tracking_state`를 확인한다. 속도 하한은 WebSocket 입력의 근거이며 실측 이동 속도나 토크 보증이 아니다. `tracking_status`의 mode/hash/protocol이 맞지 않으면 새 클라이언트가 실행을 거절하므로 저장소만 바꾸고 옛 노드를 계속 쓰는 상태를 구분할 수 있다.

시험 클라이언트의 **Ctrl+C는 그 시험 goal ID만 취소**한다. 서버 처리 종료 확인과 미확인을 출력한다. 이는 물리적 자세 유지 확인이 아니다. 별도 비상 정지 API를 호출하지 않는다. 시험이 끝나고 로봇을 지지 가능한 평지로 회수한 뒤 기존 명시적 인계를 사용한다:

```bash
rosparam set /stair_supervisor/operator_confirmed_supported_handoff true
rosservice call /stair_supervisor/acknowledge_physical_handoff
```

그 후 새 시험 위치로 배치하고 preview부터 다시 시작한다. 세션 전체 종료는 `start` 터미널에서 Ctrl+C다. rosbag 인덱스 정리가 끝날 때까지 종료를 기다린다. 기존 미니PC 센서 세션은 실행 도구의 기존 정책대로 유지한다.
