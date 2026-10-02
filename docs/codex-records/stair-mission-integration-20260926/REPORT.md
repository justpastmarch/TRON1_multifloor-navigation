# 3F → 4F 임무 주행 준비 · 2026-09-26

목표는 3층 현재 위치에서 문을 지나 계단 진입점에 NAV로 도착하고, 기존 Stair Supervisor가 두 상승 구간과 계단참 회전을 수행한 뒤, LiDAR로 계단 이탈을 확인하고 AprilTag 400으로 4층을 확인하여 기존 도착 지점 `stair_4f_from_3f`를 유지하는 것이다. 새 ROS 노드를 추가하지 않는다.

## 적용 범위

- Supervisor의 도착 후 임시 유지에 적용되던 3초 만료를 제거한다. 계단 종료 후 정상 LiDAR 보정을 NAV 인계 또는 명시적 해제까지 유지한다. 모드 응답의 `handoff_sec: 3.0` 제한은 유지한다. 도착 대기를 계단 주행 300초 예산에 합산하지 않는다. 센서 만료, 좌표계 초기화, 자세 및 지지 영역 검사는 그대로 적용한다.
- 기존 Mission Manager 위치 복구를 활성화한다. 목표는 사용자 선택에 따라 기존 4층 도착 지점이다. 위치 0.08m/각도 0.12rad에서 보정 시작, 0.06m/0.10rad 이내에서 보정 종료. 최대 30초 동안 복구를 시도한다. 이 값은 현장 시험용 초기값이다. 새 임무는 기존 복구 취소 완료 후 시작한다.
- 도착 후 AMCL 갱신을 기존 `/request_nomotion_update`로 요청한다. 4층 지도 세대에 연결된 신선한 위치를 받은 뒤 LiDAR 유지를 인계한다. 인계 중 타이머가 소유권을 잃었는데 성공으로 반환하는 경합을 수정한다.
- 설치된 Noetic `TrajectoryPlannerROS`의 동적 설정에는 목표 허용 오차가 없다. 실행 중 구간별로 바꾸지 않는다. 위치 유지가 켜진 **이번 세션의 모든 NAV 목표**에 시작 시 XY 0.05m, yaw 0.08rad, XY latch false를 적용한다. 위치 유지가 꺼진 다른 실행은 기존 0.25m/0.2rad/true를 유지한다. 이동 중 inflation, footprint, 속도는 바꾸지 않는다.
- 계단참 마지막 경유점 X를 2.875m에서 2.97m로 바꾼다. 두 번째 첫 턱 X=2.52m와 몸체 중심 간격은 0.45m다. 기존 안쪽 경유점 X=3.26m를 거친다. 설정의 45cm 정사각형을 제자리 회전시키는 기하 검사에서 기존 12cm 여유를 만족한다. 이 검사는 현장 측량이나 동역학 검증이 아니다.
- 자동 진입 정합에서 점군 좌표를 현재 LiDAR 주위로 옮겨 계산하고 원래 좌표계로 복원한다. 평지 NAV 이후 추적 원점이 멀어졌을 때 회전 오차가 큰 평행이동 오차로 계산되는 문제를 줄인다. 연속 LiDAR 추적 알고리즘과 IMU 보정은 바꾸지 않는다.
- 정상 임무에 필요한 진입 점군·NAV 지도 좌표를 한 번 등록하는 `prepare_stair_mission.py`를 추가한다. 일회성 설정 도구이며 로봇 속도나 모드 명령을 보내지 않는다. 후보 생성과 적용을 분리하고 원본 해시, 백업, 기존 점군 덮어쓰기 방지를 적용한다. 사용자가 실제 위치/방향과 지도의 정합을 확인한 경우에만 등록한다.

## 근거와 한계

**OBSERVED:** 진입점 등록 및 정지 중 갱신 수정까지 포함하여 설치된 저장소에서 오프라인 236개 시험 통과. 실제 ROS launch 파서로 유지 기능 on/off 두 설정을 검사했다. 설치된 BaseLocalPlannerConfig에서 목표 허용 오차 동적 변경 미지원도 확인했다. 독립 검토에서 지적된 설정 호환성·인계 경합·시험 등록 누락을 수정했다.

**INFERRED:** 대기 시간 자체가 만든 인계 실패와 두 번째 진입점의 회전 여유 부족이 줄어들 것으로 예상한다. 시험은 가짜 명령 수신기와 합성 좌표를 사용했다. 이 결과를 실제 계단 완주 성공률로 해석할 수 없다.

**UNVERIFIED:** 등록한 3F 기준점의 독립 측량 오차와 재시작 간 중력축 일관성, 4층 도착 지점의 물리적 적합성, 8cm/6cm 유지 설정에서 TRON의 응답, 새 설정의 실장비 완주. 과거 BAG의 고정된 경로가 새로운 제어에 반응했다고 가정하지 않았다. `.10m` anchor 예산은 기존 운영자 확인 전제이며 정합 fitness/RMSE로 증명되는 위치 오차가 아니다.

주요 수정 평가: 인계 만료/경합 S3·M3, 진입 기준/회전 여유 S3·M3, 도착 위치 복구 연결 S3·M3. 이 보고서는 저장소 전체 감사 완료 보고가 아니다.

## 실제 시험 절차

1. 기존 실행 세션을 새 파일로 시작한다. `STAIR_RECORD=0 ./run.sh`는 등록·관찰 준비용이다. 주행시험 녹화는 아래 단계에서 별도로 시작한다. 새 NAV 또는 계단 목표를 자동으로 보내지 않는다. 기존 Supervisor는 연결되면 idle zero 명령을 보낸다. zero는 물리적 자세 유지 보장이 아니다.
2. 최초 한 번, 몸체 중심이 첫 계단 턱에서 45cm 떨어진 중앙에 있고 정면을 향하도록 놓는다. RViz 지도·scan 정합과 계단 영역 overlay, 실제 설치 치수를 확인한다. 잘못된 AMCL 대칭 위치를 후보 좌표로 저장하지 않는다.
3. `prepare_stair_mission.py capture`로 후보를 만들고 지도 좌표와 기준 점군을 검토한다. `--overlay-checked`는 계단 진입 형상이 구별되고 기존 anchor 오차 예산 안에 맞는다는 운영자 확인을 뜻한다. 후보의 `commissioned: true`는 운영자 확인을 전제로 한 시험 설정이지 자율 측량 성공 증명이 아니다.
4. 후보를 `apply`한 뒤 기존 스택을 재시작한다. `check_lidar_config.py --mission`으로 자동 진입 설정을 검사한다. 기준 점군이 없는 상태에서 commissioned만 켜지 않는다.
5. 위치를 평지 출발점으로 옮긴 후 현재 위치를 다시 확인한다. 센서 및 로봇 연결, FloorState READY, Supervisor NAV를 확인하고 녹화를 시작한다. `/mission`에 목적지 `stair_4f_from_3f`, mission_type `navigate`, return_after_task false를 보낸다. 이 미션 타입은 저장소의 MissionType.NAVIGATE 정의다.
6. 마지막 상승 → LiDAR EXIT_CONFIRM → tag 400 확인 → 4층 지도 준비 → 대기 위치 유지까지 기록한다. 단독 `stair_entry_test.py`의 성공/종료는 이 전체 임무 완료가 아니다.

등록 명령은 저장소 루트에서 다음과 같다. 아직 위치를 확인하지 않았다면 확인 플래그를 입력하지 않는다.

```bash
source config.env
source /opt/ros/noetic/setup.bash
source devel/setup.bash
"$STAIR_PYTHON" src/stair_supervisor/scripts/prepare_stair_mission.py capture "$STAIR_LIDAR_CONFIG" \
  --locations src/mission_manager/config/locations.yaml \
  --output logs/entry_registration_3f4f \
  --placed-at-entry --overlay-checked --map-pose-checked
"$STAIR_PYTHON" src/stair_supervisor/scripts/prepare_stair_mission.py apply logs/entry_registration_3f4f
# 기존 스택 재시작 후:
"$STAIR_PYTHON" src/stair_supervisor/scripts/check_lidar_config.py "$STAIR_LIDAR_CONFIG" --mission
```

`logs/entry_registration_3f4f`가 이미 있으면 새 이름을 사용한다. 원본과 후보가 바뀌면 적용은 거절된다. 기존 등록 점군은 자동 덮어쓰지 않는다.

## 현장 준비 관찰

2026-09-26 13:24:53 KST, 기존 `run.sh` 세션을 다시 실행했다. TRON connected=true, Supervisor NAV, FloorState 3F READY, LiDAR TRACKED, geometry_age 약 0.24초, 계산 시간 약 0.065초였다. 새 `2026-09-26-mission-hold-v2` 코드와 8cm/6cm 유지 설정, NAV 5cm/0.08rad가 실행 중인 것을 확인했다. 이는 실시간 관찰 시점의 상태다.

RViz 창의 픽셀을 읽어 직접 확인했다. 로봇은 문 뒤의 계단 입구 구역에 표시되었다. 관찰된 AMCL 값은 약 (9.9045, -6.9652), yaw 83.4°다. 해당 pose 메시지는 관찰 시각보다 오래되어 등록에는 다시 갱신해야 한다. 현재 scan의 지도 정합 일관성은 359점 중 약 82.45%가 occupied cell 중심 10cm 이내, 최근접 거리 중앙값 7.95cm였다. 이 수치는 지도/scan 일관성이고 독립적인 위치 정답 또는 대칭 오인식 배제 증명은 아니다.

이후 사용자가 현재 위치 등록을 명시적으로 승인했다. NAV 진입점은 (9.727567325, -6.617138729), yaw 1.493151744rad로 등록했고 동일 시점의 LiDAR 기준 점군을 저장했다. 기존 4층 도착 지점은 유지했다. 등록 완료 플래그와 실제 주행 준비 여부는 다르며, 아래 재시작 검증에서 추가 문제가 확인됐다.

추가 실행 경로 검사 5개도 통과했다. 등록 도구는 일회성 ROS 클라이언트이며 자동 launch graph에 넣으면 검사가 실패한다. 상주 제어 노드 목록은 그대로다. 기존 수정은 덮어쓰지 않았고 커밋·푸시를 하지 않았다. 대용량 BAG 녹화와 주행 목표 전송은 시작하지 않았다. 준비 세션은 사용자 시험용으로 실행 상태를 유지한다.

## 현재 결론: 등록 완료, 자동 계단 출발 준비는 미완료

**OBSERVED:** 설정 파일은 configured=true, 3F→4F route commissioned=true, 기준 점군 해시 검사는 통과했다. 이동/계단 목표는 보내지 않았다. 기준은 사용자가 확인한 현재 진입 배치다. `check_lidar_config --mission`은 파일 구성 검사이며 실제 재인식 성공이나 계단 완주 검사가 아니다.

**OBSERVED:** 초기 자동 진입 RMSE 한계 .06m는 0.20m voxel을 쓰는 실제 처리에서 같은 위치도 거절했다. 원시 점군과 대응 pose를 제어기와 같은 방식으로 처리했을 때 8개 스캔 잔차는 약 .116–.157m였고 .15m는 2개를 거절했다. 현장 진입 기준의 잔차 한계만 .18m로 조정했다. 후속 8개가 인식됐으나, 이는 독립 위치 정확도 증명이 아니다. 기존 anchor uncertainty .10m, 지지 영역, 자세, 센서 만료 조건은 그대로다. 판정 TUNE, Safety S3 / Mobility M3.

**OBSERVED:** 재시작 후 같은 기준으로 8개 모두 거절됐다. 이때 정합된 좌표계의 수직축 차이는 약 4.6–5.0°로 기존 허용 범위를 넘었다. RMSE만의 문제가 아니다. 이후 별도 실험에서 완전한 3D 좌표계 정합을 유지하면 다른 시점의 8개가 인식됐지만, 기준 중력축 및 .10m 물리적 오차 예산을 독립 검증하지 못했다. 해당 실험은 운영 코드에 적용하지 않았다. 현재 자동 계단 출발을 준비 완료라고 판단할 수 없다. 중력 초기화와 정지 중 추적 자세 변화의 기여도는 추가 확인 대상이다. 판정 UNVERIFIED, Safety S3 / Mobility M3.

## 정지 중 지도 정합과 엉뚱한 위치 인식

**OBSERVED (20초 관찰):** 수동으로 확인된 진입 위치를 기존 Floor Manager에 전달한 뒤 첫 10초에는 LiDAR odometry 50개, XY 변동 폭 약1.12cm; 바퀴 odometry 약2.45cm; AMCL 메시지는 처음 받은1개였다. 다음 10초에 기존 request_nomotion_update를 초당1회 요청하자 AMCL10개가 갱신됐다. AMCL XY 범위는 약12.93cm였으며 마지막 분산은 x .00597, y .03254, yaw .00782였다. 이 변화에는 분포 수렴이 포함되므로 실제 로봇 이동 거리로 해석하지 않는다. LiDAR XY 폭도 절대 정확도 또는 모든 자세축의 안정성 증명이 아니다.

**OBSERVED:** 실행 중 AMCL의 update_min_d=.1m, update_min_a=.1rad(5.73°). ROS upstream은 odometry 변화가 기준을 넘거나 no-motion 요청이 있어야 필터 갱신을 수행한다. 기존 repository는 시작 위치 확인 및 도착 유지 중에는 갱신을 요청하지만, 일반 평지 대기에서는 요청하지 않았다.

**적용:** 기존 Floor Manager의 StartupLocalization 내부에 1초 주기의 무이동 갱신을 추가했다. FloorState READY, Supervisor NAV, 활성 action 없음, 새롭고 신선한 scan에서만 실행한다. 초기 위치 재발행이나 전역 탐색은 하지 않는다. 일시적인 서비스 실패만으로 READY를 해제하거나 임무를 취소하지 않는다. 새 노드는 없다. 판정 NARROW/TUNE, Safety S2 / Mobility M2.

**INFERRED:** 관찰된 정지 중 scan/map 어긋남에는 AMCL 갱신 공백과 odometry 변화가 기여할 수 있다. 이 구간에서 3D LiDAR XY 발산 증거는 없었다. 수직축 변동과 장시간 누적 오차까지 해결됐다고 말할 수 없다.

**UNVERIFIED:** 정지 중 갱신만으로 대칭 공간의 잘못된 위치 선택은 해결되지 않는다. 사용자가 확인한 현재 진입 위치는 수동 후보로 복원하고 기존 scan 검증을 통과시켰다. 앞으로 전체 지도 자동 후보의 구별성, 과거 확정 위치와 LiDAR 상대 이동의 연속성, AprilTag 등의 독립 근거를 함께 사용해야 한다. 자동으로 잘못된 위치를 반복 덮어쓰는 방식은 적용하지 않았다.

## 다음 시작점

등록된 첫 계단 진입 위치에서 주행 목표를 보내기 전에, 저장 기준점과 새 추적 epoch의 중력축 차이를 독립적인 계단/평지 형상 및 IMU로 확인한다. 계단2단·3단 남김 문제와 4층 실제 유지 성능은 새 폐루프 현장 시험으로 확인해야 한다. 현재 기록은 원래 전 저장소 감사의 완료 선언이 아니다.

## 기록·변경 보존

설치된 저장소의 236개 관련 시험과 5개 실행 경계 시험이 통과했고 git diff --check도 통과했다. 감사 전후 Git 상태가 동일한 것은 아니다. 이번에 승인된 구현·설정·등록 파일과 문서가 추가/변경됐으며, 기준 목록에서 수정 대상으로 지정하지 않은 파일의 해시는 모두 보존됐다. 기존 ros_node.py 및 과거 현장 기록을 되돌리지 않았다. 커밋·푸시는 하지 않았다.

준비용 기존 run.sh/ROS/RViz 세션과 운영 로그는 사용자 확인을 위해 유지한다. BAG 녹화와 이동 목표 전송은 하지 않았다. 짧은 관찰·검사 클라이언트는 종료하며 별도 review 임시 디렉터리와 관찰용 임시 ROS 로그를 제거한다. 재현 스크립트·후보/원본 백업·JSON 계측·보고서는 임시물이 아닌 영구 검증 기록으로 보관한다.

## 최종 실시간 상태 · 2026-09-26 13:55–13:56 KST

새 Floor Manager 코드는 실행 중이다. 그러나 마지막 수동 측위 확인은 READY에 도달하지 못하고 localization timeout으로 끝났다. 관찰된 wheel odometry 속도는 vx≈.00208, vy≈-.01235m/s, wz≈-.00320rad/s였고, 기존 정지 판단은 평면 속도 .01m/s 이하다. 이 시점의 속도는 기준을 넘는다. 전체 타임아웃의 모든 시점에서 같은 원인이었는지는 추가 계측이 필요하다. 기준값은 이번에 완화하지 않았다. 판정 TUNE/UNVERIFIED, Safety S2 / Mobility M3.

`idle-refresh-live-final.json`의 AMCL 34개는 수동 위치 확인 루프의 갱신이 포함된 값이며 새 READY-idle 경로의 1Hz 작동 증명으로 사용할 수 없다. 그 경로는 오프라인 테스트 및 코드 검토를 통과했고 실제 READY 이후의 최종 관찰은 남았다. 현재 UNKNOWN/NEEDS_POSE, Supervisor NAV/connected 상태이므로 전체 주행 준비는 미완료다.

사용자가 보고한 현상을 다음처럼 구분한다: (1) AMCL 대기 중 갱신 공백은 실측 및 upstream 코드로 확인했고 기존 노드 안에서 보완했다. (2) 잘못된 장소 선택은 반복 구조에 대한 추론이며 실제 오인식 순간 기록은 아직 확보하지 못했다. (3) 계단 기준점의 재시작 간 중력축 차이는 정합 로그에서 확인됐고 원인 및 물리 오차는 미검증이다.

## 수동 진입점 phase test · 2026-09-26 14:03 KST

**OBSERVED:** 자동 임무의 점군 기준 인식은 재시작 후 수직축 차이 때문에 실패할 수 있었다. 동시에 수동 `stair_entry_test.py preview`가 운영자 배치로 만든 신선한 anchor도 ROS 경계에서 무조건 `ensure_entry()`로 다시 검사해 수동 phase test를 막는 결합을 확인했다. Supervisor ROS 경계에서 `phase_test`일 때만 그 자동 인식 호출을 생략하도록 수정했다. `StairFeedback.prepare(... phase_test=True)`의 동일 epoch, 신선한 센서, 등록된 진입 위치·몸체 support 및 높이 확인은 그대로 통과해야 한다. 자동 미션의 점군 자동 인식은 변경하지 않았다. Safety S2 / Mobility M3, 판정 NARROW. 관련 29개 회귀 시험 통과, `git diff --check` 통과.

**OBSERVED:** 기존 스택을 재시작하고, 사용자가 확인한 45cm·중앙·정면 배치에서 `preview --placed-at-entry`가 `captured; no goal or velocity sent`, epoch 0, 120초 창으로 성공했다. 이 표본은 현장 수동 시험의 진입 기준이다. 아직 `run` action 목표는 전송하지 않았고 실제 계단 주행 성공은 UNVERIFIED다. 현재 FloorState UNKNOWN 때문에 전체 3F NAV→4F 임무는 별도로 미준비다. 수동 phase test는 Floor Manager admission을 사용하지 않으며 완료해도 층 도착 판정은 아니다.
