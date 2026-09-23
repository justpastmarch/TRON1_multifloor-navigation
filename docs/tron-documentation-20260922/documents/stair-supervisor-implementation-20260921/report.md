# Stair Supervisor 구현 결과

> 보관일: 2026-09-22. 원본 HTML을 당시 내용 그대로 옮긴 기록입니다. 현재 적용 상태는 [최신 적용 안내](../../APPLICATION_GUIDE.md)를 확인합니다.

원본: [stair-supervisor-implementation-20260921/report.html](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/report.html) · SHA-256: `7daf56827ca9b84cd2f750affcb8fb7a26ec1b7b989731672b123c612980a9e8`

---

<span id="title-block-header"></span>

2026-09-21

TRON1 · IMPLEMENTATION DELIVERY

검증한 녹색 LiDAR 측위를 기존 Supervisor에 연결했습니다.

상행 피드백·계단참 보정·출구 판정·취소 인계·도착 후 NAV 복원 코드를 원본 저장소에 반영했습니다. **실제 계단 운용은 아직 활성화하지 않았습니다.** 장착·경로·지지 영역과 기체 대응을 실측해 채우고 현장 시험을 통과해야 합니다.

**1,307**기준과 비교한 bag 프레임 **0**추가한 운용 ROS 노드 **41**반영한 소스·설정·테스트·문서 파일

**소프트웨어 반영: 완료. 전체 운용 수락: 미완료.** 기본값은 `off`, 도착 복원 기능도 기본 비활성입니다. `stair_lidar.yaml`은 `configured: false`, 경로와 기체 장착 변환은 미확정 상태입니다. 새 기능이 기본 실행에서 자동으로 적용된다는 뜻이 아닙니다. 로봇 재시작·실제 주행 송신은 하지 않았습니다.

OBSERVED는 코드·설정·시험에서 직접 확인, INFERRED는 그 근거에 따른 판단, UNVERIFIED는 현장 또는 미실행 시험이 필요한 항목입니다. 이 보고서는 이번 구현의 납품·검증 기록이며, 과거 전체 저장소 감사를 다시 완료했다는 선언이 아닙니다.

<span id="무엇이-달라졌나"></span>

## 무엇이 달라졌나

| 기능             | 구현한 동작                                                                      | 근거와 한계                                                                                    |
| -------------- | --------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| 녹색 측위의 실시간 연결  | 보정 회전·초기 중력 정렬·자이로 예측·GICP·최근 15개 scan 지도를 보존. 원래 센서 시각으로 관측 제공             | **OBSERVED.** core와 calibration 해시 보존, 세 bag 동등성 통과                                       |
| 오르는 중 방향·측면 보정 | 목표선에서 벗어난 거리와 방향, 실제 관측 속도로 제한된 v/yaw 계산. 작은 노이즈는 deadband 처리               | **OBSERVED(코드/시험).** 실제 조향 부호·응답·밀림 감소는 **UNVERIFIED**                                    |
| 계단참 회전·대기      | 고정 위치/방향 목표와 이중 임계값으로 보정. 예측 이동 중 기체 전체 외곽과 사용 가능한 영역 검사                    | **OBSERVED(코드/시험).** 기체 자세나 무게중심을 직접 제어하는 새 인터페이스는 없음                                     |
| 다음 구간·최종 도착    | 진행량·높이·기체 전체의 지지 영역 진입·새 관측의 안정 시간을 함께 확인                                   | **OBSERVED(코드/시험).** 150° 회전이나 앞쪽 센서만 도착한 조건의 조기 완료를 시험에서 거부. 실제 마지막 단 통과는 **UNVERIFIED** |
| 짧은 결손과 장기 유실   | 유효 시간 안의 짧은 결손은 제한된 마지막 명령을 유지하며 완료 판정을 보류. 만료·frame 변경·계산 실패는 검증된 현장 대응 선택 | **OBSERVED(코드/시험).** 모든 성분을 0으로 만들고 지지가 됐다고 판단하지 않음. 실제 대응의 효과는 **UNVERIFIED**            |
| 취소·인계          | 취소 결과와 명령 소유권을 분리. 계단 중 취소 시 자동 NAV 전환을 막고 명시적 물리 인계까지 Supervisor가 소유       | **OBSERVED.** 종료·지연 결과·마지막 송신 중 취소 경합 검사 추가                                               |
| 도착 후 위치 복원     | mission 안의 선택 기능이 기존 NavigationExecutor를 직렬 재사용. 새 임무가 오면 이전 보정 취소를 확인하고 인계 | **OBSERVED(코드/시험).** 실제 복원 정밀도는 **UNVERIFIED**, 미수렴 성공 응답의 무한 재시도 방지                      |
| 관측·디버깅         | 원래 시각의 odom, 추적 상태, 제안 명령, 목표선·기체 외곽·경계·실제 phase 출력                         | **OBSERVED.** 관측 전용 ROS 시험에서 RobotTransport 생성 및 로봇 명령 출력 없음                              |

핵심 구현은 [Supervisor](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/supervisor.py), [피드백 계산](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/stair_feedback.py), [실시간 센서 연결](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/stair_supervisor/src/stair_supervisor/ros_lidar.py), [도착 복원](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/src/mission_manager/src/mission_manager/navigation_hold.py)에 있습니다. 설정 형식·관측 시작·진단·복구 절차는 [운용 문서](/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/docs/stair-lidar-integration.md)에 기록했습니다.

<span id="기존-구조와-호환되는-범위"></span>

## 기존 구조와 호환되는 범위

**OBSERVED:** 실행 위치는 기존 제어 PC입니다. mission manager·floor manager·move\_base와 함께 실행하는 구성과 기존 Supervisor의 단일 RobotTransport를 유지했습니다. 공개 action/result/phase/state 정의와 TF 소유자는 추가하지 않았습니다. `off`에서는 새 수치 라이브러리를 불러오지 않습니다. launch의 운용 노드 구성도 동일합니다. [정적 확인](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/static-checks.json), [launch 해석 결과](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/launch-dry-check.txt)

필요한 내부 분리는 한 가지였습니다. 첫 구현에서 같은 프로세스의 점군 처리가 40Hz 명령 경로를 지연시켰습니다. 이 결과를 보고한 뒤 **ROS 노드와 로봇 소켓이 없는 계산 자식 프로세스 1개**로 raw 점군 해석과 GICP를 옮겼습니다. 명령 결정과 송신 소유자는 기존 Supervisor입니다. 입력 큐·결과 큐·지도 크기를 제한하고, 멈춘 진입 정합은 실제 계산 프로세스까지 종료한 뒤 새 epoch로 재획득합니다.

**INFERRED:** 기본 비활성·인터페이스 보존·기존 회귀시험 결과는 점진적 적용을 뒷받침합니다. **UNVERIFIED:** 실제 센서·전체 ROS 부하·기체 응답을 포함한 완전한 운용 호환성은 아직 증명하지 못했습니다.

<span id="bag과-처리-지연-결과"></span>

## bag과 처리 지연 결과

비교 대상은 사용자 확인 기준 `gyro-gicp-installed-20260921-v1`의 동결 출력입니다. 아래 수치는 **동일 알고리즘 이식의 차이**입니다. 실제 위치 정확도나 자동 등반 성공률이 아닙니다. 비교 허용치는 사전에 정한 위치 1mm, 회전 0.05°입니다.

| bag    | 비교 프레임 | 상태·시각 불일치 |     최대 위치 차이 |     최대 회전 차이 | 동등성                 |
| ------ | -----: | --------: | -----------: | -----------: | ------------------- |
| 102101 |    500 |         0 | 1.24×10⁻¹⁰ m |  2.04×10⁻⁹ ° | **OBSERVED · PASS** |
| 105713 |    340 |         0 | 1.74×10⁻¹⁰ m |  2.20×10⁻⁹ ° | **OBSERVED · PASS** |
| 103313 |    467 |         0 | 1.10×10⁻¹¹ m | 5.10×10⁻¹⁰ ° | **OBSERVED · PASS** |

합계 1,307프레임에는 bag별 초기화 1프레임씩이 포함됩니다. 초기화 pose를 기하 관측 증거로 사용하지 않습니다. 103313의 회전량을 180°에 맞추는 수정도 하지 않았습니다. [102101 결과](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/realtime-verified-102101.json), [105713 결과](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/realtime-verified-105713.json), [103313 결과](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/replay-final/103313/metrics.json)

두 사용자 확인 bag은 **1배속 기록 입력 + 실제 계산 자식 프로세스 + 메모리상의 40Hz Supervisor 명령 경로**로 측정했습니다. 실제 로봇 소켓이나 mission/move\_base/floor/RViz 전체 동시 부하 시험은 아닙니다.

| bag    | scan 계산 P99 | 명령 경로 간격 P99 | 명령 경로 최대 간격 | 누락 scan / 최대 대기열 |
| ------ | ----------: | -----------: | ----------: | ---------------: |
| 102101 |      80.2ms |       25.3ms |      29.6ms |            0 / 2 |
| 105713 |      84.3ms |       25.2ms |      30.5ms |            0 / 2 |

**OBSERVED:** 이 구성에서는 계산 P99 200ms 미만, 명령 간격 P99 33.3ms 이하·최대 50ms 이하라는 계획의 구성요소 목표를 통과했습니다. 측정 명령·관찰 시각·참조 파일 해시는 [측정 출처](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/measurement-provenance.json)에 있습니다. 종료 경합만 다룬 마지막 수정은 별도의 프로세스 회귀시험으로 확인했습니다.

**남아 있는 시간 문제:** 105713의 기록 시각−센서 header 시각 P99는 **4.158초**, 처리 완료 때의 해당 관측 나이 P99는 **4.226초**였습니다. 현재 네트워크 지연이라고 단정할 수 없지만, 이 기록을 실시간 제어에 신선한 pose로 간주할 수도 없습니다. 현재 장비의 시계와 전달 시간을 관측해야 합니다. pose 시각을 덮어쓰거나 유효 시간을 임의로 4초까지 늘리지 않았습니다.

실패한 중간 실험도 보존했습니다. [동일 프로세스 간섭](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/realtime-105713.json), [센서 재생까지 명령 프로세스에 둔 시험](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/realtime-process-105713.json), [IMU 앞 구간을 생략해 동등성이 깨진 시험](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/realtime-final-102101.json). 최종 입력은 bag 시작부터의 IMU 이력을 제공하고, 실시간 초기화도 과거·미래 각각 0.5초가 실제로 확보됐는지 확인합니다.

<span id="시험에서-확인한-것"></span>

## 시험에서 확인한 것

| 시험 묶음        | 결과                                        | 검증 범위                                                            |
| ------------ | ----------------------------------------- | ---------------------------------------------------------------- |
| catkin 결과 집계 | **행동 사례 280개 + launch wrapper 14개, 실패 0** | 도구 집계 294 tests. 기존 mission/floor/stair·통신·합성 5F/RF 회귀와 새 NAV 복원 |
| 선택 LiDAR 시험  | **39개 PASS**                              | 피드백 27, ROS 입력 어댑터 7, 실제 자식 프로세스 수명 5                            |
| 관측 전용 ROS 시험 | **1개 PASS**                               | raw LiDAR/IMU 입력, 원래 시각 odom, 로봇 명령 경로 미생성                       |
| 빌드·정적 확인     | **PASS**                                  | catkin 빌드, Python/XML/shell 구문, 변경 공백, launch 노드 비교              |

집계는 서로 분리해 표시했습니다. catkin의 294개 결과 항목에는 wrapper 14개가 포함되므로, 294개의 고유 행동 검증으로 세지 않습니다. [시험 사례·시각·결과](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/test-results.json), [실행 환경](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/runtime-versions.json)

기존 시험이 실제 현행 설정과 어긋난 부분도 정리했습니다. 미설정 상태 검사는 임시 설정으로 독립시켰고, 합성 fixture 경로와 동일 좌표·다른 방향의 매칭을 수정했습니다. 시험을 통과시키려고 운용 제한을 느슨하게 하지 않았습니다. 원본 실제 bag과 동결 기준 출력은 수정하지 않았습니다.

<span id="주요-finding과-처리"></span>

## 주요 finding과 처리

점수는 잠재 영향입니다. S1은 간접 안전 영향, S2는 지지·경계 여유 상실 가능성입니다. M1은 지연, M2는 특정 기능 미충족, M3는 진행·복구 고착입니다. 실제 사고 관찰을 뜻하지 않습니다.

| Finding                              | 근거·처리                                                                                                                                   | 이중 평가                       |
| ------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------- | --------------------------- |
| F1 · 정상 5Hz 관측을 기존 wheel 입력 간격으로 거부  | **OBSERVED.** 새 LiDAR 제어에서 관측 나이·처리 시간·유효 기하를 분리. `test_normal_five_hz_never_hits_legacy_gap_fault` 통과. NAV/legacy wheel 한계를 일괄 변경하지 않음 | **Safety S1 / Mobility M3** |
| F2 · 계산 부하가 명령 경로를 지연                | **OBSERVED.** 동일 프로세스 최대 간격 약 142ms에서 내부 계산 분리 후 두 bag 최대 29.6/30.5ms. 전체 ROS/소켓 부하는 **UNVERIFIED**                                     | **Safety S2 / Mobility M2** |
| F3 · 취소·재시작·종료 사이 소유권 및 옛 관측 경합      | **OBSERVED.** epoch 결정과 enqueue를 원자적으로 묶음. 실행 중 계산 취소·큐 종료·마지막 송신 중 취소·늦은 HOLD 해제를 검사. 취소 후 자동 NAV 성공 전환 방지                             | **Safety S2 / Mobility M3** |
| F4 · 센서 도착·고정 회전·zero를 실제 지지로 간주할 위험 | **OBSERVED(코드/합성시험), UNVERIFIED(기체).** 전신 경계·진행/높이·실제 방향·신선한 dwell로 판단. 계단참/출구 보정 추가. 실제 균형 유지가 해결됐다고 결론내리지 않음                          | **Safety S2 / Mobility M2** |
| F5 · 좋은 재생 궤적과 오래된 센서 관측의 혼동         | **OBSERVED.** 105713 timestamp 차이 P99 4.158초. 시각 보존과 진단 추가. 현재 시계/전달 조건과 운용 예산은 **UNVERIFIED**                                          | **Safety S2 / Mobility M3** |
| F6 · NAV 복원 성공 응답과 실제 목표 정밀도 불일치     | **OBSERVED(합성시험).** 요구 오차 밖인데 성공을 반환하면 반복 보정을 멈추고 원인을 표시. 새 임무에 대한 취소 범위를 정확히 제한. 실측 복원 성능은 **UNVERIFIED**                              | **Safety S1 / Mobility M2** |

<span id="제약-ledger"></span>

## 제약 ledger

이번 구현에는 물리적 최소성을 증명하지 않은 **KEEP 판정이 없습니다.** 아래 제약은 새 opt-in 계단 제어 또는 도착 복원에 한정됩니다. 정상 NAV·촬영·메일의 전역 gate로 확대하지 않았습니다. 코드 범위는 OBSERVED, 물리 한계의 최적값은 별도 표기대로 미확정입니다.

| 주요 제약                 | 차단하는 구체 상황 · 복구 범위                                              | 판정                    |
| --------------------- | --------------------------------------------------------------- | --------------------- |
| commissioning·상행 경로   | 미측량 변환/경계/응답 또는 하행에 상행 식 적용. 해당 경로의 측량·검증 후 진입                  | **UNVERIFIED**        |
| GICP 수락 한계            | 해당 scan의 큰 정합 보정. 다음 scan 재시도, 수락 전 새 기하 증거로 사용하지 않음            | **TUNE**, 동결 수치 유지    |
| IMU 초기 창·지원 공백        | 부족한 초기 중력 이력 또는 잘못된 회전 예측. 도착한 표본을 기다리거나 결손을 표시                 | **NARROW**            |
| 관측 나이·유효 시간           | 오래된 위치로 움직이거나 완료 판정. 짧은 결손은 같은 frame에서 회복, 만료 후 현장 대응/인계        | **TUNE / UNVERIFIED** |
| epoch·시계·entry anchor | 다른 실행·다른 원점의 pose 사용. 해당 결과 폐기, 새 frame의 기하 재연결                 | **NARROW**            |
| 진입 기하의 모호성            | 대칭 후보 중 잘못된 계단 기준 선택. 독립 진입 측정 또는 고유 기하의 새 관측                   | **TUNE / UNVERIFIED** |
| v/yaw·가속도·deadband    | 노이즈 추종·급격한 조향. 오차 해소 시 정상 명령 복귀                                 | **TUNE**              |
| 기체 외곽·예측 이동 영역·기울기    | 통로/계단참 밖 이동 또는 확인되지 않은 자세. 검증된 대응과 인계; 기체 실측 후 여유 조정            | **TUNE / UNVERIFIED** |
| 회전·출구 완료·새 관측 dwell   | 150° 또는 센서만 평지에 오른 조기 종료. 유효 영역 안에서 잔여 정렬/진행을 관측                | **TUNE / UNVERIFIED** |
| 장기 유실·취소 대응           | zero만으로 지지를 가정하거나 자동으로 모드 전환. 경로별 대응 증거와 명시적 물리 인계 필요           | **UNVERIFIED**        |
| 계산 큐·진입 작업 기한         | 과거 scan 무한 누적·멈춘 정합. 누락 진단, 자식 회수, 새 epoch 재획득                  | **NARROW / TUNE**     |
| 계단 종료의 임시 위치 보정       | 후속 소유자 준비 전 무소유 상태. 제한된 인계 시간과 명시적 인계                           | **TUNE / UNVERIFIED** |
| NAV 도착 복원             | 작은 흔들림의 반복 이동·틀린 층 pose의 복원·다음 임무 취소. 이중 임계값, map 세대 확인, 요청별 취소 | **NARROW / TUNE**     |

<span id="계획의-15개-수락-시나리오-판정"></span>

## 계획의 15개 수락 시나리오 판정

PASS는 아래에 명시한 시험 범위의 판정입니다. PARTIAL은 소프트웨어 근거가 있으나 원래 시나리오의 현장/전체 부하 조건은 남아 있음을 뜻합니다. 모든 항목을 판정했지만 모든 항목이 통과한 것은 아닙니다.

| ID                          | 결과                         | 확인한 범위 · 남은 확인                                                        |
| --------------------------- | -------------------------- | --------------------------------------------------------------------- |
| T01 · 102101 동등성            | **PASS · OBSERVED**        | 500프레임, 같은 상태·시각·수치 허용차                                               |
| T02 · 105713 동등성            | **PASS · OBSERVED**        | 340프레임, 큰 회전 구간 포함                                                    |
| T03 · 103313 회귀             | **PASS · OBSERVED**        | 467프레임, 회전량 임의 보정 없음                                                  |
| T04 · 인과적 입력/초기화            | **PARTIAL**                | 도착한 IMU의 앞뒤 초기 창 대기 시험 통과. 별도의 미래 입력 변조 대조시험은 미실행                     |
| T05 · timestamp/epoch       | **PASS · OBSERVED(소프트웨어)** | 중복·역행·clock 변경·지연 결과의 무효화 시험                                          |
| T06 · 5Hz/CPU 부하            | **PARTIAL**                | 구성요소 1배속 목표 통과. 전체 ROS/RViz 및 실제 송신 부하 미실행                            |
| T07 · 짧은 거절/IMU 공백          | **PARTIAL**                | 거절 후 같은 phase 회복·restamp 금지·제한 버퍼 확인. 실제 센서 공백과 주행의 결합 미확인            |
| T08 · 장기 유실/worker/watchdog | **PARTIAL**                | 계산 중 취소·실제 자식 종료·소유권 시험 통과. zero 후 실제 자세/위치 대응 미확인                    |
| T09 · yaw 응답                | **PARTIAL**                | 수학적 부호·deadband·한계·잘못된 설정 거부 통과. 기체 실응답 미확인                           |
| T10 · 진입 정렬/이탈 보정           | **UNVERIFIED(현장)**         | 변환·대칭 모호성·장착 lever arm 단위시험 있음. 외부 측량에 대한 정렬/보정 미실행                   |
| T11 · 계단참 밀림/회전             | **PARTIAL**                | 고정 목표·중간 궤적·150° 조기 완료 거부 시험. 실제 밀림 회복 미실행                            |
| T12 · 마지막 2\~3단             | **PARTIAL**                | 전신 지지 전 완료 거부 시험. 실제 마지막 단·가림 조건 미실행                                  |
| T13 · HOLD/새 NAV/정지         | **PARTIAL**                | 직렬 소유권·취소·지도 변화·미수렴 검사. 실제 장시간 위치 복원 미실행                              |
| T14 · 취소/재시작/층 전환           | **PARTIAL**                | 새 경로의 취소/epoch 경합과 기존 floor/mission 회귀 통과. 새 LiDAR 제어와 실제 층 전환 결합 미실행 |
| T15 · 하행/다른 계단/자동문          | **NOT\_RUN / UNVERIFIED**  | 새 제어는 상행 전용. 기존 합성 5F/RF 통과를 새 하행·문 응답의 실증으로 대체하지 않음                  |

<span id="다음-실장비-적용에-필요한-정보"></span>

## 다음 실장비 적용에 필요한 정보

기존에 주신 **폭 1.23m, 계단참 2.70×1.48m, 5F→RF 10+9+2단**은 문서에 보존했습니다. 3F→4F의 실측 좌표·높이·장착 translation·동적 지지 영역으로 자동 확대하지 않았습니다. 로봇의 장착 x/y와 동적 지지 크기 자료 위치, stair mode+zero에서 밀릴 때 실제로 지지에 성공한 조작은 이미 질문한 상태이며, 답을 대신 지어넣지 않았습니다.

현장 작업 순서는 계획의 M2 전체 부하 → M3 관측 전용 → M4 평지 응답/기체 대응 → M5 3F→4F 전체 → M6 장시간 도착 복원과 다른 계단입니다. 현재 코드와 catkin 빌드는 분리 작업공간에서 확인했고 원본 소스로 반영했습니다. 원본의 기존 실행 프로세스를 재시작하거나 원본 build/devel 산출물을 덮어쓰지는 않았습니다. 적용 시 운용 중이 아닌 상태에서 빌드·선택 Python 환경·보정 해시를 확인합니다.

**Safety verdict: UNVERIFIED(실장비).** 구현상의 시각·소유권·완료 계약은 시험했지만, 실제 중심 유지·기울어짐 억제·지지 대응·계단 경계 미침범은 입증하지 못했습니다.

**Mobility verdict: OBSERVED(소프트웨어 개선), UNVERIFIED(실제 성공률).** 정상 5Hz 입력과 기존 간격 gate의 충돌을 분리하고 피드백/복원을 구현했습니다. `off`에서는 기존 운용이 유지되며, 새 제어의 정상 주행 중 불필요 정지 0회는 현장 시험으로 확인해야 합니다.

**전체 운용 수락 상태: AUDIT\_INCOMPLETE.** 소프트웨어 구현·반영이 끝났다는 사실과 실장비 수락이 끝났다는 주장을 구분합니다. 남은 시작점은 측량/기체 대응 자료 연결과 전체 부하·관측 시험입니다. 하행과 5F/RF 문 동작의 신규 제어 적용도 별도 수락 대상입니다.

<span id="변경근거정리-기록"></span>

## 변경·근거·정리 기록

원본은 `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`입니다. 시작 HEAD는 `eb24e08c9220f454080bd672bd8ce3ec0aeb7159`, 시작 Git 상태는 clean이었습니다. **종료 시 의도한 수정 23개 + 추가 18개가 있습니다. Git 차이가 없다는 보고가 아닙니다.** 커밋·reset·기존 변경 되돌리기는 하지 않았습니다. HEAD는 그대로이며 변경 대상 밖 파일은 기준 해시와 일치합니다. [반영 확인](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/application-receipt.json), [파일별 변경 manifest](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/delivery-manifest.json), [변경 patch](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/changes.patch)

시작 inventory는 repository-owned 항목 1,764개입니다. 추가 18개를 포함한 최종 1,782개는 **READ 41 + METADATA\_ONLY 1,741 + EXCLUDED 0 + UNREAD 0**으로 조정했습니다. 이번 보존 ledger는 구현 파일 검토와 나머지 항목의 해시/메타데이터 확인을 구분합니다. 모든 기존 파일을 정독했다는 의미가 아닙니다. [상태/coverage ledger](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/state.json). 원본 bag·동결 결과·동기화된 sources는 보존했습니다.

raw 로그 26개는 구조화한 시험 결과로 옮긴 뒤 제거했습니다. 격리 clone의 build/devel/cache, ROS 시험 홈, 리뷰 사본, 임시 적용 스크립트와 이번 시험 PID에 대응하는 공유 메모리 폴더 6개도 정리했습니다. 영구 보관물은 이 보고서·검증 지표·출처·변경 기록입니다. [정리 영수증](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/cleanup-receipt.json)

ROS upstream과 repository 동작을 구분했습니다. raw 메시지·구독 버퍼·bag 표현은 설치된 Noetic/Livox 소스로 확인했습니다. 물리적 응답을 ROS API가 보장한다고 주장하지 않습니다. [설치 소스 근거·독립 검토·평가 한계](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-supervisor-implementation-20260921/quality-review.json)
