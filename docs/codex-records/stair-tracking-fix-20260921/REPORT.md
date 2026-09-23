# 1·2 수정 적용 결과

**실제 비교 도구 소스에 두 수정 모두 적용했습니다.** 설치된 코드로 13개 테스트가 통과했고, 원본 bag 3개의 기존 분석 구간 1,307프레임을 다시 처리했습니다. 수정된 LiDAR bag과 RViz 비교 bag도 각각 3개 만들었습니다. 관찰 시각: 2026-09-20T18:02:54.559737+00:00.

이 문서는 요청한 두 수정의 적용·검증 기록입니다. 전체 저장소 감사나 Stair Supervisor 실주행 검증을 완료했다는 보고는 아닙니다.

## 적용한 수정

| 원인과 판정 | 실제 변경 | 근거와 잠재 영향 |
| --- | --- | --- |
| F1. 우세 평면으로 위쪽 방향을 정해 초기 좌표계가 약 90° 기울어짐. **OBSERVED:** 기존 출력·코드. **INFERRED:** IMU 방향과 대조하면 벽을 바닥으로 선택한 것으로 해석됨. | 장착 회전을 적용한 IMU 가속도의 초기 ±0.5초 중앙값으로 위쪽을 정함. 주변 벽의 크기가 초기 중력을 결정하지 않음. | [registration_core.py](/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/registration_core.py:112), [설정 로드](/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/process_bag.py:165). Safety **S1**, Mobility **M2**. |
| F2. 큰 회전에서 정합을 놓친 뒤 오래된 위치를 새 시각의 odom/cloud로 출력. **OBSERVED:** 기존 코드와 105713의 166프레임 연속 거절. | 측정 header 시각과 보정된 gyro로 다음 회전을 예측하고 기하 정합으로 수정. 거절 시 odom을 쓰지 않고 pose는 NaN, 상태·사유·마지막 유효 시각을 기록. 복구 후 경로는 새 구간으로 시작. | [회전 예측](/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/registration_core.py:138), [출력 처리](/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/process_bag.py:287), [회귀 시험](/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/test_process_bag.py:86). Safety **S2**, Mobility **M3**. |

점수는 잠재 영향이며 실제 사고 관찰이 아닙니다. S0 영향 미확인, S1 간접 영향, S2 안전 여유 부족, S3 fault 중 위험 명령, S4 충돌·추락·명령 소유 상실 가능성. M0 영향 미확인, M1 간접 영향, M2 특정 기능 차단, M3 복구 고착, M4 핵심 임무 불가능.

**OBSERVED:** 실제 계산 변경은 기존 `registration_core.py`, `process_bag.py` 안에 있습니다. 나머지는 테스트·설정·설명·RViz 표시 연결입니다. 기존 파일 11개 수정, 테스트 파일 1개 추가이며 새 ROS 노드는 없습니다. [전체 변경](fix.patch) · [설치 기록](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/installed.json).

## 같은 분석 구간에서 확인한 결과

| bag | 유효 pose 수, 초기화 포함 | 최종 상대 yaw 추정 | 최장 연속 거절 구간 |
| --- | --- | --- | --- |
| 102101 | 450/500 → 500/500 | 27.5° → 181.4° | 8.0초 → 0초 |
| 103313 | 353/467 → 467/467 | 164.0° → -107.7° | 15.2초 → 0초 |
| 105713 | 168/340 → 340/340 | -2.6° → 177.8° | 33.0초 → 0초 |

**OBSERVED:** 새 출력은 3개 초기화와 1,304개 기하 갱신으로 구성됩니다. 기존 출력과 선택한 센서 시각 1,307개가 정확히 일치합니다. odom 시각·CSV 위치·상태 메시지·보정 파일 해시도 대조했습니다. [새 출력 검증](bag-validation.json) · [기존 지표](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-repair-20260921/configured-metrics.json).

**이 수치는 실제 위치 오차나 정확도 백분율이 아닙니다.** 기존에 문제를 발견한 구간에 대한 회귀 시험이며 독립 정답·미사용 평가 구간은 없습니다. 103313의 최종 상대 회전은 −107.7°로 남아 있으므로 세 bag 모두 실제 180° 회전을 정확히 맞췄다고 판정하지 않습니다. 이전 보고서의 캐시 기반 후보와 달리 이번 결과는 실제 처리 코드가 원본 bag의 IMU·LiDAR를 직접 읽은 값입니다.

## 실패와 복구가 보이는 구조

**OBSERVED:** `/lidar_registration/status`에는 `BOOTSTRAP`, `TRACKED`, `PREDICTED`, `LOST`, `WAITING_FOR_INITIALIZATION`과 실패 이유가 기록됩니다. `geometry_valid`는 초기화 때 false이고 실제 정합이 통과했을 때만 true입니다. `measurement_valid`는 초기 상대 pose도 포함하므로 정합 성공 판단에는 `geometry_valid`를 사용합니다.

정합 거절 시 현재 odom과 지도 갱신을 생성하지 않습니다. 빈 live 경로·점군 메시지를 쓰며 다음 유효 관측부터 경로를 다시 시작합니다. 전역 지도에는 마지막 유효 관측 시각을 유지합니다. 강제로 거절을 넣은 시험에서 이 동작과 복구를 확인했습니다. IMU 중간 누락 시에도 기존 범위 안의 기하 복구를 시도하고, 다시 지원되는 구간부터 gyro 예측을 재개합니다. 큰 이동을 놓치면 복구하지 못할 수 있으며 전역 재측위를 새로 구현한 것은 아닙니다.

## 제약 ledger와 주행 영향

아래 값은 오프라인 데이터 처리에만 적용됩니다. **OBSERVED:** 이번 수정이 로봇의 주행 허용 조건이나 속도 명령에 추가한 gate는 0개입니다. 물리적 안전 constraint에 대한 KEEP 판정은 내리지 않았습니다.

| 항목 | 범위·복구 | 판정 |
| --- | --- | --- |
| 정합 fitness ≥ 0.35, 예측 대비 위치 보정 ≤ 0.5m, 회전 보정 ≤ 0.35rad | 기존 숫자를 유지하되 gyro 예측 주변에서 검사. 거절은 해당 측정 출력에 한정, 다음 관측으로 복구 시도. 실제 계단 전체에 대한 최소성은 미증명. | **TUNE**: 실장비에서 조정·최소성 검증 필요 |
| IMU 표본 간격 ≤ 0.03초 | 해당 구간의 gyro 예측 지원 여부. 누락 시 같은 정합 범위로 기하 복구 가능. | **TUNE**: 기록 주기용 설정이며 로봇 정지 조건 아님 |
| PREDICTED 표시 기간 0.5초 | 진단 상태 구분만 수행. 기간 안이라도 측정 odom으로 간주하지 않음. | **TUNE**: 표시 설정 |
| 초기 가속도 ±0.5초 중앙값 | 오프라인 lookahead 사용. 동역학적 가속도가 적은 구간을 전제하며 정지·물리 보정 정확성은 인증하지 않음. | **UNVERIFIED**: 실기 초기화 조건 |
| 0.2m voxel, 0.8m 대응 거리, 15개 local scan, 0.5~20m 점 범위, 매 2번째 LiDAR packet | 기존 세 spec의 처리 범위. 새로운 주행 차단 조건 없음. | **TUNE**: 오프라인 처리 설정 |
| 거절된 측정을 새 odom으로 출력하지 않음 | 관측 유효성에만 한정. 다음 수락 관측에서 자동 재개하고 상태를 기록. | **NARROW**: 오래된 pose의 새 측정 위장 방지 |

Safety 판정: **OBSERVED**, 측정 실패를 최신 측위로 위장하던 출력은 수정·시험됨. **UNVERIFIED**, 실제 계단 안전과 중심 유지.

Mobility 판정: **OBSERVED**, 같은 세 구간에서 거절 프레임이 50/114/172개에서 모두 0개로 감소. **UNVERIFIED**, 실제 임무·계단참 회전·횡방향 보정 성공률.

## 검증과 적용 범위

**OBSERVED:** 설치된 소스의 13개 테스트, 정적 검사, 재생 스크립트 구문 검사, 실제 CLI 로딩이 통과했습니다. 빈 입력에서 기존 결과를 덮어쓰지 않는지, 원본 bag을 출력으로 지정할 수 없는지, 센서 장착 회전과 IMU 누락을 처리하는지 검사했습니다. [테스트 기록](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/tests.log).

**OBSERVED:** 새 비교 bag의 LiDAR 위치가 새 추정 CSV와 일치하고 표시 문구도 갱신됐습니다. [비교 bag 검증](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/comparison-validation.json). RViz 화면을 직접 재생해 확인하지는 않았습니다.

**UNVERIFIED:** FAST-LIO 자체의 발산 원인·수정, 절대 위치 정확도, 실제 로봇의 중심 유지, Supervisor 연동. 이 구현은 회전 예측용 gyro를 추가한 비교 도구이며 FAST-LIO의 온라인 보정 추정 상태를 재현하지 않습니다. 기존 wheel/FAST-LIO 겹침은 개별 시작 자세와 bag 기록 시각을 사용하므로 정밀 오차 측정으로 해석하지 않습니다. [실제 사용 설명](/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/README.md).

## 수정 결과 열기

새 산출물은 아래 폴더에 보관했습니다. 예전 `build/offline_lio`의 결과는 보존했으므로 새 결과를 보려면 아래 경로를 사용합니다. 각 명령은 GUI가 있는 환경에서 기존 재생 도구를 열며, 이번 작업에서는 실행하지 않았습니다.

```bash
bash "/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/replay_rviz.sh" \
  "/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/stair_captures/stair_3F_to_4F_UP_20260827_102101.bag" \
  "/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/validation/102101/lidar_registration.bag" \
  "/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/validation/102101/comparison.bag"
```
```bash
bash "/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/replay_rviz.sh" \
  "/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/stair_captures/stair_3F_to_4F_UP_20260827_103313.bag" \
  "/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/validation/103313/lidar_registration.bag" \
  "/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/validation/103313/comparison.bag"
```
```bash
bash "/home/m3tron/Desktop/TRON1_Modular_Navigation/tools/offline_lio/replay_rviz.sh" \
  "/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/stair_captures/stair_3F_to_4F_UP_20260827_105713.bag" \
  "/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/validation/105713/lidar_registration.bag" \
  "/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/validation/105713/comparison.bag"
```

## 보존·정리 기록

**OBSERVED:** 비교 도구의 기존 32개 파일은 READ 20 + EXCLUDED 12(기존 cache), 적용 후 33개는 READ 21 + EXCLUDED 12입니다. UNREAD 0. 이 범위는 전체 ROS 저장소 inventory를 대체하지 않습니다. [파일별 ledger](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/state.json).

**OBSERVED:** 운용 저장소 Git HEAD는 `eb24e08c9220f454080bd672bd8ce3ec0aeb7159`로 동일하며 작업 전후 변경 목록은 모두 비어 있습니다. 비교 도구의 상위 프로젝트는 유효 Git worktree가 없어 설치 전후 해시로 승인된 12개 파일 차이를 확인했습니다. 원본 bag 3개의 크기·mtime은 기존 기록과 동일하며 원본 bag 전체 해시 재대조는 수행하지 않았습니다.

시험용 임시 bag·검토 복사본·설치 임시 파일은 제거됐습니다. 코드 백업·패치·수정 bag/CSV·테스트 기록·검증 스크립트는 재현과 확인을 위한 최종 산출물로 보존합니다. ROS master·재생·실제 주행 프로세스는 시작하지 않았습니다. 개발 중 Python 3.8 호환성, IMU 누락 경계 검사, 완료 전 결과 검사 실패는 수정·재검증했으며 [상태 기록](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-tracking-fix-20260921/state.json)에 남겼습니다.
