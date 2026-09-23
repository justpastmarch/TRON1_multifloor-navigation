# 현재 구현과 실주행 상태

2026-09-23 · GitHub 업로드용 개발 상태 정리

현재 저장소에는 LiDAR 계단 제어, 실행·RViz·기록 도구, 시작 측위와 수동 위치 지정, 기존 mission/floor 연계 변경이 반영되어 있다. **실제 3→4층 자동 완주는 미완료**다. 최신 BAG에서 자동 상승이 경계 제약으로 중단되어 RC로 나머지를 주행했다. 이후 개선안은 문서화했으며 아직 제어기에 적용하지 않았다.

## 반영된 최근 변경

| 항목 | 현재 내용 | 근거 |
|---|---|---|
| LiDAR 기준과 기존 Supervisor 통합 | 기존 gyro/GICP 추적, body 변환, connected support 검사, 명령 소유권·phase test 연계 | [통합 기록](codex-records/stair-supervisor-implementation-20260921/REPORT.md) |
| 시작 측위 | 자동 후보 탐색과 RViz 수동 지정, 현재 위치 기준 NAV 연결 | [검증·적용 기록](codex-records/localization-startup-20260921/RESULT.md) |
| 진입 준비 | 신선한 LiDAR를 기다리고 진입 기준을 명시적으로 잡는 preview/run | [freshness 수정](codex-records/stair-entry-freshness-20260923/REPORT.md) |
| 진입 판정·진단 | NumPy 값 JSON 직렬화, VERIFY_ENTRY/ALIGN의 제한된 움직임 판정 개선 | [진입 시간 초과 수정](codex-records/stair-entry-timeout-20260923/REPORT.md) |
| 상승 명령 | 두 상승 구간 목표 normalized x=1.0. 가속·지지영역 제한은 남음 | [전진 입력 변경](codex-records/stair-full-input-20260923/REPORT.md) |
| 계단참 속도 | hold_v=0.06, max_w=0.2 유지. 상승 명령 이력이 평지 구간 상한을 넘지 않게 제한 | [계단참 보완](codex-records/stair-full-input-20260923/landing-preserve/REPORT.md) |
| 중간 구간 시험 | --reuse-reference로 기존 좌표 유지, --single-phase로 두 번째 상승 단독 시험 | [단독 시험 변경](codex-records/stair-second-flight-20260923/REPORT.md) |

선택된 현장 시험 설정은 `src/stair_supervisor/config/stair_lidar_3f_4f_test.yaml`이다. 일반 관측 파일 `stair_lidar.yaml`의 configured=false와 혼동하지 않는다. 현장 설정의 configured=true는 파일 사용 준비 상태이며 commissioned=false는 정상 층간 미션 승인이 아직 없다는 뜻이다. 9월 23일 10시 이후 실행에서는 자동 BAG를 끄고 사용자가 별도로 녹화했다. config.env의 기본값과 개별 세션의 환경 재정의는 다를 수 있다.

## 최신 주행 결과와 미구현 개선안

[두 상승 구간 정지 원인·수치·개선안](codex-records/stair-two-flight-stop-analysis-20260923/REPORT.md)을 기준으로 후속 작업을 진행한다.

- OBSERVED: 10:28:13, 10:38:07 주요 정지는 도착이나 시간 초과가 아니라 predicted support 검사 실패다. 10:35:21 초입 정지는 current support margin 검사 실패다. 직전 TX 전진 입력은 1.0이었다.
- INFERRED: 급격한 yaw 변화에 대한 조향 반응, 12cm 여유, 감속 대안 제한과 STAIR 명령→운동 모델의 결합이 중단을 만든다. 고정 자세의 오프라인 비교는 실제 회복 성공률이 아니다.
- UNVERIFIED: 물리 경계/진입 오차, 실제 조향 응답, 남은 단수와 높이 잔차의 원인, 무인 완주·정상 floor handoff.
- OBSERVED: 이번 BAG에는 /tron/sensor_joy가 없다. 수신기 실행과 실제 데이터 수신 확인을 다음 녹화 절차에 보완해야 한다. 아직 구현하지 않은 상태다.

후속 변경은 기존 제어기 안에서 조향/회복 후보·시각 정렬·실측 운동 모델·중앙 정렬과 도착 평면 검증을 다룬다. 이 계획을 이번 커밋의 구현 완료 기능으로 표시하지 않는다.

## 검증 기록의 해석

최신 관련 회귀 95개가 통과했다. 과거 각 작업의 테스트 수는 그 작업 당시 범위이며 합산해 현재 전체 테스트 수로 부풀리지 않는다. 실주행은 두 구간 모두 수동 개입이 필요했다. 시험 종료의 0 명령과 STAIR 유지는 물리적 자세 유지나 정상 층 도착 성공을 뜻하지 않는다.

[전체 CODEX 기록 색인](codex-records/README.md) · [이전 HTML의 Markdown·미디어 묶음](tron-documentation-20260922/README.md)
