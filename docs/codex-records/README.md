# CODEX 개발 기록 보관함

2026-09-23. 실행 코드의 기준은 저장소 `src/`, `config.env`, 기존 실행 스크립트다. 이 폴더의 문서·패치·코드 텍스트는 당시 분석과 검증 기록이며 실행 경로가 아니다.

- [현재 구현·실주행 상태](../development-status-20260923.md)

## 기록 색인

| 작업 | 문서 |
|---|---|
| audit-tron1-20260918 | [FINAL-AUDIT.md](audit-tron1-20260918/FINAL-AUDIT.md) · [CHECKPOINT.md](audit-tron1-20260918/CHECKPOINT.md) · [architecture-final.md](audit-tron1-20260918/architecture-final.md) · [audit-commands-final.md](audit-tron1-20260918/audit-commands-final.md) |
| lidar-before-after-video-20260921 | [REPORT.md](lidar-before-after-video-20260921/REPORT.md) |
| lidar-method-explained-20260921 | [REPORT.md](lidar-method-explained-20260921/REPORT.md) |
| localization-startup-20260921 | [RESULT.md](localization-startup-20260921/RESULT.md) · [WORK.md](localization-startup-20260921/WORK.md) |
| research-topic-selection-20260921 | [REPORT.md](research-topic-selection-20260921/REPORT.md) · [WORKLOG.md](research-topic-selection-20260921/WORKLOG.md) · [VALIDATION.md](research-topic-selection-20260921/VALIDATION.md) |
| rviz-nav-test-20260921 | [기록 폴더](rviz-nav-test-20260921/) |
| sensor-recording-analysis-20260918 | [REPORT.md](sensor-recording-analysis-20260918/REPORT.md) · [METHODS.md](sensor-recording-analysis-20260918/METHODS.md) |
| stair-centering-analysis-20260921 | [REPORT.md](stair-centering-analysis-20260921/REPORT.md) · [BAG-RESULTS.md](stair-centering-analysis-20260921/BAG-RESULTS.md) · [METHOD.md](stair-centering-analysis-20260921/METHOD.md) · [QA.md](stair-centering-analysis-20260921/QA.md) |
| stair-drive-readiness-20260923 | [WORKLOG.md](stair-drive-readiness-20260923/WORKLOG.md) |
| stair-driving-repair-implementation-20260922 | [WORKLOG.md](stair-driving-repair-implementation-20260922/WORKLOG.md) · [REVIEW.md](stair-driving-repair-implementation-20260922/REVIEW.md) |
| stair-driving-repair-plan-20260922 | [PLAN.md](stair-driving-repair-plan-20260922/PLAN.md) · [WORKLOG.md](stair-driving-repair-plan-20260922/WORKLOG.md) · [VALIDATION.md](stair-driving-repair-plan-20260922/VALIDATION.md) |
| stair-entry-freshness-20260923 | [REPORT.md](stair-entry-freshness-20260923/REPORT.md) |
| stair-entry-retry-20260923 | [기록 폴더](stair-entry-retry-20260923/) |
| stair-entry-timeout-20260923 | [REPORT.md](stair-entry-timeout-20260923/REPORT.md) |
| stair-forward-command-20260923 | [기록 폴더](stair-forward-command-20260923/) |
| stair-full-input-20260923 | [REPORT.md](stair-full-input-20260923/REPORT.md) · [REPORT.md](stair-full-input-20260923/landing-preserve/REPORT.md) |
| stair-geometry-recovery-20260923 | [ANALYSIS.md](stair-geometry-recovery-20260923/ANALYSIS.md) |
| stair-lidar-commissioning-20260921 | [REPORT.md](stair-lidar-commissioning-20260921/offline-validation/REPORT.md) · [entry-drift-check.md](stair-lidar-commissioning-20260921/entry-drift-check.md) · [scope.md](stair-lidar-commissioning-20260921/offline-validation/scope.md) · [stair-lidar-integration.md](stair-lidar-commissioning-20260921/phase-test-change/docs/stair-lidar-integration.md) |
| stair-live-check-20260921 | [LIVE-CHECK.md](stair-live-check-20260921/session-20260920T231114Z/LIVE-CHECK.md) |
| stair-live-check-20260923 | [WORKLOG.md](stair-live-check-20260923/WORKLOG.md) · [IDLE_LOCALIZATION.md](stair-live-check-20260923/IDLE_LOCALIZATION.md) · [LIVE_READINESS.md](stair-live-check-20260923/LIVE_READINESS.md) |
| stair-live-readiness-20260923 | [WORKLOG.md](stair-live-readiness-20260923/WORKLOG.md) |
| stair-rviz-preview-20260921 | [기록 폴더](stair-rviz-preview-20260921/) |
| stair-second-flight-20260923 | [REPORT.md](stair-second-flight-20260923/REPORT.md) |
| stair-supervisor-implementation-20260921 | [REPORT.md](stair-supervisor-implementation-20260921/REPORT.md) |
| stair-supervisor-integration-plan-20260921 | [PLAN.md](stair-supervisor-integration-plan-20260921/PLAN.md) |
| stair-tracking-baseline-20260921 | [REPORT.md](stair-tracking-baseline-20260921/REPORT.md) |
| stair-tracking-fix-20260921 | [REPORT.md](stair-tracking-fix-20260921/REPORT.md) |
| stair-tracking-repair-20260921 | [REPORT.md](stair-tracking-repair-20260921/REPORT.md) |
| stair-two-flight-stop-analysis-20260923 | [REPORT.md](stair-two-flight-stop-analysis-20260923/REPORT.md) |
| tron1-web-mission-plan | [IMPLEMENTATION-PLAN-DRAFT.md](tron1-web-mission-plan/IMPLEMENTATION-PLAN-DRAFT.md) · [IMPLEMENTATION-SPEC-20260920.md](tron1-web-mission-plan/IMPLEMENTATION-SPEC-20260920.md) |
| undergraduate-research-awards-20260921 | [REPORT.md](undergraduate-research-awards-20260921/REPORT.md) · [analysis.md](undergraduate-research-awards-20260921/analysis.md) |

## 보존 범위

보고서·분석 결과·테스트 결과·변경 패치와 참조 미디어를 보관했다. 원본 BAG, 대용량 센서 배열/전체 trace, 실행 중 로그·캐시, before/candidate 사본은 로컬에 보존하며 파일별 사유는 [manifest.json](manifest.json)에 있다. 기존 9월 22일 문서 묶음과 같은 미디어는 그 파일을 참조한다. 연구 주제 검토는 보류된 과거 참고 자료다.

`.py.txt`는 실행 배포와 혼동하지 않도록 텍스트로 보존한 분석 코드다. 재현 시 원본 설치 경로·데이터 의존성을 확인해야 한다. 로컬 절대 경로나 외부 데이터 링크는 원본 위치의 provenance이며 다른 컴퓨터에서 자동으로 사용할 수 있다는 뜻이 아니다.

## 현재 남은 작업

계단 경계 검사·조향 회복 개선안은 **분석 완료, 미구현**이다. 주행 입력 1.0, 계단참 기존 속도 유지, 두 번째 상승 단독 시험 클라이언트는 **반영 완료**다. `configured=true`와 `commissioned=false`를 구분하며 자동 층간 완주 성공을 주장하지 않는다.

## 저장소 파일 검증

저장소 루트에서 `python3 docs/codex-records/verify_repository_archive.py`를 실행하면 복사한 기록 631개와 기존 변환 문서·미디어의 해시를 검사한다. 원본 데이터·링크 유효성·실기 동작 검증은 포함하지 않는다. 기존 `tron-documentation-20260922/verify_archive.py`는 CODEX 원본 폴더 배치를 전제로 하는 과거 변환 검증 도구다.
