# TRON 문서 보관함

2026-09-22 · HTML 18개를 Markdown으로 보관

**현재 구현 상태는 [현재 개발 상태](../development-status-20260923.md)와 [최신 적용 안내](APPLICATION_GUIDE.md)에서 확인합니다.** 아래 18개 변환본은 당시 기록이며 현재 실기 완료를 뜻하지 않습니다.

보고서 본문·표를 보존했고, 그림·도표·MP4를 `assets/`에 함께 저장했습니다. 폴더 전체를 복사하면 문서 사이 링크와 포함된 미디어를 사용할 수 있습니다. 원본 BAG·코드·외부 참고 자료 링크는 원래 위치를 가리키며 이 문서 묶음에 복제하지 않았습니다.

## 개발 문서

| 원본 문서 | Markdown 보관본 |
|---|---|
| stair-driving-repair-plan-20260922/report.html | [TRON · 3→4층 계단 주행 복구 계획](documents/stair-driving-repair-plan-20260922/report.md) |
| stair-lidar-commissioning-20260921/offline-validation/report.html | [계단 제어 오프라인 검증](documents/stair-lidar-commissioning-20260921/offline-validation/report.md) |
| stair-tracking-baseline-20260921/report.html | [계단 LiDAR 기준 버전 · 수치와 도착 표식 수정](documents/stair-tracking-baseline-20260921/report.md) |
| stair-supervisor-implementation-20260921/report.html | [Stair Supervisor 구현 결과](documents/stair-supervisor-implementation-20260921/report.md) |
| lidar-method-explained-20260921/report.html | [TRON LiDAR 개선 원리와 발견 과정](documents/lidar-method-explained-20260921/report.md) |
| lidar-before-after-video-20260921/report.html | [LiDAR 비교 영상 3개](documents/lidar-before-after-video-20260921/report.md) |
| audit-tron1-20260918/FINAL-AUDIT.html | [TRON1 · 최종 시스템 감사](documents/audit-tron1-20260918/FINAL-AUDIT.md) |
| sensor-recording-analysis-20260918/METHODS.html | [TRON1 · 분석 방법](documents/sensor-recording-analysis-20260918/METHODS.md) |
| sensor-recording-analysis-20260918/bag-index.html | [TRON1 · bag 근거](documents/sensor-recording-analysis-20260918/bag-index.md) |
| sensor-recording-analysis-20260918/report.html | [TRON1 · 기록 센서 분석](documents/sensor-recording-analysis-20260918/report.md) |
| stair-centering-analysis-20260921/bag-results.html | [TRON1 · 계단 측위와 중앙 유지](documents/stair-centering-analysis-20260921/bag-results.md) |
| stair-centering-analysis-20260921/method.html | [TRON1 · 계단 측위와 중앙 유지](documents/stair-centering-analysis-20260921/method.md) |
| stair-centering-analysis-20260921/report.html | [TRON1 · 계단 측위와 중앙 유지](documents/stair-centering-analysis-20260921/report.md) |
| stair-supervisor-integration-plan-20260921/report.html | [LiDAR 측위와 Stair Supervisor 구현 계획](documents/stair-supervisor-integration-plan-20260921/report.md) |
| stair-tracking-fix-20260921/report.html | [계단 추적 1·2 수정 적용 결과](documents/stair-tracking-fix-20260921/report.md) |
| stair-tracking-repair-20260921/report.html | [TRON1 계단 회전 추적 수정 검증](documents/stair-tracking-repair-20260921/report.md) |

## 보류한 학술제 참고 자료

학술제 검토는 보류 상태지만 요청한 전체 HTML 보관 범위에 포함했습니다.

| 원본 문서 | Markdown 보관본 |
|---|---|
| research-topic-selection-20260921/report.html | [TRON · 학술제 연구 주제 선정](documents/research-topic-selection-20260921/report.md) |
| undergraduate-research-awards-20260921/report.html | [국내 학부 연구 수상작 37건 조사](documents/undergraduate-research-awards-20260921/report.md) |

## 보존·변환 검증

- 원본 HTML 18개와 보관 Markdown 18개를 1:1 연결했습니다.
- 본문·목록·표 셀·제목 4,804개 텍스트 블록을 변환 결과와 대조해 누락 0개를 확인했습니다. 이는 내용 전체의 재감사나 최신 사실 검증이 아닙니다.
- 영상 3개를 포함한 미디어 42개를 저장했습니다. 같은 파일은 내용 해시로 중복 저장을 피했습니다.
- 검색·접기·재생 위치 이동 같은 HTML 조작 기능은 정적 문서와 영상 링크·시각 설명으로 변환했습니다.
- 원본 HTML은 변경하지 않았습니다. 출처·SHA-256·변환 목록은 [manifest.json](manifest.json)에 있습니다.
- 과거 보고서의 단정·설정·수치는 당시 기록을 보존했습니다. 최신 상태와 충돌할 경우 적용 안내와 실제 검증 결과를 우선 확인합니다.

## 묶음 구성

- `documents/`: 18개 Markdown 보관본
- `assets/`: 이미지·SVG·MP4
- `APPLICATION_GUIDE.md`: 예상 결과, 현재 상태, 적용 절차
- `manifest.json`: 원본/문서/미디어 목록과 검증
- `archive_reports.py`: 변환 재현 도구
- `verify_archive.py`: CODEX 원본 폴더 배치에서 사용하는 과거 변환 검증 도구. 저장소 복사본 해시 검증에는 [새 검증 도구](../codex-records/verify_repository_archive.py)를 사용한다.
- [VALIDATION.md](VALIDATION.md): 보존 검증, 로컬 검사 결과, 변경·임시 자료 정리 기록
