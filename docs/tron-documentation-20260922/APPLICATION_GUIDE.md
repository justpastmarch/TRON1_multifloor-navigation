# 최신 적용 안내

2026-09-23. 현재 구현과 실주행 결과는 [현재 개발 상태](../development-status-20260923.md)를 기준으로 확인한다. 아래 보관 보고서는 당시 기록이며 현재 자동 완주나 실기 commissioning 완료를 뜻하지 않는다.

- [오늘까지 반영한 코드 변경과 최신 주행 상태](../development-status-20260923.md)
- [두 상승 구간 정지 원인과 미구현 개선안](../codex-records/stair-two-flight-stop-analysis-20260923/REPORT.md)
- [CODEX 전체 개발 기록 색인](../codex-records/README.md)
- [기존 실행 절차](../stair-drive-readiness-20260923/RUNBOOK.md)
- [18개 HTML의 Markdown 보관본과 영상](README.md)

현장 시험은 configured=true/control 설정을 사용하지만 경로는 commissioned=false다. 두 상승 구간 목표 입력 1.0, 계단참 기존 속도 유지, 두 번째 상승 단독 시험 클라이언트가 반영됐다. 실제 자동 상승은 경계 검사로 중단되어 수동 개입이 있었고, 조향·경계 회복 개선안은 아직 적용하지 않았다. 0 명령은 물리적 자세 유지를 보장하지 않는다.
