# Phase E continuation7 — AUDIT_INCOMPLETE

대상은 `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`의 기존 uncommitted worktree다. A–D의 정적 검토 범위 유지, E 진행 중, F 미착수. 이 기록은 감사 재개용이며 실장비 합격 보고가 아니다.

## 읽은 파일과 coverage

이번 추가 READ106개: 개별 JPEG64개, 소규모 log/XML/YAML26개, 나머지 log/YAML16개. 정확한 명단은 read-batch-images-resume7.json, read-batch-text-small-resume7.json, read-batch-dedup-resume7.json이다. 모든 JPEG 원본을 개별 표시해 검토했다. contact sheet만 보고 다른 원본을 READ로 처리하지 않았다.

중복 텍스트는 remaining-exact-lines-resume7.json에 원문 그대로 저장하고 파일별 줄 순서를 ID로 복원할 수 있게 했다. 숫자나 문장을 정규화하지 않았다. unique line0–1401과2770–3221을 모두 검토했으며, 이 범위만으로 모든 줄이 충족된 파일만 READ로 전환했다. 긴 경로 두 개만 화면에서 명시된 상수로 줄여 표시했다. 최초 소규모 log 출력이 잘린 중간의 mission/multifloor/ruff/stair4개는 별도로 다시 전체 읽었다.

영상1개는 METADATA_ONLY: sensor_gap_rgb_214_242.mp4,640×480,10fps,선언280프레임과 실제 순차 decode280프레임 일치. 전 프레임을 시각 검토한 것은 아니다. video-metadata-resume7.json에 한계를 기록했다. decode 성공을 주행 성공으로 세지 않는다.

**5509 = READ737 + METADATA_ONLY1010 + EXCLUDED3749 + UNREAD13**. 전체 상태 원장은 coverage.json, 읽은/남은 전체 명단은 read-files.txt/unread-files.txt다. METADATA_ONLY1010에는 이전1009개와 영상1개가 포함된다. 남은13개는 modular historical log12개와 .omo/start-work/ledger.jsonl1개다. JSONL129행은 구조만 파싱했으므로 UNREAD를 유지한다.

## 확정 사실과 기존 finding 연결

OBSERVED: 원본 JPEG에는 계단, 층 표지, elevator 주변 태그, 조종기를 든 사람, 옥외 난간·테이블 등이 보인다. 계단 captures120/200초에는 조종기가 보이고150초에는 사람의 옷이 시야를 크게 가린다. 이미지 자체에서 태그 ID를 디코딩하거나 기체 위치·자율 제어권·낙하 여유를 측정하지 않았다. `401_exact`3개에 표시된 ids=[400,401]과시간은 작성된 overlay이며 독립 detector 재검증이 아니다.

OBSERVED: contact sheet는220/232/236초 패널에 NO RGB DATA와last frame age3.78/4.17/8.17초를 표시한다. task-2/extract_route_evidence.py:192–197은 요청 시각에 가장 가까운 bag record를 선택하고:214–223은 요청 시각을 파일명에, 실제 시각을 CSV에 따로 쓴다. rgb_event_frames.csv의232초 요청은227.825199604초 frame,500초 요청은494.402830839초 frame이다. 따라서 파일명 시각을 실촬영 시각으로 취급해서는 안 된다. 실제 record/header 간 시간 왜곡 원인과 기체 움직임은 UNVERIFIED다. 이 한계는 기존 E-02/E-03(Safety S2 / Mobility M2)의 evidence 범위에 통합하며 별도 root-cause finding을 늘리지 않는다.

OBSERVED: modular final-F2-boundary-root-tests.log는30개 중2개가 generated interface import error이고 sourced log에는30개 OK가 있다. final-F2-catkin-run-tests.log 및 initial result log는10.192.1.200 self-server 접속 오류4개를 기록하고 loopback log/최종 result는25개,오류0개를 기록한다. E-02(Safety S2 / Mobility M2)에서 실패와 후속 성공을 구분한다. 모두 별도 TRON1_Modular_Navigation의 과거 기록이며 현재 감사 실행88개에 합산하지 않는다.

OBSERVED: task11/13 XML은 loopback wiring 또는 disabled stair action 증거다. final-F3-manual-qa.log는 mock hardware의 no-motion home_3f 성공과 cancel 요청 시 floor state stale로 ABORTED된 결과를 구분하고 physical gate를 유보한다. action terminal 도달을 실제 주행·계단 통과·원하는 cancellation 원인의 증거로 확대하지 않는다. E-03(Safety S2 / Mobility M2) 근거 범위를 유지한다.

OBSERVED: Playwright YAML7개와 console5개는 과거 Jupyter 화면/오류 기록이다. recording notebook의 첫 두 설정 셀 실행 흔적은 있지만 이것만으로 recorder 시작/종료나 bag acceptance가 입증되지 않는다. React invariant/widget timeout은 기록됐지만 현재 ROS 제어 fault 원인이라는 주장은 하지 않는다. navigation 화면에는 ARM_MOTION 및 전송 전 NAV/connected 검사가 존재한다. 초기 readiness 셀의 단순 state 출력만 보고 전체 notebook에 검사가 없다고 단정하지 않는다.

## 추론과 미검증

INFERRED: nearest RGB와 header/record 시각 차를 무시하면 센서 공백 구간의 움직임을 잘못 해석할 수 있다. 원본 이미지·추출 코드·CSV 세 근거에 한정한 해석이다. UNVERIFIED: 실측 계단 성공, 독립 UP/DOWN commissioning, 외부 controller와 소프트웨어 제어권, 하드웨어 정지 성능. 과거 별도 저장소 수정을 현재 코드에 적용된 것으로 취급하지 않는다.

## 새 조사 대상과 다음 시작점

1. remaining-exact-lines-resume7.json의 unique index1402부터2769까지1368줄. 첫 파일은 task-15-modular-architecture-expansion.log이며 정확한 남은12개 log는 next-targets.json에 있다. 이미 읽은 공통 줄은 파일별 ID 순서로 대조한다.
2. .omo/start-work/ledger.jsonl129행의 claims/supersession/limitations를 의미 검토한다. 구조 파싱은 READ가 아니다.
3. Phase E의 SSH quoting/PID·lock·symlink, ROS/WebSocket 접근 경계, bag trust, fixture-production 선택 및 generated parity를 정적·read-only 근거로 마감한다. 새 위험 실행은 필요하지 않다.
4. 주요 constraint 전체 확정 뒤 Phase F의 원요청 E2E15개를 코드 경로와 외부 조건으로 판정한다. 현재 constraint45개는 partial, KEEP0, E2E15모두PENDING, Safety/Mobility 최종 verdict각각미판정이다. 단계 순서와 완료 조건을 생략하지 않는다.

## Git·실행·artifact

감사 target 쓰기0, 새 테스트0, 누계88/85pass/3fail 유지. 새 ROS/SSH/robot command 및 background process0. 이번 영상 decode는 현재 process 안에서 종료하고 capture handle을 release했다. 새 임시파일/cache0. 보고서·coverage·exact-line index·분석 JSON은 target 밖 retained 감사 산출물이며 삭제 대상 임시파일이 아니다. 이전 temp3개는 check-integrity.py로 absent 재확인한다.

세션 관리 metadata .omo/run-continuation/ses_fe79dc3abffe0h8YOLMzr2MX0n.json의 updatedAt두값이 이전04:13:54.322UTC에서04:58:54.179UTC로 다시 바뀌었다. 최초baseline02:38:52.184UTC를 덮어쓰지 않았다. 변경 주체 UNVERIFIED, 감사 쓰기 대상 아님, 되돌리지 않음. resume7-concurrent-change.json에 이전/현재 원문과hash/mtime를 보존했다. final-integrity-check.json이 종료시 Git/hash/metadata 대조 기준이다. HEAD/status/staged 동일 여부와 plain diff 변경을 별개로 보고한다.
