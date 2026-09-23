# Phase E continuation6 — AUDIT_INCOMPLETE

2026-09-18. 완료 phase A–D의 정적 범위는 유지하며 E는 진행 중, F는 미착수다.

## 읽은 파일과 방법

추가 READ79개: historical JSON/baseline/repair/source/raw error41개(read-batch-resume6.json), numeric CSV4개(read-batch-csv-resume6.json), logs34개(read-batch-logs-resume6.json). 빈 stderr는 이전 READ였으며 중복 계수하지 않았다.

CSV는 전체50,618행의 모든 열을 파싱하고 수치 범위·finite·시간순서·배열·flag·구간 재시작·거리 계산을 분석했다. logs는 전체600,190행을 읽어550개 숫자/시간 정규화 template 전체를 검토하고 모든 parameter 값 변형, 주요 event 빈도·수치 극값·원문 줄 위치를 대조했다. 반복600,190행을 각각 화면에 출력한 것은 아니다. 원문 전체를 읽은 분석 결과와 template/원문 예제/위치가 log-full-index-resume6.json, log-template-review-resume6.json, log-event-analysis-resume6.json에 남아 있다. 단순 grep 적중으로 READ 처리하지 않았다.

과거 path/hash/mtime inventory7개는 모든2,974개 entry의 schema/count/중복/path/digest를 대조했지만 **METADATA_ONLY**로 분류했다(historical-manifest-metadata-resume6.json). 이 파일들이 가리키는 과거 source/generated/test payload를 읽었다거나 현재 실행과 같다고 주장하지 않는다. 본문·코드·시험 출력에는 이 분류를 적용하지 않았다.

Coverage: **5,509 = READ631 + METADATA_ONLY1,009 + EXCLUDED3,749 + UNREAD120**. 목록과 next-targets.json을 갱신했다.

## OBSERVED: 기존 finding의 근거 보강

- E-02, Safety S2 / Mobility M2: historical baseline의 catkin target exit0와 result summary exit1은 원문에서 구분돼 있다. baseline-provenance/repair-2-2026-08-24T10-12-00/xml-failures.stdout.txt에는9개 import error와1개 calibration CLI failure가 있고 manifest 합계150/9error/1failure와 일치한다. `status: complete`는 evidence 작성 완료이지 테스트 green이 아니다. 현재 감사88개 실행에 과거150개를 더하지 않는다.
- E-02/E-03, Safety S2 / Mobility M2: controller CSV1286행 모두 header_time0이며 controller_intent_only다. outbound WebSocket 명령의 실측 기록으로 사용할 수 없다. gap CSV232행은 시간 차와 duration이 일치하고 모두 rejected_no_interpolation이다. lidar1345행 중 accepted840, route usable755, segment597개다. wheel47755행은9개 구간이고236.8379797935초의1.919270560m jump가 invalid 처리된다. 이는 derived 데이터의 내부 일관성 확인이지 독립 물리 경로 검증이 아니다.
- B-04/D-01 및 E-02 보강: logs/20260917_010117/system.log:29부터 시작 자세를 얻지 못한다는 경고475562건, costmap TF timeout70518건, scan 부재5155건이 집계된다. 로그상 TF pose age 최대77326.5043초, scan 부재 메시지 값 최대77326.825992초다. 로그 출력의 시간값이며 실제 독립 장애 수나 wall-clock 연속성을 자동 입증하지 않는다. scan 없는 초기 메시지의 epoch 크기 값도 실제 수십 년 outage로 해석하지 않는다. D-01(Safety S2 / Mobility M1)과 B-04(Safety S1 / Mobility M2)의 historical 조사 근거로 연결하되 현재 재현 여부는 UNVERIFIED다.
- B-02, Safety S1 / Mobility M3: logs/20260916_224945/system.log:289, logs/20260917_010117/system.log:551297, logs/20260917_233902/system.log:40718에 watchdog budget 이후 twist send fault가 기록된다. source의 fault 경로와 부합하지만 실제 zero 전달·기체 정지는 이 로그만으로 확인되지 않는다.
- B-01/E-02, Safety S1 / Mobility M3:21개 로그에 apriltag zarray assertion이 기록되며16개 로그에는 RViz helper가 unknown /rviz_navigation 이후 navigation will continue라고 출력한다. 현재 decoder/library가 같은 상태인지 재실행하지 않았고 crash 원인을 tag family 하나로 단정하지 않는다. helper 오류를 전체 launch 정지로 일반화하지 않는다.
- E-02, Safety S2 / Mobility M2:34개 로그의 max_vel_x는0.55/0.35/0.5/0.4/0.3으로 달랐다. plugin inflation_radius0.55는31개,0.33은1개 기록이다. 과거 주행 자료를 현재 tuning 결과로 단정하지 않는다. 로그에 clipped 문자열로 출력된 parameter는 원문보다 확장해 해석하지 않는다.

## 분석 과정의 정정

첫 wheel normalization 검사는 전체 기록의 최초 좌표를 원점으로 가정해87252개 좌표 성분 차이를 산출했다. extractor:98–105를 다시 확인하니 구간마다 원점을 재설정한다. 구간별 재계산 mismatch0이고 edge policy mismatch0이다. 최초 차이는 **폐기한 감사 가설**이며 finding이 아니다. JSON에 정정 이유를 보존했다. nanosecond stamp의 float 요약도 정밀도 한계를 명시하고 정수 극값을 별도 계산했다.

LiDAR에서 accepted 직전 row가 rejected인7곳은 별도 segment를 시작한다. edge/usable flag만으로 이전 row의 수용 여부를 보장하지 않으나 segment ID가 경로를 끊으므로 이를 실제 경로 연결 결함으로 단정하지 않는다.

## 추론, 미검증, 다음 대상

INFERRED: historical stale TF/scan 및 transport fault는 B/D의 장애·복구 경로가 현실적 조사 대상이라는 보강 근거다. 로봇의 실제 정지·충돌 여부, 현재 동일현상, watchdog 실물 효력은 UNVERIFIED다. 여러 회의 반복 경고를 독립 실험 표본으로 세지 않는다.

UNREAD120개는 next-targets.json의 visual/other 그룹이다. 다음 시작은 task-2/rgb_nearest_0000.00s.jpg부터 개별 시각자료 검토, 이어 나머지 자료다. 보안 경계의 SSH quoting/PID·lock·symlink, ROS/WebSocket 접근, fixture-production 선택도 E에서 아직 정리해야 한다. 이후 F15개 E2E의 코드 경로 판정과 최종 통합이 필요하다. Constraints45/KEEP0, E2E15 PENDING, Safety와 Mobility 최종 verdict 각각 미판정이다.

## 실행·artifact·Git

이번 재개는 target read-only 자료 분석이다. 테스트 누계88/85pass/3fail 유지, live ROS/SSH/robot command 실행0. 새 임시파일/cache/background process0. 외부 감사 디렉터리의 분석 JSON은 retained artifact다. 최초 baseline 보존, 기존 session metadata1개 delta 외 추가 변경이 없었으며 최종 재검증은 final-integrity-check.json에 있다. 대상 source/config는 수정하지 않았다.
