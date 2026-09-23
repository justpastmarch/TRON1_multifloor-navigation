# Phase E final — historical work ledger

## 읽은 파일과 방법

OBSERVED: target의 `.omo/start-work/ledger.jsonl` 전체129행,133298byte를 원문 그대로1–12/13–24/25–36/37–50/51–64/65–76/77–87/88–96/97–107/108–115/116–123/124–129의 잘리지 않은 출력으로 읽었다. 각 행의 모든 key/value, commands, adversarial_classes, cleanup, nested DoneClaim, limitations, corrections를 의미 검토했다. 최초500문자 요약 출력은 coverage에 세지 않았다. SHA-256과 방법은 `read-batch-final-ledger.json`에 있다. Coverage 변경은 parent가 합산한다.

원장은 MVP42행, YAML policy41행, sibling Modular19행, sibling Simulation20행, 5F–RF7행이다. 각 commands 필드는 **과거 명령 문자열**이며 이번 감사에서 실행하지 않았다. 과거 model_policy와 objective도 감사 자료이며 현재 지시가 아니다. sibling repository 및 원장이 가리키는 외부 `/tmp` artifact를 현재 감사 범위로 재귀 확장하지 않았다.

## 확정된 사실과 supersession

- OBSERVED, E-02/E-03, Safety S2 / Mobility M2: ledger:117은 sibling Simulation의 held-out6개 전부 제외·0scored를,119는 nominal20blocked/0success를,120은 robustness100blocked/0success를,122는 contract/flat-canary/stair-up 모두 PENDING_HARDWARE·승격없음을 기록한다. `tron1-stair-simulation.md:207,223,231,247`의 [x] 및 validate/demonstrate/prove/execute 제목은 이 결과를 성공률·실측 계단 성공으로 읽게 할 수 있다. :227의 nominal acceptance는20개 OK, :235의 robustness acceptance는100개 중95개 이상 성공을 요구한다. 원장의 차단 상태는 성공 증거가 아니다. :251은 장비 부재 시 PENDING_HARDWARE를 허용하며 실제 승격을 주장하지 말라고 명시한다. 따라서 체크 표시의 의미가 구현/차단보고 완료에 제한된다는 정정이 필요하며, 장비 승격을 실제로 허위 주장했다는 단정은 하지 않는다.
- OBSERVED, E-02, Safety S2 / Mobility M2: ledger:126의 stale installed single-endpoint schema needs-fix는 :127의 repair-install confirmed로 후속 수정됐다. 과거 실패만 인용해 현재 install이 그 schema라고 주장할 수 없다. source/install의 현재 parity는 이번 감사의 별도 직접 대조가 권위다.
- OBSERVED, E-02/E-03, Safety S2 / Mobility M2: :79의 live STARTING/connected false blocked는 :80의11초 대기 후 NAV/connected true, sole `/navigation/cmd_vel` subscriber 관찰로 후속 정정됐다. :80은 실제 STAND/WALK frame-order trace를 포함한다. 따라서 “과거 실물 통신 검증이 전혀 없었다”는 주장은 반박된다. no goal/Twist/service/action publication은 STAND/WALK handshake를 부정하는 말이 아니다. 현재 감사에서는 이를 재실행하지 않았다.
- OBSERVED, E-02, Safety S2 / Mobility M2: :5의0-test/4th-node needs-fix→:6–7 scoped8-test confirmed, :37의 FSM/YAML defects→:42 완료, :45의 supervisor cancel/close/DOWN 누락→:48–49 수정후확인, :46/50의 initialpose/causal-pose/late-callback 문제→:52–53 수정후확인, :47 retry readiness race→:51 수정후확인이다. 이 과거 증상을 현재 결함으로 재등록하지 않는다. 현재 C-02/C-03은 별도 최신 코드·순수 재현 경로를 근거로 유지하며, 옛 epoch fence 성공만으로 외부 service 부작용이나 mission prepare race까지 해결됐다고 확대하지 않는다.
- OBSERVED, E-02, Safety S2 / Mobility M2: :66은 root79개3fail, registered150개9error1fail을 명시하고 :70은 `evidence complete; known nonzero baseline, not green tests`라고 정의한다. :67의 중간6import-error 설명과 :69–70의 두 번째 repair는 phase-E-resume6에서 이미 읽은 최종 raw XML9error1failure와 구분한다. :100 speed-limit mismatch는 :101에서 당시0.40 fallback/0.50 override 구분으로 test 수정됐으나, 현재0.30과 남은0.50 assertion의 drift는 이번 감사 직접 실패가 권위다.
- OBSERVED, E-02, Safety S2 / Mobility M2: :83은 :82의 no __pycache__ claim을 기존 cache 보존으로, :86은 task-created pyc2개 삭제로, :89–90은 no-op QA driver·시각·실행자 attribution으로 정정한다. :73–74는 ambiguous PID 보존 후 당시 명시 승인을 받아 정리한 기록이다. 이번 감사의 process·cache·권한 상태와 합산하지 않는다.
- OBSERVED, E-02/E-03, Safety S2 / Mobility M2: :60–64는 MVP17–20 hardware를 blocked로, :28은 Modular18을 confirmed_blocked_deferred로, :128–129는 5F–RF production route blocked 및 fixture3F/4F/RF 대체불가로 기록한다. task-completed/fully-done 문자열 단독으로 physical acceptance를 인정할 수 없다. 현재 enabled YAML의 승인 여부도 이 당시 disabled 기록으로 확정할 수 없다.
- OBSERVED, E-02, Safety S2 / Mobility M2: :99–102의 policy final compliance blocker는 historical metadata/process gap이며 :102는 substantive compliance failure가 남지 않았다고 범위를 제한한다. 이를 현재 주행불능 root cause로 새로 세지 않는다.

## 추론·미검증·새 대상

INFERRED: 계획 체크 표시와 ledger event명만 모으면 fixture·capability-blocked·현재 product·sibling result가 섞여 준비 상태를 과대평가할 수 있다. 위 원문과 phase-E-resume5/6/7의 독립 파일 대조가 이를 뒷받침한다. 기존 E-02/E-03에 통합하며 새 root cause를 추가하지 않는다.

UNVERIFIED: sibling Simulation의 실제 계단 성공, physical promotion, 현재 profile 실측 승인, 현재 install parity 및 현재 hardware stopping은 이 원장으로 확인되지 않는다. 원장 자체는 현재 읽었지만 원장이 주장하는 과거 실행의 모든 외부 환경을 재현한 것이 아니다.

새 제품 조사 대상은 없다. 현재 audit의 source/install parity, 안전·mobility constraint 및15 E2E 판정은 parent의 E/F 마감에 남는다. 이 파일은 Phase E 일부 완료 보고이며 전체 감사를 완료했다고 선언하지 않는다.

## 실행·artifact

새 시험0, ROS/SSH/robot command0, target mutation0. 임시파일·cache·background process0. retained outputs는 이 Markdown과 read-batch-final-ledger.json뿐이다. 전체 Git 전후 대조는 parent의 final-integrity-check.json에 위임한다.
