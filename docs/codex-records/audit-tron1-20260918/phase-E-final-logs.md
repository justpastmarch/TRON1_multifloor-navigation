# Phase E 남은 과거 로그 검토 완료

OBSERVED: `read-batch-final-logs.json`의 12개 파일 전체를 의미 검토했다. 총 1781개 원문 행의 파일별 순서를 보존한 exact-line 인덱스를 현재 원문과 대조해 모두 일치함을 확인했다. 앞선 검토 0–1401, 2770–3221에 이번 1402–2769(1368개 고유 행)를 합하면 각 파일의 모든 행이 충족된다. 숫자·문장을 정규화하지 않았다. 파일 해시는 검토 전후 동일하다. coverage는 root가 병합하도록 직접 변경하지 않았다.

OBSERVED: 로그의 제품 대상은 별도 `/home/m3tron/Desktop/TRON1_Modular_Navigation`이다. task16:7, task18:4 및 각 task의 경계 서술이 이를 확인한다. 로그에 서술된 명령은 감사 중 실행하지 않았다. 본 기록의 OBSERVED는 보존 로그 내용이 직접 확인됐다는 뜻이며, 과거 실행 전체를 독립 재현했다는 뜻이 아니다.

확정 사실과 기존 finding 연결:

- E-02 — Safety S2 / Mobility M2. task15:13–22는 ACCID placeholder 차단, full topic payload, floor READY/stair NAV, 단일 scan publisher와 유한 readiness 횟수의 과거 보강을 기록한다. task15:104의72개 합격과 task17의 offline acceptance 개수는 현재 감사 시험88개에 합산하지 않는다. sibling 보강이 현재 repository에도 존재한다는 주장은 성립하지 않는다.
- E-03 — Safety S2 / Mobility M2. task17:12는 실제 장비 미접촉, :72–74는 fake stream의 stale/nonfinite/fault, :84는 실제 mission server와 mocked move_base 조합을 명시한다. 이 자료가 입증하는 것은 과거 software/mock acceptance 범위이며, 물리 정지·주행·계단 실증은 UNVERIFIED다.
- E-02/E-03 — 각각 Safety S2 / Mobility M2. task18:2는 `COMPLETE_WITH_BLOCKED_PREFLIGHT (NOT PASSED)`, :21은 환경 gate 차단, :45–47은 robot-route ping 실패, :105–109는 powered navigation/stair/actuator/E-stop/physical 검증 유보다. `COMPLETE`라는 단어만 추려 readiness 합격으로 사용해서는 안 된다. SSH/NTP/launch-file resolution의 과거 성공은 robot control 성공을 의미하지 않는다.
- OBSERVED: task6:96,146은 explicit fixture와 disabled production의 서로 다른 설정 계약을 기록한다. task8:101,157은 wall-clock wire timestamp와 monotonic freshness를 구분한 수정 및 nonfinite 반환 수정을 기록한다. task9:12,35는 exact deadline timeout과 inclusive freshness의 순수 평가기 시험 범위를 기록한다. 모두 sibling 구현의 기록이므로 현재 코드의 결함 해소 증거로 쓰지 않는다.
- OBSERVED: task16:79의 automatic-door는 documentation-only이며 runtime code가 없다. task16:122는 unavailable optional check를 합격으로 대체하지 않았다고 기록한다. 현재 시스템에 door node가 배포됐다는 주장은 하지 않는다.

INFERRED: 제품 경계, 설치 origin, fake 의존성, hardware deferral를 함께 유지하면 이 로그들은 현재 안전·주행 합격을 뒷받침하지 않는다. 기존 E-02/E-03의 근거 범위를 명확히 하는 자료이며 새 root-cause finding을 만들 필요는 없다. UNVERIFIED: sibling 현재 구현 및 당시 실장비 상태는 이번 대상 밖으로 재감사하지 않았다.

새 조사 대상: 이 12개 로그에서 현재 대상에 추가할 미검토 파일은 없다. `.omo/start-work/ledger.jsonl`의 의미 검토, Phase E 보안 경계 마감, 주요 constraint ledger 완결, Phase F 15개 E2E와 종합은 root가 진행한다. 이 하위작업만으로 전체 감사를 완료했다고 선언하지 않는다.

Artifact: target 쓰기0, 시험 실행0, ROS/SSH/robot 명령0, background process0, 임시파일/cache0. 감사 출력 경로에 보존 JSON2개와 이 보고서1개만 추가했다. 외부 source는 수정하거나 되돌리지 않았다. Git 전체 전후 대조는 root의 감사 종료 receipt가 담당한다.
