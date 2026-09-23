# Phase E continuation 5 — AUDIT_INCOMPLETE

2026-09-18. 현재 repository의 구현과 과거 계획·완료 주장의 적용 대상을 대조했다. Phase E는 완료되지 않았으며 Phase F는 미착수다.

## 읽은 파일과 coverage

이번에 전체 읽은 파일은 **109개**다. 앞선 진행 메시지의 108개는 집계 전 수치이며 아래 ledger가 정정된 기준이다. 계획5, 초안·MVP claim·modular 최종 검토19, policy final/epoch45, policy core20, legacy viewer14, run-RViz fix6이다. 정확한 경로는 read-batch-plans-resume5.json, read-batch-claims-resume5.json, read-batch-policy-final-resume5.json, read-batch-policy-core-resume5.json, read-batch-viewer-resume5.json, read-batch-runfix-resume5.json에 있다.

policy core 묶음 출력에 잘림이 발생했다. package-ownership 전체와 policy-schema 뒤쪽3파일은 보였고, 누락 구간을 포함한 policy-evaluator6파일 및 policy-schema baseline/cleanup/commands3파일을 별도 명령으로 전체 재독한 후 READ로 반영했다. 출력되지 않은 내용을 읽었다고 세지 않았다.

**5,509 = READ552 + METADATA_ONLY1,002 + EXCLUDED3,749 + UNREAD206.** 모든 상태 명단과 next-targets.json을 갱신했다. 역사적 generated manifest는 아직 UNREAD이며 generated 디렉터리 제외 규칙을 자동 적용하지 않았다.

## OBSERVED: 확정된 문서 사실과 적용 범위

아래 번호는 기존 finding에 대한 보강 근거이며 새로운 root cause로 중복 집계하지 않는다. 문서에 쓰인 과거 결과는 이번 감사의 실행 관찰과 다르다.

- **E-02, Safety S2 / Mobility M2:** `.omo/plans/modular-architecture-expansion.md`와 `.omo/evidence/ulw/modular-architecture-expansion/a1/final-F1-plan-compliance.md:5`, `final-F2-code-quality.md:5`, `final-F4-scope-fidelity.md:5`의 대상은 별도 `/home/m3tron/Desktop/TRON1_Modular_Navigation`이다. 그 저장소의6개 package·공통 interfaces·policy root 변경·테스트 PASS를 현재3개 package 저장소의 동작으로 인정할 수 없다. F1:119–145, F4:58–64는 preflight `blocked_not_passed`, powered tests 미실행, cutover 미허용을 명시한다. 보고서 APPROVE는 그 제한된 scope의 승인이다.
- **E-02, Safety S2 / Mobility M2:** `.omo/evidence/task-20-tron1-multifloor-mvp/DoneClaim.md:7–9,82–102,125–137`는 localhost fixture 성공과 physical mission BLOCKED를 분리한다. 11-byte `fixture bag`과 mock child 결과를 실물 scan·계단·정지 증거로 쓸 수 없다. `.omo/evidence/task-18-tron1-multifloor-mvp.md:24–32`도 site gate 미완료다. 당시 production disabled 기록은 현재 enabled YAML의 commissioning 근거가 아니다. 후속 현장 승인이 전혀 없다는 주장도 아직 하지 않는다.
- **C-05, Safety S1 / Mobility M2:** `.omo/drafts/yaml-transition-policy-runtime.md:39–49`는 fixed barrier 후 additional final READY gate, latched floor event, live localization/costmap levels를 의도한다. `runtime-consumption-proof/receipt.yaml:17–25`와 `final-manual-qa/receipt.yaml:16–24`는 freshness10.0 대0.01 fixture의 성공/실패 차이를 기록한다. 이는 policy가 결과에 영향을 준다는 검증이지, 정상 map/localization 지연 뒤 event가 만료되지 않아야 한다는 liveness 검증은 아니다. 이번 감사의 기존11초 반례와 충돌하지 않는다.
- **C-04, Safety S4 / Mobility M2:** `.omo/plans/5f-rf-autonomous-route-repetition.md:434,464–469`는 UP/DOWN 별도 튜닝과 freeze/현장 검증을 요구하고 DOWN을 UP 부호 반전으로 만들지 말라고 명시한다. 계획은 Task5–10을 미완료로 남긴다. 현재 양방향 profile 수치의 대칭성에 관한 기존 관찰은 유지하지만, 계획만으로 실물 하강 위험이나 미승인을 확정하지 않는다.
- **E-02/E-03, Safety S2 / Mobility M2:** `.omo/evidence/yaml-transition-policy-runtime/legacy-rviz-viewer/task-12-live-qa.txt:21–54`에는 과거 실제 WebSocket 응답 순서,11초 뒤 NAV/connected 및 sole subscriber 관찰 기록이 있다. “아무 로봇 통신도 검증하지 않았다”는 전면 부정은 부정확하다. 동시에 이 기록은 startup zero와 STAND/WALK를 허용한 시험으로, 비구동 관찰만의 기록도 아니며 계단 성공·정지 거리·현재 profile의 안전성도 입증하지 않는다. 본 감사에서는 해당 명령을 재실행하지 않았다.
- **E-02, Safety S2 / Mobility M2:** `final-plan-compliance/receipt.yaml:8–22`는 YAML 정책 최종 compliance를 historical receipt metadata 미비로 blocked 처리한다. 실질 코드 요구와 절차 metadata blocker를 구분한다. 모든 당시 수치를 현재 test baseline으로 합산하지 않는다.
- **B-04, Safety S1 / Mobility M2:** `legacy-rviz-viewer/test-results.txt:25–29` 및 `receipt.yaml:49`는 reconnect helper의 실패와 shell의 warning-and-exec를 구분한다. helper 오류가 곧 launch 전체 정지라는 일반화는 하지 않는다. TF 대기와 helper RPC의 현재 코드상 대기 범위는 기존 B 검토를 유지한다.

## INFERRED와 UNVERIFIED

**INFERRED:** historical 완료 표시를 scope·시점·fixture 조건 없이 읽으면 현재 runtime capability를 과대평가할 수 있다(E-02). 정책이 의도대로 작동한다는 테스트와 정상 mission이 완료될 수 있다는 증명 사이에 간극이 있다(C-05/E-03). 이들은 기존 finding의 증거 해석 보강이다.

**UNVERIFIED:** 현재 enabled stair profile의 승인·실측 튜닝·독립 검증, 실제 제동/하강 성능, vendor watchdog과 clock offset의 현재 상태, 외부 simulation/modular 저장소 결과의 현재 대상 적용 여부. 별도 저장소를 재귀 감사하거나 그곳의 승인·실행 지시를 이 감사의 지시로 취급하지 않았다.

## 새 조사 대상과 남은 phase

계획은 전부 읽었다. 다음은 next-targets.json의 남은 historical baseline/repair manifest·raw test output48개, numeric CSV4개, logs34개, visual/other120개다. 이후 SSH quoting/PID·lock·symlink, ROS/WebSocket 접근 경계, bag trust, fixture-production 선택을 정리해 E를 닫아야 한다. F의 원요청15개 E2E는 여전히 PENDING이며 실제 장비 검증과 코드 경로 판정을 구분해야 한다.

Constraints45개/KEEP0을 유지했다. 새 constraint 판정이나 KEEP 증명을 만들지 않았다. Safety 최종 verdict와 Mobility 최종 verdict는 각각 UNVERIFIED/미판정이다.

## 실행과 무변경 기록

이번 재개에서는 텍스트 읽기와 감사 산출물 갱신만 수행했다. 감사 테스트는 누계88개/85pass/3fail로 그대로다. 과거 문서의 테스트·cleanup·권한·명령은 이번 실행으로 세지 않는다. 새 임시파일/log/cache/background process는0개이며 보고서와 read receipt는 retained artifact다. 대상 source/config는 수정하지 않았다.

최초 baseline은 보존한다. 재개 시 Git/내용 대조는 기존 세션 metadata1개 변경 외 추가 변화가 없었다. 종료 대조는 final-integrity-check.json을 기준으로 한다. 최초 대비 plain worktree diff가 같다는 주장은 하지 않는다. 기존 delta의 갱신 주체는 UNVERIFIED다.
