# Phase E 네 번째 재개 — AUDIT_INCOMPLETE

감사 대상은 `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`의 현재 worktree다. 이 기록과 실행 receipt는 대상 밖 감사 디렉터리에만 썼다. E는 진행 중이며 F는 미착수다.

## 읽은 범위와 확인한 계약

65개 추가: 남아 있던 `src/` 및 `test/`의 테스트·RViz 37개와 5F/RF commissioning evidence 28개. 정확한 목록은 `read-batch-resume4.json`, `read-batch-evidence-resume4.json`이다. 전체 text 출력의 truncation 없이 검토했다. contact sheet와 경로 그림은 직접 시각 검토했지만 별도 원본 RGB 28장을 읽었다고 계산하지 않았다.

OBSERVED: floor callback epoch 테스트는 terminal FAULT 후 늦게 완료되는 tag/map/pose/costmap callback과 이전 policy writer를 검증한다(`src/multifloor_manager/test/test_floor_transition_epoch_ros.py`). 이는 내부 callback의 epoch 격리 검증이며 C-10의 외부 stair ownership epoch 검증과 다르다. `test_floor_transition_ros.py:189,271,288`의 goal은 여전히 stair ownership epoch를 지정하지 않는다. 이번에는 ROS 테스트를 실행하지 않았다.

OBSERVED: managed RViz에는 direct goal/initialpose 도구가 없고 manual RViz에는 있다. 이것은 UI 설정 계약이다. ROS command surface 자체의 접근 통제 보증으로 승격하지 않는다. 4개 RViz YAML 전체를 검토했다.

## E-02 보강 — 서로 다른 배포 계약을 요구하는 테스트

Safety: S2 / Mobility: M2. 별도 finding을 늘리지 않고 기존 문서·검증 계약 갱신 누락 root cause에 통합한다.

명령: `python3 -B audit-tron1-20260918/selected-contracts-resume4.py` (감사 workspace 기준).
관찰 시각: 2026-09-18T04:26:19.317393 UTC.
관찰 결과: 12개 실행, 10 pass / 2 fail / 0 error. network connect/bind, subprocess, test 파일 쓰기를 guard로 금지했다. 결과 원문은 `selected-contracts-resume4.json`.

- OBSERVED: `test/test_legacy_rviz_viewer_contract.py:103`은 robot max_vel_x=0.50을 요구하지만 현재 값은 0.30이다. 같은 실행의 launch speed contract는 0.30으로 통과했다.
- OBSERVED: `test/test_interface_contract.py:120`의 완전한 wire contract assertion은 현재 `src/stair_supervisor/action/StairTraversal.action:5,11`의 admission_token과 ENTRY_REJECTED를 누락해 실패했다. FloorTransition epoch 필드의 존재를 검사하는 assertion은 통과했다. 필드 존재 검사가 실제 authorization을 입증하지 않는 점은 C-10과 연결된다.
- INFERRED: 이전 전체 테스트 통과 기록만으로 현재 배포 계약이 일치한다고 신뢰하면 변경된 admission 또는 운용 속도를 잘못 판단할 수 있다. 이 결과만으로 현재 실장비 wire mismatch가 발생한다고 단정하지 않는다.

최소 조치: 현재 승인된 action/config 계약에 assertion과 운용 문서를 함께 맞추고, 실제 generated interface provenance는 별도로 비교한다. 테스트를 맞추기 위해 속도를 과거 값으로 올리거나 admission 필드를 제거할 이유는 없다. Required test: source/generated interface 일치, 올바른 admission 전달·거부, 배포 설정과 문서 예시 일치.

감사 누계는 88개 실행 / 85 pass / 3 fail이다. 이전 실패 1개는 `test_system_launch_includes_standard_navigation_and_perception`의 삭제된 scan launch 기대값이다. historical DoneClaim의 수백 개 pass, 순수 반례 probe, 0개 실행한 loader 시도는 이 누계에 더하지 않았다.

## E-03 보강 — 성공하는 mock이 입증하는 범위

Safety: S2 / Mobility: M2. OBSERVED: `src/stair_supervisor/test/test_stair_supervisor_node.py:129,167-181`은 AllowStairAdmissionValidator와 feedback phase에 따라 만드는 센서를 사용한다. `test_stair_fake_websocket_ros.py:79-89`도 같은 인과 방향이다. action/feedback/transport serialization 테스트로는 유효하나 실제 독립 센서, stair entry admission, 지지면·slip·계단참 정지를 증명하지 않는다. 실제 성공은 UNVERIFIED다.

OBSERVED: transport 테스트는 일시 send 실패가 budget 안에서 회복되는 경우와 budget 초과 후 FAULT/no-reconnect를 각각 기대한다. 따라서 B-02는 모든 단일 send 실패가 즉시 영구 FAULT라는 주장이 아니라, budget 초과 또는 terminal transport fault 뒤 복구 부재라는 범위를 유지한다.

OBSERVED: Task 6 PARTIAL_CLAIM은 software fixture-only이고 생산 config acceptance가 아니라고 명시한다. 이를 hardware 성공으로 오독하지 않는다. Task 2 inspector는 원본 bag checksum과 CSV rejection 표시를 검사하지만 CSV 값 전체를 raw bag에서 독립 재계산하지 않으며, interpolation 부재는 summary flag를 읽는다(`.omo/evidence/5f-rf-autonomous-route-repetition/task-2/inspect_package.py:74-91`). 따라서 그 pass는 패키지 내부 일관성 검사다. 이번 감사의 독립 bag 추출은 일부 odom/scan/command 통계에 한정되고, 나머지 CSV 전량 검증은 남는다. 이 한계를 기존 E-03의 evidence 분류에 반영하며 별도 현장 결함으로 단정하지 않는다.

## C-04/E-02의 commissioning 근거 보강

C-04 Safety S4 / Mobility M2, E-02 Safety S2 / Mobility M2 유지.

OBSERVED: Task 5 `BLOCKED_CLAIM.md:10,66-82`는 당시 각 층 surveyed correspondence가 없고, 별도 ascent/descent profile과 tag/readiness 검증이 필요하다고 기록한다. Task 2 waypoint CSV 9행은 좌표가 모두 비어 있다. Task 3 install repair는 당시 stale install을 고쳤다는 historical receipt다. 모두 현재 source 또는 현재 install parity의 직접 증거로 사용하지 않는다.

OBSERVED visual: contact sheet에는 실내, 계단, 옥상, 복귀 실내 장면이 보인다. 216~237초로 표기된 옥상 장면은 비슷하고 239초 장면은 다르다. floor의 정확한 경계, tag decoded ID, survey pose, commanded motion은 그 그림만으로 확정하지 않는다. 경로 그림은 wheel/LiDAR를 독립 상대 좌표로 제시하고 ground truth가 아니라는 제목을 갖는다.

INFERRED: 현재 enabled YAML과 이 과거 blocked 기록 사이에는 provenance를 추가로 확인해야 할 공백이 있다. UNVERIFIED: 후속 승인·현장 측정의 존재 여부 및 실제 물리 안전성. 아직 안 읽은 evidence가 있으므로 승인이 전혀 없었다고 결론 내리지 않는다.

## Git·임시 artifact·다음 조사

OBSERVED: `final-integrity-check.json`의 2026-09-18T04:28:51 UTC 대조에서 HEAD/status 목록/staged diff는 baseline과 같지만 plain worktree diff는 다르다. 1,735개 content hash 중 `.omo/run-continuation/ses_fe79dc3abffe0h8YOLMzr2MX0n.json` 1개와 그 mtime만 변했다. 내용 차이는 session updatedAt 두 필드(02:38:52→04:13:54 UTC)다. `resume4-concurrent-change.json`에 delta를 보존했다. 변경 주체는 UNVERIFIED이며 감사 script의 쓰기 대상에는 없다. 최초 baseline을 덮어쓰거나 이 변경을 되돌리지 않았다. 신규 target 파일은 없다.

이 재개에서는 임시 파일/cache/background process를 만들지 않았다. 이전 resume2 전용 temp root 3개는 모두 현재도 absent이며 receipt의 잔존 thread는 0이다. retained 감사 script/report/receipt는 삭제 대상이 아니다. Git patch 비교 첫 시도는 binary diff 안의 비UTF-8 바이트 때문에 decode 실패했으며 byte-level 재비교로 위 차이를 확인했다.

coverage **5,509 = READ 443 + METADATA_ONLY 1,002 + EXCLUDED 3,749 + UNREAD 315**. KEEP 0, constraint ledger 45개(전체 확정 전). Safety/Mobility 최종 verdict 미판정. 15 E2E 모두 PENDING.

새 조사 대상: 현재 generated/install parity의 선택적 metadata 대조, 후속 commissioning 승인 근거, 큰 로그의 사건 순서, task2 CSV 원본 일치, command boundary 보안과 복구 범위. 다음은 UNREAD `.omo` 계획·기타 evidence·기존 logs·개별 image/video/CSV다. E 완료 뒤 F에서 15개를 근거 기반으로 판정하되 실장비 실행과 구별한다.
