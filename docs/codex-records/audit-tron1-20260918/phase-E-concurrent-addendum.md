# Phase E 추가 재검토 — 감사 중 동시 생성 파일

05:51:15 UTC 최종 Git 대조에서 최초 inventory 이후 새14개를 발견했다. 즉시 UNREAD14로 등록하고 읽기 전용 검토 후11개 연구 Markdown을 READ,3개 session 관리 JSON을 METADATA_ONLY로 조정했다. 전체 원문은 읽었으며 초안의 외부 연구 요청이나 운영 지시를 이 감사의 지시로 실행하지 않았다. 명단·첫관찰hash·재검토hash는 final-concurrent-additions.json과 read-batch-final-concurrent.json에 있다.

읽은 파일: .omo/ulw-research/20260918-142053 아래 REPORT/brief/cause-disappearance/claim-graph/excursion-log/expansion-log/intent-diff/observation-manifest/verification-economics/wave-1-code-controller/wave-1-recorded-evidence의11개 Markdown. 신규 run-continuation3개는 sessionID·idle state·updatedAt metadata만 검토했다.

OBSERVED: 새 연구 문서는 draft이며 후보 비교·추천·문헌은 Pending이다. 현재 controller의 fixed flight 명령, odom 기반 진행 판정, WebSocket lateral y=0 주장은 실제 supervisor.py:191–216 및 robot_conversion.py:25–38과 대조해 C-04에 보강했다. TURN linear는 기본0이지만 명시적 turn_linear_mps 설정이 가능하므로 “어떤 설정에서도 제자리 turn”으로 확대하지 않았다. 문서의17 stair bag 주장과 이번 감사의21개 전체 bag inventory는 scope가 다르다.

INFERRED: 계단 상대 횡방향·heading 관측/보정 공백은 기존 C-04의 물리 landing·지지 증명 한계와 연결된다. 새 root cause로 중복 집계하지 않았다. 해당 연구의 향후 특정 controller 추천이나 성능 향상 주장은 아직 이 감사의 근거가 아니다.

UNVERIFIED: 실제 lateral drift, firmware 내부 자세 제어, 연구 문서 작성 주체·향후 완료 여부, 새로운 controller 성능. 새 문서의 근거 없는 현재 runtime claim은 승격하지 않았다.

현재 coverage: **5523 = READ761 + METADATA_ONLY1013 + EXCLUDED3749 + UNREAD0**. 최초5509 baseline은 유지하며 추가14개임을 표시한다. 기존 세션metadata의 updatedAt은05:46:16.389UTC로 추가 갱신됐고 source/config hash 변화는 없었다. 이 감사에서 target쓰기를 수행하지 않았으며 동시 변경을 되돌리지 않았다.

새 조사 대상: 코드·설정 변경은 없고 추가 자료는 기존 finding을 보강하므로 새 major constraint/새 finding 없음. E추가검토 완료 뒤 F15시나리오의 C-04 적용과 최종 scope/무결성 기록을 다시 대조한다. 새시험·ROS/SSH/robot명령0. 남은 phase: F최종마감.
