# 최종 source reference 및 범위 보조검증

2026-09-18. `final-findings.json`21개, `architecture-final.md`, `constraint-ledger-final.md/json`67개, 원요청 첨부와 기존 phase/test/failure 보고서를 대조했다. target 수정·새 시험·ROS/SSH 실행은 없다. 이는 source reference와 치명적 범위 과장에 한정한 보조 검토다.

## 반영 확인과 남은 교정

이전 review의 C-07 result publisher141 및 rospy/topics.py812, recorder finish218, C-09 graph16/BFS193, C-11 run249/빈route85, E-01 doc4, E-03 phase callback158이 반영됐다. C-04의 `selected-tests-resume2.json` 추가도 확인했다. architecture의 master 장애 범위·child/launch 구별·내부 명령 중재 표현·rviz/topic 오기도 수정됐다.

**남은 직접 근거 교정:** E-02의 `test/test_system_operator_contract.py:34`가 아직 남아 있다. 이 줄은 정상3node 목록이며 삭제 scan launch 기대값은 **50행**이다. root에 전달했다. E-03의 observed에 있는 즉시 move_base 성공을 바로 지지하려면 `synthetic_hardware_peers.py:165`도 추가하는 것이 좋다. 현재158/195/251은 phase·motion·pose 경로를 가리킨다. 이 추가는 결론 변경이 아니다.

C-08의 보존 upstream 참조가1106/1205/1366으로 교정된 것을 확인했다. source 자체의 semantic 재검증은 root 수행으로 provenance를 구분한다. 초기 loader 시도 중단2회와 실제88개 시험은 최종 test coverage에서 분리되며, 예전 CHECKPOINT의 loader0 문구를 최종 실행 성적으로 복사하면 안 된다.

## E-05 근거와 주장 범위

E-05는 repository command subscriber, loopback SSH tunnel, WebSocket adapter, 일회용 stair token과 설치 ROS master/TCPROS source를 연결한다. 실제 firewall/endpoint 도달성·악용·robot 동작을 UNVERIFIED로 제한해 과장이 없다. `run.sh:236`의127.0.0.1 forwarding, `ros_node.py:91`의NAV subscriber, `robot_client.py:61`의connection, installed master registerPublisher735와TCPROS318은 해당 주장의 경계를 지지한다. 보안 완화가 정상 sensor/action traffic을 유지해야 한다는 비용/범위도 포함돼 있다.

E-05가 모든 보안 세부 hardening 문제를 하나의 원인으로 입증했다는 뜻은 아니다. SSH quoting/PID/tmp/path/fixture 조사 결과는 `phase-E-security-final.md`의 별도 조건부 경계 표를 최종 첨부로 유지한다. 단순 local config 실행을 인증 없는 remote exploit으로, ACCID를 유출된 secret으로 확대하지 않은 판정은 적절하다.

## Constraint67 검증

읽기 전용 대조 결과 **67행, referenced unique source52개, source SHA256 불일치0, 행 범위 오류0**다. 이는 원장의 hash·위치 보존을 검증한 것으로 모든 외부 binary·실물 hazard를 검증한 수치가 아니다. K01–K67 본문을 전부 검토했고 추가 치명적인 실제 보호 보장 또는 물리 측정의 과장 주장은 찾지 못했다. KEEP0와 threshold 최소성 UNVERIFIED 구별도 유지돼 있다.

**K36 scope 보강 필요:** 독립 master라는 이유만으로 전체 실행 lock을 나누면 두 master가 같은 물리 robot을 제어할 수 있다. `기존 owner단위로 범위를 좁힌다`를 `동일 물리 robot/command endpoint의 상호배타는 유지하고, 실제 독립 robot/resource일 때만 lock 범위를 분리한다`로 명확히 한다. 이 보강 없이 master별 lock 허용으로 읽혀서는 안 된다. FD 상속/잔류 소유권은 별도로 해결한다.

K42 unknown과 K43 backup escape는 gate가 아닌 upstream 정책이라는 표시가 적절하다. K28 inflation 전체를 통행금지로 해석하지 않고, K39 현재 override와 historical/runtime를 구별하며, K51 zero 전송을 실제 정지 보장으로 쓰지 않는 범위도 적절하다. K54는300초를 각 phase가 아니라 profile 전체로 설명한다.

## 최종11절에 연결할 분석자료

보고서를 아직 작성 중이므로 아래를 미완료 결함으로 단정하지 않는다. 완성본에서 빠뜨리기 쉬운 연결점이다.

- Executive 두 축 verdict·invariants·roadmap A–D·top5의 실제위험/mission성공/복잡도/복구/비용은 root 최종 종합에서 직접 제시해야 한다. source review만으로 이 절들을 대체하지 않는다.
- 시작 위치7항목은 C-11에만 뭉개지 않도록 3F default, 등록4F/5F, arbitrary pose, floor-only global localization, tag 기반 floor 확인, 접근 가능한 anchor 선택을 구별한다. 제안된 `--start-*` 옵션은 현재 구현이 아니라 개선안으로 표시한다.
- Navigation은 ledger 행 링크만으로 계산을 대체하지 않는다. `phase-D.md`와 `navigation-geometry-model.json`에 실측 미확인 radius0.28/padding0.02, padded 전폭0.588–0.617, inflation7cell, 비용253/246/201, grid cell5cm, 모형 제동거리0.1125m가 있다. 이 수치는 실제 최소 통로폭/안전거리 합격값이 아니다. global/local layer 차이와 optional/comfort/physical 후보 분류를 함께 제시한다.
- 10개 failure의 감지시간/자동복구/수동명령/좁은 경계는 `failure-recovery-final.md`, test9종·실제88/85/3와 recovery liveness gap은 `phase-E-test-coverage-final.md`에 있다. 원문이 제공하지 않는 reset/reconnect 명령을 발명하지 않은 구분을 유지한다.
- 최종 audit coverage에는 READ750/METADATA_ONLY1010/EXCLUDED3749/UNREAD0의 조정과 함께 binary/session/generated의 개별 이유 목록, 실제 실행과 미실행 위험 동작, source/config 무변경과 session metadata delta를 구분한다. 원장의0 UNREAD는 실장비 acceptance를 뜻하지 않는다.

이 retained Markdown만 생성했다. 임시파일/cache/background process0이며 기존 target·산출물은 수정하지 않았다.
