# 문틀 NAV의 방향 정렬 진행 인정

2026-09-29. 구현·오프라인 검증 완료. 실행 중인 ROS 스택에는 아직 반영하지 않았으며, 실기 문 통과 성공은 UNVERIFIED다.

## 변경 범위

기존 mission_manager의 NavigationExecutor에 진행 감시 어댑터를 연결했다. 새 ROS 노드, 속도 발행자, 상태머신 phase는 추가하지 않았다. 기존 ROS action goal/feedback/result/cancel 경로를 사용한다. 자동 목표 재전송으로 시간을 연장하지 않는다.

| 조건 | 구현 |
|---|---|
| 이동 진행 | 마지막으로 인정한 위치에서 XY 5cm 이상 이동하면 갱신. 우회 경로도 허용 |
| 최종 방향 정렬 | 목표 50cm 이내에서 목표 yaw 오차가 마지막 인정값보다 3° 이상 새 최저값을 만들면 갱신 |
| 방향 잡음·반복 회전 | 단순 회전량은 진행으로 인정하지 않음. 이전 최저 오차와 인정 기준을 유지하여 같은 왕복에 반복 점수 부여하지 않음 |
| 진행 정체 | 이동·방향 개선이 20초 없으면 해당 goal만 취소, NAV_NO_PROGRESS 보고 |
| 피드백 소실 | 유효 action feedback이 10초 없으면 해당 goal만 취소, NAV_FEEDBACK_STALE 보고 |
| 기존 move_base 감시 | XY-only oscillation_timeout을 10초에서 60초로 변경. 비활성화하지 않음 |
| 취소 확인 | terminal acknowledgement를 기다림. 5초 내 응답이 없으면 lifecycle 오류. 미확인 goal이 있는 동안 새 goal을 보내지 않음. 나중에 일치하는 terminal이 오면 이전 goal 소유권 해제 |

각도는 ±π 경계에서 정규화한다. 작은 각도 개선은 누적되어 3°에 도달하면 인정한다. 정렬 영역에서 나갔다 들어와도 같은 방향 개선을 중복 인정하지 않는다.

타이머는 goal 송신 준비 시 monotonic 시간으로 시작한다. 첫 유효 위치가 기준 위치를 설정하고 진행 시간을 갱신한다. feedback은 같은 goal 세대, map frame, 증가하는 pose timestamp여야 하며 수신 시각으로 freshness를 계산한다. 위치·방향이 NaN이면 진행·freshness를 갱신하지 않는다. 두 deadline이 동시에 만료되면 feedback 소실 사유가 우선이다. 대기는 ROS /clock에 의존하지 않는 threading.Event를 사용한다.

## 실패·취소·복구

진행 감시 취소는 사용자 취소와 구분하여 mission에 실패와 구체적인 사유를 전달한다. 이때 costmap clear 후 같은 명령을 자동 재시도하지 않는다. 실제 SUCCEEDED 결과가 취소와 경합하면 성공을 존중한다. 사용자 취소가 먼저 표시된 경우 PREEMPTED는 사용자 취소로 유지한다. 오래된 goal의 feedback/result는 새 goal을 완료시키거나 중단시키지 않는다.

진행 실패 원인을 확인하고 새 명령을 내리면 기준은 새로 시작한다. 취소 응답이 없는 동안 무조건 재송신하지 않는다. 충돌 검사, inflation, 이동 속도, stair supervisor 제어는 이번 변경에서 완화하지 않았다. 반지름 22.5cm와 padding 2cm는 앞서 확인한 설정을 유지한다.

## 적용 경로와 한계

- mission/UI에서 기존 NavigationExecutor를 사용하는 NAV와 arrival hold: 새 진행 감시 적용.
- RViz에서 move_base로 직접 보낸 목표, 과거 독립 문틀 시험 스크립트: 새 어댑터를 통과하지 않는다. 현재 navigation.launch를 쓰는 경우 60초 보조 한도만 적용한다. 과거 snapshot launch는 당시 설정을 계속 사용할 수 있다.
- 목표에서 먼 곳에서의 회전에는 최종 yaw 개선 점수를 주지 않는다. 그곳에서는 최종 목표 방향이 현재 경로 방향과 다를 수 있기 때문이다.
- 60초 보조 한도는 오래 제자리 회전하는 경우 여전히 종료할 수 있다. 새 감시가 move_base 내부 타이머를 리셋하지는 않는다.
- XY 5cm 이동 인정은 경로 우회를 허용하는 활동 검사이지 최종 목적지 수렴 증명이 아니다. 반복 이동·실제 기체 드리프트·AMCL 위치 점프가 진행으로 인식될 수 있다. 이 문제를 해결했다고 주장하지 않는다.
- 명령 전송 대비 기체 이동 부족은 이 수정으로 고쳐지지 않는다. 정체를 구분해 보고하며 제어권/모드/응답은 별도 확인한다.

## 검증

OBSERVED: NAV lifecycle·hold·진행 감시·정밀 도착·segment·orchestrator 66개 테스트와 runtime gating 2개 테스트가 통과했다. 정렬 45초 지속, 각도 경계, 작은 잡음/왕복 회전의 정체, 피드백 소실, 사용자 취소, 늦은 콜백, 취소 확인 시간 초과·늦은 확인 복구를 포함한다. 여러 테스트를 한 프로세스에 묶었을 때 기존 mock import 간섭으로 runtime gating 로딩이 실패하여 그 모듈은 독립 프로세스로 검증했다.

OBSERVED: 문틀 BAG 13개 안의 14개 action 구간에 기록된 feedback을 새 정책에 넣었다. 일부 BAG은 같은 주행 구간이 겹치므로 독립 시행 횟수로 보지 않는다. 기존 terminal 이후의 피드백은 새 정책 평가에 사용하지 않았다.

- 제안 정책(20초·5cm·방향 인정): 기존 terminal 이전 새 중단 없음. ALIGN 성공 4개도 중간에 차단하지 않음.
- 동일 20초·5cm에서 방향 인정만 제거: 역시 기존 terminal 이전 새 중단 없음. 따라서 위 결과 전부가 방향 감시의 효과라는 주장은 불가하다.
- 기존 근사 조건 10초·20cm를 고정하고 방향 인정만 추가: aligned의 기존 oscillation 종료 6개 중 4개에서 기존 terminal 이전 조기 중단을 피함. 2개는 이 변경만으로 해결되지 않음.

근사 비교는 move_base 전체 재실행이 아니다. 기록된 초기 feedback을 anchor로 사용하며 내부 timer 이력과 완전히 같지 않다. 기존 BAG가 중단 시점에서 끝나므로 이후 정상 통과 여부는 알 수 없다. 로봇 명령은 송신하지 않았다.

## 제약 평가

이번 한정 평가: Safety S1=직접 안전 위반 미확인, S2=명령 지속/측위 불확실성에 대한 실기 확인 필요; Mobility M3=주행 중단에 영향.

| 항목 | 판단 | Evidence | S/M | 최소 범위와 복구 |
|---|---|---|---|---|
| 방향 개선 인정 | NARROW | 구현/정책 재생 OBSERVED, 실기 UNVERIFIED | S1/M3 | 목표 근처의 새 최저 방향 오차만 인정 |
| 20초 정체·10초 feedback | TUNE | 단위시험 OBSERVED | S2/M3 | 해당 NAV goal만 취소. false positive는 원인 확인 후 새 명령으로 복구 |
| 60초 XY 보조 감시 | TUNE | launch OBSERVED | S2/M3 | 정렬 여유를 늘린 유한 한도. 실기 시간/진행 로그로 재평가 |
| 5초 취소 확인 | TUNE | 시험 OBSERVED | S2/M3 | 미확인 소유권 중복 방지. 늦은 terminal 확인 시 해제 |

새 숫자들을 최적값이나 실기 검증된 KEEP 제약으로 선언하지 않는다. 실제 시험은 기존 성공 사례를 유지하면서 수동 개입·불필요 정지·문틀 여유를 함께 확인해야 한다.

주요 파일: `navigation_progress.py`(정책), `ros_navigation_progress.py`(ROS action 연결), `navigation_executor.py`(기존 lifecycle), `navigation.launch`(60초 보조 감시).
