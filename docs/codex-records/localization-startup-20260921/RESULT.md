# 검증 및 적용 기록

요청 해석: 선택된 3층 지도 내 임의 위치에서 자동 초기 측위를 기본으로 하고, 수동 위치·방향 지정이 자동 탐색보다 우선하도록 구현했다. 새 production ROS node는 없다. 웹 앱 화면은 추가하지 않았고 RViz 입력과 동일한 ROS 인터페이스를 제공한다.

상태: IMPLEMENTATION_VERIFIED / PHYSICAL_ACCEPTANCE_UNVERIFIED. 전 시스템 감사나 계단 현장 승인 완료 판정이 아니다.

- OBSERVED: 실제 AMCL과 가상 센서의 통합 시험에서 자동 초기화, 수동 재지정, 자동 탐색 중 수동 우선, active mission 입력 거부, stale scan의 READY 차단과 입력 복구, 잘못된 quaternion 거부, no-motion publication을 확인했다.
- OBSERVED: 임의 좌표·방향, 회전된 map origin, 대칭 모서리 ambiguity, unknown/wall rejection, 탐색 취소 및 데이터 부족 단위시험 통과.
- OBSERVED: 논리적 anchor와 같은 목적지도 NAV 실행, stair-first route의 진입점 NAV, 기존 NAV-first 경로의 무우회 동작 검증. 기존 mission/floor regression 통과.
- OBSERVED: 현재 실센서 스캔 1장으로 오프라인 탐색 1.030초. 후보 (0.676897, -2.218269, -0.314159rad), geometric score 0.812722, hit fraction 0.8, margin 0.104123. 실측 현재 위치 정답이 없으므로 위치 오차나 실로봇 성공률을 주장하지 않는다.
- UNVERIFIED: 실제 3F 여러 위치의 자동 측위 정확도, 실제 flat NAV 및 3→4F 계단 주행. LiDAR 계단 route의 현장 설정·물리 hold 검증은 별도로 남아 있다.

## 검토

readchk: 자동 기본+수동 우선, home 물리 위치 가정 제거로 범위를 고정했다.
sip/shower: 독립적으로 읽은 사용 안내에서 층 선택 범위, READY 확인, 취소 경로, evidence 범위가 불명확하다는 지적을 반영했다.
factchk: ROS Noetic 공식 AMCL/move_base source로 initialpose, stationary update, current-pose planning을 확인했다. 링크는 배포 문서에 있다.
mandela: synthetic map/scan은 동일 환경의 수학적 모델이므로 실세계 정확도의 독립 검증이 아니다(Shared-pool/Verifier=designer 위험). ROS lifecycle 동작 시험으로만 해석했다. 실제 측위 성능은 현장 측량과 독립 대조해야 한다.
ssotize audit: managed RViz가 initial pose tool을 금지하던 README/validator/tests를 두 검색 방식으로 확인했다. 새 사용자 요구에 맞춘 입력 계약으로 수정하고 goal 우회 금지는 유지했다. 별도 전역 SSOT 구조 변경은 하지 않았다.
re0: 사용 안내를 현재 지원 범위, 조작 방법, 검증 범위 중심으로 정리했다. detool은 ROS 전용 운용 문서이므로 적용하지 않았다.

## 시험 중 발견 및 수정

정지 AMCL의 no-motion 갱신이 RPC 응답 이전에 처리되는 경우 무한에 가까운 증거 대기가 생기던 경로를 0.5초 갱신 재요청으로 수정했다. 기존 mission 시험 3개의 실패는 준비 단계 home 이동이 실제 NAV로 바뀐 횟수가 누적된 것이어서, 준비 완료 후 카운터를 초기화하고 동일 anchor NAV 시험을 추가했다. 수정 후 실패 0이다.

## 파일 보존

28개 파일 변경을 baseline hash와 대조해 적용했다. 기존 사용자 변경은 보존했고 commit/reset/checkout을 하지 않았다. Git 상태는 이번 요청의 의도된 변경만 추가되었다. 상세 SHA256과 전후 상태는 baseline.json/application-receipt.json을 따른다. 실행 중인 기존 센서·LiDAR 관찰 프로세스는 재시작하지 않았다. 로봇 이동 또는 모드 명령을 보내지 않았다.

검증 결과: 등록 결과 229개 = 실제 testcase 217개 + rostest wrapper 12개. 별도 UI 계약 3개, RViz 재연결 3개와 bundle 검증 통과. 반영 후 원본 run.sh --check 통과.

RViz 재연결 도구도 실제 topic 소유자를 조회하도록 수정해, 관리형 수동 위치 입력을 AMCL로 우회 연결하지 않고 Floor Manager로 연결한다.
