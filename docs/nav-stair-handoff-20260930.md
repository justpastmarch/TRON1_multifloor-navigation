# NAV → 계단 인계 수정 — 2026-09-30

## 변경

- `STAIR_ENTRY`로 향하는 임무 NAV만 0.25 m / 0.2 rad 진입 영역을 사용한다. 다른 NAV 및 위치 유지 보정은 기존 0.05 m / 0.08 rad 정밀 설정을 유지한다.
- map 프레임, 증가하는 타임스탬프, 1초 이내 feedback에서 영역 내 0.5초 지속을 확인한 후 move_base 취소를 요청한다. 실제 PREEMPTED/RECALLED 종료 응답을 받은 경우에만 이 요청의 접근 완료로 처리한다. move_base의 실제 terminal status는 보존한다. 사용자 취소는 성공으로 바꾸지 않는다.
- 계단 인계의 AMCL 위치/방향 범위를 정밀 위치 유지 설정과 분리했다. 동일한 `~stair_entry_xy_tolerance` / `~stair_entry_yaw_tolerance`를 사용한다. 기존 정지 확인, 현재 층·map generation·공분산·freshness 확인, admission token 및 LiDAR VERIFY_ENTRY를 유지한다.
- 임무 중에는 idle AMCL 갱신이 쉬므로 정지한 인계 대기 중 기존 `/request_nomotion_update`를 호출한다. 좌표를 강제로 설정하지 않으며 서비스 응답은 제한 시간 내 확인한다.
- 진입점 단독 NAV를 마친 후 대기는 실제 도착 자세를 기준으로 한다. 4층 LANDING은 기존 지정 지점을 유지한다.
- AMCL transform_tolerance를 0.3 → 0.7초로 조정했다. 이는 스캔 시각 기준 TF 유효기간이며 AMCL 위치 freshness 검사를 완화한 것이 아니다. 로컬에 저장된 upstream `amcl_node.cpp` 1485–1512행의 scan stamp + tolerance 정의를 확인했다. 실주행 TF 오류 해결 여부는 재검증해야 한다.

## 검증과 한계

OBSERVED: 기존 NAV, 인계, 위치 유지 테스트 및 새 영역 진입/영역 이탈/오래된 표본/사용자 취소/다음 goal 초기화/실제 terminal status 보존/정지 중 위치 갱신 테스트 통과. 이 테스트들은 실제 계단 성공률을 증명하지 않는다.

OBSERVED: `ui_test_20260930_084340_e85d5d66.bag`의 기록 궤적에 새 조건을 적용하면 1790725435.282 시점에 거리 0.1965 m, 방향 오차 3.66°로 접근 영역 지속 조건을 만족했다. 마지막 feedback 약 7초 전이다. 이는 기록 궤적에 대한 반사실 비교이며 실제 인계나 등반 성공을 의미하지 않는다.

OBSERVED: 직전 두 BAG의 scan 수신 지연은 p99 0.174/0.142초, 최대 0.321/0.227초였다. 기존 로그에서는 map↔odom TF 시각 부족이 최대 약 0.236초인 예를 확인했다. TF 0.7초 설정은 이 사례에 대한 조정이며 지연 원인 제거 또는 임의 길이 지연 허용이 아니다.

UNVERIFIED: 수정한 코드로 실차 NAV→계단→4층 주행. 실행 중 프로세스에는 자동 반영되지 않는다. 기존 TRON 실행 터미널을 Ctrl+C로 종료하고 바탕화면 TRON 실행 아이콘으로 다시 시작한 뒤 위치를 확인하고 UI에서 4층 이동을 요청한다. 실행 중 스택은 임의로 재시작하지 않았다.

ROS 노드 추가, 속도 증가, 실차 명령 전송은 하지 않았다. 기존 worktree 변경은 보존했다. 분석용 BAG는 읽기만 했다.
