# 현재 층 변경·수동 이동 완료 수정

2026-10-16

OBSERVED: 최근 요청은 UI에서 접수됐지만 FloorSelection의 합쳐진 거절 사유 `finish the mission and stop before selecting the actual floor`로 실패했다. 당시 UI 임무 active=false, NAV 소유권 연결 정상, 지도는 RF에 남았다. 이후 점검한 네 action 상태 목록은 모두 비어 있었다. 이전 실패 순간의 어느 분기였는지는 합쳐진 메시지만으로 확정할 수 없다.

코드상 지도만 변경하는 요청에도 startup.stationary()의 단일 오도메트리 표본을 요구했다. 하드웨어 표류 또는 표본 변화로 이 조건이 간헐적으로 실패할 가능성이 있다(INFERRED). 명시적인 층 선택에서 이 조건을 제거했다. 실제 실행 중인 임무·계단 모드·층 전환과 충돌하는 요청은 여전히 차단하고, 거절 사유를 구분해서 반환한다. 지도 변경은 READY 성공을 강제하지 않으며, 위치 확인은 별도로 진행한다.

UI의 버튼 바로 아래에 지도 변경 중·실패 이유·현재 지도·위치 확인 대기를 표시한다. 준비 요청 중 버튼 문구도 바뀐다. 실패 상세가 다른 패널에만 표시돼 버튼이 먹지 않은 것처럼 보이는 문제를 개선했다.

회귀127개 통과: 오도메트리 정지 판정이 거짓이어도 유휴 상태 지도 변경 성공, 실행 임무 중 변경 차단, 잘못된 지도 실패, 지연된 이전 작업의 상태 덮어쓰기 방지, UI 평지 준비와 관련 동작 포함. 실시간 지도 적용 결과는 별도로 기록한다.

실제 적용: 스택 재시작 후 /api/select_floor(5F) accepted=true, 지도 세대0→1 확인. /api/relocate_flat(5F) 실행 결과: {"active": false, "relocating": false, "relocation": {"state": "DONE", "floor_id": "5F", "detail": "5F 평지 준비 요청 완료. 지도 위치 확인 후 주행·순회를 시작하세요.", "nav_ready": true}, "floor": {"age_sec": 0.05379040807019919, "value": {"floor_id": "5F", "map_generation": 2, "state": 2, "detail": "auto localization ready"}}, "map": {"width": 366, "height": 278, "resolution": 0.05000000074505806, "origin": [-5.762686, -11.722302, 0.0], "frame": "map", "bounds": [65, 20, 258, 257], "revision": "a7ce6f2c6fce0153ef6c"}}
지도 전환과 별도로 자동 위치 탐색은 움직임에 따른 재시도 상태가 관찰됐다. 주행 성공이나 실제 위치 일치까지 확인한 것은 아니다.
