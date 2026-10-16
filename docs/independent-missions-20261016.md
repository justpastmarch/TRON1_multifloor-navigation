# 독립 NAV·옥상 순회·수동 이동 후 합류 구현

2026-10-16. 범위: 기존 운용 콘솔과 mission_manager. 전체 저장소 감사 또는 실주행 완료 보고가 아니다.

## 구현 결과와 사용 순서

**OBSERVED · 코드와 격리 UI 시험:** 다음 네 가지 실행을 분리했다. 새 ROS 노드는 추가하지 않았다. 기존 `/mission` → NavigationExecutor / 계단 executor → 단일 supervisor 명령 경로를 사용한다.

| UI | 실행 내용 |
|---|---|
| 목적지 | 등록 지점 NAV, 또는 `지도에서 목표 선택` 후 임의 좌표 NAV |
| 층 이동 | 진입점 NAV → 계단 주행 → 도착 층 전환. 촬영 순회를 자동 시작하지 않음 |
| 옥상 순회만 | RF 현재 위치에서 남서 → 북서 → 북동 → 남동 NAV·촬영·메일 후 대기 |
| 올라가서 촬영 | 현재 층에서 옥상 이동 후 촬영 순회. 복귀는 별도 승인 |

**계단 임무 실패 후 수동으로 RF에 올린 경우:** `중단·저장` → RF 평지까지 직접 이동 → `옥상 순회만` → `RF 평지 확인 · 순회 시작`. 확인하면 NAV 인계와 RF 지도 준비 후 순회를 시작한다. 지도 위치 확인이 끝나지 않으면 자동 시작을 취소하고, 2D 위치 지정 후 다시 시작할 수 있다. 준비만 따로 하려면 기존 `수동으로 RF 평지 도착 · 준비` 버튼을 사용한다.

2026-10-16 후속 수정: 실패 후 녹화가 없어도 중지 버튼을 사용할 수 있고 반복 중지에 오류를 반환하지 않는다. RF 순회 준비 중 중지는 예약된 시작도 취소한다. 이전 BAG 저장 중에는 녹화를 끄면 주행만 먼저 시작할 수 있다. 아래 609건/11개 UI 시험은 최초 독립 임무 구현 기록이며, 이 후속 변경의 검증 기록은 `roof-recovery-20261016`에 별도 보관한다.

준비 버튼은 이 콘솔의 이전 임무와 녹화를 종료하고 기존 supervisor 인계 서비스로 NAV를 준비한 뒤 RF 지도를 요청한다. 오래된 계단 run_id를 입력할 필요가 없다. 준비만으로 주행을 시작하거나 계단 성공 결과를 생성하지 않는다. READY 지도와 실제 위치가 확인되면 시작 버튼으로 출발한다.

**임의 지점 NAV:** `목적지` → `지도에서 목표 선택` → 지도 클릭 → 시작. 끌어서 놓으면 도착 방향까지 지정하고, 클릭만 하면 현재 표시된 로봇 방향을 쓴다. 목표 선택은 AMCL 위치 지정과 별개이며 시작 전에는 이동하지 않는다. 다른 지도/층으로 바뀌면 이전 선택을 지운다. 이동 경로 및 장애물 판정은 기존 move_base가 담당한다.

**촬영 재시도와 복귀 출발점:** 실패한 출발/순회 세션에 남은 사진이 있으면 기존 출발 좌표·사진·수신자를 보존하고 남은 지점만 방문한다. 따라서 5F에서 시작한 실패 임무를 RF에서 이어가면 승인 복귀는 원래 5F 출발점이다. 이미 촬영을 마쳐 복귀를 기다리는 상태에서 새 `옥상 순회만`을 시작하면 기존 세션을 파일로 보관하고 현재 RF 좌표를 새 출발점으로 저장한다. 이후 승인 복귀는 새 RF 출발점으로 돌아간다. 임의 좌표 NAV는 이 촬영 세션을 변경하지 않는다.

## 중단 및 기능 간 간섭 수정

평가는 이 변경 범위의 영향도다. S0=직접 안전 영향 확인 없음, S1=중단/모드 관리에 간접 영향. M3=해당 기능 진행 차단, M4=층간 임무 연결 차단. 실장비 위험도 인증 점수가 아니다.

| 원인 | 수정 | 근거와 평가 |
|---|---|---|
| RF 순회와 기존 전체 촬영 임무가 같은 종류여서 과거 복귀 대기 세션에 묶임 | `rooftop_tour`를 별도 유형으로 구분. 미완료 사진은 재개하고 완료된 이전 세션은 보관 후 새 순회 | OBSERVED · photo_mission.py:130, test_independent_missions.py. Safety S0 / Mobility M3 |
| 수동으로 평지에 옮긴 뒤 기존 계단 모드·층·실패 녹화를 각각 정리해야 함 | `relocate_flat`에서 자신의 임무 종료 → NAV 인계 → hold 해제 → 층 선택을 순차 처리. 단순 raw BAG 복사는 기다리지 않음 | OBSERVED · console_navigation.py:43. Safety S1 / Mobility M3 |
| **UI 상태 조회가 경로 기준점을 변경함**. 층 전환에서 RF 상태가 먼저 도착하면 진행 중 구간의 출발점이 RF로 바뀌어 완료 처리가 실패 | `confirmed_location_id` 조회를 읽기 전용으로 변경. 임무 시작 및 성공한 구간의 완료 처리에서만 내부 기준점 갱신 | OBSERVED · mission_orchestrator.py:54. 수정 전 격리 UI에서 결과 9 및 `anchor 'stair_rf_landing' does not match segment source 'stair_5f_to_rf'` 재현. 수정 후 같은 흐름 성공. Safety S1 / Mobility M4 |

상태 조회 오류를 실장비의 과거 모든 중단 원인으로 확정하지 않는다. 같은 코드 경로에서 발생할 가능성은 INFERRED이며, 특정 BAG과의 시간 대조는 이번 작업 범위에 포함하지 않았다.

실패/정지 후 새 임무는 이전 goal의 종료를 확인하고 시작한다. 과거 goal의 늦은 결과가 새 임무의 active 상태를 지우지 않는 것을 검사했다. 준비 요청 시작 실패, 녹화 종료 예외, 서비스 탐색 실패에서는 준비 중 플래그를 해제한다. 취소 이후에는 다음 모드·지도 요청을 보내지 않는다.

이미 전송된 모드/지도 서비스가 응답 대기 중이면 해당 작업은 여전히 진행 중으로 표시한다. HTTP 취소나 시간 초과만으로 그 요청이 없어진 것으로 취급하지 않는다. 실제 연결 장애·새 지도 위치 미확인까지 성공으로 간주하도록 변경하지 않았다. 서비스 자체가 응답하지 않는 상황에서는 연결 복구가 필요할 수 있다.

## 검증 결과

**OBSERVED:** 로컬 catkin 빌드 성공, `run.sh --check` 성공. 검사 출력의 robot_commands_sent=0. 기존 기능 및 새 회귀 테스트 **609건 통과**. 이 수치는 테스트 실행 건수이며 독립 실주행 횟수가 아니다.

실제 저장소 HTML, HTTP 서버, RosConsole, MissionActionServer, MissionOrchestrator, PhotoMission, NavigationExecutor를 연결한 Chromium 시험 **11/11 통과**. 이 과정의 POST 요청/응답 41쌍은 갱신한 API 스키마의 사용 부분과 일치했다.

| UI 시나리오 | 판정 |
|---|---|
| 지도 목표 선택·미리보기·좌표 NAV, initialpose 미발행 | PASS |
| 좌표 NAV 중지 후 다른 좌표로 재출발 | PASS |
| RF 순회 4장 촬영·발송 큐 및 RF 출발점 승인 복귀, 계단 구간 없음 | PASS |
| 5F 상승 실패·녹화 유지 → 수동 RF 평지 준비 → 남은 순회, 기존 5F 출발점 보존 | PASS |
| 2장 촬영 후 NAV 실패 → 독립 좌표 NAV → 남은 2장만 촬영 | PASS |
| 복귀 대기 상태에서 좌표 NAV와 새 RF 순회 실행 | PASS |
| 층 변경 시 오래된 지도 목표 폐기, 별도 2D 위치 지정 | PASS |
| 층 이동만 실행하여 RF 도착, 자동 촬영 없음 | PASS |
| 휴대폰 너비 390px 표시 및 버튼 접근, 가로 넘침 없음 | PASS |
| 5F 출발 → 계단 → RF 순회 → 승인 복귀 → 5F 출발점 | PASS |
| NAV 중 방향키·수동 촬영·SMTP 실패·문 요청이 자동 임무를 취소하지 않음 | PASS |

외부 로봇 이동, 실제 SMTP 발송, 실제 문 개방 요청은 모두 0회다. 모의 서버만 실행했고 실스택은 재시작하지 않았다.

### 검증의 독립성과 한계

소프트웨어 요청·결과·세션 보존·취소·경로 기준점 경쟁 조건은 실행으로 관찰했다. 정상 계단 구간은 실제 supervisor/feedback 코드에 이상적인 위치·IMU 입력을 제공했다. NAV의 move_base 통신 상대, 운동, floor/map 전환 서비스, 카메라·메일·자동문은 시험용 대체 구성이다. 새 점군 정합, 실제 미끄럼, Wi-Fi 지연, 실제 문 위치와 카메라 신선도는 재현하지 않았다.

mandela 검토에서 `Shared hallucination` / `Verifier = designer` 위험이 남는다. 설계자가 만든 운동 모형이 목표에 도달한 결과를 실장비 등판 성능 증거로 사용하면 순환 검증이 된다. 따라서 **실주행 성공률과 이번 코드의 실장비 전체 미션 완료는 UNVERIFIED**로 둔다. 이 항목의 독립 확인은 실제 BAG/조이스틱/최종 위치/새 RGB 사진 및 수신 기록으로 해야 한다. 위 PASS는 소프트웨어 연결과 상호작용 판정이다.

## 배포 및 계약

- 실제 저장소에 변경을 적용했고 로컬 메시지/패키지를 재빌드했다. 실행 중인 스택을 임의로 재시작하지 않았다.
- 기존 실행 아이콘 또는 저장소의 `run.sh`로 스택을 다시 시작하면 새 UI를 읽는다. HTML만 새로고침해도 이미 실행 중인 서버의 코드가 갱신되는 것은 아니다.
- Mission.action에 point_floor, point_map_generation, point_x, point_y, point_yaw를 추가했다. 기존 이름 기반 요청은 기본값을 쓴다. 같은 Mission action을 사용하는 프로세스는 동일한 메시지 빌드로 재시작해야 한다. 다음 정상 시작의 기존 MiniPC 배포 절차가 패키지를 동기화한다. 이번에는 MiniPC 서비스 배포·재시작을 별도로 실행하지 않았다.
- 계단 센서 정합·중앙선 제어·모터 출력 정책은 변경하지 않았다. 기존 수동 조작·녹화·자동문·메일 경로를 재사용한다.
- 상세 요청·응답은 [console-api.md](console-api.md), 기계 판독 계약은 [console-api.openapi.json](console-api.openapi.json)을 기준으로 한다.

## 변경·검증 기록

핵심 구현: `console_navigation.py`, `mission_types.py`, `mission_action_server.py`, `mission_orchestrator.py`, `ros_segments.py`, `photo_mission.py`, `ros_console.py`, `mission_console.py`, `mission_console.html`. 메시지 정의, bundle 계약, CMake 테스트 등록과 API 문서를 함께 갱신했다.

검증 자료 디렉터리: `/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/mission-independent-20261016/`.

- `regression.log`: 609건 실행 결과.
- `browser-results.json`, `browser.log`: 11개 UI 요청·응답 및 이벤트.
- `anchor-race-before.json`: 수정 전 층 전환 기준점 오류.
- `build-final.log`, `bundle-check.log`, `api-check.txt`: 빌드·구동 전 검사·API 계약.
- `desktop.png`, `mobile.png`: 모의 서버에 연결한 실제 UI 캡처.
- `before/`, `git-status-before.txt`, `git-status-after.txt`, `changes.patch`: 이번 변경을 기존 worktree 수정과 구분하는 기록.

Git 상태는 전후 동일하지 않다. 이 요청으로 코드를 수정했기 때문이다. 작업 전부터 존재한 수정은 보존했고 reset/checkout/commit/push를 수행하지 않았다. 이번에 만든 모의 서버·브라우저 프로세스와 임시 사진·메일 큐·소켓은 정리했다. 빌드 산출물은 실행에 필요한 결과로 유지하고, 위 로그/캡처/변경 전 사본은 검증 기록으로 보관한다. `changes.patch`는 변경 전 사본이 있는 파일과 신규 파일을 담는다. 기존 수정이 있었고 이번 시작 시 사본을 남기지 못한 `bundle_source_contract.py`는 이 패치에서 제외하고 `change-manifest.json`에 별도로 기록했다.
