# 주행·복구 경계 수정 — 2026-11-04

수정 전 기준 커밋: `b48d577bde8a41c7fcfce3f1be945550112b7fd9`.

현재 미커밋 코드·지도 관리 기능·설정을 먼저 위 커밋으로 보존한 뒤 별도 worktree에서 수정했다. 기존 FSM과 ROS 노드를 유지하며, 정상 계단 상승·회전·도착의 기하학적 판정값은 변경하지 않았다. 이 문서는 전체 저장소 감사나 실장비 연속 주행의 완료 선언이 아니다.

## 변경된 동작

1. **NAV 출력 제한:** 기존 Stair Supervisor의 NAV 전달 경계에서 활성 속도 상한을 적용한다. Mission Manager는 목표 전송 전 기존 planner 설정과 Supervisor 제한 반영을 확인한다. 평지 기본 0.30, RF 기본 0.21이며 정상 계단 상승 0.55와 수동 joystick 명령에는 NAV 상한을 적용하지 않는다. 기존 `dynamic_reconfigure/Reconfigure` 메시지를 쓰는 `/stair_supervisor/set_navigation_limits` 서비스를 기존 노드 안에 추가했다. 늦게 도착한 과거 제한 요청은 revision으로 거절한다. 새 ROS 노드나 생성 메시지는 없다.
2. **자동 NAV 취소:** 취소 시작 시각을 한 번만 기록한다. 지속적인 stale/no-progress가 5초 취소 응답 대기를 매번 초기화하지 않는다.
3. **UI 정지 응답:** 수동 lease 시작 RPC를 공용 console lock 밖에서 실행한다. Stop은 지연된 요청을 무효화하고, 뒤늦게 반환된 lease를 활성화하지 않는다. 끝내 응답하지 않는 RPC는 추가 수동 시작 슬롯만 pending으로 남기며 자동 임무·상태 조회·Stop의 lock은 점유하지 않는다.
4. **자식 action 대기:** 실제 goal ID의 상태·feedback을 확인하며 wall-clock으로 통신/취소/결과 유실을 처리한다. 같은 goal의 정상 상태가 계속 들어오면 전체 상승시간 제한은 없다. 완전한 통신 침묵 30초 뒤 취소를 요청하고, 취소 ACK와 terminal-status 이후 결과 전달은 각각 기존 성격의 5초 대기로 제한한다. LOST 또는 결과 없는 DONE을 도착이나 소유권 해제로 간주하지 않는다. 재요청은 유효한 해당 goal의 terminal result 또는 최신 수신 측 idle 확인 이후에 가능하다.
5. **층 실패 복구:** TRANSITIONING/FAULT에서도 마지막 확인 층을 보존한다. 종료된 FAULT는 기존 명시적 현재 층 선택/수동 이동 완료를 통해 맵을 다시 선택하고 UNKNOWN→재측위를 시작할 수 있다. 실패한 전환으로 다른 맵이 로드됐을 수 있어, FAULT에서 2D pose만 주어 곧바로 READY로 바꾸지는 않는다.
6. **층 도착 AMCL:** RPC 응답 완료시각 대신 초기 pose/마지막 수용 pose 이후의 scan 측정시각을 사용한다. 관측이 없으면 짧게 기다린 뒤 no-motion 갱신을 재요청하고, costmap 대기·최종 dwell에도 관측을 유지한다. 이전 epoch/초기화 전 관측을 거절하는 경계는 유지한다. 이미 조건이 충족되면 추가 RPC 없이 진행한다.
7. **LiDAR raw 입력:** 계산 프로세스 입구의 용량 8 큐가 차면 오래된 대기 프레임을 밀어내고 최신 관측을 넣는다. multiprocessing feeder 경합에는 최대 5ms 읽기 대기를 사용하고, 그래도 실패한 새 프레임 폐기는 별도 진단한다. IMU 범위·epoch·정합 알고리즘·제어 gate 값은 유지한다.
8. **원격 녹화 정리:** 로컬 recorder가 조기 종료되거나 시작에 실패해도 원격 raw 녹화를 기존 비동기 finalizer로 정리한다. 미확인 원격 녹화는 그대로 기록하며 정상 저장으로 가장하지 않는다. 원격 unit·프로세스가 멈추고 `.active`도 없으며 BAG 자체가 생성되지 않았음이 확인되면 `not_recorded`로 처리해 존재하지 않는 녹화 때문에 다음 기록이 영구 차단되지 않게 한다.
9. **큰 BAG 비교:** SHA256을 파일 전체 메모리 적재 대신 1MiB 블록으로 계산한다.

## 검증

- 별도 worktree에서 `catkin_make -j2 -DCATKIN_ENABLE_TESTING=OFF` 성공. 새 생성 메시지/노드가 필요하지 않다.
- `validate_bundle.py --validate-production-config`: `BUNDLE_OK`.
- 미션/UI·NAV·촬영·복귀·층 전환·계단의 선별 50개 모듈을 검사했다. 최종 로그/결과는 프로젝트의 `driving-fixes-20261104`에 보존한다. 상속으로 중복 실행된 test 수를 독립 시나리오 수로 해석하지 않는다.
- 기존 두 BAG에서 추출한 NAV 요청 462개와 1,073개를 수정된 실제 Supervisor에 재생했다. 입력 최대 약 0.653/0.619, RF 상한을 초과하는 요청은 15/28개였으며, 전달 명령의 최대는 0.21, 상한 초과 출력은 두 기록 모두 0개였다. 물리 속도나 새 센서 정합을 측정한 결과가 아니다.
- 새 회귀 검증: 계단 0.55 입력 보존, NAV 0·취소·기존 범위 명령 보존, 늦은 속도 RPC, 120초 정상 quiet action, 빈/다른 goal 상태, 결과 유실/LOST, 수동 RPC 지연 중 실제 UI cancel, 늦은 lease, FAULT 재선택, AMCL stamp 순서/epoch, 실제 multiprocessing.Queue 과부하, 원격 녹화 실패 정리, streaming hash.
- 원래 운용 경로에 적용한 뒤 핵심 11개 모듈 116회 실행이 모두 통과했다(중복 상속 포함). 기본 `test/`의 136개 실행에서는 수정 전부터 있던 동일한 계약 assertion 7개만 실패했고 새로운 failure는 없었다. worktree의 절대 경로 때문에 추가됐던 README 검사는 원래 경로에서는 통과했다. 테스트를 통과시키려고 실제 운용 속도/인터페이스를 과거 값으로 되돌리지 않았다.
- 적용 파일의 SHA256이 검증한 파일과 일치함을 확인했고, 기존 운용 YAML 변경은 0개다.

## 적용 및 남은 확인

운용 폴더에 소스 반영만 하며, 기존 프로세스는 자동 재시작하지 않는다. Mission Manager와 MiniPC Supervisor가 새 NAV 제한 API를 함께 사용하므로 다음 정상 스택 시작 시 기존 원격 배포 절차로 함께 갱신해야 한다. 새 Mission Manager만 실행하고 구 Supervisor를 유지하면 속도 설정 오류가 발생할 수 있다.

실제 계단참 인식, 두 번째 상승, 마지막 두 단과 옥상 NAV, 물리 정지거리·미끄러짐은 별도 실주행 확인이 필요하다. LiDAR 최신 프레임 선택은 지연을 줄이는 대신 관측 간 간격이 커질 수 있으므로 실제 정합 품질과 입력/처리 age를 함께 확인한다. NAV 상한 적용은 과속 명령을 제한하지만 모든 도착 감속이 단조롭거나 물리적으로 매끄럽다고 보장하지 않는다. upstream planner 설정 RPC 자체를 transaction으로 만든 것은 아니며, 최종 Supervisor 상한이 별도로 보장된다.

감사에서 제기한 WebSocket unsolicited 메시지 수신 누적은 합성 부하에서만 확인한 조건부 위험이다. 이번에 socket 수신 구조를 추가 변경하지 않았으며, 실제 펌웨어 수신량과 모드 전환 지연을 측정한 뒤 후속 판단한다.

실제 로봇 이동, 모드 변경, 카메라 촬영, 메일·자동문 요청은 검증 과정에서 보내지 않았다. 소프트웨어의 기존 full-system ROS fixture를 실제 센서 E2E 성공으로 표현하지 않는다.
