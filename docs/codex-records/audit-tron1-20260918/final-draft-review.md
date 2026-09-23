# 최종 초안 사실·범위 검토

검토: `final-findings.json`의 기존20개 finding과 `architecture-final.md`. 2026-09-18. target는 읽기 전용이며 새 실험·ROS/SSH 명령 실행 없음. E-05 추가 전 초안이라는 점을 알고 검토했다. finding 결론을 뒤집는 반증은 확인하지 않았으나 아래 범위·근거 위치를 수정해야 한다.

## 우선 수정

1. **Architecture ROS master 장애 범위:** `master 장애는전체graph 영향`은 모든 기존 node 통신이 즉시 중단된다는 뜻으로 읽힐 수 있다. `master 장애는 discovery/parameter 및 새 연결에 영향을 주며, 이미 연결된 통신이 즉시 정지함을 보장하지 않는다`로 좁힌다. 설치 `/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py:666-700`에서 메시지 송신은 기존 socket의 sendall을 직접 사용함을 확인했다. node와 master를 함께 종료하는 run.sh cleanup 결과와 master 단독 장애를 구분한다.

2. **Process 종료와 node 실패:** architecture 표에 `이 launch의 일반 node는 required/respawn이 설정되지 않았으므로 child 사망이 top-level roslaunch 종료를 뜻하지 않는다. wrapper가 기다리는 것은 system roslaunch PID다`를 명시한다. 이미 phase-B에서 설치 `roslaunch/core.py:432-434`, `pmon.py:560-625`를 확인했다. AprilTag detector의 명시적 respawn은 예외다. 이 distinction은 C-01의 child terminal 부재와 root wrapper가 살아 있는 상태가 동시에 가능함을 설명한다.

3. **C-04 probe 직접 provenance 추가:** 현재 `@audit/pure-C-probe.json`은 2.02m 누적 drift 반례만 담는다. incomplete/faulted LANDING+cancel의 실제 실행 반례는 `@audit/selected-tests-resume2.json`의 `landing_cancel_probes`에 두 건이 있다. 이 artifact를 evidence에 추가한다. source 논리와 fake-peer 반례는 OBSERVED이고 실기체 motion/추락은 UNVERIFIED라는 현재 구분은 적절하다.

4. **E-02 install/config 혼합 사실:** 초안 title은 배포 산출물을 포함하지만 observed에는 비교 결과가 없다. `phase-E-security-final.md`의 78개 선별 source→install 비교(39 동일/30 상이/9 없음), source와 install의 profile disabled 차이, StairTraversal admission 필드 차이를 짧게 반영한다. `run.sh`는 devel을 source하고 devel loader는 현재 source를 가리킨다는 제한도 반드시 함께 둔다. 오래된 install을 현재 run이 실행 중이라고 확대하지 않는다. fixture override와 real transport 혼합은 명시 override 경로의 위험으로 범위를 제한한다.

## 근거 행 교정

모든 JSON evidence의 실제 repository/설치 파일은 존재했고 지정한 줄이 EOF 밖인 경우는 없었다. 다만 아래 위치는 빈 줄 또는 다른 문장을 가리킨다.

| finding | 현재 근거 | 수정할 근거 | 이유 |
|---|---|---|---|
| C-07 | installed `actionlib/action_server.py:140` | `:141` 및 `rospy/topics.py:812` | 140은 latched status publisher다. result publisher는141이고 Publisher 기본 latch=False는812다. |
| C-07 | `record_route_runner.py:217` | `:218` | 실제 finish 호출을 직접 가리킨다. 217은 try다. |
| C-09 | `route_planner.py:223` | `:179` 또는 `:193` | 223은 edge enabled 검사다. no-directed-route 예외는193-194다. |
| C-09 | `building_graph.yaml:1` | `:16` 및 `:30` | 1은 stale disabled 주석이다. 문제의 단방향 NAV edge는16-19와30-33이며 전체graph/순수probe로 역방향 부재를 증명한다. |
| C-11 | `run.sh:245` | `:249` | 245는 빈 줄이고249-253에 독립 floor/location/pose 입력이 있다. |
| C-11 | `mission_orchestrator.py:88` | `:85` | pose/health 없이 빈 route 성공하는 branch는85-86이다. |
| E-02 | `test_system_operator_contract.py:34` | `:50` | 34는 올바른3node 목록이다. 삭제된 scan launch 기대값은50이다. |
| E-03 | `synthetic_hardware_peers.py:157` | `:158` 및 `:165` | 157은 빈 줄,158은 phase 피드백 수신,165-168은 즉시 move_base success다. |
| E-01 | `manual-mission-capture-ko.md:3` | `:4` 또는 인접 문단 전체 | 3은 실행 전제이며 no-robot-software 문장의 직접 위치로 좁히는 편이 정확하다. run.sh:14 자체도 명확한 근거다. |

C-08의 보존 upstream 줄 참조는 root가 직접 재검토해 AMCL nomotion을1106, laser update threshold/pose 경로를1205/1366으로 교정한다고 전달했다. 이 교정 전의1023/1111을 최종 직접 근거로 유지하지 않는다. 해당 교정은 이번 reviewer가 중복 검증한 결과로 세지 않는다.

## 주요 반박 가능성 점검

- **C-07 범위 적절:** 초안은 unrelated mission result가 없는 정상 단일 녹화라고 한정한다. recorder.finish가 부모 terminal 발행보다 먼저이며 result publisher가 non-latched라는 근거가 맞다. 이를 모든 possible run에서 반드시 실패한다고 확대하면 안 된다. BUSY/INVALID_GOAL 등 unrelated result가 count를 채우는 예외는 현재 설명에 보존돼 있다.
- **C-04 구분 적절:** observed는 순수 코드 반례이며 실제 추락은 unverified다. `wheel slip/drift`는 가능한 물리 원인의 예시로 유지하되 bag이나 실제 기체에서 이 반례를 재현했다고 하지 않는다.
- **B-04 구분 적절:** helper 실패는 wrapper `wait_for_tf_exec.sh:22-25`에서 경고 후 exec로 진행한다. RViz 없음=항상 navigation 불가가 아니다. 응답 없는 synchronous RPC의 조건부 hang과 잘못된 frame 문자열 검사는 별개로 남는다.
- **E-01 구분 적절:** 초안은 원격 자동 시작을 성공이 아니라 시도로 표현하고 실제 listener 시작은 unverified로 남긴다. HOME literal/실패 뒤 성공처럼 보이는 출력 경로는 source 정적으로 확인했으며 실행하지 않았다.
- **현재0.3/역사0.5:** 검토한 두 초안에는 과거 bag0.5를 현재limit로 오인하는 주장이 없다. 향후 종합에 속도를 쓰면 current config.env:45와 navigation.launch:7/68의0.30, 역사bag의0.50을 시점별로 구분한다. source 설정값과 실제 liveparameter도 구분한다.
- **외부 runtime provenance 적절:** architecture의 astra active/enabled는 phase-C-continuation의 SSH key read-only 기록(02:52:52/02:53:32UTC)에 있다. 그때 ROS process가 보이지 않았다는 사실은 current graph 전체 중지의 증거가 아니며, astra 활성은 동시 command 충돌의 증거가 아니다.

## 용어·표 정리

- architecture `system.launch(rv iz옵션)`의 띄어쓰기 오류는 `rviz 옵션`으로, `/ tron/imu`는 실제 `/tron/imu`로 수정한다.
- `현재 명령 소유권은 supervisor 내부 NAV/STAIR state와epoch에서만 보장된다`는 전체 correctness 보증처럼 읽힐 수 있다. `구현된 명령 중재 범위는 supervisor 내부 NAV/STAIR state와 epoch다`로 좁힌다. C-03/C-04/C-10과 외부 peer 경계를 동시에 인정하는 표현이다.
- architecture의 짧은 `ros_node.py` 위치는 mission/floor/stair 세 파일이 있어 모호하다. 최종 hyperlink에는 package를 포함한 절대경로를 사용한다. 이미 full path인 final-findings.json evidence는 이 문제를 갖지 않는다.

이 리뷰는 초안 사실·provenance 검토이며 모든5509파일의 독립 재감사가 아니다. 이 Markdown만 retained 산출물로 추가했고 target/다른 보고서는 수정하지 않았다. 임시파일/cache/process를 생성하지 않았다.
