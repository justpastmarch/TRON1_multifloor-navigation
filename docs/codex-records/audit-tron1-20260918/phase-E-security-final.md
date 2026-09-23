# Phase E 보안·실행 경계 마감 검토

2026-09-18. 대상은 `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`의 현재 worktree다. target 파일, ROS graph, 원격 장비를 변경하지 않았다. 이 기록은 정적 source 및 설치 upstream source 검토이며 침투 시험이나 실장비 안전 승인 결과가 아니다. 부모 감사의 coverage와 시험 누계는 수정하지 않았다.

## 판정과 증거 구분

OBSERVED는 아래 source/config/설치 source와 읽기 전용 파일 비교에서 확인한 사실이다. INFERRED는 그 사실이 허용하는 경로다. UNVERIFIED는 실제 네트워크 도달성, 호스트 ACL, 실행 중 import origin, 외부 firmware/SDK 동작이다. 별도 시험을 추가하지 않았으며 기존 88개 시험 누계와 무관하다.

보안의 핵심 경계는 workstation·Mini PC·ROS graph에 접근할 수 있는 주체다. 내부 state/epoch/일회용 token은 정상 구성요소 간 잘못된 순서를 막지만, 그 자체로 ROS participant의 신원을 검증하지 않는다. 실제 공격자가 존재하거나 악용에 성공했다고 단정하지 않는다.

## E-05 — 명령 권한이 ROS graph와 workstation 접근 신뢰에 의존

**Safety S4 / Mobility M3.** OBSERVED 구현 경계, INFERRED 조건부 영향, 실제 노출·악용은 UNVERIFIED.

Repository 정의:

- `run.sh:164-176,212`는 workstation의 LAN 주소를 ROS 주소로 요구하고 master를 시작한다. `stair_supervisor/ros_node.py:91-95,119-120`는 `/navigation/cmd_vel`의 숫자를 받아 supervisor에 넘기며 publisher identity를 검사하지 않는다. `/scan`의 publisher 수 검사는 `run.sh:365-368`의 시작 시점 검사이며 command publisher 인증은 아니다.
- `mission_action_server.py:35-45,79-112`는 목적지·mission type·BUSY를 검증한다. 발행자 identity 허용 목록은 없다. `multifloor_manager/ros_node.py:70-101`의 pose/map/service/action 경계도 graph의 신뢰를 전제한다. floor action의 ownership epoch 공백은 기존 C-10에 통합하며 중복 finding으로 세지 않는다.
- `mission_manager/stair_admission.py:64,87-102`는 예측 곤란한 token, 1회 소비, 수명 및 context 일치를 구현한다. 이 token은 goal 필드와 일반 ROS service에 실린다. 신뢰하지 않는 graph participant에게 기밀 전송이나 참가자 인증을 제공하는 코드가 아니며, 절차적 admission 보호의 존재는 인정한다.
- `run.sh:236-237`의 SSH tunnel은 local listener를 `127.0.0.1`에 제한한다. 그러나 같은 workstation의 다른 process를 구분하지 않는다. `robot_client.py:61-67`의 WebSocket 생성에는 별도 application 인증 인자가 없고 `:150-156,180-182`는 설정 ACCID를 frame 필드로 전송·비교한다. GUID는 `:185-193`에서 response correlation으로 사용한다. ACCID/GUID를 secret 또는 cryptographic 인증으로 보고하지 않는다. SSH는 별도의 인증 경계이며 host-key 검증을 끄는 옵션은 이 wrapper에 없다. Mini PC→robot 구간은 설정상 `ws://` endpoint이고 외부 서버의 추가 통제는 UNVERIFIED다.

설치 ROS upstream 정의를 직접 확인했다:

- `/opt/ros/noetic/lib/python3/dist-packages/rosmaster/master_api.py:114-190`의 `apivalidate`는 caller_id 문자열 및 argument 형태를 검증한다. `:294-309`, `:356-383`, `:735-765`의 shutdown/setParam/registerPublisher 경로에 participant 인증은 없다.
- `/opt/ros/noetic/lib/python3/dist-packages/rosgraph/network.py:241-263`은 LAN ROS_IP일 때 bind 주소를 `0.0.0.0`으로 정한다. `rosgraph/xmlrpc.py:262-309`는 그 주소에 XMLRPC server를 생성하고 handler를 등록한다. 이것은 설치 source 동작이지 실제 listen socket/방화벽 관찰이 아니다.
- `rospy/impl/tcpros_service.py:217-257`, `rospy/impl/tcpros_pubsub.py:318-369`의 연결 검사는 service/topic, callerid, type/MD5를 사용한다. MD5는 메시지 계약 일치 검사다. `actionlib/action_server.py:147-149`의 goal/cancel 입력은 ROS subscriber다.

INFERRED: 신뢰되지 않은 주체가 ROS master와 필요한 node endpoints에 도달할 수 있으면 command·goal·cancel·evidence 주입이나 서비스/parameter 변경으로 내부 절차를 우회하거나 진행을 방해할 수 있다. workstation의 동일 local endpoint에 접근 가능한 process가 WebSocket 명령을 별도로 보낼 가능성도 남는다. Robot firmware의 동시 연결 정책과 실제 motion 결과는 UNVERIFIED다. 이를 인터넷 공개 서비스나 이미 발생한 intrusion으로 표현하지 않는다.

최소 개선: 현 구성의 운영 host/네트워크 경계를 문서화하고 ROS node traffic까지 포함해 승인된 workstation·Mini PC만 통신하게 제한한다. master port 하나만 막는 것으로 node 간 연결 전체가 격리됐다고 판단하지 않는다. robot WebSocket 경로도 승인된 process/host 경계로 좁힌다. 관측용 replay는 별도 graph에서 수행한다. 보안 조치 때문에 새 sensor/mission gate를 추가하지 않는다. 운영 비용은 network rule·계정/서비스 권한·현장 접근 관리이며, 허용된 두 host와 관측 도구의 정상 sensor/action traffic을 보존하는 것이 검증 조건이다. 실제 ACL 유효성은 현장 읽기 전용 점검 후 별도 승인된 connectivity 시험이 필요하다.

## 기존 root cause 보강

### E-01: manual capture 원격 시작 시도와 문서·상태 보고의 불일치

**Safety S2 / Mobility M2.** OBSERVED 구문·문서 불일치, listener 부재 시 결과는 INFERRED, SDK ownership 영향은 UNVERIFIED.

기존 `run.sh:47-50`의 기본 autostart 시도 finding을 유지한다. 다만 성공적으로 listener가 시작된다고 단정하면 안 된다. `:50`의 local double-quoted SSH 문자열 안에서 `root='\${HOME}/.local/share/tron1-sensor-joy'`를 만들므로 remote shell은 `${HOME}`를 single quote 안의 문자로 보존한다. 이후 `"$root/venv/bin/python"` 검사와 `"$root/receiver.log"` redirection은 home 절대경로가 아니다. 기존 process가 있으면 reuse branch가 이를 피한다. 부재 branch는 remote `set -e`가 없고 시작 직후 callback health 확인 없이 `started pid`를 출력하도록 되어 있어, 정상 설치 상태에서도 실제 시작 실패와 성공처럼 보이는 문구가 함께 나올 수 있다. 이 경로를 실행하지 않았다.

최소 변경은 remote home 경로를 정확히 확장하고 시작 결과·최초 callback 상태를 보고하는 것이다. SDK init opt-in/소유권 확인과 문서 정합성은 기존 E-01 조치를 유지한다. SensorJoy 문제로 lidar/계단 외 capability를 일괄 차단하는 gate를 추가할 이유는 없다.

### E-02: 현재 source, 설치 산출물, 시험·문서의 계약이 다름

**Safety S2 / Mobility M2.** OBSERVED 파일 비교, 실제 실행 origin은 UNVERIFIED.

읽기 전용 Python bytes 비교로 package top-level Python module, config subtree, launch 총78개 source→install 대응을 확인했다. 39개 동일, 30개 상이, 9개 대응 없음이다. 모든 generated 파일의 의미 검토나 실행 검증을 했다는 뜻은 아니다. PGM은 bytes 동일성 비교만 수행했다.

구체적 차이:

- `src/stair_supervisor/config/stair_profiles.yaml:2-3`은 configured=true와 profile 목록인데 `install/share/stair_supervisor/config/stair_profiles.yaml:4-5`는 configured=false/profiles=[]다. scan_profiles도 source configured=true, install configured=false다.
- install의 `stair_supervisor/ros_node.py`는 source의 admission validator·goal token 전달·state heartbeat가 없고 IMU subscriber가 있는 이전 구조다. `install/lib/python3/dist-packages/mission_manager/stair_admission.py`, `stair_entry_gate.py`, `stair_supervisor/stair_admission.py` 등9개 대응이 없다.
- generated `_StairTraversalGoal.py`의 devel MD5는 `6af9dffb9325c7705decb110071faaec`, install은 `703a86fc4c19e292f4907208cfce80c9`다. devel에는 source action의 admission_token이 있고 install에는 없다. Result도 devel에는 ENTRY_REJECTED가 있고 install에는 없다. FloorTransitionGoal의 ownership epoch는 양쪽에 존재한다.
- `run.sh:87-94`는 devel을 source한다. devel의 세 package `__init__.py`는 현재 target의 `src/<package>/src`를 경로에 추가한다. 따라서 위 차이만으로 현재 run.sh가 오래된 install을 실행한다고 주장하지 않는다. 다른 shell/overlay/deployment의 실제 import·resource origin은 별도 관찰 대상이다.

최소 조치: 운용에 쓰는 workspace 경로와 generated interface/config 원본을 일치시키고, 배포 시 import/resource origin과 action 계약을 기록한다. 현재 source를 과거 install에 맞추거나 검증되지 않은 install을 활성화하는 조치는 아니다. 일관된 rebuild/deploy 검증에는 일회성 작업 비용이 있지만 정상 주행 중의 추가 gate는 필요 없다.

### B-04: 시작 대기에 전체 시간 상한이 없는 경계

**Safety S1 / Mobility M2.** OBSERVED timeout 부재, 실제 hang은 UNVERIFIED.

기존 TF/RViz finding 외에 `run.sh:127-143,224-225`도 전체 SSH remote command 실행 상한이 없다. 일부 `ConnectTimeout=5`는 SSH 연결 단계 설정이며 내부 topic 명령의 timeout은 개별적이다. `:131,135`는 해당 연결 옵션조차 없다. 실제 연결된 endpoint에서 remote command가 멈추는 경우 startup이 끝나지 않을 수 있다는 조건부 가용성 근거다. 각 remote 작업의 deadline과 실패 이유를 명확히 하는 좁은 개선으로 충분하며, 정상 sensor warm-up까지 같은 짧은 값으로 일괄 제한하면 false-stop이 늘 수 있다.

### E-04: replay 신뢰 경계

**Safety S4 / Mobility M3.** 기존 OBSERVED/INFERRED 판정을 유지한다.

`replay_bag_rviz.sh:28,43-61`의 master 재사용·전체 topic play와 exec/cleanup 경로는 기존 finding이다. bag은 실행 코드라고 주장하지 않으며, ROS command/evidence를 담을 수 있는 외부 입력으로 취급한다. source bag의 command 기록과 실제 로봇 명령 실행은 구분한다.

구별할 정상 보호: `replay_raw_sensors_rviz.sh:37,60-64,96-100`은 별도 기본 port, 기존 master 기본 거부, sensor topic allowlist를 갖는다. override로 reuse를 명시 허용할 수 있다. `replay_stair_state_machine.sh:41,64-81`은 별도 default master와 reuse 거부를 갖고 fake transport node를 사용한다. `replay_joy_preview.sh:43-48`은 `/replay` prefix와 Joy 단일 topic·최대30초 범위를 사용한다. 모든 replay 도구가 같은 위험 경로라고 보고하지 않는다.

## SSH·PID·lock·파일 경계의 나머지 판정

| 조사 대상 | 확인한 사실 및 한계 | 최소 조치·주행 비용 |
|---|---|---|
| SSH quoting | OBSERVED `run.sh:30`은 신뢰된 config를 shell source하고 `:130,225`는 여러 값을 remote shell 구문에 삽입한다. CAMERA_*와 SensorJoy 값 일부는 environment override다. 공백·quote가 있는 배포 값은 구문을 깨거나 remote shell 뜻을 바꿀 수 있다. 로컬 config/env를 통제할 수 있는 사용자의 코드 실행을 인증 없는 외부 exploit으로 격상하지 않는다. | 값별 domain validation 및 remote argument 경계 고정. 정상 topic/IP/path에는 추가 주행 gate가 필요 없다. SSH 계정 권한 최소화는 운영 설정의 일이다. |
| remote process 식별 | OBSERVED `run.sh:225`의 pkill 정규식은 전체 command line `^…$`에 고정되어 있고 remote shell 전체 문자열 자체와 일치하지 않는다. `/apriltag…` pgrep도 `[a]` 패턴이다. 알려진 단순 자기매칭 버그로 판정하지 않는다. 절대 interpreter/실행 모양이 달라지면 남은 owner를 놓칠 가능성은 INFERRED다. | 상태/실행 identity를 확인한 제한된 owner만 재시작. 광범위한 pkill로 고치면 안 된다. |
| local PID cleanup | OBSERVED `run.sh:189-208`은 자신이 시작한 PID 목록을 사용한다. 종료된 child의 PID 재사용 확인이나 wait/reap 완료 확인은 없다. 실제 다른 process를 종료한 증거는 없다. 핵심 stop 전달 순서는 B-03에 통합한다. | child lifecycle을 확인하고 system의 bounded stop 후 tunnel/master 정리. 임의 process 집단 종료는 불필요하다. |
| lock/FD | OBSERVED `run.sh:178`은 고정 `/tmp/tron1_system.lock`을 write-open한 FD9에 flock한다. child 시작에 FD9 close 명시가 없고 cleanup도 wait 완료를 확인하지 않는다. 다른 생존 child가 FD를 유지하는 경우 lock 잔류 가능성은 INFERRED, 실제 재현은 UNVERIFIED다. | per-user 비공유 runtime directory와 child FD close/lifecycle 정리. 정상 중복 실행 차단은 보존하되 오래된 lock을 무조건 삭제하는 복구는 피한다. |
| shared `/tmp` | OBSERVED replay log3종이 고정 `/tmp` 경로를 truncate-open한다. 로컬 신뢰 경계/파일 선점에 따라 충돌·쓰기 실패 가능성이 있다. 관찰 namespace `/tmp` mode1777, kernel protected_symlinks=1/protected_regular=2/protected_fifos=1이므로 보편적인 타사용자 임의파일 덮어쓰기를 단정하지 않는다. 같은 사용자 또는 기존 권한 문제는 별도다. | 전용 제한 directory와 고유 run path. 주행 알고리즘 영향 없음. |
| recording path traversal | OBSERVED `scan_recorder.py:114-123,180-200`은 resolve/relative_to, 상대 profile root, identifier regex, 기존 lifecycle marker를 검사한다. `mission_action_server.py:95`가 mission ID를 생성한다. `:75-85,158-160,215`는 shell 없이 argv로 외부 명령을 호출한다. 임의 mission 문자열에 의한 shell/path injection 근거는 없다. | 현재 검증 유지가 유효하지만 엄밀한 KEEP 최소범위 판정은 전체 constraint ledger에 위임한다. shared writable output parent에서 검사와 생성 사이 symlink 교체는 파일 ACL에 의존하므로 output 권한 확인이 우선이다. |
| output 접근 권한 | 관찰 환경에서 `/var/lib/tron1/scans`와 `/tmp/tron1_system.lock`의 존재를 확인하지 못했다. 이 namespace가 실제 운용 host 파일시스템과 동일하다고 가정하지 않는다. | 현장 output directory owner/mode 및 여유공간 검증. 감사에서 directory를 만들거나 수정하지 않았다. |
| config secret | OBSERVED config.env는 host/user/ACCID와 운용 값이며 이 파일에서 password/private key/token secret은 보이지 않는다. production `.py/.sh/.env/.yaml/.launch`의 secret 관련 패턴 검색 결과는 token 생성의 `secrets` import뿐이었다. history·binary·외부 SSH 설정까지 secret 없음으로 확대하지 않는다. | ACCID를 비밀번호로 오해하지 말고 SSH key/host 신뢰는 외부 운용 설정에서 관리. 사용자가 준 SSH password를 사용·저장하지 않았다. |

위 조건부 hardening 사항은 현재 작업에서 별도 고심각도 exploit finding으로 늘리지 않는다. 주요 영향은 B-03/B-04/E-01/E-04/E-05에 통합했다.

## fixture와 production 선택

OBSERVED: `run.sh:6,98`의 building_valid는 local bundle 검증용이며, optionless 실행은 `:160`에서 production 검증 후 `:246-256`에서 production 기본 launch를 사용한다. 즉 local check가 fixture를 읽는다는 이유만으로 production이 fake transport를 쓴다고 할 수 없다.

OBSERVED: `system.launch:3-5,53-74`는 allow_test_fixture=true일 때 floor/stair config를 fixture로 바꾸지만 mission은 별도 config_profile에 따른다. `mission_manager/ros_runtime.py:41-53`은 production에서 `~stair_config_root` override를 허용하고, test_fixture profile에서만 allow_test_fixture를 검사한다. 직접 launch/parameter를 바꾸는 운영자는 서로 다른 config roots를 만들 수 있다. `ros_entrypoint.py:95-113`의 production stair entrypoint는 config가 fixture인지와 무관하게 real RobotTransport를 생성한다. 이 구성 혼합은 E-02의 배포 계약 불일치에 통합한다. 기본 run 경로에서 발생했다는 뜻은 아니다.

OBSERVED: `synthetic_stair_supervisor_node.py:33-50,58-77`는 loopback 주소만 허용하며 FakeFactory를 명시 주입한다. `STAIR_REPLAY_ALLOW_ADMISSION`은 이 synthetic constructor에서만 Allow validator를 선택한다. 일반 production node에서 같은 env 하나로 admission이 해제되는 경로는 확인되지 않았다. 그러나 loopback 주소 검사만으로 같은 host의 live master와 다른 graph임을 입증하지는 않는다. 해당 wrapper의 existing-master 거부가 추가 보호다.

최소 개선은 production launch의 fixture override 조합과 실제 transport 조합을 명시적으로 검증하고, 시험 전용 override가 real transport와 섞일 수 없게 좁히는 것이다. 그 비용은 시작 시 구성 검증이며 정상 생산 구성의 sensor/mission 조건을 더 강화할 이유는 없다. synthetic test 결과를 물리 commissioning으로 인정하지 않는 기존 E-03 판정을 유지한다.

## 읽은 범위·미확인 조건·phase 인계

이번 새 repository-owned UNREAD 판정은 0개다. 이미 READ인 source를 보안 관점에서 재검토했으며 coverage 상태는 변경하지 않았다. 원문 전체 재검토: run.sh, config.env, replay_bag_rviz.sh, replay_raw_sensors_rviz.sh, replay_stair_state_machine.sh, replay_joy_preview.sh, cmd_vel_bridge.py, sensor_joy_bridge.py, .gitignore, bundle_runtime_contract.py, mission_manager의 scan_recorder.py/recording_session.py/mission_action_server.py/stair_admission.py/ros_runtime.py, stair_supervisor의 robot_transport.py/robot_client.py/ros_entrypoint.py/stair_admission.py, system.launch, synthetic_stair_supervisor_node.py, StairTraversal.action, FloorTransition.action. 이미 READ 파일의 관련 함수 재검토: mission configuration.py, stair ros_node.py, multifloor ros_node.py/ros_runtime.py, manual_mission_capture.py, bag_sensor_visualizer.py.

Generated 파일은 inventory에서 EXCLUDED를 유지한다. 세 devel package loader 원문, 위 install source diff3쌍, generated action6개의 필요한 필드, 총78개 대응 bytes 비교만 추가 확인했다. 이것은 generated 전체를 읽었다는 주장이 아니다. 설치 upstream은 위에 명시한 정확한 함수·행을 읽었다.

확정 사실: 기본 wrapper와 각 replay의 접근 경계, token의 절차적 역할, 원격 autostart 구문의 HOME literal 문제, source/install contract 차이. 추론: 신뢰 경계 밖의 접근 시 command 주입·가용성 손실, 특정 원격 stall/PID·FD 조건에서 복구 문제. 미확인: 외부 ACL/firewall·SSH config·firmware 동시 owner/watchdog·SDK init 영향·실행 중 import origin. 이들은 현장 검증 없이는 판정할 수 없으므로 UNVERIFIED로 닫으며 audit coverage 누락으로 숨기지 않는다.

새 조사 대상은 외부 배포 신뢰 경계와 actual origin이다. 현재 허용된 read-only static 감사로는 위 범위까지 판정했으며 추가 remote command나 hardware 시험을 실행하지 않았다. E의 다른 evidence/coverage 마감은 부모가 관리하고, 그 뒤 F의 원요청15개 E2E 코드 경로 종합이 남는다. KEEP를 새로 부여한 constraint는 없다.

감사 artifact: 이 Markdown은 retained 산출물이다. 임시파일·cache·background process를 만들지 않았다. 모든 파일 조회와 비교 foreground process는 종료됐으며 target 수정이나 기존 process 종료는 하지 않았다.
