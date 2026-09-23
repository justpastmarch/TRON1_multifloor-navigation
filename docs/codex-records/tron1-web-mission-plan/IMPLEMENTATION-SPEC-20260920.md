# TRON1 구현 상세안

2026-09-20 · REFERENCE_ONLY / 전체 적용 보류

현재 구현 범위는 [2026-09-21 최소 변경 계획](IMPLEMENTATION-PLAN-DRAFT.md)을 따른다. 아래 내용은 당시 상세 설계와 확장 후보다. MotionCoordinator·범용 lease·LocalMotionEstimator·raw/control TF 변경·SQLite·전용 HOLD를 일괄 구현하지 않는다. 현재 계획의 실험에서 기존 경로의 부족함이 확인된 항목만 별도 변경 보고 후 선택한다. 과거 설계 검토의 근거를 보존하기 위해 본문은 남긴다.

Understood as: 사용자는 작업 순서의 반복이 아니라, 현재 코드에 어떤 인터페이스·상태·계산·복구 경로를 넣어 요구한 운용을 구현할 것인지 검토 가능한 설계를 요구했다. 본 문서는 그 구현안이다. 운용값·실장비 성공·현재 시스템과의 완전한 호환성을 이미 검증했다는 뜻이 아니다. 현재 코드에서 인용한 동작은 OBSERVED이고, 새 타입·상태·계산과 배치는 제안이다. 물리적 정확도·기체 응답·성능은 UNVERIFIED다. 관련 Safety/Mobility 영향 등급은 기존 보고서 R1~R5 및 구현 계획의 finding 연결을 따른다.

기준 저장소는 `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`이며 이번 확인에서도 HEAD `eb24e08c9220f454080bd672bd8ce3ec0aeb7159`, 변경 없음이다. 센서 구현 후보는 별도 저장소 `/home/m3tron/Desktop/TRON1_Control/tron1-control-center`에 있다. Mini PC의 실제 배포 소스·설정·발행자와 일치하는지 배포 전에 확인한다. 원본 recording·운용 코드·ROS 상태는 변경하지 않았다.

## 1. 이번에 보고하는 구조 변경

기존 세 제어 노드를 유지하고 내부 모듈과 명시적 API를 확장한다. 노드가 그대로라는 사실만으로 구조 변경이 없다고 표현하지 않는다.

| 변경 | 이유 | 제안 배치 |
|---|---|---|
| 공통 MotionCoordinator | 기존 MissionActionServer의 `_active`는 그 서버 안에만 있어 새 요청 표면과 HOLD까지 조정하지 못함 | 기존 mission_manager 내부. legacy/v2/floor/HOLD의 접수·취소를 함께 조정 |
| MissionV2 및 InitializeFloor | 기존 Mission.action은 destination_id·mission_type·return_after_task뿐이고, FloorTransition은 계단 연결을 전제 | 기존 mission/floor 노드에 새 action endpoint. 기존 msg/action 필드와 숫자 enum 유지 |
| LocalMotionEstimator | raw odom 누적 변화만으로 실제 밀림과 주행 속도를 판단할 수 없음 | 기존 센서 bridge 내부 worker. 처음에는 관측 전용, 검증 뒤 선택적으로 제어 연결 |
| HoldController | 기존 supervisor는 NAV/STAIR만 처리하고 zero 송신으로 위치를 복원하지 않음 | 기존 stair_supervisor 내부. 기존 transport 하나 사용 |
| PHOTO_CAPTURE·DEVICE_ACTION | 현재 segment는 NAVIGATION/STAIR/FLOOR_TRANSITION/SCAN | 기존 mission executor의 typed handler 확장 |
| 웹/ROS 연결 | 확인한 launch에는 사용자 웹앱 접수 경로가 없음 | 기존 웹앱/bridge 위치 확인 후 확정. 별도 ROS client가 필요하면 새 ROS 노드이므로 사전 보고 |

기존 `SupervisorState` 숫자 enum에 HOLD를 임의 추가하지 않는다. legacy 호환용 상태는 기존 의미를 보존하고, 실제 motion activity·command acceptance는 별도 상태/API로 드러낸다. legacy NAV 요청이 HOLD와 경쟁하면 동일 coordinator를 통해 HOLD를 해제한 뒤 시작한다. 기존 payload를 유지해도 잘못된 동시 실행을 계속 허용한다는 뜻은 아니다.

## 2. 변경 파일과 책임

아래 경로는 앞의 두 저장소를 기준으로 한 구현 대상이다. 신규 파일은 아직 만들지 않았다.

| 경로 또는 추가 모듈 | 실제 수정 내용 |
|---|---|
| control-center `wf_odom_bridge.py` | 원시 timestamp·수신 시각·변환 방식 보존, raw/control 출력 분리, TF의 단일 발행, motion worker 연결 |
| control-center `sensor_integration/src/mid360s_laserscan.py`와 `scan_math.py` | 입력 시각·deskew provenance 기록, 밀린 queue를 현재 관측처럼 처리하지 않도록 최신 처리 경계 정리 |
| control-center `sensor_integration/launch/wf_mapping.launch` | 제어용 odom 선택을 명시적인 launch 인수로 제공. raw 기본 경로에서 검증 후 전환 |
| 추가 `local_motion_estimator.py`, `sensor_clock.py` | LiDAR 상대 움직임·odom 예측·시각 품질을 다루는 ROS 독립 계산 모듈 |
| mission `mission_action_server.py`, `ros_runtime.py` | `_goal_callback`, `_cancel_callback`, `_execute`, `_release`를 공통 coordinator에 연결. runtime에서 singleton 생성 |
| 추가 `motion_coordinator.py`, `mission_v2_server.py`, `mission_context.py` | 요청 중복 방지, motion lease, 원래 출발점, 현재 pose와 map revision, HOLD 수명 관리 |
| mission `mission_types.py`, `route_planner.py`, `mission_orchestrator.py` | pose 목적지, 요청별 임시 좌표, 출발 snapshot, 승인 복귀를 표현. legacy HOME 복귀 경로 보존 |
| mission `navigation_executor.py`, `ros_segments.py`, `ros_state.py` | pose goal 실행, deadline·cancel token 전달, 단계별 요구 조건, 결과/실제 도착 구분 |
| floor `ros_node.py`, `ros_runtime.py`, `ros_callbacks.py`, `tag_evidence.py` | 층 초기화와 층 전이를 공통 transaction에서 관리. tag pose와 위치 획득 품질 처리 |
| 추가 `localization_quality.py`, tag survey 설정 | tag의 map상 실제 pose, 카메라 extrinsic, 후보 위치와 불연속 판정 |
| stair `supervisor.py`, `ros_node.py`, `stair_evidence.py` | HOLD 명령 선택, motion handoff, 시간 구간의 정지 판정, 계단 완료/취소 기준 교정 |
| 추가 `hold_controller.py`, `hold_evidence.py`, `collision_check.py` | 고정 목표에 대한 국소 보정, 관측 품질, footprint와 정지 궤적 검증 |
| 추가 mission `device_action.py`, `photo_capture.py` | 문 요청/열림 확인/통과 상태, 촬영 manifest·부분 재시도 |
| launch·CMakeLists·package.xml·기존 tests | 새 interface 생성·의존성·설치 경로·기존 client 회귀를 함께 수정 |

## 3. 요청과 상태의 내부 계약

아래는 추가할 타입의 논리적 구조다. 기존 ROS 메시지에 필드를 덧붙이는 변경안이 아니다. 실제 `.msg/.action/.srv`는 같은 패키지에서 별도 버전으로 생성한다.

```text
PoseTarget:
  floor_id, frame_id, x_m, y_m, yaw_rad
  map_revision                 # 해당 층 지도 내용/좌표계의 식별자

StartSnapshot:
  snapshot_id, mission_id, captured_at
  floor_id, map_revision, map_generation, pose_epoch
  pose_map, pose_local, local_frame_epoch
  localization_source, quality

MissionV2Goal:
  request_id, operation        # navigate / inspect / approved_return
  destination                 # PoseTarget 또는 named location 참조
  photo_targets[]             # pose, heading, capture_profile_id
  hold_profile_id
  return_snapshot_id          # approved_return만 사용

MissionV2Feedback:
  mission_id, segment_id, phase, floor_id
  reason_code, human_message, allowed_recovery_actions[]

MotionLease:
  supervisor_boot_id, coordinator_boot_id, sequence, owner, cancellation_token
  map_generation, pose_epoch
```

`mission_id`는 실행 식별자, `request_id`는 중복 클릭/재접속 식별자다. 같은 ID+같은 내용은 원래 결과를 반환하고, 같은 ID+다른 내용은 충돌로 응답한다. snapshot은 첫 실제 이동 직전의 검증된 현재 pose를 저장하며 이후 진행·메일 재시도로 덮어쓰지 않는다. 출발점으로 돌아갈 때 과거 `map_generation`을 현 세대에 강제로 재사용하지 않고 지도 내용 revision과 좌표계의 호환성을 확인한다. 지도 내용이 바뀌었으면 검증된 좌표 변환 또는 사용자 재확인이 필요하다.

### 노드 사이의 소유권 API

요청 순서는 mission의 coordinator가 정하지만, **실제 권한 발급·폐기와 최종 명령 선택은 transport를 가진 supervisor만 수행**한다. coordinator의 메모리 lock만으로 노드 사이의 배타성을 보장했다고 취급하지 않는다. supervisor에 아래 versioned 서비스와 상태 topic을 추가한다.

```text
AcquireMotion(request_id, coordinator_boot_id, expected_sequence,
              requested_owner, context) -> PENDING / GRANTED(token) / CONFLICT
ReleaseMotion(request_id, token, reason) -> PENDING / RELEASED / STALE
GetMotionState() -> supervisor_boot_id, sequence, owner, activity,
                    pending_request, handoff_state, observation_state
AuthorizeMapChange(transaction_id, token, expected_floor_context)
              -> authorized_context / conflict
```

owner는 NAV/STAIR/HOLD/MAP_SWITCH/NONE이다. 최종 송신 직전마다 현재 token·취소 상태·해당 동작의 관측 유효성을 검사하고, worker 계산 결과에도 시작 token을 붙여 오래된 결과를 폐기한다. Acquire는 기존 owner의 확인된 인계 전에는 GRANTED를 반환하지 않는다. 응답이 유실되면 같은 request를 조회·재시도하고 다음 sequence를 추측해 진행하지 않는다. 인계 timeout은 PENDING/RECOVERY로 남기며 두 owner를 동시에 활성화하지 않는다.

floor 노드는 직접 들어온 legacy/v2 요청도 supervisor의 MAP_SWITCH 권한과 해당 계단 완료 문맥을 확인한다. 기존 stair epoch 필드만으로 재시작 전 실행을 승인하지 않고, supervisor가 보관한 현재 boot의 완료 기록·대상 floor·transaction 연결을 함께 검증한다. 기존 action schema는 유지하되 기존 mission client 구현도 이 내부 인계 API를 사용하도록 수정한다.

coordinator 재시작 시 새 이동은 현재 supervisor 상태를 대조한 뒤 접수한다. 이미 supervisor에서 실행 중인 계단을 웹/mission heartbeat 하나로 강제 중단하지 않는다. supervisor 재시작 시에는 과거 token을 모두 무효화하고 NONE/RECOVERY에서 시작하며 기체 상태 확인 전 과거 속도를 재송신하지 않는다. 동작별 통신 만료 처리와 실제 정지 여부는 별개다. zero 명령을 보냈다는 사실을 물리적 제동·정지의 증거로 쓰지 않는다.

### 영속 기록과 재시작

mission 노드의 단일 JobStore를 SQLite transaction으로 구현하고, 기본 저장 경로는 배포 계정의 `~/.local/state/tron1/mission/` 아래로 제안한다. 경로는 설정 가능하며 현재 디렉터리를 생성한 것은 아니다. request ID/내용 hash, job 상태, immutable snapshot, segment 결과, photo manifest, outbox를 저장한다. 웹 backend는 이 기록의 ID를 참조하며 별도 실행 원장을 만들지 않는다.

접수 ACK 전 request와 job을 commit하고, 첫 이동 전 snapshot을 commit한다. 사진은 임시 파일 기록·완료 확인·원자적 파일 교체 후 manifest를 commit한다. 이 사이에 재시작하면 미완료 파일과 manifest를 대조하며 촬영 성공을 추측하지 않는다. 재시작된 진행 중 job은 RECONCILING이 되고 실제 owner/child/지도 상태를 확인한 뒤 복구 동작을 제시한다. 재시작만으로 이동이나 문 요청을 자동 반복하지 않는다.

메일은 outbox에 PENDING을 기록한 뒤 보내며, 발송 응답 유실은 UNKNOWN_DELIVERY로 보존한다. provider의 request ID 조회/중복 방지가 없다면 exactly-once 전송을 약속하지 않는다. 실패 지점만 재시도한다. 보존 기간이 정해지기 전에는 원장·snapshot·사진을 자동 삭제하지 않는다. 디스크 부족은 촬영/저장 기능의 명시적 실패로 처리하고 기존 사진을 덮어쓰지 않는다. 새로운 출발 snapshot을 저장할 수 없는 임무는 접수 전에 이유를 알린다.

## 4. 임의 출발점에서 임의 목표까지 NAV

현재 `BuildingPlanner.plan()`은 named ID 간 BFS이고, `RosSegmentExecutor._navigation_segment()`는 `self._locations[segment.target_id]`를 조회한다. 이 두 경계에 pose 목표를 지원한다.

1. `MotionContext`에서 현재 층·실제 pose·map revision을 가져온다. 과거 `LogicalAnchor`만으로 현재 위치를 가정하지 않는다.
2. 같은 층 목표이면 `NAVIGATION(current_pose → target_pose)` 하나를 만든다. 시작점 이름과 목표 이름이 같아도 실제 pose 오차로 판단한다.
3. 다른 층 목표이면 기존 방향별 stair graph에서 연결 가능한 계단 경로를 구한다. 현재 위치에서 첫 entry까지 NAV를 붙이고, 최종 층 출구에서 목적지까지 NAV를 붙인다. 기존 graph를 HOME까지 우회하는 강제 경로로 사용하지 않는다.
4. 임시 출발·목표는 요청별 immutable 공간에 저장한다. 공유 `_locations` 사전을 수정해 다른 임무의 좌표를 바꾸지 않는다.
5. `NavigationExecutor.execute()`는 기존 named Location도 내부의 동일한 PoseTarget으로 변환해 처리한다. 목표 frame·floor·map revision·유한값을 확인하고 기존 move_base client로 발행한다.
6. `_await_terminal()`에 전체 deadline과 취소 관측을 넣는다. deadline이 지나면 cancel/reconcile 상태로 들어가며 아직 살아 있는 child를 남기고 새 goal을 보내지 않는다. 상태를 확인하지 못하면 설명 가능한 recovery 상태를 내보내고 과거 goal을 자동 재발행하지 않는다.

진입 NAV의 실패 원인은 현재 bag으로 확정되지 않았다. 최초 재현에서는 global/local path, costmap/footprint, AMCL·odom·TF, NAV 요청·최종 normalized 명령·기체 피드백을 동시 관찰한다. 단위를 포함한 변환 오차, 목표 방향, 저속 deadband, 장애물 판정, timeout 중 실제 원인에 해당하는 부분만 수정한다. 실제 자유 공간으로 연결되지 않은 목표에는 NO_ROUTE 이유와 위치를 반환한다.

## 5. 센서 시각과 국소 위치 추정의 구현

### 시각 처리

`SensorClock.observe(source_stamp, receive_ros_time, receive_monotonic)`가 source clock domain, offset/drift 추정 상태, timestamp 유효성, 재시작 여부를 반환한다. 원시 값을 잃지 않는다. 임의로 수신 시각으로 바꾼 값을 원래 측정 시각처럼 표시하지 않는다. 과거 데이터의 도착 간격과 센서 header 간격을 별도로 기록한다. queue는 bounded이며 제어용 상대 정합은 최신 유효 관측을 처리하고, 전체 원본 보존은 recorder가 담당한다.

센서 시각 offset 추정은 지연과 구별되지 않을 수 있다. 동기화 근거가 없으면 quality를 불명확으로 표시하고 출처를 보존한다. 메시지 도착이 빠르다는 이유로 실제 sample age가 작다고 주장하지 않는다.

### 국소 위치 계산

기존 bridge 내부에 낮은 우선순위 worker를 둔다. WebSocket 수신·원시 publish를 LiDAR 정합 계산으로 막지 않는다.

```text
1. raw odom 상대 증분으로 다음 pose를 예측한다.
2. raw /livox/lidar를 실제 point 시각과 extrinsic으로 해석한다.
3. 가까운 고정 구조를 국소 keyframe과 정합해 상대 pose 관측을 만든다.
4. 정합 잔차·대응 범위·퇴화 방향·시각 품질을 함께 평가한다.
5. 관측 가능한 방향만 예측을 보정하고 공분산과 provenance를 출력한다.
6. 고정된 HOLD 기준은 반복 정합 때 현재 위치로 다시 저장하지 않는다.
```

정합은 먼저 voxel downsample된 raw point 기반의 제한된 SE(2)/SE(3) 후보로 오프라인 검증한다. 정지 구간은 deskew 없는 raw cloud와 RGB도 대조하고, 이동 구간 deskew에 쓰인 odom/gyro는 관측 독립성 문맥에 표시한다. 복도에서 한 방향이 불분명하면 그 방향 정합을 신뢰한 작은 공분산을 만들지 않는다. 평지용 2D 정합을 계단 구간의 자세·진행 증거로 강제 적용하지 않는다.

**2026-09-20 추가 확인:** 현재 `/scan` 변환은 `wf_mapping.launch`가 연결한 raw odom을 deskew에 사용한다. 따라서 기존 보고서의 scan 정합을 raw odom과 완전히 독립된 근거로 해석하면 안 된다. 기존 31.6cm 사례의 숫자는 그대로이며 별도 RGB 관측도 존재하지만, 국소 추정기 승인에는 raw LiDAR·외부 실측 대조를 추가한다.

배포는 두 단계다. 먼저 `SHADOW`에서 추정 결과만 기록하고 기존 TF/주행 입력은 바꾸지 않는다. 검증 후 `CONTROL`에서 제어용 출력에 연결한다. 출력 계약은 다음과 같이 제안한다.

| mode·출력 | header.frame_id / child_frame_id | TF·소비자 |
|---|---|---|
| SHADOW `/tron/wheel_odom_raw` | 기존 `odom / base_Link` | bridge가 기존 odom→base_Link 발행. 기존 소비자 유지 |
| SHADOW `/tron/odom_shadow` | `odom_shadow / base_Link` | TF 발행 없음. 평가·기록만 사용 |
| CONTROL `/tron/wheel_odom_raw` | `odom_raw / base_Link` | raw 측정 기록과 명시적 raw 소비자 전용. odom_raw→base_Link TF를 추가 발행하지 않음 |
| CONTROL `/tron/odom_control` | `odom / base_Link` | bridge만 odom→base_Link 발행. move_base·floor·mission·supervisor의 제어 pose 소비 경로를 함께 전환 |

raw 수치·단위·원시 시각의 의미는 보존한다. 그러나 CONTROL에서는 원시 누적 좌표와 제어 좌표가 다르므로 **raw의 frame_id를 명시적으로 분리하는 호환 변경이 필요**하다. 이 변경을 숨겨 기존 raw 메시지와 제어 TF를 동일한 pose로 해석하게 만들지 않는다. raw frame은 TF 주행 경로로 사용하지 않으며 두 좌표계가 고정된 관계라고 가정하는 static TF도 만들지 않는다. raw 소비자 목록과 frame 가정을 전부 대조하고, 함께 바꿀 수 없는 소비자가 있으면 CONTROL 전환을 보류한다.

map→odom은 기존 AMCL의 책임, odom→base_Link는 bridge의 책임으로 유지한다. 제어 odom은 mode 진입 시 원점을 정렬하고 하나의 local_frame_epoch를 부여한다. 재시작·reset은 epoch를 바꾸며 기존 local HOLD 목표는 자동 재사용하지 않는다. mode 전환은 임무 중 하지 않고 정리된 시점에 수행한다. TF와 odom pose/twist의 시각·축·원점·선택 mode 일치를 배포 시험에서 확인한다.

IMU raw topic은 유지하고 소비 경계에서 g 단위와 SI 단위를 명시적으로 구분한다. quaternion (0,0,0,0)은 orientation 관측으로 사용하지 않는다. 모든 소비자를 확인해 중복 단위 변환을 피한다. IMU를 적분하는 것만으로 cm 위치가 얻어진다고 가정하지 않는다.

## 6. 현재 층 적용과 AMCL 오인식 처리

추가 `InitializeFloor` action은 floor_id, request_id, optional pose_hint와 그 출처를 받는다. 기존 계단용 `FloorTransition`과는 요청 의미가 다르지만, 같은 floor transaction mutex·map loader·시각 경계·취소 처리 코드를 재사용한다.

```text
QUIESCE_MOTION → APPLY_MAP → ACQUIRE_POSE → VERIFY_POSE → COMMIT_CONTEXT
```

`QUIESCE_MOTION`은 mission coordinator를 통해 기존 이동을 정리한다. 계단 중에는 화면 클릭만으로 지도나 소유권을 바꾸지 않는다. 이미 검증된 같은 층·같은 지도 재선택은 불필요한 reset을 하지 않는다. 지도 미리보기 API는 운용 `/map`을 바꾸지 않는다.

현재 tag 설정에는 ID·층·크기·용도가 있지만 map상 tag pose는 없다. 별도 survey에 `floor_id, map_revision, tag_id, T_map_tag, survey_uncertainty`를 저장하고 카메라 extrinsic을 확인한다. `T_A_B`를 B좌표를 A좌표로 옮기는 변환으로 정의할 때:

```text
T_map_base = T_map_tag × inverse(T_camera_tag) × inverse(T_base_camera)
```

태그의 평면 pose 모호성·가림·오검출 가능성은 여러 관측·geometry·다른 위치 근거와 함께 확인한다. 단일 ID 검출만으로 정확한 pose를 설정하지 않는다. 그 pose로 AMCL을 초기화한 뒤 scan/map 일치와 시간에 따른 움직임 일관성을 확인한다. scan 일치는 같은 구조의 다른 모서리도 맞을 수 있으므로 유일한 전역 정답 증거로 쓰지 않는다.

태그가 없을 때는 검증된 최근 위치와 연속 이동 이력이 있으면 추적을 유지한다. 임의로 들어 옮긴 로봇은 과거 위치를 그대로 확정하지 않는다. 시작 위치의 대칭 후보를 센서로 구분할 수 없으면 지도에서 위치·방향을 지정하는 보조 UI 또는 추가 위치 기준이 필요하다. 보조 UI 허용과 tag 설치는 사용자 답변 대기다. 이 조건에서 완전 자동 위치 획득을 이미 보장하지 않는다.

불연속 판정은 이전 전역 pose에 국소 상대 움직임을 적용한 예측과 새 전역 pose의 차이를 비교한다. map 변경·initialpose·odom reset을 명시적인 사건으로 먼저 구분한다. 설명되지 않는 불일치만 재획득 대상으로 만든다. 전체 NAV의 매 tick마다 태그나 AMCL 새 표본 수를 요구하지 않는다.

### 지도 적용 뒤 실패·취소된 경우

층 상태에는 `requested_floor`, `loaded_map_revision`, `committed_floor`, `map_generation`, `pose_epoch`, `transaction_state`를 별도로 둔다. 지도 적용을 시작하기 전에 새 generation과 전이 상태를 기록하고 이전 READY를 철회한다. APPLY_MAP 이후 pose 획득 실패이면 실제 적용된 map과 `POSE_REQUIRED`를 표시하고, 이전 floor를 READY인 것처럼 반환하지 않는다. 이전 확정 floor는 이력으로만 보존한다.

이때 복구는 적용된 지도에서 위치 확인을 재시도하거나, 사용자가 이전 층 복원을 선택하면 이전 map을 새 transaction으로 로드하고 위치까지 다시 검증하는 것이다. map만 되돌리고 예전 pose를 재사용해 READY로 처리하지 않는다. 취소 중 map 호출이 아직 진행 중이면 CANCEL_PENDING으로 유지하며 loader 완료·늦은 응답을 정리하기 전 다음 map 적용을 겹치지 않는다. 응답 유실로 실제 적용 map을 모르면 MAP_UNKNOWN이며, 관측한 지도 내용과 load 완료를 대조하거나 명시적 재적용으로 확인한다. 알 수 없는 상태를 추측해 commit하지 않는다. 이 상태는 map 기반 NAV의 문맥 문제이지 메일·사진 조회 같은 기능까지 막는 전역 장애가 아니다.

## 7. HOLD의 실제 제어와 명령 인계

선택안은 **기존 stair_supervisor 안의 HoldController**다. 현재 설정의 NAV 허용 오차를 전체적으로 줄이지 않고 HOLD의 목표·관측·제어 수명을 분리한다. 별도 속도 publisher나 로봇 socket을 추가하지 않는다. 정밀도·저속 응답·장착 조건은 현장 검증 전 UNVERIFIED다.

### 고정 목표

`HoldTarget`에는 요청한 map 목표, 대응 local 목표, floor/map revision, local frame epoch, 허용 반경과 선택적 목표 yaw를 저장한다. 실제 도착 위치를 목표로 몰래 치환하지 않는다. NAV 허용 오차 안에 도착했어도 HOLD 목표와의 차이가 크면 진입 시 보정을 수행할 수 있다. 좌표계 reset이 발생하면 물리적 목표 연결을 다시 확인한다.

HOLD 진입 시각 `t0`에 유효한 동일 시각의 변환으로 `T_local_target = inverse(T_map_local(t0)) × T_map_target`을 계산한다. 변환 시각이 맞지 않거나 초기 전역 위치가 아직 불명확하면 목표 결합을 완료하지 않는다. HOLD 중에는 이 local 목표와 local 관측을 사용해 지정한 물리적 지점을 유지한다. 매 AMCL 갱신마다 local 목표를 다시 계산하지 않는다.

AMCL의 작은 정상 보정은 map 표시와 목표 간 잔차로 기록한다. 설명되지 않는 큰 변경은 GLOBAL_RELOCALIZATION_REQUIRED로 표시하고, local 관측·frame이 유효한 범위의 유지와 map 기반 새 이동 가능성을 구별한다. 검증된 재위치 획득이 이전 map 결합이 잘못됐음을 밝히면 `TARGET_CONTEXT_CHANGED`를 반환한다. 다른 물리적 위치가 된 map 좌표를 향해 HOLD가 자동 추격하지 않는다. 새 NAV 요청 또는 재목표 확인으로만 변경된 목표에 이동한다. 외부 기준이 없는 장시간 local drift까지 해결했다고 주장하지 않으며 P04의 장시간 시험으로 허용 유지 범위를 정한다.

### 상태와 계산

```text
IDLE → MONITORING → CORRECTING → MONITORING
                 ↘ OBSERVATION_UNCERTAIN / PATH_BLOCKED
새 유효 명령 → YIELDING → IDLE
명시적 정지 → DISARMED
```

제어 주기는 기존 supervisor의 송신 timer와 분리된 계산 주기로 구성하되 같은 transport에서 최종 한 명령만 송신한다. 초기 실험의 계산 주기는 10~20Hz 후보이며 검증된 sensor age와 실제 CPU 지연으로 정한다.

```text
e = inverse(T_local_base_now) × p_local_target
rho = hypot(e.x, e.y)
alpha = atan2(e.y, e.x)
```

`rho > r_on`이 유효한 실제 이탈 관측 구간에서 지속되면 보정하고, `rho < r_off`에서 보정을 끝낸다. `r_off < r_on`으로 하여 경계에서 왕복하지 않게 한다. 정밀도가 낮은 방향은 신뢰 가능한 이탈이라고 판정하지 않는다. r_on/r_off·yaw 오차·속도·가속도·관측 지속 시간은 파라미터로 두되 지금 운용값을 임의 확정하지 않는다.

보정 시 x/yaw로 가능한 저속 `(v, omega)` 후보를 만들고, 예측한 짧은 궤적과 정지 궤적 전체를 footprint로 검사한다. 비용은 목표 거리·목표 방향·이전 명령과의 차이로 정한다. 정면 목표에는 작은 전진, 큰 방향 오차에는 허용된 회전 또는 검증된 후진 경로를 선택한다. 측면 이동 명령을 지원한다고 가정하지 않는다. 후보가 없으면 PATH_BLOCKED와 이유를 보낸다.

충돌 검사는 현재 footprint·장애물 관측·좌표계·시각을 공유한다. global planner의 비용 숫자를 단순 비교하는 것으로 충돌 검증을 대신하지 않는다. 회전 시 외곽과 제동 중 궤적까지 확인한다. 낭떠러지나 계단 끝의 지지면은 평면 costmap만으로 증명할 수 없어 확인된 평면 대기 범위에서 먼저 검증한다. 그 범위를 벗어나는 기능 확장은 별도 관측·실측 대상이다.

### 새 명령 우선 처리

`MotionCoordinator`는 새 요청을 먼저 검증한다. 유효하면 접수 순서를 예약하고 supervisor에 HOLD 해제와 새 owner 인계를 요청한다. supervisor가 이전 계산 결과를 폐기하고 handoff를 확인한 뒤 새 token을 발급한다. 이 과정에서 mission worker나 action handle을 HOLD 때문에 영구 active로 남기지 않는다. 명시적 정지는 자동 보정도 해제한다.

현재 `/navigation/cmd_vel`은 stamp/goal identity가 없는 Twist다. 수신 당시의 epoch를 붙이는 것만으로 네트워크에서 늦게 도착한 과거 명령의 출처를 증명할 수 없다. HOLD/STAIR 소유 중 NAV 입력은 최종 명령으로 선택하지 않으며, NAV 재개는 child 종료·기존 입력 폐기·새 goal의 handoff 절차를 거친다. **임의로 지연된 과거 Twist까지 엄밀히 배제하는 계약이 필요하면 move_base 프로세스 내부의 stamped command adapter/plugin이 추가로 필요하다.** 이는 조건부 구조 변경으로 별도 검토하며 현행 Twist만으로 완전한 epoch 보장을 약속하지 않는다.

adapter가 없는 단계에서 보장하는 것은 동시 owner 배제와 supervisor가 이미 수신한 구 명령·구 worker 결과의 폐기까지다. 새 NAV owner 이후 처음 도착한 Twist가 새 goal에서 생성됐다고 증명할 수는 없다. 단순 topic relay가 수신 시각을 찍는 것도 이 문제를 해결하지 않는다. 자동 운용의 엄밀한 command fencing을 인수 조건으로 삼을 경우, 생산 지점에서 token을 붙이는 변경을 구현 범위에 포함하고 legacy Twist는 관측용으로 남겨야 한다. 해당 변경 없이 그 인수 항목을 PASS로 표기하지 않는다.

## 8. 계단 supervisor 변경

기존 VERIFY_ENTRY → ALIGN → FORWARD_SEGMENT_1 → LANDING → TURN_TO_NEXT_FLIGHT → FORWARD_SEGMENT_2 → EXIT_CONFIRM을 유지한다.

- `ALIGN`: 현재는 새 odom 하나를 alignment 완료로 인정한다. 목표 방향과 관측 방향의 실제 오차를 평가하고 제한된 회전 보정의 결과를 확인하도록 바꾼다.
- `FORWARD_SEGMENT_*`: 검증된 계단 profile의 전진과 yaw 보정을 사용한다. 기체 stair firmware의 자세 제어와 상위 yaw/진행 제어의 책임을 먼저 확인한다. 평면 odometry의 z=0이나 0 IMU quaternion으로 기울기를 판단하지 않는다. 측면 오차 보정은 실제 관측·기체 응답이 검증된 범위에서만 추가한다.
- `LANDING/EXIT_CONFIRM`: 한 샘플의 작은 이동을 정지로 보지 않고 시간 구간의 전체 위치 범위·yaw 범위·속도와 유효 관측 지속을 확인한다. 윈도 안에 기록 공백이 있으면 그 공백을 정지 시간으로 세지 않는다. odom 정지 판정만으로 실제 착지를 증명하지 않고 현장 영상/지지면/등록된 도착 관측과 대조한다.
- `TURN_TO_NEXT_FLIGHT`: 계단참 범위에서 목표 yaw까지 보정한다. 새 주행 구간의 기준점을 완료 시점에 확립한다.
- `cancel`: LANDING이라는 phase 이름만 보고 `_finish()`하지 않고 확인된 checkpoint 조건을 검사한다. 취소 요청과 안전하게 명령 소유를 인계한 완료를 구별한다.
- `FLOOR_TRANSITION`: 기존 goal의 `stair_ownership_epoch`를 실제로 전달·검사하고, 해당 계단 실행 결과와 대상 층의 증거를 연결한다. 실패 후 늦게 도착한 map 변경도 정리해야 재시도한다.

STAIR 중 평면 HOLD는 개입하지 않는다. 목표 층의 출구·평면 위치를 확정한 뒤 HOLD를 시작한다. UP/DOWN은 각각의 profile과 검증 결과를 사용한다. 3F↔4F의 통과가 5F↔RF의 통과 증거가 되지 않는다.

## 9. 자동문·촬영·메일·복귀의 실행 단위

### 자동문

주소를 route나 브라우저 입력에 직접 넣지 않고 `device_id`를 장치 설정에 연결한다. `DEVICE_ACTION`은 `request_id, device_id, action, expected_state, deadline, direction`을 받는다.

```text
NAV_TO_WAIT_POINT → OPEN_REQUESTED → OPEN_CONFIRMED → PASSAGE → PASSED
```

HTTP 응답 성공과 실제 열린 상태를 구별한다. request ID 또는 장치 조회 계약으로 응답 유실 뒤 중복 요청을 처리한다. 문이 계단 내부에 있으면 평지 segment로 위장하지 않고 해당 phase에서 필요한 hook을 보고한다. 통과 중 닫힘과 복귀 방향의 동작도 별도로 정한다. 위치/IP/인증/열림 피드백은 답변 없이 만들어 넣지 않는다.

### 촬영과 메일

각 촬영점은 `PoseTarget + heading + capture_profile`이다. 순서는 `NAV → HOLD/촬영 안정 확인 → 새 frame 촬영 → artifact commit`이다. 도착 전에 찍힌 frame을 최신 사진으로 저장하지 않는다. 파일 저장 완료와 checksum·촬영 시각·mission/point ID를 manifest에 확정한 뒤 해당 지점 완료로 표시한다.

일부 사진 실패는 해당 지점만 실패로 보존하고 이전 사진·출발 snapshot을 유지한다. 마지막 지점에서 HOLD하면서 메일 outbox를 등록한다. 메일은 사진 artifact ID를 참조하고 이동을 재실행하지 않는다. provider 접수·배달 확인 가능 범위를 구분하며 응답 유실을 무조건 재전송해 중복 메일을 만들지 않는다.

### 승인 복귀

`POST /jobs/{id}/return`의 승인 요청을 받은 때 현재 위치부터 `StartSnapshot`까지 새 route를 계산한다. HOME으로 치환하지 않는다. 출발 당시 지도 revision이 유효한지와 실제 도달 가능성을 확인한다. 돌아갈 위치가 막혔으면 임의의 가까운 곳으로 바꾸지 않고 이유·선택 가능한 복구를 보여준다. legacy `return_after_task`는 기존 계약으로 유지한다.

## 10. 웹 API와 디버깅 계약

프레임워크는 기존 웹앱 위치 확인 후 맞춘다. API의 의미는 다음으로 고정한다.

```text
GET  /robot/state
GET  /floors/{floor}/map                  # 미리보기
POST /robot/initialize                   # 현재 층 적용, optional pose hint
POST /jobs                              # pose NAV, 층 이동, 다지점 촬영
GET  /jobs/{job_id}
POST /jobs/{job_id}/cancel
POST /jobs/{job_id}/return               # 명시 승인
GET  /events                            # SSE/WebSocket 중 기존 앱 방식 재사용
```

응답은 request_id/job_id, 현재 단계, 실패 reason_code, 가능한 다음 동작을 포함한다. 브라우저 재접속은 기존 job을 다시 조회하며 새 goal을 만들지 않는다. UI 버튼별로 임무 엔진을 새로 만들지 않는다.

상태 변화 로그 예시는 다음과 같다. 숫자는 실제 실행 시 채우며 예시를 운용값으로 사용하지 않는다.

```text
job_id, mission_id, segment_id, child_goal_id
floor_id, map_revision, map_generation, pose_epoch, motion_lease
source_stamp, receive_stamp, clock_quality
phase, reason_code, measured_value, configured_limit
nav_requested, command_selected, websocket_sent, robot_feedback
artifact_id, device_request_id, mail_attempt_id
```

UI에는 ‘4층 위치 확인 중’, ‘도착·위치 유지 중’, ‘문 열림 확인 중’, ‘사진 2/4’, ‘메일 재시도’처럼 실제 단계를 표시한다. 개발자 화면에는 위 측정·제한·선택한 명령을 연결한다. ‘연결 정상’과 ‘실제 위치 유지 성공’을 같은 상태로 표시하지 않는다.

## 11. 조건의 최소 적용 범위와 복구

새 조건에 KEEP은 부여하지 않았다. 아래는 구현 방향이며, 수치의 최소성과 실물 조건은 검증 후 확정한다.

KEEP은 구체 hazard·현실 근거·최소 적용 범위·더 좁은 대안의 불충분성·오탐 복구가 모두 증명된 조건 유지, NARROW는 필요한 동작으로 적용 범위 축소, TUNE은 관측·시간·수치 기준 조정, UNVERIFIED는 실물 또는 외부 계약 확인 전 미확정이라는 뜻이다.

| 상황 | 적용 범위 | 정상 주행을 되살리는 경로 | 판정 |
|---|---|---|---|
| tag가 안 보임 | 초기 절대 위치가 필요한 때 | 유효한 기존 추적을 계속 사용하거나 보조 위치 입력 | NARROW |
| 메일/사진 실패 | 해당 작업 결과 | 이동 결과 보존, 실패 단계만 재시도, 승인 복귀 가능 | NARROW |
| stale LiDAR/odom | 그 관측을 사용하는 동작 | 시각·입력 회복 후 현재 상태 재확인, 구 command 폐기 | TUNE |
| HOLD 관측 불확실 | HOLD 보정 | 재관측 또는 확인된 새 NAV 명령으로 인계. 유지 성공 허위 표시 없음 | TUNE |
| 계단 불완전한 착지 | 해당 계단의 소유 인계 | 실제 checkpoint·기체 상태 확인 후 복구 | UNVERIFIED |
| 문 개방 미확인 | 그 문 통과 단계 | 상태 조회·재요청·허용된 대체 경로 | UNVERIFIED |

FAULT 복구는 같은 닫힌 RobotTransport 인스턴스에 명령을 다시 넣지 않는다. 확인된 상태에서 새 세션·새 epoch를 만들고 과거 pending/desired command를 폐기하는 경로를 기존 owner에 추가한다. 새 세션 생성만으로 실제 정지를 확인했다고 간주하지 않는다.

## 12. 구현 묶음과 검증

1. **관측·회귀 묶음:** 시각 provenance, mission/goal/송신 연계, 기존 지연·취소·정지 판정 반례를 고정한다. 기존 21개 bag은 관측 회귀 자료로 사용한다. 실제 행동 변화는 기록만으로 PASS 처리하지 않는다.
2. **접수·임의 좌표 묶음:** MotionCoordinator, MissionV2, PoseTarget, snapshot, InitializeFloor를 구현한다. 기존 client와 v2 동시 요청·준비 중 cancel·중복 request·map 변경 경합을 통합 검증한다.
3. **위치 관측 묶음:** LocalMotionEstimator SHADOW, 시각 교정, tag pose 획득을 검증한다. 실제로 정지시킨 기준과 이동/미끄럼 기준을 외부 측정으로 확인한다. 제어용 odom 연결은 이 결과를 바탕으로 별도 전환한다.
4. **평지 NAV·HOLD 묶음:** 동일 층 임의 목표, 좁은 구간, 도착 오차, 노이즈만 있는 유지, 실제 밀림, 측면 밀림, 장애물, 보정 중 새 명령을 검증한다.
5. **3F↔4F 묶음:** 기존 phase별 이동·정지·회전·취소·층 전이·승인 복귀를 검사한다. 실제 출발→도착→출발 지점을 독립적으로 확인한다.
6. **5F↔RF·작업 묶음:** 문→촬영점 목록→메일→최종 지점 대기→승인 복귀를 단계별 실패·재시도와 함께 검증한다.

각 묶음은 source·devel·install 및 실제 배포 호스트의 동일성을 확인한다. legacy 메시지 정의·목표/결과/취소·단일 TF/command owner·불필요한 gate 차단 여부를 회귀한다. 변경 전후 Safety와 Mobility를 따로 평가한다. 위치 확인·hold control·stair support 관측을 자기 출력만으로 채점하지 않는다.

인수 시나리오는 [기존 계획의 P01~P15](IMPLEMENTATION-PLAN-DRAFT.md)를 사용한다. 이번 문서는 그 구현 세부를 보강하며 기존 실행 결과를 바꾸지 않는다.

| 구현 묶음 | 필수 인수 연결과 통과 조건 |
|---|---|
| 관측·회귀 | P11~P13/P15: 무관 기능의 NAV 차단 0건, 취소 이후 새 child 없음, 시각/설치 출처 확인 |
| 접수·임의 좌표 | P01/P02/P05/P12/P14/P15: 실제 pose 출발, request 중복 실행 없음, snapshot 불변, 지도/owner 경합 및 재시작 복구 |
| 위치 관측 | P01/P02/P13: 다른 모서리 오답을 참으로 승인하지 않음, 실제 이동과 odom drift를 외부 측정으로 구별 |
| 평지 NAV·HOLD | P02/P04/P11~P15: 범위 내 불필요 보정 없음, 실제 이탈 복원, 장애물/좌표 jump/새 명령 인계 확인 |
| 3F↔4F | P03/P05/P12/P13: 실물 진입·착지·회전·출구·층 확인, checkpoint 취소, 실제 출발점 복귀 |
| 5F↔RF·작업 | P06~P10: 실제 문 개방·통과, 지점별 새 사진, 부분 실패 보존, 메일 장애 중 복귀 가능 |

## 13. 확정 전 필요한 입력

이미 질문한 HOLD 시작 거리·방향 유지, 위치 지정 보조 UI 허용, tag 측량·추가 설치 가능 여부는 미답변이다. 이 값은 parameter/API로 분리해 공통 접수·기록·기존 결함 수정과 독립적으로 진행할 수 있다. 다만 검증하지 않은 값으로 실장비 보정을 시작하지 않는다.

웹앱 위치, 자동문 통과 위치와 API/열림 확인, 촬영점·메일 서비스는 해당 통합 단계 전에 확정한다. 자동문 IP·인증이나 물리 센서 지원을 추측하지 않는다. 기존 구조 수정 보고는 본 문서로 구체화했으며, 이번 턴에 코드 적용·새 ROS node·실장비 움직임·장치 요청은 수행하지 않았다.
