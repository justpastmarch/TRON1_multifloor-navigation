# TRON 주행 콘솔 HTTP API

코드 기준: 2026-10-16. 기존 mission-manager 안의 `ConsoleServer` / `RosConsole` 구현을 정리한 명세다. 아래 조건은 저장소가 정의한 동작이며, 실장비에서 도착·정지·문 개방·메일 수신을 보증하는 규약이 아니다. 이 문서와 OpenAPI는 현재 HTTP 계약을 다룬다. 날짜가 붙은 기존 운영 기록은 당시 구현의 기록으로 유지한다. 기계가 읽는 스키마는 [console-api.openapi.json](console-api.openapi.json)에 있다.

## 1. 연결 및 공통 규약

- 기본 주소: `http://127.0.0.1:8765`. 포트는 실행 설정에 따라 다르다. 서버는 loopback에 바인딩한다.
- 허용 Host: 실제 서버 포트의 `127.0.0.1:<port>` 또는 `localhost:<port>`. 다른 Host는 GET/POST 모두 403이다.
- GET `/api/state`에서 받은 `token`을 모든 POST의 `X-TRON-Token` 헤더로 보낸다. 서버 재시작 시 토큰이 바뀐다. 토큰은 진단 내보내기나 영구 저장에 넣지 않는다.
- Origin이 있는 POST는 `http://<Host>`와 일치해야 한다. CORS나 원격 접속 허용 기능은 이 API에 없다.
- POST 본문은 1~16,384바이트의 JSON object여야 한다. `{}`를 받는 명령도 빈 본문을 보내면 안 된다. 응답은 JSON이며 `Cache-Control: no-store`다.
- POST 경로는 정확히 일치해야 한다. 쿼리나 trailing slash를 붙이지 않는다. GET은 URL의 path로 분기한다.
- 200은 HTTP 요청의 처리 결과다. `submitted`, `accepted`, `pending`, 실제 진행/결과 상태를 구분한다. 이동 시작 응답은 도착 응답이 아니다.
- 자동 재시도·멱등키·API 버전 협상은 없다. 시작/복귀/전이/촬영/문 열기/메일 재전송이 타임아웃되면 상태를 확인한 뒤 사용자가 판단한다. HTTP 대기 종료는 실행 취소가 아니다.
- 서버는 HTML을 시작 시 한 번 읽는다. 파일 배포 후 운영 화면 반영에는 기존 스택 재시작이 필요하다.

| HTTP | 의미 | 본문 |
|---|---|---|
| 200 | 처리됨. 명령별 상태 필드를 확인 | 해당 응답 object 또는 `/`의 HTML |
| 403 | Host, Origin 또는 토큰 불일치 | `{"error":"..."}` |
| 404 | 알려지지 않은 경로 | `{"error":"not found"}` |
| 409 | 잘못된 JSON/자료형/크기, 현재 상태에서 불가, 서비스 거절 등 `ValueError/KeyError/TypeError` | `{"error":"이유"}` |
| 500 | 그 외 처리 예외 | `{"error":"요청 처리 실패. ROS 로그를 확인하세요."}` |

## 2. 전체 경로

선택 필드는 `?`로 표시한다. 숫자는 유한한 JSON number를 뜻한다. `record?`는 생략 시 false이며 반드시 boolean이다.

| 메서드·경로 | 요청 | 정상 응답 핵심 | 부작용 |
|---|---|---|---|
| GET `/` | 없음 | HTML | 없음 |
| GET `/api/state` | 없음 | State + token | 없음 |
| GET `/api/map` | 없음 | MapImage 또는 `{error}` | 없음 |
| POST `/api/plan` | `{destination, kind?}` 또는 아래 지도 좌표 요청 | Plan: segments, blockers, can_start | 경로/준비 판정. 주행·녹화 없음 |
| POST `/api/start` | 일반: `{destination, kind?, record?}`; 지도 좌표 요청; 계단 재개: `{kind:"stair_resume",resume_route_id,resume_run_id,resume_revision,resume_phase,record?}` | `{submitted,destination,recording}` 또는 `{submitted:false,cancelled:true}` | 필요 시 녹화 후 `/mission` 목표 전송 |
| POST `/api/photo/roof_start` | `{record?}` | start와 같음 | 현재 RF에서 촬영 순회 시작 |
| POST `/api/return` | `{record?}` | start와 같음 | 저장된 촬영 임무 출발 위치로 복귀 시작 |
| POST `/api/cancel` | `{}` | `{cancel_requested:true,note}` | 이 콘솔의 임무 취소, 수동 입력 종료, hold 해제, 녹화 저장 요청 |
| POST `/api/relocate_flat` | `{floor_id,confirmed_flat_ground:true}` | `{accepted:true,pending:true,note}` | 이 콘솔 임무 종료 → NAV 준비 → 선택 층 지도 요청. 자동 출발 없음 |
| POST `/api/select_floor` | `{floor_id}` | `{accepted:true,note}` | hold 중지 후 실제 층/지도 선택 |
| POST `/api/localization/pose_estimate` | `{floor_id,map_revision,x,y,yaw}` | `{submitted:true,previous_request_id,previous_status,note}` | 현재 지도에서 `/initialpose` 요청 |
| POST `/api/recover_control` | STAIR: `{confirmed_flat_ground:true,confirmed_physical_relocation:"평지 이동 완료",confirmed_run_id}`; 이미 NAV: `{confirmed_flat_ground:true}` | `{accepted:true,note}` 또는 `{accepted:false,pending:true,note}` | 로봇을 계단에서 물리적으로 평지로 옮긴 뒤에만 인계 서비스 호출 |
| POST `/api/operator_phase` | `{kind?,run_id,revision,phase,confirmed:true}` | `{accepted:true,...result}` | 기존 계단 실행에 전이/이어가기 요청 |
| POST `/api/manual/begin` | `{}` | `{lease,server_time,note}` | 기존 supervisor에서 수동 입력 lease 획득 |
| POST `/api/manual/update` | `{lease,sequence,forward,turn,issued_at}` | `{accepted:true}` | lease 내 수동 입력 전송 |
| POST `/api/manual/end` | `{lease}` | `{accepted:true,note}` | 수동 입력 종료. 자동 임무는 유지 |
| POST `/api/mail/settings` | `{recipient}` | `{saved:true,recipient,note}` | 다음 촬영에 쓸 수신 주소 저장 |
| POST `/api/photo/capture_mail` | `{recipient}` | `{accepted:true,id,recipient}` | 새 RGB 촬영 및 메일 큐 등록 작업 시작 |
| POST `/api/mail/retry` | `{id,confirm_unknown?}` | `{queued:true}` | FAILED/UNKNOWN 발송을 다시 큐에 넣음 |
| POST `/api/door/open_once` | `{device:"F5"|"RF"}` | `{accepted:true,note}` | 선택한 문에 비동기 개방 명령 1회 전송. F5 30초, RF 60초 |
| POST `/api/door/return` | `{}` | `{accepted:true,note}` | 이전 클라이언트 호환용. RF 층 확인 후 RF 문을 1회 요청. 주행 없음 |
| POST `/api/door/stop` | `{}` | `{note}` | 이전 클라이언트 호환용. 이미 접수된 개방 유지는 취소할 수 없음 |

## 3. 상태 및 지도 스키마

### State

| 필드 | 자료형·의미 |
|---|---|
| token | string. 현재 HTTP 서버의 POST 토큰 |
| mode | 실제 어댑터는 `live`. 프런트엔드 시험용 mock은 `preview`를 사용할 수 있음 |
| locations | `[{id,floor,x,y,yaw}]`. x/y m, yaw rad. 임무 목적지는 id로 전달 |
| anchor | string. orchestrator의 확인된 위치 id. 지도상 연속 좌표와 구분 |
| active / observed_active | boolean. 이 콘솔 임무 / 최근 2초 내 `/mission/status`에서 본 전체 활성 임무 |
| starting / recovering / floor_selecting / relocating | boolean. 명령 준비·제어권 복구·층 선택 처리 중 |
| relocation | `{state,floor_id,detail,nav_ready?}`. PREPARING/DONE/FAILED. DONE은 지도 준비 요청 완료이고 위치 READY와 별개 |
| progress | `{mission_id,state,current_floor,segment_type,segment_index,segment_count,target_id}`. 인덱스는 0부터. 비활성이면 마지막 수신 feedback일 수 있음 |
| result | object 또는 null. `{result_code,reason,mission_id,artifact_path,action_status?}`. 과거 결과일 수 있음 |
| telemetry | key별 `{age_sec,value}`. age는 서버 monotonic 수신 후 경과 초. 메시지 원본 timestamp의 age와 별개 |
| navigation | 아래 Navigation |
| recording | 아래 Recording |
| manual | `{available,active,policy,timeout_sec}`. policy=`manual_priority_keep_mission`, timeout_sec=0.35 |
| hold | `{armed,detail}`. 도착 위치 유지 상태 |
| photo | 아래 PhotoSession. 아직 임무가 없으면 `{}` |
| mail | 아래 Mail |
| capabilities | `{navigate,navigate_point,rooftop_only,photo_return,photo_return_reason}`. 설정 유무를 표현하며 실장비 준비 보증이 아님 |
| events | 최대 80개 `[{time,kind,detail}]`. time은 Unix seconds |
| door_control | 아래 Door. HTTP 서버가 backend snapshot에 추가 |
| door | door_control.reason에 해당하는 요약 string |

`telemetry`의 value는 각 repository 메시지의 원형이다. `floor={floor_id,map_generation,state,detail}`, `supervisor={state,connected,detail,ownership_epoch}`, `apriltag={detections,stamp_age_sec}` 등이 있다. `localization`, `tracking`, `control`, `operator_phase`는 upstream ROS 일반 규약이 아닌 이 저장소의 확장 JSON이다. 알 수 없는 새 필드는 무시하고 필요한 필드만 사용한다.

UI가 읽는 주요 확장 payload는 다음과 같다. 존재하지 않는 telemetry key나 선택 필드는 정상적인 초기 상태일 수 있다.

| key | value의 주요 필드 |
|---|---|
| localization | request_id(int), state(SEARCHING/NEEDS_POSE/VERIFYING/INPUT_REJECTED/AMBIGUOUS/READY), floor, source(auto/manual, 초기에는 없을 수 있음), reason, x/y/yaw?, scan_match_required?, scan_match_verified?, score?, hit_fraction? |
| operator_phase | active?, resume_available, route_id?, run_id?, revision?, phase?, phases[], waiting?, continue_supported, pending, stop_reason?, error?, last_operator_phase?, result{request_id,phase,kind?,state,reason?}. 실행 전에는 run 필드가 없음 |
| tracking | mode, geometry_age_sec(number/null), reason?, routes[{id,commissioned}], transport_delay_sec?, callback_to_processing_sec?, processing_to_completion_sec?, completion_to_publish_sec?. 점군·변환 행렬은 응답에서 제거 |
| control | phase?, reason?, tracking_state?, first_fault?, flight_end_height_conflict?. 제어 진단 원문이며 UI의 임무 완료 증거로 쓰지 않음 |
| mission_status | active(bool). 최근 2초 관측 여부와 함께 observed_active를 계산 |
| mission_feedback / mission_result | progress / result와 같은 repository action 필드. 현재 콘솔의 목표와 다른 클라이언트 목표의 관측도 포함할 수 있음 |

Mission result_code는 OK=0, BUSY=1, INVALID_GOAL=2, CAPABILITY_DISABLED=3, NAVIGATION_FAILED=4, STAIR_FAILED=5, LOCALIZATION_FAILED=6, SCAN_FAILED=7, COMMUNICATION_LOST=8, MISSION_ABORT=9다. HTTP status와 별개의 실행 결과다.

층 state: UNKNOWN=0, TRANSITIONING=1, READY=2, FAULT=3. supervisor state: DISARMED=0, NAV=1, STAIR=2, FAULT=3. 운영 층 전이는 동일 계단 경로의 최근 성공 결과를 한 번 소비한 뒤 지도·AMCL·costmap을 확인한다. AprilTag 검출기는 운영 실행에서 시작하지 않는다.

### Navigation / MapImage

`navigation={map,global_path,local_path,robot,goal,notes,floor_ready}`.

- map은 `MapImage`에서 image를 뺀 metadata 또는 null이다. global/local_path는 map frame의 `[[x,y],...]`, robot은 `[x,y,yaw]` 또는 null, goal은 `[x,y]` 또는 null이다.
- 경로는 최대 약 1,500개로 줄여 전송하며 끝점은 보존한다. local path가 3초 넘으면 표시에서 제외한다.
- floor READY 수신이 2초 이내일 때 overlays를 표시한다. 로봇 TF age 허용 범위는 -0.5~2초다. 조건을 충족하지 못하면 notes와 빈 overlay를 사용한다. 마지막 배경지도는 남을 수 있다.
- GET `/api/map`: `{revision,width,height,resolution,origin,frame,bounds,image}` 또는 `{error}`. resolution m/pixel, origin `[x,y,yaw]`, frame=`map`, bounds는 PNG상의 `[left,top,width,height]`이다. image는 `data:image/png;base64,...`.
- PNG의 Y축은 지도 배열과 반대다. 원점 yaw 회전도 적용해야 한다. revision이 변경될 때 이미지를 갱신하고, 응답 revision이 현재 State와 다르면 폐기한다.

### Recording / PhotoSession / Mail / Door

- Recording: `{state,path,error,busy,log_path,camera_format,raw_bag_path,raw_copies,raw_capture_unverified,excluded_topics_regex}`. state=`idle|starting|recording|stopping|saved|error`. `raw_copies`는 각 원본 BAG의 `{path,state,error}`이며 state=`sealing|copying|saved|error`다. 로컬 BAG 저장과 미니PC 녹화 종료 후 busy=false로 바뀌며 원본 BAG 복사는 별도로 계속돼 새 임무를 막지 않는다. 단, 원격 녹화 종료 확인이 실패하면 `raw_capture_unverified=true`이며 중복 원본 녹화를 시작하지 않는다. `.bag.active`와 구독 준비를 확인한 후 목표를 전송한다. 기본 저장은 `/mnt/ethan/tron-bags/live/ui-tests`, mount=`/mnt/ethan`. 분할·시간 제한이 없고 RGB JPEG를 보존하며 depth/IR와 중복 RAW를 제외한다.
- PhotoSession: `{version,mission_id,origin,origin_map_sha256,stage,return_started?,retry_available?,mail_recipient,photos,missed_photos,photo_outcome,detail,...}`. origin은 `{id,floor_id,type,x,y,yaw}`. photos는 `{location,path,...}`, missed_photos는 `{location,reason,...}`다. stage=`OUTBOUND|PHOTO_ARRIVED|WAITING_RETURN|RETURNING|RETURN_ARRIVED|COMPLETE|FAILED|CANCELLED|INTERRUPTED`. `retry_available`은 상태 응답에서 계산한 재시도 후보 여부이며 현장 주행 준비 보증은 아니다. `return_started`는 복귀를 이미 시작한 세션에 대한 촬영 재시도를 구분하는 저장 필드다. 자동 복귀하지 않고 명시적 승인을 기다린다. 재시작 시 진행 중 세션은 INTERRUPTED로 복원한다.
- Mail: `{configured,sender,reason?,worker_error,jobs,recipient,session_recipient,enqueue_error,manual_capture}`. jobs는 최근 최대 100개 `{id,mission_id,location,recipient,state,attempts,error,created_at,finished_at?}`. 첨부 경로/sha256은 jobs 응답에서 제거한다. state=`QUEUED|SENDING|SENT|FAILED|UNKNOWN`. SENT는 발송 서버 접수이며 최종 수신함 도착 확인이 아니다. manual_capture=`{id?,state,busy,recipient?,path?,job_id?,detail?}`.
- Door: `{active,mode,reason,attempts,run_id,server_owned,last_device,last_hold_sec,last_command_id,error,worker_error,hold_remaining_sec,physical_door_verified,one_shot}`. mode는 `''|up|return`, hold_remaining_sec는 문별 남은 요청 시간(초), physical_door_verified=false다. one_shot=`{busy,state,device?,command_id?,detail}`이고 state=`IDLE|SENDING|ACCEPTED|UNKNOWN`이다. ACCEPTED는 장치 명령 ACK이며 실제 문 위치 확인은 아니다. 브라우저를 닫아도 UI 서버가 자동 요청을 수행한다.

## 4. 명령별 실행 조건과 확인 절차

### 경로 확인 / 시작 / 옥상 순회 / 복귀

현재 배포의 목적지 id는 `locations.yaml`에서 정의한다. 층 이동 화면은 id에 `from_`가 포함된 도착 대기점과 `stair_rf_landing`을 선택지로 사용한다. 촬영 순서는 `photo_tour.yaml`의 남서→북서→북동→남동이며 마지막 지점 id는 `roof_loop_se`다. 이는 이 배포의 UI 규칙으로, 일반 ROS 규약이 아니다.

kind는 `navigate`(기본값), `navigate_point`, `photo_return`, `rooftop_photo`, `return_to_start`. UI의 `floor` 선택은 HTTP kind가 아니며 `navigate`로 변환한다.

Plan은 경로의 `origin_id,destination_id,segments`에 `kind,blockers,can_start`를 추가한다. segment는 `type,target_id,source_floor,profile_id`를 공통으로 사용하며 일반 경로는 `source_id,target_floor,edge_id,stair_id,scan_profile_id,confirmed_location_id`도 포함한다. PHOTO/RETURN_POSE segment에는 일부 필드가 없다. 준비 안 된 상태는 200 + can_start=false일 수 있고, 경로 오류/잘못된 id 등은 409일 수 있다.

현재 층 READY와 수신 age<=2초, 실제 층과 경로 시작 층의 일치, 확인 가능한 supervisor의 NAV 상태, 필요한 계단 control/commissioned profile을 검사한다. `rooftop_photo`는 RF여야 하고 층간 경로를 포함하면 안 된다. `photo_return`은 옥상까지의 이동을 포함할 수 있으며 복귀에 필요한 계단 경로도 미리 검사한다. `photo_return`은 `/mission`에 `photo_tour`, `rooftop_photo`는 `rooftop_tour`로 전달한다. `/api/photo/roof_start`는 마지막 촬영점 id와 `rooftop_photo`로 start를 호출하는 편의 경로다.

start는 실행 중 이 콘솔/다른 클라이언트의 임무, 층 선택/복구, recorder 충돌 등을 거절한다. record=true이면 녹화 준비 후 계획을 재검증하고 목표를 보낸다. 이미 recording인 녹화는 record=true로 재사용할 수 있다. 새 녹화 준비 실패 시 목표를 보내지 않는다. 성공 종료 또는 사용자 취소 시 녹화 저장을 요청하고, 실패 종료 시 수동 복구/재시도를 기록하도록 녹화를 유지한다. 프런트엔드는 임무 실패를 녹화 종료로 가정하지 않는다.

FAILED/CANCELLED/INTERRUPTED 촬영 출발 구간에서 `photo.retry_available=true`이면 기존 `photo_return` 또는 RF의 `rooftop_photo` 시작 요청으로 현재 확인된 위치에서 재시도한다. 기존 origin, mission_id, 수신 주소, 이미 저장된 사진은 유지하며 사진을 못 찍은 지점만 설정된 순서대로 다시 이동·촬영한다. 기존 missed_photos 지점에서 재촬영에 성공하면 누락 기록을 제거한다. 재시도는 새 출발점을 저장하거나 중단된 계단 동작의 중간 단계에서 바로 재개하지 않는다. 층 READY/NAV 제어권/필요 계단 경로는 시작할 때 다시 검사한다. RF 순회만 할 때는 저장된 출발 층 지도의 변경으로 순회를 막지 않는다. 그 좌표로 복귀할 때는 기존 지도 동일성 검사를 적용한다. 복귀를 이미 시작한 세션은 촬영 출발 구간으로 재시도하지 않는다.

`return`은 저장된 origin과 WAITING_RETURN/INTERRUPTED/FAILED/CANCELLED 세션이 있어야 한다. 출발점 지도가 달라지면 좌표 재확인이 필요하다. 저장된 실제 출발 pose로 돌아가는 작업이며, 현재 RF 순회 시작 버튼과 별개의 승인 동작이다.

### 임의 지도 지점으로 NAV

`plan/start` 요청: `{kind:"navigate_point",floor_id,map_revision,x,y,yaw,record?}`. 좌표는 map 기준 m/rad이며 `destination`은 필요 없다. 최신 READY 층, 화면 지도 revision, 지도 범위, 연결된 NAV 모드를 확인하고 `ui_point`라는 임시 목표를 `/mission`에 전달한다. 저장된 목적지/경로 그래프/촬영 세션은 변경하지 않는다. 실제 경로와 장애물 판정은 기존 NavigationExecutor와 move_base가 수행한다. 미리보기의 한 NAV 구간은 충돌 없는 경로를 보장하는 결과가 아니다.

Mission.action에 point_floor, point_map_generation, point_x, point_y, point_yaw를 추가했다. 일반 임무는 기본값을 사용한다. ROS 메시지 계약이 달라졌으므로 같은 action을 사용하는 프로세스는 같은 재빌드 결과로 재시작해야 한다. 새 ROS 노드는 없다.

### RF에서 독립 순회 및 중단 후 합류

- 수동으로 RF 평지에 도착했다면 `relocate_flat`으로 NAV·RF 지도를 준비하고, 지도상 위치를 확인한 뒤 `roof_start`를 사용한다. 계단 성공 결과를 위조하지 않는다.
- 실패한 출발/순회 세션에 남은 사진이 있으면 이전 출발 좌표·이미 찍은 사진·수신자를 보존하여 남은 지점만 방문한다. 5F에서 시작했던 임무라면 승인 복귀 목적지도 기존 5F 출발 위치다.
- 이전 순회가 WAITING_RETURN이거나 재시도할 미완료 사진이 없으면, RF 순회만 시작은 이전 session.json을 보관하고 RF 현재 좌표를 출발점으로 새 순회를 만든다. 이후 복귀 승인은 이 새 RF 출발점으로 향한다. 과거 출발점은 보관 기록에만 남는다.
- 임의 지점 NAV는 촬영 실패/복귀 대기 세션에 종속되지 않으며 세션을 변경하지 않는다.

### 취소 / 층 변경 / 위치 지정 / 평지 제어권 복구

- cancel은 이 콘솔이 보낸 goal만 취소한다. 다른 클라이언트의 goal 전체를 취소하지 않는다. 로컬 임무·진행 중 녹화·hold·수동 입력·평지 준비 작업이 없어도 200으로 이미 종료됐음을 알리므로 실패 후 반복 중지가 가능하다. 중지만으로 계단 모드를 NAV로 바꾸지는 않는다. 평지 준비 중 cancel은 이후 단계 요청을 중지하고 UI에 예약된 순회 시작도 취소한다. 원본 BAG 복사만 진행 중일 때는 중지 대상으로 취급하지 않는다. 요청 수신은 events의 `stop_request`에 남는다. 실제 정지는 별도 센서/상태 확인이 필요하다.
- 이전 녹화가 `stopping`이고 새 요청의 `record:false`이면 녹화 저장과 별개로 새 임무를 시작할 수 있다. `record:true`는 기존 녹화 종료를 기다려 중복 녹화를 방지한다. 녹화 종료 예외가 발생해도 mission 결과와 stop 플래그 정리를 누락하지 않는다.
- relocate_flat은 등록된 층과 사용자의 평지 확인을 받는다. 이 콘솔이 실행한 이전 임무·녹화를 종료하고, 최신 임무 종료 상태를 확인한 뒤 필요 시 기존 supervisor 인계 서비스로 NAV 전환, hold 중지, 지도 선택을 순서대로 요청한다. operator_phase의 오래된 run_id를 다시 요구하지 않는다. 다른 클라이언트의 진행 중 임무는 취소하지 않는다. 서비스 탐색/상태 대기는 제한 시간이 있고 실패하면 준비 플래그를 해제한다. 이미 전송된 서비스 응답 대기 중에는 늦은 모드·지도 변경과 새 임무가 겹치지 않도록 준비 중 상태를 유지한다. cancel 뒤에는 추가 단계 요청을 보내지 않는다. 일부 변경 후 실패해도 그 모드/지도는 유지되며 상태와 이유를 표시한다. raw BAG 복사 완료를 기다리지는 않는다.
- select_floor는 등록된 floor_id만 허용한다. 임무/녹화 중에는 불가하고 최신 mission status도 필요하다. 기존 hold를 먼저 중지한다. 서비스 완료 후 old floor telemetry를 지우므로 새 상태를 기다려야 한다. 응답의 오래된 RViz 안내 문구와 별개로 UI 지도에서도 pose_estimate를 보낼 수 있다.
- pose_estimate는 실제 층과 화면 revision을 요구한다. x/y/yaw는 유한한 숫자이며 bool은 불가하다. 지도 범위·frame·활성 map_server revision을 확인하고, 요청 중 층/지도/상태 변화도 다시 검사한다. UNKNOWN/READY 층에서 허용되며 전이/fault, 임무/계단 제어 중에는 거절한다. `/initialpose` 구독 연결도 필요하다. covariance는 x/y=0.04, yaw=0.03이며 yaw는 정규화한다. 제출 뒤 localization의 request_id가 previous_request_id보다 큰 source=manual/READY 또는 거절 상태를 기다린다. 기존 request_id의 상태/사유 변경만으로 새 요청의 성공이나 거절을 확정하지 않는다. 요청 id가 증가하지 않는 거절은 해당 요청과 연결할 수 없으므로 UI는 확인 대기 후 시간 초과를 표시할 수 있다. 이 API는 새 요청의 id를 직접 반환하지 않으므로 다른 클라이언트가 동시에 /initialpose를 보내면 정확한 요청 귀속은 보장하지 못한다. 제출만으로 성공을 표시하지 않는다.
- recover_control은 사용자의 실제 평지 확인이 필수다. STAIR 상태에서는 UI가 `평지 이동 완료`를 직접 입력받고 서버가 최신 operator_phase와 같은 run_id를 검사한다. 계단참 중단은 계단 재개를 사용한다. 이 확인은 물리적 평지를 센서로 독립 검증하지 않는다. 임무가 종료되고 최신 mission/supervisor 상태와 연결이 필요하다. NAV면 이미 완료 응답, STAIR면 인계 서비스 호출, 다른 상태면 거절한다. 서비스가 3초 안에 완료되지 않으면 pending=true로 반환할 수 있고 worker는 계속 실행한다. recovering=false와 정상 NAV telemetry를 확인하기 전 완료로 표시하지 않는다.

### 계단 단계 재개 및 수동 입력

- operator_phase의 kind 기본값은 `set_phase`; `continue`는 동일 단계 중단 대기 + continue_supported=true일 때만 가능하다. status가 1초 이내이며 active여야 한다. run_id/revision/허용 phases/confirmed=true를 일치시킨다.
- UI는 단계 확인 후 `/api/state`를 다시 읽어 같은 run/route의 최신 revision으로 요청한다. 다른 임무로 바뀌었거나 같은 단계 이어가기의 대상 단계가 바뀌었으면 전송하지 않는다. 서버의 revision 검사는 유지하고 거절된 이동 명령을 자동 재전송하지 않는다.
- 종료된 계단 액션은 `resume_available=true`일 때 `/api/start`의 `stair_resume`으로 재개한다. route_id/run_id/revision은 최신 operator_phase 상태와 정확히 일치해야 하며 phase는 해당 경로의 단계여야 한다. 계단 중간에서는 NAV 진입점 재접근·진입 점군 매칭을 반복하지 않고 기존 LiDAR 좌표 기준을 이어 쓴다. 같은 단계는 회전 pivot/진행 상태를 보존한다. 계단 완료 후에만 기존 floor transition으로 도착 층을 확정한다. 이전 임무가 촬영 순회였다면 RF 확정 후 기존 `현재 위치에서 촬영 임무 재시도`로 남은 촬영 지점을 이어간다. stair supervisor 노드 재시작으로 LiDAR 기준이 사라진 경우 자동 재개는 제공하지 않는다.
- 요청 후 최대 2초 동안 같은 request_id의 결과를 기다린다. REJECTED는 409. ACCEPTED는 아직 APPLIED가 아닐 수 있으므로 operator_phase telemetry의 result와 실제 phase를 확인한다. 타임아웃 시 자동 재전송하지 않는다.
- manual begin은 lease와 서버 Unix seconds를 반환한다. update는 동일 lease, 증가하는 정수 sequence, forward/turn 각 -1~1, 서버 기준 issued_at이 현재보다 0~0.35초 전일 때만 받는다. 0.35초 만료 전에 갱신해야 한다. UI는 서버 시간과 monotonic 경과로 issued_at을 만든다.
- 수동 입력은 자동 임무를 취소하거나 recorder/mode를 변경하지 않는다. 놓기/포커스 이탈/페이지 숨김/화면 전환 시 end를 보낸다. 만료 입력을 큐에 쌓거나 자동 재생하지 않는다. manual.available은 status 수신 1.5초 이내 여부다.

### 촬영·메일과 문

- mail/settings의 recipient는 string. 비우면 다음 임무의 메일을 끈다. 촬영 순회 시작 시 recipient를 세션에 고정하므로 진행 중 주소 변경은 기존 순회에 적용되지 않는다.
- capture_mail은 비어 있지 않은 올바른 recipient와 설정된 mail worker가 필요하다. 촬영 작업이 끝날 때까지 추가 촬영은 거절한다. 200 응답은 작업 시작이며 mail.manual_capture 및 jobs로 저장/발송 결과를 확인한다. NAV/계단 임무를 취소하지 않는다.
- retry는 FAILED/UNKNOWN job만 허용한다. UNKNOWN은 `confirm_unknown:true`가 필요하다. 메일 실패는 주행을 중단시키지 않는다.
- RGB 촬영은 기존 PhotoCapture의 단일 원격 압축 영상 구독을 사용한다. 같은 메시지를 `/mission_manager/rgb/compressed`로 로컬 공유해 BAG에 저장한다. 원격 원본과 퇴역한 태그 중계 영상은 녹화에서 제외한다. 새 ROS 노드는 추가하지 않는다.
- open_once는 live에서 F5/RF 중 한 문을 선택해 비동기로 5층 30초·옥상 60초 개방 유지 명령을 보낸다. 같은 문에 요청이 진행 중이거나 유지 시간이 남았으면 추가 요청을 막는다. door_control.one_shot에서 ACK를 확인한다. ACK 타임아웃은 명령 실행 여부가 불명확하므로 자동 재전송하지 않는다. return은 이전 클라이언트를 위한 RF 전용 별칭이며 최신 확인 층이 RF여야 한다. stop은 기존 클라이언트 호환 응답만 반환하며 접수된 장치 명령은 취소하지 못한다.
- UI 서버는 기존 임무 피드백과 계단 제어 상태를 읽어 5F→RF 출발 NAV 때 F5, 2차 상승 후 ROOFTOP_TURN 때 RF, RF→5F 복귀 계단 진입 전 RF, 5F 도착 전 마지막 하강 구간 때 F5에 각각 1회 명령한다. 장치가 개방 유지 시간 동안 직접 동작하며 주기적인 UI 재요청은 없다. 이는 UI 서버 기능이고 새 ROS 노드나 상태머신 단계가 아니다. 장치 ACK는 실제 문 개방을 증명하지 않으므로 현장 확인이 필요하다.

## 5. 프런트엔드 사용 계약

State를 약 1초마다 갱신하고 map 이미지는 revision 변경 시 받는다. 응답 이후 브라우저의 monotonic 경과 시간을 age_sec에 더해 상태가 실제로 오래되면 표시와 단계 조작을 갱신한다. GET 대기 제한은 3.5초, 일반 명령은 15초, 층 변경은 30초다. 수동 begin은 1.2초, update는 0.25초, 그 외 수동 요청은 0.45초이며 약 80ms마다 갱신한다. 이 값은 클라이언트 대기 정책이지 모든 서버 작업의 최대 실행시간이 아니다.

페이지 로드/탭 변경/목표 선택/경로 미리보기는 이동을 시작하지 않는다. 사용자의 명시적 시작·복귀·전이·수동 입력만 명령을 전송한다. 화면 연결 끊김은 로봇 정지 확인이 아니며, 오래된 map/floor/phase를 정상 실시간 상태처럼 표현하지 않는다. 결과와 readiness는 표시 시점의 관측이고, 최종 실행 판단은 기존 backend가 다시 수행한다.

프런트엔드는 다음 기능의 진입점을 하나씩 둔다: 임무 시작, 녹화 선택, 촬영 임무의 현재 위치 재시도, 복귀 승인, 현재 위치 촬영, 문 제어, 현재 층/지도 지정, 수동 입력, 계단 재개/단계 지정, 평지 제어권 복구, 진단 내보내기. 화면의 네 임무 탭은 목적지, 층 이동만, 옥상 순회만, 올라가서 촬영이다. 옥상 순회만은 항상 `rooftop_photo`, 올라가서 촬영은 항상 `photo_return`을 사용한다. 지도 목표 선택은 `/initialpose`와 별개이고 시작 버튼 전에는 이동 요청을 보내지 않는다. 현재 층 사용 및 RF 평지 도착 준비 버튼은 `/api/relocate_flat`을 사용한다. 촬영 재시도 버튼은 새 API나 별도 ROS 노드 없이 기존 시작 요청을 사용한다.

옥상 순회 탭에서 내부 층/모드가 RF/NAV가 아니면 시작 버튼은 `RF 평지 확인 · 순회 시작`으로 표시된다. 사용자가 실제 RF 평지 도착을 확인하면 `/api/relocate_flat` → 최신 RF READY/NAV 상태 확인 → `/api/photo/roof_start`를 순서대로 호출한다. 별도 복구 버튼이나 과거 계단 run_id 입력은 필요 없다. 준비 중 중지, 연결 끊김, 준비 실패 또는 30초 확인 기한 경과 시 예약 시작은 취소되며 늦게 READY가 되어도 자동 출발하지 않는다. 지도가 바뀐 뒤 위치 확인이 필요하면 2D 위치 지정 후 다시 시작한다. API의 `roof_start` 자체는 여전히 RF/NAV 준비가 선행되어야 하며, 이 연결은 기존 API를 순서대로 사용하는 UI 동작이다.

## 6. 검증과 근거

구현: `src/mission_manager/src/mission_manager/{mission_console,ros_console,console_navigation,console_map,console_manual,console_recording,console_door,photo_mission,mail_outbox}.py`.

메시지: `src/mission_manager/action/Mission.action`, `src/multifloor_manager/msg/FloorState.msg`, `src/stair_supervisor/msg/SupervisorState.msg`.

검증 범위: HTTP 라우팅/인증/요청 형식은 `test_console.py`, door HTTP는 `test_console_door_http.py`, 어댑터 명령 계약은 `test_ros_console.py`, 수동 lease는 `test_console_manual.py`에서 확인한다. 새 UI의 시각·상호작용 검증은 실제 robot 대신 로컬 mock HTTP 응답으로 수행하며, 이를 실주행 성공률이나 실장비 정지/도착 증명으로 해석하지 않는다.
