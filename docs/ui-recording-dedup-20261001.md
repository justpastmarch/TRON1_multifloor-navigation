# UI 테스트 녹화 중복 제거 · 2026-10-01

OBSERVED: 기존 RGB JPEG를 디코딩해 로컬 재발행하는 `/apriltag_camera/image_raw`를 UI 녹화의 제외식에 정확히 추가했다. 원본 RGB JPEG, RGB/AprilTag camera_info, 태그 검출 결과, LiDAR, IMU, 조이스틱, 제어 진단은 유지한다. 실제 AprilTag 영상 발행과 검출 노드는 계속 실행한다. 기존 BAG는 변경·삭제하지 않는다. 파일 분할·시간·용량 제한도 추가하지 않는다.

Finding: RGB JPEG와 디코딩한 RAW를 함께 기록하던 중복. Safety: S0 / Mobility: M1. 기록 부하 감소의 주행 영향은 UNVERIFIED.

OBSERVED: 직전 BAG `ui_test_20261001_222521_e861bc75.bag`의 메시지 직렬화 합계 3.804GB 중 중복 RAW는 2.247GB(59.1%)였다. 이 비율은 해당 BAG의 메시지 직렬화 크기 기준이며, 모든 향후 BAG의 실제 파일 크기 절감률을 보장하지 않는다. RAW 발행은 로컬이므로 같은 양의 Wi-Fi 트래픽 감소로 해석하지 않는다.

OBSERVED: 기존 녹화 회귀 테스트 5개 통과. 별도의 로컬 ROS master(11991)에서 실제 rosbag record와 가짜 센서 메시지를 사용한 검증에서도 중복 RAW는 구독·기록되지 않았고, 위 7개 보존 토픽은 각각 30개 메시지를 기록했다. 이는 필터 동작 검증이며 실제 센서 값의 정확도 검증이 아니다. 로봇 주행 명령은 보내지 않았다.

실행 중인 모듈은 파일 수정만으로 갱신되지 않으므로 사용자에게 평지 위치를 확인한 뒤 기존 스택을 재시작한다. `/api/state`의 `recording.excluded_topics_regex`가 새 제외식과 일치하는지 확인해 다음 UI 녹화에 적용된 것을 검증한다. 이 값은 실제 recorder 시작과 준비 판정에 사용하는 같은 제외식이다. 임의로 별도 `rosbag record --all`을 실행하면 UI의 필터가 자동 적용되는 것은 아니다.

변경 범위: 기존 console_recording.py의 제외식 및 진단 메타데이터, 관련 테스트와 이 문서. ROS 노드·FSM·전진/조향 정책을 변경하지 않았다. 첫 상승의 `prediction recheck lost supported footprint` 오류 해결과는 별개이며, 이번 변경으로 그 오류가 해결됐다고 주장하지 않는다.

임시 검증용 master, recorder, BAG, log, ROS cache는 종료·제거했고 검증 결과 JSON만 남겼다. 사용자 기존 worktree 변경과 원본 녹화는 보존했다. 실제 적용/재시작 상태는 같은 분석 폴더의 runtime-after.json에 기록한다.
