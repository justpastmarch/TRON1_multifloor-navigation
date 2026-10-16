# Stair Supervisor 미니PC 실행

## 선택과 범위

원본 LiDAR와 IMU는 미니PC에서 발행되고, 이전 구성에서는 원본 점군이 Wi-Fi를 건너 개발컴퓨터의 Stair Supervisor로 전송됐다. 2026-10-14 현장 측정에서는 미니PC에서 약 10 Hz·0.1초였던 점군이 개발컴퓨터에서 수 초 이상 지연됐다. 기존 Supervisor **전체**를 미니PC로 옮겨 센서 입력과 로봇 WebSocket 명령을 현지화한다. 상태머신과 ROS 제어 노드는 추가하지 않는다. mission manager, floor manager, move_base, UI, ROS master는 개발컴퓨터에 남는다.

## 실행

기본 `./run.sh`는 미니PC의 별도 `~/.local/share/tron1-stair-supervisor/ws` 오버레이에 세 ROS 패키지를 동기화하고, 변경된 경우에만 빌드한 뒤 동일한 `/stair_supervisor`와 `/stair_traversal`을 미니PC에서 실행한다. 원래 미니PC `~/catkin_ws`와 부팅 센서 서비스는 건드리지 않는다. Ctrl+C로 종료하면 미니PC Supervisor도 정지한다. 기존 개발컴퓨터 배치가 필요하면 `STAIR_EXECUTION_HOST=workstation ./run.sh`로 되돌린다. 두 배치를 동시에 실행하지 않는다.

미니PC 로그는 `ssh m3localtron@192.168.1.56 'journalctl --user -u tron1-stair-supervisor -n 100 --no-pager'`로 확인한다. 배포 로그는 미니PC의 `~/.local/share/tron1-stair-supervisor/build.log`와 `pip.log`다.

UI의 **테스트(녹화 포함)**는 미션·RGB·상태 로그를 로컬 1TB 디스크에 기록하고, 원본 `/livox/lidar`와 `/livox/imu`는 미니PC에서 별도 BAG으로 기록한다. 정지 후 원본 BAG은 같은 로컬 녹화 폴더에 `_raw.bag`으로 복사된다. 복사가 실패해도 원본은 미니PC `~/.local/share/tron1-stair-supervisor/bags/`에 남는다. CLI 자동 녹화도 원본 점군을 미니PC에서 기록해 주행 중 Wi-Fi 재전송을 피한다.

## 확인 범위

2026-10-14에 로컬 계약 테스트 25개, `./run.sh --check`, `./run.sh --preflight`, 미니PC 메시지 MD5 일치, 미니PC LiDAR 설정 및 수치 라이브러리 로딩, 로봇 WebSocket의 무명령 연결을 확인했다. **실제 미니PC 배치로 계단을 오른 결과는 아직 없다.** 첫 실주행에서는 `/stair_supervisor/tracking_status`의 점군 age와 상태머신 단계, 두 BAG의 동기화, 미니PC CPU 부하를 확인해야 한다. 이 확인 전에는 지연 감소나 등반 성공률을 수치로 단정하지 않는다.
