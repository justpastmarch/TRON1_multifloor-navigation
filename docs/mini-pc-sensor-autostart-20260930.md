# MiniPC 센서 부팅 자동 실행

2026-09-30, MiniPC `m3localtron@192.168.1.56`에 적용.

MiniPC의 `tron1-sensors.service`는 부팅 때 시작한다. 사용자 PC의 ROS master `http://192.168.1.26:11311`가 준비되면 기존 `sensor_integration wf_mapping.launch`를 실행한다. MiniPC에서 별도 master를 생성하지 않는다. 사용자 PC에서는 기존 `run.sh`로 master와 UI/임무 스택을 실행해야 한다. 전원만 켜서 주행 버튼을 사용할 수 있는 구성은 아니다.

대상은 LiDAR, 카메라, 바퀴 odometry 수신, 센서 TF, LaserScan 변환이다. 임무, NAV, 계단 제어, BAG 녹화는 자동 시작하지 않는다. 조이스틱 수신은 기존 `run.sh`의 별도 실행 절차에 남아 있다. 기존 Astra 웹 서비스는 현재 disarmed 웹 서버만 실행하고 있어 변경하지 않았다. `astra-web.service`의 웹 서버는 현재 주행이 무장되지 않은 상태다. Astra 웹에서 별도 센서 스택 시작 기능을 사용하지 않는다.

## 설치 파일과 소유권

- 원본: `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation/deploy/mini-pc-sensors/`.
- MiniPC 서비스: `~/.config/systemd/user/tron1-sensors.service`.
- 실행 파일: `~/.local/lib/tron1-sensors/{start.sh,watch_master.py}`.
- 환경: `~/.config/tron1-sensors.env`. 설치 당시 저장소 `config.env`에서 workspace, master 주소, MiniPC IP, 카메라 serial을 가져온 배포 사본이다. 해당 값을 바꾸면 이 환경 파일도 갱신하고 서비스를 재시작해야 한다.
- 기존 `run.sh`는 서비스가 설치돼 있으면 이를 시작/재사용한다. 서비스가 없는 장비에서만 기존 tmux 방식을 사용한다.

서비스는 master의 PID와 `/run_id`를 확인한다. master 교체 또는 연속 3회 연결 실패 시 센서 실행을 정리하고 재연결을 기다린다. `roslaunch --required`로 센서 프로세스 종료 시 전체 센서 launch를 다시 시작한다. 살아 있지만 데이터를 보내지 않는 프로세스까지 감지하는 서비스는 아니다. 센서 전체 재시작 때 wheel odometry 원점도 다시 잡히므로, 서비스를 재시작하기 전 주행을 정지한다. systemd가 watcher를 다시 시작하면 watcher는 master가 돌아올 때까지 기다린 뒤 센서를 실행한다.

## 사용자 PC에서 상태 확인 / 중지 / 재개

```bash
ssh m3localtron@192.168.1.56 'systemctl --user status tron1-sensors.service --no-pager'
ssh m3localtron@192.168.1.56 'journalctl --user -u tron1-sensors.service -n 50 --no-pager'
# 이번 부팅에서 센서 정지. run.sh를 다시 실행하면 시작됨.
ssh m3localtron@192.168.1.56 'systemctl --user stop tron1-sensors.service'
# 다시 시작
ssh m3localtron@192.168.1.56 'systemctl --user start tron1-sensors.service'
# 부팅 자동 실행 해제와 현재 실행 정지
ssh m3localtron@192.168.1.56 'systemctl --user disable --now tron1-sensors.service'
# 부팅 자동 실행 재설정
ssh m3localtron@192.168.1.56 'systemctl --user enable --now tron1-sensors.service'
```

`active (running)`은 대기 프로그램 실행 상태일 수 있다. 실제 센서 데이터 수신 준비 완료와는 다르다. `Waiting for workstation ROS master` 로그라면 사용자 PC에서 기존 스택을 실행한다. `disable`해도 `run.sh`의 명시적 시작은 가능하다.

실센서 준비는 사용자 PC의 기존 `run.sh` 준비 완료 표시와 센서 메시지 수신으로 확인한다. 서비스의 active 표시만으로 테스트 주행을 시작하지 않는다.

## 검증 범위

- OBSERVED: 서비스 unit 검증 통과, `enabled`, `active/running`, `Linger=yes`, watcher 1개, master 대기 로그, 불필요한 재시작 0회.
- OBSERVED: shell/Python 구문 검증. 모의 입력으로 master 대기/교체/일시 단절/지속 단절/launch 종료/연결 실패를 검증한다. 실제 센서 연결 성공의 증거는 아니다.
- OBSERVED (2026-09-30 08:24–08:28 KST): 사용자 추가 승인 후 주행 노드 없이 임시 master 실행. 기존 master/11311 포트가 비어 있음을 먼저 확인했다. 센서 노드 8개와 rosout 등록, 카메라 영상/CameraInfo 수신을 확인했다.
- OBSERVED: 최초 LiDAR 초기화는 `Create detection socket failed`로 실패했고 점군/IMU/scan이 없었다. odometry도 처음에는 연결 실패하다가 자동 재연결 후 수신됐다. 최초 소켓 실패의 근본 원인은 미확정이다.
- OBSERVED: master 종료 시 watcher가 `Master lost or replaced`를 기록했고 systemd NRestarts=1로 재시작했다. 두 번째 master에서 `/livox/lidar`, `/livox/imu`, `/scan`, `/tron/wheel_odom_raw`, `/camera1/color/image_raw`의 실제 header 수신을 모두 확인했다. `/scan` 발행자는 `/mid360s_laserscan` 하나였다. 관측 명령은 MiniPC의 ROS/workspace 환경을 불러온 후 각 토픽에 `timeout 7 rostopic echo -n 1 <topic>/header`를 실행한 것이다. 관련 코드는 `deploy/mini-pc-sensors/watch_master.py`이다.
- UNVERIFIED: 실제 전원 재부팅 후 첫 시도 성공 여부와 장시간 안정성. 이번 재연결 성공만으로 최초 LiDAR 초기화 실패가 근본 해결됐다고 보지 않는다.

기존 worktree 변경은 보존했다. 수정은 서비스 배포 파일과 `run.sh`, 관련 안내에 한정했다. 센서 ROS 노드/주행 제어는 수정하지 않았다. 테스트 BAG는 생성하지 않았다. 이번 검증의 임시 master 2회 실행은 모두 종료하며 전용 임시 로그 디렉터리도 제거한다. systemd journal은 정상 운영 로그로 유지한다.
