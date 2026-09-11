# 5층-옥상 수동 주행 기록

센서 ROS topic과 `/tron/sensor_joy`가 올라온 상태에서 아래 명령 하나만 실행합니다.
이 명령은 로봇, 센서 launch, SDK listener 또는 주행 명령을 시작하지 않습니다.

```bash
./run.sh --record-manual
```

`[READY SENSOR] yes`와 `[READY JOYSTICK] yes`를 확인한 뒤 전용 물리 조종기로
5층에서 옥상까지 수동 주행하고 사진/영상 구간을 완료한 다음, 기록 terminal에서
`Ctrl+C`를 한 번 누릅니다. 완성 bag과 결과 JSON은 기본적으로
`manual_captures/`에 생성되며 기존 파일을 덮어쓰지 않습니다.

`/tron/sensor_joy`는 SDK의 `SensorJoy.axes`와 `buttons`를 변환 없이 담은
`sensor_msgs/Joy`입니다. `Joy.header.stamp`에는 SDK source timestamp를 nanosecond 단위로
보존하고, rosbag connection time에는 ROS 수신 시각이 별도로 남습니다. 필수 topic의
메시지가 하나라도 없으면 종료 시 `.INCOMPLETE.json`이 생성되고 명령은 exit 1로 끝납니다.

topic은 `config.env`의 `CAPTURE_CAMERA_TOPIC`, `CAMERA_INFO_TOPIC`,
`RAW_LIVOX_TOPIC`, `SCAN_TOPIC`, `WHEEL_ODOM_TOPIC`, `IMU_TOPIC`, `SENSOR_JOY_TOPIC`에서 바꿀 수 있습니다.
카메라 기본값은 JPEG compressed transport이고, `RAW_LIVOX_TOPIC`은 PointCloud2가 아닌
`livox_ros_driver2/CustomMsg` 원본이어야 합니다. Control Center 명령이나 WebSocket
payload는 이 bag의 joystick evidence가 아닙니다.

## SensorJoy listener 사전 구성

listener는 SDK 3.4.0의 `Robot(RobotType.PointFoot)`, `init(robot_ip)`,
`subscribeSensorJoy(callback)`만 사용하며 `publishRobotCmd`, mode, enable, stand, twist를
호출하지 않습니다. 운영자가 SDK 초기화가 현재 controller ownership을 바꾸지 않는다는
것을 설치된 SDK source 또는 제조사 문서로 확인한 뒤, SDK가 설치된 mini PC에서 다음처럼
실행해 실제 callback을 확인해야 합니다.

```bash
source /opt/ros/noetic/setup.bash
source /home/m3localtron/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.192.1.2:11311 ROS_IP=10.192.1.20
python3 /home/m3localtron/catkin_ws/src/sensor_joy_bridge.py \
  --robot-ip 10.192.1.2 --topic /tron/sensor_joy
rostopic echo -n 1 /tron/sensor_joy
```

현재 확인한 mini PC에는 `limxsdk` Python module과 SDK source/library가 없으므로 위
listener를 배포하거나 실행하지 않았습니다. wheel 설치 또는 runtime 교체도 하지
않았습니다. 이 prerequisite가 해결되고 SDK `init`의 ownership 동작이 확인되기 전에는
synthetic 테스트 통과를 물리 조종기 연결 확인으로 해석하면 안 됩니다.
