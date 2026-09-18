# Mini PC ROS stack 전환 및 TRON1 전체 운용 가이드

## 1. 목적과 소유권

Mini PC에는 서로 동시에 실행하면 안 되는 두 ROS stack이 있다.

| stack | 시작 주체 | ROS master | 역할 |
|---|---|---|---|
| 기존 Astra stack | Mini PC 사용자 service `astra-web.service` | `http://10.192.1.2:11311` | `declan_ws` sensor, FAST-LIO, AMCL, move_base, AprilTag |
| 현재 TRON1 stack | Workstation의 `./run.sh` | `http://192.168.1.26:11311` | Mini PC sensor와 workstation mission/navigation |

두 stack은 camera, LiDAR, `/scan`, navigation, AprilTag를 중복 소유할 수 있다. **한쪽을 완전히 정지한 뒤 다른 쪽을 시작한다.** 현재 TRON1 운용에서는 workstation이 ROS master와 mission/navigation을 소유하고, Mini PC는 `sensor_integration wf_mapping.launch`로 sensor topic만 제공한다.

이 문서의 명령은 별도 표기가 없으면 workstation의 repository root에서 실행한다.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
set -a
source config.env
set +a
SSH_TARGET="${MINI_PC_USER}@${MINI_PC_HOST}"
```

## 2. 최초 한 번: 기존 Astra stack 비활성화

다음 명령은 현재 실행 중인 `astra-web.service`와 그 cgroup 아래의 `declan_ws`, FAST-LIO, AMCL, move_base, AprilTag process를 정지하고 재부팅 자동 시작도 해제한다.

```bash
ssh "$SSH_TARGET" 'systemctl --user disable --now astra-web.service'
sleep 2
```

설정은 Mini PC를 재부팅해도 유지된다. 매번 반복할 필요가 없다.

상태 확인:

```bash
ssh "$SSH_TARGET" \
  'printf "enabled="; systemctl --user is-enabled astra-web.service || true; \
   printf "active="; systemctl --user is-active astra-web.service || true'
```

정상 출력:

```text
enabled=disabled
active=inactive
```

기존 process가 남지 않았는지 확인한다.

```bash
ssh "$SSH_TARGET" \
  "pgrep -af '[r]oslaunch.*declan_ws|[f]ast_lio|[g]uarded_scan|[o]dom_guard|[a]priltag_ros_continuous_node|[m]ove_base|[p]ointcloud_to_laserscan|[l]ivox_lidar_publisher2|[r]ealsense2_camera' || true"
```

TRON1 stack을 아직 시작하지 않은 상태라면 아무것도 출력되지 않아야 한다.

Mini PC 사용자 service의 재부팅 자동 시작 조건도 확인한다.

```bash
ssh "$SSH_TARGET" 'loginctl show-user "$USER" -p Linger'
```

현재 설치처럼 `Linger=yes`여야 로그인 전에도 user service의 enable/disable 상태가 재부팅 후 적용된다. `Linger=no`이거나 user bus 연결 오류가 나오면 임의로 권한을 변경하지 말고 Mini PC 관리자에게 linger 설정을 요청한다.

## 3. 전원 재투입 후 매번 수행할 시작 절차

`astra-web.service` 비활성화는 유지되지만, 현재 TRON1 stack은 자동 시작하지 않는다. Workstation과 Mini PC 전원을 다시 켠 뒤 아래 절차를 매번 실행한다.

### 3.1 실차 안전 준비

1. 로봇 주변과 계단 접근 구역에서 사람과 장애물을 치운다.
2. 현장 감시자가 비상 정지 수단을 확보한다.
3. 로봇을 `home_3f` 위치와 방향에 실제로 배치한다.

```text
home_3f
x   = 2.599130
y   = -1.983942
yaw = 1.455898 rad
```

### 3.2 Software 및 hardware preflight

먼저 Astra service와 기존 process가 없는지 매번 확인한다.

```bash
ASTRA_STATE="$(ssh "$SSH_TARGET" 'systemctl --user is-active astra-web.service || true')"
test "$ASTRA_STATE" = inactive || {
  printf 'Astra stack is not inactive: %s\n' "$ASTRA_STATE" >&2
  exit 1
}

ssh "$SSH_TARGET" \
  "test -z \"\$(pgrep -af '[r]oslaunch.*declan_ws|[f]ast_lio|[g]uarded_scan|[o]dom_guard|[a]priltag_ros_continuous_node|[m]ove_base|[p]ointcloud_to_laserscan|[l]ivox_lidar_publisher2|[r]ealsense2_camera' || true)\""
```

두 명령이 성공한 경우에만 preflight를 진행한다.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
./run.sh --check
./run.sh --preflight
```

필수 성공 출력:

```text
[CHECK] bundle, fixture, packages, and local software: OK
[CHECK] production site profile: OK
[CHECK] mini PC, clock, SSH, and robot route: OK
```

### 3.3 TRON1 stack 시작

같은 terminal에서 실행하고 foreground로 유지한다.

```bash
./run.sh
```

`run.sh`는 다음 순서로 소유권을 구성한다.

1. Workstation에서 `http://192.168.1.26:11311` ROS master 시작
2. Mini PC에서 `sensor_integration wf_mapping.launch` 재사용 또는 시작
3. Mini PC에 `ROS_IP=192.168.1.56` 설정
4. Robot WebSocket SSH tunnel 생성
5. Workstation에서 mission, navigation, floor-transition, stair-supervisor, AprilTag 시작
6. Action, state, sensor, TF, costmap readiness 확인

다음 네 줄이 모두 출력되기 전에는 goal을 보내지 않는다.

```text
[READY] Mission, floor-transition, and stair action servers are ready.
[READY] FloorState=READY and SupervisorState=NAV.
[READY] Fresh scan, odometry, TF, and AprilTag input verified.
[READY] Submit goals only through /mission.
```

## 4. 연결 상태 확인

새 workstation terminal을 열어 환경을 구성한다.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
set -a
source config.env
set +a
export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}"
export ROS_IP="${ROS_MASTER_HOST}"
unset ROS_HOSTNAME
SSH_TARGET="${MINI_PC_USER}@${MINI_PC_HOST}"
```

### 4.1 Action과 핵심 node

```bash
python3 verify_action_servers.py
rosnode list
rosnode info /mission_manager
```

### 4.2 Floor와 command ownership

```bash
timeout 10 rostopic echo -n 1 /multifloor/floor_state
timeout 10 rostopic echo -n 1 /stair_supervisor/state
```

이동 전 필수값:

```text
FloorState.floor_id: "3F"
FloorState.state: 2
SupervisorState.state: 1
SupervisorState.connected: true
```

### 4.3 Sensor, TF, `/scan` 단일 publisher

```bash
rostopic info /scan
timeout 10 rostopic echo -n 1 /scan/header
timeout 10 rostopic echo -n 1 /tron/wheel_odom_raw/header
timeout 10 rostopic echo -n 1 /tag_detections/header
timeout 5 rosrun tf tf_echo map base_Link
timeout 10 rostopic echo -n 1 /amcl_pose
```

`rostopic info /scan`의 publisher는 정확히 하나여야 한다.

`/amcl_pose`와 `map → base_Link`가 실제 HOME 표식의 위치·방향과 일치하는지 RViz에서 교차 확인한다. 현장 승인 허용오차가 정의되지 않았거나 두 값이 일치하지 않으면 goal을 보내지 않는다. 초기 pose parameter가 `home_3f`라는 사실만으로 실제 배치를 확인한 것으로 처리하지 않는다.

### 4.4 Mini PC에서 현재 stack 확인

```bash
ssh "$SSH_TARGET" \
  "pgrep -af '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch sensor_integration wf_mapping.launch$'"
```

원격 launch의 ROS 환경 확인:

```bash
mapfile -t REMOTE_PIDS < <(ssh "$SSH_TARGET" \
  "pgrep -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch sensor_integration wf_mapping.launch$' || true")

case "${#REMOTE_PIDS[@]}" in
  0) printf 'wf_mapping is not running\n' >&2; exit 1 ;;
  1) REMOTE_PID="${REMOTE_PIDS[0]}" ;;
  *) printf 'duplicate wf_mapping processes: %s\n' "${REMOTE_PIDS[*]}" >&2; exit 1 ;;
esac

ssh "$SSH_TARGET" \
  "tr '\0' '\n' < /proc/${REMOTE_PID}/environ | grep -E '^(ROS_MASTER_URI|ROS_IP)='"
```

정상값:

```text
ROS_MASTER_URI=http://192.168.1.26:11311
ROS_IP=192.168.1.56
```

`ROS_MASTER_URI=http://10.192.1.2:11311` 또는 `ROS_IP=10.192.1.20`이 나오면 기존 Astra topology가 아직 남은 것이다. Goal을 보내지 말고 2절부터 다시 확인한다.

## 5. HOME에서 3F 계단 진입점까지 이동

목표는 `home_3f → stair_3f_up_entry`의 `NAV` edge 하나다. 이 goal은 계단 주행을 시작하지 않는다.

```text
stair_3f_up_entry
x   = 10.054808
y   = -6.652934
yaw = 1.395796 rad
```

### 5.1 결과 감시

Goal을 보내기 전에 별도 terminal을 열고 4절 첫 command block과 동일하게 ROS 환경을 구성한 뒤 실행한다.

```bash
rostopic echo /mission/result
```

필요하면 추가 terminal에서 feedback을 확인한다.

```bash
rostopic echo /mission/feedback
```

### 5.2 Mission goal 전송

4절에서 환경을 구성한 terminal에서 실행한다.

```bash
GOAL_ID="home-to-stair-3f-$(date +%s%N)"

rostopic pub -1 /mission/goal mission_manager/MissionActionGoal \
  "{goal_id: {stamp: now, id: '${GOAL_ID}'}, \
    goal: {destination_id: stair_3f_up_entry, \
           mission_type: navigate, \
           return_after_task: false}}"

printf 'GOAL_ID=%s\n' "$GOAL_ID"
```

벽으로 과도하게 접근하거나 같은 회전을 반복하면 즉시 취소한다.

```bash
rostopic pub -1 /mission/cancel actionlib_msgs/GoalID \
  "{stamp: {secs: 0, nsecs: 0}, id: '${GOAL_ID}'}"
```

같은 goal ID 취소가 적용되지 않으면 모든 active mission을 취소한다.

```bash
rostopic pub -1 /mission/cancel actionlib_msgs/GoalID \
  "{stamp: {secs: 0, nsecs: 0}, id: ''}"
```

그래도 로봇이 정지하지 않으면 현장 감시자가 즉시 물리 비상 정지를 실행한다. 그 다음 `./run.sh` terminal에서 `Ctrl+C`를 누른다. Software 명령을 기다리느라 물리 비상 정지를 지연하지 않는다.

취소 후 정지와 command ownership을 확인한다.

```bash
rostopic echo -n 1 /tron/wheel_odom_raw/twist/twist
rostopic echo -n 1 /stair_supervisor/state
```

## 6. 정상 종료

`./run.sh`를 실행한 terminal에서 `Ctrl+C`를 한 번 누른다. Workstation의 top-level launch와 Robot WebSocket tunnel은 종료된다.

Mini PC의 `wf_mapping` sensor stack은 다음 실행에서 재사용하기 위해 남는다. Mini PC 전원을 끄면 종료되며, 다음 `./run.sh`가 다시 시작한다.

Mini PC sensor stack까지 명시적으로 종료하려면 다음을 실행한다.

```bash
ssh "$SSH_TARGET" \
  "tmux kill-session -t wf_mapping 2>/dev/null || true; \
   pkill -INT -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch sensor_integration wf_mapping.launch$' 2>/dev/null || true; \
   sleep 5; \
   pkill -TERM -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch sensor_integration wf_mapping.launch$' 2>/dev/null || true"
```

종료가 실제 적용됐는지 확인한다. 출력이 남거나 명령이 실패하면 Astra stack을 활성화하지 않는다.

```bash
ssh "$SSH_TARGET" \
  "test -z \"\$(pgrep -af '[s]ensor_integration|[w]f_mapping|[p]ointcloud_to_laserscan|[l]ivox_lidar_publisher2|[r]ealsense2_camera' || true)\""
```

## 7. 기존 Astra stack으로 되돌리기

두 stack을 동시에 실행하면 안 된다. 먼저 `./run.sh` terminal에서 `Ctrl+C`를 누르고, 6절의 Mini PC sensor 종료와 종료 확인 명령을 모두 실행한다. 종료 확인이 성공한 경우에만 Astra service를 다시 활성화한다.

```bash
ssh "$SSH_TARGET" 'systemctl --user enable --now astra-web.service'
```

확인:

```bash
ssh "$SSH_TARGET" 'systemctl --user status astra-web.service --no-pager'
```

Astra master에서 `/scan` publisher가 정확히 하나인지 확인한다.

```bash
ssh "$SSH_TARGET" \
  "source /opt/ros/noetic/setup.bash; \
   export ROS_MASTER_URI=http://10.192.1.2:11311 ROS_IP=10.192.1.20; \
   timeout 10 rostopic info /scan"
```

다시 현재 TRON1 stack으로 전환할 때는 2절의 `disable --now`를 실행한다.

## 8. 빠른 문제 판별

### `./run.sh`가 기존 AprilTag detector를 보고 중단

```bash
ssh "$SSH_TARGET" 'systemctl --user is-active astra-web.service || true'
```

`active`이면 2절에 따라 Astra service를 정지한다.

### Mini PC process가 workstation master를 사용하지 않음

4.4절에서 process 환경을 확인한다. 정상 master는 `192.168.1.26:11311`, 정상 Mini PC ROS IP는 `192.168.1.56`이다.

### `/scan` publisher가 두 개 이상

```bash
rostopic info /scan
ssh "$SSH_TARGET" \
  "pgrep -af '[d]eclan_ws|[s]ensor_integration|[g]uarded_scan|[p]ointcloud_to_laserscan' || true"
```

기존 Astra process와 현재 `wf_mapping`이 동시에 실행 중인지 확인한다.

### 재부팅 후 아무 stack도 시작되지 않음

정상이다. `astra-web.service`는 의도적으로 비활성화돼 있으며 현재 TRON1 stack은 workstation의 `./run.sh`로 매번 시작한다.
