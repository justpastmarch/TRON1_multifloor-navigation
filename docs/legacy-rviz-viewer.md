# Legacy RViz stair-recording operator lane

This manual lane is independent: run each bash block in its own terminal. Every
terminal owns the process it starts. Do not start `mission_manager/system.launch`,
a legacy bridge executable, or any `rostopic` goal publication.

## 1. Remote sensor stack

The local shell owns the SSH session; the remote foreground `roslaunch` owns the
sensor stack process.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source config.env
export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}
export ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" | awk '{print $7; exit}')"
unset ROS_HOSTNAME
ssh "${MINI_PC_USER}@${MINI_PC_HOST}" "source /opt/ros/noetic/setup.bash && source ${MINI_PC_WORKSPACE}/devel/setup.bash && export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT} && export ROS_IP=${MINI_PC_ROS_IP} && export D435F_SERIAL=${D435F_SERIAL} && roslaunch sensor_integration wf_mapping.launch"
```

## 2. Local navigation

This starts only the existing navigation launch. It uses the 3F map, navigation
parameter directory, configured start pose, and conservative values from
`config.env`.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source config.env
export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}
export ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" | awk '{print $7; exit}')"
unset ROS_HOSTNAME
roslaunch multifloor_manager navigation.launch map_yaml:=$(rospack find multifloor_manager)/config/maps/floor_3F.yaml nav_params_dir:=$(rospack find multifloor_manager)/config/nav initial_x:=${NAV_START_X} initial_y:=${NAV_START_Y} initial_yaw:=${NAV_START_YAW} max_vel_x:=${MAX_VEL_X} max_vel_theta:=${MAX_VEL_THETA} min_in_place_vel_theta:=${MIN_IN_PLACE_VEL_THETA} acc_lim_x:=${ACC_LIM_X} acc_lim_theta:=${ACC_LIM_THETA}
```

## 3. Robot WebSocket SSH tunnel

Own this foreground tunnel in its own terminal.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source config.env
export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}
export ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" | awk '{print $7; exit}')"
unset ROS_HOSTNAME
ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes -L "127.0.0.1:${LOCAL_WS_PORT}:${ROBOT_HOST}:${ROBOT_WS_PORT}" "${MINI_PC_USER}@${MINI_PC_HOST}"
```

## 4. Existing standalone stair supervisor

Run the existing node, using the configured robot identity, local tunnel endpoint,
and package-owned configuration.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source config.env
export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}
export ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" | awk '{print $7; exit}')"
unset ROS_HOSTNAME
rosrun stair_supervisor stair_supervisor_node.py _config_dir:=$(rospack find stair_supervisor)/config _accid:=${ACCID} _websocket_url:=ws://127.0.0.1:${LOCAL_WS_PORT}
```

## 5. Manual RViz and stair start

Use the manual viewer's `rviz/SetInitialPose` and `rviz/SetGoal` tools to initialize
at and navigate to the stair start. The existing RViz configuration proves the
`/initialpose` and `/move_base_simple/goal` surfaces; do not publish a goal with
`rostopic`.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source config.env
export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}
export ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" | awk '{print $7; exit}')"
unset ROS_HOSTNAME
rosrun rviz rviz -d $(rospack find multifloor_manager)/rviz/wf_navigation_manual.rviz
```

## 6. Rosbag recording

Write artifacts under the configured scan output root and own this foreground
process.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source config.env
export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}
export ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" | awk '{print $7; exit}')"
unset ROS_HOSTNAME
rosbag record -O "${SCAN_OUTPUT_BASE}/stair-$(date +%Y%m%d-%H%M%S).bag" /scan /tron/wheel_odom_raw /tf /tf_static "${POINTCLOUD_TOPIC}" "${CAMERA_IMAGE_TOPIC}" "${CAMERA_INFO_TOPIC}" /navigation/cmd_vel /initialpose /move_base_simple/goal /stair_supervisor/state
```

Stop the rosbag with `Ctrl-C` after evidence capture.

## Provenance only

The legacy bridge defaults `0.55` linear and `1.57` angular are cited as
provenance only from `/home/m3tron/Desktop/TRON1_Control/ex_TRON1_RViz_Navigation (1)/cmd_vel_bridge.py`
and `/home/m3tron/Desktop/TRON1_Control/tron1-control-center/cmd_vel_bridge.py`.
Those bridge executables are not part of this lane and are not runtime inputs.

## Cleanup ownership

The terminal that started each process owns its cleanup: stop rosbag, then the
standalone stair supervisor, tunnel, local navigation, and remote sensor SSH
launch. Stop only task-owned processes; leave pre-existing processes untouched.
