# ROS Noetic AMCL/RViz late publisher 연결 문제

## 문서 목적

2026-08-20에 TRON1 localization을 디버깅하면서 확인한 RViz와 AMCL의
late publisher 연결 문제, 진단 근거, 영구 수정과 재검증 절차를 기록합니다.

이 문제는 sensor data나 TF 계산 자체가 실패한 경우와 증상이 비슷하지만, 실제
원인은 이미 실행 중인 RViz가 나중에 시작된 AMCL의 `/tf` publisher와 TCPROS
connection을 만들지 못한 것이었습니다.

## 발생 환경

- ROS Noetic master: `http://10.192.1.2:11311`
- laptop ROS IP: `192.168.1.26`
- mini PC ROS IP: `10.192.1.20`
- RViz node: `/rviz_navigation`
- localization node: `/amcl`
- Fixed Frame: `map`
- robot base frame: `base_Link`

## 증상

다음 순서로 실행했을 때 문제가 발생했습니다.

1. mini PC sensor stack 실행
2. RViz 실행
3. `navigation.launch` 실행

RViz에서 Reset을 누르면 map과 display가 다시 보이기도 했지만 다음 오류가
나타났습니다.

```text
Global Status: Error
For frame [base_Link]: Fixed Frame [map] does not exist
```

추가로 Global Options의 Fixed Frame 선택 목록에 `map`이 나타나지 않았습니다.
그러나 별도 TF probe에서는 다음 transform이 모두 정상 조회되었습니다.

```text
map -> odom: OK
map -> base_Link: OK
odom -> base_Link: OK
base_Link -> mid360_link: OK
```

즉, AMCL과 전체 TF chain은 살아 있었지만 RViz 내부 TF buffer만 `map`을 받지
못하고 있었습니다.

## 진단 방법

### 1. ROS graph와 topic 확인

```bash
rosnode list | grep -E '^/(amcl|map_server|move_base|rviz_navigation)$'
rostopic info /tf
rostopic info /map
rostopic info /scan
```

당시 `/tf` publisher 목록에는 `/amcl`이 있었고 subscriber 목록에는
`/rviz_navigation`이 있었습니다. Topic 수준의 등록 정보만 보면 정상처럼
보였습니다.

### 2. 실제 TCPROS connection 확인

```bash
rosnode info /rviz_navigation
```

문제 상태에서는 RViz의 `/tf` inbound connection에 sensor TF publisher들은
있었지만 `/amcl`은 없었습니다.

```text
topic: /tf
  to: /tron1_wf_odom_bridge      # 연결됨
  to: /camera1/realsense2_camera_manager
  to: /amcl                      # 누락
```

반면 정상 상태에서는 다음 connection이 존재합니다.

```text
topic: /tf
  to: /amcl
  direction: inbound
  transport: TCPROS
```

`rostopic info`는 ROS master에 등록된 publisher/subscriber를 보여주지만 실제
node 간 TCPROS socket이 만들어졌는지는 보장하지 않습니다. 이 문제에서는
반드시 `rosnode info`의 connection 목록까지 확인해야 합니다.

### 3. `/initialpose` 역방향 연결 확인

RViz를 AMCL보다 먼저 실행해야 AMCL이 기존 `/initialpose` publisher를 발견할 수
있었습니다.

```bash
rosnode info /amcl
```

정상 상태:

```text
topic: /initialpose
  to: /rviz_navigation
  direction: inbound
  transport: TCPROS
```

이 때문에 단순히 AMCL을 먼저 실행하고 RViz를 나중에 실행하는 방식은 TF 화면은
복구해도 `2D Pose Estimate` 연결을 불안정하게 만들었습니다.

## 원인

관측된 직접 원인은 ROS master의 graph에는 양쪽 node가 등록됐지만 기존 RViz에
AMCL의 late `/tf` publisher 정보가 전달되지 않아 TCPROS connection이 생성되지
않은 것입니다.

TRON1은 ROS master, laptop, mini PC가 서로 다른 주소를 광고하는 multi-machine
구성입니다. 이 환경에서는 새 node가 시작하면서 기존 publisher/subscriber를
조회하는 연결은 성공했지만, 이미 실행 중인 node가 나중에 등록된 peer를 자동으로
받아들이는 late-update 경로가 누락되는 현상이 재현됐습니다.

따라서 다음 두 조건을 동시에 만족해야 했습니다.

1. RViz를 먼저 실행해 AMCL이 `/initialpose` publisher를 발견하게 함
2. AMCL이 시작된 뒤 RViz에 현재 `/tf` publisher 목록을 다시 전달함

## 임시 복구에서 영구 수정까지

디버깅 중에는 RViz XMLRPC API에 표준 `publisherUpdate` callback을 직접 호출해
현재 `/tf` publisher URI 전체를 다시 전달했습니다. 직후 RViz와 AMCL 사이에
TCPROS connection이 생성되고 Fixed Frame 목록과 map display가 복구됐습니다.

이를 startup에 영구 반영한 파일은 다음과 같습니다.

- `src/multifloor_manager/scripts/rviz_tf_reconnect.py`
- `src/multifloor_manager/scripts/wait_for_tf_exec.sh`
- `src/multifloor_manager/CMakeLists.txt`
- `test/test_launch_contract.py`

### 실행 흐름

`move_base`는 기존 `wait_for_tf_exec.sh`를 launch prefix로 사용합니다.

```text
map_server와 AMCL 시작
  -> map TF 준비 대기
  -> RViz에 현재 /tf publisher URI 전체를 publisherUpdate로 전달
  -> AMCL에 RViz /initialpose publisher URI를 전달
  -> move_base가 아직 없으면 warning 후 계속 진행
  -> helper 종료
  -> 기존 move_base process로 exec
```

정상 startup log는 다음과 같습니다.

```text
[WAIT] waiting for TF map -> base_Link before starting .../move_base
[READY] TF map -> base_Link
[READY] RViz subscribed to 4 current /tf publishers
```

### 추가 ROS node를 만들지 않는 이유

`rviz_tf_reconnect.py`는 `rospy.init_node()`를 호출하거나 topic을 계속 구독하지
않습니다. ROS master와 RViz의 XMLRPC API를 한 번 호출한 뒤 즉시 종료합니다.
따라서 ROS graph에는 `/rviz_tf_reconnect` node가 생기지 않습니다.

재연결 실패도 navigation 자체를 막지 않습니다. 이 경우 gate는 warning을 남기고
기존 `move_base`를 계속 시작합니다.

## 지원 실행 순서

startup gate는 기존 순서를 유지합니다.

```text
1. mini PC sensor stack
2. RViz (/rviz_navigation)
3. navigation.launch
```

Navigation보다 RViz가 늦게 시작되면 RViz tool publisher가 master에는 등록되어도
AMCL과 move_base가 새 publisher transport를 받지 못할 수 있습니다. 두 process가
모두 시작된 뒤 다음 one-shot 명령을 실행하면 `/tf`, `/initialpose`,
`/move_base_simple/goal` connection을 함께 보완합니다. pose나 goal message는
publish하지 않습니다.

```bash
rosrun multifloor_manager rviz_tf_reconnect.py
```

## 재시작 후 검증

### ROS node inventory

```bash
rosnode list | grep -E '^/(amcl|map_server|move_base|rviz_navigation|rviz_tf_reconnect)$'
```

예상 결과에는 다음 네 node만 있어야 합니다.

```text
/amcl
/map_server
/move_base
/rviz_navigation
```

`/rviz_tf_reconnect`가 나타나면 안 됩니다.

### RViz와 AMCL의 양방향 연결

```bash
rosnode info /rviz_navigation | grep -A3 -B1 'to: /amcl'
rosnode info /amcl | grep -A3 -B1 'topic: /initialpose'
rosnode info /move_base | grep -A3 -B1 'topic: /move_base_simple/goal'
```

다음을 모두 확인합니다.

- RViz `/tf` inbound source가 `/amcl`
- RViz `/initialpose` outbound destination이 `/amcl`
- AMCL `/initialpose` inbound source가 `/rviz_navigation`
- move_base `/move_base_simple/goal` inbound source가 `/rviz_navigation`
- transport가 `TCPROS`

### RViz 화면

- Global Status에 Error가 없음
- Global Options의 Fixed Frame이 `map`
- map과 laser scan이 함께 표시됨
- AMCL pose와 particle cloud가 갱신됨
- global/local costmap이 표시됨

### software 검증

```bash
bash -n src/multifloor_manager/scripts/wait_for_tf_exec.sh
python3 -m py_compile src/multifloor_manager/scripts/rviz_tf_reconnect.py
python3 -m unittest test.test_rviz_tf_reconnect -v
python3 -m unittest test.test_launch_contract -v
catkin_make
```

수정 당시 launch contract 6개와 `catkin_make`가 통과했고, sensor/RViz/navigation을
모두 종료한 뒤 다시 시작한 live 검증에서도 수동 Reset 없이 map, scan, AMCL TF와
`/initialpose` TCPROS connection이 함께 복구됐습니다.

## 다시 문제가 발생할 때

1. `rostopic info /tf`만 보고 정상으로 판단하지 말고 `rosnode info`로 실제
   TCPROS connection을 확인합니다.
2. Navigation log에서 `[READY] RViz subscribed to ...` 메시지를 확인합니다.
3. 메시지가 없으면 RViz node 이름이 `/rviz_navigation`인지 확인합니다.
4. `[WARN] RViz TF resync skipped`가 있으면 RViz를 먼저 실행한 뒤 navigation을
   다시 시작합니다.
5. AMCL connection은 있는데 `map`이 없으면 `map -> odom` TF와 `/scan` timestamp를
   별도로 점검합니다. 이 경우는 이 문서의 late publisher 문제와 다른 원인입니다.
