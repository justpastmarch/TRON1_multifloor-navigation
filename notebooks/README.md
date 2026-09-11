# TRON1 Operator Notebooks

이 폴더는 ROS 명령을 직접 조합하지 않고도 다음 세 작업을 순서대로 수행하는 수동 운영 화면입니다.

| 하려는 일 | 열 파일 | 로봇 이동 | 결과 |
|---|---|---:|---|
| 2D 지도를 만들고 저장 | `01_mapping_and_map_save.ipynb` | 외부 승인 controller 사용 | `.pgm`, `.yaml` |
| AMCL pose를 설정하고 목적지로 이동 | `02_pose_and_nav_goal.ipynb` | Notebook이 `/move_base` goal 전송 | action result |
| 센서와 주행 데이터를 기록 | `03_rosbag_recording.ipynb` | 실행 중인 외부 작업을 기록 | `.bag` |

## 먼저 알아야 할 것

- Notebook의 회색 code cell은 **Python 코드**입니다. 일반 terminal에 붙여 넣지 않습니다.
- 이 문서의 `bash` code block만 terminal에서 실행합니다.
- Notebook은 Jupyter에서 위에서 아래로 한 셀씩 `Shift+Enter`로 실행합니다.
- `Run All`은 사용하지 않습니다. Mapping은 빈 지도를 저장하고 recording은 바로 종료될 수 있습니다.
- Notebook은 package를 설치하거나 mini PC에 파일을 복사하지 않습니다. 이미 빌드된 로컬 workspace, mini PC sensor workspace, `config.env`, ROS master가 필요합니다.
- Mapping, direct navigation, managed `./run.sh`는 서로 다른 실행 mode입니다. 동시에 사용하지 않습니다.

## 1. 공통 환경 검사

새 terminal을 열고 다음 두 명령만 먼저 실행합니다.

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
bash notebooks/check_environment.sh common
```

출력 의미:

- `[PASS]`: 준비 완료
- `[WARN]`: 현재 작업에 따라 허용할 수 있지만 안내를 확인해야 함
- `[FAIL]`: Notebook을 열기 전에 수정해야 함

검사기는 파일, command, config, clock, ROS master를 **읽기만** 합니다. node를 시작하거나 종료하지 않습니다. 실패 원인과 다음 명령은 각 `[FAIL]` 바로 아래에 출력됩니다.

자주 나오는 최초 준비:

```bash
sudo apt update
sudo apt install jupyter-notebook python3-ipykernel python3-yaml
catkin_make
```

ROS와 package 전체 설치, mini PC 준비, `config.env` provisioning은 [프로젝트 README](../README.md#최초-설치와-build)를 먼저 완료하십시오. `config.env`의 주소, topic, 좌표는 현장값이며 Notebook이 추측하지 않습니다.

## 2. Jupyter 시작

공통 검사를 실행한 terminal에서 Jupyter를 시작합니다. 이 terminal은 Jupyter server가 점유하므로 그대로 열어 둡니다. 이후 `common`, `mapping`, `navigation`, `recording` 검사는 **새 terminal**을 열어 workspace root에서 실행합니다. 검사기 자체가 ROS와 workspace를 source하므로 새 terminal에서도 결과는 같습니다.

Kernel 항목이 없을 때만 Jupyter를 시작하기 전에 다음 명령을 한 번 실행합니다.

```bash
python3 -m ipykernel install --user --name ros-noetic --display-name "Python 3 (ROS Noetic)"
```

그 다음 Jupyter를 시작합니다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
jupyter notebook notebooks
```

브라우저에서 작업할 `.ipynb` 하나만 엽니다. Kernel은 `Python 3 (ROS Noetic)` 또는 workspace를 source한 terminal의 `Python 3`를 선택합니다.

## 3. 작업별 시작 순서

### A. 지도 만들기

1. 현장에서 승인된 mapping controller와 물리적 비상 정지를 준비합니다. 이 repository에는 mapping teleop이 없습니다.
2. 새 terminal을 열고 workspace root에서 다음 검사를 실행합니다.

   ```bash
   bash notebooks/check_environment.sh mapping
   ```

3. 별도 terminal에서 아래 RViz를 열고 Fixed Frame을 `map`으로 둡니다. Mapping 중에는 표시만 보고 `2D Pose Estimate`, `2D Nav Goal` 도구는 사용하지 않습니다.

   ```bash
   cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash
   rviz -d src/multifloor_manager/rviz/wf_navigation_manual.rviz
   ```

4. `[FAIL]`이 없으면 `01_mapping_and_map_save.ipynb`를 엽니다.
5. Notebook의 작은 설정 셀에서 `MAP_NAME`을 바꾸고, controller가 실제로 준비된 뒤에만 `MAPPING_CONTROLLER_READY = True`로 바꿉니다.
6. 센서와 SLAM 셀을 실행한 뒤 셀 실행을 멈추고 외부 controller로 mapping합니다. RViz에서 벽이 겹치지 않고 복도 loop가 같은 위치로 닫히며 누락 영역이 없는지 확인합니다.
7. 저장 셀과 종료 셀을 차례로 실행합니다.

Mapping Notebook은 `/scan`이 없을 때만 SSH로 mini PC의 기존 `sensor_integration/wf_mapping.launch`를 시작합니다. SSH, remote setup, launch 파일이 없으면 검사 단계에서 정확한 위치를 알려주며 자동 설치하지 않습니다.

### B. Pose 설정과 Nav Goal

이 작업은 별도 terminal에서 수동 navigation lane이 먼저 실행 중이어야 합니다. [수동 계단 녹화 runbook](../docs/ROSBAG-RECORD)의 terminal을 번호순으로 실행합니다.

1. Terminal 1: mini PC sensor stack. 이미 `/scan`이 있으면 생략합니다.
2. Terminal 2: `navigation.launch`로 map, AMCL, move_base 시작
3. Terminal 3: robot WebSocket tunnel과 stair supervisor 시작
4. 필요하면 Terminal 4: manual RViz 시작
5. 새 terminal을 열고 workspace root에서 다음 검사를 실행합니다.

   ```bash
   bash notebooks/check_environment.sh navigation
   ```

6. `/map_server`, `/amcl`, `/move_base`, `/stair_supervisor`가 모두 `[PASS]`이고 supervisor가 `NAV(1)`, `connected=True`일 때만 `02_pose_and_nav_goal.ipynb`를 엽니다.
7. Notebook에서 production localization covariance gate를 통과한 뒤 마지막 motion checklist를 읽고 별도 arming 셀의 `ARM_MOTION = True`를 적용합니다.

`./run.sh`, `/mission_manager`, `/multifloor_manager`, `/slam_gmapping`이 실행 중이면 direct navigation을 시작하지 않습니다.

### C. Rosbag 기록

먼저 무엇을 기록할지 선택합니다.

| 기록 mode | 먼저 실행할 것 | 검사 |
|---|---|---|
| 센서만 | mini PC sensor stack | `recording` |
| 평면 navigation 포함 | 위 B의 terminal 1~3 | `navigation`, `recording` |
| AprilTag 포함 | navigation lane + runbook의 detector terminal | `navigation`, `recording` |

새 terminal을 열고 workspace root에서 필요한 검사를 실행합니다.

```bash
bash notebooks/check_environment.sh recording
```

Navigation도 기록한다면 두 검사 모두 통과해야 합니다.

```bash
bash notebooks/check_environment.sh navigation
bash notebooks/check_environment.sh recording
```

그 다음 `03_rosbag_recording.ipynb`를 열고 `RUN_LABEL`을 실제 층, 방향, 작업명으로 바꿉니다. `.bag.active` 출력이 나온 뒤에만 외부 이동 작업을 시작합니다. 작업이 끝나면 로봇을 먼저 정지시키고 Notebook의 정상 종료, 최종 검증 셀을 실행합니다.

## 자동으로 하는 것과 하지 않는 것

| 항목 | Notebook이 자동으로 함 | 운영자가 준비해야 함 |
|---|---|---|
| 공통 | `config.env`를 읽어 `ROS_MASTER_URI`, `ROS_IP` 구성 | ROS master, build, 현장 config |
| Mapping | sensor 재사용/원격 시작, SLAM, map 저장 | 승인된 controller, 주행, map 품질 판단 |
| Navigation | initial pose publish, action goal/result/cancel | 수동 lane, 승인 좌표, 경로 확인, E-stop |
| Recording | recorder 시작, SIGINT finalize, bag 내용 검증 | 기록할 graph와 이동 작업, 10 GiB 이상 공간 |

## 실패했을 때

| 증상 | 다음 행동 |
|---|---|
| `CONFIG`, `ROS_ENV`가 없다고 나옴 | 첫 환경 설정 cell부터 다시 실행 |
| ROS master unreachable | `config.env`의 master 주소와 master process를 확인한 뒤 `common` 검사 재실행 |
| SSH 또는 `wf_mapping.launch` 실패 | mini PC key login과 `${MINI_PC_WORKSPACE}/src/sensor_integration/launch/wf_mapping.launch` 확인 |
| topic type mismatch | 잘못된 deployment를 수정하고 해당 mode 검사부터 재실행 |
| conflicting node | 소유 terminal을 확인해 그 terminal에서 `Ctrl+C`; 소유권을 모르면 종료하지 않음 |
| navigation goal이 움직이지 않음 | 물리적 안전 확보 후 supervisor `state: 1`, `connected: True`와 AMCL pose 확인 |
| recorder가 이미 실행 중 | 새 recorder를 시작하지 말고 기존 recorder의 소유 terminal/kernel에서 정상 종료 |
| `.bag.active`만 남음 | 원본을 삭제하지 말고 Notebook 마지막 복구 안내에 따라 별도 reindex 판단 |

물리적으로 위험하면 Notebook cancel보다 비상 정지를 먼저 사용합니다.

## 종료 소유권

- Notebook 01은 자신이 시작한 `slam_gmapping`만 종료합니다. 원격 sensor stack은 유지합니다.
- Notebook 02는 goal만 취소합니다. navigation, supervisor, SSH tunnel은 각 소유 terminal에서 종료합니다.
- Notebook 03은 자신이 시작한 recorder만 종료하고 finalize합니다.
- 수동 lane 전체 종료는 [수동 계단 녹화 runbook](../docs/ROSBAG-RECORD)의 역순으로 각 terminal에서 `Ctrl+C`를 사용합니다.
