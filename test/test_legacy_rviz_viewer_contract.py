from __future__ import annotations

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
RVIZ_DIR = ROOT / "src" / "multifloor_manager" / "rviz"
MANAGED_RVIZ = RVIZ_DIR / "wf_navigation.rviz"
MANUAL_RVIZ = RVIZ_DIR / "wf_navigation_manual.rviz"
DOC = ROOT / "docs" / "legacy-rviz-viewer.md"
ROBOT = ROOT / "src" / "stair_supervisor" / "config" / "robot.yaml"
PREFIX = (
    f"cd {ROOT}",
    "source /opt/ros/noetic/setup.bash",
    "source devel/setup.bash",
    "source config.env",
    "export ROS_MASTER_URI=http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}",
    "export ROS_IP=\"$(ip -4 route get \"$ROS_MASTER_HOST\" | awk '" + "{print $7; exit}" + "')\"",
    "unset ROS_HOSTNAME",
)


def command_blocks(source: str) -> list[list[str]]:
    blocks = []
    in_block = False
    current = []
    for line in source.splitlines():
        if line == "```bash":
            in_block = True
            current = []
        elif line == "```" and in_block:
            blocks.append(current)
            in_block = False
        elif in_block:
            current.append(line)
    return blocks


class LegacyRvizViewerContractTest(unittest.TestCase):
    def test_manual_viewer_preserves_legacy_goal_tools_and_managed_viewer_stays_clean(self) -> None:
        # Given: the approved manual viewer and managed production viewer.
        manual_source = MANUAL_RVIZ.read_text(encoding="utf-8")
        managed_source = MANAGED_RVIZ.read_text(encoding="utf-8")
        manual = yaml.safe_load(manual_source)["Visualization Manager"]
        tools = {tool["Class"] for tool in manual["Tools"]}

        # Then: only the manual viewer has direct-goal tools/topics.
        self.assertEqual(manual["Global Options"]["Fixed Frame"], "map")
        self.assertIn("rviz/SetInitialPose", tools)
        self.assertIn("rviz/SetGoal", tools)
        self.assertIn("/initialpose", manual_source)
        self.assertIn("/move_base_simple/goal", manual_source)
        self.assertNotIn("rviz/SetInitialPose", managed_source)
        self.assertNotIn("rviz/SetGoal", managed_source)
        self.assertNotIn("/move_base_simple/goal", managed_source)

    def test_manual_viewer_preserves_every_display_from_pre_tool_viewer(self) -> None:
        # Given: the managed display baseline and existing navigation-goal display.
        manual = yaml.safe_load(MANUAL_RVIZ.read_text(encoding="utf-8"))["Visualization Manager"]["Displays"]
        managed = yaml.safe_load(MANAGED_RVIZ.read_text(encoding="utf-8"))["Visualization Manager"]["Displays"]
        navigation_goal = {"Alpha": 1, "Axes Length": 1, "Axes Radius": 0.10000000149011612, "Class": "rviz/Pose", "Color": "255; 40; 40", "Enabled": True, "Head Length": 0.30000001192092896, "Head Radius": 0.10000000149011612, "Name": "Navigation Goal", "Queue Size": 10, "Shaft Length": 1, "Shaft Radius": 0.05000000074505806, "Shape": "Arrow", "Topic": "/move_base_simple/goal", "Unreliable": False, "Value": True}

        # Then: the manual addition is limited to the goal display.
        self.assertEqual(manual, managed[:-1] + [navigation_goal, managed[-1]])

    def test_operator_blocks_have_exact_environment_prefix(self) -> None:
        # Given: all executable operator blocks.
        blocks = command_blocks(DOC.read_text(encoding="utf-8"))

        # Then: every terminal begins with the exact workspace/environment contract.
        self.assertEqual(len(blocks), 6)
        for block in blocks:
            self.assertEqual(tuple(block[:7]), PREFIX)

    def test_operator_blocks_have_exact_lane_tokens_and_exclusions(self) -> None:
        # Given: structurally separated command blocks.
        blocks = command_blocks(DOC.read_text(encoding="utf-8"))
        commands = ["\n".join(block) for block in blocks]
        joined = "\n".join(commands)

        # Then: remote, navigation, tunnel, supervisor, RViz, and recording tokens are exact.
        self.assertTrue(any('ssh "${MINI_PC_USER}@${MINI_PC_HOST}"' in block and "${MINI_PC_WORKSPACE}/devel/setup.bash" in block and "export ROS_IP=${MINI_PC_ROS_IP}" in block and "D435F_SERIAL=${D435F_SERIAL}" in block and "roslaunch sensor_integration wf_mapping.launch" in block for block in commands))
        self.assertTrue(any("map_yaml:=$(rospack find multifloor_manager)/config/maps/floor_3F.yaml" in block and "nav_params_dir:=$(rospack find multifloor_manager)/config/nav" in block and all(token in block for token in ("initial_x:=${NAV_START_X}", "initial_y:=${NAV_START_Y}", "initial_yaw:=${NAV_START_YAW}", "max_vel_x:=${MAX_VEL_X}", "max_vel_theta:=${MAX_VEL_THETA}", "min_in_place_vel_theta:=${MIN_IN_PLACE_VEL_THETA}", "acc_lim_x:=${ACC_LIM_X}", "acc_lim_theta:=${ACC_LIM_THETA}")) for block in commands))
        self.assertTrue(any('ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes -L "127.0.0.1:${LOCAL_WS_PORT}:${ROBOT_HOST}:${ROBOT_WS_PORT}" "${MINI_PC_USER}@${MINI_PC_HOST}"' in block for block in commands))
        self.assertTrue(any("rosrun stair_supervisor stair_supervisor_node.py" in block and "_accid:=${ACCID}" in block and "_websocket_url:=ws://127.0.0.1:${LOCAL_WS_PORT}" in block and "_config_dir:=$(rospack find stair_supervisor)/config" in block for block in commands))
        self.assertTrue(any("rosrun rviz rviz -d $(rospack find multifloor_manager)/rviz/wf_navigation_manual.rviz" in block for block in commands))
        self.assertTrue(any("rosbag record" in block and all(token in block for token in ("${SCAN_OUTPUT_BASE}", "/scan", "/tron/wheel_odom_raw", "/tf", "/tf_static", "${POINTCLOUD_TOPIC}", "${CAMERA_IMAGE_TOPIC}", "${CAMERA_INFO_TOPIC}", "/navigation/cmd_vel", "/initialpose", "/move_base_simple/goal", "/stair_supervisor/state")) for block in commands))

        # Then: prohibited executable command tokens are absent.
        for token in ("mission_manager/system.launch", "cmd_vel_bridge.py", "tron1_cmd_vel_bridge", "rostopic pub"):
            self.assertNotIn(token, joined)

    def test_robot_yaml_has_operator_calibration_and_preserved_interface(self) -> None:
        # Given: the deployed stair robot configuration.
        document = yaml.safe_load(ROBOT.read_text(encoding="utf-8"))

        # Then: the approved calibration and existing interface remain exact.
        self.assertTrue(document["configured"])
        self.assertEqual(document["websocket_full_scale"], {"linear_mps": 0.55, "angular_radps": 1.57})
        self.assertEqual(document["move_base"], {"max_vel_x": 0.50, "max_vel_theta": 0.80, "min_in_place_vel_theta": 0.25, "acc_lim_x": 0.4, "acc_lim_theta": 2.0})
        self.assertEqual(document["command_topics"], ["/navigation/cmd_vel", "/stair/cmd_vel"])


if __name__ == "__main__":
    unittest.main()
