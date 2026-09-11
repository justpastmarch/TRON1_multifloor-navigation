from __future__ import annotations

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree


ROOT = Path(__file__).resolve().parents[1]
SYSTEM_LAUNCH = ROOT / "src" / "mission_manager" / "launch" / "system.launch"


class SystemOperatorContractTest(unittest.TestCase):
    def test_system_launch_owns_exactly_three_custom_nodes(self) -> None:
        # Given: the single operator-facing ROS launch description.
        launch = ElementTree.parse(SYSTEM_LAUNCH).getroot()

        # When: project-owned node declarations are indexed.
        custom_nodes = {
            (node.get("pkg"), node.get("type"), node.get("name"))
            for node in launch.findall("node")
            if node.get("pkg") in {
                "mission_manager",
                "multifloor_manager",
                "stair_supervisor",
            }
        }

        # Then: only the three approved operational entrypoints are present.
        self.assertEqual(
            custom_nodes,
            {
                ("mission_manager", "mission_manager_node.py", "mission_manager"),
                ("multifloor_manager", "multifloor_manager_node.py", "multifloor_manager"),
                ("stair_supervisor", "stair_supervisor_node.py", "stair_supervisor"),
            },
        )

    def test_system_launch_includes_standard_navigation_and_perception(self) -> None:
        # Given: the single operator-facing ROS launch description.
        launch = ElementTree.parse(SYSTEM_LAUNCH).getroot()

        # When: included launch files are inspected.
        includes = {include.get("file") for include in launch.findall("include")}

        # Then: navigation, scan conversion, and AprilTag detection are composed.
        self.assertEqual(
            includes,
            {
                "$(find multifloor_manager)/launch/navigation.launch",
                "$(find multifloor_manager)/launch/pointcloud_to_laserscan.launch",
                "$(find multifloor_manager)/launch/apriltag.launch",
            },
        )

    def test_operator_wrapper_launches_only_the_top_level_system(self) -> None:
        # Given: the root operator wrapper.
        wrapper = (ROOT / "run.sh").read_text(encoding="utf-8")

        # When: ROS launch commands are inspected.
        launch_lines = tuple(
            line.strip()
            for line in wrapper.splitlines()
            if "roslaunch" in line and not line.lstrip().startswith("#")
        )

        # Then: the local runtime has one top-level launch and no bridge bypass.
        self.assertIn("roslaunch mission_manager system.launch", wrapper)
        self.assertNotIn("roslaunch multifloor_manager navigation.launch", wrapper)
        self.assertNotIn("cmd_vel_bridge.py", wrapper)
        self.assertTrue(any("system.launch" in line for line in launch_lines))

    def test_operator_wrapper_gates_all_runtime_readiness_signals(self) -> None:
        # Given: the root operator wrapper.
        wrapper = (ROOT / "run.sh").read_text(encoding="utf-8")

        # When/Then: every machine-consumed readiness seam is present.
        for token in (
            "/mission/status",
            "/multifloor/floor_transition/status",
            "/stair_traversal/status",
            "/multifloor/floor_state",
            "/stair_supervisor/state",
            "wait_for_fresh_message /scan sensor_msgs/LaserScan",
            "wait_for_fresh_message /tron/wheel_odom_raw nav_msgs/Odometry",
            "wait_for_fresh_message /tf tf2_msgs/TFMessage",
            'wait_for_fresh_message "$TAG_DETECTIONS_TOPIC" apriltag_ros/AprilTagDetectionArray',
            "/tf",
            "TAG_DETECTIONS_TOPIC",
            "rostopic info /scan",
        ):
            with self.subTest(token=token):
                self.assertIn(token, wrapper)
        self.assertIn("rostopic hz -w 2", wrapper)

    def test_managed_rviz_is_visualization_only(self) -> None:
        # Given: the managed RViz configuration.
        rviz = (
            ROOT / "src" / "multifloor_manager" / "rviz" / "wf_navigation.rviz"
        ).read_text(encoding="utf-8")

        # When/Then: observation displays remain and command tools are absent.
        for token in (
            "Class: rviz/Map",
            "Class: rviz/LaserScan",
            "Class: rviz/PoseWithCovariance",
            "Class: rviz/Path",
            "Class: rviz/Map",
            "Class: rviz/TF",
        ):
            with self.subTest(token=token):
                self.assertIn(token, rviz)
        self.assertNotIn("Class: rviz/SetGoal", rviz)
        self.assertNotIn("Class: rviz/SetInitialPose", rviz)
        self.assertNotIn("/move_base_simple/goal", rviz)


if __name__ == "__main__":
    unittest.main()
