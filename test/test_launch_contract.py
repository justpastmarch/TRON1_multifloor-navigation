from pathlib import Path
import unittest
from xml.etree import ElementTree

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "multifloor_manager"
LAUNCH = PACKAGE / "launch"
FIXTURE = ROOT / "test" / "fixtures" / "building_valid"


def parse_launch(name: str) -> ElementTree.Element:
    return ElementTree.parse(LAUNCH / name).getroot()


def node(root: ElementTree.Element, package: str) -> ElementTree.Element:
    return next(candidate for candidate in root.findall("node") if candidate.get("pkg") == package)


class LaunchContractTest(unittest.TestCase):
    def test_flat_navigation_speed_limit_is_consistent(self) -> None:
        # Given: every deployment-owned source of the flat-navigation speed limit.
        launch = parse_launch("navigation.launch")
        arguments = {argument.get("name"): argument.get("default") for argument in launch.findall("arg")}
        system = ElementTree.parse(
            ROOT / "src" / "mission_manager" / "launch" / "system.launch"
        ).getroot()
        system_arguments = {
            argument.get("name"): argument.get("default") for argument in system.findall("arg")
        }
        planner = yaml.safe_load(
            (PACKAGE / "config" / "nav" / "base_local_planner_params.yaml").read_text(encoding="utf-8")
        )
        robot = yaml.safe_load(
            (ROOT / "src" / "stair_supervisor" / "config" / "robot.yaml").read_text(encoding="utf-8")
        )
        environment = dict(
            line.split("=", 1)
            for line in (ROOT / "config.env").read_text(encoding="utf-8").splitlines()
            if line and not line.startswith("#")
        )

        # When: effective runtime values and the planner's conservative fallback are compared.
        effective_values = (
            float(arguments["max_vel_x"]),
            float(system_arguments["max_vel_x"]),
            float(robot["move_base"]["max_vel_x"]),
            float(environment["MAX_VEL_X"]),
        )

        # Then: launch overrides the conservative YAML fallback with the commissioned flat limit.
        self.assertEqual(effective_values, (0.30, 0.30, 0.30, 0.30))
        self.assertEqual(float(planner["TrajectoryPlannerROS"]["max_vel_x"]), 0.30)

    def test_explicit_costmap_plugins_exclude_legacy_static_map_parameters(self) -> None:
        # Given: common and namespace-specific costmap parameter documents.
        nav_config = PACKAGE / "config" / "nav"
        documents = {
            name: yaml.safe_load((nav_config / name).read_text(encoding="utf-8"))
            for name in (
                "costmap_common_params.yaml",
                "global_costmap_params.yaml",
                "local_costmap_params.yaml",
            )
        }

        # When: explicit plugin ownership and legacy static-map keys are inspected.
        explicit_plugins = (
            documents["global_costmap_params.yaml"]["global_costmap"]["plugins"],
            documents["local_costmap_params.yaml"]["local_costmap"]["plugins"],
        )
        legacy_files = tuple(
            name
            for name, parameters in (
                ("costmap_common_params.yaml", documents["costmap_common_params.yaml"]),
                ("global_costmap_params.yaml", documents["global_costmap_params.yaml"]["global_costmap"]),
                ("local_costmap_params.yaml", documents["local_costmap_params.yaml"]["local_costmap"]),
            )
            if "static_map" in parameters
        )

        # Then: plugin-configured costmaps cannot load the ignored Pre-Hydro parameter.
        self.assertTrue(all(explicit_plugins))
        self.assertEqual(legacy_files, ())
        self.assertNotIn("observation_sources", documents["costmap_common_params.yaml"])
        self.assertEqual(
            documents["local_costmap_params.yaml"]["local_costmap"]["obstacle_layer"]["scan"]["topic"],
            "/scan",
        )
        self.assertEqual(
            documents["local_costmap_params.yaml"]["local_costmap"]["obstacle_layer"]["observation_sources"],
            "scan",
        )
        self.assertIn("inflation_radius", documents["global_costmap_params.yaml"]["global_costmap"]["inflation_layer"])
        self.assertIn("inflation_radius", documents["local_costmap_params.yaml"]["local_costmap"]["inflation_layer"])

    def test_navigation_uses_dynamic_amcl_and_isolated_command_output(self) -> None:
        # Given: the standard navigation launch description.
        launch = parse_launch("navigation.launch")
        amcl = node(launch, "amcl")
        move_base = node(launch, "move_base")

        # When: machine-consumed parameters and remaps are indexed by name.
        amcl_params = {param.get("name"): param.get("value") for param in amcl.findall("param")}
        move_base_params = {param.get("name"): param.get("value") for param in move_base.findall("param")}
        move_base_remaps = {remap.get("from"): remap.get("to") for remap in move_base.findall("remap")}

        # Then: map replacement remains live and only the supervisor-facing command topic is emitted.
        self.assertEqual(amcl_params["use_map_topic"], "true")
        self.assertEqual(amcl_params["first_map_only"], "false")
        self.assertEqual(amcl_params["laser_max_beams"], "60")
        self.assertEqual(amcl_params["laser_max_range"], "12.0")
        self.assertEqual(amcl_params["min_particles"], "500")
        self.assertEqual(amcl_params["max_particles"], "2000")
        self.assertNotIn("max_beams", amcl_params)
        self.assertEqual(move_base_remaps["cmd_vel"], "/navigation/cmd_vel")
        self.assertEqual(move_base_remaps["odom"], "/tron/wheel_odom_raw")
        self.assertEqual(move_base_params["TrajectoryPlannerROS/min_vel_theta"], "-$(arg max_vel_theta)")
        self.assertEqual(move_base_params["recovery_behavior_enabled"], "false")

    def test_navigation_resyncs_rviz_tf_without_adding_a_ros_node(self) -> None:
        # Given: the standard navigation launch and its existing move_base TF gate.
        launch = parse_launch("navigation.launch")
        wait_script = (PACKAGE / "scripts" / "wait_for_tf_exec.sh").read_text(encoding="utf-8")

        # When: launch-owned processes and the gate's pre-exec actions are inspected.
        launched_nodes = {
            (candidate.get("pkg"), candidate.get("type"), candidate.get("name"))
            for candidate in launch.findall("node")
        }

        # Then: the existing gate performs the finite RViz resync before move_base replaces it.
        self.assertEqual(
            launched_nodes,
            {
                ("map_server", "map_server", "map_server"),
                ("amcl", "amcl", "amcl"),
                ("move_base", "move_base", "move_base"),
            },
        )
        self.assertIn("rviz_tf_reconnect.py", wait_script)
        self.assertLess(wait_script.index("rviz_tf_reconnect.py"), wait_script.index('exec "$@"'))

    def test_apriltag_detector_uses_required_camera_topics_and_fixture_tag_config(self) -> None:
        # Given: the standard continuous detector launch and generated fixture config.
        launch = parse_launch("apriltag.launch")
        relay = node(launch, "multifloor_manager")
        detector = node(launch, "apriltag_ros")
        tag_config = yaml.safe_load((FIXTURE / "apriltag_ros_tags.yaml").read_text(encoding="utf-8"))

        # When: required source arguments and detector remaps are inspected.
        arguments = {argument.get("name"): argument.get("default") for argument in launch.findall("arg")}
        parameters = {parameter.get("name"): parameter.get("value") for parameter in detector.findall("param")}
        relay_remaps = {remap.get("from"): remap.get("to") for remap in relay.findall("remap")}
        detector_remaps = {remap.get("from"): remap.get("to") for remap in detector.findall("remap")}
        loaded_files = {rosparam.get("file") for rosparam in detector.findall("rosparam")}

        # Then: callers must supply both camera streams and an ID/size config generated from test data.
        self.assertIsNone(arguments["image_rect"])
        self.assertIsNone(arguments["camera_info"])
        self.assertIsNone(arguments["tag_config"])
        self.assertEqual(parameters["tag_family"], "tagStandard41h12")
        self.assertEqual(relay_remaps["~image"], "$(arg image_rect)")
        self.assertEqual(relay_remaps["~camera_info"], "$(arg camera_info)")
        self.assertEqual(relay_remaps["~image_out"], "/apriltag_camera/image_raw")
        self.assertEqual(relay_remaps["~camera_info_out"], "/apriltag_camera/camera_info")
        self.assertEqual(detector_remaps["image_rect"], "/apriltag_camera/image_raw")
        self.assertEqual(detector_remaps["camera_info"], "/apriltag_camera/camera_info")
        self.assertIn("$(arg tag_config)", loaded_files)
        self.assertEqual(
            tag_config["standalone_tags"],
            [
                {"id": 100, "size": 0.16},
                {"id": 101, "size": 0.16},
                {"id": 201, "size": 0.20},
                {"id": 202, "size": 0.20},
            ],
        )

    def test_launch_inventory_has_one_scan_publisher_and_no_fast_lio_reference(self) -> None:
        # Given: every launch and navigation YAML asset owned by multifloor_manager.
        assets = tuple(LAUNCH.glob("*.launch")) + tuple((PACKAGE / "config" / "nav").glob("*.yaml"))

        # When: scan-output remaps and source text are inspected.
        scan_publishers = []
        source = ""
        for asset in assets:
            source += asset.read_text(encoding="utf-8").lower()
            if asset.suffix == ".launch":
                launch = ElementTree.parse(asset).getroot()
                scan_publishers.extend(
                    candidate
                    for candidate in launch.findall(".//node")
                    if any(remap.get("from") == "scan" and remap.get("to") == "/scan" for remap in candidate.findall("remap"))
                )

        # Then: the remote sensor stack owns /scan and no local converter competes with it.
        self.assertEqual(scan_publishers, [])
        self.assertNotIn("fast_lio", source)
        self.assertNotIn("fast-lio", source)


if __name__ == "__main__":
    unittest.main()
