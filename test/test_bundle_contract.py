from __future__ import annotations

from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree

import yaml

from validate_bundle import (
    validate_actions,
    validate_forbidden_runtime,
    validate_launch,
    validate_paths,
    validate_rviz,
    validate_schema_assets,
)


ROOT = Path(__file__).resolve().parents[1]
PARENT_TRAVERSAL = "." * 2 + "/"


class BundleContractTest(unittest.TestCase):
    def workspace_copy(self, temporary_dir: str) -> Path:
        workspace = Path(temporary_dir) / "workspace"
        shutil.copytree(ROOT / "src", workspace / "src")
        return workspace

    def test_duplicate_scan_publisher_is_rejected(self) -> None:
        # Given: a valid source tree with a second launch-time /scan publisher.
        with tempfile.TemporaryDirectory() as temporary_dir:
            workspace = self.workspace_copy(temporary_dir)
            duplicate = workspace / "src/multifloor_manager/launch/duplicate.launch"
            duplicate.write_text(
                '<launch><node pkg="rogue" type="converter" name="duplicate">'
                '<remap from="scan" to="/scan"/></node></launch>',
                encoding="utf-8",
            )

            # When/Then: the launch contract rejects ambiguous scan ownership.
            with self.assertRaisesRegex(SystemExit, "one /scan publisher"):
                validate_launch(workspace)

    def test_fourth_top_level_node_is_rejected(self) -> None:
        # Given: a valid system launch with an extra runtime node.
        with tempfile.TemporaryDirectory() as temporary_dir:
            workspace = self.workspace_copy(temporary_dir)
            launch_path = workspace / "src/mission_manager/launch/system.launch"
            launch = ElementTree.parse(launch_path)
            ElementTree.SubElement(
                launch.getroot(),
                "node",
                {"pkg": "rogue", "type": "helper", "name": "helper"},
            )
            launch.write(launch_path, encoding="unicode")

            # When/Then: the exact top-level node contract rejects it.
            with self.assertRaisesRegex(SystemExit, "system launch node mismatch"):
                validate_launch(workspace)

    def test_bridge_and_fast_lio_references_are_rejected(self) -> None:
        # Given: isolated copies containing two prohibited runtime paths.
        for name, source, expected in (
            (
                "bridge.launch",
                '<launch><node pkg="tools" type="cmd_vel_bridge" name="bridge"/></launch>',
                "forbidden runtime node",
            ),
            ("navigation_override.yaml", "odometry_source: FAST-LIO\n", "FAST-LIO"),
        ):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temporary_dir:
                workspace = self.workspace_copy(temporary_dir)
                path = workspace / "src/multifloor_manager/launch" / name
                path.write_text(source, encoding="utf-8")

                # When/Then: the production source scan rejects the reference.
                with self.assertRaisesRegex(SystemExit, expected):
                    validate_forbidden_runtime(workspace)

    def test_missing_map_tag_and_action_artifacts_are_rejected(self) -> None:
        # Given: one required artifact is removed from each isolated source tree.
        cases = (
            ("src/multifloor_manager/config/maps/floor_3F.pgm", validate_schema_assets, "map image"),
            ("src/multifloor_manager/config/apriltag_ros_tags.yaml", validate_schema_assets, "AprilTag detector"),
            ("src/mission_manager/action/Mission.action", validate_actions, "action contract"),
        )
        for relative_path, validator, expected in cases:
            with self.subTest(path=relative_path), tempfile.TemporaryDirectory() as temporary_dir:
                workspace = self.workspace_copy(temporary_dir)
                (workspace / relative_path).unlink()

                # When/Then: semantic validation rejects the incomplete bundle.
                with self.assertRaisesRegex(SystemExit, expected):
                    validator(workspace)

    def test_navigation_rviz_uses_mission_only_without_direct_goal_tools(self) -> None:
        # Given: the production navigation RViz configuration.
        rviz_path = ROOT / "src/multifloor_manager/rviz/wf_navigation.rviz"
        rviz = rviz_path.read_text(encoding="utf-8")

        # When: the source configuration is parsed and its visualization contract is validated.
        document = yaml.safe_load(rviz)
        validate_rviz(ROOT)

        # Then: the parsed map display remains the fixed-frame navigation view.
        manager = document["Visualization Manager"]
        displays = manager["Displays"]
        map_display = next(display for display in displays if display["Class"] == "rviz/Map")
        self.assertEqual(map_display["Topic"], "/map")
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        tool_classes = {tool["Class"] for tool in manager["Tools"]}
        self.assertNotIn("rviz/SetGoal", tool_classes)
        self.assertIn("rviz/SetInitialPose", tool_classes)
        self.assertNotIn("/move_base_simple/goal", rviz)

    def test_generated_and_test_only_misleading_references_are_ignored(self) -> None:
        # Given: forbidden-looking text exists only in generated and test trees.
        with tempfile.TemporaryDirectory() as temporary_dir:
            workspace = self.workspace_copy(temporary_dir)
            generated = workspace / "src/build/generated.yaml"
            generated.parent.mkdir(parents=True)
            generated.write_text("navigation: fast_lio\n", encoding="utf-8")
            test_asset = workspace / "src/multifloor_manager/test/misleading.yaml"
            test_asset.write_text("navigation: FAST-LIO\n", encoding="utf-8")

            # When: the production runtime source is validated.
            validate_forbidden_runtime(workspace)

            # Then: fixtures and generated output do not poison the contract.
            self.assertTrue(generated.is_file())

    def test_parent_traversal_remains_precise_and_generated_safe(self) -> None:
        # Given: parent traversal exists in generated output only.
        with tempfile.TemporaryDirectory() as temporary_dir:
            workspace = Path(temporary_dir)
            generated = workspace / "build/generated.yaml"
            generated.parent.mkdir()
            generated.write_text(f"path: {PARENT_TRAVERSAL}generated\n", encoding="utf-8")
            validate_paths(workspace)

            # When: the same traversal enters an operator-owned config.
            (workspace / "config.yaml").write_text(
                f"path: {PARENT_TRAVERSAL}outside\n",
                encoding="utf-8",
            )

            # Then: the precise portable-path gate rejects it.
            with self.assertRaisesRegex(SystemExit, "parent traversal"):
                validate_paths(workspace)

    def test_troubleshooting_markdown_parent_example_is_accepted(self) -> None:
        # Given: the operator troubleshooting document uses a parent-directory example as prose.
        documentation = ROOT / "docs" / "ros-noetic-amcl-rviz-late-publisher-troubleshooting.md"

        # When: the source path contract scans the workspace.
        # Then: documentation examples do not become config path traversal.
        self.assertTrue(documentation.is_file())
        validate_paths(ROOT)


if __name__ == "__main__":
    unittest.main()
