from __future__ import annotations

import ast
from pathlib import Path
import stat
import tempfile
from typing import Optional, Set, Tuple
import unittest
import xml.etree.ElementTree as ElementTree

from validate_bundle import validate_node_entrypoints, validate_project_packages


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_PACKAGES = {
    "mission_manager",
    "multifloor_manager",
    "stair_supervisor",
}
EXPECTED_NODE_INITIALIZERS = {
    ("src/mission_manager/scripts/mission_manager_node.py", "mission_manager"),
    ("src/multifloor_manager/scripts/multifloor_manager_node.py", "multifloor_manager"),
    ("src/stair_supervisor/scripts/stair_supervisor_node.py", "stair_supervisor"),
    ("src/multifloor_manager/scripts/camera_info_stamp_relay.py", "camera_info_stamp_relay"),
}
EXPECTED_OPERATOR_CLIENTS = {
    ("src/stair_supervisor/scripts/stair_entry_test.py", "stair_entry_test_client"),
}


class WorkspaceLayoutTest(unittest.TestCase):
    def test_exact_project_packages_exist_when_workspace_is_scanned(self) -> None:
        # Given: the workspace source directory.
        source_dir = ROOT / "src"

        # When: catkin package manifests are scanned.
        package_names = {
            ElementTree.parse(manifest).getroot().findtext("name")
            for manifest in source_dir.glob("*/package.xml")
        }

        # Then: only the three approved project packages are present.
        self.assertEqual(package_names, EXPECTED_PACKAGES)

    def test_one_runtime_entrypoint_exists_when_each_package_is_inspected(self) -> None:
        # Given: each approved package directory.
        for package_name in EXPECTED_PACKAGES:
            with self.subTest(package=package_name):
                scripts_dir = ROOT / "src" / package_name / "scripts"

                # Script directories also contain one-shot tools and library
                # checkers; the operational entrypoint remains unchanged.
                entrypoint = scripts_dir / f"{package_name}_node.py"
                self.assertTrue(entrypoint.is_file())
                self.assertTrue(entrypoint.stat().st_mode & stat.S_IXUSR)

    def test_operator_wrapper_sources_ros_and_workspace_setups(self) -> None:
        # Given: the operator-facing root wrapper.
        wrapper = (ROOT / "run.sh").read_text(encoding="utf-8")

        # When: setup sourcing statements are inspected.
        required_setups = (
            "source /opt/ros/noetic/setup.bash",
            'source "${ROOT}/devel/setup.bash"',
        )

        # Then: both ROS and this workspace are sourced.
        self.assertTrue(all(statement in wrapper for statement in required_setups))

    def test_only_approved_runtime_entrypoints_and_operator_client_initialize_ros_nodes(self) -> None:
        # Given: every project Python source file outside generated catkin trees.
        initializers: Set[Tuple[str, Optional[str]]] = set()
        for source_path in ROOT.rglob("*.py"):
            relative_path = source_path.relative_to(ROOT)
            if {"build", "devel", "install", "test"}.intersection(relative_path.parts):
                continue

            # When: direct rospy.init_node calls are extracted from the AST.
            tree = ast.parse(source_path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                function = node.func
                if not (
                    isinstance(function, ast.Attribute)
                    and function.attr == "init_node"
                    and isinstance(function.value, ast.Name)
                    and function.value.id == "rospy"
                ):
                    continue
                node_name = (
                    node.args[0].value
                    if node.args
                    and isinstance(node.args[0], ast.Constant)
                    and isinstance(node.args[0].value, str)
                    else None
                )
                initializers.add((relative_path.as_posix(), node_name))

        # Then: exactly the three approved package entrypoints initialize nodes.
        self.assertEqual(initializers, EXPECTED_NODE_INITIALIZERS | EXPECTED_OPERATOR_CLIENTS)

    def test_operator_wrapper_does_not_launch_standalone_bridge(self) -> None:
        # Given: the operator-facing root wrapper.
        wrapper = (ROOT / "run.sh").read_text(encoding="utf-8")

        # When/Then: no standalone bridge source, component, or PID is deployed.
        for forbidden_token in ("cmd_vel_bridge.py", "command_bridge", "bridge_pid"):
            with self.subTest(token=forbidden_token):
                self.assertNotIn(forbidden_token, wrapper)

    def test_root_bridge_source_is_library_only(self) -> None:
        # Given: the retained compatibility bridge source.
        bridge_path = ROOT / "cmd_vel_bridge.py"
        bridge_source = bridge_path.read_text(encoding="utf-8")

        # When/Then: it has no script marker, executable mode, or main boundary.
        self.assertFalse(bridge_source.startswith("#!"))
        self.assertFalse(bridge_path.stat().st_mode & stat.S_IXUSR)
        self.assertNotIn('if __name__ == "__main__"', bridge_source)

    def test_fourth_package_is_rejected_when_external_fixture_is_validated(self) -> None:
        # Given: an isolated package inventory containing one unapproved package.
        with tempfile.TemporaryDirectory() as temporary_dir:
            workspace_root = Path(temporary_dir)
            for package_name in EXPECTED_PACKAGES | {"rogue_package"}:
                package_dir = workspace_root / "src" / package_name
                package_dir.mkdir(parents=True)
                (package_dir / "package.xml").write_text(
                    f"<package><name>{package_name}</name></package>",
                    encoding="utf-8",
                )

            # When/Then: the production validator rejects the fourth package.
            with self.assertRaisesRegex(SystemExit, "project package mismatch"):
                validate_project_packages(workspace_root)

    def test_extra_node_source_is_rejected_when_external_fixture_is_validated(self) -> None:
        # Given: isolated approved entrypoints plus one root helper node.
        with tempfile.TemporaryDirectory() as temporary_dir:
            workspace_root = Path(temporary_dir)
            for relative_path, node_name in EXPECTED_NODE_INITIALIZERS:
                source_path = workspace_root / relative_path
                source_path.parent.mkdir(parents=True, exist_ok=True)
                source_path.write_text(
                    f'import rospy\nrospy.init_node("{node_name}")\n',
                    encoding="utf-8",
                )
            (workspace_root / "helper_node.py").write_text(
                'import rospy\nrospy.init_node("helper")\n',
                encoding="utf-8",
            )

            # When/Then: the production validator rejects the extra node source.
            with self.assertRaisesRegex(SystemExit, "node entrypoint mismatch"):
                validate_node_entrypoints(workspace_root)


if __name__ == "__main__":
    unittest.main()
