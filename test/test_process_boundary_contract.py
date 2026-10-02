from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MISSION_ROOT = ROOT / "src/mission_manager"
sys.path.insert(0, str(ROOT))

from validate_bundle import validate_node_entrypoints, validate_paths  # noqa: E402


class ProcessBoundaryContractTest(unittest.TestCase):
    def test_rostest_process_boundary_harness_is_registered(self) -> None:
        # Given: the mission package that owns the generated Mission action.
        cmake = (MISSION_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        required_paths = (
            MISSION_ROOT / "test/process_boundaries.test",
            MISSION_ROOT / "test/mock_peer_server.py",
            MISSION_ROOT / "test/rostest_python.sh",
            MISSION_ROOT / "test/test_process_boundaries.py",
        )

        # When/Then: rostest and both process roles are present in test scope.
        self.assertIn("add_rostest(", cmake)
        self.assertIn("test/process_boundaries.test", cmake)
        self.assertTrue(all(path.is_file() for path in required_paths))

    def test_test_scope_ros_initializers_do_not_expand_deployed_inventory(self) -> None:
        # Given: approved production entrypoints and a test-only ROS process.
        with tempfile.TemporaryDirectory() as directory:
            workspace_root = Path(directory)
            production_nodes = {
                "mission_manager": "mission_manager",
                "multifloor_manager": "multifloor_manager",
                "stair_supervisor": "stair_supervisor",
            }
            for package, node_name in production_nodes.items():
                source = workspace_root / "src" / package / "scripts" / f"{package}_node.py"
                source.parent.mkdir(parents=True)
                source.write_text(
                    f'import rospy\nrospy.init_node("{node_name}")\n',
                    encoding="utf-8",
                )
            relay = workspace_root / "src/multifloor_manager/scripts/camera_info_stamp_relay.py"
            relay.write_text('import rospy\nrospy.init_node("camera_info_stamp_relay")\n', encoding="utf-8")
            test_process = workspace_root / "src/mission_manager/test/mock_peer_server.py"
            test_process.parent.mkdir(parents=True)
            test_process.write_text(
                'import rospy\nrospy.init_node("todo4_mock_peer_server")\n',
                encoding="utf-8",
            )

            # When/Then: deployed-node validation ignores explicit test scope.
            validate_node_entrypoints(workspace_root)

    def test_existing_operator_client_is_allowed_but_not_auto_launched(self):
        from bundle_source_contract import EXPECTED_NODE_INITIALIZERS
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for relative, name in EXPECTED_NODE_INITIALIZERS | {
                ("src/stair_supervisor/scripts/stair_entry_test.py", "stair_entry_test_client")
            }:
                p = root / relative
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(f'import rospy\nrospy.init_node("{name}")\n')
            validate_node_entrypoints(root)
            launch = root / "src/stair_supervisor/operator.launch"
            launch.write_text('<launch><node pkg="stair_supervisor" type="stair_entry_test.py" name="unexpected"/></launch>')
            with self.assertRaisesRegex(SystemExit, "must not be auto-launched"):
                validate_node_entrypoints(root)

    def test_registration_client_is_allowed_only_as_an_explicit_operator_tool(self):
        from bundle_source_contract import EXPECTED_NODE_INITIALIZERS
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for relative,name in EXPECTED_NODE_INITIALIZERS | {
                ("src/stair_supervisor/scripts/prepare_stair_mission.py","prepare_stair_mission")
            }:
                p=root/relative;p.parent.mkdir(parents=True,exist_ok=True)
                p.write_text(f'import rospy\nrospy.init_node("{name}")\n')
            validate_node_entrypoints(root)
            launch=root/"src/stair_supervisor/operator.launch"
            launch.write_text('<launch><node pkg="stair_supervisor" type="prepare_stair_mission.py" name="unexpected"/></launch>')
            with self.assertRaisesRegex(SystemExit,"must not be auto-launched"):
                validate_node_entrypoints(root)

    def test_archived_document_paths_do_not_relax_runtime_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            docs = root / "docs"
            docs.mkdir()
            traversal = "." * 2 + "/"
            (docs / "reproduce.py").write_text(f'reference="{traversal}report.md"')
            validate_paths(root)
            (root / "runtime.py").write_text(f'reference="{traversal}outside"')
            with self.assertRaisesRegex(SystemExit, "parent traversal"):
                validate_paths(root)


if __name__ == "__main__":
    unittest.main()
