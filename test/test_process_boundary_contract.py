from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MISSION_ROOT = ROOT / "src/mission_manager"
sys.path.insert(0, str(ROOT))

from validate_bundle import validate_node_entrypoints  # noqa: E402


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
            test_process = workspace_root / "src/mission_manager/test/mock_peer_server.py"
            test_process.parent.mkdir(parents=True)
            test_process.write_text(
                'import rospy\nrospy.init_node("todo4_mock_peer_server")\n',
                encoding="utf-8",
            )

            # When/Then: deployed-node validation ignores explicit test scope.
            validate_node_entrypoints(workspace_root)


if __name__ == "__main__":
    unittest.main()
