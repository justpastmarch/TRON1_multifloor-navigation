import json
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ("mission_manager", "multifloor_manager", "stair_supervisor")


class HarnessRegistrationTest(unittest.TestCase):
    def test_all_packages_register_substantive_fixture_tests(self) -> None:
        # Given: every production catkin package.
        for package in PACKAGES:
            with self.subTest(package=package):
                package_root = ROOT / "src" / package

                # When: package test registration and source are inspected.
                cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
                test_path = package_root / "test/test_fixture_discovery.py"

                # Then: catkin owns a real package-local fixture test target.
                self.assertIn("catkin_add_nosetests(test/test_fixture_discovery.py)", cmake)
                self.assertTrue(test_path.is_file())

    def test_peer_harness_executes_as_a_separate_test_process(self) -> None:
        # Given: the documented test-only peer harness.
        harness = ROOT / "test/mock_peer_harness.py"
        self.assertTrue(harness.is_file())

        # When: it is invoked through its real CLI process boundary.
        completed = subprocess.run(
            [sys.executable, str(harness)],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        report = json.loads(completed.stdout)

        # Then: action, service, and WebSocket peers all execute without ROS nodes.
        self.assertEqual(report["action_state"], "SUCCEEDED")
        self.assertEqual(report["service_response"], "accepted")
        self.assertEqual(report["websocket_frames"], 5)

    def test_transcript_cli_accepts_fresh_fixture_root(self) -> None:
        # Given: the checked-in fixture root and loader CLI.
        command = [
            sys.executable,
            str(ROOT / "test/fixture_support.py"),
            "check-transcript",
            str(ROOT / "test/fixtures"),
        ]

        # When: transcript freshness is inspected through the CLI boundary.
        completed = subprocess.run(
            command,
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )

        # Then: all fresh frames are reported rather than silently skipped.
        self.assertEqual(completed.stdout.strip(), "transcript valid frames=5")


if __name__ == "__main__":
    unittest.main()
