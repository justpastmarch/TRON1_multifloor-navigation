from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from mission_manager.scan_recorder import SubprocessExecutor  # noqa: E402


class SubprocessExecutorTest(unittest.TestCase):
    def test_recording_child_starts_outside_terminal_process_group(self) -> None:
        # Given: a terminal-owned capture process starting rosbag.
        with mock.patch("mission_manager.scan_recorder.subprocess.Popen") as popen:
            # When: the subprocess executor starts the recording child.
            process = SubprocessExecutor().start(("rosbag", "record"))

        # Then: terminal Ctrl+C cannot signal rosbag before its owner finalizes it.
        self.assertIs(process, popen.return_value)
        popen.assert_called_once_with(
            ("rosbag", "record"),
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )


if __name__ == "__main__":
    unittest.main()
