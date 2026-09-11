from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import unittest


class YawCalibrationCliTest(unittest.TestCase):
    def test_help_exposes_safe_calibration_controls(self) -> None:
        # Given: the installed-package source script.
        package_root = Path(__file__).resolve().parents[2]
        script = package_root / "scripts" / "calibrate_yaw_rate.py"

        # When: an operator asks for usage without opening a robot connection.
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )

        # Then: the CLI is available and documents its movement controls.
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--url", completed.stdout)
        self.assertIn("--active-sec", completed.stdout)
        self.assertIn("--levels", completed.stdout)


if __name__ == "__main__":
    unittest.main()
