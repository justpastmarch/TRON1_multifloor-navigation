from __future__ import annotations

import math
import unittest

from stair_supervisor.configuration import WebSocketCalibration
from stair_supervisor.robot_conversion import NormalizedTwist, normalize_twist


class RobotConversionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.calibration = WebSocketCalibration(linear_mps=0.5, angular_radps=2.0)

    def test_physical_twist_is_normalized_and_clipped(self) -> None:
        # Given: commands beyond independently calibrated robot full-scale.
        # When: the bridge conversion is applied.
        command = normalize_twist(1.0, -3.0, self.calibration)

        # Then: the documented normalized envelope is clipped with no lateral motion.
        self.assertEqual(command, NormalizedTwist(x=1.0, y=0.0, z=-1.0))

    def test_any_nonfinite_component_fails_closed_to_zero(self) -> None:
        # Given/When: either motion component is nonfinite.
        commands = (
            normalize_twist(math.nan, 0.2, self.calibration),
            normalize_twist(0.2, math.inf, self.calibration),
        )

        # Then: the complete command is zero, never partially forwarded.
        self.assertEqual(commands, (NormalizedTwist.zero(), NormalizedTwist.zero()))


if __name__ == "__main__":
    unittest.main()
