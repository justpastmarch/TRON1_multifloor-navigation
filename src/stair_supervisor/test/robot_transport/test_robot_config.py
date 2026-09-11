from __future__ import annotations

import unittest

from stair_supervisor.robot_config import (
    CommandStreamConfig,
    RobotConnectionConfig,
    RobotTransportConfig,
)


class RobotConfigTest(unittest.TestCase):
    def test_stream_rate_below_documented_minimum_is_rejected(self) -> None:
        # Given: a stream slower than the documented 30 Hz minimum.
        # When/Then: constructing the typed boundary rejects it.
        with self.assertRaisesRegex(ValueError, "at least 30 Hz"):
            CommandStreamConfig(rate_hz=29.9, watchdog_sec=0.25)

    def test_connection_and_stream_configuration_remain_separate(self) -> None:
        # Given: independently calibrated transport and stream settings.
        connection = RobotConnectionConfig("WF_TEST_001", "ws://127.0.0.1:5000")
        stream = CommandStreamConfig(rate_hz=40.0, watchdog_sec=0.25)

        # When: they are composed for the robot transport.
        config = RobotTransportConfig(connection=connection, stream=stream)

        # Then: no move_base velocity limit is treated as robot full-scale.
        self.assertEqual(config.connection.accid, "WF_TEST_001")
        self.assertFalse(hasattr(config, "max_vel_x"))


if __name__ == "__main__":
    unittest.main()
