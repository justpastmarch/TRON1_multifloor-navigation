from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class CommandProvenanceContractTest(unittest.TestCase):
    def test_route_recording_profile_contains_command_and_state_evidence(self) -> None:
        # Given: the existing production route recording configuration.
        document = yaml.safe_load(
            (ROOT / "src/mission_manager/config/scan_profiles.yaml").read_text(
                encoding="utf-8"
            )
        )

        # When: the configured route profile topics are inspected.
        topics = set(document["profiles"][0]["topics"])

        # Then: one recorder can correlate commands, state, localization, and sensors.
        self.assertTrue(
            {
                "/navigation/cmd_vel",
                "/stair_supervisor/websocket_tx",
                "/stair_supervisor/state",
                "/mission/feedback",
                "/mission/result",
                "/multifloor/floor_state",
                "/amcl_pose",
                "/tf",
                "/tf_static",
                "/tron/wheel_odom_raw",
                "/tron/imu",
                "/scan",
                "/livox/lidar",
                "/camera1/color/image_raw",
            }.issubset(topics)
        )


if __name__ == "__main__":
    unittest.main()
