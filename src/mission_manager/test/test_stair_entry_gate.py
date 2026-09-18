"""Unit tests for the navigation-to-stair map-pose admission gate."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "mission_manager" / "src"))

from mission_manager.stair_entry_gate import (  # noqa: E402
    StairEntryCheck,
    StairEntryEvidence,
    StairEntryFence,
    StairEntryPolicy,
    StairEntryPose,
    evaluate_stair_entry,
)


POLICY = StairEntryPolicy(
    xy_tolerance_m=0.25,
    yaw_tolerance_rad=0.20,
    freshness_sec=0.50,
    max_covariance_x=0.05,
    max_covariance_y=0.05,
    max_covariance_yaw=0.10,
    required_pose_samples=3,
    max_linear_speed=0.01,
    max_angular_speed=0.02,
)
FENCE = StairEntryFence("3F", 7, 100.0, 20.0, 12)
EXPECTED = StairEntryPose(1.0, 2.0, math.pi - 0.10)


def evidence(**changes) -> StairEntryEvidence:
    values = {
        "frame_id": "map",
        "floor_id": "3F",
        "map_generation": 7,
        "pose_stamp_sec": 100.20,
        "pose_received_monotonic": 20.20,
        "pose_sequence": 15,
        "x_m": 1.0,
        "y_m": 2.0,
        "orientation_x": 0.0,
        "orientation_y": 0.0,
        "orientation_z": math.sin((-math.pi + 0.10) / 2.0),
        "orientation_w": math.cos((-math.pi + 0.10) / 2.0),
        "covariance_x": 0.05,
        "covariance_y": 0.05,
        "covariance_yaw": 0.10,
        "odom_received_monotonic": 20.20,
        "linear_speed_mps": 0.01,
        "angular_speed_radps": 0.02,
    }
    values.update(changes)
    return StairEntryEvidence(**values)


class StairEntryGateTest(unittest.TestCase):
    def test_aligned_fresh_pose_accepts_wrapped_heading_at_inclusive_bounds(self) -> None:
        # Given: three post-fence AMCL samples and stationary odometry at policy bounds.
        observed = evidence(x_m=1.25)

        # When: the live map pose is compared with the canonical stair entry.
        decision = evaluate_stair_entry(
            StairEntryCheck(POLICY, FENCE, EXPECTED, observed, 100.50, 20.50)
        )

        # Then: wrapped yaw and inclusive distance/covariance bounds are accepted.
        self.assertTrue(decision.accepted, decision.reason)

    def test_pose_or_localization_failure_rejects_entry(self) -> None:
        cases = (
            ("wrong frame", {"frame_id": "odom"}),
            ("wrong floor", {"floor_id": "4F"}),
            ("wrong generation", {"map_generation": 8}),
            ("pre-fence pose", {"pose_sequence": 12}),
            ("position mismatch", {"x_m": 1.251}),
            ("heading mismatch", {"orientation_z": math.sin(2.70 / 2.0), "orientation_w": math.cos(2.70 / 2.0)}),
            ("high covariance", {"covariance_x": 0.051}),
            ("negative covariance", {"covariance_y": -0.001}),
            ("moving", {"linear_speed_mps": 0.011}),
            ("stale odometry", {"odom_received_monotonic": 19.99}),
        )
        for reason, changes in cases:
            with self.subTest(reason=reason):
                # Given: exactly one unsafe entry-evidence class.
                observed = evidence(**changes)

                # When: admission is evaluated.
                decision = evaluate_stair_entry(
                    StairEntryCheck(POLICY, FENCE, EXPECTED, observed, 100.50, 20.50)
                )

                # Then: stair entry remains fail-closed.
                self.assertFalse(decision.accepted)


if __name__ == "__main__":
    unittest.main()
