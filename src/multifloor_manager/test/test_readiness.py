from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "multifloor_manager" / "src"))

from multifloor_manager.readiness import (  # noqa: E402
    DEFAULT_READINESS_POLICY,
    Nanoseconds,
    arm_localization,
    localization_ready,
    observe_odometry,
    observe_pose,
    observe_scan,
)


POSE_AT = Nanoseconds(1_700_000_000_000_000_000)
NOW = Nanoseconds(int(POSE_AT) + 400_000_000)
GOOD_COVARIANCE = (0.05, 0.05, 0.10)


class LocalizationReadinessTest(unittest.TestCase):
    def test_ready_requires_three_post_pose_samples_and_fresh_motion_evidence(self) -> None:
        # Given: a newly published landing pose and fresh scan/TF/stationary odometry.
        state = arm_localization(POSE_AT)
        state = observe_scan(state, Nanoseconds(int(POSE_AT) + 100), NOW, True)
        state = observe_odometry(state, Nanoseconds(int(POSE_AT) + 200), NOW, 0.0, 0.0)

        # When: three ordered, fresh covariance samples arrive after initialpose.
        for offset in (300, 400, 500):
            state = observe_pose(
                state,
                Nanoseconds(int(POSE_AT) + offset),
                NOW,
                GOOD_COVARIANCE,
            )

        # Then: every readiness predicate is jointly satisfied.
        evidence = localization_ready(state, NOW, DEFAULT_READINESS_POLICY)
        self.assertTrue(evidence.ready)
        self.assertTrue(all(predicate.value for predicate in evidence.predicates()))

    def test_pre_pose_stale_dirty_and_missing_tf_evidence_is_rejected(self) -> None:
        # Given: samples that are pre-arm, stale, moving, over covariance, or lack TF.
        state = arm_localization(POSE_AT)
        stale_received = Nanoseconds(int(POSE_AT) + 2_000_000_000)
        state = observe_scan(state, Nanoseconds(int(POSE_AT) - 1), NOW, True)
        state = observe_scan(state, Nanoseconds(int(POSE_AT) + 1), stale_received, True)
        state = observe_scan(state, Nanoseconds(int(POSE_AT) + 2), NOW, False)
        state = observe_odometry(state, Nanoseconds(int(POSE_AT) + 3), NOW, 0.02, 0.0)
        for offset in (4, 5, 6):
            state = observe_pose(
                state,
                Nanoseconds(int(POSE_AT) + offset),
                NOW,
                (0.051, 0.05, 0.10),
            )

        # When: readiness is evaluated within the transaction timeout.
        evidence = localization_ready(state, NOW, DEFAULT_READINESS_POLICY)

        # Then: none of the misleading observations can open the barrier.
        self.assertFalse(evidence.ready)
        self.assertFalse(evidence.pose_samples)
        self.assertTrue(evidence.fresh_scan)
        self.assertFalse(evidence.scan_tf)
        self.assertFalse(evidence.stationary_odom)

    def test_replayed_pose_and_expired_evidence_cannot_open_barrier(self) -> None:
        # Given: one fresh pose replayed three times with otherwise valid evidence.
        state = arm_localization(POSE_AT)
        stamp = Nanoseconds(int(POSE_AT) + 10)
        state = observe_scan(state, stamp, NOW, True)
        state = observe_odometry(state, stamp, NOW, 0.0, 0.0)
        for _ in range(3):
            state = observe_pose(state, stamp, NOW, GOOD_COVARIANCE)

        # When: readiness is checked after all evidence has expired.
        expired_now = Nanoseconds(int(NOW) + 600_000_000)
        evidence = localization_ready(state, expired_now, DEFAULT_READINESS_POLICY)

        # Then: replay protection and freshness both keep localization closed.
        self.assertFalse(evidence.pose_samples)
        self.assertFalse(evidence.fresh_pose)
        self.assertFalse(evidence.ready)

    def test_stale_scan_and_odometry_are_independently_rejected(self) -> None:
        # Given: valid poses but scan and stationary odometry older than the freshness limit.
        evaluated_at = Nanoseconds(int(POSE_AT) + 700_000_000)
        stale_stamp = Nanoseconds(int(POSE_AT) + 100_000_000)
        state = arm_localization(POSE_AT)
        state = observe_scan(state, stale_stamp, evaluated_at, True)
        state = observe_odometry(state, stale_stamp, evaluated_at, 0.0, 0.0)
        for offset in (400_000_000, 500_000_000, 600_000_000):
            stamp = Nanoseconds(int(POSE_AT) + offset)
            state = observe_pose(state, stamp, evaluated_at, GOOD_COVARIANCE)

        # When: the complete conjunction is evaluated.
        evidence = localization_ready(state, evaluated_at, DEFAULT_READINESS_POLICY)

        # Then: stale scan and odometry remain false even with valid AMCL covariance.
        self.assertTrue(evidence.pose_samples)
        self.assertFalse(evidence.fresh_scan)
        self.assertFalse(evidence.fresh_odom)
        self.assertFalse(evidence.ready)


if __name__ == "__main__":
    unittest.main()
