from __future__ import annotations

import math
import json
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stair_supervisor.configuration import Direction, StairProfile  # noqa: E402
from stair_supervisor.stair_evidence import (  # noqa: E402
    OdometrySample,
    Phase,
    StairEvidenceTracker,
)


def profile(direction: Direction = Direction.UP) -> StairProfile:
    sign = 1.0 if direction is Direction.UP else -1.0
    return StairProfile(
        id="synthetic_" + direction.value.lower(),
        direction=direction,
        enabled=True,
        linear_speed=0.12,
        angular_speed=0.20,
        alignment_yaw_rad=0.0,
        flight_1_distance_m=sign * 1.00,
        landing_dwell_sec=0.20,
        landing_turn_yaw_rad=sign * 0.50,
        flight_2_distance_m=sign * 0.80,
        exit_dwell_sec=0.20,
        distance_tolerance_m=0.02,
        yaw_tolerance_rad=0.02,
        sensor_freshness_sec=0.20,
        max_sample_gap_sec=0.15,
        max_odom_step_m=0.40,
        max_yaw_step_rad=0.30,
        timeout_sec=5.0,
    )


class SyntheticTraversal:
    def __init__(self, direction: Direction, start_yaw: float = 0.0) -> None:
        self.tracker = StairEvidenceTracker()
        self.profile = profile(direction)
        self.time = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = start_yaw
        self.tracker.update_odometry(OdometrySample(self.time, self.x, self.y, self.yaw))
        self.tracker.arm(self.profile, self.time)

    def sample(self, *, dx: float = 0.0, dyaw: float = 0.0, dt: float = 0.05) -> None:
        self.time += dt
        self.x += dx * math.cos(self.yaw)
        self.y += dx * math.sin(self.yaw)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        self.tracker.update_odometry(OdometrySample(self.time, self.x, self.y, self.yaw))

    def begin(self, phase: Phase) -> None:
        self.tracker.begin_phase(phase, self.time)

    def report(self, phase: Phase):
        return self.tracker.evaluate(phase, self.time)


class StairEvidenceHappyPathTest(unittest.TestCase):
    def test_all_seven_phases_complete_for_up_and_down_profiles(self) -> None:
        for direction in Direction:
            with self.subTest(direction=direction):
                # Given: a direction-specific profile and fresh stationary entry samples.
                run = SyntheticTraversal(direction)
                sign = 1.0 if direction is Direction.UP else -1.0
                run.sample()

                # When/Then: each phase receives its independent synthetic physical evidence.
                self.assertTrue(run.report(Phase.VERIFY_ENTRY).complete)
                run.begin(Phase.ALIGN)
                run.sample()
                self.assertTrue(run.report(Phase.ALIGN).complete)
                run.begin(Phase.FORWARD_SEGMENT_1)
                run.sample(dx=sign * 0.35)
                run.sample(dx=sign * 0.35)
                run.sample(dx=sign * 0.30)
                self.assertTrue(run.report(Phase.FORWARD_SEGMENT_1).complete)
                run.begin(Phase.LANDING)
                for _ in range(6):
                    run.sample()
                    landing = run.report(Phase.LANDING)
                self.assertTrue(landing.complete)
                run.begin(Phase.TURN_TO_NEXT_FLIGHT)
                run.sample(dyaw=sign * 0.25)
                run.sample(dyaw=sign * 0.25)
                self.assertTrue(run.report(Phase.TURN_TO_NEXT_FLIGHT).complete)
                run.begin(Phase.FORWARD_SEGMENT_2)
                run.sample(dx=sign * 0.30)
                run.sample(dx=sign * 0.30)
                run.sample(dx=sign * 0.20)
                self.assertTrue(run.report(Phase.FORWARD_SEGMENT_2).complete)
                run.begin(Phase.EXIT_CONFIRM)
                for _ in range(6):
                    run.sample()
                    exit_report = run.report(Phase.EXIT_CONFIRM)
                self.assertTrue(exit_report.complete)

    def test_align_completes_without_yaw_progress(self) -> None:
        # Given: ALIGN begins just below positive pi.
        run = SyntheticTraversal(Direction.UP, math.pi - 0.10)
        run.begin(Phase.ALIGN)

        # When: one fresh odometry sample arrives without rotation.
        run.sample()

        # Then: canonical map-pose admission makes relative yaw progress unnecessary.
        report = run.report(Phase.ALIGN)
        self.assertTrue(report.complete)
        self.assertEqual((report.progress, report.threshold), (0.0, 0.0))

    def test_flight_distance_uses_odometry_only(self) -> None:
        # Given: a first flight baseline.
        run = SyntheticTraversal(Direction.UP)
        run.begin(Phase.FORWARD_SEGMENT_1)

        # When: distance reaches threshold using odometry alone.
        run.sample(dx=0.35)
        run.sample(dx=0.35)
        run.sample(dx=0.30)

        # Then: completion depends only on bounded odometry progress.
        self.assertTrue(run.report(Phase.FORWARD_SEGMENT_1).complete)

    def test_dwell_resets_after_motion(self) -> None:
        # Given: level stationary landing evidence has begun accumulating.
        run = SyntheticTraversal(Direction.UP)
        run.begin(Phase.LANDING)
        run.sample()
        run.report(Phase.LANDING)
        run.sample()
        run.report(Phase.LANDING)

        # When: motion interrupts the dwell and the robot becomes stationary again.
        run.sample(dx=0.05)
        run.sample()

        # Then: even an interruption between evaluations resets the old dwell.
        report = run.report(Phase.LANDING)
        self.assertFalse(report.complete)
        self.assertEqual(report.progress, 0.0)


class StairEvidenceFaultTest(unittest.TestCase):
    def test_invalid_observations_latch_fault(self) -> None:
        cases = (
            ("nonfinite", lambda run: run.tracker.update_odometry(OdometrySample(0.05, math.nan, 0.0, 0.0))),
            ("stale", lambda run: setattr(run, "time", 0.25)),
            ("future", lambda run: run.tracker.update_odometry(OdometrySample(0.10, 0.0, 0.0, 0.0))),
            ("gap", lambda run: run.sample(dt=0.16)),
            ("odom_jump", lambda run: run.sample(dx=0.41)),
            ("yaw_jump", lambda run: run.sample(dyaw=0.31)),
            ("timeout", lambda run: setattr(run, "time", 5.01)),
        )
        for name, inject in cases:
            with self.subTest(name=name):
                # Given: an armed tracker with a current phase.
                run = SyntheticTraversal(Direction.UP)

                # When: one unsafe observation class is injected.
                inject(run)

                # Then: evidence faults closed and remains incomplete.
                report = run.report(Phase.VERIFY_ENTRY)
                self.assertTrue(report.faulted)
                self.assertFalse(report.complete)

    def test_reverse_distance_and_yaw_fault_in_both_directions(self) -> None:
        for direction in Direction:
            sign = 1.0 if direction is Direction.UP else -1.0
            for phase, movement in (
                (Phase.TURN_TO_NEXT_FLIGHT, {"dyaw": -sign * 0.03}),
                (Phase.FORWARD_SEGMENT_1, {"dx": -sign * 0.03}),
            ):
                with self.subTest(direction=direction, phase=phase):
                    # Given: a direction-specific phase baseline.
                    run = SyntheticTraversal(direction)
                    run.begin(phase)

                    # When: progress exceeds tolerance opposite the surveyed sign.
                    run.sample(**movement)

                    # Then: the tracker latches a reverse-progress fault.
                    self.assertTrue(run.report(phase).faulted)

    def test_recorded_gap_then_jump_cannot_become_later_progress(self) -> None:
        # Given: first-flight evidence modeled around the recorded 225-239 s gap.
        fixture_path = Path(__file__).resolve().parents[3] / "test" / "fixtures" / "stair_evidence" / "recorded_discontinuity.json"
        recorded = json.loads(fixture_path.read_text(encoding="utf-8"))
        run = SyntheticTraversal(Direction.UP)
        run.begin(Phase.FORWARD_SEGMENT_1)
        run.sample(dx=0.10)

        # When: the next sample carries Task 2's measured IMU gap and wheel jump.
        run.sample(dx=recorded["odom_step_m"], dt=recorded["gap_sec"])
        first = run.report(Phase.FORWARD_SEGMENT_1)
        run.sample(dx=0.10)
        later = run.report(Phase.FORWARD_SEGMENT_1)

        # Then: the first rejection remains latched and later samples cannot progress.
        self.assertTrue(first.faulted)
        self.assertTrue(later.faulted)
        self.assertFalse(later.complete)


if __name__ == "__main__":
    unittest.main()
