"""Regression tests for terminal ROS bag replay snapshots."""

from __future__ import annotations

from pathlib import Path
import threading

import pytest

from stair_supervisor.configuration import Direction, StairProfile
from stair_supervisor.msg import StairTraversalFeedback
from stair_supervisor.ros_bag_replay import ReplaySession
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.state_machine_dashboard import ReplayOutcome


def profile() -> StairProfile:
    return StairProfile(
        id="synthetic_up",
        direction=Direction.UP,
        enabled=True,
        linear_speed=0.12,
        angular_speed=0.20,
        alignment_yaw_rad=0.40,
        flight_1_distance_m=1.00,
        landing_dwell_sec=0.20,
        landing_turn_yaw_rad=0.50,
        flight_2_distance_m=0.80,
        exit_dwell_sec=0.20,
        distance_tolerance_m=0.02,
        yaw_tolerance_rad=0.02,
        sensor_freshness_sec=0.20,
        max_sample_gap_sec=0.15,
        max_odom_step_m=0.40,
        max_yaw_step_rad=0.30,
        timeout_sec=5.0,
    )


def replay_session(output_dir: Path) -> ReplaySession:
    session = ReplaySession.__new__(ReplaySession)
    session._bag_path = Path("recorded-stair.bag")
    session._profile = profile()
    session._output_dir = output_dir
    session._lock = threading.RLock()
    session._supervisor_state = "STAIR"
    session._active_phase = Phase.FORWARD_SEGMENT_2
    session._completed = [
        Phase.VERIFY_ENTRY,
        Phase.ALIGN,
        Phase.FORWARD_SEGMENT_1,
        Phase.LANDING,
        Phase.TURN_TO_NEXT_FLIGHT,
    ]
    session._detail = "waiting for replay feedback"
    session._progress_value = 0.0
    session._progress_threshold = 0.0
    session._outcome = ReplayOutcome.RUNNING
    session._elapsed_sec = 42.5
    session._sample_count = 4250
    return session


@pytest.mark.parametrize(
    ("outcome", "terminal_detail"),
    (
        (ReplayOutcome.SUCCEEDED, "stair traversal complete"),
        (
            ReplayOutcome.INCOMPLETE,
            "recorded odometry ended before the active phase completed",
        ),
    ),
)
def test_terminal_snapshot_retains_last_progress_when_reason_replaces_feedback(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    outcome: ReplayOutcome,
    terminal_detail: str,
) -> None:
    # Given: the replay has metric-bearing feedback immediately before termination.
    session = replay_session(tmp_path)
    monkeypatch.setattr(ReplaySession, "publish_dashboard", lambda _session: None)
    session._accept_feedback(
        StairTraversalFeedback(
            phase=Phase.FORWARD_SEGMENT_2.value,
            detail="progress=2.100000 threshold=3.619000 reason=distance progress",
        )
    )

    # When: success or bag exhaustion replaces feedback with its terminal reason.
    session._finish(outcome, terminal_detail)
    snapshot = session._snapshot()

    # Then: the reason changes while the last measured evidence remains visible.
    assert snapshot.detail == terminal_detail
    assert snapshot.progress == pytest.approx(2.1)
    assert snapshot.threshold == pytest.approx(3.619)
