"""Tests for the operator-facing stair state-machine panel."""

from __future__ import annotations

import numpy as np

from stair_supervisor.state_machine_dashboard import (
    ACTIVE,
    COMPLETE,
    FAULT,
    PENDING,
    PhaseVisualState,
    ReplayOutcome,
    StateMachineSnapshot,
    outcome_color,
    phase_visual_state,
    render_state_machine,
)
from stair_supervisor.stair_evidence import Phase


def snapshot(outcome: ReplayOutcome = ReplayOutcome.RUNNING) -> StateMachineSnapshot:
    return StateMachineSnapshot(
        supervisor_state="STAIR",
        active_phase=Phase.FORWARD_SEGMENT_1,
        completed_phases=(Phase.VERIFY_ENTRY, Phase.ALIGN),
        outcome=outcome,
        detail="progress=1.250000 threshold=4.771000 reason=distance progress",
        bag_name="stair-up.bag",
        profile_id="stair_3f_4f_up",
        elapsed_sec=12.5,
        sample_count=1250,
        progress=1.25,
        threshold=4.771,
    )


def test_phase_visual_state_distinguishes_completed_active_and_pending() -> None:
    # Given/When: one running replay has crossed two phase gates.
    state = snapshot()

    # Then: every pipeline role has a distinct machine-readable state.
    assert phase_visual_state(state, Phase.VERIFY_ENTRY) is PhaseVisualState.COMPLETE
    assert phase_visual_state(state, Phase.FORWARD_SEGMENT_1) is PhaseVisualState.ACTIVE
    assert phase_visual_state(state, Phase.LANDING) is PhaseVisualState.PENDING


def test_active_phase_becomes_fault_node_on_terminal_rejection() -> None:
    # Given/When: the replay faults while evaluating its active phase.
    state = snapshot(ReplayOutcome.FAULTED)

    # Then: the active node is explicitly classified as faulted.
    assert phase_visual_state(state, Phase.FORWARD_SEGMENT_1) is PhaseVisualState.FAULT


def test_dashboard_renders_fixed_readable_canvas_with_status_variation() -> None:
    # Given: running and faulted snapshots of the same evidence.
    running = render_state_machine(snapshot())
    faulted = render_state_machine(snapshot(ReplayOutcome.FAULTED))

    # When/Then: both are full-color 16:9 panels and terminal state changes pixels.
    assert running.shape == (720, 1280, 3)
    assert running.dtype == np.uint8
    assert np.count_nonzero(running != faulted) > 100


def test_each_terminal_outcome_has_its_own_semantic_color() -> None:
    # Given/When/Then: outcome colors preserve running, success, fault, and incomplete meaning.
    assert outcome_color(ReplayOutcome.RUNNING) == ACTIVE
    assert outcome_color(ReplayOutcome.SUCCEEDED) == COMPLETE
    assert outcome_color(ReplayOutcome.FAULTED) == FAULT
    assert outcome_color(ReplayOutcome.INCOMPLETE) == PENDING
