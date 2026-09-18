"""Render the stair evidence pipeline as an operator-facing ROS image."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Final, Tuple

import cv2
import numpy as np

from .stair_evidence import Phase


Color = Tuple[int, int, int]
CANVAS: Final[Color] = (24, 21, 18)
PANEL: Final[Color] = (38, 34, 30)
BORDER: Final[Color] = (80, 75, 68)
TEXT_PRIMARY: Final[Color] = (238, 238, 232)
TEXT_SECONDARY: Final[Color] = (165, 166, 160)
ACTIVE: Final[Color] = (255, 196, 72)
COMPLETE: Final[Color] = (104, 210, 124)
FAULT: Final[Color] = (82, 82, 235)
PENDING: Final[Color] = (118, 112, 104)
PHASES: Final[Tuple[Phase, ...]] = tuple(Phase)


class ReplayOutcome(str, Enum):
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAULTED = "FAULTED"
    INCOMPLETE = "INCOMPLETE"


class PhaseVisualState(str, Enum):
    PENDING = "WAIT"
    ACTIVE = "ACTIVE"
    COMPLETE = "DONE"
    FAULT = "FAULT"


@dataclass(frozen=True)  # noqa: SLOTS_OK - Python 3.8 runtime; explicit slots below.
class StateMachineSnapshot:
    __slots__ = (
        "supervisor_state", "active_phase", "completed_phases", "outcome",
        "detail", "bag_name", "profile_id", "elapsed_sec", "sample_count",
        "progress", "threshold",
    )
    supervisor_state: str
    active_phase: Phase
    completed_phases: Tuple[Phase, ...]
    outcome: ReplayOutcome
    detail: str
    bag_name: str
    profile_id: str
    elapsed_sec: float
    sample_count: int
    progress: float
    threshold: float


def phase_visual_state(
    snapshot: StateMachineSnapshot,
    phase: Phase,
) -> PhaseVisualState:
    """Classify one phase without relying on color-only presentation."""
    if phase in snapshot.completed_phases:
        return PhaseVisualState.COMPLETE
    if phase is snapshot.active_phase:
        if snapshot.outcome is ReplayOutcome.FAULTED:
            return PhaseVisualState.FAULT
        return PhaseVisualState.ACTIVE
    return PhaseVisualState.PENDING


def _color(state: PhaseVisualState) -> Color:
    return {
        PhaseVisualState.PENDING: PENDING,
        PhaseVisualState.ACTIVE: ACTIVE,
        PhaseVisualState.COMPLETE: COMPLETE,
        PhaseVisualState.FAULT: FAULT,
    }[state]


def outcome_color(outcome: ReplayOutcome) -> Color:
    """Return the documented semantic color for one replay outcome."""
    return {
        ReplayOutcome.RUNNING: ACTIVE,
        ReplayOutcome.SUCCEEDED: COMPLETE,
        ReplayOutcome.FAULTED: FAULT,
        ReplayOutcome.INCOMPLETE: PENDING,
    }[outcome]


def _phase_lines(phase: Phase) -> Tuple[str, ...]:
    words = phase.value.split("_")
    if len(words) <= 2:
        return (phase.value,)
    midpoint = (len(words) + 1) // 2
    return ("_".join(words[:midpoint]), "_".join(words[midpoint:]))


def _draw_phase(
    image: np.ndarray,
    snapshot: StateMachineSnapshot,
    phase: Phase,
    index: int,
    x: int,
    y: int,
) -> None:
    state = phase_visual_state(snapshot, phase)
    color = _color(state)
    cv2.rectangle(image, (x, y), (x + 270, y + 100), PANEL, -1)
    cv2.rectangle(image, (x, y), (x + 270, y + 100), color, 3)
    cv2.putText(
        image, f"{index + 1:02d}  {state.value}", (x + 14, y + 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62,
        TEXT_SECONDARY if state is PhaseVisualState.PENDING else color,
        1, cv2.LINE_AA,
    )
    for line_index, line in enumerate(_phase_lines(phase)):
        cv2.putText(
            image, line, (x + 14, y + 57 + line_index * 22),
            cv2.FONT_HERSHEY_DUPLEX, 0.62, TEXT_PRIMARY, 1, cv2.LINE_AA,
        )


def _draw_progress(image: np.ndarray, snapshot: StateMachineSnapshot) -> None:
    x, y, width = 48, 565, 1184
    cv2.rectangle(image, (x, y), (x + width, y + 18), BORDER, -1)
    center = x + width // 2
    cv2.line(image, (center, y - 4), (center, y + 22), TEXT_PRIMARY, 1, cv2.LINE_AA)
    if snapshot.threshold > 0.0:
        ratio = min(1.0, abs(snapshot.progress) / snapshot.threshold)
        travel = int((width // 2) * ratio)
        start, end, color = (
            (center - travel, center, FAULT)
            if snapshot.progress < 0.0
            else (
                center,
                center + travel,
                _color(phase_visual_state(snapshot, snapshot.active_phase)),
            )
        )
        cv2.rectangle(image, (start, y), (end, y + 18), color, -1)
    cv2.putText(
        image, "REVERSE", (x, y + 42),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62, TEXT_SECONDARY, 1, cv2.LINE_AA,
    )
    cv2.putText(
        image, "0", (center - 4, y + 42),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62, TEXT_SECONDARY, 1, cv2.LINE_AA,
    )
    cv2.putText(
        image, "TARGET", (x + width - 60, y + 42),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62, TEXT_SECONDARY, 1, cv2.LINE_AA,
    )


def render_state_machine(snapshot: StateMachineSnapshot) -> np.ndarray:
    """Render a fixed 16:9 pipeline panel for RViz or rqt_image_view."""
    image = np.full((720, 1280, 3), CANVAS, dtype=np.uint8)
    cv2.putText(
        image, "TRON1  STAIR EVIDENCE FSM REPLAY", (42, 48),
        cv2.FONT_HERSHEY_DUPLEX, 1.0, TEXT_PRIMARY, 2, cv2.LINE_AA,
    )
    state_color = outcome_color(snapshot.outcome)
    cv2.putText(
        image,
        f"{snapshot.supervisor_state} / {snapshot.outcome.value}",
        (850, 48), cv2.FONT_HERSHEY_DUPLEX, 0.8, state_color, 2, cv2.LINE_AA,
    )
    cv2.putText(
        image,
        f"BAG  {snapshot.bag_name}",
        (44, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.62, TEXT_SECONDARY, 1, cv2.LINE_AA,
    )
    cv2.putText(
        image, f"PROFILE  {snapshot.profile_id}",
        (44, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.62, TEXT_SECONDARY, 1, cv2.LINE_AA,
    )
    positions = (
        (42, 135), (352, 135), (662, 135), (972, 135),
        (197, 285), (507, 285), (817, 285),
    )
    for index, (phase, position) in enumerate(zip(PHASES, positions)):
        _draw_phase(image, snapshot, phase, index, position[0], position[1])
    cv2.rectangle(image, (42, 420), (1238, 680), PANEL, -1)
    cv2.rectangle(image, (42, 420), (1238, 680), BORDER, 1)
    cv2.putText(
        image,
        f"PHASE  {snapshot.active_phase.value}",
        (62, 462), cv2.FONT_HERSHEY_DUPLEX, 0.72, TEXT_PRIMARY, 1, cv2.LINE_AA,
    )
    cv2.putText(
        image,
        f"elapsed={snapshot.elapsed_sec:7.2f}s   samples={snapshot.sample_count:7d}   "
        f"progress={snapshot.progress:.3f} / {snapshot.threshold:.3f}",
        (62, 505), cv2.FONT_HERSHEY_SIMPLEX, 0.64, TEXT_SECONDARY, 1, cv2.LINE_AA,
    )
    cv2.putText(
        image, snapshot.detail[:115], (62, 545),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62, TEXT_PRIMARY, 1, cv2.LINE_AA,
    )
    _draw_progress(image, snapshot)
    cv2.putText(
        image,
        "Recorded odometry values; replay timestamps are refreshed for live safety checks.",
        (62, 650), cv2.FONT_HERSHEY_SIMPLEX, 0.62, TEXT_SECONDARY, 1, cv2.LINE_AA,
    )
    return image
