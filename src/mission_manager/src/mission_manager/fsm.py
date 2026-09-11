"""Deterministic, ROS-independent mission state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, unique
from typing import Dict, Tuple, Union

from mission_manager.segment_types import SegmentType


__all__ = (
    "IllegalTransitionError",
    "InvalidMissionEventError",
    "MissionEvent",
    "MissionFSM",
    "MissionState",
    "SegmentType",
)


@unique
class MissionState(str, Enum):
    WAIT_GOAL = "WAIT_GOAL"
    PLAN_MISSION = "PLAN_MISSION"
    EXECUTE_SEGMENT = "EXECUTE_SEGMENT"
    NEXT_SEGMENT = "NEXT_SEGMENT"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MISSION_ABORT = "MISSION_ABORT"


@unique
class MissionEvent(str, Enum):
    GOAL_RECEIVED = "GOAL_RECEIVED"
    PLAN_SUCCEEDED = "PLAN_SUCCEEDED"
    PLAN_FAILED = "PLAN_FAILED"
    SEGMENT_SUCCEEDED = "SEGMENT_SUCCEEDED"
    SEGMENT_RETRY = "SEGMENT_RETRY"
    SEGMENT_FAILED = "SEGMENT_FAILED"
    MORE_SEGMENTS = "MORE_SEGMENTS"
    ALL_SEGMENTS_COMPLETE = "ALL_SEGMENTS_COMPLETE"
    CANCEL = "CANCEL"
    TIMEOUT = "TIMEOUT"
    RESET = "RESET"


@dataclass(frozen=True)
class IllegalTransitionError(Exception):
    state: MissionState
    event: MissionEvent

    def __str__(self) -> str:
        return "event {} is illegal while mission is in {}".format(self.event.value, self.state.value)


@dataclass(frozen=True)
class InvalidMissionEventError(Exception):
    state: MissionState
    event_value: str

    def __str__(self) -> str:
        return "event {!r} is invalid while mission is in {}".format(self.event_value, self.state.value)


_TRANSITIONS: Dict[Tuple[MissionState, MissionEvent], MissionState] = {
    (MissionState.WAIT_GOAL, MissionEvent.GOAL_RECEIVED): MissionState.PLAN_MISSION,
    (MissionState.PLAN_MISSION, MissionEvent.PLAN_SUCCEEDED): MissionState.EXECUTE_SEGMENT,
    (MissionState.PLAN_MISSION, MissionEvent.PLAN_FAILED): MissionState.MISSION_ABORT,
    (MissionState.EXECUTE_SEGMENT, MissionEvent.SEGMENT_SUCCEEDED): MissionState.NEXT_SEGMENT,
    (MissionState.EXECUTE_SEGMENT, MissionEvent.SEGMENT_RETRY): MissionState.EXECUTE_SEGMENT,
    (MissionState.EXECUTE_SEGMENT, MissionEvent.SEGMENT_FAILED): MissionState.MISSION_ABORT,
    (MissionState.NEXT_SEGMENT, MissionEvent.MORE_SEGMENTS): MissionState.EXECUTE_SEGMENT,
    (MissionState.NEXT_SEGMENT, MissionEvent.ALL_SEGMENTS_COMPLETE): MissionState.MISSION_COMPLETE,
    (MissionState.MISSION_COMPLETE, MissionEvent.RESET): MissionState.WAIT_GOAL,
    (MissionState.MISSION_ABORT, MissionEvent.RESET): MissionState.WAIT_GOAL,
}

for _active_state in (MissionState.PLAN_MISSION, MissionState.EXECUTE_SEGMENT, MissionState.NEXT_SEGMENT):
    _TRANSITIONS[(_active_state, MissionEvent.CANCEL)] = MissionState.MISSION_ABORT
    _TRANSITIONS[(_active_state, MissionEvent.TIMEOUT)] = MissionState.MISSION_ABORT


@dataclass(frozen=True)
class MissionFSM:
    state: MissionState = MissionState.WAIT_GOAL

    def dispatch(self, event: Union[MissionEvent, str]) -> "MissionFSM":
        """Return the next immutable FSM value or reject an undeclared pair."""
        try:
            parsed_event = MissionEvent(event)
        except ValueError:
            raise InvalidMissionEventError(state=self.state, event_value=str(event)) from None
        try:
            next_state = _TRANSITIONS[(self.state, parsed_event)]
        except KeyError:
            raise IllegalTransitionError(state=self.state, event=parsed_event) from None
        return MissionFSM(state=next_state)
