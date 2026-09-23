"""Typed contracts shared by the stair command supervisor."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Protocol

from .configuration import Direction, StairProfile
from .stair_evidence import EvidenceReport, Phase


class SupervisorState(IntEnum):
    DISARMED = 0
    NAV = 1
    STAIR = 2
    FAULT = 3


class ResultCode(IntEnum):
    OK = 0
    BUSY = 1
    INVALID_GOAL = 2
    CAPABILITY_DISABLED = 3
    ENTRY_REJECTED = 4
    STAIR_FAILED = 5
    COMMUNICATION_LOST = 8


@dataclass(frozen=True)
class StairGoal:
    __slots__ = ("stair_id", "direction", "admission_token")
    stair_id: str
    direction: Direction
    admission_token: str


@dataclass(frozen=True)
class AdmissionDecision:
    __slots__ = ("accepted", "communication_error", "reason")
    accepted: bool
    communication_error: bool
    reason: str


class AdmissionValidator(Protocol):
    def validate(self, goal: StairGoal, ownership_epoch: int) -> AdmissionDecision: ...


@dataclass(frozen=True)
class TraversalResult:
    code: ResultCode
    reason: str
    cancelled: bool = False


class SupervisorClock(Protocol):
    def monotonic(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class CommandTransport(Protocol):
    def request_stair_mode_with_feedback(self, enabled: bool, progress) -> None: ...

    @property
    def stream_period_sec(self) -> float: ...

    def start(self) -> None: ...

    def update_twist(self, linear_mps: float, angular_radps: float) -> None: ...

    def send_current(self) -> None: ...

    def request_stair_mode(self, enabled: bool) -> None: ...

    def close(self) -> None: ...


class PhaseEvidence(Protocol):
    def arm(self, profile: StairProfile, started_at: float) -> None: ...

    def begin_phase(self, phase: Phase, now: float) -> None: ...

    def evaluate(self, phase: Phase, now: float) -> EvidenceReport: ...
