"""Typed mission orchestration contracts shared by pure and ROS adapters."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum, unique
from typing import Protocol

from mission_manager.recording_session import RecordingSession
from mission_manager.route_planner import RouteSegment


@unique
class MissionType(str, Enum):
    NAVIGATE = "navigate"
    INSPECT = "inspect"
    PHOTO_TOUR = "photo_tour"
    RETURN_TO_START = "return_to_start"
    RECORD_ROUTE = "record_route"


@unique
class SegmentExecutionStatus(str, Enum):
    SUCCESS = "SUCCESS"
    RETRY = "RETRY"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    ABORT = "ABORT"


@dataclass(frozen=True)
class SegmentExecution:
    __slots__ = ("status", "result_code", "reason", "artifact_path", "evidence")
    status: SegmentExecutionStatus
    result_code: int
    reason: str
    artifact_path: str
    evidence: str

    @classmethod
    def success(cls, artifact_path: str = "", evidence: str = "") -> "SegmentExecution":
        return cls(SegmentExecutionStatus.SUCCESS, 0, "", artifact_path, evidence)

    @classmethod
    def retry(cls, reason: str, evidence: str = "") -> "SegmentExecution":
        return cls(SegmentExecutionStatus.RETRY, 0, reason, "", evidence)

    @classmethod
    def failed(cls, result_code: int, reason: str, evidence: str = "") -> "SegmentExecution":
        return cls(SegmentExecutionStatus.FAILED, result_code, reason, "", evidence)

    @classmethod
    def cancelled(cls, reason: str) -> "SegmentExecution":
        return cls(SegmentExecutionStatus.CANCELLED, 9, reason, "", "safe checkpoint")

    @classmethod
    def abort(cls, reason: str, evidence: str = "") -> "SegmentExecution":
        return cls(SegmentExecutionStatus.ABORT, 9, reason, "", evidence)


@dataclass(frozen=True)
class SegmentContext:
    __slots__ = ("mission_id", "cancellation_requested")
    mission_id: str
    cancellation_requested: Callable[[], bool]


@dataclass(frozen=True)
class RouteRecordingRequest:
    __slots__ = ("mission_id", "profile_id", "location_id")
    mission_id: str
    profile_id: str
    location_id: str


class SegmentExecutor(Protocol):
    def execute(self, segment: RouteSegment, context: SegmentContext) -> SegmentExecution:
        ...

    def start_recording(self, request: RouteRecordingRequest) -> RecordingSession:
        ...

    def cancel_active(self) -> None:
        ...


@dataclass(frozen=True)
class MissionRunRequest:
    __slots__ = ("mission_id", "destination_id", "mission_type", "return_after_task")
    mission_id: str
    destination_id: str
    mission_type: MissionType
    return_after_task: bool


@dataclass(frozen=True)
class MissionProgress:
    __slots__ = (
        "mission_id", "state", "current_floor", "segment_type", "segment_index",
        "segment_count", "target_id", "evidence",
    )
    mission_id: str
    state: str
    current_floor: str
    segment_type: str
    segment_index: int
    segment_count: int
    target_id: str
    evidence: str


@dataclass(frozen=True)
class MissionRunResult:
    __slots__ = ("status", "result_code", "reason", "mission_id", "artifact_path")
    status: SegmentExecutionStatus
    result_code: int
    reason: str
    mission_id: str
    artifact_path: str


@dataclass(frozen=True)
class MissionOrchestratorSettings:
    __slots__ = ("initial_location_id", "inspect_profile_id")
    initial_location_id: str
    inspect_profile_id: str
