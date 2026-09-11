"""Route-wide recording orchestration without ROS-specific process ownership."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from mission_manager.fsm import MissionEvent, MissionFSM
from mission_manager.mission_types import (
    MissionProgress,
    MissionRunRequest,
    MissionRunResult,
    RouteRecordingRequest,
    SegmentContext,
    SegmentExecutionStatus,
    SegmentExecutor,
)
from mission_manager.recording_session import (
    RecordingDisposition,
    RecordingSession,
    ScanRecorderError,
)
from mission_manager.route_planner import BuildingPlanner, LogicalAnchor, RoutePlanningError


@dataclass(frozen=True)
class RecordRouteRuntime:
    __slots__ = ("planner", "executor", "profile_id")
    planner: BuildingPlanner
    executor: SegmentExecutor
    profile_id: str


@dataclass(frozen=True)
class RecordRouteInvocation:
    __slots__ = ("request", "feedback", "cancellation_requested")
    request: MissionRunRequest
    feedback: Callable[[MissionProgress], None]
    cancellation_requested: Callable[[], bool]


@dataclass(frozen=True)
class RecordRouteOutcome:
    __slots__ = ("result", "anchor")
    result: MissionRunResult
    anchor: LogicalAnchor


class RecordRouteRunner:
    """Bracket a directed route and optional origin return with one recording."""

    def __init__(self, runtime: RecordRouteRuntime) -> None:
        self._runtime = runtime

    def run(
        self,
        origin: LogicalAnchor,
        invocation: RecordRouteInvocation,
    ) -> RecordRouteOutcome:
        request = invocation.request
        fsm = MissionFSM().dispatch(MissionEvent.GOAL_RECEIVED)
        try:
            outbound = self._runtime.planner.plan(origin.location_id, request.destination_id)
            closing = (
                self._runtime.planner.plan(request.destination_id, origin.location_id)
                if request.return_after_task
                else None
            )
        except RoutePlanningError as error:
            fsm.dispatch(MissionEvent.PLAN_FAILED)
            return RecordRouteOutcome(
                MissionRunResult(
                    SegmentExecutionStatus.FAILED,
                    2,
                    str(error),
                    request.mission_id,
                    "",
                ),
                origin,
            )
        pending = list(outbound.segments)
        if closing is not None:
            pending.extend(closing.segments)
        if not pending:
            return RecordRouteOutcome(
                MissionRunResult(SegmentExecutionStatus.SUCCESS, 0, "", request.mission_id, ""),
                origin,
            )
        if invocation.cancellation_requested():
            fsm.dispatch(MissionEvent.CANCEL)
            return RecordRouteOutcome(
                MissionRunResult(
                    SegmentExecutionStatus.CANCELLED,
                    9,
                    "mission cancelled",
                    request.mission_id,
                    "",
                ),
                origin,
            )
        fsm = fsm.dispatch(MissionEvent.PLAN_SUCCEEDED)
        try:
            recording: RecordingSession | None = self._runtime.executor.start_recording(
                RouteRecordingRequest(request.mission_id, self._runtime.profile_id, request.destination_id)
            )
        except ScanRecorderError as error:
            fsm.dispatch(MissionEvent.SEGMENT_FAILED)
            return RecordRouteOutcome(
                MissionRunResult(
                    SegmentExecutionStatus.FAILED,
                    7,
                    str(error),
                    request.mission_id,
                    "",
                ),
                origin,
            )
        anchor = origin
        context = SegmentContext(request.mission_id, invocation.cancellation_requested)
        completed_count = 0
        total_count = len(pending)
        try:
            while pending:
                segment = pending.pop(0)
                if invocation.cancellation_requested():
                    fsm.dispatch(MissionEvent.CANCEL)
                    try:
                        result = recording.finish(RecordingDisposition.PREEMPTED)
                        recording = None
                        artifact_path = str(result.artifact_path)
                    except ScanRecorderError as error:
                        return RecordRouteOutcome(
                            MissionRunResult(
                                SegmentExecutionStatus.CANCELLED,
                                9,
                                f"mission cancelled; {error}",
                                request.mission_id,
                                "",
                            ),
                            anchor,
                        )
                    return RecordRouteOutcome(
                        MissionRunResult(
                            SegmentExecutionStatus.CANCELLED,
                            9,
                            "mission cancelled",
                            request.mission_id,
                            artifact_path,
                        ),
                        anchor,
                    )
                invocation.feedback(
                    MissionProgress(
                        request.mission_id,
                        fsm.state.value,
                        segment.source_floor,
                        segment.type.value,
                        completed_count,
                        total_count,
                        segment.target_id,
                        "segment dispatch",
                    )
                )
                execution = self._runtime.executor.execute(segment, context)
                if execution.status is SegmentExecutionStatus.RETRY:
                    fsm = fsm.dispatch(MissionEvent.SEGMENT_RETRY)
                    pending.insert(0, segment)
                    continue
                if execution.status is SegmentExecutionStatus.SUCCESS:
                    anchor = anchor.after_segment(segment, succeeded=True)
                    completed_count += 1
                    fsm = fsm.dispatch(MissionEvent.SEGMENT_SUCCEEDED)
                    try:
                        recording.assert_active()
                    except ScanRecorderError as error:
                        fsm.dispatch(MissionEvent.SEGMENT_FAILED)
                        return RecordRouteOutcome(
                            MissionRunResult(
                                SegmentExecutionStatus.FAILED,
                                7,
                                str(error),
                                request.mission_id,
                                "",
                            ),
                            anchor,
                        )
                    event = MissionEvent.MORE_SEGMENTS if pending else MissionEvent.ALL_SEGMENTS_COMPLETE
                    fsm = fsm.dispatch(event)
                    continue
                event = (
                    MissionEvent.CANCEL
                    if execution.status is SegmentExecutionStatus.CANCELLED
                    else MissionEvent.SEGMENT_FAILED
                )
                fsm.dispatch(event)
                try:
                    disposition = (
                        RecordingDisposition.PREEMPTED
                        if execution.status is SegmentExecutionStatus.CANCELLED
                        else RecordingDisposition.INTERRUPTED
                    )
                    result = recording.finish(disposition)
                    recording = None
                    artifact_path = str(result.artifact_path)
                except ScanRecorderError:
                    artifact_path = ""
                return RecordRouteOutcome(
                    MissionRunResult(
                        execution.status,
                        execution.result_code,
                        execution.reason,
                        request.mission_id,
                        artifact_path,
                    ),
                    anchor,
                )
            try:
                result = recording.finish(RecordingDisposition.COMPLETED)
                recording = None
            except ScanRecorderError as error:
                return RecordRouteOutcome(
                    MissionRunResult(
                        SegmentExecutionStatus.FAILED,
                        7,
                        str(error),
                        request.mission_id,
                        "",
                    ),
                    anchor,
                )
            return RecordRouteOutcome(
                MissionRunResult(
                    SegmentExecutionStatus.SUCCESS,
                    0,
                    "",
                    request.mission_id,
                    str(result.artifact_path),
                ),
                anchor,
            )
        finally:
            if recording is not None:
                recording.discard()
