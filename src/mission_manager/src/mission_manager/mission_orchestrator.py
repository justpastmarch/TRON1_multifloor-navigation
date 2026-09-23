"""ROS-independent mission segment orchestration and confirmed-anchor ownership."""

from __future__ import annotations

from collections.abc import Callable

from mission_manager.fsm import MissionEvent, MissionFSM
from mission_manager.mission_types import (
    MissionOrchestratorSettings,
    MissionProgress,
    MissionRunRequest,
    MissionRunResult,
    MissionType,
    RouteRecordingRequest as RouteRecordingRequest,
    SegmentContext,
    SegmentExecution as SegmentExecution,
    SegmentExecutionStatus,
    SegmentExecutor,
)
from mission_manager.record_route_runner import (
    RecordRouteInvocation,
    RecordRouteRunner,
    RecordRouteRuntime,
)
from mission_manager.route_planner import (
    BuildingPlanner,
    LogicalAnchor,
    RoutePlanningError,
)


class MissionOrchestrator:
    """Execute generic route segments while owning the confirmed logical anchor."""

    def __init__(
        self,
        planner: BuildingPlanner,
        executor: SegmentExecutor,
        settings: MissionOrchestratorSettings,
        start_from_current_pose: bool = False,
    ) -> None:
        self._planner = planner
        self._executor = executor
        self._anchor = LogicalAnchor(settings.initial_location_id)
        self._inspect_profile_id = settings.inspect_profile_id
        self._start_from_current_pose = start_from_current_pose

    @property
    def confirmed_location_id(self) -> str:
        return self._anchor.location_id

    def cancel_active(self) -> None:
        self._executor.cancel_active()

    def run(
        self,
        request: MissionRunRequest,
        feedback: Callable[[MissionProgress], None],
        cancellation_requested: Callable[[], bool],
    ) -> MissionRunResult:
        """Plan outbound once, execute it, then optionally plan a fresh return."""
        if request.mission_type is MissionType.RECORD_ROUTE:
            outcome = RecordRouteRunner(
                RecordRouteRuntime(self._planner, self._executor, self._inspect_profile_id)
            ).run(
                self._anchor,
                RecordRouteInvocation(request, feedback, cancellation_requested),
            )
            self._anchor = outcome.anchor
            return outcome.result
        fsm = MissionFSM().dispatch(MissionEvent.GOAL_RECEIVED)
        try:
            plan = self._planner.plan_from_current_pose if self._start_from_current_pose else self._planner.plan
            route = plan(
                self._anchor.location_id,
                request.destination_id,
                self._inspect_profile_id if request.mission_type is MissionType.INSPECT else None,
            )
        except RoutePlanningError as error:
            fsm.dispatch(MissionEvent.PLAN_FAILED)
            return MissionRunResult(
                SegmentExecutionStatus.FAILED,
                2,
                str(error),
                request.mission_id,
                "",
            )
        if not route.segments:
            return MissionRunResult(SegmentExecutionStatus.SUCCESS, 0, "", request.mission_id, "")
        fsm = fsm.dispatch(MissionEvent.PLAN_SUCCEEDED)
        artifact_path = ""
        completed_count = 0
        pending = list(route.segments)
        total_count = len(pending)
        return_planned = False
        context = SegmentContext(request.mission_id, cancellation_requested)
        while pending:
            segment = pending.pop(0)
            if cancellation_requested():
                fsm.dispatch(MissionEvent.CANCEL)
                return MissionRunResult(
                    SegmentExecutionStatus.CANCELLED,
                    9,
                    "mission cancelled",
                    request.mission_id,
                    artifact_path,
                )
            feedback(
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
            execution = self._executor.execute(segment, context)
            if execution.status is SegmentExecutionStatus.RETRY:
                fsm = fsm.dispatch(MissionEvent.SEGMENT_RETRY)
                pending.insert(0, segment)
                feedback(
                    MissionProgress(
                        request.mission_id,
                        fsm.state.value,
                        segment.source_floor,
                        segment.type.value,
                        completed_count,
                        total_count,
                        segment.target_id,
                        execution.evidence or execution.reason,
                    )
                )
                continue
            if execution.status is SegmentExecutionStatus.SUCCESS:
                self._anchor = self._anchor.after_segment(segment, succeeded=True)
                artifact_path = execution.artifact_path or artifact_path
                completed_count += 1
                fsm = fsm.dispatch(MissionEvent.SEGMENT_SUCCEEDED)
                if not pending and request.return_after_task and not return_planned:
                    return_route = self._planner.plan_return(self._anchor)
                    pending.extend(return_route.segments)
                    total_count += len(return_route.segments)
                    return_planned = True
                event = MissionEvent.MORE_SEGMENTS if pending else MissionEvent.ALL_SEGMENTS_COMPLETE
                fsm = fsm.dispatch(event)
                continue
            event = MissionEvent.CANCEL if execution.status is SegmentExecutionStatus.CANCELLED else MissionEvent.SEGMENT_FAILED
            fsm.dispatch(event)
            return MissionRunResult(
                execution.status,
                execution.result_code,
                execution.reason,
                request.mission_id,
                artifact_path,
            )
        return MissionRunResult(
            SegmentExecutionStatus.SUCCESS,
            0,
            "",
            request.mission_id,
            artifact_path,
        )
