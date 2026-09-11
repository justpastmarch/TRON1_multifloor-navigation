"""Mission.action boundary with explicit goal admission and terminal coherence."""

from __future__ import annotations

from dataclasses import dataclass
import re
import threading
import uuid

import actionlib
from actionlib.server_goal_handle import ServerGoalHandle
import rospy

from mission_manager.mission_orchestrator import (
    MissionOrchestrator,
    MissionProgress,
    MissionRunRequest,
    MissionType,
    SegmentExecutionStatus,
)
from mission_manager.msg import MissionAction, MissionFeedback, MissionGoal, MissionResult


_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


@dataclass(frozen=True)
class GoalAdmission:
    __slots__ = ("accepted", "mission_type", "reason")
    accepted: bool
    mission_type: MissionType | None
    reason: str


def admit_goal(goal: MissionGoal, destinations: frozenset[str]) -> GoalAdmission:
    """Parse an untrusted ROS goal before any child action can be touched."""
    if _IDENTIFIER.fullmatch(goal.destination_id) is None:
        return GoalAdmission(False, None, "destination_id is malformed")
    if goal.destination_id not in destinations:
        return GoalAdmission(False, None, "destination_id is unknown")
    try:
        mission_type = MissionType(goal.mission_type)
    except ValueError:
        return GoalAdmission(False, None, "mission_type is unknown")
    return GoalAdmission(True, mission_type, "")


class MissionActionServer:
    """Reject concurrent goals rather than implicitly preempting active missions."""

    def __init__(
        self,
        action_name: str,
        orchestrator: MissionOrchestrator,
        destinations: frozenset[str],
    ) -> None:
        self._orchestrator = orchestrator
        self._destinations = destinations
        self._lock = threading.RLock()
        self._active: ServerGoalHandle | None = None
        self._active_goal_id = ""
        self._cancel_requested = threading.Event()
        self._server = actionlib.ActionServer(
            action_name,
            MissionAction,
            self._goal_callback,
            self._cancel_callback,
            auto_start=False,
        )
        self._server.start()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self) -> None:
        """Stop actionlib's status timer before rospy closes its publishers."""
        timer = self._server.status_timer
        if timer is not None:
            timer.shutdown()

    def _goal_callback(self, handle: ServerGoalHandle) -> None:
        goal: MissionGoal = handle.get_goal()
        with self._lock:
            if self._active is not None:
                handle.set_rejected(
                    self._result(MissionResult.BUSY, "another mission is active", "", ""),
                    "mission manager busy",
                )
                return
            admission = admit_goal(goal, self._destinations)
            if not admission.accepted or admission.mission_type is None:
                handle.set_rejected(
                    self._result(MissionResult.INVALID_GOAL, admission.reason, "", ""),
                    admission.reason,
                )
                return
            mission_id = "mission_{}".format(uuid.uuid4().hex)
            self._active = handle
            self._active_goal_id = handle.get_goal_id().id
            self._cancel_requested.clear()
            handle.set_accepted("mission accepted")
            request = MissionRunRequest(
                mission_id,
                goal.destination_id,
                admission.mission_type,
                bool(goal.return_after_task),
            )
        worker = threading.Thread(
            target=self._execute,
            args=(handle, request),
            name="mission-action-worker",
            daemon=True,
        )
        worker.start()

    def _cancel_callback(self, handle: ServerGoalHandle) -> None:
        with self._lock:
            if self._active is None or handle.get_goal_id().id != self._active_goal_id:
                return
            self._cancel_requested.set()
        self._orchestrator.cancel_active()

    def _execute(self, handle: ServerGoalHandle, request: MissionRunRequest) -> None:
        try:
            outcome = self._orchestrator.run(
                request,
                lambda progress: self._publish_feedback(handle, progress),
                self._cancel_requested.is_set,
            )
            result = self._result(
                outcome.result_code,
                outcome.reason,
                outcome.mission_id,
                outcome.artifact_path,
            )
            self._release(handle)
            if outcome.status is SegmentExecutionStatus.SUCCESS:
                handle.set_succeeded(result, "mission complete")
            elif outcome.status is SegmentExecutionStatus.CANCELLED:
                handle.set_canceled(result, outcome.reason)
            else:
                handle.set_aborted(result, outcome.reason)
        except Exception as error:  # noqa: BROAD_EXCEPT_OK - ROS action boundary must terminate coherently.
            rospy.logerr("mission execution boundary failed: %s", error)
            self._release(handle)
            handle.set_aborted(
                self._result(MissionResult.MISSION_ABORT, str(error), request.mission_id, ""),
                "mission boundary failure",
            )

    def _release(self, handle: ServerGoalHandle) -> None:
        with self._lock:
            if handle.get_goal_id().id == self._active_goal_id:
                self._active = None
                self._active_goal_id = ""
                self._cancel_requested.clear()

    @staticmethod
    def _result(code: int, reason: str, mission_id: str, artifact_path: str) -> MissionResult:
        return MissionResult(
            result_code=code,
            reason=reason,
            mission_id=mission_id,
            artifact_path=artifact_path,
        )

    @staticmethod
    def _publish_feedback(handle: ServerGoalHandle, progress: MissionProgress) -> None:
        handle.publish_feedback(
            MissionFeedback(
                mission_id=progress.mission_id,
                state=progress.state,
                current_floor=progress.current_floor,
                segment_type=progress.segment_type,
                segment_index=progress.segment_index,
                segment_count=progress.segment_count,
                target_id=progress.target_id,
            )
        )
        rospy.logdebug(
            "mission segment evidence mission_id=%s segment_index=%s evidence=%s",
            progress.mission_id,
            progress.segment_index,
            progress.evidence,
        )
