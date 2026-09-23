"""Strict move_base goal lifecycle for one named navigation segment."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum, unique
import math
from threading import RLock
from typing import Protocol

import actionlib
from actionlib_msgs.msg import GoalStatus
from genpy import Time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from multifloor_manager.msg import FloorState
import rospy
from stair_supervisor.msg import SupervisorState
from std_srvs.srv import Empty

from mission_manager.configuration import Location


MOVE_BASE_ACTION = "/move_base"
CLEAR_COSTMAPS_SERVICE = "/move_base/clear_costmaps"
GLOBAL_FRAME = "map"
_RETRYABLE_STATES = (GoalStatus.ABORTED, GoalStatus.LOST)
_CANCELLED_STATES = (GoalStatus.PREEMPTED, GoalStatus.RECALLED)


class MoveBaseClient(Protocol):
    def send_goal(
        self,
        goal: MoveBaseGoal,
        done_cb: Callable[[int, MoveBaseResult], None],
    ) -> None:
        ...

    def wait_for_result(self) -> bool:
        ...

    def get_state(self) -> int:
        ...

    def cancel_goal(self) -> None:
        ...


class HandoffBarrier(Protocol):
    def stop_and_confirm_stationary(self) -> bool:
        ...


@dataclass(frozen=True)
class NavigationRequest:
    __slots__ = ("location", "expected_generation")
    location: Location
    expected_generation: int


@unique
class NavigationOutcome(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    NAVIGATION_FAILED = "NAVIGATION_FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class NavigationResult:
    __slots__ = ("location_id", "outcome", "status", "attempts")
    location_id: str
    outcome: NavigationOutcome
    status: int
    attempts: int


@dataclass(frozen=True)
class NavigationReadinessError(Exception):
    __slots__ = ("location_id", "detail")
    location_id: str
    detail: str

    def __str__(self) -> str:
        return f"navigation readiness rejected {self.location_id!r}: {self.detail}"


@dataclass(frozen=True)
class InvalidNavigationGoalError(Exception):
    __slots__ = ("location_id", "detail")
    location_id: str
    detail: str

    def __str__(self) -> str:
        return f"invalid navigation goal {self.location_id!r}: {self.detail}"


@dataclass(frozen=True)
class NavigationLifecycleError(Exception):
    __slots__ = ("detail",)
    detail: str

    def __str__(self) -> str:
        return f"move_base lifecycle violation: {self.detail}"


@dataclass(frozen=True)
class HandoffSafetyError(Exception):
    __slots__ = ("detail",)
    detail: str

    def __str__(self) -> str:
        return f"navigation-to-stair handoff rejected: {self.detail}"


@dataclass(frozen=True)
class NavigationRuntime:
    __slots__ = (
        "action_client",
        "clear_costmaps",
        "floor_state",
        "supervisor_state",
        "handoff_barrier",
        "now",
    )
    action_client: MoveBaseClient
    clear_costmaps: Callable[[], None]
    floor_state: Callable[[], FloorState]
    supervisor_state: Callable[[], SupervisorState]
    handoff_barrier: HandoffBarrier
    now: Callable[[], Time]


@dataclass(frozen=True)
class RosNavigationSources:
    __slots__ = ("floor_state", "supervisor_state", "handoff_barrier")
    floor_state: Callable[[], FloorState]
    supervisor_state: Callable[[], SupervisorState]
    handoff_barrier: HandoffBarrier


class NavigationExecutor:
    """Own mutable correlation state for at most one move_base goal."""

    def __init__(self, runtime: NavigationRuntime) -> None:
        self._runtime = runtime
        self._lifecycle_lock = RLock()
        self._next_token = 1
        self._active_token: int | None = None
        self._terminal_token: int | None = None
        self._terminal_status: int | None = None
        self._executing = False
        self._cancel_requested = False
        self._active_request = None

    @classmethod
    def create_ros(cls, sources: RosNavigationSources) -> "NavigationExecutor":
        """Create ROS adapters without initializing a node or sending a goal."""
        client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)
        clear_costmaps = rospy.ServiceProxy(CLEAR_COSTMAPS_SERVICE, Empty)

        def clear() -> None:
            clear_costmaps()

        return cls(
            NavigationRuntime(
                client,
                clear,
                sources.floor_state,
                sources.supervisor_state,
                sources.handoff_barrier,
                rospy.Time.now,
            )
        )

    @property
    def has_active_goal(self) -> bool:
        with self._lifecycle_lock:
            return self._active_token is not None

    def execute(self, request: NavigationRequest, cancellation_requested=lambda: False) -> NavigationResult:
        """Execute one named Location goal, with at most one terminal retry."""
        with self._lifecycle_lock:
            if self._active_token is not None or self._executing:
                raise NavigationLifecycleError("another goal is active")
            self._executing = True
            self._active_request = request
            self._cancel_requested = False
        try:
            for attempt in (1, 2):
                self._require_ready(request)
                goal = self._build_goal(request.location)
                token = self._send_if_eligible(goal, cancellation_requested)
                if token is None:
                    status = self._terminal_status if self._terminal_status is not None else GoalStatus.RECALLED
                    return NavigationResult(request.location.id, NavigationOutcome.CANCELLED, status, attempt)
                status = self._await_terminal(token)
                if status == GoalStatus.SUCCEEDED:
                    return NavigationResult(request.location.id, NavigationOutcome.SUCCEEDED, status, attempt)
                if status in _CANCELLED_STATES or self._cancel_requested:
                    return NavigationResult(request.location.id, NavigationOutcome.CANCELLED, status, attempt)
                if status not in _RETRYABLE_STATES or attempt == 2:
                    return NavigationResult(request.location.id, NavigationOutcome.NAVIGATION_FAILED, status, attempt)
                self._runtime.clear_costmaps()
                if self._cancel_requested:
                    return NavigationResult(request.location.id, NavigationOutcome.CANCELLED, status, attempt)
            raise NavigationLifecycleError("retry loop did not return")
        finally:
            with self._lifecycle_lock:
                self._executing = False
                self._active_request = None
                self._cancel_requested = False

    def cancel(self) -> None:
        """Cancel the tracked goal and wait for PREEMPTED or RECALLED."""
        token = self._request_cancel()
        if token is None:
            return
        status = self._await_terminal(token)
        if status not in _CANCELLED_STATES:
            raise NavigationLifecycleError(f"cancel completed with status {status}")

    def request_cancel(self) -> None:
        """Request cancellation without competing with the execution thread's wait."""
        self._request_cancel()

    def request_cancel_for(self, request: NavigationRequest) -> bool:
        """Cancel only this invocation; a delayed idle-hold tick cannot stop a new mission."""
        with self._lifecycle_lock:
            if self._active_request is not request:
                return False
            self._request_cancel()
            return True

    def _request_cancel(self) -> int | None:
        with self._lifecycle_lock:
            if self._executing:
                self._cancel_requested = True
            token = self._active_token
            if token is None:
                return None
            self._runtime.action_client.cancel_goal()
            return token

    def prepare_for_stair(self) -> None:
        """Establish a cancel/terminal/zero/stationary ownership barrier."""
        self.cancel()
        if not self._runtime.handoff_barrier.stop_and_confirm_stationary():
            raise HandoffSafetyError("zero command lacks stationary evidence")

    def _require_ready(self, request: NavigationRequest) -> None:
        floor = self._runtime.floor_state()
        supervisor = self._runtime.supervisor_state()
        valid_floor = (
            floor.state == FloorState.READY
            and floor.floor_id == request.location.floor_id
            and floor.map_generation == request.expected_generation
        )
        if not valid_floor:
            raise NavigationReadinessError(request.location.id, "floor state, floor id, or generation mismatch")
        if supervisor.state != SupervisorState.NAV:
            raise NavigationReadinessError(request.location.id, "supervisor does not own NAV")

    def _build_goal(self, location: Location) -> MoveBaseGoal:
        values = (location.x, location.y, location.yaw)
        if not all(math.isfinite(value) for value in values):
            raise InvalidNavigationGoalError(location.id, "pose contains a non-finite value")
        half_yaw = location.yaw / 2.0
        z = math.sin(half_yaw)
        w = math.cos(half_yaw)
        norm = math.hypot(z, w)
        if not math.isfinite(norm) or norm <= 0.0:
            raise InvalidNavigationGoalError(location.id, "quaternion is invalid")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = GLOBAL_FRAME
        goal.target_pose.header.stamp = self._runtime.now()
        goal.target_pose.pose.position.x = location.x
        goal.target_pose.pose.position.y = location.y
        goal.target_pose.pose.orientation.z = z / norm
        goal.target_pose.pose.orientation.w = w / norm
        return goal

    def _send_if_eligible(self, goal: MoveBaseGoal, cancellation_requested=lambda: False) -> int | None:
        with self._lifecycle_lock:
            if self._cancel_requested or cancellation_requested():
                return None
            token = self._next_token
            self._next_token += 1
            self._active_token = token
            self._terminal_token = None
            self._terminal_status = None

            def completed(status: int, _result: MoveBaseResult) -> None:
                with self._lifecycle_lock:
                    if self._active_token == token:
                        self._terminal_token = token
                        self._terminal_status = status

            self._runtime.action_client.send_goal(goal, completed)
            return token

    def _await_terminal(self, token: int) -> int:
        if not self._runtime.action_client.wait_for_result():
            raise NavigationLifecycleError("unbounded wait returned before a terminal result")
        state = self._runtime.action_client.get_state()
        with self._lifecycle_lock:
            if self._terminal_token != token or self._terminal_status != state:
                raise NavigationLifecycleError("terminal callback does not match the tracked goal")
            self._active_token = None
        return state
