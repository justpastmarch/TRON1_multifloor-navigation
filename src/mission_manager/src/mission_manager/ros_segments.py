"""ROS adapters for canonical mission route segment execution."""

from __future__ import annotations

import threading
import math
import time
from dataclasses import replace
from dataclasses import dataclass
from typing import Dict

import actionlib
from actionlib_msgs.msg import GoalStatus
from multifloor_manager.msg import (
    FloorTransitionAction,
    FloorTransitionGoal,
    FloorTransitionResult,
)
import rospy
from stair_supervisor.configuration import Direction, StairProfile
from stair_supervisor.msg import StairTraversalAction, StairTraversalGoal, StairTraversalResult

from mission_manager.configuration import Location, ScanProfile
from mission_manager.mission_orchestrator import (
    RouteRecordingRequest,
    SegmentContext,
    SegmentExecution,
    SegmentExecutor,
)
from mission_manager.recording_session import RecordingSession
from mission_manager.navigation_executor import (
    InvalidNavigationGoalError,
    NavigationExecutor,
    NavigationLifecycleError,
    NavigationOutcome,
    NavigationReadinessError,
    NavigationRequest,
)
from mission_manager.route_planner import RouteSegment
from mission_manager.scan_recorder import RecordingDisposition, RecordingIdentity, ScanRecorder, ScanRecorderError
from mission_manager.segment_types import SegmentType
from mission_manager.ros_state import RosStateMonitor
from mission_manager.stair_admission import StairAdmissionContext, StairAdmissionIssuer
from mission_manager.stair_entry_gate import StairEntryPose


class ChildActionWait:
    """One goal's wall-clock liveness; live status permits arbitrarily long phases."""

    def __init__(self, client, silence_sec, cancel_sec, clock=time.monotonic):
        if not all(math.isfinite(value) and value > 0 for value in (silence_sec, cancel_sec)):
            raise ValueError('child communication/cancel timeouts must be positive and finite')
        self.client, self.clock = client, clock
        self.silence_sec, self.cancel_sec = silence_sec, cancel_sec
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.activity = clock()
        self.last_status = getattr(getattr(client, 'action_client', None), 'last_status_msg', None)
        self.cancel_at = None
        self.reason = ''
        self.terminal_at = None
        self.status, self.result = None, None
        self.goal_id = None

    def bind_goal(self):
        # Noetic SimpleActionClient exposes the generated ID on its goal handle.
        machine = getattr(getattr(self.client, 'gh', None), 'comm_state_machine', None)
        identifier = getattr(getattr(getattr(machine, 'action_goal', None), 'goal_id', None), 'id', None)
        with self.lock:self.goal_id = identifier if isinstance(identifier, str) and identifier else None

    def feedback(self, _message):
        with self.lock:self.activity = self.clock()

    def completed(self, status, result):
        with self.lock:
            self.status, self.result = status, result
            self.done.set()

    def confirmed_terminal(self):
        with self.lock:
            return (self.done.is_set() and self.result is not None and
                    self.status in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                                    GoalStatus.PREEMPTED, GoalStatus.RECALLED))

    def cancel(self, reason=''):
        with self.lock:
            if self.done.is_set() or self.cancel_at is not None:return
            self.cancel_at, self.reason = self.clock(), reason
        self.client.cancel_goal()

    def wait(self, cancelled):
        while not self.done.wait(.2):
            now = self.clock()
            status = getattr(getattr(self.client, 'action_client', None), 'last_status_msg', None)
            get_state = getattr(self.client, 'get_state', None)
            goal_status = get_state() if callable(get_state) else None
            with self.lock:
                # Only this goal's status renews liveness: an empty array or
                # another goal's heartbeat cannot acknowledge our request.
                if status is not None and status is not self.last_status:
                    self.last_status = status
                    if self.goal_id is not None and any(
                            getattr(getattr(entry, 'goal_id', None), 'id', None) == self.goal_id
                            for entry in getattr(status, 'status_list', ())):
                        self.activity = now
                silent = now-self.activity >= self.silence_sec
                # get_state refers to this client's goal handle, not another
                # goal in the server's status array. Terminal status alone is
                # not its correlated result or proof of successful arrival.
                if goal_status in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                                   GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST):
                    if self.terminal_at is None:self.terminal_at = now
                result_missing = self.terminal_at is not None and now-self.terminal_at >= self.cancel_sec
            if cancelled():self.cancel()
            elif silent:self.cancel('child action communication stopped')
            with self.lock:
                expired = self.cancel_at is not None and now-self.cancel_at >= self.cancel_sec
            if self.done.is_set():break
            if result_missing:return None, None, 'child action reported terminal status without its result'
            if expired:
                return None, None, self.reason or 'child action did not acknowledge cancellation'
            if rospy.is_shutdown():return None, None, 'mission node shutdown during child wait'
        with self.lock:
            error = self.reason if self.status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED) else ''
            return self.status, self.result, error


@dataclass(frozen=True)
class RosSegmentResources:
    """Immutable dependency bundle shared by all ROS segment adapters."""

    __slots__ = (
        "navigation", "state", "locations", "scan_profiles", "stair_profiles",
        "scan_recorder", "stair_admission",
    )

    navigation: NavigationExecutor
    state: RosStateMonitor
    locations: tuple[Location, ...]
    scan_profiles: tuple[ScanProfile, ...]
    stair_profiles: tuple[StairProfile, ...]
    scan_recorder: ScanRecorder
    stair_admission: StairAdmissionIssuer


class RosSegmentExecutor(SegmentExecutor):
    """Route each canonical segment to exactly one operational child surface."""

    def __init__(self, resources: RosSegmentResources) -> None:
        self._navigation = resources.navigation
        self._state = resources.state
        self._locations: Dict[str, Location] = {item.id: item for item in resources.locations}
        self._scan_profiles: Dict[str, ScanProfile] = {item.id: item for item in resources.scan_profiles}
        self._stair_profiles: Dict[str, StairProfile] = {item.id: item for item in resources.stair_profiles}
        self._scan_recorder = resources.scan_recorder
        self._stair_admission = resources.stair_admission
        self._stair = actionlib.SimpleActionClient(
            rospy.get_param("~stair_action", "/stair_traversal"), StairTraversalAction
        )
        self._floor = actionlib.SimpleActionClient(
            rospy.get_param("~floor_action", "/multifloor/floor_transition"), FloorTransitionAction
        )
        self._active_lock = threading.RLock()
        self._active_type: SegmentType | None = None
        self._child_waits = {}
        self.photo_settings = None
        self.photo_camera = None
        self.return_hold_location = None

    def execute(self, segment: RouteSegment, context: SegmentContext) -> SegmentExecution:
        if context.cancellation_requested():
            return SegmentExecution.cancelled('mission cancelled before segment dispatch')
        health = self._state.health()
        if not health.healthy:
            return SegmentExecution.abort(health.reason, "state health gate")
        with self._active_lock:
            self._active_type = segment.type
        try:
            handlers = {
                SegmentType.NAVIGATION: self._navigation_segment,
                SegmentType.STAIR: self._stair_segment,
                SegmentType.FLOOR_TRANSITION: self._floor_segment,
                SegmentType.SCAN: self._scan_segment,
            }
            return handlers[segment.type](segment, context)
        finally:
            with self._active_lock:
                self._active_type = None

    def cancel_active(self) -> None:
        with self._active_lock:
            active = self._active_type
            if active is SegmentType.NAVIGATION:
                self._navigation.request_cancel()
            elif active in (SegmentType.STAIR, SegmentType.FLOOR_TRANSITION):
                client = self._stair if active is SegmentType.STAIR else self._floor
                wait = self._child_waits.get(client)
                if wait is not None:wait.cancel()
                else:client.cancel_goal()

    def _run_child(self, client, goal, cancelled):
        with self._active_lock:
            previous = self._child_waits.get(client)
            if previous is not None and not previous.confirmed_terminal():
                # LOST/DONE without a result is not proof of released control.
                # A valid late result or confirmed idle receiver permits recovery.
                from stair_supervisor.msg import SupervisorState
                idle = self._state.health().healthy and (
                    self._state.supervisor_state().state == SupervisorState.NAV
                    if client is self._stair else
                    self._state.floor_state().state == self._state.floor_state().READY)
                if not idle:
                    return None, None, 'previous child command is unresolved; confirm receiver idle before retry'
            if cancelled():return GoalStatus.PREEMPTED, None, ''
            wait = ChildActionWait(
                client, float(rospy.get_param('~child_communication_timeout', 30.)),
                float(rospy.get_param('~child_cancel_timeout', 5.)))
            self._child_waits[client] = wait
            client.send_goal(goal, done_cb=wait.completed, feedback_cb=wait.feedback)
            wait.bind_goal()
        return wait.wait(cancelled)

    def start_recording(self, request: RouteRecordingRequest) -> RecordingSession:
        """Start one mission-owned rosbag session before route dispatch."""
        profile = self._scan_profiles[request.profile_id]
        identity = RecordingIdentity(
            request.mission_id,
            request.location_id,
            int(rospy.Time.now().to_nsec()),
        )
        return self._scan_recorder.start(profile, identity)

    def _navigation_segment(self, segment: RouteSegment, context: SegmentContext) -> SegmentExecution:
        floor = self._state.floor_state()
        location = self._locations[segment.target_id]
        return self._navigate_location(location, int(floor.map_generation), context.cancellation_requested,
                                       entry=location.type in ('STAIR_ENTRY', 'LANDING'))

    def prepare_rooftop_navigation(self):
        from std_srvs.srv import Trigger
        name = '/stair_supervisor/prepare_rooftop_navigation'
        from multifloor_manager.ros_services import BoundedServiceCaller
        result = BoundedServiceCaller(5.0).call(name, rospy.ServiceProxy(name, Trigger))
        if not result.success:
            raise ValueError('옥상 WALK 전환 실패: '+result.message)

    def navigate_point(self, request, cancelled):
        health = self._state.health()
        if not health.healthy:
            return SegmentExecution.abort(health.reason, 'state health gate')
        with self._active_lock:
            self._active_type = SegmentType.NAVIGATION
        try:
            return self._navigate_location(request.point_location(), request.point_map_generation, cancelled)
        finally:
            with self._active_lock:
                self._active_type = None

    def _navigate_location(self, location, generation, cancelled, entry=False):
        try:
            result = self._navigation.execute(
                NavigationRequest(location, generation,
                                  entry_approach=entry,
                                  arrival_region=(self.photo_settings['arrival_radius_m'],self.photo_settings['arrival_yaw_rad'])
                                  if self.photo_settings and location.id in self.photo_settings['locations'] else None),
                cancellation_requested=cancelled,
            )
        except (NavigationReadinessError, InvalidNavigationGoalError) as error:
            return SegmentExecution.failed(4, str(error), "navigation boundary rejection")
        except NavigationLifecycleError as error:
            return SegmentExecution.abort(str(error), "move_base communication/correlation failure")
        if result.outcome is NavigationOutcome.SUCCEEDED:
            evidence = "NAV arrival accepted; move_base status={} attempts={} region_stop={}".format(
                result.status, result.attempts, result.status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED))
            return SegmentExecution.success(evidence=evidence)
        if result.outcome is NavigationOutcome.CANCELLED:
            return SegmentExecution.cancelled("navigation cancelled at terminal status")
        if result.status == GoalStatus.LOST:
            return SegmentExecution.abort("move_base goal communication lost", "terminal LOST")
        return SegmentExecution.failed(4, getattr(self._navigation, "failure_reason", "") or "move_base navigation failed", "attempts={}".format(result.attempts))

    def _stair_segment(self, segment: RouteSegment, context: SegmentContext) -> SegmentExecution:
        resumed = bool(context.resume_run_id)
        if resumed and self._state.floor_state().floor_id != segment.source_floor:
            return SegmentExecution.failed(4, "actual floor differs from retained stair route", "stair resume floor mismatch")
        token = ""
        if not resumed:
            expected_location = self._locations[segment.source_id]
            expected_pose = StairEntryPose(expected_location.x, expected_location.y, expected_location.yaw)
            try:
                self._navigation.prepare_for_stair()
            except NavigationLifecycleError as error:
                return SegmentExecution.abort(str(error), "stair handoff goal completion")
        if not self._stair.wait_for_server(rospy.Duration(rospy.get_param("~child_wait_timeout", 5.0))):
            return SegmentExecution.abort("stair action server unavailable", "communication timeout")
        if not resumed:
            fence = self._state.begin_stair_entry()
            entry = self._state.wait_for_stair_entry(fence, expected_pose)
            if context.cancellation_requested():
                return SegmentExecution.cancelled('mission cancelled during stair entry handoff')
            if not entry.accepted:
                return SegmentExecution.failed(4, entry.reason, "stair entry pose rejection")
        profile = self._stair_profiles[segment.profile_id]
        direction = StairTraversalGoal.UP if profile.direction is Direction.UP else StairTraversalGoal.DOWN
        if not resumed:
            token = self._stair_admission.issue(
                StairAdmissionContext(profile.id, direction,
                                      int(self._state.supervisor_state().ownership_epoch)),
                lambda: self._state.stair_entry_decision(fence, expected_pose),
            )
        try:
            status, result, error = self._run_child(
                self._stair, StairTraversalGoal(
                    stair_id=profile.id, direction=direction, admission_token=token,
                    resume_run_id=context.resume_run_id, resume_revision=context.resume_revision,
                    resume_phase=context.resume_phase,
                ), context.cancellation_requested)
            if error:
                return SegmentExecution.abort(error, "stair child communication/cancellation timeout")
        finally:
            if token:
                self._stair_admission.clear(token)
        if status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            return SegmentExecution.cancelled("stair traversal cancelled at safe checkpoint")
        if result is None or status == GoalStatus.LOST:
            return SegmentExecution.abort("stair action result unavailable", "communication loss")
        if status == GoalStatus.SUCCEEDED and result.result_code == StairTraversalResult.OK:
            return SegmentExecution.success(evidence="stair action terminal success")
        if result.result_code == StairTraversalResult.COMMUNICATION_LOST:
            return SegmentExecution.abort(result.reason, "child communication loss")
        if status == GoalStatus.SUCCEEDED:
            return SegmentExecution.abort("stair action status/result incoherent", "terminal coherence")
        return SegmentExecution.failed(int(result.result_code), result.reason, "stair child failure")

    def _floor_segment(self, segment: RouteSegment, context: SegmentContext) -> SegmentExecution:
        if not self._floor.wait_for_server(rospy.Duration(rospy.get_param("~child_wait_timeout", 5.0))):
            return SegmentExecution.abort("floor action server unavailable", "communication timeout")
        generation = int(self._state.floor_state().map_generation)
        if segment.stair_id is None:
            return SegmentExecution.abort("floor transition missing stair id", "route contract")
        status, result, error = self._run_child(
            self._floor, FloorTransitionGoal(transition_id=segment.stair_id, target_floor=segment.target_floor),
            context.cancellation_requested)
        if error:
            return SegmentExecution.abort(error, "floor child communication/cancellation timeout")
        if status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            return SegmentExecution.cancelled("floor transition cancelled at safe checkpoint")
        if result is None or status == GoalStatus.LOST:
            return SegmentExecution.abort("floor transition result unavailable", "communication loss")
        coherent = (
            status == GoalStatus.SUCCEEDED
            and result.result_code == FloorTransitionResult.OK
            and result.floor_id == segment.target_floor
            and int(result.map_generation) > generation
        )
        if coherent:
            timeout = float(rospy.get_param("~child_wait_timeout", 5.0))
            if not self._state.wait_for_floor(
                result.floor_id,
                int(result.map_generation),
                timeout,
            ):
                return SegmentExecution.failed(
                    6,
                    "floor state did not converge after transition",
                    "floor action/state synchronization timeout",
                )
            return SegmentExecution.success(
                evidence="floor={} generation={}".format(result.floor_id, result.map_generation)
            )
        if status == GoalStatus.SUCCEEDED:
            return SegmentExecution.abort("floor action status/result incoherent", "terminal coherence")
        return SegmentExecution.failed(6, result.reason, "localization child failure")

    def _scan_segment(self, segment: RouteSegment, context: SegmentContext) -> SegmentExecution:
        profile = self._scan_profiles[segment.scan_profile_id]
        identity = RecordingIdentity(
            context.mission_id,
            segment.target_id,
            int(rospy.Time.now().to_nsec()),
        )
        try:
            result = self._scan_recorder.record(profile, identity, context.cancellation_requested)
        except ScanRecorderError as error:
            return SegmentExecution.failed(7, str(error), "scan recorder failure")
        if result.disposition is RecordingDisposition.PREEMPTED:
            return SegmentExecution.cancelled("scan finalized after cancellation")
        return SegmentExecution.success(str(result.artifact_path), "validated rosbag artifact")

    def photo_preflight(self, returning=False):
        if getattr(self,'photo_arrival_hold',None) is None:
            raise ValueError('촬영/복귀 위치 유지가 설정되지 않았습니다.')
        # Camera failure is recorded per photo point by PhotoMission; it
        # cannot block navigation and stair departure from the current floor.

    def photo_origin(self, location_id):
        from std_srvs.srv import Empty
        from multifloor_manager.ros_services import BoundedServiceCaller
        deadline=time.monotonic()+3.
        snapshot=self._state.hold_pose()
        if snapshot is None:
            proxy=rospy.ServiceProxy('/request_nomotion_update',Empty)
            BoundedServiceCaller(2.).call('/request_nomotion_update',proxy)
        while snapshot is None and time.monotonic()<deadline:
            time.sleep(.05);snapshot=self._state.hold_pose()
        if snapshot is None:raise ValueError('출발 위치를 저장할 최신 지도 측위가 없습니다.')
        floor,generation,x,y,yaw=snapshot
        origin=self._locations[location_id]
        if floor!=origin.floor_id:raise ValueError('출발 위치와 현재 층이 다릅니다.')
        return replace(origin,x=x,y=y,yaw=yaw,type='HOME')

    def capture_photo(self, location_id, output, cancelled):
        if self.photo_camera is None:return SegmentExecution.failed(7,'photo camera unavailable')
        return self.photo_camera.capture(location_id,output,cancelled)

    def return_photo_origin(self, location, cancelled):
        floor=self._state.floor_state()
        if floor.floor_id!=location.floor_id:return SegmentExecution.failed(6,'return floor mismatch')
        with self._active_lock:self._active_type=SegmentType.NAVIGATION
        try:
            request=NavigationRequest(location,int(floor.map_generation),arrival_region=(
                self.photo_settings['arrival_radius_m'],self.photo_settings['arrival_yaw_rad']))
            result=self._navigation.execute(request,cancelled)
            if result.outcome is NavigationOutcome.CANCELLED:return SegmentExecution.cancelled('return pose cancelled')
            if result.outcome is not NavigationOutcome.SUCCEEDED:return SegmentExecution.failed(4,'return pose navigation failed')
            # Idle hold follows the accepted actual pose, avoiding a second
            # exact-point correction immediately after region arrival.
            self.return_hold_location=self.photo_origin(location.id)
            return SegmentExecution.success(evidence='saved starting pose region reached')
        finally:
            with self._active_lock:self._active_type=None
