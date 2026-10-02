"""ROS adapters for canonical mission route segment execution."""

from __future__ import annotations

import threading
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
    HandoffSafetyError,
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
            elif active is SegmentType.STAIR:
                self._stair.cancel_goal()
            elif active is SegmentType.FLOOR_TRANSITION:
                self._floor.cancel_goal()

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
        try:
            result = self._navigation.execute(
                NavigationRequest(self._locations[segment.target_id], int(floor.map_generation),
                                  entry_approach=self._locations[segment.target_id].type in ("STAIR_ENTRY", "LANDING"),
                                  arrival_region=(self.photo_settings['arrival_radius_m'],self.photo_settings['arrival_yaw_rad'])
                                  if self.photo_settings and self._locations[segment.target_id].type=='SCAN' else None),
                cancellation_requested=context.cancellation_requested,
            )
        except (NavigationReadinessError, InvalidNavigationGoalError) as error:
            return SegmentExecution.failed(4, str(error), "navigation boundary rejection")
        except NavigationLifecycleError as error:
            return SegmentExecution.abort(str(error), "move_base communication/correlation failure")
        if result.outcome is NavigationOutcome.SUCCEEDED:
            evidence = "NAV accepted; move_base status={} attempts={} entry_region={}".format(
                result.status, result.attempts, result.status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED))
            return SegmentExecution.success(evidence=evidence)
        if result.outcome is NavigationOutcome.CANCELLED:
            return SegmentExecution.cancelled("navigation cancelled at terminal status")
        if result.status == GoalStatus.LOST:
            return SegmentExecution.abort("move_base goal communication lost", "terminal LOST")
        return SegmentExecution.failed(4, getattr(self._navigation, "failure_reason", "") or "move_base navigation failed", "attempts={}".format(result.attempts))

    def _stair_segment(self, segment: RouteSegment, context: SegmentContext) -> SegmentExecution:
        expected_location = self._locations[segment.source_id]
        expected_pose = StairEntryPose(
            expected_location.x,
            expected_location.y,
            expected_location.yaw,
        )
        try:
            self._navigation.prepare_for_stair()
        except (NavigationLifecycleError, HandoffSafetyError) as error:
            return SegmentExecution.abort(str(error), "stair handoff barrier")
        fence = self._state.begin_stair_entry()
        entry = self._state.wait_for_stair_entry(fence, expected_pose)
        if context.cancellation_requested():
            return SegmentExecution.cancelled('mission cancelled during stair entry handoff')
        if not entry.accepted:
            return SegmentExecution.failed(4, entry.reason, "stair entry pose rejection")
        if not self._stair.wait_for_server(rospy.Duration(rospy.get_param("~child_wait_timeout", 5.0))):
            return SegmentExecution.abort("stair action server unavailable", "communication timeout")
        profile = self._stair_profiles[segment.profile_id]
        direction = StairTraversalGoal.UP if profile.direction is Direction.UP else StairTraversalGoal.DOWN
        token = self._stair_admission.issue(
            StairAdmissionContext(
                profile.id,
                direction,
                int(self._state.supervisor_state().ownership_epoch),
            ),
            lambda: self._state.wait_for_stair_entry(fence, expected_pose),
        )
        try:
            with self._active_lock:
                if context.cancellation_requested():
                    return SegmentExecution.cancelled('mission cancelled before stair goal')
                self._stair.send_goal(
                    StairTraversalGoal(
                        stair_id=profile.id,
                        direction=direction,
                        admission_token=token,
                    )
                )
            if not self._stair.wait_for_result():
                return SegmentExecution.abort("stair action wait ended without result", "communication loss")
            status = self._stair.get_state()
            result: StairTraversalResult | None = self._stair.get_result()
        finally:
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
        with self._active_lock:
            if context.cancellation_requested():
                return SegmentExecution.cancelled('mission cancelled before floor transition')
            self._floor.send_goal(
                FloorTransitionGoal(transition_id=segment.stair_id, target_floor=segment.target_floor)
            )
        if not self._floor.wait_for_result():
            return SegmentExecution.abort("floor action wait ended without result", "communication loss")
        status = self._floor.get_state()
        result: FloorTransitionResult | None = self._floor.get_result()
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
        if not returning and (self.photo_camera is None or not self.photo_camera.readiness()):
            raise ValueError('최신 RGB 카메라 입력이 없습니다. 센서 연결 후 다시 시작하세요.')

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
