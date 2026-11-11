"""Operational FloorTransition action boundary for multifloor_manager."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import threading
import time
from typing import Dict, Tuple

import actionlib
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import LoadMap, LoadMapResponse
import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import tf2_ros

from multifloor_manager.configuration import Floor, Stair, load_multifloor_configuration
from multifloor_manager.map_evidence import MapIdentity
from multifloor_manager.map_loader import load_map_identity
from multifloor_manager.msg import (
    FloorState,
    FloorTransitionAction,
    FloorTransitionFeedback,
    FloorTransitionResult,
)
from multifloor_manager.ros_runtime import EpochToken, RosEvidenceRuntime
from multifloor_manager.ros_services import BoundedServiceCaller, ServiceCallError
from multifloor_manager.stair_completion import StairCompletionCache
from multifloor_manager.transaction_record import (
    MAP_UNKNOWN,
    RESUME_INITIALPOSE,
    TransactionProgress,
    TransitionFault,
    resume_stage,
)
from stair_supervisor.msg import StairTraversalActionGoal, StairTraversalActionResult
from multifloor_manager.tag_evidence import FloorId, FloorTagSet, TagId
from multifloor_manager.transitions import PredicateObservation


class MultifloorManagerNode:
    """Own one fail-closed floor transition transaction at a time."""

    def __init__(self) -> None:
        package_root = Path(rospkg.RosPack().get_path("multifloor_manager"))
        config_root = Path(rospy.get_param("~config_root", str(package_root / "config"))).resolve()
        self.config_root = config_root
        configuration = load_multifloor_configuration(config_root)
        self.floors: Dict[str, Floor] = {floor.id: floor for floor in configuration.floors}
        self.stairs: Dict[str, Stair] = {stair.id: stair for stair in configuration.stairs}
        self.identities: Dict[str, MapIdentity] = {
            floor.id: load_map_identity(config_root / floor.map_yaml, floor.frame)
            for floor in configuration.floors
        }
        initial_floor = rospy.get_param("~initial_floor", configuration.floors[0].id)
        if initial_floor not in self.floors:
            raise ValueError("initial_floor is not configured: {}".format(initial_floor))
        grouped: Dict[str, list] = {}
        for tag in configuration.tags:
            grouped.setdefault(tag.floor_id, []).append(TagId(tag.id))
        tag_sets = tuple(
            FloorTagSet(FloorId(floor_id), tuple(sorted(tag_ids)))
            for floor_id, tag_ids in sorted(grouped.items())
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(35.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        startup_mode = rospy.get_param("~startup_localization", "disabled")
        if startup_mode not in ("auto", "manual", "disabled"):
            raise ValueError("invalid startup_localization mode")
        self.runtime = RosEvidenceRuntime(
            initial_floor,
            self.identities[initial_floor],
            tag_sets,
            self.tf_buffer,
            configuration.transitions[0],
            require_initial_localization=startup_mode != "disabled",
        )
        initialpose_topic = rospy.get_param("~initialpose_topic", "/initialpose" if startup_mode == "disabled" else "/multifloor/amcl_initialpose")
        if startup_mode != "disabled" and rospy.resolve_name(initialpose_topic) == "/initialpose":
            raise ValueError("managed initialpose output must differ from manual /initialpose input")
        self.initialpose_publisher = rospy.Publisher(
            initialpose_topic,
            PoseWithCovarianceStamped,
            queue_size=1,
        )
        self.change_map_name = rospy.get_param("~change_map_service", "/change_map")
        self.nomotion_name = rospy.get_param("~nomotion_service", "/request_nomotion_update")
        self.clear_costmaps_name = rospy.get_param("~clear_costmaps_service", "/move_base/clear_costmaps")
        self.change_map = rospy.ServiceProxy(self.change_map_name, LoadMap)
        self.nomotion_update = rospy.ServiceProxy(self.nomotion_name, Empty)
        self.clear_costmaps = rospy.ServiceProxy(self.clear_costmaps_name, Empty)
        self.service_timeout = float(rospy.get_param("~service_timeout", 5.0))
        self.services = BoundedServiceCaller(self.service_timeout)
        self.tag_timeout = float(rospy.get_param("~tag_timeout", 10.0))
        self.floor_confirmation_source = rospy.get_param("~floor_confirmation_source", "stair_result")
        if self.floor_confirmation_source not in ("stair_result", "tag"):
            raise ValueError("invalid floor_confirmation_source")
        self.stair_result_timeout = float(rospy.get_param("~stair_result_timeout", 10.0))
        self._stair_result_lock = threading.Lock()
        self.stair_completions = StairCompletionCache()
        self.transaction_timeout = float(rospy.get_param("~transaction_timeout", 30.0))
        self._active_lock = threading.Lock()
        self._active = False
        # A failed transition may resume with the same goal within this window.
        self.resume_max_age_sec = float(rospy.get_param("~transition_resume_max_age_sec", 300.0))
        self._progress = None
        self._last_fault = None
        subscribers = [
            rospy.Subscriber(rospy.get_param("~map_topic", "/map"), OccupancyGrid, self.runtime.map_callback, queue_size=2),
            rospy.Subscriber(rospy.get_param("~amcl_pose_topic", "/amcl_pose"), PoseWithCovarianceStamped, self.runtime.pose_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.runtime.scan_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param("~odom_topic", "/tron/wheel_odom_raw"), Odometry, self.runtime.odom_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param("~global_costmap_topic", "/move_base/global_costmap/costmap"), OccupancyGrid, self.runtime.costmap_callback, queue_size=2),
        ]
        if self.floor_confirmation_source == "tag":
            subscribers.append(rospy.Subscriber(
                rospy.get_param("~tag_topic", "/tag_detections"), AprilTagDetectionArray,
                self.runtime.tag_callback, queue_size=10))
        else:
            subscribers.extend((
                rospy.Subscriber("/stair_traversal/goal", StairTraversalActionGoal,
                                 self._stair_goal_callback, queue_size=10),
                rospy.Subscriber("/stair_traversal/result", StairTraversalActionResult,
                                 self._stair_result_callback, queue_size=10),
            ))
        self.subscribers = tuple(subscribers)
        self.server = actionlib.SimpleActionServer(
            rospy.get_param("~action_name", "/multifloor/floor_transition"),
            FloorTransitionAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        self.state_timer = rospy.Timer(rospy.Duration(0.5), self.runtime.publish_heartbeat)
        self.startup = None
        if startup_mode != "disabled":
            from multifloor_manager.ros_startup_localization import StartupLocalization
            self.startup = StartupLocalization(self, startup_mode)
        from multifloor_manager.ros_floor_selection import FloorSelection
        self.floor_selection = FloorSelection(self)

    def _stair_goal_callback(self, message: StairTraversalActionGoal) -> None:
        with self._stair_result_lock:
            self.stair_completions.note_goal(message.goal_id.id, message.goal.stair_id)
        with self.runtime.condition:
            self.runtime.condition.notify_all()

    def _stair_result_callback(self, message: StairTraversalActionResult) -> None:
        with self._stair_result_lock:
            self.stair_completions.note_result(
                message.status.goal_id.id, message.status.status,
                message.result.result_code, message.result.ownership_epoch)
        with self.runtime.condition:
            self.runtime.condition.notify_all()

    def _feedback(self, phase: str, detail: str) -> None:
        self.server.publish_feedback(FloorTransitionFeedback(phase=phase, detail=detail))

    def _mark(self, stage: str) -> None:
        progress = getattr(self, "_progress", None)
        if progress is not None:
            self._progress = progress.at(stage)

    def _stage(self, stage: str, detail: str) -> None:
        """Record the transaction stage, then publish it as action feedback."""
        self._mark(stage)
        self._feedback(stage, detail)

    def _observed_map_floor(self) -> str:
        """Floor whose configured identity matches the last observed /map, or ''."""
        fingerprint = self.runtime.map_state.current_fingerprint
        if fingerprint is None:
            return ""
        identity = fingerprint.identity()
        for floor_id, known in self.identities.items():
            if identity == known:
                return floor_id
        return ""

    def _observe_map(self, progress: TransactionProgress) -> TransactionProgress:
        """Upgrade the recorded map only on direct /map evidence for it."""
        observed = self._observed_map_floor()
        if observed and (observed == progress.map_floor
                         or (progress.map_floor == MAP_UNKNOWN and observed == progress.target_floor)):
            return progress.with_map(observed, True)
        return progress

    def _result(self, code: int, reason: str) -> FloorTransitionResult:
        return FloorTransitionResult(
            result_code=code,
            floor_id=self.runtime.current_floor,
            map_generation=int(self.runtime.map_state.generation),
            reason=reason,
        )

    def _fail(self, reason: str, preempted: bool = False) -> None:
        # floor_id stays the last confirmed floor; the loaded map, failed stage
        # and target confirmation are recorded beside it, never instead of it.
        progress = getattr(self, "_progress", None)
        if progress is not None:
            progress = self._observe_map(progress)
            reason = progress.describe(reason)
        self.runtime.finish_transition(
            FloorState.FAULT, reason, self.runtime.current_floor or self.runtime.initial_floor)
        if progress is not None:
            self._last_fault = TransitionFault(
                progress, int(self.runtime.map_state.generation), time.monotonic(), reason)
            self._progress = None
            try:
                rospy.set_param("~last_fault", self._last_fault.as_dict())
            except Exception as error:  # noqa: BROAD_EXCEPT_OK - diagnostics must not mask the fault.
                rospy.logwarn("could not publish ~last_fault: %s", error)
            rospy.logerr("floor transition fault: %s", reason)
        result = self._result(FloorTransitionResult.LOCALIZATION_FAILED, reason)
        if preempted:
            self.server.set_preempted(result, reason)
        else:
            self.server.set_aborted(result, reason)

    def _wait(self, predicate, deadline: float) -> Tuple[bool, str]:
        with self.runtime.condition:
            while True:
                if self.server.is_preempt_requested():
                    return False, "transition cancelled"
                if predicate():
                    return True, ""
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, "transition evidence timeout"
                self.runtime.condition.wait(min(remaining, 0.05))

    def _wait_with_localization_refresh(self, predicate, deadline: float) -> Tuple[bool, str]:
        """Refresh a stationary AMCL while waiting in the same floor transaction."""
        epoch = self.runtime.active_epoch
        while True:
            if self.server.is_preempt_requested():
                return False, "transition cancelled"
            if self.runtime.active_epoch != epoch:
                return False, "transition superseded"
            with self.runtime.condition:
                if self.runtime.active_epoch != epoch:
                    return False, "transition superseded"
                if predicate():
                    return True, ""
            if time.monotonic() >= deadline:
                return False, "transition evidence timeout"
            try:
                self.services.call(self.nomotion_name, self.nomotion_update)
            except ServiceCallError as error:
                return False, "request_nomotion_update failed: {}".format(error)
            ok, reason = self._wait(
                lambda: self.runtime.active_epoch == epoch and predicate(),
                min(deadline, time.monotonic() + 0.25),
            )
            if self.runtime.active_epoch != epoch:
                return False, "transition superseded"
            if ok or reason != "transition evidence timeout":
                return ok, reason

    def _wait_for_policy_ready(
        self,
        token: EpochToken,
        target: MapIdentity,
        costmap_marker: int,
    ) -> Tuple[bool, str]:
        self._feedback("POLICY_READY", "evaluating final transition policy")
        self.runtime.begin_policy(token)
        deadline = time.monotonic() + self.runtime.policy.timeout_sec
        refresh_at = 0.0
        while True:
            if self.server.is_preempt_requested():
                return False, "transition cancelled"
            if self.runtime.active_epoch != token.epoch:
                return False, "transition superseded"
            with self.runtime.condition:
                if self.server.is_preempt_requested():
                    return False, "transition cancelled"
                now = time.monotonic()
                localization = self.runtime.localization_evidence()
                self.runtime.record_policy_observation(
                    token,
                    "T_LOCALIZED",
                    PredicateObservation(localization is not None and localization.ready, now, token.epoch),
                )
                self.runtime.record_policy_observation(
                    token,
                    "T_COSTMAP_READY",
                    PredicateObservation(self._target_metadata_matches(target, costmap_marker), now, token.epoch),
                )
                ready, _evidence = self.runtime.evaluate_policy(token)
                if ready:
                    if self.server.is_preempt_requested():
                        return False, "transition cancelled"
                    return (True, "") if self.runtime.commit_ready(token) else (False, "policy commit rejected")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, "transition policy timeout"
                self.runtime.condition.wait(min(remaining, 0.05))
            if self.server.is_preempt_requested():
                return False, "transition cancelled"
            if self.runtime.active_epoch != token.epoch:
                return False, "transition superseded"
            if time.monotonic() >= refresh_at:
                try:
                    self.services.call(self.nomotion_name, self.nomotion_update)
                except ServiceCallError as error:
                    return False, "request_nomotion_update failed: {}".format(error)
                refresh_at = time.monotonic() + 0.25

    def _target_metadata_matches(self, target: MapIdentity, after_sequence: int) -> bool:
        observed = self.runtime.costmap_identity
        if observed is None or self.runtime.costmap_sequence <= after_sequence:
            return False
        return (
            observed.frame_id == target.frame_id
            and observed.width == target.width
            and observed.height == target.height
            and math.isclose(observed.resolution, target.resolution, abs_tol=1e-9)
            and observed.origin == target.origin
        )

    def _execute(self, goal) -> None:
        with self._active_lock:
            if self._active:
                self.server.set_aborted(self._result(FloorTransitionResult.BUSY, "transition busy"))
                return
            self._active = True
        try:
            self._run_transaction(goal.transition_id, goal.target_floor)
        finally:
            with self._active_lock:
                self._active = False

    def _run_transaction(self, transition_id: str, target_floor: str) -> None:
        stair = self.stairs.get(transition_id)
        current_floor = self.runtime.current_floor
        current_endpoint = stair.endpoint_for(current_floor) if stair is not None else None
        target_endpoint = stair.endpoint_for(target_floor) if stair is not None else None
        # Repeating the goal of a transaction that failed after the target was
        # confirmed continues from its first unfinished stage; it never spends
        # a second stair result and never repeats a map change already observed.
        resume = resume_stage(
            getattr(self, "_last_fault", None),
            transition_id=transition_id,
            target_floor=target_floor,
            in_fault=self.runtime.state == FloorState.FAULT,
            current_floor=current_floor,
            map_generation=int(self.runtime.map_state.generation),
            observed_map_floor=self._observed_map_floor(),
            now=time.monotonic(),
            max_age_sec=float(getattr(self, "resume_max_age_sec", 300.0)),
        )
        valid = (
            (self.runtime.state == FloorState.READY or resume is not None)
            and stair is not None
            and current_endpoint is not None
            and target_endpoint is not None
            and current_floor != target_floor
            and target_floor in self.identities
        )
        if not valid:
            self.server.set_aborted(self._result(FloorTransitionResult.INVALID_GOAL, "goal is not the current directed stair"))
            return
        target = self.identities[target_floor]
        previous_fault = getattr(self, "_last_fault", None)
        self._last_fault = None
        if resume is None:
            self._progress = TransactionProgress(transition_id, current_floor, target_floor,
                                                 map_floor=current_floor, map_observed=True)
        else:
            self._progress = previous_fault.progress.at("RESUME")
        if resume is None:
            self._stage("ARM_TARGET", "checking stair completion before map change")
        token = self.runtime.arm_transition(target_floor, target_endpoint.expected_tag_ids, target)
        if resume is not None:
            self._stage("RESUME", "continuing failed transition at {}; target confirmation already consumed".format(resume))
        elif self.floor_confirmation_source == "tag":
            ok, reason = self._wait(
                lambda: self.runtime.tag_accepted or self.runtime.wrong_floor_tag,
                time.monotonic() + self.tag_timeout,
            )
            if not ok or self.runtime.wrong_floor_tag:
                self._fail("wrong floor tag vote" if self.runtime.wrong_floor_tag else reason, self.server.is_preempt_requested())
                return
        else:
            expected_profile = stair.up_profile_id if target_floor == stair.to_floor else stair.down_profile_id
            completion = {"epoch": None}
            def matched_success():
                with self._stair_result_lock:
                    completion["epoch"] = self.stair_completions.consume(expected_profile)
                return completion["epoch"] is not None
            ok, reason = self._wait(matched_success, time.monotonic() + self.stair_result_timeout)
            if not ok:
                self._fail("matching recent stair success missing: " + reason, self.server.is_preempt_requested())
                return
            self._feedback("ARM_TARGET", "matched {} success epoch {}".format(expected_profile, completion["epoch"]))
        if resume is None:
            self._progress = replace(self._progress, floor_confirmed=True)
        self.runtime.record_policy_observation(
            token,
            "T_FLOOR_CONFIRMED",
            PredicateObservation(True, time.monotonic(), token.epoch),
        )
        if resume == RESUME_INITIALPOSE:
            self._stage("MAP_CONFIRM", "target map already observed in failed transaction; not reloading")
        else:
            self._stage("CHANGE_MAP", "calling map_server change_map")
            try:
                response = self.services.call(
                    self.change_map_name,
                    self.change_map,
                    str((self.config_root / self.floors[target_floor].map_yaml).resolve()),
                )
            except ServiceCallError as error:
                # A timed-out RPC may still have switched the map.
                self._progress = self._progress.with_map(MAP_UNKNOWN, False)
                self._fail("change_map failed: {}".format(error))
                return
            if response.result != LoadMapResponse.RESULT_SUCCESS:
                # map_server rejected the request and keeps serving its current map.
                self._fail("change_map result {}".format(response.result))
                return
            self._progress = self._progress.with_map(target_floor, False)
            self._stage("MAP_CONFIRM", "waiting for newer matching map publication")
            ok, reason = self._wait(
                lambda: self.runtime.map_guard is not None and not self.runtime.map_guard.armed,
                time.monotonic() + self.transaction_timeout,
            )
            if not ok:
                self._fail(reason, self.server.is_preempt_requested())
                return
            self._progress = self._progress.with_map(target_floor, True)
        pose_stamp = rospy.Time.now()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = pose_stamp
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = target_endpoint.target_landing.x
        pose.pose.pose.position.y = target_endpoint.target_landing.y
        orientation = target_endpoint.target_landing.orientation
        pose.pose.pose.orientation.x = orientation.x
        pose.pose.pose.orientation.y = orientation.y
        pose.pose.pose.orientation.z = orientation.z
        pose.pose.pose.orientation.w = orientation.w
        pose.pose.covariance = target_endpoint.covariance
        self._stage("INITIALPOSE", "publishing stored landing hypothesis")
        self.runtime.arm_localization(pose_stamp.to_nsec())
        self.initialpose_publisher.publish(pose)
        localization_deadline = time.monotonic() + self.transaction_timeout
        pose_marker = pose_stamp.to_nsec()
        for _ in range(3):
            # AMCL stamps the pose with scan acquisition time, not RPC completion.
            # Keep the initialpose/accepted-pose fence even if a new callback is
            # delivered before request_nomotion_update returns.
            ok, reason = self._wait_with_localization_refresh(
                lambda marker=pose_marker: self.runtime.has_pose_newer_than(marker),
                localization_deadline,
            )
            if not ok:
                self._fail(reason, self.server.is_preempt_requested())
                return
            pose_marker = self.runtime.pose_high_water_ns(pose_marker)
        self._stage("AMCL_READY", "waiting for covariance, samples, scan, odom, and TF")
        def localized() -> bool:
            evidence = self.runtime.localization_evidence()
            return evidence is not None and evidence.ready

        ok, reason = self._wait_with_localization_refresh(localized, localization_deadline)
        if not ok:
            self._fail(reason, self.server.is_preempt_requested())
            return
        self._stage("CLEAR_COSTMAPS", "clearing navigation costmaps")
        marker = self.runtime.costmap_sequence
        try:
            self.services.call(self.clear_costmaps_name, self.clear_costmaps)
        except ServiceCallError as error:
            self._fail("clear_costmaps failed: {}".format(error))
            return
        self._stage("COSTMAP_READY", "waiting for target global costmap metadata")
        ok, reason = self._wait_with_localization_refresh(
            lambda: self._target_metadata_matches(target, marker),
            time.monotonic() + self.transaction_timeout,
        )
        if not ok:
            self._fail(reason, self.server.is_preempt_requested())
            return
        self._mark("POLICY_READY")
        ok, reason = self._wait_for_policy_ready(token, target, marker)
        if not ok:
            self._fail(reason, self.server.is_preempt_requested())
            return
        self._progress = None
        self.runtime.finish_transition(FloorState.READY, "floor transition complete", target_floor)
        self.server.set_succeeded(self._result(FloorTransitionResult.OK, ""))


def run() -> int:
    """Construct the operational node after the sole entrypoint initializes rospy."""
    MultifloorManagerNode()
    rospy.spin()
    return 0
