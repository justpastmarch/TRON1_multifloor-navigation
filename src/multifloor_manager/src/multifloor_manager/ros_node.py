"""Operational FloorTransition action boundary for multifloor_manager."""

from __future__ import annotations

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
        self.transaction_timeout = float(rospy.get_param("~transaction_timeout", 30.0))
        self._active_lock = threading.Lock()
        self._active = False
        self.subscribers = (
            rospy.Subscriber(rospy.get_param("~map_topic", "/map"), OccupancyGrid, self.runtime.map_callback, queue_size=2),
            rospy.Subscriber(rospy.get_param("~tag_topic", "/tag_detections"), AprilTagDetectionArray, self.runtime.tag_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param("~amcl_pose_topic", "/amcl_pose"), PoseWithCovarianceStamped, self.runtime.pose_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.runtime.scan_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param("~odom_topic", "/tron/wheel_odom_raw"), Odometry, self.runtime.odom_callback, queue_size=10),
            rospy.Subscriber(rospy.get_param("~global_costmap_topic", "/move_base/global_costmap/costmap"), OccupancyGrid, self.runtime.costmap_callback, queue_size=2),
        )
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

    def _feedback(self, phase: str, detail: str) -> None:
        self.server.publish_feedback(FloorTransitionFeedback(phase=phase, detail=detail))

    def _result(self, code: int, reason: str) -> FloorTransitionResult:
        return FloorTransitionResult(
            result_code=code,
            floor_id=self.runtime.current_floor,
            map_generation=int(self.runtime.map_state.generation),
            reason=reason,
        )

    def _fail(self, reason: str, preempted: bool = False) -> None:
        self.runtime.finish_transition(FloorState.FAULT, reason)
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

    def _wait_for_policy_ready(
        self,
        token: EpochToken,
        target: MapIdentity,
        costmap_marker: int,
    ) -> Tuple[bool, str]:
        self._feedback("POLICY_READY", "evaluating final transition policy")
        self.runtime.begin_policy(token)
        deadline = time.monotonic() + self.runtime.policy.timeout_sec
        while True:
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
        valid = (
            self.runtime.state == FloorState.READY
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
        self._feedback("ARM_TARGET", "arming tag and target map evidence")
        token = self.runtime.arm_transition(target_floor, target_endpoint.expected_tag_ids, target)
        ok, reason = self._wait(
            lambda: self.runtime.tag_accepted or self.runtime.wrong_floor_tag,
            time.monotonic() + self.tag_timeout,
        )
        if not ok or self.runtime.wrong_floor_tag:
            self._fail("wrong floor tag vote" if self.runtime.wrong_floor_tag else reason, self.server.is_preempt_requested())
            return
        self.runtime.record_policy_observation(
            token,
            "T_FLOOR_CONFIRMED",
            PredicateObservation(True, time.monotonic(), token.epoch),
        )
        self._feedback("CHANGE_MAP", "calling map_server change_map")
        try:
            response = self.services.call(
                self.change_map_name,
                self.change_map,
                str((self.config_root / self.floors[target_floor].map_yaml).resolve()),
            )
        except ServiceCallError as error:
            self._fail("change_map failed: {}".format(error))
            return
        if response.result != LoadMapResponse.RESULT_SUCCESS:
            self._fail("change_map result {}".format(response.result))
            return
        self._feedback("MAP_CONFIRM", "waiting for newer matching map publication")
        ok, reason = self._wait(
            lambda: self.runtime.map_guard is not None and not self.runtime.map_guard.armed,
            time.monotonic() + self.transaction_timeout,
        )
        if not ok:
            self._fail(reason, self.server.is_preempt_requested())
            return
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
        self._feedback("INITIALPOSE", "publishing stored landing hypothesis")
        self.runtime.arm_localization(pose_stamp.to_nsec())
        self.initialpose_publisher.publish(pose)
        localization_deadline = time.monotonic() + self.transaction_timeout
        pose_marker = pose_stamp.to_nsec()
        try:
            for _ in range(3):
                self.services.call(self.nomotion_name, self.nomotion_update)
                response_marker = rospy.Time.now().to_nsec()
                pose_fence = max(pose_marker, response_marker)
                ok, reason = self._wait(
                    lambda marker=pose_fence: self.runtime.has_pose_newer_than(marker),
                    localization_deadline,
                )
                if not ok:
                    self._fail(reason, self.server.is_preempt_requested())
                    return
                pose_marker = self.runtime.pose_high_water_ns(pose_fence)
        except ServiceCallError as error:
            self._fail("request_nomotion_update failed: {}".format(error))
            return
        self._feedback("AMCL_READY", "waiting for covariance, samples, scan, odom, and TF")
        def localized() -> bool:
            evidence = self.runtime.localization_evidence()
            return evidence is not None and evidence.ready

        ok, reason = self._wait(localized, localization_deadline)
        if not ok:
            self._fail(reason, self.server.is_preempt_requested())
            return
        self._feedback("CLEAR_COSTMAPS", "clearing navigation costmaps")
        marker = self.runtime.costmap_sequence
        try:
            self.services.call(self.clear_costmaps_name, self.clear_costmaps)
        except ServiceCallError as error:
            self._fail("clear_costmaps failed: {}".format(error))
            return
        self._feedback("COSTMAP_READY", "waiting for target global costmap metadata")
        ok, reason = self._wait(
            lambda: self._target_metadata_matches(target, marker),
            time.monotonic() + self.transaction_timeout,
        )
        if not ok:
            self._fail(reason, self.server.is_preempt_requested())
            return
        ok, reason = self._wait_for_policy_ready(token, target, marker)
        if not ok:
            self._fail(reason, self.server.is_preempt_requested())
            return
        self.runtime.finish_transition(FloorState.READY, "floor transition complete", target_floor)
        self.server.set_succeeded(self._result(FloorTransitionResult.OK, ""))


def run() -> int:
    """Construct the operational node after the sole entrypoint initializes rospy."""
    MultifloorManagerNode()
    rospy.spin()
    return 0
