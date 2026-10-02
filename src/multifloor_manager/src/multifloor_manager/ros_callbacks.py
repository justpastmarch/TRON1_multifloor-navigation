"""Epoch-fenced ROS evidence callbacks for the multifloor runtime."""

from __future__ import annotations

import math

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rospy
from sensor_msgs.msg import LaserScan

from multifloor_manager.map_evidence import (
    MapBoundaryError,
    MapGeneration,
    MapGenerationState,
    MapIdentity,
    MapOrigin,
    Nanoseconds as MapNanoseconds,
    evaluate_map_observation,
    fingerprint_occupancy_grid,
)
from multifloor_manager.msg import FloorState
from multifloor_manager.readiness import Nanoseconds, observe_odometry, observe_pose, observe_scan
from multifloor_manager.tag_evidence import (
    Nanoseconds as TagNanoseconds,
    TagBoundaryError,
    evaluate_tag_observation,
    parse_apriltag_detection_array,
)


class RosEvidenceCallbacks:
    """Parse outside the lock and commit only to the same active epoch."""

    def map_callback(self, message: OccupancyGrid) -> None:
        with self.condition:
            epoch = self.active_epoch
        try:
            fingerprint = fingerprint_occupancy_grid(
                message,
                MapNanoseconds(rospy.Time.now().to_nsec()),
            )
        except MapBoundaryError as error:
            rospy.logwarn("map observation rejected: %s", error)
            return
        with self.condition:
            if epoch is None:
                if self.active_epoch is not None or self.state != FloorState.UNKNOWN:
                    return
                if self.map_state.current_fingerprint is None and fingerprint.identity() == self.initial_identity:
                    self.map_state = MapGenerationState(self.map_state.generation, fingerprint)
                    if self.require_initial_localization:
                        self.publish_floor(FloorState.UNKNOWN, "initial map observed; waiting for localization", self.initial_floor)
                    else:
                        self.publish_floor(FloorState.READY, "initial map observed", self.initial_floor)
            elif self.active_epoch == epoch and self.map_guard is not None:
                result = evaluate_map_observation(self.map_guard, fingerprint)
                self.map_guard = result.guard
                self.map_state = result.guard.state
                self._publish_predicates(result.evidence.predicates())
            self.condition.notify_all()

    def tag_callback(self, message: AprilTagDetectionArray) -> None:
        with self.condition:
            epoch, state, context = self.active_epoch, self.tag_state, self.tag_context
        if epoch is None or state is None or context is None:
            return
        try:
            observation = parse_apriltag_detection_array(
                message,
                TagNanoseconds(rospy.Time.now().to_nsec()),
            )
        except TagBoundaryError as error:
            rospy.logwarn("tag observation rejected: %s", error)
            return
        with self.condition:
            if self.active_epoch != epoch or self.tag_state is not state:
                return
            result = evaluate_tag_observation(state, observation, context)
            self.tag_state = result.state
            self.tag_accepted = result.evidence.accepted
            self.wrong_floor_tag = result.evidence.wrong_floor_vote
            self._publish_predicates(result.evidence.predicates())
            self.condition.notify_all()

    def pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        with self.condition:
            epoch, state = self.active_epoch, self.localization
        if epoch is None or state is None:
            return
        covariance = message.pose.covariance
        next_state = observe_pose(
            state,
            Nanoseconds(message.header.stamp.to_nsec()),
            Nanoseconds(rospy.Time.now().to_nsec()),
            (covariance[0], covariance[7], covariance[35]),
        )
        with self.condition:
            if self.active_epoch != epoch or self.localization is not state:
                return
            self.localization = next_state
            self.condition.notify_all()

    def scan_callback(self, message: LaserScan) -> None:
        with self.condition:
            epoch = self.active_epoch
            if epoch is None or self.localization is None:
                return
        stamp = message.header.stamp
        transform_valid = self.tf_buffer.can_transform(
            "map", "odom", stamp, rospy.Duration(0.0)
        ) and self.tf_buffer.can_transform("odom", "base_Link", stamp, rospy.Duration(0.0))
        with self.condition:
            if self.active_epoch != epoch or self.localization is None:
                return
            self.localization = observe_scan(
                self.localization,
                Nanoseconds(stamp.to_nsec()),
                Nanoseconds(rospy.Time.now().to_nsec()),
                transform_valid,
            )
            self.condition.notify_all()

    def odom_callback(self, message: Odometry) -> None:
        with self.condition:
            epoch = self.active_epoch
            if epoch is None or self.localization is None:
                return
            linear = message.twist.twist.linear
            angular = message.twist.twist.angular
            self.localization = observe_odometry(
                self.localization,
                Nanoseconds(message.header.stamp.to_nsec()),
                Nanoseconds(rospy.Time.now().to_nsec()),
                math.hypot(linear.x, linear.y),
                angular.z,
            )
            self.condition.notify_all()

    def costmap_callback(self, message: OccupancyGrid) -> None:
        with self.condition:
            epoch = self.active_epoch
        if epoch is None:
            return
        origin = message.info.origin
        identity = MapIdentity(
            message.header.frame_id,
            message.info.width,
            message.info.height,
            message.info.resolution,
            MapOrigin(
                origin.position.x, origin.position.y, origin.position.z,
                origin.orientation.x, origin.orientation.y,
                origin.orientation.z, origin.orientation.w,
            ),
            "",
        )
        with self.condition:
            if self.active_epoch != epoch:
                return
            self.costmap_sequence += 1
            self.costmap_identity = identity
            self.condition.notify_all()
