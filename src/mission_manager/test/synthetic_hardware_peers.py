"""Synthetic external ROS peers backed by production configuration."""

from __future__ import annotations

import ipaddress
import math
import os
from pathlib import Path
import threading
from urllib.parse import urlparse

import actionlib
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from multifloor_manager.configuration import load_multifloor_configuration
from multifloor_manager.readiness import DEFAULT_READINESS_POLICY
from nav_msgs.msg import OccupancyGrid, Odometry
import rospkg
import rospy
from sensor_msgs.msg import LaserScan
from stair_supervisor.configuration import StairProfile, load_stair_configuration
from stair_supervisor.msg import StairTraversalActionFeedback, StairTraversalActionGoal
from stair_supervisor.stair_evidence import Phase
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from tf2_msgs.msg import TFMessage


class SyntheticIsolationError(RuntimeError):
    """Report a synthetic peer that could escape the local ROS graph."""


class ProductionContractError(RuntimeError):
    """Report production configuration missing a required test contract."""


def require_loopback_ros() -> None:
    """Reject non-loopback ROS master and node advertisement addresses."""
    hosts = (
        urlparse(os.environ.get("ROS_MASTER_URI", "")).hostname,
        os.environ.get("ROS_IP"),
        os.environ.get("ROS_HOSTNAME"),
    )
    for host in (value for value in hosts if value):
        if host == "localhost":
            continue
        try:
            loopback = ipaddress.ip_address(host).is_loopback
        except ValueError:
            loopback = False
        if not loopback:
            raise SyntheticIsolationError(
                "synthetic hardware peers require loopback ROS addressing"
            )


class SyntheticHardwarePeers:
    """Own mutable fake hardware state while preserving production ROS contracts."""

    def __init__(self, stair_config_dir: str | Path | None = None) -> None:
        packages = rospkg.RosPack()
        multifloor_root = Path(packages.get_path("multifloor_manager")) / "config"
        if stair_config_dir is None:
            stair_root = Path(packages.get_path("stair_supervisor")) / "config"
        else:
            stair_root = Path(stair_config_dir).resolve()
        multifloor = load_multifloor_configuration(multifloor_root)
        self.profiles = {
            item.id: item for item in load_stair_configuration(stair_root).profiles
        }
        stair = next(item for item in multifloor.stairs if item.id == "stair_3f_4f")
        endpoint = stair.endpoint_for("4F")
        if endpoint is None:
            raise ProductionContractError("stair_3f_4f has no 4F endpoint")
        self.target_tag = endpoint.expected_tag_ids[0]
        self.lock = threading.RLock()
        self.latest_map: OccupancyGrid | None = None
        self.initialpose: PoseWithCovarianceStamped | None = None
        self.profile: StairProfile | None = None
        self.phase: Phase | None = None
        self.phase_progress = 0.0
        self.x_m = self.y_m = self.yaw_rad = 0.0
        self.navigation_goals: list[MoveBaseGoal] = []
        self.navigation_pose = None
        self.websocket_frames: list[str] = []
        self.map_subscriber = rospy.Subscriber(
            "/map", OccupancyGrid, self._accept_map, queue_size=1
        )
        self.pose_subscriber = rospy.Subscriber(
            "/initialpose",
            PoseWithCovarianceStamped,
            self._accept_initialpose,
            queue_size=1,
        )
        self.stair_goal_subscriber = rospy.Subscriber(
            "/stair_traversal/goal",
            StairTraversalActionGoal,
            self._accept_stair_goal,
            queue_size=1,
        )
        self.stair_feedback_subscriber = rospy.Subscriber(
            "/stair_traversal/feedback",
            StairTraversalActionFeedback,
            self._accept_stair_feedback,
            queue_size=10,
        )
        self.tx_subscriber = rospy.Subscriber(
            "/stair_supervisor/websocket_tx",
            String,
            lambda message: self.websocket_frames.append(message.data),
            queue_size=1000,
        )
        self.tag_publisher = rospy.Publisher(
            "/tag_detections", AprilTagDetectionArray, queue_size=10
        )
        self.pose_publisher = rospy.Publisher(
            "/amcl_pose", PoseWithCovarianceStamped, queue_size=10
        )
        self.scan_publisher = rospy.Publisher("/scan", LaserScan, queue_size=10)
        self.odom_publisher = rospy.Publisher(
            "/tron/wheel_odom_raw", Odometry, queue_size=10
        )
        self.tf_publisher = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.costmap_publisher = rospy.Publisher(
            "/move_base/global_costmap/costmap",
            OccupancyGrid,
            queue_size=2,
            latch=True,
        )
        self.move_base = actionlib.SimpleActionServer(
            "/move_base", MoveBaseAction, execute_cb=self._navigate, auto_start=False
        )
        self.nomotion = rospy.Service(
            "/request_nomotion_update", Empty, self._nomotion_update
        )
        self.clear_costmaps = rospy.Service(
            "/move_base/clear_costmaps", Empty, self._clear_costmaps
        )
        self.move_base.start()
        self.timer = rospy.Timer(rospy.Duration(0.02), self._publish_sensors)

    def shutdown(self) -> None:
        self.timer.shutdown()

    def _accept_map(self, message: OccupancyGrid) -> None:
        self.latest_map = message

    def _accept_initialpose(self, message: PoseWithCovarianceStamped) -> None:
        self.initialpose = message

    def _accept_stair_goal(self, message: StairTraversalActionGoal) -> None:
        with self.lock:
            self.profile = self.profiles[message.goal.stair_id]
            self.phase = None
            self.phase_progress = 0.0

    def _accept_stair_feedback(self, message: StairTraversalActionFeedback) -> None:
        phase = Phase(message.feedback.phase)
        with self.lock:
            if phase is not self.phase:
                self.phase = phase
                self.phase_progress = 0.0

    def _navigate(self, goal: MoveBaseGoal) -> None:
        self.navigation_goals.append(goal)
        self.navigation_pose = goal.target_pose
        self.move_base.set_succeeded(MoveBaseResult())

    def _nomotion_update(self, _request) -> EmptyResponse:
        rospy.Timer(rospy.Duration(0.03), self._publish_pose, oneshot=True)
        return EmptyResponse()

    def _clear_costmaps(self, _request) -> EmptyResponse:
        rospy.Timer(rospy.Duration(0.03), self._publish_costmap, oneshot=True)
        return EmptyResponse()

    def _publish_pose(self, _event) -> None:
        source = self.initialpose
        if source is None:
            return
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = source.header.frame_id
        message.pose.pose = source.pose.pose
        message.pose.covariance[0] = DEFAULT_READINESS_POLICY.max_covariance_x * 0.5
        message.pose.covariance[7] = DEFAULT_READINESS_POLICY.max_covariance_y * 0.5
        message.pose.covariance[35] = DEFAULT_READINESS_POLICY.max_covariance_yaw * 0.5
        self.pose_publisher.publish(message)

    def _publish_costmap(self, _event) -> None:
        if self.latest_map is not None:
            self.costmap_publisher.publish(self.latest_map)

    def _motion_step(self) -> tuple[float, float]:
        with self.lock:
            profile, phase = self.profile, self.phase
            if profile is None or phase is None:
                return 0.0, 0.0
            targets = {
                Phase.ALIGN: profile.alignment_yaw_rad,
                Phase.FORWARD_SEGMENT_1: profile.flight_1_distance_m,
                Phase.TURN_TO_NEXT_FLIGHT: profile.landing_turn_yaw_rad,
                Phase.FORWARD_SEGMENT_2: profile.flight_2_distance_m,
            }
            target = targets.get(phase)
            if target is None or abs(self.phase_progress) >= abs(target):
                return 0.0, 0.0
            turning = phase in (Phase.ALIGN, Phase.TURN_TO_NEXT_FLIGHT)
            limit = (
                profile.max_yaw_step_rad * 0.4
                if turning
                else profile.max_odom_step_m * 0.4
            )
            step = math.copysign(
                min(limit, abs(target) - abs(self.phase_progress)), target
            )
            self.phase_progress += step
            return (0.0, step) if turning else (step, 0.0)

    def _publish_sensors(self, _event) -> None:
        distance, yaw_step = self._motion_step()
        self.x_m += distance * math.cos(self.yaw_rad)
        self.y_m += distance * math.sin(self.yaw_rad)
        self.yaw_rad += yaw_step
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_Link"
        odom.pose.pose.position.x = self.x_m
        odom.pose.pose.position.y = self.y_m
        odom.pose.pose.orientation.z = math.sin(self.yaw_rad / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw_rad / 2.0)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_Link"
        odom_to_base.transform.translation.x = self.x_m
        odom_to_base.transform.translation.y = self.y_m
        odom_to_base.transform.rotation.z = math.sin(self.yaw_rad / 2.0)
        odom_to_base.transform.rotation.w = math.cos(self.yaw_rad / 2.0)
        self.tf_publisher.publish(TFMessage((map_to_odom, odom_to_base)))
        self.odom_publisher.publish(odom)
        navigation_pose = self.navigation_pose
        if navigation_pose is not None:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = navigation_pose.header.frame_id
            pose.pose.pose = navigation_pose.pose
            pose.pose.covariance[0] = DEFAULT_READINESS_POLICY.max_covariance_x * 0.5
            pose.pose.covariance[7] = DEFAULT_READINESS_POLICY.max_covariance_y * 0.5
            pose.pose.covariance[35] = DEFAULT_READINESS_POLICY.max_covariance_yaw * 0.5
            self.pose_publisher.publish(pose)
        rospy.sleep(0.002)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_Link"
        self.scan_publisher.publish(scan)
        tags = AprilTagDetectionArray()
        tags.header.stamp = stamp
        tags.header.frame_id = "camera_link"
        tags.detections = (AprilTagDetection(id=(self.target_tag,)),)
        self.tag_publisher.publish(tags)
