"""Thread-safe ROS state snapshots and the navigation-to-stair stop barrier."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import math
from collections import deque

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from multifloor_manager.msg import FloorState
import rospy
from stair_supervisor.msg import SupervisorState

from mission_manager.stair_entry_gate import (
    StairEntryCheck,
    StairEntryDecision,
    StairEntryEvidence,
    StairEntryFence,
    StairEntryPolicy,
    StairEntryPose,
    evaluate_stair_entry, entry_motion_window,
)


@dataclass(frozen=True)
class StateHealth:
    __slots__ = ("healthy", "reason")
    healthy: bool
    reason: str


class RosStateMonitor:
    """Own latest floor, supervisor, and odometry values received by this node."""

    def __init__(
        self,
        freshness_sec: float,
        stationary_speed: float,
        barrier_timeout: float,
        entry_policy: StairEntryPolicy,
    ) -> None:
        self._freshness_sec = freshness_sec
        self._stationary_speed = stationary_speed
        self._barrier_timeout = barrier_timeout
        self._entry_policy = entry_policy
        self._lidar_handoff = bool(rospy.get_param("~stair_lidar_handoff", False))
        self._entry_motion = deque(maxlen=256)
        self._entry_motion_stamp = None
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._floor: FloorState | None = None
        self._supervisor: SupervisorState | None = None
        self._odom: Odometry | None = None
        self._pose: PoseWithCovarianceStamped | None = None
        self._pose_floor_key: tuple[str, int] | None = None
        self._pose_sequence = 0
        self._pose_received = 0.0
        self._floor_received = 0.0
        self._supervisor_received = 0.0
        self._odom_received = 0.0
        self._subscribers = (
            rospy.Subscriber(
                rospy.get_param("~floor_state_topic", "/multifloor/floor_state"),
                FloorState,
                self._accept_floor,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param("~supervisor_state_topic", "/stair_supervisor/state"),
                SupervisorState,
                self._accept_supervisor,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param("~odom_topic", "/tron/wheel_odom_raw"),
                Odometry,
                self._accept_odom,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param("~amcl_pose_topic", "/amcl_pose"),
                PoseWithCovarianceStamped,
                self._accept_pose,
                queue_size=10,
            ),
        )

    def _accept_floor(self, message: FloorState) -> None:
        with self._condition:
            previous_key = None if self._floor is None else (
                self._floor.floor_id,
                int(self._floor.map_generation),
            )
            self._floor = message
            current_key = (message.floor_id, int(message.map_generation))
            if previous_key is not None and previous_key != current_key:
                self._pose = None
                self._pose_floor_key = None
            self._floor_received = time.monotonic()
            self._condition.notify_all()

    def _accept_supervisor(self, message: SupervisorState) -> None:
        with self._lock:
            self._supervisor = message
            self._supervisor_received = time.monotonic()

    def _accept_odom(self, message: Odometry) -> None:
        with self._condition:
            self._odom = message
            self._odom_received = time.monotonic()
            if self._lidar_handoff:
                stamp = message.header.stamp.to_sec()
                age = rospy.Time.now().to_sec()-stamp
                if (self._entry_motion_stamp is not None and stamp <= self._entry_motion_stamp
                        or not 0 <= age <= .30):
                    self._entry_motion.clear()
                else:
                    self._entry_motion.append((self._odom_received-age, message.twist.twist.linear.x,
                                               message.twist.twist.angular.z))
                self._entry_motion_stamp = stamp
            self._condition.notify_all()

    def _accept_pose(self, message: PoseWithCovarianceStamped) -> None:
        with self._condition:
            floor = self._floor
            self._pose = message
            self._pose_floor_key = None if floor is None else (
                floor.floor_id,
                int(floor.map_generation),
            )
            self._pose_sequence += 1
            self._pose_received = time.monotonic()
            self._condition.notify_all()

    def floor_state(self) -> FloorState:
        with self._lock:
            if self._floor is None:
                return FloorState(state=FloorState.UNKNOWN, detail="no floor state received")
            return self._floor

    def hold_pose(self):
        """Fresh map-associated AMCL pose for optional idle NAV restoration."""
        with self._lock:
            floor,pose=self._floor,self._pose
            if floor is None or pose is None or floor.state != FloorState.READY:
                return None
            if (time.monotonic()-self._floor_received>self._freshness_sec or
                    time.monotonic()-self._supervisor_received>self._freshness_sec or
                    self._supervisor is None or self._supervisor.state!=SupervisorState.NAV):
                return None
            key=(floor.floor_id,int(floor.map_generation))
            if self._pose_floor_key != key or time.monotonic()-self._pose_received>self._freshness_sec:
                return None
            age=rospy.Time.now().to_sec()-pose.header.stamp.to_sec()
            if pose.header.frame_id!='map' or not 0<=age<=self._freshness_sec:
                return None
            p,q=pose.pose.pose.position,pose.pose.pose.orientation
            values=(p.x,p.y,q.x,q.y,q.z,q.w)
            cov=pose.pose.covariance
            if not all(map(math.isfinite,values)) or abs(sum(v*v for v in values[2:])-1)>.01:
                return None
            limits=(self._entry_policy.max_covariance_x,self._entry_policy.max_covariance_y,self._entry_policy.max_covariance_yaw)
            if any(not math.isfinite(cov[i]) or not 0<=cov[i]<=bound for i,bound in zip((0,7,35),limits)):
                return None
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            return (*key,p.x,p.y,yaw)

    def wait_for_floor(self, floor_id: str, generation: int, timeout: float) -> bool:
        """Wait until this process observes the floor action's READY generation."""
        deadline = time.monotonic() + timeout
        with self._condition:
            while not (
                self._floor is not None
                and self._floor.state == FloorState.READY
                and self._floor.floor_id == floor_id
                and int(self._floor.map_generation) == generation
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            return True

    def supervisor_state(self) -> SupervisorState:
        with self._lock:
            if self._supervisor is None:
                return SupervisorState(state=SupervisorState.DISARMED, connected=False, detail="no supervisor state received")
            return self._supervisor

    def health(self) -> StateHealth:
        now = time.monotonic()
        with self._lock:
            floor = self._floor
            supervisor = self._supervisor
            if floor is None or supervisor is None:
                return StateHealth(False, "mission state inputs are unavailable")
            if now - self._floor_received > self._freshness_sec:
                return StateHealth(False, "floor state is stale")
            if now - self._supervisor_received > self._freshness_sec:
                return StateHealth(False, "supervisor state is stale")
            if floor.state == FloorState.FAULT:
                return StateHealth(False, "floor manager is in FAULT")
            if supervisor.state == SupervisorState.FAULT or not supervisor.connected:
                return StateHealth(False, "stair supervisor communication is unavailable")
            return StateHealth(True, "")

    def stop_and_confirm_stationary(self) -> bool:
        deadline = time.monotonic() + self._barrier_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = time.monotonic()
            with self._lock:
                odom = self._odom
                supervisor = self._supervisor
                fresh = odom is not None and now - self._odom_received <= self._freshness_sec
                owns_nav = (
                    supervisor is not None
                    and supervisor.connected
                    and supervisor.state == SupervisorState.NAV
                )
                stationary = (
                    fresh
                    and abs(odom.twist.twist.linear.x) <= self._stationary_speed
                    and abs(odom.twist.twist.angular.z) <= self._stationary_speed
                )
                if self._lidar_handoff:
                    motion = entry_motion_window(self._entry_motion, now)
                    stationary = fresh and motion is not None and motion[0] <= self._entry_policy.max_linear_speed and motion[1] <= self._entry_policy.max_angular_speed
            if owns_nav and stationary:
                return True
            rospy.sleep(0.02)
        return False

    def begin_stair_entry(self) -> StairEntryFence:
        """Fence out localization evidence produced before the handoff starts."""
        with self._lock:
            floor = self._floor
            if floor is None or floor.state != FloorState.READY:
                return StairEntryFence("", -1, rospy.Time.now().to_sec(), time.monotonic(), self._pose_sequence)
            return StairEntryFence(
                floor.floor_id,
                int(floor.map_generation),
                rospy.Time.now().to_sec(),
                time.monotonic(),
                self._pose_sequence,
            )

    def stair_entry_decision(
        self,
        fence: StairEntryFence,
        expected: StairEntryPose,
    ) -> StairEntryDecision:
        """Evaluate a consistent snapshot of current AMCL and wheel-odometry evidence."""
        with self._lock:
            pose = self._pose
            odom = self._odom
            floor_key = self._pose_floor_key
            if pose is None or odom is None or floor_key is None:
                evidence = None
            else:
                orientation = pose.pose.pose.orientation
                position = pose.pose.pose.position
                covariance = pose.pose.covariance
                motion = (entry_motion_window(self._entry_motion, time.monotonic())
                          if self._lidar_handoff else (odom.twist.twist.linear.x, odom.twist.twist.angular.z))
                if motion is None:
                    return StairEntryDecision(False, "waiting for fresh bounded entry motion")
                evidence = StairEntryEvidence(
                    pose.header.frame_id,
                    floor_key[0],
                    floor_key[1],
                    pose.header.stamp.to_sec(),
                    self._pose_received,
                    self._pose_sequence,
                    position.x,
                    position.y,
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                    covariance[0],
                    covariance[7],
                    covariance[35],
                    self._odom_received,
                    motion[0],
                    motion[1],
                )
        return evaluate_stair_entry(
            StairEntryCheck(
                self._entry_policy,
                fence,
                expected,
                evidence,
                rospy.Time.now().to_sec(),
                time.monotonic(),
            )
        )

    def wait_for_stair_entry(
        self,
        fence: StairEntryFence,
        expected: StairEntryPose,
    ) -> StairEntryDecision:
        """Wait boundedly for post-fence entry evidence and return the final reason."""
        deadline = time.monotonic() + self._barrier_timeout
        decision = self.stair_entry_decision(fence, expected)
        # The mission is still active, so the idle AMCL refresher is paused.
        # Request fresh scans explicitly while stopped instead of waiting for
        # motion-triggered AMCL samples that may never arrive at this boundary.
        from std_srvs.srv import Empty
        from multifloor_manager.ros_services import BoundedServiceCaller, ServiceCallError
        name = rospy.get_param("~nomotion_service", "/request_nomotion_update")
        refresh = rospy.ServiceProxy(name, Empty)
        caller = BoundedServiceCaller(0.2)
        next_refresh = 0.0
        while not decision.accepted and not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return decision
            if time.monotonic() >= next_refresh and remaining >= 0.4:
                try:
                    caller.call(name, refresh)
                except ServiceCallError as error:
                    rospy.logwarn("Stair entry localization refresh failed: %s", error)
                    # A transient refresh RPC must not turn a recoverable
                    # observation gap into a terminal mission failure. The
                    # same bounded deadline and fresh-evidence checks remain.
                    pass
                next_refresh = time.monotonic() + 0.2
            with self._condition:
                self._condition.wait(min(max(0, deadline-time.monotonic()), 0.02))
            decision = self.stair_entry_decision(fence, expected)
        return decision
