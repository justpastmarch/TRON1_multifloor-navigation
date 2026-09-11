"""Thread-safe ROS state snapshots and the navigation-to-stair stop barrier."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from nav_msgs.msg import Odometry
from multifloor_manager.msg import FloorState
import rospy
from stair_supervisor.msg import SupervisorState


@dataclass(frozen=True)
class StateHealth:
    __slots__ = ("healthy", "reason")
    healthy: bool
    reason: str


class RosStateMonitor:
    """Own latest floor, supervisor, and odometry values received by this node."""

    def __init__(self, freshness_sec: float, stationary_speed: float, barrier_timeout: float) -> None:
        self._freshness_sec = freshness_sec
        self._stationary_speed = stationary_speed
        self._barrier_timeout = barrier_timeout
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._floor: FloorState | None = None
        self._supervisor: SupervisorState | None = None
        self._odom: Odometry | None = None
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
        )

    def _accept_floor(self, message: FloorState) -> None:
        with self._condition:
            self._floor = message
            self._floor_received = time.monotonic()
            self._condition.notify_all()

    def _accept_supervisor(self, message: SupervisorState) -> None:
        with self._lock:
            self._supervisor = message
            self._supervisor_received = time.monotonic()

    def _accept_odom(self, message: Odometry) -> None:
        with self._lock:
            self._odom = message
            self._odom_received = time.monotonic()

    def floor_state(self) -> FloorState:
        with self._lock:
            if self._floor is None:
                return FloorState(state=FloorState.UNKNOWN, detail="no floor state received")
            return self._floor

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
            if owns_nav and stationary:
                return True
            rospy.sleep(0.02)
        return False
