#!/usr/bin/env python3
"""Test-only child action peers for end-to-end Mission.action scenarios."""

from __future__ import annotations

import math
from pathlib import Path
import threading

import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from multifloor_manager.msg import (
    FloorState,
    FloorTransitionAction,
    FloorTransitionResult,
)
from multifloor_manager.readiness import DEFAULT_READINESS_POLICY
from nav_msgs.msg import Odometry
import rospy
from stair_supervisor.msg import (
    StairTraversalAction,
    StairTraversalGoal,
    StairTraversalResult,
    SupervisorState,
)
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
import yaml


class MockMissionChildren:
    """Publish healthy state and script child terminal outcomes from one ROS param."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._execution_lock = threading.RLock()
        fixture_root = Path(rospy.get_param("~fixture_config_root", "test/fixtures/building_valid"))
        self._locations = self._load_locations(fixture_root)
        self._stairs = self._load_stairs(fixture_root)
        self._floor_id = rospy.get_param("~initial_floor", "3F")
        self._generation = 1
        self._nav_attempt = 0
        self._current_location_id = rospy.get_param("~initial_location", "home_3f")
        self._floor_pub = rospy.Publisher(
            "/multifloor/floor_state", FloorState, queue_size=1, latch=True
        )
        self._supervisor_pub = rospy.Publisher(
            "/stair_supervisor/state", SupervisorState, queue_size=1, latch=True
        )
        self._odom_pub = rospy.Publisher(
            "/tron/wheel_odom_raw", Odometry, queue_size=1, latch=True
        )
        self._amcl_pub = rospy.Publisher(
            "/amcl_pose", PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        self._clear = rospy.Service("/move_base/clear_costmaps", Empty, self._clear_costmaps)
        self._nav = actionlib.SimpleActionServer(
            "/move_base", MoveBaseAction, execute_cb=self._execute_nav, auto_start=False
        )
        self._stair = actionlib.SimpleActionServer(
            "/stair_traversal",
            StairTraversalAction,
            execute_cb=self._execute_stair,
            auto_start=False,
        )
        self._floor = actionlib.SimpleActionServer(
            "/multifloor/floor_transition",
            FloorTransitionAction,
            execute_cb=self._execute_floor,
            auto_start=False,
        )
        self._nav.start()
        self._stair.start()
        self._floor.start()
        self._synchronize_service = rospy.Service(
            "/mission_test/synchronize",
            Trigger,
            self._synchronize,
        )
        self._timer = rospy.Timer(rospy.Duration(0.05), self._publish_state)
        rospy.on_shutdown(self.shutdown)

    @staticmethod
    def _load_locations(root: Path) -> dict[str, dict[str, float]]:
        path = root / "locations.yaml"
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        return {
            item["id"]: {
                "x": float(item["x"]),
                "y": float(item["y"]),
                "yaw": float(item["yaw"]),
                "floor_id": str(item["floor_id"]),
            }
            for item in document["locations"]
        }

    @staticmethod
    def _load_stairs(root: Path) -> dict[str, dict]:
        path = root / "stairs.yaml"
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        stairs: dict[str, dict] = {}
        for item in document["stairs"]:
            stairs[item["id"]] = {
                "from_floor": item["from_floor"],
                "to_floor": item["to_floor"],
                "up_profile_id": item.get("up_profile_id"),
                "down_profile_id": item.get("down_profile_id"),
                "endpoints": {
                    floor: endpoint
                    for floor, endpoint in item["endpoints"].items()
                },
            }
        return stairs

    def shutdown(self) -> None:
        """Stop fixture timers before rospy closes their publishers."""
        self._timer.shutdown()
        for server in (self._nav, self._stair, self._floor):
            status_timer = server.action_server.status_timer
            if status_timer is not None:
                status_timer.shutdown()

    @staticmethod
    def _scenario() -> str:
        return rospy.get_param("/mission_test/scenario", "default")

    @staticmethod
    def _increment(name: str) -> int:
        value = int(rospy.get_param(name, 0)) + 1
        rospy.set_param(name, value)
        return value

    @staticmethod
    def _clear_costmaps(_request: Empty.Request) -> EmptyResponse:
        MockMissionChildren._increment("/mission_test/clear_count")
        return EmptyResponse()

    def _synchronize(self, _request: Trigger.Request) -> TriggerResponse:
        """Reset counters only after every prior child callback has returned."""
        with self._execution_lock:
            for counter in ("nav_count", "stair_count", "floor_count", "clear_count"):
                rospy.set_param("/mission_test/{}".format(counter), 0)
            self._publish_state(None, force=True)
            scenario = self._scenario()
            with self._lock:
                floor_id = self._floor_id
            active = self._nav.is_active() or self._stair.is_active() or self._floor.is_active()
            return TriggerResponse(
                success=not active,
                message="scenario={} floor={}".format(scenario, floor_id),
            )

    def _publish_state(
        self,
        _event: rospy.timer.TimerEvent | None,
        force: bool = False,
    ) -> None:
        if self._scenario() == "stale_state" and not force:
            return
        with self._lock:
            floor_id = self._floor_id
            generation = self._generation
        self._floor_pub.publish(
            FloorState(
                floor_id=floor_id,
                map_generation=generation,
                state=FloorState.READY,
                detail="fixture ready",
            )
        )
        connected = self._scenario() != "disconnected"
        self._supervisor_pub.publish(
            SupervisorState(
                state=SupervisorState.NAV if connected else SupervisorState.FAULT,
                connected=connected,
                detail="fixture supervisor",
                ownership_epoch=1,
            )
        )
        self._odom_pub.publish(Odometry())
        self._publish_amcl_pose()

    def _publish_amcl_pose(self) -> None:
        location = self._locations.get(self._current_location_id)
        if location is None:
            return
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = location["x"]
        pose.pose.pose.position.y = location["y"]
        half_yaw = location["yaw"] / 2.0
        z = math.sin(half_yaw)
        w = math.cos(half_yaw)
        norm = math.hypot(z, w)
        pose.pose.pose.orientation.z = z / norm
        pose.pose.pose.orientation.w = w / norm
        pose.pose.covariance[0] = DEFAULT_READINESS_POLICY.max_covariance_x * 0.5
        pose.pose.covariance[7] = DEFAULT_READINESS_POLICY.max_covariance_y * 0.5
        pose.pose.covariance[35] = DEFAULT_READINESS_POLICY.max_covariance_yaw * 0.5
        self._amcl_pub.publish(pose)

    def _set_current_location(self, location_id: str) -> None:
        if location_id in self._locations:
            with self._lock:
                self._current_location_id = location_id
                self._floor_id = self._locations[location_id]["floor_id"]

    def _location_id_from_nav_goal(self, goal: MoveBaseGoal) -> str | None:
        x = goal.target_pose.pose.position.x
        y = goal.target_pose.pose.position.y
        q = goal.target_pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        for location_id, location in self._locations.items():
            yaw_error = math.atan2(math.sin(location["yaw"] - yaw), math.cos(location["yaw"] - yaw))
            if (location["floor_id"] == self._floor_id and abs(location["x"] - x) < 1e-3 and
                    abs(location["y"] - y) < 1e-3 and abs(yaw_error) < 1e-3):
                return location_id
        return None

    def _execute_nav(self, goal: MoveBaseGoal) -> None:
        with self._execution_lock:
            attempt = self._increment("/mission_test/nav_count")
            scenario = self._scenario()
            if scenario == "hold":
                while not self._nav.is_preempt_requested() and not rospy.is_shutdown():
                    rospy.sleep(0.01)
                self._nav.set_preempted(MoveBaseResult())
                return
            if scenario == "nav_retry" and attempt == 1:
                self._nav.set_aborted(MoveBaseResult())
                return
            if scenario == "nav_failure":
                self._nav.set_aborted(MoveBaseResult())
                return
            self._set_current_location(self._location_id_from_nav_goal(goal) or self._current_location_id)
            self._nav.set_succeeded(MoveBaseResult())

    def _execute_stair(self, goal: StairTraversalAction.Goal) -> None:
        with self._execution_lock:
            self._increment("/mission_test/stair_count")
            scenario = self._scenario()
            if scenario in ("stair_failure", "odometry_jump"):
                reasons = {
                    "stair_failure": "injected stair failure",
                    "odometry_jump": "injected odometry jump",
                }
                self._stair.set_aborted(
                    StairTraversalResult(
                        result_code=StairTraversalResult.STAIR_FAILED,
                        reason=reasons[scenario],
                    )
                )
                return
            if scenario in ("communication_failure", "websocket_loss"):
                self._stair.set_aborted(
                    StairTraversalResult(
                        result_code=StairTraversalResult.COMMUNICATION_LOST,
                        reason=(
                            "injected WebSocket loss"
                            if scenario == "websocket_loss"
                            else "injected communication loss"
                        ),
                    )
                )
                return
            self._stair.set_succeeded(
                StairTraversalResult(result_code=StairTraversalResult.OK, reason="")
            )

    def _execute_floor(self, goal: FloorTransitionAction.Goal) -> None:
        with self._execution_lock:
            self._increment("/mission_test/floor_count")
            if self._scenario() == "incoherent_floor":
                self._floor.set_succeeded(
                    FloorTransitionResult(
                        result_code=FloorTransitionResult.LOCALIZATION_FAILED,
                        floor_id=self._floor_id,
                        map_generation=self._generation,
                        reason="injected misleading terminal result",
                    )
                )
                return
            scenario = self._scenario()
            if scenario in ("localization_failure", "stale_localization", "wrong_floor_tag"):
                reasons = {
                    "localization_failure": "injected localization failure",
                    "stale_localization": "injected stale localization",
                    "wrong_floor_tag": "injected wrong-floor tag",
                }
                self._floor.set_aborted(
                    FloorTransitionResult(
                        result_code=FloorTransitionResult.LOCALIZATION_FAILED,
                        floor_id=self._floor_id,
                        map_generation=self._generation,
                        reason=reasons[scenario],
                    )
                )
                return
            stair = self._stairs.get(goal.transition_id, {})
            endpoint = stair.get("endpoints", {}).get(goal.target_floor, {})
            self._set_current_location(endpoint.get("entry_location_id", self._current_location_id))
            with self._lock:
                self._floor_id = goal.target_floor
                self._generation += 1
                floor_id = self._floor_id
                generation = self._generation
            self._publish_state(None)
            self._floor.set_succeeded(
                FloorTransitionResult(
                    result_code=FloorTransitionResult.OK,
                    floor_id=floor_id,
                    map_generation=generation,
                    reason="",
                )
            )


def main() -> int:
    rospy.init_node("mock_mission_children")
    MockMissionChildren()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
