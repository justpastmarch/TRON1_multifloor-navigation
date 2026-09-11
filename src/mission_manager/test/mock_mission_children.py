#!/usr/bin/env python3
"""Test-only child action peers for end-to-end Mission.action scenarios."""

from __future__ import annotations

import threading

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from multifloor_manager.msg import (
    FloorState,
    FloorTransitionAction,
    FloorTransitionResult,
)
from nav_msgs.msg import Odometry
import rospy
from stair_supervisor.msg import (
    StairTraversalAction,
    StairTraversalResult,
    SupervisorState,
)
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse


class MockMissionChildren:
    """Publish healthy state and script child terminal outcomes from one ROS param."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._execution_lock = threading.RLock()
        self._floor_id = rospy.get_param("~initial_floor", "3F")
        self._generation = 1
        self._nav_attempt = 0
        self._floor_pub = rospy.Publisher(
            "/multifloor/floor_state", FloorState, queue_size=1, latch=True
        )
        self._supervisor_pub = rospy.Publisher(
            "/stair_supervisor/state", SupervisorState, queue_size=1, latch=True
        )
        self._odom_pub = rospy.Publisher(
            "/tron/wheel_odom_raw", Odometry, queue_size=1, latch=True
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

    def _execute_nav(self, _goal: MoveBaseAction.Goal) -> None:
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
            self._nav.set_succeeded(MoveBaseResult())

    def _execute_stair(self, _goal: StairTraversalAction.Goal) -> None:
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
