"""ROS boundary for the exclusive TRON1 stair supervisor."""

from __future__ import annotations

from dataclasses import dataclass
import math

import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import String

from stair_supervisor.msg import (
    StairTraversalAction,
    StairTraversalFeedback,
    StairTraversalGoal,
    StairTraversalResult,
    SupervisorState as SupervisorStateMessage,
)

from .configuration import Direction, StairSupervisorConfiguration
from .robot_transport import RobotTransport, SystemClock, TransportFault
from .stair_evidence import EvidenceReport, OdometrySample, StairEvidenceTracker
from .supervisor import ResultCode, StairGoal, StairSupervisor, SupervisorState


@dataclass(frozen=True)
class RosNodeSettings:
    __slots__ = ("action_name", "nav_freshness_sec", "stream_rate_hz", "turn_linear_mps", "odom_topic")
    action_name: str
    nav_freshness_sec: float
    stream_rate_hz: float
    turn_linear_mps: float
    odom_topic: str


def _yaw_from_orientation(x: float, y: float, z: float, w: float) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 0.01:
        return math.nan
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw


class RosStairSupervisorNode:
    """Bind the tested supervisor core to generated ROS interfaces."""

    def __init__(
        self,
        configuration: StairSupervisorConfiguration,
        transport: RobotTransport,
        settings: RosNodeSettings,
    ) -> None:
        self._state_publisher = rospy.Publisher(
            "~state",
            SupervisorStateMessage,
            queue_size=1,
            latch=True,
        )
        self._websocket_tx_publisher = rospy.Publisher(
            "/stair_supervisor/websocket_tx",
            String,
            queue_size=1000,
        )
        transport.observe_sent_frames(self._publish_websocket_tx)
        self._clock = SystemClock()
        self._evidence = StairEvidenceTracker()
        self._action = actionlib.SimpleActionServer(
            settings.action_name,
            StairTraversalAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._supervisor = StairSupervisor(
            configuration=configuration,
            transport=transport,
            evidence=self._evidence,
            clock=self._clock,
            feedback=self._publish_feedback,
            nav_freshness_sec=settings.nav_freshness_sec,
            turn_linear_mps=settings.turn_linear_mps,
        )
        self._nav_subscriber = rospy.Subscriber(
            "/navigation/cmd_vel",
            Twist,
            self._accept_navigation,
            queue_size=1,
        )
        self._odom_subscriber = rospy.Subscriber(
            settings.odom_topic,
            Odometry,
            self._accept_odometry,
            queue_size=10,
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / settings.stream_rate_hz),
            self._stream_timer,
        )
        self._shutdown = False
        self._publish_state("starting")
        self._supervisor.start()
        self._publish_state("navigation owns commands")
        self._action.start()

    @property
    def state(self) -> int:
        return int(self._supervisor.state)

    def _accept_navigation(self, message: Twist) -> None:
        self._supervisor.accept_navigation(message.linear.x, message.angular.z)

    def _monotonic_sensor_stamp(self, stamp: rospy.Time) -> float:
        wall_age = (rospy.Time.now() - stamp).to_sec()
        return self._clock.monotonic() - wall_age

    def _accept_odometry(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw = _yaw_from_orientation(orientation.x, orientation.y, orientation.z, orientation.w)
        position = message.pose.pose.position
        self._evidence.update_odometry(
            OdometrySample(
                self._monotonic_sensor_stamp(message.header.stamp),
                position.x,
                position.y,
                yaw,
            )
        )

    def _publish_websocket_tx(self, frame: str) -> None:
        self._websocket_tx_publisher.publish(String(data=frame))

    def _stream_timer(self, _event: rospy.timer.TimerEvent) -> None:
        self.stream_once()

    def stream_once(self) -> None:
        try:
            self._supervisor.stream_navigation()
        except TransportFault as error:
            self._timer.shutdown()
            self._publish_state(str(error))
            rospy.logerr("robot command transport fault: %s", error)

    def _publish_feedback(self, report: EvidenceReport) -> None:
        self._publish_state(report.detail)
        self._action.publish_feedback(
            StairTraversalFeedback(phase=report.phase.value, detail=report.detail)
        )

    def _execute(self, message: StairTraversalGoal) -> None:
        directions = {
            StairTraversalGoal.UP: Direction.UP,
            StairTraversalGoal.DOWN: Direction.DOWN,
        }
        direction = directions.get(message.direction)
        if direction is None:
            result = StairTraversalResult(
                result_code=StairTraversalResult.INVALID_GOAL,
                reason="unknown direction",
            )
            self._action.set_aborted(result)
            return
        outcome = self._supervisor.traverse(
            StairGoal(message.stair_id, direction),
            self._action.is_preempt_requested,
        )
        self._publish_state(outcome.reason)
        result = StairTraversalResult(
            result_code=int(outcome.code),
            reason=outcome.reason,
            ownership_epoch=self._supervisor.ownership_epoch,
        )
        if outcome.cancelled:
            self._action.set_preempted(result)
        elif outcome.code is ResultCode.OK:
            self._action.set_succeeded(result)
        else:
            self._action.set_aborted(result)

    def _publish_state(self, detail: str) -> None:
        state = self._supervisor.state
        message = SupervisorStateMessage(
            state=int(state),
            connected=state in (SupervisorState.NAV, SupervisorState.STAIR),
            detail=detail,
            ownership_epoch=self._supervisor.ownership_epoch,
        )
        message.header.stamp = rospy.Time.now()
        self._state_publisher.publish(message)

    def shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        self._timer.shutdown()
        self._supervisor.shutdown()
        self._publish_state("shutdown")
