"""ROS boundary for the exclusive TRON1 stair supervisor."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
import uuid
import threading

import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from stair_supervisor.msg import (
    StairTraversalAction,
    StairTraversalFeedback,
    StairTraversalGoal,
    StairTraversalResult,
    SupervisorState as SupervisorStateMessage,
)

from .configuration import Direction, StairSupervisorConfiguration
from .robot_transport import RobotTransport, SystemClock, TransportFault
from .stair_admission import RosStairAdmissionValidator
from .stair_evidence import EvidenceReport, OdometrySample, StairEvidenceTracker
from .supervisor import ResultCode, StairGoal, StairSupervisor, SupervisorState
from .supervisor_types import AdmissionValidator


_PHASE_TEST_REQUEST_LOCK = threading.Lock()


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
        admission: AdmissionValidator | None = None,
        lidar=None,
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
        self._web_transport = transport
        self._lidar = lidar
        lidar_control = lidar.control if lidar is not None and lidar.configuration.mode == "control" else None
        self._evidence = lidar_control or StairEvidenceTracker()
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
            admission=admission or RosStairAdmissionValidator(
                rospy.get_param("~admission_service", "/mission/validate_stair_admission"),
                float(rospy.get_param("~admission_timeout_sec", 1.0)),
            ),
            nav_freshness_sec=settings.nav_freshness_sec,
            turn_linear_mps=settings.turn_linear_mps,
            lidar_control=lidar_control,
            operator_recovery_enabled=rospy.get_param("~operator_recovery_enabled", True),
        )
        self._nav_subscriber = rospy.Subscriber(
            "/navigation/cmd_vel",
            Twist,
            self._accept_navigation,
            queue_size=1,
        )
        self._joy_subscriber = rospy.Subscriber('/tron/sensor_joy', Joy, self._accept_manual_input, queue_size=1)
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
        self._state_detail = "starting"
        self._publish_state("starting")
        self._supervisor.start()
        self._publish_state("navigation owns commands")
        self._action.start()
        self._state_timer = rospy.Timer(rospy.Duration(0.5), self._publish_heartbeat)
        self._release_service = rospy.Service("~release_arrival_hold", Trigger, self._release_hold)
        self._handoff_service = rospy.Service("~acknowledge_physical_handoff", Trigger, self._acknowledge_handoff)
        self._manual_publisher = rospy.Publisher("~web_manual_status", String, queue_size=1, latch=True)
        self._manual_subscriber = rospy.Subscriber("~web_manual", String, self._accept_web_manual, queue_size=1, tcp_nodelay=True)
        self._operator_publisher = rospy.Publisher("~operator_phase_status", String, queue_size=1, latch=True)
        self._operator_requests = {}
        self._operator_subscriber = rospy.Subscriber("~operator_phase_request", String, self._operator_phase, queue_size=1)
        self._operator_timer = rospy.Timer(rospy.Duration(.2), self._publish_operator_status)
        self._manual_begin_service = rospy.Service("~begin_web_manual", Trigger, self._begin_web_manual)

    def _publish_operator_status(self, _event=None):
        self._operator_publisher.publish(String(data=json.dumps(self._supervisor.operator_status())))

    def _operator_phase(self, message):
        request_id = ''
        try:
            data=json.loads(message.data)
            request_id=data.get('request_id','')
            if not isinstance(request_id,str) or len(request_id)!=32:
                raise ValueError('invalid request identifier')
            if request_id in self._operator_requests:
                self._publish_operator_status()
                return
            self._operator_requests[request_id]=True
            if len(self._operator_requests)>64:
                self._operator_requests.pop(next(iter(self._operator_requests)))
            if not 0 <= time.time()-data.get('issued_at',0) < 3:
                raise ValueError('전이 요청이 만료되었습니다. 다시 선택하세요.')
            self._supervisor.request_operator_phase(data)
        except (ValueError,TypeError,AttributeError) as error:
            status=self._supervisor.operator_status()
            status['result']=dict(request_id=request_id,state='REJECTED',reason=str(error))
            self._operator_publisher.publish(String(data=json.dumps(status)))
            return
        self._publish_operator_status()

    def _begin_web_manual(self, _request):
        try:
            lease = self._web_transport.begin_web_manual()
            self._manual_status()
            return TriggerResponse(success=True, message=json.dumps(dict(lease=lease)))
        except (ValueError, TransportFault) as error:
            return TriggerResponse(success=False, message=str(error))

    def _manual_status(self):
        if hasattr(self, '_manual_publisher'):
            self._manual_publisher.publish(String(data=json.dumps(self._web_transport.web_manual_status())))

    def _accept_web_manual(self, message):
        try:
            data = json.loads(message.data)
            if data['kind'] == 'end':
                self._web_transport.end_web_manual(data['lease'])
            elif data['kind'] == 'update':
                issued = data['issued_at']
                if type(issued) not in (int, float) or not math.isfinite(issued):
                    raise ValueError('invalid manual timestamp')
                age = time.time() - issued
                if not 0 <= age < .35:
                    raise ValueError('stale manual input')
                self._web_transport.update_web_manual(data['lease'], data['sequence'], data['forward'], data['turn'], .35-age)
            else:
                raise ValueError('unknown manual operation')
            self._manual_status()
        except (ValueError, KeyError, TypeError, TransportFault) as error:
            rospy.logwarn_throttle(2., 'web manual input rejected: %s', error)

    def _release_hold(self, _request):
        ok = self._supervisor.release_arrival_hold()
        return TriggerResponse(success=ok, message="hold released" if ok else "stair ownership retained")

    def _accept_manual_input(self, message):
        active = (not message.axes or not all(math.isfinite(v) for v in message.axes)
                  or any(abs(v) > .001 for v in message.axes) or any(message.buttons))
        try:
            self._supervisor.accept_manual_input(active)
        except TransportFault:
            self._supervisor._latch_fault()

    def _acknowledge_handoff(self, _request):
        # One-shot explicit operator attestation; never automatic on sensor recovery.
        if not rospy.get_param("~operator_confirmed_supported_handoff", False):
            return TriggerResponse(success=False, message="operator confirmation required for physical handoff")
        rospy.set_param("~operator_confirmed_supported_handoff", False)
        ok = self._supervisor.acknowledge_physical_handoff()
        return TriggerResponse(success=ok, message="handoff acknowledged" if ok else "no retained loss or traversal still active")

    @property
    def state(self) -> int:
        return int(self._supervisor.state)

    def _accept_navigation(self, message: Twist) -> None:
        self._supervisor.accept_navigation(message.linear.x, message.angular.z)

    def _monotonic_sensor_stamp(self, stamp: rospy.Time) -> float:
        wall_age = (rospy.Time.now() - stamp).to_sec()
        return self._clock.monotonic() - wall_age

    def _accept_odometry(self, message: Odometry) -> None:
        if self._lidar is not None and self._lidar.configuration.mode == "control":
            return  # wheel odometry remains available on its own diagnostic topic
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
        lidar = getattr(self, '_lidar', None)
        if lidar is not None and lidar.control is not None:
            envelope = json.loads(frame)
            if envelope.get('title') == 'request_twist':
                lidar.control.record_sent_twist(envelope['data'], self._clock.monotonic())

    def _stream_timer(self, _event: rospy.timer.TimerEvent) -> None:
        self.stream_once()

    def stream_once(self) -> None:
        try:
            self._web_transport.expire_web_manual()
            if self._web_transport.web_manual_active():
                self._web_transport.send_current()
            else:
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

    def _publish_heartbeat(self, _event: rospy.timer.TimerEvent) -> None:
        """Refresh ownership state while preserving its latest diagnostic detail."""
        self._publish_state(self._state_detail)
        self._manual_status()

    def _execute(self, message: StairTraversalGoal) -> None:
        receipt = None
        handled = False
        try:
            token = message.admission_token
            if token.startswith("phase-test:"):
                identifier = token.partition(":")[2]
                if uuid.UUID(identifier).hex != identifier:
                    raise ValueError("invalid phase-test identifier")
                key = "~phase_test_requests/" + identifier
                with _PHASE_TEST_REQUEST_LOCK:
                    envelope = rospy.get_param(key, None)
                    if envelope is None:
                        raise ValueError("phase-test request missing or already consumed")
                    rospy.delete_param(key)
                expires = envelope.get("expires_at")
                if type(expires) not in (float, int) or not math.isfinite(expires) or time.time() > expires:
                    raise ValueError("phase-test request expired")
                phase_test = envelope["test"]
                if phase_test.get("route_id") != message.stair_id:
                    raise ValueError("phase-test request belongs to another route")
                receipt = "~phase_test_receipts/" + identifier
                rospy.set_param(receipt, dict(goal_id=self._action.current_goal.get_goal_id().id, state="accepted"))
            else:
                phase_test = rospy.get_param("~phase_test", None)
                if phase_test is not None:
                    rospy.delete_param("~phase_test")  # compatibility with existing manual requests
            self._execute_request(message, phase_test)
            handled = True
        except (ValueError, KeyError, TypeError, AttributeError) as error:
            self._action.set_aborted(StairTraversalResult(result_code=StairTraversalResult.ENTRY_REJECTED, reason=str(error)))
        finally:
            if receipt is not None:
                # Receipt also allows a client with interrupted dispatch to
                # reconcile/cancel this specific goal, without cancelling others.
                current = rospy.get_param(receipt, {})
                # A boundary exception must not masquerade as completed handling.
                current["state"] = "finished" if handled else "failed"
                rospy.set_param(receipt, current)

    def _execute_request(self, message: StairTraversalGoal, phase_test) -> None:
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
        if self._lidar is not None and self._lidar.configuration.mode == "control":
            try:
                test_resume = phase_test is not None and self._supervisor.state is SupervisorState.STAIR
                if self._supervisor.state is not SupervisorState.NAV and not test_resume:
                    raise ValueError("supervisor is not in NAV")
                # Explicit phase tests use the fresh, operator-previewed anchor.
                # Automatic missions still require the surveyed cloud match.
                if phase_test is None:
                    self._lidar.ensure_entry(message.stair_id)
            except (ValueError, KeyError, OSError) as error:
                self._action.set_aborted(StairTraversalResult(result_code=StairTraversalResult.ENTRY_REJECTED, reason=str(error)))
                return
        arguments = {} if phase_test is None else {"phase_test": phase_test}
        outcome = self._supervisor.traverse(
            StairGoal(message.stair_id, direction, message.admission_token),
            self._action.is_preempt_requested,
            **arguments,
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
        self._state_detail = detail
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
        self._state_timer.shutdown()
        self._supervisor.shutdown()
        if self._lidar is not None:
            self._lidar.shutdown()
        self._manual_begin_service.shutdown()
        self._manual_subscriber.unregister()
        self._release_service.shutdown()
        self._handoff_service.shutdown()
        self._publish_state("shutdown")
