"""Fail-closed high-level robot mode and velocity transport."""

from __future__ import annotations

from .web_manual import WebManual

from dataclasses import dataclass
from enum import Enum
import threading
from typing import Callable, Mapping

from .configuration import WebSocketCalibration
from .robot_client import (
    ConnectionFactory,
    DirectRobotClient,
    ReceiveTimeout,
    RequestReceipt,
    RobotClientError,
    open_websocket,
)
from .robot_clock import Clock, SystemClock
from .robot_config import RobotTransportConfig
from .robot_conversion import NormalizedTwist, normalize_twist
from .robot_protocol import JsonValue, ProtocolError, RequestTitle, RobotMessage, RobotStatus
from .robot_tolerances import ModeRetryBudget, OutageBudget


class TransportState(str, Enum):
    NEW = "NEW"
    CONNECTED = "CONNECTED"
    READY = "READY"
    FAULT = "FAULT"
    CLOSED = "CLOSED"


@dataclass(frozen=True)
class TransportFault(Exception):
    __slots__ = ("reason",)
    reason: str

    def __str__(self) -> str:
        return self.reason


class RobotTransport:
    """Mutable owner of one non-resumable robot command session."""

    def __init__(
        self,
        config: RobotTransportConfig,
        calibration: WebSocketCalibration,
        connection_factory: ConnectionFactory = open_websocket,
        clock: Clock = SystemClock(),
    ) -> None:
        self._config = config
        self._calibration = calibration
        self._clock = clock
        self._client = DirectRobotClient(config.connection, connection_factory)
        self._state = TransportState.NEW
        self._fault: TransportFault | None = None
        self._command_lock = threading.RLock()
        self._web_manual = WebManual()
        self._desired_twist = NormalizedTwist.zero()
        self._latest_twist = NormalizedTwist.zero()
        self._updated_at = float("-inf")
        self._outage = OutageBudget(config.stream)

    @property
    def state(self) -> TransportState:
        return self._state

    @property
    def faulted(self) -> bool:
        return self._fault is not None

    @property
    def pending_request_count(self) -> int:
        return self._client.pending_request_count

    @property
    def stream_period_sec(self) -> float:
        return self._config.stream.period_sec

    def _raise_if_unavailable(self) -> None:
        if self._fault is not None:
            raise self._fault
        if self._state is TransportState.CLOSED:
            raise TransportFault("robot transport is closed")

    def observe_sent_frames(self, observer: Callable[[str], None]) -> None:
        """Install the boundary observer before the command session starts."""
        if self._state is not TransportState.NEW:
            raise TransportFault("sent-frame observer must be installed before start")
        self._client.observe_sent_frames(observer)

    def _latch_fault(self, reason: str) -> None:
        if self._fault is None:
            self._fault = TransportFault(reason)
            self._state = TransportState.FAULT
            self._client.close()
        raise self._fault

    def _send_request(
        self,
        title: RequestTitle,
        data: Mapping[str, JsonValue],
    ) -> RequestReceipt | None:
        """Send one frame and return its receipt, or None on a soft send failure."""
        try:
            receipt = self._client.send_request(title, data, self._clock.now_ms())
        except RobotClientError:
            return None
        self._outage.record_success(self._clock.monotonic())
        return receipt

    def _receive(self, receipt: RequestReceipt) -> RobotMessage | None:
        try:
            return self._client.receive()
        except ReceiveTimeout:
            return None
        except (ProtocolError, RobotClientError) as error:
            self._client.cancel_response(receipt)
            self._latch_fault(str(error))
        raise AssertionError("fault latch must raise")

    def _request_success(
        self,
        title: RequestTitle,
        data: Mapping[str, JsonValue],
        expected_status: RobotStatus | None,
        progress=None,
    ) -> None:
        mode_retry = ModeRetryBudget(
            self._config.connection,
            self._config.stream.mode_attempts,
        )
        while mode_retry.consume_attempt():
            receipt = self._send_request(title, data)
            if receipt is None:
                continue
            deadline = self._clock.monotonic() + mode_retry.attempt_timeout
            response_seen = False
            response_timestamp_ms: int | None = None
            status_seen = expected_status is None
            try:
                while self._clock.monotonic() < deadline:
                    if progress is not None:
                        progress()
                    message = self._receive(receipt)
                    if message is None:
                        continue
                    if self._client.consume_response(receipt, message):
                        if message.data.get("result") != "success":
                            self._latch_fault(f"{title.value} was rejected")
                        response_seen = True
                        response_timestamp_ms = message.timestamp_ms
                    elif (
                        message.title == "notify_robot_info"
                        and response_timestamp_ms is not None
                        and message.timestamp_ms >= response_timestamp_ms
                    ):
                        status_seen = self._message_has_status(message, expected_status)
                    if response_seen and status_seen:
                        return
            finally:
                self._client.cancel_response(receipt)
        self._latch_fault(f"{title.value} timed out before verified status")

    def _message_has_status(
        self,
        message: RobotMessage,
        expected_status: RobotStatus | None,
    ) -> bool:
        if expected_status is None:
            return True
        value = message.data.get("status")
        if type(value) is not str:
            self._latch_fault("notify_robot_info has no valid status")
        try:
            status = RobotStatus(value)
        except ValueError:
            self._latch_fault("notify_robot_info contains an unknown status")
        return status is expected_status

    def _send_twist(self, command: NormalizedTwist, now: float) -> None:
        receipt = self._send_request(
            RequestTitle.TWIST,
            {"x": command.x, "y": command.y, "z": command.z},
        )
        if receipt is None and not self._outage.check(now):
            self._latch_fault("twist send failed after watchdog budget")

    def start(self) -> None:
        """Open one session and verify fresh STAND then WALK state reports."""
        self._raise_if_unavailable()
        if self._state is not TransportState.NEW:
            raise TransportFault("robot transport can only start once")
        try:
            self._client.connect()
        except RobotClientError as error:
            self._latch_fault(str(error))
        self._state = TransportState.CONNECTED
        self._outage.record_success(self._clock.monotonic())
        for _ in range(self._config.stream.startup_zero_repeats):
            self._send_twist(NormalizedTwist.zero(), self._clock.monotonic())
            self._clock.sleep(self._config.stream.period_sec)
        # Deployed firmware acknowledges stand mode while its status remains WALK.
        self._request_success(RequestTitle.STAND_MODE, {}, None)
        self._request_success(RequestTitle.WALK_MODE, {}, RobotStatus.WALK)
        with self._command_lock:
            self._desired_twist = NormalizedTwist.zero()
            self._latest_twist = NormalizedTwist.zero()
            self._updated_at = self._clock.monotonic()
        self._state = TransportState.READY

    def update_twist(self, linear_mps: float, angular_radps: float) -> None:
        """Store one converted physical command for the next stream tick."""
        self._raise_if_unavailable()
        if self._state is not TransportState.READY:
            raise TransportFault("robot transport is not ready for motion")
        command = normalize_twist(linear_mps, angular_radps, self._calibration)
        with self._command_lock:
            self._desired_twist = command
            self._latest_twist = command
            self._updated_at = self._clock.monotonic()

    def begin_web_manual(self):
        self._require_ready()
        with self._command_lock:
            return self._web_manual.begin(self._clock.monotonic())

    def update_web_manual(self, lease, sequence, forward, turn, remaining):
        self._require_ready()
        with self._command_lock:
            now = self._clock.monotonic()
            self._web_manual.update(lease, sequence, forward, turn, now, remaining)
            self.send_current()

    def end_web_manual(self, lease):
        with self._command_lock:
            self._web_manual.end(lease)
            self._send_twist(NormalizedTwist.zero(), self._clock.monotonic())

    def expire_web_manual(self):
        with self._command_lock:
            now = self._clock.monotonic()
            if self._web_manual.lease is not None and not self._web_manual.ended and now >= self._web_manual.expires:
                self._web_manual.end(self._web_manual.lease)
                self._send_twist(NormalizedTwist.zero(), now)

    def web_manual_active(self):
        with self._command_lock:
            return self._web_manual.selected(self._clock.monotonic()) is not None

    def web_manual_status(self):
        with self._command_lock:
            return self._web_manual.snapshot(self._clock.monotonic())

    def send_current(self) -> None:
        """Emit one stream tick, replacing stale input with a complete zero."""
        self._raise_if_unavailable()
        if self._state is not TransportState.READY:
            raise TransportFault("robot transport is not ready for motion")
        with self._command_lock:
            now = self._clock.monotonic()
            age = now - self._updated_at
            desired_twist = self._desired_twist
            self._latest_twist = (
                NormalizedTwist.zero()
                if age > self._config.stream.watchdog_sec
                else desired_twist
            )
            manual = self._web_manual.selected(now)
            if manual is not None:
                self._latest_twist = manual
            self._send_twist(self._latest_twist, now)

    def request_stair_mode(self, enabled: bool) -> None:
        """Use the documented wheel-foot stair request and verify its resulting mode."""
        self._require_ready()
        expected = RobotStatus.STAIR if enabled else RobotStatus.WALK
        self._request_success(RequestTitle.STAIR_MODE, {"enable": enabled}, expected)

    def request_stair_mode_with_feedback(self, enabled: bool, progress) -> None:
        """Keep the sole owner's position feedback alive during the mode ACK.

        The same thread owns receive/response correlation; no competing socket
        reader or second command session is introduced.
        """
        self._require_ready()
        self._client.set_receive_timeout(min(self._config.connection.receive_timeout,
                                            self.stream_period_sec / 4.))
        try:
            expected = RobotStatus.STAIR if enabled else RobotStatus.WALK
            self._request_success(RequestTitle.STAIR_MODE, {"enable": enabled}, expected, progress)
        finally:
            if not self.faulted:
                self._client.set_receive_timeout(self._config.connection.receive_timeout)

    def request_emergency_stop(self) -> None:
        """Send the documented empty emergency-stop request abstraction."""
        self._require_ready()
        self._request_success(RequestTitle.EMERGENCY_STOP, {}, None)

    def set_odometry_enabled(self, enabled: bool) -> None:
        """Enable or disable documented wheel-foot odometry pushes."""
        self._require_ready()
        self._request_success(RequestTitle.ENABLE_ODOMETRY, {"enable": enabled}, None)

    def set_imu_enabled(self, enabled: bool) -> None:
        """Enable or disable documented IMU pushes."""
        self._require_ready()
        self._request_success(RequestTitle.ENABLE_IMU, {"enable": enabled}, None)

    def _require_ready(self) -> None:
        self._raise_if_unavailable()
        if self._state is not TransportState.READY:
            raise TransportFault("robot transport is not ready")

    def close(self) -> None:
        """Send repeated zero barriers before a normal connected shutdown."""
        if self._state is TransportState.CLOSED:
            return
        try:
            if self._state in (TransportState.CONNECTED, TransportState.READY):
                for _ in range(self._config.stream.close_zero_repeats):
                    self._send_request(
                        RequestTitle.TWIST,
                        {"x": 0.0, "y": 0.0, "z": 0.0},
                    )
                    self._clock.sleep(self._config.stream.period_sec)
        finally:
            self._client.close()
            if self._fault is None:
                self._state = TransportState.CLOSED

    def __enter__(self) -> "RobotTransport":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
