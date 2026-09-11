"""Fail-closed high-level robot mode and velocity transport."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading
import time
from typing import Callable, Mapping, Protocol

from .configuration import WebSocketCalibration
from .robot_client import (
    ConnectionFactory,
    DirectRobotClient,
    ReceiveTimeout,
    RequestReceipt,
    RobotClientError,
    open_websocket,
)
from .robot_config import RobotTransportConfig
from .robot_conversion import NormalizedTwist, normalize_twist
from .robot_protocol import JsonValue, ProtocolError, RequestTitle, RobotMessage, RobotStatus


class Clock(Protocol):
    def now_ms(self) -> int: ...

    def monotonic(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class SystemClock:
    def now_ms(self) -> int:
        return round(time.time() * 1000.0)

    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)


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
        self._command_lock = threading.Lock()
        self._latest_twist = NormalizedTwist.zero()
        self._updated_at = float("-inf")

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
    ) -> RequestReceipt:
        try:
            return self._client.send_request(title, data, self._clock.now_ms())
        except RobotClientError as error:
            self._latch_fault(str(error))
        raise AssertionError("fault latch must raise")

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
    ) -> None:
        receipt = self._send_request(title, data)
        deadline = self._clock.monotonic() + self._config.connection.request_timeout
        response_seen = False
        response_timestamp_ms: int | None = None
        status_seen = expected_status is None
        while self._clock.monotonic() < deadline:
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

    def _send_twist(self, command: NormalizedTwist) -> None:
        self._send_request(
            RequestTitle.TWIST,
            {"x": command.x, "y": command.y, "z": command.z},
        )

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
        for _ in range(self._config.stream.startup_zero_repeats):
            self._send_twist(NormalizedTwist.zero())
            self._clock.sleep(self._config.stream.period_sec)
        # Deployed firmware acknowledges stand mode while its status remains WALK.
        self._request_success(RequestTitle.STAND_MODE, {}, None)
        self._request_success(RequestTitle.WALK_MODE, {}, RobotStatus.WALK)
        with self._command_lock:
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
            self._latest_twist = command
            self._updated_at = self._clock.monotonic()

    def send_current(self) -> None:
        """Emit one stream tick, replacing stale input with a complete zero."""
        self._raise_if_unavailable()
        if self._state is not TransportState.READY:
            raise TransportFault("robot transport is not ready for motion")
        now = self._clock.monotonic()
        with self._command_lock:
            age = now - self._updated_at
            latest_twist = self._latest_twist
        command = (
            NormalizedTwist.zero()
            if age > self._config.stream.watchdog_sec
            else latest_twist
        )
        self._send_twist(command)

    def request_stair_mode(self, enabled: bool) -> None:
        """Use the documented wheel-foot stair request and verify its resulting mode."""
        self._require_ready()
        expected = RobotStatus.STAIR if enabled else RobotStatus.WALK
        self._request_success(RequestTitle.STAIR_MODE, {"enable": enabled}, expected)

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
                    self._send_twist(NormalizedTwist.zero())
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
