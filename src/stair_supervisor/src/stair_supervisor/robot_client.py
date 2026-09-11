"""One-shot WebSocket ownership and GUID correlation for TRON1."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Mapping, Protocol
import uuid

import websocket

from .robot_config import RobotConnectionConfig
from .robot_protocol import (
    JsonValue,
    RequestEnvelope,
    RequestTitle,
    RobotMessage,
    encode_request,
    parse_message,
)


class WebSocketConnection(Protocol):
    def send(self, payload: str) -> None: ...

    def recv(self) -> str | bytes | None: ...

    def settimeout(self, timeout: float) -> None: ...

    def close(self) -> None: ...


class ConnectionFactory(Protocol):
    def __call__(self, url: str, timeout: float) -> WebSocketConnection: ...


@dataclass(frozen=True)
class RobotClientError(Exception):
    __slots__ = ("detail",)
    detail: str

    def __str__(self) -> str:
        return self.detail


class ReceiveTimeout(RobotClientError):
    """Report one bounded receive interval with no frame."""


@dataclass(frozen=True)
class RequestReceipt:
    __slots__ = ("guid", "response_title", "sent_at_ms")
    guid: str
    response_title: str | None
    sent_at_ms: int


class _WebSocketAdapter:
    """Translate websocket-client failures into the library error boundary."""

    def __init__(self, url: str, timeout: float) -> None:
        try:
            self._connection = websocket.create_connection(
                url,
                timeout=timeout,
                enable_multithread=True,
            )
        except websocket.WebSocketException as error:
            raise RobotClientError("WebSocket connection failed") from error

    def send(self, payload: str) -> None:
        try:
            self._connection.send(payload)
        except websocket.WebSocketException as error:
            raise RobotClientError("WebSocket send failed") from error

    def recv(self) -> str | bytes | None:
        try:
            return self._connection.recv()
        except websocket.WebSocketTimeoutException as error:
            raise ReceiveTimeout("WebSocket receive timed out") from error
        except websocket.WebSocketException as error:
            raise RobotClientError("WebSocket receive failed") from error

    def settimeout(self, timeout: float) -> None:
        self._connection.settimeout(timeout)

    def close(self) -> None:
        self._connection.close()


def open_websocket(url: str, timeout: float) -> WebSocketConnection:
    """Open the production websocket-client adapter."""
    return _WebSocketAdapter(url, timeout)


class DirectRobotClient:
    """Own exactly one WebSocket; mutation tracks its pending correlations."""

    def __init__(
        self,
        config: RobotConnectionConfig,
        connection_factory: ConnectionFactory = open_websocket,
    ) -> None:
        self._config = config
        self._connection_factory = connection_factory
        self._connection: WebSocketConnection | None = None
        self._pending: set[tuple[str, str]] = set()
        self._sent_observer: Callable[[str], None] = lambda _frame: None

    @property
    def pending_request_count(self) -> int:
        return len(self._pending)

    def connect(self) -> None:
        if self._connection is not None:
            raise RobotClientError("WebSocket is already connected")
        try:
            connection = self._connection_factory(
                self._config.url,
                self._config.connect_timeout,
            )
            connection.settimeout(self._config.receive_timeout)
        except (OSError, RobotClientError) as error:
            raise RobotClientError("WebSocket connection failed") from error
        self._connection = connection

    def observe_sent_frames(self, observer: Callable[[str], None]) -> None:
        """Report exact request frames after their socket send succeeds."""
        self._sent_observer = observer

    def send_request(
        self,
        title: RequestTitle,
        data: Mapping[str, JsonValue],
        timestamp_ms: int | None = None,
    ) -> RequestReceipt:
        connection = self._connection
        if connection is None:
            raise RobotClientError("WebSocket is not connected")
        sent_at_ms = round(time.time() * 1000.0) if timestamp_ms is None else timestamp_ms
        guid = str(uuid.uuid4())
        response_title = (
            None
            if title is RequestTitle.TWIST
            else title.value.replace("request_", "response_", 1)
        )
        if response_title is not None:
            self._pending.add((response_title, guid))
        payload = encode_request(
            RequestEnvelope(
                accid=self._config.accid,
                title=title,
                timestamp_ms=sent_at_ms,
                guid=guid,
                data=data,
            )
        )
        try:
            connection.send(payload)
        except (OSError, RobotClientError) as error:
            if response_title is not None:
                self._pending.discard((response_title, guid))
            raise RobotClientError("WebSocket send failed") from error
        self._sent_observer(payload)
        return RequestReceipt(guid, response_title, sent_at_ms)

    def receive(self) -> RobotMessage:
        connection = self._connection
        if connection is None:
            raise RobotClientError("WebSocket is not connected")
        try:
            frame = connection.recv()
        except TimeoutError as error:
            raise ReceiveTimeout("WebSocket receive timed out") from error
        except OSError as error:
            raise RobotClientError("WebSocket receive failed") from error
        if frame is None or frame == "" or frame == b"":
            raise RobotClientError("WebSocket disconnected")
        message = parse_message(frame)
        if message.accid != self._config.accid:
            raise RobotClientError("robot frame ACCID does not match the connection")
        return message

    def consume_response(self, receipt: RequestReceipt, message: RobotMessage) -> bool:
        expected = receipt.response_title
        if expected is None or message.title != expected or message.guid != receipt.guid:
            return False
        correlation = (expected, receipt.guid)
        if correlation not in self._pending:
            return False
        self._pending.remove(correlation)
        return True

    def cancel_response(self, receipt: RequestReceipt) -> None:
        if receipt.response_title is not None:
            self._pending.discard((receipt.response_title, receipt.guid))

    def close(self) -> None:
        connection = self._connection
        self._connection = None
        self._pending.clear()
        if connection is not None:
            try:
                connection.close()
            except (OSError, websocket.WebSocketException):
                return
