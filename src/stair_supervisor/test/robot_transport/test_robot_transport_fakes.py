"""Deterministic WebSocket and clock fakes for robot transport tests."""

from __future__ import annotations

import json
from typing import Callable, Dict, List, Union

from stair_supervisor.configuration import WebSocketCalibration
from stair_supervisor.robot_config import (
    CommandStreamConfig,
    RobotConnectionConfig,
    RobotTransportConfig,
)
from stair_supervisor.robot_transport import RobotTransport


JsonScalar = Union[str, int, float, bool, None]
JsonValue = Union[JsonScalar, List["JsonValue"], Dict[str, "JsonValue"]]
SendHandler = Callable[[Dict[str, JsonValue], "FakeWebSocket"], None]


class FakeClock:
    """Mutable clock whose movement is controlled by the test."""

    def __init__(self, milliseconds: int = 1_700_000_000_000) -> None:
        self._milliseconds = milliseconds

    def now_ms(self) -> int:
        return self._milliseconds

    def monotonic(self) -> float:
        current = self._milliseconds / 1000.0
        self._milliseconds += 10
        return current

    def sleep(self, seconds: float) -> None:
        self._milliseconds += round(seconds * 1000.0)


class FakeWebSocket:
    """Mutable scripted socket used to inspect the transport wire contract."""

    def __init__(self, handler: SendHandler) -> None:
        self.handler = handler
        self.sent: List[Dict[str, JsonValue]] = []
        self.sent_payloads: List[str] = []
        self.inbound: List[Union[str, None, BaseException]] = []
        self.closed = False
        self.receive_timeouts: List[float] = []
        self.fail_next_send_count = 0

    def send(self, payload: str) -> None:
        if self.fail_next_send_count > 0:
            self.fail_next_send_count -= 1
            raise OSError("scripted send failure")
        parsed = json.loads(payload)
        assert isinstance(parsed, dict)
        self.sent_payloads.append(payload)
        self.sent.append(parsed)
        self.handler(parsed, self)

    def recv(self) -> str | None:
        if not self.inbound:
            raise TimeoutError()
        item = self.inbound.pop(0)
        if isinstance(item, BaseException):
            raise item
        return item

    def settimeout(self, timeout: float) -> None:
        self.receive_timeouts.append(timeout)

    def close(self) -> None:
        self.closed = True

    def queue(self, payload: Dict[str, JsonValue]) -> None:
        self.inbound.append(json.dumps(payload, separators=(",", ":")))


class FakeFactory:
    """Return one socket and record every prohibited reconnect attempt."""

    def __init__(self, socket: FakeWebSocket) -> None:
        self.socket = socket
        self.calls = 0

    def __call__(self, url: str, timeout: float) -> FakeWebSocket:
        assert url.startswith("ws://")
        assert timeout > 0.0
        self.calls += 1
        return self.socket


def make_transport(
    socket: FakeWebSocket,
    clock: FakeClock,
    request_timeout: float = 0.05,
    mode_attempts: int = 3,
) -> tuple[RobotTransport, FakeFactory]:
    """Build one transport with deterministic test-only policy."""
    factory = FakeFactory(socket)
    config = RobotTransportConfig(
        connection=RobotConnectionConfig(
            accid="WF_TEST_001",
            url="ws://127.0.0.1:5000",
            receive_timeout=0.01,
            request_timeout=request_timeout,
        ),
        stream=CommandStreamConfig(
            rate_hz=40.0,
            watchdog_sec=0.25,
            startup_zero_repeats=2,
            close_zero_repeats=4,
            mode_attempts=mode_attempts,
        ),
    )
    return RobotTransport(
        config=config,
        calibration=WebSocketCalibration(0.5, 2.0),
        connection_factory=factory,
        clock=clock,
    ), factory


def successful_mode_handler(
    request: Dict[str, JsonValue], socket: FakeWebSocket
) -> None:
    """Queue documented correlated success and fresh mode status messages."""
    title = request["title"]
    if title == "request_twist":
        return
    assert isinstance(title, str)
    guid = request["guid"]
    timestamp = request["timestamp"]
    assert isinstance(guid, str)
    assert isinstance(timestamp, int)
    socket.queue(
        {
            "accid": request["accid"],
            "title": title.replace("request_", "response_", 1),
            "timestamp": timestamp,
            "guid": guid,
            "data": {"result": "success"},
        }
    )
    statuses = {
        "request_stand_mode": "STAND",
        "request_walk_mode": "WALK",
        "request_stair_mode": "STAIR"
        if request["data"] == {"enable": True}
        else "WALK",
    }
    if title in statuses:
        socket.queue(
            {
                "accid": request["accid"],
                "title": "notify_robot_info",
                "timestamp": timestamp,
                "guid": "status-guid",
                "data": {"status": statuses[title]},
            }
        )
