"""Typed connection and command-stream policy for the TRON1 transport."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RobotConfigError(ValueError):
    """Report one invalid robot transport configuration field."""

    __slots__ = ("field", "detail")
    field: str
    detail: str

    def __str__(self) -> str:
        return f"{self.field}: {self.detail}"


def _positive(field: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise RobotConfigError(field, "must be finite and positive")


@dataclass(frozen=True)  # noqa: SLOTS_OK - Python 3.8 slots conflict with dataclass defaults.
class RobotConnectionConfig:
    """Robot identity and bounded WebSocket operation timeouts."""

    accid: str
    url: str
    connect_timeout: float = 3.0
    receive_timeout: float = 0.1
    request_timeout: float = 8.0

    def __post_init__(self) -> None:
        if not self.accid:
            raise RobotConfigError("accid", "must not be empty")
        if not self.url.startswith(("ws://", "wss://")):
            raise RobotConfigError("url", "must use ws:// or wss://")
        _positive("connect_timeout", self.connect_timeout)
        _positive("receive_timeout", self.receive_timeout)
        _positive("request_timeout", self.request_timeout)


@dataclass(frozen=True)  # noqa: SLOTS_OK - Python 3.8 slots conflict with dataclass defaults.
class CommandStreamConfig:
    """Fail-closed command cadence, freshness, and zero-barrier policy."""

    rate_hz: float
    watchdog_sec: float
    startup_zero_repeats: int = 3
    close_zero_repeats: int = 8
    mode_attempts: int = 3

    def __post_init__(self) -> None:
        if not math.isfinite(self.rate_hz) or self.rate_hz < 30.0:
            raise RobotConfigError("rate_hz", "must be finite and at least 30 Hz")
        _positive("watchdog_sec", self.watchdog_sec)
        if self.startup_zero_repeats < 1:
            raise RobotConfigError("startup_zero_repeats", "must be positive")
        if self.close_zero_repeats < 1:
            raise RobotConfigError("close_zero_repeats", "must be positive")
        if not isinstance(self.mode_attempts, int) or self.mode_attempts < 1:
            raise RobotConfigError("mode_attempts", "must be a positive integer")

    @property
    def period_sec(self) -> float:
        return 1.0 / self.rate_hz


@dataclass(frozen=True)
class RobotTransportConfig:
    """Compose transport policy without conflating robot and move_base limits."""

    __slots__ = ("connection", "stream")
    connection: RobotConnectionConfig
    stream: CommandStreamConfig
