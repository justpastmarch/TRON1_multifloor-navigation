"""Bounded fault budgets for transient WebSocket command failures."""

from __future__ import annotations

from .robot_config import CommandStreamConfig, RobotConnectionConfig


class OutageBudget:
    """Allow short twist stream outages while staying inside the robot watchdog."""

    __slots__ = ("_stream", "_last_success_at")

    def __init__(self, stream: CommandStreamConfig) -> None:
        self._stream = stream
        self._last_success_at = float("-inf")

    def record_success(self, now: float) -> None:
        """Mark a successful frame send as the fresh baseline."""
        self._last_success_at = now

    def check(self, now: float) -> bool:
        """Return True when the current outage is still within budget."""
        return now - self._last_success_at <= self._stream.watchdog_sec


class ModeRetryBudget:
    """Retry WALK/STAIR mode requests a bounded number of times."""

    __slots__ = ("_connection", "_attempts_remaining")

    def __init__(self, connection: RobotConnectionConfig, attempts: int) -> None:
        self._connection = connection
        self._attempts_remaining = max(1, attempts)

    @property
    def attempt_timeout(self) -> float:
        """Timeout for one send/response/status verification cycle."""
        return self._connection.request_timeout

    def consume_attempt(self) -> bool:
        """Return True if another attempt may be started now."""
        if self._attempts_remaining <= 0:
            return False
        self._attempts_remaining -= 1
        return True

    @property
    def exhausted(self) -> bool:
        """Return True when all configured attempts have been consumed."""
        return self._attempts_remaining <= 0
