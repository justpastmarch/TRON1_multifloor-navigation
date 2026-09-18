"""Mission-owned single-use admission broker for stair command ownership."""

from __future__ import annotations

from dataclasses import dataclass
import secrets
import threading
import time
from typing import Callable, Protocol

import rospy
from stair_supervisor.srv import (
    ValidateStairAdmission,
    ValidateStairAdmissionRequest,
    ValidateStairAdmissionResponse,
)

from mission_manager.stair_entry_gate import StairEntryDecision


@dataclass(frozen=True)
class StairAdmissionContext:
    __slots__ = ("stair_id", "direction", "ownership_epoch")
    stair_id: str
    direction: int
    ownership_epoch: int


@dataclass(frozen=True)
class StairAdmissionRequest:
    __slots__ = ("token", "stair_id", "direction", "ownership_epoch")
    token: str
    stair_id: str
    direction: int
    ownership_epoch: int


class StairAdmissionIssuer(Protocol):
    def issue(
        self,
        context: StairAdmissionContext,
        recheck: Callable[[], StairEntryDecision],
    ) -> str: ...

    def clear(self, token: str) -> None: ...


@dataclass(frozen=True)
class _ActiveAdmission:
    __slots__ = ("token", "context", "expires_at", "recheck")
    token: str
    context: StairAdmissionContext
    expires_at: float
    recheck: Callable[[], StairEntryDecision]


class StairAdmissionBroker:
    """Mint and atomically consume at most one short-lived stair grant."""

    def __init__(
        self,
        clock: Callable[[], float] = time.monotonic,
        lifetime_sec: float = 1.0,
        token_factory: Callable[[], str] = lambda: secrets.token_urlsafe(32),
    ) -> None:
        self._clock = clock
        self._lifetime_sec = lifetime_sec
        self._token_factory = token_factory
        self._lock = threading.Lock()
        self._active: _ActiveAdmission | None = None

    def issue(
        self,
        context: StairAdmissionContext,
        recheck: Callable[[], StairEntryDecision],
    ) -> str:
        token = self._token_factory()
        with self._lock:
            self._active = _ActiveAdmission(
                token,
                context,
                self._clock() + self._lifetime_sec,
                recheck,
            )
        return token

    def validate(self, request: StairAdmissionRequest) -> StairEntryDecision:
        with self._lock:
            active = self._active
            if active is None or request.token != active.token:
                return StairEntryDecision(False, "stair admission token is missing or unknown")
            self._active = None
        if self._clock() > active.expires_at:
            return StairEntryDecision(False, "stair admission token expired")
        expected = active.context
        if (request.stair_id, request.direction, request.ownership_epoch) != (
            expected.stair_id,
            expected.direction,
            expected.ownership_epoch,
        ):
            return StairEntryDecision(False, "stair admission context mismatch")
        return active.recheck()

    def clear(self, token: str) -> None:
        with self._lock:
            if self._active is not None and self._active.token == token:
                self._active = None


class RosStairAdmissionBroker:
    """Expose the broker through the stair-supervisor-owned ROS service type."""

    def __init__(self, service_name: str, lifetime_sec: float) -> None:
        self._broker = StairAdmissionBroker(lifetime_sec=lifetime_sec)
        self._service = rospy.Service(
            service_name,
            ValidateStairAdmission,
            self._validate,
        )

    def issue(
        self,
        context: StairAdmissionContext,
        recheck: Callable[[], StairEntryDecision],
    ) -> str:
        return self._broker.issue(context, recheck)

    def clear(self, token: str) -> None:
        self._broker.clear(token)

    def _validate(
        self,
        request: ValidateStairAdmissionRequest,
    ) -> ValidateStairAdmissionResponse:
        decision = self._broker.validate(
            StairAdmissionRequest(
                request.admission_token,
                request.stair_id,
                int(request.direction),
                int(request.ownership_epoch),
            )
        )
        return ValidateStairAdmissionResponse(decision.accepted, decision.reason)
