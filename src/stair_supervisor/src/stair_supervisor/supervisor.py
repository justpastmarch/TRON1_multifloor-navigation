"""Exclusive navigation and stair command ownership state machine."""

from __future__ import annotations

import math
import threading
from typing import Callable

from .configuration import StairProfile, StairSupervisorConfiguration
from .robot_transport import TransportFault
from .stair_evidence import EvidenceReport, Phase
from .supervisor_types import (
    CommandTransport,
    PhaseEvidence,
    ResultCode,
    StairGoal,
    SupervisorClock,
    SupervisorState,
    TraversalResult,
)


class StairSupervisor:
    """Mutable sole owner of one transport and its command epoch."""

    _SAFE_CHECKPOINTS = frozenset(
        (Phase.VERIFY_ENTRY, Phase.LANDING, Phase.EXIT_CONFIRM)
    )

    def __init__(
        self,
        configuration: StairSupervisorConfiguration,
        transport: CommandTransport,
        evidence: PhaseEvidence,
        clock: SupervisorClock,
        feedback: Callable[[EvidenceReport], None],
        nav_freshness_sec: float,
        turn_linear_mps: float = 0.0,
    ) -> None:
        self._configuration = configuration
        self._profiles = {profile.id: profile for profile in configuration.profiles}
        self._transport = transport
        self._evidence = evidence
        self._clock = clock
        self._feedback = feedback
        self._nav_freshness_sec = nav_freshness_sec
        self._turn_linear_mps = turn_linear_mps
        self._state = SupervisorState.DISARMED
        self._ownership_epoch = 0
        self._latest_nav: tuple[float, float, float, int] | None = None
        self._state_lock = threading.RLock()
        self._traversal_lock = threading.Lock()
        self._closed = False

    @property
    def state(self) -> SupervisorState:
        with self._state_lock:
            return self._state

    @property
    def ownership_epoch(self) -> int:
        with self._state_lock:
            return self._ownership_epoch

    def start(self) -> None:
        """Open the sole command session and arm NAV forwarding."""
        try:
            self._transport.start()
        except TransportFault:
            with self._state_lock:
                self._state = SupervisorState.FAULT
            self._close_transport()
            raise
        with self._state_lock:
            self._state = SupervisorState.NAV

    def accept_navigation(self, linear_mps: float, angular_radps: float) -> None:
        """Capture a finite NAV command in the currently owned epoch."""
        with self._state_lock:
            if self._state is not SupervisorState.NAV:
                return
            if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
                self._latest_nav = None
                return
            self._latest_nav = (
                linear_mps,
                angular_radps,
                self._clock.monotonic(),
                self._ownership_epoch,
            )

    def stream_navigation(self) -> None:
        """Emit one NAV tick or a complete zero for absent/stale input."""
        try:
            with self._state_lock:
                if self._state is not SupervisorState.NAV:
                    return
                command = self._latest_nav
                linear, angular = 0.0, 0.0
                if command is not None:
                    command_linear, command_angular, received_at, command_epoch = command
                    fresh = self._clock.monotonic() - received_at <= self._nav_freshness_sec
                    if fresh and command_epoch == self._ownership_epoch:
                        linear, angular = command_linear, command_angular
                self._transport.update_twist(linear, angular)
                self._transport.send_current()
        except TransportFault:
            self._latch_fault()
            raise

    def traverse(
        self,
        goal: StairGoal,
        cancellation_requested: Callable[[], bool],
    ) -> TraversalResult:
        """Execute one configured profile without yielding command ownership."""
        if not self._profiles:
            return TraversalResult(
                ResultCode.CAPABILITY_DISABLED,
                "stair capability is disabled",
            )
        if not self._traversal_lock.acquire(blocking=False):
            return TraversalResult(ResultCode.BUSY, "another traversal owns commands")
        try:
            profile = self._profiles.get(goal.stair_id)
            if profile is None or profile.direction is not goal.direction:
                return TraversalResult(ResultCode.INVALID_GOAL, "unknown stair or direction")
            if not profile.enabled:
                return TraversalResult(
                    ResultCode.CAPABILITY_DISABLED,
                    "stair profile is disabled",
                )
            with self._state_lock:
                if self._state is not SupervisorState.NAV:
                    return TraversalResult(ResultCode.BUSY, "supervisor is not in NAV")
                self._state = SupervisorState.STAIR
                self._latest_nav = None
            return self._execute_profile(profile, cancellation_requested)
        finally:
            self._traversal_lock.release()

    def _execute_profile(
        self,
        profile: StairProfile,
        cancellation_requested: Callable[[], bool],
    ) -> TraversalResult:
        started_at = self._clock.monotonic()
        self._evidence.arm(profile, started_at)
        cancel_pending = False
        try:
            self._zero_barrier()
            self._transport.request_stair_mode(True)
            for phase in Phase:
                self._evidence.begin_phase(phase, self._clock.monotonic())
                while True:
                    report = self._evidence.evaluate(phase, self._clock.monotonic())
                    self._feedback(report)
                    cancel_pending = cancel_pending or cancellation_requested()
                    if phase in self._SAFE_CHECKPOINTS and cancel_pending:
                        return self._finish(cancelled=True)
                    if report.faulted:
                        self._zero_barrier()
                        self._latch_fault()
                        return TraversalResult(ResultCode.STAIR_FAILED, report.detail)
                    if report.complete:
                        break
                    linear, angular = self._phase_command(phase, profile)
                    self._transport.update_twist(linear, angular)
                    self._transport.send_current()
                    self._clock.sleep(self._transport.stream_period_sec)
            return self._finish(cancelled=False)
        except TransportFault as error:
            self._latch_fault()
            return TraversalResult(ResultCode.COMMUNICATION_LOST, str(error))

    def _phase_command(
        self,
        phase: Phase,
        profile: StairProfile,
    ) -> tuple[float, float]:
        linear = min(
            profile.linear_speed,
            self._configuration.robot.websocket_full_scale.linear_mps,
        )
        angular = min(
            profile.angular_speed,
            self._configuration.robot.websocket_full_scale.angular_radps,
        )
        alignment_angular = math.copysign(angular, profile.alignment_yaw_rad)
        first_linear = math.copysign(linear, profile.flight_1_distance_m)
        landing_angular = math.copysign(angular, profile.landing_turn_yaw_rad)
        second_linear = math.copysign(linear, profile.flight_2_distance_m)
        commands = {
            Phase.VERIFY_ENTRY: (0.0, 0.0),
            Phase.ALIGN: (0.0, alignment_angular),
            Phase.FORWARD_SEGMENT_1: (first_linear, 0.0),
            Phase.LANDING: (0.0, 0.0),
            Phase.TURN_TO_NEXT_FLIGHT: (min(self._turn_linear_mps, linear), landing_angular),
            Phase.FORWARD_SEGMENT_2: (second_linear, 0.0),
            Phase.EXIT_CONFIRM: (0.0, 0.0),
        }
        return commands[phase]

    def _finish(self, *, cancelled: bool) -> TraversalResult:
        self._zero_barrier()
        self._transport.request_stair_mode(False)
        self._zero_barrier()
        with self._state_lock:
            self._ownership_epoch += 1
            self._latest_nav = None
            self._state = SupervisorState.NAV
        if cancelled:
            return TraversalResult(ResultCode.STAIR_FAILED, "cancelled", cancelled=True)
        return TraversalResult(ResultCode.OK, "stair traversal complete")

    def _zero_barrier(self) -> None:
        self._transport.update_twist(0.0, 0.0)
        self._transport.send_current()

    def _latch_fault(self) -> None:
        with self._state_lock:
            self._state = SupervisorState.FAULT
            self._latest_nav = None
        self._close_transport()

    def _close_transport(self) -> None:
        if not self._closed:
            self._closed = True
            try:
                self._transport.close()
            except TransportFault:
                with self._state_lock:
                    self._state = SupervisorState.FAULT
                    self._latest_nav = None

    def shutdown(self) -> None:
        """Stop motion before releasing the sole transport session."""
        if not self._closed:
            try:
                self._zero_barrier()
            except TransportFault:
                with self._state_lock:
                    self._state = SupervisorState.FAULT
            self._close_transport()
        with self._state_lock:
            if self._state is not SupervisorState.FAULT:
                self._state = SupervisorState.DISARMED
