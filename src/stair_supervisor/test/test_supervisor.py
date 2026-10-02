from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import Callable, Dict, List
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stair_supervisor.configuration import (  # noqa: E402
    Direction,
    MoveBaseLimits,
    RobotConfiguration,
    StairProfile,
    StairSupervisorConfiguration,
    WebSocketCalibration,
)
from stair_supervisor.robot_transport import TransportFault  # noqa: E402
from stair_supervisor.stair_evidence import EvidenceReport, Phase  # noqa: E402
from stair_supervisor.supervisor import (  # noqa: E402
    ResultCode,
    StairGoal,
    StairSupervisor,
    SupervisorState,
)
from stair_supervisor.supervisor_types import AdmissionDecision  # noqa: E402


class FakeAdmission:
    def __init__(self, decision: AdmissionDecision | None = None) -> None:
        self.decision = decision or AdmissionDecision(True, False, "admitted")
        self.calls: List[tuple[StairGoal, int]] = []

    def validate(self, goal: StairGoal, ownership_epoch: int) -> AdmissionDecision:
        self.calls.append((goal, ownership_epoch))
        return self.decision


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeTransport:
    def __init__(self) -> None:
        self.events: List[tuple] = []
        self.stream_period_sec = 0.025
        self.fail_on_stair = False

    def start(self) -> None:
        self.events.append(("start",))

    def update_twist(self, linear_mps: float, angular_radps: float) -> None:
        self.events.append(("update", linear_mps, angular_radps))

    def send_current(self) -> None:
        self.events.append(("send",))

    def request_stair_mode(self, enabled: bool) -> None:
        self.events.append(("stair", enabled))
        if self.fail_on_stair:
            raise TransportFault("socket disconnected")

    def close(self) -> None:
        self.events.append(("close",))


class ScriptedEvidence:
    def __init__(self, observations_before_true: int = 1) -> None:
        self._threshold = observations_before_true
        self._counts: Dict[Phase, int] = {}
        self.on_observe: Callable[[Phase], None] = lambda _phase: None
        self._profile: StairProfile | None = None
        self._started_at = 0.0

    def arm(self, profile: StairProfile, started_at: float, **_options) -> None:
        self._profile = profile
        self._started_at = started_at
        self._counts.clear()

    def begin_phase(self, phase: Phase, _now: float) -> None:
        self._counts[phase] = 0

    def evaluate(self, phase: Phase, now: float, **_options) -> EvidenceReport:
        self.on_observe(phase)
        count = self._counts.get(phase, 0) + 1
        self._counts[phase] = count
        profile = self._profile
        timed_out = profile is not None and now - self._started_at > profile.timeout_sec
        complete = count > self._threshold
        return EvidenceReport(
            phase,
            complete and not timed_out,
            timed_out,
            "scripted timeout" if timed_out else "scripted evidence",
            float(count),
            float(self._threshold + 1),
            0.0,
        )


def make_configuration(
    *,
    enabled: bool = True,
    timeout_sec: float = 2.0,
    linear_speed: float = 0.12,
) -> StairSupervisorConfiguration:
    profile = StairProfile(
        id="test_up",
        direction=Direction.UP,
        enabled=enabled,
        linear_speed=linear_speed,
        angular_speed=0.25,
        alignment_yaw_rad=0.40,
        flight_1_distance_m=1.00,
        landing_dwell_sec=0.20,
        landing_turn_yaw_rad=0.50,
        flight_2_distance_m=0.80,
        exit_dwell_sec=0.20,
        distance_tolerance_m=0.02,
        yaw_tolerance_rad=0.02,
        sensor_freshness_sec=0.20,
        max_sample_gap_sec=0.15,
        max_odom_step_m=0.40,
        max_yaw_step_rad=0.30,
        timeout_sec=timeout_sec,
    )
    robot = RobotConfiguration(
        move_base=MoveBaseLimits(0.35, 1.0, 0.2, 0.4, 2.0),
        websocket_full_scale=WebSocketCalibration(0.55, 1.8),
        command_topics=("/navigation/cmd_vel",),
    )
    return StairSupervisorConfiguration((profile,), robot)


class StairSupervisorTest(unittest.TestCase):
    def make_supervisor(
        self,
        *,
        enabled: bool = True,
        timeout_sec: float = 2.0,
        evidence: ScriptedEvidence | None = None,
        turn_linear_mps: float = 0.0,
        linear_speed: float = 0.12,
    ) -> tuple[StairSupervisor, FakeTransport, FakeClock, List[Phase]]:
        clock = FakeClock()
        transport = FakeTransport()
        phases: List[Phase] = []
        supervisor = StairSupervisor(
            configuration=make_configuration(
                enabled=enabled, timeout_sec=timeout_sec, linear_speed=linear_speed
            ),
            transport=transport,
            evidence=evidence or ScriptedEvidence(),
            clock=clock,
            feedback=lambda report: phases.append(report.phase),
            admission=FakeAdmission(),
            nav_freshness_sec=0.25,
            turn_linear_mps=turn_linear_mps,
        )
        return supervisor, transport, clock, phases

    def test_traversal_owns_commands_and_advances_epoch(self) -> None:
        # Given: a ready supervisor with a fresh navigation command.
        evidence = ScriptedEvidence()
        supervisor, transport, _, phases = self.make_supervisor(evidence=evidence)
        supervisor.start()
        supervisor.accept_navigation(0.2, -0.1)
        supervisor.stream_navigation()
        old_epoch = supervisor.ownership_epoch
        evidence.on_observe = lambda _phase: supervisor.accept_navigation(0.3, 0.2)

        # When: a complete stair profile runs while NAV keeps publishing.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "valid"), lambda: False)

        # Then: phases are ordered, NAV was discarded, and the old epoch is stale.
        self.assertEqual(result.code, ResultCode.OK)
        self.assertEqual(list(dict.fromkeys(phases)), [p for p in Phase if p not in (Phase.ROOFTOP_TURN, Phase.FORWARD_SEGMENT_3)])
        self.assertEqual(supervisor.state, SupervisorState.NAV)
        self.assertEqual(supervisor.ownership_epoch, old_epoch + 1)
        stair_start = transport.events.index(("stair", True))
        stair_end = transport.events.index(("stair", False))
        stair_updates = [event for event in transport.events[stair_start:stair_end] if event[0] == "update"]
        self.assertNotIn(("update", 0.3, 0.2), stair_updates)
        supervisor.stream_navigation()
        self.assertEqual(transport.events[-2], ("update", 0.0, 0.0))

    def test_stair_profile_speed_is_not_capped_by_flat_navigation_limit(self) -> None:
        # Given: a stair profile above the flat limit but within WebSocket calibration.
        clock = FakeClock()
        transport = FakeTransport()
        supervisor = StairSupervisor(
            configuration=make_configuration(linear_speed=0.50),
            transport=transport,
            evidence=ScriptedEvidence(),
            clock=clock,
            feedback=lambda _report: None,
            admission=FakeAdmission(),
            nav_freshness_sec=0.25,
        )
        supervisor.start()

        # When: the UP traversal executes its forward phases.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "valid"), lambda: False)

        # Then: the commissioned stair speed reaches the transport unchanged.
        self.assertEqual(result.code, ResultCode.OK)
        self.assertIn(("update", 0.50, 0.0), transport.events)

    def test_rejected_admission_cannot_take_ownership_or_touch_transport(self) -> None:
        # Given: a ready supervisor whose mission admission rejects the token.
        clock = FakeClock()
        transport = FakeTransport()
        supervisor = StairSupervisor(
            configuration=make_configuration(),
            transport=transport,
            evidence=ScriptedEvidence(),
            clock=clock,
            feedback=lambda _report: None,
            admission=FakeAdmission(AdmissionDecision(False, False, "token rejected")),
            nav_freshness_sec=0.25,
        )
        supervisor.start()
        before = list(transport.events)

        # When: a direct child-action goal presents an untrusted token.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "bad"), lambda: False)

        # Then: NAV ownership and the physical command surface remain untouched.
        self.assertEqual(result.code, ResultCode.ENTRY_REJECTED)
        self.assertEqual(supervisor.state, SupervisorState.NAV)
        self.assertEqual(transport.events, before)

    def test_align_is_observable_but_first_nonzero_stair_command_is_forward(self) -> None:
        # Given: a legacy in-memory profile still carrying a nonzero alignment value.
        supervisor, transport, _, phases = self.make_supervisor()
        supervisor.start()

        # When: an admitted traversal passes through ALIGN.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "valid"), lambda: False)

        # Then: ALIGN is reported but cannot issue the removed fixed rotation.
        self.assertEqual(result.code, ResultCode.OK)
        self.assertIn(Phase.ALIGN, phases)
        stair_start = transport.events.index(("stair", True))
        nonzero = [
            event for event in transport.events[stair_start:]
            if event[0] == "update" and event[1:] != (0.0, 0.0)
        ]
        self.assertEqual(nonzero[0], ("update", 0.12, 0.0))

    def test_landing_turn_can_curve_to_the_return_flight(self) -> None:
        # Given: a SIM traversal configured to cross the landing while turning.
        supervisor, transport, _, _ = self.make_supervisor(
            turn_linear_mps=0.20, linear_speed=0.20
        )
        supervisor.start()

        # When: the state machine runs through the landing turn phase.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "valid"), lambda: False)

        # Then: the turn phase emits a forward WebSocket command with angular steering.
        self.assertEqual(result.code, ResultCode.OK)
        self.assertIn(("update", 0.20, 0.25), transport.events)

    def test_disabled_profile_changes_neither_mode_nor_motion(self) -> None:
        # Given: a started supervisor whose selected profile is disabled.
        supervisor, transport, _, _ = self.make_supervisor(enabled=False)
        supervisor.start()

        # When: the disabled traversal is requested.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "valid"), lambda: False)

        # Then: capability rejection precedes every mode or nonzero command.
        self.assertEqual(result.code, ResultCode.CAPABILITY_DISABLED)
        self.assertEqual(supervisor.state, SupervisorState.NAV)
        self.assertFalse(any(event[0] == "stair" for event in transport.events))
        self.assertFalse(any(event[0] == "update" and event[1:] != (0.0, 0.0) for event in transport.events))

    def test_stale_and_nonfinite_navigation_are_zeroed(self) -> None:
        # Given: one accepted command that becomes stale.
        supervisor, transport, clock, _ = self.make_supervisor()
        supervisor.start()
        supervisor.accept_navigation(0.2, 0.1)
        clock.sleep(0.3)

        # When: stale and malformed commands reach stream ticks.
        supervisor.stream_navigation()
        supervisor.accept_navigation(math.nan, 0.1)
        supervisor.stream_navigation()

        # Then: both ticks are complete zero commands.
        updates = [event for event in transport.events if event[0] == "update"]
        self.assertEqual(updates[-2:], [("update", 0.0, 0.0)] * 2)

    def test_timeout_latches_fault_without_retry(self) -> None:
        # Given: evidence that cannot arrive before the profile deadline.
        evidence = ScriptedEvidence(observations_before_true=100)
        supervisor, transport, _, _ = self.make_supervisor(timeout_sec=0.05, evidence=evidence)
        supervisor.start()

        # When: traversal exhausts its bounded deadline.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "valid"), lambda: False)

        # Then: the supervisor faults, zeros once, and never retries stair mode.
        self.assertEqual(result.code, ResultCode.STAIR_FAILED)
        self.assertEqual(supervisor.state, SupervisorState.FAULT)
        self.assertEqual(transport.events.count(("stair", True)), 1)
        self.assertIn(("update", 0.0, 0.0), transport.events)

    def test_cancel_is_honored_at_safe_checkpoint(self) -> None:
        # Given: cancellation asserted during a moving phase.
        evidence = ScriptedEvidence()
        supervisor, transport, _, phases = self.make_supervisor(evidence=evidence)
        supervisor.start()
        cancel_requested = False

        def observe(phase: Phase) -> None:
            nonlocal cancel_requested
            if phase is Phase.FORWARD_SEGMENT_1:
                cancel_requested = True

        evidence.on_observe = observe

        # When: execution reaches the next configured safe checkpoint.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "valid"), lambda: cancel_requested)

        # Then: it preempts at LANDING, returns WALK, and advances ownership.
        self.assertTrue(result.cancelled)
        self.assertEqual(phases[-1], Phase.LANDING)
        self.assertEqual(transport.events[-3:], [("stair", False), ("update", 0.0, 0.0), ("send",)])
        self.assertEqual(supervisor.state, SupervisorState.NAV)

    def test_disconnect_latches_fault_and_shutdown_never_restarts(self) -> None:
        # Given: a transport that disconnects entering stair mode.
        supervisor, transport, _, _ = self.make_supervisor()
        supervisor.start()
        transport.fail_on_stair = True

        # When: traversal fails and shutdown runs repeatedly.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP, "valid"), lambda: False)
        supervisor.shutdown()
        supervisor.shutdown()

        # Then: communication loss is terminal and the sole transport started once.
        self.assertEqual(result.code, ResultCode.COMMUNICATION_LOST)
        self.assertEqual(supervisor.state, SupervisorState.FAULT)
        self.assertEqual(transport.events.count(("start",)), 1)
        self.assertEqual(transport.events.count(("close",)), 1)

    def test_shutdown_sends_zero_before_closing(self) -> None:
        # Given: a ready supervisor.
        supervisor, transport, _, _ = self.make_supervisor()
        supervisor.start()

        # When: the node shuts down.
        supervisor.shutdown()

        # Then: a zero barrier is sent before the socket closes.
        self.assertEqual(transport.events[-3:], [("update", 0.0, 0.0), ("send",), ("close",)])
        self.assertEqual(supervisor.state, SupervisorState.DISARMED)


if __name__ == "__main__":
    unittest.main()
