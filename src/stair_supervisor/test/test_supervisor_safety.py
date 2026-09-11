from __future__ import annotations

from pathlib import Path
import sys
import threading
from typing import List
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stair_supervisor.configuration import Direction
from stair_supervisor.robot_transport import TransportFault
from stair_supervisor.stair_evidence import EvidenceReport
from stair_supervisor.supervisor import (
    Phase,
    ResultCode,
    StairGoal,
    StairSupervisor,
    SupervisorState,
)

from test_supervisor import FakeClock, FakeTransport, ScriptedEvidence, make_configuration


class CloseFaultTransport(FakeTransport):
    def close(self) -> None:
        super().close()
        raise TransportFault("close interrupted")


class BlockingTransport(FakeTransport):
    def __init__(self) -> None:
        super().__init__()
        self.nav_tick_entered = threading.Event()
        self.release_nav_tick = threading.Event()
        self.handoff_overtook_nav = threading.Event()

    def update_twist(self, linear_mps: float, angular_radps: float) -> None:
        if linear_mps != 0.0 or angular_radps != 0.0:
            self.nav_tick_entered.set()
            self.release_nav_tick.wait(timeout=1.0)
        elif self.nav_tick_entered.is_set() and not self.release_nav_tick.is_set():
            self.handoff_overtook_nav.set()
        super().update_twist(linear_mps, angular_radps)


class FaultAfterCommandEvidence(ScriptedEvidence):
    def __init__(self) -> None:
        super().__init__(observations_before_true=0)
        self._flight_evaluations = 0

    def evaluate(self, phase: Phase, now: float) -> EvidenceReport:
        if phase is not Phase.FORWARD_SEGMENT_1:
            return super().evaluate(phase, now)
        self._flight_evaluations += 1
        faulted = self._flight_evaluations > 1
        return EvidenceReport(
            phase,
            False,
            faulted,
            "recorded odometry gap/jump" if faulted else "awaiting replay sample",
            0.1,
            1.0,
            0.0,
        )


class StairSupervisorSafetyTest(unittest.TestCase):
    def test_inflight_nav_tick_finishes_before_stair_handoff(self) -> None:
        # Given: a NAV wire tick paused after ownership validation.
        clock = FakeClock()
        transport = BlockingTransport()
        supervisor = StairSupervisor(
            configuration=make_configuration(),
            transport=transport,
            evidence=ScriptedEvidence(observations_before_true=0),
            clock=clock,
            feedback=lambda _report: None,
            nav_freshness_sec=0.25,
        )
        supervisor.start()
        supervisor.accept_navigation(0.2, 0.0)
        nav_thread = threading.Thread(target=supervisor.stream_navigation)
        nav_thread.start()
        self.assertTrue(transport.nav_tick_entered.wait(timeout=1.0))

        # When: STAIR requests ownership while that tick is in flight.
        stair_thread = threading.Thread(
            target=lambda: supervisor.traverse(
                StairGoal("test_up", Direction.UP),
                lambda: False,
            )
        )
        stair_thread.start()
        transport.handoff_overtook_nav.wait(timeout=0.05)
        transport.release_nav_tick.set()
        nav_thread.join(timeout=1.0)
        stair_thread.join(timeout=1.0)

        # Then: the NAV command is on the wire before STAIR mode begins.
        self.assertLess(
            transport.events.index(("update", 0.2, 0.0)),
            transport.events.index(("stair", True)),
        )

    def test_cancel_from_each_moving_phase_waits_for_next_safe_checkpoint(self) -> None:
        cases = (
            (Phase.ALIGN, Phase.LANDING),
            (Phase.FORWARD_SEGMENT_1, Phase.LANDING),
            (Phase.TURN_TO_NEXT_FLIGHT, Phase.EXIT_CONFIRM),
            (Phase.FORWARD_SEGMENT_2, Phase.EXIT_CONFIRM),
        )
        for cancel_phase, checkpoint in cases:
            with self.subTest(cancel_phase=cancel_phase):
                # Given: cancellation latches while one moving phase owns commands.
                evidence = ScriptedEvidence(observations_before_true=0)
                clock = FakeClock()
                transport = FakeTransport()
                phases: List[Phase] = []
                cancelled = False
                supervisor = StairSupervisor(
                    configuration=make_configuration(),
                    transport=transport,
                    evidence=evidence,
                    clock=clock,
                    feedback=lambda report: phases.append(report.phase),
                    nav_freshness_sec=0.25,
                )

                def latch(phase: Phase) -> None:
                    nonlocal cancelled
                    cancelled = cancelled or phase is cancel_phase

                evidence.on_observe = latch
                supervisor.start()

                # When: execution reaches the next safe checkpoint.
                result = supervisor.traverse(StairGoal("test_up", Direction.UP), lambda: cancelled)

                # Then: PREEMPTED occurs there and no later phase command is emitted.
                self.assertTrue(result.cancelled)
                self.assertEqual(phases[-1], checkpoint)
                self.assertEqual(supervisor.state, SupervisorState.NAV)

    def test_evidence_fault_after_motion_allows_no_later_nonzero_command(self) -> None:
        # Given: replay evidence rejects a recorded gap/jump after one flight command.
        clock = FakeClock()
        transport = FakeTransport()
        supervisor = StairSupervisor(
            configuration=make_configuration(),
            transport=transport,
            evidence=FaultAfterCommandEvidence(),
            clock=clock,
            feedback=lambda _report: None,
            nav_freshness_sec=0.25,
        )
        supervisor.start()

        # When: the discontinuity faults the active flight.
        result = supervisor.traverse(StairGoal("test_up", Direction.UP), lambda: False)

        # Then: the last nonzero is followed only by zero/close events.
        self.assertEqual(result.code, ResultCode.STAIR_FAILED)
        self.assertEqual(supervisor.state, SupervisorState.FAULT)
        last_nonzero = max(
            index
            for index, event in enumerate(transport.events)
            if event[0] == "update" and event[1:] != (0.0, 0.0)
        )
        later_updates = [event for event in transport.events[last_nonzero + 1:] if event[0] == "update"]
        self.assertTrue(later_updates)
        self.assertTrue(all(event[1:] == (0.0, 0.0) for event in later_updates))
    def test_cancel_during_safe_evidence_wait_preempts_without_fault(self) -> None:
        # Given: VERIFY_ENTRY evidence remains false until after cancellation.
        evidence = ScriptedEvidence(observations_before_true=100)
        clock = FakeClock()
        transport = FakeTransport()
        phases: List[Phase] = []
        cancelled = False
        supervisor = StairSupervisor(
            configuration=make_configuration(timeout_sec=0.05),
            transport=transport,
            evidence=evidence,
            clock=clock,
            feedback=lambda report: phases.append(report.phase),
            nav_freshness_sec=0.25,
        )
        evidence.on_observe = lambda _phase: set_cancelled()

        def set_cancelled() -> None:
            nonlocal cancelled
            cancelled = True

        supervisor.start()

        # When: cancellation arrives while waiting at the safe checkpoint.
        result = supervisor.traverse(
            StairGoal("test_up", Direction.UP),
            lambda: cancelled,
        )

        # Then: the action cancels through WALK instead of timing out to FAULT.
        self.assertTrue(result.cancelled)
        self.assertEqual(phases, [Phase.VERIFY_ENTRY])
        self.assertEqual(supervisor.state, SupervisorState.NAV)
        self.assertEqual(transport.events.count(("stair", False)), 1)

    def test_close_interruption_is_latched_and_shutdown_is_idempotent(self) -> None:
        # Given: a ready sole transport whose close operation is interrupted.
        clock = FakeClock()
        transport = CloseFaultTransport()
        supervisor = StairSupervisor(
            configuration=make_configuration(),
            transport=transport,
            evidence=ScriptedEvidence(),
            clock=clock,
            feedback=lambda _report: None,
            nav_freshness_sec=0.25,
        )
        supervisor.start()

        # When: shutdown is requested repeatedly.
        supervisor.shutdown()
        supervisor.shutdown()

        # Then: close failure cannot escape or restore NAV, and close is attempted once.
        self.assertEqual(supervisor.state, SupervisorState.FAULT)
        self.assertEqual(transport.events.count(("close",)), 1)
        self.assertIn(("update", 0.0, 0.0), transport.events)


if __name__ == "__main__":
    unittest.main()
