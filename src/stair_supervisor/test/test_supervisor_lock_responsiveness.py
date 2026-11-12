"""A slow robot send must not stall state readers, yet ownership stays ordered."""
from __future__ import annotations

import threading
import time
import unittest

from test_supervisor import FakeAdmission, FakeClock, FakeTransport, ScriptedEvidence, make_configuration
from stair_supervisor.configuration import Direction
from stair_supervisor.supervisor import ResultCode, StairGoal, StairSupervisor, SupervisorState


class BlockingTransport(FakeTransport):
    """send_current blocks while armed, like a stalled WebSocket write."""

    def __init__(self) -> None:
        super().__init__()
        self.armed = False
        self.entered = threading.Event()
        self.release = threading.Event()

    def send_current(self) -> None:
        if self.armed:
            self.armed = False
            self.entered.set()
            assert self.release.wait(5.0), "test never released the blocked send"
        super().send_current()


def finishes(function, timeout=0.5):
    """Run function in a thread; report whether it returned within timeout."""
    done = threading.Event()
    def run():
        function()
        done.set()
    threading.Thread(target=run, daemon=True).start()
    return done.wait(timeout)


class LockResponsivenessTest(unittest.TestCase):
    def setUp(self):
        self.transport = BlockingTransport()
        self.admission = FakeAdmission()
        self.supervisor = StairSupervisor(
            configuration=make_configuration(),
            transport=self.transport,
            evidence=ScriptedEvidence(),
            clock=FakeClock(),
            feedback=lambda _report: None,
            admission=self.admission,
            nav_freshness_sec=0.25,
        )
        self.supervisor.start()
        self.supervisor.accept_navigation(0.2, 0.0)
        self.transport.events.clear()
        self.addCleanup(self.transport.release.set)

    def block_nav_send(self):
        self.transport.armed = True
        stream = threading.Thread(target=self.supervisor.stream_navigation, daemon=True)
        stream.start()
        self.assertTrue(self.transport.entered.wait(2.0))
        return stream

    def test_state_queries_and_callbacks_do_not_wait_for_a_stalled_send(self):
        stream = self.block_nav_send()
        supervisor = self.supervisor
        for name, call in (
            ('state', lambda: supervisor.state),
            ('ownership_epoch', lambda: supervisor.ownership_epoch),
            ('operator_status', supervisor.operator_status),
            ('accept_manual_input', lambda: supervisor.accept_manual_input(False)),
            ('accept_navigation', lambda: supervisor.accept_navigation(0.1, 0.0)),
        ):
            self.assertTrue(finishes(call), name + ' waited for the robot send')
        self.transport.release.set()
        stream.join(2.0)
        self.assertFalse(stream.is_alive())
        self.assertIn(('update', 0.2, 0.0), self.transport.events)

    def test_ownership_change_waits_for_in_flight_nav_send(self):
        stream = self.block_nav_send()
        result = {}
        traversal = threading.Thread(
            target=lambda: result.setdefault('r', self.supervisor.traverse(
                StairGoal('test_up', Direction.UP, 'valid'), lambda: False)),
            daemon=True)
        traversal.start()
        deadline = time.monotonic() + 2.0
        while not self.admission.calls and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(self.admission.calls, 'traversal never reached admission')
        time.sleep(0.1)
        # The NAV tick decided before the change is still being sent: STAIR must wait.
        self.assertEqual(self.supervisor.state, SupervisorState.NAV)
        self.assertTrue(traversal.is_alive())
        self.transport.release.set()
        stream.join(2.0)
        traversal.join(5.0)
        self.assertFalse(traversal.is_alive())
        self.assertEqual(result['r'].code, ResultCode.OK)
        events = self.transport.events
        nav = events.index(('update', 0.2, 0.0))
        stair_on, stair_off = events.index(('stair', True)), events.index(('stair', False))
        self.assertLess(nav, stair_on)
        self.assertEqual(events[nav + 1], ('send',))
        self.assertNotIn(('update', 0.2, 0.0), events[stair_on:stair_off])


if __name__ == '__main__':
    unittest.main()
