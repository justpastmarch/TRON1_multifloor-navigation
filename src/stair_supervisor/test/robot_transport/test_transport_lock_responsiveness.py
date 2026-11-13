"""A slow socket send must not stall command-state updates or manual status."""
import threading
import unittest

from .test_robot_transport_fakes import FakeClock, FakeWebSocket, make_transport, successful_mode_handler


class SlowSendTest(unittest.TestCase):
    def setUp(self):
        self.socket = FakeWebSocket(successful_mode_handler)
        self.transport, _ = make_transport(self.socket, FakeClock())
        self.transport.start()
        self.entered, self.release = threading.Event(), threading.Event()
        original = self.socket.send

        def blocking_send(payload):
            if '"request_twist"' in payload and not self.release.is_set():
                self.entered.set()
                self.release.wait(5.)
            original(payload)

        self.socket.send = blocking_send
        self.addCleanup(self.release.set)

    def start_blocked_send(self, target):
        sender = threading.Thread(target=target, daemon=True)
        sender.start()
        self.assertTrue(self.entered.wait(2.))
        return sender

    def test_update_twist_and_manual_status_do_not_wait_for_socket(self):
        sender = self.start_blocked_send(self.transport.send_current)
        done = threading.Event()

        def control_loop():
            self.transport.update_twist(.1, .2)
            self.transport.web_manual_status()
            done.set()

        threading.Thread(target=control_loop, daemon=True).start()
        self.assertTrue(done.wait(1.), "state update blocked behind a socket send")
        self.release.set()
        sender.join(2.)

    def test_ended_manual_lease_is_never_sent_after_its_zero_barrier(self):
        transport = self.transport
        lease = transport.begin_web_manual()
        self.release.set()
        transport.update_web_manual(lease, 1, .8, -.2, .35)
        self.release.clear(); self.entered.clear()
        sender = self.start_blocked_send(transport.send_current)  # snapshot includes manual
        ender = threading.Thread(target=transport.end_web_manual, args=(lease,), daemon=True)
        ender.start()
        self.release.set()
        sender.join(2.); ender.join(2.)
        twists = [m['data'] for m in self.socket.sent if m['title'] == 'request_twist']
        self.assertEqual(twists[-1], dict(x=0., y=0., z=0.))


if __name__ == '__main__':
    unittest.main()
