from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_robot_transport_fakes import (
    FakeClock,
    FakeWebSocket,
    JsonValue,
    make_transport,
    successful_mode_handler,
)
from stair_supervisor.robot_transport import TransportFault


class RobotTransportFaultTest(unittest.TestCase):
    def test_correlated_success_without_expected_walk_status_faults(self) -> None:
        # Given: success responses paired with a state that cannot permit walking.
        clock = FakeClock()

        def wrong_status(request: dict[str, JsonValue], socket: FakeWebSocket) -> None:
            title = request["title"]
            if title == "request_twist":
                return
            assert isinstance(title, str)
            socket.queue(
                {
                    "accid": request["accid"],
                    "title": title.replace("request_", "response_", 1),
                    "timestamp": request["timestamp"],
                    "guid": request["guid"],
                    "data": {"result": "success"},
                }
            )
            socket.queue(
                {
                    "accid": request["accid"],
                    "title": "notify_robot_info",
                    "timestamp": request["timestamp"],
                    "guid": "misleading-status",
                    "data": {"status": "SIT"},
                }
            )

        transport, _ = make_transport(FakeWebSocket(wrong_status), clock)

        # When/Then: correlated success alone cannot arm motion.
        with self.assertRaises(TransportFault):
            transport.start()
        self.assertTrue(transport.faulted)

    def test_timeout_cleans_pending_request_and_latches_fault(self) -> None:
        # Given: a connected peer that never sends a response.
        clock = FakeClock()

        def timeout_handler(request: dict[str, JsonValue], socket: FakeWebSocket) -> None:
            if request["title"] != "request_twist":
                socket.inbound.append(TimeoutError())

        transport, _ = make_transport(FakeWebSocket(timeout_handler), clock)

        # When: startup reaches the bounded request timeout.
        with self.assertRaises(TransportFault):
            transport.start()

        # Then: correlation state is cleaned and the fault is permanent.
        self.assertEqual(transport.pending_request_count, 0)
        self.assertTrue(transport.faulted)

    def test_malformed_response_latches_fault(self) -> None:
        # Given: a peer that replies with valid JSON but an invalid envelope.
        clock = FakeClock()

        def malformed(request: dict[str, JsonValue], socket: FakeWebSocket) -> None:
            if request["title"] != "request_twist":
                socket.inbound.append(json.dumps({"title": "response_stand_mode"}))

        transport, _ = make_transport(FakeWebSocket(malformed), clock)

        # When/Then: malformed input is rejected and faulted at the boundary.
        with self.assertRaises(TransportFault):
            transport.start()
        self.assertTrue(transport.faulted)

    def test_stale_post_request_status_cannot_arm_motion(self) -> None:
        # Given: a correlated success plus a status generated before its request.
        clock = FakeClock()

        def stale_status(request: dict[str, JsonValue], socket: FakeWebSocket) -> None:
            title = request["title"]
            if title == "request_twist":
                return
            assert isinstance(title, str)
            timestamp = request["timestamp"]
            assert isinstance(timestamp, int)
            socket.queue(
                {
                    "accid": request["accid"],
                    "title": title.replace("request_", "response_", 1),
                    "timestamp": timestamp,
                    "guid": request["guid"],
                    "data": {"result": "success"},
                }
            )
            socket.queue(
                {
                    "accid": request["accid"],
                    "title": "notify_robot_info",
                    "timestamp": timestamp - 1,
                    "guid": "stale-status",
                    "data": {"status": "STAND"},
                }
            )

        transport, _ = make_transport(FakeWebSocket(stale_status), clock)

        # When/Then: timestamp freshness is part of mode verification.
        with self.assertRaises(TransportFault):
            transport.start()

    def test_send_failure_is_tolerated_then_latches_fault_after_outage_budget(self) -> None:
        # Given: a ready peer that fails every twist send.
        clock = FakeClock()

        def fail_twist(request: dict[str, JsonValue], socket: FakeWebSocket) -> None:
            successful_mode_handler(request, socket)
            if request["title"] == "request_twist":
                raise OSError("scripted interruption")

        socket = FakeWebSocket(fail_twist)
        transport, factory = make_transport(socket, clock)
        observed = []
        transport.observe_sent_frames(observed.append)
        transport.start()
        transport.update_twist(0.2, 0.0)
        successful_frame_count = len(observed)

        # When: the first stream tick fails inside the outage budget.
        transport.send_current()

        # Then: the transport stays ready and no false-success frame is observed.
        self.assertEqual(transport.state, "READY")
        self.assertEqual(len(observed), successful_frame_count)

        # When: time advances beyond the outage budget and another tick fails.
        clock.sleep(0.26)
        with self.assertRaises(TransportFault):
            transport.send_current()
        sent_at_fault = len(socket.sent)
        for _ in range(3):
            with self.assertRaises(TransportFault):
                transport.start()
            with self.assertRaises(TransportFault):
                transport.update_twist(0.2, 0.0)

        # Then: interruption remains latched with no reconnect or command replay.
        self.assertEqual(factory.calls, 1)
        self.assertEqual(len(socket.sent), sent_at_fault)
        self.assertEqual(len(observed), successful_frame_count)
        self.assertNotIn(socket.sent_payloads[-1], observed)

    def test_disconnect_fault_prevents_reconnect_or_resume(self) -> None:
        # Given: a peer that disconnects during the first mode request.
        clock = FakeClock()

        def disconnect(request: dict[str, JsonValue], socket: FakeWebSocket) -> None:
            if request["title"] != "request_twist":
                socket.inbound.append(None)

        socket = FakeWebSocket(disconnect)
        transport, factory = make_transport(socket, clock)

        # When: startup observes the disconnect, then callers repeatedly retry.
        for _ in range(3):
            with self.assertRaises(TransportFault):
                transport.start()
        sent_at_fault = len(socket.sent)
        with self.assertRaises(TransportFault):
            transport.update_twist(0.1, 0.0)

        # Then: no reconnect and no later command are possible.
        self.assertEqual(factory.calls, 1)
        self.assertEqual(len(socket.sent), sent_at_fault)


if __name__ == "__main__":
    unittest.main()
