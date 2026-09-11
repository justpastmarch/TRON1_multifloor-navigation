import unittest

from fixture_support import load_frozen_clock, load_websocket_transcript
from mock_peers import FakeActionPeer, FakeClock, FakeServicePeer, FakeWebSocketPeer


class MockPeerTest(unittest.TestCase):
    def test_fake_clock_advances_only_when_requested(self) -> None:
        # Given: a fake clock seeded from fixture data.
        clock = FakeClock(load_frozen_clock().now_ns)

        # When: deterministic time is advanced.
        clock.advance_ns(250_000_000)

        # Then: callers observe exactly the requested delta.
        self.assertEqual(clock.now_ns(), 1_700_000_010_250_000_000)

    def test_action_peer_records_goal_and_returns_queued_state(self) -> None:
        # Given: a non-node action peer with a terminal response.
        peer = FakeActionPeer(("SUCCEEDED",))

        # When: a consumer submits one goal.
        state = peer.send_goal("roof_landing")

        # Then: the goal and deterministic state are observable.
        self.assertEqual(peer.goals, ["roof_landing"])
        self.assertEqual(state, "SUCCEEDED")

    def test_service_peer_records_request_and_returns_named_response(self) -> None:
        # Given: a non-node service peer with one response.
        peer = FakeServicePeer({"change_map": "accepted"})

        # When: a consumer calls the service.
        response = peer.call("change_map", "4F")

        # Then: request order and response are deterministic.
        self.assertEqual(peer.requests, [("change_map", "4F")])
        self.assertEqual(response, "accepted")

    def test_websocket_peer_replays_and_captures_without_network(self) -> None:
        # Given: an in-memory peer backed by the recorded transcript.
        peer = FakeWebSocketPeer(load_websocket_transcript())

        # When: a consumer receives and sends one payload.
        frame = peer.receive()
        peer.send('{"ack":true}')

        # Then: no socket is needed to inspect both directions.
        self.assertEqual(frame.direction, "client_to_robot")
        self.assertEqual(peer.sent_payloads, ['{"ack":true}'])


if __name__ == "__main__":
    unittest.main()
