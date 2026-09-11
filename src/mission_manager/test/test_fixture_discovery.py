from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "test"))

from fixture_support import load_directed_graph  # noqa: E402
from mock_peers import FakeActionPeer  # noqa: E402


class MissionFixtureDiscoveryTest(unittest.TestCase):
    def test_asymmetric_route_fixture_drives_action_peer(self) -> None:
        # Given: a route fixture and non-node action peer.
        graph = load_directed_graph()
        peer = FakeActionPeer(("SUCCEEDED",))

        # When: the mission destination is submitted.
        state = peer.send_goal(graph.floors[-1])

        # Then: the peer records the roof goal and deterministic result.
        self.assertEqual(peer.goals, ["roof"])
        self.assertEqual(state, "SUCCEEDED")

    def test_action_peer_exhaustion_is_explicit(self) -> None:
        # Given: a peer with no queued terminal state.
        peer = FakeActionPeer(())

        # When/Then: an unconfigured action cannot report false success.
        with self.assertRaisesRegex(RuntimeError, "no fake action"):
            peer.send_goal("roof")


if __name__ == "__main__":
    unittest.main()
