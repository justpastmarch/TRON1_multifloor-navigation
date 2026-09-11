from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "test"))

from fixture_support import inspect_fixtures, load_websocket_transcript  # noqa: E402
from mock_peers import FakeWebSocketPeer  # noqa: E402


class CatkinFixtureDiscoveryTest(unittest.TestCase):
    def test_fixture_corpus_is_real_when_catkin_discovers_tests(self) -> None:
        # Given: catkin invokes this package-local test registration.
        # When: the shared root fixture corpus is parsed through its public loader.
        summary = inspect_fixtures()

        # Then: discovery executes assertions over real fixture content.
        self.assertEqual(summary.floors, ("3F", "4F", "roof"))
        self.assertEqual(summary.websocket_frame_count, 5)

    def test_fresh_transcript_drives_websocket_peer_without_network(self) -> None:
        # Given: freshness-checked frames and an in-memory WebSocket peer.
        peer = FakeWebSocketPeer(load_websocket_transcript())

        # When: stair supervision consumes one robot frame.
        frame = peer.receive()

        # Then: the deterministic direction is available without a socket or node.
        self.assertEqual(frame.direction, "client_to_robot")

    def test_fixture_helpers_are_absent_from_install_rules(self) -> None:
        # Given: every production package's CMake install inventory.
        cmake_sources = tuple(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "src").glob("*/CMakeLists.txt")
        )

        # When/Then: test helpers and fixture directories are never deployed.
        for source in cmake_sources:
            self.assertNotIn("fixture_support.py", source)
            self.assertNotIn("mock_peers.py", source)
            self.assertNotIn("mock_peer_harness.py", source)
            self.assertNotIn("DIRECTORY test", source)


if __name__ == "__main__":
    unittest.main()
