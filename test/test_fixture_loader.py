from pathlib import Path
import shutil
import tempfile
import unittest

from fixture_support import (
    FIXTURE_ROOT,
    FixtureError,
    assert_fresh,
    inspect_fixtures,
    load_directed_graph,
    load_frozen_clock,
    load_localization_cases,
    load_websocket_transcript,
    temporary_rosbag_output,
)


class FixtureLoaderTest(unittest.TestCase):
    def test_graph_is_asymmetric_when_loaded(self) -> None:
        # Given: the directed 3F, 4F, and roof graph fixture.
        graph = load_directed_graph()

        # When: directed endpoint pairs are inspected.
        endpoints = {(edge.source, edge.target) for edge in graph.edges}

        # Then: upward routing exists and the roof reverse edge is intentionally absent.
        self.assertEqual(graph.floors, ("3F", "4F", "roof"))
        self.assertIn(("3F", "4F"), endpoints)
        self.assertIn(("4F", "roof"), endpoints)
        self.assertNotIn(("roof", "4F"), endpoints)

    def test_all_map_pairs_are_reported_when_corpus_is_inspected(self) -> None:
        # Given: the complete fixture corpus.
        # When: the typed loader inspects every fixture family.
        summary = inspect_fixtures()

        # Then: all tiny maps and non-map fixtures are discoverable.
        self.assertEqual(summary.map_names, ("3F", "4F", "roof"))
        self.assertEqual(summary.localization_case_count, 7)
        self.assertEqual(summary.websocket_frame_count, 5)

    def test_valid_and_invalid_localization_inputs_are_distinct(self) -> None:
        # Given: valid and invalid tag, AMCL, and TF samples.
        cases = load_localization_cases()

        # When/Then: each invalid sample violates a different adapter boundary.
        self.assertGreater(cases.tag_valid.id, 0)
        self.assertLess(cases.tag_invalid.id, 0)
        self.assertEqual(cases.amcl_valid.frame_id, "map")
        self.assertNotEqual(cases.amcl_invalid.frame_id, "map")
        self.assertNotEqual(cases.tf_valid.parent, cases.tf_valid.child)
        self.assertEqual(cases.tf_invalid.parent, cases.tf_invalid.child)

    def test_amcl_inputs_include_pose_covariance_when_loaded(self) -> None:
        # Given: valid and invalid AMCL fixture samples.
        cases = load_localization_cases()

        # When: consumers inspect pose covariance.
        covariances = (cases.amcl_valid.covariance, cases.amcl_invalid.covariance)

        # Then: both samples expose complete 6x6 covariance arrays.
        self.assertEqual(tuple(len(covariance) for covariance in covariances), (36, 36))
        self.assertTrue(all(value >= 0.0 for covariance in covariances for value in covariance))

    def test_stale_tf_stamp_is_rejected_against_frozen_clock(self) -> None:
        # Given: a frozen clock and the intentionally stale TF input.
        clock = load_frozen_clock()
        cases = load_localization_cases()
        stamp_ns = cases.tf_stale.stamp_ns

        # When/Then: freshness checking fails deterministically.
        with self.assertRaisesRegex(FixtureError, "stale timestamp"):
            assert_fresh(stamp_ns, clock, FIXTURE_ROOT / "inputs/localization_cases.json")

    def test_malformed_graph_is_rejected_when_loaded_from_isolated_root(self) -> None:
        # Given: an isolated fixture root with malformed graph JSON.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            graph_path = root / "graphs/asymmetric_3f_4f_roof.json"
            graph_path.parent.mkdir(parents=True)
            graph_path.write_text("{malformed", encoding="utf-8")

            # When/Then: parsing reports the fixture path and malformed boundary.
            with self.assertRaisesRegex(FixtureError, "invalid JSON"):
                load_directed_graph(root)

    def test_websocket_transcript_preserves_order_when_loaded(self) -> None:
        # Given: the robot session transcript.
        # When: NDJSON frames are loaded.
        frames = load_websocket_transcript()

        # Then: timestamps and directions preserve the recorded sequence.
        self.assertEqual(tuple(frame.at_ns for frame in frames), tuple(sorted(frame.at_ns for frame in frames)))
        self.assertEqual(frames[0].direction, "client_to_robot")
        self.assertEqual(frames[-1].direction, "robot_to_client")

    def test_stale_websocket_frame_is_rejected_against_fixture_clock(self) -> None:
        # Given: an isolated transcript containing one frame older than the clock limit.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copytree(FIXTURE_ROOT / "clock", root / "clock")
            transcript = root / "websocket/robot_session.ndjson"
            transcript.parent.mkdir(parents=True)
            transcript.write_text(
                '{"at_ns":1699999990000000000,"direction":"robot_to_client","payload":{"ok":true}}\n',
                encoding="utf-8",
            )

            # When/Then: transcript loading rejects stale state deterministically.
            with self.assertRaisesRegex(FixtureError, "stale timestamp"):
                load_websocket_transcript(root)

    def test_malformed_websocket_direction_is_rejected_when_loaded(self) -> None:
        # Given: an isolated fresh frame with an unsupported direction.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copytree(FIXTURE_ROOT / "clock", root / "clock")
            transcript = root / "websocket/robot_session.ndjson"
            transcript.parent.mkdir(parents=True)
            transcript.write_text(
                '{"at_ns":1700000009000000000,"direction":"sideways","payload":{"ok":true}}\n',
                encoding="utf-8",
            )

            # When/Then: transcript schema parsing rejects the malformed frame.
            with self.assertRaisesRegex(FixtureError, "direction"):
                load_websocket_transcript(root)

    def test_rosbag_output_is_namespaced_and_removed_after_context(self) -> None:
        # Given: a test-scoped temporary rosbag convention.
        with temporary_rosbag_output("route/3F to roof") as output_path:
            # When: a recorder writes its test artifact.
            output_path.write_bytes(b"#ROSBAG TEST\n")
            directory = output_path.parent

            # Then: the path is isolated and uses the final rosbag suffix.
            self.assertTrue(output_path.name.endswith(".bag"))
            self.assertIn("route-3F-to-roof", directory.name)
        self.assertFalse(directory.exists())


if __name__ == "__main__":
    unittest.main()
