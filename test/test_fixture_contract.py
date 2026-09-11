from pathlib import Path
import unittest


FIXTURE_ROOT = Path(__file__).resolve().parent / "fixtures"
REQUIRED_FIXTURES = (
    "clock/frozen_time.json",
    "graphs/asymmetric_3f_4f_roof.json",
    "inputs/localization_cases.json",
    "maps/3F.pgm",
    "maps/3F.yaml",
    "maps/4F.pgm",
    "maps/4F.yaml",
    "maps/roof.pgm",
    "maps/roof.yaml",
    "websocket/robot_session.ndjson",
)


class FixturePresenceTest(unittest.TestCase):
    def test_required_fixture_corpus_exists_when_contract_is_inspected(self) -> None:
        # Given: the fixture paths promised to downstream test workers.
        expected_paths = tuple(FIXTURE_ROOT / path for path in REQUIRED_FIXTURES)

        # When: the fixture corpus is inspected.
        missing_paths = tuple(
            path.relative_to(FIXTURE_ROOT).as_posix()
            for path in expected_paths
            if not path.is_file()
        )

        # Then: every promised deterministic input exists.
        self.assertEqual(missing_paths, ())


if __name__ == "__main__":
    unittest.main()
