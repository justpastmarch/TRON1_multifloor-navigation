from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "test"))

from fixture_support import (  # noqa: E402
    FIXTURE_ROOT,
    FixtureError,
    assert_fresh,
    inspect_fixtures,
    load_frozen_clock,
    load_localization_cases,
)


class MultifloorFixtureDiscoveryTest(unittest.TestCase):
    def test_maps_and_amcl_covariance_load_for_every_floor(self) -> None:
        # Given: multifloor map and localization fixtures.
        summary = inspect_fixtures()
        cases = load_localization_cases()

        # When/Then: every map exists and AMCL carries full covariance.
        self.assertEqual(summary.map_names, ("3F", "4F", "roof"))
        self.assertEqual(len(cases.amcl_valid.covariance), 36)

    def test_stale_tf_is_rejected_by_frozen_clock(self) -> None:
        # Given: the stale TF sample and deterministic fixture clock.
        cases = load_localization_cases()

        # When/Then: the multifloor adapter seam rejects stale localization.
        with self.assertRaisesRegex(FixtureError, "stale timestamp"):
            assert_fresh(
                cases.tf_stale.stamp_ns,
                load_frozen_clock(),
                FIXTURE_ROOT / "inputs/localization_cases.json",
            )


if __name__ == "__main__":
    unittest.main()
