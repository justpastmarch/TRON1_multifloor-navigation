from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
for package in ("mission_manager", "multifloor_manager", "stair_supervisor"):
    sys.path.insert(0, str(ROOT / "src" / package / "src"))

from mission_manager.site_config import ConfigurationError, load_site_configuration  # noqa: E402 -- bootstrap catkin source paths


FIXTURE = ROOT / "test" / "fixtures" / "building_valid"


class StairEndpointConfigurationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_dir.name) / "site"
        shutil.copytree(FIXTURE, self.root)

    def tearDown(self) -> None:
        self.temporary_dir.cleanup()

    def rewrite(self, name: str, mutate) -> None:
        path = self.root / name
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        mutate(document)
        path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    def reset_stairs(self) -> None:
        shutil.copy2(FIXTURE / "stairs.yaml", self.root / "stairs.yaml")

    def assert_invalid(self, expected: str) -> None:
        with self.assertRaisesRegex(ConfigurationError, expected):
            load_site_configuration(self.root)

    def test_stair_loads_exactly_two_floor_keyed_endpoints(self) -> None:
        # Given: a stair with deliberately asymmetric endpoint hypotheses.
        # When: the site boundary loads stair A.
        stair = load_site_configuration(self.root).stairs[0]
        # Then: each floor key resolves its own entry, landing, covariance, and tags.
        lower = stair.endpoint_for("3F")
        upper = stair.endpoint_for("4F")
        self.assertEqual(tuple(endpoint.floor_id for endpoint in stair.endpoints), ("3F", "4F"))
        self.assertEqual((lower.entry_location_id, lower.target_landing.x), ("stair_a_entry_3f", 0.25))
        self.assertEqual((upper.entry_location_id, upper.target_landing.x), ("stair_a_landing_4f", -1.0))
        self.assertEqual((len(lower.covariance), lower.expected_tag_ids), (36, (100,)))
        self.assertEqual((len(upper.covariance), upper.expected_tag_ids), (36, (101,)))

    def test_endpoint_entry_location_must_share_its_floor(self) -> None:
        self.rewrite("stairs.yaml", lambda doc: doc["stairs"][0]["endpoints"]["4F"].update({"entry_location_id": "home_3f"}))
        self.assert_invalid(r"stairs\.yaml:stairs\[0\]\.endpoints\.4F\.entry_location_id: location floor")

    def test_endpoint_landing_pose_requires_normalized_quaternion(self) -> None:
        self.rewrite("stairs.yaml", lambda doc: doc["stairs"][0]["endpoints"]["4F"]["target_landing"]["orientation"].update({"w": 0.2}))
        self.assert_invalid(r"stairs\.yaml:stairs\[0\]\.endpoints\.4F\.target_landing\.orientation: quaternion must be normalized")

    def test_endpoint_expected_tags_must_exist(self) -> None:
        self.rewrite("stairs.yaml", lambda doc: doc["stairs"][0]["endpoints"]["4F"].update({"expected_tag_ids": [999]}))
        self.assert_invalid(r"stairs\.yaml:stairs\[0\]\.endpoints\.4F\.expected_tag_ids\[0\]: unknown tag")

    def test_endpoint_covariance_requires_36_values(self) -> None:
        self.rewrite("stairs.yaml", lambda doc: doc["stairs"][0]["endpoints"]["3F"].update({"covariance": [0.1] * 35}))
        self.assert_invalid(r"stairs\.yaml:stairs\[0\]\.endpoints\.3F\.covariance: expected 36 values")

    def test_endpoint_covariance_rejects_nonfinite_off_diagonal_and_negative_values(self) -> None:
        covariance_mutations = (
            lambda covariance: covariance.__setitem__(0, float("nan")),
            lambda covariance: covariance.__setitem__(1, 0.01),
            lambda covariance: covariance.__setitem__(7, -0.01),
        )
        expected_errors = ("expected finite number", "nonnegative 6x6 diagonal covariance", "nonnegative 6x6 diagonal covariance")
        for mutate, expected in zip(covariance_mutations, expected_errors):
            with self.subTest(expected=expected):
                self.reset_stairs()
                self.rewrite("stairs.yaml", lambda doc: mutate(doc["stairs"][0]["endpoints"]["3F"]["covariance"]))
                self.assert_invalid(expected)

    def test_missing_or_extra_stair_endpoint_floor_is_rejected(self) -> None:
        mutations = (
            lambda endpoints: endpoints.pop("3F"),
            lambda endpoints: endpoints.update({"RF": dict(endpoints["3F"])}),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.reset_stairs()
                self.rewrite("stairs.yaml", lambda doc: mutate(doc["stairs"][0]["endpoints"]))
                self.assert_invalid(r"stairs\.yaml:stairs\[0\]\.endpoints: keys must be \['3F', '4F'\]")

    def test_wrong_floor_or_stair_landing_tag_is_rejected(self) -> None:
        mutations = (
            lambda doc: doc["stairs"][0]["endpoints"]["3F"].update({"expected_tag_ids": [101]}),
            lambda doc: doc["stairs"][0]["endpoints"]["4F"].update({"expected_tag_ids": [201]}),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.reset_stairs()
                self.rewrite("stairs.yaml", mutate)
                self.assert_invalid(r"expected_tag_ids\[0\]: tag stair/floor does not match endpoint")

    def test_cross_floor_navigation_is_rejected_through_site_boundary(self) -> None:
        self.rewrite("building_graph.yaml", lambda doc: doc["edges"][0].update({"to": "roof_landing"}))
        self.assert_invalid(r"building_graph\.yaml:edges\[0\]: NAV endpoints must share a floor")

    def test_stair_edge_source_must_be_its_floor_entry_endpoint(self) -> None:
        self.rewrite("building_graph.yaml", lambda doc: doc["edges"][6].update({"from": "roof_scan"}))
        self.assert_invalid(r"building_graph\.yaml:edges\[6\]\.from: must match RF stair entry 'roof_landing'")

    def test_unreleased_single_endpoint_stair_shape_is_rejected(self) -> None:
        def restore_old_shape(document) -> None:
            stair = document["stairs"][0]
            endpoint = stair.pop("endpoints")["4F"]
            stair.update(endpoint)

        self.rewrite("stairs.yaml", restore_old_shape)
        self.assert_invalid(r"stairs\.yaml:stairs\[0\]: keys must be")

    def test_missing_directional_profiles_are_rejected_with_a_path(self) -> None:
        self.rewrite("stairs.yaml", lambda doc: doc["stairs"][1].update({"down_profile_id": "missing_down"}))
        self.assert_invalid(r"stairs\.yaml:stairs\[1\]\.down_profile_id: unknown profile")


if __name__ == "__main__":
    unittest.main()
