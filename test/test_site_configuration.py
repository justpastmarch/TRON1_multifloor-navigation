from dataclasses import FrozenInstanceError
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


class SiteConfigurationTest(unittest.TestCase):
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

    def rewrite_new_transition(self, mutate) -> None:
        def replace(document) -> None:
            document["transitions"] = [{
                "id": "floor_transition_ready",
                "enabled": 1,
                "conditions": [
                    {"name": "T_FLOOR_CONFIRMED", "enabled": 1, "required": 1},
                    {"name": "T_LOCALIZED", "enabled": 1, "required": 1},
                    {"name": "T_COSTMAP_READY", "enabled": 1, "required": 1},
                ],
                "optional_count": 0,
                "freshness_sec": 10.0,
                "dwell_sec": 0.2,
                "timeout_sec": 8.0,
            }]
            mutate(document["transitions"][0])

        self.rewrite("transitions.yaml", replace)

    def assert_invalid(self, expected: str) -> None:
        with self.assertRaisesRegex(ConfigurationError, expected):
            load_site_configuration(self.root)

    def test_current_transition_fixture_loads_through_site_boundary(self) -> None:
        # Given: the unchanged known-good site fixture.
        # When: the site boundary loads its transition configuration.
        transitions = load_site_configuration(self.root).transitions
        # Then: the configured floor transition policy is available to callers.
        self.assertEqual(tuple(policy.id for policy in transitions), ("floor_transition_ready",))

    def test_transition_fixture_loads_immutable_ordered_condition_schema(self) -> None:
        # Given: the approved configured transition fixture.
        # When: the site boundary loads the policy.
        policy = load_site_configuration(self.root).transitions[0]
        # Then: exact integer flags have become immutable typed booleans in order.
        self.assertEqual(
            tuple((condition.name, condition.enabled, condition.required) for condition in policy.conditions),
            (
                ("T_FLOOR_CONFIRMED", True, True),
                ("T_LOCALIZED", True, True),
                ("T_COSTMAP_READY", True, True),
            ),
        )
        self.assertEqual((policy.id, policy.enabled, policy.optional_count), ("floor_transition_ready", True, 0))
        self.assertEqual((policy.freshness_sec, policy.dwell_sec, policy.timeout_sec), (10.0, 0.2, 8.0))
        with self.assertRaises(FrozenInstanceError):
            policy.conditions[0].enabled = False

    def test_transition_policy_flags_require_exact_yaml_integer_bits(self) -> None:
        for value in (True, False, "1", "0", 2, -1):
            with self.subTest(value=value):
                self.rewrite_new_transition(lambda policy: policy.update({"enabled": value}))
                self.assert_invalid(r"transitions\.yaml:transitions\[0\]\.enabled: expected integer 0 or 1")

    def test_transition_condition_flags_require_exact_yaml_integer_bits(self) -> None:
        for key in ("enabled", "required"):
            for value in (True, False, "1", "0", 2, -1):
                with self.subTest(key=key, value=value):
                    self.rewrite_new_transition(lambda policy: policy["conditions"][0].update({key: value}))
                    self.assert_invalid(rf"transitions\.yaml:transitions\[0\]\.conditions\[0\]\.{key}: expected integer 0 or 1")

    def test_old_transition_predicate_arrays_are_rejected(self) -> None:
        def restore_old_keys(policy) -> None:
            policy.pop("conditions")
            policy.update({"required": ["T_LOCALIZED"], "optional": []})

        self.rewrite_new_transition(restore_old_keys)
        self.assert_invalid(r"transitions\.yaml:transitions\[0\]: keys must be")

    def test_transition_condition_names_are_closed_unique_and_complete(self) -> None:
        mutations = (
            (
                lambda policy: policy["conditions"].append({"name": "T_UNKNOWN", "enabled": 1, "required": 1}),
                r"conditions\[3\]\.name: unknown condition 'T_UNKNOWN'",
            ),
            (
                lambda policy: policy["conditions"][1].update({"name": "T_FLOOR_CONFIRMED"}),
                r"conditions\[1\]\.name: duplicate condition 'T_FLOOR_CONFIRMED'",
            ),
            (
                lambda policy: policy["conditions"].pop(),
                r"transitions\[0\]\.conditions: missing mandatory condition 'T_COSTMAP_READY'",
            ),
        )
        for mutate, expected in mutations:
            with self.subTest(expected=expected):
                self.rewrite_new_transition(mutate)
                self.assert_invalid(expected)

    def test_mandatory_transition_conditions_cannot_be_disabled_or_optional(self) -> None:
        for key in ("enabled", "required"):
            with self.subTest(key=key):
                self.rewrite_new_transition(lambda policy: policy["conditions"][0].update({key: 0}))
                self.assert_invalid(rf"conditions\[0\]\.{key}: mandatory condition must be 1")

    def test_current_transition_policy_requires_zero_optional_count(self) -> None:
        for value in (1, -1, False):
            with self.subTest(value=value):
                self.rewrite_new_transition(lambda policy: policy.update({"optional_count": value}))
                self.assert_invalid(r"transitions\[0\]\.optional_count: expected integer 0")

    def test_floor_transition_policy_must_be_the_single_enabled_policy(self) -> None:
        mutations = (
            (lambda policy: policy.update({"id": "other"}), r"transitions\[0\]\.id: expected 'floor_transition_ready'"),
            (lambda policy: policy.update({"enabled": 0}), r"transitions\[0\]\.enabled: must be 1"),
        )
        for mutate, expected in mutations:
            with self.subTest(expected=expected):
                self.rewrite_new_transition(mutate)
                self.assert_invalid(expected)
        self.rewrite_new_transition(lambda policy: None)
        self.rewrite("transitions.yaml", lambda document: document["transitions"].append(dict(document["transitions"][0])))
        self.assert_invalid(r"transitions\.yaml:transitions: must contain exactly one policy")

    def test_transition_temporal_values_are_strict(self) -> None:
        mutations = (
            (lambda policy: policy.update({"freshness_sec": 0}), r"freshness_sec: must be finite and positive"),
            (lambda policy: policy.update({"freshness_sec": float("inf")}), r"freshness_sec: must be finite and positive"),
            (lambda policy: policy.update({"timeout_sec": 0}), r"timeout_sec: must be finite and positive"),
            (lambda policy: policy.update({"dwell_sec": -0.1}), r"dwell_sec: must be finite and nonnegative"),
            (lambda policy: policy.update({"dwell_sec": 8.0}), r"dwell_sec: must be less than timeout_sec"),
        )
        for mutate, expected in mutations:
            with self.subTest(expected=expected):
                self.rewrite_new_transition(mutate)
                self.assert_invalid(expected)

    def test_future_placeholder_condition_is_unknown_until_registry_support_exists(self) -> None:
        self.rewrite_new_transition(
            lambda policy: policy["conditions"].append({
                "name": "T_ARRIVAL_ANNOUNCEMENT_DONE",
                "enabled": 1,
                "required": 0,
            })
        )
        self.assert_invalid(r"conditions\[3\]\.name: unknown condition 'T_ARRIVAL_ANNOUNCEMENT_DONE'")

    def test_configured_document_flag_remains_a_yaml_boolean(self) -> None:
        self.rewrite("transitions.yaml", lambda document: document.update({"configured": 1}))
        self.assert_invalid(r"transitions\.yaml:configured: expected boolean")

    def test_complete_asymmetric_building_loads_when_all_references_are_valid(self) -> None:
        # Given: complete maps and deliberately asymmetric directed edges.
        # When: the site boundary parses every owning package configuration.
        configuration = load_site_configuration(self.root)
        # Then: callers receive immutable typed data in deterministic order.
        self.assertEqual(tuple(edge.id for edge in configuration.edges)[-2:], ("return_across_4f", "stair_a_down"))
        self.assertEqual(configuration.floors[2].id, "RF")
        with self.assertRaises(FrozenInstanceError):
            configuration.floors[0].frame = "odom"

    def test_duplicate_ids_are_rejected_with_a_path(self) -> None:
        self.rewrite("locations.yaml", lambda doc: doc["locations"].append(dict(doc["locations"][0])))
        self.assert_invalid(r"locations\.yaml:locations\[7\]\.id: duplicate")

    def test_missing_map_images_are_rejected_with_a_path(self) -> None:
        (self.root / "maps" / "floor_4f.pgm").unlink()
        self.assert_invalid(r"floors\.yaml:floors\[1\]\.map_yaml: map image does not exist")

    def test_dangling_graph_endpoints_are_rejected_with_a_path(self) -> None:
        self.rewrite("building_graph.yaml", lambda doc: doc["edges"][0].update({"to": "missing"}))
        self.assert_invalid(r"building_graph\.yaml:edges\[0\]\.to: unknown location")

    def test_invalid_yaw_is_rejected_with_a_path(self) -> None:
        self.rewrite("locations.yaml", lambda doc: doc["locations"][0].update({"yaw": 4.0}))
        self.assert_invalid(r"locations\.yaml:locations\[0\]\.yaw: must be within")

    def test_nonpositive_tag_size_is_rejected_with_a_path(self) -> None:
        self.rewrite("apriltags.yaml", lambda doc: doc["tags"][0].update({"size_m": 0}))
        self.assert_invalid(r"apriltags\.yaml:tags\[0\]\.size_m: must be positive")

    def test_empty_scan_topic_lists_are_rejected_with_a_path(self) -> None:
        self.rewrite("scan_profiles.yaml", lambda doc: doc["profiles"][0].update({"topics": []}))
        self.assert_invalid(r"scan_profiles\.yaml:profiles\[0\]\.topics: must not be empty")

    def test_malformed_yaml_is_path_qualified(self) -> None:
        (self.root / "robot.yaml").write_text("move_base: [", encoding="utf-8")
        self.assert_invalid(r"robot\.yaml: YAML parse error")

    def test_unconfigured_documents_cannot_pass_as_production_data(self) -> None:
        self.rewrite("floors.yaml", lambda doc: doc.update({"configured": False}))
        self.assert_invalid(r"floors\.yaml:configured: must be true")


if __name__ == "__main__":
    unittest.main()
