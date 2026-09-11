"""Keep the policy-blocked ROS fixture identical except for transition freshness."""

from __future__ import annotations

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
VALID = ROOT / "test" / "fixtures" / "building_valid"
BLOCKED = ROOT / "test" / "fixtures" / "building_policy_blocked"


class PolicyFixtureParityTest(unittest.TestCase):
    def test_only_transition_freshness_differs(self) -> None:
        # Given: the normal and policy-blocked manager fixtures.
        valid_paths = sorted(path.relative_to(VALID) for path in VALID.rglob("*") if path.is_file())
        blocked_paths = sorted(path.relative_to(BLOCKED) for path in BLOCKED.rglob("*") if path.is_file())
        # When: every manager-consumed path except transitions.yaml is compared.
        self.assertEqual(valid_paths, blocked_paths)
        for relative in valid_paths:
            if relative == Path("transitions.yaml"):
                continue
            self.assertEqual((VALID / relative).read_bytes(), (BLOCKED / relative).read_bytes(), relative)
        blocked_policy = yaml.safe_load((BLOCKED / "transitions.yaml").read_text(encoding="utf-8"))
        valid_policy = yaml.safe_load((VALID / "transitions.yaml").read_text(encoding="utf-8"))
        # Then: the sole semantic difference is the final-gate freshness value.
        self.assertEqual(blocked_policy["transitions"][0]["freshness_sec"], 0.01)
        blocked_policy["transitions"][0]["freshness_sec"] = valid_policy["transitions"][0]["freshness_sec"]
        self.assertEqual(blocked_policy, valid_policy)


if __name__ == "__main__":
    unittest.main()
