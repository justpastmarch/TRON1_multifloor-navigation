from __future__ import annotations

import hashlib
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "multifloor_manager" / "src"))

from multifloor_manager.map_loader import load_map_identity  # noqa: E402


class MapLoaderTest(unittest.TestCase):
    def test_fixture_yaml_resolves_to_map_server_identity(self) -> None:
        # Given: a relative P2 map with a nonzero map yaw.
        yaml_path = ROOT / "test" / "fixtures" / "building_valid" / "maps" / "floor_4f.yaml"

        # When: its target fingerprint is armed before calling change_map.
        identity = load_map_identity(yaml_path, "map")

        # Then: dimensions, origin, and vertically flipped trinary cells match map_server.
        expected_cells = bytes((0, 255, 100, 100, 255, 0))
        self.assertEqual((identity.width, identity.height), (3, 2))
        self.assertAlmostEqual(identity.resolution, 0.1)
        self.assertAlmostEqual(identity.origin.qz, 0.124674733)
        self.assertAlmostEqual(identity.origin.qw, 0.992197667)
        self.assertEqual(identity.data_hash, hashlib.sha256(expected_cells).hexdigest())

    def test_missing_or_dirty_map_is_rejected_before_transition(self) -> None:
        # Given: fixture YAML whose image is missing or malformed.
        missing = ROOT / "test" / "fixtures" / "maps" / "missing.yaml"

        # When/Then: unresolved target content cannot become an armed identity.
        with self.assertRaises((OSError, ValueError)):
            load_map_identity(missing, "map")


if __name__ == "__main__":
    unittest.main()
