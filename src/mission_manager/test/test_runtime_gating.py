#!/usr/bin/env python3
"""Mission runtime configuration profile admission tests."""

from __future__ import annotations

from unittest import mock
import unittest
from pathlib import Path
import shutil
import tempfile
import yaml

from mission_manager.ros_runtime import _configuration_roots
from mission_manager.site_config import ConfigurationError, ConfigurationRoots, load_site_configuration_from_roots


class RuntimeGatingTest(unittest.TestCase):
    def test_unconfigured_stair_profile_remains_fail_closed(self) -> None:
        # Given: the default production profile and package-owned config roots.
        with mock.patch(
            "mission_manager.ros_runtime.rospy.get_param",
            side_effect=lambda name, default=None: "production" if name == "~config_profile" else default,
        ):
            roots = _configuration_roots()

        # Exercise the gate independently of the site's current commissioning flag.
        # The shipped site may change; this test must not silently stop testing false.
        with tempfile.TemporaryDirectory() as directory:
            stair_root = Path(directory) / "stair"
            shutil.copytree(roots.stair, stair_root)
            path = stair_root / "stair_profiles.yaml"
            document = yaml.safe_load(path.read_text())
            document["configured"] = False
            path.write_text(yaml.safe_dump(document))
            with self.assertRaises(ConfigurationError):
                load_site_configuration_from_roots(ConfigurationRoots(roots.mission, roots.multifloor, stair_root))

    def test_fixture_profile_requires_explicit_second_gate(self) -> None:
        # Given: test_fixture is selected without allow_test_fixture.
        values = {"~config_profile": "test_fixture", "~allow_test_fixture": False}
        with mock.patch(
            "mission_manager.ros_runtime.rospy.get_param",
            side_effect=lambda name, default=None: values.get(name, default),
        ):
            # When/Then: startup rejects the fixture before reading its path.
            with self.assertRaises(RuntimeError):
                _configuration_roots()


if __name__ == "__main__":
    unittest.main()
