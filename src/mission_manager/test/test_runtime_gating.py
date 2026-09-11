#!/usr/bin/env python3
"""Mission runtime configuration profile admission tests."""

from __future__ import annotations

from unittest import mock
import unittest

from mission_manager.ros_runtime import _configuration_roots
from mission_manager.site_config import ConfigurationError, load_site_configuration_from_roots


class RuntimeGatingTest(unittest.TestCase):
    def test_production_profile_remains_fail_closed_until_surveyed(self) -> None:
        # Given: the default production profile and package-owned config roots.
        with mock.patch(
            "mission_manager.ros_runtime.rospy.get_param",
            side_effect=lambda name, default=None: "production" if name == "~config_profile" else default,
        ):
            roots = _configuration_roots()

        # When/Then: current configured:false production data cannot start missions.
        with self.assertRaises(ConfigurationError):
            load_site_configuration_from_roots(roots)

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
