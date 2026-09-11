"""Verify bounded service failures remain typed and fail closed."""

from __future__ import annotations

from pathlib import Path
import sys
from unittest import mock
import unittest

import rospy

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "multifloor_manager" / "src"))

from multifloor_manager.ros_services import BoundedServiceCaller, ServiceCallError  # noqa: E402


class RosServiceFailureTest(unittest.TestCase):
    def test_nomotion_service_failure_is_reported_at_the_transition_boundary(self) -> None:
        # Given: the request_nomotion_update peer raises a ROS service error.
        caller = BoundedServiceCaller(0.2)

        def fail_nomotion() -> None:
            raise rospy.ServiceException("nomotion peer failed")

        # When: the transition boundary invokes that peer.
        with mock.patch("multifloor_manager.ros_services.rospy.wait_for_service"):
            with self.assertRaises(ServiceCallError) as context:
                caller.call("/request_nomotion_update", fail_nomotion)

        # Then: the failure identifies the exact localization service.
        self.assertEqual(context.exception.service, "/request_nomotion_update")

    def test_clear_costmaps_service_timeout_is_reported_at_the_transition_boundary(self) -> None:
        # Given: the clear-costmaps peer never returns.
        caller = BoundedServiceCaller(0.01)

        def hang_clear_costmaps() -> None:
            rospy.sleep(0.2)

        # When: the transition boundary invokes that peer.
        with mock.patch("multifloor_manager.ros_services.rospy.wait_for_service"):
            with self.assertRaises(ServiceCallError) as context:
                caller.call("/move_base/clear_costmaps", hang_clear_costmaps)

        # Then: timeout is typed with the exact navigation service name.
        self.assertEqual(context.exception.service, "/move_base/clear_costmaps")
        self.assertEqual(context.exception.detail, "response timeout")


if __name__ == "__main__":
    unittest.main()
