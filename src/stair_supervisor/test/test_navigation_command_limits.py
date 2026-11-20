"""NAV output limits must not change stair ownership or climbing commands."""
import math
import unittest

import stair_supervisor.msg  # noqa: F401  devel package with generated messages, before fixtures prepend src
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))  # sibling test fixtures
from test_supervisor import (
    FakeAdmission, FakeClock, FakeTransport, ScriptedEvidence, make_configuration,
)
from stair_supervisor.supervisor import StairSupervisor, StairGoal, ResultCode
from stair_supervisor.configuration import Direction


class NavigationCommandLimitsTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.transport = FakeTransport()
        self.supervisor = StairSupervisor(
            make_configuration(linear_speed=.55), self.transport, ScriptedEvidence(),
            self.clock, lambda _report: None, FakeAdmission(), .25)
        self.supervisor.start()

    def output(self, linear, angular=0.):
        self.supervisor.accept_navigation(linear, angular)
        self.supervisor.stream_navigation()
        return self.transport.events[-2][1:]

    def test_arrival_surges_are_limited_at_nav_output(self):
        self.supervisor.set_navigation_limits(.21, 1.2)
        for incoming in [.619361, .653070, -.619361]:
            expected = math.copysign(.21, incoming)
            self.assertEqual(self.output(incoming), (expected, 0.))

    def test_inside_limit_commands_are_unchanged_and_zero_is_immediate(self):
        self.supervisor.set_navigation_limits(.21, 1.2)
        for linear, angular in [(0., 0.), (.12, .2), (-.10, -.3), (.21, 1.2), (0., 0.)]:
            self.assertEqual(self.output(linear, angular), (linear, angular))

    def test_lowering_limit_also_applies_to_already_received_command(self):
        self.supervisor.accept_navigation(.30, .5)
        epoch = self.supervisor.ownership_epoch
        self.supervisor.set_navigation_limits(.21)
        self.supervisor.stream_navigation()
        self.assertEqual(self.transport.events[-2], ('update', .21, .5))
        self.assertEqual(self.supervisor.ownership_epoch, epoch)

    def test_floor_changes_restore_limit_without_compounding(self):
        for limit in [.21, .21, .30, .30]:
            self.supervisor.set_navigation_limits(limit)
            self.assertEqual(self.output(.50)[0], limit)

    def test_stale_and_invalid_nav_remain_zero(self):
        self.supervisor.accept_navigation(.20, .3)
        self.clock.now += 1.
        self.supervisor.stream_navigation()
        self.assertEqual(self.transport.events[-2], ('update', 0., 0.))
        self.assertEqual(self.output(float('nan')), (0., 0.))

    def test_invalid_limit_does_not_change_previous_limit(self):
        self.supervisor.set_navigation_limits(.21)
        for invalid in [float('nan'), float('inf'), -.1]:
            with self.assertRaises(ValueError):
                self.supervisor.set_navigation_limits(invalid)
        self.assertEqual(self.output(.6)[0], .21)

    def test_nav_limit_never_caps_stair_ascent(self):
        self.supervisor.set_navigation_limits(.21)
        result = self.supervisor.traverse(StairGoal('test_up', Direction.UP, 'valid'), lambda: False)
        self.assertEqual(result.code, ResultCode.OK)
        self.assertIn(('update', .55, 0.), self.transport.events)

    def test_ros_limit_service_applies_before_ack_without_starting_motion(self):
        from dynamic_reconfigure.msg import Config, DoubleParameter, StrParameter
        from dynamic_reconfigure.srv import ReconfigureRequest
        from stair_supervisor.ros_node import RosStairSupervisorNode
        node = RosStairSupervisorNode.__new__(RosStairSupervisorNode)
        node._supervisor = self.supervisor
        before = list(self.transport.events)
        response = node._set_navigation_limits(ReconfigureRequest(config=Config(
            doubles=[DoubleParameter(name='max_vel_x', value=.21)],
            strs=[StrParameter(name='nav_limit_revision', value='200')])))
        self.assertEqual(self.transport.events, before)
        self.assertEqual({p.name: p.value for p in response.config.doubles}['max_vel_x'], .21)
        self.assertEqual(self.output(.619361)[0], .21)

    def test_delayed_old_floor_limit_cannot_overwrite_acknowledged_new_limit(self):
        self.supervisor.set_navigation_limits(.21, revision=200)
        with self.assertRaises(ValueError):
            self.supervisor.set_navigation_limits(.30, revision=100)
        self.assertEqual(self.output(.619361)[0], .21)
        # A deliberate new floor change still restores its speed.
        self.supervisor.set_navigation_limits(.30, revision=300)
        self.assertEqual(self.output(.619361)[0], .30)

    def test_duplicate_limit_revision_is_idempotent_but_not_reusable(self):
        self.supervisor.set_navigation_limits(.21, revision=200)
        self.assertEqual(self.supervisor.set_navigation_limits(.21, revision=200)[0], .21)
        with self.assertRaises(ValueError):
            self.supervisor.set_navigation_limits(.30, revision=200)
        self.assertEqual(self.output(.619361)[0], .21)

    def test_timed_out_rpc_cannot_later_undo_newer_floor_limit(self):
        import threading
        from unittest.mock import patch
        from dynamic_reconfigure.msg import Config, DoubleParameter, StrParameter
        from dynamic_reconfigure.srv import ReconfigureRequest
        from multifloor_manager.ros_services import BoundedServiceCaller, ServiceCallError
        from stair_supervisor.ros_node import RosStairSupervisorNode
        node = RosStairSupervisorNode.__new__(RosStairSupervisorNode)
        node._supervisor = self.supervisor
        release, finished = threading.Event(), threading.Event()
        def request(value, revision):
            return ReconfigureRequest(config=Config(
                doubles=[DoubleParameter(name='max_vel_x', value=value)],
                strs=[StrParameter(name='nav_limit_revision', value=str(revision))]))
        def old_proxy():
            release.wait(1.)
            try:
                return node._set_navigation_limits(request(.30, 100))
            finally:
                finished.set()
        try:
            with patch('rospy.wait_for_service'):
                with self.assertRaises(ServiceCallError):
                    BoundedServiceCaller(.02).call('/test_limits', old_proxy)
                reply = BoundedServiceCaller(.2).call(
                    '/test_limits', node._set_navigation_limits, request(.21, 200))
                self.assertEqual(reply.config.doubles[0].value, .21)
                release.set()
                self.assertTrue(finished.wait(1.))
            self.assertEqual(self.output(.619361)[0], .21)
        finally:
            release.set()


if __name__ == '__main__':
    unittest.main()
