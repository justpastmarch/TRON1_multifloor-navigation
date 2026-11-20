"""Offline recovery and scan-time AMCL checks; no ROS node or service is started."""
import threading
import unittest
from types import SimpleNamespace as N
from unittest.mock import Mock, patch
import rospy

from multifloor_manager.msg import FloorState
from multifloor_manager.ros_floor_selection import FloorSelection
from multifloor_manager.ros_node import MultifloorManagerNode
from multifloor_manager.ros_startup_localization import StartupLocalization
from multifloor_manager.readiness import arm_localization, observe_pose, Nanoseconds
from multifloor_manager.ros_runtime import RosEvidenceRuntime
from multifloor_manager.map_evidence import MapIdentity, MapOrigin
from multifloor_manager.transitions import TransitionPolicy, TransitionCondition
from stair_supervisor.msg import SupervisorState

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))  # sibling test fixtures
import test_floor_selection as floor_selection_fixture


class FloorRecoveryTest(unittest.TestCase):
    def startup(self):
        startup = StartupLocalization.__new__(StartupLocalization)
        startup.runtime = N(state=FloorState.FAULT)
        startup.supervisor = N(state=SupervisorState.NAV)
        startup.action_subscribers = []
        startup.action_states = {}
        return startup

    def test_terminal_fault_is_recoverable_only_through_explicit_floor_selection(self):
        startup = self.startup()
        self.assertTrue(startup.busy())
        self.assertFalse(startup.busy(allow_fault=True))
        for state in [FloorState.TRANSITIONING]:
            startup.runtime.state = state
            self.assertTrue(startup.busy(allow_fault=True))
        startup.runtime.state = FloorState.FAULT
        startup.supervisor.state = SupervisorState.STAIR
        self.assertTrue(startup.busy(allow_fault=True))

    def test_active_or_stale_child_still_prevents_floor_reselection(self):
        startup = self.startup()
        subscriber = N(get_num_connections=lambda: 1)
        startup.action_subscribers = [subscriber]
        startup.action_states = {'mission': (100., N(status_list=[N(status=1)]))}
        with patch('multifloor_manager.ros_startup_localization.time.monotonic', return_value=100.):
            self.assertTrue(startup.busy(allow_fault=True))
            startup.action_states = {'mission': (97., N(status_list=[]))}
            self.assertTrue(startup.busy(allow_fault=True))
            startup.action_states = {'mission': (100., N(status_list=[]))}
            self.assertFalse(startup.busy(allow_fault=True))

    def test_actual_fault_busy_logic_allows_select_then_unknown_localization(self):
        fixture = floor_selection_fixture.TestFloorSelection()
        fixture.setUp()
        try:
            selection, node = fixture.fixture()
            actual = self.startup()
            actual.runtime = node.runtime
            node.runtime.state = FloorState.FAULT
            node.startup.busy = actual.busy
            with patch('multifloor_manager.ros_floor_selection.fingerprint_occupancy_grid',
                       return_value=N(identity=lambda: 'identity5')):
                result = selection.select(N(floor_id='5F'))
            self.assertTrue(result.accepted, result.reason)
            self.assertEqual(node.startup.request, ('auto', None))
            self.assertTrue(all(call.args[0] == FloorState.UNKNOWN
                                for call in node.runtime.publish_floor.call_args_list))
        finally:
            fixture.doCleanups()

    def test_failed_transaction_preserves_last_confirmed_floor_context(self):
        node = MultifloorManagerNode.__new__(MultifloorManagerNode)
        node.runtime = N(current_floor='5F', initial_floor='3F', finish_transition=Mock())
        node.server = N(set_aborted=Mock())
        node._result = Mock()
        node._fail('AMCL update missing')
        node.runtime.finish_transition.assert_called_once_with(
            FloorState.FAULT, 'AMCL update missing', '5F')

    def test_transition_retains_source_floor_without_asserting_ready(self):
        identity = MapIdentity('map', 1, 1, .1,
                               MapOrigin(0., 0., 0., 0., 0., 0., 1.), 'fixture')
        policy = TransitionPolicy('test', True,
                                  (TransitionCondition('ready', True, True),),
                                  0, 1., 0., 2.)
        with patch('multifloor_manager.ros_runtime.rospy.Publisher'), \
             patch('multifloor_manager.ros_runtime.rospy.Time.now', return_value=rospy.Time(100)):
            runtime = RosEvidenceRuntime('5F', identity, (), None, policy)
            runtime.current_floor = '5F'
            runtime.arm_transition('RF', (), identity)
        self.assertEqual(runtime.current_floor, '5F')
        self.assertEqual(runtime.state, FloorState.TRANSITIONING)


class LocalizationRefreshTest(unittest.TestCase):
    def node(self):
        node = MultifloorManagerNode.__new__(MultifloorManagerNode)
        node.runtime = N(active_epoch=9, condition=threading.Condition())
        node.server = N(is_preempt_requested=Mock(return_value=False))
        node.nomotion_name, node.nomotion_update = '/request_nomotion_update', Mock()
        node.services = N(call=Mock())
        node._wait = lambda predicate, deadline: ((True, '') if predicate()
                                                  else (False, 'transition evidence timeout'))
        return node

    def test_post_initialpose_scan_before_rpc_response_is_accepted(self):
        node = self.node()
        localization = arm_localization(Nanoseconds(100))
        def update(*_args):
            nonlocal localization
            # Scan acquired at 150; response/callback observed at 200.
            localization = observe_pose(localization, Nanoseconds(150), Nanoseconds(200), (.01, .01, .01))
        node.services.call.side_effect = update
        with patch('multifloor_manager.ros_node.time.monotonic', return_value=0.):
            result = node._wait_with_localization_refresh(
                lambda: localization.pose_stamp_ns is not None and localization.pose_stamp_ns > 100,
                20.)
        self.assertTrue(result[0])
        node.services.call.assert_called_once()

    def test_reissue_after_missing_update_and_accept_distinct_scan_stamp(self):
        node = self.node()
        wall = [0.]
        stamp = [None]
        def request(*_args):
            if node.services.call.call_count == 2:
                stamp[0] = 150
        def wait(predicate, deadline):
            if predicate():
                return True, ''
            wall[0] = deadline
            return False, 'transition evidence timeout'
        node.services.call.side_effect = request
        node._wait = wait
        with patch('multifloor_manager.ros_node.time.monotonic', side_effect=lambda: wall[0]):
            result = node._wait_with_localization_refresh(lambda: stamp[0] is not None and stamp[0] > 100, 2.)
        self.assertEqual(result, (True, ''))
        self.assertEqual(node.services.call.call_count, 2)

    def test_old_pose_does_not_satisfy_initialization_fence(self):
        localization = arm_localization(Nanoseconds(100))
        old = observe_pose(localization, Nanoseconds(99), Nanoseconds(200), (.01, .01, .01))
        self.assertEqual(old.pose_count, 0)
        self.assertFalse(old.pose_stamp_ns > 100)

    def test_cancel_prevents_another_nomotion_request(self):
        node = self.node()
        node.server.is_preempt_requested.return_value = True
        result = node._wait_with_localization_refresh(lambda: False, 20.)
        self.assertEqual(result, (False, 'transition cancelled'))
        node.services.call.assert_not_called()

    def test_epoch_change_during_service_cannot_accept_new_transaction_pose(self):
        node = self.node()
        node.services.call.side_effect = lambda *_: setattr(node.runtime, 'active_epoch', 10)
        with patch('multifloor_manager.ros_node.time.monotonic', return_value=0.):
            result = node._wait_with_localization_refresh(lambda: node.runtime.active_epoch == 10, 20.)
        self.assertEqual(result, (False, 'transition superseded'))

    def test_already_ready_evidence_does_not_require_another_service_reply(self):
        node = self.node()
        with patch('multifloor_manager.ros_node.time.monotonic', return_value=0.):
            result = node._wait_with_localization_refresh(lambda: True, 20.)
        self.assertEqual(result, (True, ''))
        node.services.call.assert_not_called()

    def test_costmap_wait_refreshes_amcl_instead_of_expiring_stationary_pose(self):
        node = self.node()
        wall = [0.]
        def wait(predicate, deadline):
            if predicate():return True, ''
            wall[0] = deadline
            return False, 'transition evidence timeout'
        node._wait = wait
        with patch('multifloor_manager.ros_node.time.monotonic', side_effect=lambda: wall[0]):
            result = node._wait_with_localization_refresh(lambda: wall[0] >= .6, 2.)
        self.assertEqual(result, (True, ''))
        self.assertGreaterEqual(node.services.call.call_count, 3)

    def test_final_policy_dwell_keeps_stationary_amcl_fresh(self):
        node = self.node()
        wall, last_refresh = [0.], [0.]
        class Condition:
            def __enter__(self):return self
            def __exit__(self, *args):pass
            def wait(self, seconds):wall[0] += seconds
        observations = {}
        node.runtime.condition = Condition()
        node.runtime.policy = N(timeout_sec=2.)
        node.runtime.begin_policy = Mock()
        node.runtime.localization_evidence = lambda: N(ready=wall[0]-last_refresh[0] < .5)
        node.runtime.record_policy_observation = lambda token, name, observation: observations.update({name: observation.value})
        node.runtime.evaluate_policy = lambda token: (wall[0] >= .7 and observations.get('T_LOCALIZED', False), None)
        node.runtime.commit_ready = lambda token: True
        node._feedback = Mock()
        node._target_metadata_matches = lambda target, marker: True
        node.services.call.side_effect = lambda *_: last_refresh.__setitem__(0, wall[0])
        with patch('multifloor_manager.ros_node.time.monotonic', side_effect=lambda: wall[0]):
            result = node._wait_for_policy_ready(N(epoch=9), None, 0)
        self.assertEqual(result, (True, ''))
        self.assertGreaterEqual(node.services.call.call_count, 3)


if __name__ == '__main__':
    unittest.main()
