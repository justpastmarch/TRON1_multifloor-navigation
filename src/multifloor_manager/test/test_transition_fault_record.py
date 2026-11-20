"""Offline floor-transition fault bookkeeping and resume; no ROS node is started."""
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock, patch

import rospy
from nav_msgs.srv import LoadMapResponse

from multifloor_manager.configuration import LandingPose, Quaternion, Stair, StairEndpoint
from multifloor_manager.msg import FloorState, FloorTransitionResult
from multifloor_manager.ros_node import MultifloorManagerNode
from multifloor_manager.ros_services import ServiceCallError
from multifloor_manager.transaction_record import (
    MAP_UNKNOWN,
    RESUME_CHANGE_MAP,
    RESUME_INITIALPOSE,
    TransactionProgress,
    TransitionFault,
    resume_stage,
)


class ResumeStageTest(unittest.TestCase):
    def fault(self, **progress):
        values = dict(transition_id='stair_a', source_floor='3F', target_floor='4F', stage='AMCL_READY',
                      floor_confirmed=True, map_floor='4F', map_observed=True)
        values.update(progress)
        return TransitionFault(TransactionProgress(**values), 7, 100., 'reason')

    def stage(self, fault, **overrides):
        values = dict(transition_id='stair_a', target_floor='4F', in_fault=True, current_floor='3F',
                      map_generation=7, observed_map_floor='4F', now=110., max_age_sec=300.)
        values.update(overrides)
        return resume_stage(fault, **values)

    def test_observed_target_map_resumes_at_initialpose(self):
        self.assertEqual(self.stage(self.fault()), RESUME_INITIALPOSE)

    def test_unobserved_or_unknown_map_resumes_at_change_map(self):
        self.assertEqual(self.stage(self.fault(map_floor=MAP_UNKNOWN, map_observed=False),
                                    observed_map_floor='3F'), RESUME_CHANGE_MAP)
        self.assertEqual(self.stage(self.fault(map_floor='4F', map_observed=False),
                                    observed_map_floor='3F'), RESUME_CHANGE_MAP)

    def test_resume_requires_same_fault_context(self):
        fault = self.fault()
        self.assertIsNone(self.stage(None))
        self.assertIsNone(self.stage(fault, in_fault=False))
        self.assertIsNone(self.stage(fault, transition_id='stair_b'))
        self.assertIsNone(self.stage(fault, target_floor='5F'))
        self.assertIsNone(self.stage(fault, current_floor='4F'))
        self.assertIsNone(self.stage(fault, map_generation=8))
        self.assertIsNone(self.stage(fault, now=401.))
        self.assertIsNone(self.stage(fault, now=99.))
        self.assertIsNone(self.stage(self.fault(floor_confirmed=False)))

    def test_description_names_map_stage_and_confirmed_floor(self):
        text = TransactionProgress('stair_a', '3F', '4F', 'AMCL_READY', True, '4F', True).describe('timeout')
        self.assertEqual(text, 'timeout [stage=AMCL_READY; map=4F; confirmed_floor=3F; '
                               'target=4F; target_confirmed=yes]')
        self.assertIn('map=unknown', TransactionProgress('s', '3F', '4F', map_floor=MAP_UNKNOWN).describe('x'))
        self.assertIn('map=4F (requested, not yet observed)',
                      TransactionProgress('s', '3F', '4F', map_floor='4F', map_observed=False).describe('x'))


class FakeRuntime:
    """Just enough of RosEvidenceRuntime for one transaction."""

    def __init__(self, identities):
        self.identities = identities
        self.state = FloorState.READY
        self.current_floor = '3F'
        self.initial_floor = '3F'
        self.generation = 7
        self.loaded = '3F'
        self.map_guard = N(armed=False)
        self.costmap_sequence = 0
        self.finished = []
        self.arms = 0

    @property
    def map_state(self):
        identity = self.identities[self.loaded] if self.loaded else None
        fingerprint = None if identity is None else N(identity=lambda: identity)
        return N(generation=self.generation, current_fingerprint=fingerprint)

    def arm_transition(self, *_args):
        self.arms += 1
        self.state = FloorState.TRANSITIONING
        return N(epoch=self.arms)

    def record_policy_observation(self, *_args):
        pass

    def arm_localization(self, *_args):
        pass

    def pose_high_water_ns(self, fallback):
        return fallback

    def finish_transition(self, state, detail, floor_id=''):
        self.state, self.current_floor = state, floor_id
        self.finished.append((state, detail, floor_id))


class Services:
    def __init__(self, runtime):
        self.runtime = runtime
        self.calls = []
        self.change_map_error = None
        self.change_map_result = LoadMapResponse.RESULT_SUCCESS

    def call(self, name, _proxy, *arguments):
        self.calls.append(name)
        if name == '/change_map':
            if self.change_map_error is not None:
                raise self.change_map_error
            if self.change_map_result == LoadMapResponse.RESULT_SUCCESS:
                self.runtime.loaded = '4F'
                self.runtime.generation += 1
            return LoadMapResponse(result=self.change_map_result)
        return None


class TransitionFaultNodeTest(unittest.TestCase):
    def setUp(self):
        self.identities = {'3F': 'identity-3F', '4F': 'identity-4F'}
        runtime = FakeRuntime(self.identities)
        node = MultifloorManagerNode.__new__(MultifloorManagerNode)
        node.runtime = runtime
        node.identities = self.identities
        node.services = Services(runtime)
        node.server = N(is_preempt_requested=lambda: False, set_succeeded=Mock(), set_aborted=Mock(),
                        set_preempted=Mock(), publish_feedback=Mock())
        node.initialpose_publisher = N(publish=Mock())
        node.config_root = Path('/tmp')
        node.change_map_name, node.change_map = '/change_map', None
        node.clear_costmaps_name, node.clear_costmaps = '/move_base/clear_costmaps', None
        node.tag_timeout = node.transaction_timeout = node.stair_result_timeout = 1.
        node.resume_max_age_sec = 300.
        node.floor_confirmation_source = 'stair_result'
        node._stair_result_lock = __import__('threading').Lock()
        node.stair_completions = N(consume=Mock(return_value=5))
        node.floors = {'3F': N(map_yaml=Path('3F.yaml')), '4F': N(map_yaml=Path('4F.yaml'))}
        covariance = (0.,) * 36
        node.stairs = {'stair_a': Stair('stair_a', '3F', '4F', (
            StairEndpoint('3F', 'entry_3f', LandingPose(0., 0., Quaternion(0., 0., 0., 1.)), covariance, ()),
            StairEndpoint('4F', 'entry_4f', LandingPose(1., 2., Quaternion(0., 0., 0., 1.)), covariance, ()),
        ), 'up', 'down')}
        node._wait = lambda predicate, deadline: (bool(predicate()), '')
        self.localization_failures = []
        def refresh(predicate, deadline):
            if self.localization_failures:
                return False, self.localization_failures.pop(0)
            return True, ''
        node._wait_with_localization_refresh = refresh
        node._wait_for_policy_ready = lambda *_args: (True, '')
        node._target_metadata_matches = lambda *_args: True
        self.node, self.runtime = node, runtime
        self.patches = [patch.object(rospy, name, Mock()) for name in ('set_param', 'logerr', 'logwarn')]
        self.patches.append(patch.object(rospy.Time, 'now', return_value=rospy.Time(100)))
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)

    def run_goal(self):
        self.node._run_transaction('stair_a', '4F')

    def aborted_reason(self):
        return self.node.server.set_aborted.call_args[0][1]

    def test_failure_after_map_change_records_loaded_map_and_resumes_without_reload(self):
        self.localization_failures = ['transition evidence timeout']
        self.run_goal()
        state, detail, floor = self.runtime.finished[-1]
        # floor_id keeps the last confirmed floor; the loaded map is stated beside it.
        self.assertEqual((state, floor), (FloorState.FAULT, '3F'))
        self.assertEqual(detail, 'transition evidence timeout [stage=INITIALPOSE; map=4F; '
                                 'confirmed_floor=3F; target=4F; target_confirmed=yes]')
        self.assertEqual(self.aborted_reason(), detail)
        self.assertEqual(self.node._last_fault.as_dict()['map_floor'], '4F')
        rospy.set_param.assert_called_with('~last_fault', self.node._last_fault.as_dict())

        self.node.server.set_aborted.reset_mock()
        self.node.services.calls.clear()
        self.run_goal()
        self.assertEqual(self.runtime.finished[-1], (FloorState.READY, 'floor transition complete', '4F'))
        self.assertNotIn('/change_map', self.node.services.calls)
        self.assertEqual(self.node.stair_completions.consume.call_count, 1)
        self.assertEqual(self.node.initialpose_publisher.publish.call_count, 2)
        self.node.server.set_aborted.assert_not_called()
        self.assertIsNone(self.node._last_fault)
        phases = [call.args[0].phase for call in self.node.server.publish_feedback.call_args_list]
        self.assertIn('RESUME', phases)

    def test_ambiguous_change_map_records_unknown_map_and_resume_reloads(self):
        self.node.services.change_map_error = ServiceCallError('/change_map', 'timeout')
        self.run_goal()
        self.assertIn('[stage=CHANGE_MAP; map=unknown', self.runtime.finished[-1][1])
        self.assertEqual(self.runtime.finished[-1][2], '3F')
        self.node.services.change_map_error = None
        self.run_goal()
        self.assertEqual(self.node.services.calls.count('/change_map'), 2)
        self.assertEqual(self.runtime.finished[-1][0], FloorState.READY)
        self.assertEqual(self.node.stair_completions.consume.call_count, 1)

    def test_rejected_change_map_keeps_source_map_and_resume_retries_it(self):
        self.node.services.change_map_result = LoadMapResponse.RESULT_UNDEFINED_FAILURE
        self.run_goal()
        self.assertIn('[stage=CHANGE_MAP; map=3F;', self.runtime.finished[-1][1])
        self.node.services.change_map_result = LoadMapResponse.RESULT_SUCCESS
        self.run_goal()
        self.assertEqual(self.runtime.finished[-1][:3:2], (FloorState.READY, '4F'))

    def test_failure_before_target_confirmation_is_not_resumable(self):
        self.node._wait = lambda predicate, deadline: (False, 'transition evidence timeout')
        self.run_goal()
        self.assertIn('target_confirmed=no', self.runtime.finished[-1][1])
        self.node._wait = lambda predicate, deadline: (bool(predicate()), '')
        self.node.server.set_aborted.reset_mock()
        self.run_goal()
        result = self.node.server.set_aborted.call_args[0][0]
        self.assertEqual(result.result_code, FloorTransitionResult.INVALID_GOAL)

    def test_floor_reselection_or_other_goal_invalidates_resume(self):
        self.localization_failures = ['timeout']
        self.run_goal()
        self.runtime.generation += 1  # e.g. /multifloor/select_floor reloaded a map
        self.node.server.set_aborted.reset_mock()
        self.run_goal()
        result = self.node.server.set_aborted.call_args[0][0]
        self.assertEqual(result.result_code, FloorTransitionResult.INVALID_GOAL)

    def test_plain_fail_without_transaction_keeps_existing_contract(self):
        node = MultifloorManagerNode.__new__(MultifloorManagerNode)
        node.runtime = N(current_floor='5F', initial_floor='3F', finish_transition=Mock())
        node.server = N(set_aborted=Mock())
        node._result = Mock()
        node._fail('AMCL update missing')
        node.runtime.finish_transition.assert_called_once_with(FloorState.FAULT, 'AMCL update missing', '5F')


if __name__ == '__main__':
    unittest.main()
