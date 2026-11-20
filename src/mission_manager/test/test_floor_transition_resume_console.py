"""Console resume of a failed floor transition: same goal only, never a new stair."""
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))  # sibling test fixtures
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from actionlib_msgs.msg import GoalStatus
from multifloor_manager.msg import FloorState
import test_ros_console as _console_fixture  # module import: do not re-collect its tests

FAULT = dict(transition_id='stair_3f_4f', source_floor='3F', target_floor='4F', stage='AMCL_READY',
             target_confirmed=True, map_floor='4F', map_observed=True, map_generation=3,
             failed_at=1.0, reason='localization timeout')


class FloorResumeTest(unittest.TestCase):
    def setUp(self):
        self.adapter = _console_fixture.TestAdapter('test_ui_pose_estimate_publishes_map_frame_pose_without_starting_motion')
        self.adapter.setUp()
        self.addCleanup(self.adapter.doCleanups)
        c = self.adapter.make()
        c.floor_selecting = False
        c.telemetry['mission_status'] = (time.monotonic(), {'active': False})
        c.telemetry['floor'] = (time.monotonic(), {'state': FloorState.FAULT, 'floor_id': '3F'})
        self.console = c

    def run_resume(self, fault=FAULT, state=GoalStatus.SUCCEEDED, data=None):
        client = Mock()
        client.wait_for_server.return_value = True
        client.wait_for_result.return_value = True
        client.get_state.return_value = state
        client.get_result.return_value = NS(reason='rejected')
        with patch('mission_manager.ros_console.rospy.get_param', return_value=fault), \
             patch('mission_manager.ros_console.actionlib.SimpleActionClient', return_value=client):
            result = self.console.resume_floor_transition(
                data if data is not None else dict(transition_id='stair_3f_4f', target_floor='4F'))
        return result, client

    def test_repeats_the_recorded_goal_and_releases_the_floor_lock(self):
        result, client = self.run_resume()
        self.assertTrue(result['accepted'])
        goal = client.send_goal.call_args.args[0]
        self.assertEqual((goal.transition_id, goal.target_floor), ('stair_3f_4f', '4F'))
        self.assertFalse(self.console.floor_selecting)
        self.console.client.send_goal.assert_not_called()  # no mission / stair goal

    def test_refuses_without_confirmed_target_or_fault_state(self):
        with self.assertRaisesRegex(ValueError, '실패 기록이 없습니다'):
            self.run_resume(fault=dict(FAULT, target_confirmed=False))
        self.console.telemetry['floor'] = (time.monotonic(), {'state': FloorState.READY, 'floor_id': '3F'})
        with self.assertRaisesRegex(ValueError, 'FAULT 상태가 아닙니다'):
            self.run_resume()

    def test_refuses_a_changed_record_or_active_mission(self):
        with self.assertRaisesRegex(ValueError, '바뀌었습니다'):
            self.run_resume(data=dict(transition_id='stair_4f_5f', target_floor='5F'))
        self.console.active = True
        with self.assertRaisesRegex(ValueError, '임무'):
            self.run_resume()

    def test_failed_resume_reports_the_floor_manager_reason(self):
        with self.assertRaisesRegex(ValueError, 'rejected'):
            self.run_resume(state=GoalStatus.ABORTED)
        self.assertFalse(self.console.floor_selecting)


if __name__ == '__main__':
    unittest.main()
