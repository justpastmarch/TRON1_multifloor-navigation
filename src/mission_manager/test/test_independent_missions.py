"""Regression cases for isolated point NAV, rooftop patrol and flat relocation."""
import io
import json
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from mission_manager.msg import MissionGoal
from mission_manager.mission_action_server import admit_goal
from mission_manager.mission_types import MissionRunRequest, MissionType, SegmentExecution, SegmentExecutionStatus
from mission_manager.route_planner import LogicalAnchor
from mission_manager.console_navigation import _relocate_worker, _confirmed_idle, relocate
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))  # sibling test fixtures
from test_ros_console import TestAdapter
from test_photo_mission import PhotoTourTest


class PointConsoleTests(unittest.TestCase):
    def make(self):
        c = TestAdapter.make(self)
        c.telemetry['floor'] = (time.monotonic(), dict(state=2, floor_id='RF', map_generation=12))
        c.telemetry['supervisor'] = (time.monotonic(), dict(state=1, connected=True))
        c.map_view.image.return_value = dict(revision='roof-map', frame='map', width=100,
                                             height=100, resolution=.1, origin=[-5., -5., 0.])
        c.locations = [dict(id='roof_loop_se', floor='RF')]
        c.relocating = c.floor_selecting = c.recovering = c.operator_pending = False
        return c

    def data(self, **updates):
        return dict(dict(kind='navigate_point', floor_id='RF', map_revision='roof-map',
                         x=.15, y=-.45, yaw=.7, record=False), **updates)

    def test_point_goal_serializes_coordinates_and_does_not_touch_route_graph(self):
        c = self.make(); c.planner = Mock()
        self.assertTrue(c.start(self.data())['submitted'])
        goal = c.client.send_goal.call_args.args[0]
        self.assertEqual(goal.mission_type, 'navigate_point')
        self.assertEqual((goal.point_floor, goal.point_map_generation), ('RF', 12))
        self.assertEqual((goal.point_x, goal.point_y, goal.point_yaw), (.15, -.45, .7))
        buffer = io.BytesIO(); goal.serialize(buffer)
        decoded = MissionGoal().deserialize(buffer.getvalue())
        self.assertEqual(decoded, goal)
        c.planner.plan_from_current_pose.assert_not_called()

    def test_stale_map_wrong_floor_nonfinite_and_outside_goals_do_not_dispatch(self):
        for update in [dict(map_revision='old'), dict(floor_id='5F'), dict(x=float('nan')),
                       dict(x=True), dict(x=100), dict(yaw='0')]:
            c = self.make()
            with self.subTest(update=update), self.assertRaises(ValueError):
                c.start(self.data(**update))
            c.client.send_goal.assert_not_called()

    def test_recorder_delay_rechecks_floor_before_dispatch(self):
        c = self.make()
        c.recorder.start.side_effect = lambda _: c.telemetry['floor'][1].update(floor_id='5F')
        with self.assertRaises(ValueError):c.start(self.data(record=True))
        c.client.send_goal.assert_not_called();self.assertFalse(c.active)

    def test_point_failure_then_named_navigation_and_late_callback_are_isolated(self):
        c = self.make();c.start(self.data());old=c.generation
        c.done(4, NS(__slots__=['reason'], reason='blocked'), old)
        c.start(self.data(x=.6));new=c.generation
        c.done(4, None, old)
        self.assertTrue(c.active);self.assertEqual(c.generation,new)
        c.done(3, None, new);self.assertFalse(c.active)

    def test_photo_session_does_not_block_point_nav(self):
        c=self.make();c.orchestrator.photo=Mock();c.orchestrator.photo.preflight.side_effect=ValueError('return pending')
        c.start(self.data())
        c.orchestrator.photo.preflight.assert_not_called()

    def test_point_admission_rejects_incoherent_fields(self):
        goal=MissionGoal(destination_id='ui_point',mission_type='navigate_point',point_floor='RF',point_x=.3)
        self.assertTrue(admit_goal(goal,frozenset()).accepted)
        goal.point_x=float('inf');self.assertFalse(admit_goal(goal,frozenset()).accepted)
        goal.point_x=0.;goal.resume_run_id='old';self.assertFalse(admit_goal(goal,frozenset()).accepted)
        goal.resume_run_id='';goal.mission_type='navigate'
        self.assertFalse(admit_goal(goal,frozenset({'ui_point'})).accepted)


class IndependentTourTests(PhotoTourTest):
    def roof_request(self, name='roof_only'):
        return MissionRunRequest(name,'roof_loop_se',MissionType.ROOFTOP_TOUR,False)

    def test_roof_walk_precedes_navigation(self):
        self.host._anchor = LogicalAnchor('stair_rf_landing')
        result = self.host.run(self.roof_request(), lambda p:None, lambda:False)
        self.assertEqual(result.status, SegmentExecutionStatus.SUCCESS)
        self.assertEqual(self.executor.events[0][0], 'WALK')
        self.assertEqual(sum(e[0]=='WALK' for e in self.executor.events), 1)

    def test_walk_failure_never_sends_nav(self):
        self.host._anchor = LogicalAnchor('stair_rf_landing')
        def fail(): raise ValueError('WALK transport failed')
        self.executor.prepare_rooftop_navigation = fail
        result = self.host.run(self.roof_request(), lambda p:None, lambda:False)
        self.assertNotEqual(result.status, SegmentExecutionStatus.SUCCESS)
        self.assertEqual(self.executor.events, [])

    def test_failed_ascent_manual_roof_then_patrol_retains_original_return(self):
        self.executor.fail_stair=True
        self.assertEqual(self.run_tour().status,SegmentExecutionStatus.FAILED)
        original=self.photo.snapshot()['origin']
        self.host._anchor=LogicalAnchor('stair_rf_landing');self.executor.events.clear()
        result=self.host.run(self.roof_request(),lambda _:None,lambda:False)
        self.assertEqual(result.status,SegmentExecutionStatus.SUCCESS)
        self.assertEqual(self.photo.snapshot()['origin'],original)
        self.assertFalse(any(x[0] in ('STAIR','FLOOR_TRANSITION') for x in self.executor.events))
        self.assertEqual([x[1] for x in self.executor.events if x[0]=='PHOTO'],self.settings['locations'])

    def test_old_waiting_return_does_not_block_new_roof_only_tour(self):
        self.run_tour();old=self.photo.snapshot()['mission_id']
        self.host._anchor=LogicalAnchor('stair_rf_landing');self.executor.events.clear()
        result=self.host.run(self.roof_request('fresh_roof'),lambda _:None,lambda:False)
        self.assertEqual(result.status,SegmentExecutionStatus.SUCCESS)
        self.assertEqual(self.photo.snapshot()['origin']['floor_id'],'RF')
        self.assertTrue((self.photo.output/old/'session.json').exists())
        self.assertFalse(any(x[0]=='STAIR' for x in self.executor.events))

    def test_roof_only_rejects_wrong_floor_without_any_segment(self):
        result=self.host.run(self.roof_request(),lambda _:None,lambda:False)
        self.assertEqual(result.status,SegmentExecutionStatus.FAILED)
        self.assertEqual(self.executor.events,[])

    def test_point_navigation_preserves_named_anchor_and_photo_session(self):
        self.executor.navigate_point=Mock(return_value=SegmentExecution.success())
        anchor=self.host.confirmed_location_id;session=self.photo.snapshot()
        request=MissionRunRequest('point','ui_point',MissionType.NAVIGATE_POINT,False,
                                  point_floor='5F',point_map_generation=4,point_x=1.,point_y=2.,point_yaw=.3)
        result=self.host.run(request,lambda _:None,lambda:False)
        self.assertEqual(result.status,SegmentExecutionStatus.SUCCESS)
        self.assertEqual(self.host.confirmed_location_id,anchor)
        self.assertEqual(self.photo.snapshot(),session)
        self.executor.navigate_point.assert_called_once()


class FlatRelocationTests(PointConsoleTests):
    def prepare(self):
        c=self.make();c.relocating=True;c.relocation_stop=threading.Event()
        c.relocation=dict(state='PREPARING',floor_id='RF')
        c.telemetry['floor'][1].update(floor_id='5F')
        c.telemetry['supervisor'][1].update(state=2)
        c.telemetry['mission_status']=(time.monotonic(),dict(active=False))
        return c

    def services(self,c, calls):
        def factory(name,kind):
            def call(*args):
                calls.append(name)
                if name.endswith('acknowledge_physical_handoff'):c.telemetry['supervisor'][1]['state']=1
                if name.endswith('select_floor'):c.telemetry['floor'][1].update(floor_id=args[0],state=0)
                return NS(success=True,accepted=True,message='',reason='')
            return call
        return factory

    def test_retained_stair_then_nav_then_map_and_no_motion_goal(self):
        c=self.prepare();calls=[]
        with patch('rospy.wait_for_service'),patch('rospy.set_param'),patch('rospy.ServiceProxy',side_effect=self.services(c,calls)):
            _relocate_worker(c,'RF')
        self.assertFalse(c.relocating);self.assertEqual(c.relocation['state'],'DONE')
        self.assertEqual(calls,['/stair_supervisor/acknowledge_physical_handoff','/mission_manager/stop_arrival_hold','/multifloor/select_floor'])
        c.client.send_goal.assert_not_called()

    def test_late_mode_reply_cannot_release_preparation_and_allow_new_motion(self):
        c=self.prepare();entered=threading.Event();release=threading.Event();calls=[]
        base=self.services(c,calls)
        def proxy(name,kind):
            if name.endswith('acknowledge_physical_handoff'):
                def call():
                    entered.set();release.wait(3)
                    return base(name,kind)()
                return call
            return base(name,kind)
        with patch('rospy.wait_for_service'),patch('rospy.set_param'),patch('rospy.ServiceProxy',side_effect=proxy):
            thread=threading.Thread(target=_relocate_worker,args=(c,'RF'));thread.start()
            self.assertTrue(entered.wait(1));c.relocation_stop.set()
            self.assertTrue(c.relocating)
            with self.assertRaisesRegex(ValueError,'준비 중'):c.start(self.data())
            release.set();thread.join(2)
        self.assertFalse(thread.is_alive());self.assertFalse(c.relocating)
        self.assertNotIn('/multifloor/select_floor',calls)

    def test_failed_service_releases_operation_for_retry(self):
        c=self.prepare()
        with patch('rospy.set_param'), patch('rospy.wait_for_service',side_effect=RuntimeError('offline')):
            _relocate_worker(c,'RF')
        self.assertFalse(c.relocating);self.assertEqual(c.relocation['state'],'FAILED')

    def test_recorder_error_does_not_leave_preparation_busy(self):
        c=self.make();c.recorder.stop.side_effect=RuntimeError('recorder disconnected')
        with self.assertRaisesRegex(RuntimeError,'recorder disconnected'):
            relocate(c,dict(floor_id='RF',confirmed_flat_ground=True))
        self.assertFalse(c.relocating)
        self.assertEqual(c.relocation['state'],'FAILED')

    def test_stale_or_missing_terminal_status_is_not_confirmed_idle(self):
        c=self.prepare()
        self.assertTrue(_confirmed_idle(c))
        c.telemetry['mission_status']=(time.monotonic()-3,dict(active=False))
        self.assertFalse(_confirmed_idle(c))
        c.telemetry.pop('mission_status')
        self.assertFalse(_confirmed_idle(c))

    def test_cancel_during_service_discovery_does_not_send_mode_change(self):
        c=self.prepare()
        with patch('rospy.set_param'),patch('rospy.ServiceProxy') as proxy,patch(
                'rospy.wait_for_service',side_effect=lambda *a,**k:c.relocation_stop.set()):
            _relocate_worker(c,'RF')
        proxy.assert_not_called()
        self.assertFalse(c.relocating);self.assertEqual(c.relocation['state'],'FAILED')
