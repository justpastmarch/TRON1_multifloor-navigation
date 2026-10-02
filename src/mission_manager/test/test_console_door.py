"""No external network: door requests are injected recording functions."""
import copy
import math
import unittest
import threading
import time
from unittest.mock import patch
from mission_manager.console_door import ConsoleDoor

class DoorTest(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.sent=[]
        self.state=dict(mode='live',active=True,telemetry={
            'supervisor':dict(age_sec=0.,value=dict(state=2,connected=True)),
            'floor':dict(age_sec=0.,value=dict(floor_id='5F',state=2)),
            'control':dict(age_sec=0.,value=dict(route_id='stair_5f_rf_up',run_id='test:1',phase='ROOFTOP_TURN',target=[0,0,3.23,math.pi/2],loss_response=False))})
        self.door=ConsoleDoor(lambda:self.state,sender=self.send,clock=lambda:self.now,threaded=False)
        self.addCleanup(self.door.close)
    def send(self):self.sent.append(self.now);return 200
    def tick(self,now):self.now=now;self.door.tick()
    def control(self):return self.state['telemetry']['control']['value']
    def test_right_turn_starts_once_and_repeats_every_twenty_seconds(self):
        for t in (0.,.5,19.9,20.,20.5,39.9,40.):self.tick(t)
        self.assertEqual(self.sent,[0.,20.,40.]);self.assertFalse(self.door.snapshot()['physical_door_verified'])
    def test_centering_first_landing_other_route_preview_and_stale_do_not_actuate(self):
        base=copy.deepcopy(self.state)
        for cause in ('centering','other_route','preview','stale','idle','disconnected','loss'):
            self.state=copy.deepcopy(base)
            if cause=='centering':self.control()['target'][3]=math.pi
            if cause=='other_route':self.control()['route_id']='stair_3f_4f_up'
            if cause=='preview':self.state['mode']='preview'
            if cause=='stale':self.state['telemetry']['control']['age_sec']=3.
            if cause=='idle':self.state['active']=False
            if cause=='disconnected':self.state['telemetry']['supervisor']['value']['connected']=False
            if cause=='loss':self.control()['loss_response']=True
            self.tick(0.);self.assertFalse(self.door.active,cause)
        self.assertEqual(self.sent,[])
    def test_missing_completion_evidence_keeps_door_request_running(self):
        self.tick(0.);self.control().update(phase='EXIT_CONFIRM',phase_complete=True,exit_supported=True)
        self.tick(20.);self.assertEqual(self.sent,[0.,20.]);self.assertTrue(self.door.active)
    def test_completed_rooftop_exit_stops_and_old_run_cannot_rearm(self):
        self.tick(0.);self.control().update(phase='EXIT_CONFIRM',phase_complete=True,exit_supported=True,completion_checks=dict(final_height=True,all_flights_seen=True))
        self.tick(20.);self.assertFalse(self.door.active);self.assertEqual(self.sent,[0.])
        self.control()['phase']='ROOFTOP_TURN';self.tick(40.);self.assertEqual(self.sent,[0.])
    def test_confirmed_rf_floor_stops_if_transient_phase_completion_was_missed(self):
        self.tick(0.);self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.tick(20.);self.assertFalse(self.door.active)
    def test_fault_or_missing_telemetry_does_not_close_passage_early(self):
        self.tick(0.);self.state=dict(mode='live',active=False,telemetry={})
        self.tick(20.);self.assertEqual(self.sent,[0.,20.]);self.assertTrue(self.door.active)
    def test_user_passage_confirmation_stops_without_robot_command(self):
        self.tick(0.);self.door.stop({});self.tick(20.)
        self.assertFalse(self.door.active);self.assertEqual(self.sent,[0.])
    def test_new_run_can_rearm_after_completed_run(self):
        self.tick(0.);self.door.stop({});self.control()['run_id']='test:2';self.tick(20.)
        self.assertEqual(self.sent,[0.,20.])
    def test_manual_return_requires_fresh_rf_floor_then_requests_only_door(self):
        with self.assertRaises(ValueError):self.door.manual_return({})
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.state['telemetry']['floor']['age_sec']=3.
        with self.assertRaises(ValueError):self.door.manual_return({})
        self.state['telemetry']['floor']['age_sec']=0.
        self.state['active']=False
        self.door.manual_return({});self.tick(0.);self.tick(20.)
        self.assertEqual(self.sent,[0.,20.]);self.assertFalse(self.state['active'])
    def test_manual_return_switches_existing_up_schedule(self):
        self.tick(0.);self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.door.manual_return({});self.tick(.5);self.tick(20.5)
        self.assertEqual(self.door.mode,'return');self.assertEqual(self.sent,[0.,.5,20.5])
    def test_return_arrival_at_five_stops_repeat(self):
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.state['active']=False;self.door.manual_return({});self.tick(0.)
        self.state['telemetry']['floor']['value']['floor_id']='5F';self.tick(20.)
        self.assertFalse(self.door.active);self.assertEqual(self.sent,[0.])
    def test_network_error_is_visible_and_retry_continues(self):
        def failed():self.sent.append(self.now);raise TimeoutError()
        self.door.sender=failed;self.tick(0.);self.assertIn('TimeoutError',self.door.snapshot()['error'])
        self.tick(20.);self.assertEqual(self.sent,[0.,20.]);self.assertTrue(self.door.active)
    def test_non_success_http_is_not_reported_as_open(self):
        self.door.sender=lambda:503;self.tick(0.)
        self.assertIn('503',self.door.snapshot()['error']);self.assertFalse(self.door.snapshot()['physical_door_verified'])
    def test_close_prevents_future_requests(self):
        self.tick(0.);self.door.close();self.tick(20.);self.assertEqual(self.sent,[0.])
    def test_return_route_hook_uses_existing_telemetry_no_new_fsm(self):
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.control().update(route_id='stair_5f_rf_down',phase='FORWARD_SEGMENT_1',target=[])
        self.tick(0.);self.assertEqual(self.door.mode,'return');self.assertEqual(self.sent,[0.])

    def test_cli_phase_test_also_triggers_door_without_mission_goal(self):
        self.state['active']=False;self.control()['phase_test']=dict(state='STOPPED')
        self.tick(0.);self.assertEqual(self.sent,[])
        self.control()['phase_test']['state']='RUNNING';self.tick(.5)
        self.assertEqual(self.sent,[.5])
    def test_bad_telemetry_containers_keep_existing_repeat(self):
        self.tick(0.)
        for i,state in enumerate((None,[],dict(telemetry=None),dict(telemetry=dict(control=None)),dict(telemetry=dict(control=dict(age_sec=0.,value=dict(completion_checks=None)))))):
            self.state=state;self.tick(20.*(i+1));self.assertTrue(self.door.active)
        self.assertEqual(len(self.sent),6)
    def test_older_tick_cannot_stop_new_manual_return_schedule(self):
        captured=threading.Event();release=threading.Event();old=copy.deepcopy(self.state)
        def read():
            if threading.current_thread().name=='old-door-tick':
                captured.set();release.wait(1.);return old
            return self.state
        self.door.read_state=read
        thread=threading.Thread(target=self.door.tick,name='old-door-tick');thread.start()
        self.assertTrue(captured.wait(1.))
        self.state['telemetry']['floor']['value']['floor_id']='RF';self.door.manual_return({})
        release.set();thread.join(1.);self.assertFalse(thread.is_alive())
        self.assertTrue(self.door.active);self.assertEqual(self.door.mode,'return')
        self.tick(0.);self.assertEqual(self.sent,[0.])
    def test_threaded_worker_repeats_after_browser_is_absent(self):
        sent=threading.Event();times=[]
        def send():times.append(self.now);sent.set();return 200
        d=ConsoleDoor(lambda:self.state,sender=send,clock=lambda:self.now,threaded=True)
        try:
            self.assertTrue(sent.wait(1.));sent.clear();self.now=20.;d.wake.set()
            self.assertTrue(sent.wait(1.));self.assertEqual(times,[0.,20.])
        finally:d.close()
        self.assertFalse(d.thread.is_alive())
    def test_threaded_stop_remains_responsive_with_blocked_http_sender(self):
        entered=threading.Event();release=threading.Event()
        def send():entered.set();release.wait(2.);return 200
        d=ConsoleDoor(lambda:self.state,sender=send,clock=lambda:self.now,threaded=True)
        try:
            self.assertTrue(entered.wait(1.));start=time.monotonic();d.stop({})
            self.assertLess(time.monotonic()-start,.2);self.assertFalse(d.active)
        finally:release.set();d.close()
        self.assertFalse(d.thread.is_alive())
    def test_worker_exception_is_exposed_and_retried(self):
        d=ConsoleDoor(lambda:self.state,sender=lambda:200,threaded=False)
        try:
            with patch.object(d,'tick',side_effect=RuntimeError('fixture')):
                d.thread=threading.Thread(target=d._loop,daemon=True);d.thread.start()
                deadline=time.monotonic()+1.
                while not d.snapshot()['worker_error'] and time.monotonic()<deadline:time.sleep(.01)
                self.assertIn('RuntimeError',d.snapshot()['worker_error']);self.assertTrue(d.thread.is_alive())
        finally:d.close()

    def test_manual_return_adopts_descent_run_so_passage_stop_cannot_rearm_it(self):
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.door.manual_return({});self.tick(0.)
        self.control().update(route_id='stair_5f_rf_down',phase='FORWARD_SEGMENT_1')
        self.tick(.5);self.assertEqual(self.door.run_id,'test:1')
        self.door.stop({});self.tick(20.)
        self.assertFalse(self.door.active);self.assertEqual(self.sent,[0.])

if __name__=='__main__':unittest.main()
