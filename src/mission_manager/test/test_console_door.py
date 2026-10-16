"""No external network: MQTT door requests use injected recording functions."""
import copy
import math
import unittest
import threading
import time
from unittest.mock import patch
from mission_manager import console_door
from mission_manager.console_door import ConsoleDoor


class DoorAdapterTest(unittest.TestCase):
    def test_explicit_device_selects_correct_provided_sdk_function(self):
        calls=[]
        class FakeSdk:
            @staticmethod
            def RF_Open(seconds):calls.append(('RF',seconds));return 'rf-ack'
            @staticmethod
            def F5_Open(seconds):calls.append(('F5',seconds));return 'f5-ack'
        with patch.object(console_door,'_sdk',return_value=FakeSdk):
            self.assertEqual(console_door.request_door('RF',60),'rf-ack')
            self.assertEqual(console_door.request_door('F5',30),'f5-ack')
        self.assertEqual(calls,[('RF',60),('F5',30)])
        with self.assertRaises(ValueError):console_door.request_door('RF',601)

class DoorTest(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.sent=[]
        self.state=dict(mode='live',active=True,telemetry={
            'supervisor':dict(age_sec=0.,value=dict(state=2,connected=True)),
            'floor':dict(age_sec=0.,value=dict(floor_id='5F',state=2)),
            'control':dict(age_sec=0.,value=dict(route_id='stair_5f_rf_up',run_id='test:1',phase='ROOFTOP_TURN',target=[0,0,3.23,math.pi/2],loss_response=False))})
        self.door=ConsoleDoor(lambda:self.state,sender=self.send,clock=lambda:self.now,threaded=False)
        self.addCleanup(self.door.close)
    def send(self,device,seconds):self.sent.append((self.now,device,seconds));return 'cmd-%d'%len(self.sent)
    def tick(self,now):self.now=now;self.door.tick()
    def control(self):return self.state['telemetry']['control']['value']
    def wait_manual(self):
        for _ in range(100):
            if not self.door.snapshot()['one_shot']['busy']:return
            time.sleep(.01)
        self.fail('manual door worker blocked')
    def test_second_ascent_roof_turn_sends_one_rf_hold_without_periodic_retry(self):
        for t in (0.,.5,19.9,20.,20.5,39.9,40.):self.tick(t)
        self.assertEqual(self.sent,[(0.,'RF',60)])
        self.assertFalse(self.door.snapshot()['physical_door_verified'])
        self.assertEqual(self.door.snapshot()['last_command_id'],'cmd-1')
    def test_centering_first_landing_other_route_preview_and_stale_do_not_actuate(self):
        base=copy.deepcopy(self.state)
        for cause in ('centering','other_route','preview','stale','idle','disconnected','loss'):
            self.state=copy.deepcopy(base)
            if cause=='centering':self.control()['phase']='FORWARD_SEGMENT_2'
            if cause=='other_route':self.control()['route_id']='stair_3f_4f_up'
            if cause=='preview':self.state['mode']='preview'
            if cause=='stale':self.state['telemetry']['control']['age_sec']=3.
            if cause=='idle':self.state['active']=False
            if cause=='disconnected':self.state['telemetry']['supervisor']['value']['connected']=False
            if cause=='loss':self.control()['loss_response']=True
            self.tick(0.);self.assertFalse(self.door.active,cause)
        self.assertEqual(self.sent,[])
    def test_missing_completion_evidence_does_not_send_again(self):
        self.tick(0.);self.control().update(phase='EXIT_CONFIRM',phase_complete=True,exit_supported=True)
        self.tick(20.);self.assertEqual(self.sent,[(0.,'RF',60)]);self.assertTrue(self.door.active)
    def test_old_run_cannot_rearm_after_hold_expires(self):
        self.tick(0.);self.control().update(phase='EXIT_CONFIRM',phase_complete=True,exit_supported=True,completion_checks=dict(final_height=True,all_flights_seen=True))
        self.tick(20.);self.assertTrue(self.door.active)
        self.control()['phase']='ROOFTOP_TURN';self.tick(320.);self.assertFalse(self.door.active)
        self.assertEqual(self.sent,[(0.,'RF',60)])
    def test_confirmed_rf_floor_does_not_cancel_device_hold(self):
        self.tick(0.);self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.tick(20.);self.assertTrue(self.door.active)
    def test_fault_or_missing_telemetry_does_not_retry_ambiguous_command(self):
        self.tick(0.);self.state=dict(mode='live',active=False,telemetry={})
        self.tick(20.);self.assertEqual(self.sent,[(0.,'RF',60)]);self.assertTrue(self.door.active)
    def test_ui_stop_cannot_cancel_device_hold(self):
        self.tick(0.);self.door.stop({});self.tick(20.)
        self.assertTrue(self.door.active);self.assertEqual(self.sent,[(0.,'RF',60)])
    def test_new_run_can_rearm_after_completed_run(self):
        self.tick(0.);self.control()['run_id']='test:2';self.tick(320.)
        self.assertEqual(self.sent,[(0.,'RF',60),(320.,'RF',60)])
    def test_manual_return_requires_fresh_rf_floor_then_requests_only_door(self):
        with self.assertRaises(ValueError):self.door.manual_return({})
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.state['telemetry']['floor']['age_sec']=3.
        with self.assertRaises(ValueError):self.door.manual_return({})
        self.state['telemetry']['floor']['age_sec']=0.
        self.state['active']=False
        self.door.manual_return({});self.wait_manual();self.tick(20.)
        self.assertEqual(self.sent,[(0.,'RF',60)]);self.assertFalse(self.state['active'])
    def test_manual_return_can_refresh_existing_hold(self):
        self.tick(0.);self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.door.manual_return({});self.wait_manual()
        self.assertEqual(self.sent,[(0.,'RF',60),(0.,'RF',60)])
    def test_manual_roof_door_is_independent_of_mission_floor_and_stale_owner(self):
        self.state.update(active=False,recovering=True,relocating=True)
        self.state['telemetry']={}
        self.door.manual_open({'device':'RF'});self.wait_manual()
        self.now=10.
        self.door.manual_open({'device':'RF'});self.wait_manual()
        self.assertEqual(self.sent,[(0.,'RF',60),(10.,'RF',60)])
        self.assertFalse(self.state['active'])
    def test_return_arrival_at_five_does_not_cancel_device_hold(self):
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.state['active']=False;self.door.manual_return({});self.wait_manual();self.tick(0.)
        self.state['telemetry']['floor']['value']['floor_id']='5F';self.tick(20.)
        self.assertTrue(self.door.active);self.assertEqual(self.sent,[(0.,'RF',60)])
    def test_network_error_is_visible_and_never_auto_retried(self):
        def failed(device,seconds):self.sent.append((self.now,device,seconds));raise TimeoutError()
        self.door.sender=failed;self.tick(0.);self.assertIn('TimeoutError',self.door.snapshot()['error'])
        self.tick(20.);self.assertEqual(self.sent,[(0.,'RF',60)]);self.assertFalse(self.door.active)
    def test_missing_device_ack_id_is_not_reported_as_open(self):
        self.door.sender=lambda device,seconds:None;self.tick(0.)
        self.assertIn('command ID',self.door.snapshot()['error']);self.assertFalse(self.door.snapshot()['physical_door_verified'])
    def test_close_prevents_future_requests(self):
        self.tick(0.);self.door.close();self.tick(20.);self.assertEqual(self.sent,[(0.,'RF',60)])
    def test_return_route_hook_uses_existing_telemetry_no_new_fsm(self):
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.control().update(route_id='stair_5f_rf_down',phase='FORWARD_SEGMENT_1',target=[])
        self.tick(0.);self.assertEqual(self.door.mode,'return');self.assertEqual(self.sent,[(0.,'RF',60)])

    def test_cli_phase_test_also_triggers_door_without_mission_goal(self):
        self.state['active']=False;self.control()['phase_test']=dict(state='STOPPED')
        self.tick(0.);self.assertEqual(self.sent,[])
        self.control()['phase_test']['state']='RUNNING';self.tick(.5)
        self.assertEqual(self.sent,[(.5,'RF',60)])
    def test_bad_telemetry_containers_do_not_repeat(self):
        self.tick(0.)
        for i,state in enumerate((None,[],dict(telemetry=None),dict(telemetry=dict(control=None)),dict(telemetry=dict(control=dict(age_sec=0.,value=dict(completion_checks=None)))))):
            self.state=state;self.tick(20.*(i+1));self.assertEqual(self.door.active,self.now<60.)
        self.assertEqual(len(self.sent),1)
    def test_older_tick_cannot_duplicate_manual_rf_hold(self):
        captured=threading.Event();release=threading.Event();old=copy.deepcopy(self.state)
        def read():
            if threading.current_thread().name=='old-door-tick':
                captured.set();release.wait(1.);return old
            return self.state
        self.door.read_state=read
        thread=threading.Thread(target=self.door.tick,name='old-door-tick');thread.start()
        self.assertTrue(captured.wait(1.))
        self.state['telemetry']['floor']['value']['floor_id']='RF';self.door.manual_return({});self.wait_manual()
        release.set();thread.join(1.);self.assertFalse(thread.is_alive())
        self.assertTrue(self.door.active)
        self.tick(0.);self.assertEqual(self.sent,[(0.,'RF',60)])
    def test_threaded_worker_sends_once_after_browser_is_absent(self):
        sent=threading.Event();times=[]
        def send(device,seconds):times.append((self.now,device,seconds));sent.set();return 'ack'
        d=ConsoleDoor(lambda:self.state,sender=send,clock=lambda:self.now,threaded=True)
        try:
            self.assertTrue(sent.wait(1.));sent.clear();self.now=20.;d.wake.set()
            time.sleep(.02);self.assertEqual(times,[(0.,'RF',60)])
        finally:d.close()
        self.assertFalse(d.thread.is_alive())
    def test_ui_stop_remains_responsive_with_blocked_mqtt_sender(self):
        entered=threading.Event();release=threading.Event()
        def send(device,seconds):entered.set();release.wait(2.);return 'ack'
        d=ConsoleDoor(lambda:self.state,sender=send,clock=lambda:self.now,threaded=True)
        try:
            self.assertTrue(entered.wait(1.));start=time.monotonic();d.stop({})
            self.assertLess(time.monotonic()-start,.2);self.assertFalse(d.active)
        finally:release.set();d.close()
        self.assertFalse(d.thread.is_alive())
    def test_worker_exception_is_exposed_and_retried(self):
        d=ConsoleDoor(lambda:self.state,sender=lambda device,seconds:'ack',threaded=False)
        try:
            with patch.object(d,'tick',side_effect=RuntimeError('fixture')):
                d.thread=threading.Thread(target=d._loop,daemon=True);d.thread.start()
                deadline=time.monotonic()+1.
                while not d.snapshot()['worker_error'] and time.monotonic()<deadline:time.sleep(.01)
                self.assertIn('RuntimeError',d.snapshot()['worker_error']);self.assertTrue(d.thread.is_alive())
        finally:d.close()

    def test_manual_rf_hold_prevents_duplicate_downhill_auto_command(self):
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.door.manual_return({});self.wait_manual();self.tick(0.)
        self.control().update(route_id='stair_5f_rf_down',phase='FORWARD_SEGMENT_1')
        self.tick(.5)
        self.door.stop({});self.tick(20.)
        self.assertTrue(self.door.active);self.assertEqual(self.sent,[(0.,'RF',60)])

    def test_five_floor_nav_start_opens_f5_once(self):
        self.state['telemetry']['control']['age_sec']=3.
        self.state['progress']=dict(mission_id='mission-1',current_floor='5F',
                                    segment_type='NAVIGATION',target_id='stair_5f_to_rf')
        self.tick(0.);self.tick(20.)
        self.assertEqual(self.sent,[(0.,'F5',30)])

    def test_downhill_final_flight_opens_f5_independent_of_rf(self):
        self.state['telemetry']['floor']['value']['floor_id']='RF'
        self.control().update(route_id='stair_5f_rf_down',phase='VERIFY_ENTRY')
        self.tick(0.)
        self.control()['phase']='FORWARD_SEGMENT_3'
        self.tick(30.)
        self.assertEqual(self.sent,[(0.,'RF',60),(30.,'F5',30)])

    def test_manual_request_in_flight_blocks_same_device_auto_duplicate(self):
        entered=threading.Event();release=threading.Event()
        def blocked(device,seconds):
            self.sent.append((self.now,device,seconds));entered.set();release.wait(1.)
            return 'ack'
        self.door.sender=blocked
        self.door.manual_open({'device':'RF'})
        self.assertTrue(entered.wait(1.))
        self.tick(0.)
        self.assertEqual(self.sent,[(0.,'RF',60)])
        with self.assertRaises(ValueError):self.door.manual_open({'device':'RF'})
        release.set();self.wait_manual()
        self.tick(1.)
        self.assertEqual(self.sent,[(0.,'RF',60)])

if __name__=='__main__':unittest.main()
