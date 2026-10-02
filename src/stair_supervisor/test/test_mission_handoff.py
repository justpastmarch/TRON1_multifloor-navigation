"""Offline mission exit/hold regressions; fake robot transport only."""
from dataclasses import replace
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from test_lidar_control import Worker,route,sample
from test_supervisor import FakeClock,FakeTransport,FakeAdmission,ScriptedEvidence,make_configuration
from stair_supervisor.stair_feedback import StairFeedback,se2,inside_polygon
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.supervisor import StairSupervisor,SupervisorState,ResultCode


class MissionHandoffTest(unittest.TestCase):
    def test_exit_settling_has_independent_budget_from_mode_ack(self):
        for separate in (False, True):
            class Control(ScriptedEvidence):
                phase=Phase.EXIT_CONFIRM
                route={'limits':{'handoff_sec':3., **({'exit_settle_sec':15.} if separate else {})}}
                def command(self):return (.01,0.)
                def begin_arrival_hold(self):self.hold_started=True
                def loss_response(self,t):t.events.append(('loss',))
            control=Control(observations_before_true=160);clock=FakeClock();transport=FakeTransport()
            supervisor=StairSupervisor(make_configuration(),transport,control,clock,lambda r:None,FakeAdmission(),.25,lidar_control=control)
            supervisor.start();supervisor._state=SupervisorState.STAIR
            supervisor._set_lidar_mode=lambda *args:None
            result=supervisor._finish(cancelled=False)
            self.assertEqual(result.code==ResultCode.OK,separate)
            self.assertEqual(control.route['limits']['handoff_sec'],3.)

    def policy(self):
        worker=Worker();control=StairFeedback(worker,np.eye(4),[route()])
        control.capture_entry('test_up',np.eye(4),.01,'synthetic survey',100.)
        control.arm(replace(make_configuration().profiles[0],timeout_sec=60.),100.)
        control._flight_seen=[True,True]
        control.begin_phase(Phase.EXIT_CONFIRM,100.)
        return worker,control

    def test_arrival_wait_is_not_traversal_timeout_but_missing_sensor_is_fault(self):
        worker,control=self.policy()
        worker.value=sample(2,161.,y=1.,z=2.,yaw=math.pi)
        self.assertTrue(control.evaluate(Phase.EXIT_CONFIRM,161.01).faulted)
        worker,control=self.policy();control.begin_arrival_hold()
        worker.value=sample(2,161.,y=1.,z=2.,yaw=math.pi)
        self.assertFalse(control.evaluate(Phase.EXIT_CONFIRM,161.01).faulted)
        self.assertTrue(control.evaluate(Phase.EXIT_CONFIRM,162.).faulted)

    def test_hold_does_not_disable_edge_attitude_or_epoch_checks(self):
        for values,epoch in (({'y':3.},0),({'y':1.},1)):
            worker,control=self.policy();control.begin_arrival_hold()
            worker.value=sample(2,200.,z=2.,yaw=math.pi,**values)
            worker.epoch=epoch
            self.assertTrue(control.evaluate(Phase.EXIT_CONFIRM,200.01).faulted)

    def test_hold_cannot_be_started_in_flight_or_without_both_flights(self):
        worker,control=self.policy();control._flight_seen=[True,False]
        with self.assertRaises(ValueError):control.begin_arrival_hold()
        control._flight_seen=[True,True];control.begin_phase(Phase.FORWARD_SEGMENT_2,100.)
        with self.assertRaises(ValueError):control.begin_arrival_hold()

    def test_finished_supervisor_holds_beyond_three_seconds_until_nav_takeover(self):
        class Control(ScriptedEvidence):
            phase=Phase.EXIT_CONFIRM
            route={'limits':{'handoff_sec':3.}}
            def command(self):return (.01,.02)
            def begin_arrival_hold(self):self.hold_started=True
            def loss_response(self,t):t.events.append(('loss',))
        control=Control(observations_before_true=-1);clock=FakeClock();transport=FakeTransport()
        supervisor=StairSupervisor(make_configuration(),transport,control,clock,lambda r:None,FakeAdmission(),.25,lidar_control=control)
        supervisor.start();supervisor._state=SupervisorState.STAIR
        supervisor._set_lidar_mode=lambda *args:None
        self.assertEqual(supervisor._finish(cancelled=False).code,ResultCode.OK)
        self.assertTrue(control.hold_started)
        clock.sleep(100.)
        supervisor.accept_navigation(0.,0.);supervisor.stream_navigation()
        self.assertTrue(supervisor._arrival_hold)
        self.assertEqual(transport.events[-2],('update',.01,.02))
        supervisor.accept_navigation(.1,0.);supervisor.stream_navigation()
        self.assertFalse(supervisor._arrival_hold)
        self.assertEqual(transport.events[-2],('update',.1,0.))
        self.assertNotIn(('loss',),transport.events)

    def test_updated_second_entry_supports_square_rotation_with_existing_reserve(self):
        path=Path(__file__).resolve().parents[1]/'config/stair_lidar_3f_4f_test.yaml'
        r=yaml.safe_load(path.read_text())['routes'][0]
        x,y,_=r['turn_path'][-1]
        self.assertAlmostEqual(x-r['flight_2'][0][0],.45)
        footprint=np.array(r['footprint']);margin=r['limits']['margin_m']+r['limits']['anchor_uncertainty_m']
        for yaw in np.linspace(-math.pi,math.pi,73):
            m=se2(x,y,yaw)
            body=footprint@m[:2,:2].T+m[:2,3]
            self.assertTrue(inside_polygon(body,r['landing_polygon'],margin))
        m=se2(2.875,y,math.pi/4)
        self.assertFalse(inside_polygon(footprint@m[:2,:2].T+m[:2,3],r['landing_polygon'],margin))

    def test_entry_match_is_invariant_to_distant_tracking_origin(self):
        from stair_supervisor.ros_lidar import fit_entry_template
        from stair_supervisor.lidar_tracking import TrackingSettings
        points=np.random.default_rng(123).uniform(-1.,1.,(400,3))
        ref=dict(yaw_candidates=[0.],min_fitness=.9,max_rmse_m=.02,score_gap=.05,
                 validated_anchor_error_m=.01,sha256='synthetic')
        for origin in ((0.,0.,0.),(12.,-9.,2.)):
            measured=sample(x=origin[0],y=origin[1],z=origin[2])
            anchor=fit_entry_template(points+origin,measured,route(),points,ref,np.eye(4),TrackingSettings().registration)
            np.testing.assert_allclose(np.array(anchor.local_from_profile).reshape(4,4)[:3,3],origin,atol=1e-5)

if __name__=='__main__':unittest.main()
