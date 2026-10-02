"""Regression for entry ACK, connected tests and actual support boundaries.

All command sinks are fakes. Scripted poses prove policy behavior, not hardware.
"""
from dataclasses import replace
import math
import unittest
import numpy as np

from stair_supervisor import stair_feedback as geometry
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.robot_transport import TransportFault
from stair_supervisor.supervisor import StairSupervisor, StairGoal, ResultCode, SupervisorState
from stair_supervisor.configuration import Direction
from test_lidar_control import Worker, route, sample, box
from test_supervisor import FakeClock, FakeTransport, FakeAdmission, ScriptedEvidence, make_configuration


class ConnectedGeometryTest(unittest.TestCase):
    def policy(self, r=None):
        worker = Worker()
        control = StairFeedback(worker, np.eye(4), [r or route()])
        control.capture_entry('test_up', np.eye(4), .01, 'synthetic measured start', 100.)
        control.arm(replace(make_configuration().profiles[0], timeout_sec=300.), 100.)
        return worker, control

    def test_adjacent_regions_do_not_create_an_internal_margin_wall(self):
        footprint = np.array(box(-.2,-.2,.2,.2))
        self.assertTrue(geometry.inside_support_union(footprint, [box(-1,-1,0,1),box(0,-1,1,1)], .12))

    def test_gap_inside_body_is_rejected_even_when_every_corner_is_supported(self):
        regions = [box(-1,-1,0,2), box(-1,-1,2,0), box(1,-1,2,2)]
        body = np.array(box(-.5,-.5,1.5,1.5))
        self.assertTrue(all(any(geometry.inside_polygon(np.array([p]), r) for r in regions) for p in body))
        self.assertFalse(geometry.inside_support_union(body, regions))

    def test_physical_gap_and_outer_margin_remain_blocked(self):
        body = np.array(box(-.2,-.2,.2,.2))
        self.assertFalse(geometry.inside_support_union(body,[box(-1,-1,-.01,1),box(.01,-1,1,1)]))
        self.assertFalse(geometry.inside_support_union(body,[box(-1,-1,0,1),box(0,-1,.25,1)],.10))

    def test_body_straddling_flight_and_landing_is_supported(self):
        r=route();r['flight_1_polygon']=box(-1,-1,1.8,1);r['landing_polygon']=box(1.8,-1,3,2)
        worker,c=self.policy(r);c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        worker.value=sample(2,100.2,x=1.78,z=.9)
        result=c.evaluate(c.phase,100.21)
        self.assertFalse(result.faulted,result.detail)
        self.assertFalse(result.complete)

    def test_complete_phase_does_not_validate_an_unsent_old_forward_command(self):
        r=route();r['flight_1_polygon']=box(-1,-1,2.13,1);r['landing_polygon']=box(1.5,-1,2.13,2)
        worker,c=self.policy(r);c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        worker.value=sample(2,100.2,x=2,z=1)
        previous=c.command()
        result=c.evaluate(c.phase,100.21)
        self.assertTrue(result.complete,result.detail);self.assertFalse(result.faulted)
        self.assertEqual(c.command(),previous)  # no fictitious sent-command history
        required=c.evaluate(c.phase,100.21,command_required=True)
        self.assertTrue(required.faulted)  # actually sending forward still cannot pass

    def test_completion_never_bypasses_current_body_support(self):
        worker,c=self.policy();c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        worker.value=sample(2,100.2,x=4,z=1)
        self.assertTrue(c.evaluate(c.phase,100.21).faulted)

    def test_neutral_ack_keeps_actual_command_history_zero(self):
        worker,c=self.policy();c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        for seq in range(2,8):
            stamp=100.+seq*.1;worker.value=sample(seq,stamp)
            self.assertFalse(c.evaluate(c.phase,stamp+.01,command_required=True,neutral_only=True).faulted)
            self.assertEqual(c.command(),(0.,0.))
        c.reset_command(101.);worker.value=sample(10,101.02)
        c.evaluate(c.phase,101.025)
        self.assertLessEqual(abs(c.command()[0]),c.route['limits']['max_accel']*.025+1e-9)

    def test_explicit_uncommissioned_tests_do_not_enable_normal_missions(self):
        r=route();r['commissioned']=False
        w=Worker();c=StairFeedback(w,np.eye(4),[r]);p=make_configuration().profiles[0]
        c.capture_entry('test_up',np.eye(4),.01,'synthetic',100.)
        with self.assertRaises(ValueError):c.prepare(p,100.)
        c.prepare(p,100.,phase_test=True,start_phase=Phase.FORWARD_SEGMENT_1)
        with self.assertRaises(ValueError):c.prepare(p,100.,phase_test=True,start_phase=Phase.FORWARD_SEGMENT_2)

    def test_uncommissioned_default_budget_is_first_flight_four_seconds(self):
        r=route();r['commissioned']=False
        c=StairFeedback(Worker(),np.eye(4),[r]);p=replace(make_configuration().profiles[0],timeout_sec=300.)
        c.test_plan(p,Phase.FORWARD_SEGMENT_1,4.,False)
        for phase,duration,prefix in [(Phase.FORWARD_SEGMENT_1,4.01,False),(Phase.TURN_TO_NEXT_FLIGHT,1.,False),(Phase.FORWARD_SEGMENT_1,4.,True)]:
            with self.assertRaises(ValueError):c.test_plan(p,phase,duration,prefix)

    def test_explicit_budgets_are_finite_named_and_bounded_by_profile(self):
        for bad in [{'unknown':2.},{'ALIGN':True},{'ALIGN':float('inf')},{'ALIGN':0.},{}]:
            r=route();r['phase_test_limits']=bad
            with self.assertRaises(ValueError):StairFeedback(Worker(),np.eye(4),[r])
        r=route();r['phase_test_limits']={p.value:50. for p in Phase}
        c=StairFeedback(Worker(),np.eye(4),[r]);p=replace(make_configuration().profiles[0],timeout_sec=120.)
        c.test_plan(p,Phase.TURN_TO_NEXT_FLIGHT,40.,False)
        with self.assertRaises(ValueError):c.test_plan(p,Phase.EXIT_CONFIRM,121.,True)

    def test_velocity_feedback_reduces_heading_and_lateral_error_in_unicycle_model(self):
        r=route();r['flight_1']=[[0,0,0],[20,0,1]];r['flight_1_polygon']=box(-1,-1,21,1)
        worker,c=self.policy(r);c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        x,y,yaw=.1,.15,.1;dt=.1
        for i in range(250):
            stamp=100.+(i+1)*dt;worker.value=sample(i+2,stamp,x=x,y=y,z=x/20,yaw=yaw)
            result=c.evaluate(c.phase,stamp+.01)
            self.assertFalse(result.faulted,result.detail)
            v,w=c.command();x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt;yaw+=w*dt
        self.assertLess(abs(y),.05);self.assertLess(abs(yaw),.05)


class ConnectedExecutionTest(unittest.TestCase):
    def system(self, *, complete=True, ack_sec=.0, stall=None):
        r=route();r['commissioned']=False;r['phase_test_limits']={p.value:1. for p in Phase}
        clock=FakeClock();transport=FakeTransport();admission=FakeAdmission()
        class Control(ScriptedEvidence):
            routes={'test_up':r}
            route=r
            test_plan=StairFeedback.test_plan
            def prepare(self,*args,**options):return r,None
            def begin_phase(self,phase,now):
                self.phase=phase;self.phases.append(phase);super().begin_phase(phase,now)
            def evaluate(self,phase,now,**options):
                result=super().evaluate(phase,now,**options)
                return replace(result,complete=False) if phase is stall else result
            def command(self):return (.1,.2)
            def reset_command(self,now):self.resets.append(now)
            def loss_response(self,t):t.update_twist(0.,0.);t.send_current()
        c=Control(observations_before_true=1 if complete else 100000);c.phases=[];c.resets=[]
        def request(enabled,progress):
            transport.events.append(('mode',enabled,clock.now))
            for _ in range(max(1,round(ack_sec/.025))):
                progress();clock.sleep(.025)
            transport.events.append(('ack',enabled,clock.now))
        transport.request_stair_mode_with_feedback=request
        transport.update_twist=lambda v,w:transport.events.append(('update',v,w,clock.now))
        config=make_configuration(timeout_sec=120.)
        s=StairSupervisor(config,transport,c,clock,lambda report:None,admission,.25,lidar_control=c);s.start()
        return s,c,transport,clock,admission

    def run_test(self,s,phase=Phase.FORWARD_SEGMENT_1,duration=.1,prefix=False,cancel=lambda:False):
        return s.traverse(StairGoal('test_up',Direction.UP,''),cancel,phase_test=dict(
            route_id='test_up',phase=phase.value,max_duration_sec=duration,operator_confirmed=True,from_entry=prefix))

    def test_all_entry_ack_commands_are_zero_and_do_not_consume_trial_time(self):
        s,c,t,clock,a=self.system(complete=False,ack_sec=.5)
        result=self.run_test(s)
        ack=next(e[2] for e in t.events if e[0]=='ack')
        self.assertTrue(result.cancelled)
        self.assertTrue(all(e[1:3]==(0.,0.) for e in t.events if e[0]=='update' and e[3]<ack))
        moving=[e[3] for e in t.events if e[0]=='update' and e[1]!=0.]
        self.assertTrue(moving);self.assertGreaterEqual(min(moving),ack)
        self.assertGreaterEqual(clock.now-ack,.1);self.assertLess(clock.now-ack,.126)
        self.assertAlmostEqual(c.resets[-1],ack)

    def test_connected_test_runs_ordered_prefix_and_never_returns_arrival_success(self):
        s,c,t,clock,a=self.system()
        result=self.run_test(s,Phase.EXIT_CONFIRM,1.,True)
        self.assertEqual(c.phases[1:],[p for p in Phase if p not in (Phase.ROOFTOP_TURN, Phase.FORWARD_SEGMENT_3)])
        self.assertEqual(c.test_status['state'],'TARGET_REACHED')
        self.assertNotEqual(result.code,ResultCode.OK);self.assertTrue(result.cancelled)
        self.assertEqual(s.state,SupervisorState.STAIR);self.assertEqual(a.calls,[])
        self.assertFalse(any(e[0]=='mode' and not e[1] for e in t.events))

    def test_phase_budget_ends_a_stalled_phase_before_total_budget(self):
        s,c,t,clock,a=self.system(stall=Phase.TURN_TO_NEXT_FLIGHT)
        c.routes['test_up']['phase_test_limits'][Phase.TURN_TO_NEXT_FLIGHT.value]=.075
        result=self.run_test(s,Phase.EXIT_CONFIRM,3.,True)
        self.assertIn('time complete at TURN_TO_NEXT_FLIGHT',result.reason)
        self.assertNotIn(Phase.FORWARD_SEGMENT_2,c.phases)

    def test_missing_prefix_budget_rejects_before_any_mode_request(self):
        s,c,t,clock,a=self.system();del c.routes['test_up']['phase_test_limits']['LANDING']
        result=self.run_test(s,Phase.TURN_TO_NEXT_FLIGHT,1.,True)
        self.assertEqual(result.code,ResultCode.ENTRY_REJECTED)
        self.assertFalse(any(e[0]=='mode' for e in t.events))

    def test_cancel_during_ack_retains_stair_and_never_sends_motion(self):
        s,c,t,clock,a=self.system(ack_sec=.5)
        result=self.run_test(s,cancel=lambda:clock.now>=100.1)
        self.assertTrue(result.cancelled)
        self.assertTrue(all(e[1:3]==(0.,0.) for e in t.events if e[0]=='update'))
        self.assertEqual(s.state,SupervisorState.STAIR)

    def test_mid_landing_handoff_keeps_measured_correction(self):
        s,c,t,clock,a=self.system(ack_sec=.1)
        self.run_test(s,Phase.TURN_TO_NEXT_FLIGHT)
        ack=next(e[2] for e in t.events if e[0]=='ack')
        self.assertTrue(any(e[0]=='update' and e[1]!=0. and e[3]<ack for e in t.events))

    def test_transport_failure_during_loss_response_is_a_result_not_an_exception(self):
        s,c,t,clock,a=self.system()
        def fail(_):raise TransportFault('lost during neutral response')
        c.loss_response=fail
        result=self.run_test(s)
        self.assertEqual(result.code,ResultCode.COMMUNICATION_LOST);self.assertEqual(s.state,SupervisorState.FAULT)

    def test_failed_rearm_restores_retained_ownership(self):
        s,c,t,clock,a=self.system();s._state=SupervisorState.STAIR;s._retained_loss=True
        def fail(*args,**kwargs):raise ValueError('anchor reset between prepare and arm')
        c.arm=fail
        result=self.run_test(s)
        self.assertEqual(result.code,ResultCode.ENTRY_REJECTED)
        self.assertEqual(s.state,SupervisorState.STAIR);self.assertTrue(s._retained_loss)

    def test_target_reached_does_not_allow_old_nav_to_resume(self):
        s,c,t,clock,a=self.system();s.accept_navigation(.3,0.)
        self.run_test(s,Phase.LANDING,1.,True)
        s.accept_navigation(.3,0.);s.stream_navigation()
        self.assertEqual([e[1:3] for e in t.events if e[0]=='update'][-1],(0.,0.))

    def test_cancel_from_feedback_cannot_be_followed_by_nonzero_send(self):
        s,c,t,clock,a=self.system(complete=False)
        cancelled=[False]
        def feedback(report):
            if c.test_status['state']=='RUNNING':cancelled[0]=True
        s._feedback=feedback
        result=self.run_test(s,cancel=lambda:cancelled[0])
        self.assertTrue(result.cancelled)
        self.assertTrue(all(e[1:3]==(0.,0.) for e in t.events if e[0]=='update'))

    def test_slow_evaluation_cannot_emit_a_command_after_trial_deadline(self):
        s,c,t,clock,a=self.system(complete=False)
        evaluate=c.evaluate
        def slow(phase,now,**options):
            result=evaluate(phase,now,**options)
            if c.test_status['state']=='RUNNING':clock.sleep(.2)
            return result
        c.evaluate=slow
        result=self.run_test(s,duration=.1)
        self.assertIn('time complete',result.reason)
        self.assertTrue(all(e[1:3]==(0.,0.) for e in t.events if e[0]=='update'))

    def test_mode_ack_failure_never_emits_entry_motion(self):
        s,c,t,clock,a=self.system()
        def fail(enabled,progress):progress();raise TransportFault('mode ACK failed')
        t.request_stair_mode_with_feedback=fail
        result=self.run_test(s)
        self.assertEqual(result.code,ResultCode.COMMUNICATION_LOST)
        self.assertTrue(all(e[1:3]==(0.,0.) for e in t.events if e[0]=='update'))

    def test_unexpected_feedback_failure_sends_zero_and_retains_ownership(self):
        s,c,t,clock,a=self.system(complete=False)
        def feedback(report):
            if c.test_status['state']=='RUNNING':raise RuntimeError('diagnostic callback failed')
        s._feedback=feedback
        result=self.run_test(s)
        self.assertEqual(result.code,ResultCode.STAIR_FAILED)
        self.assertIn('control exception RuntimeError',result.reason)
        self.assertEqual(s.state,SupervisorState.STAIR);self.assertTrue(s._retained_loss)
        self.assertEqual([e[1:3] for e in t.events if e[0]=='update'][-1],(0.,0.))

    def test_arm_rejection_keeps_existing_arrival_hold(self):
        s,c,t,clock,a=self.system();s._arrival_hold=True
        def fail(*args,**kwargs):raise ValueError('anchor expired')
        c.arm=fail
        result=self.run_test(s)
        self.assertEqual(result.code,ResultCode.ENTRY_REJECTED)
        self.assertEqual(s.state,SupervisorState.NAV);self.assertTrue(s._arrival_hold)


if __name__=='__main__':unittest.main()
