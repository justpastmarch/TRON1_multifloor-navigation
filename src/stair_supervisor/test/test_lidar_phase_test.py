"""Single-phase operator tests: fake transport only, no robot connection."""
import unittest
import numpy as np
from test_lidar_control import Worker, route
from test_supervisor import FakeClock, FakeTransport, FakeAdmission, ScriptedEvidence, make_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.supervisor import StairSupervisor, StairGoal, ResultCode, SupervisorState
from stair_supervisor.configuration import Direction

class PhaseTest(unittest.TestCase):
    def setup_system(self, complete=False):
        class Control(ScriptedEvidence):
            routes={'test_up':route()}
            route={'limits':{'handoff_sec':2.}}
            test_plan=StairFeedback.test_plan
            traversal_phases=StairFeedback.traversal_phases
            def prepare(self,*args,**kwargs): pass
            def reset_command(self,now): pass
            def begin_landing_hold(self,now):self.holding=True
            def begin_phase(self,phase,now):
                self.phase=phase
                self.seen.append(phase)
                super().begin_phase(phase,now)
            def command(self):return (.1,.2)
            def loss_response(self,t):
                t.update_twist(0.,0.);t.send_current()
        control=Control(observations_before_true=0 if complete else 10000)
        control.seen=[]
        clock=FakeClock();transport=FakeTransport();admission=FakeAdmission()
        transport.request_stair_mode_with_feedback=lambda enabled,progress:(transport.events.append(('mode',enabled)),progress())
        supervisor=StairSupervisor(make_configuration(),transport,control,clock,lambda r:None,admission,.25,lidar_control=control)
        supervisor.start()
        return supervisor,control,transport,clock,admission
    def options(self):return dict(route_id='test_up',phase='TURN_TO_NEXT_FLIGHT',max_duration_sec=.10,operator_confirmed=True)
    def test_only_selected_phase_timeout_zero_no_arrival_or_walk(self):
        s,c,t,clock,a=self.setup_system()
        result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,phase_test=self.options())
        self.assertTrue(result.cancelled)
        self.assertNotEqual(result.code,ResultCode.OK)
        self.assertEqual(set(c.seen),{Phase.TURN_TO_NEXT_FLIGHT})
        self.assertEqual(s.state,SupervisorState.STAIR)
        self.assertEqual(a.calls,[])
        self.assertNotIn(('mode',False),t.events)
        self.assertEqual([e for e in t.events if e[0]=='update'][-1],('update',0.,0.))
    def test_selected_phase_completion_does_not_advance(self):
        s,c,t,clock,a=self.setup_system(complete=True)
        result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,phase_test=self.options())
        self.assertTrue(result.cancelled)
        self.assertEqual(set(c.seen),{Phase.TURN_TO_NEXT_FLIGHT})

    def test_landing_completion_holds_until_manual_takeover_without_nav(self):
        s,c,t,clock,a=self.setup_system(complete=True)
        s.accept_manual_input(False)
        options=self.options();options['phase']='LANDING'
        result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,phase_test=options)
        self.assertIn('position hold active',result.reason)
        self.assertTrue(s._landing_test_hold)
        self.assertEqual(c.test_status['state'],'LANDING_HOLD')
        self.assertEqual(s.state,SupervisorState.STAIR)
        s.stream_navigation()
        self.assertEqual([e for e in t.events if e[0]=='update'][-1],('update',.1,.2))
        s.accept_manual_input(True)
        self.assertFalse(s._landing_test_hold)
        self.assertTrue(s._retained_loss)
        self.assertEqual([e for e in t.events if e[0]=='update'][-1],('update',0.,0.))
        s.accept_manual_input(False);s.stream_navigation()
        self.assertFalse(s._landing_test_hold)
        self.assertEqual([e for e in t.events if e[0]=='update'][-1],('update',0.,0.))

    def test_landing_hold_needs_live_manual_feedback_and_releases_when_stale(self):
        for feedback in (False,True):
            s,c,t,clock,a=self.setup_system(complete=True)
            if feedback:s.accept_manual_input(False)
            options=self.options();options['phase']='LANDING'
            s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,phase_test=options)
            self.assertEqual(s._landing_test_hold,feedback)
            clock.sleep(.6);s.stream_navigation()
            self.assertFalse(s._landing_test_hold)
            self.assertEqual([e for e in t.events if e[0]=='update'][-1],('update',0.,0.))

    def test_landing_hold_release_service_and_next_test_remove_hold_owner(self):
        for release in (False,True):
            s,c,t,clock,a=self.setup_system(complete=True);s.accept_manual_input(False)
            options=self.options();options['phase']='LANDING'
            s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,phase_test=options)
            self.assertTrue(s._landing_test_hold)
            if release:
                self.assertTrue(s.release_arrival_hold())
                self.assertFalse(s._landing_test_hold)
            result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,phase_test=self.options())
            self.assertNotEqual(result.code,ResultCode.BUSY)
            self.assertFalse(s._landing_test_hold)

    def test_manual_callback_treats_any_input_or_invalid_axes_as_takeover(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from stair_supervisor.ros_node import RosStairSupervisorNode
        node=RosStairSupervisorNode.__new__(RosStairSupervisorNode);node._supervisor=Mock()
        for axes,buttons,active in [([0.,0.],[0],False),([.01,0.],[0],True),
                                    ([0.,0.],[1],True),([float('nan')],[0],True),([],[],True)]:
            node._accept_manual_input(SimpleNamespace(axes=axes,buttons=buttons))
            node._supervisor.accept_manual_input.assert_called_with(active)
    def test_cancel_sends_zero_and_does_not_disable_stair(self):
        s,c,t,clock,a=self.setup_system()
        result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:True,phase_test=self.options())
        self.assertTrue(result.cancelled)
        self.assertEqual([e for e in t.events if e[0]=='update'][-1],('update',0.,0.))
        self.assertNotIn(('mode',False),t.events)
    def test_invalid_or_unconfirmed_test_rejected_before_mode(self):
        for key,value in [('operator_confirmed',False),('max_duration_sec',float('nan')),('phase','EXIT_CONFIRM'),('route_id','other')]:
            s,c,t,clock,a=self.setup_system();options=self.options();options[key]=value
            result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,phase_test=options)
            self.assertEqual(result.code,ResultCode.INVALID_GOAL)
            self.assertFalse(any(e[0]=='mode' for e in t.events))
    def test_real_loss_response_never_calls_emergency_api(self):
        control=StairFeedback(Worker(),np.eye(4),[route()]);t=FakeTransport()
        def forbidden():raise AssertionError('emergency API must never be called')
        t.request_emergency_stop=forbidden
        control.loss_response(t)
        self.assertEqual(t.events,[('update',0.,0.),('send',)])
    def test_rearmed_test_from_retained_stair_does_not_switch_walk(self):
        s,c,t,clock,a=self.setup_system()
        for _ in range(2):
            result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,phase_test=self.options())
            self.assertTrue(result.cancelled)
            self.assertEqual(s.state,SupervisorState.STAIR)
        self.assertEqual(sum(e==('mode',True) for e in t.events),2)
        self.assertNotIn(('mode',False),t.events)

    def test_emergency_route_rejected(self):
        r=route();r['loss_response']='emergency_stop'
        with self.assertRaises(ValueError):StairFeedback(Worker(),np.eye(4),[r])

if __name__=='__main__':unittest.main()
