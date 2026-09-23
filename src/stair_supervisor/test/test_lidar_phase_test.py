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
            def prepare(self,*args,**kwargs): pass
            def reset_command(self,now): pass
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
