import unittest
from unittest.mock import Mock
from test_lidar_phase_test import PhaseTest
from test_lidar_control import Worker,route,sample
from test_supervisor import make_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.supervisor import StairGoal,ResultCode
from stair_supervisor.configuration import Direction
import numpy as np

class OperatorPhaseTest(unittest.TestCase):
 def test_live_transition_keeps_same_traversal_and_remaining_phases(self):
  s,c,t,clock,a=PhaseTest().setup_system(complete=False)
  c._threshold=1;c.operator_phase=Mock();submitted=[]
  def applying(*args):
   status=s.operator_status()
   with self.assertRaises(ValueError):s.request_operator_phase(dict(run_id=status['run_id'],revision=status['revision'],phase='EXIT_CONFIRM',confirmed=True,request_id='B'))
  c.operator_phase.side_effect=applying
  # Keep arrival handoff out of this test; assert it is still reached once.
  s._finish=Mock(return_value='finished')
  def observe(phase):
   status=s.operator_status()
   if phase is Phase.FORWARD_SEGMENT_1 and status.get('active') and not submitted:
    request=dict(run_id=status['run_id'],revision=status['revision'],phase='TURN_TO_NEXT_FLIGHT',confirmed=True,request_id='A')
    submitted.append(s.request_operator_phase(request))
    with self.assertRaises(ValueError):s.request_operator_phase(request)
  c.on_observe=observe
  result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False)
  self.assertEqual(result,'finished');s._finish.assert_called_once()
  c.operator_phase.assert_called_once();self.assertEqual(c.operator_phase.call_args.args[0],Phase.TURN_TO_NEXT_FLIGHT)
  self.assertIn(Phase.FORWARD_SEGMENT_2,c.seen);self.assertIn(Phase.EXIT_CONFIRM,c.seen)
  self.assertNotIn(Phase.LANDING,c.seen);self.assertFalse(s.operator_status()['active'])
  self.assertEqual(len(a.calls),1)
  self.assertEqual(s.operator_status()['result']['request_id'],'A')
  self.assertEqual(s.operator_status()['result']['state'],'APPLIED')
  with self.assertRaises(ValueError):s.request_operator_phase(request if False else dict(submitted[0]))
 def test_continue_request_requires_waiting_and_same_current_phase(self):
  s,_,_,_,_=PhaseTest().setup_system()
  s._state=__import__('stair_supervisor.supervisor',fromlist=['SupervisorState']).SupervisorState.STAIR
  s._operator_run=dict(active=True,waiting=False,run_id='run',revision=4,phase='TURN_TO_NEXT_FLIGHT',phases=['TURN_TO_NEXT_FLIGHT'])
  request=dict(kind='continue',run_id='run',revision=4,phase='TURN_TO_NEXT_FLIGHT',confirmed=True,request_id='continue-1')
  with self.assertRaisesRegex(ValueError,'중단 대기'):
   s.request_operator_phase(request)
  s._operator_run['waiting']=True
  wrong=dict(request,phase='LANDING')
  with self.assertRaisesRegex(ValueError,'현재 경로'):
   s.request_operator_phase(wrong)
  accepted=s.request_operator_phase(request)
  self.assertTrue(accepted['accepted']);self.assertEqual(s.operator_status()['pending'],True)
 def test_old_run_revision_and_unknown_phase_do_not_queue(self):
  s,c,t,clock,a=PhaseTest().setup_system()
  s._state=__import__('stair_supervisor.supervisor',fromlist=['SupervisorState']).SupervisorState.STAIR
  s._operator_run=dict(active=True,run_id='new',revision=2,phases=['LANDING'])
  for request in [dict(run_id='old',revision=2,phase='LANDING',confirmed=True),dict(run_id='new',revision=1,phase='LANDING',confirmed=True),dict(run_id='new',revision=2,phase='EXIT_CONFIRM',confirmed=True),dict(run_id='new',revision=2,phase='LANDING',confirmed=False)]:
   with self.assertRaises(ValueError):s.request_operator_phase(request)
   self.assertIsNone(s._operator_request)
 def test_operator_completion_is_separate_from_sensor_evidence(self):
  w=Worker();c=StairFeedback(w,np.eye(4),[route()]);c.capture_entry('test_up',np.eye(4),.01,'fixture',100.);c.arm(make_configuration().profiles[0],100.)
  old_anchor=c.anchor;w.value=sample(2,100.1,x=2,y=0,z=1)
  c.operator_phase(Phase.TURN_TO_NEXT_FLIGHT,100.15)
  self.assertIs(c.anchor,old_anchor);self.assertEqual(c._flight_seen,[False,False]);self.assertEqual(c._operator_completed_flights,{0})
  self.assertFalse(c._all_flights_completed());self.assertEqual(c.command(),(0.,0.))
  c.operator_phase(Phase.EXIT_CONFIRM,100.15)
  self.assertTrue(c._all_flights_completed());self.assertEqual(c._flight_seen,[False,False])
  # Declaring earlier flights completed cannot declare roof arrival at landing height.
  w.value=sample(3,100.2,x=2,y=0,z=1)
  self.assertFalse(c.evaluate(Phase.EXIT_CONFIRM,100.21).complete)
 def test_stale_or_reset_tracking_does_not_change_phase(self):
  for reset in (False,True):
   w=Worker();c=StairFeedback(w,np.eye(4),[route()]);c.capture_entry('test_up',np.eye(4),.01,'fixture',100.);c.arm(make_configuration().profiles[0],100.)
   before=c.phase
   if reset:w.epoch=1
   with self.assertRaises(ValueError):c.operator_phase(Phase.TURN_TO_NEXT_FLIGHT,101.)
   self.assertEqual(c.phase,before)

class RecoveryWaitTest(unittest.TestCase):
 def setup_wait(self):
  from dataclasses import replace
  s,c,t,clock,a=PhaseTest().setup_system(complete=False)
  s._operator_recovery_enabled=True;c._threshold=1;c.operator_phase=Mock();s._finish=Mock(return_value='finished')
  evaluate=c.evaluate;faults=[]
  def report(phase,now,**kw):
   result=evaluate(phase,now,**kw)
   if phase is Phase.FORWARD_SEGMENT_1 and not faults:
    faults.append(phase);return replace(result,faulted=True,complete=False,detail='synthetic lost correction')
   return result
  c.evaluate=report
  return s,c,t,clock,a
 def test_fault_keeps_mission_until_operator_resumes_same_flight(self):
  s,c,t,clock,a=self.setup_wait();sleep=clock.sleep;requests=[]
  def wait(seconds):
   status=s.operator_status()
   if status.get('waiting') and not requests:
    self.assertTrue(s._retained_loss);self.assertEqual(status['stop_reason'],'synthetic lost correction')
    requests.append(s.request_operator_phase(dict(run_id=status['run_id'],revision=status['revision'],phase=status['phase'],confirmed=True,request_id='same-flight')))
   sleep(seconds)
  clock.sleep=wait
  result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False)
  self.assertEqual(result,'finished');self.assertEqual(len(a.calls),1);self.assertEqual(len(requests),1)
  self.assertFalse(s._retained_loss);self.assertEqual(s.operator_status()['result']['state'],'APPLIED')
  self.assertEqual(c.operator_phase.call_args.args[0],Phase.FORWARD_SEGMENT_1)
  self.assertIn(Phase.TURN_TO_NEXT_FLIGHT,c.seen);self.assertIn(Phase.EXIT_CONFIRM,c.seen)
 def test_same_phase_continue_uses_live_state_without_restarting_phase(self):
  s,c,t,clock,a=self.setup_wait();sleep=clock.sleep;requests=[]
  c.continue_phase=Mock()
  c.operator_phase=Mock()
  def wait(seconds):
   status=s.operator_status()
   if status.get('waiting') and not requests:
    requests.append(s.request_operator_phase(dict(kind='continue',run_id=status['run_id'],revision=status['revision'],
      phase=status['phase'],confirmed=True,request_id='continue-current')))
   sleep(seconds)
  clock.sleep=wait
  result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False)
  self.assertEqual(result,'finished');self.assertEqual(len(requests),1)
  c.continue_phase.assert_called_once()
  c.operator_phase.assert_not_called()
  self.assertEqual(c.seen.count(Phase.FORWARD_SEGMENT_1),1)
  self.assertFalse(s.operator_status()['waiting'])
  self.assertEqual(s.operator_status()['result']['kind'],'continue')
 def test_manual_landing_completion_can_skip_failed_flight(self):
  s,c,t,clock,a=self.setup_wait();sleep=clock.sleep;requests=[]
  def wait(seconds):
   status=s.operator_status()
   if status.get('waiting') and not requests:
    requests.append(s.request_operator_phase(dict(run_id=status['run_id'],revision=status['revision'],phase='TURN_TO_NEXT_FLIGHT',confirmed=True,request_id='manual-landing')))
   sleep(seconds)
  clock.sleep=wait
  self.assertEqual(s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False),'finished')
  self.assertEqual(c.operator_phase.call_args.args[0],Phase.TURN_TO_NEXT_FLIGHT)
  self.assertNotIn(Phase.LANDING,c.seen);self.assertIn(Phase.FORWARD_SEGMENT_2,c.seen)
 def test_explicit_stop_ends_wait_and_never_auto_resumes(self):
  s,c,t,clock,a=self.setup_wait()
  result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:s.operator_status().get('waiting',False))
  self.assertTrue(result.cancelled);self.assertFalse(s.operator_status()['active'])
  c.operator_phase.assert_not_called();s._finish.assert_not_called()

class TerminalResumeTest(unittest.TestCase):
 def test_terminal_stop_resumes_same_flight_without_new_anchor_or_admission(self):
  s,c,t,clock,a=PhaseTest().setup_system(complete=True)
  s._operator_recovery_enabled=True
  c.continue_phase=Mock()
  original_arm=c.arm;c.arm=Mock(wraps=original_arm)
  s._finish=Mock(return_value='finished')
  stopped=[]
  def cancel():
   phase=s.operator_status().get('phase')
   if phase=='FORWARD_SEGMENT_1' and not stopped:
    stopped.append(True);return True
   return False
  result=s.traverse(StairGoal('test_up',Direction.UP,''),cancel)
  self.assertTrue(result.cancelled)
  checkpoint=s.operator_status()
  self.assertTrue(checkpoint['resume_available'])
  self.assertEqual(checkpoint['route_id'],'test_up')
  self.assertEqual(checkpoint['phase'],'FORWARD_SEGMENT_1')
  seen=c.seen.count(Phase.FORWARD_SEGMENT_1)
  resume=dict(run_id=checkpoint['run_id'],revision=checkpoint['revision'],phase=checkpoint['phase'])
  self.assertEqual(s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,resume=resume),'finished')
  self.assertEqual(c.seen.count(Phase.FORWARD_SEGMENT_1),seen)
  self.assertEqual(c.arm.call_count,1)
  self.assertEqual(len(a.calls),1)
  c.continue_phase.assert_called_once()
  self.assertIn(Phase.FORWARD_SEGMENT_2,c.seen)
 def test_mode_handoff_interruption_still_retains_route(self):
  s,c,t,clock,a=PhaseTest().setup_system(complete=False)
  result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:True)
  self.assertTrue(result.cancelled)
  status=s.operator_status()
  self.assertTrue(status['resume_available'])
  self.assertEqual((status['route_id'],status['phase']),('test_up','VERIFY_ENTRY'))

 def test_separate_phase_test_invalidates_terminal_checkpoint(self):
  s,c,t,clock,a=PhaseTest().setup_system(complete=False)
  from stair_supervisor.supervisor import SupervisorState
  s._state=SupervisorState.STAIR;s._retained_loss=True
  s._operator_run=dict(active=False,run_id='old',route_id='test_up',revision=3,
                       phase='TURN_TO_NEXT_FLIGHT',phases=['TURN_TO_NEXT_FLIGHT'])
  self.assertTrue(s.operator_status()['resume_available'])
  s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,
             phase_test=dict(route_id='test_up',phase='TURN_TO_NEXT_FLIGHT',max_duration_sec=.1,operator_confirmed=True))
  self.assertFalse(s.operator_status()['resume_available'])

 def test_ros_action_resume_skips_entry_registration(self):
  from types import SimpleNamespace
  from stair_supervisor.msg import StairTraversalGoal
  from stair_supervisor.ros_node import RosStairSupervisorNode
  from stair_supervisor.supervisor import SupervisorState,TraversalResult
  node=RosStairSupervisorNode.__new__(RosStairSupervisorNode)
  node._lidar=SimpleNamespace(configuration=SimpleNamespace(mode='control'),ensure_entry=Mock())
  node._supervisor=SimpleNamespace(state=SupervisorState.STAIR,ownership_epoch=7,
    traverse=Mock(return_value=TraversalResult(ResultCode.OK,'stair complete')))
  node._action=Mock();node._publish_state=Mock()
  goal=StairTraversalGoal(stair_id='test_up',direction=StairTraversalGoal.UP,
    admission_token='',resume_run_id='retained-run',resume_revision=3,resume_phase='FORWARD_SEGMENT_2')
  node._execute_request(goal,None)
  node._lidar.ensure_entry.assert_not_called()
  self.assertEqual(node._supervisor.traverse.call_args.kwargs['resume'],
                   dict(run_id='retained-run',revision=3,phase='FORWARD_SEGMENT_2'))
  node._action.set_succeeded.assert_called_once()

 def test_stale_or_wrong_route_checkpoint_cannot_send_motion(self):
  s,c,t,clock,a=PhaseTest().setup_system(complete=True)
  s._state=__import__('stair_supervisor.supervisor',fromlist=['SupervisorState']).SupervisorState.STAIR
  s._retained_loss=True
  s._operator_run=dict(active=False,run_id='5f-rf',route_id='test_up',revision=7,
                       phase='FORWARD_SEGMENT_2',phases=['FORWARD_SEGMENT_2','EXIT_CONFIRM'])
  for request in (dict(run_id='old',revision=7,phase='FORWARD_SEGMENT_2'),
                  dict(run_id='5f-rf',revision=6,phase='FORWARD_SEGMENT_2'),
                  dict(run_id='5f-rf',revision=7,phase='LANDING')):
   before=list(t.events)
   result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:False,resume=request)
   self.assertEqual(result.code,ResultCode.ENTRY_REJECTED)
   self.assertEqual(t.events,before)

class RealControllerRecoveryTest(unittest.TestCase):
 def test_real_controller_mode_timeout_and_cancel_keep_zero(self):
  from test_supervisor import FakeClock,FakeTransport,FakeAdmission,ScriptedEvidence
  from stair_supervisor.supervisor import StairSupervisor
  from stair_supervisor.stair_evidence import EvidenceReport
  for cancel_ack in (False,True):
   w=Worker();control=StairFeedback(w,np.eye(4),[route()]);control.capture_entry('test_up',np.eye(4),.01,'fixture',100.)
   clock=FakeClock();transport=FakeTransport();script=ScriptedEvidence(0);profile=make_configuration(timeout_sec=60.);cancelled=[False];faults=[];acks=[];requests=[]
   class Evidence:
    def arm(self,p,t):control.arm(p,t);script.arm(p,t)
    def begin_phase(self,p,t):control.begin_phase(p,t);script.begin_phase(p,t)
    def evaluate(self,p,t):
     if p is Phase.FORWARD_SEGMENT_1 and not faults:
      faults.append(p);return EvidenceReport(p,False,True,'synthetic fault',0,0,0)
     return script.evaluate(p,t)
   def acknowledge(enabled,progress):
    acks.append(enabled)
    if len(acks)==2:
     if cancel_ack:cancelled[0]=True
     else:clock.now+=3.
    progress()
   transport.request_stair_mode_with_feedback=acknowledge
   s=StairSupervisor(profile,transport,Evidence(),clock,lambda r:None,FakeAdmission(),.25,lidar_control=control,operator_recovery_enabled=True)
   s._finish=Mock(return_value='finished');s.start();sleep=clock.sleep
   def wait(seconds):
    status=s.operator_status()
    if clock.now>110:raise AssertionError(repr(status))
    if status.get('waiting') and len(requests)<2 and not cancelled[0]:
     if requests:
      self.assertEqual(status['result']['state'],'REJECTED');self.assertTrue(control._interrupted)
      self.assertTrue(s._retained_loss);self.assertEqual(control.command(),(0.,0.))
     w.value=sample(len(requests)+2,clock.now,x=2,y=0,z=1)
     requests.append(s.request_operator_phase(dict(run_id=status['run_id'],revision=status['revision'],phase='TURN_TO_NEXT_FLIGHT',confirmed=True,request_id=str(len(requests)))))
    sleep(seconds)
   clock.sleep=wait
   result=s.traverse(StairGoal('test_up',Direction.UP,''),lambda:cancelled[0])
   if cancel_ack:
    self.assertTrue(result.cancelled);self.assertTrue(control._interrupted);self.assertEqual(control.command(),(0.,0.));s._finish.assert_not_called()
   else:
    self.assertEqual(result,'finished',str(result));self.assertEqual(len(requests),2);self.assertEqual(s.operator_status()['result']['state'],'APPLIED')
