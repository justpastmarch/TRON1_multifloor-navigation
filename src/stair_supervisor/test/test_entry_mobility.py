import sys,unittest,math
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import numpy as np,yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_lidar_control import Worker,sample,route
from test_supervisor import FakeClock,FakeTransport,FakeAdmission,make_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.supervisor import StairSupervisor,SupervisorState,StairGoal,ResultCode
class EntryMobility(unittest.TestCase):
 def policy(self):
  w=Worker();rr=route();rr['entry_polygon']=[[-1.5,-.615],[.12,-.615],[.12,.615],[-1.5,.615]];rr['footprint']=[[-.225,-.225],[.225,-.225],[.225,.225],[-.225,.225]];rr['flight_1'][0]=[-.45,0,0];rr['limits'].update(anchor_uncertainty_m=.1,margin_m=.02)
  c=StairFeedback(w,np.eye(4),[rr]);c.capture_entry('test_up',np.eye(4),.1,'operator',100.);p=replace(make_configuration().profiles[0],timeout_sec=60.);c.arm(p,100.);c.begin_phase(Phase.VERIFY_ENTRY,100.);return w,c,p
 def test_failed_bag_pose_now_supported(self):
  w,c,p=self.policy();w.value=sample(2,100.2,x=-.73750655,y=-.0814218,z=.00624348,yaw=.14308476)
  report=c.evaluate(Phase.VERIFY_ENTRY,100.21);self.assertFalse(report.faulted)
  w.value=replace(w.value,sequence=3,stamp=100.4,measured_at=100.4);self.assertTrue(c.can_return_from_entry(100.41))
 def test_wall_side_first_step_stale_reset_never_auto_walk(self):
  for vals in [dict(x=-1.3),dict(x=-.7,y=.45),dict(x=-.2),dict(x=-.7,z=.17)]:
   w,c,p=self.policy();w.value=sample(2,100.2,**vals);self.assertFalse(c.can_return_from_entry(100.21),vals)
  w,c,p=self.policy();w.value=sample(2,100.2,x=-.7);self.assertFalse(c.can_return_from_entry(101.));w.epoch=1;self.assertFalse(c.can_return_from_entry(100.21))
 def test_started_flight_never_auto_walk(self):
  w,c,p=self.policy();w.value=sample(2,100.2,x=-.7);c.begin_phase(Phase.FORWARD_SEGMENT_1,100.);self.assertFalse(c.can_return_from_entry(100.21))
 def test_outside_entry_rejected_before_stair_mode(self):
  w,c,p=self.policy();c.capture_entry('test_up',np.eye(4),.1,'operator',100.);w.value=sample(2,100.,x=-1.4)
  clock=FakeClock();clock.now=100.;t=FakeTransport();s=StairSupervisor(make_configuration(),t,c,clock,lambda r:None,FakeAdmission(),.25,lidar_control=c);s.start()
  result=s.traverse(StairGoal('test_up',p.direction,'test'),lambda:False)
  self.assertEqual(result.code,ResultCode.ENTRY_REJECTED);self.assertEqual(s.state,SupervisorState.NAV);self.assertFalse(any(e[0] in ('mode','stair') for e in t.events))
 def test_recovery_clears_old_nav_and_requires_ack(self):
  w,c,p=self.policy();w.value=sample(2,99.8,x=-.7);c._observe(w.value);w.value=sample(3,100.,x=-.7);clock=FakeClock();clock.now=100.;t=FakeTransport();t.request_stair_mode_with_feedback=lambda enabled,progress:(progress(),t.events.append(('mode',enabled)))
  s=StairSupervisor(make_configuration(),t,c,clock,lambda r:None,FakeAdmission(),.25,lidar_control=c);s.start();s._state=SupervisorState.STAIR;s.accept_manual_input(False);s._latest_nav=(1,1,100,0)
  self.assertTrue(s._try_entry_nav_recovery());self.assertEqual(s.state,SupervisorState.NAV);self.assertIsNone(s._latest_nav);self.assertIn(('mode',False),t.events)



class ContactEntryTest(unittest.TestCase):
 def policy(self):
  from stair_supervisor.configuration import load_lidar_configuration,load_stair_configuration
  root=Path(__file__).resolve().parents[1]/'config'
  rr=load_lidar_configuration(root/'stair_lidar_3f_4f_test.yaml','control',False).document['routes'][1]
  p=next(p for p in load_stair_configuration(root).profiles if p.id==rr['id'])
  w=Worker();c=StairFeedback(w,np.eye(4),[rr]);c.capture_entry(rr['id'],np.eye(4),.1,'recorded body frame',100.)
  return w,c,p
 def test_recorded_contact_flows_verify_align_ascent_without_retreat(self):
  w,c,p=self.policy();w.value=sample(2,100.1,x=-.21510761,y=-.08744557,z=.00520457,yaw=math.radians(-8.75))
  c.prepare(p,100.11,start_phase=Phase.VERIFY_ENTRY);c.arm(p,100.11,start_phase=Phase.VERIFY_ENTRY);c.begin_phase(Phase.VERIFY_ENTRY,100.11)
  for i in range(1,9):
   w.value=sample(i+2,100.1+i*.1,x=-.21510761,y=-.08744557,z=.00520457,yaw=math.radians(-8.75))
   report=c.evaluate(c.phase,w.value.stamp+.01)
   self.assertFalse(report.faulted,report.detail);self.assertEqual(c.command()[0],0.)
  self.assertTrue(report.complete)
  c.begin_phase(Phase.ALIGN,101.)
  w.value=replace(w.value,sequence=20,stamp=101.1,measured_at=101.1)
  report=c.evaluate(c.phase,101.11);self.assertTrue(report.complete, c.debug);self.assertEqual(c.command()[0],0.)
  c.begin_phase(Phase.FORWARD_SEGMENT_1,101.2)
  w.value=replace(w.value,sequence=21,stamp=101.3,measured_at=101.3)
  report=c.evaluate(c.phase,101.31);self.assertFalse(report.faulted,c.debug);self.assertGreater(c.command()[0],0.)
 def test_contact_is_not_permission_for_side_rear_height_or_midflight_start(self):
  for args in [dict(x=-.2,y=.5),dict(x=-.2,y=-.5),dict(x=-1.4),dict(x=.8),dict(x=-.2,z=.3)]:
   w,c,p=self.policy();w.value=sample(2,100.1,**args)
   with self.subTest(args=args),self.assertRaises(ValueError):c.prepare(p,100.11,start_phase=Phase.VERIFY_ENTRY)
 def test_fresh_contact_needed_and_wrong_heading_does_not_start_ascent(self):
  w,c,p=self.policy();w.value=sample(2,100.1,x=-.2,yaw=math.pi/2)
  c.arm(p,100.11,start_phase=Phase.VERIFY_ENTRY);c._entry_verified=True;c.begin_phase(Phase.ALIGN,100.11)
  w.value=sample(3,100.2,x=-.2,yaw=math.pi/2)
  report=c.evaluate(c.phase,100.21);self.assertFalse(report.complete);self.assertEqual(c.command()[0],0.)
  with self.assertRaises(ValueError):c.prepare(p,102.,start_phase=Phase.VERIFY_ENTRY)
  w.epoch=1
  with self.assertRaises(ValueError):c.prepare(p,100.21,start_phase=Phase.VERIFY_ENTRY)

 def test_standalone_align_uses_same_corridor_without_nominal_point_return(self):
  w,c,p=self.policy();w.value=sample(2,100.1,x=-.2)
  c.arm(p,100.11,start_phase=Phase.ALIGN);c.begin_phase(Phase.ALIGN,100.11)
  for i in range(1,10):
   w.value=sample(i+2,100.1+i*.1,x=-.2)
   report=c.evaluate(c.phase,w.value.stamp+.01)
   self.assertFalse(report.faulted);self.assertEqual(c.command()[0],0.)
  self.assertTrue(report.complete)

if __name__=='__main__':unittest.main()
