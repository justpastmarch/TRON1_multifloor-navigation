"""Policy regressions using a recorded pose, not closed-loop success claims."""
import unittest,math
from dataclasses import replace
from unittest.mock import patch
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from test_lidar_control import Worker,sample
from test_supervisor import make_configuration
from stair_supervisor.stair_feedback import StairFeedback,RouteAnchor
from stair_supervisor.stair_evidence import Phase
class ContinuityTest(unittest.TestCase):
 def setUp(self):
  self.pos=np.array([1.4358658714,.23793956937,1.0731168656]);self.yaw=.29559651164
  self.R=Rotation.from_euler('xyz',[-.15532480565,.10764367368,self.yaw]).as_matrix()
  self.fp=np.array([[-.225,-.225,0],[.225,-.225,0],[.225,.225,0],[-.225,.225,0]])
  self.body=(self.fp@self.R.T+self.pos)[:,:2]
  self.poly=[[-1.5,-.615],[4.635,-.615],[4.635,.615],[-1.5,.615]]
 def test_dense_independent_rectangle_check_agrees_for_inward_turn(self):
  for w,expected in [(-.06,True),(.06,False)]:
   self.assertEqual(StairFeedback._swept_support(self.pos[:2],self.yaw,self.body,0,w,.6,[self.poly],.1),expected)
   theta=w*np.linspace(0,.6,10001);rel=self.body-self.pos[:2];y=self.pos[1]+np.sin(theta)[:,None]*rel[:,0]+np.cos(theta)[:,None]*rel[:,1]
   self.assertEqual(bool(np.all(y<=.515)&np.all(y>=-.515)),expected)
 def test_does_not_relax_hard_margin_or_authorize_forward_at_failed_pose(self):
  self.assertFalse(StairFeedback._swept_support(self.pos[:2],self.yaw,self.body,0,-.06,.6,[self.poly],.12))
  self.assertFalse(StairFeedback._swept_support(self.pos[:2],self.yaw,self.body,.55,-.06,.6,[self.poly],.1))
 def test_real_output_slew_accumulates_during_recheck(self):
  from test_lidar_control import route
  r=route();r['limits'].update(flight_recheck_sec=2.,flight_restart_v=.3,flight_recovery_w=.06,max_alpha=.3)
  w=Worker();c=StairFeedback(w,np.eye(4),[r]);c.capture_entry('test_up',np.eye(4),.01,'synthetic',100.);c.arm(replace(make_configuration().profiles[0],timeout_sec=60.),100.);c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
  with patch.object(c,'_flight_command',return_value=None):
   w.value=sample(2,100.2,x=.2,yaw=.2);c.evaluate(c.phase,100.21)
   c.evaluate(c.phase,100.385)  # intervening zero-output tick before the new scan
   w.value=sample(3,100.4,x=.2,yaw=.2);c.evaluate(c.phase,100.41);first=abs(c.command()[1])
   prev=c.command()[1]
   for now in [100.435,100.46,100.485,100.51,100.535,100.56,100.585]:
    report=c.evaluate(c.phase,now);cur=c.command()[1]
    self.assertFalse(report.faulted);self.assertLessEqual(abs(cur-prev),.3*.025+1e-8);self.assertLessEqual(abs(cur),.06+1e-10);self.assertEqual(c.command()[0],0.);prev=cur
   self.assertGreater(abs(c.command()[1]),first)
 def test_preview_steers_against_outward_drift_without_extra_inward_correction(self):
  from test_lidar_control import route
  for first_y,last_y,expected in [(.04,.06,1),(-.04,-.06,-1),(.06,.04,0)]:
   r=route();r['limits']['flight_lateral_preview_sec']=.3
   w=Worker();c=StairFeedback(w,np.eye(4),[r]);c.capture_entry('test_up',np.eye(4),.01,'synthetic',100.);c.arm(replace(make_configuration().profiles[0],timeout_sec=60.),100.);c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
   w.value=sample(2,100.2,x=.2,y=first_y);c.evaluate(c.phase,100.21)
   w.value=sample(3,100.4,x=.22,y=last_y);c.evaluate(c.phase,100.41)
   extra=c.debug['steering_response']['extra_preview_m']
   if expected:
    self.assertGreater(extra*expected,0);self.assertLess(c.command()[1]*expected,0)
   else:self.assertEqual(extra,0.)
if __name__=='__main__':unittest.main()
