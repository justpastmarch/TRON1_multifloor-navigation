"""Behavioral tests with synthetic geometry. No physical success claims."""
import copy,math,unittest,time,threading
from dataclasses import replace
from unittest.mock import patch
import numpy as np
from test_lidar_control import Worker,route,sample,box
from test_supervisor import make_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.motion_prediction import MotionState,response_envelope,validate_model

def model(mode='shadow',commissioned=False):
 return dict(mode=mode,commissioned=commissioned,source='SYNTHETIC response fixture',forward_gain=[1.,1.],yaw_gain=[1.,1.],response_sec=.6,position_error_m=.001,velocity_error_mps=.001,yaw_error_rad=.001,yaw_rate_error_radps=.001,max_age_sec=.5)

def policy(changes=None,motion=None):
 r=route();r['limits'].update(changes or {})
 if motion and motion['mode']=='control':r['limits'].update(flight_recheck_sec=1.,flight_restart_v=.3)
 if motion:r['motion_prediction']=motion
 w=Worker();c=StairFeedback(w,np.eye(4),[r]);c.capture_entry('test_up',np.eye(4),.01,'synthetic',100.);c.arm(replace(make_configuration().profiles[0],timeout_sec=60.),100.);c.begin_phase(Phase.FORWARD_SEGMENT_1,100.);return w,c

class ResponseTest(unittest.TestCase):
 def test_uncalibrated_control_rejected(self):
  with self.assertRaisesRegex(ValueError,'shadow'):validate_model(model('control',False))
 def test_malformed_bounds_and_missing_provenance_rejected(self):
  for key,value in [('forward_gain',[1,0]),('yaw_gain',[-1,1]),('max_age_sec',float('nan')),('source',''),('commissioned',1)]:
   m=model();m[key]=value
   with self.subTest(key=key),self.assertRaises(ValueError):validate_model(m)
 def test_equilibrium_response_matches_analytic_motion(self):
  s=MotionState(np.zeros(2),0.,np.array([.2,0]),0.,0.)
  e=response_envelope(s,(.2,0),.6,model())
  self.assertAlmostEqual(e['centers'][0,-1,0],.12)
 def test_zero_request_retains_residual_backslide(self):
  s=MotionState(np.zeros(2),0.,np.array([-.06,0]),0.,0.)
  e=response_envelope(s,(0,0),.6,model())
  self.assertLess(e['centers'][0,-1,0],-.02)
  self.assertFalse(e['physical_hold_verified'])
 def test_causal_age_propagation_does_not_modify_measurement(self):
  p=np.zeros(2);s=MotionState(p,0.,np.array([.2,.1]),0.,.3)
  e=response_envelope(s,(.2,0),.6,model())
  np.testing.assert_allclose(e['propagated_position'],[.06,.03]);np.testing.assert_array_equal(p,[0,0])
  self.assertGreater(e['position_error'][-1],e['position_error'][0])
 def test_missing_or_expired_state_is_not_restamped(self):
  with self.assertRaises(ValueError):response_envelope(MotionState(np.zeros(2),0.,np.zeros(2),0.,.51),(0,0),.6,model())
 def test_control_model_checks_motion_not_command_scale(self):
  w,c=policy(motion=model('control',True))
  for i in range(3):
   w.value=sample(i+2,100.2+i*.2,x=.1+i*.02);c.evaluate(c.phase,w.value.stamp+.01)
  self.assertEqual(c.debug['motion_prediction']['state'],'evaluated')
  self.assertTrue(c.debug['motion_prediction']['affects_commands'])
 def test_unavailable_control_response_does_not_fall_back_to_command_model(self):
  w,c=policy(motion=model('control',True));w.value=sample(2,100.2,x=.1)
  with patch.object(c,'_swept_support',return_value=True):
   c.evaluate(c.phase,100.21)
  self.assertEqual(c.command(),(0.,0.));self.assertTrue(c.diagnostic_snapshot()['prediction_recheck_active'])
 def test_delayed_geometry_without_gyro_cannot_bypass_control_response(self):
  w,c=policy(motion=model('control',True))
  for i in range(3):
   w.value=sample(i+2,100.2+i*.2,x=.1+i*.02);c.evaluate(c.phase,w.value.stamp+.01)
  c.evaluate(c.phase,w.value.stamp+.6)
  self.assertEqual(c.command(),(0.,0.));self.assertTrue(c.diagnostic_snapshot()['prediction_recheck_active'])
  self.assertEqual(c.debug['motion_prediction']['state'],'unavailable')
 def test_degraded_accepted_response_cannot_select_unchecked_legacy_alternative(self):
  m=model('control',True);m['max_age_sec']=.65;w,c=policy(motion=m)
  for i in range(3):
   w.value=sample(i+2,100.2+i*.2,x=.1+i*.02);c.evaluate(c.phase,w.value.stamp+.01)
  before=c.command()
  with patch.object(c,'_swept_support',return_value=False),patch.object(c,'_supported_command',side_effect=AssertionError('unchecked legacy alternative')):
   report=c.evaluate(c.phase,w.value.stamp+.4)
  self.assertFalse(report.faulted);self.assertEqual(c.command(),before)
 def test_shadow_job_does_not_block_command_return(self):
  w,c=policy(motion=model());started=threading.Event();release=threading.Event()
  original=c._check_response_state
  def blocked(*args,**kwargs):
   started.set();release.wait(.5);return original(*args,**kwargs)
  with patch.object(StairFeedback,'_check_response_state',side_effect=blocked):
   try:
    for i in range(3):
     w.value=sample(i+2,100.2+i*.2,x=.1+i*.02)
     begin=time.perf_counter();c.evaluate(c.phase,w.value.stamp+.01);elapsed=time.perf_counter()-begin
    self.assertTrue(started.wait(.1));self.assertLess(elapsed,.1)
    self.assertEqual(c.debug['motion_prediction']['state'],'pending')
   finally:release.set()
  if c._prediction_pending:c._prediction_pending[1].result(timeout=1.)
 def test_shadow_does_not_change_commands_or_faults(self):
  wa,a=policy();wb,b=policy(motion=model())
  for i in range(6):
   wa.value=wb.value=sample(i+2,100.2+i*.2,x=.1+i*.02,y=.05,yaw=.03)
   ra=a.evaluate(a.phase,wa.value.stamp+.01);rb=b.evaluate(b.phase,wb.value.stamp+.01)
   self.assertEqual(ra.faulted,rb.faulted);np.testing.assert_allclose(a.command(),b.command())
  self.assertFalse(b.debug['motion_prediction']['affects_commands'])

class ContinuityTest(unittest.TestCase):
 def settings(self):return dict(flight_stable_sec=.6,flight_recovery_window_sec=10.,flight_recovery_window_budget_sec=4.,flight_recovery_progress_m=.05,flight_recheck_sec=2.,flight_restart_v=.3)
 def test_stable_forward_new_observations_restore_budget(self):
  w,c=policy(self.settings());c._flight_recheck_used=1.2;c._input_gap_used=.8
  for i in range(6):
   w.value=sample(i+2,100.2+i*.2,x=.1+i*.03);c.evaluate(c.phase,w.value.stamp+.01)
  self.assertEqual(c._flight_recheck_used,0);self.assertEqual(c._input_gap_used,0)
 def test_repeated_or_motionless_frames_do_not_restore_budget(self):
  for repeated in (True,False):
   w,c=policy(self.settings());c._flight_recheck_used=1.2
   for i in range(5):
    w.value=sample(2 if repeated else i+2,100.2 if repeated else 100.2+i*.2,x=.1)
    c.evaluate(c.phase,w.value.stamp+.01)
   self.assertEqual(c._flight_recheck_used,1.2)
 def test_rolling_budget_counts_overlaps_once_and_expires(self):
  _,c=policy(self.settings());c._recovery_events.extend([(100.,102.),(101.,103.)])
  self.assertEqual(c._rolling_recovery(104.),3.)
  self.assertEqual(c._rolling_recovery(114.),0.)
 def test_continuous_failure_remains_bounded(self):
  w,c=policy(self.settings())
  with patch.object(c,'_flight_command',return_value=None):
   for i in range(12):
    w.value=sample(i+2,100.2+i*.2,x=.1)
    r=c.evaluate(c.phase,w.value.stamp+.01)
   self.assertTrue(r.faulted)
 def test_second_flight_accepts_corridor_away_from_waypoint(self):
  w,c=policy(dict(flight_resume_corridor_yaw=.17));c.begin_phase(Phase.FORWARD_SEGMENT_2,100.)
  w.value=sample(2,100.2,x=2.5,y=1.2,z=1.,yaw=math.pi)
  c.evaluate(c.phase,100.21)
  self.assertFalse(c.debug['second_flight_alignment']);self.assertGreater(c.command()[0],0.)
 def test_second_flight_wrong_lane_or_heading_keeps_alignment(self):
  for y,yaw in [(0.,math.pi),(1.,math.pi-.5)]:
   w,c=policy(dict(flight_resume_corridor_yaw=.17));c.begin_phase(Phase.FORWARD_SEGMENT_2,100.)
   w.value=sample(2,100.2,x=2.5,y=y,z=1.,yaw=yaw);c.evaluate(c.phase,100.21)
   self.assertTrue(c.debug['second_flight_alignment'])
 def test_parallel_lateral_offset_can_acquire_capture_heading(self):
  _,c=policy(dict(self.settings(),flight_recovery_w=.06));c._last_command_at=100.;p=np.array([.2,.2]);body=np.asarray(c.route['footprint'])+p
  self.assertIsNotNone(c._flight_recovery(p,0.,0.,body,[c.route['flight_1_polygon']],.02,.6,100.2,-.04,0.))

class LastRiserTest(unittest.TestCase):
 def settings(self):return dict(flight_end_advance_m=.28,flight_end_advance_sec=2.,flight_end_flat_tolerance_m=.04,flight_end_height_gap_m=.22,min_flight_v=.3)
 def upper_plane(self,seq,t,x=2.,z=.8):
  s=sample(seq,t,x=x,z=z)
  cloud=np.array([[a,b,1.-z] for a in np.linspace(.24,.78,9) for b in np.linspace(-.2,.2,9)])
  return replace(s,display_points=tuple(cloud.flat))
 def test_upper_plane_can_resume_bounded_last_step_without_arrival(self):
  w,c=policy(self.settings())
  for i in range(4):
   w.value=self.upper_plane(i+2,100.2+i*.2);r=c.evaluate(c.phase,w.value.stamp+.01)
  self.assertFalse(r.complete);self.assertFalse(r.faulted);self.assertGreater(c.command()[0],0.)
  self.assertTrue(c.debug['end_geometry']['continued'])
  w.value=self.upper_plane(9,103.);c.evaluate(c.phase,103.01)
  self.assertEqual(c.command(),(0.,0.))
 def test_missing_points_or_single_scan_or_xy_jitter_cannot_resume(self):
  w,c=policy(self.settings());w.value=self.upper_plane(2,100.2);c.evaluate(c.phase,100.21)
  self.assertEqual(c.command(),(0.,0.))
  w.value=sample(3,100.4,x=1.99,z=.8);c.evaluate(c.phase,100.41)
  self.assertEqual(c.command(),(0.,0.));self.assertFalse(c.debug['end_geometry']['continued'])
 def test_registration_voxel_density_can_confirm_upper_plane(self):
  w,c=policy(self.settings())
  for i in range(4):
   points=np.array([[a,b,.2] for a in (.3,.5,.7) for b in (-.1,.1)])
   w.value=replace(sample(i+2,100.2+i*.2,x=2.,z=.8),display_points=tuple(points.flat))
   c.evaluate(c.phase,w.value.stamp+.01)
  self.assertTrue(c.debug['end_geometry']['continued'])
 def test_open3d_twenty_centimeter_voxel_output_can_confirm_plane(self):
  from stair_supervisor.lidar_tracking_core import voxel_downsample_points
  points=np.array([[a,b,.2] for a in np.linspace(0.,.9,51) for b in np.linspace(-.5,.5,51)])
  reduced=voxel_downsample_points(points,.2)
  w,c=policy(self.settings())
  for i in range(4):
   w.value=replace(sample(i+2,100.2+i*.2,x=2.,z=.8),display_points=tuple(reduced.flat))
   c.evaluate(c.phase,w.value.stamp+.01)
  self.assertTrue(c.debug['end_geometry']['continued'])
 def test_dense_line_does_not_count_as_plane(self):
  w,c=policy(self.settings())
  for i in range(4):
   points=np.array([[a,0.,.2] for a in np.linspace(.24,.78,81)])
   w.value=replace(sample(i+2,100.2+i*.2,x=2.,z=.8),display_points=tuple(points.flat))
   c.evaluate(c.phase,w.value.stamp+.01)
  self.assertFalse(c.debug['end_geometry']['continued'])
 def test_distance_budget_is_cumulative_and_not_reset_by_votes(self):
  w,c=policy(self.settings())
  for i in range(4):
   w.value=self.upper_plane(i+2,100.2+i*.2);c.evaluate(c.phase,w.value.stamp+.01)
  w.value=self.upper_plane(7,101.,x=2.3);c.evaluate(c.phase,101.01)
  self.assertEqual(c.command(),(0.,0.));self.assertFalse(c.debug['end_geometry']['continued'])

if __name__=='__main__':unittest.main()
