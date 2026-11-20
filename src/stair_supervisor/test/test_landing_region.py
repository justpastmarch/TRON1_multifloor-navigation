import unittest,math,copy
from pathlib import Path
from dataclasses import replace
import numpy as np,yaml
from test_lidar_control import sample,Worker
from test_supervisor import make_configuration
from stair_supervisor.stair_feedback import StairFeedback,support_clearance
from stair_supervisor.stair_evidence import Phase

class LandingRegionTest(unittest.TestCase):
 def test_oct16_recorded_starts_reach_second_ascent_in_ideal_kinematic_loop(self):
  # Recorded initial positions; subsequent motion is an ideal differential-drive
  # model, not a replay claim or a guarantee about physical TRON dynamics.
  starts=((3.09763354,.34227449,math.radians(70.1)),
          (3.144274935,1.389106816,math.radians(94.6084)),
          (3.13635,1.47878,math.radians(95.4)))
  for x,y,yaw in starts:
   with self.subTest(start=(x,y,yaw)):
    w,c=self.setup_control();c.turn_index=1
    c._connect_landing_leg(np.array([3.09763354,.34227449]),.03);c.turn_index=2
    for step in range(600):
     t=100.2+step*.1
     w.value=sample(step+2,t,x=x,y=y,z=1.7,yaw=yaw)
     result=c.evaluate(c.phase,t+.01)
     self.assertFalse(result.faulted,c.debug)
     if result.complete:
      c.begin_phase(Phase.FORWARD_SEGMENT_2,t+.02)
      w.value=sample(step+3,t+.2,x=x,y=y,z=1.7,yaw=yaw)
      result=c.evaluate(c.phase,t+.21)
      self.assertFalse(result.faulted,c.debug)
      self.assertEqual(c.command()[0],.55)
      break
     v,angular=c.command()
     self.assertLessEqual(abs(v),.195+1e-9)
     self.assertLessEqual(abs(angular),.2+1e-9)
     x+=v*math.cos(yaw)*.1;y+=v*math.sin(yaw)*.1;yaw+=angular*.1
    else:self.fail('landing turn did not hand off to second ascent')

 def test_rotation_space_one_cm_boundary_keeps_body_margin_separate(self):
  for clearance,accepted in ((.009,False),(.01,True),(.011,True)):
   with self.subTest(clearance=clearance):
    w,c=self.setup_control();c.turn_index=2
    x=3.45-math.hypot(.225,.225)-clearance
    c._landing_turn_path[2][:2]=[x,1.47]
    c._landing_turn_path[3][:2]=[x,1.47]
    pose=np.eye(4);pose[:3,3]=[x,1.47,1.7]
    c._capture_landing_turn(pose,.03,False)
    self.assertEqual(c.turn_index,3 if accepted else 2)
    self.assertEqual(c._landing_turn_debug['margin_m'],.01)
    self.assertEqual(c._landing_turn_debug['body_support_margin_m'],.03)

 def test_legacy_rotation_margin_falls_back_and_next_pivot_uses_override(self):
  w,c=self.setup_control();c.turn_index=1
  x=3.45-math.hypot(.225,.225)-.02
  c._connect_landing_leg(np.array([x,0.]),.03)
  self.assertAlmostEqual(c._landing_path()[2][0],3.45-math.hypot(.225,.225)-.03-.1)
  c.route['limits'].pop('rotation_space_margin_m')
  c.turn_index=2
  pose=np.eye(4);pose[:3,3]=[x,1.47,1.7]
  c._capture_landing_turn(pose,.03,False)
  self.assertEqual(c.turn_index,2)
  self.assertEqual(c._landing_turn_debug['margin_m'],.03)

 def test_oct16_recorded_rotation_block_is_not_claimed_fixed_by_one_cm(self):
  w,c=self.setup_control();c.turn_index=2
  c._landing_turn_path[2][:2]=[3.0976335401785704,1.47]
  c._landing_turn_path[3][:2]=[3.0976335401785704,1.47]
  pose=np.eye(4);pose[:3,3]=[3.1442749351001402,1.3891068159548554,1.7378875837874599]
  c._capture_landing_turn(pose,.03,False)
  self.assertFalse(c._landing_turn_debug['rotation_disk_supported'])
  self.assertEqual(c.turn_index,2)

 def test_recorded_roof_entry_recovers_heading_before_fast_ascent(self):
  # Ideal differential-drive integration tests policy continuity, not hardware.
  for x,y,yaw in [(-.8557306,1.2808697,1.753855),(-.7471981,1.5998152,1.87121)]:
   w,c=self.setup_control();c.begin_phase(Phase.ROOFTOP_TURN,100.);c.turn_index=2
   now=100.2
   for seq in range(2,1102):
    w.value=sample(seq,now,x=x,y=y,z=3.28,yaw=yaw)
    result=c.evaluate(Phase.ROOFTOP_TURN,now+.01)
    self.assertFalse(result.faulted,result.detail)
    if seq==2:self.assertFalse(result.complete)
    if result.complete:break
    v,angular=c.command()
    self.assertLessEqual(abs(v),.195+1e-9)
    x+=v*math.cos(yaw)*.1;y+=v*math.sin(yaw)*.1;yaw+=angular*.1;now+=.1
   else:self.fail('roof entry did not complete')
   c.begin_phase(Phase.FORWARD_SEGMENT_3,now+.02)
   w.value=sample(seq+1,now+.2,x=x,y=y,z=3.28,yaw=yaw)
   result=c.evaluate(Phase.FORWARD_SEGMENT_3,now+.21)
   self.assertFalse(result.faulted,result.detail)
   self.assertAlmostEqual(c.command()[0],.55)

 def setup_control(self,band=True):
  path=Path(__file__).resolve().parents[1]/'config/stair_lidar_3f_4f_test.yaml'
  r=next(r for r in yaml.safe_load(path.read_text())['routes'] if r['id']=='stair_5f_rf_up');r=copy.deepcopy(r);r['id']='test_up';r.pop('motion_prediction',None)
  if not band:r['limits'].pop('landing_margin_recovery_m')
  w=Worker();c=StairFeedback(w,np.eye(4),[r]);c.capture_entry('test_up',np.eye(4),.1,'recorded frame fixture',100.)
  c.arm(replace(make_configuration().profiles[0],timeout_sec=120.),100.);c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,100.)
  return w,c
 def test_cross_landing_can_request_point_one_nine_five_without_speeding_exit_hold(self):
  w,c=self.setup_control()
  self.assertEqual(c.route['limits']['landing_max_v'],.195)
  c.turn_index=2
  c._last_command=(.195,0.)
  c._last_command_at=100.2
  w.value=sample(2,100.3,x=2.71,y=.4,z=1.7,yaw=math.pi/2)
  result=c.evaluate(c.phase,100.31)
  self.assertFalse(result.faulted,c.debug)
  self.assertEqual(c.command()[0],.195)
  c.begin_phase(Phase.EXIT_CONFIRM,100.4)
  self.assertLessEqual(c._limit_command(.5,0.,100.5)[0],.06)

 def test_recorded_second_pivot_turns_normally_then_releases_flight_input(self):
  w,c=self.setup_control();c.turn_index=3
  c._last_command=(0.,.2);c._last_command_at=100.1
  w.value=sample(2,100.2,x=2.8338,y=1.7282,z=1.819,yaw=2.7007)
  result=c.evaluate(Phase.TURN_TO_NEXT_FLIGHT,100.21)
  self.assertFalse(result.faulted,c.debug)
  self.assertFalse(c.debug['turn_margin_recovery'])
  self.assertAlmostEqual(c.debug['recovery_command_margin_m'],.03)
  self.assertEqual(c.command()[0],0.)
  self.assertGreater(c.command()[1],0.)
  w.value=sample(3,100.4,x=2.8338,y=1.7282,z=1.819,yaw=3.0)
  result=c.evaluate(Phase.TURN_TO_NEXT_FLIGHT,100.41)
  self.assertFalse(result.faulted,c.debug)
  self.assertEqual(c.turn_index,4)
  self.assertFalse(c.debug['turn_margin_recovery'])
  for sequence,t,x,y,yaw in ((4,100.6,2.78,1.69,3.1),
                             (5,100.8,2.6,1.55,3.14),
                             (6,101.,2.43,1.47,3.14)):
   w.value=sample(sequence,t,x=x,y=y,z=1.819,yaw=yaw)
   result=c.evaluate(Phase.TURN_TO_NEXT_FLIGHT,t+.01)
   self.assertFalse(result.faulted,c.debug)
  self.assertTrue(result.complete,c.debug)
  c.begin_phase(Phase.FORWARD_SEGMENT_2,101.02)
  w.value=sample(7,101.2,x=2.41,y=1.47,z=1.83,yaw=3.14)
  result=c.evaluate(Phase.FORWARD_SEGMENT_2,101.21)
  self.assertFalse(result.faulted,c.debug)
  self.assertEqual(c.command()[0],.55)

 def turn_at_clearance(self,gap,band=True):
  w,c=self.setup_control(band);c.turn_index=3
  # Top footprint extent at this yaw is .225*(|cos|+|sin|).
  y=2.085-.225*(abs(math.cos(2.7007))+abs(math.sin(2.7007)))-gap
  w.value=sample(2,100.2,x=2.8338,y=y,z=1.819,yaw=2.7007)
  return c.evaluate(Phase.TURN_TO_NEXT_FLIGHT,100.21),c,w
 def test_turn_does_not_accept_body_below_three_cm_clearance_without_band(self):
  result,c,_=self.turn_at_clearance(.029,band=False)
  self.assertTrue(result.faulted)
  self.assertEqual(c.debug['tracking_state'],'CORRIDOR_LIMIT')
 def test_turn_below_three_cm_only_moves_inward_within_band(self):
  for gap in (.029,.021):
   with self.subTest(gap=gap):
    result,c,w=self.turn_at_clearance(gap)
    self.assertFalse(result.faulted,c.debug);self.assertTrue(c.debug['landing_margin_recovery'])
    pose,_,_,_,_,_,support=c._observe(w.value);v,omega=c.command()
    end=c._motion_end_body(pose[:2,3],math.atan2(pose[1,0],pose[0,0]),support,v,omega,c.route['limits']['expire_sec'])
    polygons=[c.route['landing_polygon']]
    self.assertGreaterEqual(support_clearance(end,polygons),support_clearance(support,polygons)-1e-6)
  result,c,_=self.turn_at_clearance(.019)
  self.assertTrue(result.faulted);self.assertEqual(c.debug['tracking_state'],'CORRIDOR_LIMIT')

 def test_turn_recovery_rejects_rotation_that_shrinks_clearance(self):
  w,c=self.setup_control();c.turn_index=3
  w.value=sample(2,100.2,x=2.8338,y=1.7282,z=1.819,yaw=2.7007)
  pose,_,_,_,_,_,support=c._observe(w.value)
  self.assertFalse(c._turn_recovery_improves_clearance(
   pose[:2,3],support,-.2,.9,[c.route['landing_polygon']]))

 def test_captures_safe_pivot_before_fixed_center(self):
  w,c=self.setup_control();original=copy.deepcopy(c.route['turn_path']);w.value=sample(2,100.2,x=2.5,z=1.99)
  result=c.evaluate(c.phase,100.21)
  self.assertFalse(result.faulted);self.assertEqual(c.turn_index,1);self.assertGreater(c.command()[1],0)
  self.assertAlmostEqual(c.command()[0],0);self.assertEqual(c._landing_path()[1][:2],[2.5,0.]);self.assertEqual(c.route['turn_path'],original)
 def test_roof_pivot_captures_center_region_without_exact_point(self):
  w,c=self.setup_control();self.assertEqual(c.route['roof_turn_policy'],'supported_region')
  c.begin_phase(Phase.ROOFTOP_TURN,100.)
  w.value=sample(2,100.2,x=-.08,y=1.5,z=3.23,yaw=math.pi)
  result=c.evaluate(c.phase,100.21)
  self.assertFalse(result.faulted,c.debug);self.assertEqual(c.turn_index,0)
  self.assertFalse(c._landing_turn_captured)
  w.value=sample(3,100.4,x=-.08,y=1.47,z=3.23,yaw=math.pi)
  result=c.evaluate(c.phase,100.41)
  self.assertFalse(result.faulted,c.debug);self.assertEqual(c.turn_index,0)
  w.value=sample(4,100.6,x=-.48,y=1.78,z=3.23,yaw=math.pi)
  result=c.evaluate(c.phase,100.61)
  self.assertFalse(result.faulted,c.debug);self.assertEqual(c.turn_index,1)
  self.assertTrue(c._landing_turn_captured)

 def test_timeout_preserves_last_unmet_turn_condition(self):
  w,c=self.setup_control();c.begin_phase(Phase.ROOFTOP_TURN,100.)
  w.value=sample(2,100.2,x=-.08,y=1.5,z=3.23,yaw=math.pi)
  c.evaluate(c.phase,100.21)
  self.assertIn('position',c.debug['incomplete_conditions'])
  result=c.evaluate(c.phase,221.)
  self.assertTrue(result.faulted);self.assertEqual(c.debug['tracking_state'],'TIMEOUT')
  self.assertIn('position',c.debug['last_incomplete_conditions'])
  self.assertEqual(c.debug['last_turn_index'],0)

 def test_roof_entry_accepts_positive_z_bias_only_on_supported_upper_landing(self):
  w,c=self.setup_control();c.begin_phase(Phase.ROOFTOP_TURN,100.)
  w.value=sample(2,100.2,x=-.46,y=1.695,z=3.43,yaw=math.pi/2)
  pose,yaw,pitch,roll,velocity,angular,support=c._observe(w.value)
  margin=.06
  self.assertTrue(c._resume_corridor(2,pose,support,margin,0.))
  low=pose.copy();low[2,3]=3.23-.16
  self.assertFalse(c._resume_corridor(2,low,support,margin,0.))
  outside=pose.copy();outside[0,3]=-.05
  outside_support=c._observe(sample(3,100.4,x=-.05,y=1.695,z=3.43,yaw=math.pi/2))[6]
  self.assertFalse(c._resume_corridor(2,outside,outside_support,margin,0.))

 def test_no_pivot_with_body_over_stair_edge(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.1,z=1.99)
  c.evaluate(c.phase,100.21);self.assertEqual(c.turn_index,0);self.assertFalse(c._landing_turn_debug['rotation_disk_supported'])
 def test_no_pivot_near_wall(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=3.2,z=1.99)
  c.evaluate(c.phase,100.21);self.assertEqual(c.turn_index,0);self.assertFalse(c._landing_turn_debug['rotation_disk_supported'])
 def test_region_progress_does_not_need_stationary_dwell(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.99);c.evaluate(c.phase,100.21)
  c._settled=lambda *a,**k:False;c._settling_detail={}
  w.value=sample(3,100.4,x=2.58,z=1.99,yaw=math.pi/2);c.evaluate(c.phase,100.41)
  self.assertEqual(c.turn_index,2);self.assertNotIn('settled',c.debug['completion_checks'])
 def test_new_turn_resets_captured_pivot(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.99);c.evaluate(c.phase,100.21)
  c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,100.3);self.assertEqual(c.turn_index,0);self.assertEqual(c._landing_path(),c.route['turn_path'])
 def test_stale_geometry_does_not_select_new_pivot(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.99);c.evaluate(c.phase,100.8);self.assertEqual(c.turn_index,0)
 def test_second_pivot_uses_arrived_position(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.99);c.evaluate(c.phase,100.21)
  c.turn_index=2;w.value=sample(3,100.4,x=2.5,y=1.4,z=1.99,yaw=math.pi/2);c.evaluate(c.phase,100.41)
  self.assertEqual(c.turn_index,3);self.assertEqual(c._landing_path()[3][:2],[2.5,1.4])

 def test_turn_progress_accepts_fifteen_degree_heading_error(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.99)
  c.evaluate(c.phase,100.21);self.assertEqual(c.turn_index,1)
  w.value=sample(3,100.4,x=2.5,z=1.99,yaw=math.pi/2-math.radians(15.))
  result=c.evaluate(c.phase,100.41)
  self.assertFalse(result.faulted,c.debug);self.assertEqual(c.turn_index,2)
  self.assertTrue(c.debug['completion_checks']['yaw'])

 def test_turn_progress_keeps_twenty_degree_heading_boundary(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.99)
  c.evaluate(c.phase,100.21);self.assertEqual(c.turn_index,1)
  w.value=sample(3,100.4,x=2.5,z=1.99,yaw=math.pi/2-math.radians(25.))
  c.evaluate(c.phase,100.41)
  self.assertEqual(c.turn_index,1);self.assertFalse(c.debug['completion_checks']['yaw'])

 def test_final_entry_handoff_still_accepts_connected_flight(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
  c.turn_index=4;c._settled=lambda *a,**k:False;c._settling_detail={}
  w.value=sample(3,100.4,x=2.42,y=1.47,z=1.7,yaw=math.pi)
  result=c.evaluate(c.phase,100.41);self.assertTrue(result.complete,c.debug);self.assertFalse(result.faulted)
  c.begin_phase(Phase.FORWARD_SEGMENT_2,100.42)
  w.value=sample(4,100.6,x=2.42,y=1.47,z=1.7,yaw=math.pi)
  result=c.evaluate(c.phase,100.61);self.assertFalse(result.faulted,c.debug);self.assertFalse(c.debug['second_flight_alignment']);self.assertGreater(c.command()[0],0.)
 def test_corrected_geometry_keeps_height_and_next_flight_join(self):
  w,c=self.setup_control();r=c.route
  self.assertAlmostEqual(r['flight_1'][1][2],r['flight_2'][0][2])
  self.assertAlmostEqual(r['flight_2'][0][0],r['landing_polygon'][0][0])
  self.assertAlmostEqual(max(p[0] for p in r['flight_2_polygon']),r['flight_2'][0][0])
  c.begin_phase(Phase.FORWARD_SEGMENT_1,100.1)
  w.value=sample(2,100.2,x=2.5,z=1.49);result=c.evaluate(c.phase,100.21)
  self.assertFalse(result.complete);self.assertFalse(c.debug['completion_checks']['height'])

 def test_recorded_large_drift_finishes_rotation_and_rebases_crossing(self):
  # BAG: heading satisfied after drift >1m, no manual input yet.
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.445289,y=.179792,z=1.7)
  c.evaluate(c.phase,100.21)
  w.value=sample(3,100.4,x=2.78735,y=1.21810,z=1.8454,yaw=math.radians(85.195))
  result=c.evaluate(c.phase,100.41)
  self.assertFalse(result.faulted,c.debug);self.assertEqual(c.turn_index,2)
  self.assertNotIn('position',c.debug['completion_checks'])
  self.assertEqual(c.command()[0],0.)
  self.assertAlmostEqual(c._landing_path()[2][0],2.78735)
  self.assertAlmostEqual(c._landing_path()[2][1],1.47)
  self.assertEqual(c._landing_path()[-1],c.route['turn_path'][-1])
  w.value=sample(4,100.6,x=2.78735,y=1.21810,z=1.8454,yaw=math.pi/2)
  result=c.evaluate(c.phase,100.61)
  self.assertFalse(result.faulted,c.debug)
  self.assertEqual(c.turn_index,3)  # already inside the second-flight lateral corridor
  self.assertEqual(c.command()[0],0.)
  self.assertGreater(c.command()[1],0.)

 def test_drift_while_turning_never_requests_return_translation(self):
  for y,yaw in [(0.3,20.),(.7,40.),(1.1,60.)]:
   w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
   c._last_command=(.06,.1);c._last_command_at=100.39
   w.value=sample(3,100.4,x=2.65,y=y,z=1.7,yaw=math.radians(yaw))
   result=c.evaluate(c.phase,100.41)
   self.assertFalse(result.faulted,c.debug);self.assertEqual(c.turn_index,1)
   self.assertEqual(c.command()[0],0.);self.assertGreater(c.command()[1],0.)

 def test_bag_turn_clearance_uses_landing_margin_without_match_budget(self):
  # Recorded ui_test_20261002_093915_a4e3d864: raw footprint clearance was
  # 0.1157 m. The commissioned 0.03 m landing margin keeps this pose in the
  # normal supported path; the separate 0.10 m entry-match budget is not added.
  w,c=self.setup_control();c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,100.1)
  c.turn_index=3;c._landing_turn_captured=True
  w.value=sample(2,100.2,x=2.77558695312926,y=1.652328946046151,
                 z=1.69345898377945,yaw=2.4447381851635757)
  result=c.evaluate(c.phase,100.21)
  self.assertFalse(result.faulted,c.debug)
  self.assertTrue(c.debug['footprint_supported'])
  self.assertTrue(c.debug['current_supported'])
  self.assertAlmostEqual(c.debug['clearance_m'],.1157195,places=5)
  self.assertAlmostEqual(c.debug['effective_margin_m'],.03)
  self.assertAlmostEqual(c.debug['recovery_command_margin_m'],.03)
  self.assertEqual(c.command()[0],0.)
  self.assertGreater(c.command()[1],0.)

 def test_degraded_rotation_does_not_advance_waypoint(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
  w.value=sample(3,100.4,x=2.6,y=.5,z=1.7,yaw=math.pi/2)
  c.evaluate(c.phase,101.0);self.assertEqual(c.turn_index,1)

 def test_heading_alone_outside_support_does_not_complete(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
  w.value=sample(3,100.4,x=3.4,z=1.7,yaw=math.pi/2)
  result=c.evaluate(c.phase,100.41)
  self.assertTrue(result.faulted);self.assertEqual(c.turn_index,1)

 def test_second_rotation_drift_still_requires_approach_to_next_entry(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
  c.turn_index=2;w.value=sample(3,100.4,x=2.5,y=1.4,z=1.7,yaw=math.pi/2);c.evaluate(c.phase,100.41)
  self.assertEqual(c.turn_index,3)
  w.value=sample(4,100.6,x=2.8,y=1.47,z=1.7,yaw=math.pi);result=c.evaluate(c.phase,100.61)
  self.assertEqual(c.turn_index,4);self.assertFalse(result.complete)
  w.value=sample(5,100.8,x=2.8,y=1.47,z=1.7,yaw=math.pi);result=c.evaluate(c.phase,100.81)
  self.assertFalse(result.complete);self.assertFalse(c.debug['completion_checks']['next_flight_corridor'])
  self.assertGreater(c.command()[0],0.)

 def test_final_handoff_accepts_lateral_offset_inside_next_corridor(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
  c.turn_index=4
  w.value=sample(3,100.4,x=2.433,y=1.374,z=1.7,yaw=3.124)
  result=c.evaluate(c.phase,100.41);self.assertTrue(result.complete,c.debug)
  self.assertGreater(np.linalg.norm(np.array([2.433,1.374])-np.array(c._landing_path()[-1][:2])),c.route['limits']['position_off'])
 def test_final_handoff_rejects_wrong_heading(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
  c.turn_index=4
  w.value=sample(3,100.4,x=2.433,y=1.374,z=1.7,yaw=math.pi/2)
  result=c.evaluate(c.phase,100.41);self.assertFalse(result.complete)
 def test_next_phase_old_observation_cannot_capture_pivot(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7)
  c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,100.21);c.evaluate(c.phase,100.22)
  self.assertEqual(c.turn_index,0)

 def test_one_scan_cannot_complete_rotation_and_capture_next_pivot(self):
  w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
  w.value=sample(3,100.4,x=2.65,y=1.47,z=1.7,yaw=math.pi/2);c.evaluate(c.phase,100.41)
  self.assertEqual(c.turn_index,2)
  c.evaluate(c.phase,100.42);self.assertEqual(c.turn_index,2)
  w.value=sample(4,100.6,x=2.65,y=1.47,z=1.7,yaw=math.pi/2);c.evaluate(c.phase,100.61)
  self.assertEqual(c.turn_index,3)

if __name__=='__main__':unittest.main()
