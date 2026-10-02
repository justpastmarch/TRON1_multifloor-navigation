import unittest,math,copy
from pathlib import Path
from dataclasses import replace
import numpy as np,yaml
from test_lidar_control import sample,Worker
from test_supervisor import make_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase

class LandingRegionTest(unittest.TestCase):
 def setup_control(self):
  path=Path(__file__).resolve().parents[1]/'config/stair_lidar_3f_4f_test.yaml'
  r=next(r for r in yaml.safe_load(path.read_text())['routes'] if r['id']=='stair_5f_rf_up');r=copy.deepcopy(r);r['id']='test_up';r.pop('motion_prediction',None)
  w=Worker();c=StairFeedback(w,np.eye(4),[r]);c.capture_entry('test_up',np.eye(4),.1,'recorded frame fixture',100.)
  c.arm(replace(make_configuration().profiles[0],timeout_sec=120.),100.);c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,100.)
  return w,c
 def test_captures_safe_pivot_before_fixed_center(self):
  w,c=self.setup_control();original=copy.deepcopy(c.route['turn_path']);w.value=sample(2,100.2,x=2.5,z=1.99)
  result=c.evaluate(c.phase,100.21)
  self.assertFalse(result.faulted);self.assertEqual(c.turn_index,1);self.assertGreater(c.command()[1],0)
  self.assertAlmostEqual(c.command()[0],0);self.assertEqual(c._landing_path()[1][:2],[2.5,0.]);self.assertEqual(c.route['turn_path'],original)
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
  self.assertFalse(result.faulted,c.debug);self.assertGreater(c.command()[0],0.)

 def test_drift_while_turning_never_requests_return_translation(self):
  for y,yaw in [(0.3,40.),(.7,70.),(1.1,80.)]:
   w,c=self.setup_control();w.value=sample(2,100.2,x=2.5,z=1.7);c.evaluate(c.phase,100.21)
   c._last_command=(.06,.1);c._last_command_at=100.39
   w.value=sample(3,100.4,x=2.65,y=y,z=1.7,yaw=math.radians(yaw))
   result=c.evaluate(c.phase,100.41)
   self.assertFalse(result.faulted,c.debug);self.assertEqual(c.turn_index,1)
   self.assertEqual(c.command()[0],0.);self.assertGreater(c.command()[1],0.)

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
