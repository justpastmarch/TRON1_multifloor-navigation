import unittest,copy,math
from pathlib import Path
import numpy as np
from dataclasses import replace
from test_lidar_control import Worker,sample
from stair_supervisor.configuration import load_lidar_configuration,load_stair_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.map_entry import map_entry_anchor

class DownRouteTest(unittest.TestCase):
 def setUp(self):
  root=Path(__file__).resolve().parents[1]/'config'
  self.config=load_lidar_configuration(root/'stair_lidar_3f_4f_test.yaml','control',False)
  self.route=next(r for r in self.config.document['routes'] if r['id']=='stair_5f_rf_down')
  self.up=next(r for r in self.config.document['routes'] if r['id']=='stair_5f_rf_up')
  profile=next(r for r in load_stair_configuration(root).profiles if r.id==self.route['id'])
  self.w=Worker();self.c=StairFeedback(self.w,np.eye(4),[self.route]);self.c.capture_entry(profile.id,np.eye(4),.1,'synthetic surveyed anchor',100.);self.c.arm(profile,100.)
  self.now=100.;self.seq=1
 def step(self,phase,x,y,z,yaw):
  if self.c.phase!=phase:self.c.begin_phase(phase,self.now)
  self.now+=.2;self.seq+=1;self.w.value=sample(self.seq,self.now,x=x,y=y,z=z,yaw=yaw)
  r=self.c.evaluate(phase,self.now+.01);self.assertFalse(r.faulted,self.c.debug);return r
 def test_reverse_geometry_has_connected_reversed_heights(self):
  for i,j in [(1,3),(2,2),(3,1)]:
   self.assertEqual(self.route['flight_%d'%i],self.up['flight_%d'%j][::-1])
  self.assertEqual(self.route['entry_polygon'],self.up['exit_polygon'])
  self.assertEqual(self.route['exit_polygon'],self.up['entry_polygon'])
 def test_forward_inputs_short_upper_lower(self):
  for phase,xyz,yaw,expected in [(Phase.FORWARD_SEGMENT_1,(-.46,2.8,3.5),-math.pi/2,.0495),(Phase.FORWARD_SEGMENT_2,(.5,1.47,2.8),0.,.0495),(Phase.FORWARD_SEGMENT_3,(1.5,0.,1.),math.pi,.055)]:
   self.c.begin_phase(phase,self.now);self.c._last_command=(expected,0.);self.c._last_command_at=self.now
   self.step(phase,*xyz,yaw);self.assertAlmostEqual(self.c.command()[0],expected)
 def test_all_downhill_flights_need_height_drop_and_supported_destination(self):
  for phase,mid,end,yaw in [(Phase.FORWARD_SEGMENT_1,(-.46,2.35,3.35),(-.46,1.695,3.23),-math.pi/2),(Phase.FORWARD_SEGMENT_2,(.9,1.47,2.3),(2.5,1.47,1.7),0.),(Phase.FORWARD_SEGMENT_3,(1.,0.,.7),(-.45,0.,0.),math.pi)]:
   self.step(phase,*mid,yaw);result=self.step(phase,*end,yaw);self.assertTrue(result.complete,self.c.debug)
  self.assertTrue(all(self.c._flight_seen))
 def test_downhill_height_is_upper_bound_not_upward_lower_bound(self):
  self.assertFalse(self.c._ascent_height_reached(2.,1.7));self.assertTrue(self.c._ascent_height_reached(1.7,1.7));self.assertTrue(self.c._ascent_height_reached(1.6,1.7))
 def test_middle_landing_rotation_uses_region_policy(self):
  self.step(Phase.ROOFTOP_TURN,2.5,1.47,1.7,0.)
  self.assertEqual(self.c.turn_index,1)
  self.step(Phase.ROOFTOP_TURN,2.6,1.2,1.7,-math.pi/2)
  self.assertEqual(self.c.turn_index,2);self.assertEqual(self.c.command()[0],0.)
 def test_exit_captures_supported_actual_position_not_precise_nominal(self):
  self.c._flight_seen=[True,True,True]
  r=self.step(Phase.EXIT_CONFIRM,-.72,.04,0.,math.pi)
  self.assertTrue(r.complete,self.c.debug);self.assertEqual(self.c.command(),(0.,0.))
  self.assertAlmostEqual(self.c._exit_hold_target[0][0],-.72)
 def test_exit_rejects_wrong_height_and_missing_flight_evidence(self):
  self.c._flight_seen=[True,True,True]
  r=self.step(Phase.EXIT_CONFIRM,-.72,.04,.4,math.pi);self.assertFalse(r.complete)
  self.c._flight_seen=[True,False,True]
  r=self.step(Phase.EXIT_CONFIRM,-.72,.04,0.,math.pi);self.assertFalse(r.complete)
 def test_stationary_rotation_disk_near_margin_has_exact_support_proof(self):
  from stair_supervisor.stair_feedback import se2
  body=np.column_stack((self.route['footprint'],np.zeros(4),np.ones(4)))
  for x,expected in [(2.40956,True),(2.407,False)]:
   pose=se2(x,1.47,.68);support=(pose@body.T).T[:,:2]
   self.assertEqual(self.c._swept_support(pose[:2,3],.68,support,0.,.2,.9,[self.route['roof_landing_polygon']],.12),expected)
 def test_map_entry_preserves_measured_roll_pitch(self):
  from scipy.spatial.transform import Rotation
  local=np.eye(4);local[:3,:3]=Rotation.from_euler('ZYX',[.2,.1,-.08]).as_matrix()
  m=replace(sample(2,100.),transform=tuple(local.flat));loc=dict(x=4.,y=5.,yaw=-1.,floor_id='RF')
  anchor=map_entry_anchor(self.route,loc,(4.,5.,-1.),[.001]*3,m,np.eye(4),'RF',100.1,100.1,100.)
  body=np.linalg.inv(np.asarray(anchor.local_from_profile).reshape(4,4))@local
  angles=Rotation.from_matrix(body[:3,:3]).as_euler('ZYX')
  self.assertAlmostEqual(angles[1],.1);self.assertAlmostEqual(angles[2],-.08)
 def test_map_covariance_is_carried_into_support_margin(self):
  loc=dict(x=4.,y=5.,yaw=-1.,floor_id='RF');m=sample(2,100.)
  good=map_entry_anchor(self.route,loc,(4.,5.,-1.),[.001]*3,m,np.eye(4),'RF',100.1,100.1,100.)
  poor=map_entry_anchor(self.route,loc,(4.,5.,-1.),[.25]*3,m,np.eye(4),'RF',100.1,100.1,100.)
  self.assertGreater(poor.uncertainty_m,good.uncertainty_m);self.assertGreater(poor.uncertainty_m,1.)
  for covariance in ([float('nan'),0,0],[-1,0,0],[0,0,.3]):
   with self.assertRaises(ValueError):map_entry_anchor(self.route,loc,(4.,5.,-1.),covariance,m,np.eye(4),'RF',100.1,100.1,100.)
 def test_map_anchor_preserves_offset_and_rejects_stale_wrong_floor(self):
  loc=dict(x=4.,y=5.,yaw=-1.,floor_id='RF');m=sample(2,100.,x=1.,y=2.,z=3.,yaw=.2)
  a=map_entry_anchor(self.route,loc,(4.1,5.,-1.),[.01,.01,.01],m,np.eye(4),'RF',100.1,100.1,100.)
  pose=np.linalg.inv(np.array(a.local_from_profile).reshape(4,4))@np.array(m.transform).reshape(4,4)
  self.assertAlmostEqual(np.linalg.norm(pose[:2,3]-self.route['flight_1'][0][:2]),.1)
  for floor,stamp in [('5F',100.),('RF',90.)]:
   with self.assertRaises(ValueError):map_entry_anchor(self.route,loc,(4.,5.,-1.),[.01]*3,m,np.eye(4),floor,100.1,100.1,stamp)

if __name__=='__main__':unittest.main()
