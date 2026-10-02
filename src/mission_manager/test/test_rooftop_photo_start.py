import unittest,time,json
from types import SimpleNamespace as NS
from unittest.mock import Mock
from test_ros_console import TestAdapter
from mission_manager.ros_console import RosConsole
from multifloor_manager.msg import FloorState
class RoofTests(TestAdapter):
 def roof(self):
  c=self.make();c.locations=[dict(id='target',floor='RF')]
  c.telemetry['floor']=(time.monotonic(),dict(state=FloorState.READY,floor_id='RF'))
  c.telemetry['supervisor']=(time.monotonic(),dict(state=1,connected=True))
  route=NS(segments=[NS(source_floor='RF',profile_id=None)],to_bytes=lambda:json.dumps(dict(segments=[])).encode())
  c.planner=NS(plan_from_current_pose=lambda *a:route)
  photo=Mock();photo.settings=dict(locations=['target']);photo.plan.return_value=dict(segments=[dict(type='NAVIGATION',source_floor='RF',target_id='target',profile_id=None),dict(type='PHOTO',source_floor='RF',target_id='target')])
  c.orchestrator.photo=photo
  return c
 def test_rf_current_pose_needs_no_entry_coordinate_or_lidar_route(self):
  c=self.roof();self.assertTrue(c.plan(dict(destination='target',kind='rooftop_photo'))['can_start'])
 def test_other_floor_cannot_start(self):
  c=self.roof();c.telemetry['floor'][1]['floor_id']='5F'
  with self.assertRaises(ValueError):c.start_rooftop_photos({})
  c.client.send_goal.assert_not_called()
 def test_stair_owner_cannot_start_nav(self):
  c=self.roof();c.telemetry['supervisor'][1]['state']=2
  with self.assertRaises(ValueError):c.start_rooftop_photos({})
  c.client.send_goal.assert_not_called()
 def test_route_with_stairs_rejected(self):
  c=self.roof();c.orchestrator.photo.plan.return_value['segments'].append(dict(type='STAIR',source_floor='RF'))
  self.assertFalse(c.plan(dict(destination='target',kind='rooftop_photo'))['can_start'])
 def test_reuses_photo_tour_goal(self):
  c=self.roof();r=c.start_rooftop_photos({});self.assertTrue(r['submitted']);goal=c.client.send_goal.call_args[0][0];self.assertEqual(goal.mission_type,'photo_tour');self.assertEqual(goal.destination_id,'target');self.assertFalse(goal.return_after_task)
 def test_record_button_uses_existing_recorder(self):
  c=self.roof();c.start_rooftop_photos(dict(record=True));c.recorder.start.assert_called_once()

 def test_stale_owner_rejected(self):
  c=self.roof();c.telemetry['supervisor']=(time.monotonic()-5,dict(state=1,connected=True))
  with self.assertRaises(ValueError):c.start_rooftop_photos({})
  c.client.send_goal.assert_not_called()
