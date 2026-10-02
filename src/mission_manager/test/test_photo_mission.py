import unittest,tempfile
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace as NS
from mission_manager.photo_mission import PhotoMission
from mission_manager.mission_orchestrator import MissionOrchestrator
from mission_manager.mission_types import *
from mission_manager.route_planner import BuildingPlanner
from mission_manager.site_config import ConfigurationRoots,load_site_configuration_from_roots

class Executor:
 def __init__(self,locations):self.locations=locations;self.events=[];self.cancel=False;self.fail_photo=False;self.fail_stair=False
 def execute(self,s,c):
  self.events.append((s.type.value,s.target_id,s.profile_id))
  if self.fail_stair and s.type.value=='STAIR':return SegmentExecution.failed(5,'stair failed')
  return SegmentExecution.success()
 def photo_origin(self,key):return replace(self.locations[key],x=1.234,y=2.345,yaw=.6)
 def capture_photo(self,key,out,cancel):
  if self.fail_photo:return SegmentExecution.failed(7,'no fresh RGB')
  if cancel():return SegmentExecution.cancelled('cancelled')
  self.events.append(('PHOTO',key,None));return SegmentExecution.success(str(Path(out)/(key+'.jpg')))
 def return_photo_origin(self,location,cancel):
  self.events.append(('RETURN_POSE',location,None));return SegmentExecution.success()
 def cancel_active(self):pass

class PhotoTourTest(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
  root=Path(__file__).resolve().parents[2]
  config=load_site_configuration_from_roots(ConfigurationRoots(*(root/pkg/'config' for pkg in ['mission_manager','multifloor_manager','stair_supervisor'])))
  self.locations={x.id:x for x in config.locations};self.planner=BuildingPlanner(config,'home_3f')
  self.executor=Executor(self.locations)
  self.host=MissionOrchestrator(self.planner,self.executor,MissionOrchestratorSettings('stair_5f_to_rf','roof_scan_profile'),start_from_current_pose=True)
  self.settings=dict(locations=['roof_loop_sw','roof_loop_nw','roof_loop_ne','roof_loop_se'])
  self.photo=PhotoMission(self.settings,self.temp.name,self.executor);self.host.photo=self.photo
 def run_tour(self,cancel=lambda:False):
  r=self.host.run(MissionRunRequest('test_tour','roof_loop_se',MissionType.PHOTO_TOUR,True),lambda p:None,cancel)
  if r.status is SegmentExecutionStatus.SUCCESS:self.photo.complete_hold(False)
  return r
 def run_return(self):
  r=self.host.run(MissionRunRequest('test_return','stair_5f_to_rf',MissionType.RETURN_TO_START,False),lambda p:None,lambda:False)
  if r.status is SegmentExecutionStatus.SUCCESS:self.photo.complete_hold(True)
  return r
 def test_four_photos_then_wait_no_automatic_descent(self):
  r=self.run_tour();self.assertEqual(r.status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual([v[1] for v in self.executor.events if v[0]=='PHOTO'],self.settings['locations'])
  self.assertFalse(any(v[2]=='stair_5f_rf_down' for v in self.executor.events))
  self.assertEqual(self.photo.snapshot()['stage'],'WAITING_RETURN')
 def test_approval_descends_and_returns_actual_pose(self):
  self.run_tour();r=self.run_return();self.assertEqual(r.status,SegmentExecutionStatus.SUCCESS)
  self.assertTrue(any(v[2]=='stair_5f_rf_down' for v in self.executor.events))
  target=self.executor.events[-1][1];self.assertEqual((target.x,target.y,target.yaw),(1.234,2.345,.6))
  self.assertEqual(self.photo.snapshot()['stage'],'COMPLETE')
 def test_return_without_session_rejected(self):
  r=self.run_return();self.assertNotEqual(r.status,SegmentExecutionStatus.SUCCESS);self.assertEqual(self.executor.events,[])
 def test_photo_failure_does_not_advance_or_automatically_return(self):
  self.executor.fail_photo=True;r=self.run_tour();self.assertEqual(r.result_code,7)
  self.assertEqual(self.host.confirmed_location_id,'roof_loop_sw');self.assertEqual(self.photo.snapshot()['stage'],'FAILED')
  self.assertFalse(any(v[2]=='stair_5f_rf_down' for v in self.executor.events))
 def test_restart_preserves_start_and_never_moves(self):
  self.run_tour();count=len(self.executor.events)
  restored=PhotoMission(self.settings,self.temp.name,self.executor)
  self.assertEqual(restored.snapshot()['origin']['x'],1.234);self.assertEqual(restored.snapshot()['stage'],'WAITING_RETURN')
  self.assertEqual(count,len(self.executor.events))
 def test_restart_mid_return_requires_new_approval(self):
  self.run_tour();self.photo.save(stage='RETURNING')
  restored=PhotoMission(self.settings,self.temp.name,self.executor);self.assertEqual(restored.snapshot()['stage'],'INTERRUPTED')
 def test_start_on_roof_returns_on_roof_without_stair(self):
  from mission_manager.route_planner import LogicalAnchor
  self.host._anchor=LogicalAnchor('stair_rf_landing');self.run_tour();r=self.run_return()
  self.assertEqual(r.status,SegmentExecutionStatus.SUCCESS)
  self.assertFalse(any(v[0]=='STAIR' for v in self.executor.events))
  self.assertEqual(self.host.confirmed_location_id,'stair_rf_landing')
  self.host.run(MissionRunRequest('next_nav','roof_loop_sw',MissionType.NAVIGATE,False),lambda p:None,lambda:False)
  self.assertEqual(self.host.confirmed_location_id,'roof_loop_sw')
 def test_cancel_keeps_origin_for_explicit_recovery(self):
  calls=[False,True]
  r=self.run_tour(lambda:calls.pop(0) if calls else True);self.assertEqual(r.status,SegmentExecutionStatus.CANCELLED)
  self.assertEqual(self.photo.snapshot()['stage'],'CANCELLED');self.assertEqual(self.executor.events,[])
 def test_rejected_second_tour_preserves_waiting_session(self):
  self.run_tour();before=self.photo.snapshot();r=self.run_tour()
  self.assertNotEqual(r.status,SegmentExecutionStatus.SUCCESS);self.assertEqual(self.photo.snapshot(),before)
 def test_cancel_before_dispatch_does_not_create_session(self):
  self.run_tour(lambda:True);self.assertEqual(self.photo.snapshot(),{});self.assertEqual(self.executor.events,[])
 def test_changed_start_map_rejects_return_before_any_motion(self):
  self.executor.photo_map_identity=lambda floor:'map-v1';self.run_tour();count=len(self.executor.events)
  self.executor.photo_map_identity=lambda floor:'map-v2';r=self.run_return()
  self.assertNotEqual(r.status,SegmentExecutionStatus.SUCCESS);self.assertEqual(len(self.executor.events),count)
  self.assertEqual(self.photo.snapshot()['stage'],'WAITING_RETURN')
 def test_missing_hold_is_rejected_before_any_movement(self):
  from mission_manager.ros_segments import RosSegmentExecutor
  self.executor.photo_arrival_hold=None;self.executor.photo_camera=NS(readiness=lambda:True)
  self.executor.photo_preflight=RosSegmentExecutor.photo_preflight.__get__(self.executor)
  with self.assertRaises(ValueError):self.photo.plan(self.planner,'stair_5f_to_rf')
  r=self.run_tour();self.assertNotEqual(r.status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual(self.executor.events,[]);self.assertEqual(self.photo.snapshot(),{})
 def test_down_failure_is_not_return_success(self):
  self.run_tour();self.executor.fail_stair=True;r=self.run_return()
  self.assertNotEqual(r.status,SegmentExecutionStatus.SUCCESS);self.assertEqual(self.photo.snapshot()['stage'],'FAILED')
  self.assertFalse(any(v[0]=='RETURN_POSE' for v in self.executor.events))

if __name__=='__main__':unittest.main()
