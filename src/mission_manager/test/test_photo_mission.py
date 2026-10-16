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
 def __init__(self,locations):self.locations=locations;self.events=[];self.cancel=False;self.fail_photo_keys=set();self.cancel_photo_keys=set();self.raise_photo_keys=set();self.fail_stair=False
 def execute(self,s,c):
  self.events.append((s.type.value,s.target_id,s.profile_id))
  if self.fail_stair and s.type.value=='STAIR':return SegmentExecution.failed(5,'stair failed')
  return SegmentExecution.success()
 def prepare_rooftop_navigation(self):self.events.append(('WALK',None,None))
 def photo_origin(self,key):return replace(self.locations[key],x=1.234,y=2.345,yaw=.6)
 def capture_photo(self,key,out,cancel):
  if key in self.cancel_photo_keys:return SegmentExecution.cancelled('photo cancelled')
  if key in self.raise_photo_keys:raise RuntimeError('camera capture exception')
  if key in self.fail_photo_keys:return SegmentExecution.failed(7,'no fresh RGB')
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
 def test_revisit_first_then_roof_exit_photographs_every_occurrence(self):
  from unittest.mock import Mock
  self.settings['locations']=['roof_loop_sw','roof_loop_nw','roof_loop_sw','roof_loop_se','stair_rf_landing']
  self.photo.mail=Mock();self.photo.mail_recipient='operator@example.com'
  self.assertEqual(self.run_tour().status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual([e[1] for e in self.executor.events if e[0]=='PHOTO'],self.settings['locations'])
  self.assertEqual([e[1] for e in self.executor.events if e[0]=='NAVIGATION' and e[1].startswith(('roof_loop_', 'stair_rf_'))],self.settings['locations'])
  self.assertEqual([p['visit_index'] for p in self.photo.session['photos']],[0,1,2,3,4])
  self.assertEqual(self.host.confirmed_location_id,'stair_rf_landing')
  self.assertEqual(self.photo.mail.enqueue.call_count,5)
  # Identical bytes at a repeated location must still get a separate mail identity.
  calls=self.photo.mail.enqueue.call_args_list
  self.assertNotEqual(calls[0][0][0],calls[2][0][0])

 def test_repeated_visit_retry_preserves_only_completed_occurrences(self):
  self.settings['locations']=['roof_loop_sw','roof_loop_nw','roof_loop_sw','stair_rf_landing']
  session=dict(origin={'id':'stair_rf_landing'},stage='FAILED',route_locations=list(self.settings['locations']),
               photos=[dict(location='roof_loop_sw',visit_index=0),dict(location='roof_loop_nw',visit_index=1)])
  self.assertTrue(self.photo._retry_available(session))
  self.assertEqual(self.photo._remaining_visits(session),[(2,'roof_loop_sw'),(3,'stair_rf_landing')])
  session['route_locations']=['roof_loop_sw','roof_loop_nw','roof_loop_ne','roof_loop_se']
  self.assertFalse(self.photo._retry_available(session))

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
 def test_photo_failure_is_recorded_and_navigation_continues(self):
  self.executor.fail_photo_keys={'roof_loop_sw'};r=self.run_tour()
  self.assertEqual(r.status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual(self.host.confirmed_location_id,'roof_loop_se');self.assertEqual(self.photo.snapshot()['stage'],'WAITING_RETURN')
  self.assertEqual(self.photo.snapshot()['photo_outcome'],'PARTIAL')
  self.assertEqual([x['location'] for x in self.photo.snapshot()['missed_photos']],['roof_loop_sw'])
  self.assertEqual([x['location'] for x in self.photo.snapshot()['photos']],self.settings['locations'][1:])
  self.assertFalse(any(v[2]=='stair_5f_rf_down' for v in self.executor.events))
 def test_capture_exception_is_recorded_and_later_points_continue(self):
  self.executor.raise_photo_keys={'roof_loop_nw'};r=self.run_tour()
  self.assertEqual(r.status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual(self.host.confirmed_location_id,'roof_loop_se')
  self.assertEqual([x['location'] for x in self.photo.snapshot()['missed_photos']],['roof_loop_nw'])
  self.assertIn('RuntimeError',self.photo.snapshot()['missed_photos'][0]['reason'])
 def test_partial_capture_status_survives_approved_return(self):
  self.executor.fail_photo_keys={'roof_loop_sw'};self.run_tour();r=self.run_return()
  self.assertEqual(r.status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual(self.photo.snapshot()['photo_outcome'],'PARTIAL')
  self.assertIn('촬영 일부 누락',self.photo.snapshot()['detail'])
 def test_cancelled_partial_tour_stays_cancelled_and_keeps_missed_point(self):
  self.executor.fail_photo_keys={'roof_loop_sw'};self.executor.cancel_photo_keys={'roof_loop_nw'}
  r=self.host.run(MissionRunRequest('cancelled_tour','roof_loop_se',MissionType.PHOTO_TOUR,True),lambda p:None,lambda:False)
  self.assertEqual(r.status,SegmentExecutionStatus.CANCELLED)
  self.assertEqual(self.photo.snapshot()['stage'],'CANCELLED')
  self.assertEqual(self.photo.snapshot()['photo_outcome'],'PARTIAL')
  self.assertIn('roof_loop_sw',self.photo.snapshot()['detail'])
 def test_cancelled_incomplete_tour_is_not_marked_complete(self):
  self.executor.cancel_photo_keys={'roof_loop_sw'}
  r=self.host.run(MissionRunRequest('cancelled_tour','roof_loop_se',MissionType.PHOTO_TOUR,True),lambda p:None,lambda:False)
  self.assertEqual(r.status,SegmentExecutionStatus.CANCELLED)
  self.assertEqual(self.photo.snapshot()['photo_outcome'],'INCOMPLETE')
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
 def test_failed_stair_retries_from_current_anchor_and_keeps_original_return_pose(self):
  from mission_manager.route_planner import LogicalAnchor
  self.executor.fail_stair=True
  failed=self.run_tour()
  self.assertEqual(failed.status,SegmentExecutionStatus.FAILED)
  original=self.photo.snapshot()['origin'].copy()
  self.photo=PhotoMission(self.settings,self.temp.name,self.executor)
  self.host.photo=self.photo
  self.assertTrue(self.photo.snapshot()['retry_available'])
  self.host._anchor=LogicalAnchor('stair_5f_to_rf')
  self.executor.fail_stair=False
  before=len(self.executor.events)
  planned=self.photo.plan(self.planner,self.host.confirmed_location_id)
  self.assertEqual(planned['origin_id'],'stair_5f_to_rf')
  self.assertEqual(len([segment for segment in planned['segments'] if segment['type']=='PHOTO']),4)
  retried=self.run_tour()
  self.assertEqual(retried.status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual(self.photo.snapshot()['origin'],original)
  self.assertEqual(self.executor.events[before][1],'stair_5f_to_rf')
  self.assertEqual(self.photo.snapshot()['stage'],'WAITING_RETURN')
  self.assertFalse(self.photo.snapshot()['retry_available'])
  self.assertEqual(self.run_return().status,SegmentExecutionStatus.SUCCESS)
  returned=self.executor.events[-1][1]
  self.assertEqual((returned.x,returned.y,returned.yaw),(original['x'],original['y'],original['yaw']))

 def test_retry_skips_saved_photo_and_keeps_original_session(self):
  from mission_manager.route_planner import LogicalAnchor
  execute=self.executor.execute
  failed_once=[False]
  def fail_after_first_photo(segment,context):
   if segment.type.value=='NAVIGATION' and segment.target_id=='roof_loop_nw' and not failed_once[0]:
    failed_once[0]=True
    return SegmentExecution.failed(7,'injected navigation failure')
   return execute(segment,context)
  self.executor.execute=fail_after_first_photo
  failed=self.run_tour()
  self.assertEqual(failed.status,SegmentExecutionStatus.FAILED)
  saved=self.photo.snapshot()
  self.assertEqual([x['location'] for x in saved['photos']],['roof_loop_sw'])
  self.assertTrue(saved['retry_available'])
  self.executor.execute=execute
  self.host._anchor=LogicalAnchor('roof_loop_se')
  planned=self.photo.plan(self.planner,self.host.confirmed_location_id)
  self.assertEqual([x['target_id'] for x in planned['segments'] if x['type']=='PHOTO'],self.settings['locations'][1:])
  retried=self.run_tour()
  self.assertEqual(retried.status,SegmentExecutionStatus.SUCCESS)
  completed=self.photo.snapshot()
  self.assertEqual(completed['mission_id'],saved['mission_id'])
  self.assertEqual(completed['origin'],saved['origin'])
  self.assertEqual([x['location'] for x in completed['photos']],self.settings['locations'])
  self.assertEqual([event[1] for event in self.executor.events if event[0]=='PHOTO'],self.settings['locations'])

 def test_retry_recaptures_missed_photo_without_duplicate_miss(self):
  execute=self.executor.execute
  self.executor.fail_photo_keys={'roof_loop_sw'}
  failed_once=[False]
  def fail_second_destination(segment,context):
   if segment.type.value=='NAVIGATION' and segment.target_id=='roof_loop_nw' and not failed_once[0]:
    failed_once[0]=True
    return SegmentExecution.failed(7,'injected navigation failure')
   return execute(segment,context)
  self.executor.execute=fail_second_destination
  self.assertEqual(self.run_tour().status,SegmentExecutionStatus.FAILED)
  self.assertEqual([x['location'] for x in self.photo.snapshot()['missed_photos']],['roof_loop_sw'])
  self.executor.execute=execute
  self.executor.fail_photo_keys.clear()
  self.assertEqual(self.run_tour().status,SegmentExecutionStatus.SUCCESS)
  completed=self.photo.snapshot()
  self.assertEqual([x['location'] for x in completed['photos']],self.settings['locations'])
  self.assertEqual(completed['missed_photos'],[])
  self.assertEqual(completed['photo_outcome'],'COMPLETE')

 def test_return_failure_does_not_offer_outbound_retry(self):
  self.run_tour()
  self.photo.save(stage='RETURNING',return_started=True)
  restored=PhotoMission(self.settings,self.temp.name,self.executor)
  self.assertFalse(restored.snapshot()['retry_available'])
  with self.assertRaises(ValueError):restored.preflight(False)

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
 def test_camera_missing_does_not_block_departure_and_each_miss_is_recorded(self):
  from mission_manager.ros_segments import RosSegmentExecutor
  self.executor.photo_arrival_hold=object();self.executor.photo_camera=None
  self.executor.photo_preflight=RosSegmentExecutor.photo_preflight.__get__(self.executor)
  self.executor.fail_photo_keys=set(self.settings['locations'])
  result=self.run_tour()
  self.assertEqual(result.status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual(self.photo.snapshot()['stage'],'WAITING_RETURN')
  self.assertEqual(len(self.photo.snapshot()['missed_photos']),4)

 def test_down_failure_is_not_return_success(self):
  self.run_tour();self.executor.fail_stair=True;r=self.run_return()
  self.assertNotEqual(r.status,SegmentExecutionStatus.SUCCESS);self.assertEqual(self.photo.snapshot()['stage'],'FAILED')
  self.assertFalse(any(v[0]=='RETURN_POSE' for v in self.executor.events))

if __name__=='__main__':unittest.main()
