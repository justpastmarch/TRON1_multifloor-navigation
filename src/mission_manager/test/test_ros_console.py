import unittest,threading,time,json
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from mission_manager.ros_console import RosConsole
from multifloor_manager.msg import FloorState

class TestAdapter(unittest.TestCase):
 def setUp(self):
  p=patch('mission_manager.ros_console.ConsoleMap');p.start();self.addCleanup(p.stop)
  p=patch('mission_manager.ros_console.rospy.Publisher');p.start();self.addCleanup(p.stop)
 def test_constructor_reuses_node_and_publishes_no_motion(self):
  from mission_manager.configuration import Location
  config=NS(locations=[Location('target','3F','HOME',0.,0.,0.)])
  with patch('mission_manager.ros_console.actionlib.SimpleActionClient') as client, patch('mission_manager.ros_console.rospy.Subscriber'), patch('mission_manager.ros_console.rospy.on_shutdown'), patch('mission_manager.ros_console.rospy.loginfo'), patch('mission_manager.ros_console.ConsoleServer'), patch('rospy.init_node') as init:
   console=RosConsole(Mock(),Mock(),config,'/mission','unused',8765)
   self.assertEqual(console.locations[0]['id'],'target')
   client.return_value.send_goal.assert_not_called();init.assert_not_called()
 def test_bind_failure_cleans_subscriptions(self):
  with patch('mission_manager.ros_console.actionlib.SimpleActionClient'), patch('mission_manager.ros_console.rospy.Subscriber') as sub, patch('mission_manager.ros_console.ConsoleServer',side_effect=OSError('port busy')):
   with self.assertRaises(OSError):RosConsole(Mock(),Mock(),NS(locations=[]),'/mission','unused',8765)
   self.assertEqual(sub.return_value.unregister.call_count,11)  # original eight + manual status + localization
 def make(self, stairs=False):
  c=RosConsole.__new__(RosConsole);c.lock=threading.RLock();c.active=False;c.result=None;c.progress={};c.events=deque(maxlen=80)
  c.recorder=Mock();c.recorder.snapshot.return_value={'busy':False,'state':'idle'}
  c.starting=False;c.stop_requested=False;c.closed=False;c.generation=0;c.stop_event=threading.Event()
  c.map_view=Mock();c.map_view.snapshot.return_value={'map':None}
  c.locations=[{'id':'target','floor':'4F'}];c.orchestrator=NS(confirmed_location_id='origin')
  seg=NS(source_floor='3F',profile_id='stair_3f_4f_up' if stairs else None)
  route=NS(segments=[seg],to_bytes=lambda:json.dumps({'segments':[]}).encode())
  c.planner=NS(plan_from_current_pose=lambda *args:route)
  c.telemetry={'floor':(time.monotonic(),{'state':FloorState.READY,'floor_id':'3F'})}
  c.client=Mock();c.client.wait_for_server.return_value=True
  return c
 def test_unconfirmed_floor_does_not_plan_from_old_anchor(self):
  c=self.make(True);c.planner=Mock()
  c.telemetry['floor']=(time.monotonic(),{'state':FloorState.UNKNOWN,'floor_id':'5F'})
  c.telemetry['localization']=(time.monotonic(),{'floor':'5F','reason':'pose and scan do not match map'})
  result=c.plan({'destination':'target'})
  self.assertFalse(result['can_start']);self.assertEqual(result['segments'],[])
  self.assertIn('pose and scan',result['blockers'][0]);self.assertEqual(len(result['blockers']),1)
  c.planner.plan_from_current_pose.assert_not_called()
  with self.assertRaises(ValueError):c.start({'destination':'target'})
  c.client.send_goal.assert_not_called()
 def test_ready_floor_plans_using_current_resolved_anchor(self):
  c=self.make();c.orchestrator.confirmed_location_id='stair_5f_to_rf'
  route=NS(segments=[NS(source_floor='5F',profile_id=None)],to_bytes=lambda:b'{"segments":[]}')
  c.planner=Mock();c.planner.plan_from_current_pose.return_value=route
  c.telemetry['floor']=(time.monotonic(),{'state':FloorState.READY,'floor_id':'5F'})
  self.assertTrue(c.plan({'destination':'target'})['can_start'])
  c.planner.plan_from_current_pose.assert_called_once_with('stair_5f_to_rf','target')
 def test_same_floor_nav_does_not_require_lidar(self):
  c=self.make();self.assertTrue(c.plan({'destination':'target'})['can_start'])
 def test_uncommissioned_stair_is_blocked_before_send(self):
  c=self.make(True)
  with self.assertRaises(ValueError):c.start({'destination':'target'})
  c.client.send_goal.assert_not_called()
 def test_commissioned_stair_plan(self):
  c=self.make(True);c.telemetry['tracking']=(time.monotonic(),{'mode':'control','routes':[{'id':'stair_3f_4f_up','commissioned':True}]})
  self.assertTrue(c.plan({'destination':'target'})['can_start'])
 def test_photo_never_silently_becomes_bag_or_navigation(self):
  c=self.make()
  with self.assertRaises(ValueError):c.start({'destination':'target','kind':'photo_return'})
  c.client.send_goal.assert_not_called()
 def test_start_uses_mission_and_never_automatic_return(self):
  c=self.make();c.start({'destination':'target'});g=c.client.send_goal.call_args[0][0]
  self.assertEqual(g.mission_type,'navigate');self.assertFalse(g.return_after_task)
  with self.assertRaises(ValueError):c.start({'destination':'target'})
  self.assertEqual(c.client.send_goal.call_count,1)
 def test_cancel_only_own_active_goal(self):
  c=self.make()
  with self.assertRaises(ValueError):c.cancel({})
  c.active=True;c.cancel({});c.client.cancel_goal.assert_called_once();c.client.cancel_all_goals.assert_not_called()
 def test_stop_can_cancel_idle_arrival_hold_without_new_mission(self):
  c=self.make();hold=Mock();hold.core.armed=True;c.orchestrator.arrival_hold=hold
  c.cancel({});hold.suspend.assert_called_once();c.recorder.stop.assert_called_once();c.client.send_goal.assert_not_called()
 def test_stale_or_wrong_floor_is_blocked(self):
  for t,f in [(time.monotonic()-3,'3F'),(time.monotonic(),'5F')]:
   c=self.make();c.telemetry['floor']=(t,{'state':FloorState.READY,'floor_id':f})
   self.assertFalse(c.plan({'destination':'target'})['can_start'])
 def test_done_failure_remains_failure(self):
  c=self.make();c.active=True;c.done(4,None);self.assertFalse(c.active);self.assertEqual(c.result['action_status'],4)
 def test_other_client_busy_is_not_replaced(self):
  c=self.make();c.telemetry['mission_status']=(time.monotonic(),{'active':True})
  with self.assertRaises(ValueError):c.start({'destination':'target'})
  c.client.send_goal.assert_not_called()

 def test_drive_only_does_not_start_recording(self):
  c=self.make();c.start({'destination':'target','record':False});c.recorder.start.assert_not_called()
 def test_recording_is_ready_before_motion(self):
  c=self.make();order=[]
  c.recorder.start.side_effect=lambda event:order.append('record')
  c.client.send_goal.side_effect=lambda *a,**k:order.append('goal')
  c.start({'destination':'target','record':True});self.assertEqual(order,['record','goal'])
 def test_recorder_failure_prevents_motion(self):
  c=self.make();c.recorder.start.side_effect=ValueError('disk missing')
  with self.assertRaises(ValueError):c.start({'destination':'target','record':True})
  c.client.send_goal.assert_not_called();self.assertFalse(c.active)
 def test_cancel_during_record_preparation_never_sends_goal(self):
  c=self.make();c.recorder.start.side_effect=lambda event:c.cancel({})
  self.assertFalse(c.start({'destination':'target','record':True})['submitted'])
  c.client.send_goal.assert_not_called();c.recorder.stop.assert_called()
 def test_stop_and_completion_stop_owned_recording(self):
  c=self.make();c.active=True;c.cancel({});c.recorder.stop.assert_called_once()
  c.recorder.reset_mock();c.done(3,None);c.recorder.stop.assert_called_once()
 def test_late_done_does_not_stop_new_recording(self):
  c=self.make();c.generation=2;c.active=True;c.done(3,None,1)
  c.recorder.stop.assert_not_called();self.assertTrue(c.active)
 def test_finalizing_recording_blocks_duplicate_start(self):
  c=self.make();c.recorder.snapshot.return_value={'busy':True}
  with self.assertRaises(ValueError):c.start({'destination':'target'})
  c.client.send_goal.assert_not_called()
 def test_revalidate_after_recording_preparation(self):
  c=self.make()
  def stale(event):c.telemetry['floor']=(0,{'state':FloorState.READY,'floor_id':'3F'})
  c.recorder.start.side_effect=stale
  with self.assertRaises(ValueError):c.start({'destination':'target','record':True})
  c.client.send_goal.assert_not_called();c.recorder.stop.assert_called()
 def test_failure_keeps_recording_until_explicit_stop(self):
  c=self.make();c.active=True;c.recorder.snapshot.return_value={'busy':True,'state':'recording'}
  c.done(4,None)
  c.recorder.stop.assert_not_called();self.assertFalse(c.active)
  c.cancel({});c.recorder.stop.assert_called_once();c.client.cancel_goal.assert_not_called()
 def test_retry_reuses_recording_and_success_saves_it(self):
  c=self.make();c.recorder.snapshot.return_value={'busy':True,'state':'recording'}
  self.assertTrue(c.start({'destination':'target','record':True})['submitted'])
  c.recorder.start.assert_not_called();c.done(3,None);c.recorder.stop.assert_called_once()
 def test_retry_error_does_not_discard_ongoing_recovery_record(self):
  c=self.make();c.recorder.snapshot.return_value={'busy':True,'state':'recording'}
  c.client.send_goal.side_effect=ValueError('send failed')
  with self.assertRaises(ValueError):c.start({'destination':'target','record':True})
  c.recorder.stop.assert_not_called();self.assertFalse(c.active)
 def test_recording_failure_during_retry_prevents_goal(self):
  c=self.make();c.recorder.snapshot.side_effect=[{'busy':True,'state':'recording'}, {'busy':False,'state':'error'}]
  with self.assertRaises(ValueError):c.start({'destination':'target','record':True})
  c.client.send_goal.assert_not_called()
 def test_control_recovery_requires_confirmation_and_idle_fresh_state(self):
  for change in ('confirmation','active','starting','external','stale','disconnected','recovering'):
   c=self.make();c.telemetry['mission_status']=(time.monotonic(),{'active':False});c.telemetry['supervisor']=(time.monotonic(),{'state':2,'connected':True})
   data={'confirmed_flat_ground':True}
   if change=='confirmation':data={}
   elif change in ('active','starting','recovering'):setattr(c,change,True)
   elif change=='external':c.telemetry['mission_status']=(time.monotonic(),{'active':True})
   elif change=='stale':c.telemetry['supervisor']=(time.monotonic()-4,{'state':2,'connected':True})
   else:c.telemetry['supervisor']=(time.monotonic(),{'state':2,'connected':False})
   with patch('rospy.set_param') as param:
    with self.assertRaises(ValueError):c.recover_control(data)
    param.assert_not_called()
 def test_control_recovery_preserves_recording_result_and_awaits_real_nav(self):
  c=self.make();c.telemetry['mission_status']=(time.monotonic(),{'active':False});c.telemetry['supervisor']=(time.monotonic(),{'state':2,'connected':True})
  c.result={'reason':'old failure'};c.recorder.snapshot.return_value={'state':'recording','busy':True}
  with patch('rospy.set_param') as param,patch('rospy.wait_for_service'),patch('rospy.ServiceProxy',return_value=Mock(return_value=NS(success=True))) as proxy:
   self.assertTrue(c.recover_control({'confirmed_flat_ground':True})['accepted'])
   self.assertEqual([x.args[1] for x in param.call_args_list],[True,False]);proxy.return_value.assert_called_once()
  c.recorder.stop.assert_not_called();c.client.send_goal.assert_not_called()
  self.assertEqual(c.result,{'reason':'old failure'});self.assertFalse(c.recovering)
  self.assertFalse(c.plan({'destination':'target'})['can_start'])
  c.telemetry['supervisor']=(time.monotonic(),{'state':1,'connected':True})
  c.start({'destination':'target','record':True});c.client.send_goal.assert_called_once();c.recorder.start.assert_not_called()
 def test_control_recovery_rejection_clears_confirmation(self):
  c=self.make();c.telemetry['mission_status']=(time.monotonic(),{'active':False});c.telemetry['supervisor']=(time.monotonic(),{'state':2,'connected':True})
  with patch('rospy.set_param') as param,patch('rospy.wait_for_service'),patch('rospy.ServiceProxy',return_value=Mock(return_value=NS(success=False,message='no retained loss'))):
   with self.assertRaises(ValueError):c.recover_control({'confirmed_flat_ground':True})
   self.assertFalse(param.call_args.args[1]);self.assertFalse(c.recovering)
 def test_start_during_recovery_rejected_before_recording(self):
  c=self.make();c.recovering=True
  with self.assertRaises(ValueError):c.start({'destination':'target','record':True})
  c.recorder.start.assert_not_called();c.client.send_goal.assert_not_called()
 def test_nav_recovery_idempotent_without_mode_request(self):
  c=self.make();c.telemetry['mission_status']=(time.monotonic(),{'active':False});c.telemetry['supervisor']=(time.monotonic(),{'state':1,'connected':True})
  with patch('rospy.set_param') as param:
   self.assertTrue(c.recover_control({'confirmed_flat_ground':True})['accepted']);param.assert_not_called()

 def test_late_recovery_keeps_start_and_floor_selection_serialized(self):
  c=self.make();c.floor_selecting=False;c.telemetry['mission_status']=(time.monotonic(),{'active':False});c.telemetry['supervisor']=(time.monotonic(),{'state':2,'connected':True})
  release=threading.Event();entered=threading.Event()
  def delayed():
   entered.set();release.wait(8);return NS(success=True)
  with patch('rospy.set_param'),patch('rospy.wait_for_service'),patch('rospy.ServiceProxy',return_value=delayed):
   try:
    result=c.recover_control({'confirmed_flat_ground':True})
    self.assertTrue(entered.is_set());self.assertTrue(result['pending']);self.assertTrue(c.recovering)
    with self.assertRaises(ValueError):c.start({'destination':'target'})
    with self.assertRaises(ValueError):c.select_floor({'floor_id':'4F'})
    with self.assertRaises(ValueError):c.recover_control({'confirmed_flat_ground':True})
   finally:
    release.set()
    deadline=time.monotonic()+2
    while c.recovering and time.monotonic()<deadline:time.sleep(.01)
   self.assertFalse(c.recovering)

if __name__=='__main__':unittest.main()

class OperatorPhaseAdapterTest(TestAdapter):
 def setup_operator(self,c):
  c.operator_condition=threading.Condition(c.lock);c.operator_pending=False;c.operator_publisher=Mock();c.recovering=False;c.floor_selecting=False
 def test_operator_phase_preserves_action_and_routes_through_supervisor(self):
  c=self.make();self.setup_operator(c)
  data=dict(run_id='run',revision=3,phase='TURN_TO_NEXT_FLIGHT',confirmed=True)
  status=dict(active=True,run_id='run',revision=3,phases=['TURN_TO_NEXT_FLIGHT'])
  c.telemetry['operator_phase']=(time.monotonic(),status)
  def sent(message):
   request=json.loads(message.data);c.observe('operator_phase',dict(status,result=dict(request_id=request['request_id'],state='ACCEPTED')))
  c.operator_publisher.publish.side_effect=sent
  self.assertTrue(c.operator_phase(data)['accepted'])
  c.client.send_goal.assert_not_called();c.client.cancel_goal.assert_not_called();c.recorder.stop.assert_not_called()
 def test_stale_operator_run_cannot_submit(self):
  c=self.make();self.setup_operator(c);c.telemetry['operator_phase']=(time.monotonic()-2,dict(active=True))
  with self.assertRaises(ValueError):c.operator_phase({})
  c.operator_publisher.publish.assert_not_called()
 def test_waiting_for_ack_releases_lock_for_stop(self):
  c=self.make();self.setup_operator(c);c.active=True
  data=dict(run_id='run',revision=3,phase='LANDING',confirmed=True)
  status=dict(active=True,run_id='run',revision=3,phases=['LANDING'])
  c.telemetry['operator_phase']=(time.monotonic(),status);sent=threading.Event();errors=[]
  def published(message):sent.set()
  c.operator_publisher.publish.side_effect=published
  def invoke():
   try:c.operator_phase(data)
   except ValueError as error:errors.append(str(error))
  worker=threading.Thread(target=invoke);worker.start();self.assertTrue(sent.wait(1))
  started=time.monotonic();c.cancel({});self.assertLess(time.monotonic()-started,.5)
  worker.join(3);self.assertFalse(worker.is_alive());self.assertTrue(errors);self.assertFalse(c.operator_pending)
