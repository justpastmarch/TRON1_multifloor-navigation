"""Wall-clock child protocol behavior; no ROS/action server connection."""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from actionlib_msgs.msg import GoalStatus
from mission_manager.ros_segments import ChildActionWait

class ChildWaitTest(unittest.TestCase):
 def make(self):
  now=[0.];client=SimpleNamespace(action_client=SimpleNamespace(last_status_msg=object()),cancel_goal=Mock(),
   gh=SimpleNamespace(comm_state_machine=SimpleNamespace(action_goal=SimpleNamespace(goal_id=SimpleNamespace(id='our-goal')))))
  wait=ChildActionWait(client,30.,5.,clock=lambda:now[0]);wait.bind_goal()
  return now,client,wait
 def status(self,goal_id,status):
  return SimpleNamespace(status_list=[SimpleNamespace(goal_id=SimpleNamespace(id=goal_id),status=status)])
 def test_quiet_ascent_with_status_heartbeat_has_no_duration_limit(self):
  now,client,wait=self.make()
  def tick(_):
   now[0]+=1.;client.action_client.last_status_msg=self.status('our-goal',GoalStatus.ACTIVE)
   if now[0]==120:wait.completed(GoalStatus.SUCCEEDED,'arrived');return True
   return False
  with patch.object(wait.done,'wait',side_effect=tick),patch('rospy.is_shutdown',return_value=False):
   self.assertEqual(wait.wait(lambda:False),(GoalStatus.SUCCEEDED,'arrived',''))
  client.cancel_goal.assert_not_called()
 def test_missing_communication_requests_cancel_once_then_leaves_wait(self):
  now,client,wait=self.make()
  def tick(_):now[0]+=1.;return False
  with patch.object(wait.done,'wait',side_effect=tick),patch('rospy.is_shutdown',return_value=False):
   status,result,error=wait.wait(lambda:False)
  self.assertEqual((status,result),(None,None));self.assertIn('communication stopped',error)
  self.assertEqual(now[0],35.);client.cancel_goal.assert_called_once()
 def test_explicit_cancel_deadline_does_not_depend_on_fresh_status(self):
  now,client,wait=self.make()
  def tick(_):now[0]+=1.;client.action_client.last_status_msg=self.status('our-goal',GoalStatus.ACTIVE);return False
  with patch.object(wait.done,'wait',side_effect=tick),patch('rospy.is_shutdown',return_value=False):
   self.assertIn('acknowledge cancellation',wait.wait(lambda:True)[2])
  self.assertEqual(now[0],6.);client.cancel_goal.assert_called_once()
 def test_old_goal_completion_cannot_finish_new_goal_wait(self):
  now,client,old=self.make();new=ChildActionWait(client,30.,5.,clock=lambda:now[0])
  old.completed(GoalStatus.PREEMPTED,'old')
  self.assertTrue(old.done.is_set());self.assertFalse(new.done.is_set())
 def test_terminal_result_before_wait_returns_without_cancelling(self):
  _,client,wait=self.make();wait.completed(GoalStatus.SUCCEEDED,'done')
  self.assertEqual(wait.wait(lambda:True),(GoalStatus.SUCCEEDED,'done',''));client.cancel_goal.assert_not_called()
 def test_terminal_result_wins_cancel_timeout_race(self):
  now,client,wait=self.make();wait.cancel()
  def tick(_):
   now[0]=6.;wait.completed(GoalStatus.PREEMPTED,'cancelled');return False
  with patch.object(wait.done,'wait',side_effect=tick),patch('rospy.is_shutdown',return_value=False):
   self.assertEqual(wait.wait(lambda:True),(GoalStatus.PREEMPTED,'cancelled',''))
 def test_invalid_deadline_cannot_disable_communication_handling(self):
  _,client,_=self.make()
  for value in [0,-1,float('nan'),float('inf')]:
   with self.assertRaises(ValueError):ChildActionWait(client,value,5.)

 def test_silence_cancel_ack_reports_communication_failure_not_operator_cancel(self):
  now,client,wait=self.make()
  client.cancel_goal.side_effect=lambda:wait.completed(GoalStatus.PREEMPTED,'ack')
  def tick(_):now[0]=31.;return False
  with patch.object(wait.done,'wait',side_effect=tick),patch('rospy.is_shutdown',return_value=False):
   self.assertEqual(wait.wait(lambda:False),(GoalStatus.PREEMPTED,'ack','child action communication stopped'))

 def test_terminal_status_with_missing_result_does_not_wait_forever_on_heartbeats(self):
  now,client,wait=self.make();client.get_state=lambda:GoalStatus.SUCCEEDED
  def tick(_):
   now[0]+=1.;client.action_client.last_status_msg=self.status('our-goal',GoalStatus.ACTIVE);return False
  with patch.object(wait.done,'wait',side_effect=tick),patch('rospy.is_shutdown',return_value=False):
   self.assertIn('terminal status without its result',wait.wait(lambda:False)[2])
  self.assertEqual(now[0],6.);client.cancel_goal.assert_not_called()
 def test_foreign_terminal_status_does_not_end_current_active_goal(self):
  now,client,wait=self.make();client.get_state=lambda:GoalStatus.ACTIVE
  def tick(_):
   now[0]+=1.
   client.action_client.last_status_msg=SimpleNamespace(status_list=[
    SimpleNamespace(goal_id=SimpleNamespace(id='other-goal'),status=GoalStatus.SUCCEEDED),
    SimpleNamespace(goal_id=SimpleNamespace(id='our-goal'),status=GoalStatus.ACTIVE)])
   if now[0]==120:wait.completed(GoalStatus.SUCCEEDED,'own result');return True
   return False
  with patch.object(wait.done,'wait',side_effect=tick),patch('rospy.is_shutdown',return_value=False):
   self.assertEqual(wait.wait(lambda:False),(GoalStatus.SUCCEEDED,'own result',''))
  client.cancel_goal.assert_not_called()

 def test_empty_status_arrays_cannot_keep_unacknowledged_goal_alive(self):
  self.check_unacknowledged_status(lambda:self.status('other-goal',GoalStatus.ACTIVE))
  self.check_unacknowledged_status(lambda:SimpleNamespace(status_list=[]))
 def check_unacknowledged_status(self,make_status):
  now,client,wait=self.make();client.get_state=lambda:GoalStatus.PENDING
  def tick(_):
   now[0]+=1.;client.action_client.last_status_msg=make_status()
   if now[0]>36:raise AssertionError('unacknowledged goal was kept alive by unrelated status')
   return False
  with patch.object(wait.done,'wait',side_effect=tick),patch('rospy.is_shutdown',return_value=False):
   self.assertIn('communication stopped',wait.wait(lambda:False)[2])
  self.assertEqual(now[0],35.);client.cancel_goal.assert_called_once()

class UnresolvedChildTest(unittest.TestCase):
 def test_retry_waits_for_late_terminal_or_confirmed_receiver_idle(self):
  from mission_manager.ros_segments import RosSegmentExecutor
  from stair_supervisor.msg import SupervisorState
  client=SimpleNamespace(send_goal=Mock(),cancel_goal=Mock())
  # SimpleNamespace is not hashable; real action clients are identity objects.
  class Client:
   def __init__(self):self.send_goal=Mock();self.cancel_goal=Mock()
  client=Client();old=ChildActionWait(client,30.,5.);clock=[0.]
  executor=RosSegmentExecutor.__new__(RosSegmentExecutor)
  executor._active_lock=threading.RLock();executor._child_waits={client:old};executor._stair=client
  state=SimpleNamespace(state=SupervisorState.STAIR)
  executor._state=SimpleNamespace(health=lambda:SimpleNamespace(healthy=True),supervisor_state=lambda:state)
  self.assertIn('unresolved',executor._run_child(client,'new',lambda:False)[2]);client.send_goal.assert_not_called()
  state.state=SupervisorState.NAV
  client.send_goal.side_effect=lambda goal,done_cb,feedback_cb:done_cb(GoalStatus.SUCCEEDED,'newresult')
  with patch('rospy.get_param',side_effect=lambda name,default:default):
   self.assertEqual(executor._run_child(client,'new',lambda:False),(GoalStatus.SUCCEEDED,'newresult',''))
  new=executor._child_waits[client];old.completed(GoalStatus.ABORTED,'oldresult')
  self.assertEqual(new.result,'newresult')
 def test_goal_id_is_bound_after_simple_client_generates_its_new_handle(self):
  from mission_manager.ros_segments import RosSegmentExecutor
  class Client:
   def __init__(self):
    self.gh=SimpleNamespace(comm_state_machine=SimpleNamespace(action_goal=SimpleNamespace(goal_id=SimpleNamespace(id='old-goal'))))
   def send_goal(self,goal,done_cb,feedback_cb):
    self.gh=SimpleNamespace(comm_state_machine=SimpleNamespace(action_goal=SimpleNamespace(goal_id=SimpleNamespace(id='generated-new-goal'))))
  client=Client();executor=RosSegmentExecutor.__new__(RosSegmentExecutor)
  executor._active_lock=threading.RLock();executor._child_waits={};executor._stair=client
  def inspect(wait,cancelled):
   self.assertEqual(wait.goal_id,'generated-new-goal')
   return GoalStatus.SUCCEEDED,'result',''
  with patch('rospy.get_param',side_effect=lambda name,default:default),patch.object(ChildActionWait,'wait',inspect):
   self.assertEqual(executor._run_child(client,'goal',lambda:False),(GoalStatus.SUCCEEDED,'result',''))

 def test_lost_or_missing_result_does_not_release_unknown_receiver(self):
  from mission_manager.ros_segments import RosSegmentExecutor
  from stair_supervisor.msg import SupervisorState
  class Client:
   def __init__(self):self.send_goal=Mock();self.cancel_goal=Mock()
  for status,result in [(GoalStatus.LOST,None),(GoalStatus.SUCCEEDED,None),(GoalStatus.ABORTED,None)]:
   client=Client();old=ChildActionWait(client,30.,5.);old.completed(status,result)
   executor=RosSegmentExecutor.__new__(RosSegmentExecutor)
   executor._active_lock=threading.RLock();executor._child_waits={client:old};executor._stair=client
   executor._state=SimpleNamespace(health=lambda:SimpleNamespace(healthy=True),supervisor_state=lambda:SimpleNamespace(state=SupervisorState.STAIR))
   self.assertIn('unresolved',executor._run_child(client,'new',lambda:False)[2]);client.send_goal.assert_not_called()
 def test_correlated_terminal_result_allows_next_goal_without_stale_idle_wait(self):
  from mission_manager.ros_segments import RosSegmentExecutor
  class Client:
   def __init__(self):self.send_goal=Mock();self.cancel_goal=Mock()
  client=Client();old=ChildActionWait(client,30.,5.);old.completed(GoalStatus.PREEMPTED,'cancelresult')
  executor=RosSegmentExecutor.__new__(RosSegmentExecutor)
  executor._active_lock=threading.RLock();executor._child_waits={client:old};executor._stair=client
  client.send_goal.side_effect=lambda goal,done_cb,feedback_cb:done_cb(GoalStatus.SUCCEEDED,'newresult')
  with patch('rospy.get_param',side_effect=lambda name,default:default):
   self.assertEqual(executor._run_child(client,'new',lambda:False),(GoalStatus.SUCCEEDED,'newresult',''))
