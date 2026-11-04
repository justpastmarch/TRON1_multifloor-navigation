import json,threading,time,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from mission_manager.console_manual import ConsoleManual

class ManualBridgeTest(unittest.TestCase):
 def setUp(self):
  self.console=SimpleNamespace(lock=threading.RLock(),closed=False,client=Mock(),recorder=Mock(),orchestrator=Mock())
  self.patches=[patch('mission_manager.console_manual.rospy.Publisher'),patch('mission_manager.console_manual.rospy.Subscriber'),patch('mission_manager.console_manual.rospy.wait_for_service'),patch('mission_manager.console_manual.rospy.ServiceProxy')]
  publisher,subscriber,wait,service=[p.start() for p in self.patches]
  service.return_value.return_value=SimpleNamespace(success=True,message=json.dumps(dict(lease='abc')))
  self.m=ConsoleManual(self.console)
 def tearDown(self):
  for p in reversed(self.patches):p.stop()
 def test_manual_does_not_touch_mission_recording_or_mode(self):
  self.m.begin({});self.m.update(dict(lease='abc',sequence=1,forward=1,turn=-.1,issued_at=time.time()-.01));self.m.end(dict(lease='abc'))
  self.assertEqual(self.console.client.mock_calls,[])
  self.assertEqual(self.console.recorder.mock_calls,[])
  self.assertEqual(self.console.orchestrator.mock_calls,[])
  messages=[json.loads(c.args[0].data) for c in self.m.publisher.publish.call_args_list]
  self.assertEqual([m['kind'] for m in messages],['update','end'])
 def test_release_rejects_delayed_update_and_other_client(self):
  self.m.begin({});self.m.end(dict(lease='abc'))
  with self.assertRaises(ValueError):self.m.update(dict(lease='abc',sequence=1,forward=1,turn=0,issued_at=time.time()))
  with self.assertRaises(ValueError):self.m.end(dict(lease='other'))
 def test_stale_packet_never_published(self):
  self.m.begin({})
  with self.assertRaises(ValueError):self.m.update(dict(lease='abc',sequence=1,forward=1,turn=0,issued_at=time.time()-1))
  self.m.publisher.publish.assert_not_called()

class ManualLeaseLivenessTest(ManualBridgeTest):
 def test_actual_ui_stop_is_delivered_while_manual_reply_is_stalled(self):
  from collections import deque
  from mission_manager.ros_console import RosConsole
  entered,release=threading.Event(),threading.Event()
  def stalled():
   entered.set();release.wait(2)
   return SimpleNamespace(success=True,message=json.dumps(dict(lease='late')))
  self.patches[-1].target.ServiceProxy.return_value=stalled
  self.console.manual=self.m;self.console.active=True;self.console.starting=False
  self.console.stop_event=threading.Event();self.console.stop_requested=False;self.console.events=deque()
  self.console.orchestrator=SimpleNamespace(arrival_hold=None)
  self.console.recorder.snapshot.return_value=dict(busy=False,state='idle')
  errors=[]
  def begin():
   try:self.m.begin({})
   except ValueError as e:errors.append(str(e))
  worker=threading.Thread(target=begin);worker.start()
  try:
   self.assertTrue(entered.wait(1))
   stopped=threading.Event()
   stopper=threading.Thread(target=lambda:(RosConsole.cancel(self.console,{}),stopped.set()))
   stopper.start();self.assertTrue(stopped.wait(.3),'Stop waited for the manual RPC')
   self.console.client.cancel_goal.assert_called_once()
  finally:
   release.set();worker.join(1)
  self.assertFalse(worker.is_alive());self.assertTrue(errors)
  self.assertTrue(self.m.ended);self.assertIsNone(self.m.lease)
  messages=[json.loads(c.args[0].data) for c in self.m.publisher.publish.call_args_list]
  self.assertEqual(messages,[dict(kind='end',lease='late')])

 def test_timeout_tracks_pending_request_and_retires_its_late_lease(self):
  entered,release=threading.Event(),threading.Event()
  def stalled():
   entered.set();release.wait(2)
   return SimpleNamespace(success=True,message=json.dumps(dict(lease='late')))
  self.patches[-1].target.ServiceProxy.return_value=stalled
  self.m.begin_wait_sec=.02
  try:
   with self.assertRaisesRegex(ValueError,'응답이 지연'):self.m.begin({})
   self.assertTrue(entered.is_set());self.assertTrue(self.m.snapshot()['pending'])
   with self.assertRaisesRegex(ValueError,'이전 수동'):self.m.begin({})
   pending=self.m.pending
  finally:release.set()
  self.assertTrue(pending['done'].wait(1));self.assertFalse(self.m.snapshot()['pending'])
  self.assertIsNone(self.m.lease)
  self.assertEqual(json.loads(self.m.publisher.publish.call_args.args[0].data),dict(kind='end',lease='late'))
