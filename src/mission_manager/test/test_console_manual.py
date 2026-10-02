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
