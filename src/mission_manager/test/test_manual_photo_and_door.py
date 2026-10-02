import tempfile,threading,time,unittest
from pathlib import Path
from mission_manager.photo_mission import PhotoMission
from mission_manager.mission_types import SegmentExecution
class Mail:
 def __init__(self):self.calls=[];self.fail=False
 def enqueue(self,*args):
  if self.fail:raise OSError('disk')
  self.calls.append(args);return 'job1'
 def snapshot(self):return dict(jobs=[],configured=True)
class Executor:
 def __init__(self):self.enter=threading.Event();self.release=threading.Event();self.fail=False;self.calls=[]
 def capture_photo(self,loc,out,cancel):
  self.calls.append((loc,out));self.enter.set();self.release.wait(2)
  if self.fail:return SegmentExecution.failed(7,'fresh RGB frame timeout')
  out.mkdir(parents=True);f=out/'image.jpg';f.write_bytes(b'image');return SegmentExecution.success(str(f))
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.e=Executor();self.m=Mail();self.p=PhotoMission({},self.tmp.name,self.e,self.m);self.p.session={'stage':'OUTBOUND','mission_id':'tour','mail_recipient':'tour@example.com'}
 def finish(self):
  self.e.release.set()
  for _ in range(100):
   if not self.p.mail_snapshot()['manual_capture']['busy']:return
   time.sleep(.01)
  self.fail('worker blocked')
 def test_capture_recipient_and_no_tour_mutation(self):
  old=self.p.snapshot();r=self.p.capture_and_mail('one@example.com');self.assertTrue(r['accepted']);self.e.enter.wait(1)
  with self.assertRaises(ValueError):self.p.capture_and_mail('two@example.com')
  self.p.mail_settings('next@example.com');self.finish()
  self.assertEqual(self.p.snapshot(),old);self.assertEqual(self.m.calls[0][-1],'one@example.com');self.assertEqual(self.p.mail_snapshot()['manual_capture']['state'],'QUEUED');self.assertEqual(len(self.e.calls),1)
 def test_invalid(self):
  for a in ['',None,'invalid','a@example.com\r\nBcc: x@example.com']:
   with self.assertRaises(ValueError):self.p.capture_and_mail(a)
  self.assertFalse(self.e.calls)
 def test_camera_failure(self):
  self.e.fail=True;self.p.capture_and_mail('one@example.com');self.finish();self.assertFalse(self.m.calls);self.assertEqual(self.p.manual_capture['state'],'FAILED')
 def test_queue_failure_preserves_image(self):
  self.m.fail=True;self.p.capture_and_mail('one@example.com');self.finish();self.assertTrue(Path(self.p.manual_capture['path']).is_file());self.assertEqual(self.p.manual_capture['state'],'FAILED')
class DoorTests(unittest.TestCase):
 def test_manual_does_not_change_automatic_schedule(self):
  from mission_manager.console_door import ConsoleDoor
  entered=threading.Event();release=threading.Event();calls=[]
  def sender():entered.set();release.wait(1);calls.append(1);return 200
  door=ConsoleDoor(lambda:dict(mode='live'),sender=sender,threaded=False)
  door._start('up','run','automatic');door.next_at=123
  door.manual_open({});entered.wait(1)
  with self.assertRaises(ValueError):door.manual_open({})
  self.assertEqual((door.active,door.mode,door.run_id,door.next_at),(True,'up','run',123))
  release.set()
  for _ in range(100):
   if not door.snapshot()['one_shot']['busy']:break
   time.sleep(.01)
  self.assertEqual(door.snapshot()['one_shot']['state'],'SENT');self.assertEqual(calls,[1]);door.close()
 def test_door_failure(self):
  from mission_manager.console_door import ConsoleDoor
  door=ConsoleDoor(lambda:dict(mode='live'),sender=lambda:500,threaded=False)
  door.manual_open({})
  for _ in range(100):
   if not door.snapshot()['one_shot']['busy']:break
   time.sleep(.01)
  self.assertEqual(door.snapshot()['one_shot']['state'],'FAILED');door.close()
if __name__=='__main__':unittest.main()
