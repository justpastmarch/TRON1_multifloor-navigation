from pathlib import Path
from unittest.mock import Mock
from test_photo_mission import PhotoTourTest
from mission_manager.mail_outbox import MailOutbox
from mission_manager.mission_types import SegmentExecutionStatus

class PhotoMailTest(PhotoTourTest):
 def setup_mail(self,sender):
  self.photo.mail=MailOutbox(self.photo.output,sender=sender,threaded=False)
  self.photo.mail_settings('person@example.com')
  original=self.executor.capture_photo
  def capture(key,out,cancel):
   r=original(key,out,cancel);Path(r.artifact_path).parent.mkdir(parents=True,exist_ok=True);Path(r.artifact_path).write_bytes(b'JPEG-'+key.encode());return r
  self.executor.capture_photo=capture
 def test_four_points_enqueue_and_smtp_failure_does_not_change_success(self):
  self.setup_mail(Mock(side_effect=OSError('offline')))
  result=self.run_tour();self.assertEqual(result.status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual(len(self.photo.mail.jobs),4)
  for _ in range(4):self.photo.mail.tick()
  self.assertTrue(all(j['state']=='FAILED' for j in self.photo.mail.jobs.values()))
  self.assertEqual(self.photo.snapshot()['stage'],'WAITING_RETURN')
 def test_queue_disk_failure_does_not_stop_motion(self):
  self.setup_mail(Mock());self.photo.mail.enqueue=Mock(side_effect=OSError('disk'))
  self.assertEqual(self.run_tour().status,SegmentExecutionStatus.SUCCESS)
  self.assertEqual(self.photo.mail.enqueue.call_count,4);self.assertIn('메일',self.photo.mail_error)
 def test_recipient_frozen_for_current_mission(self):
  self.setup_mail(Mock());capture=self.executor.capture_photo
  def changing(*args):self.photo.mail_settings('next@example.com');return capture(*args)
  self.executor.capture_photo=changing;self.run_tour()
  self.assertEqual({j['recipient'] for j in self.photo.mail.jobs.values()},{'person@example.com'})
  self.assertEqual(self.photo.mail_recipient,'next@example.com')
