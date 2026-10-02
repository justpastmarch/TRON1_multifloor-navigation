import tempfile,unittest,threading,json
from pathlib import Path
from unittest.mock import Mock,patch
from mission_manager.mail_outbox import MailOutbox,DeliveryUnknown,SmtpSender,address

class OutboxTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name);self.photo=self.root/'a.jpg';self.photo.write_bytes(b'jpeg-test')
 def box(self,sender):
  if isinstance(sender,Mock):sender.status.return_value=dict(configured=True,sender="test@example.com")
  return MailOutbox(self.root,sender=sender,threaded=False)
 def test_persistent_per_point_dedup_and_success(self):
  sender=Mock();b=self.box(sender)
  key=b.enqueue('mission','SW',self.photo,'person@example.com');self.assertEqual(key,b.enqueue('mission','SW',self.photo,'person@example.com'))
  b.enqueue('mission','NW',self.photo,'person@example.com');self.assertEqual(len(b.jobs),2)
  b.tick();b.tick();self.assertEqual(sender.call_count,2);self.assertTrue(all(x['state']=='SENT' for x in b.jobs.values()))
  restored=self.box(sender);restored.tick();self.assertEqual(sender.call_count,2)
 def test_failure_retry_unknown_and_restart(self):
  b=self.box(Mock(side_effect=RuntimeError('SECRET')));key=b.enqueue('m','SW',self.photo,'p@example.com');b.tick();self.assertEqual(b.jobs[key]['state'],'FAILED');self.assertNotIn('SECRET',str(b.snapshot()))
  b.sender=Mock(side_effect=DeliveryUnknown());b.retry(key);b.tick();self.assertEqual(b.jobs[key]['state'],'UNKNOWN')
  with self.assertRaises(ValueError):b.retry(key)
  b.retry(key,True);b.jobs[key]['state']='SENDING';b._save();restored=self.box(Mock());self.assertEqual(restored.jobs[key]['state'],'UNKNOWN')
 def test_corrupt_history_and_attachment_escape(self):
  (self.root/'mail-outbox.json').write_text('{broken');b=self.box(Mock());self.assertFalse(b.storage_ok)
  with self.assertRaises(ValueError):b.enqueue('m','SW',self.photo,'p@example.com')
  self.assertEqual((self.root/'mail-outbox.json').read_text(),'{broken')
 def test_background_sender_does_not_block_enqueue(self):
  entered=threading.Event();finish=threading.Event()
  def sender(job):entered.set();finish.wait(2)
  b=MailOutbox(self.root,sender=sender);self.addCleanup(b.close);self.addCleanup(finish.set)
  b.enqueue('m','SW',self.photo,'p@example.com');self.assertTrue(entered.wait(1))
  b.enqueue('m','NW',self.photo,'p@example.com');self.assertEqual(len(b.jobs),2);finish.set()
 def test_invalid_recipient(self):
  for v in ['bad','a@b.com\r\nBcc:x@y.com','a@b.com,c@d.com',None]:
   with self.assertRaises(ValueError):address(v)
 def test_smtp_attachment_and_quit_failure_not_duplicate(self):
  import hashlib
  sender=SmtpSender();settings=dict(host='test.invalid',port=465,security='ssl',username='u',from_address='u@example.com',password_file=str(self.root/'secret'));(self.root/'secret').write_text('private')
  client=Mock();client.send_message.return_value={};client.quit.side_effect=OSError()
  with patch.object(sender,'settings',return_value=settings),patch('mission_manager.mail_outbox.smtplib.SMTP_SSL',return_value=client):
   sender(dict(path=str(self.photo),sha256=hashlib.sha256(self.photo.read_bytes()).hexdigest(),recipient='p@example.com',location='SW',mission_id='m',id='one'))
  client.send_message.assert_called_once();message=client.send_message.call_args.args[0];self.assertEqual(message['To'],'p@example.com');self.assertEqual(next(message.iter_attachments()).get_payload(decode=True),b'jpeg-test')
 def test_enqueue_save_failure_is_not_sendable(self):
  sender=Mock();b=self.box(sender)
  with patch.object(b,'_save',side_effect=OSError('disk')):
   with self.assertRaises(OSError):b.enqueue('m','SW',self.photo,'p@example.com')
  b.tick();self.assertEqual(b.jobs,{});sender.assert_not_called()
 def test_presend_save_failure_can_be_retried_without_restart(self):
  sender=Mock();b=self.box(sender);key=b.enqueue('m','SW',self.photo,'p@example.com')
  with patch.object(b,'_save',side_effect=OSError('disk')):
   with self.assertRaises(OSError):b.tick()
  sender.assert_not_called();self.assertEqual(b.jobs[key]['state'],'FAILED')
  b.retry(key);b.tick();sender.assert_called_once();self.assertEqual(b.jobs[key]['state'],'SENT')
 def test_missing_job_fields_disable_worker_and_preserve_history(self):
  content=json.dumps({'a':{'state':'QUEUED'}});(self.root/'mail-outbox.json').write_text(content)
  b=self.box(Mock());self.assertFalse(b.storage_ok);self.assertEqual((self.root/'mail-outbox.json').read_text(),content)
