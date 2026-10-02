import unittest,tempfile,threading,time
from pathlib import Path
from unittest.mock import patch,Mock
import cv2,numpy as np,rospy
from sensor_msgs.msg import CompressedImage
from mission_manager.photo_capture import PhotoCapture
from mission_manager.mission_types import SegmentExecutionStatus

class CaptureTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  with patch('rospy.Subscriber'),patch('rospy.on_shutdown'):
   self.camera=PhotoCapture('/fake/image/compressed',.2)
  self.now=100.
  self.clock=patch('rospy.Time.now',side_effect=lambda:rospy.Time.from_sec(self.now));self.clock.start();self.addCleanup(self.clock.stop)
 def frame(self,stamp=100.05,invalid=False):
  m=CompressedImage();m.header.stamp=rospy.Time.from_sec(stamp);m.format='jpeg'
  m.data=b'bad' if invalid else cv2.imencode('.jpg',np.full((24,32,3),(20,80,150),np.uint8))[1].tobytes()
  return m
 def capture(self,frame,cancel=lambda:False):
  self.now=100.
  def deliver():self.now=100.1;self.camera.receive(frame)
  timer=threading.Timer(.03,deliver);timer.start()
  try:return self.camera.capture('roof_loop_sw',self.tmp.name,cancel)
  finally:timer.join()
 def test_new_rgb_is_readable_jpeg_with_no_temp_left(self):
  self.camera.receive(self.frame(99.9))
  r=self.capture(self.frame());self.assertEqual(r.status,SegmentExecutionStatus.SUCCESS)
  image=cv2.imread(r.artifact_path);self.assertEqual(image.shape,(24,32,3))
  self.assertEqual(len(list(Path(self.tmp.name).iterdir())),1)
 def test_old_cached_frame_is_not_a_new_photo(self):
  self.camera.receive(self.frame());r=self.camera.capture('x',self.tmp.name,lambda:False)
  self.assertEqual(r.status,SegmentExecutionStatus.FAILED);self.assertEqual(list(Path(self.tmp.name).iterdir()),[])
 def test_stale_header_or_invalid_image_does_not_report_saved(self):
  for frame in (self.frame(90.),self.frame(99.9),self.frame(100.),self.frame(invalid=True)):
   r=self.capture(frame);self.assertEqual(r.status,SegmentExecutionStatus.FAILED)
  self.assertEqual(list(Path(self.tmp.name).iterdir()),[])
 def test_cancel_writes_nothing(self):
  r=self.capture(self.frame(),lambda:True);self.assertEqual(r.status,SegmentExecutionStatus.CANCELLED)
  self.assertEqual(list(Path(self.tmp.name).iterdir()),[])
