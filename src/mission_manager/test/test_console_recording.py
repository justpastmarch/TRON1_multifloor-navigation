import signal
import re
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from mission_manager.console_recording import ConsoleRecording


class FakeProcess:
    def __init__(self, argv, **kwargs):
        self.argv, self.kwargs = argv, kwargs
        self.path = Path(argv[argv.index('-O')+1])
        Path(str(self.path)+'.active').write_bytes(b'fixture')
        self.code = None
        self.signals = []
    def poll(self): return self.code
    def send_signal(self, sig):
        self.signals.append(sig)
        Path(str(self.path)+'.active').rename(self.path)
        self.code = 0
    def wait(self): return self.code


class RecorderTest(unittest.TestCase):
    def setUp(self):
        patcher=patch.object(ConsoleRecording,'_subscriptions_ready',return_value=True)
        patcher.start();self.addCleanup(patcher.stop)
    def test_all_topics_no_split_then_sigint_finalize(self):
        with tempfile.TemporaryDirectory() as directory, patch('os.path.ismount',return_value=True), patch('subprocess.Popen',side_effect=FakeProcess):
            r=ConsoleRecording(directory,directory);s=r.start()
            self.assertEqual(s['state'],'recording')
            p=r.process
            self.assertIn('--all',p.argv)
            self.assertEqual(s['excluded_topics_regex'], p.argv[p.argv.index('--exclude')+1])
            excluded = re.compile(s['excluded_topics_regex'])
            self.assertIsNotNone(excluded.fullmatch('/apriltag_camera/image_raw'))
            for topic in ('/apriltag_camera/image_raw/compressed', '/apriltag_camera/camera_info',
                          '/tag_detections', '/livox/lidar', '/livox/imu', '/tron/sensor_joy',
                          '/stair_supervisor/control_debug'):
                self.assertIsNone(excluded.fullmatch(topic), topic)
            for forbidden in ('--split','--size','--duration','--limit'):
                self.assertNotIn(forbidden,p.argv)
            r.stop()
            deadline=time.monotonic()+2
            while r.snapshot()['state']=='stopping' and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(r.snapshot()['state'],'saved')
            self.assertEqual(p.signals,[signal.SIGINT])
            self.assertTrue(Path(s['path']).exists())

    def test_unmounted_disk_or_cancel_never_starts_process(self):
        with patch('os.path.ismount',return_value=False), patch('subprocess.Popen') as popen:
            r=ConsoleRecording()
            with self.assertRaises(ValueError):r.start()
            event=threading.Event();event.set()
            with self.assertRaises(ValueError):r.start(event)
            popen.assert_not_called()

    def test_unexpected_exit_is_not_claimed_saved(self):
        with tempfile.TemporaryDirectory() as directory, patch('os.path.ismount',return_value=True), patch('subprocess.Popen',side_effect=FakeProcess):
            r=ConsoleRecording(directory,directory);r.start();r.process.code=1
            self.assertEqual(r.snapshot()['state'],'error')
            self.assertTrue(Path(str(r.path)+'.active').exists())

    def test_escape_from_disk_rejected(self):
        with patch('os.path.ismount',return_value=True),patch('subprocess.Popen') as popen:
            with self.assertRaises(ValueError):ConsoleRecording('/tmp/outside','/mnt/ethan').start()
            popen.assert_not_called()

class CameraRecordingProfileTest(__import__('unittest').TestCase):
    def test_rgb_only_camera_profile_preserves_control_measurements(self):
        from mission_manager.console_recording import recorded_topic
        for topic in ('/camera1/color/image_raw', '/camera1/depth/image_rect_raw',
                      '/camera1/infra1/image_rect_raw', '/camera1/depth/color/points',
                      '/camera1/color/image_raw/compressed', '/camera1/color/image_raw/compressedDepth', '/camera1/depth/image_rect_raw/compressed',
                      '/apriltag_camera/image_raw',
                      '/camera1/depth/image_rect_raw/compressedDepth',
                      '/camera1/infra1/image_rect_raw/compressed',
                      '/camera1/infra2/image_rect_raw/compressed',
                      '/camera1/infra2/image_rect_raw/theora',
                      '/camera1/aligned_depth_to_color/image_raw/compressedDepth'):
            self.assertFalse(recorded_topic(topic), topic)
        for topic in ('/livox/lidar','/livox/imu','/tron/sensor_joy','/stair_supervisor/control_debug',
                      '/camera1/color/camera_info','/apriltag_camera/image_raw/compressed',
                      '/apriltag_camera/camera_info','/tag_detections',
                      '/apriltag_camera/image_raw_extra'):
            self.assertTrue(recorded_topic(topic), topic)


if __name__=='__main__':unittest.main()
