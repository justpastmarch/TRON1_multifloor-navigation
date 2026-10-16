import os
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
            for topic in ('/mission_manager/rgb/compressed', '/apriltag_camera/camera_info',
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

    def test_minipc_mode_records_raw_sensor_bag_separately(self):
        with tempfile.TemporaryDirectory() as directory:
            tool = Path(directory)/'fake_remote_record.sh'
            def remote_run(argv, **_kwargs):
                if argv[1] == 'copy':
                    Path(argv[3]).touch()
                return type('Result', (), dict(returncode=0, stderr=''))()
            env = dict(STAIR_EXECUTION_HOST='minipc', TRON_REMOTE_RECORD_TOOL=str(tool))
            with patch.dict(os.environ, env), patch('os.path.ismount',return_value=True), \
                 patch('subprocess.run', side_effect=remote_run), \
                 patch('subprocess.Popen',side_effect=FakeProcess):
                recorder=ConsoleRecording(directory,directory)
                started=recorder.start()
                excluded=re.compile(started['excluded_topics_regex'])
                self.assertIsNotNone(excluded.fullmatch('/livox/lidar'))
                self.assertIsNotNone(excluded.fullmatch('/livox/imu'))
                self.assertIsNone(excluded.fullmatch('/stair_supervisor/tracking_status'))
                recorder.stop()
                deadline=time.monotonic()+2
                while recorder.snapshot()['state']=='stopping' and time.monotonic()<deadline:
                    time.sleep(.01)
                self.assertEqual(recorder.snapshot()['state'],'saved')
                while recorder.snapshot()['raw_copies'][-1]['state']=='copying' and time.monotonic()<deadline:
                    time.sleep(.01)
                self.assertEqual(recorder.snapshot()['raw_copies'][-1]['state'],'saved')
                self.assertTrue(Path(started['raw_bag_path']).exists())

    def test_raw_copy_does_not_block_next_recording_after_local_bag_is_saved(self):
        with tempfile.TemporaryDirectory() as directory:
            copying, release = threading.Event(), threading.Event()
            def remote_run(argv, **_kwargs):
                if argv[1] == 'copy':
                    copying.set()
                    release.wait(2)
                    Path(argv[3]).touch()
                return type('Result', (), dict(returncode=0, stderr=''))()
            env = dict(STAIR_EXECUTION_HOST='minipc', TRON_REMOTE_RECORD_TOOL='/fake/remote')
            with patch.dict(os.environ, env), patch('os.path.ismount',return_value=True), \
                 patch('subprocess.run',side_effect=remote_run), \
                 patch('subprocess.Popen',side_effect=FakeProcess):
                recorder=ConsoleRecording(directory,directory)
                try:
                    recorder.start();recorder.stop()
                    deadline=time.monotonic()+2
                    while recorder.snapshot()['state']=='stopping' and time.monotonic()<deadline:
                        time.sleep(.01)
                    self.assertTrue(copying.wait(1))
                    self.assertEqual(recorder.snapshot()['state'],'saved')
                    self.assertFalse(recorder.snapshot()['busy'])
                    self.assertEqual(recorder.snapshot()['raw_copies'][-1]['state'],'copying')
                    recorder.start()
                    self.assertEqual(recorder.snapshot()['state'],'recording')
                finally:
                    release.set()
                    recorder.stop()
                    deadline=time.monotonic()+2
                    while (recorder.snapshot()['state']=='stopping' or
                           any(item['state']=='copying' for item in recorder.snapshot()['raw_copies'])) and time.monotonic()<deadline:
                        time.sleep(.01)

    def test_unverified_remote_seal_blocks_only_new_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            def remote_run(argv, **_kwargs):
                return type('Result', (), dict(returncode=1 if argv[1]=='seal' else 0, stderr='seal failed'))()
            env=dict(STAIR_EXECUTION_HOST='minipc',TRON_REMOTE_RECORD_TOOL='/fake/remote')
            with patch.dict(os.environ,env),patch('os.path.ismount',return_value=True), \
                 patch('subprocess.run',side_effect=remote_run),patch('subprocess.Popen',side_effect=FakeProcess):
                recorder=ConsoleRecording(directory,directory)
                recorder.start();recorder.stop()
                deadline=time.monotonic()+2
                while recorder.snapshot()['state']=='stopping' and time.monotonic()<deadline:
                    time.sleep(.01)
                self.assertEqual(recorder.snapshot()['state'],'saved')
                self.assertFalse(recorder.snapshot()['busy'])
                self.assertTrue(recorder.snapshot()['raw_capture_unverified'])
                with self.assertRaisesRegex(ValueError,'이전 원본 녹화 종료'):
                    recorder.start()

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
                      '/apriltag_camera/image_raw', '/apriltag_camera/image_raw/compressed',
                      '/camera1/depth/image_rect_raw/compressedDepth',
                      '/camera1/infra1/image_rect_raw/compressed',
                      '/camera1/infra2/image_rect_raw/compressed',
                      '/camera1/infra2/image_rect_raw/theora',
                      '/camera1/aligned_depth_to_color/image_raw/compressedDepth'):
            self.assertFalse(recorded_topic(topic), topic)
        for topic in ('/livox/lidar','/livox/imu','/tron/sensor_joy','/stair_supervisor/control_debug',
                      '/camera1/color/camera_info','/mission_manager/rgb/compressed',
                      '/apriltag_camera/camera_info','/tag_detections',
                      '/apriltag_camera/image_raw_extra'):
            self.assertTrue(recorded_topic(topic), topic)


if __name__=='__main__':unittest.main()
