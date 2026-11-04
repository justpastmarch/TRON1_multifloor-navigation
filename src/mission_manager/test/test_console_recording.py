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

    def wait_for_finalized(self, recorder):
        deadline = time.monotonic()+2
        while recorder.snapshot()['busy'] and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertFalse(recorder.snapshot()['busy'])

    def test_local_early_exit_seals_remote_raw_even_after_error_stop(self):
        operations=[]
        class DeadRecorder(FakeProcess):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs);self.code=1
        def remote_run(argv, **_kwargs):
            operations.append(argv[1])
            if argv[1]=='copy':Path(argv[3]).touch()
            return type('Result',(),dict(returncode=0,stderr=''))()
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(os.environ,dict(STAIR_EXECUTION_HOST='minipc',TRON_REMOTE_RECORD_TOOL='/fake/remote')), \
             patch('os.path.ismount',return_value=True),patch('subprocess.run',side_effect=remote_run), \
             patch('subprocess.Popen',side_effect=DeadRecorder):
            recorder=ConsoleRecording(directory,directory)
            with self.assertRaisesRegex(ValueError,'녹화를 시작하지 못했습니다'):recorder.start()
            recorder.stop();self.wait_for_finalized(recorder)
            self.assertIn('seal',operations)
            self.assertEqual(operations.count('seal'),1)
            self.assertEqual(recorder.snapshot()['state'],'error')
            self.assertFalse(recorder.snapshot()['raw_capture_unverified'])
            deadline=time.monotonic()+2
            while any(x['state']=='copying' for x in recorder.snapshot()['raw_copies']) and time.monotonic()<deadline:time.sleep(.01)

    def test_local_spawn_failure_does_not_leave_remote_raw_unsealed(self):
        operations=[]
        def remote_run(argv, **_kwargs):
            operations.append(argv[1])
            if argv[1]=='copy':Path(argv[3]).touch()
            return type('Result',(),dict(returncode=0,stderr=''))()
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(os.environ,dict(STAIR_EXECUTION_HOST='minipc',TRON_REMOTE_RECORD_TOOL='/fake/remote')), \
             patch('os.path.ismount',return_value=True),patch('subprocess.run',side_effect=remote_run), \
             patch('subprocess.Popen',side_effect=OSError('fixture spawn failure')):
            recorder=ConsoleRecording(directory,directory)
            with self.assertRaisesRegex(ValueError,'rosbag을 시작하지 못했습니다'):recorder.start()
            self.wait_for_finalized(recorder)
            self.assertIn('seal',operations)
            self.assertFalse(recorder.raw_pending)
            deadline=time.monotonic()+2
            while any(x['state']=='copying' for x in recorder.snapshot()['raw_copies']) and time.monotonic()<deadline:time.sleep(.01)

    def test_remote_start_timeout_attempts_seal_and_exposes_unknown_stop(self):
        operations=[]
        def remote_run(argv, **_kwargs):
            operations.append(argv[1])
            if argv[1]=='start':raise __import__('subprocess').TimeoutExpired(argv,25)
            return type('Result',(),dict(returncode=1,stderr='fixture seal failed'))()
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(os.environ,dict(STAIR_EXECUTION_HOST='minipc',TRON_REMOTE_RECORD_TOOL='/fake/remote')), \
             patch('os.path.ismount',return_value=True),patch('subprocess.run',side_effect=remote_run):
            recorder=ConsoleRecording(directory,directory)
            with self.assertRaisesRegex(ValueError,'원본 센서 녹화를 시작하지 못했습니다'):recorder.start()
            self.wait_for_finalized(recorder)
            self.assertEqual(operations,['start','seal'])
            self.assertTrue(recorder.snapshot()['raw_capture_unverified'])
            with self.assertRaisesRegex(ValueError,'이전 원본 녹화 종료'):recorder.start()
            self.assertEqual(recorder.snapshot()['raw_copies'][-1]['state'],'error')

    def test_remote_start_failure_without_allocation_does_not_block_next_recording(self):
        operations=[]
        def remote_run(argv, **_kwargs):
            operations.append(argv[1])
            failed = argv[1]=='start' and operations.count('start')==1
            return type('Result',(),dict(returncode=1 if failed else 0,
                        stderr='fixture not allocated' if failed else '',
                        stdout='RAW_BAG_ABSENT\n' if argv[1]=='seal' else ''))()
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(os.environ,dict(STAIR_EXECUTION_HOST='minipc',TRON_REMOTE_RECORD_TOOL='/fake/remote')), \
             patch('os.path.ismount',return_value=True),patch('subprocess.run',side_effect=remote_run), \
             patch('subprocess.Popen',side_effect=FakeProcess):
            recorder=ConsoleRecording(directory,directory)
            with self.assertRaisesRegex(ValueError,'fixture not allocated'):recorder.start()
            self.wait_for_finalized(recorder)
            state=recorder.snapshot()
            self.assertFalse(state['raw_capture_unverified'])
            self.assertFalse(recorder.raw_pending)
            self.assertEqual(state['raw_copies'][-1]['state'],'not_recorded')
            self.assertNotIn('copy',operations)
            self.assertEqual(recorder.start()['state'],'recording')
            recorder.stop();self.wait_for_finalized(recorder)

    def test_polling_unexpected_local_exit_closes_remote_raw(self):
        sealed=threading.Event();release=threading.Event()
        def remote_run(argv, **_kwargs):
            if argv[1]=='seal':sealed.set();release.wait(2)
            if argv[1]=='copy':Path(argv[3]).touch()
            return type('Result',(),dict(returncode=0,stderr=''))()
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(os.environ,dict(STAIR_EXECUTION_HOST='minipc',TRON_REMOTE_RECORD_TOOL='/fake/remote')), \
             patch('os.path.ismount',return_value=True),patch('subprocess.run',side_effect=remote_run), \
             patch('subprocess.Popen',side_effect=FakeProcess):
            recorder=ConsoleRecording(directory,directory);recorder.start();recorder.process.code=1
            snapshot=recorder.snapshot()
            self.assertTrue(snapshot['busy']);self.assertTrue(snapshot['raw_capture_unverified'])
            self.assertTrue(sealed.wait(1))
            # Polling and stop remain responsive while remote sealing is stalled.
            recorder.stop();self.assertEqual(recorder.snapshot()['state'],'stopping')
            release.set();self.wait_for_finalized(recorder)
            self.assertEqual(recorder.snapshot()['state'],'error')
            deadline=time.monotonic()+2
            while any(x['state']=='copying' for x in recorder.snapshot()['raw_copies']) and time.monotonic()<deadline:time.sleep(.01)

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

class RawSealScriptTest(unittest.TestCase):
    def run_seal(self, state='inactive', state_code=3, process_code=1,
                 saved=False, active_file=False):
        import shutil
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);bin_dir=root/'bin';bin_dir.mkdir()
            script=root/'remote_stair_record.sh'
            shutil.copyfile(Path(__file__).resolve().parents[3]/'remote_stair_record.sh',script)
            (root/'config.env').write_text('MINI_PC_USER=fixture\nMINI_PC_HOST=fixture\n')
            bag_base=root/'raw';(bag_base/'bags').mkdir(parents=True)
            if saved:(bag_base/'bags'/'fixture.bag').touch()
            if active_file:(bag_base/'bags'/'fixture.bag.active').touch()
            fake_ssh=bin_dir/'ssh'
            fake_ssh.write_text("#!/usr/bin/python3\nimport os,subprocess,sys\ncommand=sys.argv[-1].replace('/home/fixture/.local/share/tron1-stair-supervisor',os.environ['FIXTURE_BASE'])\nraise SystemExit(subprocess.run(['bash','-c',command]).returncode)\n")
            fake_systemctl=bin_dir/'systemctl'
            fake_systemctl.write_text("#!/usr/bin/python3\nimport os,sys\nif 'is-active' in sys.argv:\n print(os.environ['FIXTURE_STATE'])\n raise SystemExit(int(os.environ['FIXTURE_STATE_CODE']))\n")
            fake_pgrep=bin_dir/'pgrep'
            fake_pgrep.write_text("#!/usr/bin/python3\nimport os\nraise SystemExit(int(os.environ['FIXTURE_PROCESS_CODE']))\n")
            for command in (fake_ssh,fake_systemctl,fake_pgrep):command.chmod(0o700)
            env=dict(os.environ,PATH=str(bin_dir)+':'+os.environ['PATH'],FIXTURE_BASE=str(bag_base),
                     FIXTURE_STATE=state,FIXTURE_STATE_CODE=str(state_code),
                     FIXTURE_PROCESS_CODE=str(process_code))
            return subprocess.run(['bash',str(script),'seal','fixture'],env=env,
                                  capture_output=True,text=True,timeout=5)

    def test_absent_bag_is_explicit_only_when_unit_and_process_are_absent(self):
        result=self.run_seal(state='unknown',state_code=4)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(result.stdout.strip(),'RAW_BAG_ABSENT')

    def test_existing_sealed_bag_preserves_empty_success_output(self):
        result=self.run_seal(saved=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(result.stdout,'')

    def test_active_unit_cannot_claim_absent_bag(self):
        result=self.run_seal(state='active',state_code=0)
        self.assertNotEqual(result.returncode,0)
        self.assertNotIn('RAW_BAG_ABSENT',result.stdout)

    def test_lingering_process_cannot_claim_absent_bag(self):
        result=self.run_seal(process_code=0)
        self.assertNotEqual(result.returncode,0)
        self.assertNotIn('RAW_BAG_ABSENT',result.stdout)

    def test_active_bag_cannot_claim_absence_or_success(self):
        result=self.run_seal(active_file=True)
        self.assertNotEqual(result.returncode,0)
        self.assertNotIn('RAW_BAG_ABSENT',result.stdout)

    def test_unavailable_systemd_or_process_query_remains_unverified(self):
        self.assertNotEqual(self.run_seal(state='',state_code=1).returncode,0)
        self.assertNotEqual(self.run_seal(process_code=2).returncode,0)


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
