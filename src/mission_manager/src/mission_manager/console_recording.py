"""UI-owned unsplit recording; camera uses compressed transports across Wi-Fi."""
import os
import re
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import xmlrpc.client


# rosbag uses whole-topic boost::regex_match; keep the helper full-match too.
# Record RGB JPEG and control/sensor evidence. Depth/infrared streams and
# their transports are intentionally omitted, including aligned depth.
# One JPEG crosses Wi-Fi; the local relay is shared by capture and recording.
CAMERA_DUPLICATES = (r"^/camera1/(?:depth|infra[12]?|infrared[12]?|aligned_depth_to_[^/]+)(?:/.*)?$|"
                     r"^/camera1/(?:.*(?:image_raw|image_rect_raw|points)$|"
                     r"color/image_raw/compressed$|color/image_raw/compressedDepth(?:/.*)?$)|^/apriltag_camera/image_raw$")

def recorded_topic(topic):
    return re.fullmatch(CAMERA_DUPLICATES, topic) is None


class _MasterTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


class ConsoleRecording:
    def __init__(self, root='/mnt/ethan/tron-bags/live/ui-tests', mount='/mnt/ethan'):
        self.root, self.mount = Path(root), mount
        self.lock = threading.RLock()
        self.process = None
        self.state = 'idle'
        self.path = None
        self.error = ''
        self.stopping = False

    def snapshot(self):
        with self.lock:
            if self.process and self.process.poll() is not None and not self.stopping and self.state in ('starting', 'recording'):
                self.state, self.error = 'error', '녹화 프로세스가 예기치 않게 종료됐습니다. 로그를 확인하세요.'
            return dict(state=self.state, path=str(self.path or ''), error=self.error,
                        busy=self.stopping or bool(self.process and self.process.poll() is None),
                        log_path=str(self.path.with_suffix('.log')) if self.path else '',
                        camera_format='RGB JPEG only; depth/infrared and duplicate raw images excluded',
                        excluded_topics_regex=CAMERA_DUPLICATES)

    def start(self, stop_event=None):
        with self.lock:
            if stop_event is not None and stop_event.is_set():
                raise ValueError('녹화 시작이 취소됐습니다.')
            if self.stopping or (self.process and self.process.poll() is None):
                raise ValueError('이전 녹화가 종료·저장될 때까지 기다리세요.')
            if not os.path.ismount(self.mount):
                raise ValueError('1TB 녹화 디스크가 마운트되지 않았습니다.')
            try:
                self.root.resolve().relative_to(Path(self.mount).resolve())
            except ValueError:
                raise ValueError('녹화 경로는 지정된 디스크 안에 있어야 합니다.')
            self.root.mkdir(parents=True, exist_ok=True)
            from roslib.packages import find_node
            recorders = find_node('rosbag', 'record')
            if not recorders:
                raise ValueError('ROS rosbag record 실행 파일을 찾지 못했습니다.')
            name = 'ui_test_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
            self.path = self.root/(name+'.bag')
            self.error, self.state = '', 'starting'
            with self.path.with_suffix('.log').open('xb') as log:
                try:
                    self.process = subprocess.Popen(
                        [recorders[0], '--all', '--exclude', CAMERA_DUPLICATES, '-O', str(self.path), '__name:='+name],
                        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True)
                except OSError as error:
                    self.process = None
                    self.state, self.error = 'error', str(error)
                    raise ValueError('rosbag을 시작하지 못했습니다: '+str(error)) from error
            process, path = self.process, self.path
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            with self.lock:
                if stop_event is not None and stop_event.is_set():
                    self.stop()
                    raise ValueError('녹화 시작이 취소됐습니다.')
                if self.stopping or self.process is not process:
                    raise ValueError('녹화 시작이 취소됐습니다.')
                if process.poll() is not None:
                    self.state, self.error = 'error', '녹화를 시작하지 못했습니다. 녹화 로그를 확인하세요.'
                    raise ValueError(self.error)
                file_ready = Path(str(path)+'.active').exists()
            # Opening a bag precedes subscribing. Wait for the recorder's
            # subscriptions too, so the initial mission goal is not sent early.
            subscribed = file_ready and self._subscriptions_ready('/'+name)
            with self.lock:
                if process.poll() is not None:
                    self.state, self.error = 'error', '녹화 프로세스가 준비 중 종료됐습니다.'
                    raise ValueError(self.error)
                if subscribed and not self.stopping and not (stop_event and stop_event.is_set()):
                    self.state = 'recording'
                    return self.snapshot()
            time.sleep(.05)
        self.stop()
        raise ValueError('녹화 파일 준비 시간이 초과되어 주행을 시작하지 않았습니다.')

    @staticmethod
    def _subscriptions_ready(node):
        try:
            with xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311'), transport=_MasterTransport()) as master:
                code, _, state = master.getSystemState('/ui_recording_readiness')
            if code != 1:
                return False
            published = {topic for topic,nodes in state[0] if nodes and recorded_topic(topic)}
            subscribed = {topic for topic,nodes in state[1] if node in nodes}
            return published <= subscribed
        except (OSError, xmlrpc.client.Error):
            return False

    def stop(self):
        with self.lock:
            if self.stopping or self.process is None:
                return self.snapshot()
            if self.process.poll() is not None and self.state in ('saved', 'error'):
                return self.snapshot()
            process, path = self.process, self.path
            self.stopping, self.state = True, 'stopping'
            if process.poll() is None:
                try:
                    process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
            threading.Thread(target=self._finish, args=(process,path), daemon=True,
                             name='ui-bag-finalize').start()
            return self.snapshot()

    def _finish(self, process, path):
        # No SIGKILL: large files may need time to flush their final index.
        code = process.wait()
        with self.lock:
            if self.process is not process:
                return
            self.stopping = False
            if code == 0 and path.exists() and not Path(str(path)+'.active').exists():
                self.state, self.error = 'saved', ''
            else:
                self.state = 'error'
                self.error = '정상 저장을 확인하지 못했습니다. .active와 로그를 보존했습니다.'
