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
# One JPEG crosses Wi-Fi into PhotoCapture; record its local
# /mission_manager/rgb/compressed copy, without the retired tag relay.
CAMERA_DUPLICATES = (r"^/camera1/(?:depth|infra[12]?|infrared[12]?|aligned_depth_to_[^/]+)(?:/.*)?$|"
                     r"^/camera1/(?:.*(?:image_raw|image_rect_raw|points)$|"
                     r"color/image_raw/compressed$|color/image_raw/compressedDepth(?:/.*)?$)|^/apriltag_camera/image_raw(?:/compressed)?$")
REMOTE_RAW_TOPICS = r"^/livox/(?:lidar|imu)$"

def recorded_topic(topic, excluded_regex=CAMERA_DUPLICATES):
    return re.fullmatch(excluded_regex, topic) is None


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
        self.remote_raw = os.environ.get('STAIR_EXECUTION_HOST') == 'minipc'
        self.remote_tool = os.environ.get('TRON_REMOTE_RECORD_TOOL', '')
        self.excluded_topics_regex = CAMERA_DUPLICATES + ('|' + REMOTE_RAW_TOPICS if self.remote_raw else '')
        self.raw_name = None
        self.raw_path = None
        self.raw_copies = {}
        self.copy_lock = threading.Lock()
        self.raw_capture_unverified = False
        self.raw_pending = False

    def snapshot(self):
        with self.lock:
            if self.process and self.process.poll() is not None and not self.stopping and self.state in ('starting', 'recording'):
                self.state, self.error = 'error', '녹화 프로세스가 예기치 않게 종료됐습니다. 로그를 확인하세요.'
                if self.raw_pending:
                    self.stop()
            return dict(state=self.state, path=str(self.path or ''), error=self.error,
                        busy=self.stopping or bool(self.process and self.process.poll() is None),
                        log_path=str(self.path.with_suffix('.log')) if self.path else '',
                        camera_format='RGB JPEG only; depth/infrared and duplicate raw images excluded',
                        raw_bag_path=str(self.raw_path or ''),
                        raw_copies=[dict(item) for item in self.raw_copies.values()],
                        raw_capture_unverified=self.raw_capture_unverified,
                        excluded_topics_regex=self.excluded_topics_regex)

    def start(self, stop_event=None):
        with self.lock:
            if stop_event is not None and stop_event.is_set():
                raise ValueError('녹화 시작이 취소됐습니다.')
            if self.stopping or (self.process and self.process.poll() is None):
                raise ValueError('이전 녹화가 종료·저장될 때까지 기다리세요.')
            if self.remote_raw and self.raw_capture_unverified:
                raise ValueError('MiniPC 이전 원본 녹화 종료를 확인하지 못했습니다. 상태를 확인한 뒤 다시 녹화하세요.')
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
            self.process = None
            self.raw_name = name + '_raw' if self.remote_raw else None
            self.raw_path = self.root/(self.raw_name+'.bag') if self.raw_name else None
            if self.raw_name:
                if not self.remote_tool:
                    self.state = 'error'
                    raise ValueError('MiniPC 원본 센서 녹화 도구가 설정되지 않았습니다.')
                self.raw_pending = True
                try:
                    result = subprocess.run([self.remote_tool, 'start', self.raw_name],
                                            capture_output=True, text=True, timeout=25)
                except (OSError, subprocess.TimeoutExpired) as error:
                    self.state, self.error = 'error', str(error)
                    self.stop()
                    raise ValueError('MiniPC 원본 센서 녹화를 시작하지 못했습니다: '+str(error)) from error
                if result.returncode:
                    self.state, self.error = 'error', result.stderr.strip() or 'MiniPC 원본 센서 녹화 시작 실패'
                    error = self.error
                    self.stop()
                    raise ValueError(error)
            try:
                with self.path.with_suffix('.log').open('xb') as log:
                    self.process = subprocess.Popen(
                        [recorders[0], '--all', '--exclude', self.excluded_topics_regex, '-O', str(self.path), '__name:='+name],
                        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True)
            except OSError as error:
                self.process = None
                self.state, self.error = 'error', str(error)
                self.stop()
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
                    error = self.error
                    self.stop()
                    raise ValueError(error)
                file_ready = Path(str(path)+'.active').exists()
            # Opening a bag precedes subscribing. Wait for the recorder's
            # subscriptions too, so the initial mission goal is not sent early.
            subscribed = file_ready and self._subscriptions_ready('/'+name, self.excluded_topics_regex)
            with self.lock:
                if process.poll() is not None:
                    self.state, self.error = 'error', '녹화 프로세스가 준비 중 종료됐습니다.'
                    error = self.error
                    self.stop()
                    raise ValueError(error)
                if subscribed and not self.stopping and not (stop_event and stop_event.is_set()):
                    self.state = 'recording'
                    return self.snapshot()
            time.sleep(.05)
        self.stop()
        raise ValueError('녹화 파일 준비 시간이 초과되어 주행을 시작하지 않았습니다.')

    @staticmethod
    def _subscriptions_ready(node, excluded_regex=CAMERA_DUPLICATES):
        try:
            with xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311'), transport=_MasterTransport()) as master:
                code, _, state = master.getSystemState('/ui_recording_readiness')
            if code != 1:
                return False
            published = {topic for topic,nodes in state[0] if nodes and recorded_topic(topic, excluded_regex)}
            subscribed = {topic for topic,nodes in state[1] if node in nodes}
            return published <= subscribed
        except (OSError, xmlrpc.client.Error):
            return False

    def stop(self):
        with self.lock:
            if self.stopping:
                return self.snapshot()
            process, path = self.process, self.path
            if not self.raw_pending and (process is None or
                    process.poll() is not None and self.state in ('saved', 'error')):
                return self.snapshot()
            raw_name = self.raw_name if self.raw_pending else None
            raw_path = self.raw_path
            self.stopping, self.state = True, 'stopping'
            if raw_name:
                self.raw_capture_unverified = True
            if process is not None and process.poll() is None:
                try:
                    process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
            threading.Thread(target=self._finish, args=(process,path,raw_name,raw_path), daemon=True,
                             name='ui-bag-finalize').start()
            return self.snapshot()

    def _finish(self, process, path, raw_name=None, raw_path=None):
        # No SIGKILL: large files may need time to flush their final index.
        code = process.wait() if process is not None else None
        local_saved = code == 0 and path is not None and path.exists() and not Path(str(path)+'.active').exists()
        seal_error = ''
        raw_absent = False
        if raw_name:
            with self.lock:
                self.raw_copies[raw_name] = dict(path=str(raw_path), state='sealing', error='')
            try:
                result = subprocess.run([self.remote_tool, 'seal', raw_name],
                                        capture_output=True, text=True, timeout=90)
                if result.returncode:
                    seal_error = result.stderr.strip() or 'MiniPC 원본 센서 BAG 종료 실패'
                else:
                    raw_absent = getattr(result, 'stdout', '').strip() == 'RAW_BAG_ABSENT'
            except (OSError, subprocess.TimeoutExpired) as error:
                seal_error = str(error)
        with self.lock:
            if self.process is not process:
                return
            self.stopping = False
            if local_saved:
                self.state, self.error = 'saved', ''
            else:
                self.state = 'error'
                self.error = '정상 저장을 확인하지 못했습니다. .active와 로그를 보존했습니다.'
            if raw_name:
                self.raw_copies[raw_name].update(
                    state='error' if seal_error else 'not_recorded' if raw_absent else 'copying', error=seal_error)
                self.raw_capture_unverified = bool(seal_error)
                self.raw_pending = bool(seal_error)
                # Preserve the transfer result across later recording starts.
                # Copying a large raw BAG must never retain mission ownership.
                if not seal_error and not raw_absent:
                    threading.Thread(target=self._copy_raw, args=(raw_name, raw_path),
                                     daemon=True, name='ui-raw-bag-copy').start()

    def _copy_raw(self, raw_name, raw_path):
        error = ''
        with self.copy_lock:
            try:
                result = subprocess.run([self.remote_tool, 'copy', raw_name, str(raw_path)],
                                        capture_output=True, text=True)
                if result.returncode or not raw_path.exists():
                    error = result.stderr.strip() or 'MiniPC 원본 센서 BAG 복사 실패'
            except OSError as exc:
                error = str(exc)
        with self.lock:
            self.raw_copies[raw_name].update(state='error' if error else 'saved', error=error)
