"""External door requests owned by the UI server; no robot or ROS commands."""
from collections import deque
import importlib
import os
from pathlib import Path
import sys
import threading
import time

HOLD_SECONDS = {'F5': 30, 'RF': 60}
SDK_ROOT = Path(os.environ.get('TRON1_DOOR_SDK_ROOT',
                               str(Path.home() / '.local/share/tron1/door-sdk')))
_sdk_lock = threading.Lock()


def _sdk():
    """Load the provided package from private storage, outside Git."""
    path = SDK_ROOT / 'robot/door_control.py'
    if not path.is_file():
        raise RuntimeError('door SDK is not installed in the private runtime path')
    with _sdk_lock:
        if str(SDK_ROOT) not in sys.path:
            sys.path.insert(0, str(SDK_ROOT))
        module = importlib.import_module('robot.door_control')
        if Path(module.__file__).resolve() != path.resolve():
            raise RuntimeError('a different robot.door_control package was imported')
        return module


def request_door(device, seconds):
    if device not in HOLD_SECONDS or type(seconds) is not int or not 1 <= seconds <= 600:
        raise ValueError('door device or hold duration is invalid')
    module = _sdk()
    try:
        return (module.RF_Open if device == 'RF' else module.F5_Open)(seconds)
    except ConnectionError:
        module.close()  # Prepare only the next explicit command, never retry here.
        raise


class ConsoleDoor:
    def __init__(self, read_state, sender=request_door, clock=time.monotonic, threaded=True):
        self.read_state, self.sender, self.clock = read_state, sender, clock
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.wake = threading.Event()
        self.active = False
        self.mode = ''
        self.run_id = ''
        self.completed = deque(maxlen=128)
        self.holds = {}
        self.inflight = set()
        self.attempts = 0
        self.last_at = None
        self.last_device = ''
        self.last_hold_sec = None
        self.last_command_id = ''
        self.error = ''
        self.worker_error = ''
        self.reason = '문 개방 명령 대기'
        self.one_shot = dict(busy=False, state="IDLE", detail="수동 문 열기 대기")
        self.thread = None
        if threaded:
            self.thread = threading.Thread(target=self._loop, name='ui-door-request', daemon=True)
            self.thread.start()

    @staticmethod
    def _fresh(state, key):
        telemetry = state.get('telemetry') if isinstance(state, dict) else None
        row = telemetry.get(key) if isinstance(telemetry, dict) else None
        if not isinstance(row, dict):
            return {}
        age = row.get('age_sec')
        value = row.get('value', {})
        return value if isinstance(value, dict) and isinstance(age, (int, float)) and 0 <= age <= 2 else {}

    def manual_open(self, data):
        state = self.read_state()
        if not isinstance(state, dict) or state.get('mode') != 'live':
            raise ValueError('실제 운용 화면에서 문 열기를 사용하세요.')
        device = data.get('device')
        if device not in HOLD_SECONDS:
            raise ValueError('열 문을 5층 또는 옥상으로 선택하세요.')
        with self.lock:
            if self.closed.is_set():
                raise ValueError('UI 서버 종료 중')
            if self.one_shot['busy']:
                raise ValueError('문 열기 요청 처리 중')
            if device in self.inflight:
                raise ValueError('해당 문의 개방 응답을 기다리는 중입니다.')
            self.inflight.add(device)
            self.one_shot = dict(busy=True, state='SENDING', device=device, detail='장치 접수 대기')
            threading.Thread(target=self._manual_open, args=(device,), name='ui-door-once', daemon=True).start()
        return dict(accepted=True, note=device + ' 문 개방 요청을 시작했습니다. ACK 상태를 확인하세요.')

    def _manual_open(self, device):
        try:
            command_id = self.sender(device, HOLD_SECONDS[device])
            if not isinstance(command_id, str) or not command_id:
                raise ValueError('door command ID missing from device ACK')
            with self.lock:
                self.holds[device] = self.clock() + HOLD_SECONDS[device]
                self.last_command_id, self.error = command_id, ''
                self.one_shot = dict(busy=False, state='ACCEPTED', device=device,
                                     command_id=command_id,
                                     detail='장치 접수 완료 · 실제 문 위치는 확인되지 않음')
        except Exception as exc:
            with self.lock:
                self.one_shot = dict(busy=False, state='UNKNOWN', device=device,
                                     detail='접수 불명/실패: ' + type(exc).__name__ + ' · 실제 문을 확인하세요')
        finally:
            with self.lock:
                self.inflight.discard(device)
                self.attempts += 1
                self.last_at, self.last_device = self.clock(), device
                self.last_hold_sec = HOLD_SECONDS[device]
                self.active = any(until > self.clock() for until in self.holds.values())

    def manual_return(self, _data):
        state = self.read_state()
        if not isinstance(state, dict) or state.get('mode') != 'live' or self._fresh(state, 'floor').get('floor_id') != 'RF':
            raise ValueError('확인된 현재 층이 RF일 때 복귀 문 열기를 사용하세요.')
        return self.manual_open({'device': 'RF'})

    def stop(self, _data=None):
        return dict(note='장치가 접수한 개방 시간은 UI에서 취소할 수 없습니다. 자동 재요청은 없습니다.')

    def _events(self, state):
        if not isinstance(state, dict) or state.get('mode') != 'live':
            return []
        events = []
        progress = state.get('progress') if state.get('active') or state.get('observed_active') else None
        if (isinstance(progress, dict) and progress.get('mission_id')
                and progress.get('current_floor') == '5F'
                and progress.get('segment_type') == 'NAVIGATION'
                and progress.get('target_id') == 'stair_5f_to_rf'):
            events.append((progress['mission_id'] + ':f5_depart', 'F5', '5층 NAV 시작', 'up'))
        control, supervisor = self._fresh(state, 'control'), self._fresh(state, 'supervisor')
        phase_test = control.get('phase_test')
        testing = isinstance(phase_test, dict) and phase_test.get('state') == 'RUNNING'
        moving = state.get('active') or state.get('observed_active') or testing
        if not (moving and control.get('run_id') and supervisor.get('connected')
                and supervisor.get('state') == 2 and not control.get('loss_response')):
            return events
        run_id, phase, route = control['run_id'], control.get('phase'), control.get('route_id')
        if route == 'stair_5f_rf_up' and phase == 'ROOFTOP_TURN':
            events.append((run_id + ':rf_up', 'RF', '2차 상승 후 옥상 문', 'up'))
        if route == 'stair_5f_rf_down':
            if phase in ('VERIFY_ENTRY', 'ALIGN', 'FORWARD_SEGMENT_1'):
                events.append((run_id + ':rf_return', 'RF', '복귀 계단 진입 전 옥상 문', 'return'))
            if phase == 'FORWARD_SEGMENT_3':
                events.append((run_id + ':f5_return', 'F5', '5층 복귀 직전 문', 'return'))
        return events

    def _send(self, identity, device, reason, mode):
        with self.lock:
            if self.closed.is_set() or identity in self.completed:
                return
            self.completed.append(identity)  # Uncertain ACK must never auto-retry.
            now = self.clock()
            if device in self.inflight or self.holds.get(device, 0.) > now:
                self.reason = reason + ' · 이미 개방 요청 중이거나 유지 요청됨'
                return
            self.inflight.add(device)
            self.attempts += 1
            self.last_at, self.last_device = now, device
            self.last_hold_sec = HOLD_SECONDS[device]
            self.mode, self.run_id = mode, identity
            self.reason = reason + ' · 장치 접수 대기'
        try:
            command_id = self.sender(device, HOLD_SECONDS[device])
            if not isinstance(command_id, str) or not command_id:
                raise ValueError('door command ID missing from device ACK')
        except Exception as exc:
            with self.lock:
                self.error = type(exc).__name__ + ': ' + str(exc)
                self.reason = reason + ' · 접수 불명/실패 · 자동 재요청 없음'
        else:
            with self.lock:
                self.last_command_id, self.error = command_id, ''
                self.holds[device] = self.clock() + HOLD_SECONDS[device]
                self.reason = reason + ' · 장치 접수 완료 (문 위치 미확인)'
        finally:
            with self.lock:
                self.inflight.discard(device)

    def tick(self):
        try:
            state = self.read_state()
        except Exception:
            state = {}
        for event in self._events(state):
            self._send(*event)
        with self.lock:
            self.active = any(until > self.clock() for until in self.holds.values())

    def snapshot(self):
        with self.lock:
            now = self.clock()
            remaining = {device: max(0., until-now) for device, until in self.holds.items() if until > now}
            return dict(one_shot=dict(self.one_shot), active=bool(remaining), mode=self.mode,
                        reason=self.reason, attempts=self.attempts, run_id=self.run_id,
                        server_owned=True, last_device=self.last_device,
                        last_hold_sec=self.last_hold_sec, last_command_id=self.last_command_id,
                        error=self.error, worker_error=self.worker_error,
                        hold_remaining_sec=remaining, physical_door_verified=False)

    def _loop(self):
        while not self.closed.is_set():
            try:
                self.tick()
                with self.lock:
                    self.worker_error = ''
            except Exception as exc:
                with self.lock:
                    self.worker_error = type(exc).__name__+' · 상태 확인 재시도'
            self.wake.wait(.5)
            self.wake.clear()

    def close(self):
        self.closed.set()
        self.wake.set()
        if self.thread:
            self.thread.join(timeout=1.)
        if not self.thread or not self.thread.is_alive():
            module = sys.modules.get('robot.door_control')
            if module is not None and hasattr(module, 'close'):
                module.close()
