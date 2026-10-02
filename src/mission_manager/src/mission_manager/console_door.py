"""External door requests owned by the UI server; no robot or ROS commands."""
from collections import deque
import math
import threading
import time
import urllib.request
import urllib.error

DOOR_URL = 'https://dooropen.declankim.kr/trigger'
PERIOD_SEC = 20.


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *_args, **_kwargs):
        return None


def request_door():
    request = urllib.request.Request(DOOR_URL, headers={
        'Cache-Control': 'no-cache, no-store', 'Pragma': 'no-cache',
        'User-Agent': 'TRON1-UI-Door/1'}, method='GET')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
    with opener.open(request, timeout=5.) as response:
        return response.status


class ConsoleDoor:
    def __init__(self, read_state, sender=request_door, clock=time.monotonic, threaded=True):
        self.read_state, self.sender, self.clock = read_state, sender, clock
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.wake = threading.Event()
        self.active = False
        self.mode = ''
        self.run_id = ''
        self.completed = deque(maxlen=32)
        self.generation = 0
        self.next_at = 0.
        self.attempts = 0
        self.last_at = None
        self.code = None
        self.error = ''
        self.worker_error = ''
        self.reason = '옥상 우측 회전 대기'
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

    def _start(self, mode, run_id='', reason=''):
        self.active, self.mode, self.run_id = True, mode, run_id
        self.generation += 1
        self.next_at = self.clock()
        self.error = ''
        self.reason = reason

    def manual_open(self, _data):
        if self.read_state().get('mode') != 'live':
            raise ValueError('실제 운용 화면에서 문 열기를 사용하세요.')
        with self.lock:
            if self.closed.is_set():raise ValueError('UI 서버 종료 중')
            if self.one_shot['busy']:raise ValueError('문 열기 요청 처리 중')
            self.one_shot=dict(busy=True,state='SENDING',detail='문 열기 요청 중')
            try:threading.Thread(target=self._manual_open,name='ui-door-once',daemon=True).start()
            except Exception:
                self.one_shot=dict(busy=False,state='FAILED',detail='문 열기 작업 시작 실패')
                raise
        return dict(accepted=True,note='문 열기를 한 번 요청했습니다.')

    def _manual_open(self):
        code=None
        try:
            code=self.sender()
            if not 200 <= code < 300:raise ValueError('HTTP '+str(code))
            state,detail='SENT','문 열기 서버 접수 완료 · 장치 기준 60초 개방 · 실제 문 상태는 확인하세요.'
        except urllib.error.HTTPError as error:
            code=error.code
            state,detail='FAILED','문 열기 요청 실패: HTTP '+str(code)
        except Exception as error:
            state,detail='FAILED','문 열기 요청 실패: '+str(error)
        with self.lock:
            self.one_shot=dict(busy=False,state=state,detail=detail,http_status=code)

    def manual_return(self, _data):
        state = self.read_state()
        if not isinstance(state, dict) or state.get('mode') != 'live' or self._fresh(state, 'floor').get('floor_id') != 'RF':
            raise ValueError('확인된 현재 층이 RF일 때 복귀 문 열기를 사용하세요.')
        with self.lock:
            if self.closed.is_set():
                raise ValueError('UI 서버가 종료 중입니다.')
            if not self.active or self.mode != 'return':
                if self.run_id:
                    self.completed.append(self.run_id)
                self._start('return', reason='복귀 문 개방 유지 · 주행 명령은 별도')
        self.wake.set()
        return dict(note='복귀 문 열기를 시작했습니다. 문 통과 완료 시 반복 종료를 누르세요. 주행은 시작하지 않습니다.')

    def stop(self, _data=None, reason='사용자가 문 통과 확인 · 반복 종료'):
        with self.lock:
            if self.run_id:
                self.completed.append(self.run_id)
            self.active = False
            self.generation += 1
            self.reason = reason
        self.wake.set()
        return dict(note='반복 요청을 종료했습니다. 문은 마지막 요청 후 장치의 60초 유지 규칙을 따릅니다.')

    def tick(self):
        with self.lock:
            observed_generation = self.generation
        try:
            state = self.read_state()
        except Exception:
            state = {}  # Already-open passage retains its request schedule.
        if not isinstance(state, dict):
            state = {}
        control = self._fresh(state, 'control')
        floor = self._fresh(state, 'floor')
        supervisor = self._fresh(state, 'supervisor')
        live = state.get('mode') == 'live'
        phase_test = control.get('phase_test')
        testing = isinstance(phase_test, dict) and phase_test.get('state') == 'RUNNING'
        moving = state.get('active') or state.get('observed_active') or testing
        target = control.get('target', [])
        roof_turn = (control.get('route_id') == 'stair_5f_rf_up'
                     and control.get('phase') == 'ROOFTOP_TURN'
                     and isinstance(target, list) and len(target) == 4
                     and isinstance(target[3], (int, float))
                     and math.isclose(target[3], math.pi/2, abs_tol=.01))
        with self.lock:
            if self.closed.is_set() or observed_generation != self.generation:
                return  # Snapshot predates an explicit start/stop; re-read next tick.
            run_id = control.get('run_id', '')
            if (not self.active and live and moving and run_id and run_id not in self.completed
                    and supervisor.get('connected') and supervisor.get('state') == 2
                    and not control.get('loss_response')):
                if roof_turn:
                    self._start('up', run_id, '옥상 우측 회전 · 문 개방 유지')
                elif control.get('route_id') == 'stair_5f_rf_down' and floor.get('floor_id') == 'RF':
                    self._start('return', run_id, '옥상 복귀 · 문 개방 유지')
            if (self.active and self.mode == 'return' and not self.run_id and run_id
                    and control.get('route_id') == 'stair_5f_rf_down'):
                self.run_id = run_id  # Bind a manual schedule before explicit passage stop.
            if self.active:
                checks = control.get('completion_checks')
                if not isinstance(checks, dict):
                    checks = {}
                passed = (self.mode == 'up' and run_id == self.run_id
                          and control.get('phase') == 'EXIT_CONFIRM' and control.get('exit_supported') is True
                          and control.get('phase_complete') is True and checks.get('final_height') is True
                          and checks.get('all_flights_seen') is True)
                roof_arrived = (self.mode == 'up' and floor.get('floor_id') == 'RF'
                                and floor.get('state') == 2)
                return_done = self.mode == 'return' and floor.get('floor_id') == '5F' and floor.get('state') == 2
                if passed or roof_arrived or return_done:
                    self.stop(reason='옥상 도착 영역 확인 · 반복 종료' if passed or roof_arrived else '5층 도착 확인 · 반복 종료')
            now = self.clock()
            if not self.active or now < self.next_at:
                return
            generation = self.generation
            self.next_at = now + PERIOD_SEC
            self.attempts += 1
            self.last_at = now
        # Never hold the UI lock or a robot/control lock during external I/O.
        try:
            code = self.sender()
            error = '' if 200 <= code < 300 else 'HTTP 응답 %s · 문 상태 미확인' % code
        except urllib.error.HTTPError as exc:
            code, error = exc.code, 'HTTP 응답 %s · 문 상태 미확인' % exc.code
        except Exception as exc:
            code, error = None, '%s · 문 상태 미확인' % type(exc).__name__
        with self.lock:
            if generation == self.generation:
                self.code, self.error = code, error

    def snapshot(self):
        with self.lock:
            return dict(one_shot=dict(self.one_shot),active=self.active, mode=self.mode, reason=self.reason, attempts=self.attempts,
                        run_id=self.run_id, server_owned=True,
                        last_http_status=self.code, error=self.error, worker_error=self.worker_error,
                        seconds_to_next=max(0., self.next_at-self.clock()) if self.active else None,
                        last_request_age_sec=None if self.last_at is None else self.clock()-self.last_at,
                        period_sec=PERIOD_SEC, hold_sec=60,
                        physical_door_verified=False)

    def _loop(self):
        while not self.closed.is_set():
            try:
                self.tick()
                with self.lock:
                    self.worker_error = ''
            except Exception as exc:
                with self.lock:
                    self.worker_error = type(exc).__name__+' · 반복 작업 재시도'
            self.wake.wait(.5)
            self.wake.clear()

    def close(self):
        self.closed.set()
        self.stop(reason='UI 서버 종료 · 반복 종료')
        if self.thread:
            self.thread.join(timeout=1.)
