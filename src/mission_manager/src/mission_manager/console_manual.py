"""Manual input only: never cancel, pause, or replace an automatic mission."""
import json
import math
import threading
import time

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ConsoleManual:
    def __init__(self, console):
        self.console = console
        self.lock = threading.RLock()
        self.lease = None
        self.ended = True
        self.sequence = -1
        self.expires = 0.
        self.report = {}
        self.received = 0.
        self.generation = 0
        self.pending = None
        self.begin_wait_sec = 2.0
        self.publisher = rospy.Publisher('/stair_supervisor/web_manual', String, queue_size=1, tcp_nodelay=True)
        self.subscriber = rospy.Subscriber('/stair_supervisor/web_manual_status', String, self.observe, queue_size=1)

    def observe(self, message):
        try:
            report = json.loads(message.data)
            if not isinstance(report, dict):return
            with self.lock:
                self.report, self.received = report, time.monotonic()
        except ValueError:
            pass

    def snapshot(self):
        with self.lock:
            fresh = time.monotonic()-self.received < 1.5
            return dict(available=fresh, pending=self.pending is not None, active=(not self.ended and time.monotonic()<self.expires) or
                        (fresh and bool(self.report.get('active'))), policy='manual_priority_keep_mission', timeout_sec=.35)

    def begin(self, _data):
        with self.console.lock, self.lock:
            if self.console.closed:
                raise ValueError('콘솔이 종료 중입니다.')
            if self.pending is not None:
                raise ValueError('이전 수동 입력 요청의 응답을 확인 중입니다.')
            if self.lease and not self.ended and time.monotonic()<self.expires:
                raise ValueError('다른 조이스틱 입력이 진행 중입니다.')
            if not self.publisher.get_num_connections():
                raise ValueError('수동 입력 경로 연결을 기다리는 중입니다.')
            self.generation += 1
            request = dict(generation=self.generation, done=threading.Event(), error=None, lease=None)
            self.pending = request
        worker = threading.Thread(target=self._begin_request, args=(request,), daemon=True,
                                  name='manual-lease-request')
        try:
            worker.start()
        except Exception:
            with self.lock:
                if self.pending is request:self.pending = None
            raise
        if not request['done'].wait(self.begin_wait_sec):
            with self.lock:
                if request['generation'] == self.generation:
                    self.generation += 1
                    if request['lease'] is not None and self.lease == request['lease']:
                        self.end(dict(lease=request['lease']))
            # The worker retains the pending slot until its unknown remote
            # request replies; Stop and automatic mission processing stay free.
            raise ValueError('수동 입력 응답이 지연됐습니다. 자동 임무와 정지는 계속 사용할 수 있습니다.')
        with self.lock:
            if request['error'] is not None:raise ValueError(str(request['error']))
            if request['generation'] != self.generation:
                raise ValueError('수동 입력 시작이 취소됐습니다.')
            return dict(lease=request['lease'], server_time=time.time(), note='수동 우선 송신 · 자동 임무 유지')

    def _begin_request(self, request):
        lease = None
        try:
            name = '/stair_supervisor/begin_web_manual'
            rospy.wait_for_service(name, timeout=.5)
            response = rospy.ServiceProxy(name, Trigger)()
            if not response.success:raise ValueError(response.message)
            lease = json.loads(response.message)['lease']
            if not isinstance(lease, str) or not lease:raise ValueError('잘못된 수동 입력 응답')
            with self.lock:
                current = request['generation'] == self.generation and not self.console.closed
                if current:
                    self.lease, self.ended, self.sequence = lease, False, -1
                    self.expires = time.monotonic()+.35
                    request['lease'] = lease
            if not current:
                self.publisher.publish(String(data=json.dumps(dict(kind='end', lease=lease))))
        except Exception as error:
            request['error'] = error
        finally:
            with self.lock:
                if self.pending is request:self.pending = None
                request['done'].set()

    def update(self, data):
        with self.lock:
            now = time.monotonic()
            if data.get('lease') != self.lease or self.ended or now>=self.expires:
                raise ValueError('수동 입력이 만료됐습니다. 놓고 다시 잡으세요.')
            sequence, forward, turn, issued = (data.get(k) for k in ('sequence','forward','turn','issued_at'))
            if type(sequence) is not int or sequence<=self.sequence:
                raise ValueError('이전 입력 거부')
            if any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1 for v in (forward,turn)):
                raise ValueError('잘못된 조이스틱 값')
            if type(issued) not in (int,float) or not math.isfinite(issued) or not 0<=time.time()-issued<.35:
                raise ValueError('지연된 수동 입력')
            self.sequence, self.expires = sequence, now+.35
            self.publisher.publish(String(data=json.dumps(dict(kind='update', lease=self.lease, sequence=sequence,
                                                               forward=forward, turn=turn, issued_at=issued))))
            return dict(accepted=True)

    def end(self, data):
        with self.lock:
            if data.get('lease') != self.lease:
                raise ValueError('수동 제어 세션 불일치')
            self.ended = True
            self.publisher.publish(String(data=json.dumps(dict(kind='end', lease=self.lease))))
            return dict(accepted=True, note='수동 입력 종료 · 자동 명령은 계속 전송')

    def stop(self):
        with self.lock:
            self.generation += 1
            if self.lease is not None:self.end(dict(lease=self.lease))

    def close(self):
        self.stop()
        self.subscriber.unregister()
        self.publisher.unregister()
