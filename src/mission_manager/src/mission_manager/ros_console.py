"""ROS adapter inside the existing node; missions use /mission; manual input uses the existing supervisor."""
import json
import threading
import time
from collections import deque

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from stair_supervisor.msg import SupervisorState
from multifloor_manager.msg import FloorState
from mission_manager.msg import MissionAction, MissionGoal, MissionActionFeedback, MissionActionResult
from mission_manager.mission_console import ConsoleServer
from mission_manager.route_planner import RoutePlanningError
from mission_manager.console_recording import ConsoleRecording
from mission_manager.console_map import ConsoleMap
from mission_manager.console_manual import ConsoleManual


def primitive(message):
    return {name: getattr(message, name) for name in message.__slots__
            if isinstance(getattr(message, name), (str, bool, int, float))}


class RosConsole:
    def __init__(self, planner, orchestrator, configuration, action_name, html_path, port, tag_topic="/tag_detections"):
        self.lock = threading.RLock()
        self.operator_condition = threading.Condition(self.lock)
        self.operator_pending = False
        self.operator_publisher = rospy.Publisher("/stair_supervisor/operator_phase_request", String, queue_size=1)
        self.planner, self.orchestrator = planner, orchestrator
        self.locations = [dict(id=x.id, floor=x.floor_id, x=x.x, y=x.y, yaw=x.yaw) for x in configuration.locations]
        self.telemetry = {}
        self.events = deque(maxlen=80)
        self.active = False
        self.result = None
        self.progress = {}
        self.recorder = ConsoleRecording()
        self.starting = False
        self.stop_requested = False
        self.generation = 0
        self.closed = False
        self.floor_selecting = False
        self.recovering = False
        self.stop_event = threading.Event()
        self.map_view = ConsoleMap()
        self.client = actionlib.SimpleActionClient(action_name, MissionAction)
        self.subscribers = [
            rospy.Subscriber(tag_topic, AprilTagDetectionArray,
                             lambda m: self.observe('apriltag', {'detections': len(m.detections),
                                 'stamp_age_sec': rospy.Time.now().to_sec()-m.header.stamp.to_sec()}), queue_size=1),
            rospy.Subscriber('/multifloor/floor_state', FloorState, lambda m: self.observe('floor', primitive(m)), queue_size=1),
            rospy.Subscriber('/stair_supervisor/operator_phase_status', String, lambda m: self.json_observe('operator_phase', m), queue_size=1),
            rospy.Subscriber('/multifloor/localization_status', String, lambda m: self.json_observe('localization', m), queue_size=1),
            rospy.Subscriber('/stair_supervisor/state', SupervisorState, lambda m: self.observe('supervisor', primitive(m)), queue_size=1),
            rospy.Subscriber('/stair_supervisor/tracking_status', String, lambda m: self.json_observe('tracking', m), queue_size=1),
            rospy.Subscriber('/stair_supervisor/control_debug', String, lambda m: self.json_observe('control', m), queue_size=1),
            rospy.Subscriber(action_name+'/feedback', MissionActionFeedback, lambda m: self.observe('mission_feedback', primitive(m.feedback)), queue_size=1),
            rospy.Subscriber(action_name+'/result', MissionActionResult, lambda m: self.observe('mission_result', primitive(m.result)), queue_size=1),
            rospy.Subscriber(action_name+'/status', GoalStatusArray, lambda m: self.observe('mission_status', {'active':any(s.status in (0,1,6,7) for s in m.status_list)}), queue_size=1)]
        self.manual = ConsoleManual(self)
        try:
            self.http = ConsoleServer(self, html_path, port)
        except Exception:
            for subscriber in self.subscribers:
                subscriber.unregister()
            self.map_view.close()
            self.manual.close()
            self.operator_publisher.unregister()
            raise
        rospy.on_shutdown(self.close)
        rospy.loginfo('Mission console http://127.0.0.1:%d', self.http.http.server_port)

    def observe(self, key, value):
        with self.lock:
            previous = self.telemetry.get(key, (None, {}))[1]
            if key == 'mission_result' or (key in ('floor', 'supervisor') and value.get('detail') != previous.get('detail')):
                self.events.append(dict(time=time.time(), kind=key, detail=value.get('detail', value.get('reason', ''))))
            self.telemetry[key] = (time.monotonic(), value)
            if key == "operator_phase" and hasattr(self,"operator_condition"):
                self.operator_condition.notify_all()

    def json_observe(self, key, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self.observe(key, value)
        except (ValueError, TypeError):
            pass

    def snapshot(self):
        with self.lock:
            received, status = self.telemetry.get('mission_status', (0, {}))
            observed_active = time.monotonic()-received <= 2 and status.get('active', False)
            hold=getattr(self.orchestrator,'arrival_hold',None)
            hold_state=dict(armed=bool(hold is not None and hold.core.armed),detail=hold.core.detail if hold is not None else 'not configured')
            photo_manager=getattr(self.orchestrator,'photo',None)
            mail_state=photo_manager.mail_snapshot() if photo_manager is not None and hasattr(photo_manager,'mail_snapshot') else {}
            return dict(mail=mail_state,manual=self.manual.snapshot() if getattr(self,'manual',None) else {},hold=hold_state,mode='live', locations=self.locations, anchor=self.orchestrator.confirmed_location_id,
                        active=self.active, observed_active=observed_active,
                        starting=self.starting, recovering=getattr(self, 'recovering', False), floor_selecting=getattr(self,"floor_selecting",False), recording=self.recorder.snapshot(),
                        navigation=self.map_view.snapshot(),
                        progress=self.progress if self.active else self.telemetry.get('mission_feedback', (0, {}))[1],
                        result=self.result if self.active else self.telemetry.get('mission_result', (0, self.result))[1],
                        telemetry={k: dict(age_sec=time.monotonic()-t, value=v) for k,(t,v) in self.telemetry.items()},
                        events=list(self.events), capabilities=dict(navigate=True,
                        photo_return=getattr(self.orchestrator,'photo',None) is not None, photo_return_reason=''),
                        photo=(self.orchestrator.photo.snapshot() if getattr(self.orchestrator,'photo',None) is not None else {}),
                        door='사용자가 열어둠 · API 사용 안 함')

    def mail_settings(self, data):
        photo=getattr(self.orchestrator,'photo',None)
        if photo is None:raise ValueError('촬영 미션 설정이 없습니다.')
        if not isinstance(data.get('recipient'),str):raise ValueError('메일 주소를 입력하세요.')
        return photo.mail_settings(data['recipient'])

    def capture_and_mail(self, data):
        photo=getattr(self.orchestrator,'photo',None)
        if photo is None:raise ValueError('촬영 기능 미설정')
        return photo.capture_and_mail(data.get('recipient'))

    def mail_retry(self, data):
        photo=getattr(self.orchestrator,'photo',None)
        if photo is None or photo.mail is None:raise ValueError('메일 작업 미설정')
        return photo.mail.retry(data.get('id'),data.get('confirm_unknown') is True)

    def map_snapshot(self):
        return self.map_view.image()

    def operator_phase(self, data):
        import uuid
        with self.operator_condition:
            tick, status = self.telemetry.get('operator_phase', (0, {}))
            if time.monotonic()-tick > 1 or not status.get('active'):
                raise ValueError('실행 중인 계단 임무가 없습니다. 종료된 임무를 임의로 새로 시작하지 않습니다.')
            if (data.get('run_id') != status.get('run_id') or data.get('revision') != status.get('revision')
                    or data.get('phase') not in status.get('phases', []) or data.get('confirmed') is not True):
                raise ValueError('현재 경로와 단계를 다시 확인하세요.')
            if self.stop_requested or self.starting or self.recovering or self.floor_selecting or self.operator_pending:
                raise ValueError('다른 제어 요청을 처리 중입니다.')
            request = {key:data[key] for key in ('run_id','revision','phase','confirmed')}
            request.update(issued_at=time.time(),request_id=uuid.uuid4().hex)
            self.operator_pending=True
            try:
                self.operator_publisher.publish(String(data=json.dumps(request)))
                def received():
                    result=self.telemetry.get('operator_phase',(0,{}))[1].get('result',{})
                    return result.get('request_id')==request['request_id']
                # Condition.wait releases the UI lock, so stop and status remain available.
                if not self.operator_condition.wait_for(received,timeout=2.):
                    raise ValueError('전이 응답을 확인하지 못했습니다. 현재 단계와 처리 결과를 확인하세요. 자동 재전송하지 않습니다.')
                result=self.telemetry['operator_phase'][1]['result']
                if result.get('state')=='REJECTED':raise ValueError(result.get('reason','전이 거절'))
                self.events.append(dict(time=time.time(),kind='operator_phase',detail='사용자 단계 지정: '+request['phase']))
                return dict(accepted=True,**result)
            finally:
                self.operator_pending=False

    def plan(self, data):
        destination = data.get('destination')
        kind = data.get('kind', 'navigate')
        if kind not in ('navigate', 'photo_return', 'rooftop_photo', 'return_to_start'):
            raise ValueError('unknown mission kind')
        if destination not in {x['id'] for x in self.locations}:
            raise ValueError('등록된 목적지를 선택하세요.')
        with self.lock:
            floor_time, floor = self.telemetry.get('floor', (0, {}))
            if time.monotonic()-floor_time > 2 or floor.get('state') != FloorState.READY:
                # An unconfirmed old anchor must not generate a fictional route
                # through other floors (or unsupported-stair diagnostics).
                detail = floor.get('detail', '')
                loc_time, loc = self.telemetry.get('localization', (0, {}))
                if time.monotonic()-loc_time <= 30 and loc.get('floor') == floor.get('floor_id') and loc.get('reason'):
                    detail = loc['reason']
                reason = '현재 층이 주행 준비 상태가 아닙니다. 층 상태를 확인하세요.'
                if floor.get('state') == FloorState.UNKNOWN:
                    reason = '현재 층 위치 확인이 필요합니다. RViz에서 지도와 실제 위치·방향을 확인하세요.'
                if time.monotonic()-floor_time > 2:
                    reason = '현재 층 상태 수신이 끊겼습니다. 스택 연결을 확인하세요.'
                elif detail:
                    reason += ' (' + detail + ')'
                return dict(segments=[], blockers=[reason], can_start=False, kind=kind)
            if kind == 'rooftop_photo' and floor.get('floor_id') != 'RF':
                return dict(segments=[],blockers=['옥상 RF 층 확인과 NAV 인계를 완료한 뒤 촬영 순회를 시작하세요.'],can_start=False,kind=kind)
            try:
                route = self.planner.plan_from_current_pose(self.orchestrator.confirmed_location_id, destination)
            except RoutePlanningError as error:
                raise ValueError(str(error)) from error
            payload = json.loads(route.to_bytes())
            photo = getattr(self.orchestrator,'photo',None)
            if kind in ('photo_return','rooftop_photo','return_to_start'):
                if photo is None:raise ValueError('촬영 미션 설정이 없습니다.')
                payload=photo.plan(self.planner,self.orchestrator.confirmed_location_id,kind=='return_to_start')
            blocks = []
            if kind == 'rooftop_photo' and any(seg.get('type') in ('STAIR','FLOOR_TRANSITION') for seg in payload['segments']):
                blocks.append('옥상 촬영 순회에 층간 경로가 포함되어 있습니다. 현재 층과 경로를 확인하세요.')
            tick, owner = self.telemetry.get('supervisor', (0, {}))
            if kind == 'rooftop_photo' and (time.monotonic()-tick > 2 or not owner.get('connected')):
                blocks.append('NAV 제어 연결의 최신 상태를 확인하지 못했습니다.')
            if time.monotonic()-tick <= 2 and owner.get('state') != 1:
                blocks.append('평지 복귀 후 제어권 복구를 누르세요. 현재 NAV 제어권이 없습니다.')
            floor_time, floor = self.telemetry.get('floor', (0, {}))
            if time.monotonic()-floor_time > 2 or floor.get('state') != FloorState.READY:
                blocks.append('현재 층 상태가 READY가 아니거나 최신 상태를 받지 못했습니다.')
            if route.segments and floor.get('floor_id') != route.segments[0].source_floor:
                blocks.append('실제 확인된 층과 임무 기준 층이 다릅니다. 층 초기화를 먼저 확인하세요.')
            stair_segments = ([s.get('profile_id') for s in payload['segments'] if s.get('profile_id')]
                              if kind in ('photo_return','rooftop_photo','return_to_start') else [s.profile_id for s in route.segments if s.profile_id])
            if stair_segments:
                tick, tracking = self.telemetry.get('tracking', (0, {}))
                available = {r['id'] for r in tracking.get('routes', []) if r.get('commissioned')}
                if time.monotonic()-tick > 2 or tracking.get('mode') != 'control':
                    blocks.append('계단 제어 설정을 최신 상태로 확인하지 못했습니다.')
                for segment in stair_segments:
                    if segment not in available:
                        blocks.append('지원되지 않은 계단 경로: '+segment)
            if kind == 'photo_return':
                # Validate the return route before outbound motion as well.
                origin=self.orchestrator.confirmed_location_id
                last=photo.settings['locations'][-1]
                try:
                    back=self.planner.plan_from_current_pose(last,origin)
                    needed={seg.profile_id for seg in back.segments if seg.profile_id}
                    tick,tracking=self.telemetry.get('tracking',(0,{}))
                    available={r['id'] for r in tracking.get('routes',[]) if r.get('commissioned')}
                    if needed and (time.monotonic()-tick>2 or tracking.get('mode')!='control' or not needed<=available):
                        blocks.append('복귀에 필요한 계단 경로가 준비되지 않았습니다.')
                except RoutePlanningError as error:blocks.append(str(error))
            payload.update(blockers=blocks, can_start=not blocks, kind=kind)
            return payload

    def start(self, data):
        record = data.get('record', False)
        if type(record) is not bool:
            raise ValueError('record must be a boolean')
        with self.lock:
            if getattr(self, "recovering", False):
                raise ValueError("제어권을 복구하는 중입니다.")
            if getattr(self, "floor_selecting", False):
                raise ValueError("층 지도를 변경하는 중입니다.")
            if self.closed:
                raise ValueError('콘솔이 종료 중입니다.')
            recording = self.recorder.snapshot()
            reuse_recording = bool(record and recording.get('state') == 'recording' and recording['busy'])
            if self.active or (recording['busy'] and not reuse_recording):
                raise ValueError('임무가 실행 중이거나 녹화 저장 중입니다. 녹화 중에는 테스트 버튼으로 이어서 시작하세요.')
            received, status = self.telemetry.get('mission_status', (0, {}))
            if time.monotonic()-received <= 2 and status.get('active'):
                raise ValueError('다른 클라이언트의 임무가 실행 중입니다. 해당 임무가 끝난 뒤 시작하세요.')
            plan = self.plan(data)
            if plan['blockers']:
                raise ValueError(' / '.join(plan['blockers']))
            if not self.client.wait_for_server(rospy.Duration(.2)):
                raise ValueError('mission action server에 연결할 수 없습니다.')
            self.active, self.result, self.progress = True, None, {}
            self.starting, self.stop_requested = True, False
            self.stop_event = stop_event = threading.Event()
            self.generation += 1
            generation = self.generation
        try:
            if record and not reuse_recording:
                self.recorder.start(stop_event)
            with self.lock:
                if self.stop_requested or self.closed:
                    self.active, self.starting = False, False
                    self.recorder.stop()
                    return dict(submitted=False, cancelled=True)
                # Recorder preparation may take seconds; revalidate before motion.
                plan = self.plan(data)
                if plan['blockers']:
                    raise ValueError(' / '.join(plan['blockers']))
                if reuse_recording and self.recorder.snapshot().get('state') != 'recording':
                    raise ValueError('이어 쓰던 녹화가 종료됐습니다. 녹화 상태를 확인하고 다시 시작하세요.')
                self.starting = False
                self.client.send_goal(MissionGoal(destination_id=data['destination'], mission_type={'photo_return':'photo_tour','rooftop_photo':'photo_tour','return_to_start':'return_to_start'}.get(data.get('kind'),'navigate'), return_after_task=False),
                                      done_cb=lambda s,r:self.done(s,r,generation),
                                      feedback_cb=lambda m:self.feedback(m,generation))
                self.events.append(dict(time=time.time(), kind='request', detail=('테스트 시작: ' if record else '주행 시작: ')+data['destination']))
                return dict(submitted=True, destination=data['destination'], recording=record)
        except Exception:
            with self.lock:
                self.active = False
                self.starting = False
                if record and not reuse_recording:
                    self.recorder.stop()
            raise

    def start_rooftop_photos(self, data):
        photo=getattr(self.orchestrator,'photo',None)
        if photo is None:raise ValueError('촬영 순회 설정이 없습니다.')
        return self.start(dict(destination=photo.settings['locations'][-1],
                               kind='rooftop_photo',record=data.get('record',False)))

    def approve_return(self, data):
        photo=getattr(self.orchestrator,'photo',None)
        if photo is None:raise ValueError('촬영 미션 설정이 없습니다.')
        photo.preflight(True)
        destination=photo.snapshot()['origin']['id']
        return self.start(dict(destination=destination,kind='return_to_start',record=data.get('record',False)))

    def feedback(self, message, generation=None):
        with self.lock:
            if generation is not None and generation != self.generation:
                return
            self.progress = primitive(message)

    def done(self, status, result, generation=None):
        with self.lock:
            if generation is not None and generation != self.generation:
                return
            self.active = False
            # Failed trials retain evidence through RC recovery and retries.
            # Explicit Stop/cancel still finalizes immediately; successful runs
            # keep their existing automatic save behavior.
            if status == 3 or getattr(self, 'stop_requested', False):
                self.recorder.stop()
            elif self.recorder.snapshot().get('state') == 'recording':
                self.events.append(dict(time=time.time(), kind='recording',
                    detail='임무 실패 후 녹화 유지: 수동 복구도 기록합니다. 정지를 누르면 저장합니다.'))
            self.result = dict(action_status=status, **(primitive(result) if result else {'reason':'결과 없음'}))
            self.telemetry['mission_result'] = (time.monotonic(), self.result)
            self.events.append(dict(time=time.time(), kind='result', detail=str(self.result)))

    def recover_control(self, data):
        """Explicit flat-ground handoff through the existing Supervisor service."""
        from std_srvs.srv import Trigger
        if data.get('confirmed_flat_ground') is not True:
            raise ValueError('평지 복귀와 계단 모드 해제를 확인하세요.')
        with self.lock:
            now = time.monotonic()
            tick, status = self.telemetry.get('mission_status', (0, {}))
            owner_tick, owner = self.telemetry.get('supervisor', (0, {}))
            if (self.closed or self.active or self.starting or getattr(self, 'floor_selecting', False)
                    or getattr(self, 'recovering', False) or now-tick > 2 or status.get('active')):
                raise ValueError('실행 중인 임무가 끝난 뒤 제어권을 복구하세요.')
            if now-owner_tick > 2 or not owner.get('connected'):
                raise ValueError('로봇 제어 연결 상태를 확인할 수 없습니다.')
            if owner.get('state') == 1:
                return dict(accepted=True, note='이미 NAV 제어권입니다. 목적지를 선택해 시작하세요.')
            if owner.get('state') != 2:
                raise ValueError('계단 중단 후의 제어권 복구만 지원합니다. 연결 상태를 확인하세요.')
            self.recovering = True
        name = '/stair_supervisor/acknowledge_physical_handoff'
        confirmation = '/stair_supervisor/operator_confirmed_supported_handoff'
        try:
            rospy.wait_for_service(name, timeout=3.)
        except rospy.ROSException as error:
            with self.lock:
                self.recovering = False
            raise ValueError('제어권 복구 서비스에 연결할 수 없습니다: '+str(error)) from error
        finished = threading.Event()
        outcome = {}

        def invoke():
            try:
                rospy.set_param(confirmation, True)
                response = rospy.ServiceProxy(name, Trigger)()
                if not response.success:
                    raise ValueError('제어권 복구 거절: '+response.message)
                with self.lock:
                    self.events.append(dict(time=time.time(), kind='control_recovery',
                        detail='평지 복귀 확인 · 계단 모드 해제 · NAV 제어권 복구 완료'))
                outcome['accepted'] = True
            except Exception as error:
                outcome['error'] = str(error)
                with self.lock:
                    self.events.append(dict(time=time.time(), kind='control_recovery',
                        detail='제어권 복구 실패: '+str(error)))
            finally:
                try:
                    rospy.set_param(confirmation, False)
                finally:
                    with self.lock:
                        self.recovering = False
                    finished.set()

        # The worker owns completion. An HTTP timeout must not release this
        # operation while a delayed service can still change robot ownership.
        threading.Thread(target=invoke, name='console-control-recovery', daemon=True).start()
        if not finished.wait(3.):
            return dict(accepted=False, pending=True,
                        note='제어권 복구 응답을 기다립니다. 처리가 끝날 때까지 새 임무와 층 변경을 대기합니다.')
        if 'error' in outcome:
            raise ValueError(outcome['error'])
        # Do not invent telemetry or clear the last failure. Normal state
        # publication confirms NAV before the next mission.
        return dict(accepted=True, note='계단 모드를 해제했습니다. 제어 상태가 NAV로 바뀌면 다시 시작하세요. 녹화 중이면 계속 기록합니다.')

    def select_floor(self, data):
        from multifloor_manager.srv import SelectFloor
        from multifloor_manager.ros_services import BoundedServiceCaller
        from std_srvs.srv import Trigger
        target = data.get('floor_id')
        if target not in {x['floor'] for x in self.locations}:
            raise ValueError('등록된 실제 층을 선택하세요.')
        with self.lock:
            tick, status = self.telemetry.get('mission_status', (0, {}))
            if (self.active or self.starting or self.floor_selecting or getattr(self, "recovering", False) or self.recorder.snapshot()['busy']
                    or time.monotonic()-tick > 2 or status.get('active')):
                raise ValueError('임무와 녹화를 종료한 뒤 실제 층을 선택하세요.')
            self.floor_selecting = True
        try:
            caller = BoundedServiceCaller(12.)
            # Relocation must first disarm the old floor's idle restoration.
            name = '/mission_manager/stop_arrival_hold'
            response = caller.call(name, rospy.ServiceProxy(name, Trigger))
            if not response.success:
                raise ValueError(response.message)
            name = '/multifloor/select_floor'
            response = caller.call(name, rospy.ServiceProxy(name, SelectFloor), target)
            if not response.accepted:
                raise ValueError(response.reason)
            with self.lock:
                self.telemetry.pop('floor', None)
                self.events.append(dict(time=time.time(), kind='floor_selection', detail='실제 층 지정: '+target))
            return dict(accepted=True, note='지도를 바꿨습니다. 자동 위치 확인 후 주행할 수 있습니다. 위치가 다르면 RViz에서 2D Pose Estimate로 지정하세요.')
        finally:
            with self.lock:
                self.floor_selecting = False

    def cancel(self, _data):
        with self.lock:
            manual = getattr(self, 'manual', None)
            manual_active = manual is not None and manual.snapshot()['active']
            if manual is not None:manual.stop()
            hold=getattr(self.orchestrator,'arrival_hold',None)
            holding=hold is not None and hold.core.armed
            if not self.active and not self.recorder.snapshot()['busy'] and not holding and not manual_active:
                raise ValueError('이 화면에서 실행 중인 임무가 없습니다.')
            if holding:hold.suspend()
            self.stop_requested = True
            self.stop_event.set()
            try:
                if self.active and not self.starting:
                    self.client.cancel_goal()
            finally:
                self.recorder.stop()
            return dict(cancel_requested=True, note='주행 취소와 녹화 저장을 요청했습니다. 종료 결과와 저장 완료 표시를 확인하세요.')

    def close(self):
        with self.lock:
            self.closed = True
            self.stop_requested = True
            self.stop_event.set()
            if self.active and not self.starting:
                self.client.cancel_goal()
            self.recorder.stop()
        if getattr(self, "manual", None):self.manual.close()
        self.http.close()
        self.operator_publisher.unregister()
        self.map_view.close()
        for subscriber in self.subscribers:
            subscriber.unregister()
