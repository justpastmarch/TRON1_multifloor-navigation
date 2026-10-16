"""ROS adapter inside the existing node; missions use /mission; manual input uses the existing supervisor."""
import json
import math
import threading
import time
from collections import deque

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import GetMap
from stair_supervisor.msg import SupervisorState
from multifloor_manager.msg import FloorState
from mission_manager.msg import MissionAction, MissionGoal, MissionActionFeedback, MissionActionResult
from mission_manager.mission_console import ConsoleServer
from mission_manager.route_planner import RoutePlanningError
from mission_manager.console_recording import ConsoleRecording
from mission_manager.console_map import ConsoleMap, contains_world_point, encode_map
from mission_manager.console_manual import ConsoleManual


def primitive(message):
    return {name: getattr(message, name) for name in message.__slots__
            if isinstance(getattr(message, name), (str, bool, int, float))}


class RosConsole:
    def __init__(self, planner, orchestrator, configuration, action_name, html_path, port):
        self.lock = threading.RLock()
        self.operator_condition = threading.Condition(self.lock)
        self.operator_pending = False
        self.operator_publisher = rospy.Publisher("/stair_supervisor/operator_phase_request", String, queue_size=1)
        self.initialpose_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
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
        self.relocating = False
        self.relocation = {}
        self.stop_event = threading.Event()
        self.map_view = ConsoleMap()
        self.client = actionlib.SimpleActionClient(action_name, MissionAction)
        self.subscribers = [
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
            self.initialpose_publisher.unregister()
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
                        relocating=getattr(self, 'relocating', False), relocation=dict(getattr(self, 'relocation', {})),
                        navigation=self.map_view.snapshot(),
                        progress=self.progress if self.active else self.telemetry.get('mission_feedback', (0, {}))[1],
                        result=self.result if self.active else self.telemetry.get('mission_result', (0, self.result))[1],
                        telemetry={k: dict(age_sec=time.monotonic()-t, value=v) for k,(t,v) in self.telemetry.items()},
                        events=list(self.events), capabilities=dict(navigate=True, navigate_point=True, rooftop_only=True,
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

    def relocate_flat(self, data):
        from .console_navigation import relocate
        return relocate(self, data)

    def _manual_pose_context(self, floor_id):
        now = time.monotonic()
        with self.lock:
            floor_time, floor = self.telemetry.get('floor', (0, {}))
            if now-floor_time > 2 or not floor.get('floor_id'):
                raise ValueError('현재 층 상태가 최신이 아닙니다. 층 지도를 확인하세요.')
            if floor.get('floor_id') != floor_id:
                raise ValueError('지도와 선택한 층이 바뀌었습니다. 지도를 새로 확인하세요.')
            if floor.get('state') not in (FloorState.UNKNOWN, FloorState.READY):
                raise ValueError('층 전이 또는 fault 중에는 위치를 지정할 수 없습니다.')
            detail = floor.get('detail', '')
            if detail.startswith(('operator selecting actual floor:', 'floor selection failed:')):
                raise ValueError('층 지도 변경이 끝난 뒤 위치를 지정하세요.')
            if (getattr(self, 'active', False) or getattr(self, 'starting', False)
                    or getattr(self, 'stop_requested', False) or getattr(self, 'recovering', False)
                    or getattr(self, 'floor_selecting', False) or getattr(self, 'operator_pending', False)
                    or getattr(self, 'relocating', False)):
                raise ValueError('다른 제어 작업이 진행 중입니다.')
            mission_time, mission = self.telemetry.get('mission_status', (0, {}))
            if now-mission_time <= 2 and mission.get('active'):
                raise ValueError('주행 임무가 실행 중일 때는 위치를 재설정할 수 없습니다.')
            supervisor_time, supervisor = self.telemetry.get('supervisor', (0, {}))
            if (now-supervisor_time <= 2
                    and supervisor.get('state') in (SupervisorState.STAIR, SupervisorState.FAULT)):
                raise ValueError('계단 모드 또는 fault 상태에서는 위치를 재설정할 수 없습니다.')
            return floor_id, floor.get('map_generation')

    def set_initial_pose(self, data):
        def number(key):
            value = data.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('위치와 방향은 유한한 숫자여야 합니다.')
            return float(value)

        x, y, angle = number('x'), number('y'), number('yaw')
        floor_id, revision = data.get('floor_id'), data.get('map_revision')
        if not isinstance(floor_id, str) or not floor_id or not isinstance(revision, str) or not revision:
            raise ValueError('현재 층과 지도 revision이 필요합니다.')
        context = self._manual_pose_context(floor_id)
        meta = self.map_view.image()
        if meta.get('revision') != revision or meta.get('frame') != 'map':
            raise ValueError('표시 중인 지도가 바뀌었습니다. 새 지도를 확인한 뒤 다시 지정하세요.')
        if not contains_world_point(meta, x, y):
            raise ValueError('지정한 위치가 현재 지도 범위 밖입니다.')

        # Compare the picture the operator used with map_server's active map so a
        # delayed floor switch can never apply a pose against the previous floor.
        service_name = rospy.get_param('/multifloor_manager/get_map_service', '/static_map')
        try:
            rospy.wait_for_service(service_name, timeout=1.0)
            active_map = rospy.ServiceProxy(service_name, GetMap)().map
        except (rospy.ROSException, rospy.ServiceException) as error:
            raise ValueError('현재 층 지도 서버를 확인할 수 없습니다. 잠시 후 다시 시도하세요.') from error
        if encode_map(active_map)['revision'] != revision:
            raise ValueError('현재 층 지도와 화면의 지도가 다릅니다. 지도가 갱신될 때까지 기다리세요.')

        with self.lock:
            if self._manual_pose_context(floor_id) != context:
                raise ValueError('층 또는 상태가 요청 중 바뀌었습니다. 다시 확인하세요.')
            meta = self.map_view.image()
            if meta.get('revision') != revision or not contains_world_point(meta, x, y):
                raise ValueError('표시 중인 지도가 바뀌었습니다. 새 지도를 확인한 뒤 다시 지정하세요.')
            if self.initialpose_publisher.get_num_connections() < 1:
                raise ValueError('위치 관리자 /initialpose 연결 대기 중입니다. 잠시 후 다시 시도하세요.')
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = y
            angle = math.atan2(math.sin(angle), math.cos(angle))
            pose.pose.pose.orientation.z = math.sin(angle/2.0)
            pose.pose.pose.orientation.w = math.cos(angle/2.0)
            pose.pose.covariance[0] = 0.04
            pose.pose.covariance[7] = 0.04
            pose.pose.covariance[35] = 0.03
            previous_status = self.telemetry.get('localization', (0, {}))[1]
            previous = {key: previous_status.get(key) for key in ('request_id', 'state', 'source', 'reason')}
            self.initialpose_publisher.publish(pose)
            self.events.append(dict(time=time.time(), kind='localization',
                                    detail='UI에서 현재 층 지도에 수동 위치·방향 지정'))
        previous_id = previous.get('request_id')
        if not isinstance(previous_id, int):
            previous_id = -1
        return dict(submitted=True, previous_request_id=previous_id,
                    previous_status=previous,
                    note='위치 추정 요청을 보냈습니다. AMCL 확인 상태를 기다립니다.')

    def operator_phase(self, data):
        import uuid
        with self.operator_condition:
            tick, status = self.telemetry.get('operator_phase', (0, {}))
            if time.monotonic()-tick > 1 or not status.get('active'):
                raise ValueError('실행 중인 계단 임무가 없습니다. 종료된 임무를 임의로 새로 시작하지 않습니다.')
            kind = data.get('kind', 'set_phase')
            if kind not in ('set_phase', 'continue'):
                raise ValueError('지원하지 않는 계단 단계 요청입니다.')
            if (data.get('run_id') != status.get('run_id') or data.get('revision') != status.get('revision')
                    or data.get('phase') not in status.get('phases', []) or data.get('confirmed') is not True):
                raise ValueError('현재 경로와 단계를 다시 확인하세요.')
            if kind == 'continue' and (status.get('continue_supported') is not True
                    or status.get('waiting') is not True or data.get('phase') != status.get('phase')):
                raise ValueError('현재 실행은 같은 단계 이어가기를 지원하지 않거나 중단 대기 상태가 아닙니다.')
            if self.stop_requested or self.starting or self.recovering or self.floor_selecting or self.operator_pending or getattr(self, 'relocating', False):
                raise ValueError('다른 제어 요청을 처리 중입니다.')
            request = {key:data[key] for key in ('run_id','revision','phase','confirmed')}
            request['kind'] = kind
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
                label = '같은 단계 이어가기: ' if kind == 'continue' else '사용자 단계 지정: '
                self.events.append(dict(time=time.time(),kind='operator_phase',detail=label+request['phase']))
                return dict(accepted=True,**result)
            finally:
                self.operator_pending=False

    def plan(self, data):
        destination = data.get('destination')
        kind = data.get('kind', 'navigate')
        if kind == 'navigate_point':
            from .console_navigation import plan_point
            with self.lock:
                return plan_point(self, data)
        if kind == 'stair_resume':
            return self._plan_stair_resume(data)
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
                payload=photo.plan(self.planner,self.orchestrator.confirmed_location_id,kind=='return_to_start',
                                   **({'rooftop_only':True} if kind=='rooftop_photo' else {}))
            blocks = []
            if kind == 'rooftop_photo' and any(seg.get('type') in ('STAIR','FLOOR_TRANSITION') for seg in payload['segments']):
                blocks.append('옥상 촬영 순회에 층간 경로가 포함되어 있습니다. 현재 층과 경로를 확인하세요.')
            tick, owner = self.telemetry.get('supervisor', (0, {}))
            if kind == 'rooftop_photo' and (time.monotonic()-tick > 2 or not owner.get('connected')):
                blocks.append('NAV 제어 연결의 최신 상태를 확인하지 못했습니다.')
            if time.monotonic()-tick <= 2 and owner.get('state') != 1:
                blocks.append('계단 중단 상태입니다. UI의 수동·복구 탭에서 현재 단계 재개 또는 평지 이동 확인·NAV 복구를 선택하세요.')
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
                saved=photo.snapshot()
                origin=saved['origin']['id'] if saved.get('origin') and saved.get('stage')!='COMPLETE' else self.orchestrator.confirmed_location_id
                last=payload['destination_id']
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

    def _plan_stair_resume(self, data):
        """Bind an explicit restart to the retained stair run, never a guessed floor."""
        with self.lock:
            tick, status = self.telemetry.get('operator_phase', (0, {}))
            owner_tick, owner = self.telemetry.get('supervisor', (0, {}))
            floor_tick, floor = self.telemetry.get('floor', (0, {}))
            if (time.monotonic()-tick > 1 or not status.get('resume_available')
                    or status.get('active') or status.get('pending')):
                raise ValueError('종료된 계단 임무의 최신 재개 상태가 없습니다.')
            if time.monotonic()-owner_tick > 2 or owner.get('state') != SupervisorState.STAIR or not owner.get('connected'):
                raise ValueError('계단 제어권의 최신 상태를 확인할 수 없습니다.')
            if (data.get('resume_route_id') != status.get('route_id') or
                    data.get('resume_run_id') != status.get('run_id') or
                    type(data.get('resume_revision')) is not int or
                    data.get('resume_revision') != status.get('revision') or
                    data.get('resume_phase') not in status.get('phases', ())):
                raise ValueError('중단된 경로 또는 단계가 변경되었습니다. 현재 상태를 다시 확인하세요.')
            try:
                stair, transition = self.planner.stair_segments_for_profile(status['route_id'])
            except RoutePlanningError as error:
                raise ValueError(str(error)) from error
            if time.monotonic()-floor_tick > 2 or floor.get('floor_id') != stair.source_floor:
                raise ValueError('현재 층과 중단된 계단 경로의 출발 층이 다릅니다.')
            if floor.get('state') == FloorState.FAULT:
                raise ValueError('층 관리자가 FAULT 상태입니다. 상태를 확인하세요.')
            if data.get('destination') not in (None, stair.target_id):
                raise ValueError('재개 목적지가 보존된 계단 경로의 도착점과 다릅니다.')
            return dict(origin_id=stair.source_id, destination_id=stair.target_id,
                        segments=[dict(type=item.type.value, source_id=item.source_id,
                                       target_id=item.target_id, source_floor=item.source_floor,
                                       target_floor=item.target_floor, profile_id=item.profile_id)
                                  for item in (stair, transition)],
                        blockers=[], can_start=True, kind='stair_resume',
                        route_id=status['route_id'], phase=data['resume_phase'])

    def start(self, data):
        record = data.get('record', False)
        if type(record) is not bool:
            raise ValueError('record must be a boolean')
        with self.lock:
            if getattr(self, 'relocating', False):
                raise ValueError('수동 이동한 층을 준비 중입니다. 화면의 진행 상태를 확인하세요.')
            if getattr(self, "recovering", False):
                raise ValueError("제어권을 복구하는 중입니다.")
            if getattr(self, "floor_selecting", False):
                raise ValueError("층 지도를 변경하는 중입니다.")
            if self.closed:
                raise ValueError('콘솔이 종료 중입니다.')
            recording = self.recorder.snapshot()
            reuse_recording = bool(record and recording.get('state') == 'recording' and recording['busy'])
            saving_only = recording.get('state') == 'stopping' and not record
            if self.active or (recording['busy'] and not reuse_recording and not saving_only):
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
                    self.stop_requested = False
                    return dict(submitted=False, cancelled=True)
                # Recorder preparation may take seconds; revalidate before motion.
                plan = self.plan(data)
                if plan['blockers']:
                    raise ValueError(' / '.join(plan['blockers']))
                if reuse_recording and self.recorder.snapshot().get('state') != 'recording':
                    raise ValueError('이어 쓰던 녹화가 종료됐습니다. 녹화 상태를 확인하고 다시 시작하세요.')
                self.starting = False
                resume = data.get('kind') == 'stair_resume'
                point = data.get('kind') == 'navigate_point'
                destination = plan['destination_id'] if resume or point else data['destination']
                point_fields = {k:plan[k] for k in ('point_floor','point_map_generation','point_x','point_y','point_yaw')} if point else {}
                self.client.send_goal(MissionGoal(
                    destination_id=destination,
                    mission_type=('resume_stair' if resume else
                                  {'photo_return':'photo_tour','rooftop_photo':'rooftop_tour','return_to_start':'return_to_start','navigate_point':'navigate_point'}.get(data.get('kind'),'navigate')),
                    return_after_task=False,
                    resume_route_id=data.get('resume_route_id','') if resume else '',
                    resume_run_id=data.get('resume_run_id','') if resume else '',
                    resume_revision=data.get('resume_revision',0) if resume else 0,
                    resume_phase=data.get('resume_phase','') if resume else '',
                    **point_fields,
                ),
                                      done_cb=lambda s,r:self.done(s,r,generation),
                                      feedback_cb=lambda m:self.feedback(m,generation))
                self.events.append(dict(time=time.time(), kind='request', detail=('테스트 시작: ' if record else '주행 시작: ')+destination))
                return dict(submitted=True, destination=destination, recording=record)
        except Exception:
            with self.lock:
                self.active = False
                self.starting = False
                self.stop_requested = False
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
            try:
                if status == 3 or getattr(self, 'stop_requested', False):
                    self.recorder.stop()
                elif self.recorder.snapshot().get('state') == 'recording':
                    self.events.append(dict(time=time.time(), kind='recording',
                        detail='임무 실패 후 녹화 유지: 수동 복구도 기록합니다. 정지를 누르면 저장합니다.'))
            except Exception as error:
                self.events.append(dict(time=time.time(), kind='recording_error', detail=str(error)))
            self.stop_requested = False
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
            phase_tick, phase = self.telemetry.get('operator_phase', (0, {}))
            if (self.closed or self.active or self.starting or getattr(self, 'floor_selecting', False) or getattr(self, 'relocating', False)
                    or getattr(self, 'recovering', False) or now-tick > 2 or status.get('active')):
                raise ValueError('실행 중인 임무가 끝난 뒤 제어권을 복구하세요.')
            if now-owner_tick > 2 or not owner.get('connected'):
                raise ValueError('로봇 제어 연결 상태를 확인할 수 없습니다.')
            if owner.get('state') == 1:
                return dict(accepted=True, note='이미 NAV 제어권입니다. 목적지를 선택해 시작하세요.')
            if owner.get('state') != 2:
                raise ValueError('계단 중단 후의 제어권 복구만 지원합니다. 연결 상태를 확인하세요.')
            if data.get('confirmed_physical_relocation') != '평지 이동 완료':
                raise ValueError('계단참에서는 NAV로 전환하지 마세요. 로봇을 실제 평지로 옮긴 뒤 물리적 이동을 확인하세요.')
            if now-phase_tick > 2 or phase.get('active') or not phase.get('resume_available'):
                raise ValueError('최신 계단 단계를 확인한 뒤 제어권을 복구하세요.')
            if phase.get('run_id') and data.get('confirmed_run_id') != phase.get('run_id'):
                raise ValueError('계단 임무가 바뀌었습니다. 현재 단계를 다시 확인하세요.')
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
            if (self.active or self.starting or self.floor_selecting or getattr(self, "recovering", False) or getattr(self, 'relocating', False) or self.recorder.snapshot()['busy']
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
            relocation = getattr(self, 'relocating', False)
            if relocation:
                self.relocation_stop.set()
            was_active = self.active
            manual = getattr(self, 'manual', None)
            manual_active = manual is not None and manual.snapshot()['active']
            if manual is not None:manual.stop()
            hold=getattr(self.orchestrator,'arrival_hold',None)
            holding=hold is not None and hold.core.armed
            recording=self.recorder.snapshot()
            recording_active=recording['busy'] and recording['state'] in ('starting', 'recording')
            if not self.active and not self.starting and not recording_active and not holding and not manual_active and not relocation:
                self.stop_event.set()
                self.stop_requested = False
                self.events.append(dict(time=time.time(), kind='stop_request', detail='already idle'))
                return dict(cancel_requested=True, note='이 화면의 임무는 이미 종료됐습니다. 수동 이동 후 현재 층 사용 또는 RF 평지 확인·순회 시작을 선택할 수 있습니다.')
            self.events.append(dict(time=time.time(), kind='stop_request',
                detail='mission=%s recording=%s manual=%s hold=%s' %
                (self.active, recording['state'], manual_active, holding)))
            if holding:hold.suspend()
            self.stop_requested = True
            self.stop_event.set()
            try:
                if self.active and not self.starting:
                    self.client.cancel_goal()
            finally:
                try:
                    self.recorder.stop()
                finally:
                    if not self.active and not self.starting:
                        self.stop_requested = False
            note=('주행 취소와 녹화 저장을 요청했습니다. 종료 결과를 확인하세요.'
                  if was_active else '녹화 중지·저장을 요청했습니다. 로컬 BAG 저장 상태를 확인하세요.')
            return dict(cancel_requested=True, note=note)

    def close(self):
        with self.lock:
            self.closed = True
            if getattr(self, 'relocating', False):
                self.relocation_stop.set()
            self.stop_requested = True
            self.stop_event.set()
            if self.active and not self.starting:
                self.client.cancel_goal()
            self.recorder.stop()
        if getattr(self, "manual", None):self.manual.close()
        self.http.close()
        self.operator_publisher.unregister()
        self.initialpose_publisher.unregister()
        self.map_view.close()
        for subscriber in self.subscribers:
            subscriber.unregister()
