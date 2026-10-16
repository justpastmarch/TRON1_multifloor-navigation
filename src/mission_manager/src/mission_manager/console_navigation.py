"""Map-point planning and operator relocation within the existing console.

Relocation changes mode/map only; it never starts driving. Autonomous motion still
uses the one MissionActionServer and NavigationExecutor.
"""
import math
import threading
import time

import rospy
from std_srvs.srv import Trigger
from multifloor_manager.srv import SelectFloor
from .console_map import contains_world_point


def plan_point(console, data):
    values = [data.get(k) for k in ('x', 'y', 'yaw')]
    if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
        raise ValueError('지도 목표 위치와 방향을 확인하세요.')
    floor_id, revision = data.get('floor_id'), data.get('map_revision')
    tick, floor = console.telemetry.get('floor', (0, {}))
    if time.monotonic()-tick > 2 or floor.get('state') != 2:
        raise ValueError('현재 층 지도와 위치 확인 후 지도 NAV를 시작하세요.')
    if floor.get('floor_id') != floor_id:
        raise ValueError('목표를 선택한 층이 바뀌었습니다. 새 지도에서 다시 선택하세요.')
    meta = console.map_view.image()
    if not revision or meta.get('revision') != revision or meta.get('frame') != 'map':
        raise ValueError('목표를 선택한 지도가 바뀌었습니다. 다시 선택하세요.')
    if not contains_world_point(meta, *values[:2]):
        raise ValueError('목표가 지도 범위 밖입니다.')
    tick, owner = console.telemetry.get('supervisor', (0, {}))
    if time.monotonic()-tick > 2 or not owner.get('connected') or owner.get('state') != 1:
        raise ValueError('평지로 수동 이동했다면 현재 층 사용 버튼으로 NAV를 준비하세요.')
    x, y, yaw = values
    return dict(kind='navigate_point', destination_id='ui_point', origin_id='current_pose',
                point_floor=floor_id, point_map_generation=int(floor['map_generation']),
                point_x=x, point_y=y, point_yaw=math.atan2(math.sin(yaw), math.cos(yaw)),
                blockers=[], can_start=True,
                segments=[dict(type='NAVIGATION', source_floor=floor_id, target_floor=floor_id,
                               target_id='ui_point', x=x, y=y, yaw=yaw, profile_id=None)])


def relocate(console, data):
    target = data.get('floor_id')
    if target not in {loc['floor'] for loc in console.locations}:
        raise ValueError('실제로 도착한 층을 선택하세요.')
    if data.get('confirmed_flat_ground') is not True:
        raise ValueError('계단을 벗어나 평지에 도착했음을 확인하세요.')
    with console.lock:
        if (console.closed or getattr(console, 'relocating', False) or console.starting
                or getattr(console, 'recovering', False) or getattr(console, 'floor_selecting', False)
                or getattr(console, 'operator_pending', False)):
            raise ValueError('이전 준비 요청이 처리 중입니다. 현재 진행 상태를 확인하세요.')
        if not console.active and _observed_active(console):
            raise ValueError('다른 클라이언트의 실행 중인 임무를 먼저 종료하세요.')
        console.relocating = True
        console.relocation_stop = threading.Event()
        console.relocation = dict(state='PREPARING', floor_id=target, detail='이전 임무 종료 확인 중')
        try:
            if console.active:
                console.stop_requested = True
                console.stop_event.set()
                console.client.cancel_goal()
            # Do not wait for a potentially large raw-BAG copy.
            console.recorder.stop()
        except Exception as error:
            console.relocating = False
            console.relocation.update(state='FAILED', detail=str(error))
            raise
    try:
        thread = threading.Thread(target=_relocate_worker, args=(console, target),
                                  name='console-flat-relocation', daemon=True)
        thread.start()
    except Exception as error:
        with console.lock:
            console.relocating = False
            console.relocation.update(state='FAILED', detail=str(error))
        raise
    return dict(accepted=True, pending=True, note='평지 이동을 확인했습니다. 이전 임무 종료 → NAV 준비 → 층 지도를 차례로 적용합니다.')


def _observed_active(console):
    tick, status = console.telemetry.get('mission_status', (0, {}))
    return time.monotonic()-tick <= 2 and status.get('active', False)


def _confirmed_idle(console):
    tick, status = console.telemetry.get('mission_status', (0, {}))
    return not console.active and time.monotonic()-tick <= 2 and status.get('active') is False


def _wait(console, predicate, detail, seconds=12.):
    deadline = time.monotonic()+seconds
    while True:
        if console.closed or console.relocation_stop.is_set():
            raise ValueError('평지 준비를 취소했습니다. 변경된 현재 층과 모드는 화면에서 확인하세요.')
        if predicate():
            return
        if time.monotonic() >= deadline:
            raise ValueError(detail+' 응답 지연. 준비 요청은 종료됐으며 다시 시도할 수 있습니다.')
        console.relocation_stop.wait(.05)


def _relocate_worker(console, target):
    confirmation = '/stair_supervisor/operator_confirmed_supported_handoff'
    try:
        _wait(console, lambda: _confirmed_idle(console), '이전 임무 종료 상태')
        hold = getattr(console.orchestrator, 'arrival_hold', None)
        if hold is not None:
            hold.suspend()
        def call(name, service, *args):
            # This operation already owns a background worker. Keep ownership
            # until the actual reply, so a late mode/map change cannot collide
            # with a newly admitted mission after an HTTP timeout.
            if console.closed or console.relocation_stop.is_set():
                raise ValueError('평지 준비를 취소했습니다.')
            rospy.wait_for_service(name, timeout=8.)
            if console.closed or console.relocation_stop.is_set():
                raise ValueError('평지 준비를 취소했습니다.')
            return rospy.ServiceProxy(name, service)(*args)
        def owner_ready():
            tick, owner = console.telemetry.get('supervisor', (0, {}))
            return time.monotonic()-tick <= 2 and owner.get('connected')
        _wait(console, owner_ready, '로봇 연결')
        console.relocation.update(detail='평지 NAV 모드 준비 중')
        owner = console.telemetry['supervisor'][1]
        if owner.get('state') == 2:
            rospy.set_param(confirmation, True)
            try:
                name = '/stair_supervisor/acknowledge_physical_handoff'
                response = call(name, Trigger)
            finally:
                rospy.set_param(confirmation, False)
            if not response.success:
                raise ValueError('평지 모드 전환 응답: '+response.message)
        elif owner.get('state') != 1:
            raise ValueError('로봇 연결 또는 모드 오류를 확인하세요.')
        _wait(console, lambda: owner_ready() and console.telemetry['supervisor'][1].get('state') == 1, 'NAV 모드')
        console.relocation.update(nav_ready=True)
        name = '/mission_manager/stop_arrival_hold'
        response = call(name, Trigger)
        if not response.success:
            raise ValueError(response.message)
        console.relocation.update(detail=target+' 지도 준비 중')
        tick, floor = console.telemetry.get('floor', (0, {}))
        if time.monotonic()-tick > 2 or floor.get('floor_id') != target or floor.get('state') != 2:
            name = '/multifloor/select_floor'
            response = call(name, SelectFloor, target)
            if not response.accepted:
                raise ValueError(response.reason)
        console.relocation.update(state='DONE', detail=target+' 평지 준비 요청 완료. 지도 위치 확인 후 주행·순회를 시작하세요.')
    except Exception as error:
        console.relocation.update(state='FAILED', detail=str(error)+' · 이미 변경된 모드/지도는 유지됩니다. 현재 상태에서 다시 준비할 수 있습니다.')
    finally:
        with console.lock:
            console.relocating = False
            console.events.append(dict(time=time.time(), kind='relocation', detail=console.relocation['detail']))
