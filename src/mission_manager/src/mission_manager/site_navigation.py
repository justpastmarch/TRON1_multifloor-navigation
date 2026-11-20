"""Point-to-point driving and waypoint patrol on a map made in the mapping console.

Runs beside the mapping console on the mini PC (local ROS master): the chosen
map is served with the existing AMCL + move_base launch and parameters
(obstacle layer from /scan, robot radius, inflation, speed limits), and
velocity reaches the robot only through the existing Stair Supervisor
(/navigation/cmd_vel), which keeps its freshness checks and joystick
intervention. This module adds no new stop condition: a patrol stops on an
operator stop or when move_base itself reports a failed goal.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rospy

from mission_manager.mapping_engine import Process
from mission_manager.mapping_store import atomic_json

GOAL_STATES = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED',
               5: 'REJECTED', 6: 'PREEMPTING', 7: 'RECALLING', 8: 'RECALLED', 9: 'LOST'}
MAX_WAYPOINTS = 100


def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def clean_waypoints(rows):
    if not isinstance(rows, list) or len(rows) > MAX_WAYPOINTS:
        raise ValueError('지점은 최대 %d개입니다.' % MAX_WAYPOINTS)
    out = []
    for row in rows:
        if not isinstance(row, dict) or not _finite(row.get('x'), row.get('y'), row.get('yaw')):
            raise ValueError('지점 좌표가 올바르지 않습니다.')
        name = str(row.get('name') or '지점 %d' % (len(out) + 1))[:40]
        out.append(dict(x=float(row['x']), y=float(row['y']), yaw=float(row['yaw']), name=name))
    return out


class SiteNavigation:
    def __init__(self, library, buffer, env=None, supervisor_tool=''):
        self.library, self.buffer, self.env = library, buffer, env or os.environ
        self.supervisor_tool = supervisor_tool
        self.lock = threading.RLock()
        self.process = None
        self.map = None
        self.state, self.detail = 'IDLE', ''
        self.waypoints = []
        self.patrol = None
        self.patrol_stop = threading.Event()
        self.target = None
        self.plan = []
        self.client = None
        self.subscribers = []
        self.pose_publisher = None

    # ---------- lifecycle ----------
    @property
    def active(self):
        return self.process is not None

    def start(self, data, mapping_active):
        if mapping_active:
            raise ValueError('지도 작성 중에는 주행을 시작할 수 없습니다. 작성 종료 후 시작하세요.')
        if self.active:
            raise ValueError('이미 주행 준비가 되어 있습니다. 다른 지도는 주행 종료 후 선택하세요.')
        folder = self.library.result_path(data.get('map_id'), data.get('map_key'))
        if not (folder / 'map.yaml').is_file():
            raise ValueError('2D 지도 파일(map.yaml)이 없는 결과입니다.')
        pose = data.get('initial') or dict(x=0., y=0., yaw=0.)
        if not _finite(pose.get('x'), pose.get('y'), pose.get('yaw')):
            raise ValueError('시작 위치가 올바르지 않습니다.')
        import rospkg
        nav_dir = Path(rospkg.RosPack().get_path('multifloor_manager')) / 'config/nav'
        env = self.env
        command = ['roslaunch', 'multifloor_manager', 'navigation.launch',
                   'map_yaml:=%s' % (folder / 'map.yaml'), 'nav_params_dir:=%s' % nav_dir,
                   'initial_x:=%.4f' % pose['x'], 'initial_y:=%.4f' % pose['y'], 'initial_yaw:=%.4f' % pose['yaw'],
                   'max_vel_x:=%s' % env.get('MAX_VEL_X', '0.30'), 'max_vel_theta:=%s' % env.get('MAX_VEL_THETA', '1.20'),
                   'min_in_place_vel_theta:=%s' % env.get('MIN_IN_PLACE_VEL_THETA', '0.30'),
                   'acc_lim_x:=%s' % env.get('ACC_LIM_X', '0.4'), 'acc_lim_theta:=%s' % env.get('ACC_LIM_THETA', '2.0')]
        log = folder.parent / 'navigation.log'
        with self.lock:
            self.state, self.detail = 'STARTING', '주행 노드 시작 중'
            self.map = dict(id=data['map_id'], key=data['map_key'], name=data.get('map_name', ''))
            self.waypoints = self._load_waypoints(folder)
        try:
            self._supervisor('drive-start')
            self.process = Process(command, log)
            self._connect(time.monotonic() + 60)
        except Exception:
            self.stop()
            raise
        with self.lock:
            self.state, self.detail = 'READY', '대기 · 지도에서 이동할 곳이나 순회 지점을 정하세요'
        return dict(ready=True)

    def _supervisor(self, action):
        if not self.supervisor_tool:
            raise ValueError('로봇 명령 경로(Stair Supervisor) 실행 도구가 설정되지 않았습니다. 미니 PC 모드에서 사용하세요.')
        result = subprocess.run([self.supervisor_tool, action], capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise RuntimeError((result.stderr or result.stdout).strip()[-400:] or 'Stair Supervisor ' + action + ' 실패')

    def _connect(self, deadline):
        import actionlib
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from move_base_msgs.msg import MoveBaseAction
        from nav_msgs.msg import Path as PathMsg
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while time.monotonic() < deadline:
            if self.process is None or not self.process.alive:
                raise RuntimeError('주행 노드가 종료됐습니다. 로그를 확인하세요.')
            if self.client.wait_for_server(rospy.Duration(1.0)):
                break
        else:
            raise RuntimeError('move_base가 준비되지 않았습니다 (60초).')
        self.pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.subscribers = [rospy.Subscriber('/move_base/NavfnROS/plan', PathMsg, self._on_plan, queue_size=1)]

    def stop(self, data=None):
        self.halt()
        with self.lock:
            process, self.process = self.process, None
            for sub in self.subscribers:
                sub.unregister()
            self.subscribers = []
            if self.pose_publisher:
                self.pose_publisher.unregister()
                self.pose_publisher = None
            self.client = None
            self.plan, self.target = [], None
        if process:
            process.stop()
        try:
            self._supervisor('drive-stop')
        except Exception as error:
            rospy.logwarn('Stair Supervisor stop: %s', error)
        with self.lock:
            self.state, self.detail, self.map = 'IDLE', '주행 종료', None
        return dict(stopped=True)

    # ---------- commands ----------
    def set_pose(self, data):
        self._require_ready()
        x, y, yaw = data.get('x'), data.get('y'), data.get('yaw')
        if not _finite(x, y, yaw):
            raise ValueError('위치 좌표가 올바르지 않습니다.')
        from geometry_msgs.msg import PoseWithCovarianceStamped
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = rospy.Time.now()
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        covariance = [0.] * 36
        covariance[0] = covariance[7] = 0.25 ** 2
        covariance[35] = math.radians(15) ** 2
        message.pose.covariance = covariance
        self.pose_publisher.publish(message)
        with self.lock:
            self.detail = '위치 지정 (%.2f, %.2f)' % (x, y)
        return dict(published=True)

    def goto(self, data):
        self._require_ready()
        point = clean_waypoints([dict(data, name='목표')])[0]
        self._end_run(cancel_all=False)
        self.patrol_stop.clear()
        thread = threading.Thread(target=self._run, args=([point], False, '이동'), daemon=True)
        with self.lock:
            self.patrol = thread
        thread.start()
        return dict(accepted=True)

    def start_patrol(self, data):
        self._require_ready()
        points = self.waypoints if data.get('waypoints') is None else clean_waypoints(data['waypoints'])
        if not points:
            raise ValueError('순회할 지점이 없습니다. 지도에서 지점을 추가하세요.')
        start = int(data.get('start_index') or 0)
        if not 0 <= start < len(points):
            raise ValueError('시작 지점 번호가 올바르지 않습니다.')
        self._end_run(cancel_all=False)
        self.patrol_stop.clear()
        thread = threading.Thread(target=self._run, args=(points[start:] + (points[:start] if data.get('loop') else []),
                                                          bool(data.get('loop')), '순회'), daemon=True)
        with self.lock:
            self.patrol = thread
        thread.start()
        return dict(accepted=True, count=len(points))

    def halt(self, data=None):
        """Operator stop: cancel the patrol and every move_base goal."""
        self._end_run(cancel_all=True)
        with self.lock:
            if self.state == 'FAILED' and self.active:
                self.state, self.detail = 'READY', '정지됨 (운영자)'
        return dict(stopped=True)

    def _end_run(self, cancel_all):
        # A new goal only cancels the goal this console sent: a cancel-all
        # message can arrive after the next goal and preempt it.
        self.patrol_stop.set()
        client = self.client
        if client is not None:
            try:
                if cancel_all:
                    client.cancel_all_goals()
                else:
                    client.cancel_goal()
            except Exception as error:
                rospy.logwarn('cancel goals: %s', error)
        thread = self.patrol
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3)
        with self.lock:
            self.patrol, self.target = None, None
            if self.state in ('MOVING', 'PATROL'):
                self.state, self.detail = 'READY', '정지됨 (운영자)'

    def set_waypoints(self, data):
        points = clean_waypoints(data.get('waypoints'))
        with self.lock:
            if not self.map:
                raise ValueError('주행할 지도를 먼저 불러오세요.')
            folder = self.library.result_path(self.map['id'], self.map['key'])
            atomic_json(folder / 'waypoints.json', dict(version=1, waypoints=points))
            self.waypoints = points
        return dict(saved=len(points))

    # ---------- execution ----------
    def _run(self, points, loop, label):
        from move_base_msgs.msg import MoveBaseGoal
        lap = 0
        try:
            while not self.patrol_stop.is_set():
                lap += 1
                for index, point in enumerate(points):
                    if self.patrol_stop.is_set():
                        return
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = point['x'], point['y']
                    goal.target_pose.pose.orientation.z = math.sin(point['yaw'] / 2)
                    goal.target_pose.pose.orientation.w = math.cos(point['yaw'] / 2)
                    with self.lock:
                        self.target = dict(point, index=index, total=len(points), lap=lap)
                        self.state = 'PATROL' if label == '순회' else 'MOVING'
                        self.detail = '%s · %s (%d/%d)%s' % (label, point['name'], index + 1, len(points),
                                                             ' · %d바퀴째' % lap if loop else '')
                    self.client.send_goal(goal)
                    while not self.patrol_stop.is_set():
                        if self.client.wait_for_result(rospy.Duration(0.5)):
                            break
                    if self.patrol_stop.is_set():
                        return
                    status = self.client.get_state()
                    if status != 3:
                        with self.lock:
                            self.state = 'FAILED'
                            self.detail = '%s 중단 · %s 도달 실패 (move_base %s). 위치·장애물을 확인한 뒤 다시 시작하세요.' % (
                                label, point['name'], GOAL_STATES.get(status, status))
                            self.target = None
                        return
                if not loop:
                    break
            with self.lock:
                if not self.patrol_stop.is_set():
                    self.state, self.target = 'READY', None
                    self.detail = '%s 완료' % label
        except Exception as error:
            with self.lock:
                self.state, self.detail, self.target = 'FAILED', '%s 오류: %s' % (label, error), None

    # ---------- state ----------
    def _require_ready(self):
        if not self.active or self.client is None:
            raise ValueError('주행할 지도를 먼저 불러오세요 (주행 시작).')

    def _load_waypoints(self, folder):
        path = folder / 'waypoints.json'
        if not path.is_file():
            return []
        try:
            return clean_waypoints(json.loads(path.read_text()).get('waypoints', []))
        except (ValueError, OSError, AttributeError):
            return []

    def _on_plan(self, message):
        poses = message.poses
        step = max(1, len(poses) // 150)
        with self.lock:
            self.plan = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in poses[::step]]

    def robot_pose(self):
        if not self.active:
            return None
        try:
            t = self.buffer.lookup_transform('map', 'base_Link', rospy.Time(0), rospy.Duration(0.05))
        except Exception:
            return None
        q, p = t.transform.rotation, t.transform.translation
        age = (rospy.Time.now() - t.header.stamp).to_sec()
        return dict(x=round(p.x, 3), y=round(p.y, 3),
                    yaw=round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 3),
                    age=round(age, 2))

    def snapshot(self):
        with self.lock:
            if self.process is not None and not self.process.alive and self.state != 'FAILED':
                self.state, self.detail = 'FAILED', '주행 노드가 종료됐습니다. 주행 종료 후 다시 시작하세요.'
            return dict(state=self.state, detail=self.detail, map=self.map, waypoints=list(self.waypoints),
                        target=self.target, plan=list(self.plan) if self.target else [],
                        pose=self.robot_pose(), available=bool(self.supervisor_tool))
