"""Automatic/manual localization inside the existing floor-manager process."""
import copy
import json
import math
import threading
import time

import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from stair_supervisor.msg import SupervisorState

from multifloor_manager.msg import FloorState
from multifloor_manager.map_evidence import fingerprint_occupancy_grid, Nanoseconds
from multifloor_manager.startup_matching import ScanMatcher
from multifloor_manager.readiness import DEFAULT_READINESS_POLICY


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def valid_pose(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    covariance = message.pose.covariance
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w] + list(message.pose.covariance)
    return (message.header.frame_id == 'map' and all(math.isfinite(v) for v in values)
            and abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1.) < .01
            and all(0 <= covariance[i] <= 4. for i in (0,7,35))
            and abs(covariance[1]-covariance[6]) <= 1e-9
            and covariance[1]**2 <= covariance[0]*covariance[7]+1e-9)


class StartupLocalization:
    """One replaceable request; the manual request supersedes an automatic search.

    This object never publishes velocity or action goals. Startup gates READY;
    ongoing navigation is not periodically subjected to global localization.
    """
    def __init__(self, node, mode):
        if mode not in ('auto', 'manual'):
            raise ValueError('startup_localization must be auto, manual, or disabled')
        self.node, self.runtime = node, node.runtime
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.generation = 0
        self.request = ('auto', None) if mode == 'auto' else None
        self.latest_map = self.scan = self.pose = self.odom = None
        self.supervisor = None
        self.action_states = {}
        self.status = rospy.Publisher('/multifloor/localization_status', String, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber('/map', OccupancyGrid, lambda m: self.store('latest_map', m), queue_size=1),
            rospy.Subscriber('/scan', LaserScan, lambda m: self.store('scan', m), queue_size=1),
            rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, lambda m: self.store('pose', m), queue_size=1),
            rospy.Subscriber('/tron/wheel_odom_raw', Odometry, lambda m: self.store('odom', m), queue_size=1),
            rospy.Subscriber('/stair_supervisor/state', SupervisorState, lambda m: self.store('supervisor', m), queue_size=1),
            rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.manual, queue_size=1),
        ]
        self.action_subscribers = []
        for topic in ('/mission/status', '/move_base/status', '/stair_traversal/status'):
            self.action_states[topic] = None
            self.action_subscribers.append(rospy.Subscriber(
                topic, GoalStatusArray, lambda m, t=topic: self.store_action(t, m), queue_size=1))
        self.retry_service = rospy.Service('/multifloor/localize_auto', Trigger, self.retry)
        self.publish('SEARCHING' if mode == 'auto' else 'NEEDS_POSE', 'waiting for map and scan')
        self.worker = threading.Thread(target=self.run, name='floor-start-localization', daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.wake.set)

    def store(self, name, message):
        with self.lock:
            setattr(self, name, message)
        self.wake.set()

    def store_action(self, topic, message):
        with self.lock:
            self.action_states[topic] = (time.monotonic(), message)

    def publish(self, state, reason, **details):
        payload = dict(state=state, reason=reason, floor=self.runtime.current_floor or self.runtime.initial_floor,
                       request_id=self.generation, **details)
        self.status.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def busy(self):
        if self.runtime.state in (FloorState.TRANSITIONING, FloorState.FAULT):
            return True
        if self.supervisor is not None and self.supervisor.state == SupervisorState.STAIR:
            return True
        for subscriber, (_, value) in zip(self.action_subscribers, self.action_states.items()):
            if value is not None and any(s.status in (0,1,6,7) for s in value[1].status_list):
                return True
            if subscriber.get_num_connections() == 0:
                continue
            if value is None or time.monotonic()-value[0] > 2.:
                return True
        return False

    def enqueue(self, mode, pose):
        with self.node._active_lock, self.lock:
            if self.busy() or self.node._active:
                return False
            self.generation += 1
            self.request = (mode, pose)
            self.publish('SEARCHING' if mode == 'auto' else 'VERIFYING', 'waiting for fresh stationary scan', source=mode)
            # Fence mission admission immediately, even while an old search unwinds.
            with self.runtime.condition:
                self.runtime.publish_floor(FloorState.UNKNOWN, 'localizing: '+mode,
                                           self.runtime.current_floor or self.runtime.initial_floor)
            self.wake.set()
            return True

    def manual(self, message):
        if not valid_pose(message):
            self.publish('INPUT_REJECTED', 'manual pose must be finite, normalized, and in map')
            return
        if not self.enqueue('manual', copy.deepcopy(message)):
            self.publish('INPUT_REJECTED', 'finish or cancel the current mission before setting position')

    def retry(self, _request):
        accepted = self.enqueue('auto', None)
        return TriggerResponse(success=accepted, message='search queued' if accepted else 'mission or floor transition active')

    def current(self, generation):
        return generation == self.generation and not rospy.is_shutdown()

    def fresh(self, message):
        age = int(DEFAULT_READINESS_POLICY.max_age_ns)/1e9
        return message is not None and 0 <= (rospy.Time.now()-message.header.stamp).to_sec() <= age

    def stationary(self):
        if not self.fresh(self.odom):
            return False
        v, w = self.odom.twist.twist.linear, self.odom.twist.twist.angular
        return (all(math.isfinite(x) for x in (v.x,v.y,w.z))
                and math.hypot(v.x,v.y) <= DEFAULT_READINESS_POLICY.max_linear_speed
                and abs(w.z) <= DEFAULT_READINESS_POLICY.max_angular_speed)

    def scan_points(self, scan):
        transform = self.node.tf_buffer.lookup_transform('base_Link', scan.header.frame_id,
                                                        scan.header.stamp, rospy.Duration(.2)).transform
        q, t = transform.rotation, transform.translation
        rotation = np.array([[1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w)],
                             [2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z)]])
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= max(.3, scan.range_min)) & (ranges < min(12., scan.range_max))
        points = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))
        return points @ rotation.T + [t.x,t.y]

    def run(self):
        while not rospy.is_shutdown():
            self.wake.wait(.2)
            self.wake.clear()
            with self.lock:
                request, generation = self.request, self.generation
                grid, scan, odom = self.latest_map, self.scan, self.odom
            if request is None or grid is None or not self.fresh(scan) or not self.stationary():
                continue
            if self.runtime.map_state.current_fingerprint is None or self.busy():
                continue
            if self.node.initialpose_publisher.get_num_connections() == 0:
                continue
            with self.lock:
                if not self.current(generation):
                    continue
                self.request = None
            try:
                self.localize(generation, request, grid, scan, odom)
            except InterruptedError:
                pass
            except Exception as error:
                if self.current(generation):
                    self.publish('NEEDS_POSE', str(error))
                    rospy.logwarn('startup localization: %s', error)

    def localize(self, generation, request, grid, scan, odom):
        identity = fingerprint_occupancy_grid(grid, Nanoseconds(rospy.Time.now().to_nsec())).identity()
        if identity != self.runtime.map_state.current_fingerprint.identity():
            raise ValueError('observed map does not match selected floor')
        origin = grid.info.origin
        matcher = ScanMatcher(np.array(grid.data).reshape(grid.info.height,grid.info.width), grid.info.resolution,
                              (origin.position.x,origin.position.y,yaw(origin.orientation)),
                              **rospy.get_param('~startup_matching', {}))
        mode, pose = request
        if mode == 'auto':
            self.publish('SEARCHING', 'comparing scan with selected floor')
            match = matcher.search(self.scan_points(scan), lambda: not self.current(generation))
            if not self.current(generation):
                return
            if not match.unique:
                self.publish('AMBIGUOUS', 'use 2D Pose Estimate or retry automatic search',
                             score=match.score, hit_fraction=match.hit_fraction, margin=match.margin,
                             candidate=list(match.pose))
                return
            p0, p1 = odom.pose.pose.position, self.odom.pose.pose.position
            turn = yaw(odom.pose.pose.orientation)-yaw(self.odom.pose.pose.orientation)
            if (not self.stationary() or math.hypot(p0.x-p1.x,p0.y-p1.y) > .05
                    or abs(math.atan2(math.sin(turn),math.cos(turn))) > .03):
                raise ValueError('robot moved during search; retry while stationary')
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x, pose.pose.pose.position.y = match.pose[:2]
            pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = math.sin(match.pose[2]/2), math.cos(match.pose[2]/2)
            pose.pose.covariance[0] = pose.pose.covariance[7] = .04
            pose.pose.covariance[35] = .03
        with self.lock, self.runtime.condition:
            if not self.current(generation) or self.busy():
                return
            pose.header.stamp = rospy.Time.now()
            self.runtime.epoch += 1
            self.runtime.active_epoch = self.runtime.epoch
            self.runtime.arm_localization(pose.header.stamp.to_nsec())
            self.runtime.publish_floor(FloorState.UNKNOWN, 'checking '+mode+' localization',
                                       self.runtime.current_floor or self.runtime.initial_floor)
            self.node.initialpose_publisher.publish(pose)
            self.publish('VERIFYING', 'waiting for fresh AMCL pose, scan agreement, and TF', source=mode)
        deadline = time.monotonic()+self.node.transaction_timeout
        marker = pose.header.stamp.to_nsec()
        while self.current(generation) and time.monotonic() < deadline:
            if self.busy():
                raise ValueError('action became active during localization')
            self.node.services.call(self.node.nomotion_name, self.node.nomotion_update)
            fence = max(marker, rospy.Time.now().to_nsec())
            # AMCL can consume the request before the RPC response reaches us.
            # Reissue on fresh scans instead of waiting the entire transaction
            # for an update that a stationary AMCL will not produce by itself.
            update_deadline = min(deadline, time.monotonic()+.5)
            while self.current(generation) and time.monotonic() < update_deadline:
                with self.runtime.condition:
                    if self.runtime.has_pose_newer_than(fence):
                        marker = self.runtime.pose_high_water_ns(fence)
                        break
                self.wake.wait(.05)
                self.wake.clear()
            with self.lock, self.runtime.condition:
                evidence = self.runtime.localization_evidence()
                current_pose, current_scan = self.pose, self.scan
                if not self.current(generation):
                    return
                if evidence is None or not evidence.ready or not self.fresh(current_pose) or not self.fresh(current_scan):
                    continue
                p, q = current_pose.pose.pose.position, current_pose.pose.pose.orientation
                scores, hit = matcher.score([(p.x,p.y,yaw(q))], matcher.points(self.scan_points(current_scan)))
                if hit[0] < matcher.min_hit or scores[0] < matcher.min_hit:
                    raise ValueError('pose and scan do not match map; adjust position/direction')
            # No map change: clear obsolete costmap data once, then release READY.
            self.node.services.call(self.node.clear_costmaps_name, self.node.clear_costmaps)
            with self.lock, self.runtime.condition:
                if not self.current(generation) or self.busy():
                    return
                evidence = self.runtime.localization_evidence()
                if evidence is None or not evidence.ready:
                    continue
                self.runtime.finish_transition(FloorState.READY, mode+' localization ready', self.runtime.current_floor)
                self.publish('READY', 'localization ready', source=mode, x=p.x, y=p.y, yaw=yaw(q),
                             hit_fraction=float(hit[0]), score=float(scores[0]))
            return
        if self.current(generation):
            raise ValueError('localization timeout; manual pose or retry remains available')
