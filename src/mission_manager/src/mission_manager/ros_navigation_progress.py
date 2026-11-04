"""Bounded, goal-correlated progress adapter; creates no ROS node or cmd_vel."""

import math
import threading
import time

import rospy
from actionlib_msgs.msg import GoalStatus

from mission_manager.navigation_progress import NavigationProgress


_nav_limit_lock = threading.Lock()
_nav_limit_revision = 0


class ProgressAwareClient:
    def __init__(self, client, clock=time.monotonic, policy=None, entry_region=(0.25, 0.2), arrival_dwell_sec=0.5):
        if not math.isfinite(arrival_dwell_sec) or arrival_dwell_sec <= 0:
            raise ValueError('arrival dwell must be positive and finite')
        self.entry_region = entry_region
        self.default_entry_region = entry_region
        self.arrival_dwell_sec = arrival_dwell_sec
        self.entry_approach = False
        self.entry_region_reached = False
        self.entry_since = None
        self.entry_latest = None
        self.client, self.clock = client, clock
        self.policy = policy or {}
        self.lock = threading.RLock()
        self.guard = None
        self.generation = 0
        self.terminal = None
        self.failure_reason = ''
        self.warning_reason = ''
        self.user_cancel = False
        self.cancel_at = None
        self.finished = threading.Event()

    def set_arrival_region(self, region):
        if region is not None and (len(region)!=2 or any(not math.isfinite(v) or v<=0 for v in region)):
            raise ValueError('arrival region must contain positive finite radius/yaw')
        with self.lock:
            self.entry_region = self.default_entry_region if region is None else tuple(region)

    def set_entry_approach(self, enabled):
        with self.lock:
            self.entry_approach = bool(enabled)

    def send_goal(self, goal, done_cb):
        p = goal.target_pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            self.generation += 1
            generation = self.generation
            self.guard = NavigationProgress((p.position.x, p.position.y, yaw), self.clock(), **self.policy)
            self.terminal, self.failure_reason, self.warning_reason, self.user_cancel = None, '', '', False
            self.cancel_at = None
            self.feedback_stamp = None
            self.entry_region_reached = False
            self.entry_since = self.entry_latest = None
            target = (p.position.x, p.position.y, yaw)
            self.finished = finished = threading.Event()

        def feedback(message):
            with self.lock:
                if generation != self.generation or self.terminal is not None:
                    return
                if message.base_position.header.frame_id.lstrip('/') != 'map':
                    return
                stamp = message.base_position.header.stamp.to_nsec()
                if self.feedback_stamp is not None and stamp <= self.feedback_stamp:
                    return
                self.feedback_stamp = stamp
                p = message.base_position.pose
                q = p.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                now = self.clock()
                self.guard.observe(p.position.x, p.position.y, yaw, now)
                if self.entry_approach:
                    distance = math.hypot(p.position.x-target[0], p.position.y-target[1])
                    angle = abs(math.atan2(math.sin(yaw-target[2]), math.cos(yaw-target[2])))
                    age = (rospy.Time.now().to_nsec()-stamp)/1e9
                    within = (math.isfinite(distance) and math.isfinite(angle)
                              and 0 <= age <= 1.0
                              and distance <= self.entry_region[0] and angle <= self.entry_region[1])
                    self.entry_latest = now
                    self.entry_since = (now if self.entry_since is None else self.entry_since) if within else None

        def done(status, result):
            with self.lock:
                if generation != self.generation:
                    return
                if self.failure_reason and not self.user_cancel and status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
                    status = GoalStatus.ABORTED
                self.terminal = status
            # Never call the executor under this lock: operator cancel takes
            # the executor lock before entering this adapter.
            try:
                done_cb(status, result)
            finally:
                finished.set()

        self.client.send_goal(goal, done_cb=done, feedback_cb=feedback)

    def wait_for_result(self):
        # actionlib's timed wait uses ROS time; a paused /clock must not
        # suspend this wall-clock watchdog or the cancellation deadline.
        while not self.finished.wait(0.2):
            cancel = False
            with self.lock:
                warning = ''
                if (self.terminal is None and self.cancel_at is None
                        and not self.user_cancel and not self.entry_region_reached):
                    now = self.clock()
                    if (self.entry_approach and self.entry_since is not None
                            and now-self.entry_since >= self.arrival_dwell_sec and self.entry_latest is not None
                            and now-self.entry_latest <= 0.5):
                        self.entry_region_reached = True
                        cancel = True
                    else:
                        warning = self.guard.failure(now)
                        self.warning_reason = warning
                        if warning in ('NAV_NO_PROGRESS', 'NAV_FEEDBACK_STALE'):
                            # End a goal whose controller result is missing or
                            # whose feedback can no longer justify continued
                            # motion. NavigationExecutor permits one bounded
                            # retry only for fresh-feedback no-progress.
                            self.failure_reason = warning
                            cancel = True
                    if cancel:
                        self.cancel_at = self.clock()
                expired = self.terminal is None and self.cancel_at is not None and self.clock()-self.cancel_at >= 5.0
            if warning:
                rospy.logwarn_throttle(5.0, 'Navigation progress: %s', warning)
            if cancel:
                reason = 'ARRIVAL_REGION_REACHED' if self.entry_region_reached else self.failure_reason
                rospy.logwarn('Navigation stop: %s; awaiting terminal acknowledgement', reason)
                self.client.cancel_goal()
            if expired:
                from mission_manager.navigation_executor import NavigationLifecycleError
                raise NavigationLifecycleError('move_base did not acknowledge goal cancellation within 5 seconds')
            if rospy.is_shutdown():
                return False
        return True

    def wait_for_navigation_ready(self, cancelled):
        """Wait for fresh TF after terminal failure, without sending a goal."""
        import tf2_ros
        from .navigation_recovery import wait_for_recovery
        if not hasattr(self, '_recovery_buffer'):
            self._recovery_buffer = tf2_ros.Buffer()
            self._recovery_listener = tf2_ros.TransformListener(self._recovery_buffer)
        def fresh():
            try:
                transform = self._recovery_buffer.lookup_transform('map', 'base_Link', rospy.Time(0))
                age = (rospy.Time.now()-transform.header.stamp).to_sec()
                return 0 <= age <= 0.5
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return False
        rospy.logwarn('NAV waiting for fresh map/base TF before the single retry')
        ok = wait_for_recovery(fresh, lambda: cancelled() or rospy.is_shutdown())
        if not ok and not cancelled():
            self.failure_reason = 'NAV_TF_RECOVERY_TIMEOUT'
        return ok

    def get_state(self):
        with self.lock:
            return self.terminal if self.terminal is not None else self.client.get_state()

    def cancel_goal(self):
        with self.lock:
            self.user_cancel = True
            self.entry_region_reached = False
            if self.cancel_at is None:
                self.cancel_at = self.clock()
        self.client.cancel_goal()


def configure_floor_speed(floor_id):
    """Use the original linear limit outside RF; never compound the scale."""
    global _nav_limit_revision
    from dynamic_reconfigure.msg import DoubleParameter, StrParameter
    from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
    from multifloor_manager.ros_services import BoundedServiceCaller
    base = float(rospy.get_param('~nav_default_max_vel_x', 0.30))
    scale = float(rospy.get_param('~nav_rooftop_speed_scale', 0.70)) if floor_id == 'RF' else 1.0
    limit = base*scale
    if not math.isfinite(limit) or limit <= 0:
        raise ValueError('NAV linear speed must be positive and finite')
    # Reserve command ordering before either RPC; a delayed earlier call must
    # not receive a newer revision after a later floor request has completed.
    with _nav_limit_lock:
        _nav_limit_revision = revision = max(time.time_ns(), _nav_limit_revision+1)
    name = '/move_base/TrajectoryPlannerROS/set_parameters'
    request = ReconfigureRequest()
    request.config.doubles = [DoubleParameter(name='max_vel_x', value=limit)]
    # Only change max_vel_x; steering and stair commands have their own limits.
    try:
        caller = BoundedServiceCaller(2.0)
        caller.call(name, rospy.ServiceProxy(name, Reconfigure), request)
        name = '/stair_supervisor/set_navigation_limits'
        receiver_request = ReconfigureRequest()
        receiver_request.config.doubles = [DoubleParameter(name='max_vel_x', value=limit)]
        receiver_request.config.strs = [StrParameter(name='nav_limit_revision', value=str(revision))]
        caller.call(name, rospy.ServiceProxy(name, Reconfigure), receiver_request)
    except Exception as error:
        from mission_manager.navigation_executor import NavigationLifecycleError
        raise NavigationLifecycleError('NAV speed configuration failed: '+str(error)) from error
