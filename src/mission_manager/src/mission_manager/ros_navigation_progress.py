"""Bounded, goal-correlated progress adapter; creates no ROS node or cmd_vel."""

import math
import threading
import time

import rospy
from actionlib_msgs.msg import GoalStatus

from mission_manager.navigation_progress import NavigationProgress


class ProgressAwareClient:
    def __init__(self, client, clock=time.monotonic, policy=None, entry_region=(0.25, 0.2)):
        self.entry_region = entry_region
        self.default_entry_region = entry_region
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
            self.terminal, self.failure_reason, self.user_cancel = None, '', False
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
                if self.terminal is None and not self.user_cancel and not self.failure_reason and not self.entry_region_reached:
                    now = self.clock()
                    if (self.entry_approach and self.entry_since is not None
                            and now-self.entry_since >= 0.5 and self.entry_latest is not None
                            and now-self.entry_latest <= 0.5):
                        self.entry_region_reached = True
                        cancel = True
                    else:
                        self.failure_reason = self.guard.failure(now)
                        cancel = bool(self.failure_reason)
                    if cancel:
                        self.cancel_at = self.clock()
                expired = self.terminal is None and self.cancel_at is not None and self.clock()-self.cancel_at >= 5.0
            if cancel:
                rospy.logwarn('Navigation stop: %s', 'STAIR_ENTRY_REGION_REACHED; awaiting terminal acknowledgement' if self.entry_region_reached else self.failure_reason)
                self.client.cancel_goal()
            if expired:
                from mission_manager.navigation_executor import NavigationLifecycleError
                raise NavigationLifecycleError('move_base did not acknowledge goal cancellation within 5 seconds')
            if rospy.is_shutdown():
                return False
        return True

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
