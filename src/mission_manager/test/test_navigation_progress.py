import math
import unittest
import rospy
from unittest.mock import patch

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseFeedback
from mission_manager.navigation_progress import NavigationProgress
from mission_manager.ros_navigation_progress import ProgressAwareClient


class ProgressTest(unittest.TestCase):
    def test_slow_alignment_survives_more_than_old_ten_seconds(self):
        g = NavigationProgress((0, 0, 0), 0)
        for t in range(46):
            g.observe(0, 0, math.radians(90-2*t), t)
            self.assertEqual(g.failure(t), '')
        self.assertEqual(g.last_kind, 'heading_improved')

    def test_back_and_forth_yaw_does_not_renew_best_progress(self):
        g = NavigationProgress((0, 0, 0), 0)
        for t in range(31):
            g.observe(0, 0, math.radians(30 if t % 2 else 35), t)
        self.assertEqual(g.failure(30), 'NAV_NO_PROGRESS')

    def test_angle_wraparound_is_small_improvement(self):
        g = NavigationProgress((0, 0, math.radians(-179)), 0)
        g.observe(0, 0, math.radians(170), 0)
        g.observe(0, 0, math.radians(179), 19)
        self.assertEqual(g.failure(21), '')

    def test_translation_detour_is_progress_without_goal_distance_decrease(self):
        g = NavigationProgress((1, 0, 0), 0)
        for t in range(61):
            g.observe(0, t*.01, 0, t)
            self.assertEqual(g.failure(t), '')

    def test_far_goal_rotation_cannot_mask_stationary_stall(self):
        g = NavigationProgress((10, 0, 0), 0)
        for t in range(26):
            g.observe(0, 0, math.radians(90-t), t)
        self.assertEqual(g.failure(25), 'NAV_NO_PROGRESS')

    def test_stationary_noise_and_lost_feedback(self):
        g = NavigationProgress((0, 0, 0), 0)
        for t in range(22):
            g.observe(.001*(t%2), 0, .001*(t%2), t)
        self.assertEqual(g.failure(21), 'NAV_NO_PROGRESS')
        self.assertEqual(g.failure(32), 'NAV_FEEDBACK_STALE')


class Client:
    def __init__(self):
        self.cancelled = 0
        self.state = GoalStatus.ACTIVE
        self.on_wait = lambda: None
    def send_goal(self, goal, done_cb, feedback_cb):
        self.done, self.feedback = done_cb, feedback_cb
    def wait_for_result(self, duration):
        self.on_wait()
        return self.state != GoalStatus.ACTIVE
    def get_state(self): return self.state
    def cancel_goal(self):
        self.cancelled += 1
        self.finish(GoalStatus.PREEMPTED)
    def finish(self, state):
        self.state = state
        self.done(state, None)


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.raw = Client()
        self.client = ProgressAwareClient(self.raw, clock=lambda:self.now)
        self.results = []
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose.orientation.w = 1
        self.client.send_goal(self.goal, lambda s,r:self.results.append(s))

    def feedback(self, stamp=1, frame='map'):
        m = MoveBaseFeedback()
        m.base_position.header.frame_id = frame
        m.base_position.header.stamp.secs = stamp
        m.base_position.pose.orientation.w = 1
        self.raw.feedback(m)

    def test_watchdog_cancel_is_failure_not_operator_cancel(self):
        self.now = 11
        with patch('rospy.is_shutdown', return_value=False), patch('rospy.logwarn'), patch.object(self.client.finished, 'wait', side_effect=lambda _:self.client.terminal is not None):
            self.assertTrue(self.client.wait_for_result())
        self.assertEqual(self.raw.cancelled, 1)
        self.assertEqual(self.results, [GoalStatus.ABORTED])
        self.assertEqual(self.client.get_state(), GoalStatus.ABORTED)
        self.assertEqual(self.client.failure_reason, 'NAV_FEEDBACK_STALE')

    def test_user_cancel_remains_cancel(self):
        self.client.cancel_goal()
        self.assertEqual(self.results, [GoalStatus.PREEMPTED])

    def test_success_wins_watchdog_race(self):
        self.client.failure_reason = 'NAV_NO_PROGRESS'
        self.raw.finish(GoalStatus.SUCCEEDED)
        self.assertEqual(self.results, [GoalStatus.SUCCEEDED])

    def test_old_feedback_and_old_done_cannot_finish_new_goal(self):
        old_feedback, old_done = self.raw.feedback, self.raw.done
        self.client.send_goal(self.goal, lambda s,r:self.results.append(s))
        self.now = 5
        old_feedback(MoveBaseFeedback())
        old_done(GoalStatus.SUCCEEDED, None)
        self.assertIsNone(self.client.terminal)
        self.assertEqual(self.client.guard.last_feedback, 0)

    def test_repeated_stamp_or_wrong_frame_does_not_refresh_feedback(self):
        self.feedback()
        self.now = 5
        self.feedback()
        self.feedback(2, 'odom')
        self.assertEqual(self.client.guard.last_feedback, 0)

    def test_missing_cancel_ack_has_bounded_wait(self):
        from mission_manager.navigation_executor import NavigationLifecycleError
        self.raw.cancel_goal = lambda: None
        def tick(_):
            self.now += 1
            return False
        with patch('rospy.is_shutdown', return_value=False), patch('rospy.logwarn'), patch.object(self.client.finished, 'wait', side_effect=tick):
            with self.assertRaises(NavigationLifecycleError):
                self.client.wait_for_result()
        self.assertLessEqual(self.now, 16)


if __name__ == '__main__': unittest.main()


class EntryRegionTest(AdapterTest):
    def prepare_entry(self):
        self.client.set_entry_approach(True)
        self.client.send_goal(self.goal, lambda s,r:self.results.append(s))

    def sample(self, x=.20, yaw=.05, stamp=1, age=0):
        m = MoveBaseFeedback()
        m.base_position.header.frame_id = 'map'
        m.base_position.header.stamp = rospy.Time.from_sec(stamp)
        m.base_position.pose.position.x = x
        m.base_position.pose.orientation.z = math.sin(yaw/2)
        m.base_position.pose.orientation.w = math.cos(yaw/2)
        with patch('rospy.Time.now', return_value=rospy.Time.from_sec(stamp+age)):
            self.raw.feedback(m)

    def test_region_stop_keeps_preempted_terminal_for_executor(self):
        self.prepare_entry()
        self.sample()
        self.now=.6
        self.sample(stamp=2)
        with patch('rospy.is_shutdown', return_value=False), patch('rospy.logwarn'), patch.object(self.client.finished,'wait',side_effect=lambda _:self.client.terminal is not None):
            self.client.wait_for_result()
        self.assertTrue(self.client.entry_region_reached)
        self.assertEqual(self.raw.cancelled,1)
        self.assertEqual(self.results,[GoalStatus.PREEMPTED])

    def test_outside_heading_or_position_not_accepted(self):
        self.prepare_entry()
        for x,yaw in [(0.251,0),(.2,.201)]:
            self.sample(x=x,yaw=yaw,stamp=1+x+yaw)
            self.assertIsNone(self.client.entry_since)

    def test_stale_pose_and_region_exit_reset_dwell(self):
        self.prepare_entry()
        self.sample(age=2)
        self.assertIsNone(self.client.entry_since)
        self.sample(stamp=2)
        self.assertIsNotNone(self.client.entry_since)
        self.sample(x=.3,stamp=3)
        self.assertIsNone(self.client.entry_since)

    def test_operator_cancel_cannot_become_region_success(self):
        self.prepare_entry()
        self.client.entry_region_reached=True
        self.client.cancel_goal()
        self.assertFalse(self.client.entry_region_reached)
        self.assertEqual(self.results,[GoalStatus.PREEMPTED])

    def test_next_goal_resets_region_evidence(self):
        self.prepare_entry()
        self.client.entry_region_reached=True
        self.client.set_entry_approach(False)
        self.client.send_goal(self.goal,lambda *_:None)
        self.assertFalse(self.client.entry_region_reached)
        self.assertIsNone(self.client.entry_since)
