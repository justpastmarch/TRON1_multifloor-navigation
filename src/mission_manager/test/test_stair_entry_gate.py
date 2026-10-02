"""Unit tests for the navigation-to-stair map-pose admission gate."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "mission_manager" / "src"))

from mission_manager.stair_entry_gate import (  # noqa: E402
    StairEntryCheck,
    StairEntryEvidence,
    StairEntryFence,
    StairEntryPolicy,
    StairEntryPose,
    evaluate_stair_entry,
)


POLICY = StairEntryPolicy(
    xy_tolerance_m=0.25,
    yaw_tolerance_rad=0.20,
    freshness_sec=0.50,
    max_covariance_x=0.05,
    max_covariance_y=0.05,
    max_covariance_yaw=0.10,
    required_pose_samples=3,
    max_linear_speed=0.01,
    max_angular_speed=0.02,
)
FENCE = StairEntryFence("3F", 7, 100.0, 20.0, 12)
EXPECTED = StairEntryPose(1.0, 2.0, math.pi - 0.10)


def evidence(**changes) -> StairEntryEvidence:
    values = {
        "frame_id": "map",
        "floor_id": "3F",
        "map_generation": 7,
        "pose_stamp_sec": 100.20,
        "pose_received_monotonic": 20.20,
        "pose_sequence": 15,
        "x_m": 1.0,
        "y_m": 2.0,
        "orientation_x": 0.0,
        "orientation_y": 0.0,
        "orientation_z": math.sin((-math.pi + 0.10) / 2.0),
        "orientation_w": math.cos((-math.pi + 0.10) / 2.0),
        "covariance_x": 0.05,
        "covariance_y": 0.05,
        "covariance_yaw": 0.10,
        "odom_received_monotonic": 20.20,
        "linear_speed_mps": 0.01,
        "angular_speed_radps": 0.02,
    }
    values.update(changes)
    return StairEntryEvidence(**values)


class StairEntryGateTest(unittest.TestCase):
    def test_aligned_fresh_pose_accepts_wrapped_heading_at_inclusive_bounds(self) -> None:
        # Given: three post-fence AMCL samples and stationary odometry at policy bounds.
        observed = evidence(x_m=1.25)

        # When: the live map pose is compared with the canonical stair entry.
        decision = evaluate_stair_entry(
            StairEntryCheck(POLICY, FENCE, EXPECTED, observed, 100.50, 20.50)
        )

        # Then: wrapped yaw and inclusive distance/covariance bounds are accepted.
        self.assertTrue(decision.accepted, decision.reason)

    def test_pose_or_localization_failure_rejects_entry(self) -> None:
        cases = (
            ("wrong frame", {"frame_id": "odom"}),
            ("wrong floor", {"floor_id": "4F"}),
            ("wrong generation", {"map_generation": 8}),
            ("pre-fence pose", {"pose_sequence": 12}),
            ("position mismatch", {"x_m": 1.251}),
            ("heading mismatch", {"orientation_z": math.sin(2.70 / 2.0), "orientation_w": math.cos(2.70 / 2.0)}),
            ("high covariance", {"covariance_x": 0.051}),
            ("negative covariance", {"covariance_y": -0.001}),
            ("moving", {"linear_speed_mps": 0.011}),
            ("stale odometry", {"odom_received_monotonic": 19.99}),
        )
        for reason, changes in cases:
            with self.subTest(reason=reason):
                # Given: exactly one unsafe entry-evidence class.
                observed = evidence(**changes)

                # When: admission is evaluated.
                decision = evaluate_stair_entry(
                    StairEntryCheck(POLICY, FENCE, EXPECTED, observed, 100.50, 20.50)
                )

                # Then: stair entry remains fail-closed.
                self.assertFalse(decision.accepted)





class EntryRefreshTest(unittest.TestCase):
    def test_waiting_entry_requests_nomotion_samples(self):
        import threading
        from unittest.mock import Mock, patch
        from mission_manager.ros_state import RosStateMonitor
        from mission_manager.stair_entry_gate import StairEntryDecision
        state = RosStateMonitor.__new__(RosStateMonitor)
        state._barrier_timeout = 2.0
        state._condition = threading.Condition()
        state.stair_entry_decision = Mock(side_effect=[StairEntryDecision(False, 'need samples'), StairEntryDecision(True, '')])
        with patch('rospy.get_param', return_value='/request_nomotion_update'), patch('rospy.ServiceProxy'), patch('rospy.is_shutdown', return_value=False), patch('multifloor_manager.ros_services.BoundedServiceCaller') as caller:
            self.assertTrue(state.wait_for_stair_entry(None, None).accepted)
            caller.return_value.call.assert_called_once()

    def test_refresh_failure_cannot_admit_stairs(self):
        import threading
        from unittest.mock import Mock, patch
        from mission_manager.ros_state import RosStateMonitor
        from mission_manager.stair_entry_gate import StairEntryDecision
        from multifloor_manager.ros_services import ServiceCallError
        state = RosStateMonitor.__new__(RosStateMonitor)
        state._barrier_timeout = 2.0
        state._condition = threading.Condition()
        state.stair_entry_decision = Mock(return_value=StairEntryDecision(False, 'need samples'))
        with patch('rospy.get_param', return_value='/request_nomotion_update'), patch('rospy.ServiceProxy'), patch('rospy.is_shutdown', return_value=False), patch('rospy.logwarn'), patch('multifloor_manager.ros_services.BoundedServiceCaller') as caller:
            caller.return_value.call.side_effect=ServiceCallError('refresh','timeout')
            self.assertFalse(state.wait_for_stair_entry(None, None).accepted)


class BoundedEntryMotionTest(unittest.TestCase):
    def rows(self, v=.02, w=.01):
        return [(10.+i*.01,v,w) for i in range(41)]

    def test_brief_recorded_rate_peak_does_not_erase_bounded_motion(self):
        from mission_manager.stair_entry_gate import entry_motion_window, LIDAR_ENTRY_POLICY
        rows=self.rows();rows[-1]=(10.4,.024809,-.043413)
        motion=entry_motion_window(rows,10.4)
        self.assertIsNotNone(motion)
        self.assertLess(motion[0],LIDAR_ENTRY_POLICY.max_linear_speed)
        self.assertLess(motion[1],LIDAR_ENTRY_POLICY.max_angular_speed)

    def test_reversals_do_not_hide_sustained_rotation(self):
        from mission_manager.stair_entry_gate import entry_motion_window, LIDAR_ENTRY_POLICY
        rows=[(10.+i*.01,.0,.19 if i%2 else -.19) for i in range(41)]
        self.assertGreater(entry_motion_window(rows,10.4)[1],LIDAR_ENTRY_POLICY.max_angular_speed)

    def test_gaps_duplicates_future_stale_and_large_impulses_rejected(self):
        from mission_manager.stair_entry_gate import entry_motion_window
        for rows,now in [(self.rows(),11.),(self.rows(),10.3),
                         ([(10.1,0,0),(10.4,0,0)],10.4),
                         ([(10.4,0,0)]*40,10.4),
                         (self.rows()+[(10.41,.3,0)],10.41),
                         (self.rows()+[(10.41,0,1.)],10.41),
                         (self.rows()+[(10.41,float('nan'),0)],10.41)]:
            with self.subTest(rows=rows[-1],now=now):self.assertIsNone(entry_motion_window(rows,now))

    def test_lidar_coarse_gate_accepts_recorded_first_failure_but_not_wrong_floor(self):
        from dataclasses import replace
        from mission_manager.stair_entry_gate import LIDAR_ENTRY_POLICY
        p=replace(LIDAR_ENTRY_POLICY,xy_tolerance_m=.5)
        observed=evidence(pose_sequence=13,covariance_x=.058648,covariance_y=.065873,
                          linear_speed_mps=.02,angular_speed_radps=.02)
        self.assertTrue(evaluate_stair_entry(StairEntryCheck(p,FENCE,EXPECTED,observed,100.7233,20.7233)).accepted)
        for key,value in [('floor_id','RF'),('map_generation',8),('x_m',2.),('pose_stamp_sec',98.)]:
            self.assertFalse(evaluate_stair_entry(StairEntryCheck(p,FENCE,EXPECTED,replace(observed,**{key:value}),100.7233,20.7233)).accepted)

if __name__ == "__main__":
    unittest.main()


class MeasurementWindowRegressionTest(unittest.TestCase):
    def test_transport_delay_does_not_erase_stationary_history(self):
        from mission_manager.stair_entry_gate import entry_motion_window
        rows=[(100.+i*.01,.01,.02) for i in range(61)]
        for delay in (0.,.055,.077,.11,.149):
            with self.subTest(delay=delay):
                value=entry_motion_window(rows,100.6+delay)
                self.assertIsNotNone(value)
                self.assertAlmostEqual(value[0],.01)
                self.assertAlmostEqual(value[1],.02)
    def test_frozen_or_future_samples_still_fail(self):
        from mission_manager.stair_entry_gate import entry_motion_window
        rows=[(100.+i*.01,0.,0.) for i in range(61)]
        for now in (100.751,101.,100.59,float('nan')):
            self.assertIsNone(entry_motion_window(rows,now))
    def test_delay_does_not_hide_sustained_motion_or_reversal(self):
        from mission_manager.stair_entry_gate import entry_motion_window,LIDAR_ENTRY_POLICY
        rows=[(100.+i*.01,.10*(-1 if i%2 else 1),0.) for i in range(61)]
        # Latest excessive speed is still rejected before averaging.
        self.assertIsNone(entry_motion_window(rows,100.677))
        rows=[(100.+i*.01,.07*(-1 if i%2 else 1),0.) for i in range(61)]
        self.assertGreater(entry_motion_window(rows,100.677)[0],LIDAR_ENTRY_POLICY.max_linear_speed)
    def test_missing_history_and_out_of_order_samples_still_fail(self):
        from mission_manager.stair_entry_gate import entry_motion_window
        full=[(100.+i*.01,0.,0.) for i in range(61)]
        for rows in (full[-10:], full[:30]+full[-5:], full+[full[-1]], full+[full[-2]]):
            self.assertIsNone(entry_motion_window(rows,100.677))
