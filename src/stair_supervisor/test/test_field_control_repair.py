"""Offline behavioral regressions. No ROS master or robot connection."""
from dataclasses import replace
import math
import unittest
from unittest.mock import patch
from unittest.mock import Mock
from types import SimpleNamespace
import json
from pathlib import Path
import numpy as np

from test_lidar_control import Worker, route, sample, box
from test_supervisor import make_configuration, FakeTransport
from stair_supervisor.stair_feedback import StairFeedback, RouteAnchor, se2
from stair_supervisor import stair_feedback
from stair_supervisor.stair_evidence import Phase


class FieldControlRepairTest(unittest.TestCase):
    def test_landing_hold_captures_pose_and_corrects_drift_at_flat_speed(self):
        w,c=self.policy();c.begin_phase(Phase.LANDING,100.1)
        w.value=sample(2,100.2,x=2.,y=.5,z=1.)
        c.begin_landing_hold(100.21)
        w.value=sample(3,100.4,x=1.85,y=.5,z=1.)
        result=c.evaluate(Phase.LANDING,100.41,command_required=True)
        self.assertFalse(result.faulted)
        self.assertGreater(c.command()[0],0.)
        self.assertLessEqual(c.command()[0],c.route['limits']['hold_v'])
        self.assertAlmostEqual(c.debug['target'][1],.5)
        self.assertTrue(c.debug['landing_position_hold'])
        w.value=sample(4,500.,x=2.,y=.5,z=1.)
        self.assertFalse(c.evaluate(Phase.LANDING,500.01,command_required=True).faulted)
        c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,500.1)
        self.assertIsNone(c._landing_hold_target)

    def test_landing_hold_rejects_step_height_edge_stale_and_epoch(self):
        for mode in ('height','edge','stale','epoch'):
            w,c=self.policy();c.begin_phase(Phase.LANDING,100.1)
            w.value=sample(2,100.2,x=1.5 if mode=='edge' else 2.,
                           z=.5 if mode=='height' else 1.,epoch=1 if mode=='epoch' else 0)
            with self.assertRaises(ValueError):c.begin_landing_hold(101. if mode=='stale' else 100.21)

    def test_landing_hold_never_recovers_onto_flight_and_checks_surface_height(self):
        for mode in ('edge','height','small_drift'):
            w,c=self.policy();c.begin_phase(Phase.LANDING,100.1)
            w.value=sample(2,100.2,x=2.,z=1.);c.begin_landing_hold(100.21)
            w.value=sample(3,100.4,x=1.55 if mode=='edge' else 1.98,
                           z=.8 if mode=='height' else 1.)
            report=c.evaluate(Phase.LANDING,100.41,command_required=True)
            if mode=='small_drift':
                self.assertFalse(report.faulted)
                self.assertGreater(c.command()[0],0.)
                self.assertEqual(c.debug['support_regions'],['landing_polygon'])
            else:self.assertTrue(report.faulted)

    def test_yaw_rate_damping_is_bounded_and_requires_fresh_gyro(self):
        r = route(); r['limits'].update(flight_yaw_kd=.06, flight_damping_max_w=.05,
            flight_gyro_rate_max_age=.12, flight_capture_angle=.17)
        _, c = self.policy(r)
        base, _ = c._flight_steering(.1, 0., {})
        w, detail = c._flight_steering(.1, 0., dict(yaw_rate=-3., yaw_rate_age_sec=.05))
        self.assertAlmostEqual(w-base, .05)
        self.assertAlmostEqual(detail['damping_w'], .05)
        for age in (-.1, .13, math.inf):
            self.assertEqual(c._flight_steering(.1, 0., dict(yaw_rate=-3., yaw_rate_age_sec=age))[0], base)
        self.assertLessEqual(abs(c._flight_steering(0., 10., {})[0]), .17)

    def test_neutralization_does_not_accelerate_reverse_steering_or_landing(self):
        r=route(); r['limits'].update(max_alpha=.3, flight_neutral_alpha=.9)
        _,c=self.policy(r);c._last_command=(.3,-.1);c._last_command_at=100.
        self.assertAlmostEqual(c._limit_command(.3,.2,100.1)[1],-.01)
        self.assertAlmostEqual(c._limit_command(.3,.2,100.2)[1],.3*(.2-.1/.9))
        c.phase=Phase.LANDING
        self.assertAlmostEqual(c._limit_command(.3,.2,100.1)[1],-.07)

    def test_gyro_heading_rate_matches_tilted_rotation_derivative(self):
        from scipy.spatial.transform import Rotation
        _,c=self.policy()
        c.base_from_lidar[:3,:3]=Rotation.from_euler('xyz',[.03,.087,.02]).as_matrix()
        c._lidar_from_imu=Rotation.from_euler('xyz',[-.02,.04,-.13]).as_matrix()
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('xyz',[.2,.4,.1]).as_matrix()
        for t in np.arange(100.,100.111,.01):c.push_imu((t,.3,.4,.7,0.,0.,9.81))
        s=sample(1,100.)
        _,d=c._control_heading(s,pose,100.1)
        omega=c._lidar_from_imu @ np.array([.3,.4,.7])
        base=c.base_from_lidar[:3,:3]
        R=pose[:3,:3] @ base @ Rotation.from_rotvec(omega*d['gyro_advanced_sec']).as_matrix() @ base.T
        R2=pose[:3,:3] @ base @ Rotation.from_rotvec(omega*(d['gyro_advanced_sec']+1e-5)).as_matrix() @ base.T
        derivative=(math.atan2(R2[1,0],R2[0,0])-math.atan2(R[1,0],R[0,0]))/1e-5
        self.assertAlmostEqual(d['yaw_rate'],derivative,places=4)

    def test_commissioned_recovery_turns_only_with_checked_inward_motion(self):
        r=route();r['limits'].update(flight_recheck_sec=1., flight_restart_v=.3, flight_recovery_w=.06)
        w,c=self.policy(r)
        with patch.object(c,'_flight_command',return_value=None):
            w.value=sample(2,100.2,x=.2,yaw=.2)
            self.assertFalse(c.evaluate(c.phase,100.21).faulted)
            self.assertEqual(c.command(),(0.,0.))
            w.value=sample(3,100.4,x=.2,yaw=.2)
            self.assertFalse(c.evaluate(c.phase,100.41).faulted)
            self.assertEqual(c.command()[0],0.)
            self.assertLess(c.command()[1],0.)
            self.assertTrue(c.debug['recovery_steering'])
            self.assertFalse(c.evaluate(c.phase,100.45).faulted)
            self.assertLess(c.command()[1],0.)
            w.value=sample(4,100.6,x=10.,yaw=.2)
            self.assertTrue(c.evaluate(c.phase,100.61).faulted)

    def test_commissioned_recovery_budget_and_uncommissioned_default(self):
        r=route();r['limits'].update(flight_recheck_sec=.5,flight_restart_v=.3,flight_recovery_w=.06)
        w,c=self.policy(r)
        with patch.object(c,'_flight_command',return_value=None):
            for i,t in enumerate((100.2,100.4,100.6)):
                w.value=sample(i+2,t,x=.2,yaw=.2)
                self.assertFalse(c.evaluate(c.phase,t+.01).faulted)
            w.value=sample(6,100.8,x=.2,yaw=.2)
            self.assertTrue(c.evaluate(c.phase,100.81).faulted)
        r['limits'].pop('flight_recovery_w');w,c=self.policy(r)
        with patch.object(c,'_flight_command',return_value=None):
            for i,t in enumerate((100.2,100.4)):
                w.value=sample(i+2,t,x=.2,yaw=.2);c.evaluate(c.phase,t+.01)
                self.assertEqual(c.command(),(0.,0.))

    def test_optional_recovery_never_overrides_neutral_interrupt_or_new_epoch(self):
        for mode in ('neutral', 'interrupt', 'epoch', 'stale'):
            with self.subTest(mode=mode):
                r=route();r['limits'].update(flight_recheck_sec=1.,flight_restart_v=.3,flight_recovery_w=.06)
                w,c=self.policy(r)
                with patch.object(c,'_flight_command',return_value=None):
                    w.value=sample(2,100.2,x=.2,yaw=.2);c.evaluate(c.phase,100.21)
                    w.value=sample(3,100.4,x=.2,yaw=.2);c.evaluate(c.phase,100.41)
                    self.assertLess(c.command()[1],0.)
                    if mode=='interrupt':c._interrupted=True
                    if mode=='epoch':w.epoch=1
                    result=c.evaluate(c.phase,100.75 if mode=='stale' else 100.45,neutral_only=mode=='neutral')
                    if mode in ('interrupt','epoch'):self.assertTrue(result.faulted)
                    else:self.assertEqual(c.command(),(0.,0.))

    def input_policy(self):
        r = route()
        r['limits'].update(flight_recheck_sec=1., flight_restart_v=.3, flight_input_recover_sec=1.)
        w, c = self.policy(r)
        w.value = sample(2, 100.2, x=.2)
        c.evaluate(c.phase, 100.21)
        return w, c

    def test_input_delay_waits_neutral_then_checks_exact_restart(self):
        w, c = self.input_policy()
        self.assertFalse(c.evaluate(c.phase, 101.).faulted)
        self.assertEqual(c.command(), (0., 0.))
        self.assertFalse(c.evaluate(c.phase, 101.2).faulted)
        w.value = sample(3, 101.25, x=.22)
        with patch.object(c, '_swept_support', wraps=c._swept_support) as check:
            report = c.evaluate(c.phase, 101.3)
        self.assertFalse(report.faulted)
        self.assertEqual(c.command()[0], .3)
        self.assertTrue(c.debug['input_recovery_resumed'])
        self.assertTrue(check.call_args_list)
        self.assertTrue(all(call.args[3] == .3 for call in check.call_args_list))

    def test_input_recovery_rejects_old_invalid_unsafe_and_late_samples(self):
        for mode in ('same', 'invalid', 'delayed', 'edge', 'epoch', 'interrupt', 'late', 'worker'):
            with self.subTest(mode=mode):
                w, c = self.input_policy()
                c.evaluate(c.phase, 101.)
                w.value = sample(2 if mode == 'same' else 3,
                                 100.9 if mode == 'delayed' else 101.25,
                                 x=10. if mode == 'edge' else .2,
                                 epoch=1 if mode == 'epoch' else 0,
                                 valid=mode != 'invalid')
                if mode == 'interrupt': c._interrupted = True
                if mode == 'worker': w.error = 'worker failed'
                report = c.evaluate(c.phase, 102.1 if mode == 'late' else 101.3)
                if mode in ('same', 'invalid', 'delayed'):
                    self.assertFalse(report.faulted)
                    self.assertEqual(c.command(), (0., 0.))
                else: self.assertTrue(report.faulted)

    def test_input_recovery_cumulative_budget_and_no_ineffective_restart(self):
        w, c = self.input_policy()
        c.evaluate(c.phase, 101.)
        w.value = sample(3, 101.55, x=.2)
        c.evaluate(c.phase, 101.6)
        c.evaluate(c.phase, 102.3)
        self.assertTrue(c.evaluate(c.phase, 102.71).faulted)
        w, c = self.input_policy()
        c.evaluate(c.phase, 101.)
        w.value = sample(3, 101.25, x=.2)
        with patch.object(c, '_flight_command', return_value=None):
            self.assertFalse(c.evaluate(c.phase, 101.3).faulted)
        self.assertEqual(c.command(), (0., 0.))

    def test_input_recovery_requires_bounded_budget_and_explicit_restart(self):
        for changes in ({'flight_input_recover_sec': .5},
                        {'flight_input_recover_sec': 3., 'flight_recheck_sec': 1., 'flight_restart_v': .3}):
            r = route(); r['limits'].update(changes)
            with self.assertRaisesRegex(ValueError, 'flight_input_recover_sec'):
                StairFeedback(Worker(), np.eye(4), [r])

    def test_flight_end_height_conflict_stops_ascent_without_claiming_arrival(self):
        for phase, position in ((Phase.FORWARD_SEGMENT_1, dict(x=2., z=.8)),
                                (Phase.FORWARD_SEGMENT_2, dict(x=0., y=1., z=1.8, yaw=math.pi))):
            with self.subTest(phase=phase):
                w, c = self.policy()
                c.begin_phase(phase, 100.)
                c._entry_alignment_pending = False
                c._last_command = (.3, .1)
                w.value = sample(2, 100.2, **position)
                report = c.evaluate(phase, 100.21)
                self.assertFalse(report.faulted)
                self.assertFalse(report.complete)
                self.assertEqual(c.command(), (0., 0.))
                self.assertTrue(c.debug['flight_end_height_conflict'])
                self.assertIn('height conflict', report.detail)

    def test_conflict_does_not_reaccelerate_on_position_jitter_and_clears_with_evidence(self):
        w, c = self.policy()
        w.value = sample(2, 100.2, x=2., z=.8)
        c.evaluate(c.phase, 100.21)
        w.value = sample(3, 100.4, x=1.99, z=.8)
        c.evaluate(c.phase, 100.41)
        self.assertEqual(c.command(), (0., 0.))
        w.value = sample(4, 100.6, x=2., z=1.)
        self.assertTrue(c.evaluate(c.phase, 100.61).complete)

    def test_last_riser_before_horizontal_end_keeps_existing_ascent(self):
        w, c = self.policy()
        w.value = sample(2, 100.2, x=1.8, z=.8)
        report = c.evaluate(c.phase, 100.21)
        self.assertFalse(report.complete)
        self.assertFalse(c.debug['flight_end_height_conflict'])
        self.assertGreater(c.command()[0], 0.)

    def test_end_conflict_keeps_expiry_support_and_phase_reset(self):
        for failure in ('expired', 'edge', 'interrupt', 'new_phase'):
            with self.subTest(failure=failure):
                w, c = self.policy()
                w.value = sample(2, 100.2, x=2., z=.8)
                c.evaluate(c.phase, 100.21)
                if failure == 'new_phase':
                    c.begin_phase(Phase.FORWARD_SEGMENT_2, 100.3)
                    self.assertFalse(c._flight_end_conflict)
                    continue
                if failure == 'edge':
                    w.value = sample(3, 100.4, x=4., z=.8)
                if failure == 'interrupt':
                    c._interrupted = True
                report = c.evaluate(c.phase, 101. if failure == 'expired' else 100.41)
                self.assertTrue(report.faulted)

    def test_opt_in_restart_checks_and_sends_useful_input_on_both_flights(self):
        for phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2):
            with self.subTest(phase=phase):
                r = route(); r['limits'].update(flight_recheck_sec=1., flight_restart_v=.3)
                w, c = self.policy(r)
                c.begin_phase(phase, 100.)
                c._entry_alignment_pending = False
                location = dict(x=.2) if phase is Phase.FORWARD_SEGMENT_1 else dict(x=1., y=1., z=1., yaw=math.pi)
                w.value = sample(2, 100.2, **location)
                with patch.object(c, '_flight_command', return_value=None):
                    self.assertFalse(c.evaluate(c.phase, 100.21).faulted)
                self.assertEqual(c.command(), (0., 0.))
                w.value = sample(3, 100.3, **location)
                with patch.object(c, '_swept_support', wraps=c._swept_support) as sweep:
                    report = c.evaluate(c.phase, 100.31)
                self.assertFalse(report.faulted)
                self.assertEqual(c.command()[0], .3)
                self.assertTrue(c.debug['flight_restart'])
                self.assertTrue(sweep.call_args_list)
                self.assertTrue(all(call.args[3] == .3 for call in sweep.call_args_list))
                self.assertLessEqual(abs(c.command()[1]), r['limits']['max_alpha'] * .1 + 1e-9)
                self.assertFalse(c.diagnostic_snapshot()['prediction_recheck_active'])

    def test_restart_does_not_fall_back_to_an_ineffective_low_input(self):
        r = route(); r['limits'].update(flight_recheck_sec=1., flight_restart_v=.3)
        w, c = self.policy(r)
        w.value = sample(2, 100.2, x=.2)
        with patch.object(c, '_flight_command', return_value=None):
            c.evaluate(c.phase, 100.21)
        w.value = sample(3, 100.3, x=.2)
        with patch.object(c, '_swept_support', side_effect=lambda *a: a[3] < .2) as sweep:
            report = c.evaluate(c.phase, 100.31)
        self.assertFalse(report.faulted)
        self.assertEqual(c.command(), (0., 0.))
        self.assertEqual(c.debug['tracking_state'], 'PREDICTION_RECHECK')
        self.assertTrue(sweep.call_args_list)
        self.assertTrue(all(call.args[3] == .3 for call in sweep.call_args_list))

    def test_restart_needs_new_fresh_geometry_and_never_overrides_neutral(self):
        for mode in ('same', 'delayed', 'invalid', 'neutral'):
            with self.subTest(mode=mode):
                r = route(); r['limits'].update(flight_recheck_sec=1., flight_restart_v=.3)
                w, c = self.policy(r)
                w.value = sample(2, 100.2, x=.2)
                with patch.object(c, '_flight_command', return_value=None):
                    c.evaluate(c.phase, 100.21)
                if mode != 'same':
                    w.value = sample(3, 100.3, x=.2, valid=mode != 'invalid')
                report = c.evaluate(c.phase, 100.65 if mode == 'delayed' else 100.31,
                                    neutral_only=mode == 'neutral')
                self.assertFalse(report.faulted)
                self.assertEqual(c.command(), (0., 0.))

    def test_restart_option_preserves_initial_ramp_and_landing_cap(self):
        r = route(); r['limits'].update(flight_recheck_sec=1., flight_restart_v=.3)
        w, c = self.policy(r)
        w.value = sample(2, 100.02, x=.2)
        report = c.evaluate(c.phase, 100.03)
        self.assertFalse(report.faulted)
        self.assertLess(c.command()[0], .3)
        c.begin_phase(Phase.LANDING, 100.1)
        c._last_command = (.3, 0.)
        self.assertLessEqual(c._limit_command(.3, 0., 100.11)[0], r['limits']['hold_v'])

    def test_restart_option_requires_bounded_useful_input_and_recheck(self):
        for changes in ({'flight_restart_v': .3},
                        {'flight_restart_v': .4, 'flight_recheck_sec': 1.},
                        {'flight_restart_v': .2, 'min_flight_v': .3, 'flight_recheck_sec': 1.}):
            r = route(); r['limits'].update(changes)
            with self.assertRaisesRegex(ValueError, 'flight_restart_v'):
                StairFeedback(Worker(), np.eye(4), [r])

    def test_feasible_proportional_steering_is_not_amplified(self):
        w, c = self.policy()
        c._last_command = (.2, 0.)
        c._last_command_at = 100.
        pos = np.array([.2, .01])
        body = np.asarray(c.route['footprint']) + pos
        selected = c._flight_command(pos, .02, .02, body, .2, -.01, .6,
            [c.route['flight_1_polygon']], .02, 100.2, np.array([1., 0.]), np.zeros(2), False)
        self.assertEqual(selected, (.2, -.01))

    def test_prediction_recheck_waits_for_new_fresh_scan_then_resumes_with_slew(self):
        r = route(); r['limits']['flight_recheck_sec'] = 1.
        w, c = self.policy(r)
        w.value = sample(2, 100.2, x=.2)
        c._last_command = (.2, 0.)
        with patch.object(c, '_flight_command', return_value=None):
            report = c.evaluate(c.phase, 100.21)
        self.assertFalse(report.faulted)
        self.assertEqual(c.debug['tracking_state'], 'PREDICTION_RECHECK')
        self.assertEqual(c.command(), (0., 0.))
        self.assertIsNone(c.diagnostic_snapshot()['first_fault'])
        report = c.evaluate(c.phase, 100.25)
        self.assertFalse(report.faulted)
        self.assertEqual(c.command(), (0., 0.))
        w.value = sample(3, 100.3, x=.21)
        report = c.evaluate(c.phase, 100.31)
        self.assertFalse(report.faulted)
        self.assertGreater(c.command()[0], 0.)
        self.assertLessEqual(c.command()[0], r['limits']['max_accel'] * .06 + 1e-9)

    def test_recheck_budget_accumulates_across_repeated_prediction_failures(self):
        r = route(); r['limits']['flight_recheck_sec'] = .3
        w, c = self.policy(r)
        for seq, stamp, fail in ((2, 100.1, True), (3, 100.3, False),
                                 (4, 100.4, True), (5, 100.6, True)):
            w.value = sample(seq, stamp, x=.2)
            if fail:
                with patch.object(c, '_flight_command', return_value=None):
                    report = c.evaluate(c.phase, stamp + .01)
            else:
                report = c.evaluate(c.phase, stamp + .01)
        self.assertTrue(report.faulted)
        self.assertIn('budget exhausted', report.detail)

    def test_recheck_without_new_gyro_still_obeys_budget(self):
        r = route(); r['limits']['flight_recheck_sec'] = .3
        w, c = self.policy(r)
        w.value = sample(2, 100.2, x=.2)
        with patch.object(c, '_flight_command', return_value=None):
            self.assertFalse(c.evaluate(c.phase, 100.21).faulted)
        report = c.evaluate(c.phase, 100.52)
        self.assertTrue(report.faulted)
        self.assertIn('budget exhausted', report.detail)

    def test_recheck_budget_cannot_exceed_recovery_limit(self):
        r = route()
        r['limits']['flight_recheck_sec'] = r['limits']['recover_sec'] + .1
        with self.assertRaisesRegex(ValueError, 'flight_recheck_sec'):
            StairFeedback(Worker(), np.eye(4), [r])

    def test_prediction_recheck_keeps_expiry_attitude_and_support_faults(self):
        for mode in ('expired', 'edge', 'attitude', 'interrupt'):
            with self.subTest(mode=mode):
                r = route(); r['limits']['flight_recheck_sec'] = 1.
                w, c = self.policy(r)
                w.value = sample(2, 100.2, x=.2)
                with patch.object(c, '_flight_command', return_value=None):
                    self.assertFalse(c.evaluate(c.phase, 100.21).faulted)
                if mode == 'edge': w.value = sample(3, 100.3, x=10.)
                if mode == 'attitude':
                    from scipy.spatial.transform import Rotation
                    matrix = se2(.2, 0., 0.)
                    matrix[:3, :3] = Rotation.from_euler('x', .8).as_matrix()
                    w.value = replace(sample(3, 100.3), transform=tuple(matrix.flat))
                if mode == 'interrupt': c.interrupt()
                report = c.evaluate(c.phase, 101. if mode == 'expired' else 100.31)
                self.assertTrue(report.faulted)

    def test_recheck_cannot_accept_gyro_rotated_footprint_outside_margin(self):
        r = route(); r['limits']['flight_recheck_sec'] = 1.
        r['flight_1_polygon'] = box(-1., -.16, 3., .16)
        r['entry_polygon'] = box(-1., -.16, 0., .16)
        w, c = self.policy(r)
        w.value = sample(2, 100.2, x=.2, y=.01)
        with patch.object(c, '_control_heading', return_value=(math.pi/4, {'source':'causal_gyro'})), \
             patch.object(c, '_flight_command', return_value=None):
            report = c.evaluate(c.phase, 100.21)
        self.assertTrue(report.faulted)
        self.assertIn('lost supported footprint', report.detail)

    def policy(self, r=None):
        w=Worker(); c=StairFeedback(w,np.eye(4),[r or route()],lidar_from_imu=np.eye(3))
        c.capture_entry('test_up',np.eye(4),.01,'synthetic fixture',100.)
        c.arm(replace(make_configuration().profiles[0],timeout_sec=300.),100.)
        c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        return w,c

    def test_gyro_uses_received_past_samples_and_preserves_geometry_stamp(self):
        w,c=self.policy(); w.value=sample(2,100.2,x=.1)
        times=np.arange(100.19,100.601,.01)
        for t in times: c.push_imu((t,0.,0.,1.,0.,0.,9.81))
        c.evaluate(c.phase,100.4)
        d=c.diagnostic_snapshot()
        self.assertAlmostEqual(d['control_yaw'],max(t for t in times if t<=100.4)-100.2,places=6)
        self.assertEqual(d['control_heading']['geometry_stamp'],100.2)
        self.assertAlmostEqual(d['control_heading']['geometry_age_sec'],.2)
        self.assertLess(c.command()[1],0)

    def test_gyro_gap_and_epoch_cannot_create_fake_heading(self):
        for mode in ('gap','epoch'):
            w,c=self.policy(); w.value=sample(2,100.2,x=.1)
            c.push_imu((100.19,0.,0.,5.,0.,0.,9.81))
            c.push_imu((100.4,0.,0.,5.,0.,0.,9.81))
            if mode=='epoch':c._imu_epoch=3
            c.evaluate(c.phase,100.41)
            self.assertEqual(c.debug['control_heading']['source'],'geometry')

    def test_degraded_scan_cannot_accelerate_or_complete_but_new_gyro_can_steer(self):
        w,c=self.policy();w.value=sample(2,100.05,x=.1)
        c.evaluate(c.phase,100.06);v=c.command()[0]
        for t in np.arange(100.04,100.501,.01):c.push_imu((t,0.,0.,.5,0.,0.,9.81))
        report=c.evaluate(c.phase,100.5)
        self.assertFalse(report.complete);self.assertFalse(report.faulted)
        self.assertEqual(c.debug['tracking_state'],'DEGRADED')
        self.assertLessEqual(c.command()[0],v)
        self.assertLess(c.command()[1],0)
        self.assertTrue(c.evaluate(c.phase,100.8).faulted)

    def test_degraded_landing_cannot_restore_old_high_flight_command(self):
        w,c=self.policy();c._last_command=(.3,0.);c._last_command_at=100.2
        c.begin_phase(Phase.LANDING,100.2)
        w.value=sample(2,100.2,x=2.,z=1.)
        result=c.evaluate(c.phase,100.6)
        self.assertFalse(result.faulted);self.assertFalse(result.complete)
        self.assertLessEqual(abs(c.command()[0]),c.route['limits']['hold_v'])

    def test_full_forward_has_multiple_reachable_steering_candidates(self):
        w,c=self.policy();c._last_command=(.12,0.);c._last_command_at=100.
        w.value=sample(2,100.2,x=.2,y=.1,yaw=.1)
        c.evaluate(c.phase,100.21)
        ds=c.debug['candidates']
        self.assertGreater(len({x['command'][1] for x in ds}),1)
        self.assertTrue(all(abs(x['command'][1]) <= .42+1e-8 for x in ds))
        self.assertGreater(c.command()[0],0.)
        self.assertLess(c.command()[1],0.)

    def test_no_margin_recovery_outside_anchor_uncertainty(self):
        r=route();r['flight_1_polygon']=box(-1,-.3,3,.3);r['entry_polygon']=box(-1,-.3,0,.3)
        w,c=self.policy(r);w.value=sample(2,100.2,x=1,y=.195)
        result=c.evaluate(c.phase,100.21)
        self.assertTrue(result.faulted)
        self.assertEqual(c.debug['tracking_state'],'CORRIDOR_LIMIT')

    def test_extra_margin_can_be_recovered_only_with_inward_supported_path(self):
        w,c=self.policy();c._last_command=(.1,0.);c._last_command_at=100.
        # Synthetic narrow strip: 15 mm actual clearance; 10 mm anchor bound;
        # normal margin 20 mm. Heading points away from the nearest boundary.
        pos=np.array([0.,.085]);yaw=-.2
        body=np.array(box(-.1,-.1,.1,.1))+pos
        command=c._flight_command(pos,yaw,yaw,body,.1,0.,.7,[box(-1,-.2,1,.2)],.02,
                                  100.3,np.array([1.,0.]),np.zeros(2),True)
        self.assertIsNotNone(command)
        c._last_command=(.1,.8);c._last_command_at=100.29
        command=c._flight_command(pos,.8,.8,body,.1,.8,.7,[box(-1,-.2,1,.2)],.02,
                                  100.3,np.array([1.,0.]),np.zeros(2),True)
        self.assertIsNone(command)

    def test_second_flight_starts_with_landing_alignment_at_hold_speed(self):
        w,c=self.policy();c.begin_phase(Phase.FORWARD_SEGMENT_2,100.)
        w.value=sample(2,100.2,x=2.3,y=.8,z=1,yaw=math.pi)
        result=c.evaluate(c.phase,100.21)
        self.assertFalse(result.faulted)
        self.assertTrue(c.debug['second_flight_alignment'])
        self.assertLessEqual(abs(c.command()[0]),c.route['limits']['hold_v'])
        self.assertFalse(result.complete)
        w.value=sample(3,100.4,x=2,y=1,z=1,yaw=math.pi)
        c.evaluate(c.phase,100.41)
        self.assertFalse(c.debug['second_flight_alignment'])

    def test_first_fault_survives_loss_and_rviz_command_is_zero_after_successful_send(self):
        w,c=self.policy();c._last_command=(.12,.2)
        w.value=sample(2,100.2,x=10)
        c.evaluate(c.phase,100.21)
        first=c.diagnostic_snapshot()['first_fault']
        c.loss_response(FakeTransport())
        c.evaluate(c.phase,100.3)
        self.assertEqual(c.diagnostic_snapshot()['first_fault'],first)
        self.assertEqual(c.diagnostic_snapshot()['command'],[0.,0.])
        self.assertEqual(first['command_before_fault'],[.12,.2])
        c.record_sent_twist({'x':0.,'y':0.,'z':0.},100.3)
        self.assertEqual(c.diagnostic_snapshot()['last_sent_twist']['normalized_xyz'],[0.,0.,0.])

    def test_failed_zero_send_does_not_claim_successful_neutralization(self):
        w,c=self.policy();c._last_command=(.12,.2)
        t=FakeTransport()
        def fail():raise OSError('synthetic failed send')
        t.send_current=fail
        with self.assertRaises(OSError):c.loss_response(t)
        self.assertEqual(c.command(),(.12,.2))

    def test_ros_observer_records_exact_normalized_successful_send(self):
        from stair_supervisor.ros_node import RosStairSupervisorNode
        _,c=self.policy()
        node=RosStairSupervisorNode.__new__(RosStairSupervisorNode)
        node._lidar=SimpleNamespace(control=c)
        node._clock=SimpleNamespace(monotonic=lambda:100.5)
        node._websocket_tx_publisher=Mock()
        frame=json.dumps({'title':'request_twist','data':{'x':1.,'y':0.,'z':.05}})
        node._publish_websocket_tx(frame)
        self.assertEqual(node._websocket_tx_publisher.publish.call_args[0][0].data,frame)
        sent=c.diagnostic_snapshot()['last_sent_twist']
        self.assertEqual(sent['normalized_xyz'],[1.,0.,.05])
        self.assertEqual(sent['sent_at'],100.5)

    def test_new_flat_phase_does_not_stall_between_hold_thresholds(self):
        w,c=self.policy();c.begin_phase(Phase.ALIGN,100.)
        w.value=sample(2,100.2,x=-.06)
        result=c.evaluate(c.phase,100.21)
        self.assertFalse(result.faulted)
        self.assertFalse(result.complete)
        self.assertGreater(c.command()[0],0.)

    def test_verify_entry_defers_exact_pose_to_align_but_keeps_height_gate(self):
        w, c = self.policy()
        c.begin_phase(Phase.VERIFY_ENTRY, 100.)
        for i in range(1, 8):
            stamp = 100. + i * .1
            w.value = sample(i + 1, stamp, x=.06, yaw=.17)
            report = c.evaluate(c.phase, stamp + .01)
        self.assertTrue(report.complete)
        self.assertTrue(c.debug['completion_checks']['body_in_region'])
        self.assertTrue(c.debug['completion_checks']['height'])

        c.begin_phase(Phase.ALIGN, 101.)
        w.value = sample(9, 101.1, x=.1, yaw=.17)
        report = c.evaluate(c.phase, 101.11)
        self.assertFalse(report.complete)
        self.assertFalse(c.debug['completion_checks']['position'])
        self.assertFalse(c.debug['completion_checks']['yaw'])

        c.begin_phase(Phase.VERIFY_ENTRY, 102.)
        for i in range(1, 8):
            stamp = 102. + i * .1
            w.value = sample(i + 10, stamp, z=.2)
            report = c.evaluate(c.phase, stamp + .01)
        self.assertFalse(report.complete)
        self.assertFalse(c.debug['completion_checks']['height'])

    def test_verified_entry_hands_off_recorded_align_poses_without_second_dwell(self):
        from stair_supervisor.configuration import load_lidar_configuration, load_stair_configuration
        root = Path(__file__).resolve().parents[1] / 'config'
        r = load_lidar_configuration(root / 'stair_lidar_3f_4f_test.yaml', 'control', False).document['routes'][0]
        profile = next(p for p in load_stair_configuration(root).profiles if p.id == r['id'])
        # First fresh ALIGN observations from the two 17:00 BAG attempts.
        for x, y, z, yaw in ((-.490906, -.054600, -.012801, -.015984),
                             (-.538531, -.000476, .000809, -.004867)):
            with self.subTest(x=x, y=y):
                w = Worker()
                w.value = sample(1, 100., x=-.45)
                c = StairFeedback(w, np.eye(4), [r])
                c.capture_entry(r['id'], se2(-.45, 0., 0.), .1, 'recorded BAG fixture', 100.)
                c.arm(profile, 100., phase_test=True)
                c.begin_phase(Phase.VERIFY_ENTRY, 100.)
                for i in range(1, 8):
                    stamp = 100. + i * .1
                    w.value = sample(i + 1, stamp, x=-.45)
                    verified = c.evaluate(c.phase, stamp + .01)
                self.assertTrue(verified.complete)
                c.begin_phase(Phase.ALIGN, 100.8)
                w.value = sample(9, 100.9, x=x, y=y, z=z, yaw=yaw)
                handoff = c.evaluate(c.phase, 100.91)
                self.assertFalse(handoff.faulted, handoff.detail)
                self.assertTrue(handoff.complete)
                self.assertLess(c.debug['settling']['samples'], 2)
                self.assertTrue(c.debug['completion_checks']['body_in_region'])
                self.assertTrue(c.debug['completion_checks']['height'])

                c.begin_phase(Phase.ALIGN, 101.)
                w.value = sample(10, 101.1, x=x, y=y, z=.2, yaw=.25)
                rejected = c.evaluate(c.phase, 101.11)
                self.assertFalse(rejected.complete)
                self.assertFalse(c.debug['completion_checks']['height'])
                self.assertFalse(c.debug['completion_checks']['yaw'])

                c.begin_phase(Phase.ALIGN, 101.2)
                w.value = sample(11, 101.3, x=-.335)
                contact = c.evaluate(c.phase, 101.31)
                self.assertTrue(contact.complete)
                self.assertTrue(c.debug['completion_checks']['body_in_region'])

    def test_alignment_corrects_drift_into_hold_hysteresis_gap(self):
        w, c = self.policy()
        c.begin_phase(Phase.ALIGN, 100.)
        w.value = sample(2, 100.2, x=0.)
        c.evaluate(c.phase, 100.21)
        w.value = sample(3, 100.4, x=-.06)
        report = c.evaluate(c.phase, 100.41)
        self.assertFalse(report.complete)
        self.assertFalse(c.debug['completion_checks']['position'])
        self.assertGreater(c.command()[0], 0.)

    def test_template_cannot_accept_an_upside_down_gravity_axis(self):
        from stair_supervisor import ros_lidar
        reference=dict(yaw_candidates=[0.],min_fitness=.5,max_rmse_m=.1,score_gap=.1,
                       validated_anchor_error_m=.01,sha256='synthetic')
        result=SimpleNamespace(accepted=True,fitness=1.,inlier_rmse_m=.01,
                               transform=np.diag([1.,-1.,-1.,1.]))
        from stair_supervisor.lidar_tracking import TrackingSettings
        args=(np.zeros((3,3)),sample(),route(),np.zeros((3,3)),reference,np.eye(4),TrackingSettings().registration)
        with patch.object(ros_lidar,'register_scan_to_map',return_value=result):
            with self.assertRaisesRegex(ValueError,'missing or ambiguous'):ros_lidar.fit_entry_template(*args)
            result.transform=np.eye(4)
            self.assertEqual(ros_lidar.fit_entry_template(*args).epoch,0)

    def test_enclosing_hull_fast_path_matches_per_body_checks_on_notches_and_seams(self):
        rng=np.random.RandomState(260926)
        polygons=[box(-1,-1,0,1),box(0,-1,1,.3),box(.5,.3,1,1)]
        for _ in range(100):
            position=rng.uniform(-.6,.6,2);yaw=rng.uniform(-math.pi,math.pi)
            support=np.array(box(-.12,-.12,.12,.12))+position
            args=(position,yaw,support,rng.uniform(-.3,.3),rng.uniform(-.8,.8),.9,polygons,.03)
            actual=StairFeedback._swept_support(*args)
            with patch.object(stair_feedback,'ConvexHull',side_effect=stair_feedback.QhullError('force original fallback')):
                expected=StairFeedback._swept_support(*args)
            self.assertEqual(actual,expected)

    def test_second_entry_alignment_converges_in_synthetic_motion_model(self):
        w,c=self.policy();c.begin_phase(Phase.FORWARD_SEGMENT_2,100.)
        x,y,yaw=2.3,.8,math.pi
        for i in range(440):
            stamp=100.+(i+1)*.1
            w.value=sample(i+2,stamp,x=x,y=y,z=1.,yaw=yaw)
            result=c.evaluate(c.phase,stamp+.01)
            self.assertFalse(result.faulted,result.detail)
            if not c.debug['second_flight_alignment']:break
            v,angular=c.command();x+=v*math.cos(yaw)*.1;y+=v*math.sin(yaw)*.1;yaw+=angular*.1
        self.assertFalse(c.debug['second_flight_alignment'])
        self.assertLessEqual(math.hypot(x-2.,y-1.),c.route['limits']['position_off'])


    def test_field_flight_preview_admits_recorded_impact_without_erasing_edge_gate(self):
        from stair_supervisor.configuration import load_lidar_configuration, load_stair_configuration
        from stair_supervisor.stair_feedback import inside_support_union
        root = Path(__file__).resolve().parents[1] / 'config'
        r = load_lidar_configuration(root / 'stair_lidar_3f_4f_test.yaml', 'control', False).document['routes'][0]
        self.assertEqual(r['limits']['flight_prediction_sec'], .6)
        self.assertEqual(r['limits']['expire_sec'], .9)
        self.assertAlmostEqual(r['limits']['margin_m'] + r['limits']['anchor_uncertainty_m'], .12)

        w = Worker()
        c = StairFeedback(w, np.eye(4), [r])
        c.route = r
        c.anchor = RouteAnchor(0, 0, 0., tuple(np.eye(4).flat), .1, 'recorded BAG fixture')
        c.phase = Phase.FORWARD_SEGMENT_1
        c._last_command = (.55, .057477776999257914)
        c._last_command_at = 100.
        position = np.array([.11246582710684579, .0028410434318959614])
        measured_yaw = -.02461087868529233
        gyro_yaw = -.43456505131060646
        co, si = math.cos(measured_yaw), math.sin(measured_yaw)
        footprint = np.asarray(r['footprint']) @ np.array([[co, si], [-si, co]])
        regions = [r[k] for k in ('entry_polygon', 'flight_1_polygon', 'landing_polygon')]
        start = np.asarray(r['flight_1'][0][:2])
        direction = np.asarray(r['flight_1'][1][:2]) - start
        direction /= np.linalg.norm(direction)
        def command(at, horizon):
            c._candidate_debug = []
            body = footprint + at
            return c._flight_command(at, measured_yaw, gyro_yaw, body,
                .55, .07011344309939886, horizon, regions, .12,
                100.04212, direction, start, False)

        self.assertIsNone(command(position, .9))
        selected = command(position, .6)
        self.assertIsNotNone(selected)
        self.assertAlmostEqual(selected[0], .55)
        self.assertTrue(inside_support_union(footprint + position, regions, .12))
        near_edge = np.array([position[0], -.1])
        self.assertTrue(inside_support_union(footprint + near_edge, regions, .12))
        self.assertIsNone(command(near_edge, .6))

        # A later measured BAG pose needs reachable steering, not an
        # instantaneous jump to max_w. The next longer preview rejects it.
        post_position = np.array([.283, -.056])
        post_yaw = -.448
        co, si = math.cos(post_yaw), math.sin(post_yaw)
        post_body = np.asarray(r['footprint']) @ np.array([[co, si], [-si, co]]) + post_position
        c._last_command = (.55, .13)
        c._last_command_at = 100.
        limited = c._limit_command(.55, .2, 100.025)
        self.assertLess(limited[1], r['limits']['max_w'])
        def post_command(horizon):
            c._candidate_debug = []
            return c._flight_command(post_position, post_yaw, post_yaw, post_body,
                *limited, horizon, regions, .12, 100.025, direction, start, False)
        self.assertIsNotNone(post_command(.6))
        self.assertIsNone(post_command(.65))

        w.value = sample(x=-.45)
        c = StairFeedback(w, np.eye(4), [r])
        c.capture_entry(r['id'], se2(-.45, 0., 0.), .1, 'synthetic start', 100.)
        profile = next(p for p in load_stair_configuration(root).profiles if p.id == r['id'])
        c.arm(profile, 100., phase_test=True)
        c.begin_phase(Phase.FORWARD_SEGMENT_1, 100.)
        w.value = sample(2, 100.2, x=position[0], y=position[1])
        result = c.evaluate(c.phase, 100.21)
        self.assertFalse(result.faulted, result.detail)
        self.assertEqual(c.debug['prediction_horizon_sec'], .6)
        self.assertAlmostEqual(c.debug['effective_margin_m'], .12)
        delayed = c.evaluate(c.phase, 100.75)
        self.assertFalse(delayed.faulted, delayed.detail)
        self.assertEqual(c.debug['tracking_state'], 'DEGRADED')
        self.assertEqual(c.debug['prediction_horizon_sec'], .6)

    def test_flight_preview_cannot_be_shorter_than_warning_or_longer_than_expiry(self):
        r = route()
        for preview in (.1, .8):
            r['limits']['flight_prediction_sec'] = preview
            with self.assertRaisesRegex(ValueError, 'flight_prediction_sec'):
                StairFeedback(Worker(), np.eye(4), [r])


if __name__=='__main__':unittest.main()
