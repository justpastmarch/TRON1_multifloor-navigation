"""Software behavior checks using recorded estimates; not physical ground truth."""
import copy
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from dataclasses import replace
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

REPO = Path(os.environ['OBSERVED_FLIGHT_REPO']) if 'OBSERVED_FLIGHT_REPO' in os.environ else Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO/'src/stair_supervisor/test'))
from test_lidar_control import Worker, sample
from stair_supervisor.configuration import load_stair_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase


class ObservedFlight(unittest.TestCase):
    def setUp(self):
        path=Path(os.environ.get('OBSERVED_FLIGHT_CONFIG',str(REPO/'src/stair_supervisor/config/stair_lidar_3f_4f_test.yaml')))
        self.routes=[r for r in yaml.safe_load(path.read_text())['routes'] if r.get('direction')=='UP']
        self.profiles=load_stair_configuration(REPO/'src/stair_supervisor/config').profiles

    def armed(self, route=None):
        route=copy.deepcopy(route or self.routes[1]);w=Worker();w.value=sample(1,100.,x=-.3)
        c=StairFeedback(w,np.eye(4),[route]);c.capture_entry(route['id'],np.eye(4),.1,'recorded-estimate reconstruction',100.)
        c.arm(next(p for p in self.profiles if p.id==route['id']),100.01)
        c.begin_phase(Phase.FORWARD_SEGMENT_1,100.01)
        return w,c

    def fault_sample(self, w):
        # Exact recorded fault estimate from ui_test_20261001_222521_e861bc75.
        pose=np.eye(4)
        pose[:3,:3]=Rotation.from_euler('ZYX',[.13205067577745055,.05248759149309559,-.253006484215595]).as_matrix()
        pose[:3,3]=[1.225747731167522,.26232473533495526,.8903910451169336]
        w.value=replace(sample(376,100.2),transform=tuple(pose.flat))
        return 100.2+.6442685794318095

    def test_recorded_october_14_support_crossing_continues_only_with_inward_steering(self):
        # Reconstruct the 5F->RF first-flight estimate at 10:56:20.887 in
        # ui_test_20261014_105451_e7593518.bag. The body had 4.495 cm of
        # measured clearance; the old 6 cm test stopped it in one frame.
        route=copy.deepcopy(next(r for r in self.routes if r['id']=='stair_5f_rf_up'))
        def evaluate(lateral_offset=0., outward=False, recovery=True, latest=False):
            configured=copy.deepcopy(route)
            if not recovery:
                configured['limits'].pop('flight_margin_recovery_m')
            w=Worker();w.value=sample(1,100.)
            c=StairFeedback(w,np.eye(4),[configured])
            c.capture_entry(configured['id'],np.eye(4),.1,'recorded-estimate reconstruction',100.)
            c.arm(next(p for p in self.profiles if p.id==configured['id']),100.01)
            c.begin_phase(Phase.FORWARD_SEGMENT_1,100.01)
            pose=np.eye(4)
            if latest:
                pose[:3,:3]=Rotation.from_euler('xyz',[-.150237389,.109970798,-.054435836]).as_matrix()
                pose[:3,3]=[1.105703801,.322144407+lateral_offset,.770868508]
            else:
                pose[:3,:3]=Rotation.from_euler('xyz',[-.124719,.124891,.275296]).as_matrix()
                pose[:3,3]=[1.71317,.295466+lateral_offset,1.15771]
            w.value=replace(sample(2,100.2),transform=tuple(pose.flat))
            c._last_command=(.55,.0 if outward else -.128634427 if latest else -.2)
            c._last_command_at=100.1
            if outward:
                with patch.object(c,'_flight_steering',return_value=(.2,{})):
                    result=c.evaluate(c.phase,100.21)
            else:
                result=c.evaluate(c.phase,100.21)
            return result,c
        result,c=evaluate()
        self.assertFalse(result.faulted,c.debug)
        self.assertTrue(c.debug['flight_margin_recovery'])
        self.assertAlmostEqual(c.debug['clearance_m'],.0449504,places=4)
        self.assertAlmostEqual(c.debug['recovery_command_margin_m'],.03)
        self.assertEqual(c.command()[0],.55)
        self.assertLess(c.command()[1],0.)
        latest,c=evaluate(latest=True)
        self.assertFalse(latest.faulted,c.debug)
        self.assertTrue(c.debug['flight_margin_recovery'])
        self.assertAlmostEqual(c.debug['clearance_m'],.05835046,places=4)
        self.assertEqual(c.command()[0],.55)
        for kwargs in (dict(recovery=False),dict(outward=True),dict(lateral_offset=.03)):
            with self.subTest(**kwargs):
                stopped,_=evaluate(**kwargs)
                self.assertTrue(stopped.faulted)

    def test_recorded_old_center_with_new_gyro_heading_does_not_terminate_ascent(self):
        w,c=self.armed();now=self.fault_sample(w)
        c._last_command=(.55,-.04);c._last_command_at=now-.05
        detail={'source':'causal_gyro','yaw_rate':-.39293,'yaw_rate_age_sec':.0276}
        with patch.object(c,'_control_heading',return_value=(.2041439606587227,detail)):
            result=c.evaluate(c.phase,now)
        self.assertFalse(result.faulted,c.debug);self.assertFalse(result.complete)
        self.assertGreater(c.command()[0],0.);self.assertLessEqual(abs(c.command()[1]),.2)
        self.assertTrue(c.debug['footprint_supported']);self.assertTrue(c.debug['current_supported'])
        self.assertFalse(c.debug['margin_recovery'])
        self.assertFalse(c.debug['gyro_heading_affects_flight_steering'])
        self.assertEqual(c.debug['support_timestamp'],w.value.stamp)
        self.assertAlmostEqual(c.debug['epsi'],c.debug['yaw'])


    def test_observed_boundary_attitude_epoch_and_operator_interrupt_remain_effective(self):
        for cause in ('boundary','attitude','epoch','interrupt'):
            w,c=self.armed();w.value=sample(2,100.2,x=.5)
            if cause=='boundary':w.value=sample(2,100.2,x=.5,y=.65)
            if cause=='attitude':
                pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('x',.5).as_matrix();pose[0,3]=.5
                w.value=replace(w.value,transform=tuple(pose.flat))
            if cause=='epoch':w.epoch=1
            if cause=='interrupt':c.interrupt()
            result=c.evaluate(c.phase,100.21)
            self.assertTrue(result.faulted,(cause,c.debug))

    def test_legacy_prediction_metadata_is_ignored_and_observed_policy_is_default(self):
        for configured in self.routes:
            baseline = copy.deepcopy(configured)
            baseline.pop('flight_command_policy', None)
            legacy = copy.deepcopy(baseline)
            legacy['motion_prediction'] = {'mode': 'control', 'commissioned': True}
            legacy['limits']['flight_recheck_sec'] = .01
            legacy['limits']['flight_recovery_w'] = .05
            w0, c0 = self.armed(baseline)
            w1, c1 = self.armed(legacy)
            w0.value = w1.value = sample(2, 100.2, x=.2, y=.02)
            r0, r1 = c0.evaluate(c0.phase, 100.21), c1.evaluate(c1.phase, 100.21)
            self.assertEqual((r0.faulted, r0.complete), (r1.faulted, r1.complete))
            np.testing.assert_allclose(c0.command(), c1.command(), atol=1e-12)
            self.assertEqual(c1.debug['flight_command_policy'], 'observed_feedback')
            self.assertEqual(c1.debug['motion_prediction'], {'state': 'disabled', 'affects_commands': False})
            self.assertFalse(c1.diagnostic_snapshot()['prediction_recheck_active'])

    def test_continue_preserves_current_phase_turn_progress_and_pivot(self):
        w, c = self.armed()
        c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT, 100.1)
        path = np.array([[0., 0., 0.], [0.1, 0., 0.], [0.1, 0.1, math.pi/2]])
        c._landing_turn_path = path
        c._landing_turn_captured = True
        c.turn_index = 1
        c._flight_seen = [True] + [False] * (len(c._flight_seen) - 1)
        expected_flights = list(c._flight_seen)
        c._interrupted = True
        c._operator_completed_flights = {0}
        c._first_fault = {'sample_sequence': 1, 'reason': 'synthetic wait'}
        c._input_gap = {'at':100.1,'sequence':1};c._input_gap_used=.4
        c._recovery_events.append((99.9,100.1))
        w.value = sample(2,100.2,x=.2,y=.1,z=.3)
        anchor = c.anchor
        c.continue_phase(100.21)
        self.assertIs(c.anchor, anchor)
        self.assertIs(c.phase, Phase.TURN_TO_NEXT_FLIGHT)
        self.assertIs(c._landing_turn_path, path)
        self.assertTrue(c._landing_turn_captured)
        self.assertEqual(c.turn_index, 1)
        self.assertEqual(c._flight_seen, expected_flights)
        self.assertEqual(c._operator_completed_flights, {0})
        self.assertEqual(c._phase_sequence, 2);self.assertEqual(c._turn_sequence, 2)
        self.assertEqual(c.command(), (0.,0.));self.assertFalse(c._interrupted)
        self.assertIsNone(c._input_gap);self.assertEqual(c._input_gap_used,0.)
        self.assertFalse(c._recovery_events);self.assertIsNone(c._first_fault)

    def test_continue_requires_new_fresh_same_epoch_geometry(self):
        w, c = self.armed()
        c._first_fault = {'sample_sequence':1,'reason':'synthetic wait'}
        with self.assertRaisesRegex(ValueError,'새 LiDAR'):
            c.continue_phase(100.1)
        w.value = sample(2,100.2)
        w.epoch = 1
        with self.assertRaisesRegex(ValueError,'최신 LiDAR'):
            c.continue_phase(100.21)

    def test_prediction_gated_policy_is_rejected(self):
        route = copy.deepcopy(self.routes[1])
        route['flight_command_policy'] = 'prediction_gated'
        with self.assertRaisesRegex(ValueError, 'only observed_feedback'):
            StairFeedback._validate_route(route)

    def test_actual_input_loss_retains_last_command_then_exhausts_bounded_recovery(self):
        w,c=self.armed();w.value=sample(2,100.2,x=.3)
        c.evaluate(c.phase,100.21);self.assertGreater(c.command()[0],0.)
        last_command=c.command()
        first=w.value.stamp+c.route['limits']['expire_sec']+.01
        result=c.evaluate(c.phase,first);self.assertFalse(result.faulted);self.assertEqual(c.command(),last_command)
        result=c.evaluate(c.phase,first+c.route['limits']['flight_input_recover_sec']+.1)
        self.assertFalse(result.faulted,c.debug)
        self.assertEqual(c.debug['tracking_state'],'INPUT_REOBSERVING')
        self.assertEqual(c.command(),(0.,0.))
        result=c.evaluate(c.phase,first+c.route['limits']['flight_input_recover_sec']
                          +c.route['limits']['flight_input_reobserve_sec']+.1)
        self.assertTrue(result.faulted,c.debug)
        self.assertEqual(c.command(),(0.,0.))

    def test_flat_short_gap_preserves_phase_but_requires_fresh_geometry(self):
        w,c=self.armed()
        c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,100.01)
        w.value=sample(2,100.2,x=2.7,z=1.7)
        c.evaluate(c.phase,100.21)
        report=c.evaluate(c.phase,101.31)
        self.assertFalse(report.faulted,c.debug)
        self.assertFalse(report.complete)
        self.assertEqual(c.debug['tracking_state'],'FLAT_REOBSERVING')
        self.assertEqual(c.command(),(0.,0.))
        w.value=sample(3,101.4,x=2.7,z=1.7)
        report=c.evaluate(c.phase,101.41)
        self.assertFalse(report.faulted,c.debug)
        self.assertNotEqual(c.debug['tracking_state'],'FLAT_REOBSERVING')

    def test_correlated_xy_z_jump_cannot_complete_a_flight(self):
        w,c=self.armed()
        w.value=sample(2,100.2,x=.3,z=.3)
        c.evaluate(c.phase,100.21)
        w.value=sample(3,100.4,x=.65,z=.65)
        report=c.evaluate(c.phase,100.41)
        self.assertFalse(report.complete)
        self.assertFalse(report.faulted)
        self.assertEqual(c.debug['tracking_state'],'POSE_JUMP_CONFLICT')
        self.assertEqual(c.command(),(0.,0.))
        w.value=sample(4,102.5,x=.65,z=.65)
        report=c.evaluate(c.phase,102.51)
        self.assertTrue(report.faulted)
        self.assertIn('independent confirmation',report.detail)

    def test_repeated_short_input_gaps_use_rolling_budget(self):
        w,c=self.armed()
        c._last_command=(.55,0.)
        c._input_gap_used=3.9
        c._recovery_events.append((100.1,104.0))
        c._input_gap={'at':104.0,'sequence':1}
        result=c.evaluate(c.phase,104.2)
        self.assertFalse(result.faulted,c.debug)
        self.assertEqual(c.command(),(.55,0.))
        self.assertAlmostEqual(c.debug['input_recovery_elapsed_sec'],4.1)
        self.assertAlmostEqual(c.debug['input_recovery_gap_sec'],.2)
        self.assertGreater(c.debug['input_recovery_window_used_sec'],4.)
        self.assertLess(c.debug['input_recovery_window_used_sec'],c.debug['input_recovery_window_budget_sec'])

    def test_configured_stair_turn_and_recovery_budgets(self):
        for route in self.routes:
            with self.subTest(route=route['id']):
                self.assertAlmostEqual(route['limits']['flight_resume_corridor_yaw'],math.radians(20.))
                self.assertEqual(route['limits']['flight_input_recover_sec'],4.)
                self.assertEqual(route['limits']['flight_recovery_window_budget_sec'],8.)
                self.assertEqual(route['limits']['flight_recovery_window_sec'],10.)
                self.assertEqual(route['limits']['support_margin_m'],.06)

    def test_full_ascent_input_and_small_gyro_damping_remain_bounded(self):
        w,c=self.armed()
        for i in range(1,61):
            w.value=sample(i+1,100.+i*.1,x=.2,y=.03)
            with patch.object(c,'_control_heading',return_value=(.8,{'source':'causal_gyro','yaw_rate':-20.,'yaw_rate_age_sec':.01})):
                result=c.evaluate(c.phase,w.value.stamp+.01)
            self.assertFalse(result.faulted,c.debug)
            self.assertEqual(c.command()[0],.55)
            self.assertLessEqual(abs(c.command()[1]),.2)
            self.assertLessEqual(abs(c.debug['steering_response']['damping_w']),.05)
        self.assertAlmostEqual(c.command()[0],.55)

    def test_neutral_phase_test_output_is_zero_even_in_observed_margin_recovery(self):
        w,c=self.armed();now=self.fault_sample(w)
        result=c.evaluate(c.phase,now,neutral_only=True)
        self.assertFalse(result.faulted,c.debug);self.assertEqual(c.command(),(0.,0.))



if __name__=='__main__':unittest.main()
