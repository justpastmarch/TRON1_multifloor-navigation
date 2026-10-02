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

    def test_recorded_old_center_with_new_gyro_heading_does_not_terminate_ascent(self):
        w,c=self.armed();now=self.fault_sample(w)
        c._last_command=(.55,-.04);c._last_command_at=now-.05
        c._flight_recheck={'at':now-.21372045803582296,'sequence':375}
        detail={'source':'causal_gyro','yaw_rate':-.39293,'yaw_rate_age_sec':.0276}
        with patch.object(c,'_control_heading',return_value=(.2041439606587227,detail)):
            result=c.evaluate(c.phase,now)
        self.assertFalse(result.faulted,c.debug);self.assertFalse(result.complete)
        self.assertGreater(c.command()[0],0.);self.assertLessEqual(abs(c.command()[1]),.2)
        self.assertTrue(c.debug['footprint_supported']);self.assertTrue(c.debug['margin_recovery'])
        self.assertFalse(c.debug['gyro_heading_affects_flight_steering'])
        self.assertIsNone(c._flight_recheck)
        self.assertEqual(c.debug['support_timestamp'],w.value.stamp)
        self.assertAlmostEqual(c.debug['epsi'],c.debug['yaw'])

    def test_uncalibrated_future_rejection_cannot_override_observed_ascent(self):
        for route in self.routes:
            w,c=self.armed(route)
            with patch.object(c,'_flight_command',side_effect=AssertionError('prediction was allowed to gate ascent')):
                for i in range(1,8):
                    w.value=sample(i+1,100.+i*.2,x=.15*i,y=.08,z=.05*i)
                    result=c.evaluate(c.phase,w.value.stamp+.01)
                    self.assertFalse(result.faulted,c.debug);self.assertGreater(c.command()[0],0.)
                    self.assertEqual(c.debug['flight_command_policy'],'observed_feedback')

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

    def test_actual_input_loss_still_zeroes_then_exhausts_bounded_recovery(self):
        w,c=self.armed();w.value=sample(2,100.2,x=.3)
        c.evaluate(c.phase,100.21);self.assertGreater(c.command()[0],0.)
        first=w.value.stamp+c.route['limits']['expire_sec']+.01
        result=c.evaluate(c.phase,first);self.assertEqual(c.command(),(0.,0.))
        result=c.evaluate(c.phase,first+c.route['limits']['flight_input_recover_sec']+.1)
        self.assertTrue(result.faulted,c.debug)

    def test_command_floor_slew_and_small_gyro_damping_remain_bounded(self):
        w,c=self.armed()
        for i in range(1,61):
            w.value=sample(i+1,100.+i*.1,x=.2,y=.03)
            before=c.command()
            with patch.object(c,'_control_heading',return_value=(.8,{'source':'causal_gyro','yaw_rate':-20.,'yaw_rate_age_sec':.01})):
                result=c.evaluate(c.phase,w.value.stamp+.01)
            self.assertFalse(result.faulted,c.debug)
            self.assertLessEqual(c.command()[0]-before[0],.0150001)
            self.assertLessEqual(abs(c.command()[1]),.2)
            self.assertLessEqual(abs(c.debug['steering_response']['damping_w']),.05)
        self.assertAlmostEqual(c.command()[0],.55)

    def test_neutral_phase_test_output_is_zero_even_in_observed_margin_recovery(self):
        w,c=self.armed();now=self.fault_sample(w)
        result=c.evaluate(c.phase,now,neutral_only=True)
        self.assertFalse(result.faulted,c.debug);self.assertEqual(c.command(),(0.,0.))

    def test_invalid_policy_and_control_model_conflict_are_rejected(self):
        route=copy.deepcopy(self.routes[1]);route['flight_command_policy']='typo'
        with self.assertRaises(ValueError):StairFeedback._validate_route(route)
        route=copy.deepcopy(self.routes[1]);route['motion_prediction'].update(mode='control',commissioned=True)
        with self.assertRaisesRegex(ValueError,'diagnostic-only'):StairFeedback._validate_route(route)

    def test_legacy_recheck_distinguishes_observed_body_from_gyro_hypothesis(self):
        route=copy.deepcopy(self.routes[1]);route.pop('flight_command_policy')
        w,c=self.armed(route);now=self.fault_sample(w)
        c._last_command=(.55,-.04);c._last_command_at=now-.05
        c._flight_recheck={'at':now-.21372045803582296,'sequence':375}
        with patch.object(c,'_control_heading',return_value=(.2041439606587227,{'source':'causal_gyro'})):
            result=c.evaluate(c.phase,now)
        self.assertFalse(result.faulted,c.debug)
        self.assertTrue(c.debug['observed_footprint_supported']);self.assertFalse(c.debug['gyro_footprint_supported'])

if __name__=='__main__':unittest.main()
