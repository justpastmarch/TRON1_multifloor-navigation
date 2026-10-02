"""Destination-height regression; recorded estimates are not physical truth."""
import copy
import math
import unittest
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from test_observed_flight import ObservedFlight
from test_rooftop_extension import RoofTest, roof_route
from test_lidar_control import sample
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.stair_feedback import StairFeedback


class MinimumHeight(unittest.TestCase):
    def production(self, band=False):
        helper=ObservedFlight();helper.setUp()
        route=copy.deepcopy(helper.routes[1])
        # Historical 23:53 BAG coordinates use the pre-v20 survey. Keep the
        # height-policy regression independent of later live geometry updates.
        route['flight_1'][1][0] = 3.1550000000000002
        route['flight_1_polygon'] = [[0,-.615],[2.8,-.615],[2.8,.615],[0,.615]]
        route['landing_polygon'] = [[2.8,-.615],[4.28,-.615],[4.28,2.085],[2.8,2.085]]
        route['ascent_height_policy']='band' if band else 'minimum'
        w,c=helper.armed(route)
        c.anchor=replace(c.anchor,local_from_profile=tuple(np.eye(4).flat))
        return w,c

    def recorded_landing(self, worker):
        pose=np.eye(4)
        pose[:3,:3]=Rotation.from_euler('ZYX',[-.07621419964624011,-.03612550575294025,-.037049826337711256]).as_matrix()
        pose[:3,3]=[3.190808071319695,.07747069231662955,1.9506873779283356]
        worker.value=replace(sample(2,100.2),transform=tuple(pose.flat))

    def test_recorded_landing_transitions_only_with_minimum_policy(self):
        for band in (True,False):
            w,c=self.production(band);self.recorded_landing(w)
            result=c.evaluate(c.phase,100.4586852138)
            np.testing.assert_allclose(c.debug['pose'],[3.190808071319695,.07747069231662955,1.9506873779283356])
            self.assertFalse(result.faulted,c.debug)
            self.assertEqual(result.complete,not band,c.debug)
            self.assertEqual(c.debug['flight_end_height_conflict'],band)
            self.assertEqual(c.debug['incomplete_conditions'],['height'] if band else [])

    def test_below_destination_is_still_incomplete(self):
        w,c=self.production();w.value=sample(2,100.2,x=3.3,z=1.49)
        result=c.evaluate(c.phase,100.21)
        self.assertFalse(result.complete)
        self.assertFalse(c.debug['completion_checks']['height'])

    def test_high_estimate_does_not_replace_progress_or_body_support(self):
        for x,y,yaw in [(2.5,0,0),(3.17,0,math.pi/4)]:
            w,c=self.production();w.value=sample(2,100.2,x=x,y=y,z=1.95,yaw=yaw)
            result=c.evaluate(c.phase,100.21)
            self.assertFalse(result.complete,c.debug)

    def test_stale_observation_cannot_complete_high_landing(self):
        w,c=self.production();self.recorded_landing(w)
        self.assertFalse(c.evaluate(c.phase,100.8).complete)

    def test_landing_hold_captures_measured_level_and_detects_later_change(self):
        w,c=self.production();self.recorded_landing(w)
        c.begin_phase(Phase.LANDING,100.21)
        c.begin_landing_hold(100.22)
        self.assertAlmostEqual(c._landing_hold_target[0][2],1.9506873779283356)
        w.value=sample(3,100.4,x=3.3,z=2.2)
        result=c.evaluate(Phase.LANDING,100.41)
        self.assertTrue(result.faulted)
        self.assertEqual(c.debug['tracking_state'],'LANDING_HOLD_HEIGHT')

    def roof(self):
        route=roof_route();route['ascent_height_policy']='minimum'
        return RoofTest().setup_control(route)

    def test_second_and_third_flights_accept_positive_height_bias(self):
        for phase,x,y,z,yaw in [(Phase.FORWARD_SEGMENT_2,0,1,2.25,math.pi),
                                (Phase.FORWARD_SEGMENT_3,0,2,2.59,math.pi/2)]:
            w,c,_=self.roof();c.begin_phase(phase,100.)
            w.value=sample(2,100.2,x=x,y=y,z=z,yaw=yaw)
            self.assertTrue(c.evaluate(phase,100.21).complete,c.debug)

    def test_roof_turn_and_exit_use_same_destination_rule(self):
        for phase,x,y,z,yaw in [(Phase.ROOFTOP_TURN,0,1,2.25,math.pi/2),
                                (Phase.EXIT_CONFIRM,0,2,2.59,math.pi/2)]:
            w,c,_=self.roof();c.begin_phase(phase,100.)
            c.turn_index=1;c._flight_seen=[True,True,True]
            c._settled=lambda *_a,**_k:True;c._settling_detail={}
            w.value=sample(2,100.2,x=x,y=y,z=z,yaw=yaw)
            self.assertTrue(c.evaluate(phase,100.21).complete,c.debug)
            if phase is Phase.EXIT_CONFIRM:
                c._flight_seen[2]=False
                w.value=sample(3,100.4,x=x,y=y,z=z,yaw=yaw)
                self.assertFalse(c.evaluate(phase,100.41).complete)

    def test_entry_height_stays_banded(self):
        w,c=self.production();c.begin_phase(Phase.VERIFY_ENTRY,100.1)
        w.value=sample(2,100.2,x=-.45,z=.25)
        result=c.evaluate(Phase.VERIFY_ENTRY,100.21)
        self.assertFalse(result.complete)
        self.assertFalse(c.debug['completion_checks']['height'])

    def test_invalid_height_policy_is_rejected(self):
        r=roof_route();r['ascent_height_policy']='typo'
        with self.assertRaisesRegex(ValueError,'ascent_height_policy'):
            StairFeedback._validate_route(r)

if __name__=='__main__':unittest.main()
