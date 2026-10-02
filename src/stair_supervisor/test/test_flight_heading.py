"""Steering contracts; no physical success claim from synthetic/fixed poses."""
import math
import copy
import unittest
from unittest.mock import patch
from dataclasses import replace
import numpy as np
import test_observed_flight as fixture
from test_lidar_control import sample
from stair_supervisor.stair_evidence import Phase

class FlightHeading(unittest.TestCase):
    def armed(self):
        f=fixture.ObservedFlight();f.setUp();return f.armed()

    def detail(self,remaining=.02):
        return dict(source='causal_gyro',gyro_remaining_age_sec=remaining,yaw_rate=0.,yaw_rate_age_sec=remaining)

    def test_uses_received_gyro_heading_for_steering_not_support(self):
        w,c=self.armed();w.value=sample(2,100.2,x=.3,y=.1,yaw=-.12)
        with patch.object(c,'_control_heading',return_value=(.1,self.detail())):
            r=c.evaluate(c.phase,100.21)
        self.assertFalse(r.faulted,c.debug)
        self.assertAlmostEqual(c.debug['epsi'],.1)
        self.assertAlmostEqual(c.debug['yaw'],-.12)
        self.assertTrue(c.debug['gyro_heading_affects_flight_steering'])
        self.assertEqual(c.debug['support_timestamp'],100.2)
        self.assertEqual(c.debug['support_orientation_source'],'lidar_observation')
        self.assertLess(c.command()[1],0.)

    def test_old_or_missing_gyro_uses_geometry(self):
        for detail in ({'source':'geometry'},self.detail(.2),self.detail(-.01)):
            w,c=self.armed();s=sample(2,100.2)
            v,d=c._flight_heading(s,.12,-.9,detail,100.21,False,True)
            self.assertAlmostEqual(v,.12);self.assertFalse(d['gyro_used'])

    def test_filter_at_same_time_cannot_count_extra_observations(self):
        w,c=self.armed();s=sample(2,100.2)
        c._flight_heading(s,0.,0.,self.detail(),100.21,False,True)
        value,_=c._flight_heading(s,0.,1.,self.detail(),100.31,False,True)
        for _ in range(20):
            again,_=c._flight_heading(s,0.,1.,self.detail(),100.31,False,True)
            self.assertAlmostEqual(value,again)
        self.assertGreater(value,0.);self.assertLess(value,.5)

    def test_wrap_near_pi_uses_short_arc(self):
        w,c=self.armed();s=sample(2,100.2)
        c._flight_heading(s,3.13,3.13,self.detail(),100.21,False,True)
        value,_=c._flight_heading(s,-3.13,-3.13,self.detail(),100.31,False,True)
        self.assertGreater(abs(value),3.)

    def test_phase_gap_and_epoch_reset_filter(self):
        for mode in ('phase','gap','epoch','degraded'):
            w,c=self.armed();s=sample(2,100.2)
            c._flight_heading(s,.1,.1,self.detail(),100.21,False,True)
            now=100.31
            if mode=='phase':c.begin_phase(Phase.FORWARD_SEGMENT_2,100.25)
            if mode=='gap':now=101.
            if mode=='epoch':s=replace(s,epoch=1)
            if mode=='degraded':c._flight_heading(s,.1,.1,self.detail(),100.25,True,True)
            value,d=c._flight_heading(s,-.2,-.2,self.detail(),now,False,True)
            self.assertAlmostEqual(value,-.2,(mode,d));self.assertTrue(d['reset'])

    def test_gyro_cannot_move_raw_body_back_inside_corridor(self):
        w,c=self.armed();w.value=sample(2,100.2,x=.9,y=.4,yaw=.2)
        with patch.object(c,'_control_heading',return_value=(-.5,self.detail())):
            r=c.evaluate(c.phase,100.21)
        self.assertTrue(r.faulted);self.assertEqual(c.debug['tracking_state'],'CORRIDOR_LIMIT')

    def test_single_shock_is_smoothed_and_continued_turn_is_followed(self):
        w,c=self.armed();s=sample(2,100.2)
        c._flight_heading(s,.15,.15,self.detail(),100.21,False,True)
        shock,_=c._flight_heading(s,-.15,-.15,self.detail(),100.31,False,True)
        self.assertGreater(shock,0.)
        for i in range(2,12):
            value,_=c._flight_heading(s,-.15,-.15,self.detail(),100.21+i*.1,False,True)
        self.assertLess(value,-.14)

    def test_steering_magnitude_and_slew_through_impacts(self):
        w,c=self.armed();before=c.command();before_t=c._last_command_at
        for i in range(1,20):
            t=100.+i*.1;w.value=sample(i+1,t,x=.3,y=.1,yaw=.1)
            with patch.object(c,'_control_heading',return_value=((-.25 if i%2 else .3),self.detail())):
                r=c.evaluate(c.phase,t+.01)
            self.assertFalse(r.faulted);self.assertLessEqual(abs(c.command()[1]),.2)
            # Existing neutralization may be faster than growth, by contract.
            self.assertLessEqual(abs(c.command()[1]-before[1]),.9*(t+.01-before_t)+1e-8)
            before=c.command();before_t=t+.01

    def test_invalid_filter_setting_is_rejected(self):
        w,c=self.armed();route=copy.deepcopy(c.route);route['limits']['flight_heading_filter_sec']=.6
        with self.assertRaises(ValueError):c._validate_route(route)

if __name__=='__main__':unittest.main()
