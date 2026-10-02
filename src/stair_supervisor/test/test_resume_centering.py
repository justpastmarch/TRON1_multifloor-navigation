"""Behavioral checks; no robot transport and no closed-loop success claim."""
import copy,math,unittest
from dataclasses import replace
import numpy as np
from test_observed_flight import ObservedFlight
from test_lidar_control import sample
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.stair_feedback import StairFeedback

class ResumeCentering(unittest.TestCase):
    def armed(self):
        helper=ObservedFlight();helper.setUp();w,c=helper.armed()
        c.anchor=replace(c.anchor,local_from_profile=tuple(np.eye(4).flat))
        w.value=sample(2,100.2,x=.4,z=.2)
        c._last_command=(.55,-.1);c._last_command_at=100.18
        c.evaluate(c.phase,100.21)
        return w,c

    def gap(self,w,c):
        r=c.evaluate(c.phase,101.11)
        self.assertFalse(r.faulted);self.assertEqual(c.command(),(0.,0.))

    def test_fresh_resume_restores_useful_forward_but_slews_steering(self):
        w,c=self.armed();self.gap(w,c)
        c.evaluate(c.phase,102.29)
        w.value=sample(3,102.3,x=.5,y=.1,z=.3,yaw=.1)
        r=c.evaluate(c.phase,102.31)
        self.assertFalse(r.faulted,c.debug)
        self.assertEqual(c.command()[0],.55)
        self.assertTrue(c.debug['flight_restart']);self.assertTrue(c.debug['input_recovery_resumed'])
        self.assertLessEqual(abs(c.command()[1]),.3*.02+1e-8)

    def test_recorded_two_second_delay_can_recover_before_four_seconds(self):
        w,c=self.armed();self.gap(w,c)
        r=c.evaluate(c.phase,103.31)
        self.assertFalse(r.faulted);self.assertEqual(c.command(),(0.,0.))
        w.value=sample(3,103.4,x=.5,z=.3)
        self.assertFalse(c.evaluate(c.phase,103.41).faulted)
        self.assertEqual(c.command()[0],.55)

    def test_expired_input_still_exhausts_and_never_blindly_drives(self):
        w,c=self.armed();self.gap(w,c)
        r=c.evaluate(c.phase,105.12)
        self.assertTrue(r.faulted);self.assertEqual(c.command(),(0.,0.))

    def test_neutral_invalid_epoch_boundary_and_attitude_cannot_use_restart(self):
        for mode in ('neutral','epoch','boundary','attitude'):
            with self.subTest(mode=mode):
                w,c=self.armed();self.gap(w,c)
                w.value=sample(3,101.3,x=.5,y=.7 if mode=='boundary' else .1,z=.3)
                if mode=='epoch':w.epoch=1
                if mode=='attitude':
                    from scipy.spatial.transform import Rotation
                    pose=np.array(w.value.transform).reshape(4,4);pose[:3,:3]=Rotation.from_euler('x',.5).as_matrix()
                    w.value=replace(w.value,transform=tuple(pose.flat))
                r=c.evaluate(c.phase,101.31,neutral_only=mode=='neutral')
                if mode=='neutral':self.assertEqual(c.command(),(0.,0.));self.assertFalse(c.debug['flight_restart'])
                else:self.assertTrue(r.faulted,c.debug)

    def test_normal_start_and_landing_keep_existing_slew_and_caps(self):
        w,c=self.armed();c._last_command=(0.,0.);c._last_command_at=100.2
        w.value=sample(3,100.3,x=.4,z=.2)
        c.evaluate(c.phase,100.31)
        self.assertLessEqual(c.command()[0],.15*.11+1e-8)
        c.phase=Phase.LANDING;c._last_command=(.55,0.);c._last_command_at=100.3
        self.assertLessEqual(c._limit_command(.55,.5,100.4)[0],.06)

    def test_left_and_right_centering_are_symmetric_and_bounded(self):
        _,c=self.armed()
        for y in (.04,.08,.12,.25):
            left,ld=c._flight_steering(.1,y,{})
            right,rd=c._flight_steering(-.1,-y,{})
            self.assertLess(left,0);self.assertAlmostEqual(left,-right)
            self.assertLessEqual(abs(ld['capture_heading']),math.radians(10)+1e-8)
            c._last_command=(.55,0.);c._last_command_at=100.
            self.assertLessEqual(abs(c._limit_command(.55,left,100.1)[1]),.03+1e-8)

    def test_inward_heading_does_not_keep_turning_into_centerline(self):
        _,c=self.armed();_,detail=c._flight_steering(0.,.12,{})
        heading=detail['capture_heading']
        self.assertAlmostEqual(c._flight_steering(heading,.12,{})[0],0.)
        self.assertGreater(c._flight_steering(heading-.1,.12,{})[0],0.)
        self.assertEqual(c._flight_steering(0.,0.,{})[0],0.)

    def test_earlier_correction_with_same_maximum_on_recorded_midflight_pose(self):
        _,c=self.armed()
        new,_=c._flight_steering(.1253262047,.1999159028,{})
        old_limits=copy.deepcopy(c.route['limits'])
        del c.route['limits']['flight_yaw_kp'];del c.route['limits']['flight_centering_lookahead_m']
        old,_=c._flight_steering(.1253262047,.1999159028,{})
        self.assertLess(new,old)
        c.route['limits']=old_limits;c._last_command=(.55,0.);c._last_command_at=100.
        self.assertEqual(c._limit_command(.55,new,101.)[1],-.15)

    def test_incomplete_centering_config_is_rejected(self):
        _,c=self.armed();r=copy.deepcopy(c.route);del r['limits']['flight_yaw_kp']
        with self.assertRaisesRegex(ValueError,'flight centering'):StairFeedback._validate_route(r)

if __name__=='__main__':unittest.main()
