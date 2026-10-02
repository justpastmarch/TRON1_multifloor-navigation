"""Entry policy checks; synthetic geometry does not prove physical holding."""
import math
import unittest
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
import test_entry_forward_capture as fixture
from test_lidar_control import sample
from stair_supervisor.stair_evidence import Phase

class EntryContinuity(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.EntryForwardCapture()
        self.fixture.setUp()

    def armed(self, route_index=1):
        w, c, _ = self.fixture.armed(self.fixture.routes[route_index], dict(x=-.3, yaw=.2))
        c._entry_verified = False
        c.begin_phase(Phase.VERIFY_ENTRY, 100.22)
        c._settled = lambda *a, **k: False
        c._settling_detail = {'fixture': 'continuous creep, never stationary'}
        return w, c

    def test_both_corridor_routes_leave_verify_while_creeping(self):
        for route_index in (0, 1):
            w, c = self.armed(route_index)
            w.value = sample(4, 100.3, x=-.2047, y=-.1038, yaw=.2804)
            r = c.evaluate(c.phase,100.31)
            self.assertTrue(r.complete,c.debug)
            self.assertFalse(r.faulted)
            self.assertNotIn('settled',c.debug['completion_checks'])
            c.begin_phase(Phase.ALIGN,100.32)
            w.value = sample(5,100.4,x=-.21,y=-.11,yaw=.28)
            r = c.evaluate(c.phase,100.41)
            self.assertTrue(r.complete,c.debug)
            c.begin_phase(Phase.FORWARD_SEGMENT_1,100.42)
            w.value = sample(6,100.5,x=-.22,y=-.12,yaw=.28)
            r=c.evaluate(c.phase,100.51)
            self.assertFalse(r.faulted,c.debug)
            self.assertGreater(c.command()[0],0.)

    def test_same_frame_or_stale_cannot_verify(self):
        w,c=self.armed()
        self.assertFalse(c.evaluate(c.phase,100.23).complete)
        w.value=sample(4,100.3,x=-.3)
        self.assertFalse(c.evaluate(c.phase,100.85).complete)

    def test_invalid_geometry_cannot_verify(self):
        w,c=self.armed();w.value=sample(4,100.3,x=-.3,valid=False)
        self.assertFalse(c.evaluate(c.phase,100.31).complete)

    def test_height_attitude_and_hard_support_not_removed(self):
        for kind in ('height','attitude','support','midflight'):
            w,c=self.armed();s=sample(4,100.3,x=-.3)
            if kind=='height':s=sample(4,100.3,x=-.3,z=.3)
            if kind=='support':s=sample(4,100.3,x=-.3,y=-.4)
            if kind=='midflight':s=sample(4,100.3,x=.5)
            if kind=='attitude':
                pose=np.array(s.transform).reshape(4,4);pose[:3,:3]=Rotation.from_euler('x',1.).as_matrix();s=replace(s,transform=tuple(pose.flat))
            w.value=s;r=c.evaluate(c.phase,100.31)
            self.assertFalse(r.complete,(kind,c.debug))
            if kind in ('attitude','support'):self.assertTrue(r.faulted,(kind,c.debug))

    def test_legacy_route_still_requires_settling(self):
        w,c=self.armed();del c.route['limits']['entry_corridor_yaw']
        w.value=sample(4,100.3,x=-.45)
        r=c.evaluate(c.phase,100.31);self.assertFalse(r.complete)
        self.assertIn('settled',c.debug['completion_checks'])

    def reserve_pose(self,seq,stamp):
        s=sample(seq,stamp,x=-.3943270588,y=-.2109719106,z=-.0157298374)
        pose=np.array(s.transform).reshape(4,4)
        pose[:3,:3]=Rotation.from_euler('xyz',[.0023158058,-.1037660276,.3227044911]).as_matrix()
        return replace(s,transform=tuple(pose.flat))

    def test_recorded_reserve_pose_verifies_then_aligns_inward(self):
        w,c=self.armed();w.value=self.reserve_pose(4,100.3)
        r=c.evaluate(c.phase,100.31);self.assertTrue(r.complete,c.debug)
        self.assertFalse(c.debug['current_supported'])
        c.begin_phase(Phase.ALIGN,100.32);w.value=self.reserve_pose(5,100.4)
        r=c.evaluate(c.phase,100.41)
        self.assertFalse(r.complete,c.debug);self.assertFalse(r.faulted,c.debug)
        self.assertEqual(c.command()[0],0.)
        self.assertLess(c.command()[1],0.)
        self.assertLessEqual(abs(c.command()[1]),c.route['limits']['max_w'])
        self.assertLessEqual(abs(c.command()[1]),c.route['limits']['max_alpha']*.41)

    def test_reserve_turn_cannot_reduce_clearance(self):
        w,c=self.armed();w.value=self.reserve_pose(4,100.3)
        pose,yaw,pitch,roll,velocity,angular,support=c._observe(w.value)
        names,polygons=c.support_regions(Phase.ALIGN)
        r=c._entry_reserve_command(pose[:2,3],yaw,support,.2,.9,polygons,.12,100.4)
        self.assertIsNone(r)

    def test_stale_reserve_pose_cannot_authorize_new_turn(self):
        w,c=self.armed();c._entry_verified=True;c.begin_phase(Phase.ALIGN,100.25)
        w.value=self.reserve_pose(4,100.3)
        r=c.evaluate(c.phase,100.85)
        self.assertFalse(r.complete);self.assertEqual(c.command(),(0.,0.))

if __name__=='__main__':unittest.main()
