import unittest, math
import numpy as np
from dataclasses import replace
from test_lidar_control import route, box, sample, Worker
from test_supervisor import make_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase


def roof_route():
    r = route()
    r.update(roof_landing_polygon=box(-.8,.4,.8,1.6),
             roof_turn_path=[[0,1,math.pi],[0,1,math.pi/2]],
             flight_3=[[0,1,2],[0,2,2.34]],
             flight_3_polygon=box(-.8,1,.8,2.1),
             exit_polygon=box(-.8,1.7,.8,3))
    return r


class RoofTest(unittest.TestCase):
    def setup_control(self, r):
        w=Worker();c=StairFeedback(w,np.eye(4),[r])
        c.capture_entry('test_up',np.eye(4),.01,'synthetic fixture',100.)
        p=replace(make_configuration().profiles[0],timeout_sec=60.)
        c.arm(p,100.)
        return w,c,p

    def test_only_roof_routes_add_phases(self):
        _,base,p=self.setup_control(route())
        _,roof,_=self.setup_control(roof_route())
        self.assertNotIn(Phase.ROOFTOP_TURN,base.traversal_phases(p))
        self.assertEqual(roof.traversal_phases(p)[-3:],(Phase.ROOFTOP_TURN,Phase.FORWARD_SEGMENT_3,Phase.EXIT_CONFIRM))

    def test_partial_geometry_rejected(self):
        r=roof_route();del r['flight_3_polygon']
        with self.assertRaises(ValueError): self.setup_control(r)

    def test_second_flight_finishes_on_upper_landing_not_roof(self):
        w,c,_=self.setup_control(roof_route())
        c.begin_phase(Phase.FORWARD_SEGMENT_2,100.)
        w.value=sample(2,100.2,x=0,y=1,z=2,yaw=math.pi)
        result=c.evaluate(Phase.FORWARD_SEGMENT_2,100.21)
        self.assertTrue(result.complete)
        self.assertFalse(c._flight_seen[2])

    def test_turn_has_flat_speed_cap(self):
        w,c,_=self.setup_control(roof_route())
        c.begin_phase(Phase.ROOFTOP_TURN,100.)
        c._last_command=(.3,0)
        v,_=c._limit_command(.3,0,100.2)
        self.assertLessEqual(v,c.route['limits']['hold_v'])

    def test_third_flight_needs_height_and_progress(self):
        w,c,_=self.setup_control(roof_route())
        c.begin_phase(Phase.FORWARD_SEGMENT_3,100.)
        w.value=sample(2,100.2,x=0,y=1.5,z=2.15,yaw=math.pi/2)
        self.assertFalse(c.evaluate(Phase.FORWARD_SEGMENT_3,100.21).complete)
        w.value=sample(3,100.4,x=0,y=2,z=2.34,yaw=math.pi/2)
        self.assertTrue(c.evaluate(Phase.FORWARD_SEGMENT_3,100.41).complete)

    def test_exit_requires_third_flight_and_final_height(self):
        w,c,_=self.setup_control(roof_route())
        c._flight_seen=[True,True,False]
        c.begin_phase(Phase.EXIT_CONFIRM,100.)
        c._settled=lambda *_a,**_k:True;c._settling_detail={}
        w.value=sample(2,100.2,x=0,y=2,z=2.34,yaw=math.pi/2)
        self.assertFalse(c.evaluate(Phase.EXIT_CONFIRM,100.21).complete)
        c._flight_seen[2]=True
        w.value=sample(3,100.4,x=0,y=2,z=2.,yaw=math.pi/2)
        self.assertFalse(c.evaluate(Phase.EXIT_CONFIRM,100.41).complete)
        w.value=sample(4,100.6,x=0,y=2,z=2.34,yaw=math.pi/2)
        self.assertTrue(c.evaluate(Phase.EXIT_CONFIRM,100.61).complete)
