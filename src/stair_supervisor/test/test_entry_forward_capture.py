"""Behavior checks; recorded poses are not independent physical ground truth."""
import copy
import math
import os
import sys
import unittest
from pathlib import Path
import numpy as np
import yaml

REPO = Path(os.environ['ENTRY_CAPTURE_REPO']) if 'ENTRY_CAPTURE_REPO' in os.environ else Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'src/stair_supervisor/test'))
from test_lidar_control import Worker, sample
from stair_supervisor.configuration import load_stair_configuration
from stair_supervisor.stair_feedback import StairFeedback
from stair_supervisor.stair_evidence import Phase

RECORDED_POSES = [dict(x=-.21808727884372794, y=-.15990360646774512, z=-.00774534679808267, yaw=.18050765075337416),
                  dict(x=-.16556845592221767, y=-.1550284818566516, z=.02608208305361087, yaw=.24155551413552379)]


class EntryForwardCapture(unittest.TestCase):
    def setUp(self):
        path = Path(os.environ.get('ENTRY_CAPTURE_CONFIG', str(REPO / 'src/stair_supervisor/config/stair_lidar_3f_4f_test.yaml')))
        self.routes = [r for r in yaml.safe_load(path.read_text())['routes'] if r.get('direction')=='UP']
        self.profiles = load_stair_configuration(REPO / 'src/stair_supervisor/config').profiles

    def armed(self, route, pose):
        w = Worker()
        c = StairFeedback(w, np.eye(4), [copy.deepcopy(route)])
        c.capture_entry(route['id'], np.eye(4), .1, 'recorded geometry hypothesis', 100.)
        w.value = sample(2, 100.1, **pose)
        p = next(p for p in self.profiles if p.id == route['id'])
        c.arm(p, 100.11, start_phase=Phase.ALIGN)
        c._entry_verified = True
        c.begin_phase(Phase.ALIGN, 100.11)
        w.value = sample(3, 100.2, **pose)
        return w, c, c.evaluate(Phase.ALIGN, 100.21)

    def test_recorded_entry_and_final_align_pose_leave_align(self):
        for pose in RECORDED_POSES:
            with self.subTest(pose=pose):
                w, c, report = self.armed(self.routes[1], pose)
                self.assertTrue(report.complete, c.debug)
                self.assertFalse(report.faulted)
                c.begin_phase(Phase.FORWARD_SEGMENT_1, 100.22)
                for i in range(4, 47):
                    stamp = 100.2 + (i-3)*.1
                    w.value = sample(i, stamp, **pose)
                    report = c.evaluate(Phase.FORWARD_SEGMENT_1, stamp+.01)
                    self.assertFalse(report.faulted, c.debug)
                    self.assertGreater(c.command()[0], 0., c.debug)
                    self.assertLessEqual(abs(c.command()[1]), c.route['limits']['max_w'])
                self.assertAlmostEqual(c.command()[0], .55)

    def test_both_routes_capture_heading_within_twenty_degrees(self):
        for route in self.routes:
            for angle in (-19., -14., 14., 19.):
                with self.subTest(route=route['id'], angle=angle):
                    _, c, report = self.armed(route, dict(x=-.3, yaw=math.radians(angle)))
                    self.assertTrue(report.complete, c.debug)

    def test_large_heading_still_aligns_without_forward(self):
        for angle in (-30., 30., 90.):
            _, c, report = self.armed(self.routes[1], dict(x=-.3, yaw=math.radians(angle)))
            self.assertFalse(report.complete)
            self.assertEqual(c.command()[0], 0.)

    def test_exact_twenty_degree_boundary_and_just_outside(self):
        for angle in (-20., 20.):
            _, c, report = self.armed(self.routes[1], dict(x=-.3, yaw=math.radians(angle)))
            self.assertTrue(report.complete, c.debug)
        for angle in (-20.01, 20.01):
            _, c, report = self.armed(self.routes[1], dict(x=-.3, yaw=math.radians(angle)))
            self.assertFalse(report.complete)
            self.assertEqual(c.command()[0], 0.)

    def test_height_and_side_are_not_relaxed(self):
        for pose in (dict(x=-.3, y=.5, yaw=.2), dict(x=-.3, z=.3, yaw=.2)):
            with self.assertRaises(ValueError):
                self.armed(self.routes[1], pose)

    def test_height_and_side_checked_after_entry_is_prepared(self):
        for pose in (dict(x=-.3, y=.5, yaw=.2), dict(x=-.3, z=.3, yaw=.2)):
            w, c, _ = self.armed(self.routes[1], dict(x=-.3, yaw=.2))
            w.value = sample(4, 100.3, **pose)
            report = c.evaluate(Phase.ALIGN, 100.31)
            self.assertFalse(report.complete, c.debug)
            self.assertLessEqual(c.command()[0], 0.)

    def test_same_frame_and_stale_observation_cannot_transition(self):
        pose = dict(x=-.3, yaw=.2)
        w, c, _ = self.armed(self.routes[1], pose)
        c.begin_phase(Phase.ALIGN, 100.22)
        report = c.evaluate(Phase.ALIGN, 100.23)
        self.assertFalse(report.complete)
        self.assertIn('new_phase_observation', c.debug['incomplete_conditions'])
        report = c.evaluate(Phase.ALIGN, 101.2)
        self.assertTrue(report.faulted)

    def test_old_angle_reproduces_recorded_wait(self):
        rr = copy.deepcopy(self.routes[1])
        rr['limits']['entry_corridor_yaw'] = math.radians(10.)
        for pose in RECORDED_POSES:
            _, c, report = self.armed(rr, pose)
            self.assertFalse(report.complete)
            self.assertEqual(c.command()[0], 0.)
            self.assertEqual(c.debug['incomplete_conditions'], ['yaw'])


if __name__ == '__main__':
    unittest.main()
