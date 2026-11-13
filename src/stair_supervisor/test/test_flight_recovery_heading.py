"""Flight margin recovery must survive steering unwinding while the heading is inward.

Field 2026-10-16 11:00 (stair_5f_rf_up, FORWARD_SEGMENT_1): the body drifted to
ey ~ +0.33 m, recovery engaged at 5.05 cm clearance, then the stop fired at the
same clearance once centering steering decayed to ~0 although the measured
heading already pointed back toward the centerline.
"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from test_lidar_control import Worker, sample  # noqa: E402
from stair_supervisor.configuration import load_lidar_configuration, load_stair_configuration  # noqa: E402
from stair_supervisor.stair_evidence import Phase  # noqa: E402
from stair_supervisor.stair_feedback import StairFeedback  # noqa: E402

ROOT = Path(__file__).resolve().parents[1] / 'config'
# Climb drifting to +y, then the heading turns inward past the 10 deg capture angle.
DRIFT = [(-.3, -.03, 0., 0.)] + [(-.3 + .13 * i, -.03 + .033 * i, .065 * i, .05) for i in range(1, 11)]
RETURN = [(1.09, .33, .685, -.02), (1.087, .327, .686, -.077), (1.087, .327, .686, -.107),
          (1.09, .326, .688, -.12), (1.095, .325, .69, -.13), (1.10, .324, .692, -.14),
          (1.105, .323, .694, -.16), (1.11, .322, .696, -.18)]


class FlightRecoveryHeadingTest(unittest.TestCase):
    def setUp(self):
        config = load_lidar_configuration(ROOT / 'stair_lidar_3f_4f_test.yaml', 'control', False)
        route = next(r for r in config.document['routes'] if r['id'] == 'stair_5f_rf_up')
        profile = next(p for p in load_stair_configuration(ROOT).profiles if p.id == route['id'])
        self.w = Worker()
        self.c = StairFeedback(self.w, np.eye(4), [route])
        self.c.capture_entry(profile.id, np.eye(4), .1, 'synthetic surveyed anchor', 100.)
        self.c.arm(profile, 100.)
        self.c.begin_phase(Phase.FORWARD_SEGMENT_1, 100.)
        self.now, self.seq = 100., 1

    def step(self, x, y, z, yaw):
        self.now += .2
        self.seq += 1
        self.w.value = sample(self.seq, self.now, x=x, y=y, z=z, yaw=yaw)
        return self.c.evaluate(Phase.FORWARD_SEGMENT_1, self.now + .01)

    def run_track(self, track):
        results = []
        for pose in track:
            report = self.step(*pose)
            results.append((report, dict(self.c.debug), self.c.command()))
            self.assertFalse(report.faulted, self.c.debug)
        return results

    def test_inward_heading_keeps_recovery_when_steering_unwinds(self):
        self.run_track(DRIFT)
        results = self.run_track(RETURN)
        bases = [debug.get('flight_margin_recovery_basis') for _, debug, _ in results]
        # Recovery starts on inward steering ...
        self.assertEqual(results[0][1]['tracking_state'], 'TRACKED')
        self.assertTrue(results[0][1]['flight_margin_recovery'])
        self.assertEqual(bases[0], 'steering')
        # ... and stays engaged on the measured inward heading once steering is ~0.
        _, last, command = results[-1]
        self.assertTrue(last['flight_margin_recovery'], last)
        self.assertEqual(last['flight_margin_recovery_basis'], 'heading')
        self.assertLess(last['ey'] * last['epsi'], 0.)
        self.assertGreaterEqual(last['ey'] * command[1], 0.)  # steering no longer inward
        self.assertGreater(last['clearance_m'], .03)          # within the 3 cm recovery floor
        self.assertLess(last['clearance_m'], .06)              # but below the 6 cm comfort margin

    def test_body_below_recovery_floor_still_stops(self):
        self.run_track(DRIFT + RETURN)
        report = self.step(1.115, .321, .698, -.20)
        self.assertTrue(report.faulted)
        self.assertEqual(self.c.debug['tracking_state'], 'CORRIDOR_LIMIT')
        self.assertLess(self.c.debug['clearance_m'], .03)


if __name__ == '__main__':
    unittest.main()
