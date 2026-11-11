import math
import unittest

import test_landing_region as fixtures
from test_lidar_control import sample
from stair_supervisor.stair_evidence import Phase


class FinalLandingApproachTest(unittest.TestCase):
    def assert_handoff(self, phase, start, next_phase, height):
        worker, control = fixtures.LandingRegionTest().setup_control()
        control.begin_phase(phase, 100.)
        control.turn_index = len(control._landing_path()) - 1
        x, y, yaw = start
        for step in range(600):
            now = 100.2 + step * .1
            worker.value = sample(step + 2, now, x=x, y=y, z=height, yaw=yaw)
            report = control.evaluate(phase, now + .01)
            self.assertFalse(report.faulted, control.debug)
            if report.complete:
                control.begin_phase(next_phase, now + .02)
                worker.value = sample(step + 3, now + .2, x=x, y=y,
                                      z=height, yaw=yaw)
                report = control.evaluate(next_phase, now + .21)
                self.assertFalse(report.faulted, control.debug)
                self.assertEqual(control.command()[0], .55)
                return
            v, w = control.command()
            self.assertLessEqual(abs(v), .195 + 1e-9)
            self.assertLessEqual(abs(w), .2 + 1e-9)
            x += v * math.cos(yaw) * .1
            y += v * math.sin(yaw) * .1
            yaw += w * .1
        self.fail('supported final approach did not hand off: %s' % control.debug)

    def test_nominal_virtual_landing_fixedpoint_reaches_second_ascent(self):
        self.assert_handoff(Phase.TURN_TO_NEXT_FLIGHT,
            (2.356076752847241, 1.1491256494608235, 2.944951225675127),
            Phase.FORWARD_SEGMENT_2, 1.7)

    def test_landing_final_approach_handles_both_lateral_sides(self):
        for y in (1.1, 1.75):
            with self.subTest(y=y):
                self.assert_handoff(Phase.TURN_TO_NEXT_FLIGHT,
                    (2.65, y, math.pi), Phase.FORWARD_SEGMENT_2, 1.7)

    def test_roof_final_approach_handles_both_lateral_sides(self):
        for x in (-.85, -.08):
            with self.subTest(x=x):
                self.assert_handoff(Phase.ROOFTOP_TURN,
                    (x, 1.6, math.pi / 2), Phase.FORWARD_SEGMENT_3, 3.23)
