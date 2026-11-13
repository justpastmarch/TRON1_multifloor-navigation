"""Flat-landing recovery band: below 3 cm only inward motion, never below 2 cm.

Field 2026-10-16 08:19 (stair_5f_rf_up, TURN_TO_NEXT_FLIGHT): clearance fell to
2.96 cm against the 3 cm landing minimum and the turn stopped outright.
"""
import copy
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from test_lidar_control import Worker, sample  # noqa: E402
from stair_supervisor.configuration import load_lidar_configuration, load_stair_configuration  # noqa: E402
from stair_supervisor.stair_evidence import Phase  # noqa: E402
from stair_supervisor.stair_feedback import StairFeedback, support_clearance  # noqa: E402

ROOT = Path(__file__).resolve().parents[1] / 'config'
BOTTOM = -.615   # rf_up landing_polygon lower y edge
HALF = .225      # 45 cm square footprint


class LandingRecoveryBandTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.routes = load_lidar_configuration(ROOT / 'stair_lidar_3f_4f_test.yaml', 'control', False).document['routes']
        cls.profiles = {p.id: p for p in load_stair_configuration(ROOT).profiles}

    def control(self, band=True):
        route = copy.deepcopy(next(r for r in self.routes if r['id'] == 'stair_5f_rf_up'))
        if not band:
            route['limits'].pop('landing_margin_recovery_m')
        worker = Worker()
        control = StairFeedback(worker, np.eye(4), [route])
        control.capture_entry(route['id'], np.eye(4), .1, 'offline boundary fixture', 100.)
        control.arm(self.profiles[route['id']], 100.01)
        control.begin_phase(Phase.LANDING, 100.1)
        return worker, control, route

    def evaluate(self, clearance, yaw, band=True, neutral_only=False):
        worker, control, route = self.control(band)
        # Facing the lower edge (yaw -pi/2) keeps the square footprint axis-aligned.
        worker.value = sample(2, 100.2, x=2.71, y=BOTTOM + HALF + clearance, z=1.7, yaw=yaw)
        result = control.evaluate(control.phase, 100.21, neutral_only=neutral_only)
        return result, control, route

    def test_all_up_routes_configure_a_one_cm_band(self):
        for route in self.routes:
            if 'limits' in route:
                self.assertEqual(route['limits']['landing_margin_recovery_m'], .01, route['id'])
                self.assertEqual(route['limits']['landing_support_margin_m'], .03)
                self.assertEqual(route['limits']['margin_m'], .02)

    def test_inside_band_moves_back_inward_instead_of_stopping(self):
        result, control, route = self.evaluate(.025, -math.pi / 2)
        self.assertFalse(result.faulted, control.debug)
        self.assertTrue(control.debug['landing_margin_recovery'])
        self.assertAlmostEqual(control.debug['recovery_command_margin_m'], .02)
        v, w = control.command()
        # Facing the edge, the chosen motion backs away from it, slowly.
        self.assertLess(v, 0.)
        self.assertLessEqual(abs(v), route['limits']['hold_v'] + 1e-9)
        body = np.array([[x, y] for x in (-HALF, HALF) for y in (-HALF, HALF)]) + [2.71, BOTTOM + HALF + .025]
        rotated = body  # yaw -pi/2 on a square footprint is the same point set
        end = control._motion_end_body(np.array([2.71, BOTTOM + HALF + .025]), -math.pi / 2, rotated,
                                       v, w, route['limits']['expire_sec'])
        polygons = [route['landing_polygon']]
        self.assertGreater(support_clearance(end, polygons), support_clearance(rotated, polygons))

    def test_below_band_still_stops(self):
        result, control, _ = self.evaluate(.019, -math.pi / 2)
        self.assertTrue(result.faulted)
        self.assertEqual(control.debug['tracking_state'], 'CORRIDOR_LIMIT')

    def test_neutral_request_keeps_the_hard_three_cm_minimum(self):
        result, control, _ = self.evaluate(.025, -math.pi / 2, neutral_only=True)
        self.assertTrue(result.faulted)
        self.assertEqual(control.debug['tracking_state'], 'CORRIDOR_LIMIT')

    def test_without_the_band_the_three_cm_minimum_is_unchanged(self):
        result, control, _ = self.evaluate(.025, -math.pi / 2, band=False)
        self.assertTrue(result.faulted)
        self.assertEqual(control.debug['tracking_state'], 'CORRIDOR_LIMIT')

    def test_above_three_cm_nothing_changes(self):
        result, control, _ = self.evaluate(.031, -math.pi / 2)
        self.assertFalse(result.faulted, control.debug)
        self.assertFalse(control.debug.get('landing_margin_recovery'))

    def test_band_cannot_cross_margin_m(self):
        route = copy.deepcopy(next(r for r in self.routes if r['id'] == 'stair_5f_rf_up'))
        route['limits']['landing_margin_recovery_m'] = .015
        with self.assertRaises(ValueError):
            StairFeedback(Worker(), np.eye(4), [route])
        # A legacy route without a landing margin has no landing context: the band is inert.
        route['limits'].pop('landing_support_margin_m')
        route['limits']['landing_margin_recovery_m'] = .01
        StairFeedback(Worker(), np.eye(4), [route])


if __name__ == '__main__':
    unittest.main()
