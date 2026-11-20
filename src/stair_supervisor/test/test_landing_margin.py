"""Landing clearance boundaries using surveyed route geometry, not robot physics."""
import copy
import math
from pathlib import Path
import unittest

import numpy as np

from test_lidar_control import Worker, sample
from stair_supervisor.configuration import load_lidar_configuration, load_stair_configuration
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.stair_feedback import StairFeedback, effective_support_margin


class LandingMarginTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[1] / 'config'
        cls.routes = load_lidar_configuration(root / 'stair_lidar_3f_4f_test.yaml', 'control', False).document['routes']
        cls.profiles = {p.id: p for p in load_stair_configuration(root).profiles}

    def control(self, route_id='stair_5f_rf_up', legacy=False, uncertainty=.1):
        route = copy.deepcopy(next(r for r in self.routes if r['id'] == route_id))
        if legacy:
            route['limits'].pop('landing_support_margin_m')
            route['limits']['turn_margin_recovery_m'] = .01
        worker = Worker()
        control = StairFeedback(worker, np.eye(4), [route])
        control.capture_entry(route_id, np.eye(4), uncertainty, 'offline boundary fixture', 100.)
        control.arm(self.profiles[route_id], 100.01)
        return worker, control

    def test_all_routes_have_landing_three_cm_and_route_six_cm(self):
        self.assertEqual({route['id'] for route in self.routes}, {'stair_3f_4f_up', 'stair_4f_5f_up', 'stair_5f_rf_up', 'stair_5f_rf_down', 'stair_3f_4f_down', 'stair_4f_5f_down'})
        for route in self.routes:
            for uncertainty in (.01, .1, .25):
                with self.subTest(route=route['id'], uncertainty=uncertainty):
                    self.assertEqual(effective_support_margin(route, uncertainty, landing=True), .03)
                    self.assertEqual(effective_support_margin(route, uncertainty), .06)
                    self.assertEqual(route['limits']['landing_max_v'], .195)
                    self.assertNotIn('turn_margin_recovery_m', route['limits'])

    def test_landing_edge_three_cm_is_minimum_not_an_additional_recovery_budget(self):
        # Axis-aligned 45 cm footprint: bottom edge is center_y - .225.
        for uncertainty in (.01, .1):
            for clearance in (.029, .03, .031, .059):
                with self.subTest(clearance=clearance, uncertainty=uncertainty):
                    w, c = self.control(uncertainty=uncertainty)
                    c.begin_phase(Phase.LANDING, 100.1)
                    w.value = sample(2, 100.2, x=2.71, y=-.615+.225+clearance, z=1.7)
                    result = c.evaluate(c.phase, 100.21, neutral_only=True)
                    self.assertEqual(result.faulted, clearance < .03, c.debug)
                    self.assertAlmostEqual(c.debug['clearance_m'], clearance)
                    self.assertEqual(c.debug['recovery_command_margin_m'], .03)
                    self.assertEqual(c.diagnostic_snapshot()['effective_margin_m'], .03)

    def test_quarter_turn_keeps_turning_with_three_point_one_cm_clearance(self):
        for legacy in (False, True):
            w, c = self.control(legacy=legacy)
            c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT, 100.1)
            # At 45 degrees the square reaches its full 31.82 cm sweep radius.
            y = -.615 + math.hypot(.225, .225) + .031
            c._landing_turn_path = np.array(c.route['turn_path'], dtype=float)
            c._landing_turn_path[:, 1] += y
            c._landing_turn_captured = True
            c.turn_index = 1
            w.value = sample(2, 100.2, x=2.71, y=y, z=1.7, yaw=math.pi/4)
            result = c.evaluate(c.phase, 100.21)
            self.assertEqual(result.faulted, legacy, c.debug)
            if not legacy:
                self.assertEqual(c.command()[0], 0.)
                self.assertGreater(c.command()[1], 0.)
                self.assertFalse(c.debug['turn_margin_recovery'])

    def test_landing_arrival_uses_three_cm_before_fsm_transition(self):
        for route_id in ('stair_3f_4f_up', 'stair_5f_rf_up'):
            w, c = self.control(route_id)
            c.begin_phase(Phase.FORWARD_SEGMENT_1, 100.1)
            polygon = np.asarray(c.route['landing_polygon'])
            x = float(polygon[:, 0].mean())
            y = float(polygon[:, 1].min()) + .225 + .031
            w.value = sample(2, 100.2, x=x, y=y, z=c.route['flight_1'][1][2])
            result = c.evaluate(c.phase, 100.21)
            self.assertTrue(result.complete, c.debug)
            self.assertFalse(result.faulted, c.debug)
            self.assertEqual(c.debug['effective_margin_m'], .03)

    def test_landing_hold_and_phase_start_accept_the_same_boundary(self):
        for phase, region, flight in ((Phase.LANDING, 'landing_polygon', 'flight_1'),
                                     (Phase.TURN_TO_NEXT_FLIGHT, 'landing_polygon', 'flight_1'),
                                     (Phase.FORWARD_SEGMENT_2, 'landing_polygon', 'flight_1'),
                                     (Phase.ROOFTOP_TURN, 'roof_landing_polygon', 'flight_2'),
                                     (Phase.FORWARD_SEGMENT_3, 'roof_landing_polygon', 'flight_2')):
            for clearance in (.029, .031):
                with self.subTest(phase=phase, clearance=clearance):
                    w, c = self.control()
                    polygon = np.asarray(c.route[region])
                    w.value = sample(2, 100.2, x=float(polygon[:, 0].mean()),
                        y=float(polygon[:, 1].min()) + .225 + clearance,
                        z=c.route[flight][1][2])
                    # arm() consumes the pending anchor. A new phase-test
                    # preparation installs its independently captured anchor.
                    c.install_anchor(c.profile.id, c.anchor)
                    if clearance < .03:
                        with self.assertRaisesRegex(ValueError, 'entry geometry rejected'):
                            c.prepare(c.profile, 100.21, start_phase=phase)
                    else:
                        c.prepare(c.profile, 100.21, start_phase=phase)
                    if phase is Phase.LANDING:
                        c.begin_phase(phase, 100.21)
                        if clearance < .03:
                            with self.assertRaisesRegex(ValueError, 'supported landing'):
                                c.begin_landing_hold(100.22)
                        else:
                            c.begin_landing_hold(100.22)

    def test_second_flight_roof_arrival_uses_same_three_cm_boundary(self):
        for clearance in (.029, .031):
            w, c = self.control()
            c.begin_phase(Phase.FORWARD_SEGMENT_2, 100.1)
            w.value = sample(2, 100.2, x=-.46, y=.795+.225+clearance,
                             z=3.23, yaw=math.pi)
            result = c.evaluate(c.phase, 100.21)
            self.assertEqual(result.complete, clearance >= .03, c.debug)
            self.assertEqual(result.faulted, clearance < .03, c.debug)

    def test_next_flight_releases_landing_speed_cap_and_margin_follows_region(self):
        w, c = self.control()
        c.begin_phase(Phase.FORWARD_SEGMENT_2, 100.1)
        for seq, x, expected_margin in ((2, 2.42, .03), (3, 2.05, .03), (4, 1.8, .06)):
            stamp = 100. + seq*.2
            w.value = sample(seq, stamp, x=x, y=1.47, z=1.7, yaw=math.pi)
            result = c.evaluate(c.phase, stamp+.01)
            self.assertFalse(result.faulted, c.debug)
            self.assertFalse(c.debug['second_flight_alignment'])
            self.assertEqual(c.command()[0], .55)
            self.assertEqual(c.debug['effective_margin_m'], expected_margin)

    def test_roof_third_flight_alignment_checks_roof_support(self):
        w, c = self.control()
        c.begin_phase(Phase.FORWARD_SEGMENT_3, 100.1)
        w.value = sample(2, 100.2, x=-.46, y=1.47, z=3.23, yaw=0.)
        result = c.evaluate(c.phase, 100.21)
        self.assertFalse(result.faulted, c.debug)
        self.assertTrue(c.debug['second_flight_alignment'])
        self.assertEqual(c.debug['support_regions'], ['roof_landing_polygon'])
        self.assertEqual(c.debug['effective_margin_m'], .03)

    def test_landing_context_does_not_select_entry_or_exit(self):
        for route in self.routes:
            _, c = self.control(route['id'])
            for phase in (Phase.VERIFY_ENTRY, Phase.ALIGN, Phase.EXIT_CONFIRM):
                for region in ('entry_polygon', 'landing_polygon', 'exit_polygon'):
                    self.assertFalse(c._landing_margin_context(phase, np.asarray(route[region]).mean(axis=0)))

    def test_roof_margin_follows_physical_region_when_route_is_reversed(self):
        expected = {'stair_3f_4f_up': (.06, .06),
                    'stair_4f_5f_up': (.06, .06),
                    'stair_3f_4f_down': (.06, .06),
                    'stair_4f_5f_down': (.06, .06),
                    'stair_5f_rf_up': (.06, .03),
                    'stair_5f_rf_down': (.03, .06)}
        for route in self.routes:
            for uncertainty in (.01, .1, .25):
                with self.subTest(route=route['id'], uncertainty=uncertainty):
                    self.assertEqual(tuple(effective_support_margin(route, uncertainty, region=r)
                        for r in ('entry', 'exit')), expected[route['id']])

    def test_last_two_steps_arrival_and_exit_hold_use_same_three_cm_boundary(self):
        for phase in (Phase.FORWARD_SEGMENT_3, Phase.EXIT_CONFIRM):
            for clearance in (.029, .03, .031, .059):
                with self.subTest(phase=phase, clearance=clearance):
                    w, c = self.control()
                    c.begin_phase(phase, 100.1)
                    c._flight_seen = [True, True, True]
                    x = -1.075+.225+clearance
                    w.value = sample(2, 100.2, x=x, y=3.06, z=3.57, yaw=math.pi/2)
                    result = c.evaluate(phase, 100.21)
                    self.assertEqual(result.complete, clearance >= .03, c.debug)
                    self.assertEqual(result.faulted, clearance < .03, c.debug)
                    self.assertEqual(c.debug['effective_margin_m'], .03)
                    self.assertEqual(c.debug['support_margin_context'], 'exit')
                    self.assertEqual(c.diagnostic_snapshot()['effective_margin_m'], .03)
                    if phase is Phase.EXIT_CONFIRM and clearance >= .03:
                        c.begin_arrival_hold()
                        w.value = sample(3, 100.4, x=x, y=3.06, z=3.57, yaw=math.pi/2)
                        result = c.evaluate(phase, 100.41)
                        self.assertFalse(result.faulted, c.debug)
                        self.assertAlmostEqual(c.debug['recovery_command_margin_m'], .03)

    def test_roof_exit_margin_does_not_change_final_height_or_ascent_history(self):
        for height, history in ((3.40, [True, True, True]), (3.57, [True, False, True])):
            w, c = self.control()
            c.begin_phase(Phase.EXIT_CONFIRM, 100.1);c._flight_seen = history
            w.value = sample(2, 100.2, x=-.819, y=3.06, z=height, yaw=math.pi/2)
            result = c.evaluate(c.phase, 100.21)
            self.assertFalse(result.complete, c.debug)

    def test_downhill_entry_accepts_the_same_roof_clearance_then_releases_override(self):
        for clearance in (.029, .031):
            w, c = self.control('stair_5f_rf_down')
            x=-1.075+.225+clearance
            w.value = sample(2, 100.2, x=x, y=3.3, z=3.57, yaw=-math.pi/2)
            c.install_anchor(c.profile.id, c.anchor)
            if clearance < .03:
                with self.assertRaisesRegex(ValueError, 'entry geometry rejected'):
                    c.prepare(c.profile, 100.21, start_phase=Phase.VERIFY_ENTRY)
            else:
                c.prepare(c.profile, 100.21, start_phase=Phase.VERIFY_ENTRY)
                c.begin_phase(Phase.VERIFY_ENTRY, 100.21)
                w.value = sample(3, 100.4, x=x, y=3.3, z=3.57, yaw=-math.pi/2)
                result=c.evaluate(c.phase, 100.41)
                self.assertFalse(result.faulted, c.debug)
                self.assertEqual(c.debug['effective_margin_m'], .03)
        self.assertEqual(c._support_margin_context(Phase.FORWARD_SEGMENT_1, np.array([-.46,2.8,3.5])), 'entry')
        self.assertEqual(c._support_margin_context(Phase.FORWARD_SEGMENT_1, np.array([-.46,2.5,3.4])), 'route')

    def test_invalid_entry_and_exit_margins_are_rejected(self):
        for key in ('entry_support_margin_m', 'exit_support_margin_m'):
            for value in (0., -.01, .01, .061, float('nan')):
                route=copy.deepcopy(self.routes[0]);route['limits'][key]=value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    StairFeedback._validate_route(route)

    def test_invalid_landing_margin_is_rejected_and_legacy_falls_back(self):
        route = copy.deepcopy(self.routes[0])
        for value in (0., -.01, .01, .061, float('nan')):
            route['limits']['landing_support_margin_m'] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                StairFeedback._validate_route(route)
        route['limits'].pop('landing_support_margin_m')
        self.assertEqual(effective_support_margin(route, .1, landing=True), .06)


if __name__ == '__main__':
    unittest.main()
