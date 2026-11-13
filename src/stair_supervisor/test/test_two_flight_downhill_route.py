"""Two-flight forward descents reversed from the surveyed 3F-4F and 4F-5F UP routes."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from test_lidar_control import Worker, sample  # noqa: E402
from stair_supervisor.configuration import load_lidar_configuration, load_stair_configuration  # noqa: E402
from stair_supervisor.map_entry import map_entry_anchor  # noqa: E402
from stair_supervisor.reverse_route import reverse_route  # noqa: E402
from stair_supervisor.stair_evidence import Phase  # noqa: E402
from stair_supervisor.stair_feedback import StairFeedback  # noqa: E402

ROOT = Path(__file__).resolve().parents[1] / 'config'
ROUTES = (('stair_3f_4f_down', 'stair_3f_4f_up', '4F'), ('stair_4f_5f_down', 'stair_4f_5f_up', '5F'))


class TwoFlightDownRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_lidar_configuration(ROOT / 'stair_lidar_3f_4f_test.yaml', 'control', False)
        cls.routes = {r['id']: r for r in cls.config.document['routes']}
        cls.stair = load_stair_configuration(ROOT)
        cls.profiles = {p.id: p for p in cls.stair.profiles}

    def controller(self, down_id):
        route = self.routes[down_id]
        profile = self.profiles[down_id]
        self.w = Worker()
        self.c = StairFeedback(self.w, np.eye(4), [route])
        self.c.capture_entry(profile.id, np.eye(4), .1, 'synthetic surveyed anchor', 100.)
        self.c.arm(profile, 100.)
        self.now, self.seq = 100., 1
        return route

    def step(self, phase, x, y, z, yaw):
        if self.c.phase != phase:
            self.c.begin_phase(phase, self.now)
        self.now += .2
        self.seq += 1
        self.w.value = sample(self.seq, self.now, x=x, y=y, z=z, yaw=yaw)
        result = self.c.evaluate(phase, self.now + .01)
        self.assertFalse(result.faulted, self.c.debug)
        return result

    def test_geometry_is_the_up_route_reversed(self):
        for down_id, up_id, _ in ROUTES:
            with self.subTest(down_id):
                down, up = self.routes[down_id], self.routes[up_id]
                self.assertEqual(down['direction'], 'DOWN')
                self.assertEqual(down['flight_1'], up['flight_2'][::-1])
                self.assertEqual(down['flight_2'], up['flight_1'][::-1])
                self.assertEqual(down['flight_1_polygon'], up['flight_2_polygon'])
                self.assertEqual(down['flight_2_polygon'], up['flight_1_polygon'])
                self.assertEqual(down['entry_polygon'], up['exit_polygon'])
                self.assertEqual(down['exit_polygon'], up['entry_polygon'])
                self.assertEqual(down['landing_polygon'], up['landing_polygon'])
                self.assertNotIn('flight_3', down)
                self.assertNotIn('entry_reference', down)
                # Heights fall through the landing and end on the lower floor.
                self.assertGreater(down['flight_1'][0][2], down['flight_1'][1][2])
                self.assertAlmostEqual(down['flight_1'][1][2], down['flight_2'][0][2])
                self.assertAlmostEqual(down['flight_2'][1][2], 0.)

    def test_landing_turn_reuses_up_pivots_in_reverse(self):
        for down_id, up_id, _ in ROUTES:
            with self.subTest(down_id):
                down, up = self.routes[down_id], self.routes[up_id]
                path = down['turn_path']
                self.assertEqual(down['landing_turn_policy'], 'supported_region')
                self.assertEqual(path[0][:2], up['turn_path'][-2][:2])
                self.assertEqual(path[2][:2], up['turn_path'][1][:2])
                self.assertEqual(path[-1][:2], down['flight_2'][0][:2])
                self.assertAlmostEqual(path[0][2], 0.)          # down flight 1 heading
                self.assertAlmostEqual(abs(path[-1][2]), math.pi)  # down flight 2 heading
                for x, y, _ in path:
                    self.assertTrue(2.52 <= x <= 4.0 and -.615 <= y <= 2.085, (x, y))

    def test_forward_inputs_come_from_the_profile(self):
        scale = self.stair.robot.websocket_full_scale.linear_mps
        for down_id, _, _ in ROUTES:
            with self.subTest(down_id):
                inputs = self.profiles[down_id].downhill_forward_inputs
                self.assertEqual(self.routes[down_id]['flight_input_v'], [scale * inputs[0], scale * inputs[1]])

    def test_each_flight_completes_only_after_height_drop(self):
        self.controller('stair_3f_4f_down')
        # End poses put the whole 0.45 m footprint (+6 cm margin) inside the next flat region.
        for phase, mid, end, yaw in ((Phase.FORWARD_SEGMENT_1, (.9, 1.47, 2.4), (3.0, 1.47, 1.53), 0.),
                                     (Phase.FORWARD_SEGMENT_2, (1.2, 0., .8), (-.5, 0., 0.), math.pi)):
            self.assertFalse(self.step(phase, *mid, yaw).complete)
            self.assertTrue(self.step(phase, *end, yaw).complete, self.c.debug)
        self.assertEqual(self.c._flight_seen, [True, True])

    def test_landing_rotation_uses_region_policy(self):
        self.controller('stair_3f_4f_down')
        self.step(Phase.TURN_TO_NEXT_FLIGHT, 3.26, 1.47, 1.53, 0.)
        self.assertEqual(self.c.turn_index, 1)
        self.step(Phase.TURN_TO_NEXT_FLIGHT, 3.2, 1.2, 1.53, -math.pi / 2)
        self.assertEqual(self.c.turn_index, 2)

    def test_map_entry_faces_down_the_stair_from_the_landing_location(self):
        locations = {
            '4F': dict(x=8.621981, y=-4.168881, yaw=1.8303956535897932, floor_id='4F'),
            '5F': dict(x=8.230684, y=-4.731632, yaw=1.319219653589793, floor_id='5F'),
        }
        for down_id, _, floor in ROUTES:
            with self.subTest(down_id):
                route = self.routes[down_id]
                self.assertEqual(route['entry_map']['floor_id'], floor)
                loc = locations[floor]
                m = sample(2, 100.)
                for robot_yaw, expected in ((loc['yaw'], 0.), (loc['yaw'] - math.pi, math.pi)):
                    anchor = map_entry_anchor(route, loc, (loc['x'], loc['y'], robot_yaw), [.001] * 3, m,
                                              np.eye(4), floor, 100.1, 100.1, 100.)
                    body = np.linalg.inv(np.asarray(anchor.local_from_profile).reshape(4, 4)) @ np.asarray(m.transform).reshape(4, 4)
                    np.testing.assert_allclose(body[:2, 3], route['flight_1'][0][:2], atol=1e-6)
                    yaw = math.atan2(body[1, 0], body[0, 0])
                    # Facing the location yaw is facing down flight 1; the old arrival yaw faces away.
                    self.assertAlmostEqual(abs(math.atan2(math.sin(yaw - expected), math.cos(yaw - expected))), 0., places=6)

    def test_two_flight_spec_rejects_rf_only_keys_and_missing_inputs(self):
        up = self.routes['stair_3f_4f_up']
        profile = self.profiles['stair_3f_4f_down']
        spec = dict(id='stair_3f_4f_down', reverse_of=up['id'], commissioned=True,
                    entry_map=self.routes['stair_3f_4f_down']['entry_map'])
        with self.assertRaises(ValueError):
            reverse_route(dict(spec, short_descent_input=.09), up, profile, .55)
        from dataclasses import replace
        with self.assertRaises(ValueError):
            reverse_route(spec, up, replace(profile, downhill_forward_inputs=()), .55)


if __name__ == '__main__':
    unittest.main()
