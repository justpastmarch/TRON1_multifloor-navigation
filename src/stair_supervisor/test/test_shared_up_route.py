"""Shared stair geometry with separate floor-map entry; no ROS master/socket."""
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import yaml

from stair_supervisor.configuration import load_lidar_configuration
from stair_supervisor.lidar_tracking import TrackingSample
from stair_supervisor.map_entry import map_entry_anchor
from stair_supervisor.ros_lidar import RosLidarInterface
from stair_supervisor.stair_feedback import StairFeedback, se2


class SharedUpRouteTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1]
        self.config = load_lidar_configuration(
            self.root/'config/stair_lidar_3f_4f_test.yaml', 'control', False)
        self.routes = {r['id']: r for r in self.config.document['routes']}

    def test_four_to_five_uses_same_two_flights_landing_and_limits(self):
        a, b = self.routes['stair_3f_4f_up'], self.routes['stair_4f_5f_up']
        for key in ('flight_1','flight_2','entry_polygon','flight_1_polygon',
                    'landing_polygon','flight_2_polygon','exit_polygon',
                    'footprint','turn_path','limits','flight_command_policy',
                    'ascent_height_policy','phase_test_limits'):
            self.assertEqual(a[key],b[key],key)
        control = StairFeedback(None,self.config.document['base_from_lidar'],list(self.routes.values()))
        self.assertIn('stair_4f_5f_up',control.routes)

    def test_four_to_five_uses_own_map_entry_without_three_floor_cloud(self):
        route = self.routes['stair_4f_5f_up']
        self.assertIsNone(route['entry_reference'])
        self.assertEqual(route['entry_map']['floor_id'],'4F')
        self.assertEqual(route['entry_map']['location_id'],'stair_4f_to_5f')
        self.assertNotIn('entry_map',self.routes['stair_3f_4f_up'])

    def test_up_entry_dispatches_to_existing_map_entry_handler(self):
        route = self.routes['stair_4f_5f_up']
        adapter = RosLidarInterface.__new__(RosLidarInterface)
        adapter.control = SimpleNamespace(routes={route['id']:route})
        adapter._ensure_map_entry = Mock(return_value='map-anchor')
        self.assertEqual(adapter.ensure_entry(route['id']),'map-anchor')
        adapter._ensure_map_entry.assert_called_once_with(route['id'],route)

    def attitude_adapter(self, *, tracked_roll, holds_anchor=False):
        """Entry adapter with a level quasi-static IMU and a tracker at tracked_roll."""
        import test_attitude_consistency as fixture
        route = self.routes['stair_4f_5f_up']
        worker = fixture.FakeWorker(tracked_roll, 0.)
        def reset(reason):
            worker.resets.append(reason)
            worker.epoch += 1
            worker.sample = SimpleNamespace(epoch=worker.epoch, geometry_valid=True,
                                            transform=fixture.tracking_transform(0., 0.))
        worker.reset = reset
        adapter = RosLidarInterface.__new__(RosLidarInterface)
        adapter.worker = worker
        adapter.clock = SimpleNamespace(monotonic=lambda: 10.)
        adapter.control = SimpleNamespace(
            routes={route['id']: route}, base_from_lidar=fixture.BASE_FROM_LIDAR,
            holds_anchor=lambda: holds_anchor,
            entry_imu_context=lambda: fixture.context(fixture.imu_rows(0., 0.)))
        adapter._ensure_map_entry = Mock(return_value='map-anchor')
        return adapter, worker, route

    def test_entry_relevels_a_frame_that_disagrees_with_imu_gravity(self):
        adapter, worker, route = self.attitude_adapter(tracked_roll=.35)
        self.assertEqual(adapter.ensure_entry(route['id']), 'map-anchor')
        self.assertEqual(len(worker.resets), 1)
        self.assertEqual(adapter.entry_attitude_check['action'], 'relevel')
        adapter._ensure_map_entry.assert_called_once_with(route['id'], route)

    def test_entry_keeps_a_consistent_frame_or_an_active_hold(self):
        for tracked_roll, hold in ((.02, False), (.35, True)):
            adapter, worker, route = self.attitude_adapter(tracked_roll=tracked_roll, holds_anchor=hold)
            self.assertEqual(adapter.ensure_entry(route['id']), 'map-anchor')
            self.assertEqual(worker.resets, [])
            self.assertEqual(adapter.entry_attitude_check['action'], 'none')

    def anchor(self, floor='4F', offset=.1):
        route = self.routes['stair_4f_5f_up']
        # An arbitrary map origin shows this is not copied 3F map placement.
        location = dict(id='stair_4f_to_5f',x=9.972620,y=-3.892969,
                        yaw=1.870259,floor_id='4F')
        body = se2(1.,2.,.3)
        base = np.asarray(self.config.document['base_from_lidar'])
        sample = TrackingSample(0,1,100.,100.,100.,100.,100.,
            tuple((body@base).flat),True,'TRACKED','synthetic map-entry check')
        yaw = location['yaw']
        map_pose = (location['x']+offset*math.cos(yaw),
                    location['y']+offset*math.sin(yaw),yaw)
        anchor = map_entry_anchor(route,location,map_pose,[.001]*3,sample,
                                  base,floor,100.1,100.1,100.)
        placed = np.linalg.inv(np.asarray(anchor.local_from_profile).reshape(4,4))@body
        return placed

    def test_actual_map_offset_survives_in_local_stair_coordinates(self):
        placed = self.anchor()
        np.testing.assert_allclose(placed[:3,3],[-.35,0.,0.],atol=1e-9)
        self.assertAlmostEqual(math.atan2(placed[1,0],placed[0,0]),0.)

    def test_wrong_floor_does_not_reuse_map_anchor(self):
        with self.assertRaisesRegex(ValueError,'map/floor mismatch'):
            self.anchor(floor='3F')

    def test_registered_map_endpoint_exists_on_four_floor(self):
        root = self.root.parents[1]
        locations = yaml.safe_load((root/'src/mission_manager/config/locations.yaml').read_text())['locations']
        reference = self.routes['stair_4f_5f_up']['entry_map']
        location = next(x for x in locations if x['id']==reference['location_id'])
        self.assertEqual(location['floor_id'],reference['floor_id'])


if __name__ == '__main__':
    unittest.main()
