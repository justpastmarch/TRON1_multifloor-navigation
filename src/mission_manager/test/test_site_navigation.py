import json, tempfile, unittest
from pathlib import Path
from unittest.mock import Mock, patch

from mission_manager.mapping_store import MapLibrary
from mission_manager import site_navigation as sn


class SiteNavigationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.library = MapLibrary(self.temp.name)
        self.site_id, folder = self.library.create('site')
        (folder / 'saved_1').mkdir(); (folder / 'saved_1' / 'map.yaml').write_text('image: map.pgm\n')
        self.nav = sn.SiteNavigation(self.library, Mock(), {}, '/bin/true')

    def test_waypoints_are_validated_and_bounded(self):
        rows = sn.clean_waypoints([dict(x=1, y=2, yaw=0.5)])
        self.assertEqual(rows[0]['name'], '지점 1')
        for bad in ([dict(x=float('nan'), y=0, yaw=0)], [dict(x=1, y=2)], 'x', [dict(x=0, y=0, yaw=0)] * 101):
            with self.assertRaises(ValueError):
                sn.clean_waypoints(bad)

    def test_no_driving_while_mapping(self):
        with self.assertRaisesRegex(ValueError, '지도 작성 중'):
            self.nav.start(dict(map_id=self.site_id, map_key='saved_1'), mapping_active=True)

    def test_commands_need_a_loaded_map(self):
        for call in (lambda: self.nav.goto(dict(x=1, y=1, yaw=0)), lambda: self.nav.start_patrol({}),
                     lambda: self.nav.set_pose(dict(x=0, y=0, yaw=0))):
            with self.assertRaisesRegex(ValueError, '먼저'):
                call()

    def test_supervisor_failure_aborts_start_and_cleans_up(self):
        nav = sn.SiteNavigation(self.library, Mock(), {}, '/bin/false')
        with patch.object(sn, 'Process') as process:
            with self.assertRaises(RuntimeError):
                nav.start(dict(map_id=self.site_id, map_key='saved_1'), mapping_active=False)
        process.assert_not_called()
        self.assertEqual(nav.state, 'IDLE'); self.assertFalse(nav.active)

    def test_waypoints_persist_per_map(self):
        self.nav.map = dict(id=self.site_id, key='saved_1')
        self.nav.set_waypoints(dict(waypoints=[dict(x=1, y=2, yaw=0), dict(x=3, y=4, yaw=1)]))
        stored = json.loads((self.library.result_path(self.site_id, 'saved_1') / 'waypoints.json').read_text())
        self.assertEqual(len(stored['waypoints']), 2)
        self.assertEqual(len(self.nav._load_waypoints(self.library.result_path(self.site_id, 'saved_1'))), 2)

    def test_new_goal_cancels_only_own_goal_and_stop_cancels_all(self):
        self.nav.client = Mock()
        self.nav._end_run(cancel_all=False)
        self.nav.client.cancel_goal.assert_called_once(); self.nav.client.cancel_all_goals.assert_not_called()
        self.nav.halt()
        self.nav.client.cancel_all_goals.assert_called_once()

    def test_failed_goal_stops_patrol_with_reason(self):
        client = Mock(); client.wait_for_result.return_value = True; client.get_state.return_value = 4
        self.nav.client = client
        with patch.object(sn.rospy, 'Time'):
            self.nav._run([dict(x=0, y=0, yaw=0, name='A'), dict(x=1, y=0, yaw=0, name='B')], False, '순회')
        self.assertEqual(self.nav.state, 'FAILED'); self.assertIn('A', self.nav.detail)
        self.assertEqual(client.send_goal.call_count, 1)

    def test_patrol_visits_points_in_order(self):
        client = Mock(); client.wait_for_result.return_value = True; client.get_state.return_value = 3
        self.nav.client = client
        with patch.object(sn.rospy, 'Time'):
            self.nav._run([dict(x=0, y=0, yaw=0, name='A'), dict(x=1, y=0, yaw=0, name='B')], False, '순회')
        xs = [c[0][0].target_pose.pose.position.x for c in client.send_goal.call_args_list]
        self.assertEqual(xs, [0, 1]); self.assertEqual(self.nav.state, 'READY')


if __name__ == '__main__':
    unittest.main()
