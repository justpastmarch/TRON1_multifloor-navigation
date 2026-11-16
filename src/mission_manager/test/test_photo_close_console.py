"""Console: end a photo session without returning (record only, never motion)."""
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))  # sibling test fixtures
import json
import tempfile
import time
import unittest
from unittest.mock import Mock

import test_ros_console as _console_fixture  # module import: do not re-collect its tests
from mission_manager.mission_console import ConsoleServer
from mission_manager.photo_mission import PhotoMission


class PhotoCloseConsoleTest(unittest.TestCase):
    def setUp(self):
        self.adapter = _console_fixture.TestAdapter('test_ui_pose_estimate_publishes_map_frame_pose_without_starting_motion')
        self.adapter.setUp()
        self.addCleanup(self.adapter.doCleanups)
        temp = tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        self.executor = Mock()
        self.photo = PhotoMission(dict(locations=['roof_loop_sw']), temp.name, self.executor)
        self.photo.save(version=1, mission_id='tour_1', origin=dict(id='home_3f', floor_id='3F'),
                        route_locations=['roof_loop_sw'], photos=[], missed_photos=[],
                        photo_outcome='COMPLETE', stage='WAITING_RETURN', return_started=False)
        c = self.adapter.make()
        c.orchestrator.photo = self.photo
        c.recovering = c.relocating = False
        c.telemetry['mission_status'] = (time.monotonic(), {'active': False})
        self.console = c

    def test_closes_waiting_session_without_any_command(self):
        result = self.console.close_photo_session(dict(mission_id='tour_1'))
        self.assertTrue(result['closed'])
        self.assertEqual(self.photo.snapshot()['stage'], 'CLOSED')
        self.console.client.send_goal.assert_not_called()
        self.assertEqual(self.executor.mock_calls, [])

    def test_refuses_while_a_mission_runs_or_for_another_session(self):
        self.console.telemetry['mission_status'] = (time.monotonic(), {'active': True})
        with self.assertRaisesRegex(ValueError, '진행 중인 임무'):
            self.console.close_photo_session(dict(mission_id='tour_1'))
        self.console.telemetry['mission_status'] = (time.monotonic(), {'active': False})
        with self.assertRaisesRegex(ValueError, '바뀌었습니다'):
            self.console.close_photo_session(dict(mission_id='tour_0'))
        self.assertEqual(self.photo.snapshot()['stage'], 'WAITING_RETURN')

    def test_http_route_needs_the_console_token(self):
        door = Mock(); door.snapshot.return_value = dict(reason='')
        self.console.snapshot = Mock(return_value={})
        server = ConsoleServer(self.console, _Path(__file__).resolve().parents[1] / 'config' / 'mission_console.html',
                               port=0, door_factory=lambda _read: door)
        self.addCleanup(server.close)
        import http.client
        port = server.http.server_port
        def post(token):
            conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
            conn.request('POST', '/api/photo/close', json.dumps(dict(mission_id='tour_1')),
                         {'Content-Type': 'application/json', 'X-TRON-Token': token, 'Host': '127.0.0.1:%d' % port})
            response = conn.getresponse(); return response.status, json.loads(response.read())
        self.assertEqual(post('wrong')[0], 403)
        self.assertEqual(self.photo.snapshot()['stage'], 'WAITING_RETURN')
        status, body = post(server.token)
        self.assertEqual((status, body['closed']), (200, True))
        self.assertEqual(self.photo.snapshot()['stage'], 'CLOSED')


if __name__ == '__main__':
    unittest.main()
