import json
import tempfile
import unittest
import urllib.request
import urllib.error
from pathlib import Path
from mission_manager.console_door import ConsoleDoor
from mission_manager.mission_console import ConsoleServer

class Backend:
    def __init__(self):self.calls=[]
    def snapshot(self):return dict(mode='live',active=False,telemetry=dict(floor=dict(age_sec=0.,value=dict(floor_id='RF',state=2))))
    def map_snapshot(self):return {}
    def plan(self,d):return {}
    def start(self,d):self.calls.append(d);return {}
    def cancel(self,d):return {}
    def select_floor(self,d):return {}

class DoorHTTPTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        path=Path(self.tmp.name)/'index.html';path.write_text('<html>UI</html>')
        self.backend=Backend();self.sent=[]
        self.server=ConsoleServer(self.backend,path,0,door_factory=lambda read:ConsoleDoor(read,sender=lambda:self.sent.append('door') or 200,threaded=False))
        self.addCleanup(self.server.close);self.base='http://127.0.0.1:%d'%self.server.http.server_port
    def request(self,path,data=None,headers=None):
        req=urllib.request.Request(self.base+path,data=None if data is None else json.dumps(data).encode(),headers=headers or {})
        try:
            with urllib.request.urlopen(req) as r:return r.status,json.loads(r.read()) if path!='/' else r.read()
        except urllib.error.HTTPError as e:return e.code,json.loads(e.read())
    def test_read_only_ui_and_state_do_not_trigger_door_or_movement(self):
        self.assertEqual(self.request('/')[0],200)
        code,state=self.request('/api/state');self.assertIn('door_control',state)
        self.server.door.tick();self.assertEqual(self.sent,[]);self.assertEqual(self.backend.calls,[])
    def test_door_endpoints_require_token_and_matching_origin(self):
        for path in ('/api/door/return','/api/door/stop'):
            self.assertEqual(self.request(path,{})[0],403)
            self.assertEqual(self.request(path,{},dict(Origin='http://other.example',**{'X-TRON-Token':self.server.token}))[0],403)
        self.assertFalse(self.server.door.active);self.assertEqual(self.sent,[])
    def test_return_button_starts_door_repeat_without_mission_or_robot_command(self):
        headers={'X-TRON-Token':self.server.token}
        self.assertEqual(self.request('/api/door/return',{},headers)[0],200)
        self.server.door.tick();self.assertEqual(self.sent,['door']);self.assertEqual(self.backend.calls,[])
        self.assertEqual(self.request('/api/door/stop',{},headers)[0],200)
        self.assertFalse(self.server.door.active)

if __name__=='__main__':unittest.main()
