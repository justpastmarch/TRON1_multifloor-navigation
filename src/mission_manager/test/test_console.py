import json,threading,unittest,urllib.request,urllib.error
from pathlib import Path
from mission_manager.mission_console import ConsoleServer

class Backend:
    def __init__(self):self.calls=[]
    def snapshot(self):return {'mode':'preview','active':False}
    def map_snapshot(self):return {'revision':'test-map','width':2}
    def operator_phase(self,data):self.calls.append(('phase',data));return {'accepted':True}
    def plan(self,data):return {'can_start':False,'blockers':['not commissioned']}
    def start(self,data):self.calls.append(data);return {'submitted':True}
    def recover_control(self,data):self.calls.append(data);return {'accepted':True}
    def approve_return(self,data):self.calls.append(('return',data));return {'submitted':True}
    def cancel(self,data):return {'cancel_requested':True}

class TestConsole(unittest.TestCase):
    def setUp(self):
        self.backend=Backend();self.server=ConsoleServer(self.backend,Path(__import__('mission_manager.mission_console', fromlist=['x']).__file__).resolve().parents[2] / 'config' / 'mission_console.html',0)
        self.base='http://127.0.0.1:%d'%self.server.http.server_port
    def tearDown(self):self.server.close()
    def request(self,path,body=None,**headers):
        req=urllib.request.Request(self.base+path,data=None if body is None else json.dumps(body).encode(),headers=headers)
        try:
            with urllib.request.urlopen(req) as r:return r.status,r.read()
        except urllib.error.HTTPError as e:return e.code,e.read()
    def test_loading_page_sends_no_commands(self):
        self.assertEqual(self.request('/')[0],200)
        self.assertEqual(self.request('/api/state')[0],200)
        self.assertEqual(self.backend.calls,[])
    def test_operator_phase_requires_authenticated_explicit_post(self):
        data={'run_id':'run','revision':1,'phase':'LANDING','confirmed':True}
        self.assertEqual(self.request('/api/operator_phase',data)[0],403)
        self.assertEqual(self.backend.calls,[])
        self.assertEqual(self.request('/api/operator_phase',data,**{'X-TRON-Token':self.server.token})[0],200)
        self.assertEqual(self.backend.calls,[('phase',data)])
    def test_map_endpoint_is_read_only(self):
        code,body=self.request('/api/map')
        self.assertEqual(code,200);self.assertEqual(json.loads(body)['revision'],'test-map')
        self.assertEqual(self.backend.calls,[])
    def test_cross_origin_and_no_token_rejected(self):
        self.assertEqual(self.request('/api/start',{'destination':'x'})[0],403)
        self.assertEqual(self.request('/api/start',{'destination':'x'},Origin='http://other.example',**{'X-TRON-Token':self.server.token})[0],403)
        self.assertEqual(self.backend.calls,[])
    def test_dns_rebinding_host_rejected(self):
        self.assertEqual(self.request('/api/state',Host='attacker.example')[0],403)
    def test_plan_does_not_send(self):
        code,data=self.request('/api/plan',{'destination':'x'},**{'X-TRON-Token':self.server.token})
        self.assertEqual(code,200);self.assertFalse(json.loads(data)['can_start']);self.assertEqual(self.backend.calls,[])
    def test_command_requires_explicit_post(self):
        code,_=self.request('/api/start',{'destination':'x'},**{'X-TRON-Token':self.server.token})
        self.assertEqual(code,200);self.assertEqual(self.backend.calls,[{'destination':'x'}])
    def test_recovery_requires_authenticated_post(self):
        self.assertEqual(self.request('/api/recover_control',{'confirmed_flat_ground':True})[0],403)
        self.assertEqual(self.backend.calls,[])
        self.assertEqual(self.request('/api/recover_control',{'confirmed_flat_ground':True},**{'X-TRON-Token':self.server.token})[0],200)
        self.assertEqual(self.backend.calls,[{'confirmed_flat_ground':True}])
    def test_return_requires_explicit_authenticated_post(self):
        self.assertEqual(self.request('/api/return',{'record':True})[0],403)
        self.assertEqual(self.backend.calls,[])
        code,body=self.request('/api/return',{'record':True},**{'X-TRON-Token':self.server.token})
        self.assertEqual(code,200);self.assertTrue(json.loads(body)['submitted'])
        self.assertEqual(self.backend.calls,[('return',{'record':True})])
    def test_mail_configuration_requires_token_and_sends_no_mission(self):
        from unittest.mock import Mock
        self.backend.mail_settings=Mock(return_value={'saved':True})
        self.assertEqual(self.request('/api/mail/settings',{'recipient':'p@example.com'})[0],403)
        self.backend.mail_settings.assert_not_called()
        self.assertEqual(self.request('/api/mail/settings',{'recipient':'p@example.com'},**{'X-TRON-Token':self.server.token})[0],200)
        self.backend.mail_settings.assert_called_once_with({'recipient':'p@example.com'})
        self.assertEqual(self.backend.calls,[])
    def test_bad_payload(self):
        self.assertEqual(self.request('/api/start',[],**{'X-TRON-Token':self.server.token})[0],409)
if __name__=='__main__':unittest.main()
