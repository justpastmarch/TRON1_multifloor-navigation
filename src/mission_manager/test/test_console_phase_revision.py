"""Execute the shipped phase-button handler with independently stale UI state.

No browser, ROS, network or robot: only the HTTP boundaries are mocked.
"""
import json
import shutil
import subprocess
import unittest
from pathlib import Path


class PhaseRevisionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node=shutil.which('node')
        if cls.node is None:
            raise unittest.SkipTest('Node.js is required for the shipped UI handler regression')

    def run_button(self, *, current=False, change=None, retained=False, api_error=False, get_error=False):
        html=(Path(__file__).parents[1]/'config/mission_console.html').read_text()
        handler=html[html.index('async function applyPhase(current)'):html.index("$('phaseResume').onclick=")]
        old=dict(run_id='mission-one',route_id='stair_5f_rf_up',revision=5,
                 phase='FORWARD_SEGMENT_2',phases=['FORWARD_SEGMENT_2','ROOFTOP_TURN'],
                 active=not retained,resume_available=retained,waiting=True,continue_supported=True)
        fresh=dict(old,revision=7)
        fresh.update(change or {})
        row=dict(age_sec=0.,value=fresh)
        script='''
let state={telemetry:{operator_phase:{value:OLD}}}, token='old-token';
let phaseError='',phaseReply=null,phaseReplyRun='',phaseBusy=false;
const phaseNames={},calls=[],elements={phaseSelect:{value:'ROOFTOP_TURN'},phaseResult:{}};
const $=id=>elements[id],confirm=()=>true,renderPhase=()=>{},recordingChoice=()=>false;
const fetch=async()=>({ok:!GET_ERROR,json:async()=>({token:'fresh-token',telemetry:{operator_phase:ROW}})});
const api=async(path,data)=>{calls.push({path,data,token});if(API_ERROR)throw Error('revision rejected');return {accepted:true};};
HANDLER
applyPhase(CURRENT).then(()=>process.stdout.write(JSON.stringify({calls,phaseError,phaseBusy})));
'''
        for key,value in [('OLD',json.dumps(old)),('ROW',json.dumps(row)),('GET_ERROR',json.dumps(get_error)),
                          ('API_ERROR',json.dumps(api_error)),('HANDLER',handler),('CURRENT',json.dumps(current))]:
            script=script.replace(key,value)
        result=subprocess.run([self.node,'-e',script],check=True,capture_output=True,text=True,timeout=5)
        return json.loads(result.stdout)

    def test_selected_roof_phase_uses_fresh_revision_not_cached_screen_revision(self):
        result=self.run_button()
        self.assertFalse(result['phaseError'])
        self.assertEqual(result['calls'][0]['data']['revision'],7)
        self.assertEqual(result['calls'][0]['data']['phase'],'ROOFTOP_TURN')
        self.assertEqual(result['calls'][0]['token'],'fresh-token')

    def test_continue_uses_same_phase_with_current_revision(self):
        result=self.run_button(current=True)
        self.assertEqual(result['calls'][0]['data']['kind'],'continue')
        self.assertEqual(result['calls'][0]['data']['phase'],'FORWARD_SEGMENT_2')
        self.assertEqual(result['calls'][0]['data']['revision'],7)

    def test_new_mission_route_pending_or_terminated_run_never_gets_old_request(self):
        for change in ({'run_id':'other'},{'route_id':'stair_3f_4f_up'},{'pending':True},{'active':False}):
            result=self.run_button(change=change)
            self.assertEqual(result['calls'],[])
            self.assertTrue(result['phaseError'])
            self.assertFalse(result['phaseBusy'])

    def test_continue_cannot_target_a_different_or_already_running_phase(self):
        for change in ({'phase':'ROOFTOP_TURN'},{'waiting':False},{'continue_supported':False}):
            self.assertEqual(self.run_button(current=True,change=change)['calls'],[])

    def test_retained_resume_keeps_route_and_uses_latest_revision(self):
        result=self.run_button(retained=True)
        self.assertEqual(result['calls'][0]['path'],'/api/start')
        self.assertEqual(result['calls'][0]['data']['resume_revision'],7)
        self.assertEqual(result['calls'][0]['data']['resume_route_id'],'stair_5f_rf_up')

    def test_failed_refresh_sends_nothing_and_server_rejection_is_not_retried(self):
        self.assertEqual(self.run_button(get_error=True)['calls'],[])
        result=self.run_button(api_error=True)
        self.assertEqual(len(result['calls']),1)
        self.assertEqual(result['phaseError'],'revision rejected')
