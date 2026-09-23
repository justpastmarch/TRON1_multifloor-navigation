from __future__ import annotations

from dataclasses import replace
import copy
import math
from pathlib import Path
import sys
import threading
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from stair_supervisor.configuration import load_lidar_configuration, StairConfigurationError, Direction
from stair_supervisor.lidar_tracking import TrackingSample, TrackingSettings, TrackingWorker, TrackingEngine
from stair_supervisor.stair_feedback import StairFeedback, RouteAnchor, se2, match_entry_landmarks
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.supervisor import StairSupervisor, StairGoal, ResultCode, SupervisorState
from test_supervisor import FakeClock, FakeTransport, FakeAdmission, make_configuration, ScriptedEvidence


def box(x0, y0, x1, y1):
    return [[x0,y0],[x1,y0],[x1,y1],[x0,y1]]


def route():
    """SYNTHETIC METRIC FIXTURE. Never a production stair or physical tuning."""
    return dict(id="test_up", commissioned=True, direction="UP",
        flight_1=[[0,0,0],[2,0,1]], flight_2=[[2,1,1],[0,1,2]],
        entry_polygon=box(-1,-1,1,1), flight_1_polygon=box(-1,-1,3,1),
        flight_2_polygon=box(-1,0,3,2), landing_polygon=box(1.5,-1,3,2),
        exit_polygon=box(-1,0,.5,2), footprint=box(-.1,-.1,.1,.1),
        turn_path=[[2,0,0],[2,1,math.pi]],
        limits=dict(warn_sec=.3, expire_sec=.7, recover_sec=2., anchor_max_age_sec=2.,
            anchor_uncertainty_m=.03, margin_m=.01, yaw_kp=1., lateral_kp=.5,
            speed_kp=.2, hold_kp=1., hold_kd=.1, max_v=.3, max_w=.8,
            max_accel=1., max_alpha=2., hold_v=.1, position_on=.08, position_off=.04,
            yaw_tolerance=.06, stationary_v=.02, stationary_w=.03, settle_sec=.4,
            height_tolerance=.05, max_roll=.5, max_pitch=.7, handoff_sec=2.,
            yaw_deadband=.01, lateral_deadband=.01, speed_deadband=.01),
        loss_response="zero_velocity", loss_response_evidence="synthetic fake only; not hardware approval")


def sample(seq=1, stamp=100., x=0., y=0., z=0., yaw=0., epoch=0, valid=True):
    pose=se2(x,y,yaw);pose[2,3]=z
    return TrackingSample(epoch,seq,stamp,stamp,stamp,stamp,stamp,tuple(pose.flat),valid,
        "TRACKED" if valid else "DEGRADED","fixture")


class Worker:
    def __init__(self):
        self.value=sample(); self.epoch=0; self.error=""
    def snapshot(self):return self.value
    def diagnostics(self):return {"worker_error":self.error}


class FeedbackTest(unittest.TestCase):
    def setUp(self):
        self.worker=Worker();self.policy=StairFeedback(self.worker,np.eye(4),[route()])
        self.policy.capture_entry("test_up",np.eye(4),.01,"external synthetic reference",100.)
        self.profile=replace(make_configuration().profiles[0],timeout_sec=60.)
        self.policy.arm(self.profile,100.)

    def update(self, seq, stamp, **kwargs):
        self.worker.value=sample(seq,stamp,**kwargs)
        return self.policy.evaluate(self.policy.phase,stamp+.01)

    def test_normal_five_hz_never_hits_legacy_gap_fault(self):
        self.policy.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        for i in range(1,7):
            report=self.update(i+1,100.+i*.2,x=i*.02,z=i*.01)
            self.assertFalse(report.faulted)

    def test_side_error_and_wrong_heading_have_correct_correction_sign(self):
        self.policy.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        self.update(2,100.2,x=.1,y=.15,yaw=.1,z=.05)
        self.assertLess(self.policy.command()[1],0)
        self.assertGreater(self.policy.command()[0],0)

    def test_small_backward_slip_is_observed_not_discarded(self):
        self.policy.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        self.update(2,100.2,x=.3,z=.15)
        self.assertFalse(self.update(3,100.4,x=.28,z=.14).faulted)
        self.assertLess(self.policy.debug["velocity"][0],0)

    def test_old_accepted_scan_and_missing_scan_share_degraded_then_expired(self):
        self.policy.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        self.update(2,100.2,x=.1,z=.05)
        cmd=self.policy.command()
        delayed=self.policy.evaluate(self.policy.phase,100.6)
        self.assertFalse(delayed.faulted);self.assertFalse(delayed.complete)
        self.assertEqual(self.policy.command(),cmd)
        self.assertEqual(self.policy.debug['tracking_state'],'DEGRADED')
        self.assertTrue(self.policy.evaluate(self.policy.phase,101.).faulted)

    def test_single_rejection_recovers_in_same_phase_without_pose_restamp(self):
        self.policy.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        self.update(2,100.2,x=.1,z=.05)
        self.worker.value=replace(self.worker.value,sequence=3,stamp=100.4,geometry_valid=False,state='DEGRADED')
        self.assertFalse(self.policy.evaluate(self.policy.phase,100.45).faulted)
        self.assertFalse(self.update(4,100.6,x=.2,z=.1).faulted)
        self.assertEqual(self.policy.debug['tracking_state'],'TRACKED')

    def test_old_epoch_never_reuses_anchor(self):
        self.worker.epoch=1
        self.assertTrue(self.policy.evaluate(Phase.VERIFY_ENTRY,100.01).faulted)
        with self.assertRaises(ValueError):self.policy.prepare(self.profile,100.01)

    def test_repeated_pose_cannot_fill_stationary_dwell(self):
        self.policy.begin_phase(Phase.ALIGN,100.)
        self.update(2,100.2)
        for t in (100.21,100.22,100.23):
            self.assertFalse(self.policy.evaluate(Phase.ALIGN,t).complete)
        self.update(3,100.4)
        self.assertTrue(self.update(4,100.61).complete)

    def test_150_degree_turn_does_not_complete_next_flight(self):
        self.policy.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,100.)
        self.policy.turn_index=1
        for i in range(1,5):
            report=self.update(i+1,100.+i*.2,x=2,y=1,z=1,yaw=math.radians(150))
            self.assertFalse(report.complete)
        self.assertGreater(self.policy.command()[1],0)

    def test_entry_and_alignment_allow_bounded_sway_without_raising_rate_limits(self):
        for phase in (Phase.VERIFY_ENTRY, Phase.ALIGN):
            with self.subTest(phase=phase):
                self.setUp()
                self.policy.begin_phase(phase, 100.)
                complete = False
                for i in range(1, 8):
                    report = self.update(i+1, 100.+i*.2, x=.004*(-1)**i)
                    self.assertFalse(report.faulted)
                    complete = complete or report.complete
                self.assertTrue(complete)
                detail = self.policy.debug['settling']
                self.assertEqual(detail['rate_mode'], 'window_net_drift')
                self.assertGreater(detail['peak_v'], detail['stationary_v_limit'])
                self.assertLessEqual(detail['evaluated_v'], detail['stationary_v_limit'])

    def test_entry_still_rejects_sustained_translation_and_rotation(self):
        for mode in ('translation', 'rotation'):
            with self.subTest(mode=mode):
                self.setUp()
                self.policy.begin_phase(Phase.VERIFY_ENTRY, 100.)
                for i in range(1, 7):
                    dt=i*.2
                    kwargs={'x':.025*dt} if mode=='translation' else {'yaw':.035*dt}
                    report=self.update(i+1,100.+dt,**kwargs)
                    self.assertFalse(report.complete)
                    self.assertIn('settled', self.policy.debug['incomplete_conditions'])
                # Target tolerances still pass: it is sustained motion that blocks.
                self.assertTrue(self.policy.debug['completion_checks']['position'])
                self.assertTrue(self.policy.debug['completion_checks']['yaw'])

    def test_entry_may_finish_during_small_bounded_motion_not_physical_stillness(self):
        for phase in (Phase.VERIFY_ENTRY, Phase.ALIGN):
            with self.subTest(phase=phase):
                self.setUp()
                self.policy.begin_phase(phase,100.)
                for i in range(1,6):
                    report=self.update(i+1,100.+i*.2,x=.008 if i==5 else 0.)
                self.assertTrue(report.complete)
                detail=self.policy.debug['settling']
                self.assertGreater(detail['peak_v'],detail['stationary_v_limit'])
                self.assertLess(detail['evaluated_v'],detail['stationary_v_limit'])

    def test_entry_bounded_mean_does_not_hide_large_position_excursions(self):
        self.policy.begin_phase(Phase.VERIFY_ENTRY,100.)
        for i in range(1,8):
            report=self.update(i+1,100.+i*.2,x=.03*(-1)**i)
            self.assertFalse(report.complete)
        self.assertGreater(self.policy.debug['settling']['position_excursion_m'],
                           self.policy.route['limits']['position_off'])

    def test_turn_and_exit_keep_instantaneous_settling_requirement(self):
        for phase in (Phase.TURN_TO_NEXT_FLIGHT,Phase.EXIT_CONFIRM):
            with self.subTest(phase=phase):
                self.setUp()
                self.policy.begin_phase(phase,100.)
                self.policy.turn_index=1
                self.policy._flight_seen=[True,True]
                x,z=(2.,1.) if phase is Phase.TURN_TO_NEXT_FLIGHT else (0.,2.)
                for i in range(1,10):
                    report=self.update(i+1,100.+i*.2,x=x+.004*(-1)**i,y=1.,z=z,yaw=math.pi)
                    self.assertFalse(report.complete)
                self.assertEqual(self.policy.debug['settling']['rate_mode'],'instantaneous_peak')

    def test_front_sensor_or_last_two_steps_cannot_complete_flight(self):
        self.policy.begin_phase(Phase.FORWARD_SEGMENT_2,100.)
        self.assertFalse(self.update(2,100.2,x=.5,y=1,z=1.8,yaw=math.pi).complete)
        self.assertFalse(self.update(3,100.4,x=.05,y=1,z=2,yaw=math.pi).complete)
        self.assertTrue(self.update(4,100.6,x=-.01,y=1,z=2,yaw=math.pi).complete)

    def test_hold_target_does_not_follow_drift(self):
        self.policy.begin_phase(Phase.EXIT_CONFIRM,100.)
        self.policy._flight_seen=[True,True]
        self.update(2,100.2,x=.12,y=1,z=2,yaw=math.pi)
        self.assertGreater(self.policy.command()[0],0)
        self.assertEqual(self.policy.route['flight_2'][1],[0,1,2])

    def test_invalid_gain_downhill_and_unverified_response_are_rejected(self):
        for key,value in (('yaw_kp',float('nan')),('max_v',-1)):
            r=route();r['limits'][key]=value
            with self.assertRaises(ValueError):StairFeedback(self.worker,np.eye(4),[r])
        r=route();r['direction']='DOWN'
        with self.assertRaises(ValueError):StairFeedback(self.worker,np.eye(4),[r])
        r=route();r['loss_response_evidence']=''
        with self.assertRaises(ValueError):StairFeedback(self.worker,np.eye(4),[r])

    def test_landmark_transform_and_ambiguity(self):
        observed=np.array([[0.,0.],[1,0],[0,2]])
        expected=se2(2,3,.4)
        target=(expected@np.column_stack((observed,np.zeros(3),np.ones(3))).T).T[:,:2]
        fit,error=match_entry_landmarks(observed,[target],.01,.01)
        np.testing.assert_allclose(fit,expected,atol=1e-12)
        with self.assertRaises(ValueError):match_entry_landmarks(observed,[target,target],.01,.01)

    def test_small_tracking_noise_is_inside_steering_deadband(self):
        self.policy.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        self.update(2,100.2,x=.1,y=.004,z=.05,yaw=.004)
        self.assertEqual(self.policy.command()[1],0.)
        self.assertGreater(self.policy.command()[0],0.)

    def test_capture_uses_the_corresponding_immutable_sample(self):
        evidence_sample=sample(10,100.2,x=.5)
        self.worker.value=sample(11,100.21,x=.7)
        anchor=self.policy.capture_entry('test_up',se2(.2,0,0),.01,'survey',100.21,sample=evidence_sample)
        self.assertEqual(anchor.sequence,10)
        self.assertAlmostEqual(np.array(anchor.local_from_profile).reshape(4,4)[0,3],.3)

    def test_preflight_admission_does_not_mutate_active_hold(self):
        prior_route,prior_anchor=self.policy.route,self.policy.anchor
        self.policy.capture_entry('test_up',se2(.4,0,0),.01,'next survey',100.)
        self.policy.prepare(self.profile,100.)
        self.assertIs(self.policy.route,prior_route)
        self.assertIs(self.policy.anchor,prior_anchor)

    def test_entry_outside_support_cannot_complete(self):
        self.policy.begin_phase(Phase.ALIGN,100.)
        self.assertTrue(self.update(2,100.2,x=1.05).faulted)

    def test_arc_sweep_checks_intermediate_body_not_only_endpoints(self):
        # A full circle ends at the start, but its middle leaves this square.
        support=np.array(box(-.1,-.1,.1,.1))
        self.assertFalse(self.policy._swept_support(np.zeros(2),0.,support,1.,1.,2*math.pi,box(-.5,-.5,.5,.5),0.))
        self.assertTrue(self.policy._swept_support(np.zeros(2),0.,support,.01,.01,.7,box(-.5,-.5,.5,.5),0.))

    def test_mount_lever_arm_and_entry_transform_are_not_ignored(self):
        mount=se2(.2,-.1,.1)
        base=se2(3,4,.6);lidar=base@mount
        obs=replace(sample(),transform=tuple(lidar.flat))
        anchor=RouteAnchor.from_entry(obs,mount,se2(1,2,.3),.01,'survey')
        recovered=np.linalg.inv(np.array(anchor.local_from_profile).reshape(4,4))@lidar@np.linalg.inv(mount)
        np.testing.assert_allclose(recovered,se2(1,2,.3),atol=1e-12)


class StreamingTest(unittest.TestCase):
    def test_buffer_bounds_duplicate_and_reset_epochs(self):
        worker=TrackingWorker(TrackingSettings(frame_step=1,queue_capacity=2,imu_capacity=4),np.eye(3))
        for t in range(10):worker.push_imu((100+t*.01,0,0,0,0,0,9.8))
        self.assertEqual(len(worker._imu),4)
        self.assertFalse(worker.push_imu((100.,0,0,0,0,0,9.8)))
        for t in range(5):worker.push_scan(100+t,np.ones((3,3)),100+t,100+t)
        self.assertEqual(worker.diagnostics()['queue_length'],2)
        self.assertEqual(worker.dropped_scans,3)
        worker.push_scan(99.,np.ones((3,3)),105.,105.)
        self.assertEqual(worker.epoch,1)
        self.assertIsNone(worker.snapshot())

    def test_inflight_result_cannot_cross_reset(self):
        entered,release=threading.Event(),threading.Event()
        class Engine:
            def __init__(self,settings,rotation,checksum,epoch):self.epoch=epoch;self.last_stamp=1.
            def process(self,*args):
                entered.set();release.wait(2)
                return sample(epoch=self.epoch)
        worker=TrackingWorker(TrackingSettings(frame_step=1),np.eye(3),engine_factory=Engine)
        worker.push_imu((101,0,0,0,0,0,9.8));worker.start()
        try:
            worker.push_scan(100,np.ones((3,3)))
            self.assertTrue(entered.wait(1))
            worker.reset();release.set()
            time.sleep(.03)
            self.assertIsNone(worker.snapshot())
        finally:release.set();worker.shutdown()

    def test_bootstrap_does_not_use_only_half_of_gravity_window(self):
        engine=TrackingEngine(TrackingSettings(),np.eye(3))
        points=np.array([[1,0,0],[1,1,0],[2,0,0],[2,1,0]],float)
        late_start=np.array([[t,0,0,0,0,0,9.8] for t in np.arange(99.8,100.52,.01)])
        result=engine.process(100.,points,late_start,100.52,100.)
        self.assertIsNone(result.transform)
        self.assertIn('gravity window',result.reason)

    def test_bootstrap_waits_for_arrived_future_half_window(self):
        engine=TrackingEngine(TrackingSettings(),np.eye(3))
        points=np.array([[1,0,0],[1,1,0],[2,0,0],[2,1,0]],float)
        past=np.array([[t,0,0,0,0,0,9.8] for t in np.arange(99.5,100.01,.01)])
        first=engine.process(100.,points,past,100.,100.)
        self.assertIsNone(first.transform)
        arrived=np.array([[t,0,0,0,0,0,9.8] for t in np.arange(99.5,100.52,.01)])
        second=engine.process(100.,points,arrived,100.52,100.)
        self.assertEqual(second.state,'BOOTSTRAP')
        self.assertFalse(second.geometry_valid)


class OwnershipTest(unittest.TestCase):
    def test_cancel_with_lidar_never_disables_stair_or_closes(self):
        class Control(ScriptedEvidence):
            phase=Phase.VERIFY_ENTRY
            route={'limits':{'handoff_sec':2.}}
            def prepare(self,*args):pass
            def command(self):return (.1,0.)
            def loss_response(self,t):t.events.append(('commissioned_response',))
        control=Control();clock=FakeClock();transport=FakeTransport()
        transport.request_stair_mode_with_feedback=lambda enabled,progress:progress()
        supervisor=StairSupervisor(make_configuration(),transport,control,clock,lambda r:None,FakeAdmission(),.25,lidar_control=control)
        supervisor.start()
        result=supervisor.traverse(StairGoal('test_up',Direction.UP,'token'),lambda:True)
        self.assertTrue(result.cancelled)
        self.assertEqual(supervisor.state,SupervisorState.STAIR)
        self.assertNotIn(('stair',False),transport.events)
        self.assertNotIn(('close',),transport.events)
        supervisor.accept_navigation(.3,0)
        supervisor.stream_navigation()
        self.assertEqual(transport.events[-2],('update',0.,0.))

    def test_shutdown_cannot_be_followed_by_late_success_or_nav_state(self):
        entered,release=threading.Event(),threading.Event()
        class BlockingEvidence(ScriptedEvidence):
            def evaluate(self,*args):
                entered.set();release.wait(2)
                return super().evaluate(*args)
        evidence=BlockingEvidence();transport=FakeTransport();clock=FakeClock()
        supervisor=StairSupervisor(make_configuration(),transport,evidence,clock,lambda r:None,FakeAdmission(),.25)
        supervisor.start();results=[]
        traversal=threading.Thread(target=lambda:results.append(supervisor.traverse(StairGoal('test_up',Direction.UP,'token'),lambda:False)))
        traversal.start();self.assertTrue(entered.wait(1))
        stopper=threading.Thread(target=supervisor.shutdown);stopper.start()
        self.assertTrue(supervisor._shutdown_requested.wait(1))
        release.set();traversal.join(2);stopper.join(2)
        self.assertFalse(traversal.is_alive());self.assertFalse(stopper.is_alive())
        self.assertEqual(supervisor.state,SupervisorState.DISARMED)
        self.assertNotEqual(results[0].code,ResultCode.OK)
        self.assertEqual(transport.events[-1],('close',))

    def test_late_arrival_release_does_not_clear_a_new_nav_command(self):
        transport=FakeTransport();clock=FakeClock();evidence=ScriptedEvidence()
        supervisor=StairSupervisor(make_configuration(),transport,evidence,clock,lambda r:None,FakeAdmission(),.25)
        supervisor.start();supervisor.accept_navigation(.2,.1)
        self.assertTrue(supervisor.release_arrival_hold())
        supervisor.stream_navigation()
        self.assertEqual(transport.events[-2],('update',.2,.1))

    def test_cancel_during_final_send_does_not_commit_nav_success(self):
        cancelled=threading.Event()
        class Control(ScriptedEvidence):
            phase=Phase.EXIT_CONFIRM
            route={'limits':{'handoff_sec':2.}}
            def command(self):return (.01,0.)
            def loss_response(self,transport):transport.events.append(('commissioned_response',))
        control=Control(observations_before_true=-1);transport=FakeTransport();clock=FakeClock()
        supervisor=StairSupervisor(make_configuration(),transport,control,clock,lambda r:None,FakeAdmission(),.25,lidar_control=control)
        supervisor.start();supervisor._state=SupervisorState.STAIR
        supervisor._set_lidar_mode=lambda *args:None
        original=transport.send_current
        def send():original();cancelled.set()
        transport.send_current=send
        result=supervisor._finish(cancelled=False,cancellation_requested=cancelled.is_set)
        self.assertTrue(result.cancelled)
        self.assertEqual(supervisor.state,SupervisorState.STAIR)
        self.assertNotEqual(result.code,ResultCode.OK)

    def test_off_configuration_does_not_need_lidar_file(self):
        config=load_lidar_configuration(Path('/does/not/exist'),'off')
        self.assertEqual(config.mode,'off')
        path=Path(__file__).resolve().parents[1]/'config/stair_lidar.yaml'
        self.assertEqual(load_lidar_configuration(path,'observe').mode,'observe')
        with self.assertRaises(StairConfigurationError):load_lidar_configuration(path,'control')


if __name__=='__main__':unittest.main()
