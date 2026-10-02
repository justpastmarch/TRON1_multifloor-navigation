"""Boundary/entry regressions with synthetic geometry; no hardware commands."""
from dataclasses import replace
import importlib.util
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from unittest.mock import Mock
import time
import io
import threading
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from stair_supervisor.stair_feedback import StairFeedback, placed_entry_transform, se2, inside_polygon
from stair_supervisor.stair_evidence import Phase
from stair_supervisor import ros_lidar
from stair_supervisor.configuration import load_lidar_configuration, load_stair_configuration
from stair_supervisor.robot_conversion import normalize_twist
from test_lidar_control import Worker, route, sample, box
import test_lidar_ros_adapter as adapter_fixture
from test_supervisor import make_configuration


class ReadinessTest(unittest.TestCase):
    def controller(self):
        r=route()
        r.update(footprint=box(-.225,-.225,.225,.225),
                 flight_1=[[0,0,0],[2.875,0,1.53]],
                 flight_1_polygon=box(0,-.615,2.52,.615),
                 landing_polygon=box(2.52,-.615,4.,2.085),
                 turn_path=[[3.26,0,0],[3.26,0,math.pi/2],[3.26,1.47,math.pi/2]])
        r['limits'].update(margin_m=.02,anchor_uncertainty_m=.1,max_v=.15,max_w=.2,
            max_accel=.15,max_alpha=.3,warn_sec=.5,expire_sec=.9,recover_sec=2.)
        worker=Worker(); c=StairFeedback(worker,np.eye(4),[r])
        c.capture_entry('test_up',np.eye(4),.1,'synthetic fixture',100.)
        c.arm(replace(make_configuration().profiles[0],timeout_sec=300.),100.)
        return worker,c

    def test_forward_boundary_can_reduce_translation_without_erasing_margin(self):
        w,c=self.controller();c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        w.value=sample(2,100.2,x=1.50,y=-.154,yaw=math.radians(-36.5),z=.97)
        c._last_command=(.08,.2)
        result=c.evaluate(c.phase,100.21)
        self.assertFalse(result.faulted,result.detail)
        self.assertTrue(c.debug['boundary_adjusted'])
        self.assertGreater(c.command()[0],0.);self.assertLess(c.command()[0],.08)
        self.assertAlmostEqual(c.debug['effective_margin_m'],.12)
        self.assertAlmostEqual(c.command()[1],.2)
        self.assertGreaterEqual(c.command()[0],.08-.15*.21-1e-9)

    def test_command_filter_cannot_jump_outside_acceleration_window(self):
        w,c=self.controller();c.begin_phase(Phase.FORWARD_SEGMENT_1,100.)
        w.value=sample(2,100.001,x=1.50,y=-.154,yaw=math.radians(-36.5),z=.97)
        c._last_command=(.15,.2)
        result=c.evaluate(c.phase,100.001)
        self.assertTrue(result.faulted)
        self.assertEqual(c.debug['tracking_state'],'SUPPORT_LIMIT')

    def test_landing_approach_does_not_reintroduce_a_margin_at_internal_join(self):
        w,c=self.controller();c.begin_phase(Phase.LANDING,100.)
        w.value=sample(2,100.2,x=2.914,y=-.066,yaw=math.radians(24.1),z=1.60)
        result=c.evaluate(c.phase,100.21)
        self.assertFalse(result.faulted,result.detail)
        self.assertFalse(result.complete)  # body still needs full flat clearance
        self.assertGreater(c.command()[0],0.)
        self.assertEqual(c.debug['support_regions'],['flight_1_polygon','landing_polygon'])

    def test_later_turn_requires_flat_landing_and_external_edges_still_fail(self):
        w,c=self.controller();c.begin_phase(Phase.TURN_TO_NEXT_FLIGHT,100.);c.turn_index=1
        w.value=sample(2,100.2,x=2.914,y=-.066,yaw=math.radians(24.1),z=1.60)
        self.assertTrue(c.evaluate(c.phase,100.21).faulted)
        w,c=self.controller();c.begin_phase(Phase.LANDING,100.)
        w.value=sample(2,100.2,x=3.0,y=-.60,z=1.53)
        self.assertTrue(c.evaluate(c.phase,100.21).faulted)

    def test_field_configuration_has_measured_dimensions_and_useful_command_units(self):
        root=Path(__file__).resolve().parents[1]/'config'
        cfg=load_lidar_configuration(root/'stair_lidar_3f_4f_test.yaml','control',False)
        r=cfg.document['routes'][0]; profiles=load_stair_configuration(root)
        c=StairFeedback(None,cfg.document['base_from_lidar'],[r])
        if r['commissioned']:
            reference=r['entry_reference']
            self.assertIs(reference['unique_geometry_verified'],True)
            template=root/reference['path']
            self.assertEqual(hashlib.sha256(template.read_bytes()).hexdigest(),reference['sha256'])
            points=np.load(str(template),allow_pickle=False)
            self.assertGreaterEqual(len(points),200)
            self.assertEqual(points.shape[1],3)
            self.assertTrue(np.isfinite(points).all())
        else:
            self.assertNotIn('entry_reference',r)
        self.assertAlmostEqual(r['flight_1'][1][2],9*.17)
        self.assertAlmostEqual(r['flight_2'][1][2],19*.17)
        self.assertAlmostEqual(normalize_twist(r['limits']['min_flight_v'],0,profiles.robot.websocket_full_scale).x,1.)
        c.test_plan(profiles.profiles[0],Phase.FORWARD_SEGMENT_1,65.,True)
        c.test_plan(profiles.profiles[0],Phase.EXIT_CONFIRM,255.,True)
        self.assertTrue(inside_polygon(np.array(r['footprint'])+np.array(r['flight_1'][0][:2]),
                                       r['entry_polygon'],.12))

    def test_field_both_flights_reach_full_input_with_steering_and_slew(self):
        root=Path(__file__).resolve().parents[1]/'config'
        cfg=load_lidar_configuration(root/'stair_lidar_3f_4f_test.yaml','control',False)
        r=cfg.document['routes'][0]; profiles=load_stair_configuration(root)
        profile=next(p for p in profiles.profiles if p.id==r['id'])
        for phase in (Phase.FORWARD_SEGMENT_1,Phase.FORWARD_SEGMENT_2):
            with self.subTest(phase=phase):
                w=Worker(); w.value=sample(x=-.45)
                # Identity mount isolates command scaling, using the real route.
                c=StairFeedback(w,np.eye(4),[r])
                c.capture_entry(r['id'],se2(-.45,0.,0.),.1,'synthetic command test',100.)
                c.arm(profile,100.,phase_test=True)
                c.begin_phase(phase,100.)
                y,z,yaw=(0.,.7,.1) if phase is Phase.FORWARD_SEGMENT_1 else (1.47,2.2,math.pi+.1)
                commands=[]
                for i in range(1,26):
                    stamp=100.+i*.2
                    w.value=sample(i+1,stamp,x=1.,y=y,z=z,yaw=yaw)
                    result=c.evaluate(phase,stamp+.01)
                    self.assertFalse(result.faulted,result.detail)
                    commands.append(c.command())
                self.assertLess(commands[0][0],r['limits']['min_flight_v'])
                self.assertTrue(all(abs(b[0]-a[0])<=.15*.2+1e-8 for a,b in zip(commands,commands[1:])))
                sent=normalize_twist(*commands[-1],profiles.robot.websocket_full_scale)
                self.assertAlmostEqual(sent.x,1.)
                self.assertNotEqual(sent.z,0.)
                self.assertFalse(c.debug['boundary_adjusted'])
                self.assertAlmostEqual(r['limits']['hold_v'],.06)
                self.assertAlmostEqual(r['limits']['max_w'],.2)

    def test_flat_phase_caps_first_command_after_full_input_flight(self):
        root=Path(__file__).resolve().parents[1]/'config'
        cfg=load_lidar_configuration(root/'stair_lidar_3f_4f_test.yaml','control',False)
        r=cfg.document['routes'][0]
        profiles=load_stair_configuration(root)
        profile=next(p for p in profiles.profiles if p.id==r['id'])
        for phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.EXIT_CONFIRM):
            with self.subTest(phase=phase):
                w=Worker();w.value=sample(x=-.45)
                c=StairFeedback(w,np.eye(4),[r])
                c.capture_entry(r['id'],se2(-.45,0.,0.),.1,'synthetic transition test',100.)
                c.arm(profile,100.,phase_test=True)
                c._last_command=(.55,0.)
                c._last_command_at=104.
                c.begin_phase(phase,104.)
                c._flight_seen=[True,True]
                x,y,z=(3.05,0.,1.53) if phase is not Phase.EXIT_CONFIRM else (-.80,1.47,3.23)
                yaw=0. if phase is not Phase.EXIT_CONFIRM else math.pi
                w.value=sample(2,104.02,x=x,y=y,z=z,yaw=yaw)
                result=c.evaluate(phase,104.03,command_required=True)
                self.assertFalse(result.faulted,result.detail)
                self.assertLessEqual(abs(c.command()[0]),.06+1e-9)
                self.assertLessEqual(abs(normalize_twist(*c.command(),profiles.robot.websocket_full_scale).x),.06/.55+1e-9)
                self.assertLessEqual(abs(c.command()[1]),.2+1e-9)

    def test_midroute_client_keeps_reference_and_rejects_epoch_change(self):
        path=Path(__file__).resolve().parents[1]/'scripts/stair_entry_test.py'
        spec=importlib.util.spec_from_file_location('entry_client',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        evidence=dict(route_id='r',epoch=3,sensor_stamp=10.,
                      profile_from_local=se2(7.,-2.,1.2).tolist(),uncertainty_m=.1)
        renewed=module.refresh_retained_reference(evidence,dict(epoch=3),'r',80.)
        self.assertEqual(renewed['profile_from_local'],evidence['profile_from_local'])
        self.assertEqual(renewed['sensor_stamp'],80.)
        self.assertEqual(evidence['sensor_stamp'],10.)
        for status,route_id in ((dict(epoch=4),'r'),(dict(epoch=3),'other')):
            with self.assertRaises(ValueError):
                module.refresh_retained_reference(evidence,status,route_id,80.)
        with self.assertRaises(ValueError):
            module.refresh_retained_reference(dict(route_id='r',epoch=3),dict(epoch=3),'r',80.)

    def test_forward_floor_does_not_bypass_slew(self):
        w,c=self.controller();c.route['limits'].update(max_v=.30,min_flight_v=.275)
        c.begin_phase(Phase.FORWARD_SEGMENT_1,100.);w.value=sample(2,100.2,x=.1,z=.1)
        self.assertFalse(c.evaluate(c.phase,100.21).faulted)
        self.assertLessEqual(c.command()[0],.15*.21+1e-9)
        self.assertTrue(c.debug['below_flight_command_floor'])
        bad=route();bad['limits']['min_flight_v']=1.
        with self.assertRaises(ValueError):StairFeedback(w,np.eye(4),[bad])

    def test_manual_reference_preserves_gravity_and_measured_body_tilt(self):
        mount=np.eye(4);mount[2,3]=.18
        local_base=np.eye(4);local_base[:3,:3]=Rotation.from_euler('zyx',[.6,.15,-.1]).as_matrix()
        local_base[:3,3]=[.7,-.4,.8]
        reference=placed_entry_transform(local_base@mount,mount,[-.45,0,0],0.)
        measured=reference@local_base
        np.testing.assert_allclose(measured[:3,3],[-.45,0,0],atol=1e-10)
        np.testing.assert_allclose(reference[2,:3],[0,0,1],atol=1e-10)
        np.testing.assert_allclose(measured[2,:3],local_base[2,:3],atol=1e-10)
        self.assertAlmostEqual(math.atan2(measured[1,0],measured[0,0]),0.)

    def test_frame_capture_uses_new_scan_without_reseeding_body_position(self):
        adapter=adapter_fixture.AdapterTest().interface()
        evidence=dict(route_id='test_up',epoch=0,sensor_stamp=100.,
            profile_from_local=np.eye(4).tolist(),uncertainty_m=.01,source='explicit synthetic placement')
        adapter.worker.value=sample(2,100.1,x=.02)
        adapter.clock.monotonic=lambda:100.11
        with patch.object(ros_lidar.rospy,'get_param',return_value=evidence):
            result=adapter._capture_entry(None)
        self.assertTrue(result.success,result.message)
        np.testing.assert_allclose(np.array(adapter.control.anchors['test_up'].local_from_profile).reshape(4,4),np.eye(4))
        tilted=np.eye(4);tilted[:3,:3]=Rotation.from_euler('x',.1).as_matrix()
        evidence['profile_from_local']=tilted.tolist()
        with patch.object(ros_lidar.rospy,'get_param',return_value=evidence):
            self.assertFalse(adapter._capture_entry(None).success)

    def test_pending_entry_preview_never_arms_or_consumes_reference(self):
        adapter=adapter_fixture.AdapterTest().interface()
        adapter.control.capture_entry('test_up',np.eye(4),.01,'fixture',100.)
        adapter._publish_geometry(adapter.worker.value)
        markers=adapter.markers.publish.call_args[0][0].markers
        self.assertTrue(any('ENTRY PREVIEW' in m.text for m in markers))
        self.assertIsNone(adapter.control.anchor)
        self.assertIn('test_up',adapter.control.anchors)
        self.assertEqual(adapter.control.command(),(0.,0.))

    def test_client_checks_loaded_configuration_and_control_mode(self):
        path=Path(__file__).resolve().parents[1]/'scripts/stair_entry_test.py'
        spec=importlib.util.spec_from_file_location('entry_client',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        cfg=SimpleNamespace(source_sha256='x')
        status=dict(config_sha256='x',routes=[dict(id='a')],mode='observe',observe_only=True,geometry_valid=True,worker_error='',phase_test_protocol='bound-ticket-v1')
        module.check_live_status(status,cfg,'a')
        with self.assertRaises(ValueError):module.check_live_status(status,cfg,'a',control=True)
        status.update(mode='control',observe_only=False)
        module.check_live_status(status,cfg,'a',control=True)
        status['config_sha256']='other'
        with self.assertRaises(ValueError):module.check_live_status(status,cfg,'a')

    def test_bound_test_request_is_consumed_only_by_its_own_goal(self):
        from stair_supervisor import ros_node
        node=ros_node.RosStairSupervisorNode.__new__(ros_node.RosStairSupervisorNode)
        node._action=Mock();node._action.current_goal.get_goal_id.return_value=SimpleNamespace(id='specific-goal')
        node._execute_request=Mock()
        token='1234567890abcdef1234567890abcdef';key='~phase_test_requests/'+token
        data={key:dict(expires_at=time.time()+30,test={'route_id':'test_up'})}
        with patch.object(ros_node.rospy,'get_param',side_effect=lambda k,d=None:data.get(k,d)), \
             patch.object(ros_node.rospy,'set_param',side_effect=lambda k,v:data.__setitem__(k,v)), \
             patch.object(ros_node.rospy,'delete_param',side_effect=lambda k:data.pop(k)):
            # An unrelated goal cannot consume the ticket, including an empty token.
            node._execute(SimpleNamespace(admission_token='',stair_id='test_up'))
            self.assertIn(key,data)
            node._execute(SimpleNamespace(admission_token='phase-test:'+token,stair_id='test_up'))
            self.assertNotIn(key,data)
            self.assertEqual(node._execute_request.call_args[0][1],{'route_id':'test_up'})
            self.assertEqual(data['~phase_test_receipts/'+token],dict(goal_id='specific-goal',state='finished'))
            before=node._execute_request.call_count
            node._execute(SimpleNamespace(admission_token='phase-test:'+token,stair_id='test_up'))
            self.assertEqual(node._execute_request.call_count,before)
            self.assertTrue(node._action.set_aborted.called)

    def test_expired_bound_ticket_cannot_fall_back_to_normal_traversal(self):
        from stair_supervisor import ros_node
        node=ros_node.RosStairSupervisorNode.__new__(ros_node.RosStairSupervisorNode)
        node._action=Mock();node._execute_request=Mock()
        envelope=dict(expires_at=time.time()-1,test={'route_id':'test_up'})
        with patch.object(ros_node.rospy,'get_param',return_value=envelope),patch.object(ros_node.rospy,'delete_param'):
            node._execute(SimpleNamespace(admission_token='phase-test:'+'a'*32,stair_id='test_up'))
        node._execute_request.assert_not_called()
        self.assertTrue(node._action.set_aborted.called)

    def test_display_cloud_uses_matching_stamp_and_epoch_only(self):
        from livox_ros_driver2.msg import CustomMsg,CustomPoint
        from sensor_msgs.point_cloud2 import read_points
        message=CustomMsg();message.points=[CustomPoint(x=1.,y=2.,z=3.)];message.point_num=1
        buffer=io.BytesIO();message.serialize(buffer)
        adapter=adapter_fixture.AdapterTest().interface()
        adapter.cloud=Mock();adapter._raw_lock=threading.Lock()
        adapter._raw_scans=deque([(0,100.,buffer.getvalue())],maxlen=32)
        adapter._publish_cloud(sample(1,100.,x=2.,y=1.,z=.5))
        cloud=adapter.cloud.publish.call_args[0][0]
        np.testing.assert_allclose(list(read_points(cloud,field_names=('x','y','z'))),[[3.,3.,3.5]])
        self.assertEqual(cloud.header.frame_id,'stair_local_0')
        adapter._publish_cloud(sample(2,100.2))
        adapter._publish_cloud(sample(3,100.,epoch=1))
        self.assertEqual(adapter.cloud.publish.call_count,1)

    def test_concurrent_duplicate_ticket_executes_once_and_exception_is_not_finished(self):
        from stair_supervisor import ros_node
        node=ros_node.RosStairSupervisorNode.__new__(ros_node.RosStairSupervisorNode)
        node._action=Mock();node._action.current_goal.get_goal_id.return_value=SimpleNamespace(id='one-goal')
        node._execute_request=Mock(side_effect=ValueError('injected boundary error'))
        token='b'*32;key='~phase_test_requests/'+token
        data={key:dict(expires_at=time.time()+30,test={'route_id':'test_up'})}
        gate=threading.Barrier(3)
        message=SimpleNamespace(admission_token='phase-test:'+token,stair_id='test_up')
        def request():gate.wait();node._execute(message)
        with patch.object(ros_node.rospy,'get_param',side_effect=lambda k,d=None:data.get(k,d)), \
             patch.object(ros_node.rospy,'set_param',side_effect=lambda k,v:data.__setitem__(k,v)), \
             patch.object(ros_node.rospy,'delete_param',side_effect=lambda k:data.pop(k)):
            threads=[threading.Thread(target=request) for _ in range(2)]
            for thread in threads:thread.start()
            gate.wait()
            for thread in threads:thread.join(2.)
            self.assertTrue(all(not t.is_alive() for t in threads))
        self.assertEqual(node._execute_request.call_count,1)
        self.assertEqual(data['~phase_test_receipts/'+token]['state'],'failed')


class EntryFreshnessTest(unittest.TestCase):
    """First-message delay must not reject later valid input or relax admission."""
    def setUp(self):
        path = Path(__file__).resolve().parents[1]/'scripts/stair_entry_test.py'
        spec = importlib.util.spec_from_file_location('entry_freshness_client', path)
        self.client = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.client)
        self.config = SimpleNamespace(source_sha256='fixture')
        self.subscriber = Mock()
        self.callbacks = []
        self.ros = SimpleNamespace(Subscriber=Mock(side_effect=self.subscribe),
                                   is_shutdown=Mock(return_value=False), loginfo=Mock())
        self.messages = []
        self.worker = None
        self.stop = threading.Event()
        self.addCleanup(self.cleanup)

    def status(self, age, **changes):
        value = dict(config_sha256='fixture', routes=[dict(id='a')], mode='control',
                     observe_only=False, phase_test_protocol='bound-ticket-v1',
                     geometry_valid=True, worker_error='', geometry_age_sec=age)
        value.update(changes)
        return value

    def subscribe(self, _topic, _kind, callback, **_kwargs):
        def emit(value):
            self.callbacks.append(value)
            callback(SimpleNamespace(data=json.dumps(value)))
        if self.messages:
            emit(self.messages[0])
        def later():
            for value in self.messages[1:]:
                if self.stop.wait(.01):
                    return
                emit(value)
        self.worker = threading.Thread(target=later)
        self.worker.start()
        return self.subscriber

    def cleanup(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.join(1.)
            self.assertFalse(self.worker.is_alive())

    def wait(self, timeout=.5):
        return self.client.wait_for_fresh_status(self.ros, object, '/tracking',
            self.config, 'a', .5, control=True, timeout=timeout)

    def test_old_first_status_waits_for_next_fresh_status_on_one_subscription(self):
        self.messages = [self.status(.61), self.status(.31)]
        result = self.wait()
        self.assertEqual(result['geometry_age_sec'], .31)
        self.assertEqual(len(self.callbacks), 2)
        self.ros.Subscriber.assert_called_once()
        self.ros.loginfo.assert_called_once()
        self.subscriber.unregister.assert_called_once()

    def test_fresh_first_status_returns_without_wait_notice(self):
        self.messages = [self.status(.3)]
        self.assertEqual(self.wait()['geometry_age_sec'], .3)
        self.ros.loginfo.assert_not_called()
        self.subscriber.unregister.assert_called_once()

    def test_only_stale_status_times_out_instead_of_relaxing_limit(self):
        self.messages = [self.status(.7)]
        with self.assertRaisesRegex(ValueError, 'required <= 0.500s.*no goal sent'):
            self.wait(timeout=.04)
        self.subscriber.unregister.assert_called_once()

    def test_missing_topic_times_out_and_unregisters(self):
        with self.assertRaisesRegex(ValueError, 'latest age unavailable'):
            self.wait(timeout=.04)
        self.subscriber.unregister.assert_called_once()

    def test_time_spent_validating_counts_toward_freshness(self):
        self.messages = [self.status(.49)]
        validate = self.client.check_live_status
        def delayed(*args, **kwargs):
            validate(*args, **kwargs)
            time.sleep(.02)
        with patch.object(self.client, 'check_live_status', side_effect=delayed):
            with self.assertRaisesRegex(ValueError, 'no fresh LiDAR sample'):
                self.wait(timeout=.04)
        self.subscriber.unregister.assert_called_once()

    def test_time_spent_validating_cannot_bypass_timeout(self):
        self.messages = [self.status(.1)]
        validate = self.client.check_live_status
        def delayed(*args, **kwargs):
            validate(*args, **kwargs)
            time.sleep(.02)
        with patch.object(self.client, 'check_live_status', side_effect=delayed):
            with self.assertRaisesRegex(ValueError, 'no fresh LiDAR sample'):
                self.wait(timeout=.01)
        self.subscriber.unregister.assert_called_once()

    def test_configuration_mismatch_still_fails_before_later_good_sample(self):
        self.messages = [self.status(.8, config_sha256='wrong'), self.status(.3)]
        with self.assertRaisesRegex(ValueError, 'different LiDAR configuration'):
            self.wait()
        self.subscriber.unregister.assert_called_once()

    def test_worker_fault_is_not_retried_as_a_timestamp_delay(self):
        self.messages = [self.status(.8, worker_error='worker failed'), self.status(.3)]
        with self.assertRaisesRegex(ValueError, 'geometry is unavailable'):
            self.wait()
        self.subscriber.unregister.assert_called_once()

    def test_nonfinite_future_and_boolean_age_never_admit(self):
        self.messages = [self.status(age) for age in (float('nan'), float('inf'), -.1, True)]
        with self.assertRaisesRegex(ValueError, 'no fresh LiDAR sample'):
            self.wait(timeout=.1)
        self.subscriber.unregister.assert_called_once()

    def test_shutdown_exits_without_waiting_for_deadline(self):
        self.messages = [self.status(.8)]
        self.ros.is_shutdown.return_value = True
        with self.assertRaisesRegex(ValueError, 'ROS shutdown'):
            self.wait()
        self.subscriber.unregister.assert_called_once()
