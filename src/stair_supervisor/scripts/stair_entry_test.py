#!/usr/bin/env python3
"""One-shot client of the existing Supervisor: preview placement, then phase test."""
import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))


def check_live_status(status, config, route_id, *, control=False):
    if status.get('config_sha256') != config.source_sha256:
        raise ValueError('Supervisor is using a different LiDAR configuration')
    if not any(r['id'] == route_id for r in status.get('routes', [])):
        raise ValueError('requested route is not loaded')
    if control and (status.get('mode') != 'control' or status.get('observe_only') is not False):
        raise ValueError('run requires the existing Supervisor in control mode')
    if control and status.get('phase_test_protocol') != 'bound-ticket-v1':
        raise ValueError('restart the updated Supervisor before using bound phase tests')
    if status.get('geometry_valid') is not True or status.get('worker_error'):
        raise ValueError('current LiDAR geometry is unavailable')


def wait_for_fresh_status(ros, message_type, topic, config, route_id,
                          max_age_sec, *, control=False, timeout=8.):
    """Wait on one subscription; the server still rechecks freshness at admission.

    Reported ages remain publisher values; pre-receipt transport time is unknown.
    """
    condition = threading.Condition()
    latest = [None]
    deadline = time.monotonic() + timeout
    last_age = None
    announced = False

    def receive(message):
        with condition:
            latest[0] = (message.data, time.monotonic())
            condition.notify_all()

    subscriber = ros.Subscriber(topic, message_type, receive, queue_size=1)
    try:
        with condition:
            while not ros.is_shutdown():
                now = time.monotonic()
                if now >= deadline:
                    detail = 'unavailable' if last_age is None else '%.3fs' % last_age
                    raise ValueError('no fresh LiDAR sample within %.1fs '
                                     '(latest age %s; required <= %.3fs); no goal sent'
                                     % (timeout, detail, max_age_sec))
                if latest[0] is not None:
                    raw, received_at = latest[0]
                    status = json.loads(raw)
                    check_live_status(status, config, route_id, control=control)
                    now = time.monotonic()
                    age = status.get('geometry_age_sec')
                    valid_age = type(age) in (int, float) and math.isfinite(age) and age >= 0
                    last_age = age + max(0., now - received_at) if valid_age else None
                    if now >= deadline:
                        continue
                    if last_age is not None and last_age <= max_age_sec:
                        return status
                    if not announced:
                        ros.loginfo('Waiting for a fresh LiDAR sample (<= %.3fs); no goal sent.', max_age_sec)
                        announced = True
                condition.wait(min(.1, deadline - now))
        raise ValueError('ROS shutdown while waiting for fresh LiDAR; no goal sent')
    finally:
        subscriber.unregister()


def refresh_retained_reference(evidence, status, route_id, sensor_stamp):
    """Renew scan evidence without moving or rotating the existing route frame."""
    if evidence.get('route_id') != route_id or evidence.get('epoch') != status.get('epoch'):
        raise ValueError('retained route reference is missing or its tracking epoch changed')
    if 'profile_from_local' not in evidence:
        raise ValueError('no retained fixed route frame; do not use first-entry placement on a landing')
    result = dict(evidence)
    result['sensor_stamp'] = sensor_stamp
    result['source'] = 'operator mid-route test; retained route frame, current scan; no position relabeling'
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('preview', 'run'))
    parser.add_argument('config', type=Path)
    parser.add_argument('--route', default='stair_3f_4f_up')
    parser.add_argument('--namespace', default='/stair_supervisor')
    parser.add_argument('--placed-at-entry', action='store_true',
                        help='Confirm body is at the configured first-flight start, facing upstairs')
    parser.add_argument('--reuse-reference', action='store_true',
                        help='Preview current pose in the retained route frame; no position relabeling')
    parser.add_argument('--single-phase', action='store_true',
                        help='Run only the selected phase; no final floor arrival')
    parser.add_argument('--phase', default='FORWARD_SEGMENT_1',
                        help='Last phase from VERIFY_ENTRY, or the only phase with --single-phase')
    parser.add_argument('--seconds', type=float, default=65.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    if args.operation == 'preview' and args.placed_at_entry == args.reuse_reference:
        parser.error('preview requires exactly one of --placed-at-entry or --reuse-reference')
    if args.operation == 'run' and (args.placed_at_entry or args.reuse_reference):
        parser.error('placement/reference flags apply to preview only')
    if args.single_phase and args.phase == 'EXIT_CONFIRM':
        parser.error('EXIT_CONFIRM requires a connected entry test; not a single-phase test')

    import numpy as np
    from scipy.spatial.transform import Rotation
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from stair_supervisor.configuration import load_lidar_configuration, load_stair_configuration
    from stair_supervisor.stair_feedback import StairFeedback, placed_entry_transform
    from stair_supervisor.stair_evidence import Phase

    config = load_lidar_configuration(args.config.resolve(), 'control', False)
    policy = StairFeedback(None, config.document['base_from_lidar'], config.document['routes'])
    route = policy.routes[args.route]
    ns = args.namespace.rstrip('/')
    rospy.init_node('stair_entry_test_client', anonymous=True, disable_signals=True)

    def receive(topic):
        return json.loads(rospy.wait_for_message(ns+'/'+topic, String, timeout=8.).data)

    status = wait_for_fresh_status(rospy, String, ns+'/tracking_status', config,
                                  args.route, route['limits']['warn_sec'],
                                  control=args.operation == 'run')

    if args.operation == 'preview':
        odom = rospy.wait_for_message(ns+'/lidar_odom', Odometry, timeout=8.)
        if odom.header.frame_id != 'stair_local_%d' % status['epoch']:
            raise ValueError('tracking epoch changed; repeat preview at the configured entry')
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        local_lidar = np.eye(4)
        local_lidar[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        local_lidar[:3, 3] = [p.x, p.y, p.z]
        start, end = np.asarray(route['flight_1'])
        yaw = math.atan2(end[1]-start[1], end[0]-start[0])
        profile_local = placed_entry_transform(local_lidar, config.document['base_from_lidar'], start, yaw)
        if args.reuse_reference:
            evidence = refresh_retained_reference(rospy.get_param(ns+'/entry_evidence', {}),
                                                   status, args.route, odom.header.stamp.to_sec())
        else:
            evidence = dict(route_id=args.route, epoch=status['epoch'], sensor_stamp=odom.header.stamp.to_sec(),
                            profile_from_local=profile_local.tolist(),
                            uncertainty_m=route['limits']['anchor_uncertainty_m'],
                            source='operator-declared placement at route start; geometry overlay requires inspection; not automatic recognition')
        rospy.set_param(ns+'/entry_evidence', evidence)
        rospy.wait_for_service(ns+'/capture_entry', timeout=8.)
        response = rospy.ServiceProxy(ns+'/capture_entry', Trigger)()
        if not response.success:
            raise ValueError(response.message)
        print(json.dumps(dict(preview='captured; no goal or velocity sent', route=args.route,
            reference_mode='retained route frame' if args.reuse_reference else 'declared first entry',
            placement_xyz=None if args.reuse_reference else start.tolist(),
            heading_deg=None if args.reuse_reference else math.degrees(yaw),
            preview_window_sec=route['limits']['anchor_max_age_sec'],
            next='Inspect entry/landing/exit overlay in RViz, then use run with the same configuration.'), indent=2))
        return 0

    import actionlib
    # Direct checkout execution loads source first; include catkin-generated messages.
    import stair_supervisor
    from pkgutil import extend_path
    stair_supervisor.__path__ = extend_path(stair_supervisor.__path__, stair_supervisor.__name__)
    from stair_supervisor.msg import StairTraversalAction, StairTraversalGoal
    profiles = load_stair_configuration(Path(rospy.get_param(ns+'/config_dir', str(args.config.resolve().parent)))).profiles
    profile = next(p for p in profiles if p.id == args.route and p.enabled)
    phase = Phase(args.phase)
    policy.test_plan(profile, phase, args.seconds, not args.single_phase)
    client = actionlib.SimpleActionClient('/stair_traversal', StairTraversalAction)
    if not client.wait_for_server(rospy.Duration(8.)):
        raise ValueError('stair action server is unavailable')
    # Inspection never rewrites the reference to the robot's newer position.
    # The Supervisor validates the same pending anchor/current region on admission.
    debug = receive('control_debug')
    pending = next((a for a in debug.get('pending_entries', [])
                    if a['route_id'] == args.route and a['epoch'] == status['epoch']), None)
    if pending is None or not 0 <= pending['age_sec'] <= route['limits']['anchor_max_age_sec']:
        raise ValueError('no current entry preview; place the robot and preview again')
    request = dict(route_id=args.route, phase=phase.value, max_duration_sec=args.seconds,
                   operator_confirmed=True, from_entry=not args.single_phase)
    identifier = uuid.uuid4().hex
    request_key = ns+'/phase_test_requests/'+identifier
    receipt_key = ns+'/phase_test_receipts/'+identifier
    envelope = dict(expires_at=time.time()+30., test=request)
    goal = StairTraversalGoal(stair_id=args.route, direction=StairTraversalGoal.UP,
                              admission_token='phase-test:'+identifier)
    attempted = False
    owned_request = False
    finished = False
    last_phase = [None]

    def feedback(value):
        if value.phase != last_phase[0]:
            print(value.phase+': '+value.detail, flush=True)
            last_phase[0] = value.phase

    try:
        rospy.set_param(request_key, envelope)
        owned_request = True
        attempted = True
        client.send_goal(goal, feedback_cb=feedback)
        if not client.wait_for_result(rospy.Duration(args.seconds+route['limits']['handoff_sec']+10.)):
            raise RuntimeError('test result timed out; cancelling this goal')
        result = client.get_result()
        finished = True
        if result is None:
            raise RuntimeError('test ended without a result message')
        print(json.dumps(dict(result_code=result.result_code, reason=result.reason,
                              meaning='phase test only; not a floor-arrival mission result'), indent=2))
        return 0 if result.reason.startswith('phase test target reached') else 2
    finally:
        if owned_request and rospy.has_param(request_key):
            rospy.delete_param(request_key)
        if attempted and not finished:
            # Reconcile even when send_goal was interrupted before assigning its
            # client handle. The server receipt carries only this ticket's goal ID.
            from actionlib_msgs.msg import GoalID
            cancel = rospy.Publisher('/stair_traversal/cancel', GoalID, queue_size=1)
            deadline = time.monotonic()+3.
            acknowledged = False
            while time.monotonic() < deadline:
                receipt = rospy.get_param(receipt_key, None)
                if receipt and receipt.get('state') == 'finished':
                    acknowledged = True
                    break
                if receipt and receipt.get('goal_id') and cancel.get_num_connections():
                    cancel.publish(GoalID(id=receipt['goal_id']))
                time.sleep(.05)
            print('Test handling termination acknowledged; physical stop is not verified.' if acknowledged else
                  'Test termination is UNCONFIRMED; use the already established RC takeover.', file=sys.stderr)
        receipt = rospy.get_param(receipt_key, None)
        deadline = time.monotonic()+.5
        while finished and receipt and receipt.get('state') != 'finished' and time.monotonic() < deadline:
            time.sleep(.02)
            receipt = rospy.get_param(receipt_key, None)
        if receipt and receipt.get('state') == 'finished':
            rospy.delete_param(receipt_key)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, OSError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(2)
