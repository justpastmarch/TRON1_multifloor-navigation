#!/usr/bin/env python3
"""Register the operator-surveyed first entry for the selected existing stair mission.

No velocity/action goal or robot-mode command is sent. Capture writes a candidate
bundle only. Applying it is explicit and requires restarting the existing stack.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
# stair_python.sh executes through runpy, which does not add the script directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from stair_supervisor.stair_feedback import placed_entry_transform, StairFeedback
from stair_supervisor.configuration import load_lidar_configuration


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_candidate(document, cloud_local, local_lidar, route_id="stair_3f_4f_up", commission=True):
    """Use declared physical placement, not AMCL, for the local stair frame."""
    import copy
    result = copy.deepcopy(document)
    route = next(r for r in result['routes'] if r['id'] == route_id)
    start, end = np.asarray(route['flight_1'])
    matrix = placed_entry_transform(local_lidar, result['base_from_lidar'], start,
                                   math.atan2(end[1]-start[1], end[0]-start[0]))
    points = np.asarray(cloud_local, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 200 or not np.isfinite(points).all():
        raise ValueError('need at least 200 finite points from the matching live cloud')
    target = points @ matrix[:3,:3].T + matrix[:3,3]
    route['commissioned'] = bool(commission)  # explicit operator-surveyed trial, not autonomous validation
    return result, target


def capture(args):
    if not (args.placed_at_entry and args.overlay_checked and args.map_pose_checked):
        raise ValueError('capture needs --placed-at-entry --overlay-checked --map-pose-checked; do not assert unmeasured placement')
    import threading
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import String
    from std_srvs.srv import Empty
    from scipy.spatial.transform import Rotation
    from stair_entry_test import wait_for_fresh_status
    from multifloor_manager.msg import FloorState
    import stair_supervisor
    from pkgutil import extend_path
    stair_supervisor.__path__=extend_path(stair_supervisor.__path__,stair_supervisor.__name__)
    from stair_supervisor.msg import SupervisorState

    config = load_lidar_configuration(args.config.resolve(), 'control', False)
    rospy.init_node('prepare_stair_mission', anonymous=True)
    status = wait_for_fresh_status(rospy, String, '/stair_supervisor/tracking_status',
        config, args.route, .5, control=True)
    floor = rospy.wait_for_message('/multifloor/floor_state', FloorState, timeout=5.)
    supervisor = rospy.wait_for_message('/stair_supervisor/state', SupervisorState, timeout=5.)
    if floor.floor_id != args.floor or floor.state != FloorState.READY or supervisor.state != SupervisorState.NAV:
        raise ValueError('registration requires localized '+args.floor+' and idle NAV; no active mission')
    # Exact cloud/pose stamp correspondence; do not attach a newer pose to a cloud.
    records = {}; condition = threading.Condition()
    def receive(kind, message):
        with condition:
            key=(message.header.frame_id,message.header.stamp.to_nsec())
            records.setdefault(key,{})[kind]=message
            if len(records)>30:
                del records[next(iter(records))]
            condition.notify_all()
    subs=[rospy.Subscriber('/stair_supervisor/lidar_odom',Odometry,lambda m:receive('odom',m)),
          rospy.Subscriber('/stair_supervisor/aligned_cloud',PointCloud2,lambda m:receive('cloud',m))]
    try:
        deadline=time.monotonic()+10.
        pair=None
        with condition:
            while time.monotonic()<deadline and pair is None:
                for (frame,stamp),candidate in list(records.items()):
                    if (len(candidate)==2 and frame=='stair_local_%d'%status['epoch'] and
                            0<=rospy.Time.now().to_sec()-stamp/1e9<=.5):
                        pair=candidate
                        break
                if pair is None:condition.wait(.05)
        if pair is None:raise ValueError('no fresh synchronized accepted LiDAR pose/cloud')
    finally:
        for sub in subs:sub.unregister()
    # AMCL is only used to register the NAV destination. Operator must first
    # check the scan/map overlay; covariance alone cannot resolve symmetric rooms.
    amcl=[]
    sub=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,amcl.append,queue_size=1)
    try:
        rospy.wait_for_service('/request_nomotion_update',timeout=3.)
        rospy.ServiceProxy('/request_nomotion_update',Empty)()
        deadline=time.monotonic()+3.
        while not amcl and time.monotonic()<deadline:time.sleep(.02)
        if not amcl:raise ValueError('no updated AMCL pose')
        pose=amcl[-1]
    finally:sub.unregister()
    if pose.header.frame_id!='map' or not 0<=rospy.Time.now().to_sec()-pose.header.stamp.to_sec()<=.5:
        raise ValueError('AMCL pose is stale or in a different frame')
    p,q=pose.pose.pose.position,pose.pose.pose.orientation
    if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:
        raise ValueError('invalid AMCL pose')
    # Motion between the LiDAR reference and map pose invalidates a joint survey.
    newer=rospy.wait_for_message('/stair_supervisor/lidar_odom',Odometry,timeout=3.)
    old=pair['odom'];a,b=old.pose.pose,newer.pose.pose
    qa,qb=a.orientation,b.orientation
    if (newer.header.frame_id!=old.header.frame_id or
            math.dist((a.position.x,a.position.y,a.position.z),(b.position.x,b.position.y,b.position.z))>.02 or
            (Rotation.from_quat([qa.x,qa.y,qa.z,qa.w]).inv()*Rotation.from_quat([qb.x,qb.y,qb.z,qb.w])).magnitude()>math.radians(2)):
        raise ValueError('robot moved during registration; repeat while held at the surveyed entry')
    q0=a.orientation
    local=np.eye(4);local[:3,:3]=Rotation.from_quat([q0.x,q0.y,q0.z,q0.w]).as_matrix()
    local[:3,3]=[a.position.x,a.position.y,a.position.z]
    cloud=np.array(list(point_cloud2.read_points(pair['cloud'],field_names=('x','y','z'),skip_nans=True)))
    document,target=build_candidate(config.document,cloud,local,args.route,commission=not args.reference_only)
    route=next(r for r in document['routes'] if r['id']==args.route)
    if digest(args.config)!=config.source_sha256:
        raise ValueError('source configuration changed during capture; repeat registration')
    args.output.mkdir(parents=True,exist_ok=False)
    reference=args.output/'entry.npy';np.save(str(reference),target,allow_pickle=False)
    route['entry_reference']=dict(path=args.route+'_entry.npy',sha256=digest(reference),
        yaw_candidates=[-.17453292519943295,0.,.17453292519943295],
        min_fitness=.75,max_rmse_m=.06,score_gap=.05,
        validated_anchor_error_m=route['limits']['anchor_uncertainty_m'],timeout_sec=3.,
        unique_geometry_verified=True)
    StairFeedback(None,document['base_from_lidar'],document['routes'])
    locations=args.locations.resolve()
    locations_content=locations.read_bytes()
    locations_sha256=hashlib.sha256(locations_content).hexdigest()
    location_document=yaml.safe_load(locations_content)
    entry=next(x for x in location_document['locations'] if x['id']==args.entry_location)
    entry.update(x=float(p.x),y=float(p.y),yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    (args.output/'lidar.yaml').write_text(yaml.safe_dump(document,sort_keys=False))
    (args.output/'locations.yaml').write_text(yaml.safe_dump(location_document,sort_keys=False))
    manifest=dict(source_config=str(args.config.resolve()),source_config_sha256=config.source_sha256,
        source_locations=str(locations),source_locations_sha256=locations_sha256,
        candidate_hashes={name:digest(args.output/name) for name in ('entry.npy','lidar.yaml','locations.yaml')},
        operator_assertions=['body center .45m before first riser, centered and facing upstairs',
            'unique stair-entry geometry and route overlay checked; anchor error within existing .10m budget',
            args.floor+' scan/map pose visually checked'],
        evidence='operator survey; fitness is not an independent position-error measurement',
        waiting_location=args.waiting_location,reference_name=args.route+'_entry.npy',route_id=args.route,robot_commands_sent=0)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))
    print('Candidate saved. Inspect entry.npy/locations.yaml, then apply this bundle and restart existing stack.')


def apply_bundle(args):
    import os
    manifest=json.loads((args.bundle/'manifest.json').read_text())
    config=Path(manifest['source_config']);locations=Path(manifest['source_locations'])
    if digest(config)!=manifest['source_config_sha256'] or digest(locations)!=manifest['source_locations_sha256']:
        raise ValueError('source files changed since capture; no files written')
    for name,expected in manifest['candidate_hashes'].items():
        if digest(args.bundle/name)!=expected:raise ValueError('candidate changed: '+name)
    reference_name=manifest.get('reference_name','stair_3f_4f_entry.npy')
    if Path(reference_name).name!=reference_name or not reference_name.endswith('.npy'):
        raise ValueError('invalid reference filename')
    target=config.parent/reference_name
    if target.exists():raise ValueError('entry template already exists; preserve it and explicitly recommission instead')
    backup=args.bundle/'before';backup.mkdir(exist_ok=False)
    (backup/'lidar.yaml').write_bytes(config.read_bytes())
    (backup/'locations.yaml').write_bytes(locations.read_bytes())
    # Enable route last: an interrupted apply never enables without its template.
    for dest,source in ((target,'entry.npy'),(locations,'locations.yaml'),(config,'lidar.yaml')):
        temporary=dest.with_name(dest.name+'.mission-prepare.tmp')
        with temporary.open('xb') as f:f.write((args.bundle/source).read_bytes())
        os.replace(str(temporary),str(dest))
    print('Applied operator-surveyed entry. Restart the existing stack before testing. Robot commands sent: 0.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='operation',required=True)
    capture_parser=sub.add_parser('capture')
    capture_parser.add_argument('config',type=Path)
    capture_parser.add_argument('--locations',type=Path,required=True)
    capture_parser.add_argument('--output',type=Path,required=True)
    capture_parser.add_argument('--route',default='stair_3f_4f_up')
    capture_parser.add_argument('--floor',default='3F')
    capture_parser.add_argument('--entry-location',default='stair_3f_up_entry')
    capture_parser.add_argument('--waiting-location',default='stair_4f_from_3f')
    capture_parser.add_argument('--reference-only',action='store_true',help='capture entry without commissioning unverified route geometry')
    for flag in ('placed-at-entry','overlay-checked','map-pose-checked'):
        capture_parser.add_argument('--'+flag,action='store_true')
    apply_parser=sub.add_parser('apply');apply_parser.add_argument('bundle',type=Path)
    args=parser.parse_args()
    try:
        capture(args) if args.operation=='capture' else apply_bundle(args)
    except (ValueError,OSError) as error:
        parser.exit(2,str(error)+'\n')

if __name__=='__main__':main()
