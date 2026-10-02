"""Surveyed floor-map/entry transform for forward RF descent.

AMCL supplies the local offset from the registered RF stair endpoint. This is
not an ICP result and cannot disambiguate a wrong map pose on its own.
"""
import math
import numpy as np
from .stair_feedback import se2,transform,RouteAnchor,placed_entry_transform


def map_entry_anchor(route, location, map_pose, covariance, sample, base_from_lidar,
                     floor, now, ros_now, pose_stamp):
    if floor != route['entry_map']['floor_id'] or location['floor_id']!=floor:
        raise ValueError('downhill entry map/floor mismatch')
    if not 0<=ros_now-pose_stamp<=1. or not sample.geometry_valid or not 0<=now-sample.measured_at<=route['limits']['warn_sec']:
        raise ValueError('downhill entry needs fresh map pose and LiDAR observation')
    if abs(sample.stamp-pose_stamp)>1.:
        raise ValueError('map pose and LiDAR timestamps do not overlap')
    if (len(covariance)!=3 or any(not math.isfinite(v) or v<0 or v>.25 for v in covariance)
            or len(map_pose)!=3 or not all(map(math.isfinite,map_pose))):
        raise ValueError('invalid or uncertain RF map localization')
    start,end=np.asarray(route['flight_1'],float)
    yaw=math.atan2(end[1]-start[1],end[0]-start[0])
    map_entry=se2(location['x'],location['y'],location['yaw'])
    profile_entry=se2(start[0],start[1],yaw)
    profile_from_map=profile_entry @ np.linalg.inv(map_entry)
    body=profile_from_map @ se2(*map_pose);body[2,3]=start[2]
    # Real map offsets survive into the entry geometry; do not snap the robot
    # to the nominal entry or label a manually assumed position as observed.
    body_yaw=math.atan2(body[1,0],body[0,0])
    local_from_profile=np.linalg.inv(placed_entry_transform(sample.transform,base_from_lidar,body[:3,3],body_yaw))
    # Covariance is not an error guarantee. Carry a two-sigma XY/yaw
    # allowance into the existing support test instead of relabeling it 10 cm.
    radius=max(math.hypot(*xy) for xy in route['footprint'])
    uncertainty=max(route['limits']['anchor_uncertainty_m'],
                    2*math.sqrt(max(covariance[:2]))+2*radius*math.sin(min(math.pi,2*math.sqrt(covariance[2]))/2))
    return RouteAnchor(sample.epoch,sample.sequence,sample.measured_at,tuple(local_from_profile.flat),
        uncertainty,'surveyed RF map endpoint + fresh AMCL offset; no ICP match')
