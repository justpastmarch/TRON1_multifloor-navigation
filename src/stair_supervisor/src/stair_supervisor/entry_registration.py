"""Entry-only gravity correction and four-DOF registration; no motion output."""
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from .lidar_tracking_core import ImuRotation, level_transform_from_gravity


def entry_level(sample, context):
    """Infer up at the scan timestamp, compensating rotation inside the window.

    A tracker frame is gravity initialized only once and can accumulate tilt.
    Never erase that tilt by projecting an unrelated 6-DOF fit to world XY.
    Missing/unstable independent IMU evidence is an explicit error; the caller
    may retain the original gravity-preserving admission path, not force a fit.
    """
    if context['epoch'] != sample.epoch:
        raise ValueError('entry IMU epoch mismatch')
    rows=np.asarray(context['rows'],dtype=float)
    if rows.ndim!=2 or rows.shape[1]!=7:
        raise ValueError('entry gravity IMU unavailable')
    rows=rows[(rows[:,0]>=sample.stamp-.55)&(rows[:,0]<=sample.stamp+.03)]
    if len(rows)<20 or rows[-1,0]<sample.stamp or rows[0,0]>sample.stamp-.3:
        raise ValueError('entry gravity IMU window incomplete')
    imu=ImuRotation(rows,np.asarray(context['lidar_from_imu']),context['max_gap'])
    selected=rows[(rows[:,0]>=sample.stamp-.5)&(rows[:,0]<=sample.stamp)]
    imu._support(selected[0,0],sample.stamp)
    rotations=imu.interpolator(selected[:,0]-imu.times[0]).as_matrix()
    reference=imu.interpolator([sample.stamp-imu.times[0]]).as_matrix()[0]
    accelerations=selected[:,4:7]@np.asarray(context['lidar_from_imu']).T
    corrected=np.einsum('nij,nj->ni',np.einsum('ij,njk->nik',reference.T,rotations),accelerations)
    median=np.median(corrected,axis=0);norm=float(np.linalg.norm(median))
    # Livox uses g; also accept an SI IMU. Do not treat free fall as gravity.
    if not (.8<=norm<=1.2 or .8*9.80665<=norm<=1.2*9.80665):
        raise ValueError('entry gravity acceleration magnitude invalid')
    # Impact outliers do not invalidate the median by their peak magnitude.
    # The caller independently checks geometric up against this IMU estimate.
    local=np.asarray(sample.transform).reshape(4,4)[:3,:3]@(median/norm)
    return level_transform_from_gravity(local)


def refine_upright(source,target,initial,config,deadline):
    """Bounded point-to-point ICP in yaw+XYZ, keeping measured gravity fixed."""
    tree=cKDTree(target);pose=initial.copy()
    for _ in range(config.max_iterations):
        if time.monotonic()>=deadline:
            raise ValueError('entry registration time budget expired')
        points=source@pose[:3,:3].T+pose[:3,3]
        distance,index=tree.query(points);mask=distance<config.correspondence_distance_m
        if mask.sum()<3:break
        a,b=points[mask],target[index[mask]]
        ac,bc=a-a.mean(0),b-b.mean(0)
        yaw=math.atan2(float(np.sum(ac[:,0]*bc[:,1]-ac[:,1]*bc[:,0])),float(np.sum(ac[:,:2]*bc[:,:2])))
        c,s=math.cos(yaw),math.sin(yaw)
        step=np.eye(4);step[:2,:2]=[[c,-s],[s,c]]
        step[:3,3]=b.mean(0)-step[:3,:3]@a.mean(0)
        pose=step@pose
        if abs(yaw)<1e-5 and np.linalg.norm(step[:3,3])<1e-5:break
    return pose
