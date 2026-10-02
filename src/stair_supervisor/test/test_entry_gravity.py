import math,time,unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from stair_supervisor.entry_registration import entry_level,refine_upright
from stair_supervisor import ros_lidar
from stair_supervisor.lidar_tracking import TrackingSettings
from test_lidar_control import route

class GravityEntryTest(unittest.TestCase):
    def inputs(self,tilt=8.,spin=0.):
        D=np.eye(4);D[:3,:3]=Rotation.from_euler('x',tilt,degrees=True).as_matrix()
        t=np.arange(-.55,.031,.005);rows=np.zeros((len(t),7));rows[:,0]=t;rows[:,2]=spin
        rows[:,4:7]=Rotation.from_euler('y',-spin*t).apply(np.tile([0.,0.,1.],(len(t),1)))
        sample=SimpleNamespace(epoch=3,sequence=1,stamp=0.,transform=tuple(D.flat),measured_at=1.)
        return D,sample,dict(epoch=3,rows=rows,lidar_from_imu=np.eye(3),max_gap=.03)
    def test_known_gravity_drift_is_removed_without_flattening_robot(self):
        D,s,c=self.inputs()
        np.testing.assert_allclose(entry_level(s,c)@D,np.eye(4),atol=1e-10)
    def test_rotation_within_window_is_compensated(self):
        D,s,c=self.inputs(spin=.8)
        np.testing.assert_allclose(entry_level(s,c)@D,np.eye(4),atol=1e-10)
    def test_nonidentity_imu_mount_rotates_both_gyro_and_acceleration(self):
        D,s,c=self.inputs(spin=.8)
        calibration=Rotation.from_euler('xyz',[27,-18,43],degrees=True).as_matrix()
        c['rows'][:,1:4]=c['rows'][:,1:4]@calibration
        c['rows'][:,4:7]=c['rows'][:,4:7]@calibration
        c['lidar_from_imu']=calibration
        np.testing.assert_allclose(entry_level(s,c)@D,np.eye(4),atol=1e-9)
    def test_impact_outliers_do_not_replace_median(self):
        D,s,c=self.inputs(spin=.8);c['rows'][::5,4:7]+=[.8,.3,1.]
        np.testing.assert_allclose(entry_level(s,c)@D,np.eye(4),atol=1e-9)
    def test_reject_epoch_gap_freefall_and_missing_window(self):
        for mutate in [lambda c:c.update(epoch=9),lambda c:c.update(rows=c['rows'][::10]),
                       lambda c:c['rows'].__setitem__((slice(None),slice(4,7)),0.),
                       lambda c:c.update(rows=c['rows'][-5:])]:
            _,s,c=self.inputs();mutate(c)
            with self.assertRaises(ValueError):entry_level(s,c)
    def test_actual_fit_keeps_corrected_frame(self):
        D,s,c=self.inputs();target=np.random.default_rng(31).uniform(-2,2,(900,3));source=target@D[:3,:3].T
        result=SimpleNamespace(accepted=True,fitness=1.,inlier_rmse_m=.001,transform=np.linalg.inv(D))
        r=route();r['flight_1']=[[0,0,0],[3,0,1.7]]
        ref=dict(yaw_candidates=[0.],min_fitness=.9,max_rmse_m=.02,score_gap=.05,validated_anchor_error_m=.01,sha256='synthetic')
        with patch.object(ros_lidar,'register_scan_to_map',return_value=result):
            anchor=ros_lidar.fit_entry_template(source,s,r,target,ref,np.eye(4),TrackingSettings().registration._replace(voxel_size_m=.03),imu_context=c)
        np.testing.assert_allclose(np.array(anchor.local_from_profile).reshape(4,4),D,atol=.01)
    def test_bad_geometric_tilt_not_authorized_by_imu(self):
        D,s,c=self.inputs(tilt=0.);points=np.random.default_rng(31).uniform(-2,2,(900,3));bad=np.eye(4);bad[:3,:3]=Rotation.from_euler('x',15,degrees=True).as_matrix()
        result=SimpleNamespace(accepted=True,fitness=1.,inlier_rmse_m=.001,transform=bad)
        r=route();r['flight_1']=[[0,0,0],[3,0,1.7]]
        ref=dict(yaw_candidates=[0.],min_fitness=.9,max_rmse_m=.02,score_gap=.05,validated_anchor_error_m=.01,sha256='synthetic')
        with patch.object(ros_lidar,'register_scan_to_map',return_value=result),self.assertRaises(ValueError):
            ros_lidar.fit_entry_template(points,s,r,points@bad[:3,:3].T,ref,np.eye(4),TrackingSettings().registration._replace(voxel_size_m=.03),imu_context=c)
    def test_expired_refinement_has_no_result(self):
        _,_,_=self.inputs();points=np.random.default_rng(3).uniform(-1,1,(30,3))
        with self.assertRaisesRegex(ValueError,'time budget'):
            refine_upright(points,points,np.eye(4),TrackingSettings().registration,time.monotonic()-1)
