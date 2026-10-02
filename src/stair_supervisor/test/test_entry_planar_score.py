"""Entry admission checks the installed gravity-preserving transform."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from test_lidar_control import route, sample
from stair_supervisor import ros_lidar
from stair_supervisor.lidar_tracking import TrackingSettings

class PlanarEntryTest(unittest.TestCase):
    def attempt(self, source, target, tilt):
        fitted=np.eye(4);fitted[:3,:3]=Rotation.from_euler('x',tilt,degrees=True).as_matrix()
        result=SimpleNamespace(accepted=True,fitness=1.,inlier_rmse_m=.001,transform=fitted)
        reference=dict(yaw_candidates=[0.],min_fitness=.9,max_rmse_m=.02,score_gap=.05,
                       validated_anchor_error_m=.01,sha256='synthetic')
        with patch.object(ros_lidar,'register_scan_to_map',return_value=result):
            return ros_lidar.fit_entry_template(source,sample(),route(),target,reference,
                                                np.eye(4),TrackingSettings().registration)
    def test_small_solver_tilt_allowed_when_installed_transform_fits(self):
        points=np.random.default_rng(41).uniform(-3,3,(500,3))
        anchor=self.attempt(points,points,2.)
        transform=np.array(anchor.local_from_profile).reshape(4,4)
        np.testing.assert_allclose(transform,np.eye(4),atol=1e-10)
    def test_good_6d_fit_does_not_hide_bad_projected_fit(self):
        points=np.random.default_rng(91).uniform(-3,3,(500,3))
        target=points@Rotation.from_euler('x',15,degrees=True).as_matrix().T
        with self.assertRaisesRegex(ValueError,'missing or ambiguous'):
            self.attempt(points,target,15.)
    def test_upside_down_still_rejected(self):
        points=np.random.default_rng(4).uniform(-1,1,(300,3))
        with self.assertRaisesRegex(ValueError,'missing or ambiguous'):
            self.attempt(points,points,180.)
