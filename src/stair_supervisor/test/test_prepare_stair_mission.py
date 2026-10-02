"""Candidate registration / guarded apply use temporary files, no ROS runtime."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
import yaml

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/prepare_stair_mission.py'
spec=importlib.util.spec_from_file_location('prepare_stair_mission',str(SCRIPT))
prepare=importlib.util.module_from_spec(spec);spec.loader.exec_module(prepare)

class PreparationTest(unittest.TestCase):
    def test_capture_requires_physical_assertions_before_touching_ros(self):
        with self.assertRaisesRegex(ValueError,'capture needs'):
            prepare.capture(SimpleNamespace(placed_at_entry=False,overlay_checked=False,map_pose_checked=False))
    def test_reference_uses_placed_body_and_mount_not_arbitrary_tracking_origin(self):
        document=yaml.safe_load((SCRIPT.parents[1]/'config/stair_lidar_3f_4f_test.yaml').read_text())
        document['routes'][0]['commissioned']=False  # independent of live commissioning
        points=np.random.default_rng(9).uniform(-1,1,(250,3))
        local=np.array(document['base_from_lidar']);local[:3,3]+=[8.,4.,2.]
        world_points=points@local[:3,:3].T+local[:3,3]
        result,target=prepare.build_candidate(document,world_points,local)
        # A source point at the sensor maps through the known mounting transform
        # and the operator's body position (-.45,0,0), independent of local origin.
        expected=points@np.array(document['base_from_lidar'])[:3,:3].T+np.array(document['base_from_lidar'])[:3,3]+[-.45,0.,0.]
        np.testing.assert_allclose(target,expected,atol=1e-10)
        self.assertFalse(document['routes'][0]['commissioned'])
        self.assertTrue(result['routes'][0]['commissioned'])
    def test_apply_rejects_changed_original_and_preserves_existing_template(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);bundle=root/'bundle';bundle.mkdir()
            config=root/'live.yaml';locations=root/'locations.yaml'
            config.write_text('before');locations.write_text('locations before')
            for name in ('entry.npy','lidar.yaml','locations.yaml'):(bundle/name).write_text('candidate '+name)
            m=dict(source_config=str(config),source_locations=str(locations),source_config_sha256=prepare.digest(config),source_locations_sha256=prepare.digest(locations),candidate_hashes={n:prepare.digest(bundle/n) for n in ('entry.npy','lidar.yaml','locations.yaml')})
            (bundle/'manifest.json').write_text(json.dumps(m))
            config.write_text('user edit')
            with self.assertRaisesRegex(ValueError,'source files changed'):prepare.apply_bundle(SimpleNamespace(bundle=bundle))
            self.assertEqual(config.read_text(),'user edit')
            self.assertFalse((root/'stair_3f_4f_entry.npy').exists())
            config.write_text('before');(root/'stair_3f_4f_entry.npy').write_text('existing survey')
            with self.assertRaisesRegex(ValueError,'already exists'):prepare.apply_bundle(SimpleNamespace(bundle=bundle))
            self.assertEqual((root/'stair_3f_4f_entry.npy').read_text(),'existing survey')
    def test_apply_saves_preimages_and_installs_template_before_enabling(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);bundle=root/'bundle';bundle.mkdir()
            config=root/'live.yaml';locations=root/'locations.yaml'
            config.write_text('before');locations.write_text('locations before')
            for name in ('entry.npy','lidar.yaml','locations.yaml'):(bundle/name).write_text('candidate '+name)
            m=dict(source_config=str(config),source_locations=str(locations),source_config_sha256=prepare.digest(config),source_locations_sha256=prepare.digest(locations),candidate_hashes={n:prepare.digest(bundle/n) for n in ('entry.npy','lidar.yaml','locations.yaml')})
            (bundle/'manifest.json').write_text(json.dumps(m))
            prepare.apply_bundle(SimpleNamespace(bundle=bundle))
            self.assertEqual((bundle/'before/lidar.yaml').read_text(),'before')
            self.assertEqual(config.read_text(),'candidate lidar.yaml')
            self.assertFalse(list(root.glob('*.tmp')))

if __name__=='__main__':unittest.main()
