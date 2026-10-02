"""Median input plumbing, NOT a commissioned LiDAR RF return route."""
import unittest, tempfile, shutil
from pathlib import Path
from dataclasses import replace
import yaml
from test_supervisor import make_configuration,FakeClock,FakeTransport,FakeAdmission,ScriptedEvidence
from stair_supervisor.configuration import load_stair_configuration,StairConfigurationError,Direction
from stair_supervisor.supervisor import StairSupervisor
from stair_supervisor.stair_evidence import Phase
from stair_supervisor.robot_conversion import normalize_twist

class DownhillInputTest(unittest.TestCase):
 def config(self, mutation=None):
  with tempfile.TemporaryDirectory(prefix='tron-v23-config-') as temp:
   p=Path(temp)
   for f in ['stair_profiles.yaml','robot.yaml']:
    source=Path(__file__).resolve().parents[1]/'config'/f
    shutil.copy2(source,p/f)
   if mutation:
    doc=yaml.safe_load((p/'stair_profiles.yaml').read_text());mutation(doc)
    (p/'stair_profiles.yaml').write_text(yaml.safe_dump(doc))
   return load_stair_configuration(p)
 def test_medians_reach_normalized_transport_without_double_scaling(self):
  config=self.config();p=next(p for p in config.profiles if p.id=='stair_5f_rf_down')
  c=StairSupervisor(config,FakeTransport(),ScriptedEvidence(),FakeClock(),lambda r:None,admission=FakeAdmission(),nav_freshness_sec=.2)
  for phase,expected in [(Phase.FORWARD_SEGMENT_1,.09),(Phase.FORWARD_SEGMENT_2,.10)]:
   v,w=c._phase_command(phase,p);sent=normalize_twist(v,w,config.robot.websocket_full_scale)
   self.assertAlmostEqual(sent.x,expected);self.assertEqual(sent.z,0.)
  self.assertGreater(p.flight_1_distance_m,0);self.assertGreater(p.flight_2_distance_m,0)
 def test_up_and_other_profiles_keep_legacy_inputs(self):
  config=self.config();c=StairSupervisor(config,FakeTransport(),ScriptedEvidence(),FakeClock(),lambda r:None,admission=FakeAdmission(),nav_freshness_sec=.2)
  for p in config.profiles:
   if p.id=='stair_5f_rf_down':continue
   self.assertEqual(p.downhill_forward_inputs,())
   v,w=c._phase_command(Phase.FORWARD_SEGMENT_1,p)
   self.assertAlmostEqual(abs(v),min(p.linear_speed,config.robot.websocket_full_scale.linear_mps))
 def test_invalid_medians_are_rejected(self):
  for values in [[-.1,.1],[0.,.1],[1.1,.1],[float('nan'),.1],[True,.1],[.1],'.1']:
   with self.subTest(values=values):
    with self.assertRaises(StairConfigurationError):
     self.config(lambda d:d['profiles'][-1].update(downhill_forward_inputs=values))
 def test_forward_inputs_cannot_use_reverse_distances(self):
  with self.assertRaises(StairConfigurationError):
   self.config(lambda d:d['profiles'][-1].update(flight_1_distance_m=-1.))
 def test_median_policy_cannot_leak_to_up(self):
  with self.assertRaises(StairConfigurationError):
   self.config(lambda d:d['profiles'][0].update(downhill_forward_inputs=[.09,.1]))

if __name__=='__main__':unittest.main()
