import unittest
from types import SimpleNamespace as NS
from stair_supervisor.ros_lidar import wait_for_entry_observation
class FreshEntry(unittest.TestCase):
 def run_wait(self, rows, epoch=2):
  self.now=10.;self.reads=0
  anchor=NS(epoch=2,sequence=20,measured_at=9.4)
  def snapshot():
   row=rows[min(self.reads,len(rows)-1)];self.reads+=1
   return NS(epoch=row[0],sequence=row[1],measured_at=row[2],geometry_valid=row[3])
  def sleep(dt): self.now+=dt
  worker=NS(epoch=epoch,snapshot=snapshot)
  return wait_for_entry_observation(worker,anchor,.5,clock=lambda:self.now,sleep=sleep)
 def test_waits_for_new_valid_real_sample(self):
  r=self.run_wait([(2,20,9.4,True),(2,21,9.99,True)])
  self.assertEqual(r.sequence,21);self.assertEqual(r.measured_at,9.99)
 def test_old_sample_not_retimestamped(self):
  with self.assertRaisesRegex(ValueError,'unavailable'):self.run_wait([(2,20,9.4,True)])
  self.assertAlmostEqual(self.now,11.)
 def test_invalid_or_future_observation_not_admitted(self):
  for row in [(2,21,9.99,False),(2,21,20.,True)]:
   with self.assertRaisesRegex(ValueError,'unavailable'):self.run_wait([row])
 def test_reset_invalidates_anchor(self):
  with self.assertRaisesRegex(ValueError,'epoch changed'):self.run_wait([(3,21,9.99,True)],epoch=3)
if __name__=='__main__':unittest.main()
