import unittest
from mission_manager.navigation_recovery import wait_for_recovery

class RecoveryTest(unittest.TestCase):
    def run_wait(self, fresh, cancelled=lambda t:False):
        self.t=0
        def sleep(dt):self.t+=dt
        return wait_for_recovery(lambda:fresh(self.t),lambda:cancelled(self.t),
                                 clock=lambda:self.t,sleep=sleep)
    def test_outage_then_stable_recovery(self):
        self.assertTrue(self.run_wait(lambda t:t>=13.7))
        self.assertGreaterEqual(self.t,14.2)
    def test_single_fresh_sample_does_not_recover(self):
        self.assertFalse(self.run_wait(lambda t:14<=t<14.1))
        self.assertLess(self.t,30.1)
    def test_stop_interrupts_wait(self):
        self.assertFalse(self.run_wait(lambda t:False,lambda t:t>=2))
        self.assertLess(self.t,2.1)
    def test_intermittent_tf_restarts_stability_window(self):
        self.assertTrue(self.run_wait(lambda t:1<=t<1.2 or t>=5))
        self.assertGreaterEqual(self.t,5.5)
