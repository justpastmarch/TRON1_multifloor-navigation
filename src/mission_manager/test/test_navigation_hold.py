import threading
import time
from types import SimpleNamespace
import unittest

from mission_manager.configuration import Location
from mission_manager.navigation_executor import NavigationOutcome
from mission_manager.navigation_hold import NavigationHold


class Navigation:
    def __init__(self):
        self.started=threading.Event();self.cancelled=threading.Event();self.calls=[]
    def execute(self,request,cancellation_requested=lambda:False):
        if cancellation_requested():return SimpleNamespace(outcome=NavigationOutcome.CANCELLED)
        self.calls.append(request);self.started.set()
        self.cancelled.wait(2)
        return SimpleNamespace(outcome=NavigationOutcome.CANCELLED)
    def request_cancel(self):self.cancelled.set()
    def request_cancel_for(self,request):
        if self.calls and self.calls[-1] is request:self.cancelled.set()


class HoldTest(unittest.TestCase):
    def setUp(self):
        self.nav=Navigation();self.pose=('3F',1,0.,0.,0.);self.now=0.
        self.hold=NavigationHold(self.nav,lambda:self.pose,.2,.1,.2,.1,5.,1.,lambda:self.now)
        self.hold.arm(Location('arrival','3F','waypoint',0.,0.,0.),1)
    def tearDown(self):self.hold.suspend()

    def test_no_chattering_and_fixed_destination(self):
        self.pose=('3F',1,.15,0.,0.);self.hold.tick()
        self.assertEqual(self.nav.calls,[])
        self.pose=('3F',1,.3,0.,0.);self.hold.tick()
        self.assertTrue(self.nav.started.wait(1))
        self.assertEqual(self.nav.calls[0].location.x,0.)
        self.pose=('3F',1,.09,0.,0.);self.hold.tick()
        self.assertTrue(self.nav.cancelled.wait(1))
        self.hold._thread.join(1)
        self.hold.tick();self.assertEqual(len(self.nav.calls),1)

    def test_new_mission_waits_for_terminal_cancellation(self):
        self.pose=('3F',1,.3,0.,0.);self.hold.tick()
        self.assertTrue(self.nav.started.wait(1))
        self.hold.suspend()
        self.assertFalse(self.hold._thread.is_alive())
        self.hold.tick();self.assertEqual(len(self.nav.calls),1)

    def test_floor_change_and_stale_pose_disable_only_hold(self):
        self.pose=('4F',2,.3,0.,0.);self.hold.tick()
        self.assertEqual(self.nav.calls,[])
        self.assertIsNone(self.hold._target)

    def test_move_base_success_outside_precision_budget_does_not_loop(self):
        self.nav.execute=lambda request,cancellation_requested:SimpleNamespace(outcome=NavigationOutcome.SUCCEEDED)
        self.pose=('3F',1,.3,0.,0.);self.hold.tick()
        self.hold._thread.join(1)
        self.assertIsNone(self.hold._target)
        self.assertIn('outside hold tolerance',self.hold.detail)
        self.hold.tick();self.assertFalse(self.hold._thread.is_alive())

    def test_correction_timeout_has_no_unbounded_retries(self):
        self.pose=('3F',1,.3,0.,0.);self.hold.tick()
        self.assertTrue(self.nav.started.wait(1))
        self.now=6.;self.hold.tick();self.hold._thread.join(1)
        self.hold.tick();self.assertEqual(len(self.nav.calls),1)
        self.assertIsNone(self.hold._target)


if __name__=='__main__':unittest.main()
