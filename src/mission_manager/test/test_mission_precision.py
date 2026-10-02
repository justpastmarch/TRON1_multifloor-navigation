"""Idle arrival hold ownership, including concurrent handoff failures."""
import unittest
from mission_manager.configuration import Location
from test_navigation_hold import HoldTest


class HoldEpochTest(HoldTest):
    def test_refresh_failure_during_release_cannot_report_success(self):
        location=Location('arrival','3F','LANDING',0.,0.,0.)
        def takeover():
            self.hold.fail_localization(self.hold._epoch,'failure during release')
        with self.assertRaisesRegex(RuntimeError,'ownership changed'):
            self.hold.arm(location,1,takeover)
        self.assertFalse(self.hold.armed)

    def test_failed_release_never_arms_navigation_hold(self):
        def takeover():raise RuntimeError('no release acknowledgement')
        with self.assertRaisesRegex(RuntimeError,'no release'):
            self.hold.arm(Location('arrival','3F','LANDING',0.,0.,0.),1,takeover)
        self.assertFalse(self.hold.armed)
        self.hold.tick();self.assertEqual(self.nav.calls,[])

    def test_late_localization_failure_cannot_cancel_new_hold(self):
        epoch=self.hold._epoch
        self.hold.arm(Location('new','4F','LANDING',0.,0.,0.),2)
        self.hold.fail_localization(epoch,'late failed refresh')
        self.assertTrue(self.hold.armed)
        self.assertEqual(self.hold._target.id,'new')
    def test_current_localization_failure_cancels_only_owned_correction(self):
        self.pose=('3F',1,.3,0.,0.);self.hold.tick()
        self.assertTrue(self.nav.started.wait(1))
        self.hold.fail_localization(self.hold._epoch,'missing scan')
        self.assertTrue(self.nav.cancelled.wait(1));self.assertFalse(self.hold.armed)

if __name__=='__main__':unittest.main()
