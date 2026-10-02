"""Idle AMCL refresh uses the existing bounded service, never resets pose."""
import math,threading,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from multifloor_manager.ros_startup_localization import StartupLocalization,FloorState,SupervisorState

class IdleLocalizationTest(unittest.TestCase):
    def fixture(self):
        s=StartupLocalization.__new__(StartupLocalization)
        s.lock=threading.RLock();s.request=None;s.runtime=SimpleNamespace(state=FloorState.READY)
        s.supervisor=SimpleNamespace(state=SupervisorState.NAV)
        s.scan=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda:1)))
        s.idle_refresh_sec=1.;s._idle_last_attempt=-math.inf;s._idle_scan_stamp=None
        s.busy=Mock(return_value=False);s.fresh=Mock(return_value=True)
        s.node=SimpleNamespace(services=SimpleNamespace(call=Mock()),nomotion_name='/request_nomotion_update',nomotion_update=Mock())
        return s
    def test_refresh_is_bounded_and_requires_new_scan(self):
        s=self.fixture()
        with patch('multifloor_manager.ros_startup_localization.time.monotonic',return_value=10):
            s.refresh_idle();s.refresh_idle()
        self.assertEqual(s.node.services.call.call_count,1)
        s.scan.header.stamp.to_nsec=lambda:2
        with patch('multifloor_manager.ros_startup_localization.time.monotonic',return_value=10.5):s.refresh_idle()
        self.assertEqual(s.node.services.call.call_count,1)
        with patch('multifloor_manager.ros_startup_localization.time.monotonic',return_value=11):s.refresh_idle()
        self.assertEqual(s.node.services.call.call_count,2)
    def test_refresh_skips_unlocalized_stair_action_and_stale_scan(self):
        for reason in ('unknown','stairs','busy','stale','request'):
            s=self.fixture()
            if reason=='unknown':s.runtime.state=FloorState.UNKNOWN
            if reason=='stairs':s.supervisor.state=SupervisorState.STAIR
            if reason=='busy':s.busy.return_value=True
            if reason=='stale':s.fresh.return_value=False
            if reason=='request':s.request=('auto',None)
            s.refresh_idle();s.node.services.call.assert_not_called()
    def test_failure_keeps_ready_and_retries_later(self):
        s=self.fixture();s.node.services.call.side_effect=RuntimeError('timeout')
        with patch('multifloor_manager.ros_startup_localization.rospy.logwarn_throttle'):
            s.refresh_idle()
        self.assertEqual(s.runtime.state,FloorState.READY)
        s.node.services.call.side_effect=None;s._idle_last_attempt-=2;s.scan.header.stamp.to_nsec=lambda:2
        s.refresh_idle();self.assertEqual(s.node.services.call.call_count,2)

if __name__=='__main__':unittest.main()
