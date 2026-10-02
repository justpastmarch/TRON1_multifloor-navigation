import unittest
from .test_robot_transport_fakes import FakeClock,FakeWebSocket,make_transport,successful_mode_handler
from stair_supervisor.web_manual import WebManual
from stair_supervisor.robot_conversion import NormalizedTwist

class ManualTest(unittest.TestCase):
    def test_expiry_release_and_reordered_commands(self):
        m=WebManual();lease=m.begin(10.)
        m.update(lease,1,.7,-.2,10.1,.3)
        self.assertEqual(m.selected(10.2),NormalizedTwist(.7,0,-.2))
        with self.assertRaises(ValueError):m.update(lease,1,1,0,10.2,.3)
        m.end(lease)
        with self.assertRaises(ValueError):m.update(lease,2,1,0,10.2,.3)
        self.assertIsNone(m.selected(20))
        second=m.begin(20)
        with self.assertRaises(ValueError):m.end(lease)
        with self.assertRaises(ValueError):m.update(second,1,1,0,21,.3)
        m.release();self.assertIsNone(m.selected(21))

    def test_nonfinite_and_unbounded_values(self):
        for value in (float('nan'),float('inf'),2,True,'1'):
            m=WebManual();lease=m.begin(0)
            with self.assertRaises(ValueError):m.update(lease,0,value,0,.1,.3)

    def test_manual_priority_preserves_automatic_buffer_and_single_connection(self):
        c=FakeClock();s=FakeWebSocket(successful_mode_handler);t,f=make_transport(s,c);t.start()
        t.update_twist(.1,.2)
        lease=t.begin_web_manual()
        t.send_current()
        t.update_web_manual(lease,1,.8,-.2,.35)
        t.send_current()
        self.assertEqual([v['data'] for v in s.sent[-3:]], [dict(x=0.,y=0.,z=0.),dict(x=.8,y=0.,z=-.2),dict(x=.8,y=0.,z=-.2)])
        t.end_web_manual(lease)
        self.assertEqual(s.sent[-1]['data'],dict(x=0.,y=0.,z=0.))
        t.update_twist(.1,0);t.send_current();self.assertEqual(s.sent[-1]['data']['x'],.2)
        self.assertEqual(f.calls,1);t.close()

    def test_timeout_is_one_manual_zero_without_suppressing_automatic(self):
        c=FakeClock();s=FakeWebSocket(successful_mode_handler);t,_=make_transport(s,c);t.start()
        lease=t.begin_web_manual();t.update_web_manual(lease,1,1,0,.35)
        c.sleep(.36);before=len(s.sent);t.expire_web_manual()
        self.assertEqual(len(s.sent),before+1)
        self.assertEqual(s.sent[-1]['data'],dict(x=0.,y=0.,z=0.))
        t.expire_web_manual();self.assertEqual(len(s.sent),before+1)
        t.update_twist(.5,0);t.send_current();self.assertEqual(s.sent[-1]['data']['x'],1.)
        with self.assertRaises(ValueError):t.update_web_manual(lease,2,1,0,.35)
        t.close()

    def test_automatic_zeros_do_not_interrupt_held_manual_input(self):
        c=FakeClock();s=FakeWebSocket(successful_mode_handler);t,_=make_transport(s,c);t.start()
        lease=t.begin_web_manual()
        for seq in range(1,61):
            t.update_web_manual(lease,seq,1.,-.4,.35)
            t.update_twist(0.,0.);t.send_current();t.send_current()
            self.assertTrue(all(v['data']==dict(x=1.,y=0.,z=-.4) for v in s.sent[-3:]))
        t.end_web_manual(lease);t.send_current()
        self.assertEqual(s.sent[-1]['data'],dict(x=0.,y=0.,z=0.));t.close()

    def test_release_uses_latest_auto_value_and_expired_auto_is_zero(self):
        c=FakeClock();s=FakeWebSocket(successful_mode_handler);t,_=make_transport(s,c);t.start()
        lease=t.begin_web_manual();t.update_web_manual(lease,1,1,0,.35)
        t.update_twist(-.1,.4);t.send_current()
        self.assertEqual(s.sent[-1]['data']['x'],1.)
        t.end_web_manual(lease);t.send_current()
        self.assertEqual(s.sent[-1]['data'],dict(x=-.2,y=0.,z=.2))
        lease=t.begin_web_manual();t.update_web_manual(lease,1,1,0,.35)
        c.sleep(.5);t.expire_web_manual();t.send_current()
        self.assertEqual(s.sent[-1]['data'],dict(x=0.,y=0.,z=0.));t.close()

    def test_ros_timer_repeats_manual_without_cancelling_supervisor(self):
        from unittest.mock import Mock
        from stair_supervisor.ros_node import RosStairSupervisorNode
        c=FakeClock();s=FakeWebSocket(successful_mode_handler);t,_=make_transport(s,c);t.start()
        node=RosStairSupervisorNode.__new__(RosStairSupervisorNode)
        node._web_transport=t;node._supervisor=Mock()
        lease=t.begin_web_manual();t.update_web_manual(lease,1,1,0,.35)
        before=len(s.sent)
        for _ in range(3):node.stream_once()
        self.assertEqual(len(s.sent),before+3)
        self.assertTrue(all(v['data']['x']==1 for v in s.sent[-3:]))
        self.assertEqual(node._supervisor.mock_calls,[])
        t.end_web_manual(lease);node.stream_once()
        node._supervisor.stream_navigation.assert_called_once_with();t.close()
