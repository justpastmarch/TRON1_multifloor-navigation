import unittest,json
from stair_supervisor.robot_transport import RobotTransport,TransportFault
from stair_supervisor.robot_config import RobotTransportConfig,RobotConnectionConfig,CommandStreamConfig
from stair_supervisor.configuration import WebSocketCalibration
from test_websocket_tx_ros import FakeWebSocket,FakeFactory,FakeClock

class WalkNoAckTest(unittest.TestCase):
    def make(self):
        socket=FakeWebSocket()
        transport=RobotTransport(RobotTransportConfig(RobotConnectionConfig('WF_QA_ZERO','ws://fake.invalid:5000',receive_timeout=.01,request_timeout=.1),CommandStreamConfig(40,.25)),WebSocketCalibration(.5,2),FakeFactory(socket),FakeClock())
        transport.start()
        socket.sent_payloads.clear()
        def no_receive():raise AssertionError('WALK must not wait for a robot response')
        socket.recv=no_receive
        return transport,socket
    def test_missing_response_still_sends_navigation(self):
        t,s=self.make()
        s.send=lambda payload:s.sent_payloads.append(payload)
        t.send_walk_mode()
        t.update_twist(.1,0)
        t.send_current()
        frames=[json.loads(x) for x in s.sent_payloads]
        self.assertEqual(frames[0]['title'],'request_walk_mode')
        self.assertEqual(sum(x['title']=='request_walk_mode' for x in frames),1)
        self.assertTrue(any(x['title']=='request_twist' and x['data']['x']>0 for x in frames))
        self.assertEqual(t.pending_request_count,0)
    def test_send_failure_is_not_a_missing_ack(self):
        t,s=self.make();s.fail_sends=True
        with self.assertRaises(TransportFault):t.send_walk_mode()
        self.assertEqual(t.pending_request_count,0)
