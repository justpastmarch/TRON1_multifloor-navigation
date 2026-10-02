"""Run directly in a sourced ROS environment; uses an isolated temporary master.

Actual recorder/BAG verification prevents Python regex tests from masking a
mismatch with rosbag's whole-topic Boost matching. No robot topics are accessed.
"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest
import xmlrpc.client


class RecorderIntegrationTest(unittest.TestCase):
    def test_actual_recorder_excludes_transports_and_keeps_evidence(self):
        from mission_manager.console_recording import CAMERA_DUPLICATES
        with tempfile.TemporaryDirectory(prefix='tron-record-filter-') as directory:
            with socket.socket() as sock:
                sock.bind(('127.0.0.1', 0))
                port = sock.getsockname()[1]
            env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                       ROS_IP='127.0.0.1', ROS_HOME=directory)
            env.pop('ROS_HOSTNAME', None)
            os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'], ROS_IP='127.0.0.1')
            os.environ.pop('ROS_HOSTNAME', None)
            master = recorder = None
            import rospy
            import rosbag
            from std_msgs.msg import String
            try:
                master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                for _ in range(100):
                    try:
                        with xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']) as proxy:
                            ready = proxy.getPid('/test')[0] == 1
                        if ready:
                            break
                    except OSError:
                        pass
                    time.sleep(.1)
                else:
                    self.fail('isolated master did not start')
                rospy.init_node('record_filter_fixture', disable_signals=True)
                keep = ['/camera1/color/image_raw/compressed', '/livox/lidar', '/livox/imu',
                        '/tron/sensor_joy', '/stair_supervisor/control_debug', '/tag_detections']
                omit = ['/camera1/depth/image_rect_raw/compressedDepth',
                        '/camera1/depth/image_rect_raw/compressed',
                        '/camera1/infra1/image_rect_raw/compressed',
                        '/camera1/infra2/image_rect_raw/compressed',
                        '/camera1/aligned_depth_to_color/image_raw/compressedDepth',
                        '/camera1/color/image_raw', '/apriltag_camera/image_raw']
                pubs = [rospy.Publisher(topic, String, queue_size=1) for topic in keep+omit]
                path = str(Path(directory)/'test.bag')
                recorder = subprocess.Popen(['/opt/ros/noetic/lib/rosbag/record', '--all', '--exclude', CAMERA_DUPLICATES,
                                              '-O', path], env=env, stdout=subprocess.PIPE,
                                             stderr=subprocess.STDOUT)
                until = time.monotonic()+5
                while time.monotonic()<until:
                    for pub in pubs:
                        pub.publish(String(data='filter transport fixture'))
                    time.sleep(.05)
                recorder.send_signal(signal.SIGINT)
                log, _ = recorder.communicate(timeout=10)
                self.assertEqual(recorder.returncode, 0, log.decode())
                self.assertTrue(Path(path).exists(), log.decode())
                with rosbag.Bag(path) as bag:
                    topics = bag.get_type_and_topic_info().topics
                    for topic in keep:
                        self.assertIn(topic, topics)
                        self.assertGreater(topics[topic].message_count, 0)
                    for topic in omit:
                        self.assertNotIn(topic, topics)
            finally:
                rospy.signal_shutdown('fixture complete')
                for process in (recorder, master):
                    if process is not None and process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()


if __name__ == '__main__':
    unittest.main()
