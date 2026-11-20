import json, math, struct, tempfile, unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import rospy, rosbag
from genpy.dynamic import generate_dynamic
from sensor_msgs.msg import Imu

from mission_manager import mapping_cloud as mc

POINT_TEXT = 'uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n'
CUSTOM_TEXT = ('std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nCustomPoint[] points\n'
               '================================================================================\n'
               'MSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n'
               '================================================================================\n'
               'MSG: livox_ros_driver2/CustomPoint\n' + POINT_TEXT)
TYPES = generate_dynamic('livox_ros_driver2/CustomMsg', CUSTOM_TEXT)
CustomMsg, CustomPoint = TYPES['livox_ros_driver2/CustomMsg'], TYPES['livox_ros_driver2/CustomPoint']
IDENTITY = np.eye(4).tolist()


def scan(t, points, frame='mid360_link'):
    msg = CustomMsg()
    msg.header.stamp = rospy.Time.from_sec(t); msg.header.frame_id = frame
    msg.timebase = int(round(t * 1e9)); msg.point_num = len(points); msg.rsvd = [0, 0, 0]
    msg.points = [CustomPoint(offset_time=int(o * 1e9), x=x, y=y, z=z, reflectivity=50, tag=0, line=0) for o, x, y, z in points]
    return msg


def imu(t, acc=(0., 0., 1.)):
    msg = Imu(); msg.header.stamp = rospy.Time.from_sec(t)
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
    return msg


class MappingCloudTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup); self.root = Path(self.temp.name)

    def write_bag(self, scans, acc=(0., 0., 1.)):
        path = self.root / 'raw.bag'
        with rosbag.Bag(str(path), 'w') as bag:
            for k in range(0, 300):
                t = 100 + k * 0.005; bag.write('/livox/imu', imu(t, acc), rospy.Time.from_sec(t))
            for t, points in scans:
                bag.write('/livox/lidar', scan(t, points), rospy.Time.from_sec(t))
        return path

    def test_parse_matches_serialized_custom_message(self):
        import io
        buffer = io.BytesIO(); scan(5.25, [(0.01, 1., 2., 3.), (0.02, -1., 0.5, 0.)]).serialize(buffer)
        timebase, points = mc.parse_livox(buffer.getvalue())
        self.assertAlmostEqual(timebase, 5.25, places=6)
        self.assertEqual(points['x'].tolist(), [1., -1.]); self.assertEqual(points['offset'].tolist(), [10000000, 20000000])

    def test_points_follow_2d_trajectory_and_noise_is_dropped(self):
        # A wall 2 m ahead of the LiDAR, seen twice per voxel; robot at (1, 0) facing +y.
        wall = [(0., 2.0, y, z) for y in np.arange(-0.5, 0.5, 0.1) for z in (0.0, 0.3)]
        bag = self.write_bag([(100.2, wall), (100.4, wall), (100.6, [(0., 3., 3., 3.)])])
        trajectory = [[100.0, 1.0, 0.0, math.pi / 2], [101.0, 1.0, 0.0, math.pi / 2]]
        params = dict(base_from_lidar=IDENTITY, lidar_from_imu=np.eye(3).tolist())
        result = mc.build(bag, trajectory, params, self.root / 'out')
        self.assertEqual(result['scans_used'], 3)
        # The single-hit point is filtered (min_hits=2); the wall lands at y=2 in front of x=1.
        self.assertEqual(result['points'], 20)
        low, high = np.array(result['bounds'])
        self.assertAlmostEqual(low[1], 2.0, delta=0.06); self.assertAlmostEqual(high[1], 2.0, delta=0.06)
        self.assertAlmostEqual(low[0], 0.6, delta=0.06); self.assertAlmostEqual(high[0], 1.5, delta=0.06)
        self.assertTrue((self.root / 'out' / 'cloud.ply').read_bytes().startswith(b'ply\n'))
        count = struct.unpack('<I', (self.root / 'out' / 'cloud.bin').read_bytes()[:4])[0]
        self.assertEqual(count, 20)

    def test_imu_tilt_levels_the_cloud(self):
        # LiDAR pitched 10 deg nose-down: gravity seen tilted; a level point 2 m ahead must stay at z~0 after leveling.
        pitch = math.radians(10)
        acc = (math.sin(pitch), 0., math.cos(pitch))  # 'up' measured in the tilted sensor frame
        ahead = [(0., 2 * math.cos(pitch), 0., -2 * math.sin(pitch))] * 2  # world point (2,0,0) expressed in the sensor
        bag = self.write_bag([(100.5, ahead), (100.6, ahead)], acc)
        params = dict(base_from_lidar=IDENTITY, lidar_from_imu=np.eye(3).tolist())
        result = mc.build(bag, [[100.0, 0, 0, 0], [101.0, 0, 0, 0]], params, self.root / 'out')
        low, high = np.array(result['bounds'])
        self.assertAlmostEqual(high[2], 0.0, delta=0.06); self.assertAlmostEqual(high[0], 2.0, delta=0.06)

    def test_no_overlap_with_trajectory_is_reported(self):
        bag = self.write_bag([(100.2, [(0., 2., 0., 0.)] * 2)])
        params = dict(base_from_lidar=IDENTITY, lidar_from_imu=np.eye(3).tolist())
        with self.assertRaisesRegex(ValueError, '겹치는'):
            mc.build(bag, [[200., 0, 0, 0], [201., 0, 0, 0]], params, self.root / 'out')

    def test_transport_round_trip_within_one_centimetre(self):
        xyz = np.random.RandomState(1).uniform(-20, 30, (500, 3)); intensity = np.arange(500) % 256
        out = self.root / 'remote'; out.mkdir()
        mc.write_transport(out / 'cloud.npz', xyz, intensity); mc.finish_transport(out)
        data = (out / 'cloud.bin').read_bytes(); n = struct.unpack('<I', data[:4])[0]
        back = np.frombuffer(data, '<f4', n * 3, 4).reshape(-1, 3)
        self.assertEqual(n, 500); self.assertLess(np.abs(back - xyz).max(), 0.006)
        self.assertFalse((out / 'cloud.npz').exists()); self.assertTrue((out / 'cloud.ply').is_file())

    def test_gravity_filter_tracks_accelerometer(self):
        rows = [(k * 0.005, 0, 0, 0, 0.0, 0.5, 0.866) for k in range(400)]
        _, up = mc.gravity_track(rows, np.eye(3), 0.01)
        self.assertLess(np.linalg.norm(up[-1] - np.array([0, 0.5, 0.866])), 0.01)


class MappingConsoleRawTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup); self.root = Path(self.temp.name)

    def console(self, tool):
        from mission_manager import mapping_console
        with patch.object(mapping_console.rospy, 'Subscriber'), patch.object(mapping_console.tf2_ros, 'TransformListener'), patch.object(mapping_console.tf2_ros, 'Buffer'):
            return mapping_console.MappingConsole(str(self.root), env=dict(MAPPING_RAW_RECORD_TOOL=tool, MAPPING_CLOUD_TOOL='/x/cloud'))

    def test_raw_start_failure_keeps_mapping_and_is_recorded(self):
        console = self.console('/bin/false')
        identifier, directory = console.library.create('site')
        console._start_raw(identifier, directory)
        self.assertIsNone(console.raw_name)
        self.assertEqual(json.loads((directory / 'raw.json').read_text())['state'], 'failed')

    def test_sealed_raw_selects_remote_cloud_build(self):
        console = self.console('/bin/true')
        identifier, directory = console.library.create('site')
        (directory / 'input.bag').write_bytes(b'')
        (directory / 'raw.json').write_text(json.dumps(dict(name='map_x', state='sealed')))
        with patch('mission_manager.mapping_compare.run_comparison', return_value='ok') as run, \
             patch('mission_manager.mapping_console.cloud_params', return_value=dict(base_from_lidar=IDENTITY, lidar_from_imu=IDENTITY)), \
             patch('mission_manager.mapping_console.cloud_runner', return_value='runner') as runner:
            self.assertEqual(console.compare(dict(session_id=identifier, _job='j')), 'ok')
        self.assertEqual(runner.call_args[0][0], ['/x/cloud', 'build', 'map_x'])
        self.assertTrue(runner.call_args[0][1]['transport'])
        self.assertEqual(run.call_args[0][4], 'runner')

    def test_mini_pc_2d_input_is_preferred_and_failure_falls_back(self):
        console = self.console('/bin/true')
        identifier, directory = console.library.create('site')
        (directory / 'input.bag').write_bytes(b'')
        (directory / 'raw.json').write_text(json.dumps(dict(name='map_x', state='sealed', with_2d=True)))
        def fetched(command, **kwargs):
            Path(command[3]).write_bytes(b'bag')
            return type('R', (), dict(returncode=0))()
        with patch('mission_manager.mapping_compare.run_comparison', return_value='ok') as run, \
             patch('mission_manager.mapping_console.cloud_params', return_value={}), \
             patch('mission_manager.mapping_console.cloud_runner', return_value='runner'), \
             patch('mission_manager.mapping_console.subprocess.run', side_effect=fetched):
            console.compare(dict(session_id=identifier))
        self.assertEqual(run.call_args[0][0], directory / 'input_minipc.bag')
        (directory / 'input_minipc.bag').unlink()
        with patch('mission_manager.mapping_compare.run_comparison', return_value='ok') as run, \
             patch('mission_manager.mapping_console.cloud_params', return_value={}), \
             patch('mission_manager.mapping_console.cloud_runner', return_value='runner'), \
             patch('mission_manager.mapping_console.subprocess.run', return_value=type('R', (), dict(returncode=1))()):
            console.compare(dict(session_id=identifier))
        self.assertEqual(run.call_args[0][0], directory / 'input.bag')

    def test_local_raw_mode_records_and_builds_in_the_session(self):
        from mission_manager import mapping_console
        with patch.object(mapping_console.rospy, 'Subscriber'), patch.object(mapping_console.tf2_ros, 'TransformListener'), patch.object(mapping_console.tf2_ros, 'Buffer'):
            console = mapping_console.MappingConsole(str(self.root), env=dict(MAPPING_LOCAL_RAW='1'))
        identifier, directory = console.library.create('site')
        with patch.object(mapping_console, 'Process') as process:
            console._start_raw(identifier, directory)
        self.assertIn('/livox/lidar', process.call_args[0][0]); self.assertEqual(console.raw_name, 'raw.bag')
        console.session = dict(id=identifier)
        (directory / 'raw.bag').write_bytes(b'')
        console._seal_raw()
        raw = json.loads((directory / 'raw.json').read_text())
        self.assertEqual((raw['state'], raw['host']), ('sealed', 'local'))
        (directory / 'input.bag').write_bytes(b'')
        with patch('mission_manager.mapping_compare.run_comparison', return_value='ok') as run, \
             patch('mission_manager.mapping_console.cloud_params', return_value={}), \
             patch('mission_manager.mapping_console.cloud_runner', return_value='runner') as runner:
            console.compare(dict(session_id=identifier))
        self.assertEqual(runner.call_args[0][0][-1], str(directory / 'raw.bag'))
        self.assertEqual(run.call_args[0][0], directory / 'input.bag')

    def test_without_raw_no_cloud_step(self):
        console = self.console('')
        identifier, directory = console.library.create('site')
        (directory / 'input.bag').write_bytes(b'')
        with patch('mission_manager.mapping_compare.run_comparison', return_value='ok') as run:
            console.compare(dict(session_id=identifier))
        self.assertIsNone(run.call_args[0][4])


if __name__ == '__main__':
    unittest.main()
