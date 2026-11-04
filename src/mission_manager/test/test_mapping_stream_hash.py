import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import rosbag
import rospy
from sensor_msgs.msg import LaserScan
from mission_manager.mapping_compare import prepare_input


class MappingStreamHashTest(unittest.TestCase):
    def test_bag_hash_is_correct_without_allocating_the_entire_file(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'source.bag'
            output = Path(directory)/'filtered.bag'
            with rosbag.Bag(str(source), 'w') as bag:
                for stamp in [10, 11]:
                    scan = LaserScan()
                    scan.header.stamp = rospy.Time(stamp)
                    bag.write('/scan', scan, rospy.Time(stamp))
            with patch.object(Path, 'read_bytes', side_effect=AssertionError('whole-file read')):
                result = prepare_input(source, output)
            with output.open('rb') as stream:
                expected = hashlib.sha256(stream.read()).hexdigest()
            self.assertEqual(result['sha256'], expected)
            self.assertEqual(result['scan_count'], 2)


if __name__ == '__main__':
    unittest.main()
