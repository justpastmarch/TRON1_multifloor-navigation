#!/usr/bin/env python3
"""Record six real ROS message types through the non-node ScanRecorder library."""

from pathlib import Path
import subprocess
import tempfile
import threading
import unittest

from geometry_msgs.msg import PoseStamped, TransformStamped
import rosbag
import rospy
import rostest
from sensor_msgs.msg import Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

from mission_manager.configuration import ScanProfile
from mission_manager.scan_recorder import RecordingDisposition, RecordingIdentity, ScanRecorder


TOPIC_TYPES = {
    "/scan/points": "sensor_msgs/PointCloud2",
    "/camera/color/image_raw": "sensor_msgs/Image",
    "/camera/depth/image_raw": "sensor_msgs/Image",
    "/imu/data": "sensor_msgs/Imu",
    "/tf": "tf2_msgs/TFMessage",
    "/localization/pose": "geometry_msgs/PoseStamped",
}


class RealRosbagRecorderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node("scan_recorder_real_rosbag_test", anonymous=True)

    def test_six_required_topic_types_form_valid_managed_bag(self) -> None:
        # Given: live publishers for PointCloud2, RGB, depth, IMU, TF, and pose.
        publishers = {
            topic: rospy.Publisher(topic, message_type, queue_size=10)
            for topic, message_type in (
                ("/scan/points", PointCloud2),
                ("/camera/color/image_raw", Image),
                ("/camera/depth/image_raw", Image),
                ("/imu/data", Imu),
                ("/tf", TFMessage),
                ("/localization/pose", PoseStamped),
            )
        }
        stop = threading.Event()

        def publish_until_stopped() -> None:
            rate = rospy.Rate(20)
            while not stop.is_set() and not rospy.is_shutdown():
                stamp = rospy.Time.now()
                cloud = PointCloud2()
                cloud.header.stamp = stamp
                cloud.header.frame_id = "camera_link"
                rgb = Image(header=cloud.header, height=1, width=1, encoding="rgb8", step=3, data=b"\x01\x02\x03")
                depth = Image(header=cloud.header, height=1, width=1, encoding="16UC1", step=2, data=b"\x01\x00")
                imu = Imu(header=cloud.header)
                transform = TransformStamped(header=cloud.header, child_frame_id="base_link")
                pose = PoseStamped(header=cloud.header)
                publishers["/scan/points"].publish(cloud)
                publishers["/camera/color/image_raw"].publish(rgb)
                publishers["/camera/depth/image_raw"].publish(depth)
                publishers["/imu/data"].publish(imu)
                publishers["/tf"].publish(TFMessage((transform,)))
                publishers["/localization/pose"].publish(pose)
                rate.sleep()

        publisher_thread = threading.Thread(target=publish_until_stopped, daemon=True)
        publisher_thread.start()
        rospy.sleep(0.2)

        # When: the real rosbag CLI is owned for a bounded recording.
        with tempfile.TemporaryDirectory(prefix="tron1-real-scan-") as output_base:
            profile = ScanProfile("six_type", tuple(TOPIC_TYPES), 1.0, Path("managed"), 0.01)
            identity = RecordingIdentity("mission-real", "roof", 1700000000000000000)
            try:
                result = ScanRecorder(Path(output_base)).record(profile, identity)
            finally:
                stop.set()
                publisher_thread.join(timeout=3.0)

            # Then: CLI metadata and bag connection headers prove every required type.
            info = subprocess.run(
                ("rosbag", "info", "--yaml", str(result.artifact_path)),
                check=True,
                text=True,
                stdout=subprocess.PIPE,
            )
            self.assertIn("duration:", info.stdout)
            with rosbag.Bag(str(result.artifact_path), "r") as bag:
                actual_types = {
                    topic: bag.get_type_and_topic_info().topics[topic].msg_type
                    for topic in TOPIC_TYPES
                }
            self.assertEqual(result.disposition, RecordingDisposition.COMPLETED)
            self.assertEqual(actual_types, TOPIC_TYPES)
            self.assertEqual(result.artifact_path.name, "mission-real_roof_1700000000000000000.bag")


if __name__ == "__main__":
    rostest.rosrun("mission_manager", "scan_recorder_real_rosbag", RealRosbagRecorderTest)
