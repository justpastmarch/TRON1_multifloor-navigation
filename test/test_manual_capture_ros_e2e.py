from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

import cv2
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def _free_port() -> int:
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return int(listener.getsockname()[1])


def _wait_for_port(port: int) -> None:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                return
        except OSError:
            time.sleep(0.02)
    pytest.fail("private ROS master did not start")


@pytest.mark.skipif(not Path("/opt/ros/noetic/bin/rosmaster").exists(), reason="ROS Noetic unavailable")
def test_public_recorder_finalizes_synthetic_bag_without_livox_package(tmp_path: Path) -> None:
    # Given: a private ROS graph with synthetic sensors and raw joystick input.
    port = _free_port()
    environment = dict(os.environ)
    environment.pop("ROS_IP", None)
    environment.pop("ROS_HOSTNAME", None)
    environment.update({
        "ROS_MASTER_URI": f"http://127.0.0.1:{port}",
        "ROS_HOME": str(tmp_path / "ros-home"),
        "ROS_LOG_DIR": str(tmp_path / "ros-logs"),
        "CAPTURE_CAMERA_TOPIC": "/camera/compressed",
        "CAMERA_INFO_TOPIC": "/camera/info",
        "RAW_LIVOX_TOPIC": "/livox/raw",
        "SCAN_TOPIC": "/scan",
        "WHEEL_ODOM_TOPIC": "/wheel_odom",
        "IMU_TOPIC": "/imu",
        "SENSOR_JOY_RECEIVER_AUTOSTART": "0",
        "MANUAL_CAPTURE_OUTPUT_BASE": str(tmp_path),
    })
    master = subprocess.Popen(
        ("/opt/ros/noetic/bin/rosmaster", "--core", "-p", str(port), "-w", "3"),
        env=environment,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    recorder: subprocess.Popen[str] | None = None
    publish_stop = threading.Event()
    publish_thread: threading.Thread | None = None
    previous_master = os.environ.get("ROS_MASTER_URI")
    try:
        _wait_for_port(port)
        os.environ["ROS_MASTER_URI"] = environment["ROS_MASTER_URI"]
        os.environ.pop("ROS_IP", None)
        os.environ.pop("ROS_HOSTNAME", None)
        import rospy
        from genpy.dynamic import generate_dynamic
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, Joy, LaserScan
        from tf2_msgs.msg import TFMessage

        rospy.init_node("manual_capture_public_e2e", anonymous=True, disable_signals=True)
        custom_definition = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
uint8 point_type
livox_ros_driver2/CustomPoint[] points
================================================================================
MSG: livox_ros_driver2/CustomPoint
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
"""
        custom_type = generate_dynamic(
            "livox_ros_driver2/CustomMsg", custom_definition
        )["livox_ros_driver2/CustomMsg"]
        camera_pub = rospy.Publisher("/camera/compressed", CompressedImage, queue_size=2)
        info_pub = rospy.Publisher("/camera/info", CameraInfo, queue_size=2)
        livox_pub = rospy.Publisher("/livox/raw", custom_type, queue_size=2)
        scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=2)
        odom_pub = rospy.Publisher("/wheel_odom", Odometry, queue_size=2)
        tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=2)
        static_pub = rospy.Publisher("/tf_static", TFMessage, queue_size=1, latch=True)
        imu_pub = rospy.Publisher("/imu", Imu, queue_size=2)
        joy_pub = rospy.Publisher("/tron/sensor_joy", Joy, queue_size=2)

        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        transform.header.stamp = rospy.Time.now()
        static_pub.publish(TFMessage([transform]))
        encoded_ok, encoded = cv2.imencode(
            ".jpg", np.zeros((8, 8, 3), dtype=np.uint8)
        )
        assert encoded_ok
        image = CompressedImage(format="jpeg", data=encoded.tobytes())
        camera_info = CameraInfo()
        raw_livox = custom_type()
        scan = LaserScan()
        odometry = Odometry()
        imu = Imu()
        joystick = Joy(axes=[0.25, -0.5, 0.75], buttons=[0, 1, 0, 1])

        def publish_sensors() -> None:
            rate = rospy.Rate(30)
            while not publish_stop.is_set():
                stamp = rospy.Time.now()
                for message in (image, camera_info, raw_livox, scan, odometry, imu, joystick):
                    message.header.stamp = stamp
                transform.header.stamp = stamp
                camera_pub.publish(image)
                info_pub.publish(camera_info)
                livox_pub.publish(raw_livox)
                scan_pub.publish(scan)
                odom_pub.publish(odometry)
                tf_pub.publish(TFMessage([transform]))
                imu_pub.publish(imu)
                joy_pub.publish(joystick)
                rate.sleep()

        publish_thread = threading.Thread(target=publish_sensors, daemon=True)
        publish_thread.start()
        # When: the public one-command recorder preflights and starts its own rosbag.
        recorder = subprocess.Popen(
            (str(ROOT / "run.sh"), "--record-manual"),
            cwd=ROOT,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        deadline = time.monotonic() + 45.0
        active: Path | None = None
        while time.monotonic() < deadline and recorder.poll() is None:
            active = next(tmp_path.glob("*.bag.active"), None)
            if active is not None:
                break
            time.sleep(0.05)
        assert active is not None, recorder.communicate(timeout=5.0)
        publishers = (
            camera_pub, info_pub, livox_pub, scan_pub, odom_pub,
            tf_pub, static_pub, imu_pub, joy_pub,
        )
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if all(publisher.get_num_connections() > 0 for publisher in publishers):
                break
            time.sleep(0.02)
        assert all(publisher.get_num_connections() > 0 for publisher in publishers)
        callback_rate = rospy.Rate(50)
        for _ in range(20):
            joystick.header.stamp = rospy.Time.now()
            joy_pub.publish(joystick)
            callback_rate.sleep()

        recorder.send_signal(signal.SIGINT)
        stdout, stderr = recorder.communicate(timeout=15.0)

        # Then: graceful public shutdown produces a complete report and valid static TF evidence.
        assert recorder.returncode == 0, stdout + stderr
        assert "[READY SENSOR] yes" in stdout
        assert "[READY JOYSTICK] yes" in stdout
        assert "[COMPLETE]" in stdout
        reports = list(tmp_path.glob("*.bag.capture.json"))
        assert len(reports) == 1
        report = json.loads(reports[0].read_text(encoding="utf-8"))
        assert report["complete"] is True
        assert report["topic_counts"]["/livox/raw"] > 0
        assert report["topic_counts"]["/tf_static"] == 1
        assert report["topic_counts"]["/tron/sensor_joy"] > 0
        assert Path(report["artifact"]).is_file()
        assert not list(tmp_path.glob("*.INCOMPLETE.json"))
    finally:
        publish_stop.set()
        if publish_thread is not None:
            publish_thread.join(timeout=2.0)
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            recorder.wait(timeout=15.0)
        os.killpg(master.pid, signal.SIGINT)
        master.wait(timeout=10.0)
        if previous_master is None:
            os.environ.pop("ROS_MASTER_URI", None)
        else:
            os.environ["ROS_MASTER_URI"] = previous_master
