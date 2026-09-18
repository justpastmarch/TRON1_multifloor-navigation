#!/usr/bin/env python3
"""Convert recorded raw sensors into RViz-only displays without sensor fusion."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Final, Optional, Sequence, Type

from cv_bridge import CvBridge, CvBridgeError
from genpy import Message
from genpy.dynamic import generate_dynamic
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
import rosbag
import rospy
import tf2_ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from raw_sensor_display import (
    ArrowStyle,
    BagMessageError,
    LivoxMessageLike,
    LivoxPoint as LivoxPoint,
    Vector3Value,
    sample_livox_points,
    vector_arrow,
    wheel_path_markers,
)
from raw_sensor_status import render_status_image, status_lines as status_lines
from raw_sensor_transforms import (
    load_lidar_extrinsic,
    wheel_odometry_transform as wheel_odometry_transform,
)


LIVOX_TOPIC: Final = "/livox/lidar"
LIVOX_TYPE: Final = "livox_ros_driver2/CustomMsg"
MAX_POINTS_PER_FRAME: Final = 12_000
MAX_PATH_POSES: Final = 5_000


@dataclass(frozen=True)
class VisualizerSettings:
    bag_path: Path
    lidar_frame: str
    max_points_per_frame: int

def load_livox_message_type(bag_path: Path) -> Type[Message]:
    """Generate the exact recorded Livox message class from the bag connection."""
    with rosbag.Bag(str(bag_path), "r") as bag:
        connections = tuple(bag._get_connections(topics=[LIVOX_TOPIC]))
    if len(connections) != 1:
        raise BagMessageError(f"expected one {LIVOX_TOPIC} connection, got {len(connections)}")
    connection = connections[0]
    recorded_type = connection.header.get("type", b"").decode("utf-8")
    definition = connection.header.get("message_definition", b"").decode("utf-8")
    if recorded_type != LIVOX_TYPE or not definition:
        raise BagMessageError(f"bag does not contain a usable {LIVOX_TYPE} definition")
    generated = generate_dynamic(LIVOX_TYPE, definition)
    return generated[LIVOX_TYPE]


def create_livox_cloud(
    message: LivoxMessageLike,
    settings: VisualizerSettings,
) -> PointCloud2:
    """Convert one custom Livox frame into a standard RViz PointCloud2."""
    fields = (
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
    )
    header = Header(stamp=message.header.stamp, frame_id=settings.lidar_frame)
    return point_cloud2.create_cloud(
        header,
        fields,
        sample_livox_points(message.points, settings.max_points_per_frame),
    )


class BagSensorVisualizer:
    """Publish mutable replay state for raw sensor inspection in RViz."""

    def __init__(
        self,
        settings: VisualizerSettings,
        livox_message_type: Type[Message],
        lidar_extrinsic: TransformStamped,
    ) -> None:
        self._settings = settings
        self._camera_bridge = CvBridge()
        self._path = RosPath(header=Header(frame_id="odom"))
        self._path_points: list[Point] = []
        self._latest_imu: Optional[tuple[str, Vector3Value, Vector3Value]] = None
        self._latest_odometry: Optional[Point] = None
        self._wheel_tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._static_tf_broadcaster.sendTransform(lidar_extrinsic)
        self._odom_count = 0
        self._cloud_publisher = rospy.Publisher("/replay/livox/points", PointCloud2, queue_size=2)
        self._path_publisher = rospy.Publisher("/replay/wheel_odom/path", RosPath, queue_size=1, latch=True)
        self._path_marker_publisher = rospy.Publisher(
            "/replay/wheel_odom/path_marker", Marker, queue_size=1, latch=True
        )
        self._path_landmark_publisher = rospy.Publisher(
            "/replay/wheel_odom/path_landmarks", Marker, queue_size=1, latch=True
        )
        self._acceleration_publisher = rospy.Publisher("/replay/imu/acceleration", Marker, queue_size=2)
        self._angular_velocity_publisher = rospy.Publisher("/replay/imu/angular_velocity", Marker, queue_size=2)
        self._camera_publisher = rospy.Publisher("/replay/camera/image_raw", Image, queue_size=2)
        self._status_publisher = rospy.Publisher("/replay/status/image", Image, queue_size=1, latch=True)
        self._lidar_subscriber = rospy.Subscriber(LIVOX_TOPIC, livox_message_type, self._lidar_callback, queue_size=2)
        self._odom_subscriber = rospy.Subscriber("/tron/wheel_odom_raw", Odometry, self._odom_callback, queue_size=20)
        self._imu_subscriber = rospy.Subscriber("/livox/imu", Imu, self._imu_callback, queue_size=50)
        self._camera_subscriber = rospy.Subscriber(
            "/camera1/color/image_raw/compressed",
            CompressedImage,
            self._camera_callback,
            queue_size=2,
        )

    def _lidar_callback(self, message: LivoxMessageLike) -> None:
        self._cloud_publisher.publish(create_livox_cloud(message, self._settings))

    def _odom_callback(self, message: Odometry) -> None:
        self._wheel_tf_broadcaster.sendTransform(wheel_odometry_transform(message))
        self._odom_count += 1
        if self._odom_count % 5 != 0:
            return
        pose = PoseStamped(header=message.header, pose=message.pose.pose)
        self._path.header.stamp = message.header.stamp
        self._path.poses.append(pose)
        position = message.pose.pose.position
        self._latest_odometry = Point(position.x, position.y, position.z)
        self._path_points.append(Point(position.x, position.y, position.z + 0.35))
        if len(self._path.poses) > MAX_PATH_POSES:
            del self._path.poses[0]
            del self._path_points[0]
        self._path_publisher.publish(self._path)
        path_marker, landmarks = wheel_path_markers(self._path_points)
        path_marker.header.stamp = message.header.stamp
        landmarks.header.stamp = message.header.stamp
        self._path_marker_publisher.publish(path_marker)
        self._path_landmark_publisher.publish(landmarks)

    def _imu_callback(self, message: Imu) -> None:
        source_frame = message.header.frame_id or "unframed"
        self._latest_imu = (
            source_frame,
            Vector3Value(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
            Vector3Value(
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
        )
        acceleration_style = ArrowStyle(
            "base_Link",
            source_frame,
            "raw_linear_acceleration",
            0,
            1.2,
            Vector3Value(-0.72, 0.70, 1.4),
        )
        acceleration = vector_arrow(
            acceleration_style,
            message.linear_acceleration,
        )
        acceleration.header.stamp = message.header.stamp
        acceleration.color.b = 1.0
        acceleration.color.a = 1.0
        angular_velocity_style = ArrowStyle(
            "base_Link",
            source_frame,
            "raw_angular_velocity",
            1,
            20.0,
            Vector3Value(0.72, -0.70, 1.4),
        )
        angular_velocity = vector_arrow(
            angular_velocity_style,
            message.angular_velocity,
        )
        angular_velocity.header.stamp = message.header.stamp
        angular_velocity.color.r = 1.0
        angular_velocity.color.b = 1.0
        angular_velocity.color.a = 1.0
        self._acceleration_publisher.publish(acceleration)
        self._angular_velocity_publisher.publish(angular_velocity)

    def _camera_callback(self, message: CompressedImage) -> None:
        try:
            image = self._camera_bridge.compressed_imgmsg_to_cv2(message, "bgr8")
            output = self._camera_bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as error:
            rospy.logwarn("recorded camera frame rejected: %s", error)
            return
        output.header = message.header
        self._camera_publisher.publish(output)
        if self._latest_imu is not None and self._latest_odometry is not None:
            source_frame, acceleration, angular_velocity = self._latest_imu
            status = render_status_image(
                source_frame,
                acceleration,
                angular_velocity,
                self._latest_odometry,
            )
            status_output = self._camera_bridge.cv2_to_imgmsg(status, "bgr8")
            status_output.header = message.header
            self._status_publisher.publish(status_output)


def main(arguments: Sequence[str] = sys.argv[1:]) -> int:
    """Start the conversion node using the recorded message definition."""
    if len(arguments) not in (1, 2):
        print("Usage: bag_sensor_visualizer.py BAG [LIDAR_FRAME]", file=sys.stderr)
        return 2
    bag_path = Path(arguments[0]).expanduser().resolve()
    if not bag_path.is_file():
        print(f"BAG is not readable: {bag_path}", file=sys.stderr)
        return 2
    settings = VisualizerSettings(
        bag_path,
        arguments[1] if len(arguments) == 2 else "mid360_link",
        MAX_POINTS_PER_FRAME,
    )
    livox_message_type = load_livox_message_type(settings.bag_path)
    lidar_extrinsic = load_lidar_extrinsic(settings.bag_path)
    getattr(rospy, "init_node")("bag_sensor_visualizer")
    BagSensorVisualizer(settings, livox_message_type, lidar_extrinsic)
    print("[READY] raw LiDAR, IMU vectors, and wheel odometry visualization", flush=True)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
