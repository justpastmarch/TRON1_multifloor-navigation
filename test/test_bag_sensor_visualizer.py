from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


SCRIPT = Path(__file__).resolve().parents[1] / "bag_sensor_visualizer.py"
SPEC = importlib.util.spec_from_file_location("bag_sensor_visualizer", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
visualizer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = visualizer
SPEC.loader.exec_module(visualizer)


def test_sample_livox_points_bounds_cloud_size_and_preserves_coordinates() -> None:
    # Given: more raw Livox points than one RViz frame should publish.
    points = tuple(
        visualizer.LivoxPoint(float(index), float(index + 1), float(index + 2), index)
        for index in range(10)
    )

    # When: the frame is sampled to four points.
    sampled = visualizer.sample_livox_points(points, 4)

    # Then: sampling is deterministic and retains the complete selected points.
    assert sampled == (
        (0.0, 1.0, 2.0, 0.0),
        (3.0, 4.0, 5.0, 3.0),
        (6.0, 7.0, 8.0, 6.0),
        (9.0, 10.0, 11.0, 9.0),
    )


def test_vector_arrow_uses_two_points_without_integrating_imu() -> None:
    # Given: one raw IMU vector and a display-only scale.
    vector = visualizer.Vector3Value(1.0, -2.0, 0.5)

    # When: an RViz arrow is created.
    marker = visualizer.vector_arrow(
        visualizer.ArrowStyle(
            "base_Link",
            "livox_frame",
            "raw_imu",
            7,
            0.25,
            visualizer.Vector3Value(0.0, 0.0, 0.0),
        ),
        vector,
    )

    # Then: the marker represents only the instantaneous scaled vector.
    assert marker.header.frame_id == "base_Link"
    assert marker.id == 7
    assert [(point.x, point.y, point.z) for point in marker.points] == [
        (0.0, 0.0, 0.0),
        (0.25, -0.5, 0.125),
    ]


def test_status_lines_disclose_raw_sources_values_and_display_scales() -> None:
    # Given: the latest independent IMU and recorded odometry samples.
    lines = visualizer.status_lines(
        "livox_frame",
        visualizer.Vector3Value(0.1, -0.2, 9.81),
        visualizer.Vector3Value(0.01, -0.03, 0.05),
        visualizer.Point(-6.2, -18.35, 0.0),
    )

    # Then: the fixed panel cannot be mistaken for transformed or fused state.
    assert lines == (
        "ACC src=livox_frame latest raw | arrow x1.2",
        "x=+0.100 y=-0.200 z=+9.810 m/s^2",
        "GYRO src=livox_frame latest raw | arrow x20",
        "x=+0.010 y=-0.030 z=+0.050 rad/s",
        "ODOM src=odom recorded",
        "x=-6.20 y=-18.35 m",
    )


def test_wheel_path_markers_follow_the_reference_thin_trajectory_style() -> None:
    # Given: a recorded path drawn over a dense grayscale LiDAR history.
    points = tuple(visualizer.Point(float(index), 0.0, 0.35) for index in range(41))

    # When: the path is converted to operator-facing markers.
    line, landmarks = visualizer.wheel_path_markers(points)

    # Then: thin trajectory marks disclose the same coordinates without dominating.
    assert line.type == visualizer.Marker.LINE_STRIP
    assert line.scale.x <= 0.03
    assert landmarks.type == visualizer.Marker.SPHERE_LIST
    assert landmarks.scale.x <= 0.1
    assert landmarks.scale.x > line.scale.x
    assert [(point.x, point.y, point.z) for point in landmarks.points] == [
        (0.0, 0.0, 0.35),
        (20.0, 0.0, 0.35),
        (40.0, 0.0, 0.35),
    ]


def test_wheel_odometry_transform_has_exclusive_display_frames() -> None:
    # Given: one pose from the recorded wheel-odometry topic.
    odometry = visualizer.Odometry()
    odometry.header.stamp = visualizer.rospy.Time(12, 34)
    odometry.pose.pose.position.x = 1.25
    odometry.pose.pose.position.y = -2.5
    odometry.pose.pose.orientation.w = 1.0

    # When: the replay-only transform is built.
    transform = visualizer.wheel_odometry_transform(odometry)

    # Then: only wheel odometry can define the dynamic display transform.
    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "base_Link"
    assert transform.header.stamp == odometry.header.stamp
    assert transform.transform.translation.x == 1.25
    assert transform.transform.translation.y == -2.5


def test_rviz_matches_the_reference_environment_scale_visual_grammar() -> None:
    # Given: the operator-facing raw-sensor RViz configuration.
    config = (
        SCRIPT.parent / "src/multifloor_manager/rviz/wf_raw_sensor_compare.rviz"
    ).read_text(encoding="utf-8")

    # Then: a bounded raw history, subdued grid, and wider orbit lead the hierarchy.
    assert "Decay Time: 5\n" in config
    assert "Style: Points\n" in config
    assert "      Alpha: 0.22\n" in config
    assert "      Cell Size: 2\n" in config
    assert "      Alpha: 0.18\n" in config
    assert "      Size (Pixels): 1\n" in config
    assert "Image Topic: /replay/status/image\n" in config
    assert "      Distance: 28\n" in config


def test_status_panel_uses_the_reference_compact_second_dock_height() -> None:
    image = visualizer.render_status_image(
        "livox_frame",
        visualizer.Vector3Value(0.1, -0.2, 9.81),
        visualizer.Vector3Value(0.01, -0.03, 0.05),
        visualizer.Point(-6.2, -18.35, 0.0),
    )

    assert image.shape == (180, 640, 3)


def test_replay_launcher_defaults_to_an_isolated_master() -> None:
    # Given: the operator-facing replay script.
    source = (SCRIPT.parent / "replay_raw_sensors_rviz.sh").read_text(encoding="utf-8")

    # Then: it cannot silently inject recorded TF into the normal ROS master.
    assert "http://127.0.0.1:11319" in source
    assert "ALLOW_REPLAY_MASTER_REUSE" in source
    play_topics = source.split("rosbag play", 1)[1]
    assert "\n    /tf" not in play_topics
