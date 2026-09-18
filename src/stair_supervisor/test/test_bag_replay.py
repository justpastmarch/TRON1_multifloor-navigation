"""Tests for replaying recorded odometry through the ROS stair boundary."""

from __future__ import annotations

from nav_msgs.msg import Odometry
import rospy

from stair_supervisor.bag_replay import odometry_sample, restamp_odometry


def test_restamp_odometry_preserves_recorded_pose_and_uses_current_time() -> None:
    # Given: one historical wheel-odometry message from a bag.
    source = Odometry()
    source.header.stamp = rospy.Time(12, 34)
    source.header.frame_id = "odom"
    source.child_frame_id = "base_Link"
    source.pose.pose.position.x = 3.25
    source.pose.pose.position.y = -1.75
    source.pose.pose.orientation.z = 0.5
    source.pose.pose.orientation.w = 0.8660254
    current = rospy.Time(100, 500)

    # When: the replay boundary prepares it for the live supervisor.
    replayed = restamp_odometry(source, current)

    # Then: only freshness time changes; recorded evidence and source stay intact.
    assert replayed is not source
    assert replayed.header.stamp == current
    assert replayed.header.frame_id == "odom"
    assert replayed.child_frame_id == "base_Link"
    assert replayed.pose.pose.position.x == 3.25
    assert replayed.pose.pose.position.y == -1.75
    assert source.header.stamp == rospy.Time(12, 34)


def test_odometry_sample_uses_record_time_and_recorded_planar_pose() -> None:
    # Given: a replayed pose with a known one-radian yaw.
    message = Odometry()
    message.pose.pose.position.x = 2.0
    message.pose.pose.position.y = 4.0
    message.pose.pose.orientation.z = 0.47942554
    message.pose.pose.orientation.w = 0.87758256

    # When: it crosses the evidence-state-machine boundary.
    sample = odometry_sample(message, 7.5)

    # Then: bag time and recorded planar values are preserved.
    assert sample.stamp == 7.5
    assert sample.x_m == 2.0
    assert sample.y_m == 4.0
    assert abs(sample.yaw_rad - 1.0) < 1e-6
