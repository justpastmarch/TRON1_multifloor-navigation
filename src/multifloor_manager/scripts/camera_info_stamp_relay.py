#!/usr/bin/env python3
"""Relay camera image + info so both share the image header stamp.

Some RealSense deployments stamp /camera_info with ROS wall time while the
matching /image uses the device clock.  apriltag_ros requires synchronised
stamps and crashes with an empty zarray when no pair matches.  This node
republishes the image unchanged and re-stamps the latest camera_info with the
image stamp so AprilTag always sees a valid pair.
"""
from __future__ import annotations

import rospy
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from cv_bridge import CvBridge


class CameraInfoStampRelay:
    def __init__(self) -> None:
        self._info: CameraInfo | None = None
        self._pub_compressed = rospy.Publisher("~compressed_out", CompressedImage, queue_size=1)
        self._pub_image = rospy.Publisher("~image_out", Image, queue_size=2)
        self._pub_info = rospy.Publisher("~camera_info_out", CameraInfo, queue_size=2)
        self._bridge = CvBridge()
        transport = rospy.get_param("~input_transport", "raw")
        if transport == "compressed":
            rospy.Subscriber("~compressed_image", CompressedImage, self._compressed_cb,
                             queue_size=1, buff_size=2*1024*1024)
        elif transport == "raw":
            rospy.Subscriber("~image", Image, self._image_cb, queue_size=1,
                             buff_size=8*1024*1024)
        else:
            raise ValueError("unsupported camera input transport: " + transport)
        rospy.Subscriber("~camera_info", CameraInfo, self._info_cb, queue_size=2)

    def _compressed_cb(self, msg: CompressedImage) -> None:
        # Share the original bytes locally; do not re-encode or change timestamps.
        self._pub_compressed.publish(msg)
        if self._pub_image.get_num_connections() == 0:
            return
        try:
            image = self._bridge.cv2_to_imgmsg(
                self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8"), encoding="bgr8")
            image.header = msg.header
            self._image_cb(image)
        except Exception as error:
            rospy.logwarn_throttle(5., "camera compressed decode failed: %s", error)

    def _info_cb(self, msg: CameraInfo) -> None:
        self._info = msg

    def _image_cb(self, msg: Image) -> None:
        if self._pub_image.get_num_connections() == 0 and self._pub_info.get_num_connections() == 0:
            return
        self._pub_image.publish(msg)
        if self._info is None:
            rospy.logwarn_throttle(5.0, "camera_info_stamp_relay: no camera_info received yet")
            return
        info = CameraInfo()
        info.header.stamp = msg.header.stamp
        info.header.frame_id = self._info.header.frame_id
        info.height = self._info.height
        info.width = self._info.width
        info.distortion_model = self._info.distortion_model
        info.D = self._info.D
        info.K = self._info.K
        info.R = self._info.R
        info.P = self._info.P
        info.binning_x = self._info.binning_x
        info.binning_y = self._info.binning_y
        info.roi = self._info.roi
        self._pub_info.publish(info)


def main() -> int:
    rospy.init_node("camera_info_stamp_relay")
    CameraInfoStampRelay()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
