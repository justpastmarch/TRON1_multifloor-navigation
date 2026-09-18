"""Pure message conversions used by the raw-sensor RViz replay node."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import NamedTuple, Protocol, Sequence

import rospy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker


class LivoxPointLike(Protocol):
    x: float
    y: float
    z: float
    reflectivity: int


class LivoxHeaderLike(Protocol):
    stamp: rospy.Time


class LivoxMessageLike(Protocol):
    header: LivoxHeaderLike
    points: Sequence[LivoxPointLike]


class Vector3Like(Protocol):
    x: float
    y: float
    z: float


class LivoxPoint(NamedTuple):
    x: float
    y: float
    z: float
    reflectivity: int


class Vector3Value(NamedTuple):
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class ArrowStyle:
    display_frame: str
    source_frame: str
    namespace: str
    marker_id: int
    scale: float
    origin: Vector3Value


@dataclass(frozen=True)
class BagMessageError(Exception):
    detail: str

    def __str__(self) -> str:
        return self.detail


def sample_livox_points(
    points: Sequence[LivoxPointLike],
    maximum: int,
) -> tuple[tuple[float, float, float, float], ...]:
    """Sample one raw Livox frame uniformly for bounded RViz rendering."""
    if maximum <= 0:
        raise BagMessageError("maximum point count must be positive")
    stride = max(1, math.ceil(len(points) / maximum))
    return tuple(
        (point.x, point.y, point.z, float(point.reflectivity))
        for point in points[::stride]
    )


def vector_arrow(style: ArrowStyle, vector: Vector3Like) -> Marker:
    """Represent one instantaneous IMU vector without integrating orientation."""
    marker = Marker()
    marker.header.frame_id = style.display_frame
    marker.ns = style.namespace
    marker.id = style.marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.points = [
        Point(style.origin.x, style.origin.y, style.origin.z),
        Point(
            style.origin.x + vector.x * style.scale,
            style.origin.y + vector.y * style.scale,
            style.origin.z + vector.z * style.scale,
        ),
    ]
    marker.scale = Vector3(0.025, 0.05, 0.08)
    marker.pose.orientation.w = 1.0
    marker.frame_locked = True
    return marker


def wheel_path_markers(points: Sequence[Point]) -> tuple[Marker, Marker]:
    """Render one recorded path as a salient line with periodic landmarks."""
    line = Marker(ns="wheel_odometry_path", id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
    line.header.frame_id = "odom"
    line.pose.orientation.w = 1.0
    line.scale.x = 0.025
    line.color.g = 1.0
    line.color.a = 1.0
    line.points = list(points)

    landmarks = Marker(ns="wheel_odometry_landmarks", id=0, type=Marker.SPHERE_LIST, action=Marker.ADD)
    landmarks.header.frame_id = "odom"
    landmarks.pose.orientation.w = 1.0
    landmarks.scale = Vector3(0.08, 0.08, 0.08)
    landmarks.color.g = 1.0
    landmarks.color.a = 1.0
    landmarks.points = list(points[::20])
    if points and (len(points) - 1) % 20:
        landmarks.points.append(points[-1])

    return line, landmarks
