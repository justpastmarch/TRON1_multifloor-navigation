"""Render raw replay values into a projection-independent RViz image panel."""

from __future__ import annotations

from typing import Protocol

import cv2
import numpy as np


class Vector3Like(Protocol):
    x: float
    y: float
    z: float


def status_lines(
    source_frame: str,
    acceleration: Vector3Like,
    angular_velocity: Vector3Like,
    odometry: Vector3Like,
) -> tuple[str, str, str, str, str, str]:
    """Format independent latest samples without implying synchronization or fusion."""
    return (
        f"ACC src={source_frame} latest raw | arrow x1.2",
        f"x={acceleration.x:+.3f} y={acceleration.y:+.3f} z={acceleration.z:+.3f} m/s^2",
        f"GYRO src={source_frame} latest raw | arrow x20",
        f"x={angular_velocity.x:+.3f} y={angular_velocity.y:+.3f} "
        f"z={angular_velocity.z:+.3f} rad/s",
        "ODOM src=odom recorded",
        f"x={odometry.x:+.2f} y={odometry.y:+.2f} m",
    )


def render_status_image(
    source_frame: str,
    acceleration: Vector3Like,
    angular_velocity: Vector3Like,
    odometry: Vector3Like,
) -> np.ndarray:
    """Draw fixed-position raw values for RViz's depth-independent Image display."""
    image = np.full((180, 640, 3), (27, 21, 18), dtype=np.uint8)
    colors = (
        (255, 200, 80),
        (255, 200, 80),
        (255, 96, 255),
        (255, 96, 255),
        (120, 255, 80),
        (120, 255, 80),
    )
    for text, color, baseline in zip(
        status_lines(source_frame, acceleration, angular_velocity, odometry),
        colors,
        (22, 44, 72, 94, 124, 148),
    ):
        cv2.putText(image, text, (16, baseline), cv2.FONT_HERSHEY_DUPLEX, 0.6, color, 1, cv2.LINE_AA)
    cv2.line(image, (16, 57), (624, 57), (70, 70, 70), 1, cv2.LINE_AA)
    cv2.line(image, (16, 108), (624, 108), (70, 70, 70), 1, cv2.LINE_AA)
    return image
