"""Gravity initialization, gyro rotation prediction, and geometric registration."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import NamedTuple

import numpy as np
import numpy.typing as npt
import open3d as o3d
from scipy.spatial.transform import Rotation, Slerp

FloatMatrix = npt.NDArray[np.float64]


class PointCloudShapeError(Exception):
    """Raised when point-cloud or transform dimensions violate the API contract."""

    __slots__ = ("actual", "expected", "name")

    def __init__(self, name: str, expected: str, actual: tuple[int, ...]) -> None:
        super().__init__(name, expected, actual)
        self.name = name
        self.expected = expected
        self.actual = actual

    def __str__(self) -> str:
        return f"{self.name} must have shape {self.expected}; got {self.actual}"


class RegistrationConfig(NamedTuple):
    """Geometric acceptance limits relative to the supplied pose prediction.

    Field names are retained for existing specs. Corrections from a prediction
    are innovations, not independently measured one-frame physical motion.
    """

    voxel_size_m: float
    correspondence_distance_m: float
    max_iterations: int
    min_fitness: float
    max_translation_step_m: float
    max_rotation_step_rad: float


class RegistrationResult(NamedTuple):
    """One scan-to-map estimate and the evidence used to admit it."""

    transform: FloatMatrix
    fitness: float
    inlier_rmse_m: float
    translation_step_m: float
    rotation_step_rad: float
    accepted: bool

    def rotation_error_to(self, expected: FloatMatrix) -> float:
        """Return the geodesic SO(3) error to an expected transform."""
        relative_rotation = self.transform[:3, :3] @ expected[:3, :3].T
        cosine = float(np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0))
        return math.acos(cosine)


def _require_shape(name: str, value: FloatMatrix, expected: tuple[int, ...]) -> None:
    if value.shape != expected:
        raise PointCloudShapeError(
            name=name, expected=str(expected), actual=value.shape
        )


def _point_cloud(points: FloatMatrix, voxel_size_m: float) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    downsampled = cloud.voxel_down_sample(voxel_size_m)
    downsampled.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size_m * 3.0,
            max_nn=30,
        )
    )
    return downsampled


def voxel_downsample_points(
    points_xyz: FloatMatrix, voxel_size_m: float
) -> FloatMatrix:
    """Return deterministic voxel centroids as an N×3 float matrix."""
    if points_xyz.ndim != 2 or points_xyz.shape[1:] != (3,):
        raise PointCloudShapeError(
            name="points_xyz",
            expected="(N, 3)",
            actual=points_xyz.shape,
        )
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points_xyz)
    return np.asarray(cloud.voxel_down_sample(voxel_size_m).points, dtype=np.float64)


def merge_registered_scans(
    scans: Sequence[FloatMatrix],
    voxel_size_m: float,
) -> FloatMatrix:
    """Voxelize accepted world-frame scans into one full-traversal map."""
    return voxel_downsample_points(np.vstack(scans), voxel_size_m)


def _rotation_angle(transform: FloatMatrix) -> float:
    cosine = float(np.clip((np.trace(transform[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


def level_transform_from_gravity(up_lidar: FloatMatrix) -> FloatMatrix:
    """Align calibrated accelerometer up with world +Z, never a scene plane.

    Accelerometer magnitude cancels. The caller supplies a low-dynamic initial
    window; this operation alone cannot certify stationarity or calibration.
    """
    up = np.asarray(up_lidar, dtype=np.float64)
    _require_shape("up_lidar", up, (3,))
    if not np.isfinite(up).all() or np.linalg.norm(up) < 1e-9:
        raise ValueError("invalid accelerometer up vector")
    up = up / np.linalg.norm(up)
    cosine = float(np.clip(up[2], -1.0, 1.0))
    axis = np.cross(up, [0.0, 0.0, 1.0])
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    rotation = (
        Rotation.from_rotvec([math.pi, 0.0, 0.0]).as_matrix()
        if cosine < -1.0 + 1e-8
        else np.eye(3) + skew + skew @ skew / (1.0 + cosine)
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    return transform


class ImuRotation:
    """Header-time gyro integration in LiDAR axes; no translational integration."""

    def __init__(
        self, samples: FloatMatrix, lidar_from_imu: FloatMatrix, max_gap_s: float
    ) -> None:
        # Columns: header seconds, angular xyz rad/s, acceleration xyz.
        if samples.ndim != 2 or samples.shape[1] != 7 or len(samples) < 2:
            raise ValueError(
                "need at least two IMU samples [stamp, gyro xyz, acceleration xyz]"
            )
        if not np.isfinite(samples).all() or np.any(np.diff(samples[:, 0]) <= 0):
            raise ValueError(
                "IMU samples must be finite with strictly increasing header stamps"
            )
        self.times = samples[:, 0]
        self.acceleration = samples[:, 4:7] @ lidar_from_imu.T
        self.max_gap_s = max_gap_s
        gyro = samples[:, 1:4] @ lidar_from_imu.T
        rotations = [np.eye(3)]
        for i, dt in enumerate(np.diff(self.times)):
            rotations.append(
                rotations[-1]
                @ Rotation.from_rotvec(0.5 * (gyro[i] + gyro[i + 1]) * dt).as_matrix()
            )
        self.interpolator = Slerp(
            self.times - self.times[0], Rotation.from_matrix(rotations)
        )

    def _support(self, start_s: float, end_s: float) -> None:
        if start_s < self.times[0] or end_s > self.times[-1] or end_s < start_s:
            raise ValueError("IMU interval outside recorded header times")
        first = max(0, int(np.searchsorted(self.times, start_s, side="right")) - 1)
        last = min(len(self.times), int(np.searchsorted(self.times, end_s)) + 1)
        if np.max(np.diff(self.times[first:last]), initial=0.0) > self.max_gap_s:
            raise ValueError("IMU sample gap")

    def initial_transform(self, stamp_s: float, half_window_s: float) -> FloatMatrix:
        selected = np.abs(self.times - stamp_s) <= half_window_s
        if selected.sum() < 2 or not (self.times[0] <= stamp_s <= self.times[-1]):
            raise ValueError("insufficient IMU support for gravity initialization")
        # Gravity uses a median, not time integration over the whole window.
        # Require support at initialization, not uninterrupted distant samples.
        self._support(stamp_s, stamp_s)
        return level_transform_from_gravity(
            np.median(self.acceleration[selected], axis=0)
        )

    def relative(self, start_s: float, end_s: float) -> FloatMatrix:
        self._support(start_s, end_s)
        rotations = self.interpolator(
            np.array([start_s, end_s]) - self.times[0]
        ).as_matrix()
        return rotations[0].T @ rotations[1]


def register_scan_to_map(
    source_xyz: FloatMatrix,
    target_xyz: FloatMatrix,
    initial_transform: FloatMatrix,
    config: RegistrationConfig,
) -> RegistrationResult:
    """Register a current sensor scan into a local map using generalized ICP."""
    if source_xyz.ndim != 2 or source_xyz.shape[1:] != (3,):
        raise PointCloudShapeError(
            name="source_xyz",
            expected="(N, 3)",
            actual=source_xyz.shape,
        )
    if target_xyz.ndim != 2 or target_xyz.shape[1:] != (3,):
        raise PointCloudShapeError(
            name="target_xyz",
            expected="(N, 3)",
            actual=target_xyz.shape,
        )
    _require_shape("initial_transform", initial_transform, (4, 4))

    registration = o3d.pipelines.registration.registration_generalized_icp(
        _point_cloud(source_xyz, config.voxel_size_m),
        _point_cloud(target_xyz, config.voxel_size_m),
        config.correspondence_distance_m,
        initial_transform,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,
            relative_rmse=1e-6,
            max_iteration=config.max_iterations,
        ),
    )
    transform = np.asarray(registration.transformation, dtype=np.float64)
    correction = np.linalg.inv(initial_transform) @ transform
    translation_step_m = float(np.linalg.norm(correction[:3, 3]))
    rotation_step_rad = _rotation_angle(correction)
    accepted = (
        registration.fitness >= config.min_fitness
        and translation_step_m <= config.max_translation_step_m
        and rotation_step_rad <= config.max_rotation_step_rad
    )
    return RegistrationResult(
        transform=transform,
        fitness=float(registration.fitness),
        inlier_rmse_m=float(registration.inlier_rmse),
        translation_step_m=translation_step_m,
        rotation_step_rad=rotation_step_rad,
        accepted=accepted,
    )
