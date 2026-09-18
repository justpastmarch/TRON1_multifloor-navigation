"""Pure fail-closed admission policy for a canonical stair-entry map pose."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StairEntryPolicy:
    __slots__ = (
        "xy_tolerance_m",
        "yaw_tolerance_rad",
        "freshness_sec",
        "max_covariance_x",
        "max_covariance_y",
        "max_covariance_yaw",
        "required_pose_samples",
        "max_linear_speed",
        "max_angular_speed",
    )
    xy_tolerance_m: float
    yaw_tolerance_rad: float
    freshness_sec: float
    max_covariance_x: float
    max_covariance_y: float
    max_covariance_yaw: float
    required_pose_samples: int
    max_linear_speed: float
    max_angular_speed: float


@dataclass(frozen=True)
class StairEntryFence:
    __slots__ = ("floor_id", "map_generation", "ros_time_sec", "monotonic_time", "pose_sequence")
    floor_id: str
    map_generation: int
    ros_time_sec: float
    monotonic_time: float
    pose_sequence: int


@dataclass(frozen=True)
class StairEntryPose:
    __slots__ = ("x_m", "y_m", "yaw_rad")
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class StairEntryEvidence:
    __slots__ = (
        "frame_id",
        "floor_id",
        "map_generation",
        "pose_stamp_sec",
        "pose_received_monotonic",
        "pose_sequence",
        "x_m",
        "y_m",
        "orientation_x",
        "orientation_y",
        "orientation_z",
        "orientation_w",
        "covariance_x",
        "covariance_y",
        "covariance_yaw",
        "odom_received_monotonic",
        "linear_speed_mps",
        "angular_speed_radps",
    )
    frame_id: str
    floor_id: str
    map_generation: int
    pose_stamp_sec: float
    pose_received_monotonic: float
    pose_sequence: int
    x_m: float
    y_m: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    covariance_x: float
    covariance_y: float
    covariance_yaw: float
    odom_received_monotonic: float
    linear_speed_mps: float
    angular_speed_radps: float


@dataclass(frozen=True)
class StairEntryCheck:
    __slots__ = ("policy", "fence", "expected", "evidence", "ros_now_sec", "monotonic_now")
    policy: StairEntryPolicy
    fence: StairEntryFence
    expected: StairEntryPose
    evidence: StairEntryEvidence | None
    ros_now_sec: float
    monotonic_now: float


@dataclass(frozen=True)
class StairEntryDecision:
    __slots__ = ("accepted", "reason")
    accepted: bool
    reason: str


def _reject(reason: str) -> StairEntryDecision:
    return StairEntryDecision(False, reason)


def evaluate_stair_entry(check: StairEntryCheck) -> StairEntryDecision:
    """Accept only current-map, post-fence, localized, stationary entry evidence."""
    evidence = check.evidence
    if evidence is None:
        return _reject("stair entry evidence is unavailable")
    if evidence.frame_id != "map":
        return _reject("AMCL pose is not in map frame")
    if (evidence.floor_id, evidence.map_generation) != (
        check.fence.floor_id,
        check.fence.map_generation,
    ):
        return _reject("AMCL pose belongs to a different floor generation")
    if evidence.pose_sequence - check.fence.pose_sequence < check.policy.required_pose_samples:
        return _reject("insufficient post-fence AMCL pose samples")
    if (
        evidence.pose_stamp_sec <= check.fence.ros_time_sec
        or evidence.pose_received_monotonic <= check.fence.monotonic_time
        or evidence.odom_received_monotonic <= check.fence.monotonic_time
    ):
        return _reject("entry evidence predates the handoff fence")
    pose_age = check.ros_now_sec - evidence.pose_stamp_sec
    receipt_age = check.monotonic_now - evidence.pose_received_monotonic
    odom_age = check.monotonic_now - evidence.odom_received_monotonic
    if not all(0.0 <= age <= check.policy.freshness_sec for age in (pose_age, receipt_age, odom_age)):
        return _reject("entry evidence is stale or future-dated")
    numeric = (
        evidence.x_m,
        evidence.y_m,
        evidence.orientation_x,
        evidence.orientation_y,
        evidence.orientation_z,
        evidence.orientation_w,
        evidence.covariance_x,
        evidence.covariance_y,
        evidence.covariance_yaw,
        evidence.linear_speed_mps,
        evidence.angular_speed_radps,
    )
    if not all(math.isfinite(value) for value in numeric):
        return _reject("entry evidence contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in numeric[2:6]))
    if abs(norm - 1.0) > 0.01:
        return _reject("AMCL orientation is not normalized")
    covariance = numeric[6:9]
    if any(value < 0.0 for value in covariance):
        return _reject("AMCL covariance is negative")
    if covariance[0] > check.policy.max_covariance_x or covariance[1] > check.policy.max_covariance_y or covariance[2] > check.policy.max_covariance_yaw:
        return _reject("AMCL covariance exceeds the stair-entry policy")
    yaw = math.atan2(
        2.0 * (evidence.orientation_w * evidence.orientation_z + evidence.orientation_x * evidence.orientation_y),
        1.0 - 2.0 * (evidence.orientation_y * evidence.orientation_y + evidence.orientation_z * evidence.orientation_z),
    )
    if math.hypot(evidence.x_m - check.expected.x_m, evidence.y_m - check.expected.y_m) > check.policy.xy_tolerance_m:
        return _reject("robot is outside the stair-entry position tolerance")
    yaw_error = math.atan2(math.sin(yaw - check.expected.yaw_rad), math.cos(yaw - check.expected.yaw_rad))
    if abs(yaw_error) > check.policy.yaw_tolerance_rad and not math.isclose(
        abs(yaw_error), check.policy.yaw_tolerance_rad, abs_tol=1e-12
    ):
        return _reject("robot is outside the stair-entry heading tolerance")
    if abs(evidence.linear_speed_mps) > check.policy.max_linear_speed or abs(evidence.angular_speed_radps) > check.policy.max_angular_speed:
        return _reject("robot is not stationary at the stair entry")
    return StairEntryDecision(True, "fresh aligned stair-entry evidence")
