"""Pure post-initialpose localization readiness policy."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Final, NewType, Optional, Tuple


Nanoseconds = NewType("Nanoseconds", int)


@dataclass(frozen=True)
class ReadinessPolicy:
    __slots__ = (
        "max_age_ns",
        "timeout_ns",
        "max_covariance_x",
        "max_covariance_y",
        "max_covariance_yaw",
        "required_pose_samples",
        "max_linear_speed",
        "max_angular_speed",
    )
    max_age_ns: Nanoseconds
    timeout_ns: Nanoseconds
    max_covariance_x: float
    max_covariance_y: float
    max_covariance_yaw: float
    required_pose_samples: int
    max_linear_speed: float
    max_angular_speed: float


DEFAULT_READINESS_POLICY: Final = ReadinessPolicy(
    Nanoseconds(500_000_000),
    Nanoseconds(30_000_000_000),
    0.05,
    0.05,
    0.10,
    3,
    0.03,
    0.04,
)


@dataclass(frozen=True)
class LocalizationState:
    __slots__ = (
        "armed_at_ns",
        "pose_count",
        "pose_stamp_ns",
        "pose_received_ns",
        "scan_stamp_ns",
        "scan_received_ns",
        "scan_tf_valid",
        "odom_stamp_ns",
        "odom_received_ns",
        "odom_stationary",
    )
    armed_at_ns: Nanoseconds
    pose_count: int
    pose_stamp_ns: Optional[Nanoseconds]
    pose_received_ns: Optional[Nanoseconds]
    scan_stamp_ns: Optional[Nanoseconds]
    scan_received_ns: Optional[Nanoseconds]
    scan_tf_valid: bool
    odom_stamp_ns: Optional[Nanoseconds]
    odom_received_ns: Optional[Nanoseconds]
    odom_stationary: bool


@dataclass(frozen=True)
class DebugPredicate:
    __slots__ = ("name", "value")
    name: str
    value: bool


@dataclass(frozen=True)
class LocalizationEvidence:
    __slots__ = (
        "within_timeout",
        "pose_samples",
        "fresh_pose",
        "fresh_scan",
        "scan_tf",
        "stationary_odom",
        "fresh_odom",
        "ready",
    )
    within_timeout: bool
    pose_samples: bool
    fresh_pose: bool
    fresh_scan: bool
    scan_tf: bool
    stationary_odom: bool
    fresh_odom: bool
    ready: bool

    def predicates(self) -> Tuple[DebugPredicate, ...]:
        """Return the stable Boolean seams published by the ROS boundary."""
        return (
            DebugPredicate("localization_within_timeout", self.within_timeout),
            DebugPredicate("amcl_pose_samples", self.pose_samples),
            DebugPredicate("amcl_pose_fresh", self.fresh_pose),
            DebugPredicate("scan_fresh", self.fresh_scan),
            DebugPredicate("scan_tf_valid", self.scan_tf),
            DebugPredicate("odom_stationary", self.stationary_odom),
            DebugPredicate("odom_fresh", self.fresh_odom),
            DebugPredicate("localization_ready", self.ready),
        )


def arm_localization(armed_at_ns: Nanoseconds) -> LocalizationState:
    """Discard all evidence from before the managed initial pose."""
    return LocalizationState(armed_at_ns, 0, None, None, None, None, False, None, None, False)


# Measured inter-host skew is a few milliseconds; never tolerate a scan-period
# of future data. This does not increase the 0.5-second stale-data budget.
CLOCK_SKEW_TOLERANCE_NS = 10_000_000

def _fresh(stamp_ns: Optional[Nanoseconds], now_ns: Nanoseconds, max_age_ns: Nanoseconds) -> bool:
    return stamp_ns is not None and -CLOCK_SKEW_TOLERANCE_NS <= int(now_ns) - int(stamp_ns) <= int(max_age_ns)


def observe_pose(
    state: LocalizationState,
    stamp_ns: Nanoseconds,
    received_at_ns: Nanoseconds,
    covariance_diagonal: Tuple[float, float, float],
) -> LocalizationState:
    """Accept one ordered, fresh, bounded AMCL covariance sample."""
    covariance_valid = (
        all(math.isfinite(value) for value in covariance_diagonal)
        and covariance_diagonal[0] <= DEFAULT_READINESS_POLICY.max_covariance_x
        and covariance_diagonal[1] <= DEFAULT_READINESS_POLICY.max_covariance_y
        and covariance_diagonal[2] <= DEFAULT_READINESS_POLICY.max_covariance_yaw
    )
    post_arm = int(stamp_ns) > int(state.armed_at_ns)
    ordered = state.pose_stamp_ns is None or int(stamp_ns) > int(state.pose_stamp_ns)
    fresh = _fresh(stamp_ns, received_at_ns, DEFAULT_READINESS_POLICY.max_age_ns)
    accepted = covariance_valid and post_arm and ordered and fresh
    return LocalizationState(
        state.armed_at_ns,
        state.pose_count + 1 if accepted else 0,
        stamp_ns if ordered else state.pose_stamp_ns,
        received_at_ns if ordered else state.pose_received_ns,
        state.scan_stamp_ns,
        state.scan_received_ns,
        state.scan_tf_valid,
        state.odom_stamp_ns,
        state.odom_received_ns,
        state.odom_stationary,
    )


def observe_scan(
    state: LocalizationState,
    stamp_ns: Nanoseconds,
    received_at_ns: Nanoseconds,
    transform_valid: bool,
) -> LocalizationState:
    """Retain only a fresh post-arm scan and its same-stamp TF result."""
    accepted = (
        int(stamp_ns) > int(state.armed_at_ns)
        and _fresh(stamp_ns, received_at_ns, DEFAULT_READINESS_POLICY.max_age_ns)
        and (state.scan_stamp_ns is None or int(stamp_ns) > int(state.scan_stamp_ns))
    )
    return LocalizationState(
        state.armed_at_ns,
        state.pose_count,
        state.pose_stamp_ns,
        state.pose_received_ns,
        stamp_ns if accepted else state.scan_stamp_ns,
        received_at_ns if accepted else state.scan_received_ns,
        transform_valid if accepted else state.scan_tf_valid,
        state.odom_stamp_ns,
        state.odom_received_ns,
        state.odom_stationary,
    )


def observe_odometry(
    state: LocalizationState,
    stamp_ns: Nanoseconds,
    received_at_ns: Nanoseconds,
    linear_speed: float,
    angular_speed: float,
) -> LocalizationState:
    """Retain the latest fresh post-arm stationary odometry sample."""
    accepted = (
        int(stamp_ns) > int(state.armed_at_ns)
        and _fresh(stamp_ns, received_at_ns, DEFAULT_READINESS_POLICY.max_age_ns)
        and (state.odom_stamp_ns is None or int(stamp_ns) > int(state.odom_stamp_ns))
    )
    stationary = (
        math.isfinite(linear_speed)
        and math.isfinite(angular_speed)
        and abs(linear_speed) <= DEFAULT_READINESS_POLICY.max_linear_speed
        and abs(angular_speed) <= DEFAULT_READINESS_POLICY.max_angular_speed
    )
    return LocalizationState(
        state.armed_at_ns,
        state.pose_count,
        state.pose_stamp_ns,
        state.pose_received_ns,
        state.scan_stamp_ns,
        state.scan_received_ns,
        state.scan_tf_valid,
        stamp_ns if accepted else state.odom_stamp_ns,
        received_at_ns if accepted else state.odom_received_ns,
        stationary if accepted else state.odom_stationary,
    )


def localization_ready(
    state: LocalizationState,
    now_ns: Nanoseconds,
    policy: ReadinessPolicy,
) -> LocalizationEvidence:
    """Evaluate the complete AMCL, sensor, motion, TF, and timeout conjunction."""
    within_timeout = 0 <= int(now_ns) - int(state.armed_at_ns) <= int(policy.timeout_ns)
    pose_samples = state.pose_count >= policy.required_pose_samples
    fresh_pose = _fresh(state.pose_stamp_ns, now_ns, policy.max_age_ns)
    fresh_scan = _fresh(state.scan_stamp_ns, now_ns, policy.max_age_ns)
    fresh_odom = _fresh(state.odom_stamp_ns, now_ns, policy.max_age_ns)
    ready = all(
        (
            within_timeout,
            pose_samples,
            fresh_pose,
            fresh_scan,
            state.scan_tf_valid,
            state.odom_stationary,
            fresh_odom,
        )
    )
    return LocalizationEvidence(
        within_timeout,
        pose_samples,
        fresh_pose,
        fresh_scan,
        state.scan_tf_valid,
        state.odom_stationary,
        fresh_odom,
        ready,
    )
