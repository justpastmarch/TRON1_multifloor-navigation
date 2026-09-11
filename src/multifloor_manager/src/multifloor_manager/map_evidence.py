"""Pure OccupancyGrid fingerprint and guarded map-generation policy."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from typing import NewType, Optional, Protocol, Sequence, Tuple


Nanoseconds = NewType("Nanoseconds", int)
MapGeneration = NewType("MapGeneration", int)


class RosTimeLike(Protocol):
    def to_nsec(self) -> int:
        ...


class HeaderLike(Protocol):
    @property
    def frame_id(self) -> str:
        ...


class VectorLike(Protocol):
    x: float
    y: float
    z: float


class QuaternionLike(Protocol):
    x: float
    y: float
    z: float
    w: float


class PoseLike(Protocol):
    @property
    def position(self) -> VectorLike:
        ...

    @property
    def orientation(self) -> QuaternionLike:
        ...


class MapInfoLike(Protocol):
    map_load_time: RosTimeLike
    resolution: float
    width: int
    height: int
    origin: PoseLike


class OccupancyGridLike(Protocol):
    @property
    def header(self) -> HeaderLike:
        ...

    @property
    def info(self) -> MapInfoLike:
        ...

    @property
    def data(self) -> Sequence[int]:
        ...


@dataclass(frozen=True)
class MapBoundaryError(Exception):
    __slots__ = ("field", "detail")
    field: str
    detail: str

    def __str__(self) -> str:
        return "OccupancyGrid.{}: {}".format(self.field, self.detail)


@dataclass(frozen=True)
class MapOrigin:
    __slots__ = ("x", "y", "z", "qx", "qy", "qz", "qw")
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class MapIdentity:
    __slots__ = ("frame_id", "width", "height", "resolution", "origin", "data_hash")
    frame_id: str
    width: int
    height: int
    resolution: float
    origin: MapOrigin
    data_hash: str


@dataclass(frozen=True)
class MapFingerprint:
    __slots__ = (
        "frame_id",
        "width",
        "height",
        "resolution",
        "origin",
        "data_hash",
        "map_load_time_ns",
    )
    frame_id: str
    width: int
    height: int
    resolution: float
    origin: MapOrigin
    data_hash: str
    map_load_time_ns: Nanoseconds

    def identity(self) -> MapIdentity:
        """Return target-comparable content while retaining load time in the fingerprint."""
        return MapIdentity(
            self.frame_id,
            self.width,
            self.height,
            self.resolution,
            self.origin,
            self.data_hash,
        )


@dataclass(frozen=True)
class MapGenerationState:
    __slots__ = ("generation", "current_fingerprint")
    generation: MapGeneration
    current_fingerprint: Optional[MapFingerprint]


@dataclass(frozen=True)
class MapGenerationGuard:
    __slots__ = ("state", "armed_at_ns", "target_identity", "armed")
    state: MapGenerationState
    armed_at_ns: Nanoseconds
    target_identity: MapIdentity
    armed: bool


@dataclass(frozen=True)
class DebugPredicate:
    __slots__ = ("name", "value")
    name: str
    value: bool


@dataclass(frozen=True)
class MapEvidence:
    __slots__ = (
        "guard_armed",
        "post_arm",
        "target_fingerprint",
        "new_fingerprint",
        "monotonic_load_time",
        "accepted",
    )
    guard_armed: bool
    post_arm: bool
    target_fingerprint: bool
    new_fingerprint: bool
    monotonic_load_time: bool
    accepted: bool

    def predicates(self) -> Tuple[DebugPredicate, ...]:
        """Return stable Boolean seams for future std_msgs/Bool publishers."""
        return (
            DebugPredicate("map_guard_armed", self.guard_armed),
            DebugPredicate("map_post_arm", self.post_arm),
            DebugPredicate("map_target_fingerprint", self.target_fingerprint),
            DebugPredicate("map_new_fingerprint", self.new_fingerprint),
            DebugPredicate("map_load_time_monotonic", self.monotonic_load_time),
            DebugPredicate("map_accepted", self.accepted),
        )


@dataclass(frozen=True)
class MapGenerationResult:
    __slots__ = ("guard", "evidence")
    guard: MapGenerationGuard
    evidence: MapEvidence


def _finite_components(message: OccupancyGridLike) -> Tuple[float, ...]:
    position = message.info.origin.position
    orientation = message.info.origin.orientation
    return (
        float(position.x),
        float(position.y),
        float(position.z),
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )


def fingerprint_occupancy_grid(
    message: OccupancyGridLike,
    received_at_ns: Nanoseconds,
) -> MapFingerprint:
    """Parse a nav_msgs/OccupancyGrid-compatible value into a canonical fingerprint."""
    if not message.header.frame_id:
        raise MapBoundaryError("header.frame_id", "must not be empty")
    if type(message.info.width) is not int or message.info.width <= 0:
        raise MapBoundaryError("info.width", "must be a positive integer")
    if type(message.info.height) is not int or message.info.height <= 0:
        raise MapBoundaryError("info.height", "must be a positive integer")
    resolution = float(message.info.resolution)
    if not math.isfinite(resolution) or resolution <= 0:
        raise MapBoundaryError("info.resolution", "must be finite and positive")
    components = _finite_components(message)
    if not all(math.isfinite(component) for component in components):
        raise MapBoundaryError("info.origin", "must contain finite values")
    if not math.isclose(sum(value * value for value in components[3:]), 1.0, abs_tol=1e-6):
        raise MapBoundaryError("info.origin.orientation", "must be normalized")
    expected_cells = message.info.width * message.info.height
    if len(message.data) != expected_cells:
        raise MapBoundaryError("data", "length must equal width times height")
    if any(type(value) is not int or value < -1 or value > 100 for value in message.data):
        raise MapBoundaryError("data", "values must be occupancy integers from -1 through 100")
    load_time_ns = message.info.map_load_time.to_nsec()
    if load_time_ns < 0 or load_time_ns > int(received_at_ns):
        raise MapBoundaryError("info.map_load_time", "must not be negative or in the future")
    data_bytes = bytes(value & 0xFF for value in message.data)
    return MapFingerprint(
        message.header.frame_id,
        message.info.width,
        message.info.height,
        resolution,
        MapOrigin(*components),
        hashlib.sha256(data_bytes).hexdigest(),
        Nanoseconds(load_time_ns),
    )


def arm_map_generation(
    state: MapGenerationState,
    armed_at_ns: Nanoseconds,
    target_identity: MapIdentity,
) -> MapGenerationGuard:
    """Arm a one-shot generation guard after the transition starts."""
    return MapGenerationGuard(state, armed_at_ns, target_identity, True)


def evaluate_map_observation(
    guard: MapGenerationGuard,
    fingerprint: MapFingerprint,
) -> MapGenerationResult:
    """Advance generation only for a new, post-arm target fingerprint."""
    current = guard.state.current_fingerprint
    post_arm = int(fingerprint.map_load_time_ns) > int(guard.armed_at_ns)
    target_fingerprint = fingerprint.identity() == guard.target_identity
    new_fingerprint = current is None or fingerprint != current
    monotonic_load_time = (
        current is None
        or int(fingerprint.map_load_time_ns) > int(current.map_load_time_ns)
    )
    accepted = (
        guard.armed
        and post_arm
        and target_fingerprint
        and new_fingerprint
        and monotonic_load_time
    )
    next_guard = guard
    if accepted:
        state = MapGenerationState(
            MapGeneration(int(guard.state.generation) + 1),
            fingerprint,
        )
        next_guard = MapGenerationGuard(
            state,
            guard.armed_at_ns,
            guard.target_identity,
            False,
        )
    evidence = MapEvidence(
        guard.armed,
        post_arm,
        target_fingerprint,
        new_fingerprint,
        monotonic_load_time,
        accepted,
    )
    return MapGenerationResult(next_guard, evidence)
