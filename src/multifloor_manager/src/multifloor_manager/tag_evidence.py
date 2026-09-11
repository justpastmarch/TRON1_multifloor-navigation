"""Pure AprilTag boundary parsing and temporal evidence policy."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, NewType, Optional, Protocol, Sequence, Tuple


TagId = NewType("TagId", int)
FloorId = NewType("FloorId", str)
Nanoseconds = NewType("Nanoseconds", int)


class RosTimeLike(Protocol):
    def to_nsec(self) -> int:
        ...


class HeaderLike(Protocol):
    @property
    def frame_id(self) -> str:
        ...

    @property
    def stamp(self) -> RosTimeLike:
        ...


class AprilTagDetectionLike(Protocol):
    @property
    def id(self) -> Sequence[int]:
        ...


class AprilTagDetectionArrayLike(Protocol):
    @property
    def header(self) -> HeaderLike:
        ...

    @property
    def detections(self) -> Sequence[AprilTagDetectionLike]:
        ...


@dataclass(frozen=True)
class TagBoundaryError(Exception):
    __slots__ = ("field", "detail")
    field: str
    detail: str

    def __str__(self) -> str:
        return "AprilTagDetectionArray.{}: {}".format(self.field, self.detail)


@dataclass(frozen=True)
class TagDetectionObservation:
    __slots__ = ("frame_id", "stamp_ns", "received_at_ns", "tag_ids")
    frame_id: str
    stamp_ns: Nanoseconds
    received_at_ns: Nanoseconds
    tag_ids: Tuple[TagId, ...]


@dataclass(frozen=True)
class ActiveTagRoute:
    __slots__ = ("target_floor", "expected_ids")
    target_floor: FloorId
    expected_ids: Tuple[TagId, ...]


@dataclass(frozen=True)
class FloorTagSet:
    __slots__ = ("floor_id", "expected_ids")
    floor_id: FloorId
    expected_ids: Tuple[TagId, ...]


@dataclass(frozen=True)
class TemporalVotePolicy:
    __slots__ = ("required_consecutive", "window_ns", "max_age_ns", "timeout_ns")
    required_consecutive: int
    window_ns: Nanoseconds
    max_age_ns: Nanoseconds
    timeout_ns: Nanoseconds


DEFAULT_TEMPORAL_VOTE_POLICY: Final = TemporalVotePolicy(
    3,
    Nanoseconds(1_000_000_000),
    Nanoseconds(500_000_000),
    Nanoseconds(10_000_000_000),
)


@dataclass(frozen=True)
class TagVoteContext:
    __slots__ = ("active_route", "floor_tag_sets", "policy")
    active_route: ActiveTagRoute
    floor_tag_sets: Tuple[FloorTagSet, ...]
    policy: TemporalVotePolicy


@dataclass(frozen=True)
class TagVoteState:
    __slots__ = (
        "armed_at_ns",
        "candidate_floor",
        "consecutive_count",
        "first_vote_stamp_ns",
        "last_observed_stamp_ns",
    )
    armed_at_ns: Nanoseconds
    candidate_floor: Optional[FloorId]
    consecutive_count: int
    first_vote_stamp_ns: Optional[Nanoseconds]
    last_observed_stamp_ns: Optional[Nanoseconds]


@dataclass(frozen=True)
class DebugPredicate:
    __slots__ = ("name", "value")
    name: str
    value: bool


@dataclass(frozen=True)
class TagEvidence:
    __slots__ = (
        "known_tags",
        "fresh",
        "monotonic_stamp",
        "post_arm",
        "active_route",
        "target_floor",
        "within_timeout",
        "timed_out",
        "consecutive_vote",
        "wrong_floor_vote",
        "accepted",
    )
    known_tags: bool
    fresh: bool
    monotonic_stamp: bool
    post_arm: bool
    active_route: bool
    target_floor: bool
    within_timeout: bool
    timed_out: bool
    consecutive_vote: bool
    wrong_floor_vote: bool
    accepted: bool

    def predicates(self) -> Tuple[DebugPredicate, ...]:
        """Return stable Boolean seams for future std_msgs/Bool publishers."""
        return (
            DebugPredicate("tag_known", self.known_tags),
            DebugPredicate("tag_fresh", self.fresh),
            DebugPredicate("tag_stamp_monotonic", self.monotonic_stamp),
            DebugPredicate("tag_post_arm", self.post_arm),
            DebugPredicate("tag_active_route", self.active_route),
            DebugPredicate("tag_target_floor", self.target_floor),
            DebugPredicate("tag_within_timeout", self.within_timeout),
            DebugPredicate("tag_consecutive_vote", self.consecutive_vote),
            DebugPredicate("tag_wrong_floor_clear", not self.wrong_floor_vote),
            DebugPredicate("tag_accepted", self.accepted),
        )


@dataclass(frozen=True)
class TagVoteResult:
    __slots__ = ("state", "evidence")
    state: TagVoteState
    evidence: TagEvidence


def parse_apriltag_detection_array(
    message: AprilTagDetectionArrayLike,
    received_at_ns: Nanoseconds,
) -> TagDetectionObservation:
    """Parse an apriltag_ros-compatible message at the trust boundary."""
    if not message.header.frame_id:
        raise TagBoundaryError("header.frame_id", "must not be empty")
    stamp_ns = message.header.stamp.to_nsec()
    if stamp_ns < 0:
        raise TagBoundaryError("header.stamp", "must not be negative")
    if stamp_ns > int(received_at_ns):
        raise TagBoundaryError("header.stamp", "must not be in the future")
    parsed_ids = []
    for index, detection in enumerate(message.detections):
        if not detection.id:
            raise TagBoundaryError("detections[{}].id".format(index), "must not be empty")
        for identifier in detection.id:
            if type(identifier) is not int or identifier <= 0:
                raise TagBoundaryError(
                    "detections[{}].id".format(index),
                    "must contain positive integers",
                )
            parsed_ids.append(TagId(identifier))
    return TagDetectionObservation(
        message.header.frame_id,
        Nanoseconds(stamp_ns),
        received_at_ns,
        tuple(sorted(set(parsed_ids))),
    )


def arm_tag_vote(armed_at_ns: Nanoseconds) -> TagVoteState:
    """Create an empty temporal vote that cannot include pre-arm observations."""
    return TagVoteState(armed_at_ns, None, 0, None, None)


def _matching_floor(
    tag_ids: Tuple[TagId, ...],
    floor_tag_sets: Tuple[FloorTagSet, ...],
) -> Optional[FloorId]:
    matches = tuple(
        configured.floor_id
        for configured in floor_tag_sets
        if tag_ids and set(tag_ids).issubset(configured.expected_ids)
    )
    return matches[0] if len(set(matches)) == 1 else None


def evaluate_tag_observation(
    state: TagVoteState,
    observation: TagDetectionObservation,
    context: TagVoteContext,
) -> TagVoteResult:
    """Reduce one observation into explicit evidence and the next vote state."""
    all_known_ids = {
        identifier
        for configured in context.floor_tag_sets
        for identifier in configured.expected_ids
    }
    known_tags = bool(observation.tag_ids) and set(observation.tag_ids).issubset(all_known_ids)
    age_ns = int(observation.received_at_ns) - int(observation.stamp_ns)
    fresh = 0 <= age_ns <= int(context.policy.max_age_ns)
    elapsed_ns = int(observation.received_at_ns) - int(state.armed_at_ns)
    within_timeout = 0 <= elapsed_ns <= int(context.policy.timeout_ns)
    monotonic_stamp = (
        state.last_observed_stamp_ns is None
        or int(observation.stamp_ns) > int(state.last_observed_stamp_ns)
    )
    post_arm = int(observation.stamp_ns) > int(state.armed_at_ns)
    active_route = bool(observation.tag_ids) and set(observation.tag_ids).issubset(
        context.active_route.expected_ids
    )
    observed_floor = _matching_floor(observation.tag_ids, context.floor_tag_sets)
    target_floor = observed_floor == context.active_route.target_floor
    vote_floor = observed_floor if active_route or not target_floor else None
    valid_vote = (
        known_tags
        and fresh
        and within_timeout
        and monotonic_stamp
        and post_arm
        and vote_floor is not None
    )

    first_stamp = observation.stamp_ns
    count = 0
    if valid_vote:
        continues = (
            state.candidate_floor == vote_floor
            and state.first_vote_stamp_ns is not None
            and int(observation.stamp_ns) - int(state.first_vote_stamp_ns)
            <= int(context.policy.window_ns)
        )
        count = state.consecutive_count + 1 if continues else 1
        first_stamp = state.first_vote_stamp_ns if continues else observation.stamp_ns

    next_state = TagVoteState(
        state.armed_at_ns,
        vote_floor if valid_vote else None,
        count,
        first_stamp if valid_vote else None,
        observation.stamp_ns if monotonic_stamp else state.last_observed_stamp_ns,
    )
    consecutive_vote = count >= context.policy.required_consecutive
    wrong_floor_vote = consecutive_vote and vote_floor != context.active_route.target_floor
    accepted = consecutive_vote and active_route and target_floor and not wrong_floor_vote
    evidence = TagEvidence(
        known_tags,
        fresh,
        monotonic_stamp,
        post_arm,
        active_route,
        target_floor,
        within_timeout,
        not within_timeout,
        consecutive_vote,
        wrong_floor_vote,
        accepted,
    )
    return TagVoteResult(next_state, evidence)
