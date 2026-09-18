"""ROS subscriptions and synchronized evidence for one floor transaction."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

import rospy
from std_msgs.msg import Bool
import tf2_ros

from multifloor_manager.map_evidence import (
    MapGeneration,
    MapGenerationGuard,
    MapGenerationState,
    MapIdentity,
    Nanoseconds as MapNanoseconds,
    arm_map_generation,
)
from multifloor_manager.msg import FloorState
from multifloor_manager.readiness import (
    DEFAULT_READINESS_POLICY,
    LocalizationState,
    Nanoseconds,
    arm_localization,
    localization_ready,
)
from multifloor_manager.tag_evidence import (
    ActiveTagRoute,
    DEFAULT_TEMPORAL_VOTE_POLICY,
    FloorId,
    FloorTagSet,
    Nanoseconds as TagNanoseconds,
    TagId,
    TagVoteContext,
    TagVoteState,
    arm_tag_vote,
)
from multifloor_manager.transitions import (
    InvalidPredicateObservationError,
    PredicateObservation,
    TransitionEvidence,
    TransitionManager,
    TransitionPolicy,
)
from multifloor_manager.ros_callbacks import RosEvidenceCallbacks


@dataclass(frozen=True)
class EpochToken:
    """Immutable capability for one armed transition epoch."""

    __slots__ = ("epoch",)
    epoch: int


class RosEvidenceRuntime(RosEvidenceCallbacks):
    """Mutable callback accumulator guarded by one condition variable."""

    def __init__(
        self,
        initial_floor: str,
        initial_identity: MapIdentity,
        floor_tag_sets: Tuple[FloorTagSet, ...],
        tf_buffer: tf2_ros.Buffer,
        policy: TransitionPolicy,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.condition = threading.Condition()
        self.epoch = 0
        self.active_epoch: Optional[int] = None
        self.floor_state_publisher = rospy.Publisher("/multifloor/floor_state", FloorState, queue_size=1, latch=True)
        self.initial_floor = initial_floor
        self.initial_identity = initial_identity
        self.floor_tag_sets = floor_tag_sets
        self.tf_buffer = tf_buffer
        self.policy = policy
        self.clock = clock
        self.policy_manager: Optional[TransitionManager] = None
        self.policy_observations: Dict[str, PredicateObservation] = {}
        self.policy_result: Optional[bool] = None
        self.map_state = MapGenerationState(MapGeneration(0), None)
        self.map_guard: Optional[MapGenerationGuard] = None
        self.tag_state: Optional[TagVoteState] = None
        self.tag_context: Optional[TagVoteContext] = None
        self.tag_accepted = False
        self.wrong_floor_tag = False
        self.localization: Optional[LocalizationState] = None
        self.costmap_sequence = 0
        self.costmap_identity: Optional[MapIdentity] = None
        self.debug_publishers: Dict[str, rospy.Publisher] = {}
        self.state = FloorState.UNKNOWN
        self.current_floor = ""
        self.state_detail = "waiting for initial map"
        self.publish_floor(FloorState.UNKNOWN, "waiting for initial map")

    def publish_floor(self, state: int, detail: str, floor_id: str = "") -> None:
        message = FloorState()
        message.header.stamp = rospy.Time.now()
        message.floor_id = floor_id
        message.map_generation = int(self.map_state.generation)
        message.state = state
        message.detail = detail
        self.state = state
        self.current_floor = floor_id
        self.state_detail = detail
        self.floor_state_publisher.publish(message)

    def publish_heartbeat(self, _event: rospy.timer.TimerEvent) -> None:
        """Refresh the current state without changing transition ownership."""
        with self.condition:
            self.publish_floor(self.state, self.state_detail, self.current_floor)

    def _publish_predicates(self, predicates) -> None:
        for predicate in predicates:
            publisher = self.debug_publishers.get(predicate.name)
            if publisher is None:
                publisher = rospy.Publisher(
                    "/multifloor/debug/{}".format(predicate.name),
                    Bool,
                    queue_size=1,
                    latch=True,
                )
                self.debug_publishers[predicate.name] = publisher
            publisher.publish(Bool(data=predicate.value))

    def arm_transition(self, target_floor: str, expected_ids: Tuple[int, ...], target: MapIdentity) -> EpochToken:
        now = rospy.Time.now().to_nsec()
        with self.condition:
            self.epoch += 1
            self.active_epoch = self.epoch
            token = EpochToken(self.epoch)
            self.policy_manager = TransitionManager(self.policy)
            self.policy_observations = {}
            self.policy_result = None
            self.map_guard = arm_map_generation(
                self.map_state,
                MapNanoseconds(now),
                target,
            )
            self.tag_state = arm_tag_vote(TagNanoseconds(now))
            self.tag_context = TagVoteContext(
                ActiveTagRoute(FloorId(target_floor), tuple(TagId(value) for value in expected_ids)),
                self.floor_tag_sets,
                DEFAULT_TEMPORAL_VOTE_POLICY,
            )
            self.tag_accepted = False
            self.wrong_floor_tag = False
            self.localization = None
            self.publish_floor(FloorState.TRANSITIONING, "floor transition active")
            self.condition.notify_all()
            return token

    def _check_policy_token(self, token: EpochToken) -> None:
        if type(token) is not EpochToken or self.active_epoch != token.epoch:
            raise InvalidPredicateObservationError("transition epoch token is stale or mismatched")

    def record_policy_observation(
        self,
        token: EpochToken,
        name: str,
        observation: PredicateObservation,
    ) -> None:
        """Record one policy observation only for the active epoch."""
        with self.condition:
            self._check_policy_token(token)
            if self.policy_manager is None:
                raise InvalidPredicateObservationError("transition policy is not armed")
            self.policy_observations[name] = observation
            self.condition.notify_all()

    def begin_policy(self, token: EpochToken) -> None:
        """Start the policy timeout at final-gate entry."""
        with self.condition:
            self._check_policy_token(token)
            manager = self.policy_manager
            if manager is None:
                raise InvalidPredicateObservationError("transition policy is not armed")
            manager.begin(self.clock(), token.epoch)

    def evaluate_policy(self, token: EpochToken) -> Tuple[bool, TransitionEvidence]:
        """Evaluate the active epoch's observations at the monotonic clock."""
        with self.condition:
            self._check_policy_token(token)
            manager = self.policy_manager
            if manager is None:
                raise InvalidPredicateObservationError("transition policy is not armed")
            result, evidence = manager.evaluate(self.policy_observations, self.clock(), token.epoch)
            self.policy_result = result
            return result, evidence

    def commit_ready(self, token: EpochToken) -> bool:
        """Accept READY only after this epoch has produced a successful result."""
        with self.condition:
            self._check_policy_token(token)
            return self.policy_manager is not None and self.policy_result is True

    def arm_localization(self, stamp_ns: int) -> None:
        with self.condition:
            self.localization = arm_localization(Nanoseconds(stamp_ns))
            self.condition.notify_all()

    def has_pose_newer_than(self, stamp_ns: int) -> bool:
        """Report whether AMCL produced a distinct accepted-order pose stamp."""
        return (
            self.localization is not None
            and self.localization.pose_stamp_ns is not None
            and int(self.localization.pose_stamp_ns) > stamp_ns
        )

    def pose_high_water_ns(self, fallback_ns: int) -> int:
        """Read the current epoch's AMCL stamp without lowering the caller's fence."""
        with self.condition:
            if self.active_epoch is None or self.localization is None:
                return fallback_ns
            stamp = self.localization.pose_stamp_ns
            return fallback_ns if stamp is None else max(fallback_ns, int(stamp))

    def finish_transition(self, state: int, detail: str, floor_id: str = "") -> None:
        """Latch terminal floor state and make every transaction callback inert."""
        with self.condition:
            self.epoch += 1
            self.active_epoch = None
            self.map_guard = None
            self.tag_state = None
            self.tag_context = None
            self.localization = None
            self.policy_manager = None
            self.policy_observations = {}
            self.policy_result = None
            self.tag_accepted = False
            self.wrong_floor_tag = False
            self.publish_floor(state, detail, floor_id)
            self.condition.notify_all()

    def localization_evidence(self):
        if self.localization is None:
            return None
        evidence = localization_ready(
            self.localization,
            Nanoseconds(rospy.Time.now().to_nsec()),
            DEFAULT_READINESS_POLICY,
        )
        self._publish_predicates(evidence.predicates())
        return evidence
