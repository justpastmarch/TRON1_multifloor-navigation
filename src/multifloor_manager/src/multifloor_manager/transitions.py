"""Immutable transition policy schema and deterministic pure evaluator."""

from dataclasses import dataclass
import math
from typing import Final, Mapping, Optional, Tuple


FLOOR_TRANSITION_POLICY_ID: Final = "floor_transition_ready"
SUPPORTED_CONDITION_NAMES: Final[Tuple[str, ...]] = (
    "T_FLOOR_CONFIRMED",
    "T_LOCALIZED",
    "T_COSTMAP_READY",
)
MANDATORY_CONDITION_NAMES: Final[Tuple[str, ...]] = SUPPORTED_CONDITION_NAMES


@dataclass(frozen=True)
class TransitionCondition:
    __slots__ = ("name", "enabled", "required")
    name: str
    enabled: bool
    required: bool


@dataclass(frozen=True)
class PredicateObservation:
    __slots__ = ("value", "observed_at_sec", "epoch")
    value: bool
    observed_at_sec: float
    epoch: int


@dataclass(frozen=True)
class InvalidTransitionPolicyError(Exception):
    detail: str

    def __str__(self) -> str:
        return "invalid transition policy: " + self.detail


@dataclass(frozen=True)
class InvalidPredicateObservationError(Exception):
    detail: str

    def __str__(self) -> str:
        return "invalid transition observation: " + self.detail


@dataclass(frozen=True)
class TransitionPolicy:
    __slots__ = (
        "id",
        "enabled",
        "conditions",
        "optional_count",
        "freshness_sec",
        "dwell_sec",
        "timeout_sec",
    )
    id: str
    enabled: bool
    conditions: Tuple[TransitionCondition, ...]
    optional_count: int
    freshness_sec: float
    dwell_sec: float
    timeout_sec: float

    def __post_init__(self) -> None:
        if type(self.id) is not str or not self.id:
            raise InvalidTransitionPolicyError("id must be a nonempty string")
        if type(self.enabled) is not bool:
            raise InvalidTransitionPolicyError("enabled must be a boolean")
        if type(self.optional_count) is not int or self.optional_count < 0:
            raise InvalidTransitionPolicyError("optional_count must be a nonnegative integer")
        if type(self.conditions) is not tuple:
            raise InvalidTransitionPolicyError("conditions must be an immutable tuple")
        if any(type(condition) is not TransitionCondition for condition in self.conditions):
            raise InvalidTransitionPolicyError("conditions must contain TransitionCondition values")
        names = tuple(condition.name for condition in self.conditions)
        if any(type(name) is not str or not name for name in names):
            raise InvalidTransitionPolicyError("condition names must be nonempty strings")
        if len(set(names)) != len(names):
            raise InvalidTransitionPolicyError("condition names must be unique")
        if any(type(condition.enabled) is not bool or type(condition.required) is not bool for condition in self.conditions):
            raise InvalidTransitionPolicyError("condition flags must be booleans")
        optional_count = sum(condition.enabled and not condition.required for condition in self.conditions)
        if self.optional_count > optional_count:
            raise InvalidTransitionPolicyError("optional_count exceeds enabled optional conditions")
        for name, value in (("freshness_sec", self.freshness_sec), ("dwell_sec", self.dwell_sec), ("timeout_sec", self.timeout_sec)):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise InvalidTransitionPolicyError(name + " must be finite and numeric")
        if self.freshness_sec <= 0 or self.timeout_sec <= 0 or self.dwell_sec < 0:
            raise InvalidTransitionPolicyError("temporal values are out of range")
        if self.dwell_sec >= self.timeout_sec:
            raise InvalidTransitionPolicyError("dwell_sec must be strictly less than timeout_sec")


@dataclass(frozen=True)
class TransitionEvidence:
    __slots__ = (
        "conditions", "optional_count", "optional_true_count", "started_at_sec",
        "deadline_sec", "dwell_started_at_sec", "dwell_elapsed_sec", "timed_out", "result",
    )
    conditions: Tuple[Tuple[str, bool, str], ...]
    optional_count: int
    optional_true_count: int
    started_at_sec: float
    deadline_sec: float
    dwell_started_at_sec: Optional[float]
    dwell_elapsed_sec: float
    timed_out: bool
    result: bool


class TransitionManager:
    """Evaluate one epoch's policy using monotonic explicit timestamps."""

    __slots__ = ("_dwell_started_at_sec", "_last_now_sec", "_started_at_sec", "_epoch", "_policy")

    def __init__(self, policy: TransitionPolicy) -> None:
        self._policy = policy
        self._dwell_started_at_sec: Optional[float] = None
        self._last_now_sec: Optional[float] = None
        self._started_at_sec: Optional[float] = None
        self._epoch: Optional[int] = None

    def begin(self, started_at_sec: float, epoch: int) -> None:
        """Start or restart the final-gate timeout for one epoch."""
        self._check_time(started_at_sec, "started_at_sec")
        self._check_epoch(epoch)
        if self._last_now_sec is not None and started_at_sec < self._last_now_sec:
            raise InvalidPredicateObservationError("started_at_sec must not move backwards")
        self._started_at_sec = started_at_sec
        self._last_now_sec = started_at_sec
        self._dwell_started_at_sec = None
        self._epoch = epoch

    def evaluate(
        self,
        observations: Mapping[str, PredicateObservation],
        now_sec: float,
        epoch: int,
    ) -> Tuple[bool, TransitionEvidence]:
        """Return the current policy result and immutable diagnostic evidence."""
        if self._started_at_sec is None or self._epoch is None:
            raise InvalidPredicateObservationError("begin must be called before evaluate")
        self._check_time(now_sec, "now_sec")
        self._check_epoch(epoch)
        if epoch != self._epoch:
            raise InvalidPredicateObservationError("epoch does not match begun epoch")
        if self._last_now_sec is not None and now_sec < self._last_now_sec:
            raise InvalidPredicateObservationError("now_sec must not move backwards")
        conditions = []
        required_pass = True
        optional_true_count = 0
        for condition in self._policy.conditions:
            truth, reason = self._condition_truth(condition, observations.get(condition.name), now_sec, epoch)
            conditions.append((condition.name, truth, reason))
            if condition.enabled and condition.required:
                required_pass = required_pass and truth
            if condition.enabled and not condition.required and truth:
                optional_true_count += 1
        self._last_now_sec = now_sec
        deadline_sec = self._started_at_sec + self._policy.timeout_sec
        timed_out = now_sec >= deadline_sec
        instantaneous = self._policy.enabled and required_pass and optional_true_count >= self._policy.optional_count
        if not instantaneous or timed_out:
            self._dwell_started_at_sec = None
            dwell_elapsed_sec = 0.0
        else:
            if self._dwell_started_at_sec is None:
                self._dwell_started_at_sec = now_sec
            dwell_elapsed_sec = now_sec - self._dwell_started_at_sec
        dwell_complete = (
            self._dwell_started_at_sec is not None
            and now_sec >= self._dwell_started_at_sec + self._policy.dwell_sec
        )
        result = instantaneous and not timed_out and dwell_complete
        evidence = TransitionEvidence(
            tuple(conditions), self._policy.optional_count, optional_true_count,
            self._started_at_sec, deadline_sec, self._dwell_started_at_sec,
            dwell_elapsed_sec, timed_out, result,
        )
        return result, evidence

    @staticmethod
    def _check_time(value: float, name: str) -> None:
        if type(value) not in (int, float) or not math.isfinite(value):
            raise InvalidPredicateObservationError(name + " must be finite and numeric")

    @staticmethod
    def _check_epoch(epoch: int) -> None:
        if type(epoch) is not int:
            raise InvalidPredicateObservationError("epoch must be an integer")

    def _condition_truth(
        self,
        condition: TransitionCondition,
        observation: Optional[PredicateObservation],
        now_sec: float,
        epoch: int,
    ) -> Tuple[bool, str]:
        if not condition.enabled:
            return False, "disabled"
        if observation is None:
            return False, "missing"
        if type(observation) is not PredicateObservation:
            raise InvalidPredicateObservationError(condition.name + " must be a PredicateObservation")
        if type(observation.value) is not bool or type(observation.epoch) is not int:
            raise InvalidPredicateObservationError(condition.name + " has malformed value or epoch")
        self._check_time(observation.observed_at_sec, condition.name + ".observed_at_sec")
        if observation.epoch != epoch:
            return False, "wrong_epoch"
        age = now_sec - observation.observed_at_sec
        if age < 0:
            return False, "future"
        if age > self._policy.freshness_sec:
            return False, "stale"
        if not observation.value:
            return False, "false"
        return True, "true"
