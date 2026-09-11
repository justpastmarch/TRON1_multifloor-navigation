from pathlib import Path
import sys
import unittest
from typing import Dict


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "multifloor_manager" / "src"))

from multifloor_manager.transitions import (  # noqa: E402
    InvalidPredicateObservationError,
    InvalidTransitionPolicyError,
    PredicateObservation,
    TransitionCondition,
    TransitionManager,
    TransitionPolicy,
)


class TransitionManagerTest(unittest.TestCase):
    def policy(self, *, optional_count: int = 0, dwell: float = 2.0, timeout: float = 8.0) -> TransitionPolicy:
        return TransitionPolicy(
            id="floor_transition_ready",
            enabled=True,
            conditions=(
                TransitionCondition("required", True, True),
                TransitionCondition("optional", True, False),
            ),
            optional_count=optional_count,
            freshness_sec=1.0,
            dwell_sec=dwell,
            timeout_sec=timeout,
        )

    def observations(self, now: float, *, epoch: int = 4, optional: bool = True) -> Dict[str, PredicateObservation]:
        return {
            "required": PredicateObservation(True, now, epoch),
            "optional": PredicateObservation(optional, now, epoch),
        }

    def test_baseline_schema_objects_remain_immutable(self) -> None:
        # Given: the schema objects owned by the destination module.
        condition = TransitionCondition("required", True, True)
        # When/Then: construction succeeds and mutation is rejected.
        self.assertEqual(condition.name, "required")
        with self.assertRaises(AttributeError):
            setattr(condition, "name", "changed")

    def test_evaluate_requires_begin(self) -> None:
        # Given: a manager that has not entered its final gate.
        manager = TransitionManager(self.policy())
        # When/Then: evaluation is rejected rather than starting the timeout.
        with self.assertRaises(InvalidPredicateObservationError):
            manager.evaluate(self.observations(0.0), now_sec=0.0, epoch=4)

    def test_evaluate_rejects_epoch_different_from_begun_epoch(self) -> None:
        # Given: a final gate begun for epoch four.
        manager = TransitionManager(self.policy(dwell=0.0))
        manager.begin(0.0, epoch=4)
        observations = self.observations(0.0, epoch=5)
        # When/Then: a caller cannot switch the evaluation to epoch five.
        with self.assertRaises(InvalidPredicateObservationError):
            manager.evaluate(observations, now_sec=0.0, epoch=5)

    def test_policy_rejects_mutable_conditions_container(self) -> None:
        # Given/When/Then: the immutable policy contract rejects caller-owned lists.
        with self.assertRaises(InvalidTransitionPolicyError):
            TransitionPolicy(
                id="mutable",
                enabled=True,
                conditions=[TransitionCondition("required", True, True)],
                optional_count=0,
                freshness_sec=1.0,
                dwell_sec=0.0,
                timeout_sec=2.0,
            )

    def test_optional_count_zero_and_one_are_deterministic(self) -> None:
        # Given: generic policies exercising both approved quorum values.
        zero = TransitionManager(self.policy(optional_count=0, dwell=0.0))
        one = TransitionManager(self.policy(optional_count=1, dwell=0.0))
        zero.begin(10.0, 4)
        one.begin(10.0, 4)
        # When: required evidence is fresh and optional evidence is false.
        zero_result, zero_evidence = zero.evaluate(self.observations(10.0, optional=False), 10.0, 4)
        one_result, one_evidence = one.evaluate(self.observations(10.0, optional=False), 10.0, 4)
        # Then: only the quorum-satisfied policy succeeds.
        self.assertTrue(zero_result)
        self.assertFalse(one_result)
        self.assertEqual(zero_evidence.optional_true_count, 0)
        self.assertEqual(one_evidence.optional_count, 1)

    def test_freshness_boundary_is_valid_and_future_or_stale_is_false(self) -> None:
        # Given: a begun policy with one-second freshness.
        manager = TransitionManager(self.policy(dwell=0.0))
        manager.begin(10.0, 4)
        # When: observations are exactly fresh, then stale, then future-dated.
        boundary, boundary_evidence = manager.evaluate(self.observations(9.0), 10.0, 4)
        stale, stale_evidence = manager.evaluate(self.observations(8.999), 10.0, 4)
        future, future_evidence = manager.evaluate(self.observations(11.0), 10.0, 4)
        # Then: only the inclusive boundary is true and reasons are explicit.
        self.assertTrue(boundary)
        self.assertFalse(stale)
        self.assertFalse(future)
        self.assertEqual(boundary_evidence.conditions[0], ("required", True, "true"))
        self.assertEqual(stale_evidence.conditions[0][2], "stale")
        self.assertEqual(future_evidence.conditions[0][2], "future")

    def test_missing_false_and_wrong_epoch_reset_dwell(self) -> None:
        # Given: nearly complete continuous truth.
        manager = TransitionManager(self.policy())
        manager.begin(0.0, 4)
        manager.evaluate(self.observations(0.0), 0.0, 4)
        manager.evaluate(self.observations(1.9), 1.9, 4)
        # When: each invalid truth source is evaluated.
        missing, missing_evidence = manager.evaluate({}, 2.0, 4)
        wrong, wrong_evidence = manager.evaluate(self.observations(2.1, epoch=5), 2.1, 4)
        false, false_evidence = manager.evaluate(self.observations(2.2, optional=False), 2.2, 4)
        # Then: every interruption is false and starts no inherited dwell.
        self.assertFalse(missing)
        self.assertFalse(wrong)
        self.assertFalse(false)
        self.assertEqual(missing_evidence.dwell_started_at_sec, None)
        self.assertEqual(wrong_evidence.conditions[0][2], "wrong_epoch")
        self.assertEqual(false_evidence.conditions[1][2], "false")

    def test_malformed_entry_is_domain_error_and_does_not_commit_time(self) -> None:
        # Given: a manager with no committed evaluation after begin.
        manager = TransitionManager(self.policy())
        manager.begin(0.0, 4)
        # When: an invalid entry is presented at time two.
        with self.assertRaises(InvalidPredicateObservationError):
            manager.evaluate({"required": object()}, 2.0, 4)
        # Then: a valid earlier time remains legal because the failed call was atomic.
        result, evidence = manager.evaluate(self.observations(1.0), 1.0, 4)
        self.assertFalse(result)
        self.assertEqual(evidence.dwell_started_at_sec, 1.0)

    def test_decimal_dwell_uses_absolute_threshold(self) -> None:
        # Given: a dwell interval beginning at a decimal timestamp.
        manager = TransitionManager(self.policy(dwell=2.0, timeout=5.0))
        manager.begin(0.0, 4)
        manager.evaluate(self.observations(2.1), 2.1, 4)
        # When: the absolute dwell threshold is reached at four-point-one.
        result, evidence = manager.evaluate(self.observations(4.1), 4.1, 4)
        # Then: floating subtraction noise cannot deny completed dwell.
        self.assertTrue(result)
        self.assertEqual(evidence.dwell_started_at_sec, 2.1)

    def test_disabled_conditions_do_not_count(self) -> None:
        # Given: a disabled required and optional condition.
        policy = TransitionPolicy(
            id="synthetic",
            enabled=True,
            conditions=(TransitionCondition("disabled", False, True), TransitionCondition("optional", True, False)),
            optional_count=1,
            freshness_sec=1.0,
            dwell_sec=0.0,
            timeout_sec=2.0,
        )
        manager = TransitionManager(policy)
        manager.begin(0.0, 1)
        # When/Then: disabled inputs cannot make the policy true or satisfy quorum.
        result, evidence = manager.evaluate({}, 0.0, 1)
        self.assertFalse(result)
        self.assertEqual(evidence.optional_true_count, 0)
        self.assertEqual(evidence.conditions, (("disabled", False, "disabled"), ("optional", False, "missing")))

    def test_dwell_requires_strictly_before_timeout(self) -> None:
        # Given: a dwell shorter than the final-gate timeout.
        manager = TransitionManager(self.policy(dwell=2.0, timeout=3.0))
        manager.begin(0.0, 4)
        manager.evaluate(self.observations(0.0), 0.0, 4)
        # When: truth completes dwell just before the deadline.
        result, evidence = manager.evaluate(self.observations(2.9), 2.9, 4)
        # Then: success includes elapsed/deadline evidence and is not timed out.
        self.assertTrue(result)
        self.assertFalse(evidence.timed_out)
        self.assertEqual(evidence.deadline_sec, 3.0)

    def test_false_required_observation_requires_a_full_new_dwell(self) -> None:
        # Given: a required condition that has nearly completed its dwell.
        manager = TransitionManager(self.policy(dwell=2.0, timeout=8.0))
        manager.begin(0.0, 4)
        passing = self.observations(0.0)
        manager.evaluate(passing, 0.0, 4)
        manager.evaluate(self.observations(1.9), 1.9, 4)
        # When: the required condition becomes false and then returns.
        manager.evaluate({"required": PredicateObservation(False, 2.0, 4), "optional": PredicateObservation(True, 2.0, 4)}, 2.0, 4)
        resumed, resumed_evidence = manager.evaluate(self.observations(3.9), 3.9, 4)
        early, early_evidence = manager.evaluate(self.observations(5.8), 5.8, 4)
        complete, complete_evidence = manager.evaluate(self.observations(5.9), 5.9, 4)
        # Then: the interrupted interval contributes nothing to the new dwell.
        self.assertFalse(resumed)
        self.assertEqual(resumed_evidence.dwell_started_at_sec, 3.9)
        self.assertFalse(early)
        self.assertEqual(early_evidence.dwell_started_at_sec, 3.9)
        self.assertTrue(complete)
        self.assertEqual(complete_evidence.dwell_started_at_sec, 3.9)

    def test_exact_and_late_timeout_are_terminal_false(self) -> None:
        # Given: a manager whose deadline has not yet been evaluated.
        manager = TransitionManager(self.policy(dwell=2.0, timeout=3.0))
        manager.begin(0.0, 4)
        manager.evaluate(self.observations(0.0), 0.0, 4)
        # When: the exact deadline and then a later time are evaluated.
        exact, exact_evidence = manager.evaluate(self.observations(3.0), 3.0, 4)
        late, late_evidence = manager.evaluate(self.observations(4.0), 4.0, 4)
        # Then: timeout is terminal and cannot be misleadingly reported as success.
        self.assertFalse(exact)
        self.assertTrue(exact_evidence.timed_out)
        self.assertFalse(late)
        self.assertTrue(late_evidence.timed_out)

    def test_backward_and_nonfinite_clocks_are_rejected(self) -> None:
        # Given: a begun manager at a finite monotonic time.
        manager = TransitionManager(self.policy(dwell=0.0))
        manager.begin(1.0, 4)
        # When/Then: backward and non-finite values are rejected without state drift.
        with self.assertRaises(InvalidPredicateObservationError):
            manager.evaluate(self.observations(1.0), 0.5, 4)
        with self.assertRaises(InvalidPredicateObservationError):
            manager.evaluate(self.observations(1.0), float("nan"), 4)
        with self.assertRaises(InvalidPredicateObservationError):
            manager.evaluate(self.observations(1.0), True, 4)
        with self.assertRaises(InvalidPredicateObservationError):
            manager.begin(float("inf"), 4)

    def test_malformed_observation_time_is_rejected(self) -> None:
        # Given: a begun manager and a non-finite observation timestamp.
        manager = TransitionManager(self.policy(dwell=0.0))
        manager.begin(0.0, 4)
        malformed = {"required": PredicateObservation(True, float("nan"), 4)}
        # When/Then: malformed boundary input is rejected.
        with self.assertRaises(InvalidPredicateObservationError):
            manager.evaluate(malformed, 0.0, 4)

    def test_policy_defensively_rejects_equal_dwell_and_timeout(self) -> None:
        # Given/When/Then: synthetic direct construction cannot bypass strict temporal invariants.
        with self.assertRaises(InvalidTransitionPolicyError):
            self.policy(dwell=3.0, timeout=3.0)


if __name__ == "__main__":
    unittest.main()
