#!/usr/bin/env python3
"""Lock final-gate sampling and cancellation decision semantics."""

from __future__ import annotations

from types import SimpleNamespace
import threading
import unittest

from multifloor_manager.ros_node import MultifloorManagerNode
from multifloor_manager.ros_runtime import EpochToken
from multifloor_manager.transitions import (
    PredicateObservation,
    TransitionCondition,
    TransitionManager,
    TransitionPolicy,
)


class FakeActionServer:
    def __init__(self, preempt_after: int | None = None) -> None:
        self.checks = 0
        self.preempt_after = preempt_after

    def is_preempt_requested(self) -> bool:
        self.checks += 1
        return self.preempt_after is not None and self.checks >= self.preempt_after


class FakeRuntime:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.policy = SimpleNamespace(timeout_sec=1.0)
        self.observations: dict[str, PredicateObservation] = {}
        self.begin_token = None
        self.evaluate_count = 0
        self.commit_count = 0

    def begin_policy(self, token: EpochToken) -> None:
        self.begin_token = token

    @staticmethod
    def localization_evidence() -> SimpleNamespace:
        return SimpleNamespace(ready=True)

    def record_policy_observation(
        self,
        _token: EpochToken,
        name: str,
        observation: PredicateObservation,
    ) -> None:
        self.observations[name] = observation

    def evaluate_policy(self, _token: EpochToken) -> tuple[bool, None]:
        self.evaluate_count += 1
        return True, None

    def commit_ready(self, _token: EpochToken) -> bool:
        self.commit_count += 1
        return True


class PolicyFinalGateTest(unittest.TestCase):
    @staticmethod
    def _level_policy() -> TransitionPolicy:
        return TransitionPolicy(
            "floor_transition_ready",
            True,
            (
                TransitionCondition("T_LOCALIZED", True, True),
                TransitionCondition("T_COSTMAP_READY", True, True),
            ),
            0,
            1.0,
            0.2,
            2.0,
        )

    @staticmethod
    def _level_observations(
        now: float,
        *,
        localized: bool = True,
        costmap: bool = True,
    ) -> dict[str, PredicateObservation]:
        return {
            "T_LOCALIZED": PredicateObservation(localized, now, 4),
            "T_COSTMAP_READY": PredicateObservation(costmap, now, 4),
        }

    def _node(self, server: FakeActionServer) -> tuple[MultifloorManagerNode, FakeRuntime, list[str]]:
        runtime = FakeRuntime()
        feedback: list[str] = []
        node = MultifloorManagerNode.__new__(MultifloorManagerNode)
        node.runtime = runtime
        node.server = server
        node._feedback = lambda phase, _detail: feedback.append(phase)
        node._target_metadata_matches = lambda _target, _marker: True
        return node, runtime, feedback

    def test_success_samples_live_levels_and_commits_after_second_check(self) -> None:
        # Given: a final gate whose live localization and costmap levels are true.
        node, runtime, feedback = self._node(FakeActionServer())
        token = EpochToken(4)
        # When: the gate evaluates and reaches the successful second preemption check.
        result = node._wait_for_policy_ready(token, object(), 0)
        # Then: both live observations are recorded and the epoch commit occurs once.
        self.assertEqual(result, (True, ""))
        self.assertEqual(feedback, ["POLICY_READY"])
        self.assertEqual(set(runtime.observations), {"T_LOCALIZED", "T_COSTMAP_READY"})
        self.assertEqual(runtime.begin_token, token)
        self.assertEqual(runtime.commit_count, 1)

    def test_preemption_at_second_check_cannot_commit_ready(self) -> None:
        # Given: the action server requests cancellation only after a true evaluation.
        node, runtime, _feedback = self._node(FakeActionServer(preempt_after=2))
        # When: the final gate reaches its cancellation decision point.
        result = node._wait_for_policy_ready(EpochToken(4), object(), 0)
        # Then: cancellation wins and READY commit is never called.
        self.assertEqual(result, (False, "transition cancelled"))
        self.assertEqual(runtime.evaluate_count, 1)
        self.assertEqual(runtime.commit_count, 0)

    def test_false_localization_requires_a_new_dwell(self) -> None:
        # Given: live levels that were true for part of the policy dwell.
        manager = TransitionManager(self._level_policy())
        manager.begin(0.0, 4)
        manager.evaluate(self._level_observations(0.0), 0.0, 4)
        manager.evaluate(self._level_observations(0.15), 0.15, 4)
        # When: localization becomes false and then returns.
        interrupted, _ = manager.evaluate(
            self._level_observations(0.16, localized=False),
            0.16,
            4,
        )
        resumed, _ = manager.evaluate(self._level_observations(0.30), 0.30, 4)
        complete, _ = manager.evaluate(self._level_observations(0.51), 0.51, 4)
        # Then: the old dwell is discarded and only the new dwell can succeed.
        self.assertFalse(interrupted)
        self.assertFalse(resumed)
        self.assertTrue(complete)

    def test_wrong_costmap_requires_a_new_dwell(self) -> None:
        # Given: a policy whose target costmap was initially valid.
        manager = TransitionManager(self._level_policy())
        manager.begin(0.0, 4)
        manager.evaluate(self._level_observations(0.0), 0.0, 4)
        manager.evaluate(self._level_observations(0.15), 0.15, 4)
        # When: a later costmap sample is wrong before the target returns.
        interrupted, _ = manager.evaluate(
            self._level_observations(0.16, costmap=False),
            0.16,
            4,
        )
        resumed, _ = manager.evaluate(self._level_observations(0.30), 0.30, 4)
        complete, _ = manager.evaluate(self._level_observations(0.51), 0.51, 4)
        # Then: the previous target identity cannot carry the dwell across the false sample.
        self.assertFalse(interrupted)
        self.assertFalse(resumed)
        self.assertTrue(complete)


if __name__ == "__main__":
    unittest.main()
