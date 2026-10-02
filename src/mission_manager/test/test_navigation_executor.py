from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "mission_manager" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from actionlib_msgs.msg import GoalStatus  # noqa: E402
from genpy import Time  # noqa: E402
from multifloor_manager.msg import FloorState  # noqa: E402
from stair_supervisor.msg import SupervisorState  # noqa: E402

from mission_manager.configuration import Location  # noqa: E402
from mission_manager.navigation_executor import (  # noqa: E402
    HandoffSafetyError,
    InvalidNavigationGoalError,
    NavigationExecutor,
    NavigationOutcome,
    NavigationReadinessError,
    NavigationRequest,
    NavigationRuntime,
    RosNavigationSources,
)
from navigation_executor_fakes import FakeActionClient, FakeCostmaps, FakeHandoffBarrier  # noqa: E402


class NavigationExecutorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.floor_state = FloorState(floor_id="3F", map_generation=7, state=FloorState.READY)
        self.supervisor_state = SupervisorState(state=SupervisorState.NAV)
        self.location = Location("stair_entry", "3F", "stair_entry", 1.25, -0.5, math.pi / 2)
        self.costmaps = FakeCostmaps()
        self.barrier = FakeHandoffBarrier()

    def executor(
        self,
        states: tuple[int, ...],
        floor_state: Callable[[], FloorState] | None = None,
    ) -> tuple[NavigationExecutor, FakeActionClient]:
        client = FakeActionClient(states)
        runtime = NavigationRuntime(
            action_client=client,
            clear_costmaps=self.costmaps.clear,
            floor_state=floor_state if floor_state is not None else lambda: self.floor_state,
            supervisor_state=lambda: self.supervisor_state,
            handoff_barrier=self.barrier,
            now=lambda: Time(123, 456),
        )
        return NavigationExecutor(runtime), client

    def request(self) -> NavigationRequest:
        return NavigationRequest(self.location, expected_generation=7)

    def test_named_map_goal_succeeds_only_after_terminal_status(self) -> None:
        # Given: every readiness gate and a queued terminal success.
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        # When: the Location DB destination is executed.
        result = executor.execute(self.request())
        # Then: the correlated terminal result identifies one normalized map goal.
        self.assertEqual(result.outcome, NavigationOutcome.SUCCEEDED)
        self.assertEqual((result.location_id, result.attempts, result.status), ("stair_entry", 1, GoalStatus.SUCCEEDED))
        goal = client.goals[0].target_pose
        self.assertEqual((goal.header.frame_id, goal.header.stamp), ("map", Time(123, 456)))
        self.assertAlmostEqual(goal.pose.orientation.z ** 2 + goal.pose.orientation.w ** 2, 1.0)

    @mock.patch("mission_manager.navigation_executor.rospy.get_param", side_effect=lambda name, default: default)
    def test_ros_factory_uses_only_move_base_action_and_clear_service(self, _params) -> None:
        # Given: patched ROS factories and no initialized production node.
        client = FakeActionClient((GoalStatus.SUCCEEDED,))
        sources = RosNavigationSources(lambda: self.floor_state, lambda: self.supervisor_state, self.barrier)
        with mock.patch("mission_manager.navigation_executor.actionlib.SimpleActionClient", return_value=client) as action_factory:
            with mock.patch("mission_manager.navigation_executor.rospy.ServiceProxy") as service_factory:
                with mock.patch("mission_manager.navigation_executor.rospy.init_node") as init_node:
                    # When: the non-node ROS adapter is constructed.
                    executor = NavigationExecutor.create_ros(sources)
        # Then: only the managed action/service endpoints exist and no node is initialized.
        self.assertFalse(executor.has_active_goal)
        self.assertEqual(action_factory.call_args.args[0], "/move_base")
        self.assertEqual(service_factory.call_args.args[0], "/move_base/clear_costmaps")
        init_node.assert_not_called()

    def test_nonfinite_yaw_is_rejected_before_goal_send(self) -> None:
        # Given: a malformed Location that cannot form a finite quaternion.
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        request = NavigationRequest(replace(self.location, yaw=float("nan")), expected_generation=7)
        # When/Then: parsing rejects it without touching move_base.
        with self.assertRaises(InvalidNavigationGoalError):
            executor.execute(request)
        self.assertEqual(client.goals, [])

    def test_action_rejection_is_navigation_failure_without_retry(self) -> None:
        # Given: move_base rejects an otherwise valid goal.
        executor, client = self.executor((GoalStatus.REJECTED,))
        # When: the rejection becomes terminal.
        result = executor.execute(self.request())
        # Then: publication is not success and no retry is permitted.
        self.assertEqual(result.outcome, NavigationOutcome.NAVIGATION_FAILED)
        self.assertEqual((result.attempts, len(client.goals), self.costmaps.calls), (1, 1, 0))

    def test_aborted_goal_clears_costmaps_then_retries_once(self) -> None:
        # Given: one terminal abort followed by success.
        executor, client = self.executor((GoalStatus.ABORTED, GoalStatus.SUCCEEDED))
        # When: the segment is executed.
        result = executor.execute(self.request())
        # Then: one clear separates exactly two terminal attempts.
        self.assertEqual(result.outcome, NavigationOutcome.SUCCEEDED)
        self.assertEqual((result.attempts, len(client.goals), self.costmaps.calls), (2, 2, 1))

    def test_repeated_abort_fails_after_exactly_one_retry(self) -> None:
        # Given: both allowed attempts terminate ABORTED.
        executor, client = self.executor((GoalStatus.ABORTED, GoalStatus.ABORTED))
        # When: the retry also fails.
        result = executor.execute(self.request())
        # Then: NAVIGATION_FAILED leaves no active goal or third attempt.
        self.assertEqual((result.outcome, result.status), (NavigationOutcome.NAVIGATION_FAILED, GoalStatus.ABORTED))
        self.assertEqual((result.attempts, len(client.goals), self.costmaps.calls, executor.has_active_goal), (2, 2, 1, False))

    def test_progress_failure_does_not_blindly_clear_and_retry(self) -> None:
        executor, client = self.executor((GoalStatus.ABORTED,))
        client.failure_reason = 'NAV_NO_PROGRESS'
        result = executor.execute(self.request())
        self.assertEqual(result.outcome, NavigationOutcome.NAVIGATION_FAILED)
        self.assertEqual((result.attempts, self.costmaps.calls), (1, 0))
        self.assertEqual(executor.failure_reason, 'NAV_NO_PROGRESS')

    def test_late_terminal_after_wait_error_releases_only_old_goal(self) -> None:
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        def fail_wait():
            raise RuntimeError('cancel acknowledgement timeout')
        client.on_wait = fail_wait
        with self.assertRaises(RuntimeError):
            executor.execute(self.request())
        self.assertTrue(executor.has_active_goal)
        old_callback = client.done_callback
        old_callback(GoalStatus.PREEMPTED, None)
        self.assertFalse(executor.has_active_goal)
        self.assertEqual(executor.execute(self.request()).outcome, NavigationOutcome.SUCCEEDED)
        old_callback(GoalStatus.ABORTED, None)
        self.assertFalse(executor.has_active_goal)

    def test_lost_goal_permits_only_the_same_single_retry(self) -> None:
        # Given: actionlib reports LOST before a successful retry.
        executor, client = self.executor((GoalStatus.LOST, GoalStatus.SUCCEEDED))
        # When: lifecycle recovery runs.
        result = executor.execute(self.request())
        # Then: LOST is terminal before clear/retry and never creates extra attempts.
        self.assertEqual((result.outcome, result.attempts, len(client.goals), self.costmaps.calls), (NavigationOutcome.SUCCEEDED, 2, 2, 1))

    def test_stale_success_callback_cannot_complete_new_retry(self) -> None:
        # Given: a late success callback from an aborted first attempt.
        executor, client = self.executor((GoalStatus.ABORTED, GoalStatus.REJECTED))
        client.emit_stale_on_second_wait = True
        # When: the second attempt receives stale then current terminal callbacks.
        result = executor.execute(self.request())
        # Then: correlation reports the current rejection, never the stale success.
        self.assertEqual((result.outcome, result.status), (NavigationOutcome.NAVIGATION_FAILED, GoalStatus.REJECTED))

    def test_cancel_waits_for_preempted_terminal_state(self) -> None:
        # Given: cancellation arrives while the first goal wait owns the client.
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        client.on_wait = executor.cancel
        # When: execution observes the cancellation terminal state.
        result = executor.execute(self.request())
        # Then: PREEMPTED is correlated and no active goal remains.
        self.assertEqual((result.outcome, result.status), (NavigationOutcome.CANCELLED, GoalStatus.PREEMPTED))
        self.assertEqual((client.cancel_count, executor.has_active_goal), (1, False))

    def test_nonblocking_cancel_request_leaves_terminal_wait_with_executor_thread(self) -> None:
        # Given: an action-server callback requests cancel during the executor wait.
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        client.on_wait = executor.request_cancel

        # When: the execution thread owns terminal correlation.
        result = executor.execute(self.request())

        # Then: cancellation uses one client wait rather than a competing second wait.
        self.assertEqual(result.outcome, NavigationOutcome.CANCELLED)
        self.assertEqual((client.cancel_count, client.wait_count), (1, 1))

    def test_cancel_accepts_recalled_terminal_state(self) -> None:
        # Given: cancellation reaches move_base before the goal becomes active.
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        client._cancel_state = GoalStatus.RECALLED
        client.on_wait = executor.cancel
        # When: actionlib reports the pre-execution terminal state.
        result = executor.execute(self.request())
        # Then: RECALLED is a completed cancellation, not navigation success.
        self.assertEqual((result.outcome, result.status), (NavigationOutcome.CANCELLED, GoalStatus.RECALLED))

    def test_generation_mismatch_rejects_before_goal_send(self) -> None:
        # Given: READY names the floor but exposes a stale map generation.
        self.floor_state.map_generation = 6
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        # When/Then: strict readiness rejects the stale state.
        with self.assertRaises(NavigationReadinessError):
            executor.execute(self.request())
        self.assertEqual(client.goals, [])

    def test_supervisor_non_nav_rejects_before_goal_send(self) -> None:
        # Given: command ownership is held by stair mode.
        self.supervisor_state.state = SupervisorState.STAIR
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        # When/Then: no navigation command can be created.
        with self.assertRaises(NavigationReadinessError):
            executor.execute(self.request())
        self.assertEqual(client.goals, [])

    def test_transitioning_and_fault_never_send_goals(self) -> None:
        # Given/When/Then: each unsafe floor state independently blocks move_base.
        for state in (FloorState.TRANSITIONING, FloorState.FAULT):
            with self.subTest(state=state):
                self.floor_state.state = state
                executor, client = self.executor((GoalStatus.SUCCEEDED,))
                with self.assertRaises(NavigationReadinessError):
                    executor.execute(self.request())
                self.assertEqual(client.goals, [])

    def test_retry_rechecks_readiness_after_terminal_failure(self) -> None:
        # Given: clearing costmaps reveals a generated TRANSITIONING state.
        self.costmaps.on_clear = lambda: setattr(self.floor_state, "state", FloorState.TRANSITIONING)
        executor, client = self.executor((GoalStatus.ABORTED, GoalStatus.SUCCEEDED))
        # When/Then: the stale retry is blocked after the first terminal goal.
        with self.assertRaises(NavigationReadinessError):
            executor.execute(self.request())
        self.assertEqual(len(client.goals), 1)

    def test_cancel_during_costmap_clear_cannot_send_retry_goal(self) -> None:
        # Given: cancellation interrupts the gap after terminal ABORTED.
        executor, client = self.executor((GoalStatus.ABORTED, GoalStatus.SUCCEEDED))
        self.costmaps.on_clear = executor.cancel
        # When: clear_costmaps observes the interruption.
        result = executor.execute(self.request())
        # Then: the cancellation latch prevents a second goal from restarting motion.
        self.assertEqual(result.outcome, NavigationOutcome.CANCELLED)
        self.assertEqual((len(client.goals), self.costmaps.calls, executor.has_active_goal), (1, 1, False))

    def test_cancel_during_second_readiness_cannot_send_retry_goal(self) -> None:
        # Given: cancellation wins while retry readiness is being sampled.
        readiness_calls = 0
        executor: NavigationExecutor

        def floor_state() -> FloorState:
            nonlocal readiness_calls
            readiness_calls += 1
            if readiness_calls == 2:
                executor.cancel()
            return self.floor_state

        executor, client = self.executor((GoalStatus.ABORTED, GoalStatus.SUCCEEDED), floor_state)
        # When: the second-attempt floor gate triggers cancellation.
        result = executor.execute(self.request())
        # Then: cancellation wins ownership and no retry reaches move_base.
        self.assertEqual((result.outcome, len(client.goals)), (NavigationOutcome.CANCELLED, 1))

    def test_stair_handoff_during_second_readiness_cannot_send_retry_goal(self) -> None:
        # Given: stair acquisition wins while retry readiness is being sampled.
        readiness_calls = 0
        executor: NavigationExecutor

        def floor_state() -> FloorState:
            nonlocal readiness_calls
            readiness_calls += 1
            if readiness_calls == 2:
                executor.prepare_for_stair()
            return self.floor_state

        executor, client = self.executor((GoalStatus.ABORTED, GoalStatus.SUCCEEDED), floor_state)
        # When: the second-attempt floor gate completes the handoff barrier.
        result = executor.execute(self.request())
        # Then: handoff returns only after retry eligibility is permanently revoked.
        self.assertEqual((result.outcome, len(client.goals)), (NavigationOutcome.CANCELLED, 1))
        self.assertEqual(self.barrier.calls, 1)

    def test_stair_handoff_cancels_waits_then_requires_zero_stationary(self) -> None:
        # Given: stair acquisition interrupts an active navigation goal.
        executor, client = self.executor((GoalStatus.SUCCEEDED,))
        client.on_wait = executor.prepare_for_stair
        # When: execution reaches the handoff barrier.
        result = executor.execute(self.request())
        # Then: cancellation reaches PREEMPTED before zero/stationary evidence.
        self.assertEqual((result.outcome, client.cancel_count, self.barrier.calls), (NavigationOutcome.CANCELLED, 1, 1))

    def test_stair_handoff_rejects_missing_stationary_evidence(self) -> None:
        # Given: the zero barrier cannot confirm the robot is stationary.
        self.barrier.stationary = False
        executor, _client = self.executor(())
        # When/Then: stair ownership cannot be acquired.
        with self.assertRaises(HandoffSafetyError):
            executor.prepare_for_stair()


if __name__ == "__main__":
    unittest.main()


class EntryOutcomeTest(NavigationExecutorTest):
    def test_entry_region_requires_explicit_request(self):
        executor, client = self.executor((GoalStatus.PREEMPTED,))
        client.entry_region_reached = True
        result = executor.execute(self.request())
        self.assertEqual(result.outcome, NavigationOutcome.CANCELLED)

    def test_entry_region_success_preserves_real_move_base_status(self):
        from dataclasses import replace
        executor, client = self.executor((GoalStatus.PREEMPTED,))
        client.entry_region_reached = True
        result = executor.execute(replace(self.request(), entry_approach=True))
        self.assertEqual(result.outcome, NavigationOutcome.SUCCEEDED)
        self.assertEqual(result.status, GoalStatus.PREEMPTED)
