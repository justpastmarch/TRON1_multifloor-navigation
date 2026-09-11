from __future__ import annotations

from collections.abc import Callable
from typing import List, Tuple

from actionlib_msgs.msg import GoalStatus


class FakeActionClient:
    """Deterministic SimpleActionClient-shaped lifecycle fake."""

    def __init__(self, terminal_states: Tuple[int, ...], cancel_state: int = GoalStatus.PREEMPTED) -> None:
        self._terminal_states = list(terminal_states)
        self._done_callback = None
        self._state = GoalStatus.PENDING
        self._cancel_state = cancel_state
        self.goals = []
        self.done_callbacks = []
        self.cancel_count = 0
        self.wait_count = 0
        self.on_wait: Callable[[], None] | None = None
        self.emit_stale_on_second_wait = False

    def send_goal(self, goal, done_cb) -> None:
        self.goals.append(goal)
        self._done_callback = done_cb
        self.done_callbacks.append(done_cb)
        self._state = GoalStatus.ACTIVE

    def wait_for_result(self) -> bool:
        self.wait_count += 1
        if self.on_wait is not None:
            callback = self.on_wait
            self.on_wait = None
            callback()
        if self._state in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            return True
        if not self._terminal_states:
            raise RuntimeError("no terminal action state queued")
        if self.emit_stale_on_second_wait and len(self.goals) == 2:
            self.done_callbacks[0](GoalStatus.SUCCEEDED, None)
        self._state = self._terminal_states.pop(0)
        self._done_callback(self._state, None)
        return True

    def get_state(self) -> int:
        return self._state

    def cancel_goal(self) -> None:
        self.cancel_count += 1
        self._state = self._cancel_state
        self._done_callback(self._state, None)

    def emit_stale(self, callback, status: int) -> None:
        callback(status, None)

    @property
    def done_callback(self):
        return self._done_callback


class FakeCostmaps:
    def __init__(self, on_clear: Callable[[], None] | None = None) -> None:
        self.calls = 0
        self.on_clear = on_clear

    def clear(self) -> None:
        self.calls += 1
        if self.on_clear is not None:
            self.on_clear()


class FakeHandoffBarrier:
    def __init__(self, stationary: bool = True) -> None:
        self.stationary = stationary
        self.calls = 0

    def stop_and_confirm_stationary(self) -> bool:
        self.calls += 1
        return self.stationary
