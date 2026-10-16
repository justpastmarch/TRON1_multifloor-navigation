"""Small, single-use proof that the configured stair action just succeeded.

ROS action goal/result messages may arrive on separate subscriber callbacks in
either order. This cache pairs their goal IDs and never accepts a failed,
stale, wrong-route, or already-consumed completion.
"""

import time


class StairCompletionCache:
    def __init__(self, clock=time.monotonic, result_max_age_sec=30.0):
        self.clock = clock
        self.result_max_age_sec = float(result_max_age_sec)
        self.goals = {}
        self.results = {}
        self.consumed = {}

    def note_goal(self, goal_id, profile_id):
        if not goal_id or not profile_id or goal_id in self.consumed:
            return
        now = self.clock()
        self._prune(now)
        self.goals[goal_id] = (profile_id, now)

    def note_result(self, goal_id, status, result_code, ownership_epoch):
        if not goal_id or goal_id in self.consumed:
            return
        now = self.clock()
        self._prune(now)
        if status != 3 or result_code != 0:
            self.goals.pop(goal_id, None)
            self.results.pop(goal_id, None)
            return
        self.results[goal_id] = (int(ownership_epoch), now)

    def consume(self, profile_id):
        now = self.clock()
        self._prune(now)
        matches = [(result[1], goal_id, result[0])
                   for goal_id, result in self.results.items()
                   if goal_id in self.goals
                   and self.goals[goal_id][0] == profile_id
                   and 0 <= now - result[1] <= self.result_max_age_sec]
        if not matches:
            return None
        _, goal_id, epoch = max(matches)
        self.goals.pop(goal_id, None)
        self.results.pop(goal_id, None)
        self.consumed[goal_id] = now
        return epoch

    def _prune(self, now):
        for goal_id, (_, seen) in list(self.goals.items()):
            if now - seen > 600:
                self.goals.pop(goal_id, None)
        for goal_id, (_, seen) in list(self.results.items()):
            if now - seen > self.result_max_age_sec:
                self.results.pop(goal_id, None)
        for goal_id, seen in list(self.consumed.items()):
            if now - seen > 600:
                self.consumed.pop(goal_id, None)
