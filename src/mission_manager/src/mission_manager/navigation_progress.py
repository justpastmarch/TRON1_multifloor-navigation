"""Progress evidence for NAV, including useful final-heading alignment."""

import math


class NavigationProgress:
    def __init__(self, target, now, timeout=20.0, feedback_timeout=10.0,
                 distance=0.05, angle=math.radians(3), alignment_radius=0.5):
        self.target = target
        values = (*target, now, timeout, feedback_timeout, distance, angle, alignment_radius)
        if not all(math.isfinite(v) for v in values) or min(values[4:]) <= 0:
            raise ValueError('invalid navigation progress policy')
        self.timeout, self.feedback_timeout = timeout, feedback_timeout
        self.distance, self.angle, self.alignment_radius = distance, angle, alignment_radius
        self.last_progress = self.last_feedback = now
        self.anchor = None
        self.heading_best = self.heading_credit = None
        self.last_kind = 'waiting_for_feedback'

    def observe(self, x, y, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, yaw, now)):
            return
        self.last_feedback = now
        if self.anchor is None:
            self.anchor = (x, y)
            self.last_progress = now
        moved = math.hypot(x-self.anchor[0], y-self.anchor[1]) >= self.distance
        if moved:
            self.anchor = (x, y)
            self.last_progress, self.last_kind = now, 'translation'
        error = abs(math.atan2(math.sin(self.target[2]-yaw), math.cos(self.target[2]-yaw)))
        near = math.hypot(x-self.target[0], y-self.target[1]) <= self.alignment_radius
        if near:
            if self.heading_best is None:
                self.heading_best = self.heading_credit = error
            else:
                self.heading_best = min(self.heading_best, error)
                if self.heading_credit-self.heading_best >= self.angle:
                    self.heading_credit = self.heading_best
                    self.last_progress, self.last_kind = now, 'heading_improved'
        # Leaving/re-entering the radius does not reset the heading baseline.
        # Repeating the same angular sweep must not earn repeated progress.

    def failure(self, now):
        if now-self.last_feedback >= self.feedback_timeout:
            return 'NAV_FEEDBACK_STALE'
        if now-self.last_progress >= self.timeout:
            return 'NAV_NO_PROGRESS'
        return ''
