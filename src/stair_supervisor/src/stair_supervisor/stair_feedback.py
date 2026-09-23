"""Stair-local geometry, feedback and phase evidence; no command transport.

All physical limits belong to a commissioned route, not to global NAV gates.
Observed position is never replaced by commanded motion or restamped prediction.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from functools import lru_cache
import math
import threading

import numpy as np

from .configuration import Direction
from .stair_evidence import EvidenceReport, Phase


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def deadband(value, threshold):
    """Continuous dead zone; crossing its boundary does not jump the command."""
    return math.copysign(max(0., abs(value) - threshold), value)


def transform(value):
    matrix = np.asarray(value, dtype=float).reshape(4, 4)
    if (not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1]) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6) or
            np.linalg.det(matrix[:3, :3]) < 0.999):
        raise ValueError("invalid rigid transform")
    return matrix


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1.]])


def placed_entry_transform(local_from_lidar, base_from_lidar, position, yaw):
    """Profile-from-local for an explicitly placed body; preserve measured tilt.

    Position/heading are operator input in the route frame, not automatic scene
    recognition. Only a gravity-preserving yaw and translation align the frames.
    """
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all() or not math.isfinite(yaw):
        raise ValueError("entry placement requires finite XYZ and heading")
    local_base = transform(local_from_lidar) @ np.linalg.inv(transform(base_from_lidar))
    observed_yaw = math.atan2(local_base[1, 0], local_base[0, 0])
    profile_local = se2(0., 0., yaw-observed_yaw)
    profile_local[:3, 3] = position-profile_local[:3, :3] @ local_base[:3, 3]
    return profile_local


def inside_polygon(points, polygon, margin=0.0):
    """Convex CCW support polygon with a signed inward metric margin."""
    polygon = np.asarray(polygon, dtype=float)
    edges = np.roll(polygon, -1, axis=0)-polygon
    lengths = np.linalg.norm(edges, axis=1)
    if np.any(lengths < 1e-8):
        return False
    delta = np.asarray(points)[:, None, :]-polygon
    distances = (edges[:, 0]*delta[:, :, 1]-edges[:, 1]*delta[:, :, 0])/lengths
    return bool(np.all(distances >= margin))


def _area(polygon):
    if len(polygon) < 3:
        return 0.0
    p = np.asarray(polygon)
    return abs(float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1)))) / 2


def _segment_interval(a, b, polygon):
    low, high = 0., 1.
    for p, q in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = q-p
        start = edge[0]*(a[1]-p[1])-edge[1]*(a[0]-p[0])
        slope = edge[0]*(b[1]-a[1])-edge[1]*(b[0]-a[0])
        if abs(slope) < 1e-12:
            if start < -1e-10:
                return None
        elif slope > 0:
            low = max(low, -start/slope)
        else:
            high = min(high, -start/slope)
    return (low, high) if high-low > 1e-10 else None


@lru_cache(maxsize=64)
def _union_layout(key):
    """Precompute actual regions and exposed boundary segments."""
    original = [np.asarray(p, dtype=float) for p in key]
    polygons = [p for i, p in enumerate(original) if not any(
        i != j and inside_polygon(p, q, -1e-10) and
        (_area(p) < _area(q)-1e-10 or i > j)
        for j, q in enumerate(original))]
    boundary = []
    for i, polygon in enumerate(polygons):
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            edge = b-a
            outward = np.array([edge[1], -edge[0]])
            covered = []
            for j, other in enumerate(polygons):
                if i == j:
                    continue
                interval = _segment_interval(a, b, other)
                if interval is None:
                    continue
                middle = a+(b-a)*sum(interval)/2
                edges = np.roll(other, -1, axis=0)-other
                distances = edges[:,0]*(middle[1]-other[:,1])-edges[:,1]*(middle[0]-other[:,0])
                derivatives = edges[:,0]*outward[1]-edges[:,1]*outward[0]
                # An internal seam has another region immediately on its right.
                # Coincident external edges keep their outward-facing boundary.
                if np.all((distances > 1e-9) | (derivatives >= -1e-10)):
                    covered.append(interval)
            cursor = 0.
            for low, high in sorted(covered):
                if low > cursor+1e-10:
                    boundary.append([a+(b-a)*cursor, a+(b-a)*low])
                cursor = max(cursor, high)
            if cursor < 1.-1e-10:
                boundary.append([a+(b-a)*cursor, b])
    return polygons, np.asarray(boundary)


def _layout(polygons):
    if not 1 <= len(polygons) <= 3:
        raise ValueError("support requires one to three neighboring polygons")
    return _union_layout(tuple(tuple(tuple(map(float, p)) for p in polygon) for polygon in polygons))


def _covered_body(points, polygons, boundary):
    if any(inside_polygon(points, p) for p in polygons):
        return True
    # A convex body's interior is connected. With an interior witness inside
    # the union, an uncovered hole/notch must expose a union boundary inside it.
    probes = np.vstack((points, np.mean(points, axis=0)))
    inside = np.zeros(len(probes), dtype=bool)
    for polygon in polygons:
        edges = np.roll(polygon,-1,axis=0)-polygon
        delta = probes[:,None,:]-polygon
        inside |= np.all(edges[:,0]*delta[:,:,1]-edges[:,1]*delta[:,:,0] >= -1e-10, axis=1)
    if not np.all(inside):
        return False
    low, high = np.zeros(len(boundary)), np.ones(len(boundary))
    valid = np.ones(len(boundary), dtype=bool)
    a, b = boundary[:,0], boundary[:,1]
    edges = np.roll(points,-1,axis=0)-points
    for p, edge in zip(points, edges):
        distance = edge[0]*(a[:,1]-p[1])-edge[1]*(a[:,0]-p[0])
        slope = edge[0]*(b[:,1]-a[:,1])-edge[1]*(b[:,0]-a[:,0])
        crossing = np.divide(-distance, slope, out=np.zeros_like(distance), where=np.abs(slope)>1e-12)
        low = np.where(slope>1e-12, np.maximum(low,crossing),low)
        high = np.where(slope< -1e-12, np.minimum(high,crossing),high)
        valid &= ~((np.abs(slope)<=1e-12)&(distance< -1e-10))
    valid &= high>=low
    if not np.any(valid):
        return True
    middle = a[valid]+(b-a)[valid]*((low+high)[valid]/2)[:,None]
    delta = middle[:,None,:]-points
    inward = edges[:,0]*delta[:,:,1]-edges[:,1]*delta[:,:,0]
    strictly_inside = np.all(inward>1e-10*np.linalg.norm(edges,axis=1),axis=1)
    return not np.any(strictly_inside)


def _clearance_inside(points, boundary):
    """Distance between a covered convex body and the exposed union boundary."""
    def distances(vertices, segments):
        a, direction = segments[:,0], segments[:,1]-segments[:,0]
        delta = vertices[:,None,:]-a
        ratio = np.clip(np.sum(delta*direction, axis=2)/np.sum(direction*direction, axis=1), 0., 1.)
        return np.linalg.norm(delta-ratio[:,:,None]*direction, axis=2).min()
    body_edges = np.stack((points, np.roll(points,-1,axis=0)), axis=1)
    return float(min(distances(points, boundary), distances(boundary[:,0], body_edges),
                     distances(boundary[:,1], body_edges)))


def support_clearance(points, polygons):
    regions, boundary = _layout(polygons)
    return _clearance_inside(np.asarray(points), boundary) if _covered_body(points, regions, boundary) else None


def inside_support_union(points, polygons, margin=0.0):
    """Whole-body union coverage and clearance to actual external edges.

    Interior/boundary coverage checks prevent supported vertices from hiding a
    gap or hole inside the body. Internal seams contribute no margin wall.
    Region geometry is cached; no hull fills gaps between the allowed polygons.
    """
    regions, boundary = _layout(polygons)
    if any(inside_polygon(points, p, margin) for p in regions):
        return True
    if len(regions) == 1 or not _covered_body(points, regions, boundary):
        return False
    return _clearance_inside(np.asarray(points), boundary) >= margin-1e-10


@dataclass(frozen=True)
class RouteAnchor:
    epoch: int
    sequence: int
    measured_at: float
    local_from_profile: tuple
    uncertainty_m: float
    source: str

    @classmethod
    def from_entry(cls, sample, base_from_lidar, profile_from_base, uncertainty_m, source):
        if not sample.geometry_valid or uncertainty_m < 0 or not math.isfinite(uncertainty_m) or not source:
            raise ValueError("entry anchor requires fresh geometry and explicit measurement provenance")
        local_base = transform(sample.transform) @ np.linalg.inv(transform(base_from_lidar))
        matrix = local_base @ np.linalg.inv(transform(profile_from_base))
        return cls(sample.epoch, sample.sequence, sample.measured_at, tuple(matrix.flat), uncertainty_m, source)


def match_entry_landmarks(observed_xy, profile_candidates, max_residual, ambiguity_margin):
    """Fit ordered, identified surveyed landmarks, rejecting ambiguous identities.

    Correspondence candidates must come from surveyed/identified geometry. This
    does not claim to recognize arbitrary walls or steps in a raw point cloud.
    Returns profile-from-local SE(2) and maximum correspondence residual.
    """
    observed = np.asarray(observed_xy, dtype=float)
    if observed.ndim != 2 or observed.shape[1] != 2 or len(observed) < 3 or not np.isfinite(observed).all():
        raise ValueError("need finite identified entry landmarks")
    if np.linalg.svd(observed - observed.mean(axis=0), compute_uv=False)[-1] < 1e-3:
        raise ValueError("entry geometry is underconstrained")
    fits = []
    for candidate in profile_candidates:
        target = np.asarray(candidate, dtype=float)
        if target.shape != observed.shape or not np.isfinite(target).all():
            raise ValueError("landmark correspondence shape/value mismatch")
        x, y = observed - observed.mean(axis=0), target - target.mean(axis=0)
        u, _, vt = np.linalg.svd(x.T @ y)
        r = vt.T @ np.diag([1., np.linalg.det(vt.T @ u.T)]) @ u.T
        translation = target.mean(axis=0) - r @ observed.mean(axis=0)
        residual = float(np.max(np.linalg.norm(observed @ r.T + translation - target, axis=1)))
        fits.append((residual, se2(*translation, math.atan2(r[1, 0], r[0, 0]))))
    fits.sort(key=lambda pair: pair[0])
    if not fits or fits[0][0] > max_residual:
        raise ValueError("entry landmark residual exceeds surveyed budget")
    if len(fits) > 1 and fits[1][0] - fits[0][0] <= ambiguity_margin:
        raise ValueError("ambiguous entry correspondence")
    return fits[0][1], fits[0][0]


class StairFeedback:
    """Implements PhaseEvidence plus motion policy for the existing Supervisor."""

    def __init__(self, worker, base_from_lidar, routes):
        self.worker = worker
        self.base_from_lidar = transform(base_from_lidar)
        self.routes = {r["id"]: self._validate_route(r) for r in routes}
        if len(self.routes) != len(routes):
            raise ValueError("duplicate LiDAR route ID")
        self._lock = threading.RLock()
        self.anchors = {}
        self.route = self.profile = self.anchor = None
        self.phase = Phase.VERIFY_ENTRY
        self.debug = {}
        self._last_command = (0., 0.)
        self._last_command_at = None
        self._last_sample = None
        self._history = deque(maxlen=64)
        self._recovering_at = None
        self._interrupted = False
        self.test_status = None
        self._clearance_key = None

    @staticmethod
    def _validate_route(route):
        keys = {"id", "commissioned", "direction", "flight_1", "flight_2", "landing_polygon",
                "exit_polygon", "entry_polygon", "flight_1_polygon", "flight_2_polygon", "footprint",
                "turn_path", "limits", "loss_response", "loss_response_evidence"}
        optional = {"entry_reference", "phase_test_limits"}
        if not keys <= set(route) or set(route) - keys - optional:
            raise ValueError("LiDAR route keys must be " + str(sorted(keys)))
        if "phase_test_limits" in route:
            budgets = route["phase_test_limits"]
            if (not isinstance(budgets, dict) or not budgets or
                    any(k not in {p.value for p in Phase} or type(v) not in (float, int) or
                        not math.isfinite(v) or v <= 0 for k, v in budgets.items())):
                raise ValueError("phase_test_limits needs named phases and positive finite seconds")
        if route["direction"] != "UP":
            raise ValueError("reverse/downhill feedback requires separate commissioning; UP only")
        if type(route["commissioned"]) is not bool:
            raise ValueError("commissioned must be boolean")
        for name in ("flight_1", "flight_2"):
            v = np.asarray(route[name], dtype=float)
            if v.shape != (2, 3) or not np.isfinite(v).all() or np.linalg.norm(v[1, :2] - v[0, :2]) < .01 or v[1, 2] <= v[0, 2]:
                raise ValueError("flight needs surveyed ascending start/end XYZ")
        for name in ("landing_polygon", "exit_polygon", "entry_polygon", "flight_1_polygon", "flight_2_polygon", "footprint"):
            p = np.asarray(route[name], dtype=float)
            if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all() or _area(p) <= 1e-8 or not inside_polygon(p, p, -1e-9):
                raise ValueError("%s must be a finite convex CCW polygon" % name)
        points = np.asarray(route["turn_path"], dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 1 or not np.isfinite(points).all():
            raise ValueError("turn_path requires surveyed x,y,yaw waypoints")
        keys = {"warn_sec", "expire_sec", "recover_sec", "anchor_max_age_sec", "anchor_uncertainty_m",
                "margin_m", "yaw_kp", "lateral_kp", "speed_kp", "hold_kp", "hold_kd", "max_v",
                "max_w", "max_accel", "max_alpha", "hold_v", "position_on", "position_off",
                "yaw_tolerance", "stationary_v", "stationary_w", "settle_sec", "height_tolerance",
                "max_roll", "max_pitch", "handoff_sec", "yaw_deadband", "lateral_deadband", "speed_deadband"}
        limits = route["limits"]
        if not keys <= set(limits) or set(limits) - keys - {"min_flight_v"} or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in limits.values()):
            raise ValueError("route limits require finite positive commissioning values: " + str(sorted(keys)))
        if limits.get("min_flight_v", 0.) > limits["max_v"]:
            raise ValueError("min_flight_v must not exceed max_v")
        if not (limits["warn_sec"] < limits["expire_sec"] < limits["recover_sec"] and limits["position_off"] < limits["position_on"]):
            raise ValueError("invalid observation budgets or hold hysteresis")
        if not (limits["yaw_deadband"] < limits["yaw_tolerance"] < math.pi and
                limits["lateral_deadband"] < limits["position_off"] and
                limits["speed_deadband"] < limits["max_v"] and limits["hold_v"] <= limits["max_v"]):
            raise ValueError("deadbands must allow reaching arrival tolerances; hold speed must respect max_v")
        # Operator requires neutral velocity, never an emergency-stop API request.
        # Zero is not a physical hold; route evidence records operator recovery.
        if route["loss_response"] != "zero_velocity" or not isinstance(route["loss_response_evidence"], str) or not route["loss_response_evidence"].strip():
            raise ValueError("route requires documented zero_velocity/operator takeover response")
        return route

    def install_anchor(self, route_id, anchor):
        with self._lock:
            if anchor.epoch!=self.worker.epoch:
                raise ValueError("entry anchor epoch no longer current")
            self.anchors[route_id]=anchor

    def capture_entry(self, route_id, profile_from_base, uncertainty, source, now, sample=None):
        with self._lock:
            route = self.routes[route_id]
            sample = self.worker.snapshot() if sample is None else sample
            if sample is None or now - sample.measured_at > route["limits"]["warn_sec"] or sample.measured_at > now:
                raise ValueError("entry needs a fresh geometry sample")
            if uncertainty > route["limits"]["anchor_uncertainty_m"]:
                raise ValueError("entry uncertainty exceeds surveyed budget")
            if sample.epoch != self.worker.epoch:
                raise ValueError("entry epoch changed during correspondence fitting")
            anchor = RouteAnchor.from_entry(sample, self.base_from_lidar, profile_from_base, uncertainty, source)
            self.anchors[route_id] = anchor
            return anchor

    def test_plan(self, profile, phase, duration, from_entry):
        route = self.routes.get(profile.id)
        if route is None:
            raise ValueError("no LiDAR route for this test")
        phases = tuple(Phase)
        phases = phases[:phases.index(phase) + 1] if from_entry else (phase,)
        defaults = ({p.value: 30.0 for p in Phase if p is not Phase.EXIT_CONFIRM}
                    if route["commissioned"] else {Phase.FORWARD_SEGMENT_1.value: 4.0})
        budgets = route.get("phase_test_limits", defaults)
        if any(p.value not in budgets for p in phases):
            raise ValueError("test includes a phase without an explicit route budget")
        if duration > min(sum(budgets[p.value] for p in phases), profile.timeout_sec):
            raise ValueError("test duration exceeds route phase budgets or profile timeout")
        return phases, {p: budgets[p.value] for p in phases}

    def prepare(self, profile, now, *, phase_test=False, start_phase=None):
        with self._lock:
            route = self.routes.get(profile.id)
            anchor = self.anchors.get(profile.id)
            sample = self.worker.snapshot()
            if route is None or (not route["commissioned"] and not phase_test) or profile.direction is not Direction.UP:
                raise ValueError("no commissioned LiDAR route for this stair/direction")
            if (anchor is None or anchor.epoch != self.worker.epoch or sample is None or
                    sample.epoch != self.worker.epoch or not sample.geometry_valid):
                raise ValueError("missing or invalidated per-run entry anchor")
            if now - anchor.measured_at > route["limits"]["anchor_max_age_sec"] or not 0 <= now - sample.measured_at <= route["limits"]["warn_sec"]:
                raise ValueError("entry anchor/observation is stale")
            if start_phase is not None:
                pose = np.linalg.inv(transform(anchor.local_from_profile)) @ transform(sample.transform) @ np.linalg.inv(self.base_from_lidar)
                footprint = np.column_stack((route["footprint"], np.zeros(len(route["footprint"])), np.ones(len(route["footprint"]))))
                support = (pose @ footprint.T).T[:, :2]
                entry = start_phase in (Phase.VERIFY_ENTRY, Phase.ALIGN, Phase.FORWARD_SEGMENT_1)
                region = route["entry_polygon"] if entry else route["landing_polygon"]
                height = route["flight_1"][0 if entry else 1][2]
                if (not inside_polygon(support, region, route["limits"]["margin_m"] + anchor.uncertainty_m) or
                        abs(pose[2, 3] - height) > route["limits"]["height_tolerance"]):
                    raise ValueError("test start pose is outside its entry/landing region or height; use from_entry")
            return route, anchor

    def arm(self, profile, started_at, *, phase_test=False, start_phase=None):
        with self._lock:
            self.route, self.anchor = self.prepare(profile, started_at, phase_test=phase_test, start_phase=start_phase)
            self.profile = profile
            self.started_at = started_at
            self._history.clear()
            self._last_sample = None
            self._recovering_at = None
            self._interrupted = False
            self._last_command = (0., 0.)
            self._last_command_at = started_at
            self.turn_index = 0
            self._holding = False
            self._phase_sequence = 0
            self._flight_seen = [False, False]
            self.test_status = None
            self._clearance_key = None
            self.anchors.pop(profile.id, None)  # a later goal needs new entry evidence

    def reset_command(self, now):
        """Match slew history to the zero command actually sent during entry ACK."""
        with self._lock:
            self._last_command, self._last_command_at = (0., 0.), now

    def support_regions(self, phase):
        names = {
            Phase.FORWARD_SEGMENT_1: ("entry_polygon", "flight_1_polygon", "landing_polygon"),
            Phase.FORWARD_SEGMENT_2: ("landing_polygon", "flight_2_polygon", "exit_polygon"),
            Phase.VERIFY_ENTRY: ("entry_polygon",), Phase.ALIGN: ("entry_polygon",),
            # The approach still has the first flight behind it. An internal
            # join is not a drop edge when the body pitches/yaws between scans.
            # Actual turn waypoints after the approach stay on the flat landing.
            Phase.LANDING: ("flight_1_polygon", "landing_polygon"),
            Phase.TURN_TO_NEXT_FLIGHT: (("flight_1_polygon", "landing_polygon")
                                       if self.turn_index == 0 else ("landing_polygon",)),
            Phase.EXIT_CONFIRM: ("exit_polygon",),
        }[phase]
        return names, [self.route[name] for name in names]

    def begin_phase(self, phase, now):
        with self._lock:
            self.phase = phase
            self._history.clear()
            sample = self.worker.snapshot()
            self._phase_sequence = sample.sequence if sample else 0
            self._holding = False

    def _observe(self, sample):
        pose = np.linalg.inv(transform(self.anchor.local_from_profile)) @ transform(sample.transform) @ np.linalg.inv(self.base_from_lidar)
        yaw = math.atan2(pose[1, 0], pose[0, 0])
        pitch = math.asin(float(np.clip(-pose[2, 0], -1, 1)))
        roll = math.atan2(pose[2, 1], pose[2, 2])
        velocity, angular = np.zeros(3), 0.
        if self._last_sample is not None:
            old, old_pose, old_yaw = self._last_sample
            dt = sample.stamp - old.stamp
            if dt > 0:
                velocity = (pose[:3, 3] - old_pose[:3, 3]) / dt
                angular = wrap(yaw - old_yaw) / dt
        if self._last_sample is None or sample.sequence != self._last_sample[0].sequence:
            self._last_sample = (sample, pose, yaw)
            self._history.append((sample.stamp, pose[:3, 3].copy(), yaw, velocity, angular))
        elif self._history:
            velocity, angular = self._history[-1][3:]
        footprint = np.column_stack((self.route["footprint"], np.zeros(len(self.route["footprint"])), np.ones(len(self.route["footprint"]))))
        support = (pose @ footprint.T).T[:, :2]
        return pose, yaw, pitch, roll, velocity, angular, support

    def _settled(self, now_sample, *, average_rate=False):
        limits = self.route["limits"]
        recent = [h for h in self._history if h[0] >= now_sample.stamp - limits["settle_sec"] - limits["warn_sec"]]
        span = recent[-1][0] - recent[0][0] if len(recent) >= 2 else 0.
        self._settling_detail = dict(rate_mode="window_net_drift" if average_rate else "instantaneous_peak",
                                    samples=len(recent), window_sec=float(span))
        if len(recent) < 2 or span < limits["settle_sec"]:
            return False
        peak_v = max(np.linalg.norm(h[3][:2]) for h in recent)
        peak_w = max(abs(h[4]) for h in recent)
        # Entry accepts bounded sway; it does not declare physical stillness.
        # Net displacement can cancel reversals, so retain the excursion and
        # target-pose limits while rejecting sustained drift above the budget.
        v = np.linalg.norm((recent[-1][1] - recent[0][1])[:2]) / span if average_rate else peak_v
        w = abs(wrap(recent[-1][2] - recent[0][2])) / span if average_rate else peak_w
        excursion = max(np.linalg.norm(h[1] - recent[0][1]) for h in recent)
        angle = max(abs(wrap(h[2] - recent[0][2])) for h in recent)
        self._settling_detail.update(position_excursion_m=float(excursion), yaw_excursion_rad=float(angle),
            evaluated_v=float(v), evaluated_w=float(w), peak_v=float(peak_v), peak_w=float(peak_w),
            stationary_v_limit=limits["stationary_v"], stationary_w_limit=limits["stationary_w"])
        return bool(excursion <= limits["position_off"] and angle <= limits["yaw_tolerance"] and
                    v <= limits["stationary_v"] and w <= limits["stationary_w"])

    def evaluate(self, phase, now, *, command_required=False, neutral_only=False):
        with self._lock:
            sample = self.worker.snapshot()
            limits = self.route["limits"]
            if not self._interrupted and now - self.started_at > self.profile.timeout_sec:
                self.debug = dict(tracking_state="TIMEOUT", loss_response=True)
                return EvidenceReport(phase, False, True, "LiDAR traversal/hold timeout", 0, 0, math.inf)
            age = math.inf if sample is None or sample.last_geometry_at is None else now - sample.last_geometry_at
            reset = self.worker.epoch != self.anchor.epoch or sample is not None and sample.epoch != self.anchor.epoch
            expired = sample is None or age >= limits["expire_sec"] or age < 0 or bool(self.worker.diagnostics()["worker_error"])
            if reset or expired or self._interrupted:
                self._recovering_at = now if self._recovering_at is None else self._recovering_at
                overdue = now - self._recovering_at >= limits["recover_sec"]
                self.debug = dict(tracking_state="HANDOFF_OVERDUE" if overdue else "RECOVERING",
                                  loss_response=True, anchor_reset=reset, age=age,
                                  physical_handoff_overdue=overdue)
                # Once a physical loss response is selected, automatic restart
                # needs re-arming its mode and is intentionally not implicit.
                return EvidenceReport(phase, False, True, "tracking expired/reset; commissioned loss response required", 0, 0, age)
            if age > limits["warn_sec"] or not sample.geometry_valid:
                self.debug = dict(tracking_state="DEGRADED", loss_response=False, age=age)
                return EvidenceReport(phase, False, False, "bounded tracking delay; phase progress suspended", 0, 0, age)
            pose, yaw, pitch, roll, velocity, angular, support = self._observe(sample)
            self._recovering_at = None
            pos = pose[:3, 3]
            margin = limits["margin_m"] + self.anchor.uncertainty_m
            landing = inside_polygon(support, self.route["landing_polygon"], margin)
            exit_clear = inside_polygon(support, self.route["exit_polygon"], margin)
            complete = False
            settling = None
            v, w, ey, epsi, progress = 0., 0., 0., 0., 0.
            if abs(roll) > limits["max_roll"] or abs(pitch) > limits["max_pitch"]:
                self.debug = dict(tracking_state="ATTITUDE_LIMIT", loss_response=True, roll=roll, pitch=pitch)
                return EvidenceReport(phase, False, True, "commissioned attitude envelope exceeded", 0, 0, age)
            if phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2):
                idx = 0 if phase is Phase.FORWARD_SEGMENT_1 else 1
                start, end = np.asarray(self.route["flight_%d" % (idx + 1)])
                direction = end[:2] - start[:2]
                length = np.linalg.norm(direction)
                direction = direction / length
                target, target_yaw = end, math.atan2(direction[1], direction[0])
                progress = float((pos[:2] - start[:2]) @ direction)
                ey = float(direction[0] * (pos[1] - start[1]) - direction[1] * (pos[0] - start[0]))
                epsi = wrap(yaw - math.atan2(direction[1], direction[0]))
                actual_v = float(velocity[:2] @ direction)
                nominal = min(self.profile.linear_speed, limits["max_v"])
                v = np.clip(nominal + limits["speed_kp"] * deadband(nominal - actual_v, limits["speed_deadband"]), 0, limits["max_v"])
                # Smooth slowdown proportional to error, never a normal-noise stop gate.
                v /= 1 + abs(epsi) + abs(ey)
                # Operator's useful forward-command floor, in the transport's
                # input units. Slew and geometric feasibility still take priority.
                v = max(v, limits.get("min_flight_v", 0.))
                w = -limits["yaw_kp"] * deadband(epsi, limits["yaw_deadband"]) - limits["lateral_kp"] * deadband(ey, limits["lateral_deadband"])
                if pos[2] - start[2] >= (end[2] - start[2]) * .5:
                    self._flight_seen[idx] = True
                checks = dict(progress=progress >= length, ascent_seen=self._flight_seen[idx],
                              height=abs(pos[2] - end[2]) <= limits["height_tolerance"],
                              body_in_next_region=landing if idx == 0 else exit_clear)
                complete = all(checks.values())
            else:
                if phase in (Phase.VERIFY_ENTRY, Phase.ALIGN):
                    target = np.asarray(self.route["flight_1"])[0]
                    vector = np.diff(np.asarray(self.route["flight_1"]), axis=0)[0]
                    target_yaw = math.atan2(vector[1], vector[0])
                    supported = inside_polygon(support, self.route["entry_polygon"], margin)
                elif phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT):
                    waypoint = self.route["turn_path"][self.turn_index]
                    target, target_yaw, supported = np.array([*waypoint[:2], pos[2]]), waypoint[2], landing
                else:
                    target = np.asarray(self.route["flight_2"])[1]
                    vector = np.diff(np.asarray(self.route["flight_2"]), axis=0)[0]
                    target_yaw, supported = math.atan2(vector[1], vector[0]), exit_clear
                delta = target[:2] - pos[:2]
                distance = float(np.linalg.norm(delta))
                if distance >= limits["position_on"]:
                    self._holding = True
                elif distance <= limits["position_off"]:
                    self._holding = False
                heading = np.array([math.cos(yaw), math.sin(yaw)])
                along = float(delta @ heading)
                epsi = wrap(yaw - target_yaw)
                if self._holding:
                    # A small bounded unicycle arc, with reverse only on surveyed
                    # support. Each proposed step is checked below against edges.
                    v = np.clip(limits["hold_kp"] * along - limits["hold_kd"] * (velocity[:2] @ heading), -limits["hold_v"], limits["hold_v"])
                    lateral = heading[0] * delta[1] - heading[1] * delta[0]
                    w = -limits["yaw_kp"] * deadband(epsi, limits["yaw_deadband"]) + limits["lateral_kp"] * deadband(lateral, limits["lateral_deadband"]) * (1 if v >= 0 else -1)
                else:
                    w = -limits["yaw_kp"] * deadband(epsi, limits["yaw_deadband"])
                at_target = supported and distance <= limits["position_off"] and abs(epsi) <= limits["yaw_tolerance"]
                settled = self._settled(sample, average_rate=phase in (Phase.VERIFY_ENTRY, Phase.ALIGN))
                settling = dict(self._settling_detail)
                checks = dict(body_in_region=supported, position=distance <= limits["position_off"],
                              yaw=abs(epsi) <= limits["yaw_tolerance"], settled=settled)
                complete = all(checks.values())
                if phase is Phase.TURN_TO_NEXT_FLIGHT and complete:
                    if self.turn_index + 1 < len(self.route["turn_path"]):
                        self.turn_index += 1
                        self._history.clear()
                        complete = False
                if phase is Phase.LANDING:
                    complete = landing  # proceed directly to measured landing path
                    checks = dict(body_in_landing=landing)
                if phase is Phase.EXIT_CONFIRM:
                    complete = complete and all(self._flight_seen)
                    checks.update(flight_1_seen=self._flight_seen[0], flight_2_seen=self._flight_seen[1])
            checks["new_phase_observation"] = sample.sequence > self._phase_sequence
            if sample.sequence <= self._phase_sequence:
                complete = False
            names, polygons = self.support_regions(phase)
            raw_supported = inside_support_union(support, polygons)
            current_supported = inside_support_union(support, polygons, margin)
            key = (sample.epoch, sample.sequence, phase)
            if key != self._clearance_key:
                self._clearance_key = key
                self._clearance_lower = support_clearance(support, polygons) if raw_supported else None
            geometry = dict(support_regions=list(names), footprint_supported=raw_supported,
                            current_supported=current_supported, margin_m=limits["margin_m"],
                            anchor_uncertainty_m=self.anchor.uncertainty_m, effective_margin_m=margin,
                            clearance_m=self._clearance_lower,
                            height_error_m=abs(float(pos[2] - target[2])), completion_checks=checks,
                            settling=settling,
                            incomplete_conditions=[k for k, value in checks.items() if not value])
            if not current_supported:
                self.debug = dict(tracking_state="CORRIDOR_LIMIT", loss_response=True, **geometry)
                return EvidenceReport(phase, False, True, "body/clearance exceeds connected support", 0, 0, age)
            # A completed phase emits no command in the Supervisor loop. Do not
            # reject its transition using the old command's future trajectory.
            transition = complete and not command_required and not neutral_only
            if neutral_only:
                v, w = 0., 0.
            elif not transition:
                v, w = self._limit_command(float(v), float(w), now)
            horizon = limits["expire_sec"]
            proposed = (v, w)
            adjusted = False
            if not transition and not self._swept_support(pos[:2], yaw, support, v, w, horizon, polygons, margin):
                alternative = None if neutral_only else self._supported_command(
                    pos[:2], yaw, support, v, w, horizon, polygons, margin, now)
                if alternative is None:
                    self.debug = dict(tracking_state="SUPPORT_LIMIT", loss_response=True,
                                      proposed_command=[v, w], prediction_horizon_sec=horizon, **geometry)
                    return EvidenceReport(phase, False, True, "no bounded correction within connected support", 0, 0, age)
                v, w = alternative
                adjusted = True
            if not transition:
                self._last_command, self._last_command_at = (v, w), now
            self.debug = dict(tracking_state="TRACKED", loss_response=False, age=age, epoch=sample.epoch,
                              sequence=sample.sequence, phase=phase.value, pose=pos.tolist(), yaw=yaw,
                              roll=roll, pitch=pitch, velocity=velocity.tolist(), angular_velocity=angular,
                              ey=ey, epsi=epsi, command=[v, w], landing_supported=landing,
                              exit_supported=exit_clear, compute_sec=sample.compute_sec,
                              target=[*map(float, target), float(target_yaw)], progress=progress,
                              phase_complete=complete, transition_ready=transition,
                              boundary_adjusted=adjusted, proposed_command=list(proposed),
                              below_flight_command_floor=(phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2)
                                  and not transition and v < limits.get("min_flight_v", 0.)),
                              prediction_horizon_sec=horizon, neutral_only=neutral_only, **geometry)
            return EvidenceReport(phase, complete, False, "LiDAR geometry/feedback: " + phase.value, progress, 0, age)

    @staticmethod
    def _swept_support(position, yaw, support, v, w, horizon, polygon, margin):
        polygons = [polygon] if np.asarray(polygon, dtype=object).ndim == 2 else polygon
        relative=support-position
        radius=float(np.max(np.linalg.norm(relative,axis=1)))
        step=horizon/16
        interpolation_bound=(abs(v)+radius*abs(w))*step/2
        times=np.linspace(0.,horizon,17)
        theta=w*times
        if abs(w)<1e-8:
            centers=position+v*times[:,None]*np.array([math.cos(yaw),math.sin(yaw)])
        else:
            centers=position+(v/w)*np.column_stack((np.sin(yaw+theta)-math.sin(yaw),
                                                    math.cos(yaw)-np.cos(yaw+theta)))
        c,s=np.cos(theta)[:,None],np.sin(theta)[:,None]
        swept=np.stack((c*relative[:,0]-s*relative[:,1],
                        s*relative[:,0]+c*relative[:,1]),axis=2)+centers[:,None,:]
        # Typical flight corrections lie in one convex region. Check the same
        # sampled full bodies together before the more expensive union seams.
        if any(inside_polygon(swept.reshape(-1,2),p,margin+interpolation_bound) for p in polygons):
            return True
        for projected in swept:
            if not inside_support_union(projected,polygons,margin+interpolation_bound):
                return False
        return True

    def _supported_command(self, position, yaw, support, v, w, horizon, polygons, margin, now):
        """Try less translation with the same bounded steering before failing.

        Every candidate respects the existing acceleration window, full body,
        unchanged margin and prediction horizon. No reverse or new turn is
        invented by this filter; a future observation recalculates the policy.
        At most four additional sweeps. Zero/zero is not claimed as progress.
        """
        seen = {(v, w)}
        for scale in (.75, .5, .25, 0.):
            candidate = self._limit_command(v * scale, w, now)
            if candidate in seen or max(abs(x) for x in candidate) < 1e-9:
                continue
            seen.add(candidate)
            if self._swept_support(position, yaw, support, *candidate, horizon, polygons, margin):
                return candidate
        return None

    def _limit_command(self, v, w, now):
        limits = self.route["limits"]
        dt = max(0., min(now - self._last_command_at, limits["warn_sec"]))
        old_v, old_w = self._last_command
        linear = float(np.clip(v, max(-limits["max_v"], old_v - limits["max_accel"] * dt), min(limits["max_v"], old_v + limits["max_accel"] * dt)))
        if self.phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.EXIT_CONFIRM):
            # Flat-region command cap takes precedence over deceleration slew:
            # never carry the previous flight's high input onto the landing.
            # This limits requested input; it does not prove physical braking.
            linear = float(np.clip(linear, -limits["hold_v"], limits["hold_v"]))
        return (linear,
                float(np.clip(w, max(-limits["max_w"], old_w - limits["max_alpha"] * dt), min(limits["max_w"], old_w + limits["max_alpha"] * dt))))

    def command(self):
        with self._lock:
            return self._last_command

    def diagnostic_snapshot(self):
        with self._lock:
            result = dict(self.debug, phase=self.phase.value, command=list(self._last_command))
            if self.test_status is not None:
                result["phase_test"] = dict(self.test_status)
            if self.anchor is not None and self.profile is not None:
                result.update(route_id=self.profile.id,
                              run_id="%s:%d:%d" % (self.profile.id, self.anchor.epoch, self.anchor.sequence),
                              anchor_epoch=self.anchor.epoch, anchor_sequence=self.anchor.sequence,
                              anchor_source=self.anchor.source, anchor_uncertainty_m=self.anchor.uncertainty_m)
            return result

    def interrupt(self):
        with self._lock:
            self._interrupted = True

    def loss_response(self, transport):
        self.interrupt()
        transport.update_twist(0., 0.)
        transport.send_current()
