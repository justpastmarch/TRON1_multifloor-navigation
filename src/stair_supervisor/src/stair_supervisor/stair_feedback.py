"""Stair-local geometry, feedback and phase evidence; no command transport.

All physical limits belong to a commissioned route, not to global NAV gates.
Observed position retains its timestamp. Predicted motion is kept separate.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from functools import lru_cache
from concurrent.futures import Future
import math
import threading

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull, QhullError

from .motion_prediction import MotionState, response_envelope, validate_model
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

    policy_version = '2026-10-02-photo-return-v24'

    def __init__(self, worker, base_from_lidar, routes, *, lidar_from_imu=None, imu_max_gap=.03):
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
        self._flight_end_conflict = False
        self._input_gap = None
        self._input_gap_used = 0.
        self._recovering_at = None
        self._interrupted = False
        self.test_status = None
        self._clearance_key = None
        self._imu_lock = threading.Lock()
        self._imu = deque(maxlen=4096)
        self._imu_epoch = getattr(worker, 'epoch', 0)
        self._lidar_from_imu = None if lidar_from_imu is None else np.asarray(lidar_from_imu, dtype=float)
        self._imu_max_gap = imu_max_gap
        self._tx_lock = threading.Lock()
        self._last_sent = None
        self._first_fault = None
        self._last_geometry_sample = None
        self._entry_alignment_pending = None
        self._entry_verified = False
        self._flight_recheck = None
        self._flight_recheck_used = 0.
        self._candidate_debug = []
        self._reset_continuity()

    @staticmethod
    def _validate_route(route):
        keys = {"id", "commissioned", "direction", "flight_1", "flight_2", "landing_polygon",
                "exit_polygon", "entry_polygon", "flight_1_polygon", "flight_2_polygon", "footprint",
                "turn_path", "limits", "loss_response", "loss_response_evidence"}
        optional = {"entry_reference", "phase_test_limits", "roof_turn_path", "roof_landing_polygon", "flight_3", "flight_3_polygon", "motion_prediction", "flight_command_policy", "ascent_height_policy", "landing_turn_policy", "roof_turn_policy", "reverse_of", "entry_map", "flight_input_v", "exit_arrival_policy"}
        if not keys <= set(route) or set(route) - keys - optional:
            raise ValueError("LiDAR route keys must be " + str(sorted(keys)))
        if "phase_test_limits" in route:
            budgets = route["phase_test_limits"]
            if (not isinstance(budgets, dict) or not budgets or
                    any(k not in {p.value for p in Phase} or type(v) not in (float, int) or
                        not math.isfinite(v) or v <= 0 for k, v in budgets.items())):
                raise ValueError("phase_test_limits needs named phases and positive finite seconds")
        if route['direction'] not in ('UP','DOWN'):
            raise ValueError('route direction must be UP or DOWN')
        if route['direction']=='DOWN' and ('reverse_of' not in route or len(route.get('flight_input_v',[]))!=3):
            raise ValueError('DOWN requires a derived RF route and three forward flight inputs')
        if 'flight_input_v' in route and any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 or v>route['limits']['max_v'] for v in route['flight_input_v']):
            raise ValueError('flight inputs must be positive and within max_v')
        if type(route["commissioned"]) is not bool:
            raise ValueError("commissioned must be boolean")
        roof_keys = {"roof_turn_path", "roof_landing_polygon", "flight_3", "flight_3_polygon"}
        if roof_keys & set(route) and not roof_keys <= set(route):
            raise ValueError("RF extension requires turn path, landing, third flight and support polygon")
        if route.get('ascent_height_policy', 'band') not in ('band', 'minimum'):
            raise ValueError('invalid ascent_height_policy')
        if route.get('landing_turn_policy', 'waypoints') not in ('waypoints', 'supported_region'):
            raise ValueError('invalid landing_turn_policy')
        if route.get('landing_turn_policy') == 'supported_region':
            path = np.asarray(route['turn_path'], dtype=float)
            if (path.ndim != 2 or path.shape[1] != 3 or len(path) < 2
                    or not np.allclose(path[0,:2], path[1,:2])
                    or abs(wrap(path[1,2]-path[0,2])) < .1):
                raise ValueError('region turn requires an approach/rotation waypoint pair')
        for key in ('roof_turn_policy', 'exit_arrival_policy'):
            if route.get(key, 'waypoints') not in ('waypoints', 'supported_region'):
                raise ValueError('invalid '+key)
        policy = route.get('flight_command_policy', 'prediction_gated')
        if policy not in ('prediction_gated', 'observed_feedback'):
            raise ValueError('invalid flight_command_policy')
        if 'motion_prediction' in route:
            validate_model(route['motion_prediction'])
            if policy == 'observed_feedback' and route['motion_prediction']['mode'] == 'control':
                raise ValueError('observed_feedback requires diagnostic-only motion prediction')
        roof = roof_keys <= set(route)
        if roof:
            points = np.asarray(route['roof_turn_path'], dtype=float)
            if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
                raise ValueError('RF turn path must contain finite x/y/yaw waypoints')
            if not np.isclose(route['flight_3'][0][2], route['flight_2'][1][2]):
                raise ValueError('RF third flight must start at second flight landing height')
        for name in (("flight_1", "flight_2", "flight_3") if roof else ("flight_1", "flight_2")):
            v = np.asarray(route[name], dtype=float)
            if v.shape != (2, 3) or not np.isfinite(v).all() or np.linalg.norm(v[1, :2] - v[0, :2]) < .01 or (v[1, 2]-v[0, 2])*(1 if route['direction']=='UP' else -1) <= 0:
                raise ValueError("flight needs surveyed start/end XYZ with the route height direction")
        for name in (("landing_polygon", "exit_polygon", "entry_polygon", "flight_1_polygon", "flight_2_polygon", "footprint") + (("roof_landing_polygon", "flight_3_polygon") if roof else ())):
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
        optional_limits = {"min_flight_v", "flight_prediction_sec", "flight_recheck_sec", "flight_restart_v", "flight_input_recover_sec",
                           "flight_heading_filter_sec", "flight_yaw_kd", "flight_damping_max_w", "flight_capture_angle", "flight_yaw_kp", "flight_centering_lookahead_m",
                           "flight_neutral_alpha", "flight_gyro_rate_max_age", "flight_recovery_w", "flight_lateral_preview_sec", "entry_corridor_yaw", "exit_settle_sec", "flight_stable_sec",
                           "flight_recovery_window_sec", "flight_recovery_window_budget_sec",
                           "flight_recovery_progress_m", "flight_resume_corridor_yaw",
                           "flight_end_advance_m", "flight_end_advance_sec", "flight_end_flat_tolerance_m", "flight_end_height_gap_m"}
        if not keys <= set(limits) or set(limits) - keys - optional_limits or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in limits.values()):
            raise ValueError("route limits require finite positive commissioning values: " + str(sorted(keys)))
        if limits.get("entry_corridor_yaw", .1) >= math.pi/2:
            raise ValueError("entry corridor must face the first flight")
        if limits.get('flight_heading_filter_sec', limits['warn_sec']) > limits['warn_sec']:
            raise ValueError('flight heading filter must not exceed geometry freshness window')
        centering = {'flight_yaw_kp', 'flight_centering_lookahead_m'}
        if centering & set(limits) and (not centering <= set(limits) or 'flight_capture_angle' not in limits):
            raise ValueError('flight centering requires gain, lookahead and capture angle')
        damping = {'flight_yaw_kd', 'flight_damping_max_w', 'flight_gyro_rate_max_age'}
        if damping & set(limits) and not damping <= set(limits):
            raise ValueError('flight yaw damping requires gain, contribution bound and freshness')
        if (limits.get('flight_damping_max_w', limits['max_w']) > limits['max_w'] or
                limits.get('flight_gyro_rate_max_age', limits['warn_sec']) > limits['warn_sec'] or
                limits.get('flight_capture_angle', 1.) >= math.pi/2 or
                limits.get('flight_neutral_alpha', limits['max_alpha']) < limits['max_alpha']):
            raise ValueError('invalid flight steering bounds')
        if 'flight_recovery_w' in limits and (not limits.get('flight_restart_v') or
                not limits.get('flight_recheck_sec') or limits['flight_recovery_w'] > limits['max_w']):
            raise ValueError('flight_recovery_w requires bounded restart/recheck and steering')
        if 'flight_input_recover_sec' in limits and (limits['flight_input_recover_sec'] > limits['recover_sec'] or 'flight_restart_v' not in limits):
            raise ValueError('flight_input_recover_sec requires flight_restart_v and must not exceed recover_sec')
        if limits.get("min_flight_v", 0.) > limits["max_v"]:
            raise ValueError("min_flight_v must not exceed max_v")
        if limits.get('flight_recheck_sec', 0.) > limits['recover_sec']:
            raise ValueError('flight_recheck_sec must not exceed recover_sec')
        if 'flight_restart_v' in limits and (not limits.get('flight_recheck_sec') or
                limits['flight_restart_v'] > limits['max_v'] or
                limits['flight_restart_v'] < limits.get('min_flight_v', 0.)):
            raise ValueError('flight_restart_v requires recheck and must lie between min_flight_v and max_v')
        flight_preview = limits.get("flight_prediction_sec", limits["expire_sec"])
        if (type(flight_preview) not in (int, float) or not math.isfinite(flight_preview) or
                not limits["warn_sec"] <= flight_preview <= limits["expire_sec"]):
            raise ValueError("flight_prediction_sec must be between warn_sec and expire_sec")
        if not (limits["warn_sec"] < limits["expire_sec"] < limits["recover_sec"] and limits["position_off"] < limits["position_on"]):
            raise ValueError("invalid observation budgets or hold hysteresis")
        if not (limits["yaw_deadband"] < limits["yaw_tolerance"] < math.pi and
                limits["lateral_deadband"] < limits["position_off"] and
                limits["speed_deadband"] < limits["max_v"] and limits["hold_v"] <= limits["max_v"]):
            raise ValueError("deadbands must allow reaching arrival tolerances; hold speed must respect max_v")
        continuity = {'flight_stable_sec', 'flight_recovery_window_sec', 'flight_recovery_window_budget_sec', 'flight_recovery_progress_m'}
        if continuity & set(limits) and (not continuity <= set(limits) or
                limits['flight_recovery_window_sec'] <= limits['flight_stable_sec'] or
                limits['flight_recovery_window_budget_sec'] >= limits['flight_recovery_window_sec'] or
                limits['flight_recovery_window_budget_sec'] < limits.get('flight_recheck_sec', 0.)):
            raise ValueError('flight continuity requires bounded stability, rolling budget and progress')
        if limits.get('flight_resume_corridor_yaw', .1) >= math.pi/2:
            raise ValueError('flight resume corridor must face the next flight')
        end_geometry = {'flight_end_advance_m', 'flight_end_advance_sec', 'flight_end_flat_tolerance_m', 'flight_end_height_gap_m'}
        if end_geometry & set(limits) and not end_geometry <= set(limits):
            raise ValueError('flight end continuation requires distance, duration and flat evidence')
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

    def traversal_phases(self, profile):
        roof = 'flight_3' in self.routes[profile.id]
        return tuple(p for p in Phase if roof or p not in (Phase.ROOFTOP_TURN, Phase.FORWARD_SEGMENT_3))

    def test_plan(self, profile, phase, duration, from_entry):
        route = self.routes.get(profile.id)
        if route is None:
            raise ValueError("no LiDAR route for this test")
        phases = StairFeedback.traversal_phases(self, profile)
        phases = phases[:phases.index(phase) + 1] if from_entry else (phase,)
        defaults = ({p.value: 30.0 for p in Phase if p is not Phase.EXIT_CONFIRM}
                    if route["commissioned"] else {Phase.FORWARD_SEGMENT_1.value: 4.0})
        budgets = route.get("phase_test_limits", defaults)
        if any(p.value not in budgets for p in phases):
            raise ValueError("test includes a phase without an explicit route budget")
        if duration > min(sum(budgets[p.value] for p in phases), profile.timeout_sec):
            raise ValueError("test duration exceeds route phase budgets or profile timeout")
        return phases, {p: budgets[p.value] for p in phases}

    @staticmethod
    def entry_supported(route, pose, support, margin):
        if 'entry_corridor_yaw' not in route['limits']:
            return inside_polygon(support, route['entry_polygon'], margin)
        # The first riser is a traversable internal seam, not a lateral drop.
        # Start remains local to its foot; this cannot resume halfway up a flight.
        direction = np.diff(np.asarray(route['flight_1']), axis=0)[0, :2]
        direction = direction / np.linalg.norm(direction)
        first_edge = min(np.asarray(route['flight_1_polygon']) @ direction)
        near_foot = float(pose[:2, 3] @ direction) <= first_edge + margin
        return near_foot and inside_support_union(support,
            [route['entry_polygon'], route['flight_1_polygon']], margin)

    def prepare(self, profile, now, *, phase_test=False, start_phase=None):
        with self._lock:
            route = self.routes.get(profile.id)
            anchor = self.anchors.get(profile.id)
            sample = self.worker.snapshot()
            if route is None or (not route["commissioned"] and not phase_test) or profile.direction.value != route['direction']:
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
                roof_start = start_phase in (Phase.ROOFTOP_TURN, Phase.FORWARD_SEGMENT_3)
                region = route["entry_polygon"] if entry else route["roof_landing_polygon"] if roof_start else route["landing_polygon"]
                height = route["flight_2"][1][2] if roof_start else route["flight_1"][0 if entry else 1][2]
                margin = route["limits"]["margin_m"] + anchor.uncertainty_m
                supported = (self.entry_supported(route, pose, support, margin) if entry
                             else inside_polygon(support, region, margin))
                if (not supported or
                        abs(pose[2, 3] - height) > route["limits"]["height_tolerance"]):
                    raise ValueError("entry geometry rejected: supported=%s body_xyz=(%.3f,%.3f,%.3f) "
                        "height_error=%.3fm allowed=%.3fm; check side/rear support or start level" %
                        (supported, *pose[:3,3], abs(pose[2,3]-height), route['limits']['height_tolerance']))
            return route, anchor

    def arm(self, profile, started_at, *, phase_test=False, start_phase=None):
        with self._lock:
            self.route, self.anchor = self.prepare(profile, started_at, phase_test=phase_test, start_phase=start_phase)
            self.profile = profile
            self.started_at = started_at
            self._arrival_holding = False
            self._exit_hold_target = None
            self._landing_hold_target = None
            self._flight_end_conflict = False
            self._input_gap = None
            self._input_gap_used = 0.
            self._history.clear()
            self._last_sample = None
            self._recovering_at = None
            self._interrupted = False
            self._last_command = (0., 0.)
            self._last_command_at = started_at
            self.turn_index = 0
            self._holding = None
            self._phase_sequence = 0
            self._operator_completed_flights = set()
            self._flight_seen = [False] * (3 if "flight_3" in self.route else 2)
            self.test_status = None
            self._clearance_key = None
            self._first_fault = None
            self._last_geometry_sample = None
            self._entry_alignment_pending = None
            self._entry_verified = False
            self._flight_recheck = None
            self._flight_recheck_used = 0.
            self._candidate_debug = []
            self._reset_continuity()
            self.anchors.pop(profile.id, None)  # a later goal needs new entry evidence

    def can_return_from_entry(self, now):
        """Only a fresh, fully supported pre-ascent flat pose permits WALK.

        The first step/landing and any stale or reset estimate are excluded.
        This does not assert obstacle clearance or hardware posture holding.
        """
        with self._lock:
            if (self.phase not in (Phase.VERIFY_ENTRY, Phase.ALIGN) or
                    self.anchor is None or any(self._flight_seen) or self._interrupted):
                return False
            sample = self.worker.snapshot()
            limits = self.route['limits']
            if (sample is None or not sample.geometry_valid or
                    sample.epoch != self.anchor.epoch or self.worker.epoch != self.anchor.epoch or
                    not 0 <= now - sample.measured_at <= limits['warn_sec']):
                return False
            pose, yaw, pitch, roll, velocity, angular, support = self._observe(sample)
            start = self.route['flight_1'][0]
            return bool(len(self._history) >= 2
                        and np.linalg.norm(velocity[:2]) <= limits['stationary_v']
                        and abs(angular) <= limits['stationary_w']
                        and inside_polygon(support, self.route['entry_polygon'],
                                       limits['margin_m'] + self.anchor.uncertainty_m)
                        and support[:, 0].max() < -self.anchor.uncertainty_m
                        and abs(pose[2, 3] - start[2]) <= min(.05, limits['height_tolerance'])
                        and abs(roll) <= min(math.radians(10), limits['max_roll'])
                        and abs(pitch) <= min(math.radians(10), limits['max_pitch']))

    def begin_arrival_hold(self):
        """Called only after the Supervisor confirmed exit and WALK mode.

        Time spent waiting for a floor/map transaction is not traversal time.
        Sensor age, epoch, support, attitude and command checks still apply.
        """
        with self._lock:
            if self.phase is not Phase.EXIT_CONFIRM or not self._all_flights_completed():
                raise ValueError("arrival hold requires both flights and exit confirmation")
            self._arrival_holding = True

    def _ascent_height_reached(self, measured, target):
        """UP destination height: configured band, or a lower bound only.

        A positive registration/survey bias cannot disprove arrival on a
        fully supported landing. This height predicate alone never completes
        a flight; progress, ascent history and fresh observed support still do.
        Entry validation and captured-surface hold use their separate bands.
        """
        tolerance = self.route['limits']['height_tolerance']
        if self.route['direction'] == 'DOWN':
            return measured <= target + tolerance
        if self.route.get('ascent_height_policy', 'band') == 'minimum':
            return measured >= target - tolerance
        return abs(measured - target) <= tolerance

    def begin_landing_hold(self, now):
        """Capture a fresh, supported landing pose, not a turn waypoint."""
        with self._lock:
            sample = self.worker.snapshot()
            limits = self.route['limits']
            if (self.phase is not Phase.LANDING or self._interrupted or sample is None
                    or not sample.geometry_valid or sample.epoch != self.anchor.epoch
                    or sample.last_geometry_at is None or self.worker.diagnostics()['worker_error']
                    or self.worker.epoch != self.anchor.epoch
                    or not 0 <= now-sample.last_geometry_at <= limits['warn_sec']):
                raise ValueError('landing hold requires fresh same-epoch landing geometry')
            pose, yaw, pitch, roll, _, _, support = self._observe(sample)
            if (not inside_polygon(support, self.route['landing_polygon'], limits['margin_m']+self.anchor.uncertainty_m)
                    or not self._ascent_height_reached(pose[2,3], self.route['flight_1'][1][2])
                    or abs(roll) > limits['max_roll'] or abs(pitch) > limits['max_pitch']):
                raise ValueError('landing hold requires supported landing height and attitude')
            self._landing_hold_target = (pose[:3,3].copy(), yaw)
            self._holding = False
            self.reset_command(now)

    def reset_command(self, now):
        """Match slew history to the zero command actually sent during entry ACK."""
        with self._lock:
            self._last_command, self._last_command_at = (0., 0.), now

    def support_regions(self, phase):
        roof = 'flight_3' in self.route
        if phase is Phase.ROOFTOP_TURN:
            names = ('flight_2_polygon', 'roof_landing_polygon') if self.turn_index == 0 else ('roof_landing_polygon',)
            return names, [self.route[name] for name in names]
        if phase is Phase.FORWARD_SEGMENT_3:
            names = ('roof_landing_polygon', 'flight_3_polygon', 'exit_polygon')
            return names, [self.route[name] for name in names]
        names = {
            Phase.FORWARD_SEGMENT_1: ("entry_polygon", "flight_1_polygon", "landing_polygon"),
            Phase.FORWARD_SEGMENT_2: ("landing_polygon", "flight_2_polygon", "roof_landing_polygon" if roof else "exit_polygon"),
            Phase.VERIFY_ENTRY: (("entry_polygon", "flight_1_polygon") if 'entry_corridor_yaw' in self.route['limits'] else ("entry_polygon",)),
            Phase.ALIGN: (("entry_polygon", "flight_1_polygon") if 'entry_corridor_yaw' in self.route['limits'] else ("entry_polygon",)),
            # The approach still has the first flight behind it. An internal
            # join is not a drop edge when the body pitches/yaws between scans.
            # Actual turn waypoints after the approach stay on the flat landing.
            Phase.LANDING: ("flight_1_polygon", "landing_polygon"),
            Phase.TURN_TO_NEXT_FLIGHT: (("flight_1_polygon", "landing_polygon")
                                       if self.turn_index == 0 else ("landing_polygon",)),
            Phase.EXIT_CONFIRM: ("exit_polygon",),
        }[phase]
        return names, [self.route[name] for name in names]

    def _all_flights_completed(self):
        return all(seen or i in getattr(self, '_operator_completed_flights', set())
                   for i, seen in enumerate(self._flight_seen))

    def operator_phase(self, phase, now):
        """Operator declares earlier flights completed; retain measured route frame."""
        with self._lock:
            if phase not in self.traversal_phases(self.profile):
                raise ValueError('phase unavailable for this route')
            sample = self.worker.snapshot()
            if (sample is None or not sample.geometry_valid or self.anchor is None
                    or sample.epoch != self.anchor.epoch or self.worker.epoch != self.anchor.epoch
                    or not 0 <= now-sample.measured_at <= self.route['limits']['warn_sec']):
                raise ValueError('현재 경로의 최신 LiDAR 위치가 필요합니다. 측위가 복구된 뒤 다시 선택하세요.')
            phases = self.traversal_phases(self.profile)
            flights = (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3)
            self._operator_completed_flights = {i for i,p in enumerate(flights)
                if p in phases and phases.index(p) < phases.index(phase)}
            self._interrupted = False
            self.started_at = now
            self.reset_command(now)
            self._first_fault = None
            self._exit_hold_target = None
            self._arrival_holding = False
            self.begin_phase(phase, now)

    def begin_phase(self, phase, now):
        with self._lock:
            self.phase = phase
            if phase is Phase.TURN_TO_NEXT_FLIGHT:
                self._landing_turn_path = deepcopy(self.route['turn_path'])
                self._landing_turn_captured = False
                self._landing_turn_debug = {}
                self.turn_index = 0
            if phase is Phase.ROOFTOP_TURN:
                self._roof_turn_path = deepcopy(self.route['roof_turn_path'])
                self._landing_turn_captured = False
                self._landing_turn_debug = {}
                self.turn_index = 0
            self._landing_hold_target = None
            self._flight_end_conflict = False
            self._input_gap = None
            self._input_gap_used = 0.
            self._flight_recheck = None
            self._flight_recheck_used = 0.
            self._history.clear()
            self._last_sample = None
            self._reset_continuity()
            sample = self.worker.snapshot()
            self._phase_sequence = sample.sequence if sample else 0
            self._turn_sequence = self._phase_sequence
            self._holding = None
            self._entry_alignment_pending = None

    def entry_imu_context(self):
        """Immutable calibrated IMU window for the existing entry worker job."""
        with self._imu_lock:
            if self._lidar_from_imu is None or not self._imu:
                return None
            last=self._imu[-1][0]
            return dict(epoch=self._imu_epoch,
                        rows=tuple(row for row in self._imu if row[0]>=last-3.),
                        lidar_from_imu=self._lidar_from_imu.tolist(),
                        max_gap=self._imu_max_gap)

    def push_imu(self, row):
        """Mirror the existing IMU stream; no estimator reset or new ROS topic."""
        row = tuple(float(v) for v in row)
        if len(row) != 7 or not all(map(math.isfinite, row)):
            return
        epoch = self.worker.epoch
        with self._imu_lock:
            if epoch != self._imu_epoch:
                self._imu.clear()
                self._imu_epoch = epoch
            if self._imu and row[0] <= self._imu[-1][0]:
                return
            self._imu.append(row)

    def record_sent_twist(self, normalized, now):
        """Called only after the transport's WebSocket send succeeds."""
        with self._tx_lock:
            self._last_sent = dict(normalized_xyz=[float(normalized[k]) for k in ('x','y','z')],
                                   sent_at=now, evidence='websocket_send_returned')

    def _control_heading(self, sample, pose, now):
        """Short causal gyro rotation, retaining the original geometry timestamp.

        No accelerometer translation, future samples or extrapolation past the
        latest received gyro. Missing coverage falls back to measured heading.
        """
        yaw = math.atan2(pose[1, 0], pose[0, 0])
        detail = dict(source='geometry', geometry_stamp=sample.stamp,
                      geometry_age_sec=now-sample.measured_at, gyro_advanced_sec=0.)
        with self._imu_lock:
            rows = np.asarray([r for r in self._imu if sample.stamp-self._imu_max_gap <= r[0]
                               <= sample.stamp + max(0., now-sample.measured_at)])
            epoch = self._imu_epoch
        if self._lidar_from_imu is None or epoch != sample.epoch or len(rows) < 2:
            return yaw, detail
        start = sample.stamp
        i = np.searchsorted(rows[:, 0], start, side='right')-1
        if i < 0 or rows[-1, 0] <= start:
            return yaw, detail
        rows = rows[i:]
        if np.max(np.diff(rows[:, 0])) > self._imu_max_gap:
            detail['reason'] = 'gyro gap; measured heading retained'
            return yaw, detail
        times = np.r_[start, rows[rows[:, 0] > start, 0]]
        gyro = np.column_stack([np.interp(times, rows[:, 0], rows[:, k]) for k in (1,2,3)])
        rotation = np.eye(3)
        for a, b, dt in zip(gyro, gyro[1:], np.diff(times)):
            rotation = rotation @ Rotation.from_rotvec(self._lidar_from_imu @ ((a+b)*.5) * dt).as_matrix()
        # Transform body -> lidar -> gyro-updated lidar -> body. Tilt is retained.
        body_rotation = pose[:3,:3] @ self.base_from_lidar[:3,:3] @ rotation @ self.base_from_lidar[:3,:3].T
        heading = math.atan2(body_rotation[1,0], body_rotation[0,0])
        detail.update(source='causal_gyro', gyro_advanced_sec=float(times[-1]-start),
                      gyro_remaining_age_sec=float(sample.stamp + now-sample.measured_at-times[-1]),
                      yaw_delta=wrap(heading-yaw))
        # Heading derivative of the tilted body X axis, not raw sensor Z.
        # A short received-data average suppresses individual impact samples;
        # no future IMU, extrapolation, or stale-rate reuse is permitted.
        omega = np.mean(gyro[times >= times[-1]-.04], axis=0)
        world_omega = pose[:3,:3] @ self.base_from_lidar[:3,:3] @ rotation @ self._lidar_from_imu @ omega
        body_x = body_rotation[:, 0]
        derivative = np.cross(world_omega, body_x)
        horizontal = float(body_x[0]**2 + body_x[1]**2)
        if horizontal > .1:
            detail['yaw_rate'] = float((body_x[0]*derivative[1]-body_x[1]*derivative[0])/horizontal)
            detail['yaw_rate_age_sec'] = detail['gyro_remaining_age_sec']
        return heading, detail

    def _flight_heading(self, sample, yaw, control_yaw, detail, now, degraded, observed_feedback):
        """Time-aligned, lightly filtered heading for steering only.

        Observed support, arrival and attitude keep the original LiDAR pose.
        Gyro integration is anchored at each scan; no long-term IMU yaw or
        extrapolation beyond received IMU data is introduced here.
        """
        tau = self.route['limits'].get('flight_heading_filter_sec')
        if tau is None or not observed_feedback or degraded:
            self._flight_heading_state = None
            return (yaw if observed_feedback else control_yaw), dict(
                source='geometry' if observed_feedback else detail['source'],
                gyro_used=not observed_feedback and detail['source']=='causal_gyro', filtered=False)
        remaining = detail.get('gyro_remaining_age_sec', math.inf)
        gyro_used = (detail.get('source') == 'causal_gyro'
                     and math.isfinite(control_yaw) and math.isfinite(remaining)
                     and 0 <= remaining <= self.route['limits'].get('flight_gyro_rate_max_age', 0.))
        raw = control_yaw if gyro_used else yaw
        prior = getattr(self, '_flight_heading_state', None)
        reset = (prior is None or prior['epoch'] != sample.epoch
                 or sample.stamp < prior['stamp'] or now < prior['at']
                 or now-prior['at'] > self.route['limits']['warn_sec'])
        if reset:
            filtered = raw
        else:
            alpha = -math.expm1(-max(0.,now-prior['at'])/tau)
            filtered = wrap(prior['yaw']+alpha*wrap(raw-prior['yaw']))
        self._flight_heading_state = dict(epoch=sample.epoch, stamp=sample.stamp,
                                         at=now, yaw=filtered)
        return filtered, dict(source='causal_gyro' if gyro_used else 'geometry',
            gyro_used=gyro_used, raw_yaw=raw, filtered_yaw=filtered,
            filter_sec=tau, filtered=True, reset=reset)

    def _flight_steering(self, epsi, preview_ey, heading_detail):
        limits = self.route['limits']
        capture_heading = None
        capture_error = epsi
        if 'flight_centering_lookahead_m' in limits:
            # A bounded inward heading acts before the body reaches the edge.
            # Once already heading inward sufficiently, unwind the correction.
            capture_heading = -float(np.clip(math.atan2(
                deadband(preview_ey, limits['lateral_deadband']),
                limits['flight_centering_lookahead_m']),
                -limits['flight_capture_angle'], limits['flight_capture_angle']))
            capture_error = wrap(epsi - capture_heading)
            proportional = -limits['flight_yaw_kp']*deadband(capture_error, limits['yaw_deadband'])
        else:
            lateral = limits['lateral_kp'] * deadband(preview_ey, limits['lateral_deadband'])
            if 'flight_capture_angle' in limits:
                lateral = float(np.clip(lateral, -limits['yaw_kp']*limits['flight_capture_angle'],
                                        limits['yaw_kp']*limits['flight_capture_angle']))
            proportional = -limits['yaw_kp']*deadband(epsi, limits['yaw_deadband'])-lateral
        rate = heading_detail.get('yaw_rate')
        age = heading_detail.get('yaw_rate_age_sec', math.inf)
        damping = 0.
        if ('flight_yaw_kd' in limits and rate is not None and math.isfinite(rate)
                and 0 <= age <= limits['flight_gyro_rate_max_age']):
            damping = float(np.clip(-limits['flight_yaw_kd']*rate,
                -limits['flight_damping_max_w'], limits['flight_damping_max_w']))
        return proportional+damping, dict(proportional_w=proportional, damping_w=damping,
                                         capture_heading=capture_heading, capture_error=capture_error,
                                         measured_yaw_rate=rate, rate_age_sec=age if math.isfinite(age) else None)

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
            report = self._evaluate(phase, now, command_required=command_required, neutral_only=neutral_only)
            if report.faulted and self._first_fault is None:
                sample = self.worker.snapshot()
                self._first_fault = deepcopy(dict(self.debug, reason=report.detail, phase=phase.value,
                    at=now, sample_stamp=None if sample is None else sample.stamp,
                    sample_sequence=None if sample is None else sample.sequence,
                    command_before_fault=list(self._last_command), candidates=self._candidate_debug,
                    observation=self._observation_debug))
            return report

    def _evaluate(self, phase, now, *, command_required=False, neutral_only=False):
        with self._lock:
            sample = self.worker.snapshot()
            self._candidate_debug = []
            self._observation_debug = {}
            limits = self.route["limits"]
            observed_feedback = self.route.get('flight_command_policy') == 'observed_feedback'
            landing_hold = getattr(self, '_landing_hold_target', None) is not None and phase is Phase.LANDING
            holding_exit = (getattr(self, '_arrival_holding', False) and phase is Phase.EXIT_CONFIRM) or landing_hold
            if not self._interrupted and not holding_exit and now - self.started_at > self.profile.timeout_sec:
                self.debug = dict(tracking_state="TIMEOUT", loss_response=True)
                return EvidenceReport(phase, False, True, "LiDAR traversal/hold timeout", 0, 0, math.inf)
            age = math.inf if sample is None or sample.last_geometry_at is None else now - sample.last_geometry_at
            reset = self.worker.epoch != self.anchor.epoch or sample is not None and sample.epoch != self.anchor.epoch
            expired = sample is None or age >= limits["expire_sec"] or age < 0 or bool(self.worker.diagnostics()["worker_error"])
            input_resume = False
            # Only a short same-epoch input delay during an already moving
            # flight is recoverable. Missing geometry, clock/worker failures,
            # interruption and epoch changes retain the existing fault path.
            input_eligible = (phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3)
                and not reset and not self._interrupted and sample is not None
                and math.isfinite(age) and age >= 0
                and not self.worker.diagnostics()['worker_error']
                and not self._flight_end_conflict
                and not self._entry_alignment_pending
                and limits.get('flight_input_recover_sec', 0.) > 0)
            if input_eligible and self._input_gap is None and expired and self._last_command[0] > 0:
                self._input_gap = dict(at=now, sequence=sample.sequence)
            if input_eligible and self._input_gap is not None:
                elapsed = self._input_gap_used + max(0., now-self._input_gap['at'])
                exhausted = (elapsed >= limits['flight_input_recover_sec'] or
                             self._rolling_recovery(now, self._input_gap['at']) >= limits.get('flight_recovery_window_budget_sec', math.inf))
                fresh = (sample.geometry_valid and age <= limits['warn_sec']
                         and sample.sequence > self._input_gap['sequence'])
                if exhausted or not fresh:
                    self._last_command, self._last_command_at = (0., 0.), now
                    self.debug = dict(tracking_state='RECOVERING', loss_response=exhausted,
                        age=age, input_recovery=True, input_recovery_elapsed_sec=elapsed,
                        input_recovery_budget_sec=limits['flight_input_recover_sec'],
                        physical_hold_verified=False)
                    return EvidenceReport(phase, False, exhausted,
                        'input recovery budget exhausted' if exhausted else
                        'input delayed: zero command; waiting for fresh geometry', 0, 0, age)
                self._recovery_events.append((self._input_gap['at'], now))
                self._input_gap_used = elapsed
                self._input_gap = None
                input_resume = True  # all geometry/attitude/path checks below still apply
            if reset or expired or self._interrupted:
                self._recovering_at = now if self._recovering_at is None else self._recovering_at
                overdue = now - self._recovering_at >= limits["recover_sec"]
                self.debug = dict(tracking_state="HANDOFF_OVERDUE" if overdue else "RECOVERING",
                                  loss_response=True, anchor_reset=reset, age=age,
                                  physical_handoff_overdue=overdue)
                # Once a physical loss response is selected, automatic restart
                # needs re-arming its mode and is intentionally not implicit.
                return EvidenceReport(phase, False, True, "tracking expired/reset; commissioned loss response required", 0, 0, age)
            degraded = age > limits['warn_sec'] or not sample.geometry_valid
            if sample.geometry_valid:
                self._last_geometry_sample = sample
            elif self._last_geometry_sample is not None:
                sample = self._last_geometry_sample
            else:
                self.debug = dict(tracking_state='DEGRADED', loss_response=False, age=age)
                return EvidenceReport(phase, False, False, 'waiting for usable geometry', 0, 0, age)
            pose, yaw, pitch, roll, velocity, angular, support = self._observe(sample)
            control_yaw, heading_detail = self._control_heading(sample, pose, now)
            self._observation_debug = dict(pose=pose[:3,3].tolist(), yaw=yaw, pitch=pitch, roll=roll,
                geometry_stamp=sample.stamp, geometry_sequence=sample.sequence,
                geometry_measured_at=sample.measured_at, control_yaw=control_yaw,
                heading=heading_detail, measured_velocity=velocity.tolist(), measured_yaw_rate=angular)
            self._recovering_at = None
            self._prediction_context = (sample, pose, velocity, angular, control_yaw, heading_detail, now)
            pos = pose[:3, 3]
            if landing_hold and abs(float(pos[2]-self._landing_hold_target[0][2])) > limits['height_tolerance']:
                self.debug = dict(tracking_state='LANDING_HOLD_HEIGHT', loss_response=True,
                                  landing_position_hold=True, command=[0.,0.])
                return EvidenceReport(phase, False, True, 'landing hold height changed from captured surface', 0.,0.,age)
            margin = limits["margin_m"] + self.anchor.uncertainty_m
            landing = inside_polygon(support, self.route["landing_polygon"], margin)
            exit_clear = inside_polygon(support, self.route["exit_polygon"], margin)
            complete = False
            settling = None
            v, w, ey, epsi, progress = 0., 0., 0., 0., 0.
            alignment = False
            flight_end_conflict = False
            steering_detail = {}
            flight_heading_detail = {}
            if abs(roll) > limits["max_roll"] or abs(pitch) > limits["max_pitch"]:
                self.debug = dict(tracking_state="ATTITUDE_LIMIT", loss_response=True, roll=roll, pitch=pitch)
                return EvidenceReport(phase, False, True, "commissioned attitude envelope exceeded", 0, 0, age)
            if phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3):
                idx = {Phase.FORWARD_SEGMENT_1: 0, Phase.FORWARD_SEGMENT_2: 1, Phase.FORWARD_SEGMENT_3: 2}[phase]
                start, end = np.asarray(self.route["flight_%d" % (idx + 1)])
                direction = end[:2] - start[:2]
                length = np.linalg.norm(direction)
                direction = direction / length
                target, target_yaw = end, math.atan2(direction[1], direction[0])
                progress = float((pos[:2] - start[:2]) @ direction)
                ey = float(direction[0] * (pos[1] - start[1]) - direction[1] * (pos[0] - start[0]))
                steering_yaw, flight_heading_detail = self._flight_heading(
                    sample, yaw, control_yaw, heading_detail, now, degraded, observed_feedback)
                epsi = wrap(steering_yaw - math.atan2(direction[1], direction[0]))
                actual_v = float(velocity[:2] @ direction)
                nominal = min(self.profile.linear_speed, limits["max_v"])
                # A nominal input is not a measured target velocity in a response
                # model. Retain the legacy speed loop only for legacy routes.
                v = (nominal if self.route.get('motion_prediction', {}).get('mode') == 'control' else
                     np.clip(nominal + limits["speed_kp"] * deadband(nominal - actual_v, limits["speed_deadband"]), 0, limits["max_v"]))
                # Smooth slowdown proportional to error, never a normal-noise stop gate.
                v /= 1 + abs(epsi) + abs(ey)
                # Operator's useful forward-command floor, in the transport's
                # input units. Slew and geometric feasibility still take priority.
                v = max(v, limits.get("min_flight_v", 0.))
                if 'flight_input_v' in self.route:
                    v = self.route['flight_input_v'][idx]
                w = -limits["yaw_kp"] * deadband(epsi, limits["yaw_deadband"]) - limits["lateral_kp"] * deadband(ey, limits["lateral_deadband"])
                # Preview measured sideways motion without rewriting pose or
                # treating normalized input as a measured physical velocity.
                lateral_rate = float(direction[0]*velocity[1]-direction[1]*velocity[0])
                preview_ey = ey + lateral_rate*min(age, limits['warn_sec'])
                # Anticipate outward drift while clearance is still available.
                # Cap extra offset by the existing position band, and keep the
                # existing capture-angle, angular magnitude and slew limits.
                extra = 0.
                if ey*lateral_rate > 0 and not degraded:
                    extra = float(np.clip(lateral_rate*limits.get('flight_lateral_preview_sec', 0.),
                                          -limits['position_on'], limits['position_on']))
                    preview_ey += extra
                w, steering_detail = self._flight_steering(epsi, preview_ey, heading_detail)
                steering_detail.update(lateral_rate=lateral_rate, preview_ey=preview_ey, extra_preview_m=extra)
                if idx > 0:
                    if self._entry_alignment_pending is None:
                        flat_start = landing if idx == 1 else inside_polygon(support, self.route['roof_landing_polygon'], margin)
                        self._entry_alignment_pending = flat_start and (abs(ey) > limits['position_off'] or abs(epsi) > limits['yaw_tolerance'])
                    if self._entry_alignment_pending:
                        waypoint = self.route['turn_path' if idx == 1 else 'roof_turn_path'][-1]
                        delta = np.asarray(waypoint[:2])-pos[:2]
                        aligned = self._resume_corridor(idx, pose, support, margin, epsi) if 'flight_resume_corridor_yaw' in limits else (np.linalg.norm(delta) <= limits['position_off'] and abs(epsi) <= limits['yaw_tolerance'])
                        if aligned and not degraded:
                            self._entry_alignment_pending = False
                        else:
                            alignment = True
                            heading = np.array([math.cos(control_yaw), math.sin(control_yaw)])
                            along = float(delta @ heading)
                            distance = float(np.linalg.norm(delta))
                            if distance > limits['position_off']:
                                bearing = math.atan2(delta[1],delta[0])
                                approach = wrap(bearing-control_yaw-(math.pi if along < 0 else 0.))
                                v = float(np.clip(limits['hold_kp']*along-limits['hold_kd']*(velocity[:2]@heading), -limits['hold_v'], limits['hold_v']))
                                w = limits['yaw_kp']*deadband(approach,limits['yaw_deadband'])
                            else:
                                v = 0.
                                w = -limits['yaw_kp']*deadband(epsi,limits['yaw_deadband'])
                            target = np.array([*waypoint[:2], pos[2]])
                height_sign = 1. if self.route['direction']=='UP' else -1.
                if height_sign*(pos[2] - start[2]) >= height_sign*(end[2] - start[2]) * .5:
                    self._flight_seen[idx] = True
                checks = dict(progress=progress >= length, ascent_seen=self._flight_seen[idx],
                              height=self._ascent_height_reached(pos[2], end[2]),
                              body_in_next_region=(landing if idx == 0 else
                                  inside_polygon(support, self.route['roof_landing_polygon'], margin)
                                  if idx == 1 and 'flight_3' in self.route else exit_clear))
                complete = all(checks.values())
                # Reaching the surveyed horizontal end does not prove that the
                # last riser is cleared. It also cannot justify driving farther
                # at ascent input solely to chase an inconsistent Z estimate.
                # Retain the phase and its strict completion evidence; neutral
                # commands use the existing freshness/support/timeout handling.
                self._flight_end_conflict |= bool(checks['progress'] and checks['ascent_seen']
                                       and checks['body_in_next_region']
                                       and not checks['height'] and not alignment)
                end_continuation = False
                if self._flight_end_conflict and not complete:
                    end_continuation = self._end_geometry_continuation(idx, sample, pose, pitch, roll, now, degraded)
                flight_end_conflict = self._flight_end_conflict and not complete and not end_continuation
                if flight_end_conflict:
                    v, w = 0., 0.
                if alignment:
                    complete = False
                    checks['second_flight_aligned'] = False
            else:
                if phase in (Phase.VERIFY_ENTRY, Phase.ALIGN):
                    target = np.asarray(self.route["flight_1"])[0]
                    vector = np.diff(np.asarray(self.route["flight_1"]), axis=0)[0]
                    target_yaw = math.atan2(vector[1], vector[0])
                    supported = self.entry_supported(self.route, pose, support, margin)
                elif phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN):
                    roof_turn = phase is Phase.ROOFTOP_TURN
                    if phase in (Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN):
                        previous_index = self.turn_index
                        self._capture_landing_turn(pose, margin, degraded or sample.sequence <= self._turn_sequence)
                        if self.turn_index != previous_index:
                            self._turn_sequence = sample.sequence
                    path = (self._landing_path() if roof_turn else
                            self._landing_path() if phase is Phase.TURN_TO_NEXT_FLIGHT else self.route['turn_path'])
                    waypoint = path[self.turn_index]
                    level = self.route['flight_2' if roof_turn else 'flight_1'][1][2]
                    target = np.array([*waypoint[:2], level])
                    target_yaw = waypoint[2]
                    supported = inside_polygon(support, self.route['roof_landing_polygon'], margin) if roof_turn else landing
                else:
                    last_flight = "flight_3" if "flight_3" in self.route else "flight_2"
                    target = np.asarray(self.route[last_flight])[1]
                    vector = np.diff(np.asarray(self.route[last_flight]), axis=0)[0]
                    target_yaw, supported = math.atan2(vector[1], vector[0]), exit_clear
                region_exit = phase is Phase.EXIT_CONFIRM and self.route.get('exit_arrival_policy') == 'supported_region'
                if region_exit:
                    exit_yaw_tolerance = limits.get('entry_corridor_yaw', limits['yaw_tolerance'])
                    if (self._exit_hold_target is None and exit_clear and self._all_flights_completed()
                            and self._ascent_height_reached(pos[2], target[2])
                            and abs(wrap(yaw-target_yaw)) <= exit_yaw_tolerance
                            and not degraded and sample.sequence > self._phase_sequence):
                        self._exit_hold_target = (pos.copy(), yaw)
                    if self._exit_hold_target is not None:
                        target, target_yaw = self._exit_hold_target
                if landing_hold:
                    target, target_yaw = self._landing_hold_target
                    supported = landing
                region_turn = self._region_turn()
                rotation_waypoint = region_turn and self._landing_rotation_waypoint()
                evaluated_turn_index = self.turn_index
                if rotation_waypoint:
                    # Rotation has an orientation target, not a historical XY
                    # hold target. Drift is checked against observed support.
                    target[:2] = pos[:2]
                delta = target[:2] - pos[:2]
                distance = float(np.linalg.norm(delta))
                verified_entry_handoff = phase is Phase.ALIGN and self._entry_verified
                position_limit = (limits['position_on'] if verified_entry_handoff
                                  else limits['position_off'])
                if phase in (Phase.VERIFY_ENTRY, Phase.ALIGN):
                    # A verified entry needs one fresh aligned observation,
                    # not a second stationary dwell on a creeping platform.
                    # Standalone ALIGN still uses the original precise gate.
                    self._holding = distance > position_limit
                else:
                    if self._holding is None:
                        self._holding = distance > limits['position_off']
                    if distance >= limits["position_on"]:
                        self._holding = True
                    elif distance <= limits["position_off"]:
                        self._holding = False
                heading = np.array([math.cos(yaw), math.sin(yaw)])
                along = float(delta @ heading)
                epsi = wrap(yaw - target_yaw)
                if self._holding or landing_hold:
                    # A small bounded unicycle arc, with reverse only on surveyed
                    # support. Each proposed step is checked below against edges.
                    v = np.clip(limits["hold_kp"] * along - limits["hold_kd"] * (velocity[:2] @ heading), -limits["hold_v"], limits["hold_v"])
                    lateral = heading[0] * delta[1] - heading[1] * delta[0]
                    w = -limits["yaw_kp"] * deadband(epsi, limits["yaw_deadband"]) + limits["lateral_kp"] * deadband(lateral, limits["lateral_deadband"]) * (1 if v >= 0 else -1)
                else:
                    w = -limits["yaw_kp"] * deadband(epsi, limits["yaw_deadband"])
                corridor_entry = phase in (Phase.VERIFY_ENTRY, Phase.ALIGN) and 'entry_corridor_yaw' in limits
                if corridor_entry:
                    # No retreat to the nominal -45cm point, including first-step
                    # contact. Heading is aligned here; the flight controller
                    # captures lateral offset on its already-checked path.
                    self._holding = False
                    v = 0.
                    w = 0. if phase is Phase.VERIFY_ENTRY else -limits['yaw_kp']*deadband(epsi, limits['yaw_deadband'])
                if region_turn:
                    # A bounded capture region replaces exact-point settling;
                    # the normal observed support and command checks still run.
                    position_limit = (limits['position_on'] if self.turn_index < len(self._landing_path())-1
                                      else limits['position_off'])
                    if distance <= position_limit:
                        self._holding = False
                        v = 0.
                        w = -limits['yaw_kp']*deadband(epsi, limits['yaw_deadband'])
                at_target = supported and distance <= position_limit and abs(epsi) <= limits["yaw_tolerance"]
                settled = self._settled(sample, average_rate=phase in (Phase.VERIFY_ENTRY, Phase.ALIGN))
                settling = dict(self._settling_detail)
                if phase is Phase.VERIFY_ENTRY:
                    # Verify the physical start region and measured height.
                    # Precise position and yaw belong to the following ALIGN
                    # phase, which can actively correct them before ascent.
                    if corridor_entry:
                        # A creeping platform need not become stationary to
                        # enter ALIGN. Keep the anchor uncertainty hard; ALIGN
                        # must restore the extra reserve before ascent.
                        supported = supported or self.entry_supported(self.route, pose, support,
                                                                      self.anchor.uncertainty_m)
                    checks = dict(body_in_region=supported,
                                  height=abs(pos[2] - target[2]) <= limits["height_tolerance"])
                    if not corridor_entry:
                        checks['settled'] = settled
                elif corridor_entry:
                    checks = dict(body_in_region=supported,
                                  yaw=abs(epsi) <= limits['entry_corridor_yaw'],
                                  height=abs(pos[2]-target[2]) <= limits['height_tolerance'],
                                  entry_verified=verified_entry_handoff or settled)
                elif verified_entry_handoff:
                    checks = dict(body_in_region=supported, position=distance <= position_limit,
                                  yaw=abs(epsi) <= limits["yaw_tolerance"],
                                  height=abs(pos[2] - target[2]) <= limits["height_tolerance"],
                                  entry_verified=True)
                else:
                    checks = dict(body_in_region=supported, position=distance <= position_limit,
                                  yaw=abs(epsi) <= limits["yaw_tolerance"])
                    if rotation_waypoint:
                        checks.pop('position')
                    if not region_turn:
                        checks['settled'] = settled
                    elif self.turn_index+1 < len(self._landing_path()):
                        path = self._landing_path()
                        if np.allclose(path[self.turn_index][:2], path[self.turn_index+1][:2]):
                            checks['rotation_space'] = self._landing_turn_debug.get('rotation_disk_supported', False)
                if phase is Phase.ROOFTOP_TURN:
                    checks['upper_landing_height'] = self._ascent_height_reached(pos[2], target[2])
                if region_exit:
                    checks = dict(body_in_region=exit_clear,
                                  yaw=abs(epsi)<=exit_yaw_tolerance,
                                  destination_captured=self._exit_hold_target is not None)
                if region_turn:
                    if self.turn_index == len(self._landing_path())-1:
                        # Handoff to the next flight from its supported entry
                        # corridor; do not chase a 5 cm point across a seam.
                        next_idx = 2 if phase is Phase.ROOFTOP_TURN else 1
                        checks.pop('position', None)
                        checks.pop('yaw', None)
                        a,b=np.asarray(self.route['flight_%d'%(next_idx+1)])
                        direction=b[:2]-a[:2];direction/=np.linalg.norm(direction)
                        near_entry=float((pos[:2]-target[:2]) @ direction)>=-limits['position_on']
                        checks['next_flight_corridor'] = near_entry and self._resume_corridor(next_idx, pose, support, margin, epsi)
                    checks['new_waypoint_observation'] = sample.sequence > self._turn_sequence
                complete = all(checks.values())
                if phase in (Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN) and complete:
                    path = self._landing_path()
                    if self.turn_index + 1 < len(path) and not degraded and sample.sequence > self._phase_sequence:
                        if rotation_waypoint:
                            self._connect_landing_leg(pos[:2], margin)
                        self.turn_index += 1
                        self._turn_sequence = sample.sequence
                        self._history.clear()
                        complete = False
                if region_turn:
                    self._landing_turn_debug.update(evaluated_index=evaluated_turn_index,
                        waypoint_kind='rotation' if rotation_waypoint else 'translation',
                        completed=all(checks.values()) and not degraded and sample.sequence > self._phase_sequence,
                        position_target_required=not rotation_waypoint)
                if phase is Phase.LANDING:
                    complete = landing  # proceed directly to measured landing path
                    checks = dict(body_in_landing=landing)
                if phase is Phase.EXIT_CONFIRM:
                    final_height = self.route['flight_3' if 'flight_3' in self.route else 'flight_2'][1][2]
                    complete = complete and self._all_flights_completed() and self._ascent_height_reached(pos[2], final_height)
                    checks.update(final_height=self._ascent_height_reached(pos[2], final_height), all_flights_completed=self._all_flights_completed(), all_flights_seen=all(self._flight_seen))
                    checks.update(flight_1_seen=self._flight_seen[0], flight_2_seen=self._flight_seen[1])
            steering_target_w = float(w)  # desired target; never an already-sent command
            checks["new_phase_observation"] = sample.sequence > self._phase_sequence
            if sample.sequence <= self._phase_sequence or degraded:
                complete = False
            names, polygons = self.support_regions(phase)
            if landing_hold:
                names, polygons = ('landing_polygon',), [self.route['landing_polygon']]
            if alignment:
                names, polygons = ('landing_polygon',), [self.route['landing_polygon']]
            raw_supported = inside_support_union(support, polygons)
            current_supported = inside_support_union(support, polygons, margin)
            key = (sample.epoch, sample.sequence, phase, names)
            if key != self._clearance_key:
                self._clearance_key = key
                self._clearance_lower = support_clearance(support, polygons) if raw_supported else None
            geometry = dict(support_regions=list(names), footprint_supported=raw_supported,
                            current_supported=current_supported, margin_m=limits["margin_m"],
                            anchor_uncertainty_m=self.anchor.uncertainty_m, effective_margin_m=margin,
                            clearance_m=self._clearance_lower,
                            height_error_m=abs(float(pos[2] - target[2])),
                            height_delta_m=float(pos[2] - target[2]),
                            ascent_height_policy=self.route.get('ascent_height_policy', 'band'),
                            ascent_height_min_m=float(target[2] - limits['height_tolerance']),
                            completion_checks=checks,
                            settling=settling,
                            incomplete_conditions=[k for k, value in checks.items() if not value])
            # The surveyed anchor uncertainty remains hard. Only the extra
            # reserve margin may be recovered, with an inward checked path.
            recovery = not current_supported and inside_support_union(support, polygons, self.anchor.uncertainty_m)
            if not current_supported and not recovery:
                self.debug = dict(tracking_state="CORRIDOR_LIMIT", loss_response=True, **geometry)
                return EvidenceReport(phase, False, True, "body/clearance exceeds connected support", 0, 0, age)
            # A completed phase emits no command in the Supervisor loop. Do not
            # reject its transition using the old command's future trajectory.
            transition = complete and not command_required and not neutral_only
            if neutral_only or flight_end_conflict:
                v, w = 0., 0.
            elif not transition:
                v, w = self._limit_command(float(v), float(w), now)
                if degraded:
                    # A late scan cannot authorize acceleration. With no newer
                    # gyro either, recheck the prior command instead of inventing
                    # additional steering from an unchanged observation.
                    v = math.copysign(min(abs(v), abs(self._last_command[0])), v)
                    if heading_detail['source'] == 'geometry':
                        v, w = self._last_command
                if phase in (Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN) and rotation_waypoint:
                    v = 0.
                if alignment or phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN, Phase.EXIT_CONFIRM):
                    v = float(np.clip(v, -limits['hold_v'], limits['hold_v']))
            # Sensor expiry is a data-freshness deadline, not the flight
            # lookahead. Every control cycle rechecks the observed full body.
            on_flight = (phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3)
                         and not alignment and not flight_end_conflict)
            horizon = (limits.get("flight_prediction_sec", limits["expire_sec"])
                       if on_flight else limits["expire_sec"])
            proposed = (v, w)
            adjusted = False
            restart = False
            flight = on_flight and not (degraded and heading_detail['source'] == 'geometry')
            if not transition and on_flight and not observed_feedback and self._flight_recheck is not None and not flight:
                return self._recheck_flight(phase, sample, now, age, pos, yaw, control_yaw,
                                           support, polygons, margin, horizon, geometry, proposed)
            if not transition and on_flight and observed_feedback:
                # Restore the configured useful stair input only after fresh
                # same-epoch geometry and all observed support checks passed.
                # Keep angular slew from _limit_command; never jump steering.
                if input_resume and not neutral_only and not degraded:
                    v = self.route['flight_input_v'][idx] if 'flight_input_v' in self.route else limits['flight_restart_v']
                    proposed = (v, w)
                    restart = True
                # Position and body orientation come from one LiDAR observation.
                # The uncommissioned command-scale future model is diagnostic;
                # it cannot replace this observed support gate with a terminal
                # prediction fault. Freshness, attitude, observed body support,
                # completion checks and command/slew limits still run above.
                self._flight_recheck = None
            elif not transition and not neutral_only and flight:
                pending = self._flight_recheck
                if pending is not None and (degraded or sample.sequence <= pending['sequence'] or
                        self._flight_recheck_used + now - pending['at'] >= limits.get('flight_recheck_sec', 0.) or
                        self._rolling_recovery(now, pending['at']) >= limits.get('flight_recovery_window_budget_sec', math.inf)):
                    return self._recheck_flight(phase, sample, now, age, pos, yaw, control_yaw,
                                               support, polygons, margin, horizon, geometry, proposed)
                restart = (pending is not None or input_resume) and 'flight_restart_v' in limits
                if restart:
                    # Only a newer, fresh observation reaches this branch.
                    # Validate the exact useful input, not a low-speed path
                    # followed by an unchecked output jump. Steering still
                    # obeys the ordinary magnitude and slew limits.
                    v = self.route['flight_input_v'][idx] if 'flight_input_v' in self.route else limits['flight_restart_v']
                    proposed = (v, w)
                selected = self._flight_command(pos[:2], yaw, control_yaw, support, v, w,
                    horizon, polygons, margin, now, direction, start[:2], recovery,
                    restart=restart)
                if selected is None:
                    if limits.get('flight_recheck_sec', 0.) > 0.:
                        correction = None
                        if (pending is not None and not degraded and 'flight_recovery_w' in limits):
                            correction = self._flight_recovery(pos[:2], yaw, control_yaw, support,
                                polygons, margin, horizon, now, steering_target_w, target_yaw)
                        report = self._recheck_flight(phase, sample, now, age, pos, yaw, control_yaw,
                                                   support, polygons, margin, horizon, geometry, proposed)
                        # Explicit commissioning opt-in only: ability to turn
                        # on stair treads cannot be inferred from an offline bag.
                        if (not report.faulted and pending is not None and not degraded
                                and 'flight_recovery_w' in limits):
                            if correction is not None:
                                self._last_command, self._last_command_at = correction, now
                                self._flight_recheck['sequence'] = sample.sequence
                                self._flight_recheck['correction'] = correction
                                self._flight_recheck['correction_target_w'] = float(np.clip(steering_target_w, -limits['flight_recovery_w'], limits['flight_recovery_w']))
                                self.debug.update(command=list(correction), recovery_steering=True,
                                                  physical_hold_verified=False)
                                return EvidenceReport(phase, False, False,
                                    'bounded commissioned steering recovery; arrival unconfirmed', 0, 0, age)
                        return report
                    self.debug = dict(tracking_state='SUPPORT_LIMIT', loss_response=True, pose=pos.tolist(),
                        yaw=yaw, control_heading=heading_detail, proposed_command=[v,w],
                        prediction_horizon_sec=horizon, **geometry)
                    return EvidenceReport(phase, False, True, 'no bounded correction within connected support', 0, 0, age)
                v, w = selected
                if pending is not None:
                    self._recovery_events.append((pending['at'], now))
                    self._flight_recheck_used += max(0., now - pending['at'])
                    self._flight_recheck = None
                adjusted = recovery or any(row['accepted'] is False for row in self._candidate_debug)
            elif (not transition and on_flight and not neutral_only
                  and self.route.get('motion_prediction', {}).get('mode') == 'control'):
                if self._response_supported(support,yaw,control_yaw,v,w,horizon,polygons,margin)[0] is not True:
                    return self._recheck_flight(phase,sample,now,age,pos,yaw,control_yaw,
                                               support,polygons,margin,horizon,geometry,proposed)
            elif not transition and (recovery or not self._swept_support(pos[:2], yaw, support, v, w, horizon, polygons, margin)):
                entry_reserve = (phase is Phase.ALIGN and corridor_entry and recovery
                                 and not degraded and not neutral_only
                                 and abs(pos[2]-target[2]) <= limits['height_tolerance'])
                alternative = (self._entry_reserve_command(pos[:2], yaw, support, w,
                    horizon, polygons, margin, now) if entry_reserve else
                    None if neutral_only or flight_end_conflict else self._supported_command(
                    pos[:2], yaw, support, v, w, horizon, polygons, margin, now))
                if alternative is None:
                    self.debug = dict(tracking_state="SUPPORT_LIMIT", loss_response=True,
                                      proposed_command=[v, w], prediction_horizon_sec=horizon, **geometry)
                    return EvidenceReport(phase, False, True, "no bounded correction within connected support", 0, 0, age)
                v, w = alternative
                adjusted = True
            if not transition:
                if on_flight and not degraded and v > 0. and not recovery and self._flight_recheck is None and not input_resume and not restart:
                    self._credit_stable_progress(sample, pos, direction, now)
                else:
                    self._stable_progress = None
                self._last_command, self._last_command_at = (v, w), now
            elif phase is Phase.VERIFY_ENTRY:
                self._entry_verified = True
            prediction = self._prediction_diagnostic(pos[:2], yaw, control_yaw, support, v, w, horizon, polygons, margin)
            self.debug = dict(tracking_state="DEGRADED" if degraded else "TRACKED", loss_response=False, age=age, epoch=sample.epoch,
                              sequence=sample.sequence, phase=phase.value, pose=pos.tolist(), yaw=yaw,
                              roll=roll, pitch=pitch, velocity=velocity.tolist(), angular_velocity=angular,
                              ey=ey, epsi=epsi, command=[v, w], landing_supported=landing,
                              exit_supported=exit_clear, compute_sec=sample.compute_sec,
                              target=[*map(float, target), float(target_yaw)], progress=progress,
                              phase_complete=complete, transition_ready=transition,
                              boundary_adjusted=adjusted, proposed_command=list(proposed),
                              below_flight_command_floor=(phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3)
                                  and not transition and v < limits.get("min_flight_v", 0.)),
                              prediction_horizon_sec=horizon, neutral_only=neutral_only,
                              flight_restart=restart,
                              flight_command_policy=self.route.get('flight_command_policy', 'prediction_gated'),
                              support_timestamp=sample.stamp,
                              support_orientation_source='lidar_observation',
                              gyro_heading_affects_flight_steering=flight_heading_detail.get('gyro_used', False),
                              flight_heading=flight_heading_detail,
                              input_recovery_resumed=input_resume,
                              input_recovery_used_sec=self._input_gap_used,
                              flight_end_height_conflict=flight_end_conflict,
                              control_heading=heading_detail, control_yaw=control_yaw,
                              steering_response=steering_detail,
                              policy_adjusted=(v,w) != proposed,
                              second_flight_alignment=alignment, margin_recovery=recovery,
                              landing_position_hold=landing_hold,
                              prediction_model=('lagged_measured_motion_envelope' if self.route.get('motion_prediction', {}).get('mode') == 'control' else 'configured_command_scale; stair response unverified'),
                              motion_prediction=prediction,
                              recovery_window_used_sec=self._rolling_recovery(now),
                              end_geometry=getattr(self, '_end_geometry_debug', {}),
                              landing_turn=getattr(self, '_landing_turn_debug', {}), turn_index=getattr(self, 'turn_index', 0),
                              measured_motion_preview=dict(
                                  position=(pos[:2]+velocity[:2]*horizon).tolist(),
                                  yaw=wrap(control_yaw+heading_detail.get('yaw_rate', angular)*horizon),
                                  horizon_sec=horizon, geometry_age_sec=age,
                                  purpose='diagnostic only; no calibrated actuator response'),
                              candidates=self._candidate_debug, **geometry)
            detail = ("flight end height conflict: zero command; arrival unconfirmed"
                      if flight_end_conflict else "LiDAR geometry/feedback: " + phase.value)
            return EvidenceReport(phase, complete, False, detail, progress, 0, age)

    def _recheck_flight(self, phase, sample, now, age, pos, yaw, control_yaw,
                        support, polygons, margin, horizon, geometry, proposed):
        """Bounded zero-command re-observation, never a claim of physical hold.

        Only prediction failures qualify. Expiry, attitude, interruption and
        observed support gates run before this method on every evaluation.
        Gyro-only rotation of an older center is diagnostic, not an observed
        current footprint. Prediction-gated legacy routes may still compare
        both hypotheses when evaluating future commands.
        """
        delta = wrap(control_yaw - yaw)
        c, s = math.cos(delta), math.sin(delta)
        rotated = (support - pos[:2]) @ np.array([[c, s], [-s, c]]) + pos[:2]
        # A commissioned recovery may consume only the extra reserve; it must
        # still preserve the surveyed anchor uncertainty for both headings.
        required = self.anchor.uncertainty_m if self.route['limits'].get('flight_recovery_w') else margin
        supported = inside_support_union(support, polygons, required)
        gyro_supported = inside_support_union(rotated, polygons, required)
        if self._flight_recheck is None:
            self._flight_recheck = dict(at=now, sequence=sample.sequence)
        elapsed = self._flight_recheck_used + max(0., now - self._flight_recheck['at'])
        budget = self.route['limits'].get('flight_recheck_sec', 0.)
        fault = (not supported or elapsed >= budget or
                 self._rolling_recovery(now, self._flight_recheck['at']) >= self.route['limits'].get('flight_recovery_window_budget_sec', math.inf))
        reason = ('prediction recheck lost supported footprint' if not supported else
                  'prediction recheck budget exhausted' if fault else
                  'prediction recheck: zero command, awaiting fresh feasible geometry')
        continued = None
        if (not fault and sample.geometry_valid and age <= self.route['limits']['warn_sec']
                and self._flight_recheck.get('correction') is not None):
            vector = np.diff(np.asarray(self.route[{Phase.FORWARD_SEGMENT_1:'flight_1', Phase.FORWARD_SEGMENT_2:'flight_2', Phase.FORWARD_SEGMENT_3:'flight_3'}[phase]]), axis=0)[0]
            continued = self._flight_recovery(pos[:2], yaw, control_yaw, support, polygons,
                margin, horizon, now, self._flight_recheck.get('correction_target_w', self._flight_recheck['correction'][1]), math.atan2(vector[1], vector[0]))
        if not fault:
            # The same zero input was already the loss response. Automatic
            # resumption now requires a newer fresh sample and a checked path.
            self._last_command, self._last_command_at = continued or (0., 0.), now
            if continued is None:
                self._flight_recheck.pop('correction', None)
        self.debug = dict(tracking_state='SUPPORT_LIMIT' if fault else 'PREDICTION_RECHECK',
                          loss_response=fault, phase=phase.value, pose=pos.tolist(),
                          yaw=yaw, control_yaw=control_yaw, age=age,
                          command=list(continued or (0., 0.)), proposed_command=list(proposed),
                          recovery_steering=continued is not None,
                          prediction_horizon_sec=horizon, phase_complete=False,
                          recheck_elapsed_sec=elapsed, recheck_budget_sec=budget,
                          recheck_requires_sequence_after=self._flight_recheck['sequence'],
                          gyro_footprint_supported=gyro_supported, observed_footprint_supported=supported,
                          support_orientation_source='lidar_observation', physical_hold_verified=False,
                          motion_prediction=(deepcopy(self._response_debug) if self.route.get('motion_prediction', {}).get('mode') == 'control' else dict(state='not_evaluated_during_recheck',affects_commands=False)),
                          recheck_required_margin_m=required,
                          candidates=self._candidate_debug, **geometry)
        if continued is not None:
            reason = 'bounded commissioned steering recovery; arrival unconfirmed'
        return EvidenceReport(phase, False, fault, reason, 0., 0., age)

    def _flight_recovery(self, position, yaw, control_yaw, support, polygons,
                         margin, horizon, now, requested_w, target_yaw):
        limits = self.route['limits']
        cap = limits['flight_recovery_w']
        cw = self._limit_command(0., float(np.clip(requested_w, -cap, cap)), now)[1]
        # Starting a correction must improve the heading, never merely wiggle.
        capture_yaw = target_yaw
        if 'flight_recovery_progress_m' in limits:
            idx = {Phase.FORWARD_SEGMENT_1:1, Phase.FORWARD_SEGMENT_2:2, Phase.FORWARD_SEGMENT_3:3}[self.phase]
            start = np.asarray(self.route['flight_%d'%idx][0][:2])
            direction = np.array([math.cos(target_yaw), math.sin(target_yaw)])
            ey = direction[0]*(position[1]-start[1])-direction[1]*(position[0]-start[0])
            capture_yaw -= float(np.clip((limits['lateral_kp']/limits['yaw_kp'])*ey,
                                        -limits.get('flight_capture_angle', limits['yaw_tolerance']),
                                        limits.get('flight_capture_angle', limits['yaw_tolerance'])))
        before = abs(wrap(control_yaw-capture_yaw))
        if abs(cw) < 1e-6 or abs(cw) > cap or abs(wrap(control_yaw+cw*horizon-capture_yaw)) >= before:
            return None
        delta = wrap(control_yaw-yaw)
        c, s = math.cos(delta), math.sin(delta)
        rotated = (support-position) @ np.array([[c,s],[-s,c]]) + position
        required = self.anchor.uncertainty_m
        if self.route.get('motion_prediction', {}).get('mode') == 'control':
            accepted, _ = self._response_supported(support,yaw,control_yaw,0.,cw,horizon,polygons,required)
            if accepted is not True:
                return None
        for angle, body in ((yaw, support), (control_yaw, rotated)):
            if not self._swept_support(position, angle, body, 0., cw, horizon, polygons, required):
                return None
            values = []
            for t in np.linspace(0., horizon, 17):
                co, si = math.cos(cw*t), math.sin(cw*t)
                fp = (body-position) @ np.array([[co,si],[-si,co]]) + position
                values.append(support_clearance(fp, polygons))
            if any(v is None for v in values) or min(values) < min(values[0], margin)-1e-9:
                return None
        return 0., cw

    @staticmethod
    def _swept_support(position, yaw, support, v, w, horizon, polygon, margin):
        polygons = [polygon] if np.asarray(polygon, dtype=object).ndim == 2 else polygon
        relative=support-position
        radius=float(np.max(np.linalg.norm(relative,axis=1)))
        # Exact sufficient proof for a stationary pivot: every rotated body
        # point stays in this disk. Avoid rejecting a proven disk solely due
        # to the sampled sweep's extra interpolation allowance.
        if abs(v)<1e-12 and any(inside_polygon([position],p,radius+margin) for p in polygons):
            return True
        steps = 16
        motion_bound = abs(v)+radius*abs(w)
        interpolation_bound=motion_bound*horizon/(2*steps)
        if (inside_support_union(support, polygons, margin) and
                not inside_support_union(support, polygons, margin+interpolation_bound)):
            steps = 64
            interpolation_bound=motion_bound*horizon/(2*steps)
        times=np.linspace(0.,horizon,steps+1)
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
        # Most rejected forward candidates fail at the far end. Checking it
        # first avoids traversing sixteen valid bodies before the same rejection.
        if not inside_support_union(swept[-1],polygons,margin+interpolation_bound):
            return False
        # Safe fast path: if the enclosing hull fits the actual union, every
        # sampled body fits too. Never accept the union's hull (which fills gaps).
        # A non-fitting enclosure falls back to the original per-body checks.
        flat=swept.reshape(-1,2)
        try:
            enclosure=flat[ConvexHull(flat).vertices]
            if inside_support_union(enclosure,polygons,margin+interpolation_bound):
                return True
        except QhullError:
            pass
        for projected in swept[:-1]:
            if not inside_support_union(projected,polygons,margin+interpolation_bound):
                return False
        return True

    def _flight_command(self, position, yaw, control_yaw, support, v, w, horizon,
                        polygons, margin, now, direction, start, recovery, *, restart=False):
        """Search reachable steering at full requested input before deceleration.

        The command-scale unicycle is retained as a compatibility model until
        stair response is measured. No bag-derived average silently replaces a
        physical bound. Scores choose among geometrically admitted candidates;
        they are not an assertion of physical recovery.
        """
        limits = self.route['limits']
        low_v = max(0., self._limit_command(0., w, now)[0])
        low_w = self._limit_command(v, -limits['max_w'], now)[1]
        high_w = self._limit_command(v, limits['max_w'], now)[1]
        speeds = [v] if restart else sorted(set((v, (v+low_v)*.5, low_v)), reverse=True)
        steering = sorted(set((w, low_w, high_w, (low_w+high_w)*.5,
                              float(np.clip(0., low_w, high_w)))))
        delta = wrap(control_yaw-yaw)
        c, s = math.cos(delta), math.sin(delta)
        control_support = (support-position) @ np.array([[c,s],[-s,c]]) + position
        frames = [(yaw, support)]
        if abs(delta) > 1e-9:
            frames.append((control_yaw, control_support))
        for cv in speeds:
            ranked = []
            for cw in steering:
                if abs(cv)+abs(cw) < 1e-9:
                    continue
                center=self._project_center(position,control_yaw,cv,cw,horizon)
                ey=direction[0]*(center[1]-start[1])-direction[1]*(center[0]-start[0])
                angle=wrap(control_yaw+cw*horizon-math.atan2(direction[1],direction[0]))
                # An inward capture heading avoids stranding an offset body
                # parallel to the centerline. Score is not a safety predicate.
                capture=wrap(angle+(limits['lateral_kp']/limits['yaw_kp'])*ey)
                score=limits['yaw_kp']*capture*capture+limits['lateral_kp']*ey*ey
                row=dict(command=[cv,cw],accepted=None,score=score,
                         required_margin_m=self.anchor.uncertainty_m if recovery else margin,recovery=recovery)
                self._candidate_debug.append(row)
                # Prefer the bounded proportional request when it is feasible.
                # Endpoint score alone amplifies tiny errors into maximal
                # steering each cycle, especially before a stair impact.
                ranked.append((abs(cw-w),score,cw,row))
            for _,__,cw,row in sorted(ranked,key=lambda r:r[:3]):
                required = self.anchor.uncertainty_m if recovery else margin
                passes = self._candidate_supported(position, yaw, control_yaw, support, cv, cw, horizon, polygons, required, frames)
                reason = ('response_model_unavailable' if self.route.get('motion_prediction', {}).get('mode') == 'control' and self._response_debug.get('state') == 'unavailable' else 'support_envelope') if not passes else 'supported'
                if passes and recovery:
                    # No further depletion of measured clearance, and recovery
                    # must actually reach the normal margin by the horizon.
                    for angle, body in frames:
                        values=[]
                        for t in np.linspace(0., horizon, 17):
                            a=cw*t; co,si=math.cos(a),math.sin(a)
                            center=self._project_center(position, angle, cv, cw, t)
                            fp=(body-position)@np.array([[co,si],[-si,co]])+center
                            values.append(support_clearance(fp,polygons))
                        if any(x is None for x in values) or ((min(values)<required-1e-9 or values[-1]<values[0]-1e-9) if 'flight_recovery_progress_m' in limits else (min(values)<values[0]-1e-9 or values[-1]<margin)):
                            passes=False
                            reason='reserve_not_recovered_monotonically'
                            break
                row['accepted']=passes
                row['reason']=reason
                if passes:
                    return cv,cw
        return None

    def _reset_continuity(self):
        self._stable_progress = None
        self._flight_heading_state = None
        self._recovery_events = deque()
        self._prediction_context = None
        self._response_debug = dict(state='unavailable',reason='no candidate checked in current phase')
        self._prediction_cache = None
        self._prediction_generation = getattr(self, "_prediction_generation", 0)+1
        if not hasattr(self, "_prediction_pending"):
            self._prediction_pending = None
        self._end_plane_votes = deque(maxlen=8)
        self._end_advance = None
        self._end_geometry_debug = {}

    def _rolling_recovery(self, now, active_at=None):
        window = self.route['limits'].get('flight_recovery_window_sec')
        if window is None:
            return 0.
        cutoff = now-window
        while self._recovery_events and self._recovery_events[0][1] <= cutoff:
            self._recovery_events.popleft()
        intervals = list(self._recovery_events)
        if active_at is not None:
            intervals.append((active_at, now))
        # Prediction and input recovery can overlap; count elapsed time once.
        total, high = 0., cutoff
        for a,b in sorted(intervals):
            a, b = max(a, cutoff, high), min(b, now)
            total += max(0., b-a)
            high = max(high, b)
        return total

    def _credit_stable_progress(self, sample, pos, direction, now):
        limits = self.route['limits']
        if 'flight_stable_sec' not in limits:
            return
        value = self._stable_progress
        if value is not None and sample.sequence <= value['last_sequence']:
            return  # repeated frame never earns recovery credit
        if (value is None or sample.measured_at-value['last_at'] > limits['warn_sec']
                or sample.measured_at <= value['last_at']
                or float((pos[:2]-value['last_position']) @ direction) < -limits['position_off']):
            self._stable_progress = dict(at=sample.measured_at, position=pos[:2].copy(),
                last_at=sample.measured_at, last_position=pos[:2].copy(), last_sequence=sample.sequence, count=1)
            return
        value.update(last_at=sample.measured_at, last_position=pos[:2].copy(), last_sequence=sample.sequence,
                     count=value['count']+1)
        if (value['count'] >= 3 and sample.measured_at-value['at'] >= limits['flight_stable_sec']
                and float((pos[:2]-value['position']) @ direction) >= limits['flight_recovery_progress_m']):
            self._flight_recheck_used = self._input_gap_used = 0.

    def _landing_path(self):
        if self.phase is Phase.ROOFTOP_TURN:
            return getattr(self, '_roof_turn_path', self.route['roof_turn_path'])
        return getattr(self, '_landing_turn_path', self.route['turn_path'])

    def _region_turn(self):
        key = {Phase.TURN_TO_NEXT_FLIGHT:'landing_turn_policy', Phase.ROOFTOP_TURN:'roof_turn_policy'}.get(self.phase)
        return key is not None and self.route.get(key) == 'supported_region'

    def _turn_polygon(self):
        return self.route['roof_landing_polygon' if self.phase is Phase.ROOFTOP_TURN else 'landing_polygon']

    def _turn_direction(self):
        flight = self.route['flight_2' if self.phase is Phase.ROOFTOP_TURN else 'flight_1']
        direction = np.asarray(flight[1][:2])-flight[0][:2]
        return direction / np.linalg.norm(direction)

    def _landing_rotation_waypoint(self):
        path, index = self._landing_path(), self.turn_index
        return (index > 0 and np.allclose(path[index-1][:2], path[index][:2])
                and abs(wrap(path[index][2]-path[index-1][2])) >= .1)

    def _connect_landing_leg(self, position, margin):
        """Move only the next cross-landing pair to the actual rotation depth.

        Preserve the next-flight entry and the cross-landing end coordinate.
        The revised pivot must fit the same surveyed rotation disk. Commands
        along the leg still pass the ordinary observed/swept support checks.
        """
        path, index = self._landing_path(), self.turn_index
        if index+2 >= len(path)-1 or not np.allclose(path[index+1][:2], path[index+2][:2]):
            return
        direction = self._turn_direction()
        old = np.asarray(path[index+1][:2])
        next_pivot = old + float((position-old) @ direction)*direction
        radius = float(np.max(np.linalg.norm(np.asarray(self.route['footprint']), axis=1)))
        if not inside_polygon([next_pivot], self._turn_polygon(), radius+margin):
            self._landing_turn_debug['next_leg_rebased'] = False
            return
        path[index+1][:2] = path[index+2][:2] = next_pivot.tolist()
        self._holding = False
        self._landing_turn_debug.update(next_leg_rebased=True, next_pivot=next_pivot.tolist())

    def _capture_landing_turn(self, pose, margin, degraded):
        """Capture a pivot once its entire yaw sweep fits the flat landing.

        Uses surveyed landing geometry, not an unobserved claim of obstacle
        clearance. A wrongly surveyed landing must be corrected separately.
        """
        if not self._region_turn():
            return
        path = self._landing_path()
        radius = float(np.max(np.linalg.norm(np.asarray(self.route['footprint']), axis=1)))
        clear = inside_polygon([pose[:2,3]], self._turn_polygon(), radius+margin)
        self._landing_turn_debug = dict(rotation_disk_supported=clear,
            rotation_radius_m=radius, margin_m=margin,
            pivot_captured=getattr(self, '_landing_turn_captured', False))
        index = self.turn_index
        if index+1 >= len(path) or degraded or not clear:
            return
        if (not np.allclose(path[index][:2], path[index+1][:2])
                or abs(wrap(path[index+1][2]-path[index][2])) < .1):
            return
        if index > 0 and np.linalg.norm(pose[:2,3]-path[index][:2]) > self.route['limits']['position_on']:
            return
        pivot = pose[:2,3].copy()
        direction = self._turn_direction()
        offset = float((pivot-np.asarray(path[0][:2])) @ direction)*direction
        # Keep the cross-landing leg at the captured depth, but preserve the
        # final approach point for the next flight's separately checked entry.
        if index == 0:
            for j in range(len(path)-1):
                path[j][:2] = (np.asarray(path[j][:2])+offset).tolist()
        path[index][:2] = path[index+1][:2] = pivot.tolist()
        self._landing_turn_captured = True
        self.turn_index = index+1
        self._holding = False
        self._history.clear()
        self._landing_turn_debug.update(pivot_captured=True, pivot=pivot.tolist())

    def _resume_corridor(self, idx, pose, support, margin, epsi):
        limits = self.route['limits']
        start,end = np.asarray(self.route['flight_%d'%(idx+1)], dtype=float)
        direction = end[:2]-start[:2]; direction /= np.linalg.norm(direction)
        first_polygon = np.asarray(self.route['flight_%d_polygon'%(idx+1)])
        first_edge = max(0., float(np.min((first_polygon-start[:2]) @ direction)))
        along = float((pose[:2,3]-start[:2]) @ direction)
        names, regions = self.support_regions(self.phase)
        lateral = np.array([-direction[1],direction[0]])
        width = first_polygon @ lateral
        body_width = support @ lateral
        return bool(body_width.min() >= width.min()+margin and body_width.max() <= width.max()-margin
            and inside_support_union(support,regions,margin)
            and along <= first_edge+margin
            and abs(float(pose[2,3]-start[2])) <= limits['height_tolerance']
            and abs(epsi) <= limits['flight_resume_corridor_yaw'])

    def _motion_state(self):
        context = self._prediction_context
        if context is None:
            return None
        sample,pose,velocity,angular,control_yaw,detail,now = context
        history = [h for h in self._history if sample.stamp-.6 <= h[0] <= sample.stamp]
        if len(history) < 3 or history[-1][0]-history[0][0] < .3:
            return None
        rates = np.asarray([h[3][:2] for h in history[1:]])
        trend = np.median(rates,axis=0)
        spread = float(np.max(np.linalg.norm(rates-trend,axis=1)))
        return MotionState(pose[:2,3].copy(),control_yaw,trend,
            detail.get('yaw_rate',angular),now-sample.measured_at,
            abs(wrap(control_yaw-math.atan2(pose[1,0],pose[0,0]))),spread)

    def _response_supported(self, support, yaw, control_yaw, v, w, horizon, polygons, margin):
        state = self._motion_state()
        if state is None:
            result,detail=None,dict(state='unavailable',reason='need three distinct motion observations')
        else:
            result,detail=self._check_response_state(state,self.route['motion_prediction'],support,
                                                     yaw,control_yaw,v,w,horizon,polygons,margin)
        self._response_debug=dict(detail,command=[v,w],horizon_sec=horizon,
                                  affects_commands=True)
        return result,self._response_debug

    @staticmethod
    def _check_response_state(state,model,support,yaw,control_yaw,v,w,horizon,polygons,margin,check_support=True):
        try:
            envelope = response_envelope(state,(v,w),horizon,model)
        except ValueError as error:
            return None,dict(state='unavailable',reason=str(error))
        if not check_support:
            end=envelope['centers'][:,-1]
            return None,dict(state='evaluated',mode=model['mode'],commissioned=model['commissioned'],
                source=model['source'],accepted=None,geometry_admission_checked=False,
                predicted_end_min=end.min(axis=0).tolist(),predicted_end_max=end.max(axis=0).tolist(),
                endpoint_position_error_m=float(envelope['position_error'][-1]),
                measured_position=state.position.tolist(),propagated_position=envelope['propagated_position'].tolist(),
                measured_velocity=state.velocity.tolist(),geometry_age_sec=state.age,physical_hold_verified=False)
        relative = support-state.position
        delta = wrap(control_yaw-yaw)
        co,si = math.cos(delta),math.sin(delta)
        relative = relative @ np.array([[co,si],[-si,co]])
        radius = float(np.max(np.linalg.norm(relative,axis=1)))
        angles=envelope['angles']-control_yaw
        co,si=np.cos(angles)[...,None],np.sin(angles)[...,None]
        bodies=np.stack((co*relative[:,0]-si*relative[:,1],
                         si*relative[:,0]+co*relative[:,1]),axis=-1)+envelope['centers'][...,None,:]
        error=envelope['position_error']+2*radius*np.sin(np.minimum(math.pi,envelope['yaw_error'])/2.)
        interpolation=(np.linalg.norm(np.diff(envelope['centers'],axis=1),axis=2)
                       +radius*np.abs(np.diff(envelope['angles'],axis=1)))/2.
        # The last sample also needs the preceding interval's interpolation bound.
        errors=error[None,:]+np.maximum(np.pad(interpolation,((0,0),(0,1))),
                                       np.pad(interpolation,((0,0),(1,0))))
        minimum=math.inf
        allowed=False
        if inside_support_union(support,polygons,margin):
            # Vectorized one-region path avoids hundreds of Python geometry
            # calls for the common straight flight; union seams still checked.
            for polygon in polygons:
                polygon=np.asarray(polygon);edges=np.roll(polygon,-1,axis=0)-polygon
                distance=bodies[...,None,:]-polygon
                clearance=(edges[:,0]*distance[...,1]-edges[:,1]*distance[...,0])/np.linalg.norm(edges,axis=1)
                lower=np.min(clearance,axis=(-1,-2))-errors
                if np.all(lower>=margin):
                    allowed=True;minimum=float(lower.min());break
            if not allowed:
                minimum=math.inf;allowed=True
                for path,errs in zip(bodies,errors):
                    for body,err in zip(path,errs):
                        clearance=support_clearance(body,polygons)
                        lower=-math.inf if clearance is None else clearance-float(err)
                        minimum=min(minimum,lower)
                        if lower<margin:
                            allowed=False;break
                    if not allowed:break
        return allowed,dict(state='evaluated',mode=model['mode'],commissioned=model['commissioned'],
            source=model['source'],accepted=allowed,min_clearance_lower_m=None if not math.isfinite(minimum) else minimum,
            measured_position=state.position.tolist(),propagated_position=envelope['propagated_position'].tolist(),
            measured_velocity=state.velocity.tolist(),velocity_spread_mps=state.velocity_spread,
            geometry_age_sec=state.age,physical_hold_verified=False,
            limitation='conditional sampled response envelope; physical coverage unverified')

    def _candidate_supported(self, position,yaw,control_yaw,support,v,w,horizon,polygons,margin,frames):
        model=self.route.get('motion_prediction')
        if model and model['mode']=='control':
            result,detail=self._response_supported(support,yaw,control_yaw,v,w,horizon,polygons,margin)
            return result is True
        return all(self._swept_support(position,angle,body,v,w,horizon,polygons,margin)
                   for angle,body in frames)

    def _prediction_diagnostic(self,position,yaw,control_yaw,support,v,w,horizon,polygons,margin):
        model=self.route.get('motion_prediction')
        if model is None or self.phase not in (Phase.FORWARD_SEGMENT_1,Phase.FORWARD_SEGMENT_2,Phase.FORWARD_SEGMENT_3):
            return dict(state='disabled')
        if model['mode']=='control':
            return dict(getattr(self,'_response_debug',dict(state='unavailable')),
                        purpose='last evaluated candidate; emitted command is outer command')
        # A single daemon job receives immutable copies. Shadow work never
        # holds the command lock or queues an unbounded backlog of scans.
        pending=self._prediction_pending
        if pending is not None and pending[1].done():
            try:
                key,detail=pending[1].result()
                if key[0]==self._prediction_generation:
                    self._prediction_cache=(key,detail)
            except Exception as error:
                self._prediction_cache=((self._prediction_generation,None,None),
                    dict(state='unavailable',reason=str(error),affects_commands=False))
            self._prediction_pending=None
        sample=self._prediction_context[0]
        key=(self._prediction_generation,sample.epoch,sample.sequence)
        if self._prediction_pending is None and (self._prediction_cache is None or self._prediction_cache[0]!=key):
            state=self._motion_state()
            if state is None:
                return dict(state='unavailable',reason='need three distinct motion observations',affects_commands=False)
            future=Future()
            args=(state,deepcopy(model),support.copy(),yaw,control_yaw,v,w,horizon,
                  [np.array(p,dtype=float).copy() for p in polygons],margin)
            def compute():
                try:
                    _,detail=StairFeedback._check_response_state(*args,check_support=False)
                    detail.update(command=[v,w],horizon_sec=horizon,affects_commands=False,
                        comparison_epoch=sample.epoch,comparison_sequence=sample.sequence,
                        comparison_measured_at=sample.measured_at,
                        purpose='observation-time hypothetical comparison; not current command admission')
                    future.set_result((key,detail))
                except Exception as error:
                    future.set_exception(error)
            self._prediction_pending=(key,future)
            threading.Thread(target=compute,name='stair-prediction-shadow',daemon=True).start()
        return (dict(self._prediction_cache[1],pending=self._prediction_pending is not None)
                if self._prediction_cache is not None else dict(state='pending',affects_commands=False))

    def _end_geometry_continuation(self,idx,sample,pose,pitch,roll,now,degraded):
        limits=self.route['limits']
        if 'flight_end_advance_m' not in limits:
            return False
        start,end=np.asarray(self.route['flight_%d'%(idx+1)], dtype=float)
        direction=end[:2]-start[:2];direction/=np.linalg.norm(direction)
        deficit=float(end[2]-pose[2,3])
        reason='upper-plane evidence unavailable'
        observed=False
        if not degraded and sample.display_points is not None and 0 < deficit <= limits['flight_end_height_gap_m']:
            points=np.asarray(sample.display_points).reshape(-1,3)
            profile_lidar=pose @ self.base_from_lidar
            points=points @ profile_lidar[:3,:3].T+profile_lidar[:3,3]
            delta=points[:,:2]-pose[:2,3]
            along=delta @ direction
            lateral=direction[0]*delta[:,1]-direction[1]*delta[:,0]
            tolerance=limits['flight_end_flat_tolerance_m']
            mask=(np.isfinite(points).all(axis=1)&(along>.225)&(along<.8)&(np.abs(lateral)<.225)
                  &(np.abs(points[:,2]-end[2])<=tolerance))
            selected=points[mask]
            cells=np.unique(np.floor(np.column_stack((along[mask],lateral[mask]))/.15).astype(int),axis=0)
            observed=bool(len(cells)>=6 and np.ptp(along[mask])>=.25 and np.ptp(lateral[mask])>=.15)
            reason='observed upper plane' if observed else 'upper plane lacks spatial support'
        if not self._end_plane_votes or sample.sequence>self._end_plane_votes[-1][0]:
            self._end_plane_votes.append((sample.sequence,sample.measured_at,observed))
        votes=[v for v in self._end_plane_votes if v[1]>=sample.measured_at-limits['settle_sec']-limits['warn_sec']]
        confirmed=(len(votes)>=3 and all(v[2] for v in votes)
                   and votes[-1][1]-votes[0][1]+1e-9>=limits['settle_sec'])
        if confirmed and self._end_advance is None:
            self._end_advance=dict(at=now,last_seq=sample.sequence,last_position=pose[:2,3].copy(),travel=0.)
        value=self._end_advance
        if value is not None and sample.sequence>value['last_seq']:
            value['travel']+=float(np.linalg.norm(pose[:2,3]-value['last_position']))
            value.update(last_seq=sample.sequence,last_position=pose[:2,3].copy())
        active=bool(value is not None and confirmed and not degraded
                    and value['travel']<limits['flight_end_advance_m']
                    and now-value['at']<limits['flight_end_advance_sec'])
        self._end_geometry_debug=dict(reason=reason,plane_confirmed=confirmed,
            continued=active,height_deficit_m=deficit,arrival_confirmed=False,
            travel_m=None if value is None else value['travel'],
            max_travel_m=limits['flight_end_advance_m'],max_seconds=limits['flight_end_advance_sec'])
        return active

    @staticmethod
    def _project_center(position, yaw, v, w, duration):
        if abs(w)<1e-8:
            return position+v*duration*np.array([math.cos(yaw),math.sin(yaw)])
        return position+(v/w)*np.array([math.sin(yaw+w*duration)-math.sin(yaw),
                                       math.cos(yaw)-math.cos(yaw+w*duration)])

    def _entry_reserve_command(self, position, yaw, support, w, horizon, polygons, margin, now):
        """Restore only the extra entry reserve with the existing heading turn.

        No forward/reverse input or opposite steering is invented. The swept
        body must keep the anchor uncertainty and improve clearance without
        first reducing it. This geometric check is not physical drift holding.
        """
        current = support_clearance(support, polygons)
        hard = self.anchor.uncertainty_m
        if current is None or current < hard or current >= margin:
            return None
        for scale in (1., .5, .25):
            v, cw = self._limit_command(0., w*scale, now)
            if abs(v) > 1e-9 or abs(cw) < 1e-9:
                continue
            if not self._swept_support(position, yaw, support, 0., cw, horizon, polygons, hard):
                continue
            values = []
            for t in np.linspace(0., horizon, 17):
                co, si = math.cos(cw*t), math.sin(cw*t)
                body = (support-position) @ np.array([[co,si],[-si,co]]) + position
                values.append(support_clearance(body, polygons))
            if (any(value is None for value in values)
                    or min(values) < current-1e-9 or values[-1] <= current+1e-6):
                continue
            return 0., cw
        return None

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
        if self.phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN, Phase.EXIT_CONFIRM):
            # Flat-region command cap takes precedence over deceleration slew:
            # never carry the previous flight's high input onto the landing.
            # This limits requested input; it does not prove physical braking.
            linear = float(np.clip(linear, -limits["hold_v"], limits["hold_v"]))
        if (self._region_turn() and
                self._landing_rotation_waypoint()):
            linear = 0.
        desired = float(np.clip(w, -limits['max_w'], limits['max_w']))
        alpha = limits['max_alpha']
        on_flight = (self.phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3)
                     and not self._entry_alignment_pending and not self._flight_end_conflict)
        brake = limits.get('flight_neutral_alpha', alpha) if on_flight else alpha
        # Faster neutralization does not authorize faster growth in the other
        # direction. Account separately for the time to reach zero.
        if old_w*desired < 0:
            stop_time = abs(old_w)/brake
            if dt <= stop_time:
                angular = old_w-math.copysign(brake*dt, old_w)
            else:
                angular = math.copysign(min(abs(desired), alpha*(dt-stop_time)), desired)
        elif abs(desired) < abs(old_w):
            angular = old_w + float(np.clip(desired-old_w, -brake*dt, brake*dt))
        else:
            angular = old_w + float(np.clip(desired-old_w, -alpha*dt, alpha*dt))
        return linear, float(np.clip(angular, -limits['max_w'], limits['max_w']))

    def command(self):
        with self._lock:
            return self._last_command

    def diagnostic_snapshot(self):
        with self._lock:
            result = dict(self.debug, phase=self.phase.value, command=list(self._last_command),
                          operator_completed_flights=sorted(getattr(self,"_operator_completed_flights",set())),
                          control_policy=self.policy_version,
                          prediction_recheck_active=self._flight_recheck is not None,
                          prediction_recheck_used_sec=self._flight_recheck_used)
            with self._tx_lock:
                result['last_sent_twist'] = deepcopy(self._last_sent)
            result['first_fault'] = deepcopy(self._first_fault)
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
        with self._lock:
            self._last_command = (0., 0.)
