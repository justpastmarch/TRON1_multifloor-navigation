"""Stair-local geometry, feedback and phase evidence; no command transport.

All physical limits belong to a commissioned route, not to global NAV gates.
Observed position retains its timestamp. Predicted motion is kept separate.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from functools import lru_cache
import math
import threading

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull, QhullError

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


def effective_support_margin(route, anchor_uncertainty_m, landing=False, region=None):
    """Return the commissioned footprint-to-edge margin for this route.

    Entry-registration uncertainty remains available as separate provenance.
    Routes with an explicit support margin do not add that matching budget a
    second time to every stair edge; legacy/synthetic routes retain their
    previous margin plus measured-anchor behavior.
    """
    limits = route['limits']
    if region in ('entry', 'exit') and region+'_support_margin_m' in limits:
        return float(limits[region+'_support_margin_m'])
    if landing and 'landing_support_margin_m' in limits:
        return float(limits['landing_support_margin_m'])
    return float(limits.get('support_margin_m', limits['margin_m'] + anchor_uncertainty_m))


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

    policy_version = '2026-10-16-roof-entry-heading-v33'

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
        self._pose_jump_at = None
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
        self._candidate_debug = []
        self._reset_continuity()

    @staticmethod
    def _validate_route(route):
        keys = {"id", "commissioned", "direction", "flight_1", "flight_2", "landing_polygon",
                "exit_polygon", "entry_polygon", "flight_1_polygon", "flight_2_polygon", "footprint",
                "turn_path", "limits", "loss_response", "loss_response_evidence"}
        # motion_prediction remains accepted for one config migration period, but is ignored.
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
        if route['direction']=='DOWN' and ('reverse_of' not in route or
                len(route.get('flight_input_v',[])) != (3 if 'flight_3' in route else 2)):
            raise ValueError('DOWN requires a route derived from UP and one forward input per flight')
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
        policy = route.get('flight_command_policy', 'observed_feedback')
        if policy != 'observed_feedback':
            raise ValueError('only observed_feedback flight control is supported')
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
        # Legacy prediction/recheck limits are accepted during migration, but ignored by observed_feedback.
        optional_limits = {"support_margin_m", "landing_support_margin_m", "entry_support_margin_m", "exit_support_margin_m", "min_flight_v", "flight_prediction_sec", "flight_recheck_sec", "flight_restart_v", "flight_input_recover_sec",
                           "flight_heading_filter_sec", "flight_yaw_kd", "flight_damping_max_w", "flight_capture_angle", "flight_yaw_kp", "flight_centering_lookahead_m",
                           "flight_neutral_alpha", "flight_gyro_rate_max_age", "flight_recovery_w", "flight_lateral_preview_sec", "entry_corridor_yaw", "exit_settle_sec", "flight_stable_sec",
                           "flight_recovery_window_sec", "flight_recovery_window_budget_sec",
                           "flight_recovery_progress_m", "flight_resume_corridor_yaw",
                           "flight_input_reobserve_sec", "flat_reobserve_sec", "landing_max_v",
                           "pose_jump_xy_m", "pose_jump_z_m", "pose_jump_reobserve_sec",
                           "flight_end_advance_m", "flight_end_advance_sec", "flight_end_flat_tolerance_m", "flight_end_height_gap_m",
                           "turn_margin_recovery_m", "flight_margin_recovery_m", "rotation_space_margin_m", "roof_pivot_radius_m"}
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
        if 'flight_input_recover_sec' in limits and (limits['flight_input_recover_sec'] > limits['recover_sec'] or 'flight_restart_v' not in limits):
            raise ValueError('flight_input_recover_sec requires flight_restart_v and must not exceed recover_sec')
        jump_limits = {'pose_jump_xy_m', 'pose_jump_z_m', 'pose_jump_reobserve_sec'}
        if jump_limits & set(limits) and not jump_limits <= set(limits):
            raise ValueError('correlated pose-jump check requires all three limits')
        if limits.get('flat_reobserve_sec', limits['expire_sec']) < limits['expire_sec']:
            raise ValueError('flat_reobserve_sec must cover sensor expiration')
        if 'flight_input_reobserve_sec' in limits and 'flight_input_recover_sec' not in limits:
            raise ValueError('flight_input_reobserve_sec requires flight_input_recover_sec')
        if not (0. < limits.get('landing_max_v', limits['hold_v'])):
            raise ValueError('landing_max_v must be positive')
        if limits.get("min_flight_v", 0.) > limits["max_v"]:
            raise ValueError("min_flight_v must not exceed max_v")
        # Legacy recheck parameters are ignored. Restart input is independently bounded.
        if 'flight_restart_v' in limits and (limits['flight_restart_v'] > limits['max_v'] or
                limits['flight_restart_v'] < limits.get('min_flight_v', 0.)):
            raise ValueError('flight_restart_v must lie between min_flight_v and max_v')
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
                limits['flight_recovery_window_budget_sec'] >= limits['flight_recovery_window_sec']):
            raise ValueError('flight continuity requires bounded stability, rolling budget and progress')
        if limits.get('flight_resume_corridor_yaw', .1) >= math.pi/2:
            raise ValueError('flight resume corridor must face the next flight')
        for key in ('landing_support_margin_m', 'entry_support_margin_m', 'exit_support_margin_m'):
            if (key in limits and not limits['margin_m'] <= limits[key] <=
                    effective_support_margin(route, limits['anchor_uncertainty_m'])):
                raise ValueError(key+' must lie within route support margins')
        if (limits.get('turn_margin_recovery_m', 0.) >
                limits.get('support_margin_m', limits['margin_m'] + limits['anchor_uncertainty_m']) - limits['margin_m']):
            raise ValueError('turn recovery cannot cross the route minimum support margin')
        if (limits.get('flight_margin_recovery_m', 0.) >
                limits.get('support_margin_m', limits['margin_m'] + limits['anchor_uncertainty_m']) - limits['margin_m']):
            raise ValueError('flight recovery cannot cross the route minimum support margin')
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
                margin = effective_support_margin(route, anchor.uncertainty_m,
                    landing=not entry, region='entry' if entry else None)
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
            self._pose_jump_at = None
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
                                       effective_support_margin(self.route, self.anchor.uncertainty_m, region='entry'))
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
            if (not inside_polygon(support, self.route['landing_polygon'], effective_support_margin(self.route, self.anchor.uncertainty_m, landing=True))
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

    def _landing_margin_context(self, phase, position):
        """Use landing clearance through the adjoining flight handoff.

        Membership uses the measured body center. Full-body support and
        completion height are checked separately. Entry/exit overrides are
        selected by _support_margin_context. Legacy routes keep their policy.
        """
        if 'landing_support_margin_m' not in self.route['limits']:
            return False
        if phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN):
            return True
        names = {
            Phase.FORWARD_SEGMENT_1: ('landing_polygon',),
            Phase.FORWARD_SEGMENT_2: ('landing_polygon', 'roof_landing_polygon'),
            Phase.FORWARD_SEGMENT_3: ('roof_landing_polygon',),
        }.get(phase, ())
        return any(inside_polygon([position[:2]], self.route[name])
                   for name in names if name in self.route)

    def _support_margin_context(self, phase, position=None):
        for region, fixed, flight in (
                ('entry', (Phase.VERIFY_ENTRY, Phase.ALIGN), Phase.FORWARD_SEGMENT_1),
                ('exit', (Phase.EXIT_CONFIRM,), Phase.FORWARD_SEGMENT_3
                 if 'flight_3' in self.route else Phase.FORWARD_SEGMENT_2)):
            if region+'_support_margin_m' in self.route['limits'] and (
                    phase in fixed or (phase is flight and position is not None and
                    inside_polygon([position[:2]], self.route[region+'_polygon']))):
                return region
        if ((position is not None and self._landing_margin_context(phase, position)) or
                (position is None and phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN))):
            return 'landing'
        return 'route'

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

    def continue_phase(self, now):
        """Resume the current live phase without resetting its turn substep."""
        with self._lock:
            if self.profile is None or self.route is None or self.anchor is None:
                raise ValueError('현재 계단 경로를 이어갈 수 없습니다.')
            phase = self.phase
            if phase not in self.traversal_phases(self.profile):
                raise ValueError('현재 경로에 없는 단계입니다.')
            sample = self.worker.snapshot()
            diagnostics = self.worker.diagnostics()
            if (sample is None or not sample.geometry_valid or sample.last_geometry_at is None
                    or sample.epoch != self.anchor.epoch or self.worker.epoch != self.anchor.epoch
                    or diagnostics.get('worker_error')
                    or not 0 <= now-sample.last_geometry_at <= self.route['limits']['warn_sec']):
                raise ValueError('같은 좌표 기준의 최신 LiDAR 관측이 필요합니다.')
            fault_sequence = None if self._first_fault is None else self._first_fault.get('sample_sequence')
            if fault_sequence is not None and sample.sequence <= fault_sequence:
                raise ValueError('중단 이후의 새 LiDAR 관측이 필요합니다.')

            # Keep phase, completed flights, turn index, captured pivot and the
            # remaining calibrated path. Rebuild only temporal input state.
            self.started_at = now
            self._interrupted = False
            self._recovering_at = None
            self._input_gap = None
            self._input_gap_used = 0.
            self._pose_jump_at = None
            self._recovery_events.clear()
            self._stable_progress = None
            self._history.clear()
            self._last_sample = None
            self._phase_sequence = sample.sequence
            self._turn_sequence = sample.sequence
            self._last_command = (0., 0.)
            self._last_command_at = now
            self._first_fault = None
            self._candidate_debug = []
            self._observation_debug = {}

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
            self._pose_jump_at = None
            self._history.clear()
            self._last_sample = None
            self._reset_continuity()
            sample = self.worker.snapshot()
            self._phase_sequence = sample.sequence if sample else 0
            self._turn_sequence = self._phase_sequence
            self._holding = None
            self._rotation_drift_recovery = False
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
            observed_feedback = True
            landing_hold = getattr(self, '_landing_hold_target', None) is not None and phase is Phase.LANDING
            holding_exit = (getattr(self, '_arrival_holding', False) and phase is Phase.EXIT_CONFIRM) or landing_hold
            if not self._interrupted and not holding_exit and now - self.started_at > self.profile.timeout_sec:
                previous = self.debug if self.debug.get("phase") == phase.value else {}
                self.debug = dict(tracking_state="TIMEOUT", loss_response=True,
                    phase=phase.value, last_tracking_state=previous.get("tracking_state"),
                    last_incomplete_conditions=previous.get("incomplete_conditions", []),
                    last_completion_checks=previous.get("completion_checks", {}),
                    last_pose=previous.get("pose"), last_target=previous.get("target"),
                    last_turn_index=previous.get("turn_index"),
                    last_landing_turn=previous.get("landing_turn"))
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
                # Age starts at the last measured geometry, not at expiry.
                # During a long blind interval, never repeat the full ascent
                # request past the last surveyed flight end by dead reckoning.
                index = {Phase.FORWARD_SEGMENT_1: 1, Phase.FORWARD_SEGMENT_2: 2,
                         Phase.FORWARD_SEGMENT_3: 3}[phase]
                start, end = np.asarray(self.route['flight_%d' % index], dtype=float)
                axis = end[:2]-start[:2]
                length = float(np.linalg.norm(axis))
                axis /= length
                if self._last_sample is not None:
                    previous = self._last_sample[1][:2,3]
                elif sample.geometry_valid:
                    previous = (np.linalg.inv(transform(self.anchor.local_from_profile)) @
                                transform(sample.transform) @
                                np.linalg.inv(self.base_from_lidar))[:2,3]
                else:
                    previous = None
                remaining = (max(0., length-float((previous-start[:2]) @ axis))
                             if previous is not None else 0.)
                observed_bound = max((abs(float(h[3][:2] @ axis))
                    for h in self._history if sample.stamp-h[0] <= limits['settle_sec']), default=0.)
                speed_bound = max(limits['max_v'], observed_bound)
                motion_age = (remaining+limits['position_on'])/speed_bound
                self._input_gap = dict(at=now, sequence=sample.sequence,
                    motion_age_sec=motion_age, remaining_m=remaining,
                    speed_bound_mps=speed_bound)
            if input_eligible and self._input_gap is not None:
                gap_elapsed = max(0., now-self._input_gap['at'])
                elapsed = self._input_gap_used + gap_elapsed
                window_used = self._rolling_recovery(now, self._input_gap['at'])
                window_budget = limits.get('flight_recovery_window_budget_sec', math.inf)
                # A single uninterrupted registration gap has its own cap. Short
                # gaps across a loaded ascent use the separate rolling budget.
                exhausted = (gap_elapsed >= limits['flight_input_recover_sec'] or
                             window_used >= window_budget)
                fresh = (sample.geometry_valid and age <= limits['warn_sec']
                         and sample.sequence > self._input_gap['sequence'])
                reobserve_limit = (limits['flight_input_recover_sec']
                                   + limits.get('flight_input_reobserve_sec', 0.))
                blind_end = age >= self._input_gap.get('motion_age_sec', math.inf)
                if (exhausted or blind_end) and not fresh:
                    # Exhaustion ends command repetition, not the current
                    # phase. A fresh same-epoch observation may still restore it.
                    self._last_command, self._last_command_at = (0., 0.), now
                    waiting = (gap_elapsed < reobserve_limit
                               and limits.get('flight_input_reobserve_sec', 0.) > 0.)
                    self.debug = dict(tracking_state='INPUT_REOBSERVING' if waiting else 'RECOVERING',
                        loss_response=not waiting, age=age, input_recovery=True,
                        input_recovery_elapsed_sec=elapsed, input_recovery_gap_sec=gap_elapsed,
                        input_recovery_budget_sec=limits['flight_input_recover_sec'],
                        input_recovery_window_used_sec=window_used,
                        input_recovery_window_budget_sec=window_budget,
                        blind_end_reached=blind_end,
                        blind_motion_age_sec=self._input_gap.get('motion_age_sec'),
                        blind_remaining_at_last_scan_m=self._input_gap.get('remaining_m'),
                        blind_speed_bound_mps=self._input_gap.get('speed_bound_mps'),
                        command=[0., 0.], physical_hold_verified=False)
                    return EvidenceReport(phase, False, not waiting,
                        'waiting for fresh flight observation' if waiting else
                        'input recovery budget exhausted', 0, 0, age)
                if exhausted and gap_elapsed >= reobserve_limit:
                    self._last_command, self._last_command_at = (0., 0.), now
                    self.debug = dict(tracking_state='RECOVERING', loss_response=True,
                        age=age, input_recovery=True, command=[0., 0.],
                        physical_hold_verified=False)
                    return EvidenceReport(phase, False, True,
                        'input recovery budget exhausted', 0, 0, age)
                if not fresh:
                    # A tread impact can delay registration while the robot is
                    # still loading its wheels. Within the existing bounded,
                    # same-epoch recovery window, repeat only the last accepted
                    # flight command; do not infer motion or increase steering.
                    self._last_command_at = now
                    self.debug = dict(tracking_state='INPUT_DELAY_CONTINUING', loss_response=False,
                        age=age, input_recovery=True, input_recovery_elapsed_sec=elapsed,
                        input_recovery_gap_sec=gap_elapsed,
                        input_recovery_budget_sec=limits['flight_input_recover_sec'],
                        input_recovery_window_used_sec=window_used,
                        input_recovery_window_budget_sec=window_budget,
                        command=list(self._last_command), physical_hold_verified=False)
                    return EvidenceReport(phase, False, False,
                        'input delayed: retaining last bounded flight command', 0, 0, age)
                self._recovery_events.append((self._input_gap['at'], now))
                self._input_gap_used = elapsed
                self._input_gap = None
                input_resume = True  # all geometry/attitude/path checks below still apply
            flat_reobserve = (phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT,
                Phase.ROOFTOP_TURN, Phase.EXIT_CONFIRM) and expired and not reset
                and not self._interrupted and sample is not None and sample.geometry_valid
                and not self.worker.diagnostics()['worker_error']
                and limits.get('flat_reobserve_sec', 0.) > age >= limits['expire_sec'])
            if flat_reobserve:
                # A stale flat pose cannot authorize movement or completion.
                # Preserve the phase for a new same-epoch observation.
                self._last_command, self._last_command_at = (0., 0.), now
                self.debug = dict(tracking_state='FLAT_REOBSERVING',
                                  loss_response=False, age=age, command=[0., 0.],
                                  physical_hold_verified=False)
                return EvidenceReport(phase, False, False,
                    'waiting for fresh flat observation', 0, 0, age)
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
            prior = self._last_sample
            pose, yaw, pitch, roll, velocity, angular, support = self._observe(sample)
            if (self.route['direction'] == 'UP' and 'pose_jump_xy_m' in limits and prior is not None
                    and sample.sequence > prior[0].sequence
                    and 0 < sample.stamp-prior[0].stamp <= limits['warn_sec']
                    and np.linalg.norm(pose[:2,3]-prior[1][:2,3]) > limits['pose_jump_xy_m']
                    and abs(float(pose[2,3]-prior[1][2,3])) > limits['pose_jump_z_m']):
                self._pose_jump_at = now if self._pose_jump_at is None else self._pose_jump_at
            if self._pose_jump_at is not None:
                # XY and Z teleport together in one scan: progress, height and
                # next-region support are then correlated, not three witnesses.
                # Await independent localization repair instead of claiming a
                # cleared riser from the same untrusted transform.
                self._last_command, self._last_command_at = (0., 0.), now
                overdue = now-self._pose_jump_at >= limits['pose_jump_reobserve_sec']
                self.debug = dict(tracking_state='POSE_JUMP_CONFLICT',
                    loss_response=overdue, phase=phase.value,
                    pose=pose[:3,3].tolist(), age=age,
                    xy_jump_m=None if prior is None else float(np.linalg.norm(pose[:2,3]-prior[1][:2,3])),
                    z_jump_m=None if prior is None else float(abs(pose[2,3]-prior[1][2,3])),
                    command=[0., 0.], physical_hold_verified=False)
                return EvidenceReport(phase, False, overdue,
                    'correlated stair pose jump requires independent confirmation'
                    if overdue else 'reobserving correlated stair pose jump',
                    0, 0, age)
            control_yaw, heading_detail = self._control_heading(sample, pose, now)
            self._observation_debug = dict(pose=pose[:3,3].tolist(), yaw=yaw, pitch=pitch, roll=roll,
                geometry_stamp=sample.stamp, geometry_sequence=sample.sequence,
                geometry_measured_at=sample.measured_at, control_yaw=control_yaw,
                heading=heading_detail, measured_velocity=velocity.tolist(), measured_yaw_rate=angular)
            self._recovering_at = None
            pos = pose[:3, 3]
            if landing_hold and abs(float(pos[2]-self._landing_hold_target[0][2])) > limits['height_tolerance']:
                self.debug = dict(tracking_state='LANDING_HOLD_HEIGHT', loss_response=True,
                                  landing_position_hold=True, command=[0.,0.])
                return EvidenceReport(phase, False, True, 'landing hold height changed from captured surface', 0.,0.,age)
            margin_context = self._support_margin_context(phase, pos)
            landing_context = margin_context == 'landing'
            margin = effective_support_margin(self.route, self.anchor.uncertainty_m,
                landing=landing_context, region=margin_context)
            landing_margin = effective_support_margin(self.route, self.anchor.uncertainty_m, landing=True)
            landing = inside_polygon(support, self.route["landing_polygon"], landing_margin)
            exit_clear = inside_polygon(support, self.route["exit_polygon"],
                effective_support_margin(self.route, self.anchor.uncertainty_m, region='exit'))
            complete = False
            settling = None
            v, w, ey, epsi, progress = 0., 0., 0., 0., 0.
            alignment = False
            rotation_waypoint = False
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
                # Use measured path velocity as feedback for the existing speed correction.
                v = np.clip(nominal + limits["speed_kp"] * deadband(nominal - actual_v, limits["speed_deadband"]), 0, limits["max_v"])
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
                        flat_start = landing if idx == 1 else inside_polygon(support, self.route['roof_landing_polygon'], landing_margin)
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
                                  inside_polygon(support, self.route['roof_landing_polygon'], landing_margin)
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
                rotation_drift_recovery = False
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
                    flat_transit = phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN) and not landing_hold
                    speed_cap = min(limits.get('landing_max_v', limits['hold_v']), limits['max_v']) if flat_transit else limits['hold_v']
                    v = np.clip(limits["hold_kp"] * along - limits["hold_kd"] * (velocity[:2] @ heading), -speed_cap, speed_cap)
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
                    if distance <= position_limit and self._landing_turn_debug.get('rotation_disk_supported', False):
                        self._holding = False
                        v = 0.
                        w = -limits['yaw_kp']*deadband(epsi, limits['yaw_deadband'])
                final_turn_approach = region_turn and self.turn_index == len(self._landing_path())-1
                needs_pivot_space = (region_turn and not final_turn_approach
                    and not self._landing_turn_debug.get('rotation_disk_supported', False))
                if (region_turn and not rotation_waypoint
                        and (final_turn_approach or self.turn_index == 2
                             or (phase is Phase.ROOFTOP_TURN and self.turn_index == 0))
                        and (distance > position_limit or needs_pivot_space)
                        and not degraded and not neutral_only):
                    # Point toward the approach target so yaw and lateral
                    # corrections cannot cancel outside the next-flight corridor.
                    bearing = math.atan2(delta[1], delta[0])
                    travel_sign = 1. if along >= 0. else -1.
                    bearing_error = wrap(bearing-yaw+(math.pi if travel_sign < 0 else 0.))
                    w = limits['yaw_kp'] * bearing_error
                    v = travel_sign * min(limits.get('landing_max_v', limits['hold_v']),
                        limits['hold_kp'] * distance) * max(0., math.cos(bearing_error))
                    if needs_pivot_space:
                        v = travel_sign * max(abs(v), min(limits.get('landing_max_v', limits['hold_v']),2*limits['hold_v']))
                    polygon = np.asarray(self._turn_polygon())
                    edges = np.roll(polygon,-1,axis=0)-polygon
                    normals = np.column_stack((-edges[:,1],edges[:,0])) / np.linalg.norm(edges,axis=1)[:,None]
                    radius = max(np.linalg.norm(self.route['footprint'],axis=1))
                    clearances = np.sum(normals*(pos[:2]-polygon),axis=1)
                    near_edge = clearances < radius+margin+limits['position_on']
                    if np.any(near_edge & ((normals @ heading)*v < -1e-9)):
                        v = 0.  # turn inward before translating toward an edge
                    self._landing_turn_debug['inward_guidance'] = True
                if (region_turn and not rotation_waypoint and not degraded and not neutral_only
                        and not self._landing_turn_debug.get('rotation_disk_supported', False)
                        and distance > limits['position_on'] and abs(v) < limits['hold_v'] / 2):
                    # A unicycle can become stationary when along-target error
                    # vanishes but lateral error remains. A short same-heading
                    # forward/reverse arc can move onto the surveyed pivot.
                    travel = self._turn_direction()
                    lateral_axis = np.array([-travel[1], travel[0]])
                    lateral_error = float(delta @ lateral_axis)
                    lateral_heading = float(heading @ lateral_axis)
                    if (abs(lateral_error) > limits['position_on']
                            and abs(lateral_heading) > .1):
                        v = math.copysign(limits['hold_v'], lateral_error*lateral_heading)
                        w = 0.
                if region_turn and rotation_waypoint and not degraded and not neutral_only:
                    # Rotation is normally in place. Only observed retreat
                    # toward the rear landing edge warrants a small inward
                    # translation; the full-body sweep below still applies.
                    travel = self._turn_direction()
                    retreat = -float(velocity[:2] @ travel)
                    heading_inward = float(np.array([math.cos(yaw), math.sin(yaw)]) @ travel)
                    flat_clearance = support_clearance(support, [self._turn_polygon()])
                    if (flat_clearance is not None and flat_clearance < limits['position_on']
                            and retreat > limits['stationary_v'] / 2
                            and heading_inward > .5):
                        self._rotation_drift_recovery = True
                    elif (flat_clearance is None or flat_clearance > limits['position_on'] + .03
                          or heading_inward <= .5):
                        self._rotation_drift_recovery = False
                    if self._rotation_drift_recovery:
                        v = limits['hold_v']
                        rotation_drift_recovery = True
                    radius = max(np.linalg.norm(self.route['footprint'],axis=1))
                    if not inside_polygon([pos[:2]], self._turn_polygon(), radius+margin):
                        # Pivot admission has its own small reserve; if the
                        # actual turning body needs more room, move inward
                        # during the turn instead of waiting for a sweep fault.
                        boundary = np.asarray(self._turn_polygon())
                        edges = np.roll(boundary,-1,axis=0)-boundary
                        normals = np.column_stack((-edges[:,1],edges[:,0])) / np.linalg.norm(edges,axis=1)[:,None]
                        gaps = np.sum(normals*(pos[:2]-boundary),axis=1)
                        inward = normals[int(np.argmin(gaps))]
                        projection = float(heading @ inward)
                        if abs(projection) > 1e-6:
                            v = math.copysign(min(limits.get('landing_max_v',limits['hold_v']),2*limits['hold_v']),projection)
                            rotation_drift_recovery = True
                            self._landing_turn_debug['inward_rotation'] = True
                turn_yaw_tolerance = (limits.get('flight_resume_corridor_yaw', limits['yaw_tolerance'])
                                      if region_turn else limits['yaw_tolerance'])
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
                                  yaw=abs(epsi) <= turn_yaw_tolerance)
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
                name = 'roof_landing_polygon' if phase is Phase.FORWARD_SEGMENT_3 else 'landing_polygon'
                names, polygons = (name,), [self.route[name]]
            raw_supported = inside_support_union(support, polygons)
            current_supported = inside_support_union(support, polygons, margin)
            key = (sample.epoch, sample.sequence, phase, names)
            if key != self._clearance_key:
                self._clearance_key = key
                self._clearance_lower = support_clearance(support, polygons) if raw_supported else None
            geometry = dict(support_regions=list(names), footprint_supported=raw_supported,
                            current_supported=current_supported, margin_m=limits["margin_m"],
                            support_margin_m=margin,
                            support_margin_context=margin_context,
                            anchor_uncertainty_m=self.anchor.uncertainty_m, effective_margin_m=margin,
                            clearance_m=self._clearance_lower,
                            height_error_m=abs(float(pos[2] - target[2])),
                            height_delta_m=float(pos[2] - target[2]),
                            ascent_height_policy=self.route.get('ascent_height_policy', 'band'),
                            ascent_height_min_m=float(target[2] - limits['height_tolerance']),
                            completion_checks=checks,
                            settling=settling,
                            incomplete_conditions=[k for k, value in checks.items() if not value])
            # An ascent can temporarily cross the comfort margin as the body
            # rocks at a riser. Require the full footprint on surveyed support,
            # a smaller route minimum, and bounded steering toward the flight
            # centerline. A stationary rotation sweep is not the trajectory of
            # the robot while the stair firmware also drives it forward.
            recovery_floor = margin if margin_context != 'route' else min(self.anchor.uncertainty_m, margin)
            turn_margin_recovery = bool(
                rotation_waypoint and margin_context == 'route' and raw_supported and not current_supported
                and not degraded and not neutral_only
                and limits.get('turn_margin_recovery_m', 0.) > 0.)
            if turn_margin_recovery:
                recovery_floor = max(limits['margin_m'],
                    margin - limits['turn_margin_recovery_m'])
            flight_margin_recovery = bool(
                phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3)
                and self.route['direction'] == 'UP' and margin_context == 'route' and not alignment and not flight_end_conflict
                and raw_supported and not current_supported and not degraded and not neutral_only
                and limits.get('flight_margin_recovery_m', 0.) > 0.)
            if flight_margin_recovery:
                recovery_floor = max(limits['margin_m'],
                    margin - limits['flight_margin_recovery_m'])
                bounded_w = self._limit_command(0., float(w), now)[1]
                flight_margin_recovery = bool(
                    inside_support_union(support, polygons, recovery_floor)
                    and ey * bounded_w < -1e-6)
                if not flight_margin_recovery:
                    recovery_floor = margin
            recovery = not current_supported and inside_support_union(support, polygons, recovery_floor)
            geometry["recovery_command_margin_m"] = float(
                recovery_floor if recovery else margin)
            geometry["turn_margin_recovery"] = bool(turn_margin_recovery and recovery)
            geometry["flight_margin_recovery"] = bool(flight_margin_recovery and recovery)
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
                if phase in (Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN) and rotation_waypoint and not rotation_drift_recovery:
                    v = 0.
                if alignment or phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN, Phase.EXIT_CONFIRM):
                    speed_cap = (limits.get('landing_max_v', limits['hold_v'])
                        if phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN) and not landing_hold
                        else limits['hold_v'])
                    v = float(np.clip(v, -min(speed_cap,limits['max_v']), min(speed_cap,limits['max_v'])))
            # Sensor expiry is a data-freshness deadline, not the flight
            # lookahead. Every control cycle rechecks the observed full body.
            on_flight = (phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3)
                         and not alignment and not flight_end_conflict)
            horizon = (limits.get("flight_prediction_sec", limits["expire_sec"])
                       if on_flight else limits["expire_sec"])
            proposed = (v, w)
            adjusted = False
            restart = False
            if not transition and on_flight:
                # The commissioned route consumes the measured feedback command directly.
                # A fresh observation after eligible input loss restores the configured input.
                if input_resume and not neutral_only and not degraded:
                    v = self.route['flight_input_v'][idx] if 'flight_input_v' in self.route else limits['flight_restart_v']
                    proposed = (v, w)
                    restart = True
            elif not transition and (recovery or not self._swept_support(pos[:2], yaw, support, v, w, horizon, polygons, margin)):
                entry_reserve = (phase is Phase.ALIGN and corridor_entry and recovery
                                 and not degraded and not neutral_only
                                 and abs(pos[2]-target[2]) <= limits['height_tolerance'])
                alternative = (self._entry_reserve_command(pos[:2], yaw, support, w,
                    horizon, polygons, margin, now) if entry_reserve else
                    None if neutral_only or flight_end_conflict else self._supported_command(
                    pos[:2], yaw, support, v, w, horizon, polygons,
                    recovery_floor if recovery else margin, now))
                if alternative is None:
                    self.debug = dict(tracking_state="SUPPORT_LIMIT", loss_response=True,
                                      proposed_command=[v, w], prediction_horizon_sec=horizon, **geometry)
                    return EvidenceReport(phase, False, True, "no bounded correction within connected support", 0, 0, age)
                v, w = alternative
                adjusted = True
            if turn_margin_recovery and recovery and not transition:
                if (abs(v) > 1e-6 or abs(w) < 1e-6 or
                        not self._turn_recovery_improves_clearance(
                            pos[:2], support, w, horizon, polygons)):
                    self.debug = dict(tracking_state="SUPPORT_LIMIT", loss_response=True,
                                      proposed_command=list(proposed), **geometry)
                    return EvidenceReport(phase, False, True,
                        "no inward rotation within connected support", 0, 0, age)
            if not transition:
                if on_flight and not degraded and v > 0. and not recovery and not input_resume and not restart:
                    self._credit_stable_progress(sample, pos, direction, now)
                else:
                    self._stable_progress = None
                self._last_command, self._last_command_at = (v, w), now
            elif phase is Phase.VERIFY_ENTRY:
                self._entry_verified = True
            prediction = dict(state='disabled', affects_commands=False)
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
                              flight_command_policy=self.route.get('flight_command_policy', 'observed_feedback'),
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
                              prediction_model='disabled; measured feedback controls ascent',
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



    @staticmethod
    def _turn_recovery_improves_clearance(position, support, w, horizon, polygons):
        before = support_clearance(support, polygons)
        if before is None:
            return False
        theta = w * horizon
        c, s = math.cos(theta), math.sin(theta)
        relative = support-position
        end = np.column_stack((c*relative[:,0]-s*relative[:,1],
                               s*relative[:,0]+c*relative[:,1]))+position
        after = support_clearance(end, polygons)
        return after is not None and after >= before-1e-6

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


    def _reset_continuity(self):
        self._stable_progress = None
        self._flight_heading_state = None
        self._recovery_events = deque()
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
            self._input_gap_used = 0.

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
        rotation_margin = self.route['limits'].get('rotation_space_margin_m', margin)
        # Do not rebase the next pivot a few millimetres from the sweep boundary.
        # Keep the cross-landing coordinate; clamp only flight-depth to an inset
        # interval with room for the existing position tolerance and body margin.
        polygon = np.asarray(self._turn_polygon(), dtype=float)
        edges = np.roll(polygon,-1,axis=0)-polygon
        normals = np.column_stack((-edges[:,1],edges[:,0])) / np.linalg.norm(edges,axis=1)[:,None]
        desired = float((next_pivot-old) @ direction)
        for reserve in (self.route['limits']['position_on'], 0.):
            low, high = -math.inf, math.inf
            inset = radius+max(rotation_margin,margin)+reserve
            for normal,vertex in zip(normals,polygon):
                slope = float(normal @ direction)
                deficit = inset-float(normal @ (old-vertex))
                if abs(slope) < 1e-9:
                    if deficit > 1e-9: low,high=1.,0.;break
                elif slope > 0: low=max(low,deficit/slope)
                else: high=min(high,deficit/slope)
            if low <= high:
                next_pivot = old+float(np.clip(desired,low,high))*direction
                break
        if not inside_polygon([next_pivot], self._turn_polygon(), radius+rotation_margin):
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
        rotation_margin = self.route['limits'].get('rotation_space_margin_m', margin)
        clear = inside_polygon([pose[:2,3]], self._turn_polygon(), radius+rotation_margin)
        self._landing_turn_debug = dict(rotation_disk_supported=clear,
            rotation_radius_m=radius, margin_m=rotation_margin,
            body_support_margin_m=margin,
            pivot_captured=getattr(self, '_landing_turn_captured', False))
        index = self.turn_index
        if index+1 >= len(path) or degraded or not clear:
            return
        if (not np.allclose(path[index][:2], path[index+1][:2])
                or abs(wrap(path[index+1][2]-path[index][2])) < .1):
            return
        if index > 0:
            # Turn once aligned with the next flight's usable width, not only
            # at one XY point. The full pivot disk still has to fit the landing.
            flight = np.asarray(self.route['flight_2' if self.phase is Phase.TURN_TO_NEXT_FLIGHT else 'flight_3'])
            forward = flight[1,:2]-flight[0,:2];forward/=np.linalg.norm(forward)
            side = np.array([-forward[1],forward[0]])
            polygon = np.asarray(self.route['flight_2_polygon' if self.phase is Phase.TURN_TO_NEXT_FLIGHT else 'flight_3_polygon'])
            footprint = np.asarray(self.route['footprint'])
            body = pose[:2,3]+footprint[:,0,None]*forward+footprint[:,1,None]*side
            width, projected = polygon @ side, body @ side
            if projected.min() < width.min()+margin or projected.max() > width.max()-margin:
                return
        if self.phase is Phase.ROOFTOP_TURN and index == 0:
            # Reach the middle neighborhood before an in-place roof turn.
            # This is a capture region, not an exact XY settling requirement.
            center = np.asarray(self.route['roof_turn_path'][0][:2])
            radius = self.route['limits'].get('roof_pivot_radius_m', .35)
            distance = float(np.linalg.norm(pose[:2,3]-center))
            self._landing_turn_debug.update(center_distance_m=distance, center_radius_m=radius)
            if distance > radius:
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
        # At the roof's short final flight, a high-biased Z estimate must not
        # immobilize an otherwise supported turn/entry. UP entry height is a
        # lower bound; full ascent still needs progress, height, and exit support.
        height_ok = (float(pose[2,3]-start[2]) >= -limits['height_tolerance']
                     if idx in (1, 2) and self.route['direction'] == 'UP' else
                     abs(float(pose[2,3]-start[2])) <= limits['height_tolerance'])
        heading_into_flight = True
        if idx == 2 and self.route['direction'] == 'UP':
            # Test current heading at the first riser, not a future actuator model.
            # A supported body can still be aimed out of the narrow final flight.
            heading = pose[:2,0]
            advance = float(heading @ direction)
            distance = max(0., first_edge-along)
            shift = heading * (distance / advance) if advance > .1 else np.zeros(2)
            crossing_width = (support+shift) @ lateral
            heading_into_flight = bool(advance > .1 and crossing_width.min() >= width.min()+margin
                                       and crossing_width.max() <= width.max()-margin)
            if hasattr(self, '_landing_turn_debug'):
                self._landing_turn_debug['heading_into_final_flight'] = heading_into_flight
        return bool(heading_into_flight
            and body_width.min() >= width.min()+margin and body_width.max() <= width.max()-margin
            and inside_support_union(support,regions,margin)
            and along <= first_edge+margin
            and height_ok
            and abs(epsi) <= limits['flight_resume_corridor_yaw'])






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


    def _entry_reserve_command(self, position, yaw, support, w, horizon, polygons, margin, now):
        """Restore only a legacy route's extra entry reserve with its heading turn.

        No forward/reverse input or opposite steering is invented. The swept
        body stays above the smaller of anchor uncertainty and configured
        support margin, and improves clearance without first reducing it.
        Explicit commissioned margins remain hard boundaries. This check is
        geometric; it is not physical drift holding.
        """
        current = support_clearance(support, polygons)
        hard = min(self.anchor.uncertainty_m, margin)
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
        """Find a smaller same-direction command whose full-body sweep fits.

        A flat landing may brake below hold_v immediately: the flat speed cap
        already overrides flight deceleration slew. Reduction does not grant
        faster acceleration or reverse steering. Zero/zero is not progress.
        """
        if self._swept_support(position, yaw, support, v, w, horizon, polygons, margin):
            return v, w
        seen = {(v, w)}
        flat = self.phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT,
                              Phase.ROOFTOP_TURN, Phase.EXIT_CONFIRM)
        angular_scales = (1., .85, .7, .5, .25) if flat else (1.,)
        for v_scale in (.75, .5, .25, 0.):
            for w_scale in angular_scales:
                limited = self._limit_command(v * v_scale, w * w_scale, now)
                candidate = ((math.copysign(min(abs(limited[0]), abs(v * v_scale)), v)
                              if flat else limited[0]),
                             (math.copysign(min(abs(limited[1]), abs(w * w_scale)), w)
                              if flat else limited[1]))
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
        immediate_ascent = (self.phase in (Phase.FORWARD_SEGMENT_1, Phase.FORWARD_SEGMENT_2, Phase.FORWARD_SEGMENT_3)
                            and self.route.get("direction") == "UP" and v > old_v and v > 0.)
        if immediate_ascent:
            # Stair firmware needs the commissioned full forward input to load
            # the wheels at a riser and pitch the body. Keep angular slew and
            # all non-ascent linear limits unchanged.
            linear = float(np.clip(v, 0., limits["max_v"]))
        else:
            linear = float(np.clip(v, max(-limits["max_v"], old_v - limits["max_accel"] * dt), min(limits["max_v"], old_v + limits["max_accel"] * dt)))
        if self.phase in (Phase.LANDING, Phase.TURN_TO_NEXT_FLIGHT, Phase.ROOFTOP_TURN, Phase.EXIT_CONFIRM):
            # Flat-region command cap takes precedence over deceleration slew:
            # never carry the previous flight's high input onto the landing.
            # This limits requested input; it does not prove physical braking.
            cap = (limits.get('landing_max_v', limits['hold_v'])
                if self.phase is not Phase.EXIT_CONFIRM else limits['hold_v'])
            linear = float(np.clip(linear, -min(cap,limits['max_v']), min(cap,limits['max_v'])))
        # Rotation is normally commanded at zero translation by _evaluate.
        # A measured rear-edge backslide may instead request a small inward
        # flat correction, subject to the same swept-support test.
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
                          prediction_recheck_active=False, prediction_recheck_used_sec=0.)
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
                context = self._support_margin_context(self.phase)
                fallback = effective_support_margin(self.route, self.anchor.uncertainty_m,
                    landing=context == 'landing', region=context)
                result.setdefault('support_margin_m', fallback)
                result.setdefault('effective_margin_m', fallback)
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
