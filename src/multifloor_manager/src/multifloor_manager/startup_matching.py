"""Bounded, stationary scan-to-map initialization; never a motion controller.

Searches the selected floor, not a stored home pose. Scores are geometric
agreement, not probabilities or proof of the physical robot's location.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class Match:
    pose: tuple
    score: float
    hit_fraction: float
    margin: float
    unique: bool


class ScanMatcher:
    def __init__(self, grid, resolution, origin, step=0.40, beams=90,
                 min_hit=0.65, min_margin=0.06):
        self.grid = np.asarray(grid, dtype=np.int16)
        self.resolution = float(resolution)
        self.origin = tuple(origin)
        self.step, self.beams = float(step), int(beams)
        self.min_hit, self.min_margin = float(min_hit), float(min_margin)
        if (self.grid.ndim != 2 or not np.any(self.grid >= 65)
                or not np.any(self.grid == 0) or not self.resolution > 0
                or not np.isfinite(self.origin).all() or self.step <= 0
                or self.beams < 24 or not 0 < min_hit <= 1 or not 0 < min_margin < 1):
            raise ValueError("invalid localization map or search settings")
        self.distance = distance_transform_edt(self.grid < 65) * self.resolution
        stride = max(1, int(round(self.step / self.resolution)))
        y, x = np.where(self.grid[::stride, ::stride] == 0)
        xy = np.column_stack(((x * stride + .5) * self.resolution,
                              (y * stride + .5) * self.resolution))
        a = self.origin[2]
        rotation = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        self.positions = xy @ rotation.T + self.origin[:2]
        if not len(self.positions) or len(self.positions) > 100000:
            raise ValueError("map search size unsupported; adjust coarse step")

    def points(self, scan):
        points = np.asarray(scan, dtype=float)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) < 24:
            raise ValueError("insufficient finite scan returns")
        return points[np.linspace(0, len(points)-1, min(self.beams, len(points)), dtype=int)]

    def _cells(self, xy):
        xy = xy - self.origin[:2]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        x = np.floor((xy[..., 0]*c + xy[..., 1]*s) / self.resolution).astype(int)
        y = np.floor((-xy[..., 0]*s + xy[..., 1]*c) / self.resolution).astype(int)
        valid = (x >= 0) & (y >= 0) & (x < self.grid.shape[1]) & (y < self.grid.shape[0])
        return np.clip(x, 0, self.grid.shape[1]-1), np.clip(y, 0, self.grid.shape[0]-1), valid

    def score(self, poses, points):
        poses = np.asarray(poses).reshape(-1, 3)
        c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
        rays = np.stack((c*points[:, 0]-s*points[:, 1], s*points[:, 0]+c*points[:, 1]), axis=-1)
        ends = poses[:, None, :2] + rays
        x, y, valid = self._cells(ends)
        distances = np.where(valid & (self.grid[y, x] >= 0), self.distance[y, x], 10.)
        hit = np.mean(distances <= .20, axis=1)
        score = np.mean(np.exp(-.5 * (distances/.20)**2), axis=1)
        # A pose that puts the sensor in a wall/unknown cell is never a candidate.
        px, py, free = self._cells(poses[:, :2])
        free &= self.grid[py, px] == 0
        # Penalize rays through walls: endpoint agreement alone admits false rooms.
        crossing = np.zeros(len(poses))
        for fraction in (.25, .5, .75):
            rx, ry, inside = self._cells(poses[:, None, :2] + rays * fraction)
            crossing += np.mean(~inside | (self.grid[ry, rx] >= 65), axis=1) / 3
        return np.where(free, score - crossing, -1.), np.where(free, hit, 0.)

    @staticmethod
    def separated(a, b):
        yaw = math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2]))
        return math.hypot(a[0]-b[0], a[1]-b[1]) > .70 or abs(yaw) > math.radians(20)

    def _distinct(self, candidates, count=16):
        selected = []
        for item in sorted(candidates, key=lambda v: v[0], reverse=True):
            if all(self.separated(item[1], other[1]) for other in selected):
                selected.append(item)
                if len(selected) == count:
                    break
        return selected

    def search(self, scan, cancelled=lambda: False):
        points = self.points(scan)
        best = []
        for yaw in np.arange(-math.pi, math.pi, math.radians(10)):
            for offset in range(0, len(self.positions), 512):
                if cancelled():
                    raise InterruptedError("localization request superseded")
                xy = self.positions[offset:offset+512]
                poses = np.column_stack((xy, np.full(len(xy), yaw)))
                scores, _ = self.score(poses, points)
                indices = np.argsort(scores)[-20:]
                best.extend((float(scores[i]), tuple(poses[i])) for i in indices)
            best = self._distinct(best, 24)
        refined = []
        for _, pose in best:
            for span, step, yaw_span, yaw_step in ((.4,.1,10.,2.), (.10,.025,2.,.5)):
                if cancelled():
                    raise InterruptedError("localization request superseded")
                dx, dy, da = np.meshgrid(np.arange(-span, span+step/2, step),
                                        np.arange(-span, span+step/2, step),
                                        np.deg2rad(np.arange(-yaw_span, yaw_span+yaw_step/2, yaw_step)), indexing='ij')
                offsets = np.column_stack((dx.ravel(), dy.ravel(), da.ravel()))
                poses = offsets + pose
                scores, _ = self.score(poses, points)
                winner = int(np.argmax(scores))
                pose, score = tuple(poses[winner]), float(scores[winner])
            refined.append((score, pose))
        ranked = self._distinct(refined)
        score, pose = ranked[0]
        margin = score - ranked[1][0] if len(ranked) > 1 else 1.
        _, hit = self.score([pose], points)
        return Match(pose, score, float(hit[0]), margin,
                     bool(hit[0] >= self.min_hit and score >= self.min_hit and margin >= self.min_margin))
