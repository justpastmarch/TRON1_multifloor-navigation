#!/usr/bin/env python3
"""Build a 3D point cloud map from a raw Livox BAG and a 2D SLAM trajectory.

Self-contained (numpy + rosbag only) so the same file runs beside the raw BAG
on the mini PC, where the LiDAR stream is recorded; only the small result is
copied back. Read-only on the BAG; no ROS node, no robot transport.

Pose model (flat floor): x, y, yaw come from the 2D SLAM Toolbox trajectory
(map -> base_Link, interpolated per point, so motion within a sweep is
de-skewed); roll and pitch come from the LiDAR IMU through a complementary
filter; height is constant. It is a visual/survey aid, not a calibrated
3D SLAM result.

Usage: mapping_cloud.py RAW_BAG TRAJECTORY_JSON PARAMS_JSON OUTPUT_DIR
Prints "PROGRESS <0..1>" lines while reading the BAG.
"""
import json
import math
import os
from pathlib import Path
import struct
import sys
import time

import numpy as np

POINT = np.dtype([('offset', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                  ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1')])
assert POINT.itemsize == 19

DEFAULTS = dict(voxel_m=0.05, min_range_m=0.5, max_range_m=25.0, min_hits=2,
                preview_points=400000, gravity_gain=0.01)


def parse_livox(data):
    """Raw serialized livox_ros_driver2/CustomMsg -> (timebase_s, points)."""
    offset = 12  # header seq, stamp secs, stamp nsecs
    (frame_len,) = struct.unpack_from('<I', data, offset)
    offset += 4 + frame_len
    timebase, point_num = struct.unpack_from('<QI', data, offset)
    offset += 12 + 4  # timebase, point_num, lidar_id, rsvd[3]
    (count,) = struct.unpack_from('<I', data, offset)
    offset += 4
    if count != point_num:
        raise ValueError('Livox point count mismatch')
    points = np.frombuffer(data, dtype=POINT, count=count, offset=offset)
    return timebase * 1e-9, points


def rotation_to_up(up):
    """Smallest rotation taking unit vector `up` onto +z (Rodrigues)."""
    up = up / np.linalg.norm(up)
    axis = np.cross(up, [0., 0., 1.])
    s, c = np.linalg.norm(axis), float(up[2])
    if s < 1e-9:
        return np.eye(3)
    k = axis / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


def gravity_track(imu_rows, lidar_from_imu, gain):
    """Complementary filter -> (times, unit 'up' vectors in the LiDAR frame)."""
    rows = np.asarray(imu_rows, float)
    if len(rows) < 10:
        raise ValueError('IMU 데이터가 부족합니다.')
    R = np.asarray(lidar_from_imu, float)
    gyro = rows[:, 1:4] @ R.T
    acc = rows[:, 4:7] @ R.T
    up = acc[:min(len(acc), 100)].mean(axis=0)
    up /= np.linalg.norm(up)
    out = np.empty_like(acc)
    previous = rows[0, 0]
    for i in range(len(rows)):
        dt = min(max(rows[i, 0] - previous, 0.), 0.05)
        previous = rows[i, 0]
        up = up - np.cross(gyro[i], up) * dt
        a = acc[i]
        norm = np.linalg.norm(a)
        if norm > 1e-6:
            up = (1 - gain) * up + gain * a / norm
        up /= np.linalg.norm(up)
        out[i] = up
    return rows[:, 0], out


class VoxelGrid:
    def __init__(self, size):
        self.size = size
        self.keys = np.empty(0, np.int64)
        self.sums = np.empty((0, 3))
        self.refl = np.empty(0)
        self.hits = np.empty(0, np.int64)

    def add(self, xyz, reflectivity):
        if not len(xyz):
            return
        cells = np.floor(xyz / self.size).astype(np.int64) + (1 << 20)
        keys = (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]
        keys = np.concatenate([self.keys, keys])
        sums = np.concatenate([self.sums, xyz])
        refl = np.concatenate([self.refl, reflectivity.astype(float)])
        hits = np.concatenate([self.hits, np.ones(len(xyz), np.int64)])
        unique, inverse = np.unique(keys, return_inverse=True)
        self.keys = unique
        self.sums = np.zeros((len(unique), 3))
        for axis in range(3):
            self.sums[:, axis] = np.bincount(inverse, sums[:, axis], len(unique))
        self.refl = np.bincount(inverse, refl, len(unique))
        self.hits = np.bincount(inverse, hits, len(unique)).astype(np.int64)

    def result(self, min_hits):
        keep = self.hits >= min_hits
        hits = self.hits[keep][:, None].astype(float)
        return self.sums[keep] / hits, self.refl[keep] / hits[:, 0], self.hits[keep]


def write_ply(path, xyz, intensity):
    header = ('ply\nformat binary_little_endian 1.0\nelement vertex %d\n'
              'property float x\nproperty float y\nproperty float z\n'
              'property uchar intensity\nend_header\n' % len(xyz)).encode()
    rows = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', 'u1')])
    rows['x'], rows['y'], rows['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rows['i'] = np.clip(intensity, 0, 255).astype(np.uint8)
    with open(str(path), 'wb') as stream:
        stream.write(header)
        stream.write(rows.tobytes())


def write_preview(path, xyz, intensity, limit):
    """Compact browser format: uint32 count, float32 xyz[count], uint8 i[count]."""
    if len(xyz) > limit:
        pick = np.random.RandomState(0).choice(len(xyz), limit, replace=False)
        xyz, intensity = xyz[pick], intensity[pick]
    with open(str(path), 'wb') as stream:
        stream.write(struct.pack('<I', len(xyz)))
        stream.write(np.ascontiguousarray(xyz, '<f4').tobytes())
        stream.write(np.clip(intensity, 0, 255).astype(np.uint8).tobytes())
    return len(xyz)


def write_transport(path, xyz, intensity):
    """1 cm integer coordinates (finer than the voxel), compressed."""
    origin = np.floor(xyz.min(axis=0))
    cm = np.round((xyz - origin) * 100)
    if cm.max() > 65535:
        raise ValueError('3D 범위가 너무 넓습니다 (655 m 초과).')
    with open(str(path), 'wb') as stream:
        np.savez_compressed(stream, origin=origin, cm=cm.astype(np.uint16),
                            intensity=np.clip(intensity, 0, 255).astype(np.uint8))


def finish_transport(directory, preview_points=DEFAULTS['preview_points']):
    """cloud.npz -> cloud.ply + cloud.bin (workstation side of a remote build)."""
    directory = Path(directory)
    with np.load(str(directory / 'cloud.npz')) as data:
        xyz = data['cm'].astype(float) / 100 + data['origin']
        intensity = data['intensity']
    write_ply(directory / 'cloud.ply', xyz, intensity)
    write_preview(directory / 'cloud.bin', xyz, intensity, preview_points)
    (directory / 'cloud.npz').unlink()


def thin_uniform(xyz, limit):
    """Keep one point per cell, growing the cell until at most `limit` remain:
    walls and floors stay complete instead of thinning randomly."""
    if len(xyz) <= limit:
        return xyz
    low = xyz.min(axis=0)
    # Surface-like clouds shrink with the square of the cell size.
    size = max(0.05, 0.05 * math.sqrt(len(xyz) / limit) * 0.8)
    while True:
        cells = np.floor((xyz - low) / size).astype(np.int64)
        keys = (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]
        _, first = np.unique(keys, return_index=True)
        if len(first) <= limit:
            return xyz[np.sort(first)]
        size *= 1.15


def read_ply_xyz(path):
    """All points of a cloud.ply written by write_ply (float x, y, z, uchar i)."""
    data = Path(path).read_bytes()
    end = data.index(b'end_header\n') + len(b'end_header\n')
    count = int(data[:end].split(b'element vertex ')[1].split(b'\n')[0])
    rows = np.frombuffer(data, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', 'u1')], count=count, offset=end)
    return np.stack([rows['x'], rows['y'], rows['z']], axis=1)


def compact_preview(xyz, limit):
    """points -> b'TRC2' + uint32 n + float32 origin[3] + uint16 cm xyz[n].
    About half the size of float32 and thinned to `limit` points, for
    viewing over a slow link; 1 cm is finer than the 5 cm voxels."""
    if isinstance(xyz, (bytes, bytearray)):
        (count,) = struct.unpack_from('<I', xyz, 0)
        xyz = np.frombuffer(xyz, '<f4', count * 3, 4).reshape(-1, 3)
    xyz = thin_uniform(np.asarray(xyz), limit)
    origin = xyz.min(axis=0) if len(xyz) else np.zeros(3, np.float32)
    cm = np.clip(np.round((xyz - origin) * 100), 0, 65535).astype('<u2')
    return b'TRC2' + struct.pack('<I', len(xyz)) + np.asarray(origin, '<f4').tobytes() + cm.tobytes()


def build(raw_bag, trajectory, params, output, progress=None):
    import rosbag
    started = time.monotonic()
    settings = dict(DEFAULTS, **{k: v for k, v in params.items() if k in DEFAULTS})
    base_from_lidar = np.asarray(params['base_from_lidar'], float).reshape(4, 4)
    Rbl, tbl = base_from_lidar[:3, :3], base_from_lidar[:3, 3]
    track = np.asarray(trajectory, float)
    if track.ndim != 2 or track.shape[1] != 4 or len(track) < 2:
        raise ValueError('2D 궤적이 부족합니다.')
    track = track[np.argsort(track[:, 0])]
    t_track = track[:, 0]
    yaw_track = np.unwrap(track[:, 3])
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    with rosbag.Bag(str(raw_bag)) as bag:
        topics = bag.get_type_and_topic_info().topics
        if '/livox/lidar' not in topics or '/livox/imu' not in topics:
            raise ValueError('원본 BAG에 /livox/lidar 또는 /livox/imu가 없습니다.')
        imu = [(m.header.stamp.to_sec(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
               for _, m, _ in bag.read_messages(topics=['/livox/imu'])]
        imu.sort()
        t_imu, up_lidar = gravity_track(imu, params['lidar_from_imu'], settings['gravity_gain'])
        total = max(topics['/livox/lidar'].message_count, 1)
        grid = VoxelGrid(settings['voxel_m'])
        batch_xyz, batch_refl = [], []
        used = skipped = raw_points = 0
        for index, (_, message, _) in enumerate(bag.read_messages(topics=['/livox/lidar'], raw=True)):
            if progress and index % 20 == 0:
                progress(index / total)
            timebase, points = parse_livox(message[1])
            stamp = timebase + points['offset'].astype(float) * 1e-9 if len(points) else np.empty(0)
            if not len(points) or stamp[-1] < t_track[0] or stamp[0] > t_track[-1]:
                skipped += 1
                continue
            xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(float)
            distance = np.linalg.norm(xyz, axis=1)
            keep = ((distance >= settings['min_range_m']) & (distance <= settings['max_range_m']) &
                    ((points['tag'] & 0x0F) == 0) & (stamp >= t_track[0]) & (stamp <= t_track[-1]))
            xyz, stamp, refl = xyz[keep], stamp[keep], points['reflectivity'][keep]
            raw_points += len(points)
            if not len(xyz):
                continue
            middle = float(stamp[len(stamp) // 2])
            up = np.array([np.interp(middle, t_imu, up_lidar[:, k]) for k in range(3)])
            level = rotation_to_up(Rbl @ up)
            base = (xyz @ Rbl.T + tbl) @ level.T
            x = np.interp(stamp, t_track, track[:, 1])
            y = np.interp(stamp, t_track, track[:, 2])
            yaw = np.interp(stamp, t_track, yaw_track)
            c, s = np.cos(yaw), np.sin(yaw)
            world = np.stack([c * base[:, 0] - s * base[:, 1] + x,
                              s * base[:, 0] + c * base[:, 1] + y, base[:, 2]], axis=1)
            batch_xyz.append(world)
            batch_refl.append(refl)
            used += 1
            if sum(len(b) for b in batch_xyz) > 1500000:
                grid.add(np.concatenate(batch_xyz), np.concatenate(batch_refl))
                batch_xyz, batch_refl = [], []
        if batch_xyz:
            grid.add(np.concatenate(batch_xyz), np.concatenate(batch_refl))
    if not used:
        raise ValueError('2D 궤적과 겹치는 LiDAR 스캔이 없습니다. 같은 주행의 원본 BAG인지 확인하세요.')
    xyz, intensity, hits = grid.result(settings['min_hits'])
    if not len(xyz):
        raise ValueError('3D 점이 남지 않았습니다.')
    if params.get('transport'):
        # Remote build: one compact file crosses the (possibly slow) link;
        # finish_transport() writes PLY and preview on the workstation.
        write_transport(output / 'cloud.npz', xyz, intensity)
        shown = min(len(xyz), settings['preview_points'])
    else:
        write_ply(output / 'cloud.ply', xyz, intensity)
        shown = write_preview(output / 'cloud.bin', xyz, intensity, settings['preview_points'])
    low, high = xyz.min(axis=0), xyz.max(axis=0)
    result = dict(method='3d', points=int(len(xyz)), preview_points=int(shown), scans_used=used,
                  scans_outside_trajectory=skipped, raw_points=int(raw_points),
                  voxel_m=settings['voxel_m'], min_hits=settings['min_hits'],
                  bounds=[low.round(3).tolist(), high.round(3).tolist()],
                  wall_seconds=time.monotonic() - started, saved=time.time(),
                  pose_model='2D SLAM Toolbox x/y/yaw + IMU roll/pitch, flat floor')
    with open(str(output / 'result.json'), 'w') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    if progress:
        progress(1.)
    return result


def main(argv):
    if len(argv) != 5:
        print(__doc__, file=sys.stderr)
        return 2
    trajectory = json.loads(Path(argv[2]).read_text())
    params = json.loads(Path(argv[3]).read_text())

    def progress(value):
        print('PROGRESS %.3f' % value, flush=True)
    result = build(argv[1], trajectory, params, argv[4], progress)
    print('RESULT ' + json.dumps(result), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
