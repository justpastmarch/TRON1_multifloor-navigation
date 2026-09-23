#!/usr/bin/env python3
"""Sequential original-bag regression; never opens a robot transport or ROS node."""
import argparse
from collections import deque
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rosbag
from scipy.spatial.transform import Rotation

from stair_supervisor.lidar_tracking import TrackingSettings, TrackingEngine, load_imu_rotation, points_from_livox
from stair_supervisor.lidar_tracking_core import RegistrationConfig


def replay(spec_path, output, reference=None):
    spec = json.loads(Path(spec_path).read_text())
    settings = TrackingSettings(frame_step=spec["frame_step"], local_map_scans=spec["local_map_scans"],
        max_range_m=spec["max_range_m"], registration=RegistrationConfig(**spec["registration"]))
    rotation, checksum = load_imu_rotation(spec["imu_calibration_path"])
    engine = TrackingEngine(settings, rotation, checksum)
    imu = deque(maxlen=settings.imu_capacity)
    pending = deque()
    rows = []
    count = 0
    def drain():
        while pending and imu:
            stamp, points, received = pending[0]
            required = stamp + (settings.gravity_half_window_s if engine.last_stamp is None else 0)
            if imu[-1][0] < required:
                break
            pending.popleft()
            result = engine.process(stamp, points, tuple(imu), received, stamp)
            pose = np.asarray(result.transform).reshape(4, 4) if result.transform else None
            admitted = result.geometry_valid or result.state == "BOOTSTRAP"
            rows.append(dict(stamp=stamp, state=result.state, accepted=admitted,
                pose=None if not admitted else pose.tolist(), compute_sec=result.compute_sec,
                latest_imu_used=imu[-1][0], fitness=result.fitness, rmse=result.rmse))
    started = time.monotonic()
    with rosbag.Bag(spec["bag_path"], 'r') as bag:
        first = bag.get_start_time() + spec["start_offset_s"]
        end = first + spec["duration_s"]
        for topic, message, recorded in bag.read_messages(topics=[spec.get("imu_topic", "/livox/imu"), spec.get("lidar_topic", "/livox/lidar")]):
            if topic == spec.get("imu_topic", "/livox/imu"):
                w, a = message.angular_velocity, message.linear_acceleration
                stamp = message.header.stamp.to_sec()
                if not imu or stamp > imu[-1][0]:
                    imu.append((stamp, w.x, w.y, w.z, a.x, a.y, a.z))
                    while len(imu) > 2 and imu[1][0] < stamp - settings.imu_buffer_sec:
                        imu.popleft()
            elif first <= recorded.to_sec() <= end:
                if count % settings.frame_step == 0:
                    pending.append((message.header.stamp.to_sec(), points_from_livox(message), recorded.to_sec()))
                count += 1
            drain()
            if recorded.to_sec() > end + settings.gravity_half_window_s + 5 and not pending:
                break
    metrics = dict(spec=str(spec_path), input_bag=spec["bag_path"], calibration_sha256=checksum,
        frames=len(rows), accepted=sum(row["accepted"] for row in rows), pending=len(pending),
        elapsed_sec=time.monotonic() - started,
        compute_p50_sec=float(np.percentile([r["compute_sec"] for r in rows],50)),
        compute_p99_sec=float(np.percentile([r["compute_sec"] for r in rows],99)),
        physical_accuracy="UNVERIFIED; numerical baseline equivalence only")
    if reference:
        originals = list(csv.DictReader(Path(reference).open()))
        errors, angles, mismatch = [], [], []
        for i, (row, old) in enumerate(zip(rows, originals)):
            old_valid = old["accepted"].lower() in ("true", "1")
            if abs(row["stamp"] - int(old["stamp_ns"]) / 1e9) > 1e-6 or row["accepted"] != old_valid or row["state"] != old["state"]:
                mismatch.append(i)
            if row["accepted"] and old_valid:
                pose = np.asarray(row["pose"])
                errors.append(float(np.linalg.norm(pose[:3,3]-[float(old[k]) for k in ("x_m","y_m","z_m")])))
                rotation_old = Rotation.from_quat([float(old[k]) for k in ("qx","qy","qz","qw")])
                angles.append(float((rotation_old.inv()*Rotation.from_matrix(pose[:3,:3])).magnitude()))
        metrics.update(reference=str(reference), reference_sha256=hashlib.sha256(Path(reference).read_bytes()).hexdigest(),
            reference_frames=len(originals), state_or_stamp_mismatch=mismatch, max_translation_difference_m=max(errors,default=None),
            max_rotation_difference_deg=float(np.degrees(max(angles))) if angles else None)
        metrics["numerical_pass"] = len(rows)==len(originals) and not mismatch and bool(errors) and max(errors)<=.001 and max(angles)<=np.radians(.05) and not pending
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    (output/'poses.json').write_text(json.dumps(rows,allow_nan=False))
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    print(json.dumps(metrics,allow_nan=False),flush=True)
    return metrics


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('spec');parser.add_argument('output');parser.add_argument('--reference')
    args=parser.parse_args()
    result=replay(args.spec,args.output,args.reference)
    raise SystemExit(0 if result.get('numerical_pass',True) else 1)
