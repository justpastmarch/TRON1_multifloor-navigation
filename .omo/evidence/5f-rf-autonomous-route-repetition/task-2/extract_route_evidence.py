#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.8,<3.9"
# dependencies = [
#   "numpy==1.24.4",
#   "polars==1.8.2",
#   "pyarrow==17.0.0",
#   "pycryptodomex==3.23.0",
#   "pyyaml==6.0.2",
#   "python-gnupg==0.5.5",
#   "rospkg==1.6.0",
#   "typer==0.16.0",
# ]
# ///

# ─── How to run ───
# 1. Install uv: curl -LsSf https://astral.sh/uv/install.sh | sh
# 2. Source ROS Noetic and the Livox message workspace.
# 3. Run: uv run --python /usr/bin/python3 extract_route_evidence.py BAG LIDAR_CSV OUTPUT_DIR
# ──────────────────
"""Export non-interpolated route evidence from a ROS1 bag and offline-LIO output."""

from __future__ import annotations

import json
import math
from collections.abc import Iterable
from pathlib import Path
from typing import Final, NamedTuple

import numpy as np
import polars as pl
import rosbag
import typer

GAP_THRESHOLD_S: Final = 0.5
JUMP_THRESHOLD_M: Final = 0.5
TOPICS: Final = (
    "/livox/lidar",
    "/livox/imu",
    "/tron/wheel_odom_raw",
    "/tron/sensor_joy",
    "/camera1/color/image_raw/compressed",
)
FRAME_TARGETS_S: Final = tuple(float(value) for value in range(0, 501, 25)) + (
    216.0,
    218.0,
    225.0,
    232.0,
    236.6,
    236.84,
    237.5,
    239.0,
)


class Gap(NamedTuple):
    topic: str
    start_s: float
    end_s: float

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


class WheelSample(NamedTuple):
    record_s: float
    header_s: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float


class LidarClock(NamedTuple):
    stamp_ns: int
    record_s: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return yaw from a normalized ROS quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def major_gaps(topic: str, stamps: Iterable[float]) -> tuple[Gap, ...]:
    """Return every open record-time interval exceeding the evidence threshold."""
    values = np.asarray(tuple(stamps), dtype=np.float64)
    indices = np.flatnonzero(np.diff(values) > GAP_THRESHOLD_S)
    return tuple(Gap(topic, float(values[index]), float(values[index + 1])) for index in indices)


def write_wheel(samples: list[WheelSample], output: Path) -> tuple[dict[str, float], ...]:
    """Write start-normalized wheel segments without connecting gaps or jumps."""
    values = np.asarray(samples, dtype=np.float64)
    distances = np.insert(np.linalg.norm(np.diff(values[:, 2:5], axis=0), axis=1), 0, 0.0)
    deltas = np.insert(np.diff(values[:, 0]), 0, 0.0)
    edge_valid = (deltas <= GAP_THRESHOLD_S) & (distances <= JUMP_THRESHOLD_M)
    edge_valid[0] = False
    segment_ids = np.cumsum(~edge_valid)
    normalized = np.zeros((len(values), 3), dtype=np.float64)
    for segment_id in np.unique(segment_ids):
        mask = segment_ids == segment_id
        normalized[mask] = values[mask, 2:5] - values[mask, 2:5][0]
    pl.DataFrame(
        {
            "record_time_s": values[:, 0],
            "header_time_s": values[:, 1],
            "measured_x_m": values[:, 2],
            "measured_y_m": values[:, 3],
            "measured_z_m": values[:, 4],
            "measured_yaw_rad": values[:, 5],
            "derived_normalized_x_m": normalized[:, 0],
            "derived_normalized_y_m": normalized[:, 1],
            "derived_normalized_z_m": normalized[:, 2],
            "derived_step_distance_m": distances,
            "derived_edge_valid_from_previous": edge_valid,
            "derived_path_segment_id": segment_ids,
        }
    ).write_csv(output)
    rejected = np.flatnonzero((distances > JUMP_THRESHOLD_M) | (deltas > GAP_THRESHOLD_S))
    return tuple(
        {
            "start_s": float(values[index - 1, 0]),
            "end_s": float(values[index, 0]),
            "duration_s": float(deltas[index]),
            "step_distance_m": float(distances[index]),
        }
        for index in rejected
    )


def write_lidar(lidar_csv: Path, clocks: list[LidarClock], output: Path) -> None:
    """Attach bag record time and hard edge masks to the existing relative path."""
    source = pl.read_csv(lidar_csv)
    clock = pl.DataFrame(clocks, schema=["stamp_ns", "record_time_s"], orient="row")
    joined = source.join(clock, on="stamp_ns", how="left").sort("record_time_s")
    record = joined["record_time_s"].to_numpy()
    accepted = joined["accepted"].to_numpy().astype(bool)
    edge_valid = np.insert(np.diff(record) <= GAP_THRESHOLD_S, 0, False) & accepted
    segment_start = (~edge_valid) | (~np.insert(accepted[:-1], 0, False))
    joined.with_columns(
        pl.Series("derived_edge_valid_from_previous", edge_valid),
        pl.Series("derived_route_evidence_usable", accepted & edge_valid),
        pl.Series("derived_path_segment_id", np.cumsum(segment_start)),
    ).write_csv(output)


def main(bag_path: Path, lidar_csv: Path, output_dir: Path) -> None:
    """Extract route evidence while retaining raw record/header timing provenance."""
    output_dir.mkdir(parents=True, exist_ok=True)
    topic_stamps: dict[str, list[float]] = {topic: [] for topic in TOPICS}
    wheel: list[WheelSample] = []
    lidar_clocks: list[LidarClock] = []
    intent_rows: list[tuple[float, float, str, str, str]] = []
    camera_candidates: dict[float, tuple[float, float, bytes]] = {}
    previous_intent: tuple[tuple[float, ...], tuple[int, ...]] | None = None
    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start_s = bag.get_start_time()
        for topic, message, record_stamp in bag.read_messages(topics=list(TOPICS)):
            record_s = record_stamp.to_sec() - bag_start_s
            topic_stamps[topic].append(record_s)
            header = getattr(message, "header", None)
            header_s = header.stamp.to_sec() if header is not None else math.nan
            # ROS Noetic's Python 3.8 cannot parse match/case.
            if topic == "/tron/wheel_odom_raw":
                pose = message.pose.pose
                wheel.append(
                    WheelSample(
                        record_s,
                        header_s,
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        yaw_from_quaternion(
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ),
                    )
                )
            elif topic == "/livox/lidar":
                lidar_clocks.append(LidarClock(message.header.stamp.to_nsec(), record_s))
            elif topic == "/tron/sensor_joy":
                signature = (tuple(message.axes), tuple(message.buttons))
                if signature != previous_intent:
                    intent_rows.append(
                        (record_s, header_s, json.dumps(signature[0]), json.dumps(signature[1]), "controller_intent_only")
                    )
                    previous_intent = signature
            elif topic == "/camera1/color/image_raw/compressed":
                for target in FRAME_TARGETS_S:
                    current = camera_candidates.get(target)
                    if current is None or abs(record_s - target) < abs(current[0] - target):
                        camera_candidates[target] = (record_s, header_s, bytes(message.data))
            elif topic != "/livox/imu":
                raise AssertionError(topic)

    gaps = tuple(gap for topic, stamps in topic_stamps.items() for gap in major_gaps(topic, stamps))
    pl.DataFrame(
        [(gap.topic, gap.start_s, gap.end_s, gap.duration_s, True) for gap in gaps],
        schema=["topic", "start_s", "end_s", "duration_s", "rejected_no_interpolation"],
        orient="row",
    ).write_csv(output_dir / "gap_intervals.csv")
    pl.DataFrame(
        intent_rows,
        schema=["record_time_s", "header_time_s", "axes_json", "buttons_json", "provenance"],
        orient="row",
    ).write_csv(output_dir / "controller_intent_samples.csv")
    wheel_rejections = write_wheel(wheel, output_dir / "normalized_wheel_path.csv")
    write_lidar(lidar_csv, lidar_clocks[::2], output_dir / "lidar_relative_path.csv")
    frame_rows: list[tuple[float, float, float, str]] = []
    for target, (record_s, header_s, payload) in sorted(camera_candidates.items()):
        name = f"rgb_nearest_{target:07.2f}s.jpg"
        (output_dir / name).write_bytes(payload)
        frame_rows.append((target, record_s, header_s, name))
    pl.DataFrame(
        frame_rows,
        schema=["requested_record_time_s", "actual_record_time_s", "header_time_s", "file"],
        orient="row",
    ).write_csv(output_dir / "rgb_event_frames.csv")
    summary = {
        "bag_start_epoch_s": bag_start_s,
        "gap_threshold_s": GAP_THRESHOLD_S,
        "wheel_jump_threshold_m": JUMP_THRESHOLD_M,
        "topic_counts": {topic: len(stamps) for topic, stamps in topic_stamps.items()},
        "gap_count": len(gaps),
        "wheel_rejected_edges": wheel_rejections,
        "interpolation_performed": False,
        "zero_fill_performed": False,
    }
    (output_dir / "extraction_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    typer.echo(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    typer.run(main)
