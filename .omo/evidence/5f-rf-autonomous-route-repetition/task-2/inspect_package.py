#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.8,<3.9"
# dependencies = [
#   "numpy==1.24.4",
#   "polars==1.8.2",
#   "pyarrow==17.0.0",
#   "typer==0.16.0",
# ]
# ///

# ─── How to run ───
# 1. Install uv: curl -LsSf https://astral.sh/uv/install.sh | sh
# 2. Run: uv run --python /usr/bin/python3 inspect_package.py EVIDENCE_DIR BAG EXPECTED_SHA256
# ──────────────────
"""Fail-closed inspection for the Task 2 no-motion evidence package."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Final

import polars as pl
import typer

REQUIRED_TOPICS: Final = {
    "/livox/lidar",
    "/livox/imu",
    "/tron/wheel_odom_raw",
    "/tron/sensor_joy",
    "/camera1/color/image_raw/compressed",
}
GENERATED_FILES: Final = (
    "controller_intent_samples.csv",
    "extraction_summary.json",
    "gap_intervals.csv",
    "lidar_relative_path.csv",
    "normalized_wheel_path.csv",
    "rgb_event_frames.csv",
)


def sha256(path: Path) -> str:
    """Hash a file without loading it into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def intervals(frame: pl.DataFrame, source: str, usable_column: str) -> pl.DataFrame:
    """Summarize exact contiguous path intervals already assigned by extraction."""
    return (
        frame.filter(pl.col(usable_column))
        .group_by("derived_path_segment_id", maintain_order=True)
        .agg(
            pl.col("record_time_s").min().alias("start_s"),
            pl.col("record_time_s").max().alias("end_s"),
            pl.len().alias("sample_count"),
        )
        .with_columns(pl.lit(source).alias("source"))
        .select("source", "derived_path_segment_id", "start_s", "end_s", "sample_count")
    )


def main(evidence_dir: Path, bag_path: Path, expected_sha256: str) -> None:
    """Verify provenance and rejection invariants, then emit a machine-readable receipt."""
    actual_sha256 = sha256(bag_path)
    if actual_sha256 != expected_sha256:
        raise typer.BadParameter("source bag checksum mismatch")
    source_mtime_ns = bag_path.stat().st_mtime_ns
    for name in GENERATED_FILES:
        artifact = evidence_dir / name
        if not artifact.is_file() or artifact.stat().st_size == 0:
            raise typer.BadParameter(f"generated artifact missing or empty: {name}")
        if artifact.stat().st_mtime_ns < source_mtime_ns:
            raise typer.BadParameter(f"generated artifact is stale: {name}")
    summary = json.loads((evidence_dir / "extraction_summary.json").read_text(encoding="utf-8"))
    if summary["interpolation_performed"] or summary["zero_fill_performed"]:
        raise typer.BadParameter("evidence claims interpolation or zero-fill")
    if set(summary["topic_counts"]) != REQUIRED_TOPICS:
        raise typer.BadParameter("required topic inventory mismatch")

    gaps = pl.read_csv(evidence_dir / "gap_intervals.csv")
    wheel = pl.read_csv(evidence_dir / "normalized_wheel_path.csv")
    lidar = pl.read_csv(evidence_dir / "lidar_relative_path.csv")
    intent = pl.read_csv(evidence_dir / "controller_intent_samples.csv")
    if not gaps["rejected_no_interpolation"].all():
        raise typer.BadParameter("a detected gap was not rejected")
    if intent["provenance"].unique().to_list() != ["controller_intent_only"]:
        raise typer.BadParameter("Joy evidence is mislabeled as command evidence")

    invalid_wheel_edges = wheel.filter(
        (pl.col("derived_step_distance_m") > summary["wheel_jump_threshold_m"])
        | (
            pl.col("record_time_s").diff().fill_null(0.0)
            > summary["gap_threshold_s"]
        )
    )
    if invalid_wheel_edges["derived_edge_valid_from_previous"].any():
        raise typer.BadParameter("wheel path bridges a rejected edge")
    jump = invalid_wheel_edges.filter(
        pl.col("derived_step_distance_m") > summary["wheel_jump_threshold_m"]
    )
    if jump.height != 1:
        raise typer.BadParameter("expected exactly one wheel discontinuity")
    if lidar.filter(
        pl.col("derived_route_evidence_usable") & (pl.col("accepted") != 1)
    ).height:
        raise typer.BadParameter("rejected registration is marked usable")
    if lidar.filter(
        pl.col("derived_route_evidence_usable")
        & ~pl.col("derived_edge_valid_from_previous")
    ).height:
        raise typer.BadParameter("LiDAR path bridges a rejected edge")

    main_gaps = (
        gaps.filter((pl.col("start_s") >= 200.0) & (pl.col("start_s") <= 240.0))
        .sort("duration_s", descending=True)
        .group_by("topic", maintain_order=True)
        .first()
        .sort("topic")
    )
    if set(main_gaps["topic"].to_list()) != REQUIRED_TOPICS:
        raise typer.BadParameter("main outage lacks a required topic")
    common_start_s = float(main_gaps["start_s"].max())
    common_end_s = float(main_gaps["end_s"].min())
    if common_start_s >= common_end_s:
        raise typer.BadParameter("main outages have no common rejected overlap")

    valid = pl.concat(
        (
            intervals(wheel, "wheel", "derived_edge_valid_from_previous"),
            intervals(lidar, "lidar_only", "derived_route_evidence_usable"),
        )
    )
    valid.write_csv(evidence_dir / "path_valid_intervals.csv")
    jump_row = jump.row(0, named=True)
    result = {
        "status": "pass",
        "source_bag_sha256": actual_sha256,
        "common_rejected_overlap_s": [common_start_s, common_end_s],
        "main_topic_rejected_intervals": main_gaps.select(
            "topic", "start_s", "end_s", "duration_s"
        ).to_dicts(),
        "wheel_discontinuity": {
            "previous_record_time_s": float(wheel.filter(pl.col("record_time_s") < jump_row["record_time_s"])["record_time_s"][-1]),
            "record_time_s": float(jump_row["record_time_s"]),
            "step_distance_m": float(jump_row["derived_step_distance_m"]),
            "edge_rejected": True,
        },
        "wheel_valid_interval_count": valid.filter(pl.col("source") == "wheel").height,
        "lidar_valid_interval_count": valid.filter(pl.col("source") == "lidar_only").height,
        "controller_intent_label": "controller_intent_only",
        "motion_interfaces_invoked": False,
    }
    (evidence_dir / "inspection_result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    typer.echo(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    typer.run(main)
