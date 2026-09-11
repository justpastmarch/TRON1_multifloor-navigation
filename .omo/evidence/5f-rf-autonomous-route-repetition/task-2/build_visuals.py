#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.8,<3.9"
# dependencies = [
#   "matplotlib==3.7.5",
#   "numpy==1.24.4",
#   "pillow==10.4.0",
#   "polars==1.8.2",
#   "pyarrow==17.0.0",
#   "typer==0.16.0",
# ]
# ///

# ─── How to run ───
# 1. Install uv: curl -LsSf https://astral.sh/uv/install.sh | sh
# 2. Run: uv run build_visuals.py EVIDENCE_DIR
# ──────────────────
"""Render route and RGB inspection artifacts without connecting rejected edges."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import polars as pl
import typer
from PIL import Image, ImageDraw


def route_plot(evidence_dir: Path) -> None:
    """Plot wheel and LiDAR segments independently so outages remain visible."""
    wheel = pl.read_csv(evidence_dir / "normalized_wheel_path.csv")
    lidar = pl.read_csv(evidence_dir / "lidar_relative_path.csv").filter(
        pl.col("derived_route_evidence_usable")
    )
    figure, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True)
    for segment in wheel.partition_by("derived_path_segment_id", maintain_order=True):
        axes[0].plot(
            segment["derived_normalized_x_m"],
            segment["derived_normalized_y_m"],
            linewidth=0.8,
        )
    for segment in lidar.partition_by("derived_path_segment_id", maintain_order=True):
        if segment.height > 1:
            axes[1].plot(segment["x_m"], segment["y_m"], linewidth=0.8)
    axes[0].set_title("Wheel path: independently normalized valid-edge segments")
    axes[1].set_title("LiDAR-only relative path: accepted valid-edge segments")
    for axis in axes:
        axis.axis("equal")
        axis.grid(True, alpha=0.3)
        axis.set_xlabel("relative x (m)")
        axis.set_ylabel("relative y (m)")
    figure.suptitle("Diagnostic shape only; gaps and 236.838 s discontinuity are not bridged")
    figure.savefig(evidence_dir / "route_paths_with_rejections.png", dpi=160)
    plt.close(figure)


def contact_sheet(evidence_dir: Path) -> None:
    """Create a timestamped visual index of all extracted nearest RGB frames."""
    index = pl.read_csv(evidence_dir / "rgb_event_frames.csv")
    width, height = 320, 210
    columns = 4
    rows = int(np.ceil(index.height / columns))
    sheet = Image.new("RGB", (columns * width, rows * height), "white")
    for position, row in enumerate(index.iter_rows(named=True)):
        image = Image.open(evidence_dir / row["file"]).convert("RGB")
        image.thumbnail((width, height - 28))
        tile = Image.new("RGB", (width, height), "white")
        tile.paste(image, ((width - image.width) // 2, 24))
        ImageDraw.Draw(tile).text(
            (5, 5),
            f"request {row['requested_record_time_s']:.2f}s / actual {row['actual_record_time_s']:.3f}s",
            fill="black",
        )
        sheet.paste(tile, ((position % columns) * width, (position // columns) * height))
    sheet.save(evidence_dir / "rgb_route_contact_sheet.jpg", quality=90)


def main(evidence_dir: Path) -> None:
    """Build the no-motion visual inspection artifacts."""
    route_plot(evidence_dir)
    contact_sheet(evidence_dir)
    typer.echo("wrote route_paths_with_rejections.png and rgb_route_contact_sheet.jpg")


if __name__ == "__main__":
    typer.run(main)
