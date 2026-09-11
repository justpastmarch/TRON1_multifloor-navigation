#!/usr/bin/env python3
"""Run an operator-gated normalized yaw-rate calibration over TRON1 WebSocket."""

from __future__ import annotations

import argparse
from pathlib import Path
import time
from typing import Final, NamedTuple, TextIO

from stair_supervisor.configuration import WebSocketCalibration
from stair_supervisor.robot_config import (
    CommandStreamConfig,
    RobotConnectionConfig,
    RobotTransportConfig,
)
from stair_supervisor.robot_transport import RobotTransport


RATE_HZ: Final = 40.0
ZERO_SEC: Final = 2.0


class CalibrationOptions(NamedTuple):
    accid: str
    url: str
    active_sec: float
    levels: tuple[float, ...]


def _positive(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("expected a positive number")
    return parsed


def _normalized_level(value: str) -> float:
    parsed = float(value)
    if not 0.0 < parsed <= 1.0:
        raise argparse.ArgumentTypeError("expected a normalized level within (0, 1]")
    return parsed


def _options() -> CalibrationOptions:
    parser = argparse.ArgumentParser(
        description="Measure TRON1 yaw response with operator-gated normalized steps.",
    )
    parser.add_argument("--accid", default="WF_TRON1A_632")
    parser.add_argument("--url", default="ws://127.0.0.1:15000")
    parser.add_argument("--active-sec", type=_positive, default=4.0)
    parser.add_argument(
        "--levels",
        nargs="+",
        type=_normalized_level,
        default=(0.05, 0.10, 0.15),
    )
    arguments = parser.parse_args()
    return CalibrationOptions(
        arguments.accid,
        arguments.url,
        arguments.active_sec,
        tuple(arguments.levels),
    )


def _transport(options: CalibrationOptions) -> RobotTransport:
    return RobotTransport(
        RobotTransportConfig(
            connection=RobotConnectionConfig(
                accid=options.accid,
                url=options.url,
                connect_timeout=3.0,
                receive_timeout=0.1,
                request_timeout=8.0,
            ),
            stream=CommandStreamConfig(
                rate_hz=RATE_HZ,
                watchdog_sec=0.25,
                startup_zero_repeats=3,
                close_zero_repeats=8,
            ),
        ),
        WebSocketCalibration(linear_mps=1.0, angular_radps=1.0),
    )


def _stream(transport: RobotTransport, z: float, duration: float) -> None:
    started = time.monotonic()
    print(
        f"[PHASE] epoch={time.time():.6f} z={z:+.3f} duration={duration:.1f}s",
        flush=True,
    )
    while time.monotonic() - started < duration:
        transport.update_twist(0.0, z)
        transport.send_current()
        time.sleep(1.0 / RATE_HZ)


def _operator_ready(terminal: TextIO, level: float) -> bool:
    print(
        f"\n로봇 주변이 안전하면 Enter: +{level:.2f}, -{level:.2f} 회전 시험 "
        "(중단은 Ctrl+C 또는 물리 비상정지) ",
        end="",
        flush=True,
    )
    return terminal.readline() != ""


def main() -> int:
    options = _options()
    transport = _transport(options)
    interrupted = False
    try:
        with transport:
            transport.set_odometry_enabled(True)
            transport.set_imu_enabled(True)
            _stream(transport, 0.0, ZERO_SEC)
            try:
                with Path("/dev/tty").open("r", encoding="utf-8") as terminal:
                    for level in options.levels:
                        if not _operator_ready(terminal, level):
                            break
                        _stream(transport, level, options.active_sec)
                        _stream(transport, 0.0, ZERO_SEC)
                        _stream(transport, -level, options.active_sec)
                        _stream(transport, 0.0, ZERO_SEC)
            except KeyboardInterrupt:
                interrupted = True
                print("\n[STOP] operator interrupted calibration", flush=True)
            finally:
                _stream(transport, 0.0, 1.0)
    finally:
        print("[SAFE] zero sent and WebSocket closed", flush=True)
    return 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
