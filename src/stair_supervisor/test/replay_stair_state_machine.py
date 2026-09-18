#!/usr/bin/env python3
# --- How to run ---
# Use the repository wrapper: ./replay_stair_state_machine.sh BAG [PROFILE [RATE]]
"""CLI boundary for recorded odometry replay through the live stair action."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Sequence

import rospy

from stair_supervisor.ros_bag_replay import ReplayInputError, ReplaySession, load_profile
from stair_supervisor.state_machine_dashboard import ReplayOutcome


def main(arguments: Sequence[str] = sys.argv[1:]) -> int:
    parser = argparse.ArgumentParser(
        description="Replay bag odometry through the live stair evidence state machine."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--profile", default="stair_3f_4f_up")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--config-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--hold", action="store_true")
    options = parser.parse_args(arguments)
    bag_path = options.bag.expanduser().resolve()
    if not bag_path.is_file():
        raise ReplayInputError(f"bag is not readable: {bag_path}")
    if options.rate <= 0.0 or options.rate > 4.0:
        raise ReplayInputError("rate must be greater than zero and no more than four")
    if options.start < 0.0:
        raise ReplayInputError("start must not be negative")
    rospy.init_node("stair_bag_state_replay")
    session = ReplaySession(
        bag_path,
        load_profile(options.config_dir.resolve(), options.profile),
        options.output_dir.resolve(),
    )
    outcome = session.run(options.rate, options.start)
    if options.hold:
        rospy.spin()
    return 0 if outcome is ReplayOutcome.SUCCEEDED else 1


if __name__ == "__main__":
    raise SystemExit(main())
