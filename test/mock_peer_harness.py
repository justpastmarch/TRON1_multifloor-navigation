"""Executable process harness for test-only action, service, and WebSocket peers."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import NamedTuple

from fixture_support import load_frozen_clock, load_websocket_transcript
from mock_peers import FakeActionPeer, FakeClock, FakeServicePeer, FakeWebSocketPeer


class HarnessReport(NamedTuple):
    action_state: str
    service_response: str
    websocket_frames: int
    sent_payloads: int
    clock_now_ns: int


def run_harness() -> HarnessReport:
    clock = FakeClock(load_frozen_clock().now_ns)
    action = FakeActionPeer(("SUCCEEDED",))
    service = FakeServicePeer({"change_map": "accepted"})
    frames = load_websocket_transcript()
    websocket = FakeWebSocketPeer(frames)
    for _ in frames:
        websocket.receive()
    websocket.send('{"ack":true}')
    return HarnessReport(
        action.send_goal("roof_landing"),
        service.call("change_map", "4F"),
        len(frames),
        len(websocket.sent_payloads),
        clock.now_ns(),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Run deterministic non-node mock peers")
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    report_json = json.dumps(run_harness()._asdict(), sort_keys=True)
    if arguments.output:
        arguments.output.write_text(report_json + "\n", encoding="utf-8")
    print(report_json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
