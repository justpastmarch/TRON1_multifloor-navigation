#!/usr/bin/env python3
"""Record one operator-driven 5F/rooftop mission without launching robot software."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Final, Mapping, NamedTuple, Sequence

import yaml


ROOT: Final = Path(__file__).resolve().parent
SOURCE: Final = ROOT / "src" / "mission_manager" / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))

from mission_manager.configuration import ScanProfile  # noqa: E402
from mission_manager.recording_session import RecordingDisposition, ScanRecorderError  # noqa: E402
from mission_manager.scan_recorder import RecordingIdentity, ScanRecorder  # noqa: E402


DEFAULT_JOYSTICK_TOPIC: Final = "/tron/sensor_joy"


class CapturePlan(NamedTuple):
    artifact: Path
    required_topics: tuple[str, ...]
    optional_topics: tuple[str, ...]
    expected_types: tuple[tuple[str, str], ...]


class CaptureReadiness(NamedTuple):
    sensors_ready: bool
    joystick_ready: bool
    missing: tuple[str, ...]
    mismatched: tuple[str, ...]


class CaptureReport(NamedTuple):
    complete: bool
    artifact: str
    missing_required: tuple[str, ...]
    topic_counts: dict[str, int]
    note: str


def build_plan(environment: Mapping[str, str], timestamp_ns: int) -> CapturePlan:
    """Parse configurable deployment topics into one explicit rosbag plan."""
    raw_camera = environment.get("CAMERA_IMAGE_TOPIC", "/camera1/color/image_raw")
    camera = environment.get("CAPTURE_CAMERA_TOPIC", raw_camera + "/compressed")
    sensors = (
        (camera, "sensor_msgs/CompressedImage"),
        (environment.get("CAMERA_INFO_TOPIC", "/camera1/color/camera_info"), "sensor_msgs/CameraInfo"),
        (environment.get("RAW_LIVOX_TOPIC", "/livox/lidar"), "livox_ros_driver2/CustomMsg"),
        (environment.get("SCAN_TOPIC", "/scan"), "sensor_msgs/LaserScan"),
        (environment.get("WHEEL_ODOM_TOPIC", "/tron/wheel_odom_raw"), "nav_msgs/Odometry"),
        ("/tf", "tf2_msgs/TFMessage"),
        ("/tf_static", "tf2_msgs/TFMessage"),
        (environment.get("IMU_TOPIC", "/livox/imu"), "sensor_msgs/Imu"),
    )
    joystick = (environment.get("SENSOR_JOY_TOPIC", DEFAULT_JOYSTICK_TOPIC), "sensor_msgs/Joy")
    output = Path(environment.get("MANUAL_CAPTURE_OUTPUT_BASE", str(ROOT / "manual_captures"))).expanduser().resolve()
    artifact = output / f"manual-5F-rooftop_route_{timestamp_ns}.bag"
    return CapturePlan(
        artifact,
        tuple(topic for topic, _message_type in (*sensors, joystick)),
        (),
        (*sensors, joystick),
    )


def evaluate_readiness(plan: CapturePlan, available: Mapping[str, str]) -> CaptureReadiness:
    """Keep sensor readiness distinct from physical joystick callbacks."""
    expected = dict(plan.expected_types)
    sensor_topics = plan.required_topics[:8]
    missing_sensors = tuple(topic for topic in sensor_topics if topic not in available)
    mismatched = tuple(
        topic for topic in (*sensor_topics, plan.required_topics[8])
        if topic in available and available[topic] != expected[topic]
    )
    joystick_topic = plan.required_topics[8]
    joystick_ready = available.get(joystick_topic) == expected[joystick_topic]
    missing = (*missing_sensors, *((joystick_topic,) if not joystick_ready else ()))
    return CaptureReadiness(not missing_sensors and not any(topic in sensor_topics for topic in mismatched), joystick_ready, missing, mismatched)


def capture_report(plan: CapturePlan, counts: Mapping[str, int], note: str = "") -> CaptureReport:
    """Classify a finalized bag by required per-topic message counts."""
    topic_counts = {topic: int(counts.get(topic, 0)) for topic in (*plan.required_topics, *plan.optional_topics)}
    missing = tuple(topic for topic in plan.required_topics if topic_counts[topic] <= 0)
    return CaptureReport(not missing, str(plan.artifact), missing, topic_counts, note)


def capture_error_report(
    plan: CapturePlan,
    error: ScanRecorderError,
    counts: Mapping[str, int],
) -> CaptureReport:
    """Preserve available evidence while making recorder failure terminal."""
    topic_counts = {
        topic: int(counts.get(topic, 0))
        for topic in (*plan.required_topics, *plan.optional_topics)
    }
    missing = tuple(topic for topic in plan.required_topics if topic_counts[topic] <= 0)
    candidates = (
        Path(str(plan.artifact) + ".invalid"),
        Path(str(plan.artifact) + ".active.invalid"),
        plan.artifact,
    )
    retained = next((path for path in candidates if path.exists()), None)
    return CaptureReport(
        False,
        str(retained) if retained is not None else "",
        missing,
        topic_counts,
        str(error),
    )


def _topic_types(plan: CapturePlan) -> dict[str, str]:
    available: dict[str, str] = {}
    for topic, _expected in plan.expected_types:
        try:
            result = subprocess.run(("rostopic", "type", topic), capture_output=True, text=True, check=False, timeout=3.0)
        except subprocess.TimeoutExpired:
            continue
        if result.returncode == 0 and result.stdout.strip():
            available[topic] = result.stdout.strip()
    return available


def _await_message(topic: str) -> bool:
    try:
        result = subprocess.run(
            (
                sys.executable,
                str(Path(__file__).resolve()),
                "--probe-topic",
                topic,
                "--probe-timeout",
                "8",
            ),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=10.0,
        )
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0


def _probe_topic(topic: str, timeout_sec: float) -> int:
    """Receive one serialized ROS message without importing its schema package."""
    import rospy

    received = threading.Event()
    getattr(rospy, "init_node")("manual_capture_topic_probe", anonymous=True, disable_signals=True)
    subscriber = rospy.Subscriber(
        topic,
        rospy.AnyMsg,
        lambda _message: received.set(),
        queue_size=1,
    )
    try:
        return 0 if received.wait(timeout_sec) else 1
    finally:
        subscriber.unregister()


def _bag_counts(path: Path) -> dict[str, int]:
    result = subprocess.run(("rosbag", "info", "--yaml", str(path)), capture_output=True, text=True, check=False, timeout=10.0)
    if result.returncode != 0:
        return {}
    try:
        document = yaml.safe_load(result.stdout)
    except yaml.YAMLError:
        return {}
    topics = document.get("topics", []) if isinstance(document, dict) else []
    return {
        item["topic"]: item["messages"]
        for item in topics
        if isinstance(item, dict) and isinstance(item.get("topic"), str) and isinstance(item.get("messages"), int)
    }


def _write_report(plan: CapturePlan, report: CaptureReport) -> Path:
    suffix = ".capture.json" if report.complete else ".INCOMPLETE.json"
    path = Path(str(plan.artifact) + suffix)
    path.write_text(json.dumps(report._asdict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def record(plan: CapturePlan) -> int:
    """Preflight live inputs, then own rosbag until Ctrl+C and validate on stop."""
    for command in ("rosbag", "rostopic", "timeout"):
        if shutil.which(command) is None:
            print(f"[FAIL] required command missing: {command}", file=sys.stderr)
            return 2
    readiness = evaluate_readiness(plan, _topic_types(plan))
    print(f"[READY SENSOR] {'yes' if readiness.sensors_ready else 'no'}", flush=True)
    print(
        f"[READY JOYSTICK] {'yes' if readiness.joystick_ready else 'no'} "
        "(actual SensorJoy callbacks)",
        flush=True,
    )
    if readiness.missing or readiness.mismatched:
        print(f"[FAIL] missing={readiness.missing} type_mismatch={readiness.mismatched}", file=sys.stderr)
        return 2
    stale = tuple(topic for topic in plan.required_topics if not _await_message(topic))
    if stale:
        print(f"[FAIL] no message received from required prerequisites: {stale}", file=sys.stderr)
        return 2
    plan.artifact.parent.mkdir(parents=True, exist_ok=True)
    profile = ScanProfile("manual-rooftop", plan.required_topics, 86400.0, Path("."), 0.1)
    identity = RecordingIdentity("manual-5F-rooftop", "route", int(plan.artifact.stem.rsplit("_", 1)[1]))
    recorder = ScanRecorder(plan.artifact.parent)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop.set())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: stop.set())
    session = None
    try:
        session = recorder.start(profile, identity, extra_topics=plan.optional_topics)
        print(f"[RECORDING] {plan.artifact}", flush=True)
        print("[SEMANTICS] joystick values are raw; Joy header stamp is the SDK source nanoseconds", flush=True)
        print("[STOP] Ctrl+C", flush=True)
        while not stop.wait(0.5):
            session.assert_active()
        result = session.finish(RecordingDisposition.COMPLETED)
        counts = _bag_counts(result.artifact_path)
        report = capture_report(plan, counts)
    except ScanRecorderError as error:
        if session is not None:
            session.discard()
        retained = next(
            (
                path
                for path in (
                    Path(str(plan.artifact) + ".invalid"),
                    Path(str(plan.artifact) + ".active.invalid"),
                    plan.artifact,
                )
                if path.exists()
            ),
            None,
        )
        counts = _bag_counts(retained) if retained is not None else {}
        report = capture_error_report(plan, error, counts)
    report_path = _write_report(plan, report)
    print(f"[{'COMPLETE' if report.complete else 'INCOMPLETE'}] bag={plan.artifact} report={report_path}")
    for topic, count in report.topic_counts.items():
        print(f"[COUNT] {topic}={count}")
    return 0 if report.complete else 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="TRON1 manual 5F-rooftop mission recorder")
    parser.add_argument("--probe-topic", default="")
    parser.add_argument("--probe-timeout", type=float, default=8.0)
    arguments = parser.parse_args(argv)
    if arguments.probe_topic:
        return _probe_topic(arguments.probe_topic, arguments.probe_timeout)
    return record(build_plan(os.environ, time.time_ns()))


if __name__ == "__main__":
    raise SystemExit(main())
