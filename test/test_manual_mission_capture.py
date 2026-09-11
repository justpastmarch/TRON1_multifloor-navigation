from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys


SCRIPT = Path(__file__).resolve().parents[1] / "manual_mission_capture.py"
SPEC = importlib.util.spec_from_file_location("manual_mission_capture", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
capture = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)


def environment(output_root: Path) -> dict[str, str]:
    return {
        "MANUAL_CAPTURE_OUTPUT_BASE": str(output_root),
        "CAPTURE_CAMERA_TOPIC": "/camera/color/image_raw/compressed",
        "CAMERA_INFO_TOPIC": "/camera/color/camera_info",
        "RAW_LIVOX_TOPIC": "/livox/lidar_raw",
        "SCAN_TOPIC": "/scan",
        "WHEEL_ODOM_TOPIC": "/tron/wheel_odom_raw",
        "IMU_TOPIC": "/livox/imu",
    }


def test_plan_contains_sensor_static_tf_and_physical_joystick(tmp_path: Path) -> None:
    # Given: configured deployment topics and a fresh capture timestamp.
    variables = environment(tmp_path)

    # When: the manual capture plan is parsed.
    plan = capture.build_plan(variables, timestamp_ns=1700000000000000000)

    # Then: explicit required topics include every mission sensor and raw joystick input.
    assert plan.required_topics == (
        "/camera/color/image_raw/compressed",
        "/camera/color/camera_info",
        "/livox/lidar_raw",
        "/scan",
        "/tron/wheel_odom_raw",
        "/tf",
        "/tf_static",
        "/livox/imu",
        "/tron/sensor_joy",
    )
    assert plan.optional_topics == ()
    assert plan.artifact.parent == tmp_path
    assert plan.artifact.name.endswith("1700000000000000000.bag")


def test_prerequisites_distinguish_sensor_and_joystick_failures(tmp_path: Path) -> None:
    # Given: healthy sensor topics but no physical joystick callback topic.
    plan = capture.build_plan(environment(tmp_path), timestamp_ns=1700000000000000000)
    available = {topic: expected for topic, expected in plan.expected_types if topic != "/tron/sensor_joy"}

    # When: readiness is evaluated without opening ROS or starting any process.
    readiness = capture.evaluate_readiness(plan, available)

    # Then: sensor readiness and physical joystick availability are separate.
    assert readiness.sensors_ready
    assert not readiness.joystick_ready
    assert readiness.missing == ("/tron/sensor_joy",)


def test_report_marks_required_zero_count_incomplete(tmp_path: Path) -> None:
    # Given: a finalized synthetic count map missing only physical joystick input.
    plan = capture.build_plan(environment(tmp_path), timestamp_ns=1700000000000000000)
    counts = {topic: 1 for topic in plan.required_topics}
    counts["/tron/sensor_joy"] = 0

    # When: the stop report is derived from bag metadata.
    report = capture.capture_report(plan, counts)

    # Then: the capture is explicitly incomplete and names the absent requirement.
    assert not report.complete
    assert report.missing_required == ("/tron/sensor_joy",)
    assert report.topic_counts["/tron/sensor_joy"] == 0


def test_message_probe_does_not_deserialize_the_topic_schema(monkeypatch) -> None:
    # Given: a CustomMsg topic whose package is not installed on this workstation.
    commands: list[tuple[str, ...]] = []

    def run(command: tuple[str, ...], **_kwargs) -> subprocess.CompletedProcess[str]:
        commands.append(command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(capture.subprocess, "run", run)

    # When: freshness is checked.
    ready = capture._await_message("/livox/raw")

    # Then: the isolated AnyMsg probe is used instead of `rostopic echo` deserialization.
    assert ready
    assert "--probe-topic" in commands[0]
    assert "rostopic" not in commands[0]


def test_recorder_error_is_incomplete_even_when_all_topic_counts_exist(tmp_path: Path) -> None:
    # Given: rosbag finalized every stream but recorder validation still failed.
    plan = capture.build_plan(environment(tmp_path), timestamp_ns=1700000000000000000)
    invalid = Path(str(plan.artifact) + ".invalid")
    invalid.write_bytes(b"retained")
    counts = {topic: 1 for topic in (*plan.required_topics, *plan.optional_topics)}
    error = capture.ScanRecorderError("forced failure", plan.artifact)

    # When: the terminal error report is built.
    report = capture.capture_error_report(plan, error, counts)

    # Then: error state wins over apparently complete metadata and names retained evidence.
    assert not report.complete
    assert report.artifact == str(invalid)
    assert report.missing_required == ()


def test_start_failure_writes_incomplete_report_without_unbound_session(
    tmp_path: Path,
    monkeypatch,
) -> None:
    # Given: prerequisites pass but rosbag cannot start and retains no artifact.
    plan = capture.build_plan(environment(tmp_path), timestamp_ns=1700000000000000000)

    class FailingRecorder:
        def __init__(self, _output: Path) -> None:
            return None

        def start(self, _profile, _identity, extra_topics=()):
            raise capture.ScanRecorderError("unable to start rosbag", plan.artifact)

    monkeypatch.setattr(capture.shutil, "which", lambda _command: "/available")
    monkeypatch.setattr(capture, "_topic_types", lambda current: dict(current.expected_types))
    monkeypatch.setattr(capture, "_await_message", lambda _topic: True)
    monkeypatch.setattr(capture, "ScanRecorder", FailingRecorder)

    # When: the public recording lifecycle reaches recorder startup.
    result = capture.record(plan)

    # Then: startup failure is reported as incomplete rather than escaping or succeeding.
    report_path = Path(str(plan.artifact) + ".INCOMPLETE.json")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert result == 1
    assert report["complete"] is False
    assert report["artifact"] == ""
