from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from mission_manager.configuration import ScanProfile  # noqa: E402
from mission_manager.scan_recorder import (  # noqa: E402
    CommandResult,
    DiskCapacity,
    MissionScanFailure,
    RecorderRuntime,
    RecordingDisposition,
    RecordingIdentity,
    ScanRecorder,
    ScanRecorderError,
    map_scan_failure,
)


TOPICS = ("/scan", "/tf", "/camera/color/image_raw")


class FakeProcess:
    def __init__(self, artifact: Path, exit_early: bool = False, hang_on_sigint: bool = False, wait_exception=None, active_lifecycle: bool = True, active_after_waits: int = 0) -> None:
        self.artifact = artifact
        self.active_artifact = Path(str(artifact) + ".active")
        self.exit_early = exit_early
        self.hang_on_sigint = hang_on_sigint
        self.wait_exception = wait_exception
        self.active_lifecycle = active_lifecycle
        self.active_after_waits = active_after_waits
        self.signals = []
        self.killed = False
        self.recording = True
        self.reaped = False
        self.wait_count = 0
        self.wait_timeouts = []

    def wait(self, timeout: float) -> int:
        self.wait_count += 1
        self.wait_timeouts.append(timeout)
        if self.active_lifecycle and self.active_after_waits == self.wait_count:
            self.active_artifact.write_bytes(b"controlled rosbag artifact")
        if self.wait_exception is not None:
            error = self.wait_exception
            self.wait_exception = None
            raise error
        if self.exit_early:
            self.reaped = True
            return 3
        if self.recording or self.hang_on_sigint:
            raise subprocess.TimeoutExpired("rosbag", timeout)
        if self.active_lifecycle and not self.killed and self.active_artifact.exists():
            self.active_artifact.replace(self.artifact)
        self.reaped = True
        return 0

    def send_signal(self, value: int) -> None:
        self.signals.append(value)
        self.recording = False

    def kill(self) -> None:
        self.killed = True
        self.recording = False
        self.hang_on_sigint = False


class FakeExecutor:
    def __init__(self, info_yaml: str, exit_early: bool = False, hang_on_sigint: bool = False, info_code: int = 0, wait_exception=None, active_lifecycle: bool = True, active_after_waits: int = 0) -> None:
        self.info_yaml = info_yaml
        self.exit_early = exit_early
        self.hang_on_sigint = hang_on_sigint
        self.info_code = info_code
        self.wait_exception = wait_exception
        self.active_lifecycle = active_lifecycle
        self.active_after_waits = active_after_waits
        self.commands = []
        self.process = None

    def start(self, argv):
        self.commands.append(tuple(argv))
        artifact = Path(argv[argv.index("-O") + 1])
        if self.active_after_waits == 0:
            generated = Path(str(artifact) + ".active") if self.active_lifecycle else artifact
            generated.write_bytes(b"controlled rosbag artifact")
        self.process = FakeProcess(artifact, self.exit_early, self.hang_on_sigint, self.wait_exception, self.active_lifecycle, self.active_after_waits)
        return self.process

    def run(self, argv, timeout: float) -> CommandResult:
        self.commands.append(tuple(argv))
        return CommandResult(self.info_code, self.info_yaml, "info failure" if self.info_code else "")


def valid_info(topics=TOPICS, duration=12.5, messages=2) -> str:
    lines = [f"duration: {duration}", "topics:"]
    for topic in topics:
        lines.extend((f"  - topic: {topic}", f"    messages: {messages}"))
    return "\n".join(lines)


class ScanRecorderTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_dir = tempfile.TemporaryDirectory()
        self.output_base = Path(self.temporary_dir.name)
        self.profile = ScanProfile("roof", TOPICS, 0.2, Path("artifacts/scans"), 0.35)
        self.identity = RecordingIdentity("mission-42", "roof", 1700000000000000000)
        self.artifact = self.output_base / self.profile.output_root / "mission-42_roof_1700000000000000000.bag"

    def tearDown(self) -> None:
        self.temporary_dir.cleanup()

    def recorder(self, executor: FakeExecutor, free_fraction: float = 0.8, output_base=None) -> ScanRecorder:
        runtime = RecorderRuntime(executor, lambda _path: DiskCapacity(1000, int(1000 * free_fraction)), 0.1)
        return ScanRecorder(output_base or self.output_base, runtime)

    def test_valid_recording_uses_explicit_topics_and_validates_artifact(self) -> None:
        # Given: a valid profile, writable destination, and controlled rosbag process.
        executor = FakeExecutor(valid_info())
        # When: recording reaches its configured duration.
        result = self.recorder(executor).record(self.profile, self.identity)
        # Then: rosbag receives only explicit arguments and the validated artifact is returned.
        self.assertEqual(result.disposition, RecordingDisposition.COMPLETED)
        self.assertEqual(result.artifact_path, self.artifact)
        self.assertEqual(executor.commands[0], ("rosbag", "record", "-O", str(self.artifact), *TOPICS))
        self.assertNotIn("-a", executor.commands[0])
        self.assertEqual(executor.commands[1], ("rosbag", "info", "--yaml", str(self.artifact)))
        self.assertEqual(executor.process.signals, [signal.SIGINT])

    def test_started_session_remains_active_until_explicit_finish(self) -> None:
        # Given: rosbag reaches its active artifact before route navigation begins.
        executor = FakeExecutor(valid_info())
        session = self.recorder(executor).start(self.profile, self.identity)

        # When: route ownership checks the process and explicitly completes recording.
        session.assert_active()
        result = session.finish(RecordingDisposition.COMPLETED)

        # Then: one process spans the external work and validates only at finish.
        self.assertEqual(result.artifact_path, self.artifact)
        self.assertEqual(executor.process.signals, [signal.SIGINT])
        self.assertEqual(executor.commands[1], ("rosbag", "info", "--yaml", str(self.artifact)))

    def test_discarded_session_invalidates_active_artifact_without_validation(self) -> None:
        # Given: an active route recording that cannot be exposed as a result.
        executor = FakeExecutor(valid_info())
        session = self.recorder(executor).start(self.profile, self.identity)

        # When: route ownership discards the session.
        session.discard()

        # Then: the process is reaped and its generation is invalidated without rosbag info.
        self.assertEqual(executor.process.signals, [signal.SIGINT])
        self.assertEqual(len(executor.commands), 1)
        self.assertTrue(Path(str(self.artifact) + ".invalid").exists())

    def test_session_detects_process_exit_between_route_segments(self) -> None:
        # Given: rosbag creates its active marker but exits during route navigation.
        executor = FakeExecutor(valid_info(), exit_early=True)
        session = self.recorder(executor).start(self.profile, self.identity)

        # When/Then: a segment-boundary health check fails and cannot return an artifact.
        with self.assertRaisesRegex(ScanRecorderError, "exited during route recording"):
            session.assert_active()
        session.discard()
        self.assertTrue(Path(str(self.artifact) + ".active.invalid").exists())

    def test_duration_starts_after_delayed_active_artifact_readiness(self) -> None:
        # Given: rosbag advertises generation only after four bounded process waits.
        executor = FakeExecutor(valid_info(), active_lifecycle=True, active_after_waits=4)
        # When: recording is managed through delayed startup and configured duration.
        result = self.recorder(executor).record(self.profile, self.identity)
        # Then: startup waits are additional to the full 0.2-second recording window.
        self.assertEqual(result.disposition, RecordingDisposition.COMPLETED)
        self.assertAlmostEqual(sum(executor.process.wait_timeouts[:-1]), 0.6)

    def test_readiness_timeout_reaps_without_returning_artifact(self) -> None:
        # Given: rosbag remains alive but never creates its active generation file.
        executor = FakeExecutor(valid_info(), active_after_waits=1000)
        # When/Then: bounded readiness fails and recorder ownership is released.
        with self.assertRaisesRegex(ScanRecorderError, "readiness timeout"):
            self.recorder(executor).record(self.profile, self.identity)
        self.assertTrue(executor.process.reaped)
        self.assertEqual(executor.process.signals, [signal.SIGINT])

    def test_missing_mandatory_topic_rejects_artifact(self) -> None:
        # Given: rosbag metadata omits one mandatory profile topic.
        executor = FakeExecutor(valid_info(TOPICS[:-1]))
        # When/Then: validation fails instead of returning a scan result.
        with self.assertRaisesRegex(ScanRecorderError, "mandatory topic"):
            self.recorder(executor).record(self.profile, self.identity)

    def test_disk_precheck_fails_before_subprocess_start(self) -> None:
        # Given: available disk is below the profile threshold.
        executor = FakeExecutor(valid_info())
        # When/Then: precheck fails without spawning rosbag or creating an artifact.
        with self.assertRaisesRegex(ScanRecorderError, "free space"):
            self.recorder(executor, free_fraction=0.2).record(self.profile, self.identity)
        self.assertEqual(executor.commands, [])
        self.assertFalse(self.artifact.exists())

    def test_unwritable_output_shape_fails_before_subprocess_start(self) -> None:
        # Given: the requested output parent is an existing regular file.
        blocked_parent = Path(self.temporary_dir.name) / "blocked"
        blocked_parent.write_text("not a directory", encoding="utf-8")
        executor = FakeExecutor(valid_info())
        # When/Then: writable-output precheck fails before starting rosbag.
        with self.assertRaisesRegex(ScanRecorderError, "not writable"):
            self.recorder(executor, output_base=blocked_parent).record(self.profile, self.identity)
        self.assertEqual(executor.commands, [])

    def test_invalid_profile_fails_before_subprocess_start(self) -> None:
        # Given: a manually constructed profile with a relative mandatory topic.
        executor = FakeExecutor(valid_info())
        profile = ScanProfile("bad", ("scan",), 1.0, Path("scans"), 0.1)
        # When/Then: runtime precheck refuses the untrusted profile.
        with self.assertRaisesRegex(ScanRecorderError, "profile"):
            self.recorder(executor).record(profile, self.identity)
        self.assertEqual(executor.commands, [])

    def test_subprocess_failure_has_no_success_return(self) -> None:
        # Given: rosbag exits before the requested duration.
        executor = FakeExecutor(valid_info(), exit_early=True)
        # When/Then: the nonzero exit is surfaced as scan failure.
        with self.assertRaisesRegex(ScanRecorderError, "exited before duration"):
            self.recorder(executor).record(self.profile, self.identity)

    def test_cancellation_returns_preempted_valid_artifact(self) -> None:
        # Given: cancellation is requested after rosbag starts.
        executor = FakeExecutor(valid_info())
        calls = iter((False, True))
        # When: the recorder observes cancellation.
        result = self.recorder(executor).record(self.profile, self.identity, lambda: next(calls))
        # Then: SIGINT finalization and validation precede the PREEMPTED result.
        self.assertEqual(result.disposition, RecordingDisposition.PREEMPTED)
        self.assertEqual(result.artifact_path, self.artifact)
        self.assertEqual(executor.process.signals, [signal.SIGINT])

    def test_sigint_timeout_kills_process_and_marks_artifact_invalid(self) -> None:
        # Given: rosbag ignores SIGINT beyond the ten-second shutdown limit.
        executor = FakeExecutor(valid_info(), hang_on_sigint=True)
        # When/Then: forced termination is failure and cannot look like a valid bag.
        with self.assertRaisesRegex(ScanRecorderError, "forced termination"):
            self.recorder(executor).record(self.profile, self.identity)
        self.assertTrue(executor.process.killed)
        self.assertFalse(self.artifact.exists())
        self.assertTrue(Path(str(self.artifact) + ".active.invalid").exists())

    def test_artifact_validation_rejects_zero_duration_messages_and_malformed_yaml(self) -> None:
        # Given: three misleading artifacts that rosbag info can emit or expose.
        invalid_outputs = (
            valid_info(duration=0),
            valid_info(messages=0),
            "duration: [malformed",
        )
        # When/Then: none can produce a successful scan result.
        for output in invalid_outputs:
            with self.subTest(output=output):
                with self.assertRaises(ScanRecorderError):
                    self.recorder(FakeExecutor(output)).record(self.profile, self.identity)
                invalid_artifact = self.artifact.with_suffix(".bag.invalid")
                if invalid_artifact.exists():
                    invalid_artifact.unlink()

    def test_rosbag_info_subprocess_failure_rejects_artifact(self) -> None:
        # Given: recording finalizes but rosbag info exits nonzero.
        executor = FakeExecutor("", info_code=2)
        # When/Then: command failure prevents an artifact result.
        with self.assertRaisesRegex(ScanRecorderError, "rosbag info exited"):
            self.recorder(executor).record(self.profile, self.identity)

    def test_active_artifact_marker_is_rejected_as_stale_generation(self) -> None:
        # Given: an interrupted prior generation left rosbag's active marker.
        active = Path(str(self.artifact) + ".active")
        active.parent.mkdir(parents=True)
        active.write_bytes(b"interrupted")
        executor = FakeExecutor(valid_info())
        # When/Then: no process can reuse the misleading destination.
        with self.assertRaisesRegex(ScanRecorderError, "already exists"):
            self.recorder(executor).record(self.profile, self.identity)
        self.assertEqual(executor.commands, [])

    def test_stale_artifact_is_rejected_before_recording(self) -> None:
        # Given: a pre-existing bag at the requested destination.
        self.artifact.parent.mkdir(parents=True)
        self.artifact.write_bytes(b"stale")
        executor = FakeExecutor(valid_info())
        # When/Then: the recorder never overwrites or validates stale data.
        with self.assertRaisesRegex(ScanRecorderError, "already exists"):
            self.recorder(executor).record(self.profile, self.identity)
        self.assertEqual(executor.commands, [])

    def test_artifact_path_outside_profile_output_root_is_rejected(self) -> None:
        # Given: a caller selects an arbitrary destination unrelated to output_root.
        escaped_identity = RecordingIdentity("/".join(("..", "outside")), "roof", self.identity.timestamp_ns)
        executor = FakeExecutor(valid_info())
        # When/Then: profile ownership rejects the escape before process start.
        with self.assertRaisesRegex(ScanRecorderError, "recording identity"):
            self.recorder(executor).record(self.profile, escaped_identity)
        self.assertEqual(executor.commands, [])

    def test_forced_termination_marks_generated_active_bag_invalid(self) -> None:
        # Given: realistic rosbag generation exists only as capture.bag.active.
        executor = FakeExecutor(valid_info(), hang_on_sigint=True, active_lifecycle=True)
        # When/Then: forced termination reaps and invalidates the active artifact.
        with self.assertRaisesRegex(ScanRecorderError, "forced termination"):
            self.recorder(executor).record(self.profile, self.identity)
        self.assertTrue(executor.process.reaped)
        self.assertFalse(Path(str(self.artifact) + ".active").exists())
        self.assertTrue(Path(str(self.artifact) + ".active.invalid").exists())

    def test_nonfinite_duration_metadata_is_rejected(self) -> None:
        # Given/When/Then: YAML NaN and infinities cannot validate a scan.
        for duration in (".nan", ".inf", "-.inf"):
            with self.subTest(duration=duration):
                executor = FakeExecutor(valid_info(duration=duration))
                with self.assertRaisesRegex(ScanRecorderError, "duration"):
                    self.recorder(executor).record(self.profile, self.identity)
                Path(str(self.artifact) + ".invalid").unlink()

    def test_unexpected_wait_exception_still_reaps_owned_process(self) -> None:
        # Given: process waiting raises an unexpected interruption once.
        executor = FakeExecutor(valid_info(), wait_exception=RuntimeError("interrupted"), active_lifecycle=True)
        # When: the interruption escapes to the caller.
        with self.assertRaisesRegex(RuntimeError, "interrupted"):
            self.recorder(executor).record(self.profile, self.identity)
        # Then: recorder ownership still stops, reaps, and invalidates generation.
        self.assertTrue(executor.process.reaped)
        self.assertEqual(executor.process.signals, [signal.SIGINT])
        self.assertTrue(Path(str(self.artifact) + ".invalid").exists())

    def test_scan_failure_maps_to_mission_result_without_artifact_return(self) -> None:
        # Given: a recorder failure owned by a mission identity.
        error = ScanRecorderError("missing mandatory topic", self.artifact)
        # When: mission orchestration maps the library error.
        result = map_scan_failure(self.identity, error)
        # Then: Todo 14 can copy SCAN_FAILED fields without returning a failed bag.
        self.assertEqual(result, MissionScanFailure(7, str(error), "mission-42", ""))


if __name__ == "__main__":
    unittest.main()
