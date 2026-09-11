from __future__ import annotations

from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
MISSION_SOURCE = ROOT / "src" / "mission_manager" / "src"
MISSION_TESTS = ROOT / "src" / "mission_manager" / "test"
sys.path.insert(0, str(MISSION_SOURCE))
sys.path.insert(0, str(MISSION_TESTS))

from mission_manager.configuration import ScanProfile  # noqa: E402
from mission_manager.recording_session import RecordingDisposition  # noqa: E402
from mission_manager.scan_recorder import (  # noqa: E402
    DiskCapacity,
    RecorderRuntime,
    RecordingIdentity,
    ScanRecorder,
)
from test_scan_recorder import FakeExecutor, TOPICS, valid_info  # noqa: E402


def test_optional_topic_is_recorded_without_becoming_a_completion_requirement() -> None:
    # Given: an optional response stream has no messages in an otherwise valid bag.
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory)
        profile = ScanProfile("roof", TOPICS, 0.2, Path("artifacts"), 0.1)
        identity = RecordingIdentity("mission-42", "roof", 1700000000000000000)
        executor = FakeExecutor(valid_info())
        runtime = RecorderRuntime(executor, lambda _path: DiskCapacity(1000, 800), 0.1)

        # When: the session records it as an extra topic and completes.
        session = ScanRecorder(output, runtime).start(
            profile, identity, extra_topics=("/tron/manual_command/response",)
        )
        result = session.finish(RecordingDisposition.COMPLETED)

        # Then: rosbag subscribes to it while validating only the profile topics.
        artifact = output / "artifacts" / "mission-42_roof_1700000000000000000.bag"
        assert result.artifact_path == artifact
        assert executor.commands[0] == (
            "rosbag", "record", "-O", str(artifact), *TOPICS,
            "/tron/manual_command/response",
        )
