"""Managed rosbag lifecycle and mission-facing scan outcomes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from typing import Callable, Protocol, Sequence, Tuple

import yaml

from mission_manager.configuration import ScanProfile
from mission_manager.recording_session import (
    ManagedProcess,
    ProcessExitedError,
    RecordingDisposition as RecordingDisposition,
    RecordingResult,
    RecordingSession,
    RecordingSessionRuntime,
    ScanRecorderError,
)


INFO_TIMEOUT_SEC = 10.0
SCAN_FAILED_RESULT_CODE = 7
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


@dataclass(frozen=True)
class RecordingIdentity:
    __slots__ = ("mission_id", "location_id", "timestamp_ns")
    mission_id: str
    location_id: str
    timestamp_ns: int


@dataclass(frozen=True)
class MissionScanFailure:
    __slots__ = ("result_code", "reason", "mission_id", "artifact_path")
    result_code: int
    reason: str
    mission_id: str
    artifact_path: str


@dataclass(frozen=True)
class CommandResult:
    __slots__ = ("return_code", "stdout", "stderr")
    return_code: int
    stdout: str
    stderr: str


@dataclass(frozen=True)
class DiskCapacity:
    __slots__ = ("total_bytes", "free_bytes")
    total_bytes: int
    free_bytes: int


class CommandExecutor(Protocol):
    def start(self, argv: Sequence[str]) -> ManagedProcess:
        ...

    def run(self, argv: Sequence[str], timeout: float) -> CommandResult:
        ...


class SubprocessExecutor:
    def start(self, argv: Sequence[str]) -> ManagedProcess:
        return subprocess.Popen(
            tuple(argv),
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )

    def run(self, argv: Sequence[str], timeout: float) -> CommandResult:
        completed = subprocess.run(
            tuple(argv), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, timeout=timeout, check=False,
        )
        return CommandResult(completed.returncode, completed.stdout, completed.stderr)


def _disk_capacity(path: Path) -> DiskCapacity:
    usage = shutil.disk_usage(str(path))
    return DiskCapacity(usage.total, usage.free)


@dataclass(frozen=True)
class RecorderRuntime:
    __slots__ = ("executor", "disk_capacity", "poll_interval_sec")
    executor: CommandExecutor
    disk_capacity: Callable[[Path], DiskCapacity]
    poll_interval_sec: float


def map_scan_failure(identity: RecordingIdentity, error: ScanRecorderError) -> MissionScanFailure:
    """Expose fields Todo 14 can copy into MissionResult without importing ROS."""
    return MissionScanFailure(SCAN_FAILED_RESULT_CODE, str(error), identity.mission_id, "")


class ScanRecorder:
    """Own one rosbag process from profile-derived path through validation."""

    def __init__(self, output_base: Path, runtime: RecorderRuntime | None = None) -> None:
        self._output_base = output_base.resolve()
        self._runtime = runtime if runtime is not None else RecorderRuntime(SubprocessExecutor(), _disk_capacity, 0.1)

    def artifact_path(self, profile: ScanProfile, identity: RecordingIdentity) -> Path:
        """Derive the only accepted final path from profile policy and mission identity."""
        self._validate_profile_and_identity(profile, identity)
        output_root = (self._output_base / profile.output_root).resolve()
        try:
            output_root.relative_to(self._output_base)
        except ValueError as error:
            raise ScanRecorderError("profile output root escapes output base", output_root) from error
        name = f"{identity.mission_id}_{identity.location_id}_{identity.timestamp_ns}.bag"
        return output_root / name

    def record(
        self,
        profile: ScanProfile,
        identity: RecordingIdentity,
        cancellation_requested: Callable[[], bool] = lambda: False,
    ) -> RecordingResult:
        """Record until duration or cancellation, returning only a validated artifact."""
        session = self.start(profile, identity)
        try:
            disposition = session.await_duration(
                profile.duration_sec,
                cancellation_requested,
            )
            return session.finish(disposition)
        finally:
            session.discard()

    def start(
        self,
        profile: ScanProfile,
        identity: RecordingIdentity,
        extra_topics: Tuple[str, ...] = (),
    ) -> RecordingSession:
        """Start one active session whose terminal operation is caller-owned."""
        artifact = self.artifact_path(profile, identity)
        if (
            any(not topic.startswith("/") for topic in extra_topics)
            or len(set(extra_topics)) != len(extra_topics)
            or set(extra_topics).intersection(profile.topics)
        ):
            raise ScanRecorderError("invalid optional recording topics", artifact)
        self._precheck_output(profile, artifact)
        try:
            process = self._runtime.executor.start(
                ("rosbag", "record", "-O", str(artifact), *profile.topics, *extra_topics)
            )
        except OSError as error:
            raise ScanRecorderError(f"unable to start rosbag: {error}", artifact) from error
        session = RecordingSession(
            RecordingSessionRuntime(
                process,
                artifact,
                profile.topics,
                self._runtime.poll_interval_sec,
                self._validate_artifact,
                self._invalidate_generated,
            )
        )
        try:
            session.await_readiness()
        except (ProcessExitedError, ScanRecorderError):
            session.discard()
            raise
        return session

    def _validate_profile_and_identity(self, profile: ScanProfile, identity: RecordingIdentity) -> None:
        valid_profile = (
            bool(profile.id) and bool(profile.topics)
            and len(set(profile.topics)) == len(profile.topics)
            and all(topic.startswith("/") for topic in profile.topics)
            and math.isfinite(profile.duration_sec) and profile.duration_sec > 0
            and math.isfinite(profile.free_space_min) and 0 < profile.free_space_min <= 1
            and not profile.output_root.is_absolute() and ".." not in profile.output_root.parts
        )
        valid_identity = (
            _IDENTIFIER.fullmatch(identity.mission_id) is not None
            and _IDENTIFIER.fullmatch(identity.location_id) is not None
            and type(identity.timestamp_ns) is int and identity.timestamp_ns > 0
        )
        if not valid_profile or not valid_identity or self._runtime.poll_interval_sec <= 0:
            raise ScanRecorderError("invalid scan profile or recording identity", None)

    def _precheck_output(self, profile: ScanProfile, artifact: Path) -> None:
        generated = self._generated_paths(artifact)
        if any(path.exists() for path in generated):
            raise ScanRecorderError("artifact or lifecycle marker already exists", artifact)
        try:
            artifact.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryFile(dir=str(artifact.parent)):
                pass
            capacity = self._runtime.disk_capacity(artifact.parent)
        except OSError as error:
            raise ScanRecorderError(f"output is not writable: {error}", artifact) from error
        if capacity.total_bytes <= 0 or capacity.free_bytes / capacity.total_bytes < profile.free_space_min:
            raise ScanRecorderError("insufficient free space", artifact)

    def _validate_artifact(self, artifact: Path, mandatory_topics: Tuple[str, ...]) -> None:
        if not artifact.is_file() or Path(str(artifact) + ".active").exists():
            raise ScanRecorderError("finalized bag artifact is missing", artifact)
        try:
            info = self._runtime.executor.run(("rosbag", "info", "--yaml", str(artifact)), INFO_TIMEOUT_SEC)
        except (OSError, subprocess.TimeoutExpired) as error:
            raise ScanRecorderError(f"rosbag info failed: {error}", artifact) from error
        if info.return_code != 0:
            raise ScanRecorderError(f"rosbag info exited with code {info.return_code}: {info.stderr}", artifact)
        try:
            document = yaml.safe_load(info.stdout)
        except yaml.YAMLError as error:
            raise ScanRecorderError("rosbag info returned malformed YAML", artifact) from error
        duration = document.get("duration") if type(document) is dict else None
        topics = document.get("topics") if type(document) is dict else None
        if type(duration) not in (int, float) or not math.isfinite(duration) or duration <= 0:
            raise ScanRecorderError("rosbag info has invalid duration", artifact)
        if type(topics) is not list:
            raise ScanRecorderError("rosbag info has invalid topics", artifact)
        messages = {}
        for item in topics:
            if type(item) is not dict or type(item.get("topic")) is not str or type(item.get("messages")) is not int:
                raise ScanRecorderError("rosbag info has malformed topic metadata", artifact)
            messages[item["topic"]] = item["messages"]
        if any(messages.get(topic, 0) <= 0 for topic in mandatory_topics):
            raise ScanRecorderError("mandatory topic is missing or has no messages", artifact)

    @staticmethod
    def _generated_paths(artifact: Path) -> Tuple[Path, ...]:
        final = str(artifact)
        return (artifact, Path(final + ".active"), Path(final + ".invalid"), Path(final + ".active.invalid"))

    @staticmethod
    def _invalidate_generated(artifact: Path) -> None:
        for generated in (artifact, Path(str(artifact) + ".active")):
            if generated.exists():
                generated.replace(Path(str(generated) + ".invalid"))
