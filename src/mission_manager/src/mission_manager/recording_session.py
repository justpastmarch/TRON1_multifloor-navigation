"""Single-owner lifecycle for one externally bracketed rosbag process."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import signal
import subprocess
from typing import Protocol


SHUTDOWN_TIMEOUT_SEC = 10.0
READINESS_TIMEOUT_SEC = 10.0


class RecordingDisposition(str, Enum):
    COMPLETED = "COMPLETED"
    PREEMPTED = "PREEMPTED"
    INTERRUPTED = "INTERRUPTED"


@dataclass(frozen=True)
class RecordingResult:
    __slots__ = ("disposition", "artifact_path")
    disposition: RecordingDisposition
    artifact_path: Path


@dataclass(frozen=True)
class ScanRecorderError(Exception):
    __slots__ = ("detail", "artifact_path")
    detail: str
    artifact_path: Path | None

    def __str__(self) -> str:
        suffix = f": {self.artifact_path}" if self.artifact_path is not None else ""
        return f"scan recording failed: {self.detail}{suffix}"


class ProcessExitedError(ScanRecorderError):
    pass


class ManagedProcess(Protocol):
    def wait(self, timeout: float) -> int:
        ...

    def send_signal(self, value: int) -> None:
        ...

    def kill(self) -> None:
        ...


@dataclass(frozen=True)
class RecordingSessionRuntime:
    __slots__ = (
        "process",
        "artifact",
        "mandatory_topics",
        "poll_interval_sec",
        "validate_artifact",
        "invalidate_generated",
    )
    process: ManagedProcess
    artifact: Path
    mandatory_topics: tuple[str, ...]
    poll_interval_sec: float
    validate_artifact: Callable[[Path, tuple[str, ...]], None]
    invalidate_generated: Callable[[Path], None]


class RecordingSession:
    """Own a mutable rosbag process until one terminal lifecycle operation."""

    def __init__(self, runtime: RecordingSessionRuntime) -> None:
        self._runtime = runtime
        self._reaped = False
        self._terminal = False

    def await_readiness(self) -> None:
        """Wait until rosbag exposes its active generation marker."""
        active_artifact = Path(str(self._runtime.artifact) + ".active")
        elapsed = 0.0
        while elapsed < READINESS_TIMEOUT_SEC:
            if active_artifact.is_file():
                return
            interval = min(self._runtime.poll_interval_sec, READINESS_TIMEOUT_SEC - elapsed)
            try:
                return_code = self._runtime.process.wait(timeout=interval)
            except subprocess.TimeoutExpired:
                elapsed += interval
                continue
            self._reaped = True
            raise ProcessExitedError(
                f"rosbag exited before readiness with code {return_code}",
                self._runtime.artifact,
            )
        raise ScanRecorderError(
            "rosbag readiness timeout waiting for .bag.active",
            self._runtime.artifact,
        )

    def await_duration(
        self,
        duration_sec: float,
        cancellation_requested: Callable[[], bool],
    ) -> RecordingDisposition:
        """Preserve destination-only recording behavior for the compatibility API."""
        elapsed = 0.0
        while elapsed < duration_sec:
            if cancellation_requested():
                return RecordingDisposition.PREEMPTED
            interval = min(self._runtime.poll_interval_sec, duration_sec - elapsed)
            try:
                return_code = self._runtime.process.wait(timeout=interval)
            except subprocess.TimeoutExpired:
                elapsed += interval
                continue
            self._reaped = True
            raise ProcessExitedError(
                f"rosbag exited before duration with code {return_code}",
                self._runtime.artifact,
            )
        return RecordingDisposition.COMPLETED

    def assert_active(self) -> None:
        """Fail at a route-segment boundary when rosbag has exited unexpectedly."""
        self._require_open()
        try:
            return_code = self._runtime.process.wait(timeout=0.0)
        except subprocess.TimeoutExpired:
            return
        self._reaped = True
        raise ScanRecorderError(
            f"rosbag exited during route recording with code {return_code}",
            self._runtime.artifact,
        )

    def finish(self, disposition: RecordingDisposition) -> RecordingResult:
        """Finalize and validate exactly once."""
        self._require_open()
        try:
            try:
                forced = self._stop_and_reap()
            finally:
                self._reaped = True
            if forced:
                raise ScanRecorderError(
                    "forced termination after SIGINT timeout",
                    self._runtime.artifact,
                )
            self._runtime.validate_artifact(
                self._runtime.artifact,
                self._runtime.mandatory_topics,
            )
        except (OSError, ScanRecorderError, subprocess.TimeoutExpired):
            self._runtime.invalidate_generated(self._runtime.artifact)
            self._terminal = True
            raise
        self._terminal = True
        return RecordingResult(disposition, self._runtime.artifact)

    def discard(self) -> None:
        """Reap and invalidate an unfinished generation."""
        if self._terminal:
            return
        try:
            if not self._reaped:
                try:
                    self._stop_and_reap()
                finally:
                    self._reaped = True
        finally:
            self._runtime.invalidate_generated(self._runtime.artifact)
            self._terminal = True

    def _require_open(self) -> None:
        if self._terminal:
            raise ScanRecorderError("recording session is already terminal", self._runtime.artifact)

    def _stop_and_reap(self) -> bool:
        self._runtime.process.send_signal(signal.SIGINT)
        try:
            return_code = self._runtime.process.wait(timeout=SHUTDOWN_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            self._runtime.process.kill()
            self._runtime.process.wait(timeout=SHUTDOWN_TIMEOUT_SEC)
            return True
        if return_code != 0:
            raise ScanRecorderError(
                f"rosbag finalization exited with code {return_code}",
                self._runtime.artifact,
            )
        return False
