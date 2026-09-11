"""Typed loaders and temporary-output helpers for the test fixture corpus."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from json import JSONDecodeError
from pathlib import Path
import re
import sys
import tempfile
from typing import Iterator, Mapping, Tuple

from fixture_models import (
    AmclInput,
    DirectedEdge,
    DirectedGraph,
    FixtureSummary,
    FrozenClock,
    JsonValue,
    LocalizationCases,
    TagInput,
    TfInput,
    WebSocketFrame,
)


FIXTURE_ROOT = Path(__file__).resolve().parent / "fixtures"


class FixtureError(Exception):
    def __init__(self, path: Path, detail: str) -> None:
        self.path = path
        self.detail = detail
        super().__init__(str(self))

    def __str__(self) -> str:
        return f"fixture {self.path}: {self.detail}"


def _load_json(path: Path) -> JsonValue:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, JSONDecodeError) as error:
        raise FixtureError(path, f"invalid JSON: {error}") from error


def _mapping(value: JsonValue, path: Path, field: str) -> Mapping[str, JsonValue]:
    if not isinstance(value, dict):
        raise FixtureError(path, f"{field} must be an object")
    return value


def _integer(value: JsonValue, path: Path, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise FixtureError(path, f"{field} must be an integer")
    return value


def _string(value: JsonValue, path: Path, field: str) -> str:
    if not isinstance(value, str):
        raise FixtureError(path, f"{field} must be a string")
    return value


def _number(value: JsonValue, path: Path, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FixtureError(path, f"{field} must be a number")
    return float(value)


def _document(path: Path) -> Mapping[str, JsonValue]:
    document = _mapping(_load_json(path), path, "document")
    if _integer(document.get("schema_version"), path, "schema_version") != 1:
        raise FixtureError(path, "unsupported schema_version")
    return document


def load_directed_graph(root: Path = FIXTURE_ROOT) -> DirectedGraph:
    path = root / "graphs/asymmetric_3f_4f_roof.json"
    document = _document(path)
    if document.get("directed") is not True:
        raise FixtureError(path, "directed must be true")
    floors_value = document.get("floors")
    edges_value = document.get("edges")
    if not isinstance(floors_value, list) or not isinstance(edges_value, list):
        raise FixtureError(path, "floors and edges must be arrays")
    floors = tuple(_string(value, path, "floors[]") for value in floors_value)
    edges = []
    for value in edges_value:
        edge = _mapping(value, path, "edges[]")
        edges.append(DirectedEdge(
            _string(edge.get("from"), path, "edges[].from"),
            _string(edge.get("to"), path, "edges[].to"),
            _integer(edge.get("cost"), path, "edges[].cost"),
        ))
    return DirectedGraph(floors, tuple(edges))


def load_frozen_clock(root: Path = FIXTURE_ROOT) -> FrozenClock:
    path = root / "clock/frozen_time.json"
    document = _document(path)
    return FrozenClock(
        _integer(document.get("now_ns"), path, "now_ns"),
        _integer(document.get("freshness_limit_ns"), path, "freshness_limit_ns"),
    )


def assert_fresh(stamp_ns: int, clock: FrozenClock, path: Path) -> None:
    age_ns = clock.now_ns - stamp_ns
    if age_ns < 0 or age_ns > clock.freshness_limit_ns:
        raise FixtureError(path, f"stale timestamp age_ns={age_ns}")


def load_localization_cases(root: Path = FIXTURE_ROOT) -> LocalizationCases:
    path = root / "inputs/localization_cases.json"
    document = _document(path)
    tag = _mapping(document.get("tag"), path, "tag")
    amcl = _mapping(document.get("amcl"), path, "amcl")
    tf = _mapping(document.get("tf"), path, "tf")
    def parse_tag(name: str) -> TagInput:
        sample = _mapping(tag.get(name), path, f"tag.{name}")
        return TagInput(
            _integer(sample.get("id"), path, f"tag.{name}.id"),
            _string(sample.get("frame_id"), path, f"tag.{name}.frame_id"),
            _integer(sample.get("stamp_ns"), path, f"tag.{name}.stamp_ns"),
        )

    def parse_amcl(name: str) -> AmclInput:
        sample = _mapping(amcl.get(name), path, f"amcl.{name}")
        covariance_value = sample.get("covariance")
        if not isinstance(covariance_value, list):
            raise FixtureError(path, f"amcl.{name}.covariance must be an array")
        covariance = tuple(
            _number(value, path, f"amcl.{name}.covariance[]")
            for value in covariance_value
        )
        if len(covariance) != 36:
            raise FixtureError(path, f"amcl.{name}.covariance must contain 36 values")
        return AmclInput(
            _string(sample.get("frame_id"), path, f"amcl.{name}.frame_id"),
            _number(sample.get("x"), path, f"amcl.{name}.x"),
            _number(sample.get("y"), path, f"amcl.{name}.y"),
            _number(sample.get("yaw"), path, f"amcl.{name}.yaw"),
            covariance,
            _integer(sample.get("stamp_ns"), path, f"amcl.{name}.stamp_ns"),
        )

    def parse_tf(name: str) -> TfInput:
        sample = _mapping(tf.get(name), path, f"tf.{name}")
        return TfInput(
            _string(sample.get("parent"), path, f"tf.{name}.parent"),
            _string(sample.get("child"), path, f"tf.{name}.child"),
            _integer(sample.get("stamp_ns"), path, f"tf.{name}.stamp_ns"),
        )

    return LocalizationCases(
        parse_tag("valid"), parse_tag("invalid"),
        parse_amcl("valid"), parse_amcl("invalid"),
        parse_tf("valid"), parse_tf("invalid"), parse_tf("stale"),
    )


def load_websocket_transcript(root: Path = FIXTURE_ROOT) -> Tuple[WebSocketFrame, ...]:
    path = root / "websocket/robot_session.ndjson"
    clock = load_frozen_clock(root)
    frames = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise FixtureError(path, str(error)) from error
    for line_number, line in enumerate(lines, start=1):
        try:
            frame = _mapping(json.loads(line), path, f"line {line_number}")
        except JSONDecodeError as error:
            raise FixtureError(path, f"line {line_number}: invalid JSON: {error}") from error
        payload = _mapping(frame.get("payload"), path, f"line {line_number}.payload")
        at_ns = _integer(frame.get("at_ns"), path, f"line {line_number}.at_ns")
        direction = _string(frame.get("direction"), path, f"line {line_number}.direction")
        if direction not in ("client_to_robot", "robot_to_client"):
            raise FixtureError(path, f"line {line_number}.direction is unsupported")
        assert_fresh(at_ns, clock, path)
        frames.append(WebSocketFrame(
            at_ns,
            direction,
            json.dumps(payload, sort_keys=True, separators=(",", ":")),
        ))
    return tuple(frames)


def inspect_fixtures(root: Path = FIXTURE_ROOT) -> FixtureSummary:
    graph = load_directed_graph(root)
    clock = load_frozen_clock(root)
    localization = load_localization_cases(root)
    frames = load_websocket_transcript(root)
    map_names = tuple(
        name for name in graph.floors
        if (root / "maps" / f"{name}.yaml").is_file()
        and (root / "maps" / f"{name}.pgm").is_file()
    )
    return FixtureSummary(graph.floors, map_names, len(localization), len(frames), clock.now_ns)


@contextmanager
def temporary_rosbag_output(test_id: str) -> Iterator[Path]:
    safe_id = re.sub(r"[^A-Za-z0-9_.-]+", "-", test_id).strip("-") or "test"
    with tempfile.TemporaryDirectory(prefix=f"tron1-{safe_id}-rosbag-") as directory:
        yield Path(directory) / "capture.bag"


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect deterministic TRON1 test fixtures")
    parser.add_argument(
        "command",
        choices=("inspect", "check-json", "check-fresh", "check-transcript"),
    )
    parser.add_argument("value", nargs="?")
    arguments = parser.parse_args()
    try:
        if arguments.command == "inspect":
            summary = inspect_fixtures()
            print(json.dumps(summary._asdict(), sort_keys=True))
        elif arguments.command == "check-json":
            _document(Path(arguments.value or ""))
            print("fixture JSON valid")
        elif arguments.command == "check-fresh":
            stamp_ns = int(arguments.value or "0")
            assert_fresh(stamp_ns, load_frozen_clock(), FIXTURE_ROOT / "clock/frozen_time.json")
            print("timestamp fresh")
        else:
            frames = load_websocket_transcript(Path(arguments.value or FIXTURE_ROOT))
            print(f"transcript valid frames={len(frames)}")
    except (FixtureError, ValueError) as error:
        print(error, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
