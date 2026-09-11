"""Typed mission-owned site configuration loaded from strict YAML."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import re
from typing import Dict, List, Tuple, Union

import yaml


YamlScalar = Union[str, int, float, bool, None]
YamlValue = Union[YamlScalar, List["YamlValue"], Dict[str, "YamlValue"]]
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


@dataclass(frozen=True)
class MissionConfigurationError(Exception):
    __slots__ = ("file_name", "field", "detail")
    file_name: str
    field: str
    detail: str

    def __str__(self) -> str:
        separator = ":" if self.field else ""
        return f"{self.file_name}:{self.field}{separator} {self.detail}"


class EdgeType(str, Enum):
    NAV = "NAV"
    STAIR_UP = "STAIR_UP"
    STAIR_DOWN = "STAIR_DOWN"


@dataclass(frozen=True)
class Location:
    __slots__ = ("id", "floor_id", "type", "x", "y", "yaw")
    id: str
    floor_id: str
    type: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GraphEdge:
    __slots__ = ("id", "source_id", "target_id", "type", "stair_id")
    id: str
    source_id: str
    target_id: str
    type: EdgeType
    stair_id: Union[str, None]


@dataclass(frozen=True)
class ScanProfile:
    __slots__ = ("id", "topics", "duration_sec", "output_root", "free_space_min")
    id: str
    topics: Tuple[str, ...]
    duration_sec: float
    output_root: Path
    free_space_min: float


@dataclass(frozen=True)
class MissionConfiguration:
    __slots__ = ("locations", "edges", "scan_profiles")
    locations: Tuple[Location, ...]
    edges: Tuple[GraphEdge, ...]
    scan_profiles: Tuple[ScanProfile, ...]


def _error(path: Path, field: str, detail: str) -> MissionConfigurationError:
    return MissionConfigurationError(path.name, field, detail)


def _document(path: Path, root_key: str) -> List[YamlValue]:
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        detail = "YAML parse error" if isinstance(error, yaml.YAMLError) else str(error)
        raise _error(path, "", detail) from None
    mapping = _mapping(raw, path, "")
    _keys(mapping, {"schema_version", "configured", root_key}, path, "")
    if type(mapping["schema_version"]) is not int or mapping["schema_version"] != 1:
        raise _error(path, "schema_version", "expected integer 1")
    if type(mapping["configured"]) is not bool:
        raise _error(path, "configured", "expected boolean")
    if not mapping["configured"]:
        raise _error(path, "configured", "must be true before use")
    return _list(mapping[root_key], path, root_key)


def _mapping(value: YamlValue, path: Path, field: str) -> Dict[str, YamlValue]:
    if type(value) is not dict:
        raise _error(path, field, "expected mapping")
    return value


def _list(value: YamlValue, path: Path, field: str) -> List[YamlValue]:
    if type(value) is not list:
        raise _error(path, field, "expected list")
    return value


def _keys(mapping: Dict[str, YamlValue], expected: set, path: Path, field: str) -> None:
    actual = set(mapping)
    if actual != expected:
        raise _error(path, field, f"keys must be {sorted(expected)}; got {sorted(actual)}")


def _string(value: YamlValue, path: Path, field: str) -> str:
    if type(value) is not str or not value:
        raise _error(path, field, "expected nonempty string")
    return value


def _identifier(value: YamlValue, path: Path, field: str) -> str:
    identifier = _string(value, path, field)
    if not _ID_PATTERN.fullmatch(identifier):
        raise _error(path, field, "invalid ID")
    return identifier


def _number(value: YamlValue, path: Path, field: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise _error(path, field, "expected finite number")
    return float(value)


def _unique(identifier: str, seen: set, path: Path, field: str) -> None:
    if identifier in seen:
        raise _error(path, field, f"duplicate ID {identifier!r}")
    seen.add(identifier)


def _load_locations(root: Path) -> Tuple[Location, ...]:
    path = root / "locations.yaml"
    result = []
    seen = set()
    for index, value in enumerate(_document(path, "locations")):
        field = f"locations[{index}]"
        item = _mapping(value, path, field)
        _keys(item, {"id", "floor_id", "type", "x", "y", "yaw"}, path, field)
        identifier = _identifier(item["id"], path, field + ".id")
        _unique(identifier, seen, path, field + ".id")
        yaw = _number(item["yaw"], path, field + ".yaw")
        if not -math.pi <= yaw <= math.pi:
            raise _error(path, field + ".yaw", "must be within [-pi, pi]")
        result.append(Location(identifier, _identifier(item["floor_id"], path, field + ".floor_id"), _identifier(item["type"], path, field + ".type"), _number(item["x"], path, field + ".x"), _number(item["y"], path, field + ".y"), yaw))
    if not result:
        raise _error(path, "locations", "must not be empty")
    return tuple(result)


def _load_edges(root: Path) -> Tuple[GraphEdge, ...]:
    path = root / "building_graph.yaml"
    result = []
    seen = set()
    for index, value in enumerate(_document(path, "edges")):
        field = f"edges[{index}]"
        item = _mapping(value, path, field)
        required = {"id", "from", "to", "type"}
        allowed = required | {"stair_id"}
        if not required.issubset(item) or not set(item).issubset(allowed):
            raise _error(path, field, f"keys must include {sorted(required)} and only {sorted(allowed)}")
        identifier = _identifier(item["id"], path, field + ".id")
        _unique(identifier, seen, path, field + ".id")
        try:
            edge_type = EdgeType(_string(item["type"], path, field + ".type"))
        except ValueError:
            raise _error(path, field + ".type", "expected NAV, STAIR_UP, or STAIR_DOWN") from None
        stair_id = _identifier(item["stair_id"], path, field + ".stair_id") if "stair_id" in item else None
        if (edge_type is EdgeType.NAV) == (stair_id is not None):
            raise _error(path, field + ".stair_id", "required only for stair edges")
        result.append(GraphEdge(identifier, _identifier(item["from"], path, field + ".from"), _identifier(item["to"], path, field + ".to"), edge_type, stair_id))
    if not result:
        raise _error(path, "edges", "must not be empty")
    return tuple(result)


def _load_scan_profiles(root: Path) -> Tuple[ScanProfile, ...]:
    path = root / "scan_profiles.yaml"
    result = []
    seen = set()
    for index, value in enumerate(_document(path, "profiles")):
        field = f"profiles[{index}]"
        item = _mapping(value, path, field)
        _keys(item, {"id", "topics", "duration_sec", "output_root", "free_space_min"}, path, field)
        identifier = _identifier(item["id"], path, field + ".id")
        _unique(identifier, seen, path, field + ".id")
        topic_values = _list(item["topics"], path, field + ".topics")
        if not topic_values:
            raise _error(path, field + ".topics", "must not be empty")
        topics = tuple(_string(topic, path, f"{field}.topics[{topic_index}]") for topic_index, topic in enumerate(topic_values))
        if len(set(topics)) != len(topics) or any(not topic.startswith("/") for topic in topics):
            raise _error(path, field + ".topics", "topics must be unique absolute names")
        duration = _number(item["duration_sec"], path, field + ".duration_sec")
        free_space = _number(item["free_space_min"], path, field + ".free_space_min")
        output_root = Path(_string(item["output_root"], path, field + ".output_root"))
        if duration <= 0 or not 0 < free_space <= 1 or output_root.is_absolute() or ".." in output_root.parts:
            raise _error(path, field, "duration/free-space/output-root policy is invalid")
        result.append(ScanProfile(identifier, topics, duration, output_root, free_space))
    if not result:
        raise _error(path, "profiles", "must not be empty")
    return tuple(result)


def load_mission_configuration(root: Path) -> MissionConfiguration:
    """Parse all mission-owned YAML files into immutable values."""
    return MissionConfiguration(_load_locations(root), _load_edges(root), _load_scan_profiles(root))
