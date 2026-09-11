"""Typed multifloor site configuration loaded from strict YAML."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Dict, List, Tuple, Union

import yaml

from multifloor_manager.transitions import TransitionPolicy


YamlScalar = Union[str, int, float, bool, None]
YamlValue = Union[YamlScalar, List["YamlValue"], Dict[str, "YamlValue"]]
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


@dataclass(frozen=True)
class MultifloorConfigurationError(Exception):
    __slots__ = ("file_name", "field", "detail")
    file_name: str
    field: str
    detail: str

    def __str__(self) -> str:
        separator = ":" if self.field else ""
        return f"{self.file_name}:{self.field}{separator} {self.detail}"


@dataclass(frozen=True)
class Floor:
    __slots__ = ("id", "map_yaml", "frame")
    id: str
    map_yaml: Path
    frame: str


@dataclass(frozen=True)
class Quaternion:
    __slots__ = ("x", "y", "z", "w")
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class LandingPose:
    __slots__ = ("x", "y", "orientation")
    x: float
    y: float
    orientation: Quaternion


@dataclass(frozen=True)
class StairEndpoint:
    __slots__ = ("floor_id", "entry_location_id", "target_landing", "covariance", "expected_tag_ids")
    floor_id: str
    entry_location_id: str
    target_landing: LandingPose
    covariance: Tuple[float, ...]
    expected_tag_ids: Tuple[int, ...]


@dataclass(frozen=True)
class Stair:
    __slots__ = ("id", "from_floor", "to_floor", "endpoints", "up_profile_id", "down_profile_id")
    id: str
    from_floor: str
    to_floor: str
    endpoints: Tuple[StairEndpoint, ...]
    up_profile_id: str
    down_profile_id: str

    def endpoint_for(self, floor_id: str) -> Union[StairEndpoint, None]:
        """Return this stair's immutable endpoint for one connected floor."""
        return next((endpoint for endpoint in self.endpoints if endpoint.floor_id == floor_id), None)


@dataclass(frozen=True)
class AprilTag:
    __slots__ = ("id", "family", "size_m", "role", "floor_id", "stair_id")
    id: int
    family: str
    size_m: float
    role: str
    floor_id: str
    stair_id: str


@dataclass(frozen=True)
class MultifloorConfiguration:
    __slots__ = ("floors", "stairs", "tags", "transitions")
    floors: Tuple[Floor, ...]
    stairs: Tuple[Stair, ...]
    tags: Tuple[AprilTag, ...]
    transitions: Tuple[TransitionPolicy, ...]


def _error(path: Path, field: str, detail: str) -> MultifloorConfigurationError:
    return MultifloorConfigurationError(path.name, field, detail)


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
    if set(mapping) != expected:
        raise _error(path, field, f"keys must be {sorted(expected)}; got {sorted(mapping)}")


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


def _unique(identifier: Union[str, int], seen: set, path: Path, field: str) -> None:
    if identifier in seen:
        raise _error(path, field, f"duplicate ID {identifier!r}")
    seen.add(identifier)


def _load_floors(root: Path) -> Tuple[Floor, ...]:
    path = root / "floors.yaml"
    result, seen = [], set()
    for index, value in enumerate(_document(path, "floors")):
        field = f"floors[{index}]"
        item = _mapping(value, path, field)
        _keys(item, {"id", "map_yaml", "frame"}, path, field)
        identifier = _identifier(item["id"], path, field + ".id")
        _unique(identifier, seen, path, field + ".id")
        map_yaml = Path(_string(item["map_yaml"], path, field + ".map_yaml"))
        if map_yaml.is_absolute() or ".." in map_yaml.parts or map_yaml.suffix != ".yaml":
            raise _error(path, field + ".map_yaml", "must be a relative YAML path without parent traversal")
        frame = _string(item["frame"], path, field + ".frame")
        if frame != "map":
            raise _error(path, field + ".frame", "must be map")
        result.append(Floor(identifier, map_yaml, frame))
    if not result:
        raise _error(path, "floors", "must not be empty")
    return tuple(result)


def _quaternion(value: YamlValue, path: Path, field: str) -> Quaternion:
    item = _mapping(value, path, field)
    _keys(item, {"x", "y", "z", "w"}, path, field)
    values = tuple(_number(item[key], path, field + "." + key) for key in ("x", "y", "z", "w"))
    if not math.isclose(sum(component * component for component in values), 1.0, rel_tol=1e-6, abs_tol=1e-6):
        raise _error(path, field, "quaternion must be normalized")
    return Quaternion(*values)


def _load_stairs(root: Path) -> Tuple[Stair, ...]:
    path = root / "stairs.yaml"
    result, seen = [], set()
    expected = {"id", "from_floor", "to_floor", "endpoints", "up_profile_id", "down_profile_id"}
    for index, value in enumerate(_document(path, "stairs")):
        field = f"stairs[{index}]"
        item = _mapping(value, path, field)
        _keys(item, expected, path, field)
        identifier = _identifier(item["id"], path, field + ".id")
        _unique(identifier, seen, path, field + ".id")
        from_floor = _identifier(item["from_floor"], path, field + ".from_floor")
        to_floor = _identifier(item["to_floor"], path, field + ".to_floor")
        endpoints = _mapping(item["endpoints"], path, field + ".endpoints")
        endpoint_floors = {from_floor, to_floor}
        if set(endpoints) != endpoint_floors:
            raise _error(path, field + ".endpoints", f"keys must be {sorted(endpoint_floors)}; got {sorted(endpoints)}")
        parsed_endpoints = tuple(_load_endpoint(floor_id, endpoints[floor_id], path, field + ".endpoints." + floor_id) for floor_id in (from_floor, to_floor))
        result.append(Stair(identifier, from_floor, to_floor, parsed_endpoints, _identifier(item["up_profile_id"], path, field + ".up_profile_id"), _identifier(item["down_profile_id"], path, field + ".down_profile_id")))
    if not result:
        raise _error(path, "stairs", "must not be empty")
    return tuple(result)


def _load_endpoint(floor_id: str, value: YamlValue, path: Path, field: str) -> StairEndpoint:
    item = _mapping(value, path, field)
    _keys(item, {"entry_location_id", "target_landing", "covariance", "expected_tag_ids"}, path, field)
    pose = _mapping(item["target_landing"], path, field + ".target_landing")
    _keys(pose, {"x", "y", "orientation"}, path, field + ".target_landing")
    landing = LandingPose(_number(pose["x"], path, field + ".target_landing.x"), _number(pose["y"], path, field + ".target_landing.y"), _quaternion(pose["orientation"], path, field + ".target_landing.orientation"))
    covariance_values = _list(item["covariance"], path, field + ".covariance")
    if len(covariance_values) != 36:
        raise _error(path, field + ".covariance", "expected 36 values")
    covariance = tuple(_number(number, path, f"{field}.covariance[{offset}]") for offset, number in enumerate(covariance_values))
    if any(number != 0.0 for offset, number in enumerate(covariance) if offset % 7 != 0) or any(covariance[offset] < 0 for offset in range(0, 36, 7)):
        raise _error(path, field + ".covariance", "must be a nonnegative 6x6 diagonal covariance")
    tag_values = _list(item["expected_tag_ids"], path, field + ".expected_tag_ids")
    if not tag_values:
        raise _error(path, field + ".expected_tag_ids", "must not be empty")
    tag_ids = tuple(_positive_integer(tag, path, f"{field}.expected_tag_ids[{index}]") for index, tag in enumerate(tag_values))
    if len(set(tag_ids)) != len(tag_ids):
        raise _error(path, field + ".expected_tag_ids", "must be unique")
    return StairEndpoint(floor_id, _identifier(item["entry_location_id"], path, field + ".entry_location_id"), landing, covariance, tag_ids)


def _positive_integer(value: YamlValue, path: Path, field: str) -> int:
    if type(value) is not int or value <= 0:
        raise _error(path, field, "expected positive integer")
    return value


def _load_tags(root: Path) -> Tuple[AprilTag, ...]:
    path = root / "apriltags.yaml"
    result, seen = [], set()
    for index, value in enumerate(_document(path, "tags")):
        field = f"tags[{index}]"
        item = _mapping(value, path, field)
        _keys(item, {"id", "family", "size_m", "role", "floor_id", "stair_id"}, path, field)
        identifier = _positive_integer(item["id"], path, field + ".id")
        _unique(identifier, seen, path, field + ".id")
        size = _number(item["size_m"], path, field + ".size_m")
        if size <= 0:
            raise _error(path, field + ".size_m", "must be positive")
        result.append(AprilTag(identifier, _identifier(item["family"], path, field + ".family"), size, _identifier(item["role"], path, field + ".role"), _identifier(item["floor_id"], path, field + ".floor_id"), _identifier(item["stair_id"], path, field + ".stair_id")))
    return tuple(result)


def load_multifloor_configuration(root: Path) -> MultifloorConfiguration:
    """Parse all multifloor-owned YAML files into immutable values."""
    from multifloor_manager.transition_configuration import load_transitions

    return MultifloorConfiguration(_load_floors(root), _load_stairs(root), _load_tags(root), load_transitions(root))
