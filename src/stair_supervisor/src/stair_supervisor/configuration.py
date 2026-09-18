"""Typed stair policy and robot calibration loaded from strict YAML."""

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
class StairConfigurationError(Exception):
    __slots__ = ("file_name", "field", "detail")
    file_name: str
    field: str
    detail: str

    def __str__(self) -> str:
        separator = ":" if self.field else ""
        return f"{self.file_name}:{self.field}{separator} {self.detail}"


class Direction(str, Enum):
    UP = "UP"
    DOWN = "DOWN"


@dataclass(frozen=True)
class StairProfile:
    __slots__ = (
        "id", "direction", "enabled", "linear_speed", "angular_speed",
        "alignment_yaw_rad", "flight_1_distance_m", "landing_dwell_sec",
        "landing_turn_yaw_rad", "flight_2_distance_m", "exit_dwell_sec",
        "distance_tolerance_m", "yaw_tolerance_rad", "sensor_freshness_sec",
        "max_sample_gap_sec",
        "max_odom_step_m", "max_yaw_step_rad", "timeout_sec",
    )
    id: str
    direction: Direction
    enabled: bool
    linear_speed: float
    angular_speed: float
    alignment_yaw_rad: float
    flight_1_distance_m: float
    landing_dwell_sec: float
    landing_turn_yaw_rad: float
    flight_2_distance_m: float
    exit_dwell_sec: float
    distance_tolerance_m: float
    yaw_tolerance_rad: float
    sensor_freshness_sec: float
    max_sample_gap_sec: float
    max_odom_step_m: float
    max_yaw_step_rad: float
    timeout_sec: float


@dataclass(frozen=True)
class MoveBaseLimits:
    __slots__ = ("max_vel_x", "max_vel_theta", "min_in_place_vel_theta", "acc_lim_x", "acc_lim_theta")
    max_vel_x: float
    max_vel_theta: float
    min_in_place_vel_theta: float
    acc_lim_x: float
    acc_lim_theta: float


@dataclass(frozen=True)
class WebSocketCalibration:
    __slots__ = ("linear_mps", "angular_radps")
    linear_mps: float
    angular_radps: float


@dataclass(frozen=True)
class RobotConfiguration:
    __slots__ = ("move_base", "websocket_full_scale", "command_topics")
    move_base: MoveBaseLimits
    websocket_full_scale: WebSocketCalibration
    command_topics: Tuple[str, ...]


@dataclass(frozen=True)
class StairSupervisorConfiguration:
    __slots__ = ("profiles", "robot")
    profiles: Tuple[StairProfile, ...]
    robot: RobotConfiguration


def _error(path: Path, field: str, detail: str) -> StairConfigurationError:
    return StairConfigurationError(path.name, field, detail)


def _read(path: Path) -> Dict[str, YamlValue]:
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        detail = "YAML parse error" if isinstance(error, yaml.YAMLError) else str(error)
        raise _error(path, "", detail) from None
    return _mapping(raw, path, "")


def _header(mapping: Dict[str, YamlValue], keys: set, path: Path) -> bool:
    expected = keys | {"schema_version", "configured"}
    _keys(mapping, expected, path, "")
    if type(mapping["schema_version"]) is not int or mapping["schema_version"] != 1:
        raise _error(path, "schema_version", "expected integer 1")
    if type(mapping["configured"]) is not bool:
        raise _error(path, "configured", "expected boolean")
    return bool(mapping["configured"])


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


def _positive(mapping: Dict[str, YamlValue], key: str, path: Path, field: str) -> float:
    value = _number(mapping[key], path, field + "." + key)
    if value <= 0:
        raise _error(path, field + "." + key, "must be positive")
    return value


def _nonzero(mapping: Dict[str, YamlValue], key: str, path: Path, field: str) -> float:
    value = _number(mapping[key], path, field + "." + key)
    if value == 0.0:
        raise _error(path, field + "." + key, "must be nonzero")
    return value


def _load_profiles(root: Path) -> Tuple[StairProfile, ...]:
    path = root / "stair_profiles.yaml"
    document = _read(path)
    configured = _header(document, {"profiles"}, path)
    raw_profiles = _list(document["profiles"], path, "profiles")
    if not configured:
        if raw_profiles:
            raise _error(path, "profiles", "must be empty when configured is false")
        return ()
    result, seen = [], set()
    expected = {
        "id", "direction", "enabled", "linear_speed", "angular_speed",
        "alignment_yaw_rad", "flight_1_distance_m", "landing_dwell_sec",
        "landing_turn_yaw_rad", "flight_2_distance_m", "exit_dwell_sec",
        "distance_tolerance_m", "yaw_tolerance_rad", "sensor_freshness_sec",
        "max_sample_gap_sec",
        "max_odom_step_m", "max_yaw_step_rad", "timeout_sec",
    }
    for index, value in enumerate(raw_profiles):
        field = f"profiles[{index}]"
        item = _mapping(value, path, field)
        _keys(item, expected, path, field)
        identifier = _identifier(item["id"], path, field + ".id")
        if identifier in seen:
            raise _error(path, field + ".id", f"duplicate ID {identifier!r}")
        seen.add(identifier)
        try:
            direction = Direction(_string(item["direction"], path, field + ".direction"))
        except ValueError:
            raise _error(path, field + ".direction", "expected UP or DOWN") from None
        if type(item["enabled"]) is not bool:
            raise _error(path, field + ".enabled", "expected boolean")
        profile = StairProfile(
            identifier,
            direction,
            item["enabled"],
            _positive(item, "linear_speed", path, field),
            _positive(item, "angular_speed", path, field),
            _number(item["alignment_yaw_rad"], path, field + ".alignment_yaw_rad"),
            _nonzero(item, "flight_1_distance_m", path, field),
            _positive(item, "landing_dwell_sec", path, field),
            _nonzero(item, "landing_turn_yaw_rad", path, field),
            _nonzero(item, "flight_2_distance_m", path, field),
            _positive(item, "exit_dwell_sec", path, field),
            _positive(item, "distance_tolerance_m", path, field),
            _positive(item, "yaw_tolerance_rad", path, field),
            _positive(item, "sensor_freshness_sec", path, field),
            _positive(item, "max_sample_gap_sec", path, field),
            _positive(item, "max_odom_step_m", path, field),
            _positive(item, "max_yaw_step_rad", path, field),
            _positive(item, "timeout_sec", path, field),
        )
        if profile.max_sample_gap_sec > profile.sensor_freshness_sec:
            raise _error(path, field + ".max_sample_gap_sec", "must not exceed sensor_freshness_sec")
        if profile.alignment_yaw_rad != 0.0:
            raise _error(path, field + ".alignment_yaw_rad", "must be zero; entry heading is validated before stair ownership")
        result.append(profile)
    if not result:
        raise _error(path, "profiles", "must not be empty")
    return tuple(result)


def _load_robot(root: Path) -> RobotConfiguration:
    path = root / "robot.yaml"
    document = _read(path)
    configured = _header(document, {"move_base", "websocket_full_scale", "command_topics"}, path)
    if not configured:
        raise _error(path, "configured", "must be true before use")
    move_base = _mapping(document["move_base"], path, "move_base")
    move_keys = {"max_vel_x", "max_vel_theta", "min_in_place_vel_theta", "acc_lim_x", "acc_lim_theta"}
    _keys(move_base, move_keys, path, "move_base")
    websocket = _mapping(document["websocket_full_scale"], path, "websocket_full_scale")
    _keys(websocket, {"linear_mps", "angular_radps"}, path, "websocket_full_scale")
    topics_raw = _list(document["command_topics"], path, "command_topics")
    if not topics_raw:
        raise _error(path, "command_topics", "must not be empty")
    topics = tuple(_string(topic, path, f"command_topics[{index}]") for index, topic in enumerate(topics_raw))
    if len(set(topics)) != len(topics) or any(not topic.startswith("/") for topic in topics):
        raise _error(path, "command_topics", "topics must be unique absolute names")
    limits = MoveBaseLimits(*(_positive(move_base, key, path, "move_base") for key in ("max_vel_x", "max_vel_theta", "min_in_place_vel_theta", "acc_lim_x", "acc_lim_theta")))
    calibration = WebSocketCalibration(_positive(websocket, "linear_mps", path, "websocket_full_scale"), _positive(websocket, "angular_radps", path, "websocket_full_scale"))
    if limits.max_vel_x > calibration.linear_mps or limits.max_vel_theta > calibration.angular_radps:
        raise _error(path, "move_base", "limits must not exceed WebSocket full-scale calibration")
    return RobotConfiguration(limits, calibration, topics)


def load_stair_configuration(root: Path) -> StairSupervisorConfiguration:
    """Parse all stair-supervisor-owned YAML files into immutable values."""
    return StairSupervisorConfiguration(_load_profiles(root), _load_robot(root))
