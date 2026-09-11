"""Whole-site configuration composition and cross-reference validation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, List, Tuple, Union

import yaml

from mission_manager.configuration import (
    EdgeType,
    GraphEdge,
    Location,
    MissionConfigurationError,
    ScanProfile,
    load_mission_configuration,
)
from multifloor_manager.configuration import (
    AprilTag,
    Floor,
    MultifloorConfigurationError,
    Stair,
    load_multifloor_configuration,
)
from multifloor_manager.transitions import TransitionPolicy
from stair_supervisor.configuration import (
    Direction,
    RobotConfiguration,
    StairConfigurationError,
    StairProfile,
    load_stair_configuration,
)


YamlScalar = Union[str, int, float, bool, None]
YamlValue = Union[YamlScalar, List["YamlValue"], Dict[str, "YamlValue"]]


@dataclass(frozen=True)
class ConfigurationError(Exception):
    __slots__ = ("file_name", "field", "detail")
    file_name: str
    field: str
    detail: str

    def __str__(self) -> str:
        separator = ":" if self.field else ""
        return f"{self.file_name}:{self.field}{separator} {self.detail}"


@dataclass(frozen=True)
class ConfigurationRoots:
    __slots__ = ("mission", "multifloor", "stair")
    mission: Path
    multifloor: Path
    stair: Path


@dataclass(frozen=True)
class SiteConfiguration:
    __slots__ = ("locations", "edges", "scan_profiles", "floors", "stairs", "tags", "transitions", "stair_profiles", "robot")
    locations: Tuple[Location, ...]
    edges: Tuple[GraphEdge, ...]
    scan_profiles: Tuple[ScanProfile, ...]
    floors: Tuple[Floor, ...]
    stairs: Tuple[Stair, ...]
    tags: Tuple[AprilTag, ...]
    transitions: Tuple[TransitionPolicy, ...]
    stair_profiles: Tuple[StairProfile, ...]
    robot: RobotConfiguration


def _fail(file_name: str, field: str, detail: str) -> None:
    raise ConfigurationError(file_name, field, detail)


def _load_map(root: Path, floor: Floor, index: int) -> None:
    field = f"floors[{index}].map_yaml"
    map_path = root / floor.map_yaml
    try:
        raw = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError):
        _fail("floors.yaml", field, f"map YAML does not exist or is unreadable: {floor.map_yaml}")
    except yaml.YAMLError:
        _fail("floors.yaml", field, f"map YAML is malformed: {floor.map_yaml}")
    if type(raw) is not dict:
        _fail("floors.yaml", field, "map YAML must be a mapping")
    expected = {"image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"}
    if set(raw) != expected:
        _fail("floors.yaml", field, f"map keys must be {sorted(expected)}")
    image = raw["image"]
    if type(image) is not str or not image or Path(image).name != image:
        _fail("floors.yaml", field, "map image must be a sibling filename")
    image_path = map_path.parent / image
    if not image_path.is_file():
        _fail("floors.yaml", field, f"map image does not exist: {image}")
    try:
        magic = image_path.read_bytes()[:2]
    except OSError:
        _fail("floors.yaml", field, f"map image is unreadable: {image}")
    if magic not in (b"P2", b"P5"):
        _fail("floors.yaml", field, "map image must be a PGM file")
    resolution = raw["resolution"]
    if type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0:
        _fail("floors.yaml", field, "map resolution must be positive")
    origin = raw["origin"]
    if type(origin) is not list or len(origin) != 3 or any(type(value) not in (int, float) or not math.isfinite(value) for value in origin):
        _fail("floors.yaml", field, "map origin must contain three finite numbers")
    if not -math.pi <= origin[2] <= math.pi:
        _fail("floors.yaml", field, "map origin yaw must be within [-pi, pi]")
    negate, occupied, free = raw["negate"], raw["occupied_thresh"], raw["free_thresh"]
    if type(negate) is not int or negate not in (0, 1):
        _fail("floors.yaml", field, "map negate must be integer 0 or 1")
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in (occupied, free)) or not 0 <= free < occupied <= 1:
        _fail("floors.yaml", field, "map thresholds must satisfy 0 <= free < occupied <= 1")


def _validate_floor_references(configuration: SiteConfiguration, root: Path) -> None:
    floors = {floor.id: floor for floor in configuration.floors}
    for index, floor in enumerate(configuration.floors):
        _load_map(root, floor, index)
    for index, location in enumerate(configuration.locations):
        if location.floor_id not in floors:
            _fail("locations.yaml", f"locations[{index}].floor_id", f"unknown floor {location.floor_id!r}")
    for index, tag in enumerate(configuration.tags):
        if tag.floor_id not in floors:
            _fail("apriltags.yaml", f"tags[{index}].floor_id", f"unknown floor {tag.floor_id!r}")


def _validate_stairs(configuration: SiteConfiguration) -> None:
    floor_ids = {floor.id for floor in configuration.floors}
    locations = {location.id: location for location in configuration.locations}
    tags = {tag.id: tag for tag in configuration.tags}
    profiles = {profile.id: profile for profile in configuration.stair_profiles}
    for index, stair in enumerate(configuration.stairs):
        field = f"stairs[{index}]"
        if stair.from_floor not in floor_ids or stair.to_floor not in floor_ids or stair.from_floor == stair.to_floor:
            _fail("stairs.yaml", field, "from/to floors must be distinct known floors")
        for endpoint in stair.endpoints:
            endpoint_field = f"{field}.endpoints.{endpoint.floor_id}"
            entry = locations.get(endpoint.entry_location_id)
            if entry is None:
                _fail("stairs.yaml", endpoint_field + ".entry_location_id", "unknown location")
            if entry.floor_id != endpoint.floor_id:
                _fail("stairs.yaml", endpoint_field + ".entry_location_id", f"location floor {entry.floor_id!r} does not match endpoint floor {endpoint.floor_id!r}")
            for tag_index, tag_id in enumerate(endpoint.expected_tag_ids):
                tag = tags.get(tag_id)
                tag_field = f"{endpoint_field}.expected_tag_ids[{tag_index}]"
                if tag is None:
                    _fail("stairs.yaml", tag_field, f"unknown tag {tag_id}")
                if tag.stair_id != stair.id or tag.floor_id != endpoint.floor_id:
                    _fail("stairs.yaml", tag_field, "tag stair/floor does not match endpoint")
        _validate_profile_reference(profiles, stair.up_profile_id, Direction.UP, field + ".up_profile_id")
        _validate_profile_reference(profiles, stair.down_profile_id, Direction.DOWN, field + ".down_profile_id")
    stair_ids = {stair.id for stair in configuration.stairs}
    for index, tag in enumerate(configuration.tags):
        if tag.stair_id not in stair_ids:
            _fail("apriltags.yaml", f"tags[{index}].stair_id", f"unknown stair {tag.stair_id!r}")


def _validate_profile_reference(profiles: Dict[str, StairProfile], profile_id: str, direction: Direction, field: str) -> None:
    profile = profiles.get(profile_id)
    if profile is None:
        _fail("stairs.yaml", field, f"unknown profile {profile_id!r}")
    if profile.direction is not direction:
        _fail("stairs.yaml", field, f"profile direction must be {direction.value}")


def _validate_graph(configuration: SiteConfiguration) -> None:
    locations = {location.id: location for location in configuration.locations}
    stairs = {stair.id: stair for stair in configuration.stairs}
    for index, edge in enumerate(configuration.edges):
        field = f"edges[{index}]"
        source = locations.get(edge.source_id)
        target = locations.get(edge.target_id)
        if source is None:
            _fail("building_graph.yaml", field + ".from", f"unknown location {edge.source_id!r}")
        if target is None:
            _fail("building_graph.yaml", field + ".to", f"unknown location {edge.target_id!r}")
        if edge.type is EdgeType.NAV:
            if source.floor_id != target.floor_id:
                _fail("building_graph.yaml", field, "NAV endpoints must share a floor")
            continue
        stair = stairs.get(edge.stair_id)
        if stair is None:
            _fail("building_graph.yaml", field + ".stair_id", f"unknown stair {edge.stair_id!r}")
        expected = (stair.from_floor, stair.to_floor) if edge.type is EdgeType.STAIR_UP else (stair.to_floor, stair.from_floor)
        if (source.floor_id, target.floor_id) != expected:
            _fail("building_graph.yaml", field, f"stair edge floor continuity must be {expected[0]} -> {expected[1]}")
        source_endpoint = stair.endpoint_for(source.floor_id)
        target_endpoint = stair.endpoint_for(target.floor_id)
        if source_endpoint is None or target_endpoint is None:
            _fail("building_graph.yaml", field, "stair edge requires source and target floor endpoints")
        if source.id != source_endpoint.entry_location_id:
            _fail("building_graph.yaml", field + ".from", f"must match {source.floor_id} stair entry {source_endpoint.entry_location_id!r}")


def load_site_configuration_from_roots(roots: ConfigurationRoots) -> SiteConfiguration:
    """Parse and cross-check package-owned configuration roots deterministically."""
    resolved_roots = ConfigurationRoots(roots.mission.resolve(), roots.multifloor.resolve(), roots.stair.resolve())
    try:
        mission = load_mission_configuration(resolved_roots.mission)
        multifloor = load_multifloor_configuration(resolved_roots.multifloor)
        stair = load_stair_configuration(resolved_roots.stair)
    except (MissionConfigurationError, MultifloorConfigurationError, StairConfigurationError) as error:
        raise ConfigurationError(error.file_name, error.field, error.detail) from None
    configuration = SiteConfiguration(mission.locations, mission.edges, mission.scan_profiles, multifloor.floors, multifloor.stairs, multifloor.tags, multifloor.transitions, stair.profiles, stair.robot)
    _validate_floor_references(configuration, resolved_roots.multifloor)
    _validate_stairs(configuration)
    _validate_graph(configuration)
    return configuration


def load_site_configuration(root: Path) -> SiteConfiguration:
    """Parse a colocated fixture or deployed configuration directory."""
    return load_site_configuration_from_roots(ConfigurationRoots(root, root, root))
