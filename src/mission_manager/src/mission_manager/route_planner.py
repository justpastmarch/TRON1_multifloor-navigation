"""Deterministic directed route planning over validated building configuration."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from typing import Dict, List, Tuple, Union

from mission_manager.configuration import EdgeType, GraphEdge, Location
from mission_manager.segment_types import SegmentType
from mission_manager.site_config import SiteConfiguration
from multifloor_manager.configuration import Stair
from stair_supervisor.configuration import Direction, StairProfile


@dataclass(frozen=True)
class RouteValidationError(Exception):
    __slots__ = ("detail",)
    detail: str

    def __str__(self) -> str:
        return self.detail


@dataclass(frozen=True)
class RoutePlanningError(Exception):
    __slots__ = ("origin_id", "destination_id", "detail")
    origin_id: str
    destination_id: str
    detail: str

    def __str__(self) -> str:
        return f"{self.origin_id} -> {self.destination_id}: {self.detail}"


@dataclass(frozen=True)
class AnchorCommitError(Exception):
    __slots__ = ("anchor_id", "segment_source_id")
    anchor_id: str
    segment_source_id: str

    def __str__(self) -> str:
        return f"anchor {self.anchor_id!r} does not match segment source {self.segment_source_id!r}"


@dataclass(frozen=True)
class RouteSegment:
    __slots__ = (
        "type",
        "source_id",
        "target_id",
        "source_floor",
        "target_floor",
        "edge_id",
        "stair_id",
        "profile_id",
        "scan_profile_id",
        "confirmed_location_id",
    )
    type: SegmentType
    source_id: str
    target_id: str
    source_floor: str
    target_floor: str
    edge_id: Union[str, None]
    stair_id: Union[str, None]
    profile_id: Union[str, None]
    scan_profile_id: Union[str, None]
    confirmed_location_id: str


@dataclass(frozen=True)
class Route:
    __slots__ = ("origin_id", "destination_id", "segments")
    origin_id: str
    destination_id: str
    segments: Tuple[RouteSegment, ...]

    def to_bytes(self) -> bytes:
        """Serialize the route canonically for logs, tests, and handoff boundaries."""
        payload = {
            "destination_id": self.destination_id,
            "origin_id": self.origin_id,
            "segments": [
                {
                    "confirmed_location_id": segment.confirmed_location_id,
                    "edge_id": segment.edge_id,
                    "profile_id": segment.profile_id,
                    "scan_profile_id": segment.scan_profile_id,
                    "source_floor": segment.source_floor,
                    "source_id": segment.source_id,
                    "stair_id": segment.stair_id,
                    "target_floor": segment.target_floor,
                    "target_id": segment.target_id,
                    "type": segment.type.value,
                }
                for segment in self.segments
            ],
        }
        return json.dumps(payload, ensure_ascii=True, separators=(",", ":"), sort_keys=True).encode("utf-8")


@dataclass(frozen=True)
class LogicalAnchor:
    __slots__ = ("location_id",)
    location_id: str

    def after_segment(self, segment: RouteSegment, succeeded: bool) -> LogicalAnchor:
        """Return a new anchor, committing segment progress only after success."""
        if not succeeded:
            return self
        if self.location_id != segment.source_id:
            raise AnchorCommitError(self.location_id, segment.source_id)
        return LogicalAnchor(segment.confirmed_location_id)


class BuildingPlanner:
    """Plan shortest directed routes with deterministic lexical edge tie-breaking."""

    def __init__(self, configuration: SiteConfiguration, home_location_id: str) -> None:
        self._locations: Dict[str, Location] = {location.id: location for location in configuration.locations}
        self._stairs: Dict[str, Stair] = {stair.id: stair for stair in configuration.stairs}
        self._profiles: Dict[str, StairProfile] = {profile.id: profile for profile in configuration.stair_profiles}
        self._scan_profile_ids = {profile.id for profile in configuration.scan_profiles}
        self._home_location_id = home_location_id
        self._validate_unique_ids(configuration)
        self._validate_home()
        self._validate_edges(configuration.edges)
        adjacency: Dict[str, List[GraphEdge]] = {location_id: [] for location_id in self._locations}
        for edge in configuration.edges:
            if self._edge_is_enabled(edge):
                adjacency[edge.source_id].append(edge)
        self._adjacency = {
            location_id: tuple(sorted(edges, key=lambda edge: edge.id))
            for location_id, edges in adjacency.items()
        }

    def plan(
        self,
        origin_id: str,
        destination_id: str,
        scan_profile_id: Union[str, None] = None,
    ) -> Route:
        """Plan and expand one directed route between named locations."""
        if origin_id not in self._locations:
            raise RoutePlanningError(origin_id, destination_id, f"unknown location {origin_id!r}")
        destination = self._locations.get(destination_id)
        if destination is None:
            raise RoutePlanningError(origin_id, destination_id, f"unknown location {destination_id!r}")
        if scan_profile_id is not None and scan_profile_id not in self._scan_profile_ids:
            raise RoutePlanningError(origin_id, destination_id, f"unknown scan profile {scan_profile_id!r}")
        edges = self._bfs(origin_id, destination_id)
        segments: List[RouteSegment] = []
        for edge in edges:
            segments.extend(self._expand_edge(edge))
        if scan_profile_id is not None:
            segments.append(
                RouteSegment(
                    SegmentType.SCAN,
                    destination.id,
                    destination.id,
                    destination.floor_id,
                    destination.floor_id,
                    None,
                    None,
                    None,
                    scan_profile_id,
                    destination.id,
                )
            )
        self._validate_segments(origin_id, destination_id, segments)
        return Route(origin_id, destination_id, tuple(segments))

    def plan_return(self, confirmed_anchor: LogicalAnchor) -> Route:
        """Run a fresh BFS from confirmed progress to the configured home."""
        return self.plan(confirmed_anchor.location_id, self._home_location_id)

    def _bfs(self, origin_id: str, destination_id: str) -> Tuple[GraphEdge, ...]:
        queue = deque([origin_id])
        predecessor: Dict[str, GraphEdge] = {}
        visited = {origin_id}
        while queue:
            source_id = queue.popleft()
            if source_id == destination_id:
                break
            for edge in self._adjacency[source_id]:
                if edge.target_id in visited:
                    continue
                visited.add(edge.target_id)
                predecessor[edge.target_id] = edge
                queue.append(edge.target_id)
        if destination_id not in visited:
            raise RoutePlanningError(origin_id, destination_id, "no directed route")
        route: List[GraphEdge] = []
        cursor = destination_id
        while cursor != origin_id:
            edge = predecessor[cursor]
            route.append(edge)
            cursor = edge.source_id
        route.reverse()
        return tuple(route)

    def _expand_edge(self, edge: GraphEdge) -> Tuple[RouteSegment, ...]:
        source = self._locations[edge.source_id]
        target = self._locations[edge.target_id]
        if edge.type is EdgeType.NAV:
            return (RouteSegment(SegmentType.NAVIGATION, source.id, target.id, source.floor_id, target.floor_id, edge.id, None, None, None, target.id),)
        stair = self._stairs[edge.stair_id]
        profile = self._profile_for(edge, stair)
        return (
            RouteSegment(SegmentType.STAIR, source.id, target.id, source.floor_id, target.floor_id, edge.id, stair.id, profile.id, None, source.id),
            RouteSegment(SegmentType.FLOOR_TRANSITION, source.id, target.id, source.floor_id, target.floor_id, edge.id, stair.id, None, None, target.id),
        )

    def _profile_for(self, edge: GraphEdge, stair: Stair) -> StairProfile:
        if edge.type is EdgeType.STAIR_UP:
            return self._profiles[stair.up_profile_id]
        if edge.type is EdgeType.STAIR_DOWN:
            return self._profiles[stair.down_profile_id]
        raise RouteValidationError(f"edge {edge.id!r} is not a stair edge")

    def _edge_is_enabled(self, edge: GraphEdge) -> bool:
        if edge.type is EdgeType.NAV:
            return True
        stair = self._stairs[edge.stair_id]
        profile_id = stair.up_profile_id if edge.type is EdgeType.STAIR_UP else stair.down_profile_id
        profile = self._profiles.get(profile_id)
        expected = Direction.UP if edge.type is EdgeType.STAIR_UP else Direction.DOWN
        return profile is not None and profile.enabled and profile.direction is expected

    def _validate_unique_ids(self, configuration: SiteConfiguration) -> None:
        groups = (
            ("location", tuple(location.id for location in configuration.locations)),
            ("edge", tuple(edge.id for edge in configuration.edges)),
            ("stair", tuple(stair.id for stair in configuration.stairs)),
            ("profile", tuple(profile.id for profile in configuration.stair_profiles)),
        )
        for label, identifiers in groups:
            if len(identifiers) != len(set(identifiers)):
                raise RouteValidationError(f"duplicate {label} ID")

    def _validate_home(self) -> None:
        if self._home_location_id not in self._locations:
            raise RouteValidationError(f"unknown home location {self._home_location_id!r}")

    def _validate_edges(self, edges: Tuple[GraphEdge, ...]) -> None:
        for edge in edges:
            source = self._locations.get(edge.source_id)
            target = self._locations.get(edge.target_id)
            if source is None or target is None:
                raise RouteValidationError(f"edge {edge.id!r} has an unknown endpoint")
            if edge.type is EdgeType.NAV:
                if source.floor_id != target.floor_id:
                    raise RouteValidationError(f"edge {edge.id!r} violates NAV floor continuity")
                continue
            stair = self._stairs.get(edge.stair_id)
            if stair is None:
                raise RouteValidationError(f"edge {edge.id!r} references an unknown stair")
            expected = (stair.from_floor, stair.to_floor) if edge.type is EdgeType.STAIR_UP else (stair.to_floor, stair.from_floor)
            if (source.floor_id, target.floor_id) != expected:
                raise RouteValidationError(f"edge {edge.id!r} violates stair floor continuity")
            source_endpoint = stair.endpoint_for(source.floor_id)
            target_endpoint = stair.endpoint_for(target.floor_id)
            if source_endpoint is None or target_endpoint is None:
                raise RouteValidationError(f"edge {edge.id!r} requires source and target floor endpoints")
            if source.id != source_endpoint.entry_location_id:
                raise RouteValidationError(f"edge {edge.id!r} source does not match the stair entry endpoint")

    def _validate_segments(self, origin_id: str, destination_id: str, segments: List[RouteSegment]) -> None:
        anchor = LogicalAnchor(origin_id)
        for segment in segments:
            anchor = anchor.after_segment(segment, succeeded=True)
        if anchor.location_id != destination_id:
            raise RouteValidationError(f"expanded route ends at {anchor.location_id!r}, expected {destination_id!r}")
