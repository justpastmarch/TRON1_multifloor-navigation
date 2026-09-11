"""Immutable value types returned by test fixture loaders."""

from typing import Dict, List, NamedTuple, Tuple, Union


JsonValue = Union[None, bool, int, float, str, List["JsonValue"], Dict[str, "JsonValue"]]


class DirectedEdge(NamedTuple):
    source: str
    target: str
    cost: int


class DirectedGraph(NamedTuple):
    floors: Tuple[str, ...]
    edges: Tuple[DirectedEdge, ...]


class FrozenClock(NamedTuple):
    now_ns: int
    freshness_limit_ns: int


class TagInput(NamedTuple):
    id: int
    frame_id: str
    stamp_ns: int


class AmclInput(NamedTuple):
    frame_id: str
    x: float
    y: float
    yaw: float
    covariance: Tuple[float, ...]
    stamp_ns: int


class TfInput(NamedTuple):
    parent: str
    child: str
    stamp_ns: int


class LocalizationCases(NamedTuple):
    tag_valid: TagInput
    tag_invalid: TagInput
    amcl_valid: AmclInput
    amcl_invalid: AmclInput
    tf_valid: TfInput
    tf_invalid: TfInput
    tf_stale: TfInput


class WebSocketFrame(NamedTuple):
    at_ns: int
    direction: str
    payload_json: str


class FixtureSummary(NamedTuple):
    floors: Tuple[str, ...]
    map_names: Tuple[str, ...]
    localization_case_count: int
    websocket_frame_count: int
    now_ns: int
