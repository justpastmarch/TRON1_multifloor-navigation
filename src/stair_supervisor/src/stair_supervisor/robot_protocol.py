"""Strict JSON boundary for the documented TRON1 high-level protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from typing import Dict, List, Mapping, Union


JsonScalar = Union[str, int, float, bool, None]
JsonValue = Union[JsonScalar, List["JsonValue"], Dict[str, "JsonValue"]]


class RequestTitle(str, Enum):
    TWIST = "request_twist"
    STAND_MODE = "request_stand_mode"
    WALK_MODE = "request_walk_mode"
    STAIR_MODE = "request_stair_mode"
    EMERGENCY_STOP = "request_emgy_stop"
    ENABLE_ODOMETRY = "request_enable_odom"
    ENABLE_IMU = "request_enable_imu"


class RobotStatus(str, Enum):
    STAND = "STAND"
    WALK = "WALK"
    SIT = "SIT"
    DAMPING = "DAMPING"
    ROTATE = "ROTATE"
    STAIR = "STAIR"
    ERROR_FALLOVER = "ERROR_FALLOVER"
    RECOVER = "RECOVER"
    ERROR_RECOVER = "ERROR_RECOVER"


@dataclass(frozen=True)
class ProtocolError(Exception):
    __slots__ = ("detail",)
    detail: str

    def __str__(self) -> str:
        return self.detail


@dataclass(frozen=True)
class RequestEnvelope:
    __slots__ = ("accid", "title", "timestamp_ms", "guid", "data")
    accid: str
    title: RequestTitle
    timestamp_ms: int
    guid: str
    data: Mapping[str, JsonValue]


@dataclass(frozen=True)
class RobotMessage:
    __slots__ = ("accid", "title", "timestamp_ms", "guid", "data")
    accid: str
    title: str
    timestamp_ms: int
    guid: str
    data: Dict[str, JsonValue]


def encode_request(request: RequestEnvelope) -> str:
    """Serialize one request using exactly the documented five-field envelope."""
    return json.dumps(
        {
            "accid": request.accid,
            "title": request.title.value,
            "timestamp": request.timestamp_ms,
            "guid": request.guid,
            "data": request.data,
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )


def _is_json_value(value) -> bool:
    value_type = type(value)
    if value is None or value_type in (str, int, bool):
        return True
    if value_type is float:
        return math.isfinite(value)
    if value_type is list:
        return all(_is_json_value(item) for item in value)
    if value_type is dict:
        return all(type(key) is str and _is_json_value(item) for key, item in value.items())
    return False


def parse_message(text: str | bytes) -> RobotMessage:
    """Parse an untrusted robot frame into a complete typed message."""
    try:
        value = json.loads(text)
    except (TypeError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ProtocolError("robot frame is not valid JSON") from error
    if type(value) is not dict:
        raise ProtocolError("robot frame must be a JSON object")
    expected = {"accid", "title", "timestamp", "guid", "data"}
    if set(value) != expected:
        raise ProtocolError("robot frame must contain exactly the five envelope fields")
    accid = value["accid"]
    title = value["title"]
    timestamp = value["timestamp"]
    guid = value["guid"]
    data = value["data"]
    if type(accid) is not str or not accid:
        raise ProtocolError("robot frame accid must be a nonempty string")
    if type(title) is not str or not title:
        raise ProtocolError("robot frame title must be a nonempty string")
    if (
        type(timestamp) not in (int, float)
        or not math.isfinite(timestamp)
        or timestamp < 0
    ):
        raise ProtocolError("robot frame timestamp must be a nonnegative finite number")
    timestamp_ms = int(timestamp)
    if type(guid) is not str or not guid:
        raise ProtocolError("robot frame guid must be a nonempty string")
    if type(data) is not dict:
        raise ProtocolError("robot frame data must be an object")
    if not _is_json_value(data):
        raise ProtocolError("robot frame data contains an invalid JSON value")
    if title.startswith("response_") and (
        type(data.get("result")) is not str or not data["result"]
    ):
        raise ProtocolError("robot response data must contain a nonempty result")
    return RobotMessage(accid, title, timestamp_ms, guid, data)
