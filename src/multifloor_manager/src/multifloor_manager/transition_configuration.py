"""Strict loader for the single floor-transition safety policy."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Tuple

from multifloor_manager.configuration import YamlValue, _document, _error, _identifier, _keys, _list, _mapping
from multifloor_manager.transitions import (
    FLOOR_TRANSITION_POLICY_ID,
    MANDATORY_CONDITION_NAMES,
    SUPPORTED_CONDITION_NAMES,
    TransitionCondition,
    TransitionPolicy,
)


def _bit(value: YamlValue, path: Path, field: str) -> bool:
    if type(value) is not int or value not in (0, 1):
        raise _error(path, field, "expected integer 0 or 1")
    return value == 1


def load_transitions(root: Path) -> Tuple[TransitionPolicy, ...]:
    """Parse the configured transition policy from its package root."""
    path = root / "transitions.yaml"
    values = _document(path, "transitions")
    if len(values) != 1:
        raise _error(path, "transitions", "must contain exactly one policy")
    field = "transitions[0]"
    item = _mapping(values[0], path, field)
    expected = {"id", "enabled", "conditions", "optional_count", "freshness_sec", "dwell_sec", "timeout_sec"}
    _keys(item, expected, path, field)
    identifier = _identifier(item["id"], path, field + ".id")
    if identifier != FLOOR_TRANSITION_POLICY_ID:
        raise _error(path, field + ".id", f"expected {FLOOR_TRANSITION_POLICY_ID!r}")
    enabled = _bit(item["enabled"], path, field + ".enabled")
    if not enabled:
        raise _error(path, field + ".enabled", "must be 1")
    conditions, seen = [], set()
    for index, value in enumerate(_list(item["conditions"], path, field + ".conditions")):
        condition_field = f"{field}.conditions[{index}]"
        condition = _mapping(value, path, condition_field)
        _keys(condition, {"name", "enabled", "required"}, path, condition_field)
        name = _identifier(condition["name"], path, condition_field + ".name")
        if name not in SUPPORTED_CONDITION_NAMES:
            raise _error(path, condition_field + ".name", f"unknown condition {name!r}")
        if name in seen:
            raise _error(path, condition_field + ".name", f"duplicate condition {name!r}")
        seen.add(name)
        condition_enabled = _bit(condition["enabled"], path, condition_field + ".enabled")
        condition_required = _bit(condition["required"], path, condition_field + ".required")
        if not condition_enabled:
            raise _error(path, condition_field + ".enabled", "mandatory condition must be 1")
        if not condition_required:
            raise _error(path, condition_field + ".required", "mandatory condition must be 1")
        conditions.append(TransitionCondition(name, condition_enabled, condition_required))
    for mandatory_name in MANDATORY_CONDITION_NAMES:
        if mandatory_name not in seen:
            raise _error(path, field + ".conditions", f"missing mandatory condition {mandatory_name!r}")
    optional_count = item["optional_count"]
    if type(optional_count) is not int or optional_count != 0:
        raise _error(path, field + ".optional_count", "expected integer 0")
    freshness = item["freshness_sec"]
    timeout = item["timeout_sec"]
    dwell = item["dwell_sec"]
    if type(freshness) not in (int, float) or not math.isfinite(freshness) or freshness <= 0:
        raise _error(path, field + ".freshness_sec", "must be finite and positive")
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
        raise _error(path, field + ".timeout_sec", "must be finite and positive")
    if type(dwell) not in (int, float) or not math.isfinite(dwell) or dwell < 0:
        raise _error(path, field + ".dwell_sec", "must be finite and nonnegative")
    if dwell >= timeout:
        raise _error(path, field + ".dwell_sec", "must be less than timeout_sec")
    return (TransitionPolicy(identifier, enabled, tuple(conditions), optional_count, float(freshness), float(dwell), float(timeout)),)
