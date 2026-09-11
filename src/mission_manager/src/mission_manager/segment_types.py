"""Canonical mission route segment types shared by planning and execution."""

from enum import Enum, unique


@unique
class SegmentType(str, Enum):
    NAVIGATION = "NAVIGATION"
    STAIR = "STAIR"
    FLOOR_TRANSITION = "FLOOR_TRANSITION"
    SCAN = "SCAN"
