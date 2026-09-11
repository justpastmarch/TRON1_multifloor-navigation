"""Pure physical-to-normalized TRON1 twist conversion."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .configuration import WebSocketCalibration


@dataclass(frozen=True)
class NormalizedTwist:
    """Documented unitless TRON1 walking command in the range [-1, 1]."""

    __slots__ = ("x", "y", "z")
    x: float
    y: float
    z: float

    @classmethod
    def zero(cls) -> "NormalizedTwist":
        return cls(0.0, 0.0, 0.0)


def normalize_twist(
    linear_mps: float,
    angular_radps: float,
    calibration: WebSocketCalibration,
) -> NormalizedTwist:
    """Convert SI motion into a clipped command, failing closed on nonfinite input."""
    if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
        return NormalizedTwist.zero()
    return NormalizedTwist(
        x=max(-1.0, min(1.0, linear_mps / calibration.linear_mps)),
        y=0.0,
        z=max(-1.0, min(1.0, angular_radps / calibration.angular_radps)),
    )
