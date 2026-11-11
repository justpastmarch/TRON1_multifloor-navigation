"""ROS-independent bookkeeping for one floor transaction and its failure.

A failed transaction keeps ``FloorState.floor_id`` at the last confirmed floor.
The map that ``map_server`` actually holds, the stage that failed and whether
the target floor had already been confirmed are recorded separately, so an
operator or client can see the real state and the same transition can resume
from the first unfinished stage instead of starting over.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional

MAP_UNKNOWN = "UNKNOWN"

# Resume entry points, in transaction order.
RESUME_CHANGE_MAP = "CHANGE_MAP"
RESUME_INITIALPOSE = "INITIALPOSE"


@dataclass(frozen=True)
class TransactionProgress:
    """What one transaction has done so far; replaced, never mutated."""

    transition_id: str
    source_floor: str
    target_floor: str
    stage: str = "ARM_TARGET"
    # Stair result or tag vote for the target was consumed in this transaction.
    floor_confirmed: bool = False
    # Floor whose map map_server holds; MAP_UNKNOWN after an ambiguous RPC.
    map_floor: str = ""
    # A /map publication with that floor's identity was observed.
    map_observed: bool = True

    def at(self, stage: str) -> "TransactionProgress":
        return replace(self, stage=stage)

    def with_map(self, floor: str, observed: bool) -> "TransactionProgress":
        return replace(self, map_floor=floor, map_observed=observed)

    def describe(self, reason: str) -> str:
        if self.map_floor == MAP_UNKNOWN:
            map_text = "unknown (change_map outcome not reported)"
        else:
            map_text = self.map_floor + ("" if self.map_observed else " (requested, not yet observed)")
        return "{} [stage={}; map={}; confirmed_floor={}; target={}; target_confirmed={}]".format(
            reason, self.stage, map_text, self.source_floor, self.target_floor,
            "yes" if self.floor_confirmed else "no")


@dataclass(frozen=True)
class TransitionFault:
    """Terminal failure of one transaction, bound to the map generation it left."""

    progress: TransactionProgress
    map_generation: int
    failed_at: float
    reason: str

    def as_dict(self) -> dict:
        p = self.progress
        return dict(
            transition_id=p.transition_id, source_floor=p.source_floor,
            target_floor=p.target_floor, stage=p.stage,
            target_confirmed=p.floor_confirmed, map_floor=p.map_floor,
            map_observed=p.map_observed, map_generation=self.map_generation,
            failed_at=self.failed_at, reason=self.reason,
        )


def resume_stage(
    fault: Optional[TransitionFault],
    *,
    transition_id: str,
    target_floor: str,
    in_fault: bool,
    current_floor: str,
    map_generation: int,
    observed_map_floor: str,
    now: float,
    max_age_sec: float,
) -> Optional[str]:
    """Return where a repeated goal may continue, or None to require READY.

    Resuming is allowed only for the same directed transition, while the floor
    manager is still in the FAULT that transaction produced (no floor selection
    or other map change since), after the target floor was already confirmed,
    and within ``max_age_sec`` so the stored landing pose is still plausible.
    """
    if fault is None or not in_fault:
        return None
    p = fault.progress
    if (p.transition_id, p.target_floor, p.source_floor) != (transition_id, target_floor, current_floor):
        return None
    if not p.floor_confirmed or int(map_generation) != fault.map_generation:
        return None
    if not 0.0 <= now - fault.failed_at <= max_age_sec:
        return None
    return RESUME_INITIALPOSE if observed_map_floor == target_floor else RESUME_CHANGE_MAP
