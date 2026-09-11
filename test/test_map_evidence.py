from __future__ import annotations

from pathlib import Path
import sys
from typing import NamedTuple, Tuple
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "multifloor_manager" / "src"))

from multifloor_manager.map_evidence import (  # noqa: E402
    MapBoundaryError,
    MapGeneration,
    MapGenerationState,
    Nanoseconds,
    arm_map_generation,
    evaluate_map_observation,
    fingerprint_occupancy_grid,
)


ARMED_AT = Nanoseconds(1_700_000_010_000_000_000)


class FakeStamp(NamedTuple):
    nanoseconds: int

    def to_nsec(self) -> int:
        return self.nanoseconds


class FakeHeader(NamedTuple):
    frame_id: str


class FakeVector(NamedTuple):
    x: float
    y: float
    z: float


class FakeQuaternion(NamedTuple):
    x: float
    y: float
    z: float
    w: float


class FakePose(NamedTuple):
    position: FakeVector
    orientation: FakeQuaternion


class FakeMapInfo(NamedTuple):
    map_load_time: FakeStamp
    resolution: float
    width: int
    height: int
    origin: FakePose


class FakeOccupancyGrid(NamedTuple):
    header: FakeHeader
    info: FakeMapInfo
    data: Tuple[int, ...]


def grid(load_time_ns: int, data: Tuple[int, ...] = (0, 0, 100, -1)) -> FakeOccupancyGrid:
    return FakeOccupancyGrid(
        FakeHeader("map"),
        FakeMapInfo(
            FakeStamp(load_time_ns),
            0.05,
            2,
            2,
            FakePose(FakeVector(-1.0, 2.0, 0.0), FakeQuaternion(0.0, 0.0, 0.0, 1.0)),
        ),
        data,
    )


def fingerprint(message: FakeOccupancyGrid, received_at_ns: int):
    return fingerprint_occupancy_grid(message, Nanoseconds(received_at_ns))


class MapEvidenceTest(unittest.TestCase):
    def test_valid_target_fingerprint_advances_generation_once(self) -> None:
        # Given: generation seven, an armed target identity, and a post-arm map.
        current = fingerprint(grid(int(ARMED_AT) - 2), int(ARMED_AT) - 1)
        target = fingerprint(grid(int(ARMED_AT) + 1, (0, 100, 100, -1)), int(ARMED_AT) + 2)
        state = MapGenerationState(MapGeneration(7), current)
        guard = arm_map_generation(state, ARMED_AT, target.identity())

        # When: the expected post-arm fingerprint is observed twice.
        accepted = evaluate_map_observation(guard, target)
        duplicate = evaluate_map_observation(accepted.guard, target)

        # Then: the generation is monotonic and this arm can increment only once.
        self.assertEqual(accepted.guard.state.generation, MapGeneration(8))
        self.assertTrue(accepted.evidence.accepted)
        self.assertTrue(all(predicate.value for predicate in accepted.evidence.predicates()))
        self.assertEqual(duplicate.guard.state.generation, MapGeneration(8))
        self.assertFalse(duplicate.evidence.accepted)

    def test_pre_arm_target_map_is_ignored_until_new_generation_arrives(self) -> None:
        # Given: a target map loaded before the guard was armed.
        current = fingerprint(grid(int(ARMED_AT) - 3), int(ARMED_AT) - 2)
        pre_arm = fingerprint(
            grid(int(ARMED_AT) - 1, (100, 0, 100, -1)),
            int(ARMED_AT) + 1,
        )
        state = MapGenerationState(MapGeneration(3), current)
        guard = arm_map_generation(state, ARMED_AT, pre_arm.identity())

        # When: the latched old map arrives, followed by a newly loaded target map.
        stale = evaluate_map_observation(guard, pre_arm)
        post_arm = fingerprint(
            grid(int(ARMED_AT) + 2, (100, 0, 100, -1)),
            int(ARMED_AT) + 3,
        )
        accepted = evaluate_map_observation(stale.guard, post_arm)

        # Then: only the post-arm generation advances state.
        self.assertFalse(stale.evidence.post_arm)
        self.assertEqual(stale.guard.state.generation, MapGeneration(3))
        self.assertTrue(accepted.evidence.accepted)
        self.assertEqual(accepted.guard.state.generation, MapGeneration(4))

    def test_data_hash_distinguishes_identical_metadata(self) -> None:
        # Given: two grids with identical metadata and different occupancy cells.
        first = fingerprint(grid(int(ARMED_AT), (0, 0, 100, -1)), int(ARMED_AT))
        second = fingerprint(grid(int(ARMED_AT), (0, 100, 100, -1)), int(ARMED_AT))

        # When/Then: only the canonical data hash and resulting identity differ.
        self.assertNotEqual(first.data_hash, second.data_hash)
        self.assertNotEqual(first.identity(), second.identity())

    def test_wrong_target_and_current_fingerprint_do_not_advance(self) -> None:
        # Given: a guard expecting changed occupancy data.
        current = fingerprint(grid(int(ARMED_AT) - 2), int(ARMED_AT) - 1)
        target = fingerprint(grid(int(ARMED_AT) + 1, (100, 0, 0, -1)), int(ARMED_AT) + 2)
        state = MapGenerationState(MapGeneration(11), current)
        guard = arm_map_generation(state, ARMED_AT, target.identity())

        # When: current and misleading non-target maps are replayed.
        replay = evaluate_map_observation(guard, current)
        misleading = fingerprint(grid(int(ARMED_AT) + 1, (0, 100, 0, -1)), int(ARMED_AT) + 2)
        wrong = evaluate_map_observation(replay.guard, misleading)

        # Then: neither stale content nor a fresh wrong hash advances generation.
        self.assertFalse(replay.evidence.new_fingerprint)
        self.assertFalse(wrong.evidence.target_fingerprint)
        self.assertEqual(wrong.guard.state.generation, MapGeneration(11))

    def test_malformed_dirty_and_future_maps_are_rejected_at_boundary(self) -> None:
        # Given: wrong cell count, invalid occupancy, and future-generated maps.
        malformed = grid(int(ARMED_AT), (0,))
        dirty = grid(int(ARMED_AT), (0, 0, 101, -1))
        generated_in_future = grid(int(ARMED_AT) + 1)

        # When/Then: malformed external values never become fingerprints.
        for message, received_at in (
            (malformed, int(ARMED_AT)),
            (dirty, int(ARMED_AT)),
            (generated_in_future, int(ARMED_AT)),
        ):
            with self.subTest(message=message):
                with self.assertRaises(MapBoundaryError):
                    fingerprint(message, received_at)


if __name__ == "__main__":
    unittest.main()
