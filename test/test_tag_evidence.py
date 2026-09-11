from __future__ import annotations

from pathlib import Path
import sys
from typing import NamedTuple, Tuple
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "multifloor_manager" / "src"))

from multifloor_manager.tag_evidence import (  # noqa: E402
    ActiveTagRoute,
    DEFAULT_TEMPORAL_VOTE_POLICY,
    FloorId,
    FloorTagSet,
    Nanoseconds,
    TagBoundaryError,
    TagId,
    TagVoteContext,
    arm_tag_vote,
    evaluate_tag_observation,
    parse_apriltag_detection_array,
)


NOW = Nanoseconds(1_700_000_010_000_000_000)


class FakeStamp(NamedTuple):
    nanoseconds: int

    def to_nsec(self) -> int:
        return self.nanoseconds


class FakeHeader(NamedTuple):
    frame_id: str
    stamp: FakeStamp


class FakeDetection(NamedTuple):
    id: Tuple[int, ...]


class FakeDetectionArray(NamedTuple):
    header: FakeHeader
    detections: Tuple[FakeDetection, ...]


def message(stamp_ns: int, *tag_ids: int) -> FakeDetectionArray:
    return FakeDetectionArray(
        FakeHeader("camera_link", FakeStamp(stamp_ns)),
        tuple(FakeDetection((tag_id,)) for tag_id in tag_ids),
    )


def context() -> TagVoteContext:
    return TagVoteContext(
        ActiveTagRoute(FloorId("4F"), (TagId(101),)),
        (
            FloorTagSet(FloorId("4F"), (TagId(101), TagId(111))),
            FloorTagSet(FloorId("RF"), (TagId(202),)),
        ),
        DEFAULT_TEMPORAL_VOTE_POLICY,
    )


def observe(state, stamp_ns: int, *tag_ids: int):
    batch = parse_apriltag_detection_array(message(stamp_ns, *tag_ids), NOW)
    return evaluate_tag_observation(state, batch, context())


class AprilTagEvidenceTest(unittest.TestCase):
    def test_three_consecutive_fresh_route_detections_accept(self) -> None:
        # Given: an armed 4F route and three fresh, ordered tag arrays.
        state = arm_tag_vote(Nanoseconds(int(NOW) - 1_000_000_000))

        # When: the expected landing tag is seen three times within one second.
        for age_ns in (400_000_000, 200_000_000, 0):
            result = observe(state, int(NOW) - age_ns, 101)
            state = result.state

        # Then: the reusable evidence exposes only satisfied Boolean predicates.
        self.assertTrue(result.evidence.accepted)
        self.assertTrue(result.evidence.consecutive_vote)
        self.assertTrue(all(predicate.value for predicate in result.evidence.predicates()))

    def test_unknown_tag_resets_a_partial_vote(self) -> None:
        # Given: one valid vote followed by an unconfigured tag.
        state = arm_tag_vote(Nanoseconds(int(NOW) - 1_000_000_000))
        state = observe(state, int(NOW) - 300_000_000, 101).state

        # When: unknown evidence interrupts the sequence.
        result = observe(state, int(NOW) - 200_000_000, 999)

        # Then: it is rejected and the next valid observation starts at one.
        self.assertFalse(result.evidence.known_tags)
        restarted = observe(result.state, int(NOW) - 100_000_000, 101)
        self.assertEqual(restarted.state.consecutive_count, 1)

    def test_stale_and_replayed_detections_do_not_vote(self) -> None:
        # Given: an armed route with no evidence.
        state = arm_tag_vote(Nanoseconds(int(NOW) - 1_000_000_000))

        # When: an old sample and then the same fresh sample twice are observed.
        stale = observe(state, int(NOW) - 500_000_001, 101)
        first = observe(stale.state, int(NOW) - 100_000_000, 101)
        replay = observe(first.state, int(NOW) - 100_000_000, 101)

        # Then: stale and non-monotonic evidence cannot contribute votes.
        self.assertFalse(stale.evidence.fresh)
        self.assertFalse(replay.evidence.monotonic_stamp)
        self.assertEqual(replay.state.consecutive_count, 0)

    def test_delayed_frames_cannot_lower_timestamp_high_water_or_rebuild_vote(self) -> None:
        # Given: a partial vote whose newest timestamp is NOW minus 100 ms.
        state = arm_tag_vote(Nanoseconds(int(NOW) - 1_000_000_000))
        newest = observe(state, int(NOW) - 100_000_000, 101)

        # When: an older frame arrives, followed by three delayed increasing frames.
        delayed = observe(newest.state, int(NOW) - 400_000_000, 101)
        state = delayed.state
        delayed_results = []
        for age_ns in (300_000_000, 200_000_000, 150_000_000):
            result = observe(state, int(NOW) - age_ns, 101)
            delayed_results.append(result)
            state = result.state

        # Then: the old high-water mark remains and every delayed vote is rejected.
        high_water = Nanoseconds(int(NOW) - 100_000_000)
        self.assertEqual(delayed.state.last_observed_stamp_ns, high_water)
        self.assertTrue(all(not result.evidence.monotonic_stamp for result in delayed_results))
        self.assertTrue(all(not result.evidence.accepted for result in delayed_results))
        self.assertEqual(state.last_observed_stamp_ns, high_water)
        self.assertEqual(state.consecutive_count, 0)

        recovered = observe(state, int(NOW) - 50_000_000, 101)
        self.assertTrue(recovered.evidence.monotonic_stamp)
        self.assertEqual(recovered.state.consecutive_count, 1)

    def test_fresh_pre_arm_detection_does_not_vote(self) -> None:
        # Given: a sample captured shortly before the transition was armed.
        state = arm_tag_vote(NOW)

        # When: the still-fresh latched sample is delivered after arming.
        result = observe(state, int(NOW) - 100_000_000, 101)

        # Then: capture time, not delivery freshness, protects the new vote.
        self.assertTrue(result.evidence.fresh)
        self.assertFalse(result.evidence.post_arm)
        self.assertEqual(result.state.consecutive_count, 0)

    def test_future_timestamp_is_rejected_before_it_can_poison_vote_state(self) -> None:
        # Given: an armed vote and a detection timestamp beyond the injected clock.
        state = arm_tag_vote(Nanoseconds(int(NOW) - 1_000_000_000))
        future = message(int(NOW) + 1, 101)

        # When: the external message crosses the typed boundary.
        with self.assertRaisesRegex(TagBoundaryError, "must not be in the future"):
            parse_apriltag_detection_array(future, NOW)

        # Then: unchanged state accepts the next valid frame as the first vote.
        valid = parse_apriltag_detection_array(message(int(NOW), 101), NOW)
        result = evaluate_tag_observation(state, valid, context())
        self.assertTrue(result.evidence.monotonic_stamp)
        self.assertEqual(result.state.last_observed_stamp_ns, NOW)
        self.assertEqual(result.state.consecutive_count, 1)

    def test_intermittent_empty_array_breaks_consecutive_vote(self) -> None:
        # Given: two valid detections separated by a no-detection frame.
        state = arm_tag_vote(Nanoseconds(int(NOW) - 1_000_000_000))
        first = observe(state, int(NOW) - 300_000_000, 101)
        gap = observe(first.state, int(NOW) - 200_000_000)

        # When: two more expected detections arrive.
        second = observe(gap.state, int(NOW) - 100_000_000, 101)
        third = observe(second.state, int(NOW), 101)

        # Then: only the post-gap pair is consecutive.
        self.assertFalse(third.evidence.accepted)
        self.assertEqual(third.state.consecutive_count, 2)

    def test_wrong_route_tag_on_target_floor_is_rejected(self) -> None:
        # Given: tag 111 is known on 4F but not expected by the active route.
        state = arm_tag_vote(Nanoseconds(int(NOW) - 1_000_000_000))

        # When: that same-floor tag is observed.
        result = observe(state, int(NOW), 111)

        # Then: floor identity alone cannot satisfy the active route.
        self.assertTrue(result.evidence.target_floor)
        self.assertFalse(result.evidence.active_route)
        self.assertFalse(result.evidence.accepted)

    def test_equally_valid_wrong_floor_vote_aborts(self) -> None:
        # Given: a 4F route and RF's configured expected tag.
        state = arm_tag_vote(Nanoseconds(int(NOW) - 1_000_000_000))

        # When: RF evidence forms the same three-sample temporal vote.
        for age_ns in (400_000_000, 200_000_000, 0):
            result = observe(state, int(NOW) - age_ns, 202)
            state = result.state

        # Then: the wrong-floor vote aborts rather than accepting a floor.
        self.assertTrue(result.evidence.wrong_floor_vote)
        self.assertFalse(result.evidence.accepted)

    def test_timeout_and_dirty_mixed_array_are_rejected(self) -> None:
        # Given: an expired vote and a mixed expected/unknown array.
        expired = arm_tag_vote(Nanoseconds(int(NOW) - 10_000_000_001))

        # When: otherwise fresh evidence arrives after timeout and with dirt.
        timeout = observe(expired, int(NOW), 101)
        dirty = observe(arm_tag_vote(NOW), int(NOW), 101, 999)

        # Then: neither can advance an expected vote.
        self.assertTrue(timeout.evidence.timed_out)
        self.assertFalse(timeout.evidence.accepted)
        self.assertFalse(dirty.evidence.known_tags)
        self.assertEqual(dirty.state.consecutive_count, 0)

    def test_malformed_ros_boundary_is_rejected(self) -> None:
        # Given: ROS-shaped arrays with an empty frame and empty detection ID.
        empty_frame = FakeDetectionArray(FakeHeader("", FakeStamp(int(NOW))), ())
        empty_id = FakeDetectionArray(
            FakeHeader("camera_link", FakeStamp(int(NOW))),
            (FakeDetection(()),),
        )

        # When/Then: malformed external messages never enter policy code.
        with self.assertRaises(TagBoundaryError):
            parse_apriltag_detection_array(empty_frame, NOW)
        with self.assertRaises(TagBoundaryError):
            parse_apriltag_detection_array(empty_id, NOW)


if __name__ == "__main__":
    unittest.main()
