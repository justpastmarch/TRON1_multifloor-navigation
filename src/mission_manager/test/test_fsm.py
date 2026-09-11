from pathlib import Path
import sys
import unittest


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from mission_manager.fsm import (  # noqa: E402
    IllegalTransitionError,
    InvalidMissionEventError,
    MissionEvent,
    MissionFSM,
    MissionState,
    SegmentType,
)


LEGAL_TRANSITIONS = {
    (MissionState.WAIT_GOAL, MissionEvent.GOAL_RECEIVED): MissionState.PLAN_MISSION,
    (MissionState.PLAN_MISSION, MissionEvent.PLAN_SUCCEEDED): MissionState.EXECUTE_SEGMENT,
    (MissionState.PLAN_MISSION, MissionEvent.PLAN_FAILED): MissionState.MISSION_ABORT,
    (MissionState.EXECUTE_SEGMENT, MissionEvent.SEGMENT_SUCCEEDED): MissionState.NEXT_SEGMENT,
    (MissionState.EXECUTE_SEGMENT, MissionEvent.SEGMENT_RETRY): MissionState.EXECUTE_SEGMENT,
    (MissionState.EXECUTE_SEGMENT, MissionEvent.SEGMENT_FAILED): MissionState.MISSION_ABORT,
    (MissionState.NEXT_SEGMENT, MissionEvent.MORE_SEGMENTS): MissionState.EXECUTE_SEGMENT,
    (MissionState.NEXT_SEGMENT, MissionEvent.ALL_SEGMENTS_COMPLETE): MissionState.MISSION_COMPLETE,
    (MissionState.MISSION_COMPLETE, MissionEvent.RESET): MissionState.WAIT_GOAL,
    (MissionState.MISSION_ABORT, MissionEvent.RESET): MissionState.WAIT_GOAL,
    (MissionState.PLAN_MISSION, MissionEvent.CANCEL): MissionState.MISSION_ABORT,
    (MissionState.EXECUTE_SEGMENT, MissionEvent.CANCEL): MissionState.MISSION_ABORT,
    (MissionState.NEXT_SEGMENT, MissionEvent.CANCEL): MissionState.MISSION_ABORT,
    (MissionState.PLAN_MISSION, MissionEvent.TIMEOUT): MissionState.MISSION_ABORT,
    (MissionState.EXECUTE_SEGMENT, MissionEvent.TIMEOUT): MissionState.MISSION_ABORT,
    (MissionState.NEXT_SEGMENT, MissionEvent.TIMEOUT): MissionState.MISSION_ABORT,
}


class MissionFSMTest(unittest.TestCase):
    def test_every_legal_state_event_pair_has_the_declared_result(self) -> None:
        # Given: the complete mission transition table.
        for (state, event), expected in LEGAL_TRANSITIONS.items():
            with self.subTest(state=state, event=event):
                # When: each legal event is dispatched from its source state.
                actual = MissionFSM(state).dispatch(event)
                # Then: the transition is deterministic and leaves the input unchanged.
                self.assertEqual(actual.state, expected)
                self.assertEqual(MissionFSM(state).state, state)

    def test_every_undeclared_state_event_pair_is_illegal(self) -> None:
        # Given: every pair outside the complete legal transition table.
        for state in MissionState:
            for event in MissionEvent:
                if (state, event) in LEGAL_TRANSITIONS:
                    continue
                with self.subTest(state=state, event=event):
                    # When/Then: dispatch rejects the pair without an implicit fallback.
                    with self.assertRaises(IllegalTransitionError):
                        MissionFSM(state).dispatch(event)

    def test_malformed_raw_event_has_stable_typed_error_message(self) -> None:
        # Given: an unparsed raw event value at the public boundary.
        # When: dispatch rejects it.
        with self.assertRaises(InvalidMissionEventError) as raised:
            MissionFSM().dispatch("BAD_EVENT")
        # Then: formatting the typed error is stable and preserves the raw value.
        self.assertEqual(str(raised.exception), "event 'BAD_EVENT' is invalid while mission is in WAIT_GOAL")

    def test_complete_route_replay_reaches_mission_complete(self) -> None:
        # Given: a fresh mission FSM and a two-segment mission event sequence.
        fsm = MissionFSM()
        events = (
            MissionEvent.GOAL_RECEIVED,
            MissionEvent.PLAN_SUCCEEDED,
            MissionEvent.SEGMENT_SUCCEEDED,
            MissionEvent.MORE_SEGMENTS,
            MissionEvent.SEGMENT_SUCCEEDED,
            MissionEvent.ALL_SEGMENTS_COMPLETE,
        )
        # When: the complete route is replayed.
        states = tuple((fsm := fsm.dispatch(event)).state for event in events)
        # Then: every intermediate state and the terminal state are exact.
        self.assertEqual(
            states,
            (
                MissionState.PLAN_MISSION,
                MissionState.EXECUTE_SEGMENT,
                MissionState.NEXT_SEGMENT,
                MissionState.EXECUTE_SEGMENT,
                MissionState.NEXT_SEGMENT,
                MissionState.MISSION_COMPLETE,
            ),
        )

    def test_retry_result_reexecutes_the_same_segment_state(self) -> None:
        # Given: a segment currently being executed.
        executing = MissionFSM(MissionState.EXECUTE_SEGMENT)
        # When: its handler reports a retry result.
        retried = executing.dispatch(MissionEvent.SEGMENT_RETRY)
        # Then: execution remains active rather than advancing or aborting.
        self.assertEqual(retried.state, MissionState.EXECUTE_SEGMENT)

    def test_cancellation_and_timeout_abort_every_active_state(self) -> None:
        # Given: each nonterminal state after a goal has been accepted.
        active_states = (MissionState.PLAN_MISSION, MissionState.EXECUTE_SEGMENT, MissionState.NEXT_SEGMENT)
        # When: cancellation or timeout interrupts any active state.
        for state in active_states:
            for event in (MissionEvent.CANCEL, MissionEvent.TIMEOUT):
                with self.subTest(state=state, event=event):
                    aborted = MissionFSM(state).dispatch(event)
                    # Then: every interruption deterministically aborts the mission.
                    self.assertEqual(aborted.state, MissionState.MISSION_ABORT)

    def test_state_and_segment_sets_have_no_floor_specific_variants(self) -> None:
        # Given/When: public state and segment names are enumerated.
        states = {state.name for state in MissionState}
        segments = {segment.name for segment in SegmentType}
        # Then: only the generic contract exists.
        self.assertEqual(
            states,
            {"WAIT_GOAL", "PLAN_MISSION", "EXECUTE_SEGMENT", "NEXT_SEGMENT", "MISSION_COMPLETE", "MISSION_ABORT"},
        )
        self.assertEqual(segments, {"NAVIGATION", "STAIR", "FLOOR_TRANSITION", "SCAN"})


if __name__ == "__main__":
    unittest.main()
