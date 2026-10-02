#!/usr/bin/env python3
"""Verify mission segment adapters send receiver-owned identifiers."""

from __future__ import annotations

from types import SimpleNamespace
from pathlib import Path
import sys
from unittest import mock
import unittest

from actionlib_msgs.msg import GoalStatus
from multifloor_manager.msg import FloorState, FloorTransitionResult
from stair_supervisor.msg import StairTraversalResult

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "mission_manager" / "src"))
sys.path.insert(0, str(ROOT / "src" / "multifloor_manager" / "src"))
sys.path.insert(0, str(ROOT / "src" / "stair_supervisor" / "src"))

from mission_manager.configuration import Location, ScanProfile  # noqa: E402
from mission_manager.mission_orchestrator import RouteRecordingRequest, SegmentContext  # noqa: E402
from mission_manager.ros_segments import RosSegmentExecutor, RosSegmentResources  # noqa: E402
from mission_manager.route_planner import RouteSegment  # noqa: E402
from mission_manager.segment_types import SegmentType  # noqa: E402
from mission_manager.stair_entry_gate import StairEntryDecision, StairEntryFence  # noqa: E402
from stair_supervisor.configuration import Direction, StairProfile  # noqa: E402


class FakeActionClient:
    """Record generated goals and return a configured terminal result."""

    def __init__(self, result: SimpleNamespace) -> None:
        self.result = result
        self.goals = []

    def wait_for_server(self, _timeout) -> bool:
        return True

    def send_goal(self, goal) -> None:
        self.goals.append(goal)

    def wait_for_result(self) -> bool:
        return True

    def get_state(self) -> int:
        return GoalStatus.SUCCEEDED

    def get_result(self):
        return self.result

    def cancel_goal(self) -> None:
        return None


class FakeNavigation:
    def prepare_for_stair(self) -> None:
        return None


class FakeState:
    def __init__(self) -> None:
        self._floor = FloorState(floor_id="3F", map_generation=4, state=FloorState.READY)
        self.waited_for_floor = None
        self.entry_decision = StairEntryDecision(True, "aligned")

    def health(self) -> SimpleNamespace:
        return SimpleNamespace(healthy=True, reason="")

    def floor_state(self) -> FloorState:
        return self._floor

    def wait_for_floor(self, floor_id: str, generation: int, _timeout: float) -> bool:
        self.waited_for_floor = (floor_id, generation)
        return True

    def begin_stair_entry(self) -> StairEntryFence:
        return StairEntryFence("3F", 4, 10.0, 20.0, 1)

    def wait_for_stair_entry(self, _fence, _expected) -> StairEntryDecision:
        return self.entry_decision

    def stair_entry_decision(self, _fence, _expected) -> StairEntryDecision:
        return StairEntryDecision(True, "aligned")

    def supervisor_state(self) -> SimpleNamespace:
        return SimpleNamespace(ownership_epoch=3)


class FakeAdmission:
    def __init__(self) -> None:
        self.cleared = []

    def issue(self, _context, _recheck) -> str:
        return "test-token"

    def clear(self, token: str) -> None:
        self.cleared.append(token)


def resources(state: FakeState, scan_profiles=(), scan_recorder=None) -> RosSegmentResources:
    return RosSegmentResources(
        navigation=FakeNavigation(),
        state=state,
        locations=(Location("home", "3F", "home", 0.0, 0.0, 0.0),),
        scan_profiles=scan_profiles,
        stair_profiles=(StairProfile(
            "cautious_up", Direction.UP, True, 0.12, 0.25,
            0.0, 1.00, 0.20, 0.50, 0.80, 0.20,
            0.02, 0.02, 0.20, 0.15, 0.40, 0.30, 60.0,
        ),),
        scan_recorder=scan_recorder,
        stair_admission=FakeAdmission(),
    )


class FakeScanRecorder:
    def __init__(self) -> None:
        self.started = []
        self.session = SimpleNamespace()

    def start(self, profile, identity):
        self.started.append((profile, identity))
        return self.session


class RosSegmentIdentifierTest(unittest.TestCase):
    def test_cancelled_dispatch_sends_no_child_goal(self):
        state=FakeState();clients=[FakeActionClient(SimpleNamespace()),FakeActionClient(SimpleNamespace())]
        with mock.patch('mission_manager.ros_segments.actionlib.SimpleActionClient',side_effect=clients), mock.patch('mission_manager.ros_segments.rospy.get_param',side_effect=lambda n,d:d):
            executor=RosSegmentExecutor(resources(state))
            segment=RouteSegment(SegmentType.STAIR,'home','landing','3F','4F','edge','stair_a','cautious_up',None,'home')
            result=executor.execute(segment,SegmentContext('cancelled',lambda:True))
            self.assertEqual(result.status.value,'CANCELLED')
            self.assertEqual(clients[0].goals,[])

    def test_cancel_during_entry_wait_does_not_dispatch_stair(self):
        state=FakeState();cancelled=[False]
        def wait(*args):
            cancelled[0]=True
            return StairEntryDecision(True,'aligned')
        state.wait_for_stair_entry=wait
        clients=[FakeActionClient(SimpleNamespace()),FakeActionClient(SimpleNamespace())]
        with mock.patch('mission_manager.ros_segments.actionlib.SimpleActionClient',side_effect=clients), mock.patch('mission_manager.ros_segments.rospy.get_param',side_effect=lambda n,d:d):
            executor=RosSegmentExecutor(resources(state))
            segment=RouteSegment(SegmentType.STAIR,'home','landing','3F','4F','edge','stair_a','cautious_up',None,'home')
            result=executor.execute(segment,SegmentContext('cancelled',lambda:cancelled[0]))
            self.assertEqual(result.status.value,'CANCELLED')
            self.assertEqual(clients[0].goals,[])

    def test_navigation_receives_live_cancellation_predicate(self):
        state=FakeState();clients=[FakeActionClient(SimpleNamespace()),FakeActionClient(SimpleNamespace())]
        with mock.patch('mission_manager.ros_segments.actionlib.SimpleActionClient',side_effect=clients), mock.patch('mission_manager.ros_segments.rospy.get_param',side_effect=lambda n,d:d):
            executor=RosSegmentExecutor(resources(state));executor._navigation=mock.Mock()
            from mission_manager.navigation_executor import NavigationOutcome
            executor._navigation.execute.return_value=SimpleNamespace(outcome=NavigationOutcome.CANCELLED)
            segment=RouteSegment(SegmentType.NAVIGATION,'home','home','3F','3F','edge',None,None,None,'home')
            context=SegmentContext('nav',lambda:False)
            executor.execute(segment,context)
            self.assertIs(executor._navigation.execute.call_args.kwargs['cancellation_requested'],context.cancellation_requested)

    def test_route_recording_resolves_profile_and_mission_identity(self) -> None:
        # Given: one configured route profile and an in-process recorder adapter.
        state = FakeState()
        profile = ScanProfile("roof_scan_profile", ("/livox/lidar",), 30.0, Path("roof"), 0.2)
        recorder = FakeScanRecorder()
        stair_client = FakeActionClient(SimpleNamespace())
        floor_client = FakeActionClient(SimpleNamespace())

        # When: orchestration starts a route-wide recording session.
        with mock.patch(
            "mission_manager.ros_segments.actionlib.SimpleActionClient",
            side_effect=(stair_client, floor_client),
        ), mock.patch(
            "mission_manager.ros_segments.rospy.get_param",
            side_effect=lambda name, default: default,
        ), mock.patch(
            "mission_manager.ros_segments.rospy.Time.now",
            return_value=SimpleNamespace(to_nsec=lambda: 123456789),
        ):
            executor = RosSegmentExecutor(resources(state, (profile,), recorder))
            session = executor.start_recording(
                RouteRecordingRequest("mission-loop", "roof_scan_profile", "roof_loop_se")
            )

        # Then: the existing recorder owns one profile-derived, mission-scoped artifact.
        self.assertIs(session, recorder.session)
        started_profile, identity = recorder.started[0]
        self.assertIs(started_profile, profile)
        self.assertEqual(
            (identity.mission_id, identity.location_id, identity.timestamp_ns),
            ("mission-loop", "roof_loop_se", 123456789),
        )

    def test_stair_and_floor_adapters_use_their_receiver_keys(self) -> None:
        # Given: a route whose physical stair ID and selected profile ID differ.
        state = FakeState()
        stair_client = FakeActionClient(
            SimpleNamespace(result_code=StairTraversalResult.OK, reason="")
        )
        floor_client = FakeActionClient(
            SimpleNamespace(
                result_code=FloorTransitionResult.OK,
                floor_id="4F",
                map_generation=5,
                reason="",
            )
        )
        segment = RouteSegment(
            SegmentType.STAIR,
            "home",
            "stair_landing",
            "3F",
            "4F",
            "edge_3f_4f",
            "stair_a",
            "cautious_up",
            None,
            "home",
        )
        floor_segment = RouteSegment(
            SegmentType.FLOOR_TRANSITION,
            "home",
            "stair_landing",
            "3F",
            "4F",
            "edge_3f_4f",
            "stair_a",
            None,
            None,
            "stair_landing",
        )

        with mock.patch(
            "mission_manager.ros_segments.actionlib.SimpleActionClient",
            side_effect=(stair_client, floor_client),
        ), mock.patch(
            "mission_manager.ros_segments.rospy.get_param",
            side_effect=lambda name, default: default,
        ):
            executor = RosSegmentExecutor(resources(state))
            # When: both operational child adapters execute their route segments.
            context = SegmentContext("mission-1", lambda: False)
            self.assertEqual(executor.execute(segment, context).status.value, "SUCCESS")
            self.assertEqual(executor.execute(floor_segment, context).status.value, "SUCCESS")

        # Then: each child receives the identifier it actually indexes.
        self.assertEqual(stair_client.goals[0].stair_id, "cautious_up")
        self.assertEqual(stair_client.goals[0].admission_token, "test-token")
        self.assertEqual(floor_client.goals[0].transition_id, "stair_a")

    def test_stair_entry_mismatch_sends_no_child_goal(self) -> None:
        # Given: stationary handoff succeeds but fresh AMCL rejects the canonical entry pose.
        state = FakeState()
        state.entry_decision = StairEntryDecision(False, "robot is outside the stair-entry heading tolerance")
        stair_client = FakeActionClient(SimpleNamespace())
        floor_client = FakeActionClient(SimpleNamespace())
        segment = RouteSegment(
            SegmentType.STAIR,
            "home",
            "stair_landing",
            "3F",
            "4F",
            "edge_3f_4f",
            "stair_a",
            "cautious_up",
            None,
            "home",
        )

        # When: the mission tries to cross the NAV-to-stair boundary.
        with mock.patch(
            "mission_manager.ros_segments.actionlib.SimpleActionClient",
            side_effect=(stair_client, floor_client),
        ), mock.patch(
            "mission_manager.ros_segments.rospy.get_param",
            side_effect=lambda name, default: default,
        ):
            outcome = RosSegmentExecutor(resources(state)).execute(
                segment,
                SegmentContext("mission-unsafe", lambda: False),
            )

        # Then: mismatch is a navigation failure and the stair action is untouched.
        self.assertEqual(outcome.status.value, "FAILED")
        self.assertEqual(outcome.result_code, 4)
        self.assertEqual(stair_client.goals, [])


if __name__ == "__main__":
    unittest.main()
