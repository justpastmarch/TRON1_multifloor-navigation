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

    def health(self) -> SimpleNamespace:
        return SimpleNamespace(healthy=True, reason="")

    def floor_state(self) -> FloorState:
        return self._floor

    def wait_for_floor(self, floor_id: str, generation: int, _timeout: float) -> bool:
        self.waited_for_floor = (floor_id, generation)
        return True


def resources(state: FakeState, scan_profiles=(), scan_recorder=None) -> RosSegmentResources:
    return RosSegmentResources(
        navigation=FakeNavigation(),
        state=state,
        locations=(Location("home", "3F", "home", 0.0, 0.0, 0.0),),
        scan_profiles=scan_profiles,
        stair_profiles=(StairProfile(
            "cautious_up", Direction.UP, True, 0.12, 0.25,
            0.40, 1.00, 0.20, 0.50, 0.80, 0.20,
            0.02, 0.02, 0.20, 0.05, 0.20, 0.15, 0.40, 0.30, 60.0,
        ),),
        scan_recorder=scan_recorder,
    )


class FakeScanRecorder:
    def __init__(self) -> None:
        self.started = []
        self.session = SimpleNamespace()

    def start(self, profile, identity):
        self.started.append((profile, identity))
        return self.session


class RosSegmentIdentifierTest(unittest.TestCase):
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
        self.assertEqual(floor_client.goals[0].transition_id, "stair_a")


if __name__ == "__main__":
    unittest.main()
