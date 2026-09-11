#!/usr/bin/env python3
"""Mission orchestration behavior over the validated asymmetric fixture."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))
sys.path.insert(0, str(WORKSPACE_ROOT / "src" / "multifloor_manager" / "src"))
sys.path.insert(0, str(WORKSPACE_ROOT / "src" / "stair_supervisor" / "src"))

from mission_manager.mission_orchestrator import (  # noqa: E402
    MissionOrchestrator,
    MissionOrchestratorSettings,
    MissionRunRequest,
    MissionType,
    SegmentExecution,
    SegmentExecutionStatus,
)
from mission_manager.route_planner import BuildingPlanner, Route, RouteSegment  # noqa: E402
from mission_manager.scan_recorder import RecordingDisposition, RecordingResult  # noqa: E402
from mission_manager.segment_types import SegmentType  # noqa: E402
from mission_manager.site_config import load_site_configuration  # noqa: E402


class ScriptedSegments:
    """Mutable test executor that records the actual generic segment stream."""

    def __init__(self, scripted: tuple[SegmentExecution, ...] = ()) -> None:
        self.scripted = list(scripted)
        self.executed: list[RouteSegment] = []
        self.cancel_count = 0

    def execute(self, segment: RouteSegment, _context) -> SegmentExecution:
        self.executed.append(segment)
        if self.scripted:
            return self.scripted.pop(0)
        artifact = "/tmp/fixture_scan.bag" if segment.type.value == "SCAN" else ""
        return SegmentExecution.success(artifact_path=artifact)

    def cancel_active(self) -> None:
        self.cancel_count += 1


class LoopPlanner:
    def plan(self, origin_id: str, destination_id: str, _scan_profile_id=None) -> Route:
        if origin_id == "stair_rf_landing" and destination_id == "roof_loop_se":
            targets = ("roof_loop_sw", "roof_loop_nw", "roof_loop_ne", "roof_loop_se")
        elif origin_id == "roof_loop_se" and destination_id == "stair_rf_landing":
            targets = ("stair_rf_landing",)
        else:
            raise AssertionError(f"unexpected route {origin_id} -> {destination_id}")
        segments = []
        source = origin_id
        for target in targets:
            segments.append(
                RouteSegment(
                    SegmentType.NAVIGATION,
                    source,
                    target,
                    "RF",
                    "RF",
                    f"{source}_to_{target}",
                    None,
                    None,
                    None,
                    target,
                )
            )
            source = target
        return Route(origin_id, destination_id, tuple(segments))


class RouteRecordingSession:
    def __init__(self, events: list[str]) -> None:
        self.events = events

    def assert_active(self) -> None:
        self.events.append("recording-active")

    def finish(self, disposition: RecordingDisposition) -> RecordingResult:
        self.events.append(f"finish:{disposition.value}")
        return RecordingResult(disposition, Path("/tmp/roof-loop.bag"))

    def discard(self) -> None:
        return None


class RouteRecordingSegments(ScriptedSegments):
    def __init__(self, scripted: tuple[SegmentExecution, ...] = ()) -> None:
        super().__init__(scripted)
        self.events: list[str] = []

    def start_recording(self, _request) -> RouteRecordingSession:
        self.events.append("start-recording")
        return RouteRecordingSession(self.events)

    def execute(self, segment: RouteSegment, context) -> SegmentExecution:
        self.events.append(segment.target_id)
        return super().execute(segment, context)


class MissionOrchestratorTest(unittest.TestCase):
    def setUp(self) -> None:
        fixture = WORKSPACE_ROOT / "test" / "fixtures" / "building_valid"
        configuration = load_site_configuration(fixture)
        self.planner = BuildingPlanner(configuration, "home_3f")

    def test_fresh_return_is_planned_from_confirmed_scan_anchor(self) -> None:
        # Given: an inspect mission over an asymmetric graph.
        segments = ScriptedSegments()
        orchestrator = MissionOrchestrator(
            self.planner,
            segments,
            MissionOrchestratorSettings("home_3f", "roof_scan_profile"),
        )

        # When: outbound scan and return are requested.
        result = orchestrator.run(
            MissionRunRequest("mission-1", "roof_scan", MissionType.INSPECT, True),
            lambda _feedback: None,
            lambda: False,
        )

        # Then: the return follows the graph's distinct return_4f anchor, not reversal.
        self.assertEqual(result.result_code, 0)
        self.assertEqual(result.artifact_path, "/tmp/fixture_scan.bag")
        self.assertEqual(
            [segment.target_id for segment in segments.executed],
            [
                "stair_a_entry_3f",
                "stair_a_landing_4f",
                "stair_a_landing_4f",
                "stair_b_entry_4f",
                "roof_landing",
                "roof_landing",
                "roof_scan",
                "roof_scan",
                "roof_landing",
                "return_4f",
                "return_4f",
                "stair_a_landing_4f",
                "home_3f",
                "home_3f",
            ],
        )
        self.assertEqual(orchestrator.confirmed_location_id, "home_3f")

    def test_failed_scan_never_starts_return(self) -> None:
        # Given: every outbound segment succeeds until scan fails.
        scripted = (SegmentExecution.success(),) * 7 + (
            SegmentExecution.failed(7, "scan fixture failed"),
        )
        segments = ScriptedSegments(scripted)
        orchestrator = MissionOrchestrator(
            self.planner,
            segments,
            MissionOrchestratorSettings("home_3f", "roof_scan_profile"),
        )

        # When: the return-capable mission reaches the failed scan.
        result = orchestrator.run(
            MissionRunRequest("mission-2", "roof_scan", MissionType.INSPECT, True),
            lambda _feedback: None,
            lambda: False,
        )

        # Then: no return segment executes and the confirmed anchor remains the scan site.
        self.assertEqual(result.result_code, 7)
        self.assertEqual(len(segments.executed), 8)
        self.assertEqual(orchestrator.confirmed_location_id, "roof_scan")

    def test_failure_stops_before_the_next_segment(self) -> None:
        # Given: navigation succeeds and stair traversal fails.
        segments = ScriptedSegments(
            (
                SegmentExecution.success(),
                SegmentExecution.failed(5, "stair fixture failed"),
            )
        )
        orchestrator = MissionOrchestrator(
            self.planner,
            segments,
            MissionOrchestratorSettings("home_3f", "roof_scan_profile"),
        )

        # When: a multi-floor navigation mission runs.
        result = orchestrator.run(
            MissionRunRequest("mission-3", "stair_a_landing_4f", MissionType.NAVIGATE, False),
            lambda _feedback: None,
            lambda: False,
        )

        # Then: floor transition is never started and only the NAV anchor commits.
        self.assertEqual(result.result_code, 5)
        self.assertEqual(len(segments.executed), 2)
        self.assertEqual(orchestrator.confirmed_location_id, "stair_a_entry_3f")

    def test_cancelled_segment_preserves_last_confirmed_anchor(self) -> None:
        # Given: the first segment reports a safe cancellation checkpoint.
        segments = ScriptedSegments((SegmentExecution.cancelled("cancelled at checkpoint"),))
        orchestrator = MissionOrchestrator(
            self.planner,
            segments,
            MissionOrchestratorSettings("home_3f", "roof_scan_profile"),
        )

        # When: mission execution observes cancellation.
        result = orchestrator.run(
            MissionRunRequest("mission-4", "roof_scan", MissionType.NAVIGATE, False),
            lambda _feedback: None,
            lambda: True,
        )

        # Then: the terminal is cancelled and no unconfirmed progress commits.
        self.assertEqual(result.status, SegmentExecutionStatus.CANCELLED)
        self.assertEqual(orchestrator.confirmed_location_id, "home_3f")

    def test_record_route_brackets_every_loop_segment_with_one_recording(self) -> None:
        # Given: a directed RF loop and one route-recording executor session.
        segments = RouteRecordingSegments()
        orchestrator = MissionOrchestrator(
            LoopPlanner(),
            segments,
            MissionOrchestratorSettings("stair_rf_landing", "roof_scan_profile"),
        )

        # When: record_route traverses the loop and returns to its mission origin.
        result = orchestrator.run(
            MissionRunRequest("mission-loop", "roof_loop_se", MissionType.RECORD_ROUTE, True),
            lambda _feedback: None,
            lambda: False,
        )

        # Then: one bag spans every outbound and closing navigation segment.
        self.assertEqual(result.result_code, 0)
        self.assertEqual(result.artifact_path, "/tmp/roof-loop.bag")
        self.assertEqual(orchestrator.confirmed_location_id, "stair_rf_landing")
        self.assertEqual(
            segments.events,
            [
                "start-recording",
                "roof_loop_sw",
                "recording-active",
                "roof_loop_nw",
                "recording-active",
                "roof_loop_ne",
                "recording-active",
                "roof_loop_se",
                "recording-active",
                "stair_rf_landing",
                "recording-active",
                "finish:COMPLETED",
            ],
        )

    def test_record_route_cancellation_finalizes_preempted_at_confirmed_anchor(self) -> None:
        # Given: navigation cancels before the first loop waypoint is confirmed.
        segments = RouteRecordingSegments((SegmentExecution.cancelled("operator cancel"),))
        orchestrator = MissionOrchestrator(
            LoopPlanner(),
            segments,
            MissionOrchestratorSettings("stair_rf_landing", "roof_scan_profile"),
        )

        # When: record_route receives the child terminal cancellation.
        result = orchestrator.run(
            MissionRunRequest("mission-cancel", "roof_loop_se", MissionType.RECORD_ROUTE, True),
            lambda _feedback: None,
            lambda: False,
        )

        # Then: recording is finalized once and the unconfirmed waypoint never commits.
        self.assertEqual(result.status, SegmentExecutionStatus.CANCELLED)
        self.assertEqual(result.artifact_path, "/tmp/roof-loop.bag")
        self.assertEqual(orchestrator.confirmed_location_id, "stair_rf_landing")
        self.assertEqual(
            segments.events,
            ["start-recording", "roof_loop_sw", "finish:PREEMPTED"],
        )

    def test_record_route_navigation_failure_finalizes_interrupted(self) -> None:
        # Given: navigation fails before the first loop waypoint is confirmed.
        segments = RouteRecordingSegments((SegmentExecution.failed(4, "blocked"),))
        orchestrator = MissionOrchestrator(
            LoopPlanner(),
            segments,
            MissionOrchestratorSettings("stair_rf_landing", "roof_scan_profile"),
        )

        # When: route execution terminates on the navigation failure.
        result = orchestrator.run(
            MissionRunRequest("mission-fail", "roof_loop_se", MissionType.RECORD_ROUTE, True),
            lambda _feedback: None,
            lambda: False,
        )

        # Then: the failure code survives and the partial bag is truthfully finalized.
        self.assertEqual(result.result_code, 4)
        self.assertEqual(result.artifact_path, "/tmp/roof-loop.bag")
        self.assertEqual(orchestrator.confirmed_location_id, "stair_rf_landing")
        self.assertEqual(
            segments.events,
            ["start-recording", "roof_loop_sw", "finish:INTERRUPTED"],
        )


if __name__ == "__main__":
    unittest.main()
