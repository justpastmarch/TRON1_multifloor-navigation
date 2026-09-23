from dataclasses import replace
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
for package in ("mission_manager", "multifloor_manager", "stair_supervisor"):
    sys.path.insert(0, str(ROOT / "src" / package / "src"))

from mission_manager.configuration import EdgeType, GraphEdge  # noqa: E402
from mission_manager.fsm import SegmentType as FSMSegmentType  # noqa: E402
from mission_manager.route_planner import (  # noqa: E402
    BuildingPlanner,
    LogicalAnchor,
    RoutePlanningError,
    RouteValidationError,
    SegmentType,
)
from mission_manager.site_config import load_site_configuration  # noqa: E402
from stair_supervisor.configuration import Direction  # noqa: E402


FIXTURE = ROOT / "test" / "fixtures" / "building_valid"


class BuildingPlannerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.configuration = load_site_configuration(FIXTURE)
        self.planner = BuildingPlanner(self.configuration, "home_3f")

    def test_planner_and_fsm_share_canonical_segment_type(self) -> None:
        # Given/When: planner and FSM public segment types are imported independently.
        # Then: both boundaries expose the same class, not merely equal string values.
        self.assertIs(SegmentType, FSMSegmentType)

    def test_current_pose_same_named_anchor_still_navigates(self) -> None:
        route = self.planner.plan_from_current_pose('home_3f', 'home_3f')
        self.assertEqual(len(route.segments), 1)
        self.assertIs(route.segments[0].type, SegmentType.NAVIGATION)
        self.assertEqual(route.segments[0].target_id, 'home_3f')

    def test_current_pose_at_logical_stair_anchor_reaches_entry_first(self) -> None:
        route = self.planner.plan_from_current_pose('stair_a_entry_3f', 'stair_a_landing_4f')
        self.assertEqual(tuple(s.type for s in route.segments),
                         (SegmentType.NAVIGATION, SegmentType.STAIR, SegmentType.FLOOR_TRANSITION))
        self.assertEqual(route.segments[0].target_id, 'stair_a_entry_3f')
        anchor = LogicalAnchor(route.origin_id)
        for segment in route.segments:
            anchor = anchor.after_segment(segment, succeeded=True)
        self.assertEqual(anchor.location_id, route.destination_id)

    def test_existing_navigation_route_does_not_detour_to_home(self) -> None:
        original = self.planner.plan('home_3f', 'stair_a_landing_4f')
        self.assertEqual(self.planner.plan_from_current_pose('home_3f', 'stair_a_landing_4f'), original)

    def test_same_floor_route_expands_navigation_when_locations_share_floor(self) -> None:
        # Given: two connected locations on 3F.
        # When: a route is planned between them.
        route = self.planner.plan("home_3f", "stair_a_entry_3f")
        # Then: the directed NAV edge becomes one navigation segment.
        self.assertEqual(tuple(segment.type for segment in route.segments), (SegmentType.NAVIGATION,))
        self.assertEqual(route.segments[0].edge_id, "leave_home")

    def test_multi_floor_scan_expands_all_generic_segment_types(self) -> None:
        # Given: the validated home-to-roof directed graph.
        # When: a scan route is planned across two stairs.
        route = self.planner.plan("home_3f", "roof_scan", "roof_scan_profile")
        # Then: navigation, stair, floor transition, and scan segments are explicit.
        self.assertEqual(
            tuple(segment.type for segment in route.segments),
            (
                SegmentType.NAVIGATION,
                SegmentType.STAIR,
                SegmentType.FLOOR_TRANSITION,
                SegmentType.NAVIGATION,
                SegmentType.STAIR,
                SegmentType.FLOOR_TRANSITION,
                SegmentType.NAVIGATION,
                SegmentType.SCAN,
            ),
        )

    def test_bidirectional_stair_expansion_targets_each_floor_endpoint(self) -> None:
        # Given: stair A has asymmetric 3F and 4F landing hypotheses.
        stair = self.configuration.stairs[0]
        # When: each directed stair edge is expanded independently.
        upward = self.planner.plan("stair_a_entry_3f", "stair_a_landing_4f")
        downward = self.planner.plan("stair_a_landing_4f", "home_3f")
        # Then: each route is STAIR then FLOOR_TRANSITION and selects its target floor endpoint.
        self.assertEqual(tuple(segment.type for segment in upward.segments), (SegmentType.STAIR, SegmentType.FLOOR_TRANSITION))
        self.assertEqual(tuple(segment.type for segment in downward.segments), (SegmentType.STAIR, SegmentType.FLOOR_TRANSITION))
        self.assertEqual((upward.segments[0].profile_id, upward.segments[1].target_floor), ("cautious_up", "4F"))
        self.assertEqual((downward.segments[0].profile_id, downward.segments[1].target_floor), ("cautious_down", "3F"))
        self.assertEqual(stair.endpoint_for(upward.segments[1].target_floor).expected_tag_ids, (101,))
        self.assertEqual(stair.endpoint_for(downward.segments[1].target_floor).expected_tag_ids, (100,))

    def test_unreachable_destination_is_rejected_when_no_directed_path_exists(self) -> None:
        # Given: the roof scan location and a one-way roof landing edge.
        configuration = replace(
            self.configuration,
            edges=tuple(edge for edge in self.configuration.edges if edge.id != "scan_back"),
        )
        # When/Then: return cannot invent a reverse edge.
        with self.assertRaisesRegex(RoutePlanningError, "no directed route"):
            BuildingPlanner(configuration, "home_3f").plan_return(LogicalAnchor("roof_scan"))

    def test_disabled_stair_profile_removes_that_direction_from_routing(self) -> None:
        # Given: the only 3F-up profile is disabled.
        profiles = tuple(
            replace(profile, enabled=False) if profile.id == "cautious_up" else profile
            for profile in self.configuration.stair_profiles
        )
        # When/Then: the planner rejects the route requiring that stair direction.
        with self.assertRaisesRegex(RoutePlanningError, "no directed route"):
            BuildingPlanner(replace(self.configuration, stair_profiles=profiles), "home_3f").plan(
                "home_3f", "stair_a_landing_4f"
            )

    def test_directionally_absent_stair_profile_is_not_routable(self) -> None:
        # Given: stair A's configured up profile has the wrong direction.
        profiles = tuple(
            replace(profile, direction=Direction.DOWN) if profile.id == "cautious_up" else profile
            for profile in self.configuration.stair_profiles
        )
        # When/Then: an UP edge cannot use a DOWN profile.
        with self.assertRaisesRegex(RoutePlanningError, "no directed route"):
            BuildingPlanner(replace(self.configuration, stair_profiles=profiles), "home_3f").plan(
                "home_3f", "stair_a_landing_4f"
            )

    def test_equal_length_routes_use_edge_id_order_independent_of_input_order(self) -> None:
        # Given: two equal-length home-to-4F routes stored in reverse lexical order.
        alternate = (
            GraphEdge("z_home_to_return", "home_3f", "return_4f", EdgeType.STAIR_UP, "stair_a"),
            GraphEdge("a_home_to_return", "home_3f", "return_4f", EdgeType.STAIR_UP, "stair_a"),
        )
        locations = tuple(
            replace(location, floor_id="4F") if location.id == "return_4f" else location
            for location in self.configuration.locations
        )
        stairs = tuple(
            replace(stair, endpoints=(replace(stair.endpoints[0], entry_location_id="home_3f"), stair.endpoints[1]))
            if stair.id == "stair_a"
            else stair
            for stair in self.configuration.stairs
        )
        configuration = replace(
            self.configuration,
            locations=locations,
            stairs=stairs,
            edges=alternate,
        )
        # When: planner input ordering is changed.
        forward = BuildingPlanner(configuration, "home_3f").plan("home_3f", "return_4f")
        reverse = BuildingPlanner(replace(configuration, edges=tuple(reversed(alternate))), "home_3f").plan(
            "home_3f", "return_4f"
        )
        # Then: lexical edge ID wins and serialized output is byte-identical.
        self.assertEqual(forward.segments[0].edge_id, "a_home_to_return")
        self.assertEqual(forward.to_bytes(), reverse.to_bytes())

    def test_invalid_floor_continuity_is_rejected_before_route_search(self) -> None:
        # Given: a malformed NAV edge crossing from 3F directly to RF.
        malformed = replace(self.configuration.edges[0], target_id="roof_scan")
        # When/Then: graph construction rejects the invalid typed input.
        with self.assertRaisesRegex(RouteValidationError, "floor continuity"):
            BuildingPlanner(replace(self.configuration, edges=(malformed,)), "home_3f")

    def test_return_is_fresh_bfs_over_asymmetric_edges(self) -> None:
        # Given: a completed outbound route whose reverse edge sequence does not exist.
        outbound = self.planner.plan("home_3f", "roof_scan", "roof_scan_profile")
        confirmed = LogicalAnchor("roof_scan")
        # When: return is planned from the confirmed destination.
        returned = self.planner.plan_return(confirmed)
        # Then: BFS selects the dedicated directed return edges, not reversed outbound edges.
        self.assertEqual(
            tuple(segment.edge_id for segment in returned.segments if segment.edge_id is not None),
            ("scan_back", "stair_b_down", "stair_b_down", "return_across_4f", "stair_a_down", "stair_a_down"),
        )
        self.assertNotEqual(returned.to_bytes(), outbound.to_bytes()[::-1])

    def test_logical_anchor_changes_only_after_successful_committing_segment(self) -> None:
        # Given: an anchor and the outbound segments through the first stair.
        route = self.planner.plan("home_3f", "stair_a_landing_4f")
        anchor = LogicalAnchor("home_3f")
        # When: navigation succeeds, stair fails, then stair and floor confirmation succeed.
        after_navigation = anchor.after_segment(route.segments[0], succeeded=True)
        after_failure = after_navigation.after_segment(route.segments[1], succeeded=False)
        after_stair = after_failure.after_segment(route.segments[1], succeeded=True)
        after_transition = after_stair.after_segment(route.segments[2], succeeded=True)
        # Then: failed work and unconfirmed stair motion do not move the logical anchor.
        self.assertEqual(
            tuple(item.location_id for item in (after_navigation, after_failure, after_stair, after_transition)),
            ("stair_a_entry_3f", "stair_a_entry_3f", "stair_a_entry_3f", "stair_a_landing_4f"),
        )

    def test_serialized_route_is_byte_for_byte_deterministic(self) -> None:
        # Given: the same configuration with opposite edge storage order.
        reversed_configuration = replace(self.configuration, edges=tuple(reversed(self.configuration.edges)))
        # When: equivalent planners produce a complete scan route.
        first = self.planner.plan("home_3f", "roof_scan", "roof_scan_profile").to_bytes()
        second = BuildingPlanner(reversed_configuration, "home_3f").plan(
            "home_3f", "roof_scan", "roof_scan_profile"
        ).to_bytes()
        # Then: route bytes are identical regardless of source document ordering.
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
