"""Construct the operational mission manager from gated ROS parameters."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import rospkg
import rospy
from multifloor_manager.readiness import DEFAULT_READINESS_POLICY

from mission_manager.mission_action_server import MissionActionServer
from mission_manager.mission_orchestrator import MissionOrchestrator, MissionOrchestratorSettings
from mission_manager.navigation_executor import NavigationExecutor, RosNavigationSources
from mission_manager.ros_segments import RosSegmentExecutor, RosSegmentResources
from mission_manager.ros_state import RosStateMonitor
from mission_manager.route_planner import BuildingPlanner
from mission_manager.scan_recorder import ScanRecorder
from mission_manager.site_config import ConfigurationRoots, load_site_configuration_from_roots
from mission_manager.stair_admission import RosStairAdmissionBroker
from mission_manager.stair_entry_gate import StairEntryPolicy


@dataclass(frozen=True)
class RuntimeSettings:
    __slots__ = (
        "roots", "home_location_id", "initial_location_id", "inspect_profile_id",
        "scan_output_base", "action_name",
    )
    roots: ConfigurationRoots
    home_location_id: str
    initial_location_id: str
    inspect_profile_id: str
    scan_output_base: Path
    action_name: str


def _configuration_roots() -> ConfigurationRoots:
    profile = rospy.get_param("~config_profile", "production")
    packages = rospkg.RosPack()
    if profile == "production":
        mission_root = Path(packages.get_path("mission_manager")) / "config"
        multifloor_root = Path(packages.get_path("multifloor_manager")) / "config"
        # Allow integration tests to substitute a validated stair fixture while
        # keeping the shipped production stair profile fail-closed.
        stair_override = rospy.get_param("~stair_config_root", "")
        stair_root = Path(stair_override) if stair_override else Path(packages.get_path("stair_supervisor")) / "config"
        return ConfigurationRoots(mission_root, multifloor_root, stair_root)
    if profile == "test_fixture":
        if not rospy.get_param("~allow_test_fixture", False):
            raise RuntimeError("test_fixture profile requires allow_test_fixture=true")
        fixture_root = Path(rospy.get_param("~fixture_config_root")).resolve()
        return ConfigurationRoots(fixture_root, fixture_root, fixture_root)
    raise RuntimeError("config_profile must be production or test_fixture")


def load_runtime_settings() -> RuntimeSettings:
    """Load startup policy; fixture data requires two explicit test-only parameters."""
    return RuntimeSettings(
        _configuration_roots(),
        rospy.get_param("~home_location_id"),
        rospy.get_param("~initial_location_id"),
        rospy.get_param("~inspect_profile_id"),
        Path(rospy.get_param("~scan_output_base", "/var/lib/tron1")).resolve(),
        rospy.get_param("~action_name", "/mission"),
    )


def create_mission_action_server() -> MissionActionServer:
    """Build all non-node components after the script has initialized ROS once."""
    settings = load_runtime_settings()
    configuration = load_site_configuration_from_roots(settings.roots)
    planner = BuildingPlanner(configuration, settings.home_location_id)
    state = RosStateMonitor(
        float(rospy.get_param("~state_freshness_sec", 2.0)),
        float(rospy.get_param("~stationary_speed_threshold", 0.02)),
        float(rospy.get_param("~handoff_timeout", 2.0)),
        StairEntryPolicy(
            float(rospy.get_param("/move_base/TrajectoryPlannerROS/xy_goal_tolerance", 0.25)),
            float(rospy.get_param("/move_base/TrajectoryPlannerROS/yaw_goal_tolerance", 0.2)),
            float(DEFAULT_READINESS_POLICY.max_age_ns) / 1_000_000_000.0,
            DEFAULT_READINESS_POLICY.max_covariance_x,
            DEFAULT_READINESS_POLICY.max_covariance_y,
            DEFAULT_READINESS_POLICY.max_covariance_yaw,
            DEFAULT_READINESS_POLICY.required_pose_samples,
            DEFAULT_READINESS_POLICY.max_linear_speed,
            DEFAULT_READINESS_POLICY.max_angular_speed,
        ),
    )
    navigation = NavigationExecutor.create_ros(
        RosNavigationSources(state.floor_state, state.supervisor_state, state)
    )
    segment_executor = RosSegmentExecutor(
        RosSegmentResources(
            navigation,
            state,
            configuration.locations,
            configuration.scan_profiles,
            configuration.stair_profiles,
            ScanRecorder(settings.scan_output_base),
            RosStairAdmissionBroker(
                rospy.get_param("~stair_admission_service", "/mission/validate_stair_admission"),
                float(rospy.get_param("~stair_admission_lifetime_sec", 1.0)),
            ),
        )
    )
    orchestrator = MissionOrchestrator(
        planner,
        segment_executor,
        MissionOrchestratorSettings(
            settings.initial_location_id,
            settings.inspect_profile_id,
        ),
    )
    return MissionActionServer(
        settings.action_name,
        orchestrator,
        frozenset(location.id for location in configuration.locations),
    )
