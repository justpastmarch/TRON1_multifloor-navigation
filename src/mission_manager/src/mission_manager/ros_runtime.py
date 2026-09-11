"""Construct the operational mission manager from gated ROS parameters."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import rospkg
import rospy

from mission_manager.mission_action_server import MissionActionServer
from mission_manager.mission_orchestrator import MissionOrchestrator, MissionOrchestratorSettings
from mission_manager.navigation_executor import NavigationExecutor, RosNavigationSources
from mission_manager.ros_segments import RosSegmentExecutor, RosSegmentResources
from mission_manager.ros_state import RosStateMonitor
from mission_manager.route_planner import BuildingPlanner
from mission_manager.scan_recorder import ScanRecorder
from mission_manager.site_config import ConfigurationRoots, load_site_configuration_from_roots


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
        return ConfigurationRoots(
            Path(packages.get_path("mission_manager")) / "config",
            Path(packages.get_path("multifloor_manager")) / "config",
            Path(packages.get_path("stair_supervisor")) / "config",
        )
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
