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
from mission_manager.stair_entry_gate import StairEntryPolicy, LIDAR_ENTRY_POLICY


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
    lidar_handoff = bool(rospy.get_param("~stair_lidar_handoff", False))
    entry_defaults = LIDAR_ENTRY_POLICY if lidar_handoff else StairEntryPolicy(
        .25, .2, float(DEFAULT_READINESS_POLICY.max_age_ns)/1e9,
        DEFAULT_READINESS_POLICY.max_covariance_x, DEFAULT_READINESS_POLICY.max_covariance_y,
        DEFAULT_READINESS_POLICY.max_covariance_yaw, DEFAULT_READINESS_POLICY.required_pose_samples,
        DEFAULT_READINESS_POLICY.max_linear_speed, DEFAULT_READINESS_POLICY.max_angular_speed)
    state = RosStateMonitor(
        float(rospy.get_param("~state_freshness_sec", 2.0)),
        float(rospy.get_param("~stationary_speed_threshold", 0.02)),
        float(rospy.get_param("~handoff_timeout", 4.0 if lidar_handoff else 2.0)),
        StairEntryPolicy(
            float(rospy.get_param("~stair_entry_xy_tolerance", entry_defaults.xy_tolerance_m)),
            float(rospy.get_param("~stair_entry_yaw_tolerance", entry_defaults.yaw_tolerance_rad)),
            entry_defaults.freshness_sec, entry_defaults.max_covariance_x,
            entry_defaults.max_covariance_y, entry_defaults.max_covariance_yaw,
            entry_defaults.required_pose_samples, entry_defaults.max_linear_speed,
            entry_defaults.max_angular_speed,
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
    def floor_anchor(location_id):
        floor = state.floor_state()
        locations = {x.id: x for x in configuration.locations}
        # A logical start selects the floor graph; plan_from_current_pose still
        # navigates from the actual localized pose, not this named point.
        if floor.state != floor.READY or not state.health().healthy:
            return location_id
        if locations[location_id].floor_id == floor.floor_id:
            return location_id
        candidates = [x.id for x in configuration.locations if x.floor_id == floor.floor_id]
        return candidates[0] if candidates else location_id

    orchestrator = MissionOrchestrator(
        planner,
        segment_executor,
        MissionOrchestratorSettings(
            settings.initial_location_id,
            settings.inspect_profile_id,
        ),
        start_from_current_pose=True,
        floor_anchor_resolver=floor_anchor,
    )
    from mission_manager.photo_capture import PhotoCapture
    from mission_manager.photo_mission import PhotoMission
    import yaml, math
    photo_path=settings.roots.mission / 'photo_tour.yaml'
    if photo_path.exists():
        photo=yaml.safe_load(photo_path.read_text())
        required={'locations','image_topic','arrival_radius_m','arrival_yaw_rad','frame_timeout_sec'}
        locations={loc.id:loc for loc in configuration.locations}
        if (set(photo)!=required or not isinstance(photo['locations'],list) or not photo['locations']
                or len(set(photo['locations']))!=len(photo['locations'])
                or any(key not in locations or locations[key].floor_id!='RF' for key in photo['locations'])
                or any(type(photo[k]) not in (int,float) or not math.isfinite(photo[k]) or photo[k]<=0
                       for k in ('arrival_radius_m','arrival_yaw_rad','frame_timeout_sec'))):
            raise ValueError('invalid photo tour settings')
        import hashlib
        def map_identity(floor_id):
            floor=next(x for x in configuration.floors if x.id==floor_id)
            path=settings.roots.multifloor / floor.map_yaml
            raw=path.read_bytes();metadata=yaml.safe_load(raw)
            image=path.parent / metadata['image']
            return hashlib.sha256(raw+image.read_bytes()).hexdigest()
        segment_executor.photo_map_identity=map_identity
        segment_executor.photo_settings=photo
        segment_executor.photo_camera=PhotoCapture(photo['image_topic'],photo['frame_timeout_sec'])
        from mission_manager.mail_outbox import MailOutbox
        photo_mail=MailOutbox(Path(settings.scan_output_base)/'photos')
        orchestrator.photo=PhotoMission(photo,settings.scan_output_base,segment_executor,mail=photo_mail)
        rospy.on_shutdown(orchestrator.photo.capture_stop.set)
        rospy.on_shutdown(photo_mail.close)
    arrival_hold=None
    if rospy.get_param("~arrival_hold_enabled",False):
        from mission_manager.navigation_hold import RosNavigationHold
        arrival_hold=RosNavigationHold(navigation,state,configuration.locations)
    segment_executor.photo_arrival_hold=arrival_hold
    orchestrator.arrival_hold=arrival_hold
    server = MissionActionServer(
        settings.action_name,
        orchestrator,
        frozenset(location.id for location in configuration.locations),
        arrival_hold=arrival_hold,
    )
    console_port = int(rospy.get_param("~console_port", 0))
    if console_port:
        try:
            from mission_manager.ros_console import RosConsole
            html = Path(rospkg.RosPack().get_path("mission_manager")) / "config" / "mission_console.html"
            server.console = RosConsole(planner, orchestrator, configuration, settings.action_name, html, console_port,
                                        tag_topic=rospy.get_param("~tag_topic", "/tag_detections"))
        except Exception as error:
            # An optional display failure must not take down mission execution.
            rospy.logerr("Mission console unavailable; mission server remains active: %s", error)
    return server
