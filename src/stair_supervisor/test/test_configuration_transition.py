from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

import yaml


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stair_supervisor.configuration import (  # noqa: E402
    Direction,
    MoveBaseLimits,
    RobotConfiguration,
    StairConfigurationError,
    StairSupervisorConfiguration,
    WebSocketCalibration,
    load_stair_configuration,
)
from stair_supervisor.supervisor import (  # noqa: E402
    ResultCode,
    StairGoal,
    StairSupervisor,
)


class FakeClock:
    def monotonic(self) -> float:
        return 100.0


class RecordingTransport:
    def __init__(self) -> None:
        self.events: list[tuple] = []
        self.stream_period_sec = 0.025

    def start(self) -> None:
        self.events.append(("start",))

    def update_twist(self, linear_mps: float, angular_radps: float) -> None:
        self.events.append(("update", linear_mps, angular_radps))

    def send_current(self) -> None:
        self.events.append(("send",))

    def request_stair_mode(self, enabled: bool) -> None:
        self.events.append(("stair", enabled))

    def close(self) -> None:
        self.events.append(("close",))


def robot_yaml(configured: bool = True) -> str:
    return f"""schema_version: 1
configured: {str(configured).lower()}
move_base:
  max_vel_x: 0.35
  max_vel_theta: 1.0
  min_in_place_vel_theta: 0.2
  acc_lim_x: 0.4
  acc_lim_theta: 2.0
websocket_full_scale:
  linear_mps: 0.55
  angular_radps: 1.8
command_topics:
  - /navigation/cmd_vel
"""


def write_root(profiles: str, robot: str) -> Path:
    root = Path(tempfile.mkdtemp())
    (root / "stair_profiles.yaml").write_text(profiles, encoding="utf-8")
    (root / "robot.yaml").write_text(robot, encoding="utf-8")
    return root


def enabled_profile_yaml() -> str:
    return """schema_version: 1
configured: false
profiles:
  - id: test_up
    direction: UP
    enabled: true
    linear_speed: 0.12
    angular_speed: 0.25
    timeout_sec: 2.0
"""


def empty_configuration() -> StairSupervisorConfiguration:
    robot = RobotConfiguration(
        MoveBaseLimits(0.35, 1.0, 0.2, 0.4, 2.0),
        WebSocketCalibration(0.55, 1.8),
        ("/navigation/cmd_vel",),
    )
    return StairSupervisorConfiguration((), robot)


class StairConfigurationTransitionTest(unittest.TestCase):
    def test_required_measured_profile_field_is_rejected_when_missing(self) -> None:
        # Given: a configured fixture profile with one continuity bound removed.
        fixture = Path(__file__).resolve().parents[3] / "test" / "fixtures" / "building_valid" / "stair_profiles.yaml"
        document = yaml.safe_load(fixture.read_text(encoding="utf-8"))
        del document["profiles"][0]["max_yaw_step_rad"]
        root = write_root(yaml.safe_dump(document), robot_yaml())

        # When/Then: parsing rejects the incomplete measured profile at the boundary.
        with self.assertRaises(StairConfigurationError) as context:
            load_stair_configuration(root)
        self.assertEqual(context.exception.field, "profiles[0]")

    def test_sample_gap_above_freshness_is_rejected(self) -> None:
        # Given: the continuity bound is looser than the freshness gate.
        fixture = Path(__file__).resolve().parents[3] / "test" / "fixtures" / "building_valid" / "stair_profiles.yaml"
        document = yaml.safe_load(fixture.read_text(encoding="utf-8"))
        document["profiles"][0]["sensor_freshness_sec"] = 0.10
        root = write_root(yaml.safe_dump(document), robot_yaml())

        # When/Then: the malformed sensor policy cannot enter runtime.
        with self.assertRaises(StairConfigurationError) as context:
            load_stair_configuration(root)
        self.assertEqual(context.exception.field, "profiles[0].max_sample_gap_sec")

    def test_disabled_nonempty_stair_profiles_reject(self) -> None:
        # Given: a disabled stair document that still contains a profile.
        root = write_root(enabled_profile_yaml(), robot_yaml())

        # When: the complete stair configuration is loaded.
        with self.assertRaises(StairConfigurationError) as context:
            load_stair_configuration(root)

        # Then: disabled nonempty stair profiles are rejected.
        self.assertEqual(context.exception.field, "profiles")

    def test_robot_configured_false_rejects(self) -> None:
        # Given: empty disabled stair profiles and an unconfigured robot document.
        root = write_root(
            "schema_version: 1\nconfigured: false\nprofiles: []\n",
            robot_yaml(configured=False),
        )

        # When: the complete configuration is loaded.
        with self.assertRaises(StairConfigurationError) as context:
            load_stair_configuration(root)

        # Then: the robot configuration remains a hard safety rejection.
        self.assertEqual(context.exception.file_name, "robot.yaml")
        self.assertEqual(context.exception.field, "configured")

    def test_disabled_empty_profiles_load_as_empty_tuple(self) -> None:
        # Given: the deployment stair profile is disabled with no profiles.
        root = write_root(
            "schema_version: 1\nconfigured: false\nprofiles: []\n",
            robot_yaml(),
        )

        # When: the complete configuration is loaded.
        configuration = load_stair_configuration(root)

        # Then: disabled stairs load with an empty immutable profile set.
        self.assertEqual(configuration.profiles, ())

    def test_empty_profiles_reject_before_ownership_state_or_motion(self) -> None:
        # Given: a supervisor with no available stair profiles.
        transport = RecordingTransport()
        supervisor = StairSupervisor(
            empty_configuration(), transport, lambda: False, FakeClock(), lambda _phase: None, 0.25
        )

        supervisor.start()
        before_events = list(transport.events)
        before_state = supervisor.state
        before_epoch = supervisor.ownership_epoch

        # When: a stair goal is requested while NAV owns the transport.
        result = supervisor.traverse(StairGoal("missing", Direction.UP), lambda: False)

        # Then: capability rejection leaves NAV ownership and transport untouched.
        self.assertEqual(result.code, ResultCode.CAPABILITY_DISABLED)
        self.assertEqual(supervisor.state, before_state)
        self.assertEqual(supervisor.ownership_epoch, before_epoch)
        self.assertEqual(transport.events, before_events)
        self.assertFalse(any(event[0] == "stair" for event in transport.events[len(before_events):]))
        self.assertFalse(any(event[0] == "update" and event[1:] != (0.0, 0.0) for event in transport.events[len(before_events):]))

    def test_empty_profiles_reject_before_existing_traversal_lock(self) -> None:
        # Given: an empty-profile supervisor whose traversal lock is already held.
        transport = RecordingTransport()
        supervisor = StairSupervisor(
            empty_configuration(), transport, lambda: False, FakeClock(), lambda _phase: None, 0.25
        )
        supervisor._traversal_lock.acquire()
        try:
            # When: a stair goal is requested while another traversal owns the lock.
            result = supervisor.traverse(StairGoal("missing", Direction.UP), lambda: False)
        finally:
            supervisor._traversal_lock.release()

        # Then: capability rejection precedes traversal-lock ownership.
        self.assertEqual(result.code, ResultCode.CAPABILITY_DISABLED)

    def test_fresh_nav_forwards_through_existing_transport_seam(self) -> None:
        # Given: a started empty-profile supervisor and a fresh NAV command.
        transport = RecordingTransport()
        supervisor = StairSupervisor(
            empty_configuration(), transport, lambda: False, FakeClock(), lambda _phase: None, 0.25
        )
        supervisor.start()
        supervisor.accept_navigation(0.2, -0.1)

        # When: the NAV stream tick reaches the transport seam.
        supervisor.stream_navigation()

        # Then: the fresh command is forwarded unchanged.
        self.assertIn(("update", 0.2, -0.1), transport.events)


if __name__ == "__main__":
    unittest.main()
