"""Static contract checks for the operator wrapper's node boundary."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "run.sh"
CONFIG = ROOT / "config.env"


class RunScriptContractTest(unittest.TestCase):
    def test_workstation_owns_ros_master_on_shared_lan(self) -> None:
        # Given: the deployed workstation and mini-PC LAN addresses.
        values = dict(
            line.split("=", 1)
            for line in CONFIG.read_text(encoding="utf-8").splitlines()
            if line and not line.startswith("#") and "=" in line
        )

        # When/Then: ROS master and both advertised node addresses use the LAN
        # where callbacks are bidirectionally reachable.
        self.assertEqual(values["ROS_MASTER_HOST"], "192.168.1.26")
        self.assertEqual(values["MINI_PC_ROS_IP"], "192.168.1.56")

    def _network(self, choice, reachable=(), tailscale_status=''):
        # Source config.env with a stub ping that answers only for `reachable`.
        with tempfile.TemporaryDirectory() as stub:
            ping = Path(stub) / "ping"
            ping.write_text('#!/bin/sh\nfor a; do last="$a"; done\ncase " %s " in *" $last "*) exit 0;; esac\nexit 1\n'
                            % " ".join(reachable))
            ping.chmod(0o755)
            status = Path(stub) / "tailscale"
            status.write_text("#!/bin/sh\nprintf '%s'\n" % tailscale_status.replace("'", ""))
            status.chmod(0o755)
            env = dict(os.environ, PATH=stub + os.pathsep + os.environ["PATH"])
            env.pop("TRON_NETWORK", None)
            if choice:
                env["TRON_NETWORK"] = choice
            out = subprocess.run(
                ["bash", "-c", 'source "$1"; printf "%s %s %s %s" "$TRON_NETWORK" "$MINI_PC_HOST" '
                 '"$ROS_MASTER_HOST" "$MINI_PC_ROS_IP"', "_", str(CONFIG)],
                env=env, check=True, capture_output=True, text=True).stdout
        return out.split()

    def test_network_path_prefers_lan_then_tailscale(self) -> None:
        lan = ["lan", "192.168.1.56", "192.168.1.26", "192.168.1.56"]
        tail = ["tailscale", "100.86.246.109", "100.78.87.6", "100.86.246.109"]
        self.assertEqual(self._network(None, ["192.168.1.56", "100.86.246.109"]), lan)
        self.assertEqual(self._network(None, ["100.86.246.109"]), tail)
        # Nothing answers (mini PC booting): keep the LAN values so run.sh waits as before.
        self.assertEqual(self._network(None, []), lan)
        self.assertEqual(self._network("tailscale", ["192.168.1.56"]), tail)
        self.assertEqual(self._network("lan", ["100.86.246.109"]), lan)
        # Pings lost on a weak hotspot, but Tailscale lists the mini PC as connected.
        self.assertEqual(self._network(None, [], "100.86.246.109  minipc  user  linux  active; relay tok\\n"), tail)
        self.assertEqual(self._network(None, [], "100.86.246.109  minipc  user  linux  offline\\n"), lan)

    def test_wrapper_owns_master_and_probes_complete_action_connections(self) -> None:
        # Given: the production startup path.
        source = RUN.read_text(encoding="utf-8")

        # When/Then: the wrapper starts the selected master before sensors and
        # refuses READY unless real actionlib clients connect in both directions.
        master_start = source.index('start_component ros_master roscore')
        sensor_start = source.index('SENSOR_STACK_ACTION="')
        self.assertLess(master_start, sensor_start)
        self.assertIn('python3 "${ROOT}/verify_action_servers.py"', source)

    def test_remote_mapping_status_is_not_expanded_by_local_strict_shell(self) -> None:
        # Given: the SSH command that starts or reuses the mini-PC sensor stack.
        source = RUN.read_text(encoding="utf-8")
        remote_command = source.split('SENSOR_STACK_ACTION="', 1)[1].split(
            "\nprintf '[SSH] mini PC sensor stack:",
            1,
        )[0]

        # When/Then: the remote-only variable remains escaped in the local string.
        self.assertIn(r'\$mapping_status', remote_command)

    def test_remote_sensor_probe_avoids_pid_expansion_and_requires_camera_frames(self) -> None:
        # Given: the SSH command that decides whether the mini-PC stack is reusable.
        source = RUN.read_text(encoding="utf-8")
        remote_command = source.split('SENSOR_STACK_ACTION="', 1)[1].split(
            "\nprintf '[SSH] mini PC sensor stack:",
            1,
        )[0]

        # When/Then: local shell PID expansion cannot corrupt remote topic matches,
        # and a live LiDAR/odometry subset cannot hide a stalled RealSense stream.
        self.assertNotIn("/livox/lidar$$", remote_command)
        self.assertIn("/livox/lidar", remote_command)
        self.assertIn("${CAMERA_IMAGE_TOPIC}", remote_command)
        self.assertIn("${CAMERA_INFO_TOPIC}", remote_command)
        self.assertIn("rostopic echo -n 1", remote_command)

    def test_remote_sensor_restart_cannot_kill_its_own_ssh_shell(self) -> None:
        # Given: the SSH command that terminates a stale sensor roslaunch.
        source = RUN.read_text(encoding="utf-8")
        remote_command = source.split('SENSOR_STACK_ACTION="', 1)[1].split(
            "\nprintf '[SSH] mini PC sensor stack:",
            1,
        )[0]

        # When/Then: pkill matches only the complete roslaunch process command,
        # not the containing SSH shell that also carries the word roslaunch.
        expected = (
            "^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch "
            "sensor_integration wf_mapping.launch$"
        )
        self.assertIn(expected, remote_command)
        self.assertNotIn("-f '[r]oslaunch sensor_integration", remote_command)

    def test_action_readiness_uses_action_server_gate_instead_of_frequency(self) -> None:
        # Given: the readiness gates in the operator wrapper.
        source = RUN.read_text(encoding="utf-8")

        action_gate = source.split("for topic in /mission/status", 1)[1].split("done", 1)[0]

        # When/Then: each listed action server waits for its GoalStatusArray topic
        # to appear and receive at least one message, instead of using rostopic hz.
        self.assertIn(
            'wait_for_action_server "$topic"',
            action_gate,
        )
        self.assertNotIn('wait_for_stream "$topic"', action_gate)
        self.assertNotIn('wait_for_fresh_message "$topic"', action_gate)

    def test_freshness_probe_does_not_initialize_a_fourth_ros_node(self) -> None:
        # Given: the single operator wrapper source.
        source = RUN.read_text(encoding="utf-8")

        # Then: freshness checks remain CLI-based and cannot create a ROS node.
        self.assertNotIn("rospy.init_node", source)
        self.assertIn("rostopic echo -n 1", source)
        self.assertIn("rostopic type", source)

    def test_replay_uses_bounded_filtered_preview_topic(self) -> None:
        # Given: a finalized bag and an observable rosbag-play boundary.
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "route.bag"
            bag.touch()
            arguments = root / "arguments"
            executable = root / "rosbag"
            executable.write_text(
                "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\" >\"$CAPTURE_ARGS\"\n",
                encoding="utf-8",
            )
            executable.chmod(0o755)
            environment = dict(
                os.environ,
                PATH=f"{root}:{os.environ['PATH']}",
                CAPTURE_ARGS=str(arguments),
            )

            # When: the operator requests a short, slower-than-recorded preview.
            result = subprocess.run(
                (str(RUN), "--replay-joy", str(bag), "2", "5", "0.5"),
                cwd=ROOT,
                env=environment,
                check=False,
            )

            # Then: rosbag receives only the bounded source topic under /replay.
            self.assertEqual(result.returncode, 0)
            self.assertEqual(
                arguments.read_text(encoding="utf-8").splitlines(),
                [
                    "play",
                    str(bag),
                    "--quiet",
                    "--prefix=/replay",
                    "--delay=0.5",
                    "--start=2",
                    "--duration=5",
                    "--rate=0.5",
                    "--topics",
                    "/tron/sensor_joy",
                ],
            )

    def test_replay_rejects_duration_above_safety_bound(self) -> None:
        # Given: a finalized bag and a command probe that must remain untouched.
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "route.bag"
            bag.touch()
            arguments = root / "arguments"
            executable = root / "rosbag"
            executable.write_text(
                "#!/usr/bin/env bash\ntouch \"$CAPTURE_ARGS\"\n",
                encoding="utf-8",
            )
            executable.chmod(0o755)
            environment = dict(
                os.environ,
                PATH=f"{root}:{os.environ['PATH']}",
                CAPTURE_ARGS=str(arguments),
            )

            # When: replay duration exceeds the short-preview ceiling.
            result = subprocess.run(
                (str(RUN), "--replay-joy", str(bag), "0", "31", "1"),
                cwd=ROOT,
                env=environment,
                check=False,
            )

            # Then: validation rejects it before rosbag can publish anything.
            self.assertEqual(result.returncode, 2)
            self.assertFalse(arguments.exists())

    def test_replay_ignores_environment_attempt_to_select_live_command_topic(self) -> None:
        # Given: a finalized bag and a hostile source-topic environment override.
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "route.bag"
            bag.touch()
            arguments = root / "arguments"
            executable = root / "rosbag"
            executable.write_text(
                "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\" >\"$CAPTURE_ARGS\"\n",
                encoding="utf-8",
            )
            executable.chmod(0o755)
            environment = dict(
                os.environ,
                PATH=f"{root}:{os.environ['PATH']}",
                CAPTURE_ARGS=str(arguments),
                SENSOR_JOY_TOPIC="/navigation/cmd_vel",
            )

            # When: preview is invoked through the operator wrapper.
            result = subprocess.run(
                (str(RUN), "--replay-joy", str(bag)),
                cwd=ROOT,
                env=environment,
                check=False,
            )

            # Then: only recorded Joy can be selected and it remains under /replay.
            self.assertEqual(result.returncode, 0)
            values = arguments.read_text(encoding="utf-8").splitlines()
            self.assertIn("--prefix=/replay", values)
            self.assertEqual(values[-2:], ["--topics", "/tron/sensor_joy"])

    def test_replay_rejects_ros_remap_arguments_before_rosbag(self) -> None:
        # Given: a finalized bag and a rosbag probe that must remain untouched.
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "route.bag"
            bag.touch()
            arguments = root / "arguments"
            executable = root / "rosbag"
            executable.write_text(
                "#!/usr/bin/env bash\ntouch \"$CAPTURE_ARGS\"\n",
                encoding="utf-8",
            )
            executable.chmod(0o755)
            environment = dict(
                os.environ,
                PATH=f"{root}:{os.environ['PATH']}",
                CAPTURE_ARGS=str(arguments),
            )

            # When: an extra remap targets each forbidden live command surface.
            forbidden = (
                "/tron/sensor_joy",
                "/navigation/cmd_vel",
                "/stair/cmd_vel",
                "/stair_supervisor/websocket_tx",
            )
            for destination in forbidden:
                with self.subTest(destination=destination):
                    result = subprocess.run(
                        (
                            str(RUN),
                            "--replay-joy",
                            str(bag),
                            "0",
                            "5",
                            "1",
                            f"/tron/sensor_joy:={destination}",
                        ),
                        cwd=ROOT,
                        env=environment,
                        check=False,
                    )
                    self.assertEqual(result.returncode, 2)

            # Then: validation rejects every remap before rosbag can run.
            self.assertFalse(arguments.exists())


if __name__ == "__main__":
    unittest.main()
