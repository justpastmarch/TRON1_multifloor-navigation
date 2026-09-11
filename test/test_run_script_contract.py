"""Static contract checks for the operator wrapper's node boundary."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "run.sh"


class RunScriptContractTest(unittest.TestCase):
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
