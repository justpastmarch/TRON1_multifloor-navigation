from __future__ import annotations

import json
from pathlib import Path
import subprocess
from typing import TypedDict, Union
import unittest

from test.operator_notebook_support import run_localization_gate


ROOT = Path(__file__).resolve().parents[1]
NOTEBOOKS = ROOT / "notebooks"
ENVIRONMENT_CHECKER = NOTEBOOKS / "check_environment.sh"


class NotebookCell(TypedDict):
    cell_type: str
    source: Union[str, list[str]]


class OperatorMetadata(TypedDict):
    mode: str
    owns: list[str]
    requires: list[str]


class NotebookMetadata(TypedDict):
    tron1_operator: OperatorMetadata


class NotebookDocument(TypedDict):
    cells: list[NotebookCell]
    metadata: NotebookMetadata
    nbformat: int


def load_notebook(name: str) -> NotebookDocument:
    return json.loads((NOTEBOOKS / name).read_text(encoding="utf-8"))


def code_source(notebook: NotebookDocument) -> str:
    return "\n".join(
        "".join(cell["source"])
        for cell in notebook["cells"]
        if cell["cell_type"] == "code"
    )


def localization_gate_source(notebook: NotebookDocument) -> str:
    return next(
        "".join(cell["source"])
        for cell in notebook["cells"]
        if cell["cell_type"] == "code" and "LOCALIZATION GATE: PASS" in "".join(cell["source"])
    )


class OperatorNotebookContractTest(unittest.TestCase):
    def test_read_only_environment_checker_has_valid_cli(self) -> None:
        syntax = subprocess.run(
            ["bash", "-n", str(ENVIRONMENT_CHECKER)],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(syntax.returncode, 0, syntax.stderr)

        help_result = subprocess.run(
            ["bash", str(ENVIRONMENT_CHECKER), "--help"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(help_result.returncode, 0, help_result.stderr)
        for mode in ("common", "mapping", "navigation", "recording"):
            self.assertIn(mode, help_result.stdout)

        invalid = subprocess.run(
            ["bash", str(ENVIRONMENT_CHECKER), "invalid-mode"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(invalid.returncode, 2)

    def test_environment_checker_contains_no_runtime_mutation(self) -> None:
        source = ENVIRONMENT_CHECKER.read_text(encoding="utf-8")

        for forbidden in (
            "rosnode kill",
            "rosnode cleanup",
            "rostopic pub",
            "rosbag record",
            "dynamic_reconfigure",
            "ssh -N",
            "nohup",
            "setsid",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)

    def test_three_operator_notebooks_are_valid_and_separate(self) -> None:
        expected = {
            "01_mapping_and_map_save.ipynb": "mapping",
            "02_pose_and_nav_goal.ipynb": "navigation",
            "03_rosbag_recording.ipynb": "recording",
        }

        self.assertEqual(
            {path.name for path in NOTEBOOKS.glob("*.ipynb")},
            set(expected),
        )
        for name, mode in expected.items():
            with self.subTest(name=name):
                notebook = load_notebook(name)
                self.assertEqual(notebook["nbformat"], 4)
                self.assertEqual(notebook["metadata"]["tron1_operator"]["mode"], mode)
                self.assertGreaterEqual(len(notebook["cells"]), 8)
                self.assertEqual(notebook["cells"][0]["cell_type"], "markdown")

    def test_every_notebook_bootstraps_the_ros_environment_without_installing(self) -> None:
        for path in NOTEBOOKS.glob("*.ipynb"):
            with self.subTest(name=path.name):
                source = code_source(load_notebook(path.name))
                self.assertIn("config.env", source)
                self.assertIn("ROS_MASTER_URI", source)
                self.assertIn("/opt/ros/noetic/setup.bash", source)
                self.assertNotIn("pip install", source)
                self.assertNotIn("%pip", source)

    def test_every_python_cell_compiles(self) -> None:
        for path in NOTEBOOKS.glob("*.ipynb"):
            notebook = load_notebook(path.name)
            for index, cell in enumerate(notebook["cells"]):
                if cell["cell_type"] != "code":
                    continue
                with self.subTest(name=path.name, cell=index):
                    compile("".join(cell["source"]), f"{path.name}:cell-{index}", "exec")

    def test_mapping_notebook_owns_slam_and_refuses_localization_conflicts(self) -> None:
        source = code_source(load_notebook("01_mapping_and_map_save.ipynb"))

        for token in (
            "slam_gmapping",
            "_base_frame:=base_Link",
            "_odom_frame:=odom",
            "_map_frame:=map",
            "map_saver",
            "/map_server",
            "/amcl",
            "/move_base",
            "/scan",
            "/tron/wheel_odom_raw",
            "MAPPING_CONTROLLER_READY = False",
        ):
            self.assertIn(token, source)

    def test_navigation_notebook_requires_explicit_motion_arm_and_action_result(self) -> None:
        source = code_source(load_notebook("02_pose_and_nav_goal.ipynb"))

        for token in (
            "ARM_MOTION = False",
            "/initialpose",
            "PoseWithCovarianceStamped",
            "SimpleActionClient",
            '"/move_base"',
            "MoveBaseGoal",
            "cancel_goal",
            "/stair_supervisor/state",
            "DEFAULT_READINESS_POLICY",
            "required_pose_samples",
            "LOCALIZATION GATE: PASS",
        ):
            self.assertIn(token, source)

    def test_navigation_localization_gate_requests_three_fresh_stationary_samples(self) -> None:
        service_calls, stamps = run_localization_gate(
            localization_gate_source(load_notebook("02_pose_and_nav_goal.ipynb"))
        )

        self.assertEqual(service_calls, 3)
        self.assertEqual(stamps, [101, 102, 103])

    def test_navigation_localization_gate_rejects_stale_and_uncertain_samples(self) -> None:
        source = localization_gate_source(load_notebook("02_pose_and_nav_goal.ipynb"))

        with self.assertRaisesRegex(RuntimeError, "did not advance after nomotion update"):
            run_localization_gate(source, advance_stamps=False)
        with self.assertRaisesRegex(RuntimeError, "covariance gate failed"):
            run_localization_gate(source, covariance=(0.06, 0.04, 0.09))

    def test_navigation_localization_gate_explains_missing_nomotion_service(self) -> None:
        source = localization_gate_source(load_notebook("02_pose_and_nav_goal.ipynb"))

        with self.assertRaisesRegex(RuntimeError, "nomotion update service is unavailable"):
            run_localization_gate(source, service_available=False)
        with self.assertRaisesRegex(RuntimeError, "AMCL pose baseline is unavailable"):
            run_localization_gate(source, messages_available=False)

    def test_recording_notebook_owns_one_recorder_and_validates_final_artifact(self) -> None:
        source = code_source(load_notebook("03_rosbag_recording.ipynb"))

        for token in (
            '"rosbag", "record"',
            "CAMERA_COMPRESSED_TOPIC",
            "TAG_TOPIC",
            "SIGINT",
            'suffix + ".active"',
            '"rosbag", "info", "--yaml"',
            "pgrep",
            "/rosbag/[r]ecord",
            "RECORD_PROCESS.send_signal(signal.SIGINT)",
            'CONFIG["POINTCLOUD_TOPIC"]',
            'CONFIG["CAMERA_IMAGE_TOPIC"]',
            'CONFIG["CAMERA_INFO_TOPIC"]',
            'CONFIG["TAG_DETECTIONS_TOPIC"]',
        ):
            self.assertIn(token, source)
        self.assertNotIn("os.killpg(RECORD_PROCESS", source)


if __name__ == "__main__":
    unittest.main()
