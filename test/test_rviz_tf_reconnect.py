from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

import rosgraph


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "src" / "multifloor_manager" / "scripts" / "rviz_tf_reconnect.py"
WAIT_SCRIPT = SCRIPT.with_name("wait_for_tf_exec.sh")


class FakeMaster:
    def __init__(self, _caller_id: str, missing_nodes: frozenset[str] = frozenset()) -> None:
        self.missing_nodes = missing_nodes
        self.node_uris = {
            "/amcl": "http://amcl:1111/",
            "/move_base": "http://move-base:2222/",
            "/rviz_navigation": "http://rviz:3333/",
            "/tf_source": "http://tf-source:4444/",
        }

    def lookupNode(self, node: str) -> str:
        if node in self.missing_nodes:
            raise rosgraph.MasterError(f"unknown node {node}")
        return self.node_uris[node]

    def getSystemState(self):
        return ([('/tf', ['/tf_source'])], [], [])


class FakeServer:
    def __init__(self, uri: str, calls: list[tuple[str, str, str, list[str]]]) -> None:
        self.uri = uri
        self.calls = calls

    def publisherUpdate(self, caller_id: str, topic: str, publishers: list[str]):
        self.calls.append((self.uri, caller_id, topic, publishers))
        return 1, "ok", 0


class RvizTfReconnectTest(unittest.TestCase):
    def load_module(self):
        spec = importlib.util.spec_from_file_location("rviz_tf_reconnect", SCRIPT)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_existing_tf_publishers_are_sent_to_rviz(self) -> None:
        # Given: RViz and one current TF publisher are registered with the master.
        module = self.load_module()
        calls: list[tuple[str, str, str, list[str]]] = []
        module.rosgraph.Master = FakeMaster
        module.ServerProxy = lambda uri: FakeServer(uri, calls)

        # When: the one-shot reconnect helper runs.
        result = module.main()

        # Then: the existing RViz TF publisher update is preserved.
        self.assertEqual(result, 0)
        self.assertIn(
            (
                "http://rviz:3333/",
                "/rviz_tf_reconnect",
                "/tf",
                ["http://tf-source:4444/"],
            ),
            calls,
        )
        self.assertIn(
            (
                "http://amcl:1111/",
                "/rviz_tf_reconnect",
                "/initialpose",
                ["http://rviz:3333/"],
            ),
            calls,
        )
        self.assertIn(
            (
                "http://move-base:2222/",
                "/rviz_tf_reconnect",
                "/move_base_simple/goal",
                ["http://rviz:3333/"],
            ),
            calls,
        )

    def test_missing_required_nodes_exit_nonzero_for_the_nonfatal_launch_gate(self) -> None:
        for missing_node in ("/rviz_navigation", "/amcl", "/move_base"):
            with self.subTest(missing_node=missing_node):
                # Given: one required endpoint is absent from the ROS master.
                module = self.load_module()
                calls: list[tuple[str, str, str, list[str]]] = []
                module.rosgraph.Master = lambda caller_id: FakeMaster(
                    caller_id,
                    frozenset({missing_node}),
                )
                module.ServerProxy = lambda uri: FakeServer(uri, calls)

                # When/Then: the one-shot process fails closed instead of reporting success.
                with self.assertRaises(rosgraph.MasterError):
                    module.main()
        wait_source = WAIT_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('if ! python3 "$script_dir/rviz_tf_reconnect.py"; then', wait_source)
        self.assertLess(wait_source.index("rviz_tf_reconnect.py"), wait_source.index('exec "$@"'))


if __name__ == "__main__":
    unittest.main()
