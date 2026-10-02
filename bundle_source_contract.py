"""Validate project package, entrypoint, path, and action source contracts."""

from __future__ import annotations

import ast
from pathlib import Path
import stat

from bundle_contract_common import fail, parse_xml, project_files


EXPECTED_PACKAGES = frozenset({"mission_manager", "multifloor_manager", "stair_supervisor"})
EXPECTED_NODE_INITIALIZERS = frozenset(
    {
        ("src/mission_manager/scripts/mission_manager_node.py", "mission_manager"),
        ("src/multifloor_manager/scripts/camera_info_stamp_relay.py", "camera_info_stamp_relay"),
        ("src/multifloor_manager/scripts/multifloor_manager_node.py", "multifloor_manager"),
        ("src/stair_supervisor/scripts/stair_supervisor_node.py", "stair_supervisor"),
    }
)
# Existing one-shot operator client; it must never enter a launch graph.
OPERATOR_CLIENT_INITIALIZERS = frozenset({
    ("src/stair_supervisor/scripts/stair_entry_test.py", "stair_entry_test_client"),
    ("src/stair_supervisor/scripts/prepare_stair_mission.py", "prepare_stair_mission"),
})
TEXT_SUFFIXES = frozenset({".env", ".sh", ".py", ".launch", ".yaml", ".rviz", ".xml"})
ACTION_CONTRACTS = {
    "src/mission_manager/action/Mission.action": (
        "string destination_id", "string mission_type", "bool return_after_task", "---",
        "uint8 OK=0", "uint8 BUSY=1", "uint8 INVALID_GOAL=2", "uint8 CAPABILITY_DISABLED=3",
        "uint8 NAVIGATION_FAILED=4", "uint8 STAIR_FAILED=5", "uint8 LOCALIZATION_FAILED=6",
        "uint8 SCAN_FAILED=7", "uint8 COMMUNICATION_LOST=8", "uint8 MISSION_ABORT=9",
        "uint8 result_code", "string reason", "string mission_id", "string artifact_path", "---",
        "string mission_id", "string state", "string current_floor", "string segment_type",
        "uint32 segment_index", "uint32 segment_count", "string target_id",
    ),
    "src/multifloor_manager/action/FloorTransition.action": (
        "string transition_id", "string target_floor", "uint64 stair_ownership_epoch", "---",
        "uint8 OK=0", "uint8 BUSY=1",
        "uint8 INVALID_GOAL=2", "uint8 LOCALIZATION_FAILED=6", "uint8 result_code",
        "string floor_id", "uint64 map_generation", "string reason", "---", "string phase", "string detail",
    ),
    "src/stair_supervisor/action/StairTraversal.action": (
        "string stair_id", "uint8 UP=1", "uint8 DOWN=2", "uint8 direction",
        "string admission_token", "---", "uint8 OK=0",
        "uint8 BUSY=1", "uint8 INVALID_GOAL=2", "uint8 CAPABILITY_DISABLED=3",
        "uint8 ENTRY_REJECTED=4", "uint8 STAIR_FAILED=5", "uint8 COMMUNICATION_LOST=8",
        "uint8 result_code", "string reason", "uint64 ownership_epoch", "---",
        "string phase", "string detail",
    ),
}


def validate_project_packages(root: Path) -> None:
    manifests = tuple(sorted((root / "src").glob("*/package.xml")))
    package_names = tuple(parse_xml(path, root).findtext("name") for path in manifests)
    if set(package_names) != EXPECTED_PACKAGES or len(package_names) != len(EXPECTED_PACKAGES):
        fail(f"project package mismatch; expected={sorted(EXPECTED_PACKAGES)} actual={sorted(package_names)}")
    for package_name in EXPECTED_PACKAGES:
        package_root = root / "src" / package_name
        script = package_root / "scripts" / f"{package_name}_node.py"
        required = (package_root / "CMakeLists.txt", package_root / "setup.py", script)
        missing = [str(path.relative_to(root)) for path in required if not path.is_file()]
        if missing:
            fail(f"package contract missing for {package_name}: {missing}")
        if not script.stat().st_mode & stat.S_IXUSR:
            fail(f"node entrypoint is not executable: {script.relative_to(root)}")


def validate_node_entrypoints(root: Path) -> None:
    initializers: set[tuple[str, str | None]] = set()
    for source_path in project_files(root):
        if source_path.suffix != ".py" or "test" in source_path.relative_to(root).parts:
            continue
        try:
            tree = ast.parse(source_path.read_text(encoding="utf-8"))
        except (OSError, SyntaxError) as error:
            fail(f"invalid Python {source_path.relative_to(root)}: {error}")
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr != "init_node" or not isinstance(node.func.value, ast.Name) or node.func.value.id != "rospy":
                continue
            name = node.args[0].value if node.args and isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str) else None
            initializers.add((source_path.relative_to(root).as_posix(), name))
    runtime_initializers = initializers - OPERATOR_CLIENT_INITIALIZERS
    if runtime_initializers != EXPECTED_NODE_INITIALIZERS:
        fail(f"project node entrypoint mismatch; expected={sorted(EXPECTED_NODE_INITIALIZERS)} actual={sorted(initializers)}")
    clients = {Path(path).name for path, _name in initializers & OPERATOR_CLIENT_INITIALIZERS}
    for launch_path in project_files(root):
        if launch_path.suffix != ".launch" or "test" in launch_path.relative_to(root).parts:
            continue
        for node in parse_xml(launch_path, root).findall(".//node"):
            if Path(node.get("type", "")).name in clients:
                fail(f"operator client must not be auto-launched: {launch_path.relative_to(root)}")


def validate_paths(root: Path) -> None:
    checkout_markers = ("/home/" + "m3tron", "tron1-" + "control-center")
    parent_traversal = "." * 2 + "/"
    for path in project_files(root):
        # Archived reports and their reproduction tools are not runtime assets.
        # Source/config/launch files retain the original portable-path contract.
        if path.relative_to(root).parts[0] == "docs":
            continue
        if path.suffix not in TEXT_SUFFIXES:
            continue
        text = path.read_text(encoding="utf-8")
        if any(marker in text for marker in checkout_markers):
            fail(f"checkout-specific path in {path.relative_to(root)}")
        if parent_traversal in text:
            fail(f"parent traversal in {path.relative_to(root)}")


def validate_actions(root: Path) -> None:
    for relative_path, expected in ACTION_CONTRACTS.items():
        path = root / relative_path
        if not path.is_file():
            fail(f"action contract missing: {relative_path}")
        actual = tuple(line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip() and not line.lstrip().startswith("#"))
        if actual != expected:
            fail(f"action schema mismatch: {relative_path}")
