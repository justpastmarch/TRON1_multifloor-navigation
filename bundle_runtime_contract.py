"""Validate deployment schemas, launch composition, maps, and managed RViz."""

from __future__ import annotations

from pathlib import Path

import yaml

from bundle_contract_common import GENERATED_DIRS, fail, parse_xml
from bundle_source_contract import EXPECTED_PACKAGES


def validate_schema_assets(root: Path) -> None:
    documents = {
        "src/mission_manager/config/locations.yaml": "locations",
        "src/mission_manager/config/building_graph.yaml": "edges",
        "src/mission_manager/config/scan_profiles.yaml": "profiles",
        "src/multifloor_manager/config/floors.yaml": "floors",
        "src/multifloor_manager/config/stairs.yaml": "stairs",
        "src/multifloor_manager/config/apriltags.yaml": "tags",
        "src/multifloor_manager/config/transitions.yaml": "transitions",
        "src/stair_supervisor/config/stair_profiles.yaml": "profiles",
        "src/stair_supervisor/config/robot.yaml": "command_topics",
    }
    for relative_path, collection in documents.items():
        path = root / relative_path
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            fail(f"invalid schema document {relative_path}: {error}")
        if not isinstance(document, dict) or document.get("schema_version") != 1:
            fail(f"schema_version must be 1: {relative_path}")
        if not isinstance(document.get("configured"), bool) or not isinstance(document.get(collection), list):
            fail(f"schema fields are invalid: {relative_path}")
    map_path = root / "src/multifloor_manager/config/maps/floor_3F.yaml"
    try:
        map_document = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        fail(f"invalid map YAML: {error}")
    image = map_document.get("image") if isinstance(map_document, dict) else None
    if not isinstance(image, str) or Path(image).name != image or not (map_path.parent / image).is_file():
        fail("map image must exist as a sibling filename")
    detector_path = root / "src/multifloor_manager/config/apriltag_ros_tags.yaml"
    detector = yaml.safe_load(detector_path.read_text(encoding="utf-8")) if detector_path.is_file() else None
    if not isinstance(detector, dict) or not isinstance(detector.get("standalone_tags"), list):
        fail("AprilTag detector artifact is missing or invalid")


def validate_launch(root: Path) -> None:
    launch_root = root / "src/multifloor_manager/launch"
    navigation = parse_xml(launch_root / "navigation.launch", root)
    navigation_nodes = {(node.get("pkg"), node.get("type")) for node in navigation.findall("node")}
    expected_navigation = {("map_server", "map_server"), ("amcl", "amcl"), ("move_base", "move_base")}
    if navigation_nodes != expected_navigation:
        fail(f"unexpected navigation nodes: {sorted(navigation_nodes)}")
    system = parse_xml(root / "src/mission_manager/launch/system.launch", root)
    includes = {include.get("file") for include in system.findall("include")}
    expected_includes = {f"$(find multifloor_manager)/launch/{name}" for name in ("navigation.launch", "apriltag.launch")}
    if includes != expected_includes:
        fail(f"system launch include mismatch: {sorted(includes)}")
    direct_nodes = {(node.get("pkg"), node.get("type"), node.get("name")) for node in system.findall("node")}
    expected_nodes = {(name, f"{name}_node.py", name) for name in EXPECTED_PACKAGES} | {("rviz", "rviz", "rviz")}
    if direct_nodes != expected_nodes:
        fail(f"system launch node mismatch: {sorted(direct_nodes)}")
    # /scan is produced by the mini PC sensor stack (mid360s_laserscan).
    # The local launch graph must not add a second publisher.
    scan_publishers = []
    for launch_path in launch_root.glob("*.launch"):
        for node in parse_xml(launch_path, root).findall(".//node"):
            if any(remap.get("from") == "scan" and remap.get("to") == "/scan" for remap in node.findall("remap")):
                scan_publishers.append((node.get("pkg"), node.get("type")))
    if scan_publishers:
        fail(f"local /scan publisher declarations must be empty; mini PC owns /scan: {scan_publishers}")


def validate_forbidden_runtime(root: Path) -> None:
    forbidden_nodes = ("cmd_vel_bridge", "safety_node", "localization_manager", "sensor_fusion", "ui_node", "cmd_vel_mux")
    for launch_path in (root / "src").rglob("*.launch"):
        relative = launch_path.relative_to(root)
        if GENERATED_DIRS.intersection(relative.parts) or "test" in relative.parts:
            continue
        launch = parse_xml(launch_path, root)
        for node in launch.findall(".//node"):
            identity = " ".join(filter(None, (node.get("pkg"), node.get("type"), node.get("name")))).lower()
            if any(name in identity for name in forbidden_nodes):
                fail(f"forbidden runtime node in {relative}: {identity}")
    for path in (root / "src").rglob("*"):
        if not path.is_file() or path.suffix not in {".launch", ".yaml", ".xml"}:
            continue
        relative = path.relative_to(root)
        if GENERATED_DIRS.intersection(relative.parts) or "test" in relative.parts:
            continue
        source = path.read_text(encoding="utf-8").lower().replace("-", "_")
        if "fast_lio" in source:
            fail(f"FAST-LIO navigation reference in {relative}")


def normalize_map_image(root: Path) -> None:
    map_path = root / "src/multifloor_manager/config/maps/floor_3F.yaml"
    document = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    image = document.get("image") if isinstance(document, dict) else None
    if not isinstance(image, str) or not image:
        fail("map image is missing")
    image_name = Path(image).name
    if not (map_path.parent / image_name).is_file():
        fail(f"map image is missing: {image_name}")
    if image != image_name:
        document["image"] = image_name
        map_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        print(f"MAP_NORMALIZED image={image_name}")


def validate_rviz(root: Path) -> None:
    text = (root / "src/multifloor_manager/rviz/wf_navigation.rviz").read_text(encoding="utf-8")
    required = {"Fixed Frame: map", "Class: rviz/Map", "Topic: /scan", "Class: rviz/PoseWithCovariance", "Class: rviz/Path", "Class: rviz/TF"}
    forbidden = {"Class: rviz/SetInitialPose", "Class: rviz/SetGoal", "/move_base_simple/goal"}
    missing = sorted(token for token in required if token not in text)
    present = sorted(token for token in forbidden if token in text)
    if missing or present:
        fail(f"RViz visualization contract mismatch; missing={missing} forbidden={present}")
