#!/usr/bin/env python3
"""CLI facade for operator-facing ROS bundle validation."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

from bundle_contract_common import fail
from bundle_runtime_contract import (
    normalize_map_image,
    validate_forbidden_runtime,
    validate_launch,
    validate_rviz,
    validate_schema_assets,
)
from bundle_source_contract import (
    validate_actions,
    validate_node_entrypoints,
    validate_paths,
    validate_project_packages,
)


ROOT = Path(__file__).resolve().parent
for PACKAGE_NAME in ("mission_manager", "multifloor_manager", "stair_supervisor"):
    sys.path.insert(0, str(ROOT / "src" / PACKAGE_NAME / "src"))

from mission_manager.site_config import (  # noqa: E402 -- source workspace bootstrap
    ConfigurationError,
    ConfigurationRoots,
    load_site_configuration,
    load_site_configuration_from_roots,
)


def _validate_detector_tags(config_root: Path, tags: tuple) -> None:
    path = config_root / "apriltag_ros_tags.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8")) if path.is_file() else None
    actual = document.get("standalone_tags") if isinstance(document, dict) else None
    expected = [{"id": tag.id, "size": tag.size_m} for tag in tags]
    if actual != expected:
        fail(f"AprilTag detector artifact does not match apriltags.yaml: {path}")


def validate_site_fixture(config_root: Path) -> None:
    try:
        configuration = load_site_configuration(config_root)
    except ConfigurationError as error:
        fail(str(error))
    _validate_detector_tags(config_root, configuration.tags)
    print(f"SITE_CONFIG_OK floors={len(configuration.floors)} locations={len(configuration.locations)} edges={len(configuration.edges)}")


def validate_production_site(root: Path) -> None:
    roots = ConfigurationRoots(root / "src/mission_manager/config", root / "src/multifloor_manager/config", root / "src/stair_supervisor/config")
    try:
        configuration = load_site_configuration_from_roots(roots)
    except ConfigurationError as error:
        fail(str(error))
    _validate_detector_tags(roots.multifloor, configuration.tags)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate the portable bundle")
    parser.add_argument("--normalize-map", action="store_true")
    parser.add_argument("--workspace-root", type=Path, default=ROOT)
    site_group = parser.add_mutually_exclusive_group()
    site_group.add_argument("--site-config-root", type=Path)
    site_group.add_argument("--validate-production-config", action="store_true")
    arguments = parser.parse_args()
    workspace_root = arguments.workspace_root.resolve()
    if arguments.normalize_map:
        normalize_map_image(workspace_root)
    validate_project_packages(workspace_root)
    validate_node_entrypoints(workspace_root)
    validate_paths(workspace_root)
    validate_actions(workspace_root)
    validate_schema_assets(workspace_root)
    validate_launch(workspace_root)
    validate_forbidden_runtime(workspace_root)
    validate_rviz(workspace_root)
    if arguments.site_config_root:
        validate_site_fixture(arguments.site_config_root.resolve())
    if arguments.validate_production_config:
        validate_production_site(workspace_root)
    print("BUNDLE_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
