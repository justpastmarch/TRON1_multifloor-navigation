#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${script_dir}/../../../../.." && pwd)"
setup_file="${1:-${workspace_root}/install/setup.bash}"
fixture_root="${2:-${workspace_root}/test/fixtures/building_valid}"
install_root="$(cd "$(dirname "${setup_file}")" && pwd)"

source "${setup_file}"
PYTHONNOUSERSITE=1 python3 - "${fixture_root}" "${install_root}" <<'PY'
from pathlib import Path
import sys

import multifloor_manager.configuration as multifloor_configuration
if not hasattr(multifloor_configuration, "StairEndpoint"):
    print("STALE_INSTALL_SCHEMA: missing StairEndpoint", file=sys.stderr)
    raise SystemExit(42)

from multifloor_manager.configuration import StairEndpoint
import mission_manager.site_config as site_config

fixture_root = Path(sys.argv[1]).resolve()
package_root = Path(sys.argv[2]).resolve() / "lib/python3/dist-packages"
for module in (multifloor_configuration, site_config):
    module_path = Path(module.__file__).resolve()
    try:
        module_path.relative_to(package_root)
    except ValueError:
        raise SystemExit(f"module escaped selected install space: {module_path}") from None

configuration = site_config.load_site_configuration(fixture_root)
for stair in configuration.stairs:
    endpoint_floors = frozenset(endpoint.floor_id for endpoint in stair.endpoints)
    expected_floors = frozenset((stair.from_floor, stair.to_floor))
    if endpoint_floors != expected_floors:
        raise SystemExit(f"stair {stair.id} endpoint floors do not match its floor pair")
    if not all(isinstance(endpoint, StairEndpoint) for endpoint in stair.endpoints):
        raise SystemExit(f"stair {stair.id} contains a retired endpoint representation")

print(f"INSTALL_SCHEMA_OK stairs={len(configuration.stairs)} package_root={package_root}")
PY
