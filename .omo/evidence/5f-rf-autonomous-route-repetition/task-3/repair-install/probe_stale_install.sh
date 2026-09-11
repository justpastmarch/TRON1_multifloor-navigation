#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${script_dir}/../../../../.." && pwd)"
stale_root="$(mktemp -d /tmp/task3-stale-install-XXXXXX)"
trap 'rm -rf "${stale_root}"' EXIT

mkdir -p "${stale_root}/lib/python3/dist-packages/multifloor_manager"
cat >"${stale_root}/setup.bash" <<'SETUP'
_stale_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${_stale_root}/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
unset _stale_root
SETUP
touch "${stale_root}/lib/python3/dist-packages/multifloor_manager/__init__.py"
cat >"${stale_root}/lib/python3/dist-packages/multifloor_manager/configuration.py" <<'PY'
class Stair:
    __slots__ = (
        "id", "from_floor", "to_floor", "entry_location_id", "target_landing",
        "covariance", "expected_tag_ids", "up_profile_id", "down_profile_id",
    )
PY

set +e
"${script_dir}/validate_install_space.sh" \
    "${stale_root}/setup.bash" \
    "${workspace_root}/test/fixtures/building_valid"
status=$?
set -e

if ((status == 0)); then
    echo "ERROR: stale single-endpoint install passed validation" >&2
    exit 1
fi
if ((status != 42)); then
    echo "ERROR: stale install failed for an unrelated reason (exit ${status})" >&2
    exit 1
fi

echo "STALE_INSTALL_REJECTED exit=${status} root=${stale_root}"
