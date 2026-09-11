# DoneClaim: Task 3 install-space repair

## Outcome

- **Status:** done; ready for independent install-space verification.
- **Evidence scope:** this receipt supplements the earlier `task-3/DONE_CLAIM.md` and is canonical for install-space deployment status.
- **Repair:** `catkin_make install` was sufficient. Existing `catkin_python_setup()` rules copied the current source modules into `install/lib/python3/dist-packages`; no product, CMake, ROS action, message, launch-topology, or compatibility-shim change was needed.
- **Persistent additions:** `validate_install_space.sh`, `probe_stale_install.sh`, and this receipt, all under this evidence directory.
- **Generated refresh:** Catkin refreshed install-space Python modules, bytecode/egg metadata, package config, and changed package data. Schema-critical refreshed modules are:
  - `install/lib/python3/dist-packages/multifloor_manager/configuration.py`
  - `install/lib/python3/dist-packages/multifloor_manager/ros_node.py`
  - `install/lib/python3/dist-packages/mission_manager/site_config.py`
  - `install/lib/python3/dist-packages/mission_manager/route_planner.py`

## Failing-first baseline

Before installation, this command failed against the valid current fixture:

```bash
source install/setup.bash && python3 -c "from pathlib import Path; import multifloor_manager.configuration as mf; import mission_manager.site_config as site; print('configuration=' + mf.__file__); print('site_config=' + site.__file__); print('StairEndpoint=' + str(hasattr(mf, 'StairEndpoint'))); site.load_site_configuration(Path('test/fixtures/building_valid'))"
```

Observed:

```text
configuration=.../install/lib/python3/dist-packages/multifloor_manager/configuration.py
site_config=.../install/lib/python3/dist-packages/mission_manager/site_config.py
StairEndpoint=False
ConfigurationError: stairs.yaml:stairs[0]: keys must be ['covariance', 'down_profile_id', 'entry_location_id', 'expected_tag_ids', 'from_floor', 'id', 'target_landing', 'to_floor', 'up_profile_id']; got ['down_profile_id', 'endpoints', 'from_floor', 'id', 'to_floor', 'up_profile_id']
```

Pre-refresh SHA-256 pairs also differed:

| Module | Source | Stale install |
|---|---|---|
| `multifloor_manager/configuration.py` | `29704624...aedeaf18b` | `064f0edc...3a4055` |
| `multifloor_manager/ros_node.py` | `f08360ac...cea620d` | `b04ae790...fe08` |
| `mission_manager/site_config.py` | `d6104ee6...be66d6584` | `2e67efe1...f80f639` |
| `mission_manager/route_planner.py` | `bd1e9ef6...a1bfe3` | `5e1ec609...e596` |

This proves the original build/check success was misleading: `catkin_make` and `./run.sh --check` did not update the declared install surface.

## Install refresh and manual QA

Exact repair command, run twice:

```bash
catkin_make install
```

- First run: exit 0; `setup.py build/install` explicitly copied the four schema-critical source modules into install space.
- Second run: exit 0; unchanged schema-critical modules remained up to date, proving repeated installation is safe.

The durable install validation command is:

```bash
.omo/evidence/5f-rf-autonomous-route-repetition/task-3/repair-install/validate_install_space.sh
```

It sources `install/setup.bash`, requires both imported modules to originate below that selected install prefix, loads the real bidirectional fixture, requires `StairEndpoint`, and checks each stair's endpoint-floor pair. It passed after each install run:

```text
INSTALL_SCHEMA_OK stairs=2 package_root=.../install/lib/python3/dist-packages
```

Direct install-space QA also reported:

```text
StairEndpoint=True
endpoint_floors=3F,4F
INSTALL_SCHEMA_OK stairs=2
```

## Source/devel/install agreement

- Source fixture load: `SOURCE_SCHEMA_OK ... endpoints=3F,4F`.
- `source devel/setup.bash` fixture load: `DEVEL_SCHEMA_OK ... endpoints=3F,4F`; Catkin devel correctly resolves the live source module.
- `source install/setup.bash` fixture load: `INSTALL_SCHEMA_OK stairs=2` from the install package root.
- Final source/install SHA-256 values are equal:

| Module | Final SHA-256 |
|---|---|
| `multifloor_manager/configuration.py` | `29704624de8d6c4de6c4422ac7ee12a3a6fc735c6e2eff50d3329e1aedeaf18b` |
| `multifloor_manager/ros_node.py` | `f08360acaa53e07d28970aa4c2b1ff1a3be1ce8e6084f2a127df24000cea620d` |
| `mission_manager/site_config.py` | `d6104ee69ef91a992e339b46ce40f48dbdbe49d601994bef329ce88be66d6584` |
| `mission_manager/route_planner.py` | `bd1e9ef6450c3321583248455443d0c173e9dd1d967b8ca42bad8e2ed2a1bfe3` |

## Adversarial stale-install probe

```bash
.omo/evidence/5f-rf-autonomous-route-repetition/task-3/repair-install/probe_stale_install.sh
```

The probe builds an isolated temporary install prefix exposing the retired single-endpoint `Stair` slots and no `StairEndpoint`, invokes the same validation gate, and requires the dedicated stale-schema exit status 42 rather than accepting an unrelated failure. Observed:

```text
STALE_INSTALL_SCHEMA: missing StairEndpoint
STALE_INSTALL_REJECTED exit=42 root=/tmp/task3-stale-install-*
```

The trap removed the isolated fixture. Final `/tmp/task3-stale-install-*` glob: zero matches.

## Task 3 verification

| Command | Result |
|---|---|
| `source devel/setup.bash && python3 -m unittest test.test_site_configuration test.test_stair_endpoint_configuration src.mission_manager.test.test_route_planner -v` | **PASS**, 45/45 in 2.371s |
| `python3 validate_bundle.py --site-config-root test/fixtures/building_valid` | **PASS**, `SITE_CONFIG_OK floors=3 locations=7 edges=9`, `BUNDLE_OK` |
| `./run.sh --check` | **PASS**, bundle, fixture, packages, and local software OK |
| `catkin_make` | **PASS**, all generated-message targets built, exit 0 |
| `env -u ROS_HOSTNAME ROS_IP=127.0.0.1 bash -c 'source devel/setup.bash && rostest multifloor_manager floor_transition_delayed_pose_ros.test'` | **PASS**, 1/1, 0 errors/failures |
| `bash -n .omo/evidence/5f-rf-autonomous-route-repetition/task-3/repair-install/validate_install_space.sh .omo/evidence/5f-rf-autonomous-route-repetition/task-3/repair-install/probe_stale_install.sh` | **PASS** |

The first rostest attempt inherited `ROS_IP=10.192.1.200` and failed before test execution because ROS could not contact its own server. The loopback-isolated rerun above passed; no project configuration was changed.

## Contract and topology preservation

Source/install interface SHA-256 pairs remain equal:

- `Mission.action`: `83157e5987e17a6e405c00890d4a25a297f7e180676b63aeeae8113371bc3ac8`
- `FloorTransition.action`: `4fb6c09c1656c393be39ddf306c9435814c015d90326d9e024d582acc10ac720`
- `StairTraversal.action`: `54d7c397307e5d1f2fb6ef77c9938c4974ada3c6c6f71c31ed79939a54057e8e`
- `FloorState.msg`: `a21ba5f760a2317340d494b90e0e85dbd9c68579a82399cafd0c939f613e47f2`
- `SupervisorState.msg`: `0ce89dae8fdae7d08ab30c31b019ce300b87e0c84205cd8ae52b8f232d0845cf`

Install-space `roslaunch --nodes mission_manager system.launch ... rviz:=false` listed the same three project nodes (`/mission_manager`, `/multifloor_manager`, `/stair_supervisor`) plus the five standard navigation/sensor nodes. The command only rendered launch topology; it started no node and caused no robot motion or network contact.

## Diagnostics, workspace state, and cleanup

- Python LSP diagnostics: no diagnostics in the four schema-critical source modules.
- Shell LSP diagnostics: unavailable because the Bash language server is not installed and installation was previously declined. `bash -n` passed both new scripts; `shellcheck` is not installed.
- Script size: both scripts are below the 200-line healthy threshold. Each owns one responsibility: install contract validation or stale-output adversarial setup.
- Workspace attribution: `git status --short` returns `fatal: not a git repository`; this shared workspace has no Git metadata. No unrelated source/product file was edited.
- Expected generated changes: Catkin `build/`, `devel/`, and `install/` outputs plus ROS test result/log artifacts. The generated install refresh is intentional and required.
- Cleanup: no `floor_transition_delayed_pose_ros` process remains; no stale probe directory remains. Pre-existing ROS processes were not inspected, stopped, or modified.
- Safety: no ROS motion, mission, navigation goal, robot transport, SSH, WebSocket, or physical robot contact was started.
