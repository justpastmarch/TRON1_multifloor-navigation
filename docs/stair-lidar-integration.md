# Stair Supervisor LiDAR integration

The accepted `gyro-gicp-installed-20260921-v1` estimator runs inside the existing
Stair Supervisor deployment. No ROS node, TF authority, WebSocket connection,
action definition, public phase string, or state enum was added. A socket-free
child process handles raw LiDAR decoding and registration because the initial
same-process load test interrupted the command timer.

**Deployment defaults remain `off`; `stair_lidar.yaml` is uncommissioned.**
The software supports observation and surveyed UP routes. A bag trace does not
establish closed-loop climbing, balance control, or a safe emergency response.
In particular, stair mode plus zero twist is not considered position/posture hold.

## Runtime and modes

| Mode | Tracking | Commands |
| --- | --- | --- |
| `off` | No numerical imports, child process or sensor adapter | Existing wheel-evidence Supervisor |
| `observe`, `lidar_observe_only=true` | LiDAR/IMU, diagnostics, optional phase/command proposal | No RobotTransport constructed, no robot socket or zero commands |
| `observe`, `lidar_observe_only=false` | Tracking/proposals alongside existing operation | Legacy Supervisor remains sole command owner |
| `control` | Commissioned body transform, route, entry anchor and LiDAR feedback | Existing Supervisor/RobotTransport only |

`system.launch` forwards `stair_lidar_mode`, `stair_lidar_observe_only`,
`stair_lidar_config` and optional `stair_python` (a Python interpreter path).
`run.sh` exposes `STAIR_LIDAR_MODE`, `STAIR_LIDAR_OBSERVE_ONLY`, `STAIR_LIDAR_CONFIG`
and `STAIR_PYTHON`. It still starts the existing system and SSH infrastructure;
for standalone observation use only the existing stair node, not the full run script.

The numerical environment is Python 3.8, NumPy 1.24.4, SciPy 1.10.1 and
Open3D 0.18.0. Install `src/stair_supervisor/requirements-lidar.txt` into a dedicated
Python environment with access to the sourced Noetic/catkin message packages and
existing websocket-client dependency. Do not replace system NumPy/ROS globally.
`livox_ros_driver2` must match the installed `CustomMsg` layout/MD5. A `stair_python`
override changes only this existing node's interpreter. `off` needs none of the new
numerical dependencies.

Example observation, after sourcing ROS and the built workspace and selecting the
numerical interpreter (the interpreter path is site-specific):

```bash
/path/to/lidar-python src/stair_supervisor/scripts/stair_supervisor_node.py \
  _lidar_mode:=observe _lidar_observe_only:=true
```

This produces tracking only with the shipped configuration. The observation start
service can additionally evaluate surveyed phases and proposed commands, without
transmitting them. It requires a valid route/profile and entry evidence.

## Preserved estimator and time contract

`lidar_tracking_core.py` is byte-identical to the accepted registration core:
SHA-256 `1210ca227b1046750808aff118f9457179c290607c28ee790d829e9b1dd70d10`.
The copied calibration hash is
`d3166bd2547aa5fdca388d68b4c5213d6454be8f9350ae1981731eb48f9c9ddc`.

The estimator keeps the calibrated IMU rotation, initial accelerometer median over
±0.5 s, relative gyro rotation seed, GICP, 0.20 m voxels, 0.80 m correspondence
range, fitness 0.35, 0.50 m/0.35 rad correction limits, and 15 accepted local scans.
It does not integrate accelerometer translation, compensate gyro bias, deskew, or
reset at the landing. Bootstrap is not geometry evidence.

`rospy.AnyMsg` avoids constructing large point lists in the command owner. The
child decodes the installed 19-byte CustomPoint representation into exactly the
same float32 XYZ values, then passes them to the unchanged core. Buffers are bounded:
8 raw packets, 4096 IMU rows, 8 pending selected scans, 15 local-map scans and 4
outbound snapshot slots. No unlimited global cloud is retained.

Measurement, receipt, processing completion and last accepted geometry times are
separate. Rejections never republish a pose with a fresh stamp. A timestamp reversal,
clock jump or child restart invalidates the epoch and entry anchor. The child does
not call `rospy.init_node`, publish TF, or open a robot socket. Its mode/entry work
has a deadline; a hung entry job is terminated with that child and observation
restarts in a new epoch. The next action needs a new anchor. Normal short rejection
recovers in the same frame without closing the robot session.

The 105713 bag contains recording-time minus sensor-header differences whose P99
is about 4.16 s. This is a property of recorded timestamps, not proof of current
network latency. Displaying a good retrospective trajectory cannot prove fresh
online observations. Measure current sensor/host clock alignment and delivery
latency before choosing control age budgets; do not hide this with restamping or
an arbitrary four-second expiry allowance.

## Commissioned geometry

The extension is separate from the strictly validated existing `stair_profiles.yaml`.
Its route IDs must match enabled existing profile IDs. `configured: true` does not
substitute for survey/response evidence. Production contains no invented body
mount, centerline, footprint, gain, arrival distance or loss response.

`T_A_B` maps B coordinates into A. `base_from_lidar` is a rigid 4×4 surveyed body
mount transform, separate from the LiDAR/IMU calibration. Route coordinates use a
surveyed entry reference and body path height; their Z coordinates are not sensor
height or an assumed riser-count conversion. An entry observation creates
`local_from_profile = local_from_lidar × inverse(base_from_lidar) × inverse(profile_from_base)`.

Each UP route requires:

| Field | Contract |
| --- | --- |
| `id`, `commissioned`, `direction` | Existing stair ID; explicit commissioning flag; currently `UP` only |
| `flight_1`, `flight_2` | Two surveyed ascending body-reference XYZ endpoints per flight |
| `entry_polygon`, `flight_1_polygon`, `flight_2_polygon`, `landing_polygon`, `exit_polygon` | Convex CCW XY usable regions; flight regions include the supported boundary handover |
| `footprint` | Measured dynamic body/support envelope in the body frame, not the NAV costmap radius |
| `turn_path` | Surveyed `[x,y,yaw]` landing waypoints; final yaw is the actual next-flight direction |
| `limits` | All commissioning values below, in metres, seconds and radians |
| `loss_response`, `loss_response_evidence` | Only `zero_velocity` is selectable; operator takeover evidence is required. Zero twist is not a posture guarantee |
| optional `entry_reference` | Surveyed point template and independent error/ambiguity limits |

Required `limits` fields:

- Observation: `warn_sec < expire_sec < recover_sec`, `anchor_max_age_sec`, `anchor_uncertainty_m`.
- Geometry: `margin_m`, `height_tolerance`, `max_roll`, `max_pitch`.
- Feedback: `yaw_kp`, `lateral_kp`, `speed_kp`, `hold_kp`, `hold_kd`.
- Command bounds: `max_v`, `max_w`, `max_accel`, `max_alpha`, `hold_v`.
- Noise/arrival: `yaw_deadband`, `lateral_deadband`, `speed_deadband`,
  `position_off < position_on`, `yaw_tolerance`, `stationary_v`, `stationary_w`, `settle_sec`.
- Handoff: `handoff_sec` bounds mode handoff and temporary exit ownership.

All are finite positive values. Deadbands must lie below their associated arrival
limits, and `hold_v <= max_v`. Body and survey errors enter the polygon margin.
Values in `test_lidar_control.route()` are synthetic test fixtures only.

Known records are preserved: stair width 1.23 m; landing 2.70 × 1.48 m;
5F→RF has 10 + 9 + 2 exit steps. They do not establish 3F→4F coordinates,
mount translation, dynamic body support or response gains. DOWN and other stairs
need their own controller/physical commissioning; control mode rejects them rather
than applying the forward UP law backwards. Legacy `off` behavior is unchanged.

## Per-run entry evidence

The private `~capture_entry` Trigger reads `~entry_evidence`:
`route_id`, `epoch`, `sensor_stamp`, `uncertainty_m`, `source`, and either
`profile_from_base` (4×4 independent measured entry pose), or identified
`observed_local_xy`, `profile_candidates`, `ambiguity_margin_m` correspondences.
The captured immutable sample must correspond to that measurement. Ordered
landmarks are checked for observability and ambiguous matches. Repeated fit residual
is not an independent bound on global entry error. Anchors are consumed once.

For automatic entry, optional `entry_reference` requires `path` (surveyed Nx3 NumPy
array), `sha256`, `yaw_candidates`, `min_fitness`, `max_rmse_m`, `score_gap`,
`validated_anchor_error_m`, `timeout_sec`, and `unique_geometry_verified: true`.
Registration runs on the compute worker, considers the commissioned hypotheses,
rejects ambiguous/poor/tilted fits, and uses an independently measured combined
entry/mount uncertainty. This matches a surveyed template; it is not a generic
stair detector or proof that AMCL cannot choose a symmetric office corner.

For phase-only observation set `~observe_route_id`, supply entry evidence and call
`~start_observation`. Observed arrival/phase means that the software's surveyed
geometry conditions were met, not an independent certification of physical arrival.

## Feedback, arrival and interruption

The existing action/phase sequence is preserved. Flight steering uses heading and
lateral errors with continuous deadbands. Actual LiDAR speed contributes bounded
forward-speed compensation; backward slip is retained as observed motion. The
output remains v/yaw, with no new lateral or joint/posture control interface.

Landing and exit use a fixed position/yaw target, hysteresis and bounded correction.
The predicted unicycle arc and full projected body are checked over the observation
expiry horizon; this is a geometric command check, not a dynamics proof. A normal
5 Hz observation is not rejected using the legacy 0.12 s wheel-sample gap.

Flight completion requires surveyed progress, observed height progression and the
whole body in the next support region. Final arrival additionally needs distinct
fresh samples of stable position/velocity/yaw. A repeated pose cannot fill dwell,
and 150 degrees or a visual `FINAL_ARRIVAL` marker cannot complete the turn.

Mode enable/disable waits continue the same owner's feedback using bounded receive
polls. No competing receiver or robot socket is added. The final send is followed
by another cancellation/shutdown check before committing NAV ownership.

Short age/rejection within the commissioned validity budget retains the bounded
last command and suspends completion. It does not ramp all components to zero and
claim support. Expiry, invalid epoch, worker failure or geometry/attitude violation
selects the commissioned response. After that response, automatic motion is not
rearmed merely because a sensor returned. `recover_sec` reports an overdue physical
handoff, while the Supervisor retains ownership. This narrower behavior is required
until the actual mode/posture recovery procedure is demonstrated.

Cancellation returns the existing cancelled action result while ownership remains
STAIR. It does not report arrival or silently switch to WALK/NAV. Explicit physical
handoff uses private `~acknowledge_physical_handoff`, gated by one-shot
`~operator_confirmed_supported_handoff: true`. It requests mode off and transfers
ownership only after operator attestation; a network ACK alone never proves support.
Transport loss cannot guarantee delivery of a new response. Shutdown still neutralizes
and closes the sole transport; it is not proof the body remains upright.

## Arrival restoration using existing NAV

Optional mission `arrival_hold_enabled` uses one low-priority monitor and the
**same** NavigationExecutor. No second move_base client or velocity publisher is
created. Required `arrival_hold` parameters are `r_on`, `r_off`, `yaw_on`, `yaw_off`,
`correction_timeout`, `cancel_timeout`. The on thresholds exceed off thresholds.
`system.launch` accepts an `arrival_hold_settings` YAML path; `run.sh` accepts
`ARRIVAL_HOLD_SETTINGS`. No guessed physical thresholds are shipped.

After a successful mission the monitor fixes its target to the confirmed named
location and map generation. Once ready it releases the Supervisor's short exit
hold. New missions cancel and join any existing correction. Cancellation is scoped
to the exact NavigationRequest, so a late hold tick cannot cancel the next mission.
Floor/generation change, stale localization, external cancellation or correction
failure disables this optional correction with a reason; it does not globally
latch ordinary NAV. `~stop_arrival_hold` stops the monitor and releases the short
Supervisor hold. Loss of a trustworthy map pose is not repaired by a LiDAR local
frame. Actual restoration precision and posture stability remain field tests.

## Diagnostics and reproducibility

| Existing node private topic/service | Meaning |
| --- | --- |
| `~lidar_odom` | Accepted sensor pose only, original timestamp, `stair_local_<epoch>` frame; display-only large covariance placeholder, not a fusion input |
| `~tracking_status` | Epoch, queue/drop counts, original calibration hash, measurement/receipt/completion times, age, fit/innovation, rejection/worker errors |
| `~control_debug` | Phase, body pose/attitude, velocity, lateral/yaw errors, proposed v/yaw, landing/exit support and response state |
| `~geometry_markers` | Targets, surveyed regions, body outline and actual software phase; stale body red, marker lifetime 0.5 s |
| `/stair_supervisor/websocket_tx` | Existing exact successfully sent frame observation, used with proposed commands to debug the transport boundary |
| mission `~hold_status` | Optional arrival correction state/reason |

Set RViz Fixed Frame to the active `stair_local_<epoch>` when displaying local
geometry; no `map→odom` or `odom→base` transform is added. A frame reset deliberately
requires changing/reacquiring the reference. Record input topics, diagnostics,
phase/action/state and the existing actual-transmit topic together. The node does
not create unbounded disk logs or automatically record point clouds.

`replay_lidar_tracking.py SPEC OUTPUT --reference CSV` feeds original recordings
chronologically through the same core and compares frozen pose/state results.
`profile_lidar_stream.py SPEC OUTPUT.json` uses a separate recorded sensor source,
the actual compute child and memory-only 40 Hz Supervisor NAV path. It never
constructs RobotTransport or publishes velocity. Its metrics are component timing,
not the complete mission/move_base/floor/RViz load or actual socket deadlines.

Legacy tests run through catkin. Optional `test_lidar_control.py`,
`test_lidar_ros_adapter.py` and `test_lidar_process.py` require the numerical Python
and generated ROS/Livox messages. `STAIR_LIDAR_TESTS` is off in legacy catkin builds.
Synthetic metrics/geometry do not commission a route.

Rollback is an idle-time switch to `stair_lidar_mode=off` and disabling optional
arrival hold. Never switch to wheel evidence in the middle of a LiDAR-controlled
stair run. Preserve the calibration/core hashes and independently survey changed
mounts/routes. Live observation, full-stack timing, yaw/forward response, actual
support/loss response, full 3F→4F arrival, and long-duration hold remain required
physical acceptance work before enabling unattended control.

## Operator single-phase tests (2026-09-21)

LiDAR loss/cancellation sends zero twist, never `emergency_stop`. Operator remote
control may interrupt at any time; software zero does not imply physical hold.
The existing action accepts one-shot private `~phase_test` configuration with
`route_id`, `phase`, `max_duration_sec` (0 < duration <= 30) and
`operator_confirmed: true`. Supported phases are ALIGN, FORWARD_SEGMENT_1,
LANDING, TURN_TO_NEXT_FLIGHT and FORWARD_SEGMENT_2. A current anchor describing
the robot's actual pose in the route is still required. Starting mid-route must
not relabel that position as the route origin. Geometry and tracking checks run
for the selected phase during mode entry. Mission entry-token validation is
replaced only for this explicit operator test; normal missions retain it.
Tests do not advance to another phase or report floor arrival. Completion,
cancellation, or a fault sends zero and retains STAIR ownership for operator
takeover. Supported handoff acknowledgement remains explicit; it switches to
WALK and must not be called merely because a test ended on stairs.
These software changes do not commission missing route geometry or gains.
