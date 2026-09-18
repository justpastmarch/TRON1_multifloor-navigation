# Wave 1: Current Stair Controller

Worker: `bg_a3844fa5`

## Findings

- The controller observes only `/tron/wheel_odom_raw`; it has no stair-relative lateral, boundary, pitch, roll, LiDAR, camera, or contact observation.
- Flight commands are fixed `(linear_speed, 0)` and turn commands are fixed angular speed. Evidence determines completion or fault, not steering correction.
- Distance progress is the odometry displacement projected onto a phase heading. Cross-track displacement is not bounded.
- `ALIGN` completes on a new odometry sample and performs no physical alignment.
- Production thresholds include 0.40 m/s flight speed, 1.57 rad/s turn speed, 0.50 m distance tolerance, 0.35 rad yaw tolerance, 0.20 s freshness, and 0.12 s maximum sample gap.
- Cancellation is deferred to VERIFY_ENTRY, LANDING, or EXIT_CONFIRM.
- Evidence and transport faults close the sole command transport and do not reconnect or resume.

## Primary anchors

- `src/stair_supervisor/src/stair_supervisor/stair_evidence.py:96`
- `src/stair_supervisor/src/stair_supervisor/stair_evidence.py:187`
- `src/stair_supervisor/src/stair_supervisor/supervisor.py:157`
- `src/stair_supervisor/src/stair_supervisor/supervisor.py:191`
- `src/stair_supervisor/src/stair_supervisor/robot_conversion.py:25`
- `src/stair_supervisor/config/stair_profiles.yaml:4`

## Expansion leads

- Admission token and epoch semantics.
- Pre-stair pose/localization gate.
- Recorded replay limitations and action-level behavior.
