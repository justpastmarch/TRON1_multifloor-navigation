# Wave 4: Independent Architecture Review

Worker: `bg_694eb2e7`

## Verdict

The architecture direction is defensible, but the 2D sensor choice is not yet earned. Keep a backend-neutral observer contract and select 2D only after independently surveyed pose-grid evidence. IMU/odometry may propagate an accepted stair-relative pose briefly but cannot create or validate it.

## Required invariants

- Observer output includes `e_y`, `e_psi`, timestamp, uncertainty, association validity, and age.
- The supervisor owns forward speed, yaw, unconditional stop, and command timeout.
- Propagation ends on a time, distance, or uncertainty bound derived from remaining clearance.
- STOP is latched and recovery is deliberate.
- Internal 2D and 3D outputs from the same LiDAR are not independent validation.

## Single next experiment

At independently surveyed lateral/yaw offsets and representative pitch/roll poses, record raw 3D LiDAR, exact `/scan`, transforms, RGB, IMU, odometry, and timestamps. Define error, coverage, false-association, and dropout limits from physical clearance before examining estimator performance. Select 2D only if it passes the intended pose envelope.
