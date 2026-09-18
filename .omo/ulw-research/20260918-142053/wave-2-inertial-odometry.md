# Wave 2: IMU and Odometry

Worker: `bg_95c9d3ed`

## Conclusion

IMU and wheel odometry can improve short-horizon body-motion estimation but cannot observe lateral offset from a stair centerline. IMU should provide roll/pitch, angular-rate propagation, impact/vibration indicators, and consistency checks. Wheel odometry should provide local progress with slip-dependent uncertainty. Neither is an absolute stair-relative heading or position reference.

## Key evidence

- Forster et al., *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*, DOI `10.1109/TRO.2016.2597321`.
- Huang et al., *Observability-based Rules for Designing Consistent EKF SLAM Estimators*, DOI `10.1109/IJRR.2010.2050890`.
- Woodman, *An Introduction to Inertial Navigation*, UCAM-CL-TR-696.
- Martinelli, *Observability analysis for mobile robot localization*, DOI `10.1109/ROBOT.2002.1013440`.
- Noh et al., *GaRLILEO*, DOI `10.1177/02783649261457941`.
- Kong et al., *TRACE*, arXiv `2608.05975`.

## Caveats

- Stair impacts and vibration invalidate naive gravity updates.
- Slip invalidates contact-based wheel/foot velocity constraints.
- Short duration reduces drift magnitude but does not create missing observability.
- Recent platform-specific performance does not prove TRON1 accuracy.
