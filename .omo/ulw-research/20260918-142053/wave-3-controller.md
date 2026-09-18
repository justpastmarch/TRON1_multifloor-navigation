# Wave 3: Correction Controller

Worker: `bg_ad80af93`

## Conclusion

The smallest controller matching TRON1's forward-speed plus yaw-rate interface is a bounded heading-plus-lateral proportional law:

`omega = sat(k_psi * e_psi + k_y * atan2(e_y, L))`

It requires yaw-rate slew limiting, fresh stair-relative evidence, a short bounded IMU/odom propagation interval, and fail-closed stop behavior. No integral term is justified initially.

## Rationale

- A raw linear lateral term is speed- and units-sensitive.
- `atan2(e_y, L)` converts lateral error into a bounded heading-like correction and remains well behaved at low speed.
- Stanley is the closest precedent but its original steering-angle and nonzero-speed assumptions do not directly validate stair traversal.
- Pure pursuit weakens near zero speed; LQR offers little extra for one straight operating condition; MPC, sliding mode, and learned control add unjustified verification burden.
- Local convergence of the unsaturated small-error model does not prove edge clearance under delay, dropout, or saturation.

## Evidence states

- `TRACKING`: full feedback from fresh stair-relative geometry.
- `PROPAGATING`: short bounded IMU/odom prediction.
- `DEGRADED`: reduced speed and yaw authority.
- `STOP/ABORT`: evidence timeout, excessive uncertainty, persistent saturation, or command-motion disagreement.

## Key evidence

- Hoffmann et al., *Autonomous Automobile Trajectory Tracking for Off-Road Driving*, DOI `10.1109/ACC.2007.4282788`.
- Thrun et al., *Stanley: The Robot that Won the DARPA Grand Challenge*.
