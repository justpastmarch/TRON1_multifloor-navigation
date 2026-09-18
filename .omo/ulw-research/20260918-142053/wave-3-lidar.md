# Wave 3: LiDAR Stair Geometry

Worker: `bg_31183f39`

## Conclusion

A persistent side wall or handrail line can directly yield lateral distance and heading at the existing `/scan` rate with the lowest implementation cost. Two-sided geometry is preferable; one-sided geometry requires stable side identity and a calibrated target offset. Multi-edge 3D stair fitting is a useful initializer, validator, or fallback. LIO is infrastructure for deskew and short-term propagation, not stair-relative truth by itself.

This recommendation is conditional: the repository does not own the mini PC `mid360s_laserscan` projection parameters. `system.launch:30-31` states that `/scan` is externally produced, so the local source cannot establish whether walls or rails are included at stair pitch angles.

## Evidence

- Westfechtel et al., *Robust stairway-detection and localization method*, DOI `10.1177/0278364918798039`.
- Sriganesh et al., *Fast Staircase Detection and Estimation*, arXiv `2211.00610`.
- Lin and Zhang, *LOAM-Livox*, arXiv `1909.06700`.
- Xu et al., *FAST-LIO2*, DOI `10.1109/TRO.2022.3141876`.

## Failure modes

- Wall/rail association changes at landings, openings, clutter, and people.
- Rails may be discontinuous, curved, or offset from the desired center.
- 3D fitting needs several compatible edges and roll/pitch compensation.
- Livox sparsity, non-repetitive sampling, motion distortion, and repetitive stair geometry weaken scan registration.
