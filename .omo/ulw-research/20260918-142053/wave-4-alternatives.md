# Wave 4: Simpler Alternatives

Worker: `bg_c8a5c1a6`

## Verdict

The minimum viable architecture is validity-gated 2D boundary-line feedback with a strictly bounded IMU/odometry bridge. Three-dimensional stair validation is not required for the lateral-drift safety claim. If continuous boundary visibility fails, instrumented fiducials are the next simpler independently observable option; pre-alignment, heading hold, open loop, and landing-only correction cannot observe drift during a flight.

## Load-bearing objection

The candidate assumes that `/scan` continuously observes the same stair-relative boundary through the body-pitch envelope. A strong line fit can still be the wrong wall, riser, landing edge, doorway, or rail posts. Until visibility and identity are tested against independent truth, downstream controller sophistication does not create observability.

## Additional gate

Software zeroing is not automatically a physically safe stair stop. Representative mid-flight zero-command stability must be tested before `STOP` can be called fail-safe.
