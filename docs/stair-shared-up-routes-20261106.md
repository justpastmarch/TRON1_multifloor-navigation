# Shared 3F→4F and 4F→5F uphill geometry

The operator confirmed on 2026-11-06 that both stairways have the same physical
geometry. The 4F→5F LiDAR route now inherits the 3F→4F 9-step flight, landing,
10-step flight, controller limits, and phase budgets through a YAML merge.
The effective 3F→4F route and existing 5F/RF routes are unchanged.

The 4F entry uses `stair_4f_to_5f` on the 4F map. It does not reuse the 3F
point-cloud template. The existing map-entry transform, previously used for
RF descent, now accepts any route explicitly configured with `entry_map`.
The same freshness, floor, covariance, offset, and body attitude handling
continues to apply. No new ROS node, FSM state, or numerical gate was added.

Offline checks cover shared geometry, UP map-entry dispatch, preservation of
the real map offset, wrong-floor rejection, registered endpoint existence,
and existing entry/downhill/landing behavior. Full cloud registration and
physical 4F→5F operation require a live test; no robot command is part of
these checks. Running MiniPC processes receive the change on the next normal
stack deployment/start, not merely when the source files are edited.
