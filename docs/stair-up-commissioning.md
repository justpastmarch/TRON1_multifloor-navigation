# Stair UP commissioning facts

## AprilTags

- Family: `tagStandard41h12`
- Printed size: `0.18 m × 0.18 m`
- UP landing IDs: 3F `300`, 4F `400`, 5F `500`, RF `600`
- DOWN landing IDs: 3F `301`, 4F `401`, 5F `501`
- ID `400` is detected in both 3F→4F UP bags.

## Stair geometry

- 3F→4F: 9 steps, landing, 10 steps
- 4F→5F: 9 steps, landing, 10 steps
- 5F→RF: 10 steps, landing, 9 steps, then 2 exit steps

## Operating observations

- The operator enables Stair Mode at the stair entry.
- Stair Mode remains enabled through the landing and is disabled only after reaching the target floor and moving slightly forward.
- Forward commands below normalized WebSocket `x=0.5` do not produce enough torque to clear a step.
- Added hardware shifts the center of mass left. The operator used intermittent right steering while climbing.
- The first 3F→4F bag is `RECOVERED_UNSTABLE`; the repeat is `OPERATOR_COMPENSATED_PARTIAL_STABLE`.
