# Map alignment status: BLOCKED

No reviewed or commissioned waypoint coordinate is issued as a Task 2 output. The
map files are present, but their presence is not treated as reviewed map alignment.

The available inputs contain **zero reviewed, spatially separated surveyed correspondences** for 5F and zero for RF. Task 2 requires at least two per floor. The existing disabled `locations.yaml` contains recorded poses, but it does not identify surveyed anchor pairs, measurement artifacts, or transform residuals, so those values are not adopted as independently reviewed alignment evidence.

| Floor | Map metadata | Image | Surveyed correspondences | Required | Transform/residual |
| --- | --- | --- | ---: | ---: | --- |
| 5F | resolution 0.05 m/px; origin `[-5.762686, -11.722302, 0]`; SHA-256 `fba944bc913edbea02e524b6dc3ef72b35081c916d7342edeb2f9e6c865c1e7b` | 366x278 PGM; SHA-256 `69b274b91801f125436740d0374b70927aa95ef93c56ccba50f25f2e068bb721` | 0 | 2 | unavailable |
| RF | resolution 0.05 m/px; origin `[-8.644581, -15.281230, 0]`; SHA-256 `ccb16c58d8cd77cc13ca169101ef0bb6fd6d7079ea30c1ce192934d0d457ff5c` | 404x380 PGM; SHA-256 `549229bed3a8c34a55abd8baac5953820f3a4ef42b618ff91e5b1c5eb7aefa93` | 0 | 2 | unavailable |

## Site-survey unblock procedure

1. Keep the robot stationary at two or more visually distinct, spatially separated points on each floor.
2. Record reviewed anchor IDs and map-frame poses independently of Joy, wheel odometry, FAST-LIO, and this source route.
3. Associate each anchor with the corresponding accepted LiDAR-only relative pose, retaining timestamp delta and source checksums.
4. Fit one rigid transform per floor and report every anchor residual; do not warp route subsegments.
5. Review proposed `home_5f`, stair endpoints, and roof-loop poses on the live map while stationary before Task 5 may consume them.

Until those steps are complete, all entries in `waypoint_proposals.csv` remain unavailable and blocked.
