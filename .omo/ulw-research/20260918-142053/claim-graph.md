# Claim Graph

## Verified claims

Pending.

## Claims

| claim_id | statement | type | risk | scope | intent ids | supporting observations | contradicting observations | independent groups | convergence | counter-search | primary source | dependencies | status | synthesis location |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C1 | Current TRON1 has no mechanism to observe or correct stair-relative lateral offset or gradual heading drift. | code/runtime | high | stair traversal | I1 | O1,O2 | None found | controller-code | supported | Source search found no alternate path | Repository source | None | supported | Current implementation |
| C2 | One candidate architecture offers the best safety, mobility, and simplicity tradeoff. | recommendation | high | controller architecture | I2 | Pending | Pending | Pending | pending | Pending | Pending | C1,C3 | unresolved | Pending |
| C3 | Current recordings are insufficient to validate metric stair-relative correction or identify IMU bias and wheel slip independently. | empirical | high | captured routes | I3 | O3,O4 | Rich relative sensor coverage | capture-inventory | partial | Pending external-method comparison | Recorded artifacts | None | partial | Data limitations |
| C4 | IMU and wheel odometry should propagate short-horizon motion and detect inconsistency, not define stair-center truth. | scientific | high | estimator | I4 | O5 | Recent robust proprioceptive odometry results | inertial-observability | supported | Successful systems still use external truth and do not observe stair boundaries | Primary literature | C3 | supported | Needs TRON1 bias/slip characterization |
| C5 | Forward RGB should be secondary feedback, with conditional heading utility rather than sole metric centering. | scientific | medium | perception | I5 | O6 | Monocular tracked-vehicle traversal results | vision | partial | Positive results rely on visible geometry and other platform dynamics | Primary literature | C2 | partial | Compare with LiDAR/RGB-D |
| C6 | A 2D wall/rail line observer is the simplest primary geometry source if the deployed scan includes persistent side geometry. | architecture | high | perception | I6 | O7,O9 | 3D multi-edge stair observer | lidar-geometry | partial | Actual external scan projection and stair visibility are unverified | Literature plus deployment source | C5 | partial | Validate bags and mini PC config |
| C7 | A bounded heading-plus-lateral proportional yaw law is sufficient as the first controller candidate. | architecture | high | controller | I7 | O8 | LQR, MPC, robust and learned alternatives | control | partial | Stair-specific recoverable region remains unmeasured | Primary literature | C6 | partial | Independent architecture review |
| C8 | The minimum defensible architecture excludes mandatory 3D and RGB feedback. | architecture | high | complete system | I8 | O10,O11 | Full hybrid and fiducial alternatives | adversarial-alternatives,independent-review | supported | Both reviews retain 3D/RGB only for demonstrated failure modes | Independent synthesis | C6,C7 | supported | Conditional on 2D experiment |
| C9 | The architecture is not deployable until an independently surveyed pose-grid validates observation and identity. | safety | high | commissioning | I9 | O3,O9,O10,O11 | Internal replay and same-LiDAR cross-check | capture-inventory,deployment,independent-review | supported | Existing bags lack independent truth and exact scan projection proof | Mixed independent sources | None | supported | Decisive next gate |
