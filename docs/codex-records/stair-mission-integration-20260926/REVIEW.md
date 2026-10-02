# Review and validation limits

The sip/shower review read copied artifacts without repository context. Handoff review found unsupported runtime goal-tolerance changes, a handoff race, missing test registration, and a capture source-hash race. These were corrected before deployment. A live execution found a runpy sibling-import failure; the operator tool now explicitly adds its scripts directory.

The proposed full-3D entry frame alignment was NOT deployed. Review found yaw-only candidate deduplication could accept opposite-tilt hypotheses. The experimental copy now compares SO(3) angle and includes a negative regression. This does not establish physical accuracy: the reference's gravity alignment and existing .10m anchor budget remain unverified. Before/after samples are different scans, not a paired ablation. No exact accuracy or field success-rate claim is made.

The independent follow-up read both the idle AMCL refresh implementation and tests end-to-end and found no concrete regression: bounded existing service only, READY/NAV/idle/fresh distinct scan, no pose reset/global search/motion, retry after failure without invalidating READY.

factchk: AMCL's movement threshold and no-motion update behavior were checked against the archived Noetic upstream amcl_node.cpp, lines 1105–1110 and 1195–1208, plus live /amcl parameters. This separates upstream behavior from this repository's new idle timer.

mandela: repeated cloud matches and synthetic tests are self-consistency/repeatability, not independent position ground truth. Shared-scene bias and designer-as-verifier apply to interpreting those metrics as accuracy; independent surveyed checkpoints and physical closed-loop trials are still required. Covariance reduction is not proof that a symmetric-room pose is correct.

ssotize: runtime settings remain in source configuration; records are dated snapshots. This change adds no persistent ROS node. The experimental alignment files must not be copied into production. No portability claim is made, so detool was not applicable.
