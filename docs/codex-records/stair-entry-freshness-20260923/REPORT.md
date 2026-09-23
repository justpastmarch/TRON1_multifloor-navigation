# Entry freshness wait correction — 2026-09-23

OBSERVED: The prior entry-test client rejected the first tracking status if its reported geometry age exceeded 0.5 seconds. It did not wait for another sample. The exact age of the message that triggered the user’s failure was not recorded.

OBSERVED: A subsequent 12-second read-only observation received 120 tracking statuses, all below 0.5 seconds (reported range 0.2824–0.4828 seconds). This does not reproduce or explain every possible cause of the original delay.

INFERRED: A transient stale first status can therefore reject an otherwise usable tracking stream. This mechanism is reproduced by a regression test; it is not a direct reconstruction of the original incident.

The existing one-shot client now waits at most 8 seconds on one subscription for a valid, fresh status. It retains the route’s 0.5-second admission threshold. Configuration, geometry, and worker failures still reject. The Supervisor independently validates its latest sensor state on admission. Pre-receipt transport delay is not measured by the client helper.

Only the client and its existing test module changed. No deployed node, state-machine logic, controller, route configuration, or launch file changed. No process restart is required. Original versions are retained under `before/`.

Validation: 28 tests passed, including 10 freshness cases. A read-only check of the applied helper returned TRACKED in 0.0466 seconds with reported geometry age 0.3832 seconds. No preview service, action goal, or robot command was sent during this check. Physical stair traversal remains unverified by this test.

The existing entry preview had expired. The operator must place the robot at the configured entry, create a fresh preview, inspect its RViz overlay, and run within the 120-second preview window. This change does not automatically re-anchor a robot that has moved.

Cleanup and worktree: see `verification.json`. Temporary candidate copies and probe logs were removed; test evidence and original-file backups are deliberately retained. Existing worktree changes were preserved. Identical Git porcelain text does not imply zero content changes because the two edited files were already untracked.
