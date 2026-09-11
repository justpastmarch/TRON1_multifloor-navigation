# Adversarial probes

| Class | Probe | Observable result |
| --- | --- | --- |
| malformed metadata | Ran sibling `process_bag.py` against `adversarial_fixtures/malformed_lidar_spec.json`, which includes `"unexpected_success": "ok"`, a numeric path, negative duration, and missing fields | Nonzero validation failure with 10 Pydantic errors; no registration output created |
| stale generated artifact | Copied `normalized_wheel_path.csv`, set mtime to epoch 0, linked remaining inputs, and ran `inspect_package.py` | Nonzero failure: `generated artifact is stale: normalized_wheel_path.csv` |
| dirty workspace | Ran `git status --short` in both workspaces | Both report `not a git repository`; dirty-state provenance is unavailable. Source preservation is instead proved by identical before/after bag and incomplete-metadata SHA-256 values |
| misleading success output | Malformed spec carried a success-looking field while violating the typed schema | Typed validation rejected it; success-like text did not create a pass |
| hung/long command | Wrapped the real sibling `process_bag.py` invocation with bounded `timeout --kill-after=5s` | Command terminated at one second rather than hanging |
| repeated interruptions | Interrupted the real command once with SIGINT and again with SIGTERM using the same temporary output root | Both terminated; `pgrep` found no `process_bag.py`; output directory remained empty after each attempt |
| stale source/input swap | Final inspector recomputed the 1.5 GB source SHA-256 rather than trusting the manifest | Exact expected hash matched; a mismatch is a hard error |
| gap/jump concealment | Inspector searched all wheel deltas above 0.5 s or 0.5 m and all rejected LiDAR updates | Every affected edge had `derived_edge_valid_from_previous=false`; exactly one >0.5 m jump was found and rejected |
| prompt injection | Not applicable | No external text is interpreted as instructions or sent to a model |
| flaky timing test | Not applicable | No timing-dependent unit assertion; intervals derive from immutable bag record timestamps |

Thresholds 0.5 s and 0.5 m are evidence-segmentation thresholds, not production motion limits. They are recorded in `extraction_summary.json` and do not infer behavior inside a rejected interval.
