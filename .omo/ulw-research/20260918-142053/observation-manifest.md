# Observation Manifest

| observation_id | source | evidence layer | observer group | independence basis | observer | observed_at | valid_at | artifact | quote or anchor | contamination notes |
|---|---|---|---|---|---|---|---|---|---|---|
| O1 | `stair_evidence.py` | source | controller-code | direct implementation | explore-worker-1 | 2026-09-18 | current worktree | `wave-1-code-controller.md` | lines 96-209 | Current uncommitted worktree |
| O2 | `supervisor.py` | source | controller-code | direct implementation | explore-worker-1 | 2026-09-18 | current worktree | `wave-1-code-controller.md` | lines 157-216 | Current uncommitted worktree |
| O3 | stair bags and extracted evidence | recorded data | capture-inventory | independent recorded artifacts | explore-worker-2 | 2026-09-18 | capture dates | `wave-1-recorded-evidence.md` | inventory and metadata | No independent pose truth |
| O4 | replay result JSON files | replay | capture-inventory | independent replay artifacts | explore-worker-2 | 2026-09-18 | current replay outputs | `wave-1-recorded-evidence.md` | all FAULTED or INCOMPLETE | Replay rewrites timestamps |
| O5 | inertial/odometry primary literature | external literature | inertial-observability | independent publications | librarian-worker-3 | 2026-09-18 | publication dates | `wave-2-inertial-odometry.md` | DOI and section anchors | Platform transfer requires validation |
| O6 | visual stair-perception literature | external literature | vision | independent publications | librarian-worker-4 | 2026-09-18 | publication dates | `wave-2-vision.md` | DOI and section anchors | Mostly non-quadruped evidence |
| O7 | LiDAR stair and Livox literature | external literature | lidar-geometry | independent publications | librarian-worker-5 | 2026-09-18 | publication dates | `wave-3-lidar.md` | DOI, arXiv and implementation anchors | 2D recommendation conditional on actual scan field |
| O8 | mobile robot path-control literature | external literature | control | independent publications | librarian-worker-6 | 2026-09-18 | publication dates | `wave-3-controller.md` | equations and convergence assumptions | Not stair-specific validation |
| O9 | `system.launch` | source | deployment | direct implementation | lead | 2026-09-18 | current worktree | `src/mission_manager/launch/system.launch` | lines 30-31 | Mini PC projection config absent from repository |
| O10 | alternatives analysis | synthesis | adversarial-alternatives | independent worker | deep-worker-7 | 2026-09-18 | current evidence | `wave-4-alternatives.md` | ranked comparison and counterexamples | Uses same supplied local facts |
| O11 | architecture review | synthesis | independent-review | independent reasoning | oracle-worker-8 | 2026-09-18 | current evidence | `wave-4-oracle-review.md` | verdict and decisive experiment | No new physical evidence |
