---
slug: yaml-transition-policy-runtime
status: review-complete
intent: clear
review_required: true
pending-action: start a separate worker with `$start-work yaml-transition-policy-runtime`
approach: Keep the existing multifloor transaction as the fixed safety authority; move the pure Boolean evaluator into multifloor_manager and apply one YAML-selected policy only as an additional final READY gate. Migrate the public schema to a condition list with exact integer 0/1 enabled/required flags and retain optional_count. Allowlist runtime predicates and reject attempts to disable or optionalize mandatory safety aggregates. Preserve a future extension seam by keeping Boolean conditions side-effect free and requiring typed, allowlisted manager actions for commands such as opening a configured door; do not implement unused action machinery or arbitrary YAML network/shell execution now.
---

# Draft: yaml-transition-policy-runtime

## Components (topology ledger)
<!-- Lock the SHAPE before depth. One row per top-level component that can succeed or fail independently. -->
<!-- id | outcome (one line) | status: active|deferred | evidence path -->
| policy-schema | The three current safety conditions are fixed 1/1 with optional_count 0; a comment-only future non-safety example shows the producer/registry/test prerequisite. | active | `src/multifloor_manager/src/multifloor_manager/configuration.py`, `test/fixtures/building_valid/transitions.yaml` |
| evaluator-ownership | Boolean dwell/quorum evaluation is owned by `multifloor_manager` without creating a reverse package dependency. | active | `src/mission_manager/src/mission_manager/transitions.py`, package manifests |
| runtime-wiring | Parsed policy affects the observable floor-transition action only after all fixed tag/map/localization/costmap barriers pass. | active | `src/multifloor_manager/src/multifloor_manager/ros_node.py`, `ros_runtime.py` |
| safety-regression | Cancellation, epoch fencing, causal localization, map identity, and post-clear costmap guarantees remain intact. | active | `src/multifloor_manager/test/test_floor_transition_*` |
| future-actions | Future door/device commands use typed allowlisted actions separate from pure conditions; no action framework is implemented in this plan. | deferred | owner approval in current planning session |

## Open assumptions (announced defaults)
<!-- Record any default you adopt instead of asking, so the user can veto it at the gate. -->
<!-- assumption | adopted default | rationale | reversible? -->
| Current predicate vocabulary | `T_FLOOR_CONFIRMED`, `T_LOCALIZED`, and `T_COSTMAP_READY` only | These are the aggregates the existing multifloor transaction can produce without a new cross-package evidence channel. | yes, by adding an allowlisted producer and tests |
| Safety condition mutability | mandatory aggregates must remain enabled and required | YAML must delay success, never bypass existing safety barriers. | no without a separate safety review |
| Current non-safety vocabulary | none | No existing multifloor aggregate is safely optional; optional_count remains contract-tested at zero in runtime config. | yes, after explicit classification plus producer/registry/tests |
| Side-effect commands | conditions remain pure; future commands are typed manager actions referencing configured device identities | Prevents retries/dwell evaluation from repeatedly issuing network effects and prevents arbitrary command injection. | yes at future feature design time |
| Test strategy | TDD for schema, evaluator, and runtime consumption; preserve all existing ROS regression tests | The current defect is parsed-but-unused configuration and requires a decisive red integration test. | yes |

## Findings (cited - path:lines)
- `src/multifloor_manager/src/multifloor_manager/configuration.py:255` parses strict `required`, `optional`, and `optional_count` data but returns it only as immutable configuration.
- `src/multifloor_manager/src/multifloor_manager/ros_node.py:155` owns the real transaction and currently never consumes `configuration.transitions`.
- `src/multifloor_manager/src/multifloor_manager/ros_node.py:170`, `:190`, `:231`, and `:247` already enforce tag, target-map, localization, and post-clear costmap order.
- `src/mission_manager/src/mission_manager/transitions.py:101` implements required conjunction, optional quorum, and dwell but is not wired to multifloor runtime.
- `test/fixtures/building_valid/transitions.yaml:4` retains the original `optional_count: 1` contract and includes two predicates that multifloor runtime does not own.
- `src/multifloor_manager/config/transitions.yaml:4` remains intentionally unconfigured for production commissioning.

## Decisions (with rationale)
- Replace `required`/`optional` arrays with ordered `conditions` entries containing `name`, exact integer `enabled`, and exact integer `required`; retain `optional_count` and temporal fields.
- Keep the outer document `configured` field as the repository-wide YAML boolean contract; use 0/1 only for operator-tuned transition and condition switches.
- Evaluate one `floor_transition_ready` policy at the final READY boundary after fixed transaction barriers; do not evaluate one sequential policy at incompatible pre-map and localization phases.
- Add `T_COSTMAP_READY`; remove `T_GOAL_REACHED` and `T_STAIR_EXIT` from the multifloor fixture because they have no current local evidence producer.
- Relocate evaluator ownership into `multifloor_manager`; never import `mission_manager.transitions` from multifloor and never add a ROS node.
- Reject unknown predicates, duplicates, all-disabled policies, disabled-required entries, invalid optional quorum, and attempts to weaken mandatory safety predicates at startup.
- Do not add generic action hooks, raw URLs, arbitrary IP payloads, shell commands, or network retries in this work. Document the future rule: a new pure predicate needs an allowlisted producer; a side effect needs a typed idempotent manager action plus separately validated device configuration.
- Treat floor confirmation as an epoch-latched event and localization/costmap readiness as live levels that write false as well as true during final dwell.
- Start policy timeout at `POLICY_READY`, independent of existing transaction phase budgets; require `dwell_sec < timeout_sec`.
- Require explicit epoch tokens for node-thread observation/evaluate/commit operations so epoch N work cannot enter N+1.
- Use the successful second actionlib preemption check as the decision point; do not claim atomicity across actionlib and runtime locks.
- Keep structure concise: one new pure `transitions.py`, one manager/observation state in runtime, one node final-gate helper, no generic dispatch or second FSM.
- There is no current non-safety condition. Keep only a comment example for `T_ARRIVAL_ANNOUNCEMENT_DONE`; copying it into configured conditions before producer/registry/tests must fail.
- For a future door, `OPEN_DOOR` is a typed action; if motion depends on the door, `T_DOOR_OPEN_CONFIRMED` is a fixed required safety condition.

## Scope IN
- Parser/domain migration, fixture migration, evaluator relocation, runtime policy injection, final READY gating, deterministic clocks/state reset, and directly affected tests/build registration.
- Operator documentation for condition editing and the predicate-versus-action extension rule.
- Non-Git, stale-generated-copy-aware evidence receipts and resumable task boundaries.

## Scope OUT (Must NOT have)
- New production ROS nodes, topics, actions, messages, production-manager/system launch parameters, or reverse package dependencies. One test-node-private outcome selector is allowed.
- Runtime policy authority over service ordering, epoch lifecycle, cancellation, map identity, localization causality, or costmap identity.
- Production policy activation or field tuning.
- Implementation of automatic doors, HTTP/TCP clients, arbitrary network commands, shell execution, expression languages, plugins, or dynamic imports.
- Direct edits under `build/`, `devel/`, or `install/`.

## Open questions
None. The user approved the bounded design, required concise/clear structure, prohibited Kimi-family agents, and requested a comment-only example showing how a future non-safety condition is added after producer/registry/tests exist.

## Approval gate
status: reviewed-and-approved
<!-- When exploration is exhausted and unknowns are answered, set status: awaiting-approval. -->
<!-- That durable record is the loop guard: on a later turn read it and resume at the gate instead of re-running exploration. -->
approved scope: bounded multifloor predicates, fixed safety barriers, no new node, future typed-action separation
plan path: `.omo/plans/yaml-transition-policy-runtime.md`
plan sha256: `0004e24b2a07fd1964877990a0b8cdfb837bf9126b61d136e1fed8ae9cea0b62`
plan shape: 9 implementation todos plus 5 final-verification tasks
review method: direct source/plan audit plus non-Kimi Oracle; Momus omitted because delegated model family could not be guaranteed under the user's no-Kimi constraint
oracle session: `ses_fcedc98faffehNqB3FzV1jfeSx`
oracle result: APPROVE for the exact plan digest above
next workflow action: start a separate worker with `$start-work yaml-transition-policy-runtime`; do not implement product code in this planning session
