---
slug: modular-architecture-expansion
status: lean-plan-reviewed-approved
intent: clear
review_required: true
review_model_constraint: no-kimi-family-agents
plan_path: .omo/plans/modular-architecture-expansion.md
plan_sha256: 674b2f67a7f0e6eccc9f3e47d9c4f566e991f227365b6c1ce1d8ad83d8892817
review_round_id: 583bfb46-74cc-4868-95a2-94d4351ad322
round_status: approved
pending-action: execute separately with $start-work modular-architecture-expansion
review:
  momus:
    status: approved
    workspace_root: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
    runtime_home: null
    target: .omo/plans/modular-architecture-expansion.md
    round_id: 583bfb46-74cc-4868-95a2-94d4351ad322
    plan_sha256: 674b2f67a7f0e6eccc9f3e47d9c4f566e991f227365b6c1ce1d8ad83d8892817
    launch_id: 0c183f7d-8b34-4bae-b503-a737a154b9f0
    session: ses_fc702f068ffe7GtFf7nCageYoi
    result: "OKAY: every Todo and F1-F4 has executable QA with no plan blocker"
  independent:
    status: approved
    workspace_root: /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
    runtime_home: null
    target: .omo/plans/modular-architecture-expansion.md
    round_id: 583bfb46-74cc-4868-95a2-94d4351ad322
    plan_sha256: 674b2f67a7f0e6eccc9f3e47d9c4f566e991f227365b6c1ce1d8ad83d8892817
    launch_id: 3b7b432a-b900-4f34-b0dc-4815e5a2d6b9
    session: ses_fc702ed75ffeFbeiGCGqD6fM7t
    result: "OKAY: lean architecture is decision-complete with direct command flow and no hidden safety complexity"
approach: Build a clean-slate modular ROS Noetic workspace at `/home/m3tron/Desktop/TRON1_Modular_Navigation/`, leaving the current workspace read-only. Use six directional catkin packages: interfaces, pure condition engine, three operational node packages, and zero-node bringup. Replace hardcoded mandatory transition names with a typed registry and explicit policy expression where every logical condition, including safety-classified conditions, is uniformly configurable as required, optional/quorum, or ignored; classification is diagnostic metadata. Keep finite/stale/cancel/shutdown/WebSocket handling visible in the existing stair command stream/transport path with no additional safety subsystem. Generate architecture diagrams and a traceability manifest mapping packages, nodes, interfaces, condition producers, and functions. Use lean behavioral, dependency, launch, and mini-PC-only QA; defer powered-TRON motion gates.
---

# Draft: modular-architecture-expansion

## Components (topology ledger)
<!-- id | outcome | status | evidence -->
1 | New Desktop workspace is created from authoritative source only and the current workspace remains read-only | active | current `src/`, root scripts/docs/tests; generated `build/devel/install` excluded
2 | Six-package directional architecture separates interfaces, condition evaluation, three runtime capabilities, and bringup | active | current package manifests, cross-package imports, `system.launch`
3 | Typed condition registry/policy DAG manages all logical conditions uniformly as required, optional/quorum, or ignored | active | current `transitions.py`, `configuration._load_transitions`, `ros_runtime.py`, `ros_node.py`
4 | Condition producers and evidence snapshots isolate tag/localization/map/costmap dependencies from evaluation | active | `tag_evidence.py`, `readiness.py`, `map_evidence.py`, current predicate writes
5 | Architecture-as-code maps every package, node, interface, producer, module, and key function to Mermaid IDs and tests | active | current launch/actions/messages and requested explicit logical diagrams
6 | Zero-node deployment/bringup owns launch, navigation/perception assets, RViz, mini-PC checks, and one operator wrapper | active | `run.sh:30-299`, `system.launch`, generic multifloor launch assets
7 | Lean offline and mini-PC QA proves behavior/dependencies now; powered-TRON motion QA remains an explicit deferred gate | active | current validators/tests, TRON-off constraint, mini-PC availability
8 | Cutover uses the new workspace only, retains the old workspace as rollback, and never launches both simultaneously | active | user requirement and current runtime ownership

## Open assumptions (announced defaults)
Target directory | `/home/m3tron/Desktop/TRON1_Modular_Navigation/` | Clear standalone workspace name; old workspace remains untouched | yes
Package topology | `tron1_interfaces`, `tron1_condition_engine`, `mission_manager`, `multifloor_manager`, `stair_supervisor`, `tron1_bringup` | Clean dependency direction and independently testable zero-node libraries/assets | yes
Runtime topology | Exactly three project nodes: mission, multifloor, stair | No evidence supports more runtime processes; modularity is package/function ownership, not node proliferation | yes
Condition modes | Every registered logical condition permits policy-selected `required`, `optional`, or `ignored` | Explicitly requested; removes name- and classification-hardcoded mandatory gates | yes
Condition classification | `safety/operational/diagnostic` is trace/report metadata only | Prevents classification from silently overriding the configured mode | yes
Ignored semantics | Ignored evidence is still produced and appears in evaluation traces but contributes no truth/quorum | Supports diagnosis and one-place management without hidden behavior | yes
Condition grouping | Explicit `all`, `any`, and `quorum` groups compiled into an acyclic dependency DAG | Handles mixed dependencies without if/elif chains or ambiguous optional counts | yes
Minimal validation | Validate only boundary shape/types, registered IDs/producers, group references, cycles, timing ranges, and producer evidence identity | Prevents silent drift and impossible evaluation while avoiding ceremonial exact-layout checks | yes
Command handling | In the existing command stream loop, nonfinite/stale input selects zero; cancel/shutdown makes a best-effort zero send while connected; WebSocket error enters FAULT and stops further commands | The user confirmed a low-risk environment with immediate physical stop, so no deeper software safety layer is required | yes
Interface cutover | Preserve action/topic/service/node endpoint names and field/result semantics, but change package-qualified ROS datatypes to `tron1_interfaces` in one coordinated cutover | Shared ownership is explicit; no compatibility bridge or simultaneous old/new runtime is introduced | yes
Interface ownership | Move shared `.action/.msg` definitions to `tron1_interfaces`; nodes depend on interfaces, not each other | Clean-slate scope now justifies independent shared wire ownership | yes
Condition engine ownership | `tron1_condition_engine` is a pure Python/catkin library with no ROS node or global mutable registry | Reusable, testable evaluator independent of callbacks and hardware | yes
Domain configuration | Mission, floor/map/tag, and stair/robot data remain with their capability package | Prevents bringup/common packages from becoming a data dumping ground | yes
Diagram source | `docs/architecture/architecture.yaml` is the canonical trace manifest; Mermaid files are derived/reviewable views | One machine-readable mapping can validate nodes/functions/interfaces/tests | yes
QA posture | One acceptance wrapper runs focused behavior, dependency, launch, config, diagram, and mini-PC checks | Explicitly avoids unnecessary validation while keeping executable evidence | yes
Hardware posture | TRON-powered navigation/stair/actuator/E-stop tests are deferred and cannot be claimed from offline results | TRON is currently off | yes

## Findings (cited)
- Current transition parsing hardcodes one policy ID, a closed supported-name set, all conditions enabled+required, mandatory-name presence, and `optional_count == 0` in `multifloor_manager/configuration.py:256-308`.
- Current evaluator already has useful pure epoch/freshness/dwell/timeout behavior in `multifloor_manager/transitions.py:115-221`, but it is flat and string-keyed.
- `ros_runtime.py` owns epoch-scoped evidence synchronization; `ros_node.py` mixes producer decisions and policy orchestration.
- Mission currently depends directly on multifloor/stair messages and configuration models through `navigation_executor.py`, `ros_state.py`, `ros_segments.py`, `site_config.py`, and `route_planner.py`.
- `run.sh:30-299` mixes validation, remote mini-PC lifecycle, tunneling, launch, process cleanup, and readiness polling.
- Current authoritative source is under `src/`; `build/`, `devel/`, `install/`, caches, logs, bags, `.omo`, credentials, and runtime artifacts must not be copied.
- Current `./run.sh --check` passes; root unit baseline is 98/100 with two layout-contract failures caused by counting helper scripts as runtime entrypoints.
- Offline validation can cover policy semantics, package DAG, launch graph, interfaces, configs, diagrams, and mock process boundaries; powered TRON is required only for motion/sensor/actuator claims.

## Decisions (with rationale)
- Do not launch any Kimi-family subagent. The interrupted Metis lane is not retried; gap analysis is performed directly. Momus and independent Oracle review may run only after their configured models are verified as non-Kimi, otherwise the affected review lane is reported unavailable rather than substituted with a prohibited model.
- Build only under `/home/m3tron/Desktop/TRON1_Modular_Navigation/`; execution begins by copying selected authoritative inputs, never by modifying the current workspace or copying generated trees.
- `tron1_interfaces` owns `Mission.action`, `FloorTransition.action`, `FloorState.msg`, `StairTraversal.action`, and `SupervisorState.msg`; implementation packages import only shared wire types, not each other.
- `tron1_condition_engine` owns typed IDs, immutable registry, policy compiler, evidence/status types, DAG evaluation, grouping, freshness, epoch, dwell, timeout, and structured traces; it has no ROS imports, node, side effects, or mutable singleton.
- Capability packages own condition producers. Producers receive immutable snapshots and return typed evidence; they do not call services, publish, mutate runtime, or decide transitions.
- Policy YAML declares condition definitions, producers, mode, explicit groups, and timing. No condition name is special-cased; changing a condition from required to ignored is a data change, not an evaluator edit.
- A YAML `producer` value is a stable registry key, never a Python import path or dynamically evaluated function name. Startup composition explicitly binds each key to one typed producer callable, so configuration cannot execute arbitrary code and dependencies remain inspectable.
- Condition producers are observation-only. External side effects such as opening an automatic door use a typed action/service owned by a capability adapter; a separate producer consumes the resulting cached state and emits `T_DOOR_OPEN_CONFIRMED` evidence.
- Extension granularity is evidence-driven: a pure derived condition adds one producer function; a cohesive set of producers adds one package-local Python module; independently deployed external hardware/network lifecycle adds one capability package/node plus a local evidence producer. Adding a node is never the default.
- The automatic-door HTTP example adds an optional `door_adapter` capability package/node only when deployed: it owns the provisioned endpoint, bounded HTTP request, response parsing, typed `OpenDoor` action, and `DoorState` publication. The condition policy never contains a shell command, IP, URL, header, or payload.
- If the implementation literally invokes `curl`, it uses a fixed argument vector with no shell expansion, fixed operation templates, bounded timeout, and secrets outside source control. HTTP acceptance and physical door-open confirmation are separate states; a 2xx response alone cannot emit `T_DOOR_OPEN_CONFIRMED=true` unless the configured protocol contract defines that response as authoritative state.
- Command handling is capability-scoped and remains in the command owner. The motion stream selects zero for stale/nonfinite input while connected, performs best-effort zero on cancel/shutdown, and enters `FAULT` on WebSocket error; a door HTTP adapter reports only its own action failure.
- Persistent-session errors are determined by the existing WebSocket send/receive path. Staleness is checked in the existing command stream loop from the monotonic receipt time of the last valid command. Nonfinite values are rejected before command-state mutation.
- No separate safety node, watchdog thread, command-guard state machine, delivery-acknowledgement model, or hidden automatic recovery is added. The existing loop selects/sends zero for stale or invalid input while connected; cancel/shutdown performs the existing best-effort zero close; WebSocket errors enter `FAULT` and stop further commands.
- A dead connection, hard process kill, host power loss, or kernel failure has no software delivery guarantee. The operating environment relies on an immediately accessible physical stop, documented plainly rather than modeled as another software subsystem.
- Condition truth has no hidden prerequisite or procedural gating. One explicit YAML `all` root is the sole truth source: required leaves are direct members, optional leaves occur once below an explicit optional subtree, and ignored leaves are absent from root but retained in traces. Mode changes may require a matching YAML expression edit but never code.
- Old-workspace immutability uses hashes only for the explicit authoritative source-selection list that the worker reads or recreates. Every product/build/temp write is under the new target; the only old-workspace write exception is the exact current `<attemptDir>` under `.omo/evidence/`. No complete inventory or credential/generated-tree hashing is required.
- Root `--check` is local-only, not process-free: it may run bounded verification subprocesses and write target generated/test outputs plus execution evidence, but never target source/config, the old workspace outside `.omo`, remote systems, hardware, tunnels, or persistent ROS runtime.
- Installed-space verification uses one clean ROS-plus-target-install smoke process after `catkin_make install`; no three-way source/devel/install provenance matrix is required.
- Final verifiers run sequentially `F1 -> F2 -> F3 -> F4` because build/install, ROS runtime, and provenance share mutable state; F3 must fully shut down before the final scope/provenance audit.
- `safety` classification does not constrain mode. An ignored safety condition remains visible as `ignored` in trace and cannot satisfy a group.
- Keep the straightforward stream-loop/transport error handling outside the policy engine; it is ordinary command ownership, not a hidden configurable transition or safety subsystem.
- `tron1_bringup` owns the only production system launch, navigation/perception launch assets, navigation parameters, RViz, and installed operator composition. It contains no custom node or domain logic.
- Root deployment is decomposed into zero-node modules behind one `run.sh`; mini-PC-only checks remain executable while TRON motion checks are reported deferred.
- `docs/architecture/architecture.yaml` maps stable IDs to package, node, entrypoint, module, function/symbol, public interface, condition producer, config key, diagram edge, and tests. Mermaid views cover package DAG, runtime node graph, condition evaluation, mission sequence, floor transition, and command ownership.
- Verification rejects unresolved diagram IDs, undeclared package edges, import cycles, ROS imports in the pure condition engine, hidden node initializers, stale generated imports, and behavior changes in required/optional/ignored evaluation.

## Scope IN
- Complete clean-slate package/module layout in the new Desktop workspace.
- Shared interface package and pure condition-engine package.
- Three operational nodes and one zero-node bringup package.
- Uniform condition modes, an explicit grouped expression DAG, evidence provenance, concise traces, and configurable timing.
- Migration/rewrite of existing mission, floor/map/tag/localization, stair/robot transport, scan, deployment, launch, config, and RViz capabilities into explicit owners.
- Architecture manifest, Mermaid logic diagrams, function/node/condition traceability table, and generated correspondence report.
- A documented extension example for automatic-door command/action, state topic, registered confirmation producer, explicit expression membership, and traceability mapping.
- Focused tests and offline/mini-PC acceptance wrapper, source/install provenance, and rollback/cutover documentation.

## Scope OUT (Must NOT have)
- No edits to the current workspace during implementation.
- No copied `build/`, `devel/`, `install/`, cache, logs, bags, `.omo`, credentials, or generated artifacts.
- No extra operational nodes for policy, diagrams, deployment, bridges, UI, muxing, safety, localization, or sensor fusion.
- No mutable global condition registry, runtime plugin loading, string-name if/elif evaluator, hidden producer side effects, or condition-defined commands/URLs/socket payloads.
- No hidden condition prerequisite gates, safety node, watchdog thread, command-guard state machine, delivery acknowledgement model, full old-workspace inventory, or automatic reconnect layer.
- No condition classification that silently changes configured mode.
- No duplicate robot-command transport or `/scan` publisher.
- No claim of TRON navigation, stair, actuator, emergency-stop, or physical safety readiness while TRON is powered off.
- No tests that pass only by grep/self-report, and no broad exact-file-layout ceremony unrelated to behavior/dependency contracts.
- No simultaneous launch of old and new systems during cutover.

## Open questions
None. The user explicitly selected the minimal low-risk structure: condition truth remains explicit in the policy expression, command handling stays in the existing stream/transport code, and an easily accessible physical stop is the final response when software cannot act.

## Review round history
- Round `7b80ff8d-1b45-490e-a522-20729cc15183`, plan SHA-256 `a345668d7cb4adb0f01169a250f8ce762397c07a12be53d1c01f44eb56bb4bcd`.
- Momus session `ses_fc7385d5fffe9cXxZ80H3vyrZM` (`openai/gpt-5.6-terra`, non-Kimi): REJECTED selective provenance versus whole-workspace immutability.
- Independent session `ses_fc7385afcffeDy52eTLgMPutpV` (`openai/gpt-5.6-sol`, non-Kimi): requested aligned `--check` side effects, complete provenance, explicit mode-edge semantics, truthful zero delivery plus active watchdog, isolated installed-space verification, and serialized final verification.
- All cited findings were incorporated into the plan; the changed plan invalidates both receipts and requires a fresh dual-review round.
- Round `57b135ea-1ded-471c-9381-bcbc37b2d4f4`, plan SHA-256 `9d51e227270182894d1e6ddf4c9ee127553c4922da29c1a0f4d427414d194919`.
- Momus session `ses_fc72db0b2ffewaYkVP9JYiN4wP` (`openai/gpt-5.6-terra`, non-Kimi): unconditional OKAY.
- Independent session `ses_fc72dae07ffemgPb4rfvgpXHX0` (`openai/gpt-5.6-sol`, non-Kimi): unconditional OKAY with no blocking changes.
- The user then rejected the approved plan's safety/provenance depth as excessive for a no-injury, easy-physical-stop environment and approved a simplified revision. The prior digest remains historical and is not valid for handoff.
- Simplified round `b93340e3-79b5-4466-b84d-39edd25f6443`, plan SHA-256 `aa2817cab55299fedfc49af1c0b979b525ad966cfee893ba13542c599a53928e`.
- Momus session `ses_fc717bebaffeFx5y0YGlGnyqHF` (`openai/gpt-5.6-terra`, non-Kimi): OKAY.
- Independent session `ses_fc717bc07ffeKyY5WpsDAzsRsk` (`openai/gpt-5.6-sol`, non-Kimi): requested one explicit final root as sole truth and one exact `.omo/evidence/<attemptDir>` write exception.
- Both issues were corrected in the plan, invalidating that round and requiring one fresh dual review.
- Final-simplified round `09a2fc38-d8f5-4724-bb3a-cecc3af9007a`, plan SHA-256 `d4ce116bad62060ac35bccd3147cf75c0bacdbf3b0eecf5d38cdbd90411a312a`.
- Momus session `ses_fc70ff8a9ffeKdeD0Z3YNkStTo` (`openai/gpt-5.6-terra`, non-Kimi): requested removal of the broad evidence fallback and exact agent-runnable F1-F4 QA.
- Independent session `ses_fc70ff512ffej8V39gM0jFUVfH` (`openai/gpt-5.6-sol`, non-Kimi): requested one concrete bound `currentAttemptDir` with no fallback.
- The plan now stops before writes when `currentAttemptDir` is absent/unsafe and gives each final verifier exact commands, assertions, outcomes, and evidence paths; the changed digest requires a fresh dual review.
- Lean round `241ff088-73fb-4c81-b266-b559388bd582`, plan SHA-256 `71705e7db9960e3ff7d255fcb015ae179bdc28a2aecd125ced6affb7b8528d36`.
- Momus session `ses_fc70948deffeJvxKA1KgMFOW7M` (`openai/gpt-5.6-terra`, non-Kimi): requested explicit commands in each implementation QA line.
- Independent session `ses_fc70945ebffeotlrPFReTN5XbC` (`openai/gpt-5.6-sol`, non-Kimi): unconditional OKAY for the lean architecture.
- Todo 1-18 QA now names a concrete Bash/rostest/unittest/roslaunch invocation, happy/failure assertions, pass condition, and evidence path; this plan-only formatting change requires a fresh dual review.
- Command-complete round `583bfb46-74cc-4868-95a2-94d4351ad322`, plan SHA-256 `674b2f67a7f0e6eccc9f3e47d9c4f566e991f227365b6c1ce1d8ad83d8892817`.
- Momus session `ses_fc702f068ffe7GtFf7nCageYoi` (`openai/gpt-5.6-terra`, non-Kimi): unconditional OKAY.
- Independent session `ses_fc702ed75ffeFbeiGCGqD6fM7t` (`openai/gpt-5.6-sol`, non-Kimi): unconditional OKAY; explicitly approved the direct stream/transport flow and absence of hidden safety abstractions.

## Approval gate
status: approved-for-simplified-plan
The user explicitly approved the minimal low-risk structure after rejecting the previous safety/provenance depth. Approval authorizes rewriting and high-accuracy reviewing `.omo/plans/modular-architecture-expansion.md` only. It does not authorize implementation; execution remains a separate `$start-work modular-architecture-expansion` session targeting `/home/m3tron/Desktop/TRON1_Modular_Navigation/`.
<!-- When exploration is exhausted and unknowns are answered, set status: awaiting-approval. -->
<!-- That durable record is the loop guard: on a later turn read it and resume at the gate instead of re-running exploration. -->
