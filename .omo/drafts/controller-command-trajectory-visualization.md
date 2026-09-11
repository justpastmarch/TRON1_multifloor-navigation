---
slug: 5f-rf-autonomous-route-repetition
status: planned
intent: clear
review_required: false
pending-action: execute .omo/plans/5f-rf-autonomous-route-repetition.md in a separate work session
approach: Represent the recorded drive as surveyed map-frame locations and bidirectional stair transitions, execute it through the existing closed-loop Mission path, and use the bag only as commissioning and comparison evidence.
---

# Decision Record: 5F-RF Autonomous Route Repetition

## Goal
- Autonomously repeat the recorded 5F-to-rooftop route and return through `Mission.action`.
- Use AMCL/move_base for closed-loop planar motion and the robot's high-level stair mode with fresh physical phase evidence for stairs.

## Fixed decisions
- The application is the sole normal-motion sender; the physical remote remains neutral and is used only for the manufacturer emergency-stop procedure during commissioning.
- Raw SensorJoy and the existing bag are evidence only. Blind timed Joy replay is excluded because the recording has a common gap and open-loop replay would drift.
- `stair_supervisor` remains the only WebSocket owner. No fourth runtime node, command mux, low-level joint control, or FAST-LIO navigation dependency is added.
- The route is represented as surveyed 5F/RF map-frame locations, directed graph edges, and direction-specific stair endpoint hypotheses.
- Physical success is established through sequential network/stop, flat-floor, ascent, descent, and complete-Mission gates. RViz comparison does not substitute for robot use.

## Confirmed blockers
- Production site and stair YAML remains disabled.
- The current graph omits 5F-to-RF `STAIR_UP` and encodes RF-to-5F as an invalid cross-floor `NAV` edge.
- The current stair schema has only one landing hypothesis and tag set, so it cannot represent both directions.
- Stair phase Bool subscriptions have no production publisher.
- The source bag has Joy but no exact outbound WebSocket transcript.

## Evidence rules
- Preserve the 225-239 s recorded-input/sensor gap and the 236.838 s wheel-odometry discontinuity as rejected evidence.
- Treat resemblance to the source bag as diagnostic because that bag also informs waypoint proposals.
- Accept the route only from independent surveyed endpoint error, live localization, external video, frozen-profile validation runs, and a successful physical Mission artifact.

## Scope
- In: bidirectional config correction, in-process stair evidence, exact command logging, reviewed 5F/RF site data, automated failure tests, and sequential hardware commissioning.
- Out: Joy-to-command replay, interpolation across outages, simultaneous remote/application control, automatic motion retry, new runtime nodes, and fabricated production values.

## Handoff
- Canonical execution plan: `.omo/plans/5f-rf-autonomous-route-repetition.md`
- The plan supersedes unchecked hardware/site Tasks 17-20 in `.omo/plans/tron1-multifloor-mvp.md` for the 5F-RF objective.
- Open questions: none.
