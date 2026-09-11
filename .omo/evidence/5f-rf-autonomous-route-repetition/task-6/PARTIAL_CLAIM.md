# Task 6: partial fixture no-motion evidence

## Scope

This is partial fixture-only evidence, not production 5F-RF acceptance. The
exact 5F-RF-shaped `record_route` mock and its isolated failure matrix are now
proven in software, but production configuration is still blocked by Task 5's
missing independent survey and remains `configured: false`.

## Results

The following deterministic fixture tests were exercised without robot, SSH, or
WebSocket contact:

```text
source devel/setup.bash && python3 -m pytest \
  src/mission_manager/test/test_mission_orchestrator.py \
  src/mission_manager/test/test_route_planner.py -q
19 passed

source devel/setup.bash && python3 -m pytest \
  src/mission_manager/test/test_fsm.py \
  src/mission_manager/test/test_navigation_executor.py \
  src/mission_manager/test/test_scan_recorder.py -q
49 passed

python3 -m pytest src/stair_supervisor/test/test_stair_evidence.py -q
7 passed

rostest mission_manager mission_system.test  # loopback ROS master
RESULT: SUCCESS
TESTS: 15
ERRORS: 0
FAILURES: 0

export ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
source devel/setup.bash && rostest mission_manager mission_5f_rf_synthetic.test
RESULT: SUCCESS
TESTS: 9
ERRORS: 0
FAILURES: 0
```

The ROS surface covered busy handling, cancellation and resume, child
communication failure, dirty scan artifact rejection, validated scan return,
localization failure, malformed/unknown goals, incoherent child success,
multi-floor child ordering, bounded navigation retry, repeated interruptions,
scan failure, stair failure, and stale state. The final fixture artifact directory
was not retained as a Task 6 acceptance artifact in this evidence record.

The synthetic 5F/RF ROS surface covered the exact directed route through
`Mission.action`: 5F navigation, stair ascent, roof transition, four roof-loop
navigation segments, fresh return planning, stair descent, 5F transition, and
home navigation. It also isolated navigation abort, stale localization,
wrong-floor tag, odometry jump, missing IMU slope, WebSocket loss, scan-record
startup failure, and cancellation. The route-wide `record_route` case produced
one completed `.bag` artifact and 11 ordered route feedback segments.

## Test-only repair

During the recorded test run, the test launch wrote artifacts under `/tmp`, whose
free-space ratio was below the fixture's 0.35 threshold. `mission_system.test` now writes
only test artifacts under `/dev/shm`; the production free-space policy and YAML
were not changed. This was verified by the successful loopback rostest above.

## Remaining boundary

This does not prove the requested 5F-RF production route because Task 5 has no
reviewed 5F/RF map correspondences, endpoint poses, detector evidence, or frozen
ascent/descent profiles. No physical task was started from this artifact.
