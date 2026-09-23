# VERIFY_ENTRY timeout repair — 2026-09-23

The existing Supervisor has been updated and restarted through the existing `run.sh`. No traversal goal was submitted. This is a focused repair, not a completed repository audit or a claim of successful physical stair traversal.

## Findings

**OBSERVED — Diagnostic publication crashed. Safety S1 / Mobility M3.** The session log records `TypeError: Object of type bool_ is not JSON serializable` in `ros_lidar.py` while publishing control debug. The last diagnostic messages were recorded about 0.5 seconds after the entry phase began; action feedback continued until the phase timeout. A subsequent live three-second subscription received no tracking status or control debug messages. The installed ROS timer calls its callback without catching this exception: `/opt/ros/noetic/lib/python3/dist-packages/rospy/timer.py:240`. The original code reproduced the serialization failure in two regression cases.

**OBSERVED — Entry test timed out; INFERRED — strict instantaneous settling was the persistent blocker. Safety S1 / Mobility M2.** The route gives VERIFY_ENTRY five seconds independently of the requested 255-second total. Available live debug passed support, target position, and heading checks, while settling remained incomplete. Diagnostics did not cover the full five seconds, so the final live checks cannot be reconstructed exactly from debug. Reprocessing the original LiDAR and IMU yielded no entry completion over the corresponding five-second interval under the old policy. Instantaneous pose differences repeatedly exceeded the 0.03 m/s and 0.04 rad/s settling limits. Later in that reconstruction the robot also moved beyond the five-centimetre target tolerance. Wheel odometry independently recorded about 12.8 cm net displacement over the test interval, but wheel slip and localization error remain possible.

Source recording: `logs/stair_test_20260923_091321_pEOWV9/stair.bag`. Original system log: `logs/20260923_081522/system.log`. JSON extracts, reconstructed poses, and comparisons are retained alongside this report.

## Changes and constraint decision

`ros_lidar.json_finite` converts NumPy scalar and array values before JSON encoding and retains null conversion for nonfinite numeric values.

**NARROW — entry settling only.** VERIFY_ENTRY and ALIGN now measure net displacement and net yaw change over the observation window, divided by its duration. Position excursion and yaw excursion limits remain enforced. The implementation explicitly names this `window_net_drift`: opposing movements can cancel, so passing it permits bounded motion and does not declare physical stillness. A regression case explicitly covers a small final movement after a quiet prefix. Sustained translation or rotation above the existing limits still prevents transition.

This removes an entry blockage caused by short pose differences while retaining a minimum observation duration, new phase observations, target pose tolerances, footprint support, attitude checks, and freshness checks. The config values, five-second entry budget, phase order, speed limits, and new-goal ownership rules are unchanged. Turning and exit settling still use instantaneous peaks. Their physical suitability has not been newly validated by this repair. No blanket KEEP verdict is assigned to existing constraints.

Control debug now includes the settling method, window duration, excursion, evaluated net rates, peak rates, and rate thresholds. If sustained drift blocks entry again, the operator can recover and reposition on the entry floor, then create a new preview; no automatic re-anchoring or traversal retry was added.

## Validation and limits

- Applied-source regression suite: **99 passed**; original-source serialization negative controls: **two failures reproduced**.
- Fixed-sensor-sequence replay: old policy had no entry completion within five seconds; updated policy completed VERIFY_ENTRY at **0.6005 s** and ALIGN at **1.8006 s**. These are offline policy decisions on reconstructed measurements, not measured physical completion times. The counterfactual controller would change subsequent motion; this replay cannot establish closed-loop stair success.
- Live observation after restart, 2026-09-23 09:32:53 KST: **80 tracking statuses and 80 control debug messages over 8.007 s**, TRACKED, correct configuration hash, no worker error. Reported geometry age **0.2714–0.3730 s**. Supervisor NAV and connected; no active mission or stair action. All **320 observed twist messages were zero**.
- **Safety verdict:** physical stair behavior and localization accuracy remain UNVERIFIED. This change accepts bounded entry motion, and zero velocity is not a physical hold guarantee.
- **Mobility verdict:** the recorded entry blockage is removed in offline policy evaluation; live diagnostics are restored. A user-controlled physical retry is still required.

The independent cold read identified cancellation of opposing movements and dilution of recent motion by a quiet prefix. The code comment, diagnostic name, and explicit bounded-motion regression document this behavior rather than calling it physical stillness. Synthetic checks and replay are not independent position ground truth.

## Deployment and artifacts

Two production source files and two existing test modules changed. Baseline hashes, originals, final patch, and test results are retained. The unchanged route config and Supervisor state-machine source were hash-verified; existing worktree changes were preserved. Git porcelain added only the new operational session log entries because the edited source files were already untracked. See `verification.json` for exact scope; this was not a whole-worktree content inventory.

Temporary candidate copies and probe log directories were removed. Evidence and operational logs are intentionally retained. The existing local stack is running as session PID 465448; automatic BAG recording is disabled, and no new motion goal was sent. The normal startup script also reported restarting mini-PC mapping after the ROS master restart. Normal NAV startup was still waiting for map localization when inspected; the explicit stair test uses the independent, manually declared entry path. The entry preview must be recreated after restart.
