# Stair full forward input — 2026-09-23

User requested normalized forward input **x=1.0** for stair ascent and confirmed the robot was on the entry flat floor. The current 3F→4F test route now has `min_flight_v=max_v=0.55`. Its unchanged normalization scale is 0.55, yielding x=1.0 before slew and support checks in both forward phases.

The floor overrides the preceding nominal speed, speed feedback and heading/lateral speed reductions. Steering remains active. Slew, footprint support and margin checks remain active and can reduce the final command. Nominal ramp from zero is 0.55/0.15 ≈3.7 seconds. Hold and angular settings, node inventory and phase order are unchanged. A subsequent landing-preservation fix now caps flat-phase linear commands after slew; see landing-preserve/REPORT.md. This is normalized command input, not proof of measured speed or torque.

OBSERVED: 82 regression tests passed, including both forward phases reaching x=1 while steering and acceleration limits remain active. Re-evaluating 440 recorded body poses reached x=1 at 3.784 seconds and retained it thereafter without boundary reductions or faults. This is a fixed-pose command calculation, not a prediction of the new physical trajectory. Physical climbing and higher-input stopping/turning behavior remain UNVERIFIED.

The existing run.sh was restarted to load the setting. See ready.json for the initial loaded configuration hash and NAV waiting state. The later robot reboot and recovery are recorded under landing-preserve/. No traversal goal was submitted. Automatic BAG recording is off; a new recording and entry preview are needed for the operator’s next test.

The earlier readiness report’s 0.50–0.55 command range describes the preceding tuning. The active YAML is authoritative for this change. Original files and patch are retained; existing worktree changes were preserved. New session logs are operational artifacts, and probe logs were removed. See verification.json.
