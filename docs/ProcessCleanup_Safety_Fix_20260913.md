# Scoped process cleanup fix

User-approved scope: remove cross-task process killing from two legacy test
runners and stop the three explicitly identified PC ground-station remnants.
No planner, flight-control, UDP, scenario parameters or launch files may change.

## File whitelist

- src/swarm_planner/bspline_traj/test/batch10_scenarios.py
- src/swarm_planner/bspline_traj/test/pillar_left_to_right_stability_test.py
- src/swarm_planner/bspline_traj/test/process_cleanup_safety_test.py
- this record

## Required behavior

- No pgrep/name-wide kill, and no killing a port owner discovered through ss.
- Track only Popen handles created by the current script. Each managed child
  has a private process group; graceful stop and escalation address that group
  only while its directly owned process is alive.
- Normal cleanup is idempotent, handles already-exited children, and closes
  log handles. Startup failure must not leak an open log.
- Occupied test port is an error, not authority to stop its owner.
- Cleanup on ordinary interpreter exit uses the same owned-handle registry.
- Do not claim attribution for the earlier SIGKILL without a sender audit log.

## Verification

Syntax validation and ROS-free tests: AST-load only process helpers (do not
import/run ROS scenarios), spawn test-owned sleepers, verify tracked children
stop while an untracked sleeper named roslaunch survives, verify idempotence
and occupied-port refusal, and statically reject broad-kill code paths.
Run relevant git diff/status/check; preserve unrelated worktree edits.

Baseline: branch pro_review_current_20260903,
HEAD d3adc7370d3f4e76c1ed0b071fe17cccc8e6cacf. Both target scripts had no
pre-existing tracked diff; the rest of the worktree was dirty and is preserved.

The separately approved operational cleanup is limited to the verified old
ground_station_receiver / visualizer / goal processes (PIDs 687448/687449/687451),
with identity rechecked before signaling. No name-wide process kill is allowed.

## Result

Completed the localized cleanup patch. Process handles are tracked per script;
the existing stop method is used on those handles only. No process discovery,
monitor service or ROS/UDP behavior was added. The pillar runner's existing
read-only pgrep liveness check remains; it sends no signal.

Three ROS-free regression tests pass for both runners: tracked-vs-untracked
process isolation (including an unrelated test-owned process named roslaunch),
log cleanup on spawn failure, and refusal when the test port is occupied.
Full Python syntax checks and scoped git diff --check pass. No ROS scenarios
or aircraft processes were started for verification.

Ground-station PIDs 687448/687449/687451 were identity-checked, sent SIGINT,
and confirmed exited. No global kill command or port-based killing was used.
Other simulation and aircraft processes were not targeted. No commit made.
This batch is complete; no further cleanup or unrelated changes are authorized.
