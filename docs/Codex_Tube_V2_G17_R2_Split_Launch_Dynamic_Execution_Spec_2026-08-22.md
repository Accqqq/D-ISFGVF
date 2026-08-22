# Tube V2 G17 R2 split-launch dynamic execution spec

    DOCUMENT_ROLE=DYNAMIC_ACCEPTANCE_EXECUTION_SPEC
    DOCUMENT_STATUS=ACTIVE
    DATE=2026-08-22
    PLAN_OWNER=PRIMARY_CODEX_AGENT
    EXECUTOR=LUNA_MAX_SINGLE_AGENT
    SUBAGENT_CREATION_ALLOWED=false
    PRODUCT_EDIT_ALLOWED=false
    LAUNCH_PARAMETER_EDIT_ALLOWED=false
    USER_OR_EXTERNAL_ROS_PROCESS_SIGNAL_ALLOWED=false

## 1. Purpose and entry state

This run determines whether the completed G17 authority-subset implementation
fixes the real split-launch construction and navigation failure.

Static entry evidence is frozen at:

    /tmp/tube_v2_g17_r2_20260822_174055/

The G17-focused implementation, R1 raw-evidence preservation, and R2 locked
H2 identity capture passed their focused suites and full build.  The direct
matrix is 456/457.  The single failing SurfaceValidator fixture is a recorded
pre-G17 baseline contract failure: its production header, implementation, and
test hashes are unchanged.  Dynamic execution must not modify that fixture or
Validator to manufacture a green result.

This is a read/execute/record acceptance run.  It authorizes no source,
parameter, launch, map, or test changes.

## 2. Isolation and storage

The root filesystem has less than 0.5 GB free.  Put the complete run under a
fresh directory of the form:

    /dev/shm/tube_v2_g17_dynamic_<timestamp>/

This includes ROS_HOME, ROS_LOG_DIR, bag, copied harness, readiness output,
derived reports, and all launch logs.  Do not write a bag or ROS logs to `/`,
`/tmp`, or the workspace.

Before launching:

1. Record `df`, current listeners, ROS masters, and the full process scan.
2. Select a fresh unused loopback port; verify it is unused immediately before
   `roscore` starts.
3. Set `ROS_MASTER_URI`, `ROS_HOME`, and `ROS_LOG_DIR` only for the task-owned
   shell/process group.
4. Record PID, PGID, start time, command line, binary Build-ID/SHA256, launch
   SHA256, map SHA256, and workspace source hashes.
5. Never attach to, signal, kill, or clean any existing ROS master, planner,
   simulator, RViz, recorder, map node, or launch process.
6. Use a cleanup trap and only the exact PIDs/process group created by this
   run.  Do not use global `rosnode kill`, `pkill`, `killall`, or name-based
   cleanup.

## 3. Recorder-before-publishers order

Start in this exact order:

1. private `roscore`;
2. bounded low-rate rosbag recorder;
3. separate simulator launch;
4. separate user planner launch;
5. readiness observer;
6. five-goal state-driven harness.

The recorder must be alive and subscribed before simulator/planner publishers
start.  Record at least:

    /rosout
    /move_base_simple/goal
    /sim/odom
    /position_cmd
    /particle0/path
    /formation_planning/phase_offset_manual/base_path
    /formation_planning/phase_offset_manual/active_path
    /formation_planning/phase_offset_manual/frame
    /formation_planning/phase_offset_manual/tube_candidate
    /formation_planning/phase_offset_manual/tube
    /formation_planning/phase_offset_manual/diagnostics
    /formation_planning/phase_offset_manual/tube_epoch_diagnostics
    /formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
    /formation_planning/phase_offset_manual/tube_cloud_snapshot_diagnostics

Do not record the full high-rate local-map stream.  Readiness may subscribe to
it without storing it in the bag.

## 4. Unchanged separate launches

Launch separately and do not add another phase-offset launch:

    roslaunch so3_quadrotor_simulator simulator.launch

    roslaunch bspline_race test_gvf.launch \
      phase_offset_mode:=manual \
      phase_offset_manual_amplitude:=0.10 \
      phase_offset_manual_observe_only:=false \
      phase_offset_manual_profile_period:=2.0 \
      phase_offset_manual_tube_source:=esdf \
      phase_offset_tube_fixed_delta_max:=0.04 \
      phase_offset_tube_cloud_obstacle_set_complete:=true \
      phase_offset_tube_preincluded_map_uncertainty:=0.10

Every unspecified argument remains at its checked-in default.  Do not change
query/depth/attempt limits, margin, rate, horizon, map, gains, or timing.

## 5. Readiness gate

Do not publish G1 until all facts are recorded:

- private master responds;
- recorder is alive and has topic subscriptions;
- simulator odometry is nonempty;
- `/formation_planning` is present;
- local map has at least the previously used 900 valid messages;
- mock map, occupancy, ESDF, and update-range each have valid nonempty data;
- expected phase-offset candidate, certified, epoch, raw, and cloud topics are
  advertised;
- formation-planning and phase-offset library hashes match the static R2
  artifacts.

Readiness failure is an infrastructure result.  Preserve it and clean only
task-owned processes; do not compensate with parameter or source changes.

## 6. Five goals and state-driven progression

Copy the established G14 harness to the run directory and only rename its node
and log prefixes to G17.  Do not edit its coordinates, reach timeouts, goal
order, or state-driven behavior.

    G1 = ( 7.381,   0.378, 0.0)
    G2 = (-6.823,  -0.685, 0.0)
    G3 = ( 0.537,  12.745, 0.0)
    G4 = (-0.301, -13.714, 0.0)
    G5 = (-7.913,  13.439, 0.0)

Publish each goal once.  G1 through G4 may advance only after a new
`[POINT_GOAL][REACHED]` event after that goal's publication.  Never use a short
fixed sleep to overwrite an active goal.  If any goal times out, do not publish
later goals; preserve the exact first-false evidence.

For G5, accept either a real REACHED event or an evidence-complete bounded
terminal classification.  A terminal classification is not success and may
not be called FIXED.

The overall harness stays bounded by the established per-goal 240 seconds and
overall 1500-second limit.  No timeout extension is authorized.

## 7. Mandatory dynamic facts

Derive all of the following from bag and launch logs, with timestamps and
candidate/map/session identity where available:

1. Goal publication and REACHED/timeout result for each goal actually sent.
2. Candidate marker ADD/DELETE counts and broad raw interval/width examples.
3. Certified marker ADD/DELETE counts and certified interval/width examples.
4. Proof that candidate remains UNCERTIFIED and certified ADD occurs only for
   `active_display_certified=1` or equivalent active certified status.
5. Requested authority interval; expected configured neutral request is
   `[-0.10,+0.10]`, expanded only by existing interior margin or retained
   delta when those facts require it.
6. Validator attempt, query, geometry-cell, leaf-cell, split-v, limit, and
   first-failure facts for broad raw cohorts and actual narrow requests.
7. Whether the former `249993/250000` full-I_geo explosion recurs.  It is only
   acceptable if the narrow request itself is directly proven complex.
8. Whether any exact-current zero-capacity cohort performs 101 inward attempts;
   G16 D0 requires zero equivalent inward attempts.
9. Every nonzero Pair bootstrap/commit/selection/execution and retained delta.
10. H2 replacement stage/commit timeline, path/profile/session identity, and
    direct failure classification if it does not commit.
11. Neutral planner continuation during Tube failure.  There must be no stable
    `candidate_count=36 valid_count=0 fallback_reason=all_candidates_path_end_clamped`
    HOLD when safe neutral continuation is available.
12. Any Runtime dry-run or joint-port denial remains fail-closed and is not
    hidden as construction success.

## 8. Classification

Write `FIXED` only if all of these are true:

- G1 through G5 all reach;
- certified Tube has genuine ADD geometry;
- at least one nonzero certified Pair commits and executes;
- H2 replacement either commits matching narrow authority or has a direct safe
  classification without causing navigation failure;
- the former full-raw query explosion and 101-equivalent inward loop do not
  recur on authority-subset cohorts;
- candidate/certified marker ownership stays separate;
- no stable 36/0/36 path-end HOLD blocks an otherwise valid neutral path.

Otherwise write `PARTIALLY_FIXED` or `NOT_FIXED`, name the first failed owner,
and retain the shortest direct evidence.  Do not re-enter M6, add a recovery
controller, change parameters, add gates/retries, or call broad raw geometry a
bug.

## 9. Cleanup and post-run audit

After the harness ends or times out:

1. stop only the recorded task-owned harness, planner launch, simulator launch,
   recorder, and private roscore, in that order;
2. allow rosbag to close cleanly and run `rosbag info`;
3. verify the chosen port is free and task-owned PIDs/PGID are gone;
4. record after process/listener scans;
5. recompute binary, launch, map, and G17 source hashes and compare before/after;
6. run `git diff --check` read-only and prove the dynamic run made no product
   edits;
7. produce concise TSV/Markdown summaries for goals, markers, Validator costs,
   Pair/H2, zero baselines, HOLD events, cleanup, and final classification.

Report the evidence directory and classification to the primary agent.  Do not
modify product code or start another run without a new primary-agent plan.
