# PhaseOffsetSwarm B1 G4D Independent Encounter Read-Only Diagnostic Execution Specification

Date: 2026-08-22

Repository:

    /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws

Authorized stage: add test-only bounded encounter instrumentation, perform one
diagnostic replay of the unchanged B1 three-independent-UAV scenario, classify
the UAV0-UAV1 proximity event, write a self-audit, and stop.

Execution owner: the existing Luna Max agent, supervised by the root agent.
Luna Max must execute personally. It must not spawn, delegate to, consult,
request, message, or wait for any sub-agent, nested agent, other model, or
parallel worker. If it cannot continue personally, it must stop and report.

AUTO_ADVANCE=false. This stage does not authorize a passing B1 result, a
scenario correction, another acceptance rerun, B2 live shadow integration,
B3 active swarm, allocator, CBF, Tube activation, controller changes, or paper
experiments.

## 1. Authority and precedence

This is a new root-owned diagnostic stage following the valid-but-failing B1
G4 run. It is governed by:

    AGENTS.md
    docs/Codex_PhaseOffsetSwarm_B1_Independent_Three_UAV_Navigation_Execution_Spec_2026-08-21.md
    docs/Codex_PhaseOffsetSwarm_B1_R1_Legacy_Goal_Height_Semantic_Alignment_And_Independent_Three_UAV_Execution_Spec_2026-08-21.md
    docs/Codex_PhaseOffsetSwarm_B1_R1_G3R_Invalid_Fixture_Correction_Execution_Spec_2026-08-21.md

This document overrides the prior STOP only for the bounded read-only
instrumentation and one diagnostic three-agent run described here.

The previous B1 pair criteria remain unchanged:

    minimum pair distance > 0.80 m
    samples below 0.60 m = 0

The diagnostic run is expected to retain a failing JUnit result if the same
pair criterion fails. This stage must not turn the known failure into a pass.

## 2. Frozen repository identity

Before every edit or dynamic run:

    branch = main
    HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43

Frozen specification and source hashes:

    85e5750a72fb64758b891eed2355e4d6090454e28d402b51dc6605fe980356c7  docs/Codex_PhaseOffsetSwarm_B1_R1_Legacy_Goal_Height_Semantic_Alignment_And_Independent_Three_UAV_Execution_Spec_2026-08-21.md
    58f33031f7a843b5205e78e638979059c3a827b670688b73e0f2d8d5a46906e3  docs/Codex_PhaseOffsetSwarm_B1_R1_G3R_Invalid_Fixture_Correction_Execution_Spec_2026-08-21.md
    67be9c324a88957a9e4bd172666c06758a9d58cff869d65aee8d8429a65285f1  src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py

The complete protected/B1 manifest recorded by the B1E final self-audit and
B1-R1 G0 remains authoritative. Recompute it before editing and after the
diagnostic run. Only the authorized test and new G4D self-audit may differ.

## 3. Frozen G4 failure evidence

The valid-but-failing G4 directory is:

    /tmp/codex_b1_r1_g4_three_20260821T125523Z_EkGLGC

Frozen evidence hashes:

    ce95605957a8188865d563df234930ae804537dc7d62917a92ac27b616c26d68  test_results/bspline_race/rosunit-b1_independent_three_navigation.xml
    d0614a44086fd5830330baa5f68fbe51dc3718effe9e12f7c2c66b95f6f500d0  run_identity.txt

The directory is read-only evidence for this task. Do not write, delete,
rename, normalize, regenerate, or reuse any file below it.

Authoritative G4 facts:

- all three UAVs published exactly one goal;
- published z remained 1.0 m and effective accepted z remained 2.0 m;
- all three stable arrivals passed;
- all stream, finite-data, rate, path, and obstacle-clearance metrics passed;
- UAV0-UAV1 minimum distance was 0.3304317698720089 m;
- UAV0-UAV1 had 300 directed matched samples below 0.60 m;
- UAV0-UAV2 minimum distance was 2.071212154751052 m;
- UAV1-UAV2 minimum distance was 1.2258342030113403 m;
- the test failed exactly at the unchanged greater-than-0.80 m assertion;
- graph and parameter metrics were absent only because the pair assertion
  occurred before those audits;
- the run had no infrastructure exception;
- owned master/test process groups and port 20002 were cleaned.

## 4. Diagnostic questions

G4D must answer, from retained bounded test-side evidence:

1. At what receiver-relative and ROS-header-relative time did UAV0-UAV1 reach
   minimum distance?
2. What were both positions, velocities, obstacle clearances, XY separation,
   vertical separation, relative position, relative velocity, distance rate,
   and closing speed at that instant?
3. When did the pair first enter and last leave the existing 0.80 m and 0.60 m
   bands?
4. What was the approximate time spent below those existing bands?
5. Was header-stamp skew small and observable, or could asynchronous sample
   pairing explain the reported minimum?
6. What compact path geometry was active for each UAV immediately before the
   closest encounter?
7. Did those two installed paths themselves approach or cross within the
   existing 0.80 m and 0.60 m distances?
8. Were both paths and physical vehicles individually clear of the pillar?
9. Did the graph and parameter audits still show fully isolated independent
   instances with no cross-agent command or algorithm edge?
10. Is the evidence most consistent with:
    - a sampling artifact;
    - a multi-instance isolation/wiring defect;
    - a tracking deviation from separated planned paths;
    - independently planned paths converging around the obstacle;
    - or insufficient evidence?

G4D does not decide a new control law or acceptance threshold.

## 5. Complete workspace file whitelist

Root-created and read-only during Luna execution:

    docs/Codex_PhaseOffsetSwarm_B1_G4D_Independent_Encounter_Read_Only_Diagnostic_Execution_Spec_2026-08-22.md

Luna Max may modify exactly:

    src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py

Luna Max may create exactly:

    docs/Codex_PhaseOffsetSwarm_B1_G4D_Independent_Encounter_Read_Only_Diagnostic_Self_Audit_2026-08-22.md

Everything else in the workspace is read-only. In particular:

    src/swarm_planner/bspline_traj/test/b1_independent_single.test
    src/swarm_planner/bspline_traj/test/b1_independent_three.test
    src/swarm_planner/bspline_traj/launch/b1_independent_agent.launch
    src/swarm_planner/bspline_traj/launch/b1_pillar_single_baseline.launch
    src/swarm_planner/bspline_traj/launch/b1_pillar_independent_3.launch
    src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/**
    src/swarm_planner/bspline_traj/src/**
    src/swarm_planner/bspline_traj/include/**
    src/swarm_planner/bspline_traj/launch/**
    src/swarm_planner/bspline_traj/config/**
    src/swarm_planner/bspline_traj/CMakeLists.txt
    src/swarm_planner/bspline_traj/package.xml
    src/uav_simulator/**
    src/swarm_planner/phase_offset/**
    src/swarm_planner/plan_env/**
    src/swarm_planner/path_searching/**
    src/swarm_planner/common_msgs/**

The only exception within the otherwise read-only test tree is the named
B1 Python test.

Runtime evidence may be written only under:

    /tmp/codex_b1_g4d_three_<UTC timestamp>_<unique suffix>

No workspace pycache, pyc, bag, CSV, image, generated launch, result, or log is
authorized.

## 6. No production change and no new gate

This stage is diagnostic-only. Do not add:

- a runtime feature flag;
- diagnostic mode parameter;
- G4D enable parameter;
- health/readiness gate;
- command timeout;
- watchdog;
- retry or goal resend;
- fallback;
- hold or arming state;
- collision controller;
- velocity or acceleration cap;
- lane selector;
- route selector;
- path priority;
- automatic replan trigger;
- new topic, service, publisher, subscriber, timer, or node;
- environment-variable switch;
- hidden test bypass;
- special-case pair acceptance;
- threshold change.

Do not reorder, delay, stagger, or serialize goal publications. Do not change
initial positions, goals, map, gains, planner settings, plant, SO3, or the
legacy +1.0 m height behavior.

Do not launch or connect Swarm Intent, Tube, allocator, CBF, phase-offset
active mode, matched port, visualizer, scenario publisher, RViz, or recorder.

The added structures are bounded in-memory test observations only. They do
not influence any ROS publisher, callback input, planner, controller, plant,
or acceptance threshold.

## 7. Frozen scenario and acceptance behavior

Starts:

| UAV | x | y | z | yaw |
| --- | ---: | ---: | ---: | ---: |
| 0 | -5.0 | -4.75 | 1.0 | 0.0 |
| 1 | -5.0 | -3.50 | 1.0 | 0.0 |
| 2 | -5.0 | -2.25 | 1.0 | 0.0 |

Published goals:

| UAV | x | y | z published | z effective |
| --- | ---: | ---: | ---: | ---: |
| 0 | 5.0 | -4.75 | 1.0 | 2.0 |
| 1 | 5.0 | -3.50 | 1.0 | 2.0 |
| 2 | 5.0 | -2.25 | 1.0 | 2.0 |

All three remain independent:

\[
g_i^{swarm}=0.
\]

Keep all existing arrival, liveness, clearance, graph, parameter, and pair
assertions unchanged. The diagnostic JUnit may therefore retain one failure
at the same pair-distance assertion.

## 8. Existing odometry record contract

Do not change the current odometry record tuple:

    (
        receiver_monotonic_time,
        ROS_header_stamp_seconds,
        x, y, z,
        vx, vy, vz,
        raw_pillar_XY_clearance,
    )

Do not change queue sizes, deque lengths, subscribers, callback frequencies,
or finite-data checks.

## 9. Path summary instrumentation

The current Path callback already validates every pose and computes minimum
pillar clearance. Extend it without changing those results.

### 9.1 Bounded internal path sample

Add constants:

    DIAGNOSTIC_PATH_SAMPLE_LIMIT = 200
    DIAGNOSTIC_X_STATIONS = (-4.0, -2.0, 0.0, 2.0, 4.0)

These are diagnostic sampling definitions, not acceptance thresholds or
runtime parameters.

For each non-empty finite Path:

1. Form the ordered finite point list (x,y,z).
2. Retain at most 200 points in memory:
   - if count <= 200, retain all;
   - otherwise choose deterministic evenly spaced indices including first
     and last;
   - remove duplicate indices without changing order.
3. Do not mutate the incoming ROS message.
4. Do not publish the retained points.
5. Do not write per-callback files.

### 9.2 Compact path summary fields

Add one bounded deque, maximum length 1000, containing internal summaries:

    receiver_monotonic_time
    ROS_header_stamp_seconds
    original_pose_count
    retained_sample_count
    start_xyz
    end_xyz
    min_xyz
    max_xyz
    path_length_xy_m
    path_length_3d_m
    minimum_raw_pillar_clearance_m
    station_signature
    retained_points

station_signature must contain, for each fixed x station, the retained path
point with minimum absolute x difference:

    requested_x
    sampled_xyz
    absolute_x_error_m

The internal retained_points are used only for path-pair geometry. Do not emit
all retained points in B1_METRICS.

Add path_summaries to the locked snapshot. Keep existing path_stamps,
path_clearances, path_pose_counts, and path_frames unchanged.

### 9.3 Active path selection

For an encounter receiver time t, select independently for each UAV:

1. the latest non-empty path summary whose receiver time is <= t;
2. if none exists, return None;
3. do not select a future path;
4. do not interpolate or combine path revisions.

Output only the selected compact summary without retained_points.

## 10. Pair matching algorithm

Preserve the existing directed nearest-header-stamp matching exactly:

1. Iterate every record in the lower-ID left UAV trace in order.
2. Maintain the monotone index into the higher-ID right trace.
3. Advance while the next right header stamp is at least as close to the left
   header stamp as the current right stamp.
4. Pair the left record with that nearest right record.

This preserves the current acceptance value and count semantics. Do not
replace it with interpolation, resampling, symmetric matching, wall-time
matching, Hungarian matching, or a different synchronizer.

The function may be renamed from pair_statistics to pair_diagnostics if all
callers are updated. Its returned minimum distance and below-0.60 count must
be numerically identical for identical inputs.

## 11. Pair geometry formulas

For a matched left/right sample:

\[
r=p_{right}-p_{left},
\]

\[
v_{rel}=v_{right}-v_{left},
\]

\[
d=\lVert r\rVert,
\]

\[
d_{xy}=\sqrt{r_x^2+r_y^2},
\]

\[
d_z=|r_z|,
\]

\[
\dot d=
\begin{cases}
\dfrac{r^T v_{rel}}{d}, & d>10^{-12},\\
0, & d\le 10^{-12},
\end{cases}
\]

\[
v_{closing}=\max(0,-\dot d).
\]

Header-stamp skew:

\[
\Delta t_{stamp}=t^{header}_{right}-t^{header}_{left}.
\]

Receiver-relative times:

\[
\tau_{left}=t^{recv}_{left}-t^{goal}_{left},
\]

\[
\tau_{right}=t^{recv}_{right}-t^{goal}_{right}.
\]

Do not add a TTC acceptance rule. A diagnostic TTC value may be reported only
when closing speed is strictly positive:

\[
TTC=d/v_{closing}.
\]

Otherwise report no finite TTC.

## 12. Required per-pair diagnostics

For each pair 0-1, 0-2, and 1-2, emit:

    minimum_distance_m
    samples_below_0_80m
    samples_below_0_60m
    first_below_0_80
    last_below_0_80
    first_below_0_60
    last_below_0_60
    approximate_receiver_duration_below_0_80_s
    approximate_receiver_duration_below_0_60_s
    maximum_absolute_header_stamp_skew_s
    closest_encounter
    active_path_context

The existing pair_samples_by_pair remains the below-0.60 count and the
existing total pair_samples_below_0.60m remains unchanged.

### 12.1 Band boundary records

Each non-null first/last band record contains:

    left_time_from_goal_s
    right_time_from_goal_s
    left_header_stamp_s
    right_header_stamp_s
    header_stamp_skew_s
    distance_m
    left_xyz
    right_xyz

Duration is last left receiver time minus first left receiver time, clamped
only to a minimum of zero. It is an approximate observed duration, not a new
continuous-time safety proof.

### 12.2 Closest encounter

closest_encounter contains:

    left_id
    right_id
    left_time_from_goal_s
    right_time_from_goal_s
    left_header_stamp_s
    right_header_stamp_s
    header_stamp_skew_s
    distance_m
    xy_distance_m
    vertical_distance_m
    midpoint_xyz
    left_xyz
    right_xyz
    left_velocity_xyz
    right_velocity_xyz
    relative_position_xyz
    relative_velocity_xyz
    distance_rate_mps
    closing_speed_mps
    diagnostic_ttc_s_or_null
    left_obstacle_clearance_m
    right_obstacle_clearance_m

All values must be finite except the explicitly nullable TTC.

### 12.3 Active path context

For the path summaries selected immediately before the closest left receiver
time, emit:

    left_path_compact_summary_or_null
    right_path_compact_summary_or_null
    planned_path_pair_geometry_or_null

The compact summary omits retained_points and contains all other Section 9.2
fields, with receiver time expressed relative to that UAV's goal publication.

planned_path_pair_geometry compares every retained left/right point and emits:

    sampled_minimum_3d_distance_m
    sampled_minimum_xy_distance_m
    sampled_vertical_distance_at_3d_min_m
    left_closest_xyz
    right_closest_xyz
    samples_left
    samples_right

This is a bounded sampled-path diagnostic, not an exact continuous curve
certificate.

## 13. Trajectory envelope

For each UAV after its goal publication, emit:

    sample_count
    min_xyz
    max_xyz
    path_length_xy_m
    path_length_3d_m
    minimum_raw_pillar_clearance_m
    first_xyz
    final_xyz

Use the existing received odometry records. Do not add a subscriber or
increase retention.

## 14. Audit ordering correction

The G4 test currently asserts pair criteria before graph and parameter audit,
so those metrics are absent after the known pair failure.

Move only the pair assertions:

    assert minimum pair distance > 0.80
    assert total samples below 0.60 == 0

to after the unchanged parameter and graph audits.

Requirements:

1. Compute and store pair diagnostics at the original location.
2. Run the existing parameter_audit and all its unchanged assertions.
3. Run the existing graph_audit and all its unchanged assertions.
4. Then execute the exact existing pair assertions for scenario three.
5. Do not catch, downgrade, aggregate, suppress, invert, or bypass the pair
   failure.
6. JUnit must still fail when the pair criterion fails.

This is evidence-completion ordering in a test, not a runtime gate or
acceptance relaxation.

## 15. Structured metrics

Keep exactly one original B1_METRICS emission.

Add:

    pair_diagnostics
    trajectory_envelopes

Do not emit raw full trajectories, raw full paths, environment dumps, ROS
parameters beyond the existing bounded parameter audit, or secrets.

Sanitize all new dictionaries through the existing sanitize function.

No output file is written by the test itself. Standard rostest JUnit capture
is allowed and must remain the single source containing B1_METRICS.

## 16. Static and synthetic tests

Required synthetic tests before dynamic execution:

1. Existing effective-goal and stable_arrival cases all pass.
2. Existing decode_system_state valid/invalid cases all pass.
3. Even path sampling:
   - empty path returns no sample;
   - 1 point returns that point;
   - <=200 retains all;
   - >200 retains <=200, includes first/last, is deterministic and ordered.
4. Station signature returns one nearest retained point for all five stations.
5. Path length calculation on a known polyline matches exact XY and 3D
   lengths.
6. Active path selection never selects a future path and selects the latest
   prior path.
7. Planned path-pair geometry matches known separated and crossing samples.
8. Pair diagnostics:
   - identical parallel translation preserves distance and zero closing;
   - approaching pair has negative distance rate and positive closing;
   - separating pair has positive distance rate and zero closing;
   - zero-distance case stays finite;
   - known closest sample and band first/last records are exact;
   - header-stamp skew sign and magnitude are exact;
   - old minimum/below-0.60 results match an independent copy of the previous
     pair_statistics algorithm on synthetic traces.
9. Trajectory envelope matches a known trace.
10. Metrics remain JSON-serializable with allow_nan=false.

No synthetic test may start ROS or write the workspace.

## 17. G0 personal preflight

Luna Max must personally:

1. State no sub-agent or other model will be used.
2. Read this specification, AGENTS.md, original B1 spec, R1 spec, G3R spec,
   and G4 failure JUnit completely.
3. Verify branch, HEAD, all Section 2 and Section 3 hashes.
4. Recompute the full B1E protected/B1 manifest.
5. Record status and diff summaries.
6. Verify whitelist and absent G4D self-audit.
7. Check no build/test writer.
8. List listeners read-only without contacting any ROS master.
9. Confirm no B1-R1 owned process remains.
10. Record inode, size, and mtime manifest for the frozen G4 directory.

Report and wait. Do not edit before root G1 release.

## 18. G1 instrumentation and static validation

After root release:

1. Modify only the B1 Python test.
2. Keep the patch localized to constants, pure helpers, AgentTrace bounded
   path summaries, pair diagnostics, trajectory envelopes, metrics, and audit
   ordering.
3. Show the exact diff against the G4 test hash.
4. Run system Python syntax validation with temporary pycache outside the
   workspace.
5. Import without starting ROS.
6. Run every Section 16 synthetic test.
7. Verify no existing threshold or scenario tuple changed.
8. Verify exactly one goal Publisher and one publish call remain.
9. Verify no command/controller publisher or new ROS interface.
10. Verify graph and parameter audit bodies are unchanged.
11. Verify pair assertions are textually and numerically unchanged and only
    moved.
12. Verify no new gate/parameter/mode/retry/fallback/watchdog.
13. Verify no raw trajectory/path is emitted.
14. Recompute all protected hashes.
15. Run git diff --check, whitespace, mode, and cache checks.
16. Verify frozen G4 evidence inode/size/mtime and hashes remain unchanged.

Report exact new test hash and wait for G2.

## 19. G2 one diagnostic three-agent run

Only after root release:

1. Recheck branch, HEAD, spec/test/protected hashes.
2. Recheck no build/test writer.
3. Select a new fresh loopback port excluding:
   - 12914;
   - 20000, 20001, and 20002;
   - every current listener.
4. Create a new task-owned run directory, ROS_HOME, and
   ROS_TEST_RESULTS_DIR.
5. Start a private master with an owned PID/PGID.
6. Run once, without --text:

       rostest --reuse-master bspline_race b1_independent_three.test

7. Do not rerun regardless of outcome.
8. Require standard JUnit creation.
9. Require exactly one original B1_METRICS emission.
10. Require no infrastructure exception.
11. Require all three arrivals, stream, rate, finite-data, path, obstacle,
    parameter, and graph audits to complete.
12. Require the unchanged pair assertions to determine JUnit pass/failure.
13. Retain the complete bounded diagnostic metrics.
14. Clean only exact owned master/test process groups.
15. Prove owned PIDs gone and port released.

A JUnit with one unchanged pair-distance failure is a valid diagnostic result,
not a G4D infrastructure failure.

Stop and report before analysis/self-audit.

## 20. G3 evidence analysis

After root confirms run validity, perform read-only analysis only.

For every pair:

1. Validate internal metric consistency.
2. Compare minimum distance and counts with G4.
3. Check maximum header-stamp skew.
4. Check closest-event position and relative motion.
5. Check physical obstacle clearances.
6. Check active path sampled separation.
7. Check trajectory envelopes.

For UAV0-UAV1, determine one classification:

### A. Sampling artifact

Supported only if timestamp skew or matching inconsistency explains the
reported proximity and the actual synchronized evidence does not.

### B. Isolation or wiring defect

Supported only if graph/parameter evidence shows cross-agent command,
algorithm, goal, odom, map, or controller coupling.

### C. Tracking deviation

Supported only if the two selected planned paths remain separated by more
than 0.80 m in sampled geometry while physical trajectories approach below
0.60 m, with no isolation defect.

### D. Independent path convergence

Supported if graph/parameters remain isolated, both agents individually
navigate normally and clear the obstacle, and the active sampled paths
themselves approach/cross below the existing pair thresholds near the
physical encounter.

### E. Insufficient evidence

Use when the selected path snapshots are absent, too coarse, temporally
unrepresentative, or internally inconsistent.

Do not claim causality beyond the evidence. Do not recommend a threshold
change merely because the scenario fails.

## 21. G4 final self-audit

Create exactly:

    docs/Codex_PhaseOffsetSwarm_B1_G4D_Independent_Encounter_Read_Only_Diagnostic_Self_Audit_2026-08-22.md

It must include:

- Luna Max personal/no-sub-agent statement;
- start/end timestamps;
- branch and full HEAD;
- G4D spec path and SHA;
- parent spec hashes;
- initial/final test hashes;
- complete whitelist proof;
- frozen G4 evidence hashes and preservation proof;
- exact instrumentation diff summary;
- all static/synthetic results;
- dynamic run directory, command, port, PID/PGID, JUnit, and cleanup;
- per-agent arrival, liveness, path, clearance, parameter, and graph results;
- complete per-pair diagnostic table;
- detailed UAV0-UAV1 closest encounter;
- band-entry/exit and approximate-duration evidence;
- active path compact summaries and sampled path-pair geometry;
- trajectory envelopes;
- comparison with prior G4;
- classification A/B/C/D/E with explicit supporting and contradicting
  evidence;
- no-new-gate, no-control, no-Tube, no-Swarm-Intent, no-CBF proof;
- protected hash comparison;
- git diff --check, whitespace, cache, and process cleanup;
- deviations and limitations;
- explicit STOP before any scenario correction or coordination activation.

Report draft path, line count, SHA, and wait for root final audit.

## 22. Concurrent Tube V2 and user-process protection

Treat every pre-existing process, listener, ROS master, build output, and
workspace edit as user-owned or another task's property.

Mandatory:

1. Never use port 12914 even if it appears free.
2. Never contact an existing ROS master.
3. Never signal a non-owned PID or PGID.
4. Never use pkill, killall, process-name cleanup, or broad descendant kill.
5. Never modify another /tmp/codex_* directory.
6. Never edit Tube, plan_env, phase_offset_navigation, Swarm Intent, simulator,
   SO3, planner, or controller files.
7. Never run catkin_make or a full workspace build in G4D.
8. If catkin_make, cmake --build, make, ninja, or run_tests.py is active, do
   not compete for shared outputs.
9. Use an owned setsid process-group boundary for dynamic execution.
10. Cleanup only the exact owned groups and port.

## 23. Stop conditions

Stop immediately without expanding scope if:

- branch or HEAD differs;
- this specification changes after release;
- a non-whitelist edit is required;
- protected hashes change unexpectedly;
- the frozen G4 evidence changes;
- a build becomes necessary;
- safe private ROS isolation cannot be established;
- instrumentation would affect callbacks, publications, planner/controller
  inputs, timing, thresholds, or acceptance semantics;
- the diagnostic needs a bag recorder, new node, new topic, production log,
  Tube, Swarm Intent, allocator, CBF, or control change;
- the dynamic run has an infrastructure exception;
- evidence is insufficient for classification.

Insufficient evidence is a valid final classification. Do not add another run
or broader instrumentation without a new root specification.

## 24. Required final conclusion and STOP

The final conclusion must state:

1. whether the G4 proximity failure reproduced;
2. exact closest-encounter evidence;
3. whether graph/parameter isolation passed;
4. whether installed sampled paths converged;
5. classification A, B, C, D, or E;
6. that no control, Tube, Swarm Intent, allocator, CBF, new runtime gate,
   threshold change, or scenario change occurred.

Then STOP. Any decision to revise B1 scenario acceptance or proceed to B2/B3
requires a separate root-owned execution specification and user direction.
