# PhaseOffsetSwarm B1-R1 Legacy Goal-Height Semantic Alignment and Independent Three-UAV Execution Specification

Date: 2026-08-21

Repository:

    /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws

Authorized stage: repair only the B1 acceptance fixture's interpretation of
the protected legacy goal-height contract, then complete B1 single-agent
readiness and three-agent independent-navigation acceptance.

Execution owner: the existing Luna Max agent, supervised by the root agent.
Luna Max must execute personally. It must not spawn, delegate to, consult,
request, message, or wait for any sub-agent, nested agent, other model, or
parallel worker. If Luna Max cannot continue personally, it must stop and
report to the root agent.

AUTO_ADVANCE=false. Stop after the B1-R1 tests, regressions, cleanup, and
self-audit. Do not begin live Swarm Intent integration, allocator work, CBF,
Tube activation, matched-port swarm injection, B3/B4, scenario expansion, or
paper experiments.

## 1. Authority and precedence

This specification is a narrow root-owned revision of:

    docs/Codex_PhaseOffsetSwarm_B1_Independent_Three_UAV_Navigation_Execution_Spec_2026-08-21.md

It is informed by the completed diagnostic:

    docs/Codex_PhaseOffsetSwarm_B1E_Original_To_B1_Single_Equivalence_Diagnostic_Self_Audit_2026-08-21.md

The original B1 specification remains authoritative except where this
revision explicitly changes the interpretation of the accepted terminal
height. This revision does not authorize any production behavior change.

The user has confirmed that the original navigation's protected +1.0 m
goal-height behavior is expected and must remain unchanged. The strict B1
fixture must therefore measure arrival relative to the protected effective
goal rather than incorrectly treating the published marker height as the
physical terminal height.

This document does not authorize changing goals, thresholds, gains, map,
planner logic, controller logic, or runtime modes. It authorizes a test-only
semantic correction and the already-specified B1 runtime acceptance.

## 2. Frozen identity and preconditions

Before every workspace edit or dynamic run, verify:

    branch = main
    HEAD   = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43

Frozen input hashes at root release:

    992438a9efb573dce1fdeacfc46f9096e6f63c3a916b3394d38b9724f9d348a3  docs/Codex_PhaseOffsetSwarm_B1_Independent_Three_UAV_Navigation_Execution_Spec_2026-08-21.md
    ec09ff5fd58625b4f0c11dd6e426a6c9b10c91cb67c2ba6c1fa751c23ce20bc5  docs/Codex_PhaseOffsetSwarm_B1E_Original_To_B1_Single_Equivalence_Diagnostic_Self_Audit_2026-08-21.md
    931bc55b6fe5673e3d8b6129494846780008df5c91fb55c933ceba18f7d5a044  src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py
    2f26d62d0ec9ef3734116cb6a4067efa389b4aa7af8559d5a734579a9fa0ed3e  src/swarm_planner/bspline_traj/test/b1_independent_single.test
    8d5f53de60df7d998c81362b3b6afe00aab69d860981e7e799338097580eae00  src/swarm_planner/bspline_traj/test/b1_independent_three.test

The B1E audit records the protected implementation and B1 file hashes. Luna
Max must verify those recorded hashes before editing. A mismatch in the one
authorized test file is expected only after its own authorized patch. Any
other mismatch must be reported to the root agent before continuing.

The worktree is intentionally dirty and contains user-owned and concurrent
Tube V2, PhaseOffset, simulator, Swarm Intent, and historical assets. Preserve
all of them. Do not reset, restore, clean, stash, stage, commit, branch, tag,
rebase, push, move, rename, or delete user files.

## 3. Completed diagnostic fact and exact interpretation

The protected legacy implementation in gvf_manager::goalCallback constructs
the active goal height as:

\[
z_i^{effective}=z_i^{published}+1.0\ \mathrm{m}.
\]

For the frozen B1 published goals:

\[
z_i^{published}=1.0\ \mathrm{m},
\]

therefore:

\[
z_i^{effective}=2.0\ \mathrm{m}.
\]

B1E E0R through E8 all navigated normally and settled near 2.0 m. Their old
strict result was false only because the test evaluated height error against
1.0 m. Manual-map state, namespaces, SO3 loading topology, frame metadata,
hover initialization, plant implementation, external-yaw selection, and GVF
gain scaling did not change this result.

This revision accepts that fact. Do not diagnose, compensate for, parameterize,
or remove the +1.0 m legacy behavior.

## 4. Stage objective

Complete exactly these outcomes:

1. Make the B1 test distinguish the published marker goal from the protected
   effective terminal goal.
2. Keep publishing the exact original B1 goals at z=1.0 m.
3. Evaluate stable physical arrival at z=2.0 m.
4. Re-run the one-agent B1 readiness fixture.
5. If the valid one-agent run passes, run the fixed three-agent independent
   navigation fixture with g_swarm equal to zero.
6. Verify graph isolation, liveness, obstacle clearance, pair-distance
   diagnostics, and exact absence of swarm/control integration.
7. Run the specified regressions, write one self-audit, clean only owned
   runtime processes, and stop.

The one-agent-before-three-agent order is an execution checkpoint only. It
must not create or modify a runtime gate, ROS parameter, production mode,
watchdog, fallback, or controller state.

## 5. Complete workspace file whitelist

Root-created and read-only during Luna execution:

    docs/Codex_PhaseOffsetSwarm_B1_R1_Legacy_Goal_Height_Semantic_Alignment_And_Independent_Three_UAV_Execution_Spec_2026-08-21.md

Luna Max may modify exactly one existing workspace file:

    src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py

Luna Max may create exactly one workspace file:

    docs/Codex_PhaseOffsetSwarm_B1_R1_Legacy_Goal_Height_Semantic_Alignment_And_Independent_Three_UAV_Self_Audit_2026-08-21.md

All other workspace files are read-only, including:

    src/swarm_planner/bspline_traj/test/b1_independent_single.test
    src/swarm_planner/bspline_traj/test/b1_independent_three.test
    src/swarm_planner/bspline_traj/launch/b1_independent_agent.launch
    src/swarm_planner/bspline_traj/launch/b1_pillar_single_baseline.launch
    src/swarm_planner/bspline_traj/launch/b1_pillar_independent_3.launch
    src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/initial_states_1.yaml
    src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/initial_states_3.yaml
    src/swarm_planner/bspline_traj/src/gvf_manager.cpp
    src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
    src/swarm_planner/bspline_traj/launch/test_gvf.launch
    src/uav_simulator/dynamic_map_generator/resource/pillar.pcd
    src/uav_simulator/dynamic_map_generator/resource/phase_offset_*.pcd
    src/uav_simulator/so3_quadrotor_simulator/**
    src/uav_simulator/so3_control/**
    src/swarm_planner/phase_offset/**
    src/swarm_planner/plan_env/**
    src/swarm_planner/path_searching/**
    src/swarm_planner/common_msgs/**
    src/swarm_planner/bspline_traj/include/**
    src/swarm_planner/bspline_traj/src/**
    src/swarm_planner/bspline_traj/launch/**
    src/swarm_planner/bspline_traj/config/**
    src/swarm_planner/bspline_traj/CMakeLists.txt
    src/swarm_planner/bspline_traj/package.xml

The explicit single-file exception under the otherwise read-only test tree is:

    src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py

Runtime evidence may be written only below a fresh task-owned directory:

    /tmp/codex_b1_r1_<stage>_<UTC timestamp>_<root-owned unique suffix>

Do not write new JUnit results into another task's retained evidence directory.
Normal ROS/catkin runtime-generated files below the task-owned /tmp directory
are allowed. No source-tree cache, pyc, bag, log, generated map, or test output
is allowed.

## 6. Exact authorized test patch

The patch must be small and semantic. Do not rewrite or reformat the test.

### 6.1 Add one frozen constant

Add:

    LEGACY_GOAL_Z_OFFSET_M = 1.0

This is a test description of the protected implementation contract. It is
not a ROS parameter, launch argument, feature flag, calibration value, or
runtime option.

### 6.2 Distinguish published and effective goals

Rename the scenario dictionary field from goals to published_goals, retaining
all numerical tuples unchanged:

    single published goal: (5.0, -3.50, 1.0)
    UAV 0 published goal:  (5.0, -4.75, 1.0)
    UAV 1 published goal:  (5.0, -3.50, 1.0)
    UAV 2 published goal:  (5.0, -2.25, 1.0)

Add one pure helper:

    def effective_goal(published_goal):
        return (
            published_goal[0],
            published_goal[1],
            published_goal[2] + LEGACY_GOAL_Z_OFFSET_M,
        )

The helper must have no ROS access, parameter lookup, mutable state, fallback,
environment dependency, or observation of planner/controller output.

Do not infer the accepted height from the final PositionCommand or odometry.
That would make the test self-fulfilling. The expected +1.0 m relationship is
frozen explicitly from the protected source and completed B1E evidence.

### 6.3 Publication contract

For each agent, construct and publish the PoseStamped from published_goals.
The test must still:

- publish exactly once per agent;
- publish frame_id world;
- publish x and y unchanged;
- publish z=1.0 m;
- retain the publisher without republishing;
- record goal_publish_count=1.

No goal acknowledgment, resend, retry, latch workaround, timeout controller,
or hidden publication is authorized.

### 6.4 Arrival contract

Pass effective_goal(published_goal) to stable_arrival. Do not change the
stable_arrival algorithm or thresholds.

For agent i, define:

\[
\bar g_i=(x_i^g,y_i^g,z_i^{published}),
\]

\[
g_i^{effective}=(x_i^g,y_i^g,z_i^{published}+1.0),
\]

\[
e_{xy,i}(t)=
\sqrt{(x_i(t)-x_i^g)^2+(y_i(t)-y_i^g)^2},
\]

\[
e_{z,i}(t)=
\left|z_i(t)-z_i^{effective}\right|,
\]

\[
v_i(t)=
\sqrt{\dot x_i(t)^2+\dot y_i(t)^2+\dot z_i(t)^2}.
\]

Acceptance remains:

    e_xy <= 0.25 m
    e_z  <= 0.10 m
    speed <= 0.15 m/s
    all three continuously for >= 3.0 s
    first qualifying start <= 45.0 s after publication
    post-stable drift <= 0.10 m for another 5.0 s

Do not relax, widen, shorten, scale, or dynamically select any threshold.

### 6.5 Structured metrics

Add these deterministic fields to B1_METRICS:

    legacy_goal_z_offset_m
    published_goals
    effective_goals

Use string robot IDs consistently with the existing JSON maps. Each goal must
be recorded as a finite three-element list. Arrival final_z_error_m must be
relative to effective_goals.

Keep exactly one B1_METRICS JSON line per run. Do not copy the metrics line
into JUnit system-out manually.

Do not change unrelated duplicate keys, formatting, graph logic, parameter
logic, PCD logic, rate logic, pair logic, exception handling, or process
behavior in this patch.

## 7. Frozen scenario and behavior

### 7.1 Single agent

    robot_id = 0
    start = (-5.0, -3.50, 1.0)
    published goal = (5.0, -3.50, 1.0)
    effective goal = (5.0, -3.50, 2.0)

### 7.2 Three independent agents

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

Every agent remains an independent copy of the original stack:

\[
g_i^{swarm}=0.
\]

No agent consumes another agent's odometry, map, goal, command, path, Tube,
swarm intent, allocator result, or CBF result.

## 8. No new gating or hidden behavior

This revision authorizes no new runtime gate.

Do not add any:

- enable flag;
- readiness parameter;
- health state;
- acceptance mode;
- legacy-height mode;
- automatic z detector;
- planner-output-derived target;
- goal retry or resend;
- command timeout;
- watchdog;
- fallback;
- arming state;
- hold state;
- scenario selector beyond the existing single/three test parameter;
- dynamic population selector;
- hidden test bypass;
- environment-variable switch;
- ROS service or topic used to release navigation;
- special-case controller behavior.

Do not add a parameter such as legacy_goal_offset, expected_goal_z,
accept_z_plus_one, or b1_r1_enabled. The constant belongs only in the test
source and is unconditional for the frozen B1 fixture.

The existing finite-input checks, stable-arrival predicates, graph assertions,
and ordered single/three execution are tests, not new production gates. Do not
modify production behavior to satisfy them.

## 9. Production and research boundaries

Forbidden in B1-R1:

- any edit to gvf_manager or goalCallback;
- changing the published goal to z=0;
- changing the effective navigation goal to z=1;
- changing GVF gains 0.8/-0.88;
- changing maximum velocity, acceleration, saturation, SO3 gains, yaw mode,
  planner limits, map, initial positions, lanes, or obstacle threshold;
- launching or connecting Swarm Intent;
- publishing or consuming g_coord, g_sep, g_coh, or g_conf;
- activating phase-offset, allocator, CBF, Tube, matched port, or recovery;
- adding a safety controller because a pair-distance diagnostic fails;
- changing B0 plant integration;
- modifying any controller or command message;
- RViz, visualizer, scenario publisher, bag recording, or paper statistics.

The existing isolated Swarm Intent packages remain read-only and unconnected.
The Tube V2 work remains entirely outside this task.

## 10. Concurrent Tube V2 and user-process protection

Treat all pre-existing ROS masters, listeners, processes, build outputs, and
workspace changes as user-owned or another task's property.

Specific protections:

1. Never use TCP port 12914, even if it appears free at one observation.
2. Never use any port already listening at run preflight.
3. Never query an existing ROS master with rosnode, rostopic, rosparam,
   rosservice, rosgraph, or XML-RPC.
4. Never signal, terminate, pause, attach a debugger to, or change the
   environment of an existing process.
5. Never use pkill, killall, process-name matching, global PID scans for
   cleanup, or recursive child discovery outside the owned process group.
6. Never remove or overwrite another /tmp/codex_* directory.
7. Never modify Tube, Tube evidence, phase_offset_navigation, plan_env,
   current-anchor, path-tube, or continuity files.
8. Before a build-like command, check read-only for catkin_make, cmake
   --build, make, ninja, and run_tests.py. B1-R1 normally requires no build.
9. If a concurrent writer appears, do not compete for the build tree. Stop
   that checkpoint and report to root.
10. Cleanup only the exact process group created for the current private ROS
    master run.

Each dynamic run must use:

- a new task-owned /tmp run directory;
- a new ROS_HOME below it;
- a new ROS_TEST_RESULTS_DIR below it;
- a fresh loopback port other than 12914;
- an explicitly captured master PID and PGID;
- setsid or an equivalent owned process-group boundary;
- rostest --reuse-master;
- TERM to the owned PGID after the test;
- a bounded wait;
- KILL only to the same owned PGID if TERM fails;
- final proof that the owned PID is gone and the owned port no longer listens.

## 11. G0: Luna personal preflight

Luna Max must personally:

1. State that it will not create or use any sub-agent.
2. Read this specification completely.
3. Read the original B1 specification completely.
4. Read the B1E final self-audit completely.
5. Read AGENTS.md completely.
6. Record branch, HEAD, git status --short, and git diff --stat.
7. Verify every frozen hash in Section 2.
8. Verify the complete whitelist and confirm only the test and final self-audit
   are writable.
9. Search for build writers read-only.
10. List listening ports read-only without contacting any ROS master.
11. Record current hashes for the original B1 implementation, B0 plant,
    protected goalCallback, Swarm Intent package, and Tube-related protected
    files cited by the B1E and concurrent Tube audits.
12. Confirm no B1E runtime remains.

Report G0 to root and wait. Do not edit before root release.

## 12. G1: minimal semantic patch and static validation

After root release, modify only:

    src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py

Required validation:

1. Show the exact diff.
2. Confirm numerical published goals remain unchanged.
3. Confirm only the effective arrival target adds +1.0 m.
4. Run Python syntax validation using a temporary pycache outside the
   workspace.
5. Import the test module without starting ROS nodes and assert:

       effective_goal((5.0, -3.5, 1.0)) == (5.0, -3.5, 2.0)

6. Run synthetic stable_arrival cases:
   - a stable trace at z=2 passes for effective goal z=2;
   - the same trace does not pass for published goal z=1;
   - a trace violating XY fails;
   - a trace violating speed fails;
   - a trace shorter than 3 s fails;
   - a trace drifting more than 0.10 m fails.
7. Verify the test still has exactly one Publisher construction for the goal
   and exactly one publish call.
8. Verify there is no PositionCommand, SO3Command, motors, corrections, Tube,
   Swarm Intent, allocator, CBF, or controller publisher.
9. Verify no new parameter lookup, environment switch, fallback, retry,
   timeout, watchdog, mode, or gate was added.
10. Run git diff --check and a trailing-whitespace scan on the authorized
    test.
11. Verify no workspace pycache or pyc was created.

Report G1 diff, hash, and validation results to root and wait.

## 13. G2: reuse executable provenance without rebuilding

B1-R1 changes only a Python acceptance test. Do not run catkin_make at this
checkpoint.

Verify and retain:

    /tmp/codex_b1_g2_catkin_make_20260821.log

The log must contain:

    BUILD_EXIT_CODE=0

and successful existing targets for the B0 multi simulator, map publisher and
local sensing, SO3 control, and bspline_race planner.

Verify the devel test executable resolves to the authorized source test or an
up-to-date wrapper that invokes it. If a rebuild appears necessary only to
refresh a wrapper, stop and ask root. Do not start a shared build
unilaterally.

Report G2 provenance to root and wait for single-run release.

## 14. G3: corrected single-agent readiness run

Run exactly one valid corrected single-agent acceptance on a new private ROS
master:

    rostest --reuse-master --text bspline_race b1_independent_single.test

The outer runner owns roscore and cleanup. The test must not manage processes.

Validity requirements:

- correct private master and isolated ROS_HOME;
- one test case and a JUnit result;
- complete run, not killed by external timeout;
- exactly one B1_METRICS line;
- scenario=single;
- one goal publication;
- published goal exactly [5.0, -3.5, 1.0];
- effective goal exactly [5.0, -3.5, 2.0];
- legacy_goal_z_offset_m exactly 1.0;
- finite odom, local-map, PositionCommand, SO3Command, and path data;
- positive timestamp progress for all four streams;
- expected graph and parameter audit;
- owned master/PID/PGID/port cleanup complete.

Acceptance requirements:

- stable arrival passes against the effective goal;
- final XY error <= 0.25 m;
- final Z error relative to 2.0 m <= 0.10 m;
- final speed <= 0.15 m/s;
- 3.0 s stable window;
- 5.0 s drift <= 0.10 m;
- odom rate 80--130 Hz;
- non-empty finite path;
- path and physical raw pillar clearance > 0.45 m;
- no forbidden node, topic, parameter, cross-agent edge, or control input.

If the run is infrastructure-invalid, preserve evidence and report; root may
authorize one corrected rerun. If it is valid but fails an acceptance item,
do not tune, patch production, add a gate, or start the three-agent run.
Report the exact first failure and stop at G3.

If it passes, report metrics and cleanup evidence to root and wait for G4.

## 15. G4: three-agent independent-navigation run

Only after explicit root release, run exactly one valid three-agent acceptance
on a different new private ROS master:

    rostest --reuse-master --text bspline_race b1_independent_three.test

Require for each UAV:

- exact start and published/effective goal from Section 7;
- exactly one goal publication;
- stable arrival against z=2.0 m using unchanged thresholds;
- 80--130 Hz odometry;
- finite, live odom, map, PositionCommand, SO3Command, and path streams;
- non-empty safe path;
- physical obstacle clearance > 0.45 m;
- independent planner, map, controller, and plant topics;
- no cross-agent algorithm edge.

Three-agent diagnostics remain:

    minimum pair distance > 0.80 m
    samples below 0.60 m = 0

These are acceptance diagnostics only. A failure does not authorize Swarm
Intent, CBF, Tube, fallback, controller tuning, lane changes, goal changes, or
new gating.

Require no Swarm Intent node and no g_coord, g_sep, g_coh, or g_conf consumer.
Require no phase-offset active mode, allocator, CBF, Tube, visualizer, scenario
publisher, RViz, or fourth UAV.

If the run is infrastructure-invalid, preserve evidence and report; root may
authorize one corrected rerun. If it is valid but fails, diagnose only
launch/test evidence read-only and report. Do not edit outside the single test
file, and do not change semantics beyond Section 6.

Report metrics and cleanup evidence to root and wait for G5.

## 16. G5: regressions and final reruns

Only after G4 passes and root releases G5:

1. Re-run the corrected single-agent test once on a fresh private master.
2. Re-run the corrected three-agent test once on another fresh private master.
3. Run the two registered B0 simulator rostests on private masters or through
   their safe registered test target, without contacting existing masters.
4. Run phase_offset_swarm package tests only if no build/test writer is active
   and the existing package test target can be invoked without modifying
   source. This proves the isolated shadow package remains healthy and
   unconnected.
5. Do not run a full workspace build in B1-R1. The test-only change does not
   justify competing with concurrent Tube work.
6. Recompute all B1E protected hashes and all current B1 implementation hashes.
   Only the authorized test hash and the new self-audit may differ from G0.
7. Run graph/static searches for no swarm, no CBF, no Tube, no extra gates, no
   cross-agent command flow, and no fourth UAV.
8. Run git diff --check and targeted trailing-whitespace checks.
9. Verify no owned master, port, process group, pycache, pyc, or build writer
   remains.

Any regression failure must be recorded and attributed. Do not repair an
unrelated historical or concurrent defect.

## 17. Root-supervision checkpoints

Luna Max must report and wait after:

1. G0 personal preflight;
2. G1 exact patch and static/synthetic validation;
3. G2 existing executable provenance;
4. G3 corrected single-agent metrics and cleanup;
5. G4 three-agent metrics and cleanup;
6. G5 regressions and cleanup;
7. G6 draft self-audit hash and final status.

At every checkpoint Luna Max must include:

- confirmation that no sub-agent was used;
- files written since the previous checkpoint;
- commands run;
- result and first failure if any;
- private master port/PID/PGID ownership and cleanup status;
- whether any concurrent build or Tube process changed;
- explicit request for the next root release.

Luna Max may not self-release the next checkpoint.

## 18. G6 final self-audit

Create exactly:

    docs/Codex_PhaseOffsetSwarm_B1_R1_Legacy_Goal_Height_Semantic_Alignment_And_Independent_Three_UAV_Self_Audit_2026-08-21.md

It must include:

- executor identity and no-sub-agent statement;
- start/end timestamps;
- branch and full HEAD;
- root specification path and SHA256;
- original B1 specification and B1E audit hashes;
- initial/final status and diff summaries;
- exact authorized test diff and final hash;
- mathematical published/effective goal contract;
- proof that published z stayed 1.0 and accepted effective z became 2.0;
- proof thresholds, gains, map, lanes, controller, and production code stayed
  unchanged;
- G1 syntax and synthetic test results;
- reused G2 build provenance;
- first and final single-agent metrics;
- first and final three-agent metrics;
- B0 regression results;
- Swarm Intent regression result or precise safe reason it was not run;
- graph, parameter, topic, isolation, rate, clearance, pair-distance, and
  finite-data evidence;
- exact private master ports/PIDs/PGIDs and cleanup;
- protected hash comparison;
- Tube V2 and user-process protection evidence;
- no-extra-gate audit;
- no control-chain, Tube, allocator, CBF, or Swarm Intent connection;
- deviations and any invalid-run history;
- git diff --check, whitespace, cache, process, and final status results;
- explicit STOP before B2 live shadow integration and every active-control
  stage.

The self-audit must not claim a pass that is absent from retained evidence.

## 19. Stop conditions

Stop immediately and report without expanding scope if:

- branch or HEAD differs;
- the root specification hash changes after release;
- a non-whitelist edit is required;
- a protected file changes unexpectedly;
- a shared build/test writer occupies required resources;
- safe private ROS isolation cannot be established;
- single-agent acceptance cannot pass with the exact test-only semantic
  correction;
- three-agent acceptance requires changing goals, lanes, map, gains, planner,
  controller, plant, or production code;
- passing appears to require a new gate, mode, timeout, watchdog, fallback,
  resend, retry, or hidden bypass;
- passing appears to require Swarm Intent, allocator, CBF, Tube, or matched-port
  control input;
- an owned ROS process cannot be distinguished safely from a user process;
- ROS runtime is unavailable;
- an unrelated historical defect blocks only a broad regression.

Do not work around a stop condition. Preserve evidence and wait for root.

## 20. Required final conclusion

If all required valid runs pass, the only authorized conclusion is:

    The original protected z+1.0 goal-height behavior is preserved.
    B1 acceptance now measures the protected effective goal rather than the
    published marker height. One and three independent UAV instances navigate
    successfully with g_swarm=0, without Tube, Swarm Intent, allocator, CBF,
    controller integration, or new runtime gates.

Then STOP. A later B2 live three-UAV Swarm Intent shadow-integration stage
requires a separate root-owned execution specification.
