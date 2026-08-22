# PhaseOffsetSwarm B1 Independent Three-UAV Navigation Execution Specification

Date: 2026-08-21

Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

Authorized stage: **B1 fixed three-UAV independent point-to-point navigation on
the original `pillar.pcd`, with no swarm input and no new control-chain
integration**.

Execution owner: one `gpt-5.6-luna` agent at `max` reasoning effort, supervised
by the root agent. The execution owner must work alone and must not spawn,
delegate to, request, or wait for any sub-agent or other model.

`AUTO_ADVANCE=false`: complete B1, its tests, and its self-audit, then stop. Do
not start active Swarm Intent, allocator/projection, CBF, Tube activation,
matched-port swarm injection, B3/B4, new scenarios, or paper experiments.

## 1. Authorization and relationship to completed stages

The user authorizes the root agent to plan and supervise the next stage and
requires Luna Max to execute it independently without sub-agents.

The following work is already complete and is not to be repeated or rewritten:

1. B0 isolated one/three-agent plant simulator;
2. isolated Swarm Intent V1 pure C++ core and ROS1 shadow wrapper;
3. the current single-UAV phase-offset/Tube work owned by other concurrent
   tasks.

The next roadmap step is B1: three independent copies of the already existing
single-UAV navigation/control stack, connected only to their own B0 plant
topics. B1 does **not** connect Swarm Intent to any control input. The existing
per-UAV chain is instantiated without mathematical or controller changes:

```text
goal_i + local_map_i + odom_i
        -> formation_planning_i
        -> PositionCommand_i
        -> existing SO3Control_i
        -> SO3Command_i
        -> B0 Quadrotor_i
        -> odom_i / imu_i
```

For every `i != j`, there must be no topic edge by which `goal_j`, `odom_j`,
`local_map_j`, `PositionCommand_j`, or `SO3Command_j` is consumed by agent `i`.

This stage may start the existing per-agent controller chain because independent
navigation cannot be tested without it. It may not add, modify, or insert a new
Swarm Intent, phase-offset, Tube, allocator, CBF, governor, SO3, or fallback
component into that chain.

## 2. Mandatory reading order

Before editing, Luna Max must read completely, in this order:

1. `AGENTS.md`;
2. this execution specification;
3. `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`,
   especially B0--B4 and P2P-0;
4. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`;
5. the completed B0 execution specification and self-audit;
6. the completed Swarm Intent V1 execution specification and self-audit;
7. current `test_gvf.launch`, `gvf_manager` topic creation, `SDFMap` topic
   creation, `local_sensing.cpp`, `so3_control_nodelet.cpp`, B0 multi simulator
   launch/source/header, and `pillar.pcd` header;
8. the historical prototype launch/test/config files listed in Section 7,
   strictly as read-only negative reference material.

This specification is owned by the root agent. Luna Max must not edit it.

## 3. Preconditions and dirty-worktree protection

Expected identity:

```text
branch = main
HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
```

The worktree is intentionally dirty. Tube V2, single-agent, Swarm Intent, B0,
prototype, map, test, and documentation changes are user-owned. Do not reset,
restore, reformat, move, delete, stage, stash, or overwrite them.

At G0 record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
```

Record whether every B1 whitelist path already exists. If a whitelist path
unexpectedly contains user work, stop and report rather than replacing it.

Before each Catkin build or rostest, check for processes targeting the shared
workspace:

```text
catkin_make
cmake --build
make
ninja
run_tests.py
```

If another task is writing `build/`, `devel/`, or `build/test_results`, wait for
an explicit safe window from the root supervisor. Never signal or stop it.

Every dynamic acceptance must use a fresh private master, a unique `ROS_HOME`,
and output under `/tmp`. Never attach to or terminate an existing ROS master.

## 4. Protected read-only files and baseline hashes

The following files are read-only for B1. Recompute their SHA256 at G0 and G5.
The hashes observed by the root while writing this specification are:

```text
AGENTS.md
ce1ff4a3daca6efeef39507c14227afd806f94d55b85c200b8c8a5032b213fc5

docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md
f39e3f7c4e7752f9d73e6f69f82d11b6509d412ba7038352c556bf50101d836e

docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md
59de9cffb0b87d25fb40c4a885b9c379a9bb7b4ba830f556e4057652ee1bedef

src/swarm_planner/bspline_traj/launch/test_gvf.launch
cc6ae11d739c62d20f937acc5267fa542857f0a89e4d57dc56f5ae8823985555

src/swarm_planner/bspline_traj/src/gvf_manager.cpp
4a454d6a997e875d50114b51d920a109638f84c8fc38cd2ef803d4d401f43314

src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
d13475010de18dd3f7280b7aee31aad6b7431560cc248468608906de0c290788

src/uav_simulator/dynamic_map_generator/src/local_sensing.cpp
f98805f3a8ff44386480dabdab1de80934dd0e6fc7076ad583172a4806f01880

src/uav_simulator/dynamic_map_generator/resource/pillar.pcd
609cab63db1209be0d0390931782eb702859a0d85964a7f3ff8394966ec23414

src/uav_simulator/so3_control/src/so3_control_nodelet.cpp
b39a59b20383b90b26075a207a3df4c5e402b0955ee7df6bd60f6d61726fc661

src/uav_simulator/so3_quadrotor_simulator/launch/multi_simulator.launch
708f6922c6f541cf9d979bc6a12f7692abb69203d4bd79f066f4d33bee11df07

src/uav_simulator/so3_quadrotor_simulator/src/multi_quadrotor_simulator_so3.cpp
bee45213c89f203c8364ebd0e5da7b9e641375c846fdc2d08accf3541962c6e2

src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/multi_quadrotor_simulator.h
7b639f072e7acb88426a14018e1c8ab93106c06bdd028f10eb9371cdb9e153c3
```

The current single-agent/Tube files may change only through their separate
owner. If one of these protected dependencies changes before a B1 dynamic run,
stop at the current checkpoint and ask the root to re-audit the dependency.
Do not merge or reconcile it yourself.

Also preserve the four original single-simulator hashes recorded by B0.

## 5. B1 objective and fixed scenario

Build two launchable fixtures from the same per-agent wiring:

1. a one-agent readiness fixture;
2. the fixed three-agent B1 acceptance fixture.

Both use only the original map:

```text
src/uav_simulator/dynamic_map_generator/resource/pillar.pcd
```

No RViz, visualizer, scenario publisher, Swarm Intent node, Tube node, or CBF
node is started.

### 5.1 Single-agent readiness fixture

```text
robot_id = 0
start = (-5.0, -3.50, 1.0)
goal  = ( 5.0, -3.50, 1.0)
```

This is an execution-stage hard gate. It proves the current single-UAV
planner/local-map/controller chain can complete the frozen task when attached
to the already accepted B0 plant. It is not a runtime feature gate.

If it fails due to a protected single-agent defect, stop and report. B1 does
not authorize changes to `gvf_manager`, `test_gvf.launch`, SDFMap, local
sensing, governor, SO3, or Tube.

### 5.2 Three-agent fixture

Initial positions:

| UAV | x | y | z | yaw |
|---|---:|---:|---:|---:|---:|
| 0 | -5.0 | -4.75 | 1.0 | 0.0 |
| 1 | -5.0 | -3.50 | 1.0 | 0.0 |
| 2 | -5.0 | -2.25 | 1.0 | 0.0 |

Goals:

| UAV | x | y | z |
|---|---:|---:|---:|
| 0 | 5.0 | -4.75 | 1.0 |
| 1 | 5.0 | -3.50 | 1.0 |
| 2 | 5.0 | -2.25 | 1.0 |

The lanes and acceptance values come from the existing point-to-point
stabilization plan. Do not invent another map, start, goal, or formation.

## 6. No extra gating or hidden behavior

Do not add a `b1_enabled`, `independent_mode`, `swarm_disabled`, readiness
flag, health gate, command timeout, watchdog, fallback, arming mode, scenario
mode, retry mode, dynamic population selector, or test bypass.

The separate B1 launch files define the stage. They do not expose feature
flags. The per-agent include fixes the already existing
`phase_offset/mode=disabled` parameter because B1 is the original independent
navigation baseline. It must not expose that value as a runtime option.

Do not add a new control mode to production code. Do not modify any existing
mode or gate. Do not add a command timeout to the B0 plant.

The only execution sequencing is:

```text
static implementation -> build -> one-agent acceptance -> three-agent acceptance
```

This sequencing is not a runtime controller gate.

## 7. Historical prototype files are reference-only

Do not modify, invoke, include, register, or copy wholesale from:

```text
src/swarm_planner/bspline_traj/launch/phase_offset_agent.launch
src/swarm_planner/bspline_traj/launch/phase_offset_swarm_sim.launch
src/swarm_planner/bspline_traj/launch/phase_offset_swarm_3.launch
src/swarm_planner/bspline_traj/launch/phase_offset_pillar_left_to_right_3.launch
src/swarm_planner/bspline_traj/test/pillar_left_to_right_stability_test.py
src/uav_simulator/so3_quadrotor_simulator/config/initial_states_pillar_left_to_right_3.yaml
```

Reasons:

- the old swarm launch supports seven agents and contains a command-timeout
  interface removed by B0;
- it exposes phase-offset, swarm, Tube, CBF, ablation, and scenario gates;
- its old initial-state YAML contains a now-forbidden `name` field;
- its test hard-codes `localhost:11311`, uses NumPy, and kills processes by
  global name patterns, which could destroy user ROS sessions.

Useful topic lists and fixed scenario coordinates may be migrated only after
line-by-line review.

## 8. Authorized file whitelist

Luna Max may create or modify only:

```text
src/swarm_planner/bspline_traj/launch/b1_independent_agent.launch
src/swarm_planner/bspline_traj/launch/b1_pillar_single_baseline.launch
src/swarm_planner/bspline_traj/launch/b1_pillar_independent_3.launch
src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/initial_states_1.yaml
src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/initial_states_3.yaml
src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py
src/swarm_planner/bspline_traj/test/b1_independent_single.test
src/swarm_planner/bspline_traj/test/b1_independent_three.test
docs/Codex_PhaseOffsetSwarm_B1_Independent_Three_UAV_Navigation_Self_Audit_2026-08-21.md
```

No CMake or package manifest modification is authorized. The two `.test`
files are intentionally invoked explicitly by `rostest`; this avoids racing or
editing the heavily modified user-owned `bspline_traj/CMakeLists.txt`.

No file under these paths is authorized:

```text
src/swarm_planner/bspline_traj/src/**
src/swarm_planner/bspline_traj/include/**
src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/package.xml
src/swarm_planner/plan_env/**
src/swarm_planner/phase_offset/**
src/uav_simulator/so3_control/**
src/uav_simulator/dynamic_map_generator/**
src/uav_simulator/so3_quadrotor_simulator/**
```

The two new B1 YAML files live under `bspline_traj` and are loaded by the
read-only B0 multi-simulator launch through its existing `initial_state_file`
argument.

## 9. Per-agent launch contract

Create `b1_independent_agent.launch` as a thin fixed wiring adapter. It starts
no plant, map publisher, manager, RViz, test, or swarm node.

Inputs are limited to:

```text
robot_id
init_x
init_y
init_z
```

The top-level fixtures provide only robot IDs 0, 1, and 2. Topic names are
derived exactly as `/uav_<robot_id>/...`; arbitrary names are not accepted.

For agent `i`, the include must:

1. load one existing `SO3ControlNodelet` into `/so3_nodelet_manager`;
2. include `test_gvf.launch` exactly once inside namespace `/uav_i`;
3. pass `phase_offset_mode=disabled` explicitly;
4. set current map/planner limits to their existing values;
5. override only `gvf_gain1=0.8` and `gvf_gain2=-0.88`, preserving the
   roadmap-approved 0.4 common scale of the original `2.0/-2.2` pair;
6. keep every maximum velocity, maximum acceleration, search limit, governor
   saturation, and SO3 gain unchanged;
7. set per-agent topic parameters explicitly after the include;
8. disable the existing manual-map input/auto-save for this fixed
   `pillar.pcd` fixture;
9. add only topic remaps required to isolate existing absolute legacy names.

This launch must not accept phase-offset, swarm, Tube, CBF, velocity-limit,
map, scenario, or mode arguments.

## 10. Exact topic wiring

For each `i in {0,1,2}`:

Plant/controller/planner topics:

```text
/uav_i/sim/odom
/uav_i/sim/imu
/uav_i/sim/local_map
/uav_i/goal
/uav_i/position_cmd
/uav_i/so3_cmd
/uav_i/motors
/uav_i/corrections
```

Planner/path diagnostic topics must also be isolated:

```text
/uav_i/gvf_force
/uav_i/path_vis
/uav_i/goal_vis
/uav_i/optimization_path
/uav_i/particle0/path
/uav_i/particle0/kinopath
/uav_i/particle0/circle_reference
/uav_i/particle0sdf_map/occupancy
/uav_i/particle0sdf_map/occupancy_inflate
/uav_i/particle0sdf_map/esdf
/uav_i/particle0sdf_map/update_range
/uav_i/particle0sdf_map/unknown
/uav_i/particle0sdf_map/depth_cloud
/uav_i/particle0/gvf/occupancy
/uav_i/particle0/gvf/occupancy_inflate
/uav_i/particle0/gvf/esdf
/uav_i/particle0/gvf/update_range
/uav_i/particle0/gvf/traj_vis
/uav_i/particle0/gvf/vector_field
```

Existing manual/dynamic diagnostic inputs, although disabled in this fixture,
must be remapped per agent so no global cross-agent endpoint is created:

```text
/uav_i/clicked_point
/uav_i/manual_obstacle
/uav_i/manual_boundary
/uav_i/manual_map/occupancy
/uav_i/dynamic/obj
```

The only intentional shared algorithm input is the read-only global map:

```text
/mock_map
```

Infrastructure topics such as `/rosout` are allowed.

The graph must contain none of these un-namespaced operational endpoints:

```text
/sim/odom
/sim/imu
/sim/local_map
/move_base_simple/goal
/position_cmd
/so3_cmd
/gvf_force
/path_vis
/goal_vis
/optimization_path
/particle0/**
/particle0sdf_map/**
```

## 11. Explicit parameter wiring audit

For every agent, the test must read the parameter server and require:

```text
/uav_i/formation_planning/gvf/odom_topic       = /uav_i/sim/odom
/uav_i/formation_planning/gvf/cloud_topic      = /uav_i/sim/local_map
/uav_i/formation_planning/gvf/cmd_topic        = /uav_i/position_cmd
/uav_i/formation_planning/phase_offset/mode    = disabled
/uav_i/formation_planning/gvf/gvf_gain1        = 0.8
/uav_i/formation_planning/gvf/gvf_gain2        = -0.88
/uav_i/formation_planning/sdf_map/enable_manual_map = false
/uav_i/formation_planning/sdf_map/manual_map_auto_load = false
/uav_i/formation_planning/sdf_map/manual_map_auto_save = false
/uav_i/map_generator/odom_topic                = /uav_i/sim/odom
/uav_i/map_generator/global_map_topic          = /mock_map
/uav_i/map_generator/local_map_topic           = /uav_i/sim/local_map
/uav_i/map_generator/output_frame              = world
```

No parameter under `/uav_i/formation_planning` may reference a Swarm Intent
state topic, `g_coord`, allocator, CBF, or Tube activation.

## 12. Top-level launch contracts

### 12.1 Single baseline

`b1_pillar_single_baseline.launch` starts exactly:

- one `map_pub` using the protected original `pillar.pcd`;
- one B0 `multi_quadrotor_simulator_so3` with `num_agents=1`;
- one `so3_nodelet_manager`;
- one `b1_independent_agent` for robot 0.

It loads `initial_states_1.yaml`, fixes rates at 1000 Hz plant / 100 Hz odom,
uses `frame_id=world`, and `start_at_hover=true`.

### 12.2 Three-agent acceptance

`b1_pillar_independent_3.launch` starts exactly:

- one global `map_pub`;
- one B0 multi simulator with `num_agents=3`;
- one shared nodelet manager;
- three independent B1 agent includes for IDs 0, 1, and 2.

It starts no RViz, visualizer, goal publisher, recorder, Swarm Intent node, or
scenario node.

## 13. Mathematical and isolation invariants

B1 introduces no new guidance or control formula. It validates structural
composition of existing single-agent functions.

For agent `i`:

\[
u_i(t)=F\bigl(x_i(t),\,M_i(t),\,g_i\bigr),
\]

where `x_i` is its own odometry, `M_i` is its own local map derived from the
shared read-only `pillar.pcd`, and `g_i` is its own goal.

The required structural isolation is:

\[
\frac{\partial u_i}{\partial x_j}=0,
\quad
\frac{\partial u_i}{\partial M_j}=0,
\quad
\frac{\partial u_i}{\partial g_j}=0,
\qquad i\ne j,
\]

as demonstrated by exact ROS topic/parameter wiring, not by numerical
perturbation of another agent.

There is no swarm term in B1:

\[
g_i^{\mathrm{swarm}}=0.
\]

This means no Swarm Intent node is launched and no `g_coord`, `g_sep`,
`g_coh`, or `g_conf` value is consumed by any planner or controller.

Acceptance metrics use:

\[
e_{xy,i}(t)=\left\|p_{xy,i}(t)-p^{goal}_{xy,i}\right\|,
\]

\[
e_{z,i}(t)=\left|z_i(t)-1.0\right|,
\]

\[
v_i(t)=\left\|\dot p_i(t)\right\|,
\]

\[
d_{ij}(t)=\left\|p_i(t)-p_j(t)\right\|,
\]

and, for obstacle points `q` in `pillar.pcd` within the flight-height band,

\[
d_{obs,i}(t)=\min_q\left\|p_{xy,i}(t)-q_{xy}\right\|.
\]

## 14. Test implementation contract

Create one focused system-Python test used by the single and three-agent
`.test` files.

The test must use only Python standard library plus ROS Noetic modules. Do not
use NumPy, SciPy, pandas, Miniconda, or the historical test runner. Use an
explicit `#!/usr/bin/python3` shebang and log the interpreter path/version.

The test must never start, discover by global pattern, signal, or kill ROS
processes. Process ownership and cleanup belong to the outer explicit private
master runner.

### 14.1 PCD clearance calculation

Parse the ASCII PCD with standard library code after the `DATA ascii` line.
Retain points in the flight-height band `0.5 <= z <= 1.5`.

Build a two-dimensional 0.1 m spatial hash. For each downsampled odometry/path
point, inspect nearby cells and compute the exact Euclidean distance to the
candidate raw PCD points. Do not allocate a dense all-pairs matrix.

Use the pre-existing point-to-point test margin:

```text
minimum raw pillar-surface distance > 0.45 m
```

Do not lower it after observing a run.

### 14.2 Goal publication

For each agent:

1. wait for valid finite odom and local-map samples;
2. create a publisher on `/uav_i/goal`;
3. wait for at least one subscriber;
4. publish exactly one `geometry_msgs/PoseStamped` goal in frame `world`;
5. retain the publisher for the test duration but never publish the formal goal
   again;
6. record `goal_publish_count=1`;
7. require a non-empty, finite `/uav_i/particle0/path` after the goal.

Do not add an acknowledgment field or change production messages.

### 14.3 Runtime data

For each agent concurrently record:

- odometry;
- local map header/rate;
- `PositionCommand`;
- `SO3Command`;
- installed path messages;
- final node/topic/parameter graph.

All inspected numeric command, odom, and path fields must be finite.

### 14.4 Arrival acceptance

Each agent must, within 45 seconds after its goal publication:

```text
horizontal goal error <= 0.25 m
height error <= 0.10 m
speed <= 0.15 m/s
```

All three conditions must hold continuously for at least 3.0 seconds.

After first satisfying the stable arrival condition, observe another 5.0
seconds and require maximum displacement from the stable reference position:

```text
drift <= 0.10 m
```

The test time limit must leave enough time for the 45 s arrival horizon,
3 s stable window, and 5 s drift window.

### 14.5 Rate and liveness acceptance

For each agent:

```text
80 <= odom rate <= 130 Hz
local map continues publishing after goal
PositionCommand continues publishing during navigation
SO3Command continues publishing during navigation
all required nodes remain present until acceptance completes
```

Do not invent additional command-rate thresholds. Log observed rates and sample
counts; require positive timestamp progress and finite new samples.

### 14.6 Path and obstacle acceptance

For every non-empty path installed after the goal:

- every pose is finite and in frame `world` or an explicitly empty inherited
  frame accepted by the existing message;
- minimum raw pillar-surface clearance is greater than 0.45 m;
- the corresponding physical odometry trace also maintains greater than
  0.45 m clearance;
- at least one path is installed per agent.

### 14.7 Three-agent separation diagnostics

For the three-agent run, retain the existing point-to-point safety criteria:

```text
samples with inter-agent distance < 0.60 m = 0
minimum inter-agent distance > 0.80 m
```

These are scenario acceptance metrics only. They do not authorize a CBF,
fallback, collision controller, or Swarm Intent input. A failure is reported as
an independent-navigation scenario failure; do not add a controller to make it
pass.

### 14.8 Graph acceptance

Require:

- one map publisher;
- one multi simulator;
- one nodelet manager;
- one formation planner, one local-sensing node, and one SO3 logical instance
  per configured agent;
- exact per-agent odom/local-map/goal/PositionCommand/SO3Command wiring;
- only `/mock_map` is shared as an algorithm input;
- no fourth-agent endpoints in the three-agent run;
- no un-namespaced operational endpoints listed in Section 10;
- no node or topic from `phase_offset_swarm`, Swarm Intent diagnostics,
  allocator, CBF, Tube activation, scenario publisher, or visualizer;
- no planner subscribes to another agent's odom, local map, or goal;
- no SO3 instance subscribes to another agent's PositionCommand or odom.

Normal `/rosout` infrastructure is filtered separately and must not weaken the
operational graph audit.

### 14.9 Structured metrics

On success and failure, emit one structured `B1_METRICS` JSON object containing
at least:

```text
scenario
python executable/version
per-agent goal publish count
per-agent path count and minimum path clearance
per-agent arrival time
per-agent final xy/z error, speed, drift
per-agent odom/local-map/PositionCommand/SO3Command rates and sample counts
per-agent minimum physical obstacle clearance
three-agent minimum pair distance and count below 0.60 m
node/topic/parameter graph result
finite-data result
```

## 15. ROS private-master process isolation

Every explicit dynamic run must use:

- a unique `/tmp/codex_b1_independent_*` directory;
- a `ROS_HOME` inside that directory;
- a prechecked free TCP port;
- `ROS_MASTER_URI=http://127.0.0.1:<port>`;
- an independently owned roscore process group;
- an independently owned rostest process group;
- `rostest --reuse-master`;
- graph snapshots obtained only through that private URI;
- cleanup limited to the run-owned process groups;
- final proof that the owned PIDs are gone and the port is free.

Do not use port 11311 by default. Do not inspect through or terminate an
existing user-owned ROS master.

## 16. Build and execution gates

### G0: preflight and dependency audit

- record branch/HEAD/status/diff;
- record protected hashes;
- confirm whitelist paths are absent or reconcile only if clearly stage-owned;
- inspect historical prototypes and record their forbidden behavior;
- verify no shared build process is active;
- report to root and wait before editing.

### G1: static implementation

- create only the whitelist launch/config/test files;
- implement fixed per-agent wiring and fixed single/three fixtures;
- implement the safe system-Python test;
- run `roslaunch --nodes` or equivalent static XML validation without starting
  the dynamic scenario;
- run `xmllint` if available, Python syntax checking with a temporary pycache
  prefix, exact topic/remap searches, no-gate searches, and whitespace checks;
- report to root and wait before building.

### G2: package build

After a safe build window:

```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --pkg so3_quadrotor_simulator map_generator so3_control bspline_race
```

Do not clean. Do not repair an unrelated or protected-package failure. Record
current executable provenance for the B0 multi simulator, formation planner,
local sensing, and SO3 control.

Report to root before dynamic tests.

### G3: one-agent readiness acceptance

Run `b1_independent_single.test` on a fresh private master with
`--reuse-master`. Require every applicable Section 14 criterion. If it fails
because the existing single-agent stack cannot complete the task, stop. Do not
start the three-agent run and do not edit protected production code.

Report metrics and cleanup evidence to root; wait for G4 release.

### G4: three-agent independent navigation

Run `b1_independent_three.test` once on a new private master. Require all Section
14 criteria for all three agents, including topic isolation, arrival, obstacle
clearance, and pair-distance diagnostics.

If it fails, diagnose only launch/test wiring within the whitelist. Do not tune
maxima, change goals/map, add a swarm term, add CBF, or edit the existing
control chain.

Report metrics and cleanup evidence to root; wait for G5 release.

### G5: regression and final audit

- rerun the single and three-agent tests on fresh private masters;
- rerun both B0 registered simulator rostests;
- run `phase_offset_swarm` package tests if the shared tree is available,
  proving the isolated shadow package remains unconnected and healthy;
- attempt a full incremental workspace build with `-j2` only in a safe window;
- compare protected hashes;
- verify final graph/static no-swarm/no-CBF/no-Tube/no-extra-gate boundaries;
- run `git diff --check` and explicit trailing-whitespace scans;
- compare final status with G0 and attribute only whitelist additions to B1;
- write the final self-audit and stop.

An unrelated full-workspace build/test failure is recorded, not repaired.

## 17. Required static searches

The B1 production launch/config files must contain no runtime argument or node
for:

```text
enable_swarm
swarm_enabled
g_coord
g_sep
g_coh
g_conf
AgentState
phase_offset_swarm
allocator
CBF
cbf
Tube
tube
command_timeout
failsafe
watchdog
ablation
scenario
num_agents=7
uav_3
```

The fixed literal `phase_offset_mode=disabled` is required and is not exposed
as an argument. Comments should avoid forbidden tokens when possible so the
audit remains unambiguous.

The test may mention forbidden topic names only in explicit negative graph
assertions. It must not publish or subscribe to them.

Verify additionally:

- exactly one and three plant configs are used;
- no arbitrary population loop exists in launch files;
- no global kill/process-name matching exists in the test;
- no `localhost:11311` is hard-coded;
- no NumPy or non-system Python dependency exists;
- no command or state is shared across agents;
- no maximum velocity/acceleration/saturation parameter is overridden;
- no protected or historical prototype file is included.

## 18. Self-audit deliverable

Create:

```text
docs/Codex_PhaseOffsetSwarm_B1_Independent_Three_UAV_Navigation_Self_Audit_2026-08-21.md
```

It must contain:

- exact start/end timestamps, branch, and full HEAD;
- initial/final status and tracked diff summaries;
- exact B1 files created;
- protected hashes before/after and any concurrent dependency change;
- explanation of rejected prototype timeout/gate/process-kill behavior;
- static launch/remap/parameter audit;
- package build command/result and executable provenance;
- one-agent metrics and private master/PID/port cleanup evidence;
- three-agent metrics and private master/PID/port cleanup evidence;
- final rerun metrics;
- B0 and Swarm Intent regression results;
- full workspace build result;
- no-extra-gating, no-swarm-control, whitelist, cache, and formatting audits;
- deviations/blockers;
- explicit stop before Swarm Intent execution, allocator, CBF, Tube activation,
  matched-port swarm injection, B3/B4, scenario expansion, and experiments.

## 19. Stop conditions

Stop without expanding scope if:

- branch or HEAD differs;
- a whitelist path contains conflicting user work;
- a protected dependency changes and root has not re-audited it;
- a required edit falls outside the whitelist;
- the one-agent baseline cannot pass without production changes;
- the three-agent run requires changing `gvf_manager`, SDFMap, local sensing,
  governor, SO3, B0 plant, or map;
- passing appears to require a new runtime gate, timeout, fallback, mode,
  Swarm Intent input, allocator, CBF, Tube, or controller tuning outside the
  frozen `0.8/-0.88` gains;
- ROS runtime is unavailable;
- another task repeatedly occupies the shared build tree;
- an unrelated historical defect blocks a workspace-wide regression.

Do not commit, stage, push, create a branch/tag/stash, reset, restore, clean,
rebase, or delete user files.

## 20. Root-supervision checkpoints

Luna Max must report and wait at:

1. G0 preflight/prototype/dependency findings;
2. G1 static launch/config/test implementation;
3. G2 build/provenance;
4. G3 one-agent readiness metrics and cleanup;
5. G4 three-agent metrics and cleanup;
6. G5 final self-audit and stop.

At no checkpoint may Luna Max create, consult, or delegate to a sub-agent.
