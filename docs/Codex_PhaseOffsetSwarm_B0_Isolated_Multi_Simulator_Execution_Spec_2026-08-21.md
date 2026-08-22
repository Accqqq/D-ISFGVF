# PhaseOffsetSwarm B0 Isolated Multi-Simulator Execution Specification

Date: 2026-08-21

Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

Authorized stage: **B0 isolated multi-instance plant simulation only**.

Execution owner: one `gpt-5.6-luna` agent at `max` reasoning effort, supervised
by the root agent. The execution owner must work alone and must not spawn,
delegate to, request, or wait for any sub-agent or other model.

`AUTO_ADVANCE=false`: complete this B0 implementation, tests, self-audit, and
stop. Do not start B1 independent navigation, B2 three-agent communication
shadow, allocator/projection, CBF, Tube integration, swarm activation, or any
paper experiment.

## 1. Authorization and precedence

The user explicitly authorizes B0 now and asks the root agent to write the
detailed plan and supervise Luna Max. This document is the dedicated stage
authorization required by `AGENTS.md`.

This authorization overrides only the stale roadmap status text that names an
older A-stage as current. Every Git-safety, module-boundary, baseline,
whitelist, testing, and stop rule in `AGENTS.md` remains active.

Before editing, the execution owner must completely read, in order:

1. `AGENTS.md`;
2. this execution specification;
3. `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`,
   especially B0--B4 and the stop rules;
4. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`;
5. the current original single-UAV simulator source, launch, CMake, package,
   dynamics source, and public `Quadrotor` header;
6. the historical untracked multi-simulator header/source/launch/config/tests,
   strictly as reference-only material before line-by-line migration.

This specification is owned by the root agent. Luna Max must not edit it.

## 2. Expected preflight and dirty-worktree protection

Expected baseline:

```text
branch = main
HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
```

The worktree is intentionally highly dirty. Tracked/untracked Tube,
single-agent, Swarm Intent, simulator prototype, scenario, map, test, and
documentation files are user-owned assets.

Before any edit, record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff -- src/uav_simulator/so3_quadrotor_simulator
```

Also record SHA256 for the protected original single-agent files listed in
section 5. If branch or HEAD differs, stop and report. If an authorized path
changes concurrently and cannot be reconciled without overwriting user work,
stop and report.

The existing `build/` and `devel/` trees contain stale historical artifacts,
including an old `multi_quadrotor_simulator_so3` binary even though the current
source CMake does not declare that target. No existing binary is acceptance
evidence. The new target must be reconfigured and rebuilt from the current
authorized source, and its timestamp/build provenance must be recorded.

Before every Catkin build or registered test run, check for another process
targeting this workspace:

```text
catkin_make
cmake --build
make
ninja
run_tests.py
```

If one is active, do not race writes into the shared `build/`, `devel/`, or
`build/test_results` trees. Report to the root supervisor and wait for an
explicit build window. Never signal, stop, or clean another task's process.

## 3. B0 objective

Add one independent executable and launch that maintain either one or three
fully independent instances of the existing quadrotor plant in one process:

```text
SO3Command_i + disturbance_i
        -> existing Quadrotor plant math_i
        -> Odometry_i + Imu_i
```

The multi simulator is a plant container only. It must not calculate or own:

- planning, goals, maps, local sensing, or obstacle state;
- ISF-GVF or phase-offset state;
- Tube, Runtime, Pair, port, allocator, QP, or CBF;
- neighbor cache, AgentState communication, Swarm Intent, or cohesion;
- SO3 position control, governor logic, or final position commands;
- traffic priority, emergency policy, replanning, or behavior modes.

The required B0 result is:

1. the original single-UAV executable and default launch remain unchanged;
2. `num_agents=1` multi-plant behavior matches the original simulator under
   the same initial state, rates, and command stream;
3. `num_agents=3` exposes three isolated plant topic sets and independent
   states;
4. a command or disturbance sent to one agent does not affect the other two;
5. no B1/B2/B3 functionality is started or connected.

## 4. No extra gating or hidden behavior

Do not add runtime gates, feature flags, watchdog modes, command-timeout
failsafes, health states, arming states, dropout recovery, automatic agent
removal, dynamic population changes, fallback layouts, or test-only bypasses.

In particular, the historical prototype's following behavior is forbidden and
must be removed from the production B0 implementation:

```text
command_timeout -> hover failsafe
command_received / failsafe_active state
automatic arbitrary-N circular initial layout
num_agents=7 support or tests
```

The only authorized validation/conditional behavior is:

- startup rejection of malformed configuration;
- the original simulator's existing `start_at_hover` parameter, with the same
  meaning and explicit value in equivalence tests;
- the original per-motor non-finite RPM fallback to the previous RPM, migrated
  only to preserve existing plant-loop numerical behavior;
- normal ROS shutdown checks and publish-rate scheduling.

Do not add a `swarm_enabled`, `multi_enabled`, `mode`, `failsafe`, or similar
parameter. The separate executable/launch is itself the isolation boundary.

## 5. Protected original single-agent baseline

The following files are read-only and their SHA256 must be identical before and
after B0:

```text
src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp
src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch
src/uav_simulator/so3_quadrotor_simulator/src/dynamics/Quadrotor.cpp
src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/Quadrotor.h
```

Baseline hashes observed by the root at specification time:

```text
quadrotor_simulator_so3.cpp  9a2452e2c329f3ef8f49faadb93e05e7f7f888d81dc977b040457009bfc65c92
simulator.launch             d6339f7d6b0cde4beffa3b037ef28a2415ccd5fa1419f46a6b97df2c9d3134c6
Quadrotor.cpp                0d5d0af994674ffefa87886a73bc580fc36606b34389346c9799b60d54d3281f
Quadrotor.h                  86652b16839d503bc0fc7a21d8420926af5b7488383112f6acb89d76d02fb222
```

Do not refactor the original single executable to share a new header in B0.
Dynamic equivalence must be proved by tests, not by rewriting the trusted
baseline.

The existing tracked user modification to
`config/swarm_rviz.rviz` is outside B0 and must not be changed or reverted.

## 6. Authorized file whitelist

Luna Max may create or modify only:

```text
src/uav_simulator/so3_quadrotor_simulator/CMakeLists.txt
src/uav_simulator/so3_quadrotor_simulator/package.xml
src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/multi_quadrotor_simulator.h
src/uav_simulator/so3_quadrotor_simulator/src/multi_quadrotor_simulator_so3.cpp
src/uav_simulator/so3_quadrotor_simulator/launch/multi_simulator.launch
src/uav_simulator/so3_quadrotor_simulator/config/initial_states_1.yaml
src/uav_simulator/so3_quadrotor_simulator/config/initial_states_3.yaml
src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_test.py
src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_equivalence.test
src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_three_agent.test
docs/Codex_PhaseOffsetSwarm_B0_Isolated_Multi_Simulator_Self_Audit_2026-08-21.md
```

The historical `test/multi_sim_tests.py`, its existing `__pycache__`, all
seven-agent/scenario initial-state YAML files, `swarm_rviz.rviz`, and every
other prototype file remain reference-only and must not be modified, deleted,
renamed, or connected to CMake.

No modification is authorized in:

```text
src/swarm_planner/**
src/uav_simulator/so3_control/**
src/uav_simulator/map_generator/**
src/uav_simulator/odom_visualization/**
src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch
src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp
src/uav_simulator/so3_quadrotor_simulator/src/dynamics/**
```

Do not modify any proposal, roadmap, Tube document, Swarm Intent file, or this
execution specification.

## 7. Production architecture

Create target and executable:

```text
multi_quadrotor_simulator_so3
```

Use one ROS process with a fixed vector of one or three heap-owned agent
objects. Agent addresses must remain stable before subscriber callbacks are
registered; `std::vector<std::unique_ptr<...>>` is acceptable.

Each agent owns, independently:

- one `Quadrotor` plant;
- one command copy;
- one force disturbance and one moment disturbance;
- one previous motor-RPM vector for the inherited numerical fallback;
- its odometry/IMU messages and next publish time;
- its own publishers and subscribers.

There must be no shared command, disturbance, plant state, previous control,
publish timestamp, or callback target between agents.

Callbacks only copy their own message into their own agent state. The main loop
performs, for every agent in deterministic robot-ID order:

```text
read copied command/disturbance
-> compute motor RPM using the existing single-simulator formula
-> apply only that agent's input/disturbance
-> Quadrotor::step(dt)
-> publish only that agent's odom/imu when due
```

No callback or loop iteration may inspect another agent's state.

## 8. Plant-math parity contract

The following computation must be migrated algebraically from the current
single simulator without sign, coefficient, gain, quaternion, or saturation
changes:

- propeller `kf/km` correction;
- current yaw/external-yaw rotation handling;
- desired quaternion rotation matrix;
- `Psi < 1` thrust guard;
- attitude and angular-rate errors;
- inertia cross terms;
- moment command;
- motor mixing and negative squared-RPM clamp;
- square-root conversion;
- state-to-odometry and plant-to-IMU mapping.

Do not introduce new physical parameters or tune mass, inertia, arm length,
gravity, motor coefficients, rates, gains, or saturation.

The B0 implementation may factor this copied multi-only math inside the new
header/source, but the original single source remains untouched.

## 9. Configuration contract

Supported `num_agents` values are exactly:

```text
1
3
```

Reject every other value at startup. This is a stage-scope validation, not a
runtime mode.

The launch must load an explicit `initial_states` array. It must contain
exactly `num_agents` entries with unique contiguous `robot_id` values
`0..num_agents-1`. Each entry contains only:

```text
robot_id
x
y
z
yaw
```

Agent ROS names and topic namespaces are derived exactly as `uav_<robot_id>`;
do not accept arbitrary per-agent names. All numeric values must be finite and
`z > 0`. Missing, duplicate, out-of-range, or extra entries fail startup.

Authorized initial-state files:

`initial_states_1.yaml`:

```text
uav_0 = (0.0, 0.0, 1.0), yaw=0
```

`initial_states_3.yaml`:

```text
uav_0 = (0.0, -2.0, 1.0), yaw=0
uav_1 = (0.0,  0.0, 1.0), yaw=0
uav_2 = (0.0,  2.0, 1.0), yaw=0
```

This line layout is a neutral isolation fixture, not a formation controller or
paper scenario. Do not generate a circle, hexagon, corridor, or seven-agent
layout.

Required private parameters:

```text
num_agents
frame_id
rate/simulation
rate/odom
start_at_hover
initial_states
```

Validation requires finite positive rates and `rate/odom <= rate/simulation`.
Default launch values:

```text
num_agents=3
frame_id=world
simulation_rate=1000.0
odom_rate=100.0
start_at_hover=true
initial_state_file=initial_states_3.yaml
```

No command timeout parameter is permitted.

## 10. Topic and frame contract

For each `i in [0, num_agents)`:

Subscribers:

```text
/uav_i/so3_cmd
/uav_i/force_disturbance
/uav_i/moment_disturbance
```

Publishers:

```text
/uav_i/sim/odom
/uav_i/sim/imu
```

Messages remain the existing types:

```text
quadrotor_msgs/SO3Command
geometry_msgs/Vector3
nav_msgs/Odometry
sensor_msgs/Imu
```

Frame contract:

```text
odom.header.frame_id = frame_id
imu.header.frame_id = frame_id
odom.child_frame_id = uav_i/base_link
```

No leading slash is allowed in `child_frame_id`. Child frames must be unique.

The node must not advertise or subscribe to:

```text
PositionCommand
AgentState
g_coord/g_sep/g_coh/g_conf
map/cloud topics
goal topics
neighbor/swarm topics
phase/offset/Tube/CBF topics
```

## 11. Launch boundary

`multi_simulator.launch` starts exactly one
`multi_quadrotor_simulator_so3` node. It must not start:

- `so3_control`;
- `formation_planning` or any planner;
- map generator, local sensing, RViz, odom visualization, or TF helper;
- Swarm Intent nodes;
- any goal or command publisher.

The launch accepts only the parameters in section 9 and loads exactly one of
the authorized 1/3-agent state files selected by its arguments.

The original `simulator.launch` must remain byte-identical.

## 12. CMake and package contract

Modify the existing package minimally:

1. preserve the original `quadrotor_dynamics` and
   `quadrotor_simulator_so3` targets and link order;
2. add the new executable, linked to the same `quadrotor_dynamics` and Catkin
   libraries;
3. declare direct message dependencies used by the source rather than relying
   on transitive includes;
4. under `CATKIN_ENABLE_TESTING`, find `rostest` and register exactly the two
   B0 `.test` files;
5. do not add Swarm, planner, map, phase-offset, Tube, or CBF dependencies;
6. do not change global compiler standard, optimization, or original target
   flags as part of B0.

The test script may use only ROS Noetic system Python modules and standard
library code. Do not require Miniconda, install packages, or use the historical
hard-coded `localhost:11311` runner.

## 13. Test implementation contract

Create one focused Python rostest executable used by two launch tests. It must
not import or call the historical `multi_sim_tests.py`.

The test code may contain deterministic command-waveform helpers for test
stimulus; these are test fixtures, not production modes or simulator gates.

### 13.1 Equivalence rostest

`b0_multi_simulator_equivalence.test` starts, on the same private master:

1. one original `quadrotor_simulator_so3` node remapped to
   `/single/sim/odom`, `/single/sim/imu`, `/single/so3_cmd`;
2. one multi simulator with `num_agents=1`, explicit
   `initial_states_1.yaml`, and topics under `/uav_0/...`;
3. the equivalence test process.

Both plants use:

```text
initial position=(0,0,1)
initial yaw=0
start_at_hover=true
simulation_rate=1000 Hz
odom_rate=100 Hz or higher, equal on both nodes
```

After publisher connections exist, publish the exact same time-indexed,
bounded SO3 command to both command topics for at least 6 seconds. Use a small
smooth pitch/yaw-neutral excitation that retains positive thrust; do not tune
plant parameters.

Align odometry by command-relative timestamp and interpolate the denser track.
Acceptance requires at least 300 matched samples and:

```text
max position error <= 0.020 m
max linear-velocity error <= 0.020 m/s
max attitude geodesic error <= 0.010 rad
all compared values finite
both odom rates >= 80 Hz
```

These tolerances are frozen. Do not loosen them after observing a failure. A
failure must be diagnosed within the B0 implementation or reported.

Also assert the original node still publishes only its remapped single-agent
topics and the multi node publishes only `/uav_0/...` topics.

### 13.2 Three-agent isolation rostest

`b0_multi_simulator_three_agent.test` starts one multi simulator with
`num_agents=3`, explicit `initial_states_3.yaml`, and `start_at_hover=true`.

The test subscribes to all six odom/IMU topics concurrently. After at least 3
seconds of hover, require for every agent:

```text
finite odom and IMU
80 <= odom rate <= 130 Hz
position error from configured initial state < 0.020 m
yaw error < 0.050 rad
z > 0.95 m
unique child_frame_id == uav_i/base_link
```

Require exactly one multi-simulator ROS node and no `/uav_3` plant topics.

Command isolation phase:

1. record all three reference positions;
2. publish a bounded smooth positive-x tilt command only to
   `/uav_0/so3_cmd` for at least 4 seconds;
3. require `uav_0` displacement > 0.10 m;
4. require `uav_1` and `uav_2` displacement < 0.020 m;
5. require all three odom streams remain live and finite.

Disturbance isolation phase:

1. record new references for `uav_1` and `uav_2`;
2. publish a bounded +x force disturbance only to
   `/uav_1/force_disturbance`, then publish zero to clear it;
3. require a measurable `uav_1` response;
4. require `uav_2` displacement remains < 0.020 m;
5. require no topic or state mutation for another plant is needed to clear the
   disturbance.

The graph audit must confirm the multi node's publisher/subscriber endpoints
are exactly the per-agent plant topics in section 10, plus normal ROS
infrastructure. No planner, map, Swarm Intent, PositionCommand, or shared
cross-agent command topic may exist on the private master.

## 14. Required execution gates

Follow this order exactly.

### G0: preflight and reference audit

- record branch/HEAD/status/diff and protected hashes;
- confirm CMake/package are tracked-clean at start;
- confirm the multi header/source/launch and 1/3 YAMLs are existing untracked
  reference assets and save their hashes;
- confirm no authorized file is being edited concurrently;
- confirm no build process is active.

Report G0 evidence to the root supervisor before editing.

### G1: static implementation

- remove prototype command timeout/failsafe and arbitrary-N layout behavior;
- implement exact 1/3-agent configuration validation;
- implement independent plant ownership and exact topic contract;
- add the minimal CMake/package declarations;
- add the focused launch/config/tests;
- run Python syntax checking without leaving a new `__pycache__` in the
  authorized B0 test files (use a temporary pycache prefix or remove only
  stage-created cache artifacts).

Before building, perform static searches proving no Swarm/Tube/CBF/planner/map
dependency and no extra runtime gate.

### G2: package build

With ROS Noetic sourced and no concurrent build:

```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --pkg so3_quadrotor_simulator
```

Record that both original and new executables were linked from the current
sources. Do not use the stale August 6 multi binary as evidence.

### G3: equivalence test

Run the registered equivalence rostest and one explicit private-master
`--reuse-master` acceptance. Both must pass the frozen metrics in 13.1.

If equivalence fails, do not adjust tolerances, rates, plant parameters, or the
original simulator. Fix only an actual B0 implementation defect or stop.

### G4: three-agent isolation test

Run the registered three-agent test and one explicit private-master
`--reuse-master` acceptance. All hover, topic, frame, command-isolation,
disturbance-isolation, and graph checks must pass.

### G5: regressions and final audit

- re-run both B0 registered rostests together;
- run a direct original single-simulator smoke test on a private master;
- attempt a full incremental workspace build with `-j2` if no concurrent build
  is active; if an unrelated dirty-worktree defect blocks it, preserve the B0
  evidence and report without fixing outside the whitelist;
- compare protected hashes;
- run dependency/no-gate/topic searches;
- run `git diff --check` and an explicit trailing-whitespace scan for untracked
  whitelist files;
- compare final status against G0 and verify only whitelist files were changed
  by B0.

## 15. ROS process isolation

Every explicit acceptance run must use:

- a new unique `/tmp/codex_b0_multi_sim_*` directory;
- a new `ROS_HOME` inside it;
- a new private ROS master on a prechecked free port;
- recorded master URI, master PID/PGID, launch/test PID/PGID, topic list, and
  node list;
- cleanup limited to the run's own process groups;
- final proof that every run-owned PID is gone and the private port is free.

Do not attach to, query through, signal, or terminate any existing user-owned
ROS master. The machine currently contains multiple historical private masters;
their existence is expected and not a reason to reuse them.

Registered `rostest` may use its own temporary master, but the explicit
acceptance must use the chosen master with `--reuse-master` so the identity and
cleanup evidence are unambiguous.

## 16. Required static audits

Search production multi-simulator files and require no matches for:

```text
phase_offset
Tube
CBF
allocator
neighbor
AgentState
g_coord
formation_planning
SDFMap
map_generator
PositionCommand
command_timeout
failsafe
watchdog
swarm_enabled
mode
```

Ordinary English comments such as "no neighbor logic" should be avoided so the
audit remains unambiguous.

Verify additionally:

- only 1 and 3 are accepted;
- no circle/default arbitrary layout code exists;
- no seven-agent config is loaded by the launch or tests;
- no shared `SimulatorCommand`, disturbance, control, or plant exists outside
  individual agent objects;
- no post-command clamp or safety filter was added beyond inherited motor
  mixing and numerical RPM handling;
- original target/launch behavior was not placed behind a new flag;
- no historical prototype test was added to CMake.

## 17. Self-audit deliverable

Create:

```text
docs/Codex_PhaseOffsetSwarm_B0_Isolated_Multi_Simulator_Self_Audit_2026-08-21.md
```

It must include:

- start/end timestamps, branch, full HEAD;
- initial/final status and tracked diff summary;
- exact modified/created files;
- protected original hashes before/after;
- explanation of which prototype behaviors were rejected and removed;
- build commands and results;
- equivalence metrics and sample counts;
- three-agent hover/rate/frame metrics;
- command and disturbance isolation measurements;
- private master/ROS_HOME/PID/port identities and cleanup proof;
- original single smoke and full-build result;
- dependency, topic, no-extra-gating, whitelist, and formatting audits;
- deviations/blockers;
- explicit stop before B1/B2/B3, Swarm Intent execution, allocator, CBF, Tube,
  planner, and control-chain integration.

## 18. Stop conditions

Stop without expanding scope if:

- branch or HEAD differs;
- an authorized reference path contains concurrent user work that cannot be
  preserved;
- a required change touches a non-whitelist file;
- equivalence requires modifying the original simulator/dynamics/launch;
- acceptance appears to require command timeout, failsafe, a new gate/mode,
  controller, planner, map, Swarm Intent, or CBF;
- the three-agent test requires a seven-agent/scenario asset;
- ROS runtime is unavailable;
- another task repeatedly occupies the shared build tree and the root cannot
  provide a safe build window;
- a workspace-wide regression fails for an unrelated historical defect.

Do not create a commit, branch, tag, stash, or push. Do not stage files. Do not
reset, restore, clean, rebase, or overwrite unrelated changes.

## 19. Root-supervision checkpoints

Luna Max must report to `/root` at these checkpoints and continue only after
root acceptance where stated:

1. G0 preflight and prototype findings -- wait for root release to edit;
2. G1 static implementation and no-extra-gating audit -- wait for root release
   to build;
3. G2 build -- report before runtime tests;
4. G3 equivalence metrics -- wait for root acceptance before G4;
5. G4 three-agent metrics and cleanup -- report before final regression;
6. final self-audit -- stop and send the final result.

At no checkpoint may Luna Max create or consult a sub-agent.
