# PhaseOffsetSwarm B1E Original-to-B1 Single-UAV Equivalence Diagnostic Execution Specification

Date: 2026-08-21

Owner: Codex root supervisor

Executor: Luna Max only

Repository:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

Expected identity:

```text
branch = main
HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
```

Execution policy:

```text
AUTO_ADVANCE = false
LUNA_SUBAGENTS = forbidden
PRODUCTION_SOURCE_EDITS = forbidden
TUBE_EDITS = forbidden
SWARM_CONTROL_INTEGRATION = forbidden
NEW_RUNTIME_GATES = forbidden
```

This is a diagnostic recovery stage. It does not authorize the B1 three-UAV
run, active Swarm Intent, Tube, allocator, CBF, controller tuning, or a
terminal-control correction.

## 1. Why this diagnostic stage exists

The first B1 single-UAV private-master run did not satisfy the frozen B1
stable-arrival condition. That run nevertheless showed valid initial state,
one accepted goal publication, successful path installation and replanning,
physical navigation to about 0.198 m XY distance from the goal, no node crash,
and finite observed data.

The user then independently launched the original navigation entry points:

```text
so3_quadrotor_simulator/launch/simulator.launch
bspline_race/launch/test_gvf.launch
```

and observed normal navigation and stopping.

Therefore the previous classification, "protected original single-agent
baseline failure", is withdrawn. The evidence only proves that the B1 fixture
and its strict acceptance did not pass. It does not prove an original
navigation defect.

The B1 fixture differs from the original single-UAV entry in several ways:

1. global operational topics versus per-UAV namespaced topics;
2. manual-map layer enabled versus disabled;
3. standalone SO3 nodelet versus manager-loaded SO3 nodelet;
4. original single simulator versus B0 multi simulator with one agent;
5. original GVF gains `2.0/-2.2` versus B1 gains `0.8/-0.88`;
6. effective SO3 `use_external_yaw=true` in the original launch versus the B1
   explicit value `false`;
7. user-observed visual stopping versus the frozen quantitative B1 condition.

B1E isolates these differences one at a time. It must produce evidence before
any production correction is proposed.

## 2. Mandatory executor rules

Luna Max must execute this stage personally and must not:

- start or delegate to a sub-agent;
- ask another model or agent to inspect, implement, test, or summarize;
- edit this execution specification;
- edit any file outside the whitelist in Section 8;
- edit, format, move, delete, stage, stash, restore, or overwrite user work;
- modify Tube, Swarm Intent, allocator, CBF, governor, planner, SDF, simulator,
  SO3, CMake, package manifests, messages, or production source;
- add a runtime mode, feature flag, health gate, readiness gate, watchdog,
  command timeout, fallback, retry controller, terminal controller, or
  acceptance-dependent production behavior;
- tune a velocity limit, acceleration limit, saturation, collision threshold,
  goal threshold, SO3 gain, or planner bound;
- use port 11311;
- attach to, inspect through, signal, stop, or clean a user-owned ROS master;
- run the next checkpoint without an explicit root release.

If a required action is outside the whitelist, stop and report.

## 3. Mandatory reading order

Before editing, Luna Max must read completely, in this order:

1. `AGENTS.md`;
2. this B1E execution specification;
3. `docs/Codex_PhaseOffsetSwarm_B1_Independent_Three_UAV_Navigation_Execution_Spec_2026-08-21.md`;
4. the B1 G3 failure evidence under
   `/tmp/codex_b1_independent_g3_20260821T144736_1010930`;
5. `src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch`;
6. `src/swarm_planner/bspline_traj/launch/test_gvf.launch`;
7. the eight current B1 implementation files listed in Section 7;
8. the original single simulator source and B0 multi simulator source;
9. `src/uav_simulator/so3_control/src/so3_control_nodelet.cpp`;
10. current planner topic/parameter construction in `gvf_manager.cpp`,
    `gvf.cpp`, `SDFMap`, and `local_sensing.cpp`.

Historical phase-offset, Tube, swarm, and scenario prototypes are not inputs to
this stage and must not be included by a B1E launch.

## 4. Scope and non-goals

### 4.1 Authorized objective

Create headless, isolated, single-UAV diagnostic fixtures which preserve the
same fixed B1 physical scenario while changing exactly one integration factor
between adjacent variants.

Collect enough receiver-side data to determine which condition prevented the
first B1 run from satisfying stable arrival.

### 4.2 Explicit non-goals

B1E does not:

- make the original navigation "pass" a new requirement;
- change the original navigation;
- change B1 acceptance thresholds;
- rerun the B1 three-agent scenario;
- connect Swarm Intent to control;
- enable phase offset, Tube, allocator, or CBF;
- add terminal hold behavior;
- change the point-goal reached threshold;
- infer failure from visual appearance alone;
- use the user's currently running original-navigation master as test input.

## 5. Frozen physical scenario

Every B1E variant uses the same map, start, goal, plant rate, odometry rate,
mass, and SO3 gains except for explicitly isolated factors. Original frame
metadata and hover initialization are preserved first, then changed in their
own variants before the plant implementation is replaced.

Map:

```text
src/uav_simulator/dynamic_map_generator/resource/pillar.pcd
```

Start and goal:

```text
start = (-5.0, -3.50, 1.0), yaw 0.0
goal  = ( 5.0, -3.50, 1.0), frame world
```

Rates and controller constants:

```text
plant integration = 1000 Hz where supported by the selected plant
odometry = 100 Hz
mass = 0.98
gains/rot/z = 1.0
gains/ang/z = 0.1
gains_hummingbird.yaml = unchanged
corrections_hummingbird.yaml = unchanged
```

The test publishes exactly one goal per run. It must not republish the goal to
make a variant settle.

## 6. Frozen diagnostic matrix

The variants are ordered. Each adjacent pair changes exactly the factor shown.

| ID | Plant | SO3 topology | Planner topics | Manual map | GVF K1/K2 | Difference from prior |
|---|---|---|---|---|---|---|
| E0 | original single | standalone | global | enabled, empty file | 2.0/-2.2 | headless original-equivalent reference |
| E1 | original single | standalone | global | disabled | 2.0/-2.2 | manual layer only |
| E2 | original single | standalone | `/uav_0` isolated | disabled | 2.0/-2.2 | namespace/remap boundary only |
| E3 | original single | manager-loaded | `/uav_0` isolated | disabled | 2.0/-2.2 | SO3 loading topology only |
| E4 | original single, frame `world`, no hover initialization | manager-loaded | `/uav_0` isolated | disabled | 2.0/-2.2 | frame metadata only |
| E5 | original single, frame `world`, hover initialized | manager-loaded | `/uav_0` isolated | disabled | 2.0/-2.2 | hover initialization only |
| E6 | B0 multi, N=1, frame `world`, hover initialized | manager-loaded | `/uav_0` isolated | disabled | 2.0/-2.2 | plant implementation only |
| E7 | B0 multi, N=1, frame `world`, hover initialized | manager-loaded, B1 SO3 effective params | `/uav_0` isolated | disabled | 2.0/-2.2 | `use_external_yaw` only |
| E8 | B0 multi, N=1, frame `world`, hover initialized | manager-loaded, B1 SO3 effective params | `/uav_0` isolated | disabled | 0.8/-0.88 | common gain scale only |

E8 must include the existing read-only
`b1_pillar_single_baseline.launch`; it must not duplicate or modify it.

### 6.1 E0 safety exception

The original `test_gvf.launch` enables manual-map auto-save to a repository
file. B1E must not allow diagnostic shutdown to rewrite that user file.

E0 preserves:

```text
enable_manual_map = true
manual_map_auto_load = true
manual_map_file = existing protected empty manual-map file
```

and overrides only:

```text
manual_map_auto_save = false
```

The protected manual file currently contains only its comment header. Its hash
must be verified before and after E0. No manual clicks are published.

### 6.2 Fixed factor definitions

"Original single plant" means the existing `quadrotor_simulator_so3` binary
with the original simulator parameter/private-topic contract, instantiated
headlessly at the frozen start. E0--E3 preserve the original source defaults:

```text
simulator/frame_id = /simulator
simulator/start_at_hover = false
```

E4 changes only `simulator/frame_id` to `world`. E5 then changes only
`simulator/start_at_hover` to `true`. Do not include RViz or odom
visualization.

"B0 multi, N=1" means protected `multi_simulator.launch` with the existing B1
one-agent YAML.

"Standalone" means:

```text
nodelet standalone so3_control/SO3ControlNodelet
```

"Manager-loaded" means one `/so3_nodelet_manager` and one logical
`/so3_control_nodelet_0`.

The original `simulator.launch` writes `use_external_yaw ` with a trailing
space, while `SO3ControlNodelet` reads `use_external_yaw` and uses default
`true` when the correct key is absent. B1E encodes effective behavior, not the
launch typo:

```text
E0--E6: use_external_yaw = true
E7--E8: use_external_yaw = false
```

The similarly spaced `use_angle_corrections ` key is not read by the protected
SO3 nodelet and is not a behavioral factor in this matrix.

"Global" means:

```text
/sim/odom
/sim/imu
/sim/local_map
/move_base_simple/goal
/position_cmd
/so3_cmd
```

"`/uav_0` isolated" means the exact current B1 per-agent topic/remap contract.

## 7. Existing B1 files are read-only

The following existing B1 files must not change:

```text
src/swarm_planner/bspline_traj/launch/b1_independent_agent.launch
src/swarm_planner/bspline_traj/launch/b1_pillar_single_baseline.launch
src/swarm_planner/bspline_traj/launch/b1_pillar_independent_3.launch
src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/initial_states_1.yaml
src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/initial_states_3.yaml
src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py
src/swarm_planner/bspline_traj/test/b1_independent_single.test
src/swarm_planner/bspline_traj/test/b1_independent_three.test
```

Frozen SHA256 values:

```text
12e706e8153140d5c46663a7c2af0ed3c3ea8adcb077a7617a248ae1b202410a  b1_independent_agent.launch
ee1e171610c0b2072a8fdf49e52de7d5483bc34c836ec724932a9c37f5d1eb4b  b1_pillar_single_baseline.launch
841e449cd1d7be5a1ccfc1e4112eec886480c0cf8c29ab971848c326f423259a  b1_pillar_independent_3.launch
7b75d77f567f6521c213c7ba5d893dfd74c88ddc36ab63f3d8116aeeb6853acb  initial_states_1.yaml
727c1c98efeb06aa133346e5cfbf4f126a0288762f2b487b099f6b2117cac668  initial_states_3.yaml
931bc55b6fe5673e3d8b6129494846780008df5c91fb55c933ceba18f7d5a044  b1_independent_navigation_test.py
2f26d62d0ec9ef3734116cb6a4067efa389b4aa7af8559d5a734579a9fa0ed3e  b1_independent_single.test
8d5f53de60df7d998c81362b3b6afe00aab69d860981e7e799338097580eae00  b1_independent_three.test
```

If any value differs, stop for root re-audit.

## 8. Complete B1E file whitelist

Luna Max may create or modify only:

```text
src/swarm_planner/bspline_traj/launch/b1e_original_global_manual_on.launch
src/swarm_planner/bspline_traj/launch/b1e_original_global_manual_off.launch
src/swarm_planner/bspline_traj/launch/b1e_namespaced_single_standalone_original_gains.launch
src/swarm_planner/bspline_traj/launch/b1e_namespaced_single_manager_original_gains.launch
src/swarm_planner/bspline_traj/launch/b1e_namespaced_single_manager_world_frame_original_gains.launch
src/swarm_planner/bspline_traj/launch/b1e_namespaced_single_manager_world_hover_original_gains.launch
src/swarm_planner/bspline_traj/launch/b1e_namespaced_multi_manager_original_gains.launch
src/swarm_planner/bspline_traj/launch/b1e_namespaced_multi_manager_original_gains_b1_so3_params.launch
src/swarm_planner/bspline_traj/test/b1e_single_equivalence_diagnostic_test.py
src/swarm_planner/bspline_traj/test/b1e_e0_original_global_manual_on.test
src/swarm_planner/bspline_traj/test/b1e_e1_original_global_manual_off.test
src/swarm_planner/bspline_traj/test/b1e_e2_namespaced_single_standalone.test
src/swarm_planner/bspline_traj/test/b1e_e3_namespaced_single_manager.test
src/swarm_planner/bspline_traj/test/b1e_e4_namespaced_single_manager_world_frame.test
src/swarm_planner/bspline_traj/test/b1e_e5_namespaced_single_manager_world_hover.test
src/swarm_planner/bspline_traj/test/b1e_e6_namespaced_multi_manager_original_gains.test
src/swarm_planner/bspline_traj/test/b1e_e7_namespaced_multi_manager_b1_so3_params.test
src/swarm_planner/bspline_traj/test/b1e_e8_b1_scaled_gains.test
docs/Codex_PhaseOffsetSwarm_B1E_Original_To_B1_Single_Equivalence_Diagnostic_Self_Audit_2026-08-21.md
```

No CMake/package registration is authorized. Tests are invoked directly by
`rostest`. The root-owned execution specification is read-only to Luna Max.

## 9. Protected no-touch scope

Everything outside Section 8 is read-only. This explicitly includes:

- all `src/swarm_planner/phase_offset/` files;
- all Tube and phase-offset integration files under `bspline_traj`;
- `gvf_manager`, `gvf`, continuous phase path, Tube builder, allocator, CBF,
  neighbor model, and adapters;
- `plan_env` source, headers, CMake, package, and tests;
- both simulator implementations and their launches/configs;
- SO3 source/config/package;
- `test_gvf.launch`;
- all existing B0/B1/Swarm Intent/Tube documents;
- every `CMakeLists.txt` and `package.xml`;
- the user-owned manual-map file.

If the concurrent Tube V2 owner changes a dependency used by B1E between
variants, stop and ask root to re-audit. Do not reconcile it.

## 10. Protected dependency hashes

At G0 and final audit record existing B1 hashes plus:

```text
ce1ff4a3daca6efeef39507c14227afd806f94d55b85c200b8c8a5032b213fc5  AGENTS.md
cc6ae11d739c62d20f937acc5267fa542857f0a89e4d57dc56f5ae8823985555  test_gvf.launch
4a454d6a997e875d50114b51d920a109638f84c8fc38cd2ef803d4d401f43314  gvf_manager.cpp
d13475010de18dd3f7280b7aee31aad6b7431560cc248468608906de0c290788  gvf_manager.h
f98805f3a8ff44386480dabdab1de80934dd0e6fc7076ad583172a4806f01880  local_sensing.cpp
609cab63db1209be0d0390931782eb702859a0d85964a7f3ff8394966ec23414  pillar.pcd
b39a59b20383b90b26075a207a3df4c5e402b0955ee7df6bd60f6d61726fc661  so3_control_nodelet.cpp
708f6922c6f541cf9d979bc6a12f7692abb69203d4bd79f066f4d33bee11df07  multi_simulator.launch
bee45213c89f203c8364ebd0e5da7b9e641375c846fdc2d08accf3541962c6e2  multi_quadrotor_simulator_so3.cpp
7b639f072e7acb88426a14018e1c8ab93106c06bdd028f10eb9371cdb9e153c3  multi_quadrotor_simulator.h
9a2452e2c329f3ef8f49faadb93e05e7f7f888d81dc977b040457009bfc65c92  quadrotor_simulator_so3.cpp
d6339f7d6b0cde4beffa3b037ef28a2415ccd5fa1419f46a6b97df2c9d3134c6  simulator.launch
0d5d0af994674ffefa87886a73bc580fc36606b34389346c9799b60d54d3281f  dynamics/Quadrotor.cpp
86652b16839d503bc0fc7a21d8420926af5b7488383112f6acb89d76d02fb222  include/Quadrotor.h
740b4f1da4d42cf75729cb0af1b0bc2f7f3d422905e30cfb5cd43f13a8fd434f  manual_obstacles_test_gvf.txt
```

Use full paths in evidence.

## 11. Launch implementation contracts

### 11.1 Common rules

Every new launch must:

- start exactly one `map_pub` using `pillar.pcd`;
- start exactly one plant and one SO3 logical controller;
- include `test_gvf.launch` exactly once;
- set `phase_offset_mode=disabled` explicitly;
- start no RViz, visualizer, scenario publisher, waypoint generator, recorder,
  Tube node, Swarm Intent node, allocator, or CBF node;
- expose no arbitrary population, gain, mode, feature, or scenario argument;
- contain no runtime conditional selecting a variant;
- encode its variant as a separate fixed launch file;
- set the frozen start literally;
- preserve production maxima and saturation parameters.

### 11.2 E0 global/manual-on

Use original single simulator binary, original standalone SO3 topology, and
global topics. Include `test_gvf.launch` at root with mode disabled and
original gains. After include override only:

```text
/formation_planning/sdf_map/manual_map_auto_save = false
```

Verify enable=true, auto_load=true, gains `2.0/-2.2`.

### 11.3 E1 global/manual-off

Identical to E0 except set all three manual-map flags false. No topic, plant,
controller, or gain change.

### 11.4 E2 namespaced/original-single/standalone

Keep original single simulator and standalone SO3, but remap all operational
endpoints to exact `/uav_0` B1 topics. Run planner/local sensing under
`/uav_0`, apply current B1 absolute legacy remaps, disable manual map, and keep
gains `2.0/-2.2`.

### 11.5 E3 namespaced/original-single/manager

Identical to E2 except replace standalone SO3 with one manager and exactly one
loaded `/so3_control_nodelet_0`. All other facts remain identical.

### 11.6 E4 namespaced/original-single/manager/world frame

Identical to E3 except set the original single plant private parameter:

```text
simulator/frame_id = world
```

Keep `simulator/start_at_hover=false`. This isolates frame metadata only.

### 11.7 E5 namespaced/original-single/manager/world frame/hover

Identical to E4 except set:

```text
simulator/start_at_hover = true
```

This isolates initial motor RPM/default hover command initialization only.

### 11.8 E6 namespaced/multi/manager/original gains

Identical to E5 except replace original single plant with protected B0 multi
simulator N=1 and existing B1 one-agent YAML. Keep gains `2.0/-2.2` and
effective `use_external_yaw=true`.

### 11.9 E7 namespaced/multi/manager/B1 SO3 parameters

Identical to E6 except set the correctly named SO3 private parameter:

```text
use_external_yaw = false
```

Keep gains `2.0/-2.2`. This isolates the effective SO3 parameter difference.

### 11.10 E8 current B1 scaled gains

Include existing read-only `b1_pillar_single_baseline.launch`. Do not recreate
its nodes. E8 differs from E7 only in fixed gains `0.8/-0.88`.

## 12. Diagnostic test topic contract

The Python diagnostic supports two layouts selected only by private test
parameters in fixed `.test` files.

Global layout:

```text
odom=/sim/odom
imu=/sim/imu
local_map=/sim/local_map
goal=/move_base_simple/goal
position_cmd=/position_cmd
so3_cmd=/so3_cmd
path=/particle0/path
```

Namespaced layout:

```text
odom=/uav_0/sim/odom
imu=/uav_0/sim/imu
local_map=/uav_0/sim/local_map
goal=/uav_0/goal
position_cmd=/uav_0/position_cmd
so3_cmd=/uav_0/so3_cmd
path=/uav_0/particle0/path
```

Each `.test` fixes variant ID, layout, expected plant node, SO3 topology,
gains, and manual flags. These observation parameters are not forwarded to
production nodes and are not feature gates. No `.test` exposes variant-changing
arguments.

## 13. Observation timeline

Each variant uses receiver-monotonic time:

1. wait up to 25 s for finite odom and local map;
2. require initial position error `<0.02 m`;
3. create one latched goal publisher;
4. wait up to 15 s for a subscriber;
5. record goal anchor immediately before publish;
6. publish exactly one goal;
7. observe exactly 60.0 s after the anchor;
8. compute complete metrics;
9. run graph/parameter audits;
10. emit exactly one `B1E_METRICS` JSON object.

Every `.test` has `time-limit="125.0"`. This is test containment, not a
production gate. Do not stop early when strict acceptance succeeds.

## 14. Mathematical metrics

For sample `k`, receive time `t_k`, position `p_k`, velocity `v_k`, goal `g`:

```text
e_xy(k) = sqrt((p_x(k)-g_x)^2 + (p_y(k)-g_y)^2)
e_z(k)  = abs(p_z(k)-g_z)
s(k)    = ||v_k||_2
```

Frozen B1 instantaneous set:

```text
S = {k | e_xy<=0.25 m, e_z<=0.10 m, s<=0.15 m/s}
```

Without changing thresholds, compute:

- minimum XY, Z, and 3D goal errors with time/state;
- minimum speed after first `e_xy<=0.25` entry;
- first sample in `S`;
- longest continuous observed dwell in `S`;
- first complete 3.0 s continuous window in `S`;
- stable reference at the first sample at/after its endpoint;
- maximum displacement for the following 5.0 s;
- strict B1 pass/fail and component failure reasons.

Use receive-monotonic time for goal windows, ROS header stamps for rate/progress.
Never require exact floating-point endpoint equality.

## 15. Required terminal diagnostics

### 15.1 Final five-second physical state

For `[goal+55 s, goal+60 s]`, record count, first/last/mean position, min/max XY
and Z error, min/max/mean speed, and maximum displacement from the first final
window position.

### 15.2 PositionCommand terminal state

Record bounded finite scalar data for every PositionCommand: receive time,
header stamp, position, velocity, acceleration, yaw, yaw rate.

Report count after goal; first/last relative times; last commanded position,
velocity, acceleration; its XY/Z/3D goal error; silence from last command to
observation end; physical state closest in time to the last command; and final
physical distance from the last commanded position.

### 15.3 SO3 and liveness

Record finite SO3Command stamps/counts. Report positive timestamp progress for
odom, local map, PositionCommand, and SO3Command. Keep odom 80--130 Hz as a
diagnostic. Add no command-rate threshold.

### 15.4 Path and clearance

Record non-empty path count, every frame, last path endpoint and goal error,
minimum raw PCD path clearance, and minimum physical raw PCD clearance. The
current B1 conservative PCD hash may be copied; do not import or edit B1 test.

## 16. Diagnostic outcome semantics

The test distinguishes:

```text
run_valid
strict_b1_pass
```

`run_valid` requires finite readiness, correct initial position, connected goal
subscriber, exactly one goal publish, finite data, at least one non-empty path,
variant-correct nodes/topics/parameters, and no forbidden runtime endpoints.

`strict_b1_pass` is a metric, not the diagnostic process exit condition. A
valid variant that misses strict acceptance still exits zero after complete
metrics. Infrastructure/wiring failure fails after metrics emission. This does
not relax B1 acceptance; it prevents loss of diagnostic evidence.

## 17. Graph and parameter audit

No live node/topic may contain, case-insensitively:

```text
swarm phase_offset_swarm swarm_intent allocator cbf tube scenario visualizer rviz
```

Do not generically scan protected disabled parameter names for `tube`.

E0/E1 require `/map_pub`, `/quadrotor_simulator_so3`, `/so3_control`,
`/formation_planning`, `/map_generator`, and global operational topics.

E2 requires original plant and standalone SO3 with `/uav_0/formation_planning`,
`/uav_0/map_generator`, namespaced operational topics, and no global legacy
operational endpoints.

E3 requires E2 planner/plant facts plus `/so3_nodelet_manager` and NodeletList
exactly `[/so3_control_nodelet_0]`. Do not infer logical nodelet ownership from
ROS master caller identity.

E4/E5 retain `/quadrotor_simulator_so3` and the E3 manager/planner graph.
Their parameter audit additionally distinguishes original frame/hover settings.

E6/E7/E8 require `/multi_quadrotor_simulator_so3` and forbid the original single
plant node.

Require exact topic parameters, disabled phase-offset mode, manual flags, and
gains for each variant. Do not add or inspect a new production enable flag.

## 18. Structured output

Emit exactly one strict-JSON line beginning `B1E_METRICS `. It must include:

```text
variant_id
run_valid
infrastructure_failures
strict_b1_pass
strict_failure_components
python executable/version
start/goal
initial position error
goal publish count
observation duration
per-stream finite/progress/rate/count
path count/frames/endpoint/clearance
physical clearance
minimum XY/Z/3D errors with times and states
first instantaneous strict-set entry
longest strict-set dwell
first stable window
post-stable drift
final five-second state summary
PositionCommand terminal summary
SO3 summary
graph audit
parameter audit
protected variant facts
```

Use `allow_nan=false`; sanitize non-finite diagnostic values to null. The
`finally` path must emit collected metrics on exceptions.

## 19. Static implementation requirements

The Python test must use `/usr/bin/python3`, set `sys.dont_write_bytecode=True`
before ROS imports, retain the existing safe message fallback, avoid NumPy,
rosbag, plotting, shell/process/network operations, retain bounded scalar/tuple
traces rather than raw clouds, lock callbacks, and unregister only its own goal
publisher.

It must never publish control, state, path, or phase-offset data.

Run synthetic pure-Python checks for:

1. strict pass;
2. XY pass/Z fail;
3. XY/Z pass/speed fail;
4. stable pass/drift fail;
5. later re-stabilization not washing out first-stable drift failure;
6. final-window summary;
7. terminal PositionCommand summary;
8. ROS master endpoint inversion.

## 20. Private-master and secret-safe execution

Every variant uses unique `/tmp/codex_b1e_<variant>_<timestamp>_<pid>`,
`ROS_HOME`, `ROS_TEST_RESULTS_DIR`, loopback port, roscore PGID, and rostest
PGID.

Select a free ephemeral port and reject 11311, 12901, and every listening port.
Immediately before start, record and verify the command contains:

```text
roscore -p <selected_port>
```

Plain `roscore` or missing `-p` is a hard stop.

Run ROS children under a minimal allowlisted environment containing only local
runtime necessities:

```text
HOME USER LOGNAME SHELL PATH LANG LC_ALL
ROS_ROOT ROS_ETC_DIR ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
ROS_PACKAGE_PATH ROS_MASTER_URI ROS_HOME ROS_TEST_RESULTS_DIR
CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
```

Do not pass API keys, auth tokens, cookies, proxy credentials, Codex/session
identifiers, or unrelated variables. Do not print secret values while building
the allowlist.

Use separate process groups. Cleanup only recorded owned PGIDs: TERM, bounded
wait, then KILL only the same group if needed. Prove owned PIDs/PGIDs gone and
port free. Never use `pkill`, `killall`, name matching, or global ROS cleanup.

## 21. Build contract

After static acceptance and a root safe window, use exactly:

```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --pkg so3_quadrotor_simulator map_generator so3_control bspline_race
```

Do not clean, repair unrelated failures, or add CMake registration.

Before build/every dynamic run check for shared writers: `catkin_make`,
`cmake --build`, `make`, `ninja`, `run_tests.py`. If present, wait for root and
never signal them.

## 22. Root-supervised checkpoints

### G0: preflight and read-only audit

Confirm no subagents; record branch/HEAD/status/diff stat/protected hashes;
confirm whitelist paths absent/non-conflicting; inspect G3 evidence and current
contracts; prove all nine variants need no production edit; safely observe build
writers and user-owned ROS processes; report and wait. No editing.

### G1: static implementation

After release, create only Section 8 launch/test files except self-audit. Run
`xmllint`, `roslaunch --nodes`, temporary-pycache syntax check, synthetic
checks, exact variant searches, no-gate/no-Tube/no-Swarm scans, whitelist/mode,
whitespace, `git diff --check`, and protected hashes. Report and wait.

### G2: package build

After safe-window release, run Section 21 exact build, record provenance and
hashes, report, and wait.

### G3--G11: one variant per root release

Run exactly once on a fresh private master, report complete `B1E_METRICS`,
secret-safe environment evidence, logs, cleanup, and adjacent delta, then wait:

```text
G3 = E0
G4 = E1
G5 = E2
G6 = E3
G7 = E4
G8 = E5
G9 = E6
G10 = E7
G11 = E8
```

E8 uses the current B1 launch through the B1E diagnostic test; it is not a
rerun of the old early-asserting acceptance test.

### G12: comparison and self-audit

Produce a nine-row table; identify the first adjacent material change; separate
topology, parameter, and acceptance effects; do not propose an unsupported
production fix; rerun static/hash audits; write self-audit; stop before
production edits or B1 G3/G4 continuation.

## 23. Comparison rules

For each adjacent pair report exact deltas in strict pass, minimum XY/Z/3D
error, closest-approach time, minimum near-goal speed, longest strict dwell,
stable result, drift, final-window mean state, last PositionCommand time/state,
command silence, path endpoint, clearances, and rates/counts.

Do not use "looks similar" as primary evidence. A causal finding requires both
variants valid and the named factor to be the only launch-contract difference.

If no adjacent factor explains the result, report that the fixed scenario or
strict quantitative criterion differs from the user's visual observation. Do
not invent a controller change.

## 24. Stop conditions

Stop if branch/HEAD/protected hash/B1 file changes; Luna edits Tube/Swarm or any
file outside Section 8; a variant requires a production flag/gate/timeout/
fallback/mode; a user master would need querying/stopping; a shared build writer
is active without safe release; roscore command lacks `-p`; port is forbidden
or occupied; ROS unavailable; a variant cannot yield valid comparison;
minimal environment is not used; logs inherit secrets; or passing requires
threshold/control changes.

Do not continue to the next variant after a stop condition.

## 25. Final required conclusion

The self-audit must end with one evidence-backed outcome:

1. manual layer causes the divergence;
2. namespace/remap boundary causes it;
3. SO3 loading topology causes it;
4. frame metadata causes it;
5. hover initialization causes it;
6. single-to-multi plant substitution causes it;
7. effective SO3 `use_external_yaw` causes it;
8. common gain scaling causes it;
9. all implementations navigate normally but strict B1 acceptance differs
   from visual stopping;
10. evidence is insufficient due to a named stop condition.

Then stop. Production correction, B1 revision, or three-agent continuation
requires a new root-owned execution specification.
