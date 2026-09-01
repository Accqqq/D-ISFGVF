# D-ISFGVF SIM-A Generic-N Physical Simulator

Frozen execution specification for the new multi-UAV simulation workstream.

DOCUMENT_STATUS=FROZEN_SPEC_AUDITED_PASS
PRIOR_FRESH_SOL_SPEC_AUDIT=SPEC_AUDIT_PASS
FRESH_SOL_DELTA_AUDIT=SPEC_AUDIT_PASS
AUTHORITATIVE_REPOSITORY=git@github.com:Accqqq/D-ISFGVF.git
AUTHORITATIVE_BRANCH=main
AUTHORITATIVE_BASELINE_COMMIT=7cf216a61891b88d2400e57ab81ea110eaad5b01
AUTHORITATIVE_BASELINE_MESSAGE=fix: close C3 navigation liveness regression

SIM_A_SPEC_PRE_AUDIT_SHA256=0fdd8b12d4790a852efec6bec32a21e8e6dc117892919c210242931e3e75c13c
SIM_A_SPEC_BOUND_FILE_SHA256=<REPORTED_EXTERNALLY_AFTER_BINDING>
SIM_A_IMPLEMENTED=false
SIM_A_AUTHORIZED=false
SIM_A_ACCEPTED=false
LUNA_LAUNCHED=false
SIM_B_AUTHORIZED=false
SIM_C_AUTHORIZED=false
SIM_D_AUTHORIZED=false
AUTO_ADVANCE=false

## 1. Scope and authority

This specification authorizes planning and, only after a separate explicit
user authorization, implementation of SIM-A: removal of the historical
`N={1,3}` restriction from the existing physical multi-quadrotor simulator.

This is specification-only until the gates in Sections 10 and 12 pass. This
document does not authorize implementation, worktree creation, building,
testing, staging, committing, pushing, or advancement to SIM-B, SIM-C, or
SIM-D.

The authoritative source is commit
`7cf216a61891b88d2400e57ab81ea110eaad5b01` on `main`. Superseded historical
overlays, candidate worktrees, and the old generic-N branch are reference-only
and must not be used as implementation baselines.

## 2. Frozen architecture

The final architecture is fixed and is not redesigned by SIM-A.

### WORLD

Future conceptual entry point: `phase_offset_world.launch`.

WORLD owns the shared map/world, RViz, and read-only visualization/metrics.
It must not own dynamics, planners, NeighborManager, SPH, `g_coord`, `g_des`,
or vehicle commands.

### SWARM

Future conceptual entry point: `phase_offset_swarm.launch`.

SWARM owns one generic-N `multi_quadrotor_simulator_so3` process, one shared
SO3 nodelet manager, and N independent instances of the current
`phase_offset_agent.launch` stack. It is not a central planner, central
`gvf_manager`, central NeighborManager, or central SPH controller.

The per-agent chain remains:

`/uav_i/sim/odom -> local_sensing_i -> local_map_i -> formation_planning_i ->`
`current accepted D-ISFGVF/PhaseOffset_i -> /uav_i/position_cmd ->`
`SO3Control_i -> /uav_i/so3_cmd -> Quadrotor_i -> /uav_i/sim/odom`.

SIM-B is future work. ROS1 XML has no native arbitrary-N loop, so the recorded
preferred SIM-B orchestration is a thin Python roslaunch launcher/generator
that only validates configuration, derives N, instantiates existing agent
launches, and starts the process topology. It must not compute control,
neighbors, SPH, `g_coord`, `g_des`, or commands.

SIM-C later adds one `AgentState` publisher and one local NeighborManager per
UAV. SIM-D later adds local SPH intent, `g_coord`, and per-command-tick `g_des`.
Neither stage is authorized here.

The explicit 50-UAV requirement is stage-separated: SIM-A must prove one
physical simulator process can instantiate and continuously simulate 50
plants; SIM-B must later prove WORLD/SWARM orchestration of 50 independent
`phase_offset_agent.launch` stacks; SIM-C/D must later prove 50-agent
communication, local NeighborManager/SPH, and `g_coord`/`g_des` integration.
N=50 SIM-A success is not acceptance of the full 50-agent algorithm stack.

## 3. Baseline findings

The following facts were independently checked in the authoritative commit:

- Current limit: hard-coded `num_agents == 1 || num_agents == 3` in both
  `main()` and `MultiQuadrotorSimulator::initialize()`.
- Storage: `std::vector<std::unique_ptr<SimulatedAgent>> agents_`.
- One process contains N independent `Quadrotor` plants.
- Initialization, integration, odometry/IMU publication, and topic creation
  are already loop-based.
- Current simulator-owned endpoints per agent are:
  `/uav_i/so3_cmd`, `/uav_i/sim/odom`, `/uav_i/sim/imu`,
  `/uav_i/force_disturbance`, and `/uav_i/moment_disturbance`.
- `phase_offset_agent.launch` is parameterized by `robot_id` and per-agent
  topic arguments and is reusable as an N-fold launch unit.
- `phase_offset_swarm_sim.launch` manually expands agents 0 through 6,
  including `init0_*` through `init6_*`, so the old orchestration has a
  practical seven-agent ceiling.
- Existing registered simulator regressions are
  `b0_multi_simulator_equivalence.test` (N=1) and
  `b0_multi_simulator_three_agent.test` (N=3).

Two known legacy inconsistencies are recorded but not repaired by SIM-A:

1. The parser currently requires exactly five fields per state, while several
   checked-in N>=7/state files include optional `name` metadata.
2. `phase_offset_swarm_sim.launch` forwards `command_timeout` to
   `multi_simulator.launch`, but that launch declares no such argument and the
   physical simulator does not implement timeout semantics.

The second item is classified as SIM-B/legacy-launch cleanup. SIM-A must not
modify either launch file for it. If it unexpectedly prevents SIM-A tests from
running, implementation stops and reports the blocker.

## 4. SIM-A objective

Make the physical simulator generic-N while preserving all accepted single-UAV
and multi-plant physical semantics:

- one simulator process;
- one independent `SimulatedAgent` and `Quadrotor` plant per state;
- independent command, previous-control, force disturbance, moment
  disturbance, odometry, IMU, publishers, and subscribers;
- loop-based integration and publication;
- deterministic per-ID topic generation;
- unchanged Quadrotor dynamics, motor model, `SO3Command` interpretation,
  hover initialization, frame IDs, timing, and disturbance handling.

SIM-A must not add AgentState, NeighborManager, SPH, `g_coord`, `g_des`,
planner logic, swarm coordination, or command sharing.

## 5. Canonical N and state-schema contract

`initial_states` is the sole authoritative source of N.

The simulator must require that `initial_states` exists, is an XML-RPC array,
and is nonempty. It then sets `N = initial_states.size()`.

If the `num_agents` private parameter is present for launch compatibility, it
must be an integer equal to N. It is a consistency check only and must not be
used as an independent source of N. Absence of the parameter is acceptable if
launch compatibility remains safe.

There must be no fixed ceiling or equivalent hidden ceiling: no N=1/N=3
guard, N<=7, N<=8, `MAX_AGENTS`, or other hard-coded limit.

Every state must contain:

- `robot_id` as an integer;
- numeric, finite `x`, `y`, `z`, and `yaw`;
- `z > 0`.

Robot IDs must be unique, contiguous, and exactly `0 ... N-1`. Missing,
duplicate, negative, out-of-range, or non-contiguous IDs are invalid.

Required fields are validated individually. The old `item.size() == 5` check
must be removed. Extra metadata, including `name`, must be tolerated and
ignored by the simulator; it must not acquire runtime, control, or topic
semantics.

## 6. Exact SIM-A production whitelist

The only authorized production path is:

1. `src/uav_simulator/so3_quadrotor_simulator/src/multi_quadrotor_simulator_so3.cpp`

The expected edit is limited to state parsing, canonical-N derivation,
optional `num_agents` consistency validation, optional-metadata tolerance, and
removal of the two historical N=1/N=3 guards. Existing dynamics, callbacks,
loops, and publication code are not to be redesigned.

No simulator header change is authorized. If implementation proves a header
change strictly necessary, stop before editing and report the exact conflict.

Explicitly forbidden production paths include:

- `src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/multi_quadrotor_simulator.h`
- `src/uav_simulator/so3_quadrotor_simulator/launch/multi_simulator.launch`
- `src/swarm_planner/bspline_traj/launch/phase_offset_agent.launch`
- `src/swarm_planner/bspline_traj/launch/phase_offset_swarm_sim.launch`
- `src/swarm_planner/bspline_traj/launch/test_gvf.launch`
- all `gvf_manager.*` files;
- all `phase_offset_matched_adapter.*` files;
- PhaseOffset allocator/tube production;
- planner, SPH, AgentState, NeighborManager, SO3-controller, and swarm
  production.

SIM_A_SINGLE_UAV_ALGORITHM_PRODUCTION_CHANGE=false
GVF_PRODUCTION_CHANGE_REQUIRED=false
PHASE_OFFSET_SCOPE_LEAK=false
SPH_PRODUCTION_CHANGE_REQUIRED=false

## 7. Exact SIM-A test and fixture whitelist

The following simulator-local paths are authorized:

1. `src/uav_simulator/so3_quadrotor_simulator/CMakeLists.txt` — only to
   register the new generic-N rostest, if registration is needed.
2. `src/uav_simulator/so3_quadrotor_simulator/test/sim_a_generic_n.test` —
   new generic-N rostest launch.
3. `src/uav_simulator/so3_quadrotor_simulator/test/sim_a_generic_n_test.py` —
   new generic-N runtime and configuration-validation test script.
4. `src/uav_simulator/so3_quadrotor_simulator/config/initial_states_8.yaml` —
   simulator-local N=8 fixture, if a checked-in fixture is needed.

N50_TEST_WHITELIST_ADDITION=NONE. The authorized
`sim_a_generic_n_test.py` must construct the deterministic 50-state input
programmatically; no 50-agent XML expansion or additional fixture/helper path
is required.

The existing files below are regression references and should be reused
unchanged:

- `src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_equivalence.test`
- `src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_three_agent.test`
- `src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_test.py`

No other path may be changed. In particular, do not modify existing launch
files solely to address `command_timeout`, and do not modify unrelated tests
for formatting or convenience.

## 8. Required acceptance tests

### 8.1 N=1 regression

Run the existing one-agent equivalence test. The multi simulator must retain
the accepted single-plant trajectory, odometry, IMU, rates, and topic graph.

### 8.2 N=3 regression

Run the existing three-agent regression unchanged. It must retain hover state,
finite odometry/IMU, one-process topology, per-agent topics, and command and
disturbance isolation.

### 8.3 N=7 generic-N smoke

Using the same executable and generic implementation, launch N=7 from a
state fixture containing required fields and optional metadata. Require:

- exactly one simulator process and simulator node;
- seven independent plants;
- simulator-owned `/uav_0` through `/uav_6` odom/IMU publications and
  command/force/moment subscriptions;
- no simulator-owned `/uav_7` plant interface;
- finite state and expected odom/IMU publication for all seven agents;
- no source or XML per-agent block copied for this test.

### 8.4 N=8 ceiling break

Using the same executable, same launch, and same implementation, change only
the input/configuration to launch N=8. Require one process, eight independent
plants, simulator-owned interfaces for `/uav_0` through `/uav_7`, and no
simulator-owned `/uav_8` interface.

The test must demonstrate that changing N=7 to N=8 requires no C++ source
edit, no new executable, and no manually copied XML UAV block.

### 8.5 N=8 command isolation

Before the isolation window, for every UAV 0 through 7, wait for at least
250 odometry samples and 250 IMU samples, with a 15.0 s timeout. Require an
initial hover observation interval of at least 3.0 s. During that interval,
all odometry and IMU values must be finite, each odometry stream must be
between 80.0 and 130.0 Hz, initial-position error relative to the configured
state must be less than 0.020 m, and yaw error relative to the configured yaw
must be less than 0.050 rad.

Publish a command only to `/uav_0/so3_cmd`; no command publisher or message
may target `/uav_1` through `/uav_7` during the isolation window. Use this
exact payload, matching the accepted B0 `tilt_command()` semantics:

- pitch = `+0.15` rad;
- `force.x = 0`, `force.y = 0`,
  `force.z = HOVER_FORCE / cos(0.15)`;
- `orientation.x = 0`, `orientation.y = sin(0.15 / 2)`,
  `orientation.z = 0`, `orientation.w = cos(0.15 / 2)`;
- `kR = [3.0, 3.0, 3.0]` and `kOm = [0.1, 0.1, 0.1]`;
- `aux.enable_motors = true`;
- all remaining fields retain the neutral/default semantics used by the B0
  helper.

Publish at 100.0 Hz for 4.0 s. Immediately before the command window,
capture `p_i_before` as the latest odometry position for each UAV. Immediately
after it, capture `p_i_after` and define
`d_i = ||p_i_after - p_i_before||`. Require `d_0 > 0.10 m` and
`d_i < 0.020 m` for every i from 1 through 7.

For every agent during the command phase, require odometry sample count to
increase, the latest header timestamp to advance, all new odometry to be
finite, and command-phase odometry rate to be at least 80.0 Hz. These
thresholds intentionally inherit the accepted B0 N=3 isolation contract.
An optional post-measurement hover command to UAV0 may be used only for test
cleanup/stabilization and must not affect isolation metrics.

The test must also inspect the simulator node's owned ROS master endpoints:
each command subscriber must be its own `/uav_i/so3_cmd`, and the expected
simulator-owned endpoint set must contain exactly five endpoints per agent.
Unrelated ROS infrastructure with similar names must not cause a failure.

### 8.6 Invalid configuration

The generic-N test must verify nonzero startup failure and absence of a
simulator-owned plant interface for each of these invalid cases:

- missing `initial_states`;
- non-array `initial_states`;
- empty array;
- present `num_agents` with a non-integer XML-RPC type. Exercise string
  `"8"`, double `8.0`, and boolean `true`; startup must fail without
  coercion;
- provided `num_agents` that does not equal N;
- duplicate, negative, out-of-range, or non-contiguous IDs;
- missing required field;
- wrong `robot_id` type;
- nonnumeric or non-finite `x`, `y`, `z`, or `yaw`;
- `z <= 0`.

The same test must also verify the positive compatibility case: with a valid,
nonempty N=8 `initial_states` array and no private `num_agents` parameter,
startup succeeds, N equals `initial_states.size()`, and exactly IDs 0 through
7 are instantiated. Together with the integer-mismatch rejection, this proves
that `num_agents` is only an optional compatibility check.

### 8.7 Optional metadata

Run two logically identical N=8 configurations in independent launches:

- PLAIN: required fields only;
- METADATA: identical required fields plus inert metadata such as
  `name: "uav_i"`.

No required field may differ between the variants. Both launches must pass the
N=8 startup, endpoint-topology, finite-state, 80.0..130.0 Hz baseline,
initial-position (<0.020 m), yaw (<0.050 rad), and complete N=8
command-isolation contracts above.

Additionally require these direct comparisons between the two runs:

- simulator-owned publication/subscription endpoint sets are exactly equal;
- instantiated robot-ID sets are exactly equal to `{0,1,2,3,4,5,6,7}`;
- for every UAV i, baseline position difference is <=0.020 m;
- for every UAV i, baseline yaw difference is <=0.050 rad;
- command-isolation classification is identical: UAV0 commanded-response PASS
  in both, and UAV1 through UAV7 isolation PASS in both.

Do not require bit-exact floating-point trajectory equality across separate
launches. Metadata must not alter N, IDs, topic names, numeric state,
command routing, or simulator-owned endpoints, and must not become a ROS
plant endpoint.

### 8.8 N=50 physical scalability smoke

The physical simulator must support at least 50 UAV plants. Using the same
`multi_quadrotor_simulator_so3` executable, generic implementation, and
generic state parser, run N=50 with robot IDs exactly 0 through 49. Changing
N=8 to N=50 must require no production-source edit, new executable, or
manually expanded 50-agent launch XML.

The existing authorized generic-N test script must construct 50 initial
states programmatically. Use deterministic unique positions, positive z, and
finite yaw so each ID-to-state mapping can be checked independently; for
example, a 10-by-5 grid with 2.0 m spacing, `z=1.0`, and `yaw=0.0`.

Configure and report `simulation_rate=1000.0 Hz` and `odom_rate=100.0 Hz`,
matching the existing simulator regression convention. Allow at most 30.0 s
for all 50 odometry and IMU streams to become live. At readiness require:

- exactly one `multi_quadrotor_simulator_so3` process and simulator node;
- exactly 50 instantiated physical plants with IDs `{0,...,49}`;
- each odometry `child_frame_id` equals `uav_i/base_link` and each initial
  position matches its configured state within 0.020 m;
- simulator-owned publications are exactly the 100 names
  `/uav_i/sim/odom` and `/uav_i/sim/imu` for i=0..49;
- simulator-owned subscriptions are exactly the 150 names
  `/uav_i/so3_cmd`, `/uav_i/force_disturbance`, and
  `/uav_i/moment_disturbance` for i=0..49;
- the union is exactly 250 UAV-scoped simulator-owned endpoint names;
- no simulator-owned publication or subscription exists under `/uav_50`.

After all streams are live, observe continuously for at least 5.0 s. For
every UAV 0 through 49, both odometry and IMU sample counts must increase,
their latest header timestamps must advance, and every new odometry and IMU
sample must remain finite. The simulator process must remain alive; the exact
50-plant endpoint sets must still be present at the end of the window; no
plant may disappear and no `/uav_50` plant interface may appear.

For functional liveness, calculate each agent's odometry rate from advancing
message header timestamps over the observation window and apply the already
accepted/source-backed lower threshold of 80.0 Hz (and the existing 130.0 Hz
upper sanity bound) rather than inventing a new scalability threshold. Report
the actual per-agent minimum, median, and maximum odometry rates, configured
simulation and odometry rates, process survival, and measured wall-clock
duration. Also report CPU usage and RSS/memory when obtainable from standard
process or `/proc` information without production dependencies; if not
obtainable, report them as unavailable with the reason. CPU/RSS have no SIM-A
pass/fail threshold.

This is a physical scalability/liveness smoke only. Do not duplicate the full
N=8 command-isolation experiment across 50 agents; Section 8.5 remains the
detailed command-routing proof.

## 9. Hash binding rule

The pre-audit digest is deliberately self-excluded and reproducible. After
all semantic edits are complete, set the entire line exactly to

`SIM_A_SPEC_PRE_AUDIT_SHA256=<SELF_EXCLUDED>`

and compute SHA-256 over the exact UTF-8/LF bytes of that document. Replace
`<SELF_EXCLUDED>` with the resulting digest. A reviewer reproduces the digest
by replacing the populated pre-audit line with the exact self-excluded line
above and recomputing SHA-256; the values must match.

`SIM_A_SPEC_BOUND_FILE_SHA256` is the ordinary SHA-256 of the final bound file
containing the populated pre-audit digest. It is reported separately in the
handoff and is not substituted into the canonical pre-audit calculation, so
the bookkeeping is not recursively self-referential.

## 10. Verification and stop gates

Implementation, when separately authorized, must perform a simulator-local
build and the exact tests in Section 8. Main must independently inspect the
actual diff, changed paths, `git diff --check`, process topology, endpoint
ownership, all N=1/N=3/N=7/N=8/N=50 results, invalid-config results, metadata
results, and the N=50 performance evidence.

Main must prove that no fixed ceiling remains, one process owns N independent
plants including the required N=50 case, command isolation holds, and no
SIM-B/C/D or single-UAV production change occurred.

Stop immediately on baseline drift, worktree/branch conflict, whitelist
expansion, header requirement, algorithm/planner/GVF/PhaseOffset/SPH change,
command-timeout dependency, build/test failure, ROS resource collision, or
any failed acceptance gate. Do not auto-repair outside this specification.

## 11. Worktree and branch plan

Planned implementation worktree:

`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws_multi_sim`

Planned branch:

`research/generic-n-sim`

The path and branch were absent in the read-only audit. Immediately before
future creation, re-check path absence, branch absence in local and fetched
remotes, and resolvability of the authoritative baseline commit. Create the
worktree directly from `7cf216a61891b88d2400e57ab81ea110eaad5b01`; do not copy
or overlay another worktree.

## 12. Authorization boundary and audit sequence

This document does not authorize Luna or implementation.

After this document is written, one fresh read-only Sol Max reviewer must
independently inspect the authoritative baseline and this specification and
return exactly one of `SPEC_AUDIT_PASS`, `SPEC_AUDIT_MAJOR`, or
`SPEC_AUDIT_BLOCKER`.

Only `SPEC_AUDIT_PASS` permits Main to update audit bookkeeping fields in this
document. It does not authorize worktree creation, Luna, implementation,
SIM-B, commit, or push.

If the audit passes, Main may set:

`DOCUMENT_STATUS=FROZEN_SPEC_AUDITED_PASS`

and record the audit result plus the post-bookkeeping SHA-256. If it does not
pass, report the exact finding and stop without changing the specification.

Required Sol review fields:

- baseline and two-launch architecture correctness;
- confirmation of current 1/3 limit, vector-N plants, and loop-based topics;
- canonical-N, `num_agents`, and optional-metadata contracts;
- minimal production and test whitelists;
- sufficiency of N=1, N=3, N=7, N=8, isolation, invalid-config, and metadata
  tests;
- sufficiency of the N=50 one-process/50-plant topology, liveness window,
  finite-state, `/uav_50` exclusion, programmatic generation, and performance
  reporting gates;
- no fixed ceiling and preserved independent-plant semantics;
- correct deferral of `command_timeout`;
- no GVF, PhaseOffset, SPH, or SIM-B scope leak;
- clear Luna boundary, Main acceptance gate, and fresh-result gate.

SIM_A_AUTHORIZED=false
AUTO_ADVANCE=false
