# PhaseOffsetSwarm Swarm Intent V1 Execution Specification

Date: 2026-08-20

Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

Authorized stage: isolated B2/B3 nominal distributed swarm-intent layer, followed by a ROS1 shadow wrapper that does not connect to the control chain.

Execution owner: one `gpt-5.6-luna` agent at `max` reasoning effort, supervised by the root agent. The execution owner must not spawn, delegate to, request, or wait for any sub-agent or other model.

`AUTO_ADVANCE=false`: stop after the deliverables, tests, and self-audit in this specification. Do not begin the phase-offset allocator, CBF, matched-port integration, multi-UAV controller activation, scenario expansion, or paper experiment stage.

## 1. Authorization and precedence

The user explicitly authorizes this isolated Swarm Intent stage before production B0-B1 multi-instance simulation is complete. This authorization overrides only the B0-B1-before-B2 ordering in `AGENTS.md`; every other repository rule and module boundary remains in force.

The stage must remain independent of concurrent Tube V2 work. Existing changes are user-owned and must not be overwritten, reformatted, staged, reverted, moved, or cleaned.

The execution owner must work serially inside its own whitelist. Before any Catkin build, check for another build process targeting this workspace; if one is active, report to the root supervisor and wait for coordination instead of racing writes into the shared `build/` or `devel/` trees. Never repair a Tube V2 or unrelated-package failure as part of this stage.

Before editing, record:

- current branch and `HEAD`;
- `git status --short`;
- tracked diff summary;
- whether either authorized package directory already exists.

Expected baseline at specification time:

- branch: `main`;
- `HEAD`: `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- dirty worktree with concurrent Tube and single-agent work.

If branch or `HEAD` differs, stop and report. Ordinary concurrent edits outside the whitelist are expected; do not treat them as authorization to touch those files.

## 2. Stage objective

Implement a standalone, unit-testable nominal swarm-intent layer:

\[
(id_j,t_j,x_j,v_j),\ \beta_i
\longrightarrow
g_i^{\mathrm{coord}},
\]

with

\[
g_i^{\mathrm{coord}}
=
g_i^{\mathrm{sep}}
+\beta_i g_i^{\mathrm{coh},0}
+g_i^{\mathrm{conf}},
\]

followed by exactly one vector-norm saturation:

\[
g_i^{\mathrm{coord}}
\leftarrow
\operatorname{sat}_{g_{\max}}
\left(g_i^{\mathrm{coord}}\right).
\]

The output is a two-dimensional, world-frame, velocity-like nominal intent. It is not a force and must not be sent to the vehicle, governor, SO3 controller, ISF-GVF output, matched port, or phase-offset runtime in this stage.

## 3. Explicit non-goals

Do not implement or modify:

- Tube builder, cross-section, filter, validator, viability, epoch, runtime, or Path-Tube handoff;
- `phase_offset_core` or `phase_offset_navigation`;
- `gvf_manager.cpp`, `gvf_manager.h`, or any existing `bspline_traj` production integration;
- phase-offset allocator, QP, CBF, robust safety filter, matched port, emergency controller, phase reversal, or post-QP logic;
- branch events, ETA, priority, conflict-region lease, or any low-frequency coordination protocol;
- LOS/map queries, SDF/ESDF dependencies, obstacle insertion, teammate point-cloud filtering, or simulator changes;
- comparison or communication of neighbor phase, offset, Tube, beta, intent, or control solution;
- full velocity consensus or unconditional radial velocity damping;
- direct publication of a final UAV command;
- proposal edits, parameter tuning of existing packages, new scenarios, maps, or paper statistics.

### 3.1 No extra gating

Do not invent runtime gates beyond the exact contracts in this specification.
The only authorized selection/state mechanisms are input validity and timestamp
ordering, the frozen FRESH/STALE/LOST classification, and the specified distance
hysteresis for neighbor-set membership. The pure C++ test hard gate is an
implementation-stage ordering rule, not a runtime controller gate.

In particular, do not add feature flags, enable/disable switches, modes, state
machines, eligibility gates, secondary acceptance filters, confidence gates,
fallback gates, or hidden test-only bypasses. `beta_preview` only clamps to
`[0,1]` and scales cohesion; it must not gate separation, conflict friction, the
complete output, or message processing. Safety diagnostics remain diagnostics
and must not gate nominal intent.

## 4. Authorized file whitelist

The execution owner may create or modify only:

```text
src/swarm_planner/phase_offset/phase_offset_msgs/**
src/swarm_planner/phase_offset/phase_offset_swarm/**
docs/Codex_PhaseOffsetSwarm_Swarm_Intent_V1_Self_Audit_2026-08-20.md
```

This execution specification is owned by the root agent and must not be edited by the execution owner.

No existing production file is authorized. In particular, do not modify or connect the historical reference-only files under:

```text
src/swarm_planner/bspline_traj/include/bspline_race/swarm_neighbor_model.h
src/swarm_planner/bspline_traj/src/swarm_neighbor_model.cpp
src/swarm_planner/bspline_traj/test/swarm_neighbor_model_test.cpp
src/swarm_planner/common_msgs/**
```

Catkin package discovery must be used; do not edit a workspace root CMake file or `bspline_traj/CMakeLists.txt`.

## 5. Target package layout

Create these two packages under `src/swarm_planner/phase_offset/`.

```text
phase_offset_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── AgentState.msg

phase_offset_swarm/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/phase_offset_swarm/
│   ├── agent_state.h
│   ├── neighbor_state.h
│   ├── swarm_parameters.h
│   ├── swarm_output.h
│   ├── neighbor_manager.h
│   ├── pair_geometry.h
│   ├── elastic_interaction.h
│   ├── conflict_friction.h
│   └── swarm_intent.h
├── src/
│   ├── neighbor_manager.cpp
│   ├── elastic_interaction.cpp
│   ├── conflict_friction.cpp
│   ├── swarm_intent.cpp
│   └── swarm_intent_shadow_node.cpp
├── config/
│   └── swarm_intent_v1.yaml
├── launch/
│   └── swarm_intent_shadow.launch
└── test/
    ├── neighbor_manager_test.cpp
    ├── elastic_interaction_test.cpp
    ├── conflict_friction_test.cpp
    ├── swarm_intent_test.cpp
    ├── swarm_intent_shadow.test
    └── swarm_intent_shadow_test.py
```

One header must represent one concept. Do not introduce a mixed global types header. Keep algorithm `.cpp` files focused and below approximately 500 lines.

## 6. Dependency boundary

### 6.1 Core library

Create target:

```text
phase_offset_swarm_core
```

The following files form the pure C++14/Eigen core:

- all public headers except no ROS-specific header is permitted;
- `neighbor_manager.cpp`;
- `elastic_interaction.cpp`;
- `conflict_friction.cpp`;
- `swarm_intent.cpp`.

The core must contain no include or dependency on:

- ROS, ROS time, messages, topics, tf;
- `phase_offset_core` or `phase_offset_navigation`;
- `bspline_race`, `common_msgs`, `plan_env`, SDFMap;
- Tube, allocator, QP, CBF, matched port, `w`, or `delta`.

It may depend only on Eigen and the C++ standard library.

`pair_geometry.h` is an authorized ROS-free shared mathematical helper so that
distance, deterministic zero-distance direction, distance rate, closing speed,
and TTC are computed once and reused consistently by Neighbor Manager, elastic
interaction diagnostics, conflict friction, and aggregate diagnostics. It must
not contain selection logic, feature gates, modes, or ROS concerns.

### 6.2 ROS wrapper

The ROS node may depend on:

- `roscpp`;
- `nav_msgs`;
- `geometry_msgs`;
- `std_msgs`;
- `phase_offset_msgs`;
- the local `phase_offset_swarm_core` target.

ROS callbacks only convert data and call the core interfaces. They must not contain the swarm formulas.

## 7. Minimal high-frequency communication

`phase_offset_msgs/msg/AgentState.msg` must contain only:

```text
std_msgs/Header header
uint16 robot_id
geometry_msgs/Point position_world
geometry_msgs/Vector3 velocity_world
```

For the algorithm, the four information categories are exactly:

```text
robot_id
timestamp
position_world
velocity_world
```

Do not add session, custom sequence, control mode, phase, offset, Tube, beta, intent, QP result, ETA, branch, or priority fields. Standard `Header` infrastructure is allowed; only `header.stamp` and `header.frame_id` are consumed by this stage.

The algorithm assumes synchronized clocks and a pre-established common world frame. It does not solve clock synchronization or frame registration.

## 8. Core public data contracts

Use `Eigen::Vector2d`; this V1 is the fixed-altitude two-dimensional coordination layer. Add `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` and aligned vector aliases where required by C++14 Eigen alignment.

### 8.1 Agent state

`agent_state.h` must define the semantic equivalent of:

```cpp
struct SwarmAgentState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int id = -1;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  double stamp = 0.0;
};
```

Reject non-finite states and invalid IDs at the cache boundary.

### 8.2 Neighbor state and snapshot

`neighbor_state.h` must define:

```cpp
enum class NeighborFreshness {
  FRESH,
  STALE,
  LOST
};

struct NeighborState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int id = -1;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d predicted_position = Eigen::Vector2d::Zero();
  double source_stamp = 0.0;
  double message_age = 0.0;
  NeighborFreshness freshness = NeighborFreshness::LOST;
};
```

Also define an aligned `NeighborStateVector` and a `NeighborSnapshot` holding:

- all retained neighbors;
- organization neighbors;
- conflict neighbors;
- safety-diagnostic neighbors;
- fresh, stale, and lost counts.

Only FRESH neighbors may enter organization, conflict, or safety-diagnostic sets in V1. STALE and LOST states remain available only in `all` and counters until retention expiry.

### 8.3 Parameters

`swarm_parameters.h` must centralize every parameter and provide explicit validation. No formula file may contain unexplained physical magic numbers.

Required parameters and units:

| Parameter | Unit | Contract |
|---|---:|---|
| `d_safe` | m | nominal safety-distance reference used by TTC and kernels |
| `d_minus` | m | separation becomes zero at and above this value |
| `d_plus` | m | weak cohesion is zero at and below this value |
| `r_comm` | m | organization support outer radius |
| `r_conf` | m | conflict distance activation outer radius |
| `r_safe` | m | safety-neighbor diagnostic radius |
| `k_sep` | m/s | maximum per-pair separation intent magnitude |
| `k_coh` | m/s | maximum per-pair weak-cohesion intent magnitude |
| `k_conf` | dimensionless | radial closing-friction gain |
| `ttc_activation` | s | TTC activation horizon and safety diagnostic threshold |
| `closing_speed_activation` | m/s | closing-speed activation scale |
| `fresh_timeout` | s | maximum FRESH message age |
| `stale_timeout` | s | maximum STALE message age |
| `lost_retention_timeout` | s | maximum retained LOST message age |
| `g_max` | m/s | final total intent norm limit |
| `distance_epsilon` | m | zero-distance numerical threshold |
| `ttc_epsilon` | m/s | TTC denominator regularizer |
| `neighbor_hysteresis` | m | distance-set enter/exit hysteresis width |
| `future_stamp_tolerance` | s | accepted small future timestamp tolerance |

Validation must require at least:

\[
0<d_{safe}<d_-<d_+<r_{comm},
\]

\[
d_{safe}<r_{conf},\qquad d_{safe}<r_{safe},
\]

\[
0<fresh\_timeout<stale\_timeout<lost\_retention\_timeout,
\]

all gains, radii, activation scales, limits, and epsilons finite and non-negative where appropriate, with strictly positive divisors and `g_max`.

Default configuration must satisfy `k_coh < k_sep`.

### 8.4 Output

`swarm_output.h` must define the semantic equivalent of:

```cpp
struct SwarmOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d g_coord = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_sep = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_coh = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_conf = Eigen::Vector2d::Zero();
  double min_distance = std::numeric_limits<double>::infinity();
  double min_ttc = std::numeric_limits<double>::infinity();
  int num_org_neighbors = 0;
  int num_conflict_neighbors = 0;
  int num_safety_neighbors = 0;
  int num_fresh_neighbors = 0;
  int num_stale_neighbors = 0;
  int num_lost_neighbors = 0;
  bool output_saturated = false;
};
```

The three component fields are pre-total-saturation components. Only `g_coord` is the final saturated sum.

## 9. Neighbor Manager contract

Implement a stateful `NeighborManager`; do not force freshness and hysteresis into a stateless function.

Required interface semantics:

```cpp
class NeighborManager {
public:
  NeighborManager(int self_id, const SwarmParameters& parameters);
  bool update(const SwarmAgentState& received);
  NeighborSnapshot snapshot(
      const SwarmAgentState& self,
      double now);
};
```

Exact behavior:

1. `update` rejects the configured self ID, invalid IDs, non-finite state values, and timestamps older than or equal to the cached timestamp for the same ID.
2. For each retained neighbor at `snapshot` time:

   \[
   \tau_{ij}=t-t_j.
   \]

   If `tau < -future_stamp_tolerance`, exclude the state from selected sets and report it as LOST for this snapshot. Otherwise use:

   \[
   message\_age=\max(0,\tau_{ij}),
   \]

   \[
   \hat x_j=x_j(t_j)+v_j(t_j)message\_age.
   \]

3. Freshness is:

```text
FRESH: age <= fresh_timeout
STALE: fresh_timeout < age <= stale_timeout
LOST:  stale_timeout < age <= lost_retention_timeout
```

4. Remove cached entries only after `age > lost_retention_timeout`.
5. All distance, distance-rate, TTC, neighbor-set, and intent calculations use `predicted_position`, not the raw source position.
6. Filter self ID independently of topic behavior.
7. Organization selection requires FRESH and distance membership around `r_comm` with hysteresis:

```text
enter when d <= r_comm - neighbor_hysteresis
remain while d <= r_comm
exit when d > r_comm
```

8. Conflict selection requires FRESH, positive closing speed above numerical epsilon, and either distance activation or TTC activation. Use hysteresis only for the distance part:

```text
distance enter when d <= r_conf - neighbor_hysteresis
distance remain while d <= r_conf
```

   A small TTC may independently select the neighbor even outside `r_conf`.
9. Safety diagnostics require FRESH and either `d <= r_safe` or `TTC <= ttc_activation`; they do not create a controller or safety command.
10. STALE and LOST neighbors do not produce separation, cohesion, or friction in V1. They are not silently relabeled as fresh and remain counted until retention expiry.

Freshness is age-monotone between packet arrivals, so no additional freshness hysteresis is required. Packet arrival may legitimately transition a state back to FRESH.

## 10. Pair geometry and zero-distance rule

For non-degenerate pairs:

\[
d_{ij}=\|\hat x_j-x_i\|,
\qquad
n_{ij}=\frac{\hat x_j-x_i}{d_{ij}}.
\]

For `d <= distance_epsilon`, do not divide by distance. Use a deterministic ID-based axis:

```text
self.id < neighbor.id: n_ij = (+1, 0)
self.id > neighbor.id: n_ij = (-1, 0)
```

Self IDs are filtered, so equality is invalid. This rule prevents NaN/Inf and preserves anti-symmetric pair directions in the coincident-position test.

Distance rate and closing speed are:

\[
\dot d_{ij}=(v_j-v_i)^Tn_{ij},
\]

\[
c_{ij}=\max(0,-\dot d_{ij}).
\]

TTC is:

\[
TTC_{ij}=
\begin{cases}
\infty,&c_{ij}\le ttc\_epsilon,\\
\dfrac{\max(0,d_{ij}-d_{safe})}{c_{ij}+ttc\_epsilon},&c_{ij}>ttc\_epsilon.
\end{cases}
\]

Any non-finite intermediate result must fail closed to zero nominal contribution and remain observable through test failure or diagnostics; never emit NaN/Inf.

## 11. Smooth kernels

Use these exact kernels unless a compile-level issue forces an equivalent algebraic form.

Define:

\[
H(s)=6s^5-15s^4+10s^3,
\qquad s\in[0,1],
\]

with the argument clamped to `[0,1]`.

### 11.1 Separation

\[
s_r(d)=
\operatorname{clamp}
\left(
\frac{d_- - d}{d_- - d_{safe}},0,1
\right),
\]

\[
\psi_r(d)=k_{sep}H(s_r(d)),
\]

\[
g_i^{sep}
=-
\sum_{j\in\mathcal N_i^{org}}
\psi_r(d_{ij})n_{ij}.
\]

Properties:

- maximum bounded magnitude `k_sep` per pair for `d <= d_safe`;
- C2 transition to zero at `d_minus`;
- zero in the comfort band and outside it;
- never multiplied by `beta_preview`.

### 11.2 Weak cohesion

For

\[
s_c(d)=\frac{d-d_+}{r_{comm}-d_+},
\]

define the compact C2 bump:

\[
B(s)=64s^3(1-s)^3.
\]

Then:

\[
\psi_c(d)=
\begin{cases}
k_{coh}B(s_c(d)),&d_+<d<r_{comm},\\
0,&\text{otherwise},
\end{cases}
\]

\[
g_i^{coh,0}
=
\sum_{j\in\mathcal N_i^{org}}
\psi_c(d_{ij})n_{ij},
\]

\[
g_i^{coh}
=
\operatorname{clamp}(\beta_i,0,1)
g_i^{coh,0}.
\]

No neighbor beta is accepted or used. There is no fixed preferred distance or formation target.

### 11.3 Selective radial conflict friction

Define:

\[
\omega_d(d)=
H\left(
\operatorname{clamp}
\frac{r_{conf}-d}{r_{conf}-d_{safe}}
\right),
\]

\[
\omega_c(c)=
H\left(
\operatorname{clamp}
\frac{c}{closing\_speed\_activation}
\right),
\]

and, only for finite TTC,

\[
\omega_t(TTC)=
H\left(
\operatorname{clamp}
\frac{ttc\_activation-TTC}{ttc\_activation}
\right).
\]

For infinite TTC, `omega_t=0`.

Use:

\[
\omega_{ij}^{conf}
=
\omega_c(c_{ij})
\max\{\omega_d(d_{ij}),\omega_t(TTC_{ij})\}.
\]

The max avoids unintentionally requiring both spatial proximity and small TTC; conflict selection is defined by distance **or** TTC. Each activation is smooth on its own transition. The `max` is continuous but not differentiable where the two activations cross; C2 smoothness is required for the distance kernels, not for this logical OR selector. If a smooth OR is preferred, the only authorized replacement is:

\[
1-(1-\omega_d)(1-\omega_t).
\]

Compute:

\[
g_i^{conf}
=
k_{conf}
\sum_{j\in\mathcal N_i^{conf}}
\omega_{ij}^{conf}
\dot d_{ij}n_{ij}.
\]

Sanity contracts:

- approaching pair: `dot_d < 0`, so the output points away from the neighbor and opposes closing;
- separating pair: `c=0`, so `omega_c=0` and conflict friction is exactly zero;
- common translation: `v_i=v_j`, so `dot_d=0` and conflict friction is zero;
- do not replace this law with `sum(v_j-v_i)` or with radial damping active during separation.

## 12. Swarm intent aggregation

`SwarmIntentCalculator` must expose the semantic equivalent of:

```cpp
class SwarmIntentCalculator {
public:
  explicit SwarmIntentCalculator(const SwarmParameters& parameters);
  SwarmOutput compute(
      const SwarmAgentState& self,
      const NeighborSnapshot& neighbors,
      double beta_preview) const;
};
```

Implementation order:

1. compute and preserve `g_sep`;
2. compute unscaled cohesion, clamp local `beta_preview` to `[0,1]`, and preserve scaled `g_coh`;
3. compute and preserve `g_conf`;
4. form the raw sum;
5. if its norm exceeds `g_max`, scale the complete vector once to norm `g_max` and set `output_saturated=true`;
6. never separately saturate the three logged components;
7. populate neighbor counts from the snapshot;
8. compute `min_distance` over retained FRESH neighbors using predicted positions;
9. compute `min_ttc` over finite TTC values of retained FRESH closing neighbors; otherwise leave infinity.

The library must never emit `u_w`, `u_delta`, phase rate, offset rate, physical command, or a control-mode decision.

## 13. Required pure C++ tests

Register each test with `catkin_add_gtest` and link only `phase_offset_swarm_core` plus Eigen as needed.

### 13.1 Neighbor Manager tests

At minimum test:

1. constant-velocity prediction equals `position + velocity * age`;
2. self ID is rejected;
3. older/equal timestamp update is rejected;
4. FRESH/STALE/LOST thresholds and removal after retention;
5. organization enter/exit hysteresis;
6. stale does not enter organization/conflict/safety sets;
7. conflict selected by distance;
8. conflict selected by small TTC outside the distance radius;
9. safety diagnostic selection;
10. future timestamp beyond tolerance does not enter selected sets;
11. invalid/non-finite input is rejected.

### 13.2 Elastic interaction tests

At minimum test:

1. `d < d_minus`: separation points away from neighbor;
2. `d_minus <= d <= d_plus`: separation and cohesion are zero without conflict;
3. `d_plus < d < r_comm`: weak cohesion points toward neighbor;
4. `beta_preview=0`: cohesion is zero and separation is unchanged;
5. endpoints and nearby samples show continuous bounded behavior;
6. multiple neighbors aggregate rather than choosing only the nearest;
7. coincident positions produce finite anti-symmetric contributions.

### 13.3 Conflict friction tests

At minimum test:

1. approaching pair with small TTC gives nonzero friction opposing closing;
2. separating pair gives exactly/nearly zero friction;
3. common translation gives zero friction;
4. near-zero closing speed produces finite zero/negligible output;
5. `d <= d_safe` is finite and does not generate invalid TTC;
6. a far but rapidly approaching pair can activate through TTC;
7. symmetric pair evaluation gives opposite pair contributions.

### 13.4 Aggregate tests

At minimum reproduce the twelve user acceptance cases:

1. too close;
2. comfort band;
3. weak cohesion;
4. beta zero;
5. approaching pair;
6. separating pair;
7. common translation;
8. multiple neighbors;
9. stale message;
10. zero-distance robustness;
11. bounded total output;
12. pair symmetry sanity check.

Also test:

- `g_coord == g_sep + g_coh + g_conf` when unsaturated;
- exactly one total-vector saturation preserves the raw-sum direction;
- invalid parameter configurations are rejected;
- output fields are finite except the explicitly permitted infinity sentinels for unavailable `min_distance`/`min_ttc`.

Pure C++ tests are a hard gate. Do not start the ROS wrapper until the core package builds and all core tests pass.

## 14. ROS1 shadow wrapper

After the pure C++ gate passes, implement `swarm_intent_shadow_node.cpp`.

### 14.1 Node behavior

The node must:

1. subscribe to a configurable local `nav_msgs/Odometry` topic;
2. convert local x/y position and velocity to `SwarmAgentState`;
3. publish the local four-field `phase_offset_msgs/AgentState` on a configurable shared state topic;
4. subscribe to the same shared state topic and feed non-self messages to `NeighborManager`;
5. run `snapshot + compute` on a timer at configurable `update_rate_hz`;
6. publish these local shadow-only topics as `geometry_msgs/Vector3Stamped`, with z fixed to zero:

```text
g_coord
g_sep
g_coh
g_conf
```

7. log throttled neighbor counts, minimum distance/TTC, and saturation state;
8. publish no `PositionCommand`, velocity command, phase, offset, allocator result, or other control output;
9. make no call into `gvf_manager`, `phase_offset_navigation`, governor, or SO3;
10. compute nothing until valid local odometry is available.

### 14.2 ROS parameters

At minimum support private parameters:

```text
robot_id
expected_world_frame
local_odom_topic
state_topic
g_coord_topic
g_sep_topic
g_coh_topic
g_conf_topic
update_rate_hz
beta_preview
```

and every field in `SwarmParameters`.

Default `beta_preview=1.0`. Clamp it before use. This is a local node parameter, not a broadcast field. Do not subscribe to Tube V2 in this stage.

If `expected_world_frame` is non-empty, reject or warn-and-ignore odometry and AgentState messages whose non-empty frame differs. Do not implement tf transforms.

### 14.3 ROS wrapper test

Add a minimal `rostest`:

- start one shadow node for robot 1 with a private test namespace;
- publish valid local odometry;
- publish one neighbor `AgentState` on the configured shared state topic;
- assert that the node publishes `g_coord` and the component topics;
- choose an approaching/too-close setup and assert the output is finite and nonzero in the expected x direction;
- verify that the node does not advertise a `PositionCommand` or configured command topic;
- shut down only processes launched by the test.

Run the ROS shadow acceptance in an isolated test environment with a new private
ROS master, a new `ROS_HOME`, and a unique port. Record the chosen master URI and
verify that the master, test node, and port are gone afterward. Do not attach to,
reuse, stop, or inspect through any user-owned ROS master already running on the
machine.

If ROS runtime or `rostest` is unavailable, preserve the passing core implementation, stop, and report the wrapper-test blocker. Do not bypass the gate by attaching to a user-owned ROS master.

## 15. Build and test order

Use a sourced ROS Noetic environment. Prefer package-scoped commands and do not repair unrelated workspace failures.

Required sequence:

1. First create only the ROS-free `phase_offset_swarm_core` sources, public
   contracts, core-only Catkin/Eigen build declarations, and the four pure C++
   gtest targets. Do not create or compile `phase_offset_msgs`, the shadow node,
   launch/config, or rostest yet.
2. Configure/build the core and run every pure C++ test. Confirm zero failures;
   this is the hard gate.
3. Only after that gate passes, create `phase_offset_msgs`, add the final ROS
   dependencies to `phase_offset_swarm`, and implement the shadow node,
   launch/config, and rostest.
4. Build the message package and shadow node, then run the isolated ROS shadow
   wrapper test.
5. Re-run all `phase_offset_swarm` tests and report package-scoped results so
   unrelated historical test artifacts are not misreported as this stage's
   failures.
6. Run existing `phase_offset_core` tests as a dependency-boundary regression.
   If an unrelated concurrent-work failure appears, preserve it as evidence and
   do not edit outside the whitelist.

Suitable commands may include:

```bash
# Core gate, before ROS wrapper files/targets are added:
catkin_make --pkg phase_offset_swarm
catkin_make run_tests_phase_offset_swarm

# After the core gate:
catkin_make --pkg phase_offset_msgs phase_offset_swarm
catkin_make run_tests_phase_offset_swarm
catkin_test_results build/phase_offset_swarm
catkin_make run_tests_phase_offset_core
```

Adapt command details to the actual Catkin workspace, but do not switch build systems or perform a workspace-wide cleanup.

## 16. Required dependency and scope audits

Before completion, run searches proving the core contains none of:

```text
ros/
ros::
nav_msgs
geometry_msgs
phase_offset_msgs
common_msgs
SDFMap
phase_offset_navigation
Tube
CBF
u_w
u_delta
PositionCommand
```

The ROS node is allowed ROS/message strings but still must contain none of:

```text
phase_offset_navigation
gvf_manager
PositionCommand publisher
matched port
CBF
allocator
```

Also verify:

- no file outside the whitelist was modified by this stage;
- no historical prototype file was added to CMake;
- no neighbor beta or event field exists in `AgentState.msg`;
- no total-intent component is individually saturated;
- no full velocity-consensus expression is present;
- no runtime gate, feature flag, mode/state machine, secondary eligibility
  filter, or hidden bypass was added beyond the exact frozen selection contracts;
- `git diff --check` passes for authorized files.

## 17. Self-audit report

Create:

```text
docs/Codex_PhaseOffsetSwarm_Swarm_Intent_V1_Self_Audit_2026-08-20.md
```

It must record:

- start and end timestamps;
- branch and `HEAD`;
- initial and final `git status --short` summaries;
- exact files created/modified;
- build and test commands with pass/fail counts;
- mathematical sign checks for separation, cohesion, and conflict friction;
- confirmation of the four-field communication contract;
- confirmation that `beta_preview` is local only;
- dependency-boundary search results;
- no-extra-gating audit results;
- ROS shadow topics and proof that no control topic is published;
- private ROS master/`ROS_HOME` identity and post-test process/port cleanup evidence;
- any deviations or blockers;
- an explicit statement that the agent stopped before allocator/CBF/control integration.

## 18. Stop conditions

Stop without broadening scope if:

- branch or `HEAD` differs from the expected baseline;
- an authorized package path unexpectedly exists with non-stage user work that cannot be preserved;
- a required edit would touch a file outside the whitelist;
- core tests cannot pass without modifying Tube, single-UAV integration, simulator, or historical prototype code;
- wrapper acceptance requires allocator, CBF, matched port, or control-chain integration;
- ROS runtime is unavailable after the pure C++ stage passes;
- concurrent work modifies an authorized file and safe reconciliation is not possible.

Do not create a commit, branch, tag, stash, or push. Do not stage files. Do not spawn sub-agents. Report to the root agent when complete or blocked.
