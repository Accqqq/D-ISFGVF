# D-ISFGVF Post-Stage-1A Bounded-Tube / Preview / Batch-C Roadmap

```text
DOCUMENT_ROLE=IMPLEMENTATION_PLAN
DOCUMENT_STATUS=CANDIDATE / READY FOR SOL REVISION REVIEW
PREVIOUS_ACCEPTED_BASELINE=5355be736339a650432280deec53362f61749a43
STAGE1A_BASELINE=cd313b315fcf4157273a7b0de84622073b3afaf6
STAGE1A_ACCEPTANCE_STATUS=PUSHED_CANDIDATE
SOL_PLAN_AUDIT=PENDING_REVISION_REVIEW
IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
BOUNDED_TUBE_IMPLEMENTATION_AUTHORIZED=false
STAGE1B_AUTHORIZED=false
PREVIEW_IMPLEMENTATION_AUTHORIZED=false
BATCH_C_AUTHORIZED=false
BATCH_D_AUTHORIZED=false
```

This is one candidate plan only. It records a revised roadmap after the
pushed Stage 1A commit and is not an implementation authorization.

## 1. Authority boundary and source-of-truth hierarchy

This document is response/documentation work. It does not authorize Luna,
production edits, test edits, launch/config edits, staging, committing, or
pushing.

The priority order for future decisions is:

1. the latest paper supplied in the current planning window;
2. accepted/frozen production semantics;
3. the verified pushed Stage 1A code;
4. this candidate plan after Sol revision review;
5. only then, a separately authorized execution specification.

The latest uploaded paper is the method source of truth. Older detailed
proposals and Frozen Plan sections are retained as historical material only
where they do not conflict with the latest paper.

## 2. Exact repository baseline

The repository is:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

The previous accepted baseline was:

```text
5355be736339a650432280deec53362f61749a43
feat: freeze R3 and Horizontal-N accepted baseline
```

The verified local state after Stage 1A is:

```text
branch: main
local HEAD: cd313b315fcf4157273a7b0de84622073b3afaf6
origin/main: cd313b315fcf4157273a7b0de84622073b3afaf6
d_isfgvf/main: 5355be736339a650432280deec53362f61749a43
configured upstream: d_isfgvf/main
ahead/behind against configured upstream: 1/0
```

`origin/main` is the remote that contains the pushed Stage 1A commit. The
configured `d_isfgvf/main` remote still identifies the earlier accepted
Horizontal-N baseline. No pull, merge, reset, or remote mutation is implied.

## 3. Stage 1A pushed baseline and acceptance status

The exact Stage 1A commit is:

```text
cd313b315fcf4157273a7b0de84622073b3afaf6
feat: accept Stage 1A tube freshness scheduling
```

The commit is a direct child of `5355be736339a650432280deec53362f61749a43`
and changes exactly these four paths:

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
```

The commit adds a scheduler-only Tube timer entry point, a single joined
worker, one latest-only pending request slot, request/work identities,
job-local Tube state, lifecycle joining, stale-result gates, and test coverage
for reset/deactivate/shutdown, provenance, duplicate scheduling, pending
replacement, and publication races. `gvf_manager.cpp` wires the configured
`tube_update_period` and calls the scheduler instead of the old synchronous
timer path.

There is no independent Sol implementation-acceptance artifact in the
verified repository state. The honest status is therefore:

```text
STAGE1A_ACCEPTANCE_STATUS=PUSHED_CANDIDATE
```

Before P1 implementation, a separate Stage 1A implementation-acceptance gate
is required. No second Stage 1A implementation is planned.

## 4. Stage 1A responsibilities and protected semantics

Stage 1A is limited to:

- scheduling and freshness;
- one worker and one pending slot;
- no FIFO backlog;
- stale pending rejection;
- job-local transactional state;
- stale completion abandonment;
- task/source/path/frame provenance checks;
- reset/deactivate/shutdown lifecycle;
- joined worker ownership;
- actual `tube_update_period` wiring.

Stage 1A does not change:

- Tube geometry or `rho_nom`;
- `search_extent` semantics;
- `TubeCrossSectionSolver`;
- `TubeFilter` geometry;
- `TubeSurfaceValidator` proof;
- planner safe distance;
- Horizontal-N;
- Runtime, Recovery, Handoff, or authority semantics;
- R3 publication semantics;
- Preview, swarm coordination, QP, CBF, or Batch C/D.

Any P1 change touching the four Stage 1A paths is limited to nominal Tube
configuration/propagation or explicitly reviewed diagnostics. It must not
rewrite worker ownership, pending-slot behavior, currentness gates, or
lifecycle handling.

## 5. Accepted A/B/R3/Horizontal-N contracts

The following are not reopened:

- Batch A geometry foundation;
- Batch B recovery and handoff;
- transactional execution authority;
- persistent `w` and authoritative `delta` handling;
- exact-zero and tiny-nonzero behavior;
- Candidate / Active / Certified Geometry distinction;
- source, path, frame, map, and task currentness;
- full 3-D planner centerline;
- planner-owned altitude;
- `N(w)=normalize(e_z x p_w(w))`;
- `N_z=0`;
- `r(w,delta)=p(w)+N(w)delta`;
- `r_z=p_z`;
- `||r_w(w,delta)|| >= m_r`;
- matched reference/physical injection;
- original planner, governor, ISF-GVF, and low-level command chain.

Any future touch to an A/B-era file is a bounded post-acceptance amendment,
not a rewrite of Batch A or Batch B.

## 6. Latest-paper method alignment

The latest paper defines a 3-D local planner path, one scalar horizontal
offset, and a world-frame coordination intent. Its method is:

```text
geometric Path-Tube I(w)
  -> preview-feasible K_k and continuous K(s)
  -> beta from preview width
  -> g_sep + beta*g_coh + g_damp
  -> bounded g_des
  -> analytic u_w_nom and u_delta_nom
  -> scalar admissible-set projections
  -> final u_w, u_delta
  -> matched physical/reference injection
```

The latest paper does not define a joint QP as the final coordination
mechanism and explicitly describes the interaction as soft spacing
regulation. It does not support a claim that nominal interaction alone is a
formal pairwise collision-invariant CBF layer.

## 7. Revised remaining roadmap

The revised dependency order is:

```text
current code baseline
  = accepted R3 + accepted Horizontal-N + pushed Stage 1A candidate
        |
        v
Stage 1A implementation acceptance gate, if still unreviewed
        |
        v
P1 bounded nominal asymmetric Horizontal-N Tube
        |
        v
P1 safety and performance acceptance
        |
        v
Stage 1B cooperative cancellation, conditional only
        |
        v
P2 standalone Preview K(s)
        |
        v
P2 Preview acceptance
        |
        v
P3 final-paper NoQP Batch C production coordination
        |
        v
P4 Batch D cleanup, observability, and experiment support
```

Preview is not hidden inside Batch C. Batch C may not consume `K(s)` until
Preview has its own implementation and acceptance.

## 8. Bounded nominal asymmetric Tube specification

Define one configurable nominal horizontal half-width:

```text
rho_nom > 0
```

The requested open-space interval is:

```text
I_nom(w) = [-rho_nom, +rho_nom]
```

The final interval is:

```text
I(w) = nominal interval
       intersect obstacle-admissible interval
       intersect full-3D regularity interval
       intersect continuous-certification admissible interval
```

Required behavior:

```text
open space:              [-rho_nom, +rho_nom]
only -N obstacle:        [clipped_lower, +rho_nom]
only +N obstacle:        [-rho_nom, clipped_upper]
both sides constrained:  [clipped_lower, clipped_upper]
```

The two sides are independent. The implementation must not enforce
`lower(w)=-upper(w)` after clipping, must not maximize width, and must not
search beyond `rho_nom` to discover additional free space.

The nominal value must support practical experimental settings such as
`1.0 m` and `1.5 m`; neither value is a mathematical constant.

## 9. `rho_nom` parameter contract

The proposed authoritative owner is:

```text
phase_offset_navigation::TubeCrossSectionConfig::nominal_half_width
```

The proposed ROS parameter is:

```text
phase_offset/tube/nominal_half_width
```

The value must be finite and strictly positive.

`fixed_delta_max` remains a distinct legacy/fixed-source test parameter until
its migration is separately reviewed. `max_offset` and
`environment_search_extent` must not remain authoritative for the ESDF
bounded nominal path. During migration they may be retained only as explicit
deprecated compatibility fields, with diagnostics showing that the effective
ESDF bound is `nominal_half_width`.

The migration must not permit a hidden second nominal-width owner. If both a
new and legacy parameter are supplied, the new parameter is authoritative and
the conflict is diagnosed deterministically.

## 10. Independent `+N/-N` clipping contract

For a neutral zero-connected candidate, evaluate each side only over:

```text
delta in [0, +rho_nom]       for +N
delta in [-rho_nom, 0]       for -N
```

An occupied, unknown, unavailable, or out-of-map result terminates or denies
only the affected side according to the existing fail-closed status model;
it must not manufacture free space on the opposite side.

The side result must preserve:

- raw bound;
- termination reason;
- required planner clearance;
- map uncertainty accounting;
- path/frame/map provenance;
- regularity intersection.

For an active nonzero state, the candidate must preserve the connected safe
component containing the authoritative current delta, then intersect that
component with the bounded nominal interval. It must never cross an unsafe
gap merely to recover zero or to widen the candidate.

## 11. Current Builder versus proposed Builder flow

The current cloud-clearance path is:

```text
adaptive path sampling
  -> PathCellGeometryCertificate preflight
  -> TubeCrossSectionSolver
  -> TubeFilter
  -> full-width SurfaceValidator
  -> repeated inward candidate construction
  -> repeated SurfaceValidator calls
```

The proposed normal path is:

```text
adaptive path sampling where required
  -> bounded nominal interval
  -> independent side clipping
  -> regularity intersection
  -> local conservative PWL representation
  -> one continuous SurfaceValidator pass
  -> bounded certified Candidate
```

`CertifiedTubeBuilder` must not recreate the old
`BOTH_SIDED/POSITIVE_ONLY/NEGATIVE_ONLY` scale-halving families as the normal
path. If the one bounded candidate cannot be certified, the result is
fail-closed: retain the accepted planner zero baseline or reject the
candidate, with explicit failure provenance. It does not silently retry a
larger or differently scaled Tube.

## 12. Component disposition

| Component | Disposition | Future treatment |
|---|---|---|
| `TubeCrossSectionSolver` | `REUSE_WITH_BOUNDED_AMENDMENT` | Add nominal limit and neutral/active anchor behavior; retain directional clearance and regularity |
| `TubeBuilder` | `REUSE_WITH_BOUNDED_AMENDMENT` | Keep adaptive sampling, cloud query, certificates, and fail-closed handling; prohibit nominal-external search |
| `TubeFilter` | `REUSE_UNCHANGED` | Keep local PWL representation and no fixed-slope validity gate |
| `CertifiedTubeBuilder` | `SIMPLIFY` | Remove repeated inward families and perform one bounded certification |
| `TubeSurfaceValidator` | `REUSE_UNCHANGED` | Retain continuous cell/surface proof |
| `PathCellGeometryCertificate` | `REUSE_UNCHANGED` | Retain frame, path-variation, and regularity evidence |
| `TubeEpochManager` | `REUSE_UNCHANGED` | Preserve Candidate/Active/currentness/installation semantics |
| matched adapter Tube integration | `REUSE_WITH_BOUNDED_AMENDMENT` | Add only parameter propagation and bounded-Tube diagnostics |
| old allocator/CBF prototype | `REMOVE_FROM_PRODUCTION_PATH` | Do not reconnect it to final-paper production; audit for later Batch D cleanup |

## 13. Continuous-certification proof obligations

The pointwise interval at each sample is insufficient. The accepted proof must
continue to cover the complete swept cell between samples.

The future bounded candidate must prove:

1. path sample provenance and monotonic phase ordering;
2. independent side bounds at every retained knot;
3. conservative interpolation with no outward expansion;
4. path and normal variation over each cell;
5. full-3D regularity over the entire candidate interval;
6. obstacle clearance of the continuous swept surface;
7. unknown/unavailable/out-of-map fail-closed behavior;
8. current anchor and certified forward segment coverage.

`PathCellGeometryCertificate` can reduce redundant geometry/resolution erosion
when complete and revision-matched. It does not automatically replace the
obstacle clearance proof. `TubeSurfaceValidator` remains required unless Sol
accepts a separately proven equivalent validator.

## 14. Computation-cost and combined Stage 1A interpretation

The bounded Tube amendment is expected to eliminate:

- search outside the requested nominal domain;
- maximum-width expansion objective;
- three-family inward search;
- repeated scale-halving;
- repeated alternate candidate creation;
- repeated SurfaceValidator execution for those alternate candidates.

Stage 1A already addresses:

- ROS timer blocking by heavy work;
- latest-only freshness;
- stale completion publication;
- stale job mutation of committed state;
- reset/deactivate/shutdown ownership.

After both changes, the remaining possible costs are:

- adaptive path sampling;
- PathCellGeometryCertificate queries;
- bounded directional cloud/ESDF queries;
- one continuous SurfaceValidator pass;
- path/cell subdivision;
- normal publication and currentness checks.

The combined architecture is complementary:

```text
Stage 1A = scheduling/freshness/concurrency boundary
bounded Tube = one-build computation structure and bounded geometry
```

No final runtime claim is made until measurement.

## 15. Stage 1B conditional triggers

Stage 1B is not authorized by this document.

It may be proposed only after Stage 1A acceptance plus P1 measurement, and
only if meaningful stale in-flight waste remains, such as:

- build p95 or maximum materially exceeding the configured permit period;
- frequent stale in-flight jobs;
- expensive stale completions despite latest-only scheduling;
- adaptive or validation loops dominating wall time;
- cancellation demonstrably saving computation without weakening proof.

Every build outcome must distinguish:

```text
SUCCESS   current bounded geometry was certified
FAILED    current geometry genuinely failed certification
CANCELLED obsolete work stopped before making a geometry/safety conclusion
```

If P1 plus Stage 1A makes builds consistently short, the valid disposition is:

```text
STAGE1B_NOT_REQUIRED
```

## 16. Semantic distinction: `rho_nom`, Tube bounds, and current delta

These quantities are never interchangeable:

```text
rho_nom       configured requested nominal half-width
lower/upper   actual geometric Tube boundaries at a phase
delta_current current authoritative PhaseOffset state/reference
```

The invariant is:

```text
delta_current in [lower(w), upper(w)]
```

For example, `delta_current=+1.2 m` is a state value. It does not mean that
the new nominal boundary is `upper=+1.2 m`.

## 17. Authoritative nonzero delta transition audit

Consider:

```text
delta_current = +1.2 m
rho_nom = 1.0 m
I_nom = [-1.0, +1.0]
```

The system must not silently perform either:

```text
delta_current := +1.0
delta_current := 0.0
```

The existing mechanisms to inspect are:

- `CURRENT_DELTA_CONNECTED` selection;
- retained delta in Runtime/Recovery;
- `ActiveReferenceAuthority`;
- PathTubePair and epoch transition;
- Handoff state machine;
- transactional source/currentness checks.

The current plan does not invent a recovery architecture. The required Sol
classification is one of:

```text
EXISTING_RECOVERY_HANDOFF_SUFFICIENT
SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED
BOUNDED_TRANSITION_AMENDMENT_REQUIRED
NEW_RECOVERY_MECHANISM_REQUIRED
NEEDS_SOL_DECISION
```

Preferred initial hypothesis is `SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED` only
if current code evidence proves that the existing authority correctly denies
or transitions the case. Otherwise the plan must remain `NEEDS_SOL_DECISION`.

Any recovery-only extension, if later proven necessary, must stay within the
same continuously certified connected component, never cross an unsafe gap,
never widen general swarm freedom, never silently clamp, and preserve existing
Recovery/Handoff/ExecutionAuthority ownership.

## 18. Normal preview contraction versus exceptional transition

Normal operation is expected to handle contraction through Preview:

```text
future Tube narrows
  -> K(s) contracts early
  -> U_delta^K limits future transverse rate
  -> delta evolves before the narrow section
```

An authoritative current delta suddenly outside a newly built nominal Tube is
an exceptional event associated with replanning, source/path/frame replacement,
abrupt map change, or epoch transition. It must not become the normal Tube
contraction mechanism.

## 19. Preview `K(s)` as a standalone stage

P2 Preview owns dynamic forward feasibility, not geometric Tube construction.
Its pipeline is:

```text
I(w)
  -> approximately uniform spatial/arc-length samples
  -> I_k
  -> K_H = I_H
  -> K_k = I_k intersect (K_{k+1} Minkowski-sum R_delta)
  -> conservative continuous inner K(s)
  -> kappa_lower(s), kappa_upper(s)
  -> b_pre and beta
  -> U_delta^K
```

The current `PreviewFeasibility` implementation does not yet provide the full
backward recursion and continuous envelope. It must not be treated as already
paper-complete.

All Preview structures must accept:

```text
lower(w) != -upper(w)
```

No global-width or symmetric interval assumption is permitted.

## 20. Final-paper Batch C architecture

P3 Batch C uses the latest NoQP method:

```text
immutable timestamped 3-D neighbor snapshot
  -> world-frame pair geometry
  -> g_sep
  -> beta * g_coh
  -> g_damp
  -> bounded saturation
  -> centerline recovery -k_rec*delta*N
  -> g_des

u_w_nom     = r_w^T*g_des / ||r_w||^2
u_delta_nom = N^T*g_des

  -> scalar phase admissible set U_w
  -> project u_w_nom
  -> total phase rate and v_s
  -> Preview-derived U_delta^K
  -> project u_delta_nom
  -> final u_w, u_delta
  -> matched injection
```

The phase-offset layer remains the sole owner of authoritative `w`, `delta`,
`u_w`, and `u_delta`. Swarm code supplies intent, not final UAV commands.

## 21. Single-side Tube contraction with a neighbor in the contraction direction

This is a future Batch-C acceptance scenario, not a P1 feature:

1. UAV `i` is near one Tube side.
2. An obstacle contracts only that side.
3. Preview detects the contraction before arrival.
4. Neighbor `j` occupies the workspace direction into which transverse motion
   would otherwise move `i`.
5. Coordination must use phase/longitudinal deformation, transverse motion,
   or both before no-common-feasible-motion is reached.

The scenario must record:

```text
lower_i, upper_i, I_i(w), K_i(s)
kappa_lower, kappa_upper, b_pre, beta_i
delta_i, delta_j
||x_i-x_j|| in world frame
g_sep, g_coh, g_damp, g_des
u_w_nom, u_delta_nom, U_w, U_delta^K
final u_w, final u_delta
phase separation
minimum pairwise distance
minimum obstacle distance
```

Collision risk must use actual world-frame separation, not only
`abs(delta_i-delta_j)`.

## 22. SPH paper disposition

The SPH paper is reference material only.

Potentially useful analogies are:

```text
SPH repulsion       -> g_sep
weak cohesion       -> g_coh
viscosity/damping   -> g_damp
```

It does not authorize copying the SPH planner, environmental force,
controller, raycasting gate, or complete dynamical model. The latest D-ISFGVF
paper remains authoritative.

## 23. Soft spacing and hard-safety non-claim

The final-paper Batch C nominal interaction provides soft spacing regulation.
The plan must not claim formal pairwise collision invariance from:

```text
g_sep + beta*g_coh + g_damp
```

If a system-level hard-safety supervisor is required, its owner, inputs,
authority, and proof must be separately specified and authorized. It must not
be smuggled back in as the old joint-QP/CBF Batch C.

## 24. Revised Batch D

P4 Batch D is cleanup and observability after Batch C acceptance. It may
include, subject to dependency closure:

- removal or test-only quarantine of obsolete QP/CBF code;
- removal of obsolete 2-D swarm shadow assumptions;
- parameter migration from legacy width/search names;
- topic and namespace cleanup;
- experiment and bag-analysis interfaces;
- layered Tube/Preview/interaction/port diagnostics;
- removal of stale CBF residual/activity metrics as primary paper metrics.

Preferred diagnostics are:

```text
I(w), lower(w), upper(w), K(s), kappa_lower, kappa_upper
b_pre, beta, g_sep, g_coh, g_damp, g_des
u_w_nom, u_delta_nom, U_w, U_delta^K
u_w, u_delta, delta, dot_w, v_s
matched reference velocity
```

## 25. Exact P1 bounded-Tube whitelist against Stage 1A

### Production files

```text
MODIFY
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

Reasons:

- add and validate `nominal_half_width`;
- propagate the one authoritative parameter;
- bound directional cross-section queries;
- preserve full-3D regularity and fail-closed evidence;
- remove repeated inward-family construction;
- keep Stage 1A scheduler semantics untouched while exposing configuration.

### P1 test files

```text
MODIFY
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
```

The following are regression-only by default and may not be modified without
a new reason:

```text
REGRESSION_ONLY
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

### P1 configuration/launch

```text
MODIFY ONLY IF REQUIRED TO EXPOSE THE NEW PARAMETER
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

If NodeHandle configuration is sufficient, both launch files remain
`LEAVE_UNCHANGED`. No CMake or package change is expected for P1.

### Explicit P1 exclusions

```text
EXCLUDED
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_recovery_owner.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
all planner/governor sources
all Horizontal-N frame sources
phase_offset_allocator.*
phase_offset_cbf_constraints.*
AGENTS.md
all existing user-owned uncommitted/untracked files
```

## 26. Exact P2 Preview whitelist

```text
ADD
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_viability.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_viability.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_viability_test.cpp

MODIFY
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/preview_feasibility.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/preview_feasibility.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/preview_feasibility_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
```

The P2 whitelist owns only `I_k`, `K_k`, continuous `K(s)`, contraction-rate
facts, preview width, beta input, and asymmetric interval tests. It does not
own swarm interaction or final port integration.

## 27. Exact P3 final-paper Batch-C whitelist

The initial pure-core whitelist is:

```text
MODIFY
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/agent_state.h
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/neighbor_state.h
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/neighbor_manager.h
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/pair_geometry.h
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/elastic_interaction.h
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/conflict_friction.h
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/swarm_intent.h
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/swarm_output.h
src/swarm_planner/phase_offset/phase_offset_swarm/include/phase_offset_swarm/swarm_parameters.h
src/swarm_planner/phase_offset/phase_offset_swarm/src/neighbor_manager.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/src/elastic_interaction.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/src/conflict_friction.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/src/swarm_intent.cpp
corresponding phase_offset_swarm unit tests
```

Integration files require a separate P3 sub-authorization:

```text
MODIFY ONLY WHEN INTEGRATION IS FROZEN
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

`phase_offset_msgs/msg/AgentState.msg` is `LEAVE_UNCHANGED` unless a concrete
timestamp/frame dependency requires a message amendment.

P3 must not modify Stage 1A worker semantics, Tube construction, Runtime
authority ownership, planner, or governor as a side effect.

## 28. Exact P4 Batch-D audit whitelist

Batch D must begin with a dependency scan. The bounded audit/removal scope is:

```text
AUDIT / MODIFY / DELETE ONLY AFTER DEPENDENCY CLOSURE
src/swarm_planner/bspline_traj/include/bspline_race/phase_offset_allocator.h
src/swarm_planner/bspline_traj/src/phase_offset_allocator.cpp
src/swarm_planner/bspline_traj/test/phase_offset_allocator_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/phase_offset_cbf_constraints.h
src/swarm_planner/bspline_traj/src/phase_offset_cbf_constraints.cpp
src/swarm_planner/bspline_traj/test/phase_offset_cbf_constraints_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/swarm_neighbor_model.h
src/swarm_planner/bspline_traj/src/swarm_neighbor_model.cpp
src/swarm_planner/bspline_traj/test/swarm_neighbor_model_test.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/config/swarm_intent_v1.yaml
src/swarm_planner/phase_offset/phase_offset_swarm/launch/swarm_intent_shadow.launch
corresponding diagnostics and bag-analysis interfaces
```

No file is automatically deleted merely because it contains an obsolete term.
The final P4 delete list requires zero production references and passing
regressions.

## 29. Deterministic P1 test plan

P1 must cover:

1. open space at `rho_nom=1.0`: exactly `[-1.0,+1.0]` within a documented
   tolerance;
2. open space at `rho_nom=1.5`;
3. only `-N` obstacle clips the lower side;
4. only `+N` obstacle clips the upper side;
5. both-side obstacles clip independently;
6. no clearance query exceeds `rho_nom`;
7. regularity clips only the mathematically inadmissible side;
8. unknown evidence fails closed;
9. out-of-map evidence fails closed;
10. unavailable query fails closed;
11. sloped full-3D centerline preserves Horizontal-N and altitude;
12. continuous cell certification remains required;
13. no repeated inward-family/scale-halving path is executed;
14. normal bounded candidate has one final Validator pass;
15. current delta zero selects zero-connected behavior;
16. authoritative nonzero delta selects the current connected component;
17. disconnected geometry is never convexified across a gap;
18. repeated immutable input is deterministic;
19. path/source/frame replacement cannot publish stale geometry;
20. R3 Candidate/Active/Certified Geometry regressions remain green;
21. `rho_nom`, `lower`, `upper`, and `delta_current` remain distinct in
   diagnostics and assertions.

## 30. P2 Preview test plan

P2 must add deterministic tests for:

- asymmetric `I_k` intervals;
- exact backward recursion;
- empty `K_0` while geometric `I(w)` remains nonempty;
- continuous inner `K(s)` containment;
- contraction-rate bound;
- piecewise-C1 junctions and one-sided derivatives;
- preview width `b_pre` and beta saturation;
- no global intersection caused by a remote bottleneck;
- `U_delta^K` nonemptiness for accepted envelopes;
- current delta outside `K(s)`;
- path/frame/map revision mismatch;
- no assumption that `lower=-upper`.

## 31. P3 Batch-C acceptance scenario and tests

In addition to ordinary separation, cohesion, damping, freshness, and
bounded-output tests, P3 must include:

- the single-side Tube contraction with neighbor in contraction direction;
- comparison of longitudinal versus transverse deformation;
- actual world-frame pair distance rather than delta difference;
- stale neighbor handling;
- beta affecting cohesion only;
- separation remaining active at beta zero;
- no full velocity consensus requirement;
- matched injection with identical final port values on internal and physical
  channels;
- no hard pairwise CBF guarantee claim;
- 2-D/fixed-altitude versus 3-D world-frame dimensionality explicitly tested.

## 32. ROS and performance acceptance

After Stage 1A acceptance and P1 implementation, measure:

- Tube request/permit rate;
- worker build rate;
- build median, p90, p95, and maximum;
- request-to-publication age;
- Candidate publication rate;
- Certified Geometry publication rate;
- stale pre-build rejection count;
- stale in-flight completion count;
- stale abandonment count;
- SurfaceValidator invocation count;
- inward-search attempt count;
- ESDF/cloud query count;
- maximum queried transverse offset;
- worker busy ratio;
- `cmdCallback` timing;
- `traj_vis` rate;
- `local_map` rate;
- configured `tube_update_period`.

Structural expectations:

```text
normal bounded candidate inward-search attempts = 0
normal bounded candidate final validation passes = 1 target
maximum queried offset <= rho_nom
```

No unmeasured 1 ms guarantee is permitted. The ROS A/B sequence is:

```text
accepted baseline evidence
  -> Stage 1A baseline
  -> Stage 1A + bounded Tube
```

P1 must show no regression in local map, trajectory visualization, Runtime,
Recovery, R3, or Horizontal-N behavior.

## 33. Stage 1A acceptance gate

Before P1 implementation, the existing pushed Stage 1A candidate requires an
independent implementation review against:

- scheduler-only ROS timer;
- one joined worker;
- one in-flight build;
- one latest-only pending slot;
- no FIFO;
- stale pending rejection;
- job-local state isolation;
- stale completion abandonment;
- path/source/frame/task provenance;
- candidate resurrection race closure;
- reset/deactivate clearing obsolete work;
- safe shutdown join;
- configured timer-period wiring;
- no Tube geometry or safety change.

The acceptance result must be either:

```text
STAGE1A_ACCEPTED_BASELINE
```

or:

```text
PUSHED_STAGE1A_CANDIDATE_BASELINE_REQUIRES_REPAIR
```

No P1 work may silently repair Stage 1A at the same time.

## 34. Open decisions for Sol

Sol must decide:

1. whether the Stage 1A commit is accepted or remains a pushed candidate;
2. whether `origin/main` or the configured `d_isfgvf/main` should be named the
   authoritative remote baseline for the next execution window;
3. whether the existing Recovery/Handoff path is sufficient for a nonzero
   authoritative delta outside a new nominal Tube;
4. whether that issue needs only a contract test or a bounded transition;
5. whether margin accounting is Contract A or Contract B;
6. whether `nominal_half_width` replaces or temporarily aliases legacy width
   parameters;
7. whether one Validator pass is sufficient with the existing cell proof;
8. whether Stage 1B is required after P1 measurements;
9. whether the swarm core should be 3-D world-frame or an explicitly bounded
   fixed-altitude projection;
10. which independent system-level safety owner, if any, remains after the
    latest paper removes hard pairwise CBF from nominal Batch C;
11. whether old QP/CBF files can be deleted in Batch D after dependency scan.

## 35. SOL REVISION REVIEW CHECKLIST

The existing Sol planning-review window must independently verify:

A. the exact Stage 1A commit and remote identities;
B. whether Stage 1A stayed within scheduling/freshness/concurrency scope;
C. whether Stage 1A is safe as the P1 baseline;
D. bounded nominal Tube mathematical safety;
E. independent `+N/-N` clipping;
F. absence of scalar centerline symmetric shrinkage;
G. genuine configurability of `rho_nom`;
H. support for both 1.0 m and 1.5 m without hard-coded geometry;
I. no search/query beyond `rho_nom`;
J. actual removal of maximum-width and repeated inward behavior;
K. sufficiency of one continuous Validator pass;
L. fail-closed behavior after Validator failure;
M. continuous inter-sample/cell proof;
N. distinction among `rho_nom`, bounds, and current delta;
O. existing Recovery/Handoff handling of authoritative nonzero delta;
P. necessity or non-necessity of a recovery-only transition;
Q. Stage 1B measurement conditionality;
R. Preview separation from Tube construction and Batch C;
S. asymmetric `I_k`, `K_k`, `K(s)`, and `U_delta^K`;
T. final-paper NoQP Batch C alignment;
U. correct placement of neighbor-contraction scenario in Batch C;
V. world-frame separation rather than delta difference;
W. no overclaim of hard pairwise safety;
X. SPH remaining reference-only;
Y. protection of Stage 1A shared files from geometry rewrites;
Z. dependency closure of every regenerated whitelist.

## 36. Stop conditions

Stop and report instead of silently redesigning if:

- Stage 1A cannot be reliably verified;
- Stage 1A materially changed Tube geometry or safety semantics;
- the two remote identities cannot be explained to the review window;
- bounded Tube cannot preserve continuous fail-closed certification;
- repeated inward search proves necessary for safety and no bounded equivalent
  exists;
- planner, governor, Runtime, Recovery, Handoff, R3, or Horizontal-N redesign
  is required;
- the nonzero delta transition requires a major new Recovery architecture;
- Preview requires symmetric intervals;
- Batch C still requires obsolete joint QP/CBF semantics;
- a second authority or neighbor cache is needed;
- Stage 1A shared files cannot be preserved;
- a whitelist cannot be dependency-closed;
- unrelated user-owned files must be modified;
- a future action would require modifying the latest paper, Frozen Plan,
  AGENTS.md, or existing user-owned worktree artifacts.

## 37. Repository hygiene at candidate-plan creation

The candidate document is the only new file created by this turn. Existing
worktree entries remain untouched and are not part of this plan's acceptance
scope.

Expected post-write read-only state:

```text
branch: main
HEAD: cd313b315fcf4157273a7b0de84622073b3afaf6
configured upstream: d_isfgvf/main
origin/main: cd313b315fcf4157273a7b0de84622073b3afaf6
d_isfgvf/main: 5355be736339a650432280deec53362f61749a43
staged paths: 0
git diff --check: PASS
```

The following remain user-owned and must not be cleaned, restored, staged, or
attributed to this candidate plan:

```text
AGENTS.md modification
Stage 1A-related worktree/document entries if present
.codex
Testing/
existing docs
__pycache__ directories
```

## 38. Final execution boundary

This document does not authorize:

- bounded Tube implementation;
- Stage 1B;
- Preview implementation;
- Batch C;
- Batch D;
- Luna or any implementation writer;
- commit, push, tag, branch, stash, reset, restore, clean, or checkout.

The next action is only the existing Sol revision review of this exact
candidate document.

