# D-ISFGVF Post-Stage-1A Bounded-Tube / Preview / Batch-C Roadmap — Rev1

```text
DOCUMENT_ROLE=IMPLEMENTATION_PLAN
DOCUMENT_REVISION=REV1
DOCUMENT_STATUS=CANDIDATE / READY FOR SOL XHIGH REVISION REVIEW
SUPERSEDES_CANDIDATE_SHA256=605d825f538d365e1b457687d4a6b4786c669637257586481acb40e477df54e6
PREVIOUS_ACCEPTED_R3_HORIZONTAL_N_BASELINE=5355be736339a650432280deec53362f61749a43
STAGE1A_FROZEN_BASELINE=cd313b315fcf4157273a7b0de84622073b3afaf6
STAGE1A_ACCEPTANCE_STATUS=PREVIOUSLY ACCEPTED / FROZEN
STAGE1A_STALE_DIAGNOSTIC_CONTRACT=EXISTING_ACCEPTED_DIAGNOSTIC_EXCEPTION
STAGE1A_SHUTDOWN_CONTRACT=NO_PUBLICATION_AFTER_OWNERSHIP_FLIP
MARGIN_ACCOUNTING=CONTRACT_A
CURRENT_DELTA_TRANSITION=SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED
STAGE1B=MEASUREMENT_CONDITIONAL
PREVIEW_ARCHITECTURE=P2A_VALUE_ONLY_CORE_THEN_P2B_CONTROLLED_INTEGRATION
BATCH_C_GEOMETRY=3D_WORLD_FRAME
SOL_PLAN_AUDIT=PENDING
IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
BOUNDED_TUBE_IMPLEMENTATION_AUTHORIZED=false
STAGE1B_AUTHORIZED=false
PREVIEW_IMPLEMENTATION_AUTHORIZED=false
BATCH_C_AUTHORIZED=false
BATCH_D_AUTHORIZED=false
```

This is a revised candidate implementation plan. It is documentation only:
it does not authorize code, tests, launch/configuration, ROS, Git, Luna, or
any future stage. It is created beside, not over, the prior candidate.

## 1. Authority and source-of-truth hierarchy

The latest paper supplied in this planning window is the method source of
truth. It defines a three-dimensional local planner path, one scalar
horizontal offset, world-frame neighbor interaction, preview-constrained
reference motion, analytic scalar admissible-set projections, and matched
physical/reference injection. It does not define a final joint QP and does
not establish a formal hard pairwise CBF guarantee for nominal interaction.

Future decisions have this priority:

1. latest paper;
2. accepted/frozen A/B/R3/Horizontal-N and Stage-1A contracts;
3. verified implementation at `cd313b...`;
4. this Rev1 candidate after independent Sol review;
5. a separately approved execution specification.

Older proposals and prototypes remain historical reference only where they do
not conflict with those authorities. Accepted planner, governor, ISF-GVF,
C2, H2, Runtime, Recovery, Handoff, R3, and Horizontal-N semantics are not
reopened by this document.

## 2. Corrected Stage 1A history and evidence

The previous candidate inferred `PUSHED_CANDIDATE` because an independent Sol
acceptance was not materialized as a repository file. That inference is
historically wrong. Independent Sol review occurred before the commit and
push.

The authoritative historical status is:

```text
STAGE1A_PREVIOUSLY_ACCEPTED_BASELINE
STAGE1A_ACCEPTANCE_STATUS=PREVIOUSLY ACCEPTED / FROZEN
```

The actual sequence was:

```text
implementation
  -> independent Sol implementation audit
  -> bounded lifecycle repair
  -> fresh Sol xhigh re-audit
  -> STAGE_1A_IMPLEMENTATION_ACCEPTABLE
  -> real ROS acceptance
  -> STAGE_1A_REAL_ROS_ACCEPTED
  -> commit/push cd313b...
```

Original workflow evidence:

```text
/home/cxq/.codex/sessions/2026/08/26/rollout-2026-08-26T22-45-17-01a03e88-5640-71f1-bdce-e09add5689d1.jsonl
```

The final fresh Sol evidence recorded:

```text
Overall: STAGE_1A_IMPLEMENTATION_ACCEPTABLE
Lifecycle repair: PASS
Lock-order/deadlock audit: PASS
Regression test: PASS
Previously accepted Stage 1A invariants: PASS
Navigation fixture: PREEXISTING_NON_STAGE_1A
Remaining blocking issue: NONE
Final recommendation: PROCEED_TO_POST_REPAIR_REAL_ROS_ACCEPTANCE
```

The same review classified stale raw/cloud attempt behavior as:

```text
ACCEPTABLE_BY_FROZEN_CONTRACT
```

The subsequent real ROS result was `STAGE_1A_REAL_ROS_ACCEPTED`; the pushed
result was `STAGE_1A_ACCEPTED_FROZEN_AND_PUSHED`. These are the exact
historical facts authorizing `cd313b...`; a missing repository-local Sol
artifact does not downgrade them.

The real ROS acceptance record reported five successful goal arrivals with no
crash, deadlock, or worker stall; approximately 4 ms to the first path
message and approximately 604 ms to asynchronous Candidate arrival; command
publication near 50 Hz; local-map and trajectory-visualization publication
near 10 Hz; 14 Candidate and 14 Certified Geometry publications; 54 Tube
builds with minimum/median/mean/p90/p95/maximum durations of 28.958/
450.552/627.148/1236.661/2937.012/3715.862 ms; 31 source-current
finalizations; 23 stale/rejected completions; 54 raw/cloud attempts; 18,574
command callbacks with a 53.878 ms maximum; and scheduling evidence
`NO_FIFO`, `NO_50HZ_BUILD_CHURN`, `LATEST_ONLY_PENDING`, and
`CALLBACKS_HEALTHY`. R3, Horizontal-N, and exact-zero/tiny-nonzero checks
passed. This evidence is acceptance of Stage 1A scheduling/freshness, not a
promise that future P1 Tube geometry has any fixed completion latency.

## 3. Repository and remote identity

```text
repository: /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
branch: main
HEAD: cd313b315fcf4157273a7b0de84622073b3afaf6
origin/main: cd313b315fcf4157273a7b0de84622073b3afaf6
d_isfgvf/main: 5355be736339a650432280deec53362f61749a43
configured upstream: d_isfgvf/main
ahead/behind against configured upstream: 1/0
```

Verified remotes:

```text
d_isfgvf  git@github.com:Accqqq/D-ISFGVF.git
origin    git@github.com:Accqqq/gvf_ws.git
lifted    git@github.com:Accqqq/Lifted_GVF.git
```

`origin/main` contains the pushed Stage-1A continuation. `d_isfgvf/main`
still identifies the earlier accepted R3 + Horizontal-N baseline. Neither
remote is mutated or selected as a new authority by this plan.

## 4. Frozen baselines and protected Stage-1A scope

Previous accepted R3 + Horizontal-N baseline:

```text
5355be736339a650432280deec53362f61749a43
```

Stage-1A frozen baseline:

```text
cd313b315fcf4157273a7b0de84622073b3afaf6
feat: accept Stage 1A tube freshness scheduling
```

The commit changes exactly:

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
```

Stage 1A owns scheduling/freshness/lifecycle only: scheduler-only timer
permits; one joined worker; one in-flight build; one latest-only pending
request; immutable request/work identity; job-local state; stale completion
abandonment; source/path/frame/task gates; reset/deactivate/shutdown
invalidation; and configured `tube_update_period` wiring.

It does not change Tube geometry, `rho_nom`, search semantics, filtering,
continuous proof, planner clearance, Horizontal-N, Runtime, Recovery, Handoff,
R3 geometry, Preview, swarm coordination, QP, CBF, or Batch C/D.

Inherited contracts remain: planner path and active reference are distinct;
Candidate/Active/Certified/Runtime authority remain distinct; exact zero and
finite tiny-nonzero Horizontal-N behavior remain; the centerline and altitude
remain full 3-D and planner-owned; `N(w)=normalize(e_z x p_w(w))\),
`N_z=0`, `r=p+N delta`, and `r_z=p_z\); and disconnected unsafe gaps are
never convexified.

## 5. Acceptance reconciliation

### 5.1 Existing accepted stale diagnostic exception

The accepted semantics distinguish:

```text
AUTHORITATIVE TUBE PUBLICATION
  versus
DIAGNOSTIC ATTEMPT PROVENANCE
```

Stale same-task raw/cloud diagnostic attempt evidence is allowed under the
existing provenance and exactly-once contract. This is:

```text
STAGE1A_STALE_DIAGNOSTIC_CONTRACT=EXISTING_ACCEPTED_DIAGNOSTIC_EXCEPTION
```

The exception does not allow stale completion to commit or expose Candidate
authority, Certified Geometry authority, Active Tube installation, Runtime or
execution state, authoritative epoch/profile/cache/cloud state, or any state
that can resurrect obsolete Tube authority. The accepted topics are not moved,
renamed, or redesigned here. Finding 1 is not a Stage-1A defect and is not a
P1 repair target.

### 5.2 Shutdown contract clarification

The current accepted lifecycle baseline is:

```text
STAGE1A_SHUTDOWN_BASELINE=NO_PUBLICATION_AFTER_OWNERSHIP_FLIP
```

After the shutdown ownership/publication-disable linearization boundary, no
diagnostic, Candidate, Certified Geometry, Runtime, command, or DELETE
publication may occur. Pending work is invalidated, running work may return
from existing computation, no new work starts, and the worker is joined.

The older R3 final `publishManualDelete()` concern is separately recorded as:

```text
OPTIONAL_POST_ACCEPTANCE_OBSERVABILITY_AMENDMENT
```

If separately authorized later, it must prove exactly one DELETE before the
no-publication boundary, no later ADD, no authority resurrection, idempotent
repeated shutdown, and no worker race. It is not implemented here and does
not make the accepted Stage 1A baseline defective.

## 6. Revised roadmap

There is no second Stage-1A implementation and no generic Stage-1A acceptance
gate. The roadmap is:

```text
accepted/frozen R3 + Horizontal-N
  -> previously accepted/frozen Stage 1A (cd313b...)
  -> P1 bounded nominal asymmetric Horizontal-N Tube
  -> P1 safety/performance measurement
  -> Stage 1B only if measurement justifies it
  -> P2A value-only Preview viability core
  -> P2A deterministic acceptance
  -> P2B controlled Preview integration and regressions
  -> P2 Preview acceptance
  -> P3 final-paper 3-D world-frame NoQP Batch C
  -> P4 Batch-D cleanup/observability/experiment support
```

All stages after the accepted baseline remain unauthorized by this plan.

## 7. Latest-paper method alignment

For UAV \(i\), the planner supplies a locally parameterized 3-D centerline
`p_i(w_i)\). A scalar horizontal `delta_i` moves the active reference
inside its local Path-Tube. Since paths and phases are local, interaction is
defined in common physical workspace coordinates, not phase differences.

The final intended pipeline is:

```text
geometric Path-Tube I(w)
  -> preview-feasible K_k and continuous K(s)
  -> b_pre and beta
  -> g_sep + beta*g_coh + g_damp
  -> bounded g_des
  -> analytic u_w_nom and u_delta_nom
  -> scalar admissible-set projections
  -> final u_w and u_delta
  -> matched physical/reference injection
```

The swarm layer provides intent only. It never owns authoritative `w`,
`delta`, final port values, or physical command publication.

## 8. P1 bounded nominal asymmetric Tube

P1 retains one configurable nominal horizontal half-width:

```text
rho_nom > 0
I_nom(w) = [-rho_nom,+rho_nom]
```

The final interval intersects the nominal interval, independent obstacle
bounds, full-3D regularity bounds, and continuous-certification admissibility:

```text
open space:              [-rho_nom,+rho_nom]
only -N obstacle:        [clipped_lower,+rho_nom]
only +N obstacle:        [-rho_nom,clipped_upper]
both sides constrained:  [clipped_lower,clipped_upper]
```

Lower and upper are independent and may be asymmetric. P1 must not apply a
scalar symmetric shrink, maximize width, search beyond `rho_nom`, recreate
three-family inward search, repeatedly halve scales, or validate alternate
candidates as a normal path.

The intended bounded flow is:

```text
bounded nominal interval
  -> independent +N/-N clipping
  -> full-3D regularity intersection
  -> conservative local PWL representation
  -> one continuous SurfaceValidator pass, if proof is sufficient
  -> bounded certified Candidate
```

Failure is fail-closed: retain the accepted planner zero baseline or reject
the candidate with explicit provenance. It never silently expands the domain.

## 9. P1 parameter, builder, and proof contract

Authoritative owner and ROS parameter:

```text
phase_offset_navigation::TubeCrossSectionConfig::nominal_half_width
phase_offset/tube/nominal_half_width
```

The value is finite and strictly positive; 1.0 m and 1.5 m are experimental
settings, not constants. `fixed_delta_max`, `max_offset`, and
`environment_search_extent` are not alternate authoritative nominal owners.
Any compatibility alias must be explicit, deterministic, subordinate, and
diagnosed.

`TubeCrossSectionSolver` and `TubeBuilder` may receive only bounded
directional-query and connected-component amendments. `TubeFilter`,
`TubeSurfaceValidator`, and `PathCellGeometryCertificate` retain their
accepted roles. `CertifiedTubeBuilder` may use one bounded candidate and one
continuous validation pass only if the complete proof is shown sufficient; it
is not a performance shortcut.

The proof must cover immutable provenance, monotone phase, independent knot
bounds, conservative interpolation, path/normal variation, full-3D
regularity, continuous swept-surface obstacle clearance, unknown/unavailable/
out-of-map fail-closed behavior, current-anchor inclusion, and certified
forward coverage.

## 10. Contract-A margin accounting

```text
MARGIN_ACCOUNTING=CONTRACT_A
```

Contract A freezes the following:

1. cloud/voxel evidence is inflation-aware under the accepted snapshot map
   contract;
2. `planning/safe_distance` is the sole authoritative production physical
   obstacle clearance;
3. UAV radius, tracking, localization, map-uncertainty, and equivalent legacy
   quantities are not added again when already represented in the snapshot;
4. visible legacy fields remain diagnostic/compatibility-only;
5. continuous proof may add only a geometric cover term for interpolation,
   path, and normal variation;
6. that cover term is not a second physical margin;
7. missing, insufficient, revision-mismatched, unavailable, or unprovable
   inflation evidence fails closed.

Actual code seams to preserve and audit:

```text
phase_offset_navigation::TubeCrossSectionConfig::planner_safe_distance
phase_offset_navigation::CloudOccupancySnapshot::included_map_inflation
plan_env::cloudOccupancySnapshotConsistent()
plan_env::queryCloudOccupancySnapshotClearance()
PhaseOffsetMatchedAdapter::loadConfig() -> planning/safe_distance
cloud_occupancy_query_config_.required_preincluded_map_uncertainty
```

Legacy/non-authoritative fields include `RobustTubeMargins::uav_radius`,
`map_uncertainty`, `localization_uncertainty`, `tracking_error_bound`,
`preincluded_map_uncertainty`, `fullEffectiveRadius()`,
`residualEffectiveRadius()`, and legacy `search_extent` where it is not
the bounded nominal owner. No double inflation or silent omission is allowed.

## 11. Current delta transition

The following quantities remain distinct:

```text
rho_nom       configured requested half-width
lower/upper   actual geometric bounds at a phase
delta_current authoritative current reference state
```

For `delta_current=+1.2`, `rho_nom=1.0`, and `I_nom=[-1,+1]`, the system
must neither clamp to +1 nor reset to zero. The bounded Tube does not expand
to capture the old state. Existing Recovery/Handoff/Authority retains the
authoritative reference while the candidate cannot take ownership; a new
epoch may take over only after valid re-entry through existing semantics.

```text
CURRENT_DELTA_TRANSITION=SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED
```

The test must prove no clamp, no zero reset, no unsafe takeover, retained
authority consistency, valid re-entry takeover, and no disconnected-gap
convexification. No recovery-only Tube extension or production Recovery/
Handoff change is authorized. If the test fails, stop and reclassify from
evidence rather than silently repairing Recovery in P1.

## 12. Stage 1B conditionality

Stage 1B is cooperative cancellation of obsolete in-flight computation and is
not automatic. Measure after accepted Stage 1A plus P1. Recommend it only if
meaningful stale waste remains: high build p95/max, frequent expensive stale
jobs, or validation loops that can be safely cancelled.

```text
SUCCESS   current bounded geometry certified
FAILED    current geometry failed certification
CANCELLED obsolete work stopped before a geometry/safety conclusion
```

If stale waste is negligible, record `STAGE1B_NOT_REQUIRED`. Do not authorize
Stage 1B in Rev1.

## 13. P2A value-only Preview viability core

The old wording that Preview was completely standalone was too broad. Current
production code already has `completeThroughRecoveryOwner()` calling
`PreviewFeasibility::evaluate()`. “Standalone” therefore means an
architecturally independent value-only core before controlled integration,
not absence of every existing Preview-related dependency.

P2A may add a side-effect-free `tube_viability` core owning:

```text
I_k
K_H = I_H
K_k = I_k intersect (K_{k+1} Minkowski-sum R_delta)
continuous conservative K(s)
kappa_lower / kappa_upper
contraction facts
b_pre and beta input facts
```

P2A must not mutate Runtime, Recovery, Handoff, authority, commands, or current
`PreviewFeasibility` production behavior. It preserves asymmetric intervals;
`lower != -upper` is a first-class deterministic case.

P2A acceptance covers exact recursion, continuous inner-envelope containment,
contraction-rate bounds, piecewise-C1 junctions and one-sided derivatives,
empty `K_0` while geometric `I(w)` remains nonempty, preview width and beta
saturation, no remote-bottleneck global intersection, current delta outside
the envelope, provenance mismatch, and deterministic repeated input.

## 14. P2B controlled Preview integration

Only after P2A acceptance may P2B integrate the accepted value-only result
into the existing production Preview path. The call graph requiring explicit
closure is:

```text
PhaseOffsetMatchedAdapter::completeThroughRecoveryOwner()
  -> PreviewFeasibility::evaluate()
  -> HandoffStateMachine / Recovery-owner decision
```

P2B must preserve Recovery/Handoff/Runtime authority and include regressions
for Preview status/provenance, current-owner retention, Recovery transitions,
Runtime prepared steps, matched ports, stale path/frame/profile rejection, and
the rule that a value-only failure cannot directly publish a command or
install authority.

Preview remains before Batch C. Batch C cannot consume `K(s)` until P2A and
P2B are independently accepted.

## 15. Batch C: final-paper 3-D world-frame NoQP

```text
BATCH_C_GEOMETRY=3D_WORLD_FRAME
```

Production Batch C uses 3-D world-frame position, velocity, relative position,
relative velocity, pair geometry, and pair distance `||x_i-x_j||`.
`g_sep`, `g_coh`, `g_damp`, and bounded `g_des` are 3-D world vectors.
No production neighbor, collision, or separation decision may silently
project to XY.

The intended pipeline is:

```text
immutable timestamped 3-D neighbor snapshot
  -> world-frame pair geometry
  -> g_sep + beta*g_coh + g_damp
  -> bounded g_des
  -> analytic u_w_nom / u_delta_nom
  -> scalar U_w projection
  -> preview-derived U_delta^K projection
  -> final u_w / u_delta
  -> matched injection
```

The per-agent navigation layer remains the sole owner of authoritative phase,
offset, ports, and matched execution. The old production joint QP/CBF is not
reintroduced.

## 16. Z-sensitive and single-side-contraction acceptance

The current `phase_offset_swarm` core has `Eigen::Vector2d` fields and its
shadow node sets Z to zero. P3 must migrate those assumptions or stop.
`phase_offset_msgs/msg/AgentState.msg` already carries 3-D fields and remains
`LEAVE_UNCHANGED` unless a concrete timestamp/frame dependency requires a
separate authorization.

Deterministic Z tests must place two agents at equal or nearly equal XY with a
material Z separation, verify nonzero 3-D distance, and verify the resulting
neighbor/separation consequence. The inverse sensitivity changes only Z and
must change the 3-D distance and corresponding decision. No production safety
test may use only `abs(delta_i-delta_j)` or XY distance.

Single-side Tube contraction plus a neighbor in the contraction direction
remains a Batch-C acceptance scenario, not P1. Record:

```text
lower_i upper_i I_i(w) K_i(s)
kappa_lower kappa_upper b_pre beta_i
delta_i delta_j
3-D ||x_i-x_j||
g_sep g_coh g_damp g_des
u_w_nom u_delta_nom U_w U_delta^K
final u_w final u_delta
phase separation
minimum pairwise distance
minimum obstacle distance
```

Evaluate both longitudinal/phase and transverse rearrangement. Keep separation
active at beta zero, avoid a full velocity-consensus requirement, and verify
identical final ports on internal and physical matched channels. Do not invent
a yield state machine.

## 17. SPH and hard-safety disposition

SPH remains reference material only. Permitted analogies are repulsion to
`g_sep`, weak cohesion to `g_coh`, and damping/viscosity to `g_damp`.
Do not import SPH environmental force, planner, full controller/dynamics, or
raycasting gate automatically.

Final Batch C is soft spacing regulation. This plan makes no formal hard
pairwise invariance claim from `g_sep + beta*g_coh + g_damp`. Any future hard
safety supervisor requires a separate system-level owner, inputs, authority,
and proof; it must not be reintroduced as the old joint QP/CBF.

## 18. Batch D

P4 follows Batch-C acceptance and is limited to cleanup, observability, and
experiment support. It may, after a dependency scan, remove or quarantine
obsolete QP/CBF code, eliminate obsolete 2-D shadow assumptions, migrate
legacy width/search parameters, clean topic namespaces, add bag-analysis
interfaces, and replace stale CBF metrics with layered Tube/Preview/
interaction/port diagnostics.

Preferred diagnostics include `I(w)`, bounds, `K(s)`, contraction rates,
`b_pre`, beta, interaction terms, `g_des`, nominal/projected scalar ports,
`delta`, `dot_w`, `v_s`, and matched reference velocity.

## 19. Exact P1 bounded-Tube whitelist

No path below is authorized until a separate execution specification freezes
that batch.

### 19.1 P1 production — MODIFY

```text
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

The adapter files are parameter/diagnostic propagation only. Stage-1A worker,
pending-slot, currentness, and lifecycle logic are protected.

### 19.2 P1 geometry tests — TEST_ONLY

```text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
```

### 19.3 P1 transition amendment — TEST_ONLY

```text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
```

These exact tests own the `delta_current=+1.2`, `rho_nom=1.0` contract.
They do not authorize production Recovery/Handoff edits.

### 19.4 P1 regression-only — LEAVE_UNCHANGED

```text
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/active_reference_authority_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/preview_feasibility_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/handoff_state_machine_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_recovery_owner_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

### 19.5 P1 launch/config — MODIFY ONLY IF REQUIRED

```text
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

If NodeHandle loading is sufficient, both remain unchanged. No P1 CMake or
package edit is expected.

### 19.6 P1 exclusions

```text
EXCLUDED
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_recovery_owner.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
all planner/governor/Horizontal-N frame sources
phase_offset_allocator.*
phase_offset_cbf_constraints.*
AGENTS.md
all user-owned worktree artifacts
```

## 20. Exact P2A value-only whitelist

### ADD

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_viability.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_viability.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_viability_test.cpp
```

### MODIFY only to compile the value core

```text
src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
```

`package.xml` and `tube_types.h` are `LEAVE_UNCHANGED` unless a concrete
new dependency or immutable value-type requirement is proven first. Such a
need is a whitelist exception, not implied authorization.

### EXCLUDED during P2A

```text
preview_feasibility.*
handoff_state_machine.*
phase_offset_recovery_owner.*
phase_offset_runtime.*
phase_offset_matched_adapter.*
all authority and command publication paths
```

## 21. Exact P2B controlled integration whitelist

### MODIFY

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/preview_feasibility.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/preview_feasibility.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

The smallest expected integration is Preview plus the current adapter call
site. The following are `LEAVE_UNCHANGED` production regression dependencies
unless evidence proves a direct P2B edit is unavoidable:

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/handoff_state_machine.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/handoff_state_machine.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_recovery_owner.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_recovery_owner.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
```

### TEST_ONLY / REGRESSION_ONLY

```text
src/swarm_planner/phase_offset/phase_offset_navigation/test/preview_feasibility_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/handoff_state_machine_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_recovery_owner_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/active_reference_authority_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

`phase_offset_navigation/CMakeLists.txt` is MODIFY only if P2B adds a new test
target beyond the P2A target. Any production Recovery/Handoff edit requires a
new bounded whitelist authorization.

## 22. Exact P3 3-D Batch-C whitelist

### 22.1 Core — MODIFY_TO_3D

```text
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
```

### 22.2 ROS shadow/config — MODIFY_TO_3D

```text
src/swarm_planner/phase_offset/phase_offset_swarm/src/swarm_intent_shadow_node.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/config/swarm_intent_v1.yaml
src/swarm_planner/phase_offset/phase_offset_swarm/launch/swarm_intent_shadow.launch
```

The existing shadow node is classified `MODIFY_TO_3D`; it may not remain an
ambiguous XY/Z=0 production shadow.

### 22.3 Build metadata — MODIFY if required

```text
src/swarm_planner/phase_offset/phase_offset_swarm/CMakeLists.txt
src/swarm_planner/phase_offset/phase_offset_swarm/package.xml
```

### 22.4 Exact unit/ROS tests — TEST_ONLY

```text
src/swarm_planner/phase_offset/phase_offset_swarm/test/neighbor_manager_test.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/test/elastic_interaction_test.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/test/conflict_friction_test.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/test/swarm_intent_test.cpp
src/swarm_planner/phase_offset/phase_offset_swarm/test/swarm_intent_shadow.test
src/swarm_planner/phase_offset/phase_offset_swarm/test/swarm_intent_shadow_test.py
```

### 22.5 bspline integration — separate P3 sub-authorization

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/package.xml
```

The pure `phase_offset_core` kernels already expose stateless admissible-port
projection and matched composition. They remain `LEAVE_UNCHANGED /
REGRESSION_ONLY` unless a separately frozen P3 integration proves that the
latest-paper scalar projection cannot be represented without a bounded core
amendment:

```text
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_types.h
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_projector.h
src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/port_projector_test.cpp
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/matched_port.h
src/swarm_planner/phase_offset/phase_offset_core/src/matched_port.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/matched_port_test.cpp
```

The adapter may orchestrate immutable 3-D intent and accepted scalar results,
but must not become the home for new control mathematics. If the pure kernels
are insufficient, P3 stops and requests an exact ADD/MODIFY whitelist.

`src/swarm_planner/phase_offset/phase_offset_msgs/msg/AgentState.msg` is
`LEAVE_UNCHANGED` unless a concrete 3-D timestamp/frame dependency proves an
amendment necessary. P3 must not rewrite Stage-1A scheduling, Tube construction,
Runtime authority, planner, or governor.

## 23. Exact P4 Batch-D audit whitelist

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
```

Diagnostics and bag-analysis interfaces are `AUDIT_ONLY` until exact paths
are dependency-closed. No deletion follows merely from an obsolete term;
deletion requires zero production references, regressions, and authorization.

## 24. Deterministic acceptance matrix

P1 deterministic tests cover:

1. exact open-space bounds for 1.0 m and 1.5 m;
2. independent -N, +N, and both-side clipping;
3. no query beyond `rho_nom`;
4. side-specific regularity intersection;
5. unknown, unavailable, and out-of-map fail-closed behavior;
6. sloped 3-D centerline preserving Horizontal-N and altitude;
7. continuous inter-sample proof;
8. zero normal inward-family attempts;
9. one final validator target;
10. zero and nonzero connected components;
11. no gap convexification;
12. immutable-input determinism;
13. stale path/source/frame rejection;
14. Candidate/Active/Certified regressions;
15. distinct `rho_nom`, bounds, and `delta_current`.

P2 deterministic tests cover asymmetric `I_k`, exact backward recursion,
continuous inner `K(s)`, contraction rates, one-sided derivatives, empty
`K_0`, width/beta saturation, non-global bottlenecks, `U_delta^K`,
current-state-outside-envelope, provenance mismatch, and no symmetric
assumption.

P3 deterministic tests cover 3-D neighbor freshness, equal-XY/different-Z
distance, separation/cohesion/damping directions, beta affecting cohesion
only, separation at beta zero, bounded intent, no full consensus requirement,
world distance rather than delta difference, and the single-side contraction
scenario.

## 25. ROS and performance acceptance

After separately authorized P1 implementation, measure at least:

```text
Tube request/permit rate
worker build rate
build median p90 p95 max
request-to-publication age
Candidate publication rate
Certified Geometry publication rate
stale pre-build rejection
stale in-flight completion
stale abandonment
SurfaceValidator invocation count
inward-search attempt count
ESDF/cloud query count
maximum queried transverse offset
worker busy ratio
cmdCallback timing
traj_vis rate
local_map rate
tube_update_period
```

Structural targets are normal bounded inward-search attempts equal to zero,
maximum queried transverse offset no greater than `rho_nom`, and one final
continuous validation pass only when proof obligations are sufficient. No
unmeasured 1 ms guarantee is allowed. The A/B comparison is accepted Stage 1A
versus Stage 1A plus P1, with no Runtime, Recovery, R3, Horizontal-N, map, or
trajectory-visualization regression.

## 26. Remaining genuinely open decisions

Resolved items are not open: Stage-1A status, stale diagnostics, shutdown,
Contract A, current-delta classification, Stage-1B conditionality, P2A/P2B
architecture, and 3-D Batch-C geometry.

Genuinely open later decisions are limited to:

- exact legacy-width compatibility diagnostics;
- proof sufficiency of one validator pass or a bounded equivalent;
- measured need for Stage 1B;
- exact P2B immutable adapter type after P2A acceptance;
- final 3-D swarm parameter defaults and timestamp policy;
- whether a separately owned hard-safety supervisor is needed;
- exact Batch-D delete list after dependency closure.

## 27. Stop conditions

Stop and report if:

- Contract A cannot avoid double or omitted margins;
- bounded correctness requires search beyond `rho_nom`;
- independent clipping cannot preserve continuous certification;
- one-pass validation lacks proof and no bounded alternative is specified;
- the current-delta contract test fails accepted Recovery/Handoff semantics;
- P2A cannot remain value-only;
- P2B cannot preserve Recovery/Handoff/Runtime authority;
- P3 cannot be dependency-closed in 3-D;
- a production XY-only pair-safety assumption remains;
- Stage-1A concurrency needs a geometry-driven rewrite;
- unrelated user-owned files require modification;
- the latest paper, Frozen Plan, AGENTS.md, or user artifacts would need edits.

Do not widen scope to solve a stop condition.

## 28. Repository hygiene

The prior candidate remains an immutable historical artifact:

```text
docs/Codex_D-ISFGVF_Post-Stage1A_BoundedTube_Preview_BatchC_Roadmap_Plan_2026-08-27.md
SHA256=605d825f538d365e1b457687d4a6b4786c669637257586481acb40e477df54e6
```

Rev1 is the only file created by this documentation phase. Existing user-owned
worktree entries remain untouched:

```text
AGENTS.md modification
.codex
Testing/
docs/Codex_Tube_Freshness_Stage1A_Implementation_Plan_2026-08-27.md
docs/D-ISFGVF_PhaseOffset_Tube_Architecture_Refactor_Brief_2026-08-23.md
docs/D-ISFGVF_Tube_Freshness_Stage1_Handoff_2026-08-26.md
src/swarm_planner/bspline_traj/test/__pycache__/
src/uav_simulator/so3_quadrotor_simulator/test/__pycache__/
```

No staging, commit, push, cleanup, restore, reset, checkout, or ROS launch is
part of Rev1 creation. Expected checks are branch `main`, HEAD and
`origin/main` at `cd313b...`, `d_isfgvf/main` at `5355be...`, upstream
`d_isfgvf/main`, ahead/behind `1/0`, staged paths zero, and
`git diff --check` passing. `/tmp` ROS logs are not repository artifacts.

## 29. Sol xhigh Rev1 review checklist

A new independent Sol xhigh reviewer must inspect this exact path and computed
SHA read-only. It must verify:

- artifact identity and SHA;
- corrected Stage-1A history and original Sol/ROS evidence;
- no downgrade from missing repo-local evidence;
- accepted stale diagnostic exception with no stale authority exposure;
- no publication after shutdown ownership flip;
- optional final DELETE kept separate;
- Contract A against actual functions and no double/omitted margin;
- bounded asymmetric Tube, independent sides, no external search or maximum
  objective, no repeated inward families, and valid continuous proof;
- current-delta classification and dependency-closed E2E test;
- Stage 1B measurement conditionality;
- P2A value-only and P2B call-graph integration/regressions;
- Preview remaining before Batch C;
- unequivocal 3-D Batch C and all Vector2d/XY/Z=0 assumptions accounted for;
- exact P3 node/build/package/test closure and Z-sensitive tests;
- contraction-plus-neighbor placement in Batch C;
- SPH reference-only and no hard pairwise claim;
- exact dependency-closed P1/P2A/P2B/P3/P4 whitelists;
- every prior MUST_FIX corrected or reconciled.

Required report sections:

```text
ARTIFACT_IDENTITY
STAGE1A_HISTORY_RECONCILIATION_AUDIT
STALE_DIAGNOSTIC_CONTRACT_AUDIT
SHUTDOWN_CONTRACT_AUDIT
CONTRACT_A_MARGIN_AUDIT
BOUNDED_TUBE_AUDIT
CURRENT_DELTA_TRANSITION_AUDIT
STAGE1B_AUDIT
P2A_VALUE_ONLY_PREVIEW_AUDIT
P2B_INTEGRATION_AUDIT
BATCH_C_3D_DEPENDENCY_AUDIT
Z_SENSITIVE_TEST_AUDIT
WHITELIST_AUDIT
MUST_FIX_BEFORE_FREEZE
SHOULD_FIX
OPTIONAL
FINAL_VERDICT
```

Every MUST_FIX must cite the exact Rev1 section, code/file/function evidence,
blocking reason, and smallest documentation correction. Sol may not reopen
accepted Stage-1A semantics merely because another design is possible.

Allowed final verdicts:

```text
REVISED_PLAN_REV1_ACCEPTABLE_READY_FOR_FREEZE
REVISED_PLAN_REV1_ACCEPTABLE_WITH_REQUIRED_CORRECTIONS_NOT_READY_FOR_FREEZE
REVISED_PLAN_REV1_NOT_ACCEPTABLE_NOT_READY_FOR_FREEZE
```

## 30. Execution boundary

This Rev1 plan does not authorize P1, Stage 1B, P2, P3, P4, Luna, code/test/
config changes, staging, commit, push, tag, branch, reset, restore, clean, or
ROS. After Sol returns, Main must not silently edit Rev1. Any MUST_FIX leaves
it candidate and stops implementation. An acceptable verdict makes it only
documentation-freeze-ready; freezing and implementation require separate
authorization.
