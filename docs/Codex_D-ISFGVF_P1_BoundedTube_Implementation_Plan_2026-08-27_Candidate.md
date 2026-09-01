# D-ISFGVF P1 Bounded Nominal Asymmetric Tube — Candidate Plan

~~~text
DOCUMENT_ROLE=P1_IMPLEMENTATION_PLAN
DOCUMENT_STATUS=CANDIDATE / READY FOR SOL XHIGH REVIEW
P1_SCOPE=BOUNDED_NOMINAL_ASYMMETRIC_TUBE
MASTER_REV1_REFERENCE_SHA256=e3d2818cb2f03c06c249af5fc420440a7a0eb719df4137d113f5b18789cbc69b
PREVIOUS_ACCEPTED_R3_HORIZONTAL_N_BASELINE=5355be736339a650432280deec53362f61749a43
STAGE1A_FROZEN_BASELINE=cd313b315fcf4157273a7b0de84622073b3afaf6
STAGE1A_ACCEPTANCE_STATUS=PREVIOUSLY_ACCEPTED / FROZEN
STAGE1A_STALE_DIAGNOSTIC_CONTRACT=EXISTING_ACCEPTED_DIAGNOSTIC_EXCEPTION
STAGE1A_SHUTDOWN_CONTRACT=NO_PUBLICATION_AFTER_OWNERSHIP_FLIP
MARGIN_ACCOUNTING=CONTRACT_A
CURRENT_DELTA_TRANSITION=SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED
STAGE1B=MEASUREMENT_CONDITIONAL
SOL_P1_PLAN_AUDIT=PENDING
P1_IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
~~~

This is a self-contained, documentation-only Candidate Plan for exactly one
future implementation stage: P1 bounded nominal asymmetric Tube. It does not
authorize production code, tests, launch/configuration edits, ROS, Luna, Git,
or any future stage. It does not revise the master Roadmap, the latest paper,
the Frozen Stage-1A Plan, or accepted A/B/R3/Horizontal-N contracts.

## 1. Scope, authority, and non-goals

P1 changes only the geometric Tube construction objective and the minimum
configuration/diagnostic propagation needed to request one bounded nominal
horizontal interval. It preserves the accepted scheduling, freshness,
authority, lifecycle, frame, planner, and execution architecture.

P1 is not a Stage-1A repair, Stage-1B cancellation implementation, Preview
implementation, Recovery/Handoff redesign, Runtime redesign, Horizontal-N
redesign, swarm/Batch-C implementation, QP/CBF implementation, or planner,
governor, C2, H2, or R3 rewrite.

The authority order is:

1. latest supplied D-ISFGVF paper;
2. accepted/frozen A/B/R3/Horizontal-N contracts;
3. frozen Stage-1A implementation at cd313b;
4. this Candidate Plan after fresh Sol xhigh review;
5. a separately authorized P1 execution handoff.

No implementation action follows from the roadmap alone.

## 2. Exact repository and remote identity

~~~text
repository: /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
branch: main
HEAD: cd313b315fcf4157273a7b0de84622073b3afaf6
origin/main: cd313b315fcf4157273a7b0de84622073b3afaf6
d_isfgvf/main: 5355be736339a650432280deec53362f61749a43
configured upstream: d_isfgvf/main
ahead/behind against configured upstream: 1/0
~~~

Verified remotes:

~~~text
d_isfgvf  git@github.com:Accqqq/D-ISFGVF.git
origin    git@github.com:Accqqq/gvf_ws.git
lifted    git@github.com:Accqqq/Lifted_GVF.git
~~~

origin/main contains the pushed Stage-1A continuation. d_isfgvf/main identifies
the earlier accepted R3 + Horizontal-N baseline. Neither remote is mutated by
this plan.

The master roadmap remains Candidate at:

~~~text
docs/Codex_D-ISFGVF_Post-Stage1A_BoundedTube_Preview_BatchC_Roadmap_Plan_2026-08-27_Rev1.md
SHA256=e3d2818cb2f03c06c249af5fc420440a7a0eb719df4137d113f5b18789cbc69b
~~~

Its future P3 blockers are explicitly out of scope and do not block P1.

## 3. Closed Stage-1A history

Stage 1A is a previously accepted and frozen baseline, not an unreviewed
candidate. The accepted commit is:

~~~text
cd313b315fcf4157273a7b0de84622073b3afaf6
feat: accept Stage 1A tube freshness scheduling
~~~

Independent Sol acceptance occurred before commit/push. Original evidence:

~~~text
/home/cxq/.codex/sessions/2026/08/26/rollout-2026-08-26T22-45-17-01a03e88-5640-71f1-bdce-e09add5689d1.jsonl
~~~

The final fresh Sol record was:

~~~text
Overall: STAGE_1A_IMPLEMENTATION_ACCEPTABLE
Lifecycle repair: PASS
Lock-order/deadlock audit: PASS
Regression test: PASS
Previously accepted Stage 1A invariants: PASS
Navigation fixture: PREEXISTING_NON_STAGE_1A
Remaining blocking issue: NONE
Final recommendation: PROCEED_TO_POST_REPAIR_REAL_ROS_ACCEPTANCE
~~~

The subsequent real ROS result was STAGE_1A_REAL_ROS_ACCEPTED and the final
historical result was STAGE_1A_ACCEPTED_FROZEN_AND_PUSHED. There is no second
Stage-1A acceptance gate in P1.

The four Stage-1A paths remain protected:

~~~text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
~~~

The adapter paths occur in the P1 list only for nominal-width configuration,
propagation, and bounded-Tube diagnostics. Their worker, pending-slot,
currentness, provenance, reset, deactivate, shutdown, and join semantics are
frozen.

## 4. Protected Stage-1A contracts

P1 must preserve:

- one scheduler-only production Tube timer permit source;
- one joined worker and at most one in-flight build;
- one latest-only pending request slot and no FIFO;
- command callbacks that do not perform heavy Tube construction;
- immutable request/work identity;
- job-local Tube state;
- stale pre-build rejection and stale completion abandonment;
- task/source/path/frame/authority currentness gates;
- Candidate and Certified publication gates;
- reset/deactivate invalidation;
- shutdown ownership flip, worker join, and no post-shutdown publication;
- configured tube_update_period semantics.

The accepted distinction is:

~~~text
AUTHORITATIVE TUBE PUBLICATION
  versus
DIAGNOSTIC ATTEMPT PROVENANCE
~~~

Stale same-task raw/cloud attempt evidence remains allowed under its existing
provenance and exactly-once contract. It cannot commit or expose Candidate
authority, Certified Geometry, Active Tube, Runtime, execution authority,
authoritative profile/cache/epoch state, or obsolete geometry.

The shutdown contract remains:

~~~text
STAGE1A_SHUTDOWN_CONTRACT=NO_PUBLICATION_AFTER_OWNERSHIP_FLIP
~~~

P1 does not add a final marker DELETE requirement and does not change shutdown.

## 5. Frozen Horizontal-N geometry

P1 uses the accepted Horizontal-N specialization without modification:

~~~text
N(w) = normalize(e_z x p_w(w))
N_z = 0
r(w,delta) = p(w) + N(w) delta
r_z = p_z
~~~

The planner centerline p(w) remains full 3-D and planner-owned. Altitude
remains planner-owned. The fixed-w Tube cross-section is horizontal. P1 does
not add a vertical offset, alter frame provenance, or redesign the normal.

The certified regularity requirement remains:

~~~text
||r_w(w,delta)|| >= m_r
~~~

over the entire certified interval, including inter-sample/cell coverage.
Exact delta == 0.0 and finite tiny-nonzero behavior remain distinct.

## 6. P1 objective and current call graph

The current code supports a broad legacy construction path. The current
CertifiedTubeBuilder builds a full candidate, validates it, and on retryable
failure constructs inward BOTH_SIDED, POSITIVE_ONLY, and NEGATIVE_ONLY
families with scale halving and repeated SurfaceValidator calls. The current
TubeCrossSectionSolver uses cross_section.search_extent, while TubeBuilderConfig
contains fixed_delta_max and max_offset.

P1 changes the normal objective to:

~~~text
certify the safe subset of one requested bounded nominal Tube
~~~

The read-only production dependency chain is:

~~~text
PhaseOffsetMatchedAdapter::loadConfig()
  -> PhaseOffsetMatchedAdapterConfig::tube
  -> makeBuildRequest()
  -> immutable TubeBuildRequest
  -> buildTubeEpoch()
  -> TubeEpochManager::update()
  -> CertifiedTubeBuilder::build()
  -> TubeBuilder::buildCloudClearance() for ESDF
  -> TubeCrossSectionSolver::solve()
  -> TubeFilter::filter()
  -> PathCellGeometryCertificate / cell-bound query
  -> TubeSurfaceValidator::validate()
  -> TubeEpochManager candidate/install decision
  -> finalizeTubeEpoch()
  -> Candidate / Certified Geometry / Active handoff gates
~~~

P1 modifies no authority decision in TubeEpochManager and no lifecycle
operation in the adapter.

## 7. Nominal interval and exactly one owner

Define:

~~~text
rho_nom > 0
I_nom(w) = [-rho_nom,+rho_nom]
~~~

The sole proposed authoritative owner is:

~~~text
phase_offset_navigation::TubeCrossSectionConfig::nominal_half_width
phase_offset/tube/nominal_half_width
~~~

The value must be finite and strictly positive. rho_nom = 1.0 m and rho_nom =
1.5 m are configurable experiment values, not constants.

rho_nom is not current delta, planner safe distance, UAV radius, uncertainty,
fixed_delta_max, max_offset, search_extent, or an unbounded search extent.

If a new and legacy field are both supplied, the new field is authoritative and
the conflict is deterministic and diagnosable. No hidden second owner exists.

## 8. Legacy field disposition

Actual current seams:

~~~text
PhaseOffsetMatchedAdapter::loadConfig()
  phase_offset/tube/fixed_delta_max
  phase_offset/tube/environment_search_extent
  planning/safe_distance
  phase_offset/tube/preincluded_map_uncertainty

TubeBuilderConfig
  fixed_delta_max
  max_offset
  cross_section.search_extent

TubeCrossSectionConfig
  search_extent
  planner_safe_distance
  margins.*
~~~

P1 classifications:

| Field | Classification | P1 rule |
|---|---|---|
| nominal_half_width | AUTHORITATIVE_NEW | sole nominal half-width |
| fixed_delta_max | LEGACY_COMPATIBILITY_ONLY / TEST_ONLY | cannot bound new ESDF path |
| max_offset | LEGACY_COMPATIBILITY_ONLY | cannot widen or replace rho_nom |
| environment_search_extent | UNRELATED_SEARCH_PARAMETER / DEPRECATED | cannot define nominal width |
| cross_section.search_extent | DEPRECATED_COMPATIBILITY_ONLY | bounded queries use rho_nom |
| planner_safe_distance | AUTHORITATIVE_PHYSICAL_CLEARANCE | Contract-A physical clearance |
| margins.uav_radius | DIAGNOSTIC_ONLY | never re-added to accepted inflation |
| margins.map_uncertainty | DIAGNOSTIC_ONLY | never double-counted |
| margins.localization_uncertainty | DIAGNOSTIC_ONLY | never double-counted |
| margins.tracking_error_bound | DIAGNOSTIC_ONLY | never double-counted |
| margins.preincluded_map_uncertainty | SNAPSHOT_CONTRACT_INPUT | proves required pre-inflation |

Legacy fields may remain visible for compatibility and diagnostics only.

## 9. Independent asymmetric clipping

At each valid path phase:

~~~text
open space:
  [-rho_nom,+rho_nom]

obstacle only on -N:
  [clipped_lower,+rho_nom]

obstacle only on +N:
  [-rho_nom,clipped_upper]

obstacles on both sides:
  [clipped_lower,clipped_upper]
~~~

The final interval is the intersection of the nominal interval, independent
obstacle bounds, full-3D regularity bounds, and continuous proof bounds.

Lower and upper may be asymmetric. An obstacle or unknown result on one side
does not manufacture an opposite-side restriction. Unknown, unavailable,
occupied, and out-of-map evidence follows the existing fail-closed model.

For exact zero, retain ZERO_CONNECTED. For finite nonzero current state,
preserve CURRENT_DELTA_CONNECTED when provable. Never cross an unsafe
disconnected gap to regain zero or widen the candidate.

## 10. Contract-A margin accounting

Freeze:

~~~text
MARGIN_ACCOUNTING=CONTRACT_A
~~~

Contract A requires:

1. cloud/voxel evidence is inflation-aware under the accepted immutable
   snapshot contract;
2. planning/safe_distance is the sole authoritative physical obstacle
   clearance for P1 Tube construction;
3. UAV radius, tracking, localization, map uncertainty, and equivalent legacy
   quantities are not added again when already represented by snapshot
   inflation;
4. visible legacy fields remain diagnostic or compatibility-only;
5. continuous proof may add only geometric cover for interpolation, path,
   normal, voxel, and swept-cell variation;
6. geometric cover is not a second physical margin;
7. missing, stale, insufficient, unavailable, revision-mismatched, or
   unprovable inflation evidence fails closed.

Exact current production seams:

~~~text
TubeCrossSectionConfig::planner_safe_distance
CloudOccupancySnapshot::included_map_inflation
plan_env::cloudOccupancySnapshotConsistent()
plan_env::queryCloudOccupancySnapshotClearance()
PhaseOffsetMatchedAdapter::loadConfig() -> planning/safe_distance
cloud_occupancy_query_config_.required_preincluded_map_uncertainty
~~~

Current TubeBuilder::buildCloudClearance records continuous_inset and uses a
cell certificate when eligible, otherwise a resolution-sized fallback.
P1 must charge this geometric residual exactly once. The validator's requested
radius, half-voxel support, interpolation, path variation, and normal
variation must be traceable. P1 must neither double-charge continuous_inset
and equivalent validator cover nor remove cover without proof.

## 11. Bounded Builder simplification

The intended normal path is:

~~~text
immutable path/snapshot input
  -> bounded nominal interval
  -> independent +N/-N clipping
  -> full-3D regularity intersection
  -> conservative local PWL representation
  -> PathCellGeometryCertificate where complete and revision-matched
  -> one final continuous SurfaceValidator pass if proof is sufficient
  -> bounded certified Candidate
~~~

Remove from the normal P1 path:

- nominal-external width exploration;
- maximum-width optimization;
- BOTH_SIDED alternate retry;
- POSITIVE_ONLY alternate retry;
- NEGATIVE_ONLY alternate retry;
- repeated scale halving;
- repeated alternate Candidate construction;
- repeated SurfaceValidator calls caused only by those alternatives.

One final SurfaceValidator pass is a proof-conditioned target, not a shortcut.
If insufficient, stop and specify a bounded proof-equivalent path; do not
restore unbounded or maximum-width search.

## 12. Continuous-certification obligations

Point samples are insufficient. A successful Candidate must prove:

- immutable path, source, frame, map, and task provenance;
- monotone phase ordering;
- independent directional knot bounds;
- conservative interpolation with no outward expansion;
- path and Horizontal-N variation over each cell;
- full-3D regularity over the complete interval;
- continuous swept-surface obstacle clearance;
- unknown/unavailable/out-of-map fail-closed semantics;
- current-anchor inclusion and certified forward-segment coverage.

PathCellGeometryCertificate may replace redundant geometric erosion only when
complete and revision-matched for every active cell and offset. It does not
replace obstacle-clearance proof. TubeSurfaceValidator remains required unless
a separately proven equivalent is authorized.

## 13. Current-delta transition contract

Keep these separate:

~~~text
rho_nom       configured requested half-width
lower/upper   actual geometry at w
delta_current authoritative retained transverse state
~~~

Critical test:

~~~text
delta_current = +1.2 m
rho_nom = 1.0 m
I_nom = [-1.0,+1.0]
~~~

The system must not clamp to +1.0 or reset to 0.0. Bounded geometry does not
expand beyond rho_nom. Existing accepted Recovery/Handoff/Authority retains
the authoritative reference while the candidate cannot take ownership. A new
epoch may take over only after valid re-entry through existing semantics.

Freeze:

~~~text
CURRENT_DELTA_TRANSITION=SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED
~~~

No production Recovery/Handoff edit, recovery-only Tube extension, or unsafe
gap convexification is authorized. If the test fails, stop and reclassify.

## 14. Zero/nonzero component behavior

The current implementation exposes:

~~~text
TubeComponentSelection::ZERO_CONNECTED
TubeComponentSelection::CURRENT_DELTA_CONNECTED
TubeEpochReason::CURRENT_OFFSET_OUTSIDE
TubeProfile::zero_component_contains_zero
TubeProfile::current_component_contains_delta
~~~

Preserve exact delta == 0.0 neutral behavior and finite nonzero behavior. Do
not introduce epsilon-based zero classification. A zero-only planner baseline
may remain valid when nonzero offset capacity is unavailable.

## 15. Exact P1 production whitelist

Only these production files may be modified by a separately authorized P1
execution:

~~~text
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
~~~

Allowed changes are limited to nominal_half_width validation/propagation,
bounded directional queries, connected-component/full-3D proof preservation,
normal retry removal, bounded diagnostics, and Contract-A accounting.

The adapter files are explicitly not authorized for scheduler, worker,
pending-slot, currentness, provenance, reset, deactivate, shutdown, or join
changes.

## 16. Exact P1 test and regression whitelist

Geometry tests, TEST_ONLY:

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
~~~

Transition contract tests, TEST_ONLY:

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
~~~

The first owns candidate/install and CURRENT_OFFSET_OUTSIDE assertions. The
second owns the end-to-end Runtime/authority/PathTubePair/publication seam.

Regression-only, LEAVE_UNCHANGED:

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/active_reference_authority_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/preview_feasibility_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/handoff_state_machine_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_recovery_owner_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
~~~

These are run as regressions and are not modified unless a concrete P1 omission
is separately demonstrated and authorized.

## 17. Launch/configuration disposition

Parameter loading is in:

~~~text
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
~~~

Existing launch paths:

~~~text
src/swarm_planner/bspline_traj/launch/test_gvf.launch
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
~~~

Disposition:

~~~text
test_gvf.launch:
  MODIFY ONLY IF REQUIRED to expose nominal_half_width as a launch argument;
  otherwise LEAVE_UNCHANGED.

phase_offset_esdf_tube_single.launch:
  LEAVE_UNCHANGED when test_gvf default is sufficient;
  MODIFY ONLY IF an explicit experiment value must be forwarded.
~~~

No P1 CMake or package metadata edit is expected. Launch edits may expose only
the new parameter and may not alter existing planner, simulator, map, or
Stage-1A settings.

## 18. Explicit P1 exclusions

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_recovery_owner.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/handoff_state_machine.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
all planner and governor sources
all Horizontal-N frame sources
phase_offset_swarm
phase_offset_msgs
phase_offset_allocator.*
phase_offset_cbf_constraints.*
all Preview production implementation
latest paper
Frozen Stage-1A Plan
master Roadmap and Rev1
AGENTS.md
all unrelated user-owned worktree artifacts
~~~

If P1 appears to require any excluded file, stop instead of widening scope.

## 19. Deterministic P1 geometry tests

The suite must deterministically cover:

1. open space rho_nom 1.0 gives approximately [-1.0,+1.0];
2. open space rho_nom 1.5 gives approximately [-1.5,+1.5];
3. -N-only obstacle clips lower only;
4. +N-only obstacle clips upper only;
5. two-sided obstacles clip independently;
6. asymmetric lower/upper values are preserved;
7. no directional query exceeds rho_nom;
8. no maximum-width search;
9. no normal three-family retry;
10. no normal repeated scale-halving;
11. side-specific full-3D regularity clipping;
12. unknown, unavailable, and out-of-map fail closed;
13. insufficient Contract-A inflation fails closed;
14. sloped 3-D centerline preserves Horizontal-N;
15. planner altitude remains unchanged;
16. continuous cell certification remains required;
17. one final SurfaceValidator target when proof suffices;
18. exact zero and tiny-nonzero behavior;
19. current-connected nonzero behavior;
20. no disconnected-gap convexification;
21. repeated immutable input is deterministic;
22. stale path/frame/source/map/task evidence cannot install;
23. Candidate/Active/Certified semantics remain unchanged;
24. rho_nom, bounds, and delta_current remain distinct;
25. +1.2 current delta is not clamped;
26. +1.2 current delta is not reset to zero;
27. +1.2 case has no unsafe epoch takeover;
28. retained authority remains consistent;
29. valid re-entry permits accepted takeover;
30. Stage-1A scheduling/currentness/lifecycle regressions remain green.

Assertions must check values, provenance, component selection, and reason
fields, not only a boolean result.

## 20. Current-delta end-to-end test requirements

Use the exact case:

~~~text
authoritative retained delta: +1.2 m
new nominal half-width: 1.0 m
new nominal interval: [-1.0,+1.0]
~~~

Prove:

- no clamp to +1.0;
- no reset to 0.0;
- TubeEpochManager reports an outside/retention decision;
- old authoritative pair/owner remains in force;
- no new Candidate/Active snapshot claims execution authority;
- Runtime/Recovery/Handoff retains the authoritative state;
- valid later re-entry can use existing takeover semantics;
- no unsafe gap is convexified;
- no production Recovery/Handoff source changes.

## 21. Contract-A deterministic tests

Explicitly test valid and sufficient snapshot inflation, invalid snapshots,
insufficient required preincluded inflation, revision/provenance mismatch,
missing cloud evidence, unknown/unavailable/out-of-map queries, one application
of planning/safe_distance, non-authoritative legacy margins, exactly-once
geometric cover, and fail-closed behavior. Diagnostics must distinguish
physical clearance, geometric cover, and compatibility values.

## 22. Performance and ROS acceptance

Compare:

~~~text
A = accepted/frozen Stage-1A baseline
B = Stage 1A + P1 bounded nominal Tube
~~~

Measure:

~~~text
Tube request/permit rate
worker build rate
build minimum, median, p90, p95, maximum
request-to-publication age
Candidate publication rate
Certified Geometry publication rate
stale pre-build rejection count
stale in-flight completion count
stale abandonment count
SurfaceValidator invocation count
normal inward-search attempt count
ESDF/cloud query count
maximum queried transverse offset
worker busy ratio
cmdCallback timing and rate
traj_vis rate
local_map rate
configured tube_update_period
~~~

Structural expectations:

~~~text
normal bounded inward-family attempts = 0
maximum queried transverse offset <= rho_nom
normal final SurfaceValidator count = 1 target when proof is sufficient
~~~

Do not promise a 1 ms build or publication guarantee. Show no regression in
commands, map, visualization, Runtime, Recovery/Handoff, R3, Horizontal-N,
planner path, or altitude.

## 23. Stage 1B gate

Stage 1B remains measurement-conditional and is not implemented in P1.
Recommend it only when substantial stale in-flight waste remains, such as high
build p95/max, frequent expensive stale builds, or validation recursion that
can be safely cancelled.

~~~text
SUCCESS    current geometry certified
FAILED     current geometry genuinely failed
CANCELLED  obsolete work stopped before a geometry/safety conclusion
~~~

If stale waste is insignificant, record STAGE1B_NOT_REQUIRED.

## 24. Future authorized implementation workflow

A future Main must:

1. read the Frozen Plan and verify its SHA;
2. verify the Stage-1A baseline and preserve user worktree changes;
3. obtain explicit implementation authorization;
4. prepare an exact implementation handoff;
5. launch exactly one Luna for the frozen P1 whitelist;
6. require Stage-1A worker/lifecycle preservation;
7. inspect diff and path scope;
8. run deterministic tests and all regressions;
9. run diff-check and hygiene checks;
10. launch a fresh Sol xhigh implementation audit;
11. perform only separately authorized bounded repairs;
12. perform ROS A/B acceptance;
13. decide Stage 1B only from measurements;
14. stop and report.

## 25. Stop conditions

Stop rather than widen scope if:

- rho_nom cannot have one authoritative owner;
- correctness requires querying beyond rho_nom;
- independent clipping cannot preserve continuous certification;
- Contract A causes double or omitted physical clearance;
- geometric cover cannot be charged exactly once;
- one validator pass is insufficient and no bounded equivalent is specified;
- unknown/unavailable/out-of-map evidence would become fail-open;
- the current-delta test fails accepted authority semantics;
- Recovery/Handoff production changes appear necessary;
- Stage-1A worker/pending/currentness/lifecycle changes appear necessary;
- TubeEpochManager authority must be rewritten;
- a required path lies outside the whitelist;
- unrelated user-owned files require modification;
- planner altitude or Horizontal-N would change;
- P1 starts solving Preview, swarm, QP/CBF, Batch C, or P3.

## 26. Candidate hygiene

Existing user-owned worktree entries must remain untouched:

~~~text
 M AGENTS.md
?? .codex
?? Testing/
?? docs/Codex_D-ISFGVF_Post-Stage1A_BoundedTube_Preview_BatchC_Roadmap_Plan_2026-08-27.md
?? docs/Codex_D-ISFGVF_Post-Stage1A_BoundedTube_Preview_BatchC_Roadmap_Plan_2026-08-27_Rev1.md
?? docs/Codex_Tube_Freshness_Stage1A_Implementation_Plan_2026-08-27.md
?? docs/D-ISFGVF_PhaseOffset_Tube_Architecture_Refactor_Brief_2026-08-23.md
?? docs/D-ISFGVF_Tube_Freshness_Stage1_Handoff_2026-08-26.md
?? src/swarm_planner/bspline_traj/test/__pycache__/
?? src/uav_simulator/so3_quadrotor_simulator/test/__pycache__/
~~~

Do not clean, reset, restore, stash, delete, stage, attribute, commit, or push
these entries as P1. /tmp ROS logs and evidence are not repository artifacts.

After Candidate creation, verify:

~~~text
branch main
HEAD cd313b315fcf4157273a7b0de84622073b3afaf6
origin/main cd313b315fcf4157273a7b0de84622073b3afaf6
d_isfgvf/main 5355be736339a650432280deec53362f61749a43
upstream d_isfgvf/main
ahead/behind 1/0
staged paths 0
git diff --check PASS
~~~

No staging, commit, push, tag, branch, remote mutation, or ROS launch is part
of Candidate creation.

## 27. Sol xhigh review checklist

A new independent Sol xhigh reviewer must inspect this exact Candidate path
and computed SHA read-only. It must verify:

- artifact identity and SHA;
- correct Stage-1A previously accepted/frozen history;
- no second Stage-1A gate and no reopened scheduling semantics;
- future P3 blockers excluded from P1;
- one rho_nom owner and exact open-space interval;
- independent asymmetric clipping;
- no symmetric shrink, external search, maximum-width objective, or repeated
  inward family;
- Contract-A physical clearance, no double/omitted margin, and fail-closed
  inflation evidence;
- geometric cover exactly once;
- PathCellGeometryCertificate and continuous proof;
- conditional one-validator target;
- exact-zero/tiny-nonzero and connected-component semantics;
- +1.2 transition test sufficiency and no Recovery redesign;
- adapter whitelist cannot change Stage-1A;
- exact production/test/regression/launch exclusions;
- deterministic matrix and ROS/performance evidence;
- Stage 1B conditionality.

For every finding, report severity, exact Candidate section, exact
source-file/function evidence, and smallest documentation correction.

Required report sections:

~~~text
ARTIFACT_IDENTITY
BASELINE_AND_STAGE1A_AUDIT
RHO_NOM_OWNER_AUDIT
ASYMMETRIC_CLIPPING_AUDIT
CONTRACT_A_MARGIN_AUDIT
BUILDER_SIMPLIFICATION_AUDIT
CONTINUOUS_CERTIFICATION_AUDIT
CURRENT_DELTA_TRANSITION_AUDIT
ZERO_NONZERO_COMPONENT_AUDIT
WHITELIST_AUDIT
TEST_MATRIX_AUDIT
ROS_PERFORMANCE_AUDIT
STAGE1B_BOUNDARY_AUDIT
STOP_CONDITION_AUDIT
MUST_FIX_BEFORE_FREEZE
SHOULD_FIX
OPTIONAL
FINAL_VERDICT
~~~

Allowed verdicts:

~~~text
P1_PLAN_ACCEPTABLE_READY_FOR_FREEZE
P1_PLAN_ACCEPTABLE_WITH_REQUIRED_CORRECTIONS_NOT_READY_FOR_FREEZE
P1_PLAN_NOT_ACCEPTABLE_NOT_READY_FOR_FREEZE
~~~

No verdict authorizes implementation.

## 28. Bounded documentation correction loop

If Sol accepts, proceed only to documentation-only Freeze. If Sol requires
documentation corrections, leave this artifact unchanged and create a new
Candidate_Rev1 document recording the superseded SHA. Recompute its SHA and
launch a new fresh Sol xhigh audit. At most two correction/re-review rounds are
allowed after this initial Candidate.

A blocker requiring Stage-1A, Runtime, Recovery, Handoff, planner, governor,
Horizontal-N, or a new safety architecture stops the workflow rather than
widening P1.

## 29. Execution boundary

This Candidate Plan does not authorize:

~~~text
production implementation
test implementation
launch/config edits
Stage 1B
Preview
Batch C or P3
Luna
commit
push
tag
branch
reset
restore
clean
stash
~~~

The next action is only a fresh Sol xhigh read-only audit of this exact
Candidate. If accepted, a separately authorized documentation-only Freeze may
create the Frozen Plan and then the self-contained new-window handoff.
