# D-ISFGVF P1 Bounded Nominal Asymmetric Tube — Candidate Rev3

~~~text
DOCUMENT_ROLE=P1_IMPLEMENTATION_PLAN
DOCUMENT_REVISION=REV3
DOCUMENT_STATUS=FROZEN / SOL XHIGH ACCEPTED
P1_SCOPE=BOUNDED_NOMINAL_ASYMMETRIC_TUBE
SUPERSEDES_CANDIDATE_PATH=docs/Codex_D-ISFGVF_P1_BoundedTube_Implementation_Plan_2026-08-27_Candidate_Rev2.md
SUPERSEDES_CANDIDATE_SHA256=44edba51e73a22a0e2be73d0d94d73bf170e06e2eab5633d8e377622a4fcf558
MASTER_REV1_REFERENCE_SHA256=e3d2818cb2f03c06c249af5fc420440a7a0eb719df4137d113f5b18789cbc69b
PREVIOUS_ACCEPTED_R3_HORIZONTAL_N_BASELINE=5355be736339a650432280deec53362f61749a43
STAGE1A_FROZEN_BASELINE=cd313b315fcf4157273a7b0de84622073b3afaf6
STAGE1A_ACCEPTANCE_STATUS=PREVIOUSLY ACCEPTED / FROZEN
STAGE1A_STALE_DIAGNOSTIC_CONTRACT=EXISTING_ACCEPTED_DIAGNOSTIC_EXCEPTION
STAGE1A_SHUTDOWN_CONTRACT=NO_PUBLICATION_AFTER_OWNERSHIP_FLIP
MARGIN_ACCOUNTING=CONTRACT_A
CURRENT_DELTA_TRANSITION=SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED
STAGE1B=MEASUREMENT_CONDITIONAL
REV3_CORRECTION_SCOPE=CONTRACT_A_COVER_AND_CONSTRUCTION_QUERY_ACCOUNTING_ONLY
SOURCE_ACCEPTED_CANDIDATE_PATH=docs/Codex_D-ISFGVF_P1_BoundedTube_Implementation_Plan_2026-08-27_Candidate_Rev3.md
SOURCE_ACCEPTED_CANDIDATE_SHA256=817243bd43e3742960d9a93d1a49db77301222989ce8e834f939e18426db59f3
SOL_P1_PLAN_AUDIT=P1_PLAN_ACCEPTABLE_READY_FOR_FREEZE
SOL_P1_PLAN_VERDICT=P1_PLAN_ACCEPTABLE_READY_FOR_FREEZE
P1_PLAN_FREEZE_DATE=2026-08-27
P1_IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
~~~

This is the complete self-contained Rev3 P1 Candidate Plan. It supersedes the
Rev2 Candidate for review but does not modify it. It contains only the two
documentation corrections requested by the final Rev2 Sol xhigh audit and
directly necessary cross-reference updates.
It does not implement P1, authorize Luna, reopen Stage 1A, or address P2/P3.

## 1. Authority, scope, and future-stage separation

P1 changes only the Tube construction objective and the minimum configuration,
diagnostic, test, and acceptance seams required to request one bounded nominal
horizontal interval. It preserves accepted scheduling, freshness, authority,
lifecycle, frame, planner, and execution architecture.

Authority order:

1. latest supplied D-ISFGVF paper;
2. accepted A/B/R3/Horizontal-N contracts;
3. frozen Stage-1A implementation at cd313b;
4. this exact Candidate Rev3 after the final fresh Sol acceptance;
5. a separately authorized P1 implementation handoff.

Explicitly out of scope:

~~~text
Stage 1A repair or redesign
Stage 1B cooperative cancellation
P2A tube_viability
P2B Preview integration and all P2 Preview production work
P3 and Batch C
swarm transport or swarm control mathematics
QP/CBF
Runtime, Recovery, Handoff, planner, governor, C2, H2, or R3 redesign
Horizontal-N redesign
~~~

Future P3 blockers in the master Roadmap are not P1 prerequisites.

## 2. Repository and baseline identity

~~~text
repository: /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
branch: main
HEAD: cd313b315fcf4157273a7b0de84622073b3afaf6
origin/main: cd313b315fcf4157273a7b0de84622073b3afaf6
d_isfgvf/main: 5355be736339a650432280deec53362f61749a43
configured upstream: d_isfgvf/main
ahead/behind against upstream: 1/0
~~~

~~~text
d_isfgvf  git@github.com:Accqqq/D-ISFGVF.git
origin    git@github.com:Accqqq/gvf_ws.git
lifted    git@github.com:Accqqq/Lifted_GVF.git
~~~

No remote mutation, merge, reset, rebase, staging, commit, or push is implied.

## 3. Stage-1A history is closed

The frozen Stage-1A commit is:

~~~text
cd313b315fcf4157273a7b0de84622073b3afaf6
feat: accept Stage 1A tube freshness scheduling
~~~

Independent Sol acceptance preceded commit/push. Original evidence:

~~~text
/home/cxq/.codex/sessions/2026/08/26/rollout-2026-08-26T22-45-17-01a03e88-5640-71f1-bdce-e09add5689d1.jsonl
~~~

The final review recorded STAGE_1A_IMPLEMENTATION_ACCEPTABLE, lifecycle,
lock-order, regression, and prior-invariant PASS, no blocking issue, and
PROCEED_TO_POST_REPAIR_REAL_ROS_ACCEPTANCE. Real ROS then recorded
STAGE_1A_REAL_ROS_ACCEPTED, followed by STAGE_1A_ACCEPTED_FROZEN_AND_PUSHED.

There is no second Stage-1A acceptance gate in P1. The four Stage-1A paths
remain protected:

~~~text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
~~~

The adapter paths are P1 configuration/propagation/diagnostic seams only.
Their worker, pending-slot, currentness, provenance, reset, deactivate,
shutdown, and join behavior must remain byte-for-byte semantically equivalent.

## 4. Protected Stage-1A contracts and inherited waiver

P1 preserves one scheduler-only production permit source, one joined worker,
one in-flight build, one latest-only pending slot, no FIFO, immutable request
identity, job-local state, stale pre-build rejection, stale completion
abandonment, task/source/path/frame/authority gates, Candidate/Certified
publication gates, reset/deactivate invalidation, shutdown ownership flip,
worker join, and no post-flip publication.

Accepted stale same-task raw/cloud attempt provenance remains permitted:

~~~text
STAGE1A_STALE_DIAGNOSTIC_CONTRACT=EXISTING_ACCEPTED_DIAGNOSTIC_EXCEPTION
~~~

It cannot mutate or expose Candidate authority, Certified Geometry, Active Tube,
Runtime, execution authority, authoritative profile/cache/epoch state, or
obsolete geometry.

Shutdown remains:

~~~text
STAGE1A_SHUTDOWN_CONTRACT=NO_PUBLICATION_AFTER_OWNERSHIP_FLIP
~~~

The exact inherited navigation exception is:

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
TubeSurfaceValidatorTest.ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel
PREEXISTING_NON_STAGE_1A
~~~

P1 neither repairs nor reclassifies it. A/B acceptance must show no additional
failure and no worsening of this named inherited result.

## 5. Frozen Horizontal-N geometry

~~~text
N(w) = normalize(e_z x p_w(w))
N_z = 0
r(w,delta) = p(w) + N(w) delta
r_z = p_z
||r_w(w,delta)|| >= m_r
~~~

The planner centerline and altitude remain full 3-D and planner-owned. The
fixed-w cross-section remains horizontal. Exact delta == 0.0 and finite
tiny-nonzero values remain distinct; no epsilon zero conversion is introduced.

## 6. P1 objective and actual dependency chain

Current code uses cross_section.search_extent, validates legacy
fixed_delta_max/max_offset, and CertifiedTubeBuilder retries
BOTH_SIDED/POSITIVE_ONLY/NEGATIVE_ONLY candidates with scale halving and
repeated SurfaceValidator calls.

P1 changes the normal objective to:

~~~text
certify the safe subset of one requested bounded nominal Tube
~~~

The actual call graph is:

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
  -> Candidate / Certified Geometry / Active gates
~~~

P1 does not modify TubeEpochManager production authority or adapter lifecycle.

## 7. Single rho_nom owner, default, and precedence

~~~text
rho_nom > 0
I_nom(w) = [-rho_nom,+rho_nom]
owner = TubeCrossSectionConfig::nominal_half_width
ROS parameter = phase_offset/tube/nominal_half_width
default when absent = 1.0 m
~~~

The default applies to ESDF P1 whenever the parameter is absent. A finite
1.5 m value is a supported experiment. Zero, negative, NaN, and infinity are
invalid and fail configuration closed.

Precedence is exact:

1. nominal_half_width always owns ESDF P1 geometry;
2. an explicit nominal value wins over every legacy width/search field;
3. absence still selects the 1.0 m default;
4. differing explicitly supplied legacy values emit a deterministic diagnostic;
5. legacy values never veto or widen valid ESDF P1 geometry.

Required diagnostics are:

~~~text
nominal_width_source = DEFAULT_ABSENT | EXPLICIT_PARAMETER
nominal_width_legacy_conflict = exact boolean over explicitly present legacy
  width/search parameters whose value differs from effective rho_nom
effective_nominal_half_width_m = rho_nom in metres
~~~

There is no hidden second nominal owner.

## 8. Source-specific validation and legacy disposition

The current generic validation must be split by source.

For ESDF, validate nominal_half_width and ESDF ray/boundary/regularity/
clearance configuration. Do not require fixed_delta_max, max_offset, or
deprecated search_extent, and do not let invalid legacy fields veto ESDF P1.

For FIXED/test compatibility, fixed_delta_max may retain fixed-source
validation. Legacy DistanceQuery max_offset may be validated only on that
legacy path. TubeCrossSectionSolver sample budget is derived from the bounded
nominal interval and ray_step, not deprecated search_extent.

The source-aware API is frozen:

~~~text
TubeCrossSectionSolver::configurationValid()
  validates nominal_half_width, ray_step, boundary_tolerance, regularity,
  planner_safe_distance, minimum_reference_speed, and the sample budget
  derived from 2*rho_nom/ray_step
  does not validate deprecated search_extent

TubeBuilder::configurationValidForSource(TubeSource source)
  ESDF: validates common + ESDF P1 fields only
  FIXED: additionally validates fixed_delta_max
  legacy DistanceQuery-only entry: validates max_offset only when invoked

CertifiedTubeBuilder::configurationValidForSource(TubeSource source)
  validates Filter/Validator common configuration
  delegates the exact source to TubeBuilder

CertifiedTubeBuilder::build(input)
  determines input.source before validation
  calls configurationValidForSource(input.source)
  never calls a generic all-source legacy gate first
~~~

No-argument configurationValid methods may remain only as source-neutral
helpers/tests; production ESDF build must use this source-aware path.

~~~text
nominal_half_width       AUTHORITATIVE_ESDF
fixed_delta_max          FIXED_SOURCE_ONLY / TEST_ONLY
max_offset               LEGACY_DISTANCE_QUERY_ONLY
environment_search_extent DEPRECATED_COMPATIBILITY_ONLY
cross_section.search_extent DEPRECATED_COMPATIBILITY_ONLY
planner_safe_distance    AUTHORITATIVE_PHYSICAL_CLEARANCE
RobustTubeMargins fields DIAGNOSTIC_ONLY
preincluded_map_uncertainty SNAPSHOT_EVIDENCE_REQUIREMENT
~~~

## 9. Absolute asymmetric clipping

The endpoints are absolute:

~~~text
lower_nom = -rho_nom
upper_nom = +rho_nom
~~~

They are not per-ray travel distances from a selected seed. Required behavior:

~~~text
open:                 [-rho_nom,+rho_nom]
-N obstacle only:     [clipped_lower,+rho_nom]
+N obstacle only:     [-rho_nom,clipped_upper]
both-side obstacles:  [clipped_lower,clipped_upper]
~~~

Lower and upper are independent. No scalar symmetric shrink, maximum-width
objective, nominal-external search, or unsafe-gap convexification is allowed.

## 10. Construction-query and current-safety-query separation

Bounded construction/proof work has five non-overlapping query families:

~~~text
cross_section_directional_query_count
  = every ClearanceQuery call made by TubeCrossSectionSolver construction,
    including its nominal-domain centre and directional samples

adaptive_refinement_centerline_query_count
  = the three ClearanceQuery calls made by each AdaptiveNeedsSubdivision()
    invocation that reaches its clearance-comparison block

adaptive_sample_base_clearance_query_count
  = the one base-centreline ClearanceQuery made by buildCloudClearance() for
    each adaptive sample that reaches cross-section construction

certified_cell_bound_query_count
  = every PathCellBoundQuery proof callback used for the bounded Candidate;
    report Builder BuildCertifiedCellInsets() and Validator
    CertifiedCellCoverRadius() sub-counts separately and sum them once

validator_surface_query_count
  = TubeSurfaceValidationResult::query_sample_count, the exact number of
    underlying Validator surface ClearanceQuery calls
~~~

For a construction clearance query with an explicit transverse centre:

~~~text
abs(delta_query) <= rho_nom
~~~

Adaptive refinement and adaptive-sample base-clearance queries are centreline
queries with delta=0. PathCellBoundQuery is a phase-cell geometry proof query,
not a directional ray, so it is not assigned a fabricated scalar delta. Its
certificate must correspond to the same revision-matched, nominal-bounded
Candidate. No construction/proof family intentionally investigates transverse
Tube width outside the requested nominal domain.

TubeEpochManager may separately inspect the unchanged retained reference and
actual UAV position for current-state safety. Those are labelled retained-
reference/actual-position safety queries, not width searches, and are measured
separately. They cannot change rho_nom, clamp delta, or install an invalid
candidate.

There is a third post-build safety category:

~~~text
base_centerline_safety_query =
  TubeEpochManager::update() CheckClearance(input.current_path.p)
~~~

Command-time LatestCategoricalUnsafe reference/actual queries are a fourth
category. They are categorical freshness evidence, not Tube construction or a
per-build query gate. Their existing categorical-evidence path remains
unchanged; they are excluded from both construction and current-safety totals.
They are counted separately only when the existing measurement path can identify
them, and their absence from a whitelisted collector is not filled by a new
instrumentation seam.

Required metrics therefore distinguish:

~~~text
max_bounded_construction_abs_delta <= rho_nom
cross_section_directional_query_count
adaptive_refinement_centerline_query_count
adaptive_sample_base_clearance_query_count
builder_certified_cell_bound_query_count
validator_certified_cell_bound_query_count
certified_cell_bound_query_count
validator_surface_query_count
total_tube_construction_query_count
total_tube_construction_clearance_query_count
retained_reference_safety_query_count
actual_position_safety_query_count
base_centerline_safety_query_count
~~~

Optional existing-path evidence may additionally report
command_categorical_reference_query_count and
command_categorical_actual_query_count, but neither is a mandatory P1
construction/per-build gate.

The exact accounting identities are:

~~~text
certified_cell_bound_query_count =
  builder_certified_cell_bound_query_count
  + validator_certified_cell_bound_query_count

total_tube_construction_clearance_query_count =
  cross_section_directional_query_count
  + adaptive_refinement_centerline_query_count
  + adaptive_sample_base_clearance_query_count
  + validator_surface_query_count

total_tube_construction_query_count =
  total_tube_construction_clearance_query_count
  + certified_cell_bound_query_count

total_current_safety_queries =
  retained_reference + actual_position + base_centerline
command categorical queries are separate and are not part of either total
~~~

Each underlying callback invocation is routed to exactly one leaf count. The
two certified-cell proof sub-counts are not added directly again after their
aggregate is formed. Current/post-build safety and command categorical evidence
are never included in construction totals or in one ambiguous transverse
maximum.

## 11. Out-of-nominal current delta and exact component labels

For:

~~~text
delta_current = +1.2
rho_nom = 1.0
I_nom = [-1.0,+1.0]
~~~

P1 retains +1.2 unchanged in metadata, uses exact zero as the bounded
construction anchor, performs all Tube construction/Validator queries within
[-1,+1], sets current_component_contains_delta=false, and lets
TubeEpochManager report CURRENT_OFFSET_OUTSIDE while retaining the prior active
epoch/pair/authority. Separately labelled current-safety checks may inspect the
retained reference. No clamp and no reset are allowed. This construction
anchor rule applies even when the retained reference is evaluated separately
for safety.

Component contract:

~~~text
delta == 0.0
  -> ZERO_CONNECTED
finite nonzero inside I_nom
  -> CURRENT_DELTA_CONNECTED only when actually proven
finite nonzero outside I_nom
  -> bounded ZERO_CONNECTED construction
  -> current_component_contains_delta=false
  -> no false CURRENT_DELTA_CONNECTED label
~~~

An open-space tiny-nonzero test must still produce both absolute nominal
endpoints.

The component-label owner is the already whitelisted
CertifiedTubeBuilder::build(). It computes construction_delta as retained
delta only when finite and inside the nominal interval, otherwise exact zero;
it invokes Builder and Filter with construction_delta, restores unchanged
retained current_delta metadata, and forces out-of-range results to
ZERO_CONNECTED with current_component_contains_delta=false. tube_filter.cpp
is not modified.

## 12. Contract-A margin and exactly-once geometric cover

~~~text
MARGIN_ACCOUNTING=CONTRACT_A
physical clearance = planning/safe_distance exactly once
geometric proof cover = one traceable residual exactly once
~~~

Exact seams:

~~~text
TubeCrossSectionConfig::planner_safe_distance
plan_env::CloudOccupancySnapshot::included_map_inflation
plan_env::cloudOccupancySnapshotConsistent()
plan_env::queryCloudOccupancySnapshotClearance()
PhaseOffsetMatchedAdapter::loadConfig() -> planning/safe_distance
cloud_occupancy_query_config_.required_preincluded_map_uncertainty
~~~

Snapshot inflation must be immutable, sufficient, available, and
revision/provenance matched or P1 fails closed. RobustTubeMargins values are
diagnostic/compatibility only. They cannot re-add UAV, tracking,
localization, or map physical margins.

The exact residual ownership is frozen:

| mode | Builder continuous_inset | Validator cover C | requested clearance |
|---|---:|---:|---:|
| complete matching cell certificate | 0 | `certificate.midpoint_position_variation_bound` + 0.5*`maximum_delta`*`certificate.normal_variation_bound` + 0.5*`delta_slope`*`h` + 0.5*`maximum_width`*`v_span` + 0.5*`snapshot_resolution` | planner_safe_distance + C + cover_epsilon |
| sampled fallback, nondegenerate cell | 0 | 1.1*sampled_radius + 0.5*snapshot_resolution | planner_safe_distance + C + cover_epsilon |
| sampled fallback, exact current-anchor cell | 0 | 1.1*sampled_radius | planner_safe_distance + C + cover_epsilon |
| certificate attempted but regularity/cell proof fails | no fallback | no accepted cover | fail closed |

P1 changes TubeBuilder so continuous_inset is zero metadata on the ESDF
candidate. TubeSurfaceValidator owns the single geometric cover, exactly as
implemented by `CertifiedCellCoverRadius()`: midpoint position variation,
normal variation scaled by maximum delta, delta-slope variation, transverse
width variation (`maximum_width*v_span`), and the half-voxel support. The
half-voxel support is charged once except for the proven exact-anchor
degenerate branch, and planner_safe_distance is charged once. The Validator
production implementation and `CertifiedCellCoverRadius()` remain
LEAVE_UNCHANGED; P1 documents their existing formula and does not remove any
geometric term for performance. A certificate failure never silently falls
back to sampled cover.

## 13. Bounded Builder and continuous proof

Normal flow:

~~~text
immutable bounded request
  -> absolute nominal interval
  -> independent +N/-N clipping
  -> full-3D regularity intersection
  -> conservative local PWL bounds
  -> revision-matched PathCellGeometryCertificate
  -> one final SurfaceValidator invocation when proof suffices
  -> one bounded Candidate
~~~

Remove from the normal path nominal-external exploration, maximum-width
optimization, all three alternate inward families, repeated scale halving,
alternate Candidate construction, and their repeated Validator calls.

One Validator is a proof-conditioned target. If that invocation fails, the
deterministic result is:

~~~text
preserve raw/proof failure provenance;
if zero centreline is continuously covered, collapse offset capacity to the
complete ZERO_ONLY_PLANNER_BASELINE;
otherwise return an incomplete fail-closed Candidate;
never run an alternate inward family or scale-halving retry.
~~~

## 14. Exact production whitelist

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

Allowed edits are nominal width ownership/default/validation/propagation,
bounded directional construction, component/proof preservation, removal of
normal retries, bounded diagnostics, and Contract-A accounting. Adapter edits
cannot touch Stage-1A scheduling or lifecycle.

## 15. Exact test and regression whitelist

Geometry tests — TEST_ONLY:

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
~~~

Current-delta transition — TEST_ONLY:

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
~~~

Contract-A/evidence regressions — REGRESSION_ONLY / LEAVE_UNCHANGED:

~~~text
src/swarm_planner/plan_env/test/cloud_occupancy_snapshot_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_cloud_occupancy_query_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_raw_candidate_diagnostics_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_diagnostics_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp
src/swarm_planner/bspline_traj/test/continuous_phase_normal_frame_test.cpp
src/swarm_planner/bspline_traj/test/continuous_phase_path_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/active_reference_authority_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/preview_feasibility_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/handoff_state_machine_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_recovery_owner_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
~~~

The named inherited validator failure in section 4 remains the only permitted
pre-existing failure; it is compared in A/B and never silently waived.

## 16. Exact launch/configuration whitelist

The new parameter is loaded in:

~~~text
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
~~~

Both real acceptance launch files are explicitly in the P1 whitelist:

~~~text
MODIFY
src/swarm_planner/bspline_traj/launch/test_gvf.launch
  default phase_offset_tube_nominal_half_width=1.0 and forward it as
  phase_offset/tube/nominal_half_width

MODIFY
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
  default and forward phase_offset_tube_nominal_half_width=1.0;
  1.5 is an explicit override experiment only
~~~

Both launches also expose/forward the existing measurement parameters without
changing their owning implementation:

~~~text
launch args:
  phase_offset_measurement_enable (default=false)
  phase_offset_measurement_callback_csv_path (default="")
  phase_offset_measurement_tube_due_csv_path (default="")
forwarded params:
  phase_offset/measurement/enable
  phase_offset/measurement/callback_csv_path
  phase_offset/measurement/tube_due_csv_path
~~~

No other launch, CMake, or package file is modified by P1 unless a concrete
dependency is proved and separately authorized. Existing planner, simulator,
map, and Stage-1A settings remain unchanged.

## 17. Deterministic test matrix

Required tests assert values, provenance, component labels, and reason fields:

1. open rho_nom=1.0 gives [-1,+1];
2. open rho_nom=1.5 gives [-1.5,+1.5];
3. -N-only, +N-only, and both-side independent clipping;
4. asymmetric bounds and absolute endpoint semantics;
5. no construction/Validator query beyond rho_nom;
6. no maximum-width or three-family retry;
7. no normal scale halving;
8. full-3D regularity and unchanged planner altitude;
9. unknown, unavailable, out-of-map, and insufficient inflation fail closed;
10. continuous cell proof remains required;
11. one Validator target when proof suffices;
12. exact zero ZERO_CONNECTED;
13. tiny finite nonzero remains nonzero;
14. in-range nonzero CURRENT_DELTA_CONNECTED only when proven;
15. out-of-range +1.2 gives bounded evidence and false current-component flag;
16. immutable repeated input is deterministic;
17. stale path/frame/source/map/task cannot install;
18. Candidate/Active/Certified semantics remain unchanged;
19. no disconnected-gap convexification;
20. default/absence rho_nom=1.0;
21. zero/negative/NaN/infinity nominal values reject;
22. explicit new-versus-legacy precedence and diagnostic;
23. invalid legacy values do not veto valid ESDF;
24. +1.2 construction queries remain bounded while separately labelled
    current-safety queries may inspect the retained reference;
25. no clamp, reset, or unsafe takeover;
26. retained authority and valid later re-entry;
27. inherited validator exception remains exact and no new failure appears;
28. all Stage-1A regressions remain green otherwise;
29. base-centerline safety-query classification/count is distinct from
    retained-reference and actual-position safety;
30. nominal_width_source is exactly DEFAULT_ABSENT or EXPLICIT_PARAMETER,
    conflict is an exact boolean, and effective width is in metres;
31. the Contract-A residual table is preserved term-for-term, including
    `0.5*maximum_width*v_span`, the exact-current-anchor branch, the unchanged
    Validator owner, and the no-fallback proof-failure result;
32. a sole Validator failure produces exactly the frozen complete-zero or
    incomplete fail-closed outcome;
33. both acceptance launches default nominal_half_width to 1.0 m, with 1.5 m
    only as an explicit override;
34. each construction/proof query family is routed exactly once: cross-section
    directional, adaptive-refinement centreline, adaptive-sample base
    clearance, Builder and Validator certified-cell-bound proof sub-counts,
    and Validator surface clearance; the aggregate equals the exact sum, no
    family is omitted, and neither current-safety nor command-categorical
    evidence is included.

## 18. Current-delta end-to-end contract

Use authoritative retained delta +1.2 m, nominal half-width 1.0 m, and
nominal interval [-1.0,+1.0]. Use exact zero only as a bounded construction
anchor when the retained delta is outside the nominal interval. Retain +1.2
unchanged in request/epoch/authority metadata. TubeEpochManager reports
CURRENT_OFFSET_OUTSIDE and retains the previous active pair/owner. Later valid
re-entry may use existing accepted takeover semantics. No Recovery/Handoff
production source changes are permitted.

## 19. Observable performance and ROS acceptance

P1 must persist and expose these facts through the whitelisted
TubeBuildDiagnostics/adapter measurement path:

~~~text
Tube request/permit rate
worker build rate
build min/median/p90/p95/max
request-to-publication age
Candidate publication rate
Certified Geometry publication rate
stale pre-build rejection
stale in-flight completion
stale abandonment
SurfaceValidator invocation count
inward-search attempt count
cross_section_directional_query_count
adaptive_refinement_centerline_query_count
adaptive_sample_base_clearance_query_count
builder_certified_cell_bound_query_count
validator_certified_cell_bound_query_count
certified_cell_bound_query_count
validator_surface_query_count
total_tube_construction_clearance_query_count
total_tube_construction_query_count
max_bounded_construction_abs_delta
retained-reference safety query count
actual-position safety query count
base-centerline safety query count
worker busy ratio
cmdCallback timing/rate
traj_vis rate
local_map rate
tube_update_period
~~~

The exact routing and aggregation contract is:

| metric | producer | raw field | handoff/payload | collector | aggregation |
|---|---|---|---|---|---|
| cross-section directional clearance count and maximum explicit transverse offset | TubeCrossSectionSolver / TubeBuilder | proposed P1 `TubeBuildDiagnostics::cross_section_directional_query_count`, `max_bounded_construction_abs_delta` | Builder profile diagnostics -> CertifiedTubeBuilder -> TubeEpochManager candidate/result -> adapter | existing tube diagnostics/measurement route | per build; sum count and maximum absolute delta over the complete run |
| adaptive refinement centreline clearance count | `AdaptiveNeedsSubdivision()` via TubeBuilder-owned wrapped ClearanceQuery | proposed P1 `TubeBuildDiagnostics::adaptive_refinement_centerline_query_count` | same Builder profile diagnostic handoff | existing tube diagnostics/measurement route | actual underlying calls; each reached clearance-comparison block contributes exactly three |
| adaptive-sample base-clearance count | `TubeBuilder::buildCloudClearance()` | proposed P1 `TubeBuildDiagnostics::adaptive_sample_base_clearance_query_count` | same Builder profile diagnostic handoff | existing tube diagnostics/measurement route | one for each adaptive sample reaching base/cross-section evaluation |
| certified-cell-bound proof counts | `BuildCertifiedCellInsets()` and unchanged Validator certificate callback, counted by CertifiedTubeBuilder wrapper | proposed P1 Builder/CertifiedBuilder sub-counts plus `TubeBuildDiagnostics::certified_cell_bound_query_count` | CertifiedTubeBuilder profile diagnostics -> epoch result -> adapter | existing tube diagnostics/measurement route | sum Builder and Validator callback invocations once; retain both sub-counts |
| Validator surface-clearance count | unchanged TubeSurfaceValidator | `TubeSurfaceValidationResult::query_sample_count` copied to proposed P1 `TubeBuildDiagnostics::validator_surface_query_count` | CertifiedTubeBuilder result/profile diagnostics -> epoch result -> adapter | existing tube diagnostics/measurement route | exact underlying surface ClearanceQuery calls |
| total construction counts | TubeBuilder / CertifiedTubeBuilder | proposed P1 `total_tube_construction_clearance_query_count`, `total_tube_construction_query_count` | same profile/result handoff | existing tube diagnostics/measurement route | exact identities below; never mix safety/categorical evidence |
| Builder/Validator invocation, inward-attempt, and bounded max diagnostics | TubeBuilder / CertifiedTubeBuilder | proposed P1 `TubeBuildDiagnostics` fields | CertifiedTubeBuilder profile diagnostics -> TubeEpochManager candidate/result -> adapter | existing tube diagnostics/measurement route | per build; counts/maxima over the complete run |
| duration, finalized status, raw/cloud attempt, and new P1 timing fields | adapter finalization | `TubeDueTimingSample` | TubeEpochManager result -> adapter due sample | existing tube-due CSV | one row per due/build completion; percentiles over complete-run rows |
| request, permit, build, Candidate, Certified, stale, worker-busy counters | adapter scheduler/finalization | existing adapter counters/CSV fields | adapter measurement state | existing adapter measurement CSV | count/rate over the complete run; busy ratio = busy time / run time |
| `cmdCallback` timing/rate | unchanged `gvf_manager.cpp` callback recorder | existing callback timing fields | existing measurement CSV selected by `phase_offset/measurement/callback_csv_path` | gvf-manager callback CSV | complete-run count, rate, min/median/p90/p95/max |
| `traj_vis`, `local_map`, `position_cmd` rates | ROS topics | topic timestamps | no Tube payload | external `rostopic hz` capture | steady-state rate over the declared observation window |
| `tube_update_period` | ROS configuration | launch/rosparam value | parameter server | launch/config record | fixed configured value, not inferred from build rate |
| retained-reference, actual-position, base-centerline safety counts | TubeEpochManager result/status conditions | existing result/status evidence | epoch result/status -> adapter only where already observable | existing diagnostics/CSV | per run; never merged with construction total |
| command categorical reference/actual counts | existing `LatestCategoricalUnsafe()` categorical path | existing categorical evidence only | existing path if already surfaced | existing collector if available | per run; no new seam in P1 |

The mandatory identities are:

~~~text
certified_cell_bound_query_count =
  builder_certified_cell_bound_query_count
  + validator_certified_cell_bound_query_count

total_tube_construction_clearance_query_count =
  cross_section_directional_query_count
  + adaptive_refinement_centerline_query_count
  + adaptive_sample_base_clearance_query_count
  + validator_surface_query_count

total_tube_construction_query_count =
  total_tube_construction_clearance_query_count
  + certified_cell_bound_query_count

total_current_safety_queries =
  retained_reference + actual_position + base_centerline
~~~

Every actual query callback belongs to one leaf construction/proof count. The
aggregate certified-cell count is a sum of its two sub-counts, not an extra
callback family. Builder/Validator/inward/query/max-offset diagnostics are not
silently reinterpreted as current-safety counts. `cmdCallback` reuses the unchanged
gvf-manager CSV; `traj_vis`, `local_map`, and `position_cmd` are measured
externally. If an item has no producer, payload, and collector inside this
whitelist, it is removed from the mandatory acceptance gate rather than
invented.

Exact A/B protocol:

1. Use the same pillar/map snapshot and identical five-goal sequence for A and
   B, with the same start/reset procedure and observation window.
2. A is the accepted/frozen Stage-1A baseline. B is Stage 1A + P1 with
   `phase_offset/tube/nominal_half_width=1.0` m; 1.5 m is not the acceptance
   value and may be run only as an explicit override experiment.
3. Record complete run duration, all due/build rows, CSV paths, and external
   topic-rate windows. Do not discard startup or shutdown rows when computing
   count identities.
4. Check approximately 50 Hz `position_cmd`, approximately 10 Hz `local_map`
   and `traj_vis`, and compare request/publication ages, Candidate/Certified
   counts, stale outcomes, query categories, and worker busy ratio using the
   routing table above.
5. Require no crash, deadlock, worker stall, FIFO, 50 Hz rebuild churn,
   callback coupling, stale resurrection, or new test failure. The inherited
   fixture is the only permitted known exception.

Compare:

~~~text
A = accepted/frozen Stage-1A baseline
B = Stage 1A + P1 with nominal_half_width=1.0 m
~~~

Use the accepted approximate references of 50 Hz command and 10 Hz map/traj
publication, with no crash, deadlock, worker stall, FIFO, callback coupling,
stale authority resurrection, or new test failure. No 1 ms promise is allowed.

Reproducible build/regression sequence from the workspace root is:

~~~bash
source devel/setup.bash
catkin_make -j2
catkin_make run_tests_phase_offset_core
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_bspline_race
~~~

The named P1 targets are then run directly from `devel/lib` (or through the
corresponding `run_tests_*` target):

~~~text
phase_offset_tube_cross_section_test
phase_offset_tube_builder_test
phase_offset_certified_tube_builder_test
phase_offset_tube_filter_test
phase_offset_tube_surface_validator_test
phase_offset_tube_epoch_manager_test
phase_offset_matched_adapter_test
~~~

For A/B, launch the simulator once per isolated run, then launch
`bspline_race test_gvf.launch` with the same five goals and map; set
`phase_offset_mode:=manual`, `phase_offset_manual_tube_source:=esdf`,
`phase_offset_tube_nominal_half_width:=1.0`,
`phase_offset_measurement_enable:=true`, and distinct values for
`phase_offset_measurement_callback_csv_path` and
`phase_offset_measurement_tube_due_csv_path`. Capture
`rostopic hz /position_cmd`, `/sim/local_map`, and
`/particle0/gvf/traj_vis`, retain the complete CSVs, and stop only processes
started by the run.

## 20. Stage 1B gate

~~~text
STAGE1B=MEASUREMENT_CONDITIONAL
~~~

Do not implement Stage 1B in P1. Consider it only if measured stale
in-flight work or validation/query cost remains substantial and safe
cooperative cancellation can save computation.

~~~text
SUCCESS = current geometry certified
FAILED = current geometry genuinely failed
CANCELLED = obsolete work stopped before a geometry/safety conclusion
~~~

If stale waste is insignificant, record STAGE1B_NOT_REQUIRED.

## 21. Explicit exclusions and stop conditions

P1 excludes TubeSurfaceValidator production implementation, TubeEpochManager,
Runtime, RecoveryOwner, Handoff, gvf_manager, planner/governor, Horizontal-N,
phase_offset_swarm, phase_offset_msgs, all P2A/P2B Preview work, QP/CBF,
latest paper, Frozen Stage-1A Plan, master Roadmap/Rev1, AGENTS.md, and all
unrelated user-owned artifacts.

Stop if:

- nominal_half_width cannot be the sole ESDF owner/default;
- invalid legacy fields can veto ESDF;
- a construction/Validator query must exceed rho_nom;
- current-state safety cannot be separated from construction metrics;
- Contract A double-counts or omits clearance;
- geometric cover cannot be charged once;
- continuous proof or bounded one-pass validation is insufficient;
- any evidence path becomes fail-open;
- component labels are false out of range;
- the current-delta test fails accepted authority;
- Stage-1A semantics or excluded production files must change;
- required metrics cannot be observed within the whitelist;
- P1 starts solving P2, P3, swarm, QP/CBF, or another stage;
- unrelated user files require modification.

## 22. Hygiene and bounded revision record

The initial Candidate remains unchanged:

~~~text
docs/Codex_D-ISFGVF_P1_BoundedTube_Implementation_Plan_2026-08-27_Candidate.md
SHA256=5129728ee6ac5b16123e235347a1df98eec84c865eb4df5420ab7903cecc2f14
~~~

Rev3 is the only new file in this documentation correction round. Existing
AGENTS.md, .codex, Testing/, prior Roadmap and Stage-1A documents,
__pycache__ directories, and all other user-owned changes must not be cleaned,
reset, restored, stashed, deleted, staged, attributed, committed, or pushed.

After Rev3, compute its exact SHA and byte size; verify branch, HEAD, remotes,
status, staged paths zero, and git diff --check. Do not stage or commit.

Rev3 is the final authorized documentation correction round. No Rev4 is
permitted. A blocker requiring code or architecture expansion stops the
workflow.

## 23. Rev2 -> Rev3 semantic delta report

Changed sections are limited to §1 metadata, §10 construction-query taxonomy
and identities, §12 Contract-A matching-certificate cover wording, §17 test
items 31 and 34, §19 metric routing/identities, and this review boundary.

Blocker 1 is closed in the plan by documenting the unchanged
`CertifiedCellCoverRadius()` matching-certificate cover exactly, including
`midpoint_position_variation_bound`, `0.5*maximum_delta*normal_variation_bound`,
`0.5*delta_slope*h`, `0.5*maximum_width*v_span`, and
`0.5*snapshot_resolution`; `TubeSurfaceValidator` remains LEAVE UNCHANGED.

Blocker 2 is closed in the plan by replacing the coarse query identity with
five explicit, non-overlapping families: cross-section directional,
`AdaptiveNeedsSubdivision()` refinement centreline, adaptive-sample base
clearance, Builder/Validator certified-cell-bound proof sub-counts, and
Validator surface clearance. The exact aggregate identities and safety/query
exclusions are frozen in §§10 and 19.

No other P1 contract changed. In particular, rho_nom ownership/default,
absolute asymmetric clipping, current-delta semantics, Stage-1A boundary,
whitelist scope, Stage 1B boundary, and P2/P3 exclusion are unchanged from
Rev2.

## 24. Sol xhigh Rev3 strict delta-closure review

A new independent Sol xhigh reviewer must inspect only the Rev2 -> Rev3 delta,
`CertifiedCellCoverRadius()`, `AdaptiveNeedsSubdivision()`,
`buildCloudClearance()`, `BuildCertifiedCellInsets()`, and directly relevant
query/diagnostic structures. Previously accepted P1 contracts are inherited and
must not be reopened.

Required sections:

~~~text
ARTIFACT_IDENTITY
REV2_TO_REV3_DELTA_AUDIT
CONTRACT_A_COVER_CLOSURE
CONSTRUCTION_QUERY_ACCOUNTING_CLOSURE
REV3_REGRESSION_CHECK
MUST_FIX_BEFORE_FREEZE
FINAL_VERDICT
~~~

MUST_FIX may contain findings only from the two specified blockers or a direct
Rev3-induced contradiction. No unrelated new review findings are in scope.

Allowed final verdicts:

~~~text
P1_PLAN_ACCEPTABLE_READY_FOR_FREEZE
P1_PLAN_ACCEPTABLE_WITH_REQUIRED_CORRECTIONS_NOT_READY_FOR_FREEZE
P1_PLAN_NOT_ACCEPTABLE_NOT_READY_FOR_FREEZE
~~~

## 25. Execution boundary

This Rev3 Candidate does not authorize production or test implementation,
launch/config edits, Stage 1B, P2A/P2B, P3, Batch C, Luna, staging, commit,
push, tag, branch, reset, restore, clean, or stash. The next action is only a
fresh Sol xhigh strict delta-closure read-only audit of this exact Rev3 hash. If accepted, a
separately authorized documentation-only Freeze may create the Frozen Plan
and then the self-contained new-window handoff.
