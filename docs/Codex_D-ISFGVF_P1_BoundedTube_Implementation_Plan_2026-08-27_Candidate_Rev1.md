# D-ISFGVF P1 Bounded Nominal Asymmetric Tube — Candidate Rev1

~~~text
DOCUMENT_ROLE=P1_IMPLEMENTATION_PLAN
DOCUMENT_REVISION=REV1
DOCUMENT_STATUS=CANDIDATE / READY FOR SOL XHIGH RE-REVIEW
P1_SCOPE=BOUNDED_NOMINAL_ASYMMETRIC_TUBE
SUPERSEDES_CANDIDATE_PATH=docs/Codex_D-ISFGVF_P1_BoundedTube_Implementation_Plan_2026-08-27_Candidate.md
SUPERSEDES_CANDIDATE_SHA256=5129728ee6ac5b16123e235347a1df98eec84c865eb4df5420ab7903cecc2f14
MASTER_REV1_REFERENCE_SHA256=e3d2818cb2f03c06c249af5fc420440a7a0eb719df4137d113f5b18789cbc69b
PREVIOUS_ACCEPTED_R3_HORIZONTAL_N_BASELINE=5355be736339a650432280deec53362f61749a43
STAGE1A_FROZEN_BASELINE=cd313b315fcf4157273a7b0de84622073b3afaf6
STAGE1A_ACCEPTANCE_STATUS=PREVIOUSLY ACCEPTED / FROZEN
STAGE1A_STALE_DIAGNOSTIC_CONTRACT=EXISTING_ACCEPTED_DIAGNOSTIC_EXCEPTION
STAGE1A_SHUTDOWN_CONTRACT=NO_PUBLICATION_AFTER_OWNERSHIP_FLIP
MARGIN_ACCOUNTING=CONTRACT_A
CURRENT_DELTA_TRANSITION=SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED
STAGE1B=MEASUREMENT_CONDITIONAL
SOL_P1_PLAN_AUDIT=PENDING_REV1_REVIEW
P1_IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
~~~

This is the complete self-contained Rev1 P1 Candidate Plan. It supersedes the
initial Candidate for review but does not modify it. It contains only the
documentation corrections requested by the first fresh Sol xhigh audit.
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
4. this exact Rev1 after fresh Sol acceptance;
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

Required diagnostics are nominal_width_source, nominal_width_legacy_conflict,
and effective_nominal_half_width. There is no hidden second nominal owner.

## 8. Source-specific validation and legacy disposition

The current generic validation must be split by source.

For ESDF, validate nominal_half_width and ESDF ray/boundary/regularity/
clearance configuration. Do not require fixed_delta_max, max_offset, or
deprecated search_extent, and do not let invalid legacy fields veto ESDF P1.

For FIXED/test compatibility, fixed_delta_max may retain fixed-source
validation. Legacy DistanceQuery max_offset may be validated only on that
legacy path. TubeCrossSectionSolver sample budget is derived from the bounded
nominal interval and ray_step, not deprecated search_extent.

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

Bounded construction/proof queries include cross-section rays, Builder samples,
and Validator surface centers. Every such transverse center must satisfy:

~~~text
abs(delta_query) <= rho_nom
~~~

TubeEpochManager may separately inspect the unchanged retained reference and
actual UAV position for current-state safety. Those are labelled retained-
reference/actual-position safety queries, not width searches, and are measured
separately. They cannot change rho_nom, clamp delta, or install an invalid
candidate.

Required metrics therefore distinguish:

~~~text
max_bounded_construction_abs_delta <= rho_nom
retained_reference_safety_query_count
actual_position_safety_query_count
~~~

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
retained reference. No clamp and no reset are allowed.

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

TubeBuilder continuous_inset and TubeSurfaceValidator cover must be reconciled
term-by-term. Half-voxel, interpolation, path, normal, and swept-cell
residuals are charged exactly once. Certificate-complete cells may remove only
redundant geometric erosion, never physical clearance or obstacle proof.

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

One Validator is a proof-conditioned target. If it is insufficient, stop and
specify a bounded proof-equivalent path; never restore unbounded retries.

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
  add phase_offset_tube_nominal_half_width and forward it as
  phase_offset/tube/nominal_half_width

MODIFY
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
  expose and forward the explicit nominal experiment value
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
28. all Stage-1A regressions remain green otherwise.

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
ESDF/cloud query count
max_bounded_construction_abs_delta
retained-reference safety query count
actual-position safety query count
worker busy ratio
cmdCallback timing/rate
traj_vis rate
local_map rate
tube_update_period
~~~

The metrics must survive CertifiedTubeBuilder to TubeEpochManager/adapter
handoff and be emitted in the existing measurement/diagnostic route. If a
metric cannot be made observable within the whitelist, remove it from the
claimed acceptance gate instead of guessing.

Compare:

~~~text
A = accepted/frozen Stage-1A baseline
B = Stage 1A + P1 with nominal_half_width=1.0 m
~~~

Use the accepted approximate references of 50 Hz command and 10 Hz map/traj
publication, with no crash, deadlock, worker stall, FIFO, callback coupling,
stale authority resurrection, or new test failure. No 1 ms promise is allowed.

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

## 22. Hygiene, revision loop, and future workflow

The initial Candidate remains unchanged:

~~~text
docs/Codex_D-ISFGVF_P1_BoundedTube_Implementation_Plan_2026-08-27_Candidate.md
SHA256=5129728ee6ac5b16123e235347a1df98eec84c865eb4df5420ab7903cecc2f14
~~~

Rev1 is the only new file in this documentation correction round. Existing
AGENTS.md, .codex, Testing/, prior Roadmap and Stage-1A documents,
__pycache__ directories, and all other user-owned changes must not be cleaned,
reset, restored, stashed, deleted, staged, attributed, committed, or pushed.

After Rev1, compute its exact SHA and byte size; verify branch, HEAD, remotes,
status, staged paths zero, and git diff --check. Do not stage or commit.

If Sol requires further documentation corrections, leave Rev1 unchanged,
create Candidate_Rev2, record this SHA, compute a new SHA, and use a new fresh
Sol xhigh audit. At most two correction/review rounds after the initial
Candidate are authorized. A blocker requiring code or architecture expansion
stops the workflow.

## 23. Sol xhigh Rev1 review checklist and output

A new independent Sol xhigh reviewer must verify artifact identity, closed
Stage-1A history, rho_nom default/precedence, source-specific validation,
absolute endpoint semantics, asymmetric clipping, construction-versus-safety
query separation, out-of-range anchor behavior, exact component labels,
Contract-A and exactly-once cover, continuous proof, bounded Builder
simplification, all exact whitelists, both launch edits, inherited waiver,
deterministic tests, observable performance metrics, Stage 1B conditionality,
and explicit P2/P3 exclusions.

Required sections:

~~~text
ARTIFACT_IDENTITY
BASELINE_AND_STAGE1A_AUDIT
RHO_NOM_OWNER_AUDIT
LEGACY_SOURCE_SPECIFIC_VALIDATION_AUDIT
ASYMMETRIC_CLIPPING_AUDIT
CONSTRUCTION_VS_SAFETY_QUERY_AUDIT
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

Every MUST_FIX must cite severity, exact Rev1 section, source file/function
evidence, blocking reason, and smallest documentation correction.

Allowed final verdicts:

~~~text
P1_PLAN_ACCEPTABLE_READY_FOR_FREEZE
P1_PLAN_ACCEPTABLE_WITH_REQUIRED_CORRECTIONS_NOT_READY_FOR_FREEZE
P1_PLAN_NOT_ACCEPTABLE_NOT_READY_FOR_FREEZE
~~~

## 24. Execution boundary

This Rev1 Candidate does not authorize production or test implementation,
launch/config edits, Stage 1B, P2A/P2B, P3, Batch C, Luna, staging, commit,
push, tag, branch, reset, restore, clean, or stash. The next action is only a
fresh Sol xhigh read-only audit of this exact Rev1 hash. If accepted, a
separately authorized documentation-only Freeze may create the Frozen Plan
and then the self-contained new-window handoff.
