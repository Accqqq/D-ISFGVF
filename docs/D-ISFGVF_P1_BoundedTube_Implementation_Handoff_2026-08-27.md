# D-ISFGVF P1 Bounded Tube — New-Window Implementation Handoff

~~~text
PROJECT=D-ISFGVF
NEXT_STAGE=P1_BOUNDED_NOMINAL_ASYMMETRIC_TUBE_IMPLEMENTATION
STAGE1A_FROZEN_BASELINE=cd313b315fcf4157273a7b0de84622073b3afaf6
P1_FROZEN_PLAN_PATH=docs/Codex_D-ISFGVF_P1_BoundedTube_Implementation_Plan_2026-08-27_FROZEN.md
P1_FROZEN_PLAN_SHA256=44b64ded014859ca17b7b0450a1a8854bd7b6b85b6ba1ce9129285c5b6ec0904
P1_PLAN_SOL_VERDICT=P1_PLAN_ACCEPTABLE_READY_FOR_FREEZE
MARGIN_ACCOUNTING=CONTRACT_A
CURRENT_DELTA_TRANSITION=SMALL_CONTRACT_TEST_AMENDMENT_REQUIRED
STAGE1B=MEASUREMENT_CONDITIONAL
P1_IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
NEXT_WINDOW_TASK=IMPLEMENT_FROZEN_P1_AFTER_EXPLICIT_USER_AUTHORIZATION
~~~

## 1. Repository baseline

~~~text
repository=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
branch=main
HEAD=cd313b315fcf4157273a7b0de84622073b3afaf6
origin/main=cd313b315fcf4157273a7b0de84622073b3afaf6
d_isfgvf/main=5355be736339a650432280deec53362f61749a43
configured_upstream=d_isfgvf/main
ahead/behind=1/0
~~~

Remote mapping:

~~~text
origin    git@github.com:Accqqq/gvf_ws.git
d_isfgvf  git@github.com:Accqqq/D-ISFGVF.git
lifted    git@github.com:Accqqq/Lifted_GVF.git
~~~

The previous accepted R3 + Horizontal-N baseline is
`5355be736339a650432280deec53362f61749a43`. Stage 1A is accepted, frozen,
committed, and pushed at `cd313b315fcf4157273a7b0de84622073b3afaf6`.
Do not downgrade it to an unreviewed candidate or reopen its architecture.

## 2. Frozen source of truth

Before any implementation, independently compute the SHA-256 of:

~~~text
docs/Codex_D-ISFGVF_P1_BoundedTube_Implementation_Plan_2026-08-27_FROZEN.md
~~~

It must equal:

~~~text
44b64ded014859ca17b7b0450a1a8854bd7b6b85b6ba1ce9129285c5b6ec0904
~~~

The accepted source Candidate was Rev3:

~~~text
docs/Codex_D-ISFGVF_P1_BoundedTube_Implementation_Plan_2026-08-27_Candidate_Rev3.md
SHA256=817243bd43e3742960d9a93d1a49db77301222989ce8e834f939e18426db59f3
~~~

The independent strict delta-closure review returned:

~~~text
CONTRACT_A_COVER_CLOSED
CONSTRUCTION_QUERY_ACCOUNTING_CLOSED
NO_REV3_INDUCED_P1_CONTRACT_REGRESSION
P1_PLAN_ACCEPTABLE_READY_FOR_FREEZE
~~~

The Frozen Plan differs from accepted Rev3 only in lifecycle/audit metadata.
It is the implementation contract; this handoff is a concise orientation, not
a replacement for it.

## 3. Closed Stage-1A boundary

Preserve exactly:

- one joined Tube worker;
- one latest-only pending slot and no FIFO;
- scheduling permit cadence independent of 50 Hz command traffic;
- immutable request identity and job-local build state;
- stale pre-build rejection and stale completion abandonment;
- task/source/path/frame/owner/authority/pair currentness gates;
- no stale Candidate, Certified Geometry, Active Tube, Runtime, authority, or
  authoritative cache/profile/epoch commit;
- accepted stale same-task raw/cloud diagnostic-attempt exception under its
  existing provenance and exactly-once contract;
- reset/deactivate/shutdown invalidation and joined shutdown;
- no publication after the accepted shutdown ownership flip.

Adapter changes in P1 are limited to nominal-width configuration, propagation,
diagnostics, and measurement fields. Do not alter worker scheduling, pending
slot, currentness, publication, reset, deactivate, shutdown, or join semantics.

## 4. P1 objective and geometry

P1 certifies the safe subset of one requested bounded nominal Tube. It does not
search for or optimize maximum width.

~~~text
rho_nom > 0
I_nom(w)=[-rho_nom,+rho_nom]
authoritative owner=TubeCrossSectionConfig::nominal_half_width
ROS parameter=phase_offset/tube/nominal_half_width
absent default=1.0 m
acceptance value=1.0 m
1.5 m=explicit override experiment only
~~~

Open space produces exactly `[-rho_nom,+rho_nom]`. Obstacles independently clip
the `-N` and `+N` sides. No scalar symmetric shrink, nominal-external search,
maximum-width objective, unsafe-gap convexification, three-family normal retry,
or repeated normal scale halving remains.

Horizontal-N remains frozen:

~~~text
N(w)=normalize(e_z x p_w(w))
N_z=0
r(w,delta)=p(w)+N(w)delta
r_z=p_z
~~~

The planner centreline remains full 3-D and owns altitude.

## 5. Source-specific configuration

The ESDF production path uses source-aware validation across:

~~~text
TubeCrossSectionSolver::configurationValid()
TubeBuilder::configurationValidForSource(TubeSource)
CertifiedTubeBuilder::configurationValidForSource(TubeSource)
~~~

For ESDF, invalid `fixed_delta_max`, `max_offset`, or deprecated
`search_extent` must not veto valid nominal-width geometry. Legacy fields remain
limited to their fixed/test, legacy DistanceQuery, or compatibility roles.

Required width provenance:

~~~text
nominal_width_source=DEFAULT_ABSENT | EXPLICIT_PARAMETER
nominal_width_legacy_conflict=<exact boolean over explicitly present legacy fields>
effective_nominal_half_width_m=<rho_nom in metres>
~~~

## 6. Contract A and continuous cover

~~~text
physical clearance=planning/safe_distance exactly once
geometric proof cover=one traceable residual exactly once
~~~

Immutable snapshot inflation/provenance must be sufficient and revision
matched. Missing, stale, unavailable, mismatched, or insufficient evidence
fails closed. RobustTubeMargins fields are diagnostic/compatibility only and
must not re-add physical margins.

For a complete matching cell certificate, unchanged
`TubeSurfaceValidator::CertifiedCellCoverRadius()` owns:

~~~text
C = certificate.midpoint_position_variation_bound
  + 0.5 * maximum_delta * certificate.normal_variation_bound
  + 0.5 * delta_slope * h
  + 0.5 * maximum_width * v_span
  + 0.5 * snapshot_resolution

requested_clearance = planning/safe_distance + C + cover_epsilon
~~~

The ESDF Builder records zero `continuous_inset`; the Validator production
implementation remains unchanged. Nondegenerate sampled fallback uses
`1.1*sampled_radius + 0.5*snapshot_resolution`; the exact current-anchor
degenerate branch uses `1.1*sampled_radius`. Once a complete matching
certificate is attempted, proof failure has no sampled fallback.

## 7. Complete construction-query taxonomy

Each actual callback invocation belongs to exactly one leaf family:

~~~text
cross_section_directional_query_count
adaptive_refinement_centerline_query_count
adaptive_sample_base_clearance_query_count
builder_certified_cell_bound_query_count
validator_certified_cell_bound_query_count
validator_surface_query_count
~~~

Exact identities:

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
~~~

`AdaptiveNeedsSubdivision()` contributes exactly three centreline
ClearanceQuery calls whenever it reaches its clearance-comparison block.
`buildCloudClearance()` contributes one base-centreline query per adaptive
sample reaching evaluation. `TubeSurfaceValidationResult::query_sample_count`
is the exact Validator surface-clearance count. CertifiedBuilder may wrap the
PathCellBoundQuery to retain separate Builder and Validator proof sub-counts.

Construction/proof accounting excludes:

- retained-reference safety queries;
- actual-position safety queries;
- TubeEpochManager post-build base-centreline safety queries;
- command-time `LatestCategoricalUnsafe()` reference/actual evidence.

Explicit transverse construction centres satisfy
`abs(delta_query)<=rho_nom`. Adaptive/base queries are at delta zero.
PathCellBoundQuery is a phase-cell proof callback and must not be assigned a
fabricated ray delta.

## 8. Current-delta transition

For `delta_current=+1.2 m` and `rho_nom=1.0 m`:

- retain `+1.2` unchanged in request/epoch/authority metadata;
- use exact zero only as the bounded construction anchor;
- do not clamp to `+1.0` and do not reset authority to zero;
- keep all nominal construction/Validator transverse centres within `[-1,+1]`;
- restore retained current-delta metadata after Builder/Filter;
- force the bounded result to `ZERO_CONNECTED` and
  `current_component_contains_delta=false`;
- let TubeEpochManager return `CURRENT_OFFSET_OUTSIDE` and retain the previous
  active owner/pair/authority;
- permit later takeover only through existing valid re-entry semantics.

No Recovery or Handoff production redesign is authorized. Exact zero remains
exact zero; tiny finite nonzero remains nonzero.

## 9. Production whitelist

Only the following production files may be modified after explicit user
implementation authorization:

~~~text
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

Do not modify `tube_surface_validator.cpp`, `tube_epoch_manager.cpp`, Runtime,
RecoveryOwner, Handoff, `gvf_manager.cpp`, planner/governor, Horizontal-N,
Preview, swarm packages, QP/CBF, or future-stage code.

## 10. Test and launch whitelist

TEST_ONLY:

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
~~~

REGRESSION_ONLY / LEAVE UNCHANGED:

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

The sole inherited pre-existing exception remains:

~~~text
TubeSurfaceValidatorTest.ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel
~~~

Launch-only modifications:

~~~text
src/swarm_planner/bspline_traj/launch/test_gvf.launch
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
~~~

Both default `phase_offset_tube_nominal_half_width=1.0` and forward
`phase_offset/tube/nominal_half_width`. They may also expose/forward the
existing measurement enable, callback CSV, and tube-due CSV parameters. The
user-owned `gvf/circle_test/enable=false` and `auto_start=false` settings remain
unchanged.

## 11. Required verification and ROS acceptance

Deterministic tests must cover exact 1.0/1.5 open bounds, independent one- and
two-sided clipping, no external width queries/retries, full-3D regularity,
fail-closed evidence, exact zero/tiny nonzero, the +1.2 transition, Contract-A
term ownership, sole-Validator outcome, source provenance/default/precedence,
launch defaults, and the exact construction-query sum identity.

Run, at minimum:

~~~bash
source devel/setup.bash
catkin_make -j2
catkin_make run_tests_phase_offset_core
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_bspline_race
~~~

Real ROS acceptance compares frozen Stage 1A against Stage 1A + P1 at
`rho_nom=1.0`, using the same pillar/map and identical five-goal sequence.
Record full-run CSVs, build/request/publication/stale/query/busy metrics, and
external `rostopic hz` evidence for `/position_cmd`, `/sim/local_map`, and
`/particle0/gvf/traj_vis`. Require no crash, deadlock, worker stall, FIFO,
50 Hz build churn, callback coupling, stale resurrection, or new regression.

## 12. Stage 1B and future stages

Stage 1B cooperative cancellation is not part of P1. After P1 measurement:

~~~text
SUCCESS=current geometry certified
FAILED=current geometry genuinely failed
CANCELLED=obsolete work stopped without a geometry/safety conclusion
~~~

If stale waste is insignificant, `STAGE1B_NOT_REQUIRED` is valid.

Future P3 Roadmap blockers, P2 Preview, Batch C, swarm control/transport,
NoQP, QP/CBF, and gradient-representation work are out of current scope and
must not delay or expand P1.

## 13. User-owned worktree protection

At handoff creation, known user-owned/unrelated entries include:

~~~text
 M AGENTS.md
?? .codex
?? Testing/
?? existing Candidate/Roadmap/Stage-1A documentation
?? src/swarm_planner/bspline_traj/test/__pycache__/
?? src/uav_simulator/so3_quadrotor_simulator/test/__pycache__/
~~~

The future window must record the actual status and preserve every unrelated
entry. Do not clean, reset, restore, stash, delete, stage, commit, attribute, or
push user-owned changes as P1 work.

## 14. New-window implementation workflow

1. Read this handoff and the exact Frozen Plan.
2. Recompute the Frozen Plan SHA-256 and verify repository branch/HEAD/remotes,
   staged paths, and unrelated worktree state.
3. Do not redesign or broadly re-review P1. The plan is frozen and independently
   accepted.
4. Wait for explicit user implementation authorization. This document alone
   does not authorize implementation or Luna.
5. After authorization, Main prepares the exact Frozen-Plan execution handoff
   and starts one Luna implementation agent limited to the frozen whitelist.
6. Luna implements only P1 and stops; it does not expand scope, redesign the
   plan, commit, or push unless separately authorized.
7. Main independently checks the complete diff, whitelist, deterministic tests,
   regressions, `git diff --check`, staged paths, and worktree hygiene.
8. A fresh Sol xhigh performs the independent read-only P1 implementation audit.
9. Any repair requires a separately bounded authorization and evidence-based
   whitelist; do not silently repair outside scope.
10. Perform the frozen real ROS/performance A/B acceptance and decide whether
    Stage 1B is required from measurement.

## 15. Stop conditions

Stop rather than expand scope if the Frozen Plan hash differs, the baseline is
materially different without a later accepted checkpoint, a required edit lies
outside the whitelist, Contract A would double-count/omit cover, construction
queries cannot remain nominal-bounded/accounted exactly once, Stage-1A semantics
would change, Recovery/Handoff redesign is required, or acceptance requires a
future stage.

No P1 implementation, Luna launch, stage, commit, or push is authorized by this
handoff.
