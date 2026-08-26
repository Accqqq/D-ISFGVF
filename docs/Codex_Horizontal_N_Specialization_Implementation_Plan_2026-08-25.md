# Horizontal-N Specialization — Bounded Future Implementation Plan

```text
DOCUMENT_ROLE=FUTURE_SPECIALIZATION_PLAN
DOCUMENT_STATUS=FROZEN / IMPLEMENTATION NOT AUTHORIZED
PLAN_REVISION=3
DATE=2026-08-25
PLAN_OWNER=MAIN_CODEX
AUTHORITATIVE_NOQP=Paper/ICRA/ICRA_PhaseOffset_NoQP_Revised_v4.tex
MAIN_PLAN_REVIEW=PASS
SOL_PLAN_AUDIT=PLAN_ACCEPTABLE
SOL_AUDITED_PREFREEZE_SHA256=8a67cb610557d1357f3405b9536baecf16ada5e836b2c8f43d16ba00c6787e51
REV2_MAIN_PLAN_REVIEW=PASS
REV2_SOL_PLAN_AUDIT=PLAN_ACCEPTABLE
REV2_SOL_AUDITED_PREFREEZE_SHA256=cc8c9bd127096ea9bac74f9b682f808aa9064177a92d358dcc100252834b8485
REV2_FROZEN_SHA256=ad7d72f93688e24b929f161dc719bc96318d0e93e3c052d799ece142890c848d
PRIOR_SOL_PLAN_AUDIT=PLAN_REVISION_REQUIRED
PRIOR_SOL_PLAN_AUDIT_SHA256=721737c3d449e34f509127680982f385645f40417115a284440c5a3f5742a886
IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
BATCH_C_AUTHORIZED=false
COMMIT_AUTHORIZED=false
PUSH_AUTHORIZED=false
```

## 1. Governance and stage boundary

R3 is accepted, but this document is a separate future-stage specification. It
does not authorize Horizontal-N implementation, Luna, a commit, a push, Batch C,
allocator work, or control changes. During this Revision-3 planning/audit turn,
all production files and tests remain read-only, including the three files
shared with accepted R3. A later separately authorized Horizontal-N
implementation may edit those three shared files only within the explicit
Horizontal-N scope in Section 8.6 while preserving accepted R3 semantics.

During plan preparation and audit, only this Markdown file may be edited. The
production source, tests, R3 documents, Frozen architecture, configuration,
launch files, and papers are read-only. Existing user-owned worktree changes
must be preserved exactly; no reset, restore, clean, stash, stage, commit, or
push is permitted.

The stage is a geometry-semantics specialization/consistency repair. It is not
a new algorithmic method and is not a 2-D planner.

## 2. Authoritative evidence

The exact inputs are:

1. `Paper/ICRA/ICRA_PhaseOffset_NoQP_Revised_v4.tex` (the corrected NoQP v4
   paper named by the user; it supersedes any earlier NoQP filename).
2. `New_ISFGVF/PHASE_OFFSET_REFACTOR_FROZEN_PLAN_V3_1.md` (Frozen public
   interfaces, ownership, failure semantics, and batch boundaries).
3. `New_ISFGVF/gvf_ws/AGENTS.md` (workflow and repository boundaries).
4. The prior side-audit scope request at
   `/home/cxq/.codex/attachments/b8ff8a3d-6265-42ee-a4b0-faa026e5f569/pasted-text.txt`.
   The completed side agent reported `HORIZONTAL_N_SPECIALIZATION_FEASIBLE`
   to Main, but no persisted result artifact was found. Therefore the
   attachment is cited only as the audit scope/method-decision record, not as a
   stored verdict, and the current exact-plan audit must independently verify
   feasibility.
5. The actual source and tests in `New_ISFGVF/gvf_ws`, branch `main`, HEAD
   `7f162b3d3e019971821db1d209abe0d820a9f4b5`.

No earlier paper, historical proposal, or chat-memory contract may override
NoQP v4. The v4 method retains full three-dimensional `p_i(w)`, scalar
`delta_i`,

```text
r_i(w,delta_i)   = p_i(w) + N_i(w) delta_i
r_{w,i}          = p_{w,i}(w) + N_{w,i}(w) delta_i
G_i B_i          = 0
```

the scalar analytic tangent allocation, interval projection, preview-envelope
modulation, and no QP layer. Horizontal-N changes only the geometric convention
for `N` and the certificate/consumer consistency around it.

## 3. Current production mismatch

The current chain is:

```text
ContinuousPhasePath p,p_w,p_ww (full 3-D)
  -> ContinuousPhaseNormalFrame
  -> GeometryEvaluator / PhaseOffsetGeometry / TubeBuilder / validators
  -> immutable executed-reference query
  -> matched adapter, successor and recovery ownership
```

The synthetic/fallback `phase_offset_core::GeometryEvaluator` already computes
the world-horizontal normal from `e_z x p_w` and its analytic derivative. In
contrast, the production `ContinuousPhaseNormalFrame` currently uses a
least-parallel seed, Bishop/parallel-style projection transport, and a
projected-Hermite represented normal. The matched adapter also still projects
and sign-aligns `ActiveReferenceSnapshot::executed_N` into a successor frame.
Thus one path revision can currently expose non-horizontal production `N` while
fallback/core code is horizontal. This is a geometry-contract mismatch, not a
planner-validity, R3, or visualization defect.

The planner path, B-spline/C2 connector, full-3-D ISF-GVF state, scalar
PhaseOffset state, Runtime, ExecutionAuthority, owner modes, map contract, and
swarm contract remain outside this specialization.

## 4. Target Horizontal-N contract

For `e_z = [0,0,1]^T`, define on every location where nonzero PhaseOffset
capability is authorized:

```text
h       = e_z x p_w
q       = ||h|| = ||p_{w,xy}||
N       = h / q
h_w     = e_z x p_ww
N_w     = (I - N N^T) h_w / q
r       = p + N delta
r_w     = p_w + N_w delta
```

Required pointwise invariants are:

```text
q > kHorizontalNormalSpeedEpsilon
||N|| = 1
N^T p_w = 0
N_z = 0
(N_w)_z = 0
r_z = p_z
(r_w)_z = (p_w)_z
T = p_w / ||p_w||                 (full 3-D tangent)
||r_w|| >= minimum_reference_speed
```

At `delta=0`, the identity is exact: `r=p` and `r_w=p_w`. The centerline may
climb, descend, or turn in 3-D; only the scalar transverse displacement is
restricted to the world-horizontal plane. No horizontal-N formula may replace
the full-3-D planner tangent or introduce a two-dimensional planner state.

### 4.1 One production degeneracy threshold

The single production owner/value is:

```text
phase_offset_core::kHorizontalNormalSpeedEpsilon = 1e-8
```

It is declared in `phase_offset_core/normal_frame.h`. Frame queries and closed
cell certificates require the strict inequality
`||p_{w,xy}|| > kHorizontalNormalSpeedEpsilon`. The default
`GeometryParams::horizontal_tangent_epsilon` must reference this constant so
the synthetic fallback has the same default boundary. Test-only parameter
overrides do not redefine the production frame threshold. The existing local
frame epsilon may continue to guard generic finite/unit-vector arithmetic, but
it must not own Horizontal-N capability.

## 5. Formal reconciliation of Frozen V3.1

The following reconciliation is the only intended contract change:

| Frozen V3.1 contract | Horizontal-N reconciliation |
|---|---|
| One immutable frame per path revision | Preserved exactly. |
| `ImmutableNormalFrame::query/certifyCell` | Public interface and result shape preserved: `T,N,N_w`, revisions, validity, provenance. |
| Bishop/parallel transport and projected-Hermite production `N` | Superseded for the scalar production frame by the unique `normalize(e_z x p_w)` representation and its exact derivative. No two selectable frame conventions may coexist for one path revision. |
| Successor `executed_N` projection/sign-align seed | Superseded as a source of normal direction. Successor `N,N_w` are determined by successor `p_w,p_ww` and `e_z`. |
| `ActiveReferenceSnapshot::executed_N` | Remains an observed copy of the executed shared-frame normal for snapshot integrity/diagnostics. It is no longer a seed and must not be required, projected, sign-aligned, or consumed to construct a successor frame. |
| `r=p+N delta`, `r_w=p_w+N_w delta` | Preserved exactly with horizontal `N,N_w`. |
| Full-3-D regularity and connected Tube intervals | Preserved. Positive horizontal speed is an additional PhaseOffset frame-capability precondition. |
| PathTubePair, session/revision/replan mailbox, RecoveryOwner and atomic handoff | Preserved. Frame mismatch uses the existing replan-required route. |
| Runtime, ExecutionAuthority, matched port and owner rules | Preserved. No new owner, cache, FSM, gate, QP, CBF, or allocator is introduced. |

Every accepted production frame/certificate/query must carry one provenance
family, for example
`ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct`. A Bishop certificate
and a Horizontal-N certificate may not be mixed in one proof or Tube profile.

## 6. Degenerate, neutral, and successor behavior

* If `q <= kHorizontalNormalSpeedEpsilon`, or `N/N_w` is non-finite, the
  frame query and nonzero PhaseOffset capability fail closed with finite
  diagnostics.
* A closed cell whose conservative horizontal-speed lower bound is not strictly
  above the threshold is not a complete nonzero frame/cell certificate.
* This removes only nonzero PhaseOffset/Tube capability. It must not convert a
  planner-valid 3-D path into `PLANNER_INVALID`, create a new HOLD gate, change
  map/safety ownership, or clear active nonzero authority.
* Neutral planner-only operation remains valid. An active nonzero owner retains
  the existing owner until the existing RecoveryOwner/replan evidence permits a
  successor or neutral handoff. No silent centerline fallback may execute while
  retaining a nonzero `delta`.
* Successor construction queries Horizontal-N from the successor path itself;
  it never uses `executed_N` to choose, project, or sign-align the new normal.
  Existing predecessor snapshot/revision checks may remain, but successor
  staging must not fail solely because `executed_N` is absent, stale, or has a
  different value; the frame query is the normal authority.
* Existing session, revision, atomic publication, and mailbox contracts remain
  the only lifecycle mechanism.

## 7. Cell certificate contract

For each closed cell `[w0,w1]`, production evidence must conservatively bind the
same path/frame revision and convention and provide:

```text
q_min       = inf ||p_{w,xy}||
a_xy        = sup ||p_{ww,xy}||
sup ||N_w|| <= a_xy / q_min
```

The formula is the direct operator-norm bound for the normalized cross product;
the numerator is the horizontal acceleration bound and the denominator is the
strictly positive horizontal-speed lower bound. Implementations may use a
conservative larger bound, but may not use the old full-speed Bishop bound as a
Horizontal-N proof.

`PathCellGeometryCertificate` must add one explicit conservative field for
`sup_horizontal_p_ww_norm` and one explicit presence flag
`horizontal_acceleration_bound_complete`, defaulting to `false`.
`ContinuousPhasePath` already computes this closed-cell bound internally and
must set the flag only when the bound was conservatively produced. Together
with the existing
`inf_horizontal_p_w_norm`, it is sufficient for the required `N_w` and
`normal_variation_bound` proof. A public `q_max` field is not required by any
current consumer. The already-computed horizontal jerk remains producer-internal
because this stage does not expose `N_ww`; if a consumer is found to require it,
that is a stop condition rather than implicit scope expansion.

`inf_horizontal_p_w_norm`, `sup_horizontal_p_ww_norm`, `sup_N_w_norm`,
`normal_variation_bound`, and the full-3-D derivative fields must be mutually
consistent. Certificate completeness must reject non-finite facts, a threshold-
violating `q_min`, missing horizontal acceleration evidence, mixed convention,
or revision/segment/range mismatch.

The tangent proof is separate from the horizontal-normal proof:

```text
sup ||T_w|| <= sup ||p_ww|| / inf ||p_w||
tangent_variation_bound =
    (sup ||p_ww|| / inf ||p_w||) * (w1 - w0)
```

`ContinuousPhaseNormalFrame::certifyCell()` must not reuse
`normal_variation_bound` for `tangent_variation_bound`. A path with constant
horizontal direction and changing vertical slope is the required regression:
`N_w=0` may hold while `T_w` is nonzero.

The full-3-D regularity proof for `||p_w + N_w delta||` remains authoritative.
TubeBuilder and TubeSurfaceValidator consume that corrected evidence while
retaining full-3-D Euclidean cover, ESDF, connected-component, ZERO_ONLY, and
regularity semantics. They do not add a second map-margin erosion or a new
safety contract.

## 8. Bounded future implementation whitelist

All paths below are relative to `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/`. This is
a future whitelist only; it is not current authorization.

### 8.1 Required production candidates

* `src/swarm_planner/bspline_traj/include/bspline_race/continuous_phase_normal_frame.h`
* `src/swarm_planner/bspline_traj/src/continuous_phase_normal_frame.cpp`
* `src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/normal_frame.h`
* `src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/geometry_types.h`
* `src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/path_state.h`
* `src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp`
* `src/swarm_planner/bspline_traj/include/bspline_race/phase_offset_geometry.h`
* `src/swarm_planner/bspline_traj/src/phase_offset_geometry.cpp`
* `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
* `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
* `src/swarm_planner/bspline_traj/src/integration/phase_offset_executed_reference_query.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/active_reference_snapshot.h`

The frame implementation must remove the old production seed/transport meaning,
compute the Horizontal-N formula and derivative from the same path query, and
emit the new provenance. The certificate structure/producer must bind the
horizontal speed and acceleration evidence. The bspline geometry fallback must
use the same analytic Horizontal-N derivative rather than its current sloped-3-D
legacy planar-curvature derivative. The matched adapter must stop using or
requiring `executed_N` for successor frame construction and must validate the
shared Horizontal-N provenance/revisions through the existing replan path. The
snapshot comment/meaning must be updated from seed ownership to observed-frame
integrity/diagnostics without exposing it to governor-facing consumers.
The executed-reference query must compose the queried frame's Horizontal-N
provenance into its own provenance string so accepted query provenance is not
merely generic.
The `phase_offset_geometry.h` comments must remove the stale level-flight and
`1-kappa*delta` production wording. Its currently unused `v_xy_min`,
`z_tolerance`, and `max_delta` members remain compatibility metadata in this
bounded stage and must not become alternative capability/regularity owners.

### 8.2 Conditional production candidates

Only if a read-only implementation audit proves a direct semantic change is
needed:

* `src/swarm_planner/phase_offset/phase_offset_core/src/geometry.cpp`
* `src/swarm_planner/bspline_traj/src/integration/phase_offset_recovery_continuation_provider.cpp`

The conditional scope is limited to an implementation-audit-proven need for an
explicit Horizontal-N provenance/fail-closed check in an otherwise generic
consumer. It may not redesign transactions, Runtime, authority, recovery,
planner, C2, or the public `ImmutableNormalFrame` interface.

### 8.3 Read-only consumers and validation targets

These files are not expected to change merely because `N` changes:

* `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp`

They must be inspected and regression-tested as consumers of the corrected
certificate and shared frame. Any required edit must be justified by a concrete
semantic mismatch and separately added to the conditional list before
implementation authorization. No new ESDF/map query or margin erosion is
allowed.

### 8.4 Required test-edit candidates

* `src/swarm_planner/bspline_traj/test/continuous_phase_normal_frame_test.cpp`
* `src/swarm_planner/bspline_traj/test/continuous_phase_path_test.cpp`
* `src/swarm_planner/phase_offset/phase_offset_core/test/geometry_test.cpp`
* `src/swarm_planner/bspline_traj/test/phase_offset_geometry_test.cpp`
* `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
* `src/swarm_planner/bspline_traj/test/phase_offset_executed_reference_query_test.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp`
* `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp`

The Tube test edits are limited to the new certificate field, horizontal ray
altitude invariants, and unchanged consumer behavior. No Tube production edit
is implied.

### 8.5 Read-only regression suites

At minimum, run the existing:

* `src/swarm_planner/phase_offset/phase_offset_core/test/matched_port_test.cpp`
* `src/swarm_planner/bspline_traj/test/phase_offset_recovery_continuation_provider_test.cpp`

Launch/YAML/RViz, planner/GVF manager, governor, NeighborManager, swarm
allocator, CBF, Batch C, unrelated papers, and all R3-only files identified in
Section 8.6 are explicitly outside the whitelist.

### 8.6 Accepted-R3 overlap reconciliation

The accepted R3 visualization semantics remain frozen. The following three
files overlap the accepted R3 worktree and are nevertheless explicit future
Horizontal-N implementation files:

* `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
* `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
* `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

After separate implementation authorization, these shared files may change
only as required by the Frozen Horizontal-N specialization, including removal
of `executed_N` successor-seed ownership and the associated Horizontal-N
frame provenance/revision checks and focused regressions. This future
authorization does not reopen, replace, or redesign R3.

Every Horizontal-N edit to these shared files must preserve unchanged:

* `/phase_offset_manual/tube_certified_geometry` semantics;
* immutable Candidate-only geometry ownership;
* geometry from `candidate_profile.samples` and
  `filtered_lower/filtered_upper` only;
* stable request/Candidate currentness binding;
* source-replacement publication linearization at the final immutable request
  identity check;
* no Pair fallback;
* no Runtime or ExecutionAuthority dependency for certified geometry;
* exactly all-ADD or all-DELETE marker behavior;
* all accepted R3 deterministic and regression behavior.

The following R3-only marker files remain outside the Horizontal-N whitelist
and must not change:

* `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_markers.h`
* `src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_markers.cpp`
* `src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp`

No R3 marker implementation change is authorized. All other accepted R3
source/tests not explicitly listed as Horizontal-N candidates likewise remain
outside the whitelist.

### 8.7 Future implementation start snapshot and diff composition

Accepted R3 currently exists as user-owned, uncommitted worktree content.
Before authorizing Luna for a future Horizontal-N implementation, Main must
record:

* branch, HEAD, upstream, and local/upstream relationship;
* staged path count and complete `git status --short`;
* the exact current diff of every Horizontal-N whitelist file;
* SHA-256 hashes of the three R3-overlapping files in Section 8.6;
* the exact accepted-R3 diff boundary across those three shared files and the
  three excluded marker files.

The future Luna must modify the existing accepted-R3 worktree in place. It must
not restore, replace, overwrite from HEAD, or reconstruct any shared file. The
recorded start snapshot is the immutable comparison baseline for Main
acceptance.

Final Main acceptance must prove that the resulting worktree diff is:

```text
accepted R3 diff at the recorded start snapshot
  + only explicitly authorized Horizontal-N changes
```

Main must reject an accidental R3 rollback, reimplementation, marker-file
change, or unrelated rewrite. It must re-hash the shared files, verify the
excluded marker files retain their start-snapshot bytes, review the exact
incremental Horizontal-N diff on top of accepted R3, and rerun the accepted R3
regressions.

### 8.8 Revision-3 technical non-change

Revision 3 changes governance and whitelist wording only. It does not alter the
accepted Revision-2 technical decisions: full three-dimensional planner
centerline, scalar `delta`, Horizontal-N and analytic `N_w`, horizontal
`N/N_w`, altitude-preserving `r/r_w`, full-3-D `T` and regularity, the one
production threshold, supersession of Bishop transport and `executed_N` seed
ownership, the horizontal speed/acceleration cell proof, absence of `N_ww`
and strict nonzero `r_ww` requirements, unchanged TubeBuilder/SurfaceValidator
semantics, and no Runtime, ExecutionAuthority, Governor, Planner, or Batch-C
change.

## 9. Acceptance and regression matrix

### Geometry and derivatives

Use sloped 3-D lines, ascending/descending curved paths, helix, and varying-
height quintic/C2 paths. Check analytic `N_w` against central finite
differences away from the threshold, `N_z=0`, `(N_w)_z=0`, `r_z=p_z`,
`(r_w)_z=(p_w)_z`, orthonormality, deterministic query order, no sign flips, and
exact zero-offset recovery of full-3-D `p,p_w,r,r_w`. Near-vertical and
vertical horizontal projections must fail closed with finite diagnostics and no
planner-invalid/HOLD escalation.

Use `std::nextafter` or exact representable fixtures below, equal to, and above
`kHorizontalNormalSpeedEpsilon` for point queries, synthetic default geometry,
and closed-cell certificates. At/below the threshold must fail nonzero
PhaseOffset capability; above it must succeed when all other evidence is valid.
An active-nonzero successor regression must prove this failure retains old
authority and uses the existing replan-required route.

### Certificates and Tube

Check dense adversarial samples against `q_min`, horizontal acceleration,
and `sup_N_w_norm`; reject mixed or mismatched provenance and revisions; retain
full-3-D regularity as the active-reference decision. Verify every ESDF ray
sample for nonzero `delta` satisfies `sample.z == p_z(w)`. Preserve connected
components, ZERO_ONLY, continuous cover, full-3-D validator behavior, and the
single existing margin policy.

### Successor, recovery, and regressions

Verify successor frame independence from every `executed_N` seed value; exact
successor seams use successor path derivatives and Horizontal-N `N/N_w`; an
incompatible convention/revision retains old authority and emits existing
replan-required evidence; executed-reference query and matched adapter consume
the shared frame. Keep R1, R2/D1, D2, G18, and Seeded-Recovery suites green.
Accepted R3 visualization behavior must remain unchanged. The three shared
adapter files are inside the future whitelist only for the bounded
Horizontal-N edits in Section 8.6; the R3-only marker files remain excluded.

### Governance

The implementation must pass the focused build/tests, relevant regression
suites, dependency-boundary checks, and `git diff --check`, with no changed file
outside the subsequently authorized whitelist. No planner, Runtime,
ExecutionAuthority, Governor, map, swarm, authority, or Batch-C contract may be
widened.

## 10. Stop conditions

Stop without implementation if the actual baseline or Frozen architecture
differs materially; a needed edit lies outside the whitelist; a cell lacks a
closed positive horizontal-speed bound; a consumer requires `N_ww`, strict
nonzero `r_ww`, C3, or a second C2 system; or any failure would cause planner
invalidation, persistent HOLD, silent centerline fallback, stale nonzero
authority, an accepted-R3 semantic regression, an R3-only marker-file change,
an edit to a shared R3 file outside the Section 8.6 Horizontal-N scope, Batch C,
launch/YAML/RViz changes, or unrelated-user-file changes.

## 11. Main review and exact-plan audit gate

Main/Codex review of this candidate is recorded as:

```text
MAIN_PLAN_REVIEW=PASS
```

This Revision-3 Main review confirms that the corrected NoQP v4 paper and
accepted Revision-2 technical contract are unchanged. The only substantive
document-body change is governance reconciliation of the three shared R3
adapter files, explicit exclusion of the three R3-only marker files, and the
future implementation start-snapshot/diff-composition requirement. Accepted R3
visualization semantics remain frozen, and implementation remains unauthorized.

Main must compute the SHA-256 of these exact bytes and then launch exactly one
new independent read-only `gpt-5.6-sol` at the highest available reasoning
level (`xhigh` preferred, otherwise `max`). That Sol task must inspect this
exact plan, NoQP v4, Frozen V3.1, the actual source/worktree, the accepted R3
Plan/Handoff and semantics, and the existing Horizontal-N technical contract.
It must return either `PLAN_ACCEPTABLE` or concrete blockers, must not modify
any file, and must not invoke Luna. This Revision-3 procedure permits exactly
one fresh Sol audit; a blocker leaves Revision 3 unfrozen and ends the workflow.

Only after `PLAN_ACCEPTABLE` may Main perform a metadata-only transition to:

```text
DOCUMENT_STATUS=FROZEN / IMPLEMENTATION NOT AUTHORIZED
PLAN_REVISION=3
MAIN_PLAN_REVIEW=PASS
SOL_PLAN_AUDIT=PLAN_ACCEPTABLE
SOL_AUDITED_PREFREEZE_SHA256=<exact audited candidate SHA-256>
```

The pre-freeze SHA-256 must remain recorded as the exact audited plan identity.
If Sol does not return `PLAN_ACCEPTABLE`, leave this document `UNFROZEN`, do
not implement anything, do not launch a replacement auditor, and report the
blockers. The final frozen SHA-256 is recorded in Main's final report because a
file cannot contain its own final hash without changing that hash.
