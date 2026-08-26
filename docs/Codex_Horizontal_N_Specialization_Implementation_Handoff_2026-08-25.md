# Horizontal-N Specialization — Fresh Main/Codex Implementation Handoff

```text
DOCUMENT_ROLE=FRESH_MAIN_EXECUTION_HANDOFF
DOCUMENT_STATUS=SUPPLEMENT TO FROZEN PLAN / IMPLEMENTATION NOT AUTHORIZED
DATE=2026-08-25
AUTHORITATIVE_PLAN=Codex_Horizontal_N_Specialization_Implementation_Plan_2026-08-25.md
AUTHORITATIVE_PLAN_REVISION=3
AUTHORITATIVE_PLAN_STATUS=FROZEN / IMPLEMENTATION NOT AUTHORIZED
MAIN_PLAN_REVIEW=PASS
SOL_PLAN_AUDIT=PLAN_ACCEPTABLE
IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
BATCH_C_AUTHORIZED=false
COMMIT_AUTHORIZED=false
PUSH_AUTHORIZED=false
```

## 1. Role and source-of-truth boundary

This handoff gives a fresh Main/Codex window the minimum complete execution
context for the already-frozen Horizontal-N specialization. It is an execution
handoff supplement only. It does not replace, amend, reinterpret, or authorize
implementation of the Frozen Plan.

The sole Horizontal-N architecture/whitelist/acceptance authority is:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/
Codex_Horizontal_N_Specialization_Implementation_Plan_2026-08-25.md
```

Its current exact identity is:

```text
Revision: 3
Status: FROZEN / IMPLEMENTATION NOT AUTHORIZED
Main review: PASS
fresh Sol plan audit: PLAN_ACCEPTABLE
Sol-audited pre-freeze SHA-256:
  8a67cb610557d1357f3405b9536baecf16ada5e836b2c8f43d16ba00c6787e51
final frozen SHA-256:
  36add701d4f57aece8055f52a47254db230259f1a55a83f198bbd9e294aab773
```

Revision 2 historical provenance remains:

```text
Rev2 Sol-audited pre-freeze SHA-256:
  cc8c9bd127096ea9bac74f9b682f808aa9064177a92d358dcc100252834b8485
Rev2 frozen SHA-256:
  ad7d72f93688e24b929f161dc719bc96318d0e93e3c052d799ece142890c848d
```

If this handoff conflicts with the Frozen Plan, the Frozen Plan wins. If the
real source/worktree conflicts materially with the Frozen Plan assumptions,
stop and report; do not silently redesign either document.

## 2. Project background and protected architecture

The repository develops the distributed multi-UAV PhaseOffsetSwarm extension
of the trusted single-UAV D-ISFGVF system. The original planner, B-spline and C2
continuation, map/ESDF, ISF-GVF guidance, governor, simulator, and SO3 command
chain remain protected.

The base path and active PhaseOffset reference are distinct:

```text
planner centerline: p(w) in R^3
PhaseOffset state:  (w, delta), with scalar delta
active reference:   r(w,delta) = p(w) + N(w) delta
```

Batch B is accepted as the infrastructure/recovery-ready milestone. Batch C,
the allocator/coordination production stage, remains unauthorized. Horizontal-N
is a bounded geometry-semantics specialization; it is not Batch C, a new
allocator, a QP layer, a new planner, or a two-dimensional reinterpretation of
the method.

The authoritative paper/method input is:

```text
/home/cxq/ISF-GVF/Paper/ICRA/
ICRA_PhaseOffset_NoQP_Revised_v4.tex
```

The latest Frozen architecture baseline is:

```text
/home/cxq/ISF-GVF/New_ISFGVF/
PHASE_OFFSET_REFACTOR_FROZEN_PLAN_V3_1.md
```

## 3. Accepted R3 status

R3 — Geometric Certified Path-Tube Visualization — has been implemented,
reviewed, and accepted in the current worktree. The final independent
implementation audit disposition was `R3_IMPLEMENTATION_ACCEPTABLE`. The R3
changes remain uncommitted and are therefore user-owned worktree state rather
than content recoverable from the current HEAD.

The accepted R3 plan and its original fresh-context handoff are:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/
Codex_R3_Geometric_Certified_Path_Tube_Visualization_Plan_2026-08-25.md

/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/
Codex_R3_Geometric_Certified_Path_Tube_Visualization_Handoff_2026-08-25.md
```

Current identities:

```text
R3 Frozen Plan SHA-256:
  0ee560e70c3329350abbc575506770031bf13f14b5134a235bdc2e4492dcb8f0
R3 historical handoff SHA-256:
  db8069f28fe0ae8a61cb8404bf6e12754563ff2fc39ef67de1678611b5a8c8a7
```

The older R3 documents contain their pre-implementation authorization state.
For current execution context, the user's later R3 acceptance and the accepted
six-file worktree are authoritative facts. They do not authorize any new R3
work.

Accepted R3 semantics that Horizontal-N must preserve include:

* relative MANUAL topic
  `phase_offset_manual/tube_certified_geometry`;
* meaning: latest completed, current-task/current-source, immutable
  provenance-safe, ESDF/safety-certified, filtered geometric Candidate Tube;
* sole geometry chain
  `latest_candidate_epoch_snapshot_ -> candidate_profile ->
  candidate_profile.samples -> filtered_lower/filtered_upper`;
* immutable Candidate-only ownership and no Pair/Active fallback;
* stable request/Candidate currentness binding;
* source-replacement publication linearization at the final immutable request
  identity check;
* no Runtime or ExecutionAuthority dependency for certified geometry;
* exactly three ADD or exactly three DELETE markers, never mixed/partial;
* all accepted R3 deterministic and regression behavior.

## 4. Source of the Horizontal-N inconsistency

The NoQP method keeps a full three-dimensional planner centerline and one scalar
transverse state. The future method decision restricts only that scalar
transverse direction to the world-horizontal plane.

The real repository currently has inconsistent geometry conventions:

* synthetic/fallback `phase_offset_core::GeometryEvaluator` already derives
  the normal from `e_z x p_w` and its analytic derivative;
* production `ContinuousPhaseNormalFrame` still uses a least-parallel seed,
  Bishop/parallel-style transport, and a projected-Hermite represented normal,
  which may be non-horizontal;
* successor staging still projects/sign-aligns
  `ActiveReferenceSnapshot::executed_N` as a new-frame seed;
* existing path cells compute horizontal derivative bounds internally, but the
  production frame/certificate contract was originally bound to the older
  transport convention;
* the bspline-side legacy geometry fallback contains stale planar-curvature and
  level-flight wording/behavior for sloped three-dimensional paths.

This mismatch is not caused by R3. R3 exposed/visualized already-certified
geometry and must remain behaviorally unchanged while the shared frame
convention is specialized.

## 5. Frozen Horizontal-N geometry semantics

Let:

```text
e_z = [0,0,1]^T
h   = e_z x p_w
q   = ||h|| = ||p_{w,xy}||
```

At every point where nonzero PhaseOffset capability is available:

```text
N   = h / q
h_w = e_z x p_ww
N_w = (I - N N^T) h_w / q

r   = p + N delta
r_w = p_w + N_w delta
```

Mandatory invariants:

```text
q > phase_offset_core::kHorizontalNormalSpeedEpsilon
phase_offset_core::kHorizontalNormalSpeedEpsilon = 1e-8
||N|| = 1
N^T p_w = 0
N_z = 0
(N_w)_z = 0
r_z = p_z
(r_w)_z = (p_w)_z
T = p_w / ||p_w||                  full 3-D
||r_w|| >= minimum_reference_speed full 3-D regularity
delta = 0 => r = p and r_w = p_w
```

Near/below-threshold horizontal speed removes only nonzero PhaseOffset/Tube
capability. It must not invalidate a planner-valid 3-D path, create persistent
HOLD, clear active nonzero authority, or silently retain nonzero `delta` while
executing the centerline.

Cell proof remains:

```text
q_min = inf ||p_{w,xy}||
a_xy  = sup ||p_{ww,xy}||
sup ||N_w|| <= a_xy / q_min
```

`PathCellGeometryCertificate` gains the explicit conservative
`sup_horizontal_p_ww_norm` fact plus
`horizontal_acceleration_bound_complete=false` as the missing-evidence
sentinel. Tangent variation remains independently bounded by:

```text
sup ||T_w|| <= sup ||p_ww|| / inf ||p_w||
```

There is no public `N_ww` requirement and no strict nonzero `r_ww`
requirement. TubeBuilder and TubeSurfaceValidator remain consumers of the
corrected frame/certificate and do not receive a semantic redesign.

The NoQP matched structure, `G_i B_i=0`, scalar analytic tangent allocation,
interval projection, preview envelope, Runtime/authority model, and scalar Tube
interval representation remain unchanged.

## 6. R3-overlapping and R3-excluded files

The exact full future whitelist is Frozen Plan Section 8. This handoff does not
duplicate or replace it. The R3 overlap requires special handling.

The following three accepted-R3 files are explicit future Horizontal-N
implementation files:

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/
  phase_offset_matched_adapter.h

src/swarm_planner/bspline_traj/src/integration/
  phase_offset_matched_adapter.cpp

src/swarm_planner/bspline_traj/test/
  phase_offset_matched_adapter_test.cpp
```

After a separate explicit implementation authorization, they may change only
for Frozen Horizontal-N work, including removal of `executed_N`
successor-seed ownership, Horizontal-N frame provenance/revision checks, and
focused regressions. Their accepted R3 behavior must not regress.

These three R3-only marker files remain outside the Horizontal-N whitelist:

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/
  phase_offset_tube_markers.h

src/swarm_planner/bspline_traj/src/integration/
  phase_offset_tube_markers.cpp

src/swarm_planner/bspline_traj/test/
  phase_offset_tube_markers_test.cpp
```

No R3 marker implementation change is authorized.

## 7. Current uncommitted worktree risk

Observed at handoff creation:

```text
workspace: /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
branch: main
HEAD: 7f162b3d3e019971821db1d209abe0d820a9f4b5
upstream: d_isfgvf/main
ahead/behind: 0/0
staged paths: 0
```

The accepted R3 implementation exists only in the worktree. Restoring shared
files from HEAD would erase accepted R3. Reconstructing them from the R3 plan,
from a prior patch, or from memory could also lose later accepted refinements.
Conversely, treating the whole eventual adapter diff as Horizontal-N could hide
an accidental R3 rollback or unrelated rewrite.

Observed current shared-file hashes:

```text
phase_offset_matched_adapter.h
  1256e9d0361a3590037f0e7077ee85379e3c86216394efd9e3b18f6808db66a2
phase_offset_matched_adapter.cpp
  a0ccff11ee85ffa6df8f5a9fd4d3aa3fdd5e28a19e39d5c87ef8ebdfdfb04ba0
phase_offset_matched_adapter_test.cpp
  18dc919413b7141af9ee8030fbf1e4ee33e19d251a08eaa0c50516d141726397
```

Observed excluded marker-file hashes:

```text
phase_offset_tube_markers.h
  8147819e75fca39ddf135a1a978abbd170834f85a53045f6cda930c55591a84f
phase_offset_tube_markers.cpp
  9976e35f68975df25bfbf2f50dd0a0e64e61ce62400148d6577648c84a93a284
phase_offset_tube_markers_test.cpp
  d9380225d8445bd9b8678a8174965dc0263dcbe84e245cffcac1f29f1c44b397
```

These values are historical handoff evidence only. The next Main must re-read
the actual filesystem and must not use these hashes as a substitute for the
mandatory fresh start snapshot.

Other modified/untracked paths, including `AGENTS.md`, `.codex/`,
`Testing/`, documentation and `__pycache__/`, are user-owned. Do not clean,
restore, stage, delete, reformat, or attribute them to Horizontal-N.

## 8. Mandatory Revision-3 Section 8.7 start snapshot

Before any future Luna edit, Main must record from the real worktree:

1. branch, HEAD, upstream, and exact local/upstream relationship;
2. staged path count and complete `git status --short`;
3. exact current diff of every file in the Horizontal-N whitelist;
4. SHA-256 hashes of the three shared adapter files;
5. exact accepted-R3 diff boundary across the three shared adapter files and
   the three excluded marker files.

This snapshot must be taken after reading the Frozen Plan and immediately
before any implementation writer is authorized. It is the immutable acceptance
comparison baseline.

The future Luna must modify the accepted R3 worktree in place. It must not:

* restore or replace shared files from HEAD;
* overwrite them with an earlier R3 patch;
* reconstruct them from chat memory, plans, or old handoffs;
* change any excluded marker file;
* stage or commit the combined worktree.

Final Main acceptance must prove:

```text
final diff
  = accepted R3 diff at the recorded start snapshot
  + only explicitly authorized Horizontal-N changes
```

Main must inspect the incremental Horizontal-N diff, re-hash all six R3 files,
prove the three marker files are byte-identical to the start snapshot, and
rerun the accepted R3 regressions. Any rollback/reimplementation of R3 is an
acceptance blocker.

## 9. Exact next-window execution sequence

Opening this handoff does not authorize implementation. The next Main/Codex
must proceed in this order:

1. Read `AGENTS.md` completely.
2. Read the exact Revision-3 Frozen Horizontal-N Plan completely and verify its
   frozen hash
   `36add701d4f57aece8055f52a47254db230259f1a55a83f198bbd9e294aab773`.
3. Read this handoff, Frozen V3.1, NoQP v4, and the accepted R3 Plan/Handoff.
4. Reinspect the real source and tests; do not reconstruct the implementation
   target from this summary.
5. Record the complete Section 8 start snapshot before authorizing any writer.
6. Compare the actual baseline with the Frozen Plan assumptions. On a material
   mismatch, stop and report; do not revise the Frozen Plan automatically.
7. Confirm a new, explicit user authorization for Horizontal-N implementation.
   If it is absent, stop. This handoff and the Frozen Plan are not authorization.
8. Only after explicit authorization and a valid start snapshot, follow the
   repository agent roles: Main remains orchestrator/reviewer and exactly one
   Luna Max may be the production writer. Give it the exact Frozen Plan,
   handoff, whitelist, start snapshot, and in-place R3 preservation rule.
9. Luna must edit only the Frozen Plan whitelist. A conditional candidate may
   be touched only when the real source proves it necessary within its stated
   condition. Any needed outside-whitelist file is a stop, not an automatic
   expansion.
10. Luna must run the focused build/tests and required regressions, run
    `git diff --check`, report exact changed files/evidence, and stop without
    staging, committing, pushing, or advancing a stage.
11. Main must independently review every incremental diff, the Section 8
    diff-composition equation, shared/marker hashes, R3 semantics, Horizontal-N
    invariants, builds/tests, staged count, and final worktree status.
12. Return the evidence and disposition to the user. Do not infer commit,
    push, Batch C, a second writer, or any later-stage authorization.

Do not launch Luna merely because this handoff exists. Do not start a
production writer while the implementation authorization gate is false.

## 10. Stop conditions and forbidden actions

Until a future explicit implementation authorization:

* do not implement Horizontal-N;
* do not invoke Luna;
* do not modify production source or tests;
* do not modify the Frozen Plan or reinterpret this handoff as a plan;
* do not stage, commit, push, reset, restore, clean, or stash;
* do not enter or scaffold Batch C.

Even after a future Horizontal-N implementation authorization, the Frozen Plan
forbids:

* edits outside its exact required/conditional/test whitelist;
* any edit to the three R3-only marker files;
* regression of accepted R3 certified-geometry semantics;
* Pair fallback or a second path/map/ESDF query for R3 geometry;
* Runtime or ExecutionAuthority dependency for the R3 certified topic;
* planner, Governor, Runtime, ExecutionAuthority, RecoveryOwner, owner/FSM,
  map/safety-margin, swarm, CBF, allocator, or Batch-C redesign;
* TubeBuilder or TubeSurfaceValidator semantic redesign;
* a QP layer, second allocator, second C2 system, public `N_ww`, strict
  nonzero `r_ww`, C3-equivalent path requirement, or a two-dimensional
  planner reinterpretation;
* altitude modification through `delta`;
* silent centerline fallback with retained nonzero authority;
* rollback, replacement, or reconstruction of accepted R3 shared files;
* unrelated cleanup, formatting, tuning, or user-worktree changes;
* automatic commit, push, or next-stage advance after tests pass.

Stop and report if any required change lies outside the whitelist, a near-
vertical cell lacks a closed positive horizontal-speed proof, a consumer
requires higher-order geometry outside the Frozen contract, the current
worktree cannot be separated from accepted R3, or preservation of R3 semantics
cannot be proved.

## 11. Handoff closeout

```text
Horizontal-N Plan: Revision 3 FROZEN
Main plan review: PASS
fresh Sol plan audit: PLAN_ACCEPTABLE
R3 implementation in worktree: ACCEPTED / UNCOMMITTED
Horizontal-N implementation authorization: NONE
Luna: NOT AUTHORIZED
commit/push: NOT AUTHORIZED
Batch C: NOT AUTHORIZED
next action: fresh Main rereads sources and waits for explicit implementation authorization
```

This handoff is complete. Stop here.
