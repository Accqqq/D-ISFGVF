# D-ISFGVF P1 Certificate-Tightness Causal-Replay Handoff

## Status

P1 production implementation is ACCEPTED, COMMITTED, PUSHED, and FROZEN.

Accepted production baseline:

~~~text
branch=main
HEAD=9e91f370fc1c44eb4ada8ffd8d209adbab66d688
origin/main=9e91f370fc1c44eb4ada8ffd8d209adbab66d688
parent=cd313b315fcf4157273a7b0de84622073b3afaf6
~~~

Current next-stage objective is a certificate-tightness / causal-replay diagnosis
of intermittent ZERO_BASELINE / Certified Tube discontinuity. This is not a P1
performance redesign, not Stage 1B, and not P2/P3.

No Luna has been launched in this planning window. No production code, test
code, or launch code was modified. No staging, commit, or push is authorized by
this diagnostic handoff.

## Accepted P1 performance and ROS evidence

~~~text
A goals = 5/5
B goals = 5/5

A build median = 3682.055 ms
B build median = 46.408 ms

A build p95 = 6257.209 ms
B build p95 = 143.808 ms

A build max = 7226.822 ms
B build max = 272.799 ms

A worker busy ratio ~= 88.9%
B worker busy ratio ~= 44.6%

B build rate ~= 8.01 Hz
~~~

B structural evidence: SurfaceValidator exactly one invocation for 503/503 builds;
inward search zero for 503/503; stale completions 23/503 with approximately 2.354 s
accumulated compute and 8.4% of B worker busy time; maximum bounded construction
delta 1.0 m; nominal width source EXPLICIT_PARAMETER.

Healthy rates:

~~~text
position_cmd A/B = 49.844 / 49.631 Hz
local_map A/B = 10.001 / 10.001 Hz
traj_vis A/B = 9.983 / 9.983 Hz
~~~

Accepted query accounting totals remain certified_cell_bound=632459,
construction_clearance=5741880, and total_construction=6374339.

Stage 1B remains future work only:
STAGE1B_MEASUREMENT_JUSTIFIES_FUTURE_PROPOSAL.

## Protected production semantics

P1 bounded nominal interval is [-rho_nom,+rho_nom], rho_nom=1.0 m, owned by
phase_offset/tube/nominal_half_width. +N and -N clipping are independent;
asymmetric sections are allowed; there is no normal widening, inward-search
family, repeated scale halving, or nominal-external widening. Continuous proof
remains required and inability to prove it fails closed.

Stage 1A scheduler-only permit, one joined worker, one inflight build,
latest-only pending work, stale drops, transactional state, provenance/currentness,
stale completion rejection, reset/deactivation/shutdown lifecycle, and
tube_update_period semantics are protected. Contract A remains frozen:
planning/safe_distance is the sole authoritative physical obstacle-clearance
margin and is 0.4 m in the accepted P1 runtime.

Do not reopen P1 architecture or touch Runtime, Recovery, Handoff,
TubeEpochManager, gvf_manager, planner, governor, launch authority, Stage 1B,
P2, or P3.

## Observed phenomenon and established semantics

The user observed repeated ZERO_BASELINE warnings with
obstacle_certified=0, first_stop_reason=insufficient_clearance, and
invalid_reason="full-width surface failed; retained complete zero planner
baseline", including first_stop_w values:

~~~text
13.823663507
14.023571001
14.223779121
14.428689351
14.640893632
14.853396214
~~~

Source tracing established that this warning requires a complete candidate
classification of ZERO_ONLY_PLANNER_BASELINE while raw build samples still have
nonzero capacity. Thus bounded/asymmetric raw geometry existed, subsequent
continuous certification failed, and the complete zero planner baseline was
retained. “Full-width” means the currently bounded candidate, not restoration of
the old [-1,+1] search or an inward retry.

Current production certificate relation:

~~~text
queried_clearance >= planning_safe_distance
                    + geometric_cover
                    + cover_epsilon
~~~

geometric_cover includes midpoint-position variation, 0.5*maximum_delta*normal
variation, 0.5*delta_slope*cell_w_span, 0.5*maximum_width*v_span, and support
accounting including 0.5*snapshot_resolution. With resolution 0.1 m, the
half-voxel term is 0.05 m. Its necessity/tightness remains open; do not alter it.

## Prior quantitative evidence

Prior isolated diagnostic worktree:
/tmp/d_isfgvf_p1_conservatism_diag.

Fixed identical Validator replay was deterministic 20/20. Representative real PASS:

~~~text
map_seq=52
path_revision=1
cell=554
w=[2.231130713,2.231227622]
required=0.450298361 m
queried=0.450298361 m
certificate residual=0
~~~

Representative real FAIL:

~~~text
map_seq=52
path_revision=1
cell=568
w=[2.231227622,2.231324531]
status=KNOWN_FREE
required=0.450298361 m
minimum queried=0.450291541 m
certificate residual=-0.000006820 m
dense physical margin=+0.050291541 m
dominant cover term=half_voxel=0.050000000 m
~~~

This proves some individual certificate-driven behavior, not global/material
conservatism.

Mixed terminal-failure population: 46 quantifiable failures; 7 negative dense
physical margins; 30 near zero; 4 above +0.02 m. Accepted prior classification:
ROOT_CAUSE=BORDERLINE_COMBINATION,
MATERIAL_CERTIFICATE_CONSERVATISM=BORDERLINE,
P1_IMPLEMENTATION_DEFECT=NOT_SUPPORTED.

Prior narrow Sol evidence verdict:
P1_CONSERVATISM_DIAGNOSIS_EVIDENCE_NOT_ACCEPTABLE. Sol accepted the arithmetic,
instrumentation timing, and deterministic replay, but rejected causal
attribution because fixed-path/varying-snapshot and fixed-snapshot/varying-path
were not safely isolated. The previous .cpp-only seam had no owner registry;
fake callback mixing was correctly refused.

## Current Candidate/Frozen Plan status

Candidate Plan:

~~~text
docs/Codex_D-ISFGVF_P1_Certificate_Tightness_CausalReplay_Diagnostic_Plan_2026-08-28.md
SHA256=a13757054e091a4ff76a7b88cf8658110d74689b56dcd835a38700cbe8e13913
bytes=20661
lines=244
~~~

Frozen Plan:

~~~text
docs/Codex_D-ISFGVF_P1_Certificate_Tightness_CausalReplay_Diagnostic_Plan_2026-08-28_FROZEN.md
SHA256=5a571727548c9b444249928d0d93d554b5c17351e0d454450cf37f97598ccf66
bytes=21179
lines=252
~~~

FROZEN_PLAN_SEMANTIC_DIFF=FREEZE_METADATA_ONLY: normalized technical content
is identical; Frozen adds only status and Sol audit metadata.

Fresh Sol xhigh PLAN audit:

~~~text
REPLAY_FIDELITY_PLAN=PASS
OBSERVATIONAL_INSTRUMENTATION_PLAN=PASS
REPLAY_EQUIVALENCE_GATE=PASS
SNAPSHOT_ISOLATION_PLAN=PASS
PATH_ISOLATION_PLAN=PASS
FULL_PIPELINE_CAUSAL_PLAN=PASS
DENSE_DIAGNOSTIC_PLAN=PASS
HALF_VOXEL_ACCOUNTING_PLAN=PASS
V_SPAN_TIGHTNESS_PLAN=PASS
MATERIAL_CONSERVATISM_CRITERION=PASS
FUTURE_LUNA_SCOPE=PASS
BLOCKING_FINDINGS=NONE
SOL_PLAN_VERDICT=P1_CAUSAL_REPLAY_DIAGNOSTIC_PLAN_ACCEPTABLE
~~~

The Frozen Plan is implementation-ready but is not itself implementation
authorization. It requires a later explicit user authorization in a new Main
window before Luna may be launched.

## Source-grounded replay architecture

The authoritative semantic path owner is
shared_ptr<const FLAG_Race::ContinuousPhasePath>, with exact segment evaluators,
path revision, p/p_w/p_ww, and optional complete cell certificates. The paired
shared_ptr<const ContinuousPhaseNormalFrame> owns Horizontal-N
normalize(e_z x p_w), N_w, frame revision, and frame-cell proof. Both are already
carried together in TubeBuildRequest.

The authoritative snapshot owner is
shared_ptr<const plan_env::CloudOccupancySnapshot>. Its public metadata and
occupied voxel vector are self-contained; its private column index is only an
optional validated acceleration cache. Clearance is to closed occupied voxel
volumes, with UNKNOWN outside the observed AABB and requested-radius capping.

The future full replay must retain/reconstruct the exact owner pair and snapshot,
then rebuild a fresh CertifiedTubeBuildInput for every path/snapshot pair:
path + frame + snapshot -> production Builder -> Filter -> Validator. It must
never swap a candidate between owners or use position-to-clearance lookup tables
as a substitute. In-process immutable owner retention is preferred; concrete
serialization is conditional and must preserve exact mapped-B-spline,
quintic-Hermite, and slice semantics.

## Future diagnostic scope

The Frozen Plan requires:

- exact immutable owner capture at the TubeBuildRequest/timer-build seam;
- snapshot capture including complete occupied vector, metadata, configuration, and hash;
- replay-equivalence gate against one real PASS and one real FAIL;
- Experiment A: one exact path+snapshot pair repeated at least 20 times;
- Experiment B: one exact path/frame/configuration with multiple real snapshots,
  rebuilding the candidate for each snapshot;
- Experiment C: one exact snapshot/configuration with multiple real path/frame
  owners, rebuilding the candidate for each path;
- full per-cell/per-query certificate evidence;
- empirical dense scans with N/2N/4N convergence;
- half-voxel, double-accounting, and v-span audits;
- classification into PHYSICAL_FAILURE, BORDERLINE_PHYSICAL,
  CERTIFICATE_ONLY_REJECTION, or INCONCLUSIVE;
- material conservatism only after repeated real, converged, causally isolated
  evidence.

Dense sampling is diagnostic evidence only and never a continuous safety proof.
No production formula, threshold, query order/count, scheduler, authority, or
lifecycle semantics may change.

## Future Luna whitelist and exclusions

Luna may modify only the smallest necessary subset, in a new isolated worktree
based exactly on the accepted P1 commit:

- bspline_race/integration/phase_offset_matched_adapter.h/.cpp for immutable
  capture records at the existing request/build seam, without scheduler or
  authority changes;
- certified_tube_builder.h/.cpp only for read-only build identity hooks if
  strictly necessary;
- tube_surface_validator.h/.cpp only for observational per-cell/query hooks
  and deferred dense utility;
- tube_builder.h/.cpp only for read-only input/output capture if the adapter
  seam cannot expose it;
- plan_env/cloud_occupancy_snapshot.h/.cpp only for versioned public snapshot
  serialization/round-trip validation;
- continuous_phase_path.h/.cpp and
  continuous_phase_normal_frame.h/.cpp only for exact concrete-owner
  serialization if cross-process replay requires it;
- a new isolated diagnostic executable/script/test.

No production launch, scheduler, Runtime, Recovery, Handoff, TubeEpochManager,
gvf_manager, planner, governor, Contract A, safe distance, TubeFilter policy,
P1 geometry policy, Stage 1B, P2, P3, unrelated dirty files, or broad build
infrastructure may be modified. No stage/commit/push/cleanup/stash/reset/restore.

If any required edit lies outside the whitelist, Luna must STOP and report.

## Repository hygiene at handoff

Original repository verification at handoff:

~~~text
branch=main
HEAD=9e91f370fc1c44eb4ada8ffd8d209adbab66d688
origin/main=9e91f370fc1c44eb4ada8ffd8d209adbab66d688
staged paths=0
git diff --check=PASS
production code modified=false
test code modified=false
launch code modified=false
Luna launched=false
commit/push in this planning turn=false
~~~

Existing user-owned dirty/untracked content remains present, including AGENTS.md,
.codex, Testing/, historical docs, and __pycache__ entries. Do not clean,
stash, reset, restore, delete, overwrite, or stage them.

## NEW WINDOW START INSTRUCTIONS

1. Read this Handoff in full.
2. Verify branch=main, HEAD=9e91f370fc1c44eb4ada8ffd8d209adbab66d688,
   origin/main is identical, and staged paths are zero.
3. Read the Frozen causal-replay diagnostic Plan.
4. Verify its SHA256 against this Handoff.
5. Do not reopen P1 design and do not rewrite the Frozen Plan.
6. Do not launch Luna until the user explicitly authorizes implementation in the
   new window.
7. After authorization, create a new isolated temporary worktree and launch
   Luna as implementation-only.
8. No production commit/push is authorized by this diagnostic Plan.
9. Follow the Frozen Plan's exact whitelist, replay-equivalence gate, A/B/C
   protocol, dense convergence, and STOP conditions.
10. If source conflicts with the Frozen Plan or immutable owner pairing cannot be
    maintained, STOP and report.

## Copy-paste prompt for a new Main window

Continue as a NEW fresh Main/orchestration window for D-ISFGVF.

Read first:

docs/D-ISFGVF_P1_Certificate_Tightness_CausalReplay_Handoff_2026-08-28.md

Then read and verify the Frozen Plan path and SHA recorded in that Handoff.

Verify repository identity and hygiene before doing any work. P1 production is
frozen at 9e91f370fc1c44eb4ada8ffd8d209adbab66d688. Do not reopen P1 planning.
Do not implement anything and do not launch Luna until my explicit Luna
implementation authorization is present in the new window. Follow the Handoff's
NEW WINDOW START INSTRUCTIONS exactly.


