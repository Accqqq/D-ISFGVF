# D-ISFGVF P1 Certificate-Tightness Causal-Replay Diagnostic Plan

Status: FROZEN — accepted for later diagnostic implementation only; implementation is not authorized in this window.
Date: 2026-08-28

## 1. Objective

Causally determine why approximately the same physical obstacle region can yield an accepted continuous P1 Tube in one build and ZERO_BASELINE/discontinuity in another. Separate physical clearance limitation, immutable map/snapshot variation, path/replan geometry variation, analytic certificate conservatism, half-voxel support, v-span cover, and possible accounting overlap.

This is diagnosis only. It must not change P1 production behavior, safety thresholds, Contract A, scheduler/lifecycle/currentness, Runtime, Recovery, authority, Stage 1B, P2, or P3.

## 2. Frozen production baseline

Repository: /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws

~~~text
branch=main
HEAD=9e91f370fc1c44eb4ada8ffd8d209adbab66d688
origin/main=9e91f370fc1c44eb4ada8ffd8d209adbab66d688
parent=cd313b315fcf4157273a7b0de84622073b3afaf6
~~~

All implementation must use a temporary isolated worktree based exactly on the accepted P1 commit. The original worktree remains untouched, unstaged, uncommitted, and unpushed.

Frozen P1 uses bounded nominal [-rho_nom,+rho_nom], rho_nom=1.0 m, independent asymmetric clipping, one bounded candidate, continuous proof required, and fail-closed behavior. planning/safe_distance is the sole Contract-A physical margin owner and remains 0.4 m.

Protected Stage-1A scheduler/worker/latest-only/currentness/lifecycle, Runtime, Recovery, Handoff, TubeEpochManager, and authority semantics are excluded.

## 3. Prior evidence and unresolved question

Accepted P1 ROS A/B evidence is frozen: A=5/5 and B=5/5 goals; build medians 3682.055 ms and 46.408 ms; p95 6257.209 ms and 143.808 ms; maxima 7226.822 ms and 272.799 ms; busy ratios about 88.9% and 44.6%; B rate about 8.01 Hz; Validator exactly 1 for 503/503; inward search 0 for 503/503; max bounded delta 1.0 m; nominal width source EXPLICIT_PARAMETER.

ZERO_BASELINE is a real certification transition: bounded nonzero raw capacity exists, continuous certification rejects the candidate, and a complete zero planner baseline is retained. It is not adequately explained as RViz transport flicker.

Production certificate relation:

~~~text
queried_clearance >= planning_safe_distance + geometric_cover + cover_epsilon
~~~

geometric_cover contains midpoint-position, normal, delta-slope, maximum-width*v-span, and 0.5*snapshot_resolution support.

Previous instrumentation established fixed-input determinism 20/20 and valid arithmetic/observational timing, but did not safely isolate fixed-path/varying-snapshot or fixed-snapshot/varying-path. Its mixed population was 46 quantifiable terminal failures: 7 negative dense margins, 30 near zero, 4 above +0.02 m. Prior Sol verdict was P1_CONSERVATISM_DIAGNOSIS_EVIDENCE_NOT_ACCEPTABLE solely because causal isolation was missing.

## 4. Source ownership analysis

### 4.1 Semantic path owner

FLAG_Race::ContinuousPhasePath owns ordered immutable Segment values with phase range, identity, label, point evaluator, and optional cell-bound evaluator. evaluate(w,false) supplies p, dp_dw, d2p_dw2, velocity, and provenance. cellBounds(w0,w1,certificate) delegates to the exact segment proof and binds path revision/segment identity.

Production owners are mapped B-spline evaluators, quintic-Hermite C2 connectors, and retained slices. makeMappedBspline captures value copies of UniformBspline/derivatives, executable arc-length map, and phase mapping. makeQuinticHermite captures exact polynomial coefficients and endpoint derivatives. The owner is therefore self-contained behind shared_ptr<const ContinuousPhasePath>, although arbitrary std::function closures are not generically serializable.

### 4.2 Frame owner

ContinuousPhaseNormalFrame is constructed for one immutable path and path/frame revisions. It derives Horizontal-N as normalize(e_z x p_w), computes N_w from the same derivatives, and exposes query/certifyCell with canonical provenance. Replay must retain the exact path/frame shared_ptr pair; never recompute N from a separately sampled path or mix owners.

### 4.3 Builder and Validator

CertifiedTubeBuildInput carries preview samples; DistanceQuery/ClearanceQuery; PathStateQuery/PathCellBoundQuery; cloud snapshot resolution; current phase/delta; source/tube revisions; and map observation identity. CertifiedTubeBuilder runs production TubeBuilder, TubeFilter, then TubeSurfaceValidator.

TubeBuilder::buildCloudClearance performs adaptive exact path samples and centerline clearance queries. BuildCertifiedCellInsets consumes complete cell certificates and applies zero local erosion; Validator owns continuous cover. Validator samples exact 3x3 cell surfaces, recursively subdivides, and checks safe distance plus cover.

### 4.4 Immutable snapshot owner

plan_env::CloudOccupancySnapshot contains validity, observation sequence/time, global/observed AABBs, grid origin/count, resolution, included map inflation, and the authoritative dense occupied-voxel vector. Its private column index is only an acceleration cache validated against public data and may be absent.

queryCloudOccupancySnapshotClearance measures distance to closed occupied voxel volumes, not centers. It caps certified free results at requested radius and returns UNKNOWN outside the observed AABB. makeCloudOccupancyClearanceQuery captures shared_ptr<const CloudOccupancySnapshot> and immutable query configuration.

## 5. Path capture architecture

Capture at the immutable TubeBuildRequest/timer-build seam, before profile mutation, including semantic_path_owner, frame_owner, supplied/collected samples, source/path/frame revisions, segment identity/ranges, phase range, current state, and exact Builder/Filter/Validator configuration.

Primary replay retains the owner pair directly and invokes the same owner-backed MakeTimerPathStateQuery and MakeTimerPathCellBoundQuery factories. This preserves p, p_w, p_ww, Horizontal-N, frame continuity, cell partition, and complete variation bounds exactly.

Generic serialization of std::function is forbidden. If cross-process persistence is required, add an explicit serialization contract only for concrete mapped-B-spline, quintic-Hermite, and slice segment kinds, with exact coefficients/control points, phase maps, segment identities, and certificate round-trip tests. Sparse linear interpolation is forbidden because it changes derivatives, N, variation bounds, or partitioning.

## 6. Snapshot capture architecture

Capture the exact immutable snapshot shared_ptr, cloud query configuration, resolution, included inflation, observation sequence/time, AABBs/grid metadata, and SHA256 of the occupied vector.

For persistence, serialize all public snapshot fields and the complete occupied vector in a versioned format. On load validate cloudOccupancySnapshotConsistent, metadata, vector length/hash, and query equivalence; optionally rebuild the private index. Never save only observed position-to-clearance pairs because counterfactual Builder output produces new positions.

## 7. Serialization/deserialization design

Preferred implementation is an in-process immutable capture registry retaining owner pairs and snapshots for one diagnostic run. It is guarded by a diagnostic-only mutex, immutable after capture, and never consulted by production authority/control.

If restart is required, use versioned path and snapshot files. Reject unknown versions, non-finite values, invalid ranges, owner/frame revision mismatch, certificate round-trip mismatch, or snapshot hash mismatch. Deserialized records are immutable.

## 8. Full replay architecture

~~~text
exact captured path + exact frame + exact immutable snapshot + exact configuration
    -> production CertifiedTubeBuilder
    -> production TubeBuilder
    -> production TubeFilter
    -> production TubeSurfaceValidator
    -> diagnostic evidence only
~~~

Each replay constructs fresh query closures from selected immutable owners. Never reuse a candidate generated under a different path/snapshot. The diagnostic executable/harness must not invoke Runtime installation, scheduler timers, authority, Recovery, or command publication.

## 9. Replay equivalence contract

Before A/B/C, replay one original real PASS and one original real FAIL. Require exact booleans/classifications/completeness, cell count, first failing cell and w interval, relevant bounds, and numerically matching required/minimum clearances, residual, and all cover components.

Identity/revision booleans match bit-for-bit. Finite scalars use max(1e-12, 32*binary64_epsilon*max(1,abs(x))) unless an existing source tolerance is explicitly looser; vectors use the same componentwise rule. Any mismatch sets REPLAY_EQUIVALENCE=FAIL and stops causal experiments.

## 10. Observational instrumentation

Disabled by default and enabled only by explicit diagnostic configuration. It may read, count, hash, serialize, and write evidence, but may not change comparisons, tolerances, geometry, query locations/order/count, branches, return values, scheduling, authority, or lifecycle.

Production queries must execute once in the same order with instrumentation enabled. Dense and replay queries run only after production results are fully determined and use separate diagnostic counters/callbacks; they cannot affect later production state.

## 11. Experiment A — fixed path + fixed snapshot

Select one exact real path/frame/snapshot/configuration pair near the intermittent region. Run full Builder+Validator at least 20 times without mutation. Record owner hashes, revisions, classification, first failure, required/minimum clearances, residual, and cover terms. Require 20/20 identical outcomes and stable values; any alternation stops the remaining experiments.

## 12. Experiment B — fixed path, varying snapshot

Hold one exact semantic path/frame owner and path-side configuration fixed. Pair it only with multiple real immutable snapshots from the same physical region. For every snapshot rebuild a fresh candidate through Builder and Filter before Validator; never hold a candidate fixed while changing snapshot.

Record map sequence/time/hash, raw/filtered bounds, classification, first failing cell/w, cover breakdown, required/minimum clearance, residual, dense minimum/margin, and final result.

Compute:

~~~text
SNAPSHOT_INDUCED_RESIDUAL_SPAN_M = max(residual) - min(residual)
SNAPSHOT_ONLY_PASS_FAIL_FLIP = YES iff results differ while path owner/configuration
                               and replay equivalence remain fixed
~~~

Path owner pointer/revision/frame hash must be identical in every row; otherwise B is invalid.

## 13. Experiment C — fixed snapshot, varying path

Hold one exact immutable snapshot, query configuration, resolution, included inflation, and planning safe distance fixed. Pair it with several real captured path/frame owners from the same region. Rebuild each candidate through full Builder+Filter+Validator.

Record path/frame revisions and hashes, candidate geometry, raw/filtered bounds, midpoint/normal/delta-slope/v-span/half-voxel cover, required/minimum clearance, residual, dense margin, and final result.

Compute:

~~~text
PATH_INDUCED_RESIDUAL_SPAN_M = max(residual) - min(residual)
PATH_ONLY_PASS_FAIL_FLIP = YES iff results differ while snapshot hash/metadata and
                           replay equivalence remain fixed
~~~

Any snapshot mismatch invalidates C.

## 14. Full pipeline versus Validator-only

All causal conclusions and PASS/FAIL flips must come from path+snapshot -> Builder -> Filter -> Validator. A frozen-candidate Validator-only probe may isolate cover-term sensitivity, but must be labeled VALIDATOR_ONLY_COUNTERFACTUAL and cannot support path/snapshot causal claims. Candidate and owner mixing is forbidden.

## 15. Per-cell/query evidence schema

Versioned schema must include run/capture/build identity; candidate identity/classification; source/path/frame revisions; map sequence/hash; snapshot resolution/included inflation; cell index/depth/w interval/v span; raw and filtered endpoint bounds; maximum delta/width; midpoint bound/cover; normal bound/cover; delta-slope/cover; width-v-span cover; half-voxel; geometric cover without half voxel; total cover; safe distance; epsilon; required/requested clearance; query index/position/surface-side-probe role; clearance/status/certified flag/PASS-FAIL/residual; cell min clearance/max requested radius/worst residual/first failure; and final Validator result.

Separate manifest stores exact owner hashes and configuration. CSV headers/field counts are stable, strings escaped, and NaN/inf require explicit validity fields.

## 16. Dense diagnostic protocol

Dense scan is empirical only and never replaces continuous proof. For terminal failures and representative accepted cells, evaluate the complete surface set over deterministic tensor grids in w and v using the same immutable snapshot. Query radius must be at least the exact cell requested radius so API capping cannot hide positive margin.

Perform N/2N/4N samples per axis on representative cells and report change in dense minimum. A positive margin is called converged only under a tolerance justified by binary64 error, query tolerance, map resolution, and observed convergence.

## 17. Half-voxel source/math audit

Use source and replay evidence to document that clearance is to closed occupied voxel volumes. Determine why Validator adds 0.5*snapshot_resolution, whether it is isotropic support, whether query geometry already owns voxel support, and whether a tighter exact support function exists.

Compare occupied centers, closed volumes, included map inflation, and query-ball containment. Do not remove or weaken the term during this diagnostic.

## 18. Double-accounting audit

Trace independently: snapshot included_map_inflation; configured pre-included uncertainty; planning/safe distance; local occupancy expansion; closed-voxel clearance geometry; and Validator half-voxel support. Identify the geometric object/uncertainty for each and conclude only NO_DOUBLE_ACCOUNTING_FOUND, POTENTIAL_DOUBLE_ACCOUNTING_FOUND, or DOUBLE_ACCOUNTING_CONFIRMED. Two nonzero values alone are not double accounting.

## 19. v-span tightness audit

Compare analytic 0.5*maximum_width*v_span with observed dense transverse/frame-induced variation over the same cell. Report absolute slack and ratio with convergence state. Classify TIGHT, MODERATELY_CONSERVATIVE, STRONGLY_CONSERVATIVE, or INSUFFICIENT_EVIDENCE. Do not change the production term.

## 20. Failure classification

After replay and convergence classify:

- PHYSICAL_FAILURE: converged dense physical margin clearly below zero.
- BORDERLINE_PHYSICAL: margin near zero within justified tolerance.
- CERTIFICATE_ONLY_REJECTION: residual below zero while converged dense margin clearly positive and identifiable cover terms dominate.
- INCONCLUSIVE: unavailable/unstable evidence or failed equivalence.

Tolerance must derive from binary64 error, source query tolerances, dense convergence, and 0.1 m map resolution, not desired outcome.

## 21. Material-conservatism criteria

MATERIAL_CERTIFICATE_CONSERVATISM=CONFIRMED requires repeated real full-pipeline A/B/C evidence with negative certificate residual, clearly positive converged dense margin, and identifiable dominant analytic term(s). One cell or one pair is insufficient. Otherwise report BORDERLINE, NOT_SUPPORTED, or INSUFFICIENT_EVIDENCE.

## 22. Root-cause ranking

Rank physical clearance, snapshot variation, path geometry variation, half-voxel support, v-span cover, remaining analytic cover, and implementation defect as PRIMARY, SECONDARY, MINOR, or NOT_SUPPORTED. Cite residual spans, valid PASS/FAIL flips, dense convergence, and equivalence. Uncontrolled pointer/revision correlation cannot receive a causal rank.

## 23. Future Luna whitelist

Future explicit authorization is required. Luna is DIAGNOSTIC-ONLY IMPLEMENTATION WRITER and may work only in an isolated worktree. Select the smallest subset of:

1. bspline_race/integration/phase_offset_matched_adapter.h/.cpp: diagnostic immutable capture ring and owner/hash handoff at TubeBuildRequest/timer build seam; no scheduler/worker/currentness/authority/lifecycle/publication changes.
2. certified_tube_builder.h/.cpp: read-only build identity hook only if adapter seam cannot expose it; no pipeline/result changes.
3. tube_surface_validator.h/.cpp: observational per-cell/query hook and deferred dense utility only; no formula/branch/query/order changes.
4. tube_builder.h/.cpp: read-only capture of already-created inputs/outputs only if adapter seam cannot expose them; no geometry/query changes.
5. plan_env/cloud_occupancy_snapshot.h/.cpp: versioned public snapshot serialization and round-trip validation only; no query semantics.
6. continuous_phase_path.h/.cpp and continuous_phase_normal_frame.h/.cpp: concrete-owner serialization only if cross-process replay requires it; preserve exact evaluator/cell-bound/Horizontal-N semantics.
7. New isolated diagnostic executable/script/test for capture manifests, replay, A/B/C, dense convergence, and reporting.

If in-process shared owners suffice, path serialization hooks are not authorized. No production launch, scheduler, Runtime, Recovery, Handoff, TubeEpochManager, map authority, or command files may be modified.

## 24. Protected/excluded scope

Never modify P1 geometry policy, nominal width, side clipping, TubeFilter semantics, TubeEpochManager, Runtime, Recovery, Handoff, gvf_manager/planner/governor, Contract A, safe distance, launch semantics, ROS authority, Stage 1B/P2/P3, accepted commit history, unrelated dirty files, or broad build/test infrastructure.

No cleanup, stash, reset, restore, stage, commit, or push. If a required edit is outside the conditional whitelist, STOP.

## 25. Build/test protocol

In isolation: build affected libraries and diagnostic target; run baseline tests before/after; compare disabled and enabled production decisions/query order/count; test owner/snapshot round trips; run replay PASS/FAIL equivalence; A 20x; full-pipeline B/C; dense N/2N/4N; half-voxel/accounting/v-span analyses; diff check and whitelist verification. Preserve any inherited Validator waiver and record it explicitly.

## 26. Capture/ROS protocol

Use a private ROS master, private ROS_HOME/logs, and only task-started processes. Capture actual paired path/frame owners and immutable snapshots with bag, launch parameters, binary/source hashes, map sequence, and goal timeline. Do not attach to or terminate user-owned processes. If paired owners cannot be exposed without protected scheduler/authority edits, STOP and mark the experiment unavailable.

## 27. Analysis outputs

Produce replay-equivalence table; A 20-row determinism table; B snapshot table and SNAPSHOT_INDUCED_RESIDUAL_SPAN_M; C path table and PATH_INDUCED_RESIDUAL_SPAN_M; per-cell/query certificate table; dense convergence and physical margins; cover-term ranking; half-voxel audit; double-accounting verdict; v-span classification; root-cause ranking; material-conservatism classification; owner/snapshot hashes; and repository hygiene manifest.

## 28. STOP conditions

STOP without causal claims if repository identity differs; exact owner/snapshot cannot be captured; sampled interpolation or mixed owners is required; replay equivalence fails; instrumentation changes production behavior; B/C identity invariants fail; dense results are capped/invalid/non-convergent; required file is outside whitelist; worktree cleanup or unrelated edits are needed; or evidence supports only uncontrolled correlation.

## 29. Repository hygiene

Before/after any future implementation verify branch main, HEAD and origin/main equal 9e91f370fc1c44eb4ada8ffd8d209adbab66d688, and staged paths=0. Preserve AGENTS.md, .codex, Testing/, historical docs, __pycache__, and all other user-owned dirty/untracked content. Diagnostic changes remain isolated and uncommitted.

## 30. Future implementation handoff

This Candidate Plan is not implementation authorization. A later Main window must obtain one fresh Sol xhigh PLAN audit. Only Sol verdict P1_CAUSAL_REPLAY_DIAGNOSTIC_PLAN_ACCEPTABLE permits a semantically identical Frozen Plan and Handoff. Only a later explicit user authorization may launch Luna.

Luna must read the Frozen Plan, use a new isolated worktree based on the accepted P1 commit, implement only the final whitelist, and stop on source conflicts. Main owns build/test/evidence orchestration. No production commit or push is authorized.



## Freeze and audit metadata

Fresh Sol xhigh PLAN audit: `P1_CAUSAL_REPLAY_DIAGNOSTIC_PLAN_ACCEPTABLE`.

Audit fields: REPLAY_FIDELITY_PLAN=PASS; OBSERVATIONAL_INSTRUMENTATION_PLAN=PASS; REPLAY_EQUIVALENCE_GATE=PASS; SNAPSHOT_ISOLATION_PLAN=PASS; PATH_ISOLATION_PLAN=PASS; FULL_PIPELINE_CAUSAL_PLAN=PASS; DENSE_DIAGNOSTIC_PLAN=PASS; HALF_VOXEL_ACCOUNTING_PLAN=PASS; V_SPAN_TIGHTNESS_PLAN=PASS; MATERIAL_CONSERVATISM_CRITERION=PASS; FUTURE_LUNA_SCOPE=PASS. BLOCKING_FINDINGS=NONE.

