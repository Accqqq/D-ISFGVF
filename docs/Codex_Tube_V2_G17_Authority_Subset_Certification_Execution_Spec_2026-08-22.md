# Tube V2 G17 authority-subset certification execution spec

    DOCUMENT_ROLE=PRODUCT_CORRECTION_EXECUTION_SPEC
    DOCUMENT_STATUS=ACTIVE
    DATE=2026-08-22
    PLAN_OWNER=PRIMARY_CODEX_AGENT
    EXECUTOR=LUNA_MAX_SINGLE_AGENT
    SUBAGENT_CREATION_ALLOWED=false
    DELEGATION_ALLOWED=false
    PARALLEL_AGENT_WORK_ALLOWED=false
    IMPLEMENTATION_AUTHORIZED=true
    NO_NEW_GATE_CACHE_RETRY_TIMER_MODULE=true
    NO_LIMIT_MARGIN_RATE_HORIZON_CHANGE=true

## 1. Goal and evidence

G17 fixes the remaining certified-construction owner without changing raw
environment geometry or inventing M6 recovery.

Observed production facts:

- ESDF raw candidate geometry correctly searches up to environment
  search_extent=3.0 m per side;
- large raw candidates had same-knot physical widths of 3.275 m and 4.55625 m;
- the certified topic had zero ADD messages;
- full raw validation reached 249993/250000 queries;
- exact-current zero-capacity cohorts previously entered 101 inward attempts,
  now removed by G16 D0;
- an attempted minimal M6 recenter was independently proven physically unsafe
  under production phase/velocity slew and is being fully reverted.

The raw candidate size is not a geometry bug.  It is I_geo, the environmental
free-space interval.  The defect is asking SurfaceValidator to certify the
whole I_geo even though Runtime can only request a much smaller offset
authority set.

G17 therefore separates, without a new persistent owner:

    raw/environment evidence I_geo
    -> unchanged TubeFilter local-PWL evidence
    -> post-filter authority candidate I_geo intersect I_authority
    -> existing SurfaceValidator
    -> existing Runtime dry-run and atomic Pair

## 2. Frozen contracts

The following are unchanged:

- TubeCrossSection search_extent, ray_step, curvature, clearance, and margins;
- raw_lower/raw_upper, environment bounds, and raw_build_samples;
- candidate marker continues to show current authoritative raw I_geo and stays
  explicitly UNCERTIFIED;
- TubeFilter still outputs filtered endpoints equal to raw endpoints before
  CertifiedTubeBuilder authority selection;
- zero-connected normal Tube and exact current anchor;
- SurfaceValidator continuous cover, UNKNOWN/OOM/query/depth behavior;
- Runtime exact-PWL, PortProjector, rates, margins, and future witness;
- nonzero PathTubePair handoff remains atomic;
- neutral planner is not gated by Tube;
- fixed_delta_max remains FIXED-source geometry and is not reused for ESDF;
- max_offset and search_extent are not controller authority limits.

Do not modify CrossSection, TubeBuilder raw geometry, TubeFilter production,
SurfaceValidator production, Runtime, manager, planner, launch, parameters,
ROS schema, query limits, safety margins, rates, or horizons in G17.

## 3. Mandatory preflight

Before G17 product edits:

1. Finish the exact inverse rollback of all G16 E/M6/E-R1 product and test
   changes.
2. Preserve G15 A/B/C, G16 D0, marker ownership, and the integration Pair
   oracle corrections.
3. Required focused baseline:
   - Runtime 35;
   - Adapter 83;
   - GVF switch 49;
   - CertifiedBuilder 12;
   - Integration 10;
   - git diff --check clean.
4. Confirm no new production requestReturnToCenter/returningToCenter/RECENTER
   symbols remain beyond the pre-existing Runtime preflight lifecycle.
5. Record hashes and results under a fresh /tmp/tube_v2_g17_<id>/ directory.

## 4. Authority request contract

### 4.1 Internal value only

Reuse the existing TubeBounds value type as an immutable authority request.
Only lower, upper, and valid are meaningful; derivative fields remain zero.

Propagate one value through existing internal C++ inputs:

    adapter command lock capture
    -> TubeBuildRequest
    -> TubeEpochUpdateInput
    -> CertifiedTubeBuildInput
    -> CertifiedTubeBuilder

This is not a ROS parameter, schema, state machine, cache, gate, or installed
authority.  It is transaction input, like retained_delta.

### 4.2 Conservative request formula

Under adapter runtime_command_mutex_, capture the same retained delta and
task/source/session/map identity already used by the build request.

Let:

    A = abs(configured manual amplitude)
    m = existing Runtime interior_margin
    S_lower = min(-A, retained_delta, 0)
    S_upper = max(+A, retained_delta, 0)
    request.lower = S_lower - m
    request.upper = S_upper + m

Requirements:

- all values finite;
- A uses configured amplitude, not fixed_delta_max, max_offset, search_extent,
  or retained delta;
- configured amplitude is used deliberately because new-owner accepted
  amplitude is computed only after Tube build during H2 dry-run, creating a
  circular dependency;
- configured amplitude is a conservative superset of any preflight-reduced
  accepted amplitude;
- retained delta is included for active H2 replacement even if it lies outside
  configured amplitude;
- zero is always included;
- the existing interior margin is added exactly once so every required actual
  delta can satisfy Runtime's inside predicate;
- do not add tracking, clearance, regularity, or invariant padding.  Those
  remain in their existing owners.

Invalid request input fails the ESDF candidate build before validation with an
existing input/precondition result.  It is never replaced by raw full-width
validation.

### 4.3 Scope by source

- ESDF categorical production requires a valid authority request.
- FIXED source behavior and fixed_delta_max remain unchanged.
- NONE source remains unchanged.
- observe-only/manual sidecar may build the same authority candidate for
  diagnostics, but it gains no Pair or control authority.

## 5. CertifiedBuilder authority candidate

### 5.1 Preserve raw and G1 Filter evidence

Run existing Builder and TubeFilter first.  Preserve their result unchanged as
raw/environment evidence and raw_build_samples.

Do not overwrite raw_lower/raw_upper, environment bounds, fixed compatibility
fields, Builder provenance, snapshot identity, current anchor, or the raw
candidate marker source.

### 5.2 Post-filter intersection

Create a local copy for certification.  At each sample:

    cert_lower = max(filtered_lower, request.lower)
    cert_upper = min(filtered_upper, request.upper)

The request and I_geo both contain zero, so their valid intersection remains
zero-connected.  Reject/fall back zero-only if the exact current intersection
has no nonzero capacity.

For the certification copy only:

- write cert_lower/cert_upper into filtered_lower/filtered_upper;
- recompute exact local one-sided PWL slopes and min_width;
- preserve raw/environment/raw_build_samples;
- preserve Builder truncation and snapshot provenance;
- do not interpolate across an invalid/incomplete/remote gap;
- do not add crossing knots.  Knotwise intersection with a constant interval
  is conservative; any extra contraction is acceptable and must be tested.

Generalize/reuse the existing post-filter inward-candidate helper rather than
introducing a second narrowing implementation.

### 5.3 Validation and inward behavior

SurfaceValidator's first full-width attempt now means full requested authority,
not full environmental I_geo.

- validate the authority candidate with the existing immutable snapshot and
  exact current/path/cell queries;
- if it succeeds, install only that validated narrow profile;
- if it fails with an existing retryable classification, existing deterministic
  inward search may operate only inside the authority candidate, never restart
  from raw I_geo;
- G16 D0 remains: exact-current zero capacity skips all inward attempts;
- query terminal, INVALID_PATH, UNKNOWN, OOM, incomplete cover, and unsafe
  results remain fail-closed;
- do not increase/decrease attempt, query, or depth limits in G17.

The final profile delivered to Epoch/Runtime is exactly the narrow profile that
SurfaceValidator certified.  Raw candidate diagnostics and markers remain the
full I_geo.

## 6. Files in scope

Allowed product files only as required:

- certified_tube_builder.h/.cpp;
- tube_epoch_types.h and tube_epoch_manager.cpp;
- phase_offset_matched_adapter.h/.cpp;
- existing focused tests for CertifiedBuilder, EpochManager, adapter, markers,
  integration, and Builder/Filter/Validator regressions;
- G17 spec/self-audit and /home/cxq/ISF-GVF/handoff.md.

Prefer no TubeProfile schema expansion.  Use existing TubeBounds in internal
input structures.  SurfaceValidator, TubeBuilder, TubeFilter, Runtime,
gvf_manager, launch, CMake, and ROS messages are read-only.

## 7. Required focused tests

### 7.1 Request construction

- configured amplitude 0.10, retained 0, margin 0 -> [-0.10,+0.10];
- retained -0.15 -> [-0.15,+0.10];
- retained +0.16 -> [-0.10,+0.16];
- nonzero interior margin expands both sides exactly once;
- accepted/preflight amplitude changes do not alter the configured conservative
  request for H2;
- fixed_delta_max=0.04 does not clamp ESDF request or raw geometry;
- invalid finite/order facts fail closed.

### 7.2 Raw versus certified profile

- search_extent=3 produces raw_build_samples near [-3,+3];
- authority request [-0.10,+0.10] leaves raw/environment evidence unchanged;
- certification profile filtered bounds stay within [-0.10,+0.10];
- candidate marker remains broad raw I_geo;
- certified marker/profile is narrow and only ADDs after validation;
- FIXED source remains controlled by fixed_delta_max.

### 7.3 Safety and cost

- wide open raw I_geo with narrow authority validates with materially fewer
  queries/v-splits than raw full-width baseline;
- an obstacle outside I_authority does not reject the narrow authority profile;
- an obstacle inside I_authority still rejects/fails closed;
- current retained delta plus interior margin is contained after certification;
- exact-current zero capacity returns existing zero-only result and performs no
  inward loop;
- narrow retryable failure never widens back to raw I_geo;
- UNKNOWN/OOM/query terminal/invalid path remain unchanged;
- Runtime dry-run still rejects a statically certified but dynamically
  infeasible profile;
- atomic Pair pointer/profile/status identity remains exact.

Do not use wall-clock time as the proof.  Assert Validator invocation,
query_sample_count, geometry/leaf cells, and v-split counts.

## 8. Static verification

Run in order and preserve logs:

1. CertifiedBuilder, SurfaceValidator, TubeBuilder, TubeFilter;
2. EpochManager;
3. Adapter and marker tests;
4. integration and GVF regressions;
5. Runtime and continuous path regressions;
6. all related direct binaries;
7. catkin_make -j2;
8. git diff --check;
9. source/binary hashes and forbidden-change audit.

Audit must show:

- no M6/recenter product code;
- no new gate/cache/retry/timer/module/state;
- no limit/margin/rate/horizon/default/launch change;
- no raw geometry or marker contraction;
- no candidate-to-certified promotion;
- only post-filter authority copy is narrowed.

## 9. Dynamic split-launch acceptance

After static pass, use a fresh private master and /dev/shm evidence if root disk
remains below rosbag's threshold.  Recorder starts before publishers.  Use the
unchanged separate simulator.launch and user test_gvf.launch command/parameters.

Run known G1--G5 sequentially.  Do not skip a timed-out goal.

Required evidence:

- raw candidate still visibly shows broad I_geo where the map permits it;
- certified Tube shows a narrow profile only after real validation;
- at least one nonzero certified Pair commits and executes;
- large raw cohorts no longer produce the former 249993 full-authority query
  explosion unless the narrow request itself is genuinely complex;
- D0 cohorts perform no 101 inward attempts;
- candidate and certified topics remain semantically separate;
- H2 replacement commits a matching narrow authority profile or fails with a
  direct safe classification;
- neutral navigation remains unaffected;
- G1 through G5 all reach and no stable 36/0/36 HOLD occurs for FIXED status.

If certified remains zero ADD, any goal times out, H2 cannot install, or query
terminal persists on the narrow request, label PARTIALLY_FIXED/NOT_FIXED and
retain direct evidence.  Do not re-enter M6, change parameters, or call the raw
candidate a bug.

## 10. Completion status

Only write FIXED when static tests pass and the unchanged five-goal user
split-launch passes.  Report separately:

- raw I_geo size;
- requested authority interval;
- certified interval;
- Validator cost before/after;
- Pair/H2 timeline;
- navigation outcomes;
- remaining inward/query failures.
