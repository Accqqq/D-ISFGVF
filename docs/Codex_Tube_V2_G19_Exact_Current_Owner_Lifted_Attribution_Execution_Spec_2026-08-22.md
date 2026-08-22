# Tube V2 G19 exact-current owner/lifted-reference attribution execution spec

    DOCUMENT_ROLE=READ_ONLY_PRODUCT_ATTRIBUTION_SPEC
    DOCUMENT_STATUS=ACTIVE
    DATE=2026-08-22
    PLAN_OWNER=PRIMARY_CODEX_AGENT
    EXECUTOR=LUNA_MAX_SINGLE_AGENT
    SUBAGENT_CREATION_ALLOWED=false
    AUTO_ADVANCE=false
    PRODUCT_LOGIC_CHANGE_ALLOWED=false
    NAVIGATION_RECOVERY_CHANGE_ALLOWED=false
    NO_NEW_GATE_CACHE_RETRY_TIMER_MODULE_STATE=true
    NO_LIMIT_MARGIN_RATE_HORIZON_LAUNCH_CHANGE=true

## 1. Current result and exact question

G18 is only `PARTIALLY_FIXED`:

- Tube construction cost/explosion is substantially reduced;
- Candidate and Certified Tube visibility is fixed;
- G1 reaches its goal, but G2 times out after the old Pair path is exhausted;
- 1,173 exact `36/0/36 HOLD` decisions persist for about 232.8 s;
- no system-wide `FIXED` claim is allowed.

The first failing G2 H2 transaction is:

    captured_w=9.828509478
    future_seam_w=10.263573946
    new_owner_end_w=15.069005520
    retained_delta=-0.090206323 (nearby observed retained value)
    map_observation_sequence=1131
    candidate_classification=ZERO_ONLY_PLANNER_BASELINE
    exact_current_raw_interval=[0,0]
    stage_failure=TUBE_BUILD_PRECONDITION

The successful C2 log immediately before it is:

    mode=point
    future_seam_w=10.263573946
    join_delta_w=1.0
    exact_path=1

For an H2 timer handoff, the new immutable owner is intended to contain:

    [captured_w, future_seam_w] = exact slice of the old owner
    [future_seam_w, join_w]     = C2 quintic connector
    [join_w, end_w]             = mapped new B-spline

G19 answers exactly one question before any repair is authorized:

> At the first exact-current `[0,0]` H2 failure, are the old Pair owner, the
> new C2 owner, the Builder current anchor, the active lifted reference
> `r = p + N*delta`, and the immutable cloud snapshot geometrically and
> provenance-wise identical where they are required to be identical?

G19 does not repair the failure.  It adds bounded transaction-local evidence,
tests that the evidence is observational only, performs one split-launch ROS
run, classifies the first false, cleans up its own processes, and reports.

## 2. Frozen product contracts

The executor must not change any of the following:

- Builder, Filter, Validator, Runtime or PortProjector decisions;
- raw cross-section geometry, authority-subset certification, D0 exact-zero
  shortcut, query/depth/attempt limits, safety margins or tolerances;
- C2 construction, prefix interval, connector, mapped branch or seam choice;
- Pair staging/commit/CAS, authority retirement or neutral planner fallback;
- controller, governor, planner selection, HOLD decision or command output;
- timer cadence, callback behavior, retry behavior, launch files or maps;
- ROS messages, topics, parameters or persisted diagnostics schemas;
- G18 Candidate/Certified publication ownership.

Forbidden additions include a gate, cache, retry, timer, watchdog, mailbox,
state machine, module, recovery command, recenter behavior, direct neutral
switch, forced delta reset or enlarged authority.

Do not reintroduce or prototype the rejected M6/effective-base recenter idea.

## 3. Authorized files

Production source whitelist:

    src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
    src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

Focused test whitelist:

    src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

Documentation whitelist:

    docs/Codex_Tube_V2_G19_Exact_Current_Owner_Lifted_Attribution_Execution_Spec_2026-08-22.md
    docs/Codex_Tube_V2_G19_Exact_Current_Owner_Lifted_Attribution_Self_Audit_2026-08-22.md

The adapter header may change only to add one optional transaction-local
`bool*` output to the private `buildPreparedTubeEpoch()` method so the exact
`staged_manager.update()` return value reaches the G19 evidence line.  It must
default to null, must not be stored, and must not change any other caller.
No public product type, ROS interface or decision predicate may change.  If
any other header or production file is required, stop before editing it and
report the exact missing interface to the primary agent.  Do not expand the
whitelist yourself.

## 4. Phase A: transaction-local evidence

### 4.1 Emission boundary

Add one adapter-local helper and call it only from the existing
`stagePathTubePair()` failure branch when all of these are true:

1. the stage has an `expected_pair` with a non-null old `path_owner`;
2. `buildPreparedTubeEpoch()` returned false;
3. `prepared_build_failure_layer == "tube_update_status"`;
4. `classifyTubeUpdateStatusStageFailure(prepared_epoch)` equals
   `TUBE_BUILD_PRECONDITION`;
5. the prepared Candidate contains a raw sample at the exact captured phase
   within the existing `kTubeCurrentPhaseAnchorTolerance` contract;
6. that raw sample has `raw_lower == 0` and `raw_upper == 0` within the existing
   capacity tolerance.

The helper is observational.  It returns no predicate consumed by staging,
does not modify the prepared epoch, and must not change the existing return
path or failure enum.  Do not use a persistent "already logged" bit.  One line
per affected H2 transaction is acceptable.

Use the exact immutable values already owned by this transaction:

- `expected_pair->path_owner` as old owner;
- `new_path_owner` as new owner;
- `captured_w0` as the only current phase;
- `retained_delta` captured under `runtime_command_mutex_`;
- `position` passed to staging;
- `frozen_cloud_occupancy_snapshot` used by the failed build;
- `prepared_epoch` produced by the single actual build.

Do not read a newer timer epoch, newer map snapshot, mutable SDF buffer or live
Runtime state during attribution.  Do not rebuild the Tube.

### 4.2 Exact raw-sample selection

Search `prepared_epoch.candidate_profile.raw_build_samples` for the sample
whose `w` exactly equals `captured_w0` by `ExactDoubleBits`.  If no bitwise
match exists, allow only the already frozen
`kTubeCurrentPhaseAnchorTolerance` match and log `raw_anchor_exact_bits=0`.
Log the number of within-tolerance matches.  It must be exactly one; do not
choose silently when it is not.

Record, without altering values:

- `raw_lower`, `raw_upper`, `filtered_lower`, `filtered_upper`;
- `p`, `N`, `base_signed_distance`;
- `c_minus_raw`, `c_plus_raw`;
- `pre_inset_lower`, `pre_inset_upper`, `continuous_inset`;
- `environment_lower`, `environment_upper`, `environment_width`;
- `environment_interval_nonempty`, `environment_contains_zero`,
  `pre_inset_contains_zero`, `post_inset_contains_zero`,
  `filter_input_contains_zero`, `filtered_contains_zero`;
- integer `cross_section_reason`, `positive_ray_termination`,
  `negative_ray_termination`;
- `cell_geometry_certificate_used`, `cell_geometry_inset`;
- Candidate `raw_complete`, `filtered_complete`, `complete`, classification,
  obstacle certification and first stop facts;
- all status facts which currently collapse into `TUBE_BUILD_PRECONDITION`:
  `staged_update_ok`, `candidate_complete`, `active_available`,
  `active_current_validation_valid`, candidate/active source revision,
  candidate/active map sequence, snapshot bit, active classification,
  and current-interval/retained-delta facts.

Expose `staged_update_ok` through the single optional private-method output
authorized in Section 3.  Existing callers omit it and remain unchanged.

### 4.3 Owner segment provenance

For both old and new owners, inspect the public immutable `segments()` vector
using the same inclusive ordered selection rule as
`ContinuousPhasePath::evaluate()`:

    captured_w >= segment.w0 - epsilon
    captured_w <= segment.w1 + epsilon
    first matching segment wins

Do not add an API or change `ContinuousPhasePath`.

Record for each owner:

- owner pointer identity only as a diagnostic hexadecimal address;
- owner start/end and segment count;
- selected segment index, identity, label, `w0`, `w1`;
- number of segments matching the inclusive rule;
- whether the selected new segment ends exactly at `future_seam_w`;
- whether `captured_w0 < future_seam_w` and therefore the point is required
  to be in the copied prefix.

Pointer addresses are evidence only and must never participate in a decision.

### 4.4 Old/new owner differential and lifted-reference equality

Evaluate both owners at the same exact `captured_w0` with the existing
`EvaluateOwnerState()` helper.  Use the unchanged production
`GeometryEvaluator` configuration and the same actual `position` and
`retained_delta` to compute both geometries.

Record both values for:

- path: `p`, `p_w`, `p_ww`;
- geometry: `N`, `N_w`, `curvature`, `regularity`, `r`, `r_w`, `T`;
- tracking: `error`, `e_parallel`, norm of `e_perp`;
- validity and invalid reason.

Also record direct differences:

    ||new.p     - old.p||
    ||new.p_w   - old.p_w||
    ||new.p_ww  - old.p_ww||
    ||new.N     - old.N||
    ||new.N_w   - old.N_w||
    ||new.r     - old.r||
    ||new.r_w   - old.r_w||
    ||new.T     - old.T||
    |new.curvature - old.curvature|

Record the Builder raw sample differences against the new owner:

    ||raw.p - new.p||
    ||raw.N - new.N||

Do not introduce a new equality tolerance or gate.  Emit numeric differences
and let the offline report compare them against the existing
`kPreparedCoverageTolerance` only.

### 4.5 Same-snapshot clearance facts

Use `makeCloudOccupancyClearanceQuery()` with exactly
`frozen_cloud_occupancy_snapshot` and the existing query configuration.  Make
only four bounded read-only diagnostic queries, all with the unchanged
`config_.tube.cross_section.planner_safe_distance`:

1. old base centre `old.p`;
2. new base centre `new.p`;
3. old retained lifted reference `old.r`;
4. new retained lifted reference `new.r`.

For each log status, returned clearance and `clearance_certified`.  Also log
snapshot pointer, observation sequence, resolution, included inflation and
the requested safe distance.

These four queries must not be passed to Builder/Validator/Runtime and must
not affect their counters, status or decisions.  Do not scan rays or query
other deltas.  This is not a second Tube build.

### 4.6 Log shape

Emit one concise but complete single-line ROS warning beginning exactly:

    [PHASE_OFFSET][G19][H2_EXACT_CURRENT_ATTRIBUTION]

Use stable `key=value` tokens and quoted segment labels/reasons.  Do not emit
per-knot, per-ray or per-Validator-query logs.  The line must include the Pair
generation/session/source revision, new staged revision, map sequence,
captured/seam/owner-end phase, retained delta and actual position.

The existing G18 `[ZERO_BASELINE]`, `[H2][STAGE]` and
`[H2][STAGING_DRY_RUN]` lines remain unchanged.

## 5. Phase B: focused tests

Add tests in the existing adapter test file.  Reuse existing fixtures and
helpers; do not create a harness, executable, ROS node or test package.

Required cases:

1. **Copied prefix equality**
   - construct an old immutable owner with at least two labeled segments;
   - construct a new owner by slicing the old owner through a future seam,
     then append connector/mapped branches;
   - choose captured w strictly before the seam;
   - prove old/new `p,p_w,p_ww,N,N_w,r,r_w,T` agree at retained nonzero delta.

2. **Seam selection attribution**
   - choose captured w before the seam and at the exact seam;
   - prove the diagnostic selection mirrors the owner's existing first-match
     inclusive rule without changing owner evaluation.

3. **Exact current raw-anchor uniqueness**
   - Candidate raw samples contain the exact current anchor plus nearby
     values;
   - prove exact-bit selection wins and the input profile is unchanged;
   - duplicate within-tolerance anchors must be reported as non-unique, not
     silently selected.

4. **Observational same-snapshot clearance**
   - use a deterministic immutable snapshot query fixture;
   - prove old/new identical points return identical facts;
   - prove four diagnostic queries do not mutate Candidate/Active/Runtime or
     change the stage failure classification.

5. **Non-target silence**
   - raw interval nonzero, raw build failure, filter failure, Validator
     failure, coverage failure and staging dry-run failure do not enter the
     G19 attribution helper.

6. **G18 regression**
   - exact live Pair Candidate/Certified ownership and diagnostic publication
     tests remain unchanged and pass.

Do not make product methods public merely to test them.  Prefer adapter-local
pure helper functions in the `.cpp` translation unit and validate behavior
through existing public test seams.  If that is impossible without a header
change, report before editing.

## 6. Static verification

Record outputs under a new directory:

    /dev/shm/tube_v2_g19_static_<timestamp>/

Run, in this order:

1. focused adapter test binary;
2. Tube marker tests;
3. Tube epoch integration and epoch diagnostics tests;
4. GVF switch-policy tests;
5. Runtime tests;
6. CertifiedBuilder tests;
7. `catkin_make -j2`;
8. `git diff --check`;
9. forbidden-change `rg` audit;
10. before/after hashes for all source files outside the whitelist.

The executor must use existing CMake targets and commands documented by G18.
Do not edit CMake or add a test target.

Static acceptance requires:

- all previously passing G18 groups still pass;
- build passes;
- no forbidden file changes;
- no changed launch/map/parameter/message file;
- the product decision path is semantically unchanged;
- tests demonstrate that evidence collection is observational.

If a pre-existing unrelated fixture fails with unchanged hashes, classify it
separately and do not modify it.

## 7. One split-launch dynamic run

Only after static acceptance, perform exactly one independent ROS run using
the same unchanged split launches and G1--G5 goal procedure as G18.  Check for
user-owned ROS processes first and do not attach to or kill them.

Evidence root:

    /dev/shm/tube_v2_g19_dynamic_<timestamp>/

Required run rules:

- launch `simulator.launch` and `test_gvf.launch` separately;
- do not change launch arguments, parameters, map, goals or timeouts;
- do not rerun because the outcome is undesirable;
- capture both launch logs, rostopic evidence and bag using the G18 procedure;
- send G1--G5 sequentially only while prior goals reach the unchanged bound;
- close the bag cleanly;
- terminate only PIDs/PGIDs created by this run;
- verify ports, ROS master, task PIDs and PGIDs are clean afterward;
- verify source, binaries, launch/map and git before/after state.

The dynamic report must align the first target line with:

- preceding C2 success;
- G19 exact-current attribution;
- existing zero-baseline line;
- H2 stage result;
- any subsequent staging dry-run;
- first `36/0/36 HOLD` and old owner exhaustion.

Also retain the G18 regressions:

- Candidate and Certified ADD/DELETE counts and widths;
- Pair commit count and nonzero executed delta count;
- `249993` count and `attempts=101` count;
- goal outcomes and exact HOLD duration.

## 8. Mandatory classification matrix

The executor reports facts and selects exactly one primary classification.
It must not implement a repair.

### A. `C2_PREFIX_OWNER_MISMATCH_PROVEN`

Select only if `captured_w < seam_w`, the new selected segment is intended to
be the old prefix, but any old/new `p,p_w,p_ww` difference exceeds the existing
owner-match tolerance, or segment provenance shows the connector/mapped branch
was selected before the seam.

Implication: next plan may repair only the proven slice/domain/evaluation bug.

### B. `ACTIVE_LIFTED_CONTINUITY_MISMATCH_PROVEN`

Select only if base path equality holds but `N/N_w/r/r_w/T` differs materially
at the same retained delta.

Implication: next plan may repair only the proven normal/lifted-reference
continuity ownership error; it may not recenter or reset delta.

### C. `BUILDER_CURRENT_ANCHOR_MISMATCH_PROVEN`

Select only if new owner evaluation is valid but the exact raw sample's `p/N`
does not match it, or the current anchor is missing/non-unique.

Implication: next plan may repair only the proven preview partition/current
anchor binding error.

### D. `SNAPSHOT_PROVENANCE_MISMATCH_PROVEN`

Select only if the build's recorded snapshot/map sequence does not match the
frozen transaction snapshot used for attribution, or equal old/new points on
the same snapshot return different query facts.

Implication: next plan may repair only the proven snapshot binding error.

### E. `GENUINE_SAME_OWNER_ENVIRONMENT_ZERO_PROVEN`

Select only if:

- copied-prefix provenance is correct;
- old/new base and lifted geometry agree within existing tolerances;
- raw sample matches the new owner;
- snapshot provenance agrees;
- both old/new base-centre queries on the same snapshot return the same fact;
- the cross-section reason/query evidence explains why nonzero capacity is
  unavailable under the unchanged planner safe distance.

Implication: the `[0,0]` refusal is correct and must remain.  The next primary
plan must address navigation liveness without widening Tube or weakening
safety, and must separately analyze the later `STAGING_DRY_RUN` polygon-empty
failure.

### F. `ATTRIBUTION_INCONCLUSIVE`

Select if required evidence is missing, contradictory or the first target
event does not occur.  State the exact missing field.  Do not infer a repair.

Secondary observations may be reported, but exactly one primary
classification is required.

## 9. Deliverables

The executor must produce:

1. source/test diff limited to the whitelist;
2. static evidence directory;
3. one dynamic evidence directory;
4. parsed TSV/JSON or text summaries for the target line and timeline;
5. self-audit document;
6. exact primary classification from Section 8;
7. explicit statement that no navigation repair was implemented;
8. exact cleanup result and repository before/after result.

Report the system status as:

    TUBE_CONSTRUCTION_COMPLEXITY=SIGNIFICANTLY_IMPROVED
    TUBE_VISIBILITY=FIXED
    G2_H2_NAVIGATION_CONTINUITY=NOT_FIXED
    G19_ROLE=ATTRIBUTION_ONLY
    OVERALL=PARTIALLY_FIXED

Never report `FIXED` merely because G19 attribution succeeds.

## 10. Executor stop boundary

After the one dynamic run, evidence parsing, self-audit and classification,
stop the Luna execution and return all evidence to the primary agent.  Do not
write G20, choose a repair, implement a conditional branch from Section 8, or
start another ROS run.  The primary agent owns the next root-cause decision
and execution plan.
