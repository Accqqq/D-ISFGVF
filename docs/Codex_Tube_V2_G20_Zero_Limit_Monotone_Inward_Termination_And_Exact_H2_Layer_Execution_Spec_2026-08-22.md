# Tube V2 G20 zero-limit monotone inward termination and exact H2 layer execution spec

```text
DOCUMENT_ROLE=PRODUCT_SIMPLIFICATION_AND_DIRECT_FAILURE_CLASSIFICATION_SPEC
DOCUMENT_STATUS=ACTIVE
DOCUMENT_REVISION=R3_SOL_MAX_CERTIFIED_COVER_ONLY_ZERO_LIMIT
DATE=2026-08-22
PLAN_OWNER=PRIMARY_CODEX_AGENT
EXECUTOR=LUNA_MAX_SINGLE_AGENT
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
AUTO_ADVANCE=true_WITHIN_G20_ONLY
NAVIGATION_RECOVERY_CHANGE_ALLOWED=false
NO_NEW_GATE_CACHE_RETRY_TIMER_WATCHDOG_MODULE_QUEUE_STATE=true
NO_LIMIT_MARGIN_RATE_HORIZON_LAUNCH_PARAMETER_CHANGE=true
```

## 1. Current result and exact scope

The authoritative pre-G20 result is G19:

```text
TUBE_CONSTRUCTION_COMPLEXITY=SIGNIFICANTLY_IMPROVED_BUT_87_TO_89_INWARD_VALIDATIONS_REMAIN
TUBE_VISIBILITY=FIXED
G2_H2_NAVIGATION_CONTINUITY=NOT_FIXED
OVERALL=PARTIALLY_FIXED
```

Authoritative evidence:

```text
/dev/shm/tube_v2_g19_static_20260822_203154/
/dev/shm/tube_v2_g19_dynamic_20260822_210225/
```

The one dynamic run reached G1, timed out G2 at 240 s, did not send G3--G5,
and produced 1,177 exact `36/0/36` HOLD decisions for 233.320028543 s.  The
direct navigation chain was:

```text
one successful H2 Pair install
-> later H2 stage failure
-> another H2 stage failure
-> no replacement PathTubePair
-> retained old path end
-> all 36 governor candidates path-end clamped
-> fail-closed HOLD
```

The same run contained zero `attempts=101` cohorts and zero 249993-query
terminal cohorts, but its zero-baseline diagnostics still reported 87--89
independently re-run inward validations with last reason
`insufficient_clearance` and `limit_exceeded=1`.

G20 has exactly three product owners:

1. remove the large G19 temporary attribution implementation and its G19-only
   tests;
2. remove inward validations that are mathematically dominated by one exact
   zero-width limit validation on the same immutable build;
3. stop collapsing the already-computed `tube_profile_coverage` and
   `tube_profile_owner_match` failures into one enum.

G20 does **not** implement return-to-center, recovery, neutral switching, a
new Tube, or an H2 retry.  It must not be reported as a navigation fix unless
the unchanged independent dynamic run actually passes G1--G5.

## 2. Frozen product contracts

Preserve all of the following:

- planner-authoritative zero-connected Tube semantics;
- broad raw/environment `I_geo` Candidate evidence;
- G17 post-Filter authority-subset certification;
- G18 exact live-Pair Candidate/Certified publication ownership;
- the current planner safe distance, regularity margin and interior margin;
- exact-PWL Filter and continuous SurfaceValidator;
- immutable snapshot/path owner/provenance;
- current-anchor policy and zero-connectedness;
- Runtime, PortProjector, future witness, rates and limits;
- atomic PathTubePair stage/commit/CAS and old-Pair fail-closed retention;
- neutral planner remains ungated when no executed nonzero authority exists;
- query, subdivision-depth and inward scale constants remain unchanged;
- launch files, maps, ROS messages, parameters, goals and timeouts remain
  unchanged.

Forbidden approaches include:

- changing any query/depth/attempt constant or loop tolerance;
- adding a cache, memo table, retry, timer, watchdog, gate, state machine,
  module, queue or persistent diagnostic owner;
- skipping a distinct nonzero candidate merely because an earlier nonzero
  candidate failed;
- treating a failed proof as a successful certificate;
- promoting zero-only to `OFFSET_CERTIFIED`;
- reusing old Tube geometry for a new path owner;
- expanding authority or reducing clearance/margins;
- changing Runtime/PortProjector or implementing M6/effective-base recenter;
- direct delta reset, direct neutral Pair retirement, or governor override.

## 3. Phase A -- remove G19 temporary attribution completely

G19 was observational only and its target event did not occur.  Its large
single-line logging implementation must not remain in production.

Use the preserved G19 before/diff artifacts as the authoritative removal map:

```text
/dev/shm/tube_v2_g19_static_20260822_203154/g18_final_phase_offset_matched_adapter.h
/dev/shm/tube_v2_g19_static_20260822_203154/g18_final_phase_offset_matched_adapter.cpp
/dev/shm/tube_v2_g19_static_20260822_203154/g18_final_phase_offset_matched_adapter_test.cpp
/dev/shm/tube_v2_g19_static_20260822_203154/g19_adapter_header.diff
/dev/shm/tube_v2_g19_static_20260822_203154/g19_adapter_cpp.diff
/dev/shm/tube_v2_g19_static_20260822_203154/g19_adapter_test.diff
```

Do not copy an entire old file blindly.  Inspect the preserved G19 diffs and
remove only G19-owned hunks with `apply_patch`, preserving all G18 and later
non-G19 content.

Required removals:

- `LogG19ExactCurrentAttribution()` and its helper-only includes/constants;
- the optional `temporary_staged_update_ok` output added only for G19;
- the exact-current raw-anchor search and G19 call in `stagePathTubePair()`;
- G19-only helper fixtures and seven G19 attribution tests;
- any string/symbol beginning with `G19` or
  `[PHASE_OFFSET][G19][H2_EXACT_CURRENT_ATTRIBUTION]` in product/test source.

Preserve:

- G18 `LogZeroBaselineAttribution()`;
- `CanonicalOwnerStateReuse` and the owner-aligned current-state reuse that
  already existed in the preserved G18 source;
- G18 live-Pair marker ownership;
- all non-G19 adapter tests.

Phase A acceptance:

```text
rg -n 'G19|H2_EXACT_CURRENT_ATTRIBUTION|LogG19ExactCurrentAttribution' \
  <three adapter product/test files>
```

returns no match, the adapter builds, and all non-G19 adapter tests pass.

## 4. Phase B -- exact zero-width limit validation

> **R3 overriding correction:** the zero-limit termination is authorized only
> for the analytic certified-cell cover described below.  It is forbidden for
> a nondegenerate sampled-fallback cell and forbidden when the zero-limit run
> itself reaches the Validator query budget.  These restrictions override any
> broader wording later in this section.

Sol Max produced a strict sampled-fallback counterexample on the existing unit
circle path: with `max_depth=0`, open clearance and resolution 0.01, the exact
zero ribbon has sampled cover about 0.5492887 and fails target 0.54, while the
one-sided `[0,0.2]` ribbon has cover about 0.5329443 and succeeds.  Therefore
sampled fallback is not set-contraction monotone and must keep the old inward
search.

### 4.1 Mathematical dominance contract

The dominance proof is valid only when all of these facts hold:

```text
original_filtered_profile.cell_geometry_certified == true
input.path_cell_bound_query is nonempty
the same immutable/hereditary PathCellBoundQuery is used by zero and inward
every nondegenerate recursive cell therefore uses CertifiedCellCoverRadius
candidate interval contains delta=0
candidate interval is a superset of the exact zero-width interval [0,0]
candidate uses the same path owner, current_w, snapshot, clearance query,
required clearance, regularity margin and profile domain
```

For that analytic certified-cell branch, contracting an interval to `[0,0]`
cannot increase any proof burden:

- `maximum_delta` cannot increase;
- `delta_slope` becomes zero;
- `maximum_width` becomes zero;
- the v-direction cover becomes zero;
- the w-direction path certificate is unchanged or smaller;
- regularity and active-speed lower bounds cannot become worse;
- the certified closed-ball cover proves the complete v interval, so an
  asymmetric ribbon does not require a sampled v point to land exactly on
  zero;
- fixed snapshot support, required clearance and cover epsilon are unchanged;
- path validity and immutable UNKNOWN/out-of-map/occupied/clearance facts cannot
  improve by adding nonzero surface points.

Therefore, if the exact zero-width profile cannot be continuously validated on
the required current-connected range, no strictly wider zero-connected inward
candidate from this same immutable certified-cover build can be certified by
this Validator, except when the zero-limit run itself exhausts the global query
budget.  One non-query-terminal exact zero-limit validation may terminate the
whole inward family search.

The exact-current degenerate anchor is safe under this restriction even though
it uses the sampled formula: the zero profile has cover radius zero there,
while any containing affine interval has nonnegative cover.  This exception
does not authorize sampled fallback on any nondegenerate w cell.

If `cell_geometry_certified` is false, the callback is empty, a nondegenerate
cell can fall back to the sampled cover, or the zero-limit result satisfies
`QueryBudgetExhausted`, the dominance conclusion is unavailable and the
existing inward search must run unchanged.

This is proof-local dominance, not a planner gate and not a zero-Tube
certificate requirement.  The final product result remains the valid
`ZERO_ONLY_PLANNER_BASELINE` fallback even when the zero-limit validation
fails.

### 4.2 Build the zero-limit proof candidate

Modify only the existing private narrowing implementation in
`certified_tube_builder.cpp`.

Preferred implementation:

- generalize `PrepareNarrowedCandidate()` so `scale==0` is accepted only when:
  - `family == BOTH_SIDED`;
  - `allow_zero_only == true`;
- in that exact case set every filtered lower/upper bound to zero;
- recompute local PWL slopes and `min_width` through the same existing code;
- preserve raw/environment/raw_build_samples, owner, revision, snapshot,
  truncation and all non-filter provenance;
- clear prior Validator evidence/classification exactly as for other narrowed
  candidates;
- do not expose scale zero to ordinary inward families.

An equivalent small private helper is acceptable only if it does not duplicate
the slope/profile-finalization logic.  Do not add a public type or API.

### 4.3 Execution order inside `CertifiedTubeBuilder::build()`

Keep the existing full requested-authority validation first.

When and only when all of these are true:

```text
source == ESDF
full requested-authority validation failed
failure is currently retryable
exact-current width is strictly nonzero
original_filtered_profile.cell_geometry_certified == true
input.path_cell_bound_query is nonempty
```

perform this sequence:

1. Create one exact zero-limit candidate from
   `original_filtered_profile`.
2. Validate it once with the exact same:
   - `current_w`;
   - path-state query;
   - path-cell-bound query;
   - immutable cloud-clearance query;
   - snapshot resolution;
   - planner safe distance;
   - regularity margin.
3. If zero-limit validation returns false and
   `QueryBudgetExhausted(zero_limit_validation, validator_config_)` is false:
   - do not enter any BOTH/POSITIVE/NEGATIVE inward halving loop;
   - preserve the original full-width failure as the public failure
     provenance;
   - collapse the product candidate to planner zero baseline exactly as now;
   - use one concise Builder diagnostic stating that inward search was skipped
     because the same-build zero-width limit was not continuously certifiable;
   - include the zero-limit reason/query/limit facts in that existing string,
     without a new schema or ROS log.
4. If zero-limit validation returns true, or if it returns false because its
   query budget is exhausted:
   - do not promote it to nonzero authority;
   - do not install it as `OFFSET_CERTIFIED`;
   - proceed into the existing independent inward search unchanged;
   - every strictly nonzero candidate still requires its own complete
     SurfaceValidator success.

The existing G16 exact-current-zero-capacity shortcut remains earlier and
unchanged.  It must not perform the extra zero-limit validation because the
current interval already proves that no current-connected nonzero authority
exists.

Full-width query-budget terminal, invalid path/configuration and other existing
nonretryable failures remain terminal before the zero-limit step.

Do not add an OOM classification.  The current Validator does not catch
`std::bad_alloc`, so G20 must not claim a product OOM outcome that the source
does not expose.

### 4.4 Result/provenance rules

- `result.surface_validation` continues to report the selected certified
  inward validation on success, otherwise the original full-width validation;
- the zero-limit validation is local proof work only;
- final zero fallback remains `obstacle_certified=false` and
  `ZERO_ONLY_PLANNER_BASELINE`;
- raw Candidate geometry and marker width remain unchanged;
- no zero-limit result reaches Runtime or Epoch as nonzero authority;
- no attempt/query limit or tolerance is changed;
- sampled-fallback profiles and zero-query-terminal profiles retain the old
  inward sequence;
- if zero-limit succeeds and the inward loop runs, the old attempt-selection
  order and best-candidate rule remain bit-for-bit/semantically unchanged.

## 5. Phase C -- exact H2 failure enums from existing strings

The adapter already computes transaction-local strings:

```text
tube_profile_coverage
tube_profile_owner_match
```

They currently collapse into `TUBE_COVERAGE_OR_VALIDATOR`, hiding the next
product owner.  Replace only that aggregate enum with:

```text
TUBE_PROFILE_COVERAGE
TUBE_PROFILE_OWNER_MATCH
```

Requirements:

- map `tube_profile_coverage` directly to `TUBE_PROFILE_COVERAGE`;
- map `tube_profile_owner_match` directly to `TUBE_PROFILE_OWNER_MATCH`;
- update the existing name function and focused enum/name assertions;
- use the existing manager `[GVF][H2][STAGE]` log unchanged except that it now
  prints the exact enum name;
- add no log, field, schema, persistent state or branching decision;
- both failures continue returning false and preserving the old Pair exactly
  as before;
- `TUBE_RAW_BUILD`, `TUBE_FILTER`, `TUBE_SURFACE_VALIDATOR`,
  `TUBE_BUILD_PRECONDITION` and `STAGING_DRY_RUN` remain unchanged.

## 6. Authorized files

Product files:

```text
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
```

Focused tests:

```text
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
```

Documentation:

```text
docs/Codex_Tube_V2_G20_Zero_Limit_Monotone_Inward_Termination_And_Exact_H2_Layer_Execution_Spec_2026-08-22.md
docs/Codex_Tube_V2_G20_Zero_Limit_Monotone_Inward_Termination_And_Exact_H2_Layer_Self_Audit_2026-08-22.md
```

Read-only and hash-frozen:

- TubeBuilder, TubeFilter, TubeSurfaceValidator and their headers;
- TubeEpochManager/types;
- Runtime and PortProjector;
- `gvf_manager.h` and every `gvf_manager.cpp` line except the existing
  `pathTubePairStageFailureName()` enum-to-string switch;
- planner, C2 path, governor and SO3;
- launch, map, parameter, message and CMake files.

If implementation requires a file outside this whitelist, report to the
primary agent before editing.  Do not expand scope yourself.

The `gvf_manager.cpp` authorization is deliberately narrow: update only the
two replacement enum cases in `pathTubePairStageFailureName()`.  Do not alter
H2 routing, stage calls, logs, locks, frontends, neutral handling or any other
manager behavior.  `gvf_switch_policy_test.cpp` may change only the existing
enum-name coverage assertion/fixture needed to compile and prove both new
names.  This R1 closure was required because the manager owns a second name
switch used by the existing `[GVF][H2][STAGE]` log.

## 7. Required focused tests

### 7.1 G19 removal regressions

- all G19-only tests are removed;
- all non-G19 adapter tests still pass;
- no G19 symbol or log prefix remains;
- G18 Candidate/Certified dual-owner marker behavior is unchanged;
- zero-baseline attribution remains unchanged.

### 7.2 Zero-limit dominance

Add focused Builder tests with deterministic query counters:

1. `CertifiedZeroLimitFailureSkipsAllStrictInwardCandidates`
   - `cell_geometry_certified=true` and the same complete hereditary
     `PathCellBoundQuery` is used for every recursive subcell;
   - full authority fails retryably;
   - exact zero-width analytic limit fails without query-budget exhaustion;
   - exactly one zero-limit validation occurs after full width;
   - no BOTH/POSITIVE/NEGATIVE strict inward candidate is validated;
   - final result is zero-only, not certified;
   - raw evidence is unchanged;
   - diagnostic records zero-limit reason/query/limit facts.
2. `SampledFallbackCounterexamplePreservesRealInwardSearch`
   - reproduce the Sol Max unit-circle counterexample or a numerically
     equivalent fixture: zero sampled cover is about 0.5492887 and fails a
     0.54 target, while one-sided `[0,0.2]` sampled cover is about 0.5329443
     and succeeds;
   - `cell_geometry_certified=false`;
   - Builder must not use zero-limit termination and must retain the real
     nonzero inward ribbon.
3. `CertifiedZeroQueryBudgetPreservesInwardSearch`
   - analytic zero-limit reaches query-budget terminal;
   - Builder does not use that incomplete proof to terminate inward;
   - a distinct inward candidate receives its existing opportunity.
4. `CertifiedDepthOrImmutableUnknownMayTerminate`
   - analytic zero-limit fails by persistent depth exhaustion or immutable
     UNKNOWN/out-of-map/occupied/insufficient-clearance without query-budget
     terminal;
   - no strict inward candidate is revalidated.
5. `CertifiedZeroLimitSuccessPreservesRealInwardSearch`
   - full width fails because of an outer-only obstacle;
   - exact zero-width limit validates;
   - a strictly nonzero inward ribbon validates;
   - final profile is `OFFSET_CERTIFIED` and nonzero;
   - zero-limit itself is never installed as the selected result.
6. `CertifiedZeroLimitSuccessButNoNonzeroCandidateRemainsFailClosed`
   - zero limit validates;
   - every strict inward candidate fails;
   - final result remains zero-only and no failed profile is promoted.
7. `ExactCurrentDegenerateAnchorRemainsMonotone`
   - use an asymmetric two-sided certified profile;
   - prove the exact-current zero anchor cover is zero and cannot be harder
     than the containing affine interval;
   - do not require a v sample to equal zero in the nondegenerate cells.
8. `RemoteTruncationDoesNotCreateFalseZeroFailure`
   - remote analytic cells fail but a current-connected certified segment
     exists;
   - zero-limit returns complete/truncated and inward search remains allowed.
9. `RecursiveSubcellCertificatesRemainCompleteAndMatched`
   - every recursive PathCellBoundQuery result is complete and exactly matches
     requested w0/w1;
   - a missing/mismatched certificate does not silently use dominance.
10. `ExactCurrentZeroCapacityStillUsesG16Shortcut`
   - no full/zero-limit/inward SurfaceValidator call occurs when the exact
     current interval has zero capacity;
   - the existing G16 diagnostic/result remains unchanged.
11. `FullWidthSuccessDoesNotRunZeroLimit`
   - successful full requested authority still performs one validation only.
12. `FullWidthTerminalDoesNotRunZeroLimit`
   - query-budget terminal and invalid path/configuration retain current
     terminal behavior and do not add a second validation.
13. `ZeroLimitPreparationPreservesRawAndSnapshotProvenance`
   - raw/environment/raw_build_samples, revision and snapshot facts remain
     unchanged while filtered bounds/slopes become exact zero.

Tests must assert callback/query invocation counts or unique fixture facts, not
wall-clock time.

### 7.3 Exact H2 enum mapping

- a prepared profile too short for the required range returns
  `TUBE_PROFILE_COVERAGE`;
- an owner/sample mismatch returns `TUBE_PROFILE_OWNER_MATCH`;
- both remain false/no-mutation stage results;
- both the adapter name function and the manager's existing
  `pathTubePairStageFailureName()` return the exact new names;
- all existing stage failure names remain covered;
- successful staging returns `NONE`;
- no retry or altered Pair/Runtime/frontend state appears.

## 8. Static verification order

Create one fresh evidence directory:

```text
/dev/shm/tube_v2_g20_static_<timestamp>/
```

Record before hashes/status, then run in order:

1. CertifiedTubeBuilder focused and full tests;
2. TubeSurfaceValidator, TubeBuilder and TubeFilter regressions;
3. adapter focused and full tests;
4. marker tests;
5. EpochManager, epoch integration and diagnostics;
6. Runtime tests;
7. GVF switch-policy and continuous-path tests;
8. required CTest set;
9. `catkin_make -j2` incremental build;
10. `git diff --check` and no-index whitespace check for untracked sources;
11. forbidden-symbol and forbidden-file audit;
12. before/after hashes for all source outside the whitelist and all
    launch/map/parameter/message/CMake files.

Static acceptance requires every previously passing non-G19 test to pass and
no unauthorized file change.  Do not repair unrelated dirty-worktree files.

## 9. One direct independent split-launch ROS test

After static acceptance, run one direct independent ROS test using the user's
unchanged separate launches:

```text
roslaunch so3_quadrotor_simulator simulator.launch
roslaunch bspline_traj test_gvf.launch
```

Use a fresh private ROS master/port.  Check existing ROS ownership first, do
not attach to or terminate user processes, and clean only task-owned PIDs and
PGIDs.  Reuse the known G1--G5 goals and unchanged per-goal bound.  Do not
create or edit a repository harness.  Reuse the prior command procedure and
collect only launch logs, the existing essential topics/bag, goal outcomes,
and cleanup proof.

Run exactly once.  Do not rerun because the result is undesirable.

Required dynamic report:

- G1--G5 outcome;
- maximum and representative inward `attempts=` counts;
- whether the zero-limit skip diagnostic occurred;
- Candidate and Certified ADD/DELETE counts and widths;
- Pair commit/nonzero execution counts;
- exact H2 stage failure enum(s), now distinguishing coverage/owner match;
- first old-path-end `36/0/36` HOLD and duration, if any;
- 249993 and attempts=101 counts;
- source/binary/launch/map before/after equality;
- task-owned process cleanup.

Dynamic success for construction simplification requires the reproduced
87--89 dominated cohort to terminate after the one zero-limit proof, unless
the zero-limit actually certifies and distinct nonzero candidates therefore
remain necessary.  Do not claim query reduction from a non-reproduced cohort.

System `FIXED` requires G1--G5 all reach and no stable old-path-end HOLD.  If
navigation still fails, report `PARTIALLY_FIXED`; the exact H2 enum becomes the
input to the next primary-agent plan.

## 10. Deliverables and executor boundary

Deliver:

1. whitelist-limited source/test diff;
2. G20 self-audit;
3. static evidence directory;
4. one dynamic evidence directory;
5. exact before/after validation counts;
6. exact H2 failure enum timeline;
7. honest final status.

The Luna executor must not write G21, implement return-to-neutral, change
Runtime/manager behavior, or start a second dynamic run.  It must not create a
subagent.  Return all evidence to the primary agent, which owns the next plan
and continuous dispatch.
