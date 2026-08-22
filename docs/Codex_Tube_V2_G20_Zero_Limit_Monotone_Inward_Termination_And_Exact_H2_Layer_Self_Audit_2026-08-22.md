# Tube V2 G20 zero-limit monotone inward termination and exact H2 layer self-audit

```text
DOCUMENT_ROLE=PRODUCT_SIMPLIFICATION_AND_DIRECT_FAILURE_CLASSIFICATION_SELF_AUDIT
DOCUMENT_STATUS=COMPLETE
DATE=2026-08-22
STAGE=G20
STATIC_RESULT=PASS_WITH_ONE_PREEXISTING_UNRELATED_FIXTURE_FAILURE
DYNAMIC_RESULT=G1_REACHED_G2_TIMEOUT
NAVIGATION_REPAIR_IMPLEMENTED=false
OVERALL=PARTIALLY_FIXED
```

## 1. Result and boundary

G20 implemented the authorized construction simplification and exact H2 enum
split.  It did not implement recovery, return-to-center, neutral switching,
Runtime changes, a new Tube, or an H2 retry.  The honest product result is:

```text
TUBE_CONSTRUCTION_COMPLEXITY=SIMPLIFIED_FOR_CERTIFIED_ZERO_LIMIT_CASES
TUBE_VISIBILITY=NOT_FIXED
G2_H2_NAVIGATION_CONTINUITY=NOT_FIXED
OVERALL=PARTIALLY_FIXED
```

The one dynamic run is evidence of the unchanged navigation boundary, not a
claim of system `FIXED`.

## 2. Authorized implementation

### Phase A: G19 removal

The temporary G19 attribution implementation, optional staged-output flag,
raw-anchor search/call, and G19-only fixtures/tests were removed.  The
preserved G18 zero-baseline attribution, live-Pair marker ownership and
owner-aligned reuse remain.  The three adapter product/test files contain no
`G19`, `H2_EXACT_CURRENT_ATTRIBUTION`, `LogG19ExactCurrentAttribution`, or
`temporary_staged_update_ok` symbol.

### Phase B: certified-cell zero-limit proof

`PrepareNarrowedCandidate()` accepts `scale==0` only for the private
`BOTH_SIDED`/`allow_zero_only` exact-limit call.  It sets filtered bounds to
zero, recomputes the existing local PWL slopes and `min_width`, clears
Validator evidence/classification, and preserves raw/environment/raw-build
samples, owner, revision and snapshot provenance.

After the existing full requested-authority validation, the zero-limit check
is eligible only for an ESDF retryable failure with nonzero exact-current
width, certified cell geometry and a nonempty path-cell callback.  It uses the
same current phase, immutable path/cloud queries, snapshot resolution,
clearance and regularity settings.  The local callback requires every
recursive certificate to be complete and to match the requested `(w0,w1)`.

If that exact certified zero-limit validation fails without Validator query
budget exhaustion, the strict inward families are skipped and the original
full-width failure remains the public provenance.  The diagnostic includes
the zero-limit reason, query count and limit flag.  A zero-limit success, a
query-budget terminal result, sampled fallback, callback incompleteness or
mismatch, and the pre-existing exact-current zero-capacity case retain the
old inward/shortcut behavior.  The zero-limit profile is never promoted to
`OFFSET_CERTIFIED` or Runtime/Epoch authority.

Focused Builder regressions cover certified zero failure/success, sampled
fallback non-monotonicity, zero-query-budget fallback, depth/UNKNOWN failure,
exact-current anchor monotonicity, remote truncation, recursive certificate
matching, provenance preservation and the unchanged G16 zero-capacity
shortcut.

### Phase C: exact H2 enums

The aggregate `TUBE_COVERAGE_OR_VALIDATOR` enum was replaced by
`TUBE_PROFILE_COVERAGE` and `TUBE_PROFILE_OWNER_MATCH`.  The adapter maps the
two existing transaction-local strings directly, and the existing
`gvf_manager.cpp` name switch returns the exact uppercase names.  No H2
routing, Pair mutation, retry, log schema, or control predicate changed.

## 3. Static verification

Evidence root:

```text
/dev/shm/tube_v2_g20_static_20260822_215155/
```

Direct suites passed:

| Suite | Result |
|---|---:|
| CertifiedTubeBuilder | 28/28 |
| Matched Adapter (non-G19) | 94/94 |
| GVF switch policy / H2 lifecycle | 49/49 |
| Tube markers | 14/14 |
| Tube EpochManager | 55/55 |
| Tube epoch integration | 10/10 |
| Tube epoch diagnostics | 5/5 |
| Runtime | 35/35 |

The remaining core/query/diagnostic/continuous-path suites passed.  The
TubeSurfaceValidator suite was 21/22 with the known unrelated pre-existing
fixture failure `ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel`; the
fixture was not changed.  The stale unlinked
`phase_offset_tube_dynamic_feasibility_test` still has its existing symbol
lookup failure and no CMake target.  CTest package discovery reported no
registered tests despite the direct binaries.

`catkin_make -j2` passed and `git diff --check` was empty.  The source hash
audit found 914/914 source files outside the whitelist unchanged; launch,
map, parameter, message and CMake hashes were 156/156 unchanged.  The
forbidden G19-symbol audit and Builder-scope audit were clean.

## 4. One dynamic run

Evidence root:

```text
/dev/shm/tube_v2_g20_dynamic_20260822_220740/
```

The private ROS master used port `49329`.  Readiness and identity checks
passed.  G1 reached at distance `0.196`; G2 timed out at the unchanged
240-second bound; G3--G5 were not sent.  Harness exit was `rc=3`.

Construction facts from the raw log and derived TSVs:

- `/tube_candidate`: 2,552 ADD messages and 27 DELETE messages, containing
  7,656 ADD and 81 DELETE markers; ADD width range 0--6.0 m.
- `/tube`: 0 ADD messages and 2,579 DELETE messages, containing 0 ADD and
  7,737 DELETE markers.  Dynamic Certified visibility was not reproduced and
  is therefore `NOT_FIXED` for this run; static marker ownership remains
  covered by the 14/14 marker suite.
- Zero-baseline events: 8 total; exact-current zero-capacity skips: 5;
  same-build zero-limit skips: 3.
- The three zero-limit diagnostics had `reason=insufficient_clearance`,
  query counts 5,229, 2,151 and 1,494, with `limit_exceeded=1` from depth
  exhaustion rather than global query-budget exhaustion.
- No dynamic inward attempt was recorded; `attempts=101` and `249993`
  occurrences were both zero.
- Pair bootstrap commits: 2; Pair runtime executions: 2; nonzero Pair
  selected/executed events: 12; live Runtime certificate denials: 4.

The exact H2 rows were one pre-stage lifecycle `NONE` row and four
`STAGING_DRY_RUN` rows (three stage-boundary, one lifecycle).  There were no
dynamic `TUBE_PROFILE_COVERAGE` or `TUBE_PROFILE_OWNER_MATCH` rows.  The first
exact stage-boundary enum was `STAGING_DRY_RUN` at
`1787408236.814920187`; its transaction-local reason was an empty joint port
polygon at witness step 4, with old-Pair/retained-state preservation.

The first exact `36/0/36` old-path-end HOLD was at
`1787408238.456971645`.  There were 1,185 total HOLD events, 1,112 through
the G2 deadline, lasting `234.939946651 s` through that deadline.  The old
path remained retained and fail-closed, so navigation continuity is not
fixed.

## 5. Integrity, cleanup and stop boundary

All task-owned PGIDs were gone and port `49329` was free.  Pre-existing
user-owned ROS processes/masters were preserved.  Source, binary,
launch/map/parameter/message identity, worktree status and diff checks were
unchanged across the run.  The corrected raw-fact dynamic report is:

```text
/dev/shm/tube_v2_g20_dynamic_20260822_220740/derived/final_report.md
```

No second dynamic run was started.  No G21 work, recovery command, Runtime
change, manager behavior change or navigation claim was made.

```text
G20_STATIC=PASS_WITH_PREEXISTING_UNRELATED_SURFACE_FIXTURE
G20_DYNAMIC=PARTIALLY_FIXED
G2_NAVIGATION=NOT_FIXED
STOP__RETURN_TO_PRIMARY_AGENT
```
