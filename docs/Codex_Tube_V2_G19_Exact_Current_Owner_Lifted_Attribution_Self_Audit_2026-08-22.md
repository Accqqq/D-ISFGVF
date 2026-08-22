# Tube V2 G19 exact-current owner/lifted attribution self-audit

    STAGE=G19
    ROLE=ATTRIBUTION_ONLY
    PRIMARY_CLASSIFICATION=ATTRIBUTION_INCONCLUSIVE
    NAVIGATION_REPAIR_IMPLEMENTED=false
    DYNAMIC_RUN_COUNT=1
    OVERALL=PARTIALLY_FIXED

## 1. Authorized implementation

G19 added transaction-local evidence only.

- `phase_offset_matched_adapter.h` adds the one authorized optional private
  `bool*` output to `buildPreparedTubeEpoch()`.
- `phase_offset_matched_adapter.cpp` propagates the exact local
  `staged_manager.update()` return, selects the frozen exact-current raw
  anchor, and emits the bounded G19 warning only at the specified replacement
  `tube_update_status -> TUBE_BUILD_PRECONDITION -> raw [0,0]` boundary.
- The helper reads only the transaction's old/new immutable owners, captured
  phase and retained delta, actual position, prepared epoch, and frozen cloud
  snapshot. Its four clearance queries are local read-only values and do not
  feed Builder, Validator, Runtime, staging, commit, or control predicates.
- No persistent bit, gate, cache, retry, timer, watchdog, mailbox, state
  machine, recovery command, recenter behavior, neutral switch, authority
  enlargement, limit, margin, rate, horizon, launch, map, parameter, or ROS
  schema was added or changed.

The capacity constant used by the existing zero-baseline diagnostic was moved
from function scope to adapter-local scope without changing its `1e-10` value,
so G19 can use the exact same existing capacity tolerance. This is not a new
acceptance tolerance or predicate.

The primary agent concurrently clarified Section 3 before implementation to
authorize only the private default-null `bool*` output above. No other public
type or interface changed.

## 2. Focused and regression verification

Static evidence:

    /dev/shm/tube_v2_g19_static_20260822_203154/

Results, in the required order:

| Suite | Result |
|---|---:|
| Matched Adapter, including 7 G19 cases | 101/101 |
| Tube markers | 14/14 |
| Tube EpochManager | 55/55 |
| Tube epoch integration | 10/10 |
| Tube epoch diagnostics | 5/5 |
| GVF switch policy / H2 lifecycle | 49/49 |
| Runtime | 35/35 |
| CertifiedBuilder | 20/20 |
| Total | 289/289 |

The G19 cases cover copied-prefix equality at retained nonzero delta, inclusive
first-match segment attribution before and at a seam, exact-bit raw-anchor
selection and non-unique reporting, four same-snapshot observational queries,
one complete target warning, canonicalized current-anchor uniqueness, and
silence for raw/filter/Validator/coverage, nonzero-capacity, and staging
dry-run paths.

Additional static results:

    catkin_make -j2 = PASS
    git diff --check = EMPTY
    no-index whitelist whitespace check = EMPTY
    source hashes outside whitelist = 914/914 unchanged
    launch/map/parameter/message hashes = 156/156 unchanged
    forbidden-symbol audit = no new forbidden owner/state/control symbol

The full build retained two pre-existing warnings in unchanged
`gvf_manager.cpp`; G19 did not modify that file.

## 3. Exactly one dynamic run

Dynamic evidence:

    /dev/shm/tube_v2_g19_dynamic_20260822_210225/

The unchanged split launches, parameters, map and G1--G5 harness ran once on
private port `49319`. Recorder startup preceded both publishers. Readiness
passed the original 900-local-map gate. The loaded formation binary and both
phase-offset libraries matched the static hashes.

Goal result:

| Goal | Result |
|---|---|
| G1 `(7.381, 0.378, 0.0)` | REACHED, distance `0.186` |
| G2 `(-6.823, -0.685, 0.0)` | TIMEOUT at unchanged 240 s bound |
| G3--G5 | not sent after G2 timeout |

Retained regressions:

- Candidate markers: 7,467 ADD and 108 DELETE; width up to 6.0 m.
- Certified markers: 3 ADD and 7,572 DELETE; the genuine ADD width is 0.20 m.
- Pair bootstraps committed: 2.
- Nonzero Pair selected/executed events: 8.
- `249993` occurrences: 0.
- `attempts=101` occurrences: 0; maximum reported attempts: 89.
- Maximum reported query count: 8,640.
- Exact `36/0/36 HOLD` events: 1,177.
- HOLD duration through the G2 timeout: 233.320028543 s.

The bag closed cleanly (`76,384` messages, 367 s, 9.6 MB).

## 4. Target timeline and missing attribution

The target timeline is preserved in:

    derived/g19_target_timeline.tsv
    derived/g19_target_summary.json
    derived/g19_attribution_full_fields.tsv

Observed sequence:

1. C2 success at `1787404021.123823643`, `phase_w=9.988`,
   `join_delta_w=1.000`, `exact_path=1`.
2. First exact `36/0/36 HOLD` at `1787404022.395593643`.
3. Zero-baseline at `1787404024.495476007`, map sequence `1132`, first stop
   `w=9.557980115`, attempts `87`, queries `8640`.
4. H2 stage failure 42 microseconds later at `1787404024.495518446`:

       pair_generation=8
       session=8
       captured_w=9.557980115
       seam_w=9.988341012
       owner_end_w=14.942606996
       stage_success=0
       stage_failure=TUBE_BUILD_PRECONDITION

5. Next exact HOLD at `1787404024.515615940`.

The run emitted:

    H2_EXACT_CURRENT_ATTRIBUTION events = 0
    H2_STAGING_DRY_RUN events = 0

Therefore there is no first attribution line whose full fields can be
summarized. The exact missing evidence is the complete dynamic
`[PHASE_OFFSET][G19][H2_EXACT_CURRENT_ATTRIBUTION]` record; consequently the
old/new owner differential, lifted-reference differential, Builder raw-anchor
match and four same-snapshot query facts are all unavailable for the target
transaction. Static tests prove the helper is observational and emits for a
deterministic matching transaction, but static evidence cannot substitute for
the required dynamic target record.

A prior G2 H2 attempt failed as `TUBE_COVERAGE_OR_VALIDATOR` at
`1787404020.694787502`. No staging dry-run occurred in this run and none is
claimed.

## 5. Section 8 classification

Exactly one primary classification is selected:

    ATTRIBUTION_INCONCLUSIVE

Classifications A through E require dynamic owner/lifted/raw-anchor/snapshot
fields which were not emitted. F applies because the required target
attribution event is absent. G18's earlier exact-zero observation is not used
as G19 evidence and no repair is inferred.

## 6. Cleanup and integrity

Cleanup completed in exact task-owned order: harness, readiness, planner,
simulator, recorder, private roscore.

    task-owned PGIDs remaining = 0/6
    selected port listeners remaining = 0
    external ROS processes preserved = 32/32
    external ROS masters preserved = 14/14
    binary/launch/map identity diff = EMPTY
    source outside whitelist diff = EMPTY
    whitelist hash diff across dynamic run = EMPTY
    git status/diff/cached diff across dynamic run = EMPTY

The process preflight scan contained one task preflight-shell false positive;
after excluding that non-ROS shell PID, all actual pre-existing ROS processes
were unchanged. No user-owned process was signalled or terminated.

## 7. Stop boundary

No navigation repair was implemented. No second dynamic run was started. No
G20 or conditional repair branch was written.

    TUBE_CONSTRUCTION_COMPLEXITY=SIGNIFICANTLY_IMPROVED
    TUBE_VISIBILITY=FIXED
    G2_H2_NAVIGATION_CONTINUITY=NOT_FIXED
    G19_ROLE=ATTRIBUTION_ONLY
    OVERALL=PARTIALLY_FIXED
    STOP__RETURN_TO_PRIMARY_AGENT
