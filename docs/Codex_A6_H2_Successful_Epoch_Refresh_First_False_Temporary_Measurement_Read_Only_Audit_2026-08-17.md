# A6/H2 successful-epoch refresh first-false — read-only audit

```text
DOCUMENT_ROLE=READ_ONLY_TEMPORARY_MEASUREMENT_AUDIT
DATE=2026-08-17
STAGE=A6_H2_SUCCESSFUL_EPOCH_REFRESH_FIRST_FALSE
PRODUCT_CHANGE=NONE
RESULT=STOP__LATEST_RUNTIME_STATE_IS_NOT_EXECUTABLE_ON_ONE_FRESH_COMPLETED_PROFILE
```

## One-episode method and restoration

Exactly one task-owned private active episode ran at
`http://127.0.0.1:11764`, with private ROS home/log directories, the checked
in ESDF launch, the sole pre-existing
`phase_offset_manual_observe_only:=false` override, and one `(8,0,1)` goal.
The bag at
`/tmp/h2_successful_epoch_refresh_first_false_20260817/evidence/h2_successful_epoch_refresh_first_false.bag`
is 76 seconds, 422.5 MB, and 14,680 messages.  Its owned recorder, launch,
master, planner and simulator descendants were stopped; port 11764 is absent
from post-shutdown evidence.  No user ROS process was attached to or stopped.

Temporary stderr only was present in the adapter.  It did not alter a branch,
lock scope/order, state write, schema/topic, parameter, gate, retry, map,
A5, Filter, C2, H2 predicate, or control output.  It was removed immediately
after the episode.  Restoration is exact:

```text
phase_offset_matched_adapter.cpp f6b7b0a5fdf77ece8372175c0a949c6b40518bef268b26e6b7229fee72450521
phase_offset_matched_adapter.h   414015f908eecc51c75bd226a25163b96673b11f4016864bd67ef2447d24d7c1
TEMP_H2_REFRESH_FIRST_FALSE source marker absent
restored adapter Build-ID        432e58b750d280b0a4c8cbdd39b6a136f29cdf05
restored GVF Build-ID            e742720e7863e4f3770efa5abe2f8cb05ccfa817
```

The instrumented adapter Build-ID is retained in evidence.  Restored focused
regressions all pass: adapter 63/63, epoch diagnostics 5/5, epoch integration
10/10, GVF/H2 74/74, and continuous path 6/6.  `git diff --check` passes.

## Tagged result

The temporary transcript contains exactly nine eligible events:

| event | count |
| --- | ---: |
| successful prepare followed by finalize/CAS success | 8 |
| A: latest-request or live authority identity false | 0 |
| B: runtime try-lock or active pin/lease false | 0 |
| C: first false | 1 |
| finalize drift/CAS false after prepare success | 0 |

The sole C event is:

```text
stage=prepare bucket=C first_false=runtime_exact_pwl
req=72 build=7 rev=1 gen=4 session=1 map=47
state=2 disposition=1 reason=0 inside=1 horizon=1 mode=3 witness=0
```

`mode=3` is the existing `WAITING_FOR_CANDIDATE` value.  The raw candidate
payload associated with map observation 47 records a complete candidate at
its build request (`current_w=2.356215`, requested range
`[2.183334,4.316668]`, certified tail `4.316668`, current preferred delta
inside).  Thus the epoch is complete, Active and current-valid at its own
build snapshot, exactly as required by the measurement filter.

## What the C label proves

The label is emitted after the existing geometry and base-guidance checks and
after all A/B identity/lock/pin checks.  `PhaseOffsetRuntime::dryRun` first
copies Runtime, refreshes preflight, and invokes `prepare`; only a successful
prepare reaches `complete` and the continuous exact-PWL witness search.

The observed `WAITING_FOR_CANDIDATE` comes from `makePrepared`: with a complete
active profile and no explicit unsafe epoch, it can occur only when the
**latest request** cannot obtain valid current bounds from that immutable
profile or its retained delta is outside those bounds.  Therefore this event
does not demonstrate an exact-PWL search/CAS defect, and it must not be
interpreted as a reason to weaken the PWL witness or current-containment
check.

The evidence establishes a timing-state mismatch: the profile was current-
valid at the timer build request but not executable at the later latest
command request used by prepare.  The temporary stage intentionally does not
record the latest phase/bounds value, so it cannot distinguish profile-domain
loss from a later retained-delta-outside result.  That distinction is not a
license to guess or change behavior.

## STOP — no product repair in this stage

The existing fail-closed refusal is correct.  Relaxing it, reusing the stale
profile, skipping the Runtime dry run, changing Filter/A5/C2 geometry, or
adding a gate would make the system claim execution authority without a proof
at the current command state.

There is therefore no authorised minimal product fix from this measurement.
Any future work needs a new dedicated specification that first decides the
architecture-level recency policy (for example, whether a timer build may be
coalesced/rebased to a newer command snapshot before construction).  It must
prove the new snapshot through the existing exact Runtime check and retain the
same fail-closed behavior on a later change.  Until that decision and
specification exist, the correct status is **STOP**.
