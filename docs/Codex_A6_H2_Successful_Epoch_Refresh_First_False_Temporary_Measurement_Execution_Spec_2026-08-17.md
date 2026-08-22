# A6/H2 successful-epoch refresh first-false — temporary measurement specification

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DATE=2026-08-17
STAGE=A6_H2_SUCCESSFUL_EPOCH_REFRESH_FIRST_FALSE
STATUS=TEMPORARY_INSTRUMENTATION / AUTHORIZED_BY_USER_2026-08-17
IMPLEMENTATION_AUTHORIZED=YES__ONE_PRIVATE_EPISODE_ONLY
AUTO_ADVANCE=false
```

## Objective

Classify the single remaining lifecycle ambiguity from the preceding offline
audit: a timer epoch that is complete, Active and current-valid can still
return `prepare_false` in `prepareTimerPairRefresh`.  The measurement records
the **existing first false branch only**.  It neither changes nor relaxes a
predicate.

## Strict whitelist and baseline

Temporary source edit only:

- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`

Permanent files only: this specification and one read-only audit.  The header
is read-only.  Before patching, record:

```text
phase_offset_matched_adapter.cpp f6b7b0a5fdf77ece8372175c0a949c6b40518bef268b26e6b7229fee72450521
phase_offset_matched_adapter.h   414015f908eecc51c75bd226a25163b96673b11f4016864bd67ef2447d24d7c1
restored adapter Build-ID        432e58b750d280b0a4c8cbdd39b6a136f29cdf05
```

Also record the worktree status, `git diff --check`, and confirm that
`rg TEMP_H2_REFRESH_FIRST_FALSE src/swarm_planner/bspline_traj` is empty.

## Exact temporary evidence

Use fixed-prefix stderr only:

```text
TEMP_H2_REFRESH_FIRST_FALSE
```

Instrument `prepareTimerPairRefresh` only when the supplied epoch already has
an existing complete Candidate, available Active profile and valid current
validation.  Do not emit for failed/incomplete builds or from the 50 Hz
governor path.  At its earliest existing return, emit exactly one mutually
exclusive category:

- **A**: `latest_request` or live-pair/authority identity mismatch; include
  existing request/build/revision/generation/session/map identity only;
- **B**: runtime try-lock or active H2 pin/lease; include the same identity
  and the existing mutex/pin result only;
- **C**: candidate provenance, geometry, base guidance, Runtime exact-PWL
  dry run, or latest categorical veto; include only existing epoch
  state/disposition/reason, current-inside/horizon facts, Runtime mode and
  boolean witness result.

After a successful prepare, `finalizePreparedTimerPairRefresh` may emit only
an existing runtime/request drift result or shared-pointer CAS result.  It
must not emit for failed prepare, change its comparison order, or report
Runtime port values.

No header, ROS topic/message/schema, enum/reason/state, parameter, gate,
retry, latch, timer schedule, map, A5, Filter, C2, H2 predicate, ownership,
lock scope, return value, or physical control output may change.

## One private episode, rollback, and acceptance

Build `bspline_race`, then run exactly one task-owned loopback private master
with private ROS home/log directories, a fresh free port, the checked-in
`phase_offset_esdf_tube_single.launch`, and the sole existing override
`phase_offset_manual_observe_only:=false`.  Publish one `(8,0,1)` goal;
record diagnostics/raw/cloud/epoch/path/cmd/odom/goal/local-map/rosout plus
the stderr transcript, binary SHA/Build-ID, parameters, PID/maps and cleanup
evidence.  Run at most 70 seconds after the goal; never issue a second goal
or run.

The evidence target is at least several `update_ok`/complete/current-valid
epochs and their refresh outcome.  A missing target is valid negative
evidence, not permission to retry or tune.

Regardless of outcome, immediately remove every temporary hunk, verify the
adapter and header SHA values exactly, verify marker absence, rebuild the
restored product, confirm restored Build-IDs, run the adapter/epoch/GVF-H2/
continuous focused regressions, `git diff --check`, and verify task-owned
process/port cleanup.  Only after restoration write the audit.  Do not
implement any product repair in this stage.
