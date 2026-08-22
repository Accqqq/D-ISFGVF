# A6/H2 timer-refresh prebuild coalescing — execution specification

```text
DOCUMENT_ROLE=PROPOSED_PRODUCT_EXECUTION_SPECIFICATION
DATE=2026-08-17
STAGE=A6_H2_TIMER_REFRESH_PREBUILD_COALESCING
IMPLEMENTATION_AUTHORIZED=NO
STATUS=STOP_BY_ARCHITECTURE_AUDIT
```

## Decision

Do not implement prebuild request coalescing as a repair.  The read-only
architecture audit proves that the timer has no worker queue to drain and
that an extra load immediately before construction cannot prevent command
updates during construction.  The current exact latest-state Runtime dry-run
and final CAS already reject that condition safely.

No product files are authorised by this specification.

## Reserved minimal whitelist if a future owner separately authorises an optimisation

- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
- a successor dedicated specification and audit only

No header, manager, A5/Filter/C2 geometry, parameter, schema, enum, reason,
state, gate, latch, retry loop, or ROS launch file may be changed.

## Non-negotiable invariants for any successor

1. Coalesce only immutable requests matching active/source-revision/session/
   exact base-pair pointer/generation/path-owner domain.
2. Do not compare or reuse a map snapshot as pair identity; preserve the
   selected request's frozen-map provenance.
3. Do not hold `runtime_command_mutex_` during sampling, construction, map
   queries, Filter work, or validator work.
4. Keep `prepareTimerPairRefresh()`'s latest-request geometry/base/exact
   Runtime dry-run/latest categorical veto unchanged.
5. Keep `finalizePreparedTimerPairRefresh()`'s request/Runtime bitwise
   revalidation and pair CAS unchanged.
6. A command arriving after any optional extra load must remain able to make
   the completion fail closed; no retry/rebuild loop is permitted.

## Required proof before any GO

The complete matrix in the paired architecture audit must be implemented as
deterministic tests and pass together with the existing 63 adapter, 5 epoch
diagnostics, 10 epoch integration, 74 GVF/H2 and 6 continuous-path tests.
Until then, `STOP` remains the executable outcome.
