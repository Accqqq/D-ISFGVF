# A6/H2 timer-refresh lifecycle correlation — temporary measurement specification

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DATE=2026-08-17
STAGE=A6_H2_TIMER_REFRESH_TO_SEAM_AVAILABILITY_CORRELATION
STATUS=TEMPORARY_INSTRUMENTATION / AUTHORIZED_BY_USER_2026-08-17
IMPLEMENTATION_AUTHORIZED=YES__ONE_PRIVATE_EPISODE_ONLY
AUTO_ADVANCE=false
```

## Objective

Correlate the active certificate tail seen by H2 with the existing timer
request, build, same-owner refresh prepare/CAS, and H2 C2/stage path. The
previous selector-counter run proved that nonterminal empty sets occur when
the active certified end has not reached `captured_w + 0.4`; it did not show
why the current timer lifecycle had not made a later completed profile active.

## Whitelist and baseline

Temporary source edits only:

- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`

Permanent files only: this specification and one read-only audit. No header is
needed: all emitters are file-local. Before applying a forward patch, record:

```text
gvf_manager.cpp              078ae5378b49b7942b12ecef927f928f70fda0b0f5642d386691e650fdb07c18
phase_offset_matched_adapter.cpp
f6b7b0a5fdf77ece8372175c0a949c6b40518bef268b26e6b7229fee72450521
phase_offset_matched_adapter.h
414015f908eecc51c75bd226a25163b96673b11f4016864bd67ef2447d24d7c1
```

Also record `git status --short`, `git diff --check`, product Build-IDs and
`rg TEMP_H2_LIFECYCLE src/swarm_planner/bspline_traj` (must be empty).

## Exact temporary evidence

Use only fixed-prefix stderr lines `TEMP_H2_LIFECYCLE`, emitted from existing
low-rate timer/H2 transaction operations. Preserve every existing branch
condition, short-circuit order, return value, lock scope, state write, CAS,
parameter, and schedule. The lines must correlate a stable identity composed
of pair generation/source revision/session, request/build sequence, map
sequence, and phase where available.

Required events/fields:

1. timer due/request identity, source revision, base pair generation/session,
   request current w, requested preview/certified range and frozen map;
2. build completion from the existing epoch update: update_ok, state,
   disposition, reason, candidate complete/current bounds/retained-inside/
   horizon/active-available, candidate certified start/end and map;
3. same-owner refresh prepare: eligible identity, latest Runtime bits/current
   bounds/horizon/exact witness result, categorical veto, active certificate
   tail before prepare; finalize/CAS result and tail/generation after CAS;
4. H2 selector nonempty case: captured w, selected seam, C2 success/failure,
   `stagePathTubePair` result, pending/commit result, and active certificate
   tail before/after; and low-rate manager slot/replan outcome;
5. an aggregate transcript groups the first false category by identity.

No persistent diagnostic schema/topic, enum/reason, parameter, gate, retry,
latch, A5/Filter/C2/H2-condition change, or physical-control change is
permitted. The code must not emit from 50 Hz governor paths.

## One episode and rollback

Build the instrumented binary, then run exactly one loopback private master
with private ROS home/log directories, only
`phase_offset_manual_observe_only:=false`, the existing map/launch and one
`(8,0,1)` goal. Bag existing diagnostics/path/cmd/odom/goal/cloud/rosout,
save PIDs, maps, parameters, Build-IDs, stdout and checksums. Stop after at
most 70 seconds after goal publication; never start a second run.

Regardless of build/run result, immediately reverse every temporary hunk,
verify both source SHA values exactly, verify marker absence, rebuild restored
product, check restored Build-IDs, run Adapter/epoch/GVF-H2/continuous focused
tests, `git diff --check`, final status and port/process cleanup. Interpret
only after restoration. Do not implement any product repair; either write the
minimal subsequent execution plan or STOP pending evidence.

