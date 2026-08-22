# A6/H2 point-phase accepted replan first-false — temporary measurement specification

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DATE=2026-08-17
STAGE=A6_H2_POINT_PHASE_ACCEPTED_REPLAN_FIRST_FALSE
STATUS=TEMPORARY_INSTRUMENTATION / AUTHORIZED_BY_USER_2026-08-17
IMPLEMENTATION_AUTHORIZED=YES__ONE_PRIVATE_EPISODE_ONLY
AUTO_ADVANCE=false
```

## Objective

Classify the first existing false return for an **already accepted**
point-phase replan before it can install a new H2 path--tube frontend.  The
target event is the observed 11764 class at `phase_w≈4.453`, not a general
tube-performance experiment.  The temporary logs must observe existing
branches only; they must not alter their order, meaning, or outcome.

## Strict scope, whitelist, and baseline

Temporary source edits are limited to:

- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`

No header, CMake, launch, parameter, test, message, schema, or product
document changes are permitted.  Permanent outputs are this specification
and one resulting read-only audit only.

Before the temporary patch, record worktree status, `git diff --check`, and
these restored source fingerprints:

```text
gvf_manager.cpp                 078ae5378b49b7942b12ecef927f928f70fda0b0f5642d386691e650fdb07c18
phase_offset_matched_adapter.cpp f6b7b0a5fdf77ece8372175c0a949c6b40518bef268b26e6b7229fee72450521
phase_offset_matched_adapter.h   414015f908eecc51c75bd226a25163b96673b11f4016864bd67ef2447d24d7c1
libbspline_gvf.so Build-ID       e742720e7863e4f3770efa5abe2f8cb05ccfa817
libphase_offset_matched_adapter.so Build-ID 432e58b750d280b0a4c8cbdd39b6a136f29cdf05
```

Also confirm both temporary markers are absent before patching:

```text
rg 'TEMP_H2_POINT_REPLAN_FIRST_FALSE' src/swarm_planner/bspline_traj
# no output
```

## Exact temporary evidence

Use fixed-prefix `ROS_WARN`/stderr lines only:

```text
TEMP_H2_POINT_REPLAN_FIRST_FALSE
```

Instrumentation is allowed only on the low-rate accepted point-phase H2
transaction path.  It must emit the **first reached category only** for an
accepted candidate, in this fixed causal order:

```text
pin -> old_pair_or_slot -> selector -> c2 -> stage_pair -> pending_slot
    -> prepare -> finalize -> consume
```

For the exact category, emit `result=ok` only when control reaches the next
category.  At the first false, emit `result=false` and the existing immediate
sub-branch name, then return exactly as before.  A successful full handoff
may emit the complete ordered sequence.  Never emit from the 50 Hz command
or governor loop except the existing one-shot transaction calls above.

Allowed facts are only existing immutable/categorical identity and boolean
facts:

- **pin:** validity; old pair source revision, generation, authority session,
  and map sequence if a pair exists;
- **old_pair_or_slot:** old-pair structural/session predicates and the
  existing pending/completed-slot booleans;
- **selector:** captured `w`, certified interval endpoints,
  configured minimum forward distance, candidate count, and chosen seam;
- **c2:** candidate seam, input path ranges/counts, and existing C2 return;
- **stage_pair:** seam, transaction pair/session identity, and one existing
  sub-branch identifier from `stagePathTubePair`:
  `input`, `pin_lease`, `live_pair_or_session`, `owner_at_capture`,
  `prepared_epoch`, or `runtime_dry_run`;
- **pending_slot:** session/slot booleans and candidate-pair/session facts;
- **prepare/finalize/consume:** existing pair/session/generation identity,
  captured `w`, and boolean result only.

Do not print position, port vectors, delta values, unbounded sample data, map
payloads, or per-control-cycle diagnostics.  Do not introduce a transaction
ID, persistent field, parameter, counter, state, reason, enum, schema,
topic, timer, retry, latch, gate, lock-scope change, or new branch.  The
correlation key is the already existing captured phase plus pair identity.

The `stage_pair` sub-branch logging belongs in the adapter only after the
manager has entered `stage_pair`; it must preserve all existing early returns
and never log outside an H2 replacement carrying its existing pin/capture.
The adapter must not instrument timer refreshes.  `prepare`, `finalize`, and
`consume` logs are manager-side only and must not change their serialization
or CAS behavior.

## One private episode

Build only the affected `bspline_race` target, then run exactly one
task-owned, loopback ROS master with private ROS home/log directories and a
fresh unused port.  Use the checked-in
`phase_offset_esdf_tube_single.launch` with the sole established override
`phase_offset_manual_observe_only:=false`; preserve all checked-in tube,
Filter, C2, and point-phase values, including
`min_certified_forward_w=0.40`.

Publish exactly one `(8,0,1)` goal.  Record stdout/stderr, rosout, all
existing diagnostics/raw/cloud/epoch/path/cmd/odom/goal/local-map topics,
parameters, process IDs, port cleanup, binary fingerprints, and one bag.
Allow at most 70 seconds after the goal.  A target event absent in that one
episode is valid negative evidence: do not rerun, tune, or broaden scope.

The resulting audit must state whether the target class was observed and,
when observed, name the one first-false category and immediate existing
sub-branch.  It must distinguish exact same-run proof from merely nearby
earlier evidence.  It may not infer a product repair from a missing event.

## Mandatory rollback and acceptance

Immediately after the single episode, reverse every temporary hunk with
`apply_patch`; do not use reset, restore, clean, or stash.  Verify all three
source SHA-256 values above exactly, marker absence, and `git diff --check`.
Rebuild the restored product and verify the two restored Build-IDs.  Run the
restored focused regressions:

- `phase_offset_matched_adapter_test`;
- `phase_offset_tube_epoch_diagnostics_test`;
- `phase_offset_tube_epoch_integration_test`;
- `gvf_switch_policy_test` (including H2 coverage);
- `continuous_phase_path_test`.

Confirm that only task-owned ROS processes were stopped and the private port
is free.  No product repair, parameter change, or follow-on stage is
authorized after the audit; stop and report the evidence.
