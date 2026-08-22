# Active pair control-diagnostics publication — execution specification

```text
DOCUMENT_ROLE=DEDICATED_EXECUTION_SPEC
DATE=2026-08-17
STATUS=AUTHORIZED_BY_USER_2026-08-17
AUTO_ADVANCE=false
STAGE=ACTIVE_PAIR_DIAGNOSTIC_SNAPSHOT_WIRING
```

## Observed defect

With an immutable `PathTubePair`, `PhaseOffsetMatchedAdapter::update()` completes
the pair-owned Runtime step, fills the existing manual diagnostics, and returns
without constructing the existing `ControlPublishSnapshot`.  The timer publishes
manual/epoch/tracking diagnostics only from that snapshot.  Consequently an
active pair can execute while those control-owned telemetry topics remain empty.

This is an observation-wiring defect.  It is not evidence of an A5 tube, Filter,
Runtime selection, H2 transaction, or physical-control defect.

## Authorized change

Whitelist:

- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
- this execution specification and its self-audit

At the pair-owned branch's existing final output point—after all Runtime/output,
pair provenance, legacy tube status, and 83-field diagnostics are complete—call
the existing `makeControlPublishSnapshot()` and then retain the existing return.
The snapshot must carry the same request, exact immutable pair epoch, and final
output facts for that command cycle.  It must not mutate Runtime, selection,
validity, output guidance, pair authority, manager state, or commit ordering.

The existing staged-pair epoch currently copies the prepared Active profile but
leaves its Candidate profile empty, while the pre-existing pair output exposes
that same immutable profile as both Candidate and Active.  The staged epoch may
therefore copy that *same pointer* into its Candidate profile at construction so
the existing publisher's output-to-epoch consistency check can pass.  It does
not construct, filter, install, reuse across owners, or alter any tube profile.

## Explicitly prohibited

- Any change to Runtime/control selection, retained port, H2/CAS/pin/session,
  A5 geometry/Filter/validator/margins, map policy, parameters, launch files,
  schemas, `AGENTS.md`, or `gvf_manager`.
- New gate, mode, state, reason, enum, diagnostic field, ROS parameter, retry,
  tuning, or fallback behavior.
- Reuse of an old tube/profile/sample on another owner.

## Required verification

Focused unit coverage must prove that pair-selected, pair-denied/nonselected,
and observe-only/pre-gate pair steps publish a snapshot with fields consistent
with the final output and pair epoch, while adding no Runtime or pair-authority
mutation.  Existing no-pair snapshot behavior must remain covered.

Run the Adapter, Runtime, and H2 focused regressions, `git diff --check`, and a
whitelist/status self-audit.  Then run one task-owned private ESDF active episode
with the previously authorized `phase_offset_manual_observe_only:=false`
override only, checking that existing manual/epoch/tracking topics publish and
that H2 replacement/goal behavior is not regressed.  Do not touch user ROS
processes; clean up only task-owned processes.
