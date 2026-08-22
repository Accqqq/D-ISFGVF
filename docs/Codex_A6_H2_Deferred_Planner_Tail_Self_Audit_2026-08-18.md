# A6/H2 deferred planner-tail — self-audit

```text
DOCUMENT_ROLE=STAGE_SELF_AUDIT
STAGE=A6_H2_EVENT_DRIVEN_DEFERRED_PLANNER_TAIL
EXECUTION_SPEC=Codex_A6_H2_Deferred_Planner_Tail_Execution_Spec_2026-08-17.md
SOURCE_RESULT=PASS
DYNAMIC_RESULT=PARTIAL__H2_REPLACEMENTS_AND_GOAL_REACHED__DEFERRED_CHAIN_NOT_ATTRIBUTABLE
AUTO_ADVANCE=false
```

## Implemented contract

- An accepted point-phase candidate saves one private immutable planner
  artifact only when the old immutable path **and full-sample tail** still
  structurally reach the required future seam but its active certificate ends
  before that seam.  Terminal path exhaustion and all other empty-selector
  cases remain an immediate fail-closed rejection.  The value owns no pin,
  tube/profile/sample/clearance/witness, Runtime state, authority, or H2
  pending/completed mailbox entry.
- The artifact is retained only while its exact base pair remains current.  It
  is consumed once only for a direct same-owner pair at `G+1`, with the same
  source revision/session/path owner.  Generation skips, pair/session/revision/
  owner/goal drift, reset, a new replan, or an occupied H2 mailbox clear it.
- Before consumption installs anything, the manager re-runs existing candidate
  acceptance with the current progress, path end, index and goal; rebuilds the
  phase samples and C2 path; builds a wholly new owner/tube against the latest
  snapshot; and delegates the unchanged Runtime rebase/dry-run/exact CAS to
  the existing H2 transaction.
- Stage checks the expected `G+1` pair after taking the pin and requires exact
  capture equality.  A second refresh between observation and pin acquisition
  therefore rejects rather than stages on a stale pair.

No A5 geometry/Filter/validator/TubeEpochManager, Adapter, Runtime contract,
margin, rate, speed, parameter, launch file, gate, mode, reason, enum,
diagnostic schema or physical recovery behavior was changed.  No old owner
tube/profile/sample/witness is reused on a new connector.

## Source and deterministic verification

| Target | Result |
| --- | --- |
| `formation_planning` focused build | PASS |
| `gvf_switch_policy_test` | 78/78 PASS |
| `phase_offset_matched_adapter_test` | 63/63 PASS |
| `phase_offset_runtime_test` | 33/33 PASS |
| `phase_offset_port_projector_test` | 24/24 PASS |
| `phase_offset_tube_epoch_manager_test` | 49/49 PASS |
| `phase_offset_tube_epoch_integration_test` | 10/10 PASS |
| `phase_offset_tube_epoch_diagnostics_test` | 5/5 PASS |
| `continuous_phase_path_test` | 6/6 PASS |
| `git diff --check` | PASS |
| `rg TEMP_H2_ src/swarm_planner/bspline_traj` | no output |

The four new directed H2 tests prove: (1) certificate lag with structural
headroom stores no authority and publishes no mailbox payload, (2) terminal
path exhaustion cannot be deferred, (3) the first direct same-owner `G+1`
consumes exactly once, and (4) `G+2`, authority drift, reset and mailbox
conflict clear without a consume.  Existing Adapter tests retain the H2 new-owner,
pin/session, future-seam, Runtime rebase/dry-run and final-CAS rejection
matrices; existing continuous-path tests retain the strict connector-interior
non-reuse proof.

## Scope audit

This stage changed only its approved manager orchestration/header/test files
and its two documents.  The pre-existing broad dirty worktree remains
preserved.  No user ROS process has been attached to, signalled or stopped.

## One private active ESDF episode

The one task-owned episode used private master `127.0.0.1:11331`, private
`ROS_HOME`/logs and only
`phase_offset_manual_observe_only:=false`.  It recorded 108 seconds, 12,497
messages and 512.0 MB at:

`/tmp/a6_h2_deferred_tail_active_20260818_r2/evidence/h2_deferred_tail_active.bag`

All task-owned nodes, recorder and master were stopped; port 11331 is released.
No user process was attached to, signalled or stopped.

Observed physical facts:

- The point goal was reached (`POINT_GOAL`, reported distance `0.191 m`); the
  final recorded odometry is approximately `(8.000, 0.000, 1.000)`.
- The bag has one initial nonempty path followed by two nonempty H2 path
  replacements (115, 71 and 29 poses).  Their publication times follow the
  two logged C2 successes, and no `all_candidates_path_end_clamped` or terminal
  governor HOLD occurred after bootstrap.
- Four other replans still logged `replan not installed` at phases 0.406,
  2.312, 5.865 and 7.988.  The retained public schemas do not identify their
  selector/Adapter/Runtime first-false branch.

The run therefore establishes that the active binary can execute H2
replacements and reach the goal.  It does **not** establish the new specific
chain `certificate-lag no seam -> direct same-owner G+1 -> deferred-tail
replacement`, because that private artifact intentionally has no ROS-facing
state or diagnostic and the run used no temporary instrumentation.  This is
not a failure of the terminal safety contract and it is not permission to add
logging, a retry, a gate or parameter tuning.  The deferred-chain dynamic
criterion remains **NOT PASS / not attributable by this one allowed run**.

The earlier `w≈4.453` accepted replan audit remains deliberately outside this
mechanism unless its selector is empty specifically because of certificate lag.
If that transaction has a nonempty seam set and fails later in C2, Adapter,
Runtime or final CAS, this stage neither retains nor masks it; the unchanged
existing H2 failure path remains authoritative.
