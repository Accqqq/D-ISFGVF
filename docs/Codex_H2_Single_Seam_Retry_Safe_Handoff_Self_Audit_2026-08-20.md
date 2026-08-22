# H2-L1 Self-Audit: Single-Seam Retry-Safe Active Handoff

Date: 2026-08-20

## Scope

Implemented only H2-L1 from
`Codex_H2_Single_Seam_Retry_Safe_Handoff_Execution_Spec_2026-08-20.md`.
This is a lifecycle correction for an already active nonzero Path--Tube pair;
it does not change Tube geometry, Runtime mathematics, planner/C2 geometry,
global `w`, ROS schemas, parameters, threads, recenter behavior, authority
architecture, pin registry, or the two-mailbox design.

## Implementation audit

- Production future-seam selection is now one deterministic scalar:
  `selectStructuralFutureSeam()` returns the first eligible immutable,
  owner-aligned structural sample.  It rejects nonmonotonic or owner-mismatched
  input and never examines old Tube coverage.
- `stageFutureSeamPathTubeTransaction()` invokes `buildPhaseV2C2Frontend()` and
  `stagePathTubePair()` at most once per callback.  The old vector enumeration
  and same-callback fallback helper were removed.  A later periodic/collision
  callback obtains a fresh live `w` and may select its naturally later seam.
- Pending preparation copies the shared-pointer mailbox identity; it no longer
  moves pending before the lock-free adapter prepare.  Adapter `false`, phase
  generation drift, captured-phase mismatch, and final short-CAS `false` are
  retained as `RETRY_PENDING` while the same identity/session remains live and
  `w < future_seam_w`.
- Only manager-observable permanent facts clear pending: malformed/pin/session
  mismatch is `DROPPED_STALE`; a non-finite seam or `w >= future_seam_w` is
  `DROPPED_EXPIRED`.  The transaction pin is moved to a local owner before
  clearing and is destroyed after manager/Runtime locks leave scope.
- A successful adapter CAS is the only pending-to-completed transition.  It
  moves the pin outside the mailbox, clears pending and publishes the same
  frontend identity as completed under one manager serialization boundary.
- Completed consumption copies—not moves—the mailbox identity.  It validates
  pair/session/pin-free state, frontend mirror dimensions/finite monotone
  samples/owner range, and live adapter authority before any mirror mutation.
  Stale entries drop without mirror change.  A valid entry is cleared only
  after `applyPathTubeFrontendMirrorLocked()` and
  `installAuthoritativePathMirrorLocked()` both succeed.
- The remaining `installAuthoritativePathMirrorLocked()` false branch is not
  expected after the new prevalidation: every one of its input predicates
  (positive rows, 3-vector columns, matching row counts and finite monotone
  `w`) is checked first.  The branch retains completed and reports
  `RETRY_PENDING` defensively; it is not treated as a successful consume.
- The internal result enum is manager-local only:
  `NONE`, `RETRY_PENDING`, `COMMITTED`, `DROPPED_EXPIRED`,
  `DROPPED_STALE`, `CONSUMED`.  It is not a Runtime mode, navigation gate,
  Tube reason, parameter, or ROS message.

## Test audit

`gvf_switch_policy_test` adds/updates deterministic tests for:

- earliest scalar seam, exact lead boundary, certificate-lag independence,
  later-callback seam selection, nonmonotonic/owner mismatch rejection;
- retryable prepare failure retaining the *same* pending identity across two
  attempts, with the pin remaining exclusive until expiry;
- seam expiry and session drift dropping the pending entry and releasing its
  pin for a new acquisition;
- generation drift retaining the same pending identity;
- one successful prepare changing pending to one completed identity exactly
  once;
- completed success consuming exactly once, temporary pre-consume missing
  `pm.gvf_` retaining its completed identity until restored, and stale
  completed rejection leaving the FSM mirror unchanged.

The TimerBootstrap fixture now calls its timer-owned Tube build once before
expecting a bootstrap authority, and explicitly sets the Z1 planner clearance
to `0.40`.  Those are fixture contract updates, not production parameter
changes.

## Whitelist audit

Modified files are limited to the H2-L1 whitelist:

- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- this self-audit;
- `/home/cxq/ISF-GVF/handoff.md`.

The workspace was already dirty.  No adapter, Runtime, Tube, planner, launch,
or proposal source was edited by this stage.

## Verification

All selected tests and the required workspace build pass after final source
changes:

- `gvf_switch_policy_test`: 41/41;
- `phase_offset_matched_adapter_test`: 75/75;
- `phase_offset_runtime_test`: 35/35;
- `phase_offset_tube_epoch_manager_test`: 52/52;
- `continuous_phase_path_test`: 11/11;
- Z1 regressions: CrossSection 7/7, Builder 9/9, Filter 16/16, and
  SurfaceValidator 11/11;
- `catkin_make -j2`;
- `git diff --check`.

The first Runtime invocation used an older standalone test executable while
the navigation shared library had just been relinked; it faulted in a C++
value assignment before any test assertion.  Rebuilding the existing
`phase_offset_runtime_test` target without source edits restored a consistent
binary/library pair, after which it passed 35/35.  No frozen Runtime code was
modified.

## Explicit remaining boundary

H2-L1 does not create a successor when none can be staged before the old path
ends, and it does not recenter or release active offset authority.  Certified
recenter/recovery, a single `ExecutionAuthority`, and removal of the pin and
pending/completed mailboxes remain separate later stages.
