# Codex A6/H2 Runtime rebase self-audit

```text
DOCUMENT_ROLE=STAGE_SELF_AUDIT
STAGE=A6_H2_RUNTIME_REBASE_AT_FINAL_COMMAND_BOUNDARY
EXECUTION_SPEC=Codex_A6_H2_Runtime_Rebase_Execution_Spec_2026-08-17.md
RESULT=SOURCE_AND_FOCUSED_REGRESSION_PASS
AUTO_ADVANCE=false
DYNAMIC_H2_ACCEPTANCE=NOT_RUN__SEPARATE_PRIVATE_ROS_EPISODE_REQUIRED
```

## Implemented contract

The independent-review revision was followed exactly.

- `stagePathTubePair()` still requires exact old pair pointer, capture provenance,
  authority session and active pin lease. It no longer compares live Runtime bits
  against the pin's historical bits. Under the existing Runtime mutex it captures
  the then-live delta/previous port, uses those values for the isolated prepared
  tube build and first dry-run, and records them only as stage provenance.
- `preparePathTubePairCommit()` again captures live Runtime state under the same
  mutex after its existing exact pair/session/pin checks. Its existing new-owner
  structural coverage, exact `w < future_seam`, frozen-profile provenance,
  Runtime exact-PWL dry-run and latest categorical map veto remain unchanged.
  The latest delta/port become the private expected values in
  `PathTubePairCommitPreparation`.
- `finalizePreparedPathTubePairCommit()` keeps the exact pair/session/lease
  checks and single pair CAS. It now requires the live Runtime bits to equal the
  prepare-time expected bits, rather than stage/pin-history bits. It does not
  write Runtime. Any bit change after prepare rejects without publication.

No timer retry, gate, mode, reason, parameter, lifecycle state, profile/sample
reuse, A5 geometry/Filter change, C2 change, map-policy relaxation, or
`current_w < future_seam_w` relaxation was added. `refreshPairFromTimerEpoch()`
retains its pre-existing historical Runtime-bit rejection.

## Scope audit

The only files edited for this stage are within the approved whitelist:

- `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
- this audit and the execution specification.

The worktree was already broadly dirty and remains preserved. No unrelated
changes were reverted, staged, committed, or cleaned. `git diff --check` passed.

## Rebase-focused tests

- A production quintic replacement now mutates Runtime after acquiring the old
  pin and before stage, then again between stage and prepare. Stage and prepare
  each use their own latest values, while the pin continues to bind the old
  exact authority. The test also verifies connector-interior tube samples come
  from the new owner, not the old owner.
- A Runtime change between stage and prepare succeeds only through the latest
  prepared dry-run/CAS predicate and leaves the live Runtime state unchanged by
  pair installation.
- A Runtime change after prepare causes final CAS rejection with no pair
  publication.
- Existing tests retain coverage of phase-at-seam rejection, wrong authority,
  retirement/session, pin/lease mismatch, frozen/latest map rules, timer-refresh
  historical-bit rejection, no path-only authority and 100-cycle Runtime
  non-pollution.

## Verification

| Target | Result |
| --- | --- |
| `phase_offset_matched_adapter_test` | 58/58 PASS |
| `phase_offset_runtime_test` | 33/33 PASS |
| `phase_offset_port_projector_test` | 24/24 PASS |
| `phase_offset_tube_epoch_manager_test` | 49/49 PASS |
| `phase_offset_tube_epoch_integration_test` | 10/10 PASS |
| `continuous_phase_path_test` | 6/6 PASS |
| `gvf_switch_policy_test` | 74/74 PASS |
| `git diff --check` | PASS |

The focused GVF build emitted two pre-existing compiler warnings in
`gvf_manager.cpp`, an out-of-whitelist file that this stage did not edit; the
build and all listed tests completed successfully.

## Remaining acceptance boundary

No ROS master, simulator or user process was started, attached to or stopped in
this source stage. The next required evidence is a task-owned private ROS active
episode, with only the already-authorised
`phase_offset_manual_observe_only:=false` override, demonstrating one real H2
replacement before the old local path ends. Until that occurs, dynamic navigation
and physical closed-loop acceptance remain **NOT PASS**; this patch must not be
represented as an A5 geometry fix or as a physical-flight pass.
