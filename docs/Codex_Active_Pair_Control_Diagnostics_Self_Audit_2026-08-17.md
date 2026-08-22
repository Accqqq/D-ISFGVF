# Active pair control-diagnostics publication — self-audit

```text
DOCUMENT_ROLE=STAGE_SELF_AUDIT
STAGE=ACTIVE_PAIR_DIAGNOSTIC_SNAPSHOT_WIRING
EXECUTION_SPEC=Codex_Active_Pair_Control_Diagnostics_Execution_Spec_2026-08-17.md
RESULT=PASS__OBSERVATION_WIRING_ONLY
AUTO_ADVANCE=false
```

## Implemented scope

Only the following product/test files changed:

- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

The pair-owned `update()` branch now creates the existing immutable
`ControlPublishSnapshot` only after it has already completed Runtime/output,
legacy tube status, and the 83-field manual diagnostics.  It then retains the
same `return output.selected`.  The call receives the exact pair-owned epoch
and final output; it does not mutate Runtime, selection, guidance, the pair
authority, TubeEpochManager, H2 transaction, or commit order.

Focused verification found an existing snapshot-consistency omission: a newly
staged immutable pair epoch had its Active profile but no Candidate profile,
although the pre-existing pair branch exposes that same profile as both
Candidate and Active evidence.  The staged epoch now holds the same immutable
profile pointer for both fields.  No profile is constructed, filtered,
installed, or reused across owners by this correction.

No header, `gvf_manager`, Runtime, H2 CAS/pin/session, A5 geometry/Filter,
margin/map policy, schema, parameter, launch file, or `AGENTS.md` was edited.
No gate, mode, reason, enum, diagnostic field, parameter, retry, or fallback
was introduced.

## Focused unit/regression evidence

The three added/extended Adapter fixtures cover:

- a pair command before gate opening and after normal selection;
- an observe-only pair command;
- a pair categorical certificate denial;
- exact snapshot/profile/epoch consistency, pair identity stability, and no
  nonselected Runtime retained-port mutation.

Results:

| Target | Result |
| --- | --- |
| `phase_offset_matched_adapter_test` | 60/60 PASS |
| `phase_offset_runtime_test` | 33/33 PASS |
| `gvf_switch_policy_test` | 74/74 PASS |
| `phase_offset_tube_epoch_integration_test` | 10/10 PASS |
| `phase_offset_tube_epoch_manager_test` | 49/49 PASS |
| `continuous_phase_path_test` | 6/6 PASS |
| `git diff --check` | PASS |

The workspace's generated `phase_offset_matched_adapter_test` link command
contains `libgtest.so` but omits `gtest_main`, so the normal CMake target fails
with `undefined reference to main`; CMake is outside this stage whitelist.  The
already compiled test object was therefore linked only to a private `/tmp`
binary with the system `libgtest_main.a`; no repository/build configuration was
edited.  This is a test-infrastructure limitation, not a product result.

## One private active ESDF episode

```text
PRIVATE_MASTER=http://127.0.0.1:11737
ONLY_LAUNCH_OVERRIDE=phase_offset_manual_observe_only:=false
OTHER_LAUNCH_ARGUMENTS=UNCHANGED_DEFAULTS
PRODUCT_SOURCE_OR_LAUNCH_EDITS_DURING_RUN=NONE
USER_ROS_OR_PROCESSES_TOUCHED=NO
ARTIFACT_ROOT=/tmp/active_pair_diagnostics_20260817/evidence
```

Bag `active_pair_diagnostics.bag` is 56.7 seconds / 6089 messages with SHA-256
`86560036a90c6f76eddf8a2004d8a8f92f5b635c093b740b9cfb1839c7b2eb65`.

The previously silent pair-owned control topics now published:

- `phase_offset_manual/diagnostics`: 6 messages, 83 fields each;
- `phase_offset_manual/tube_epoch_diagnostics`: 6 messages, 50 fields each;
- raw candidate and cloud snapshot evidence: 35 messages each.

The episode reached the test goal (`distance=0.187`).  Logs show the initial
range `[0.050, 5.707]` followed by installed later ranges `[4.535, 8.361]` and
`[6.805, 8.637]`; no `all_candidates_path_end_clamped` hold occurred.  This is
an observation-wiring acceptance only; it does not claim a new A5, H2, tracking
bound, or recovery-owner acceptance result.

All task-owned ROS nodes, master, recorder, and diagnostic subprocesses were
stopped.  Pre-existing user ROS processes were neither queried nor altered.

## Read-only bag reanalysis

The two control-facing schemas are unchanged:

- manual payload: 83 fields; `delta_ref/delta/delta_error` are fields 8–10,
  raw/final `(u_w,u_delta)` are 11–14, selected/valid are 27–28,
  current-inside/next-inside are 57–58, and runtime tracking norm/bound are
  67–68;
- epoch payload: 50 fields; active/current validation/current geometry/current
  bounds/retained-inside are 15–19, tracking-within-bound/horizon are 24–25,
  retained delta/tracking norm/bound are 26–28, active-display-certified is
  42, runtime mode is 44, selected is 45, certificate-denied is 46, and
  Runtime failure reason is 49.

There is no separate `current_tracking` or `max_tracking` field.  The only
tracking scalar is the current command snapshot's `tracking_error_norm`; any
reported maximum below is only the maximum of these six event snapshots, not a
continuous-time or per-command maximum.

| Set | n | tracking norm p50 / p95 / recorded max (m) | S4 `<0.05 m` |
| --- | ---: | --- | --- |
| All control snapshots | 6 | 0.05448 / 0.06789 / 0.06789 | NOT PASS (1/6 ≤ 0.05) |
| Strict normal snapshots | 3 | 0.05079 / 0.05448 / 0.05448 | NOT PASS (1/3 ≤ 0.05) |

`p50`/`p95` use nearest-rank on this tiny sample.  A strict normal snapshot
requires `NORMAL`, selected, active/current-valid/current-geometry/current-
bounds/retained-inside, tracking-within-bound, horizon-sufficient, and no
certificate denial.  Its three values are 0.05079, 0.05448 and 0.04946 m.
Thus the field is real Runtime tracking telemetry, but this short event-driven
archive is both numerically non-passing and too sparse to establish a global
S4 maximum or a steady-state acceptance claim.

All six rows had a complete Candidate and active profile, active-current
validation, safe current map status, tracking-within-bound against the 0.15 m
Runtime bound, horizon-sufficient, epoch state `ROLLING`, reason `NONE`, and
failure reason `NONE`.  Per-command Runtime facts still varied: rows 0 and 3
were `WAITING_FOR_CANDIDATE` with current-bounds/retained-inside false;
rows 1, 2 and 4 were `NORMAL` and selected; row 5 was
`CERTIFICATE_DENIED` and nonselected.  Active-display-certified likewise was
false, true, true, false, true, false.  The last denial is an observed Runtime
fact, not an inferred epoch reason: epoch reason remained `NONE`.

The selected normal-port rows carried respectively:

- source revision 2: delta -0.04985 / -0.05471, final ports
  `(0.02560,-0.24304)` / `(0.02800,-0.24421)`;
- source revision 3: delta -0.02957, final port `(-0.01837,0.25000)` after
  the raw `u_delta=0.63442` was limited by the existing projector.

The final row is nonselected/certificate-denied and reports final port `(0,0)`.
The two schemas have no final physical-command-source field; the independent
GVF log reports `VEL_MATCH_GOVERNOR` for the 22 executing command summaries and
one pre-initialisation `GOVERNOR_INVALID_HOLD`, rather than a path-end hold.

H2 evidence is sparse before the first replacement: the only pre-replacement
control row is pre-gate/nonselected (tracking 0.06789 m), so no before/after
tracking causality claim is justified.  The first installed later range
`[4.535,8.361]` has two strict-normal rows (0.05079, 0.05448 m) and one
waiting row (0.06283 m).  The later `[6.805,8.637]` range has one
strict-normal row (0.04946 m) and one certificate-denied row (0.06051 m).
No `all_candidates_path_end_clamped` message occurs before final goal arrival.

The 56.7-second bag contains manual/epoch rows only over a 3.313-second active
window: their interarrival gaps are 2.302, 0.101, 0.100, 0.718 and 0.091 s.
Raw/cloud build evidence has 35 rows over 3.866 seconds.  This is expected
from the existing publication ownership: command cycles only overwrite the
latest immutable snapshot; `publishManual()` runs after a completed timer epoch
and emits 83/50 only if that snapshot and epoch have matching source/provenance
and output facts.  Raw/cloud evidence is emitted before that control-publish
match.  The vehicle reached the goal about 0.63 s after the final control
snapshot; the remaining bag window is idle/no-goal odometry.  Therefore six is
not a new telemetry rate and does not motivate a rate or gate change.

### Final `CERTIFICATE_DENIED` snapshot boundary

The final denial is at bag time `1786968550.868336678` (manual snapshot
`1786968550.868331671`), source/active revision 3, candidate/active epoch 1,
and map observation sequence 564.  Its direct fields are:

| Fact | Value |
| --- | --- |
| selected / valid | false / false |
| Runtime mode / certificate-denied | `CERTIFICATE_DENIED` (5) / true |
| active/current validation/current geometry/current bounds/retained-inside | all true |
| tracking / bound / horizon | 0.06051 m / 0.15 m / true |
| retained delta; current and next inside | -0.02457; true and false |
| raw port; final port | `(-0.02263, 0.62551)`; `(0, 0)` |
| epoch reason / Runtime failure reason | `NONE` / `NONE` |

`RuntimeExecutionStatus::executable` is not a published 83/50 field.  It is
nevertheless source-deducible here: the current facts first make Runtime
prepare a normal executable step; when both exact-PWL witness attempts fail,
Runtime changes mode to `CERTIFICATE_DENIED` but deliberately leaves that
prepared executable bit intact.  This is the P2a-r1 case, not a current-offset
outside case.

Thus `requiresCurrentStateRecovery()` is source-deducibly true for this output:
it sees `!selected && !valid` and the explicit Runtime certificate denial.
The bag/log carries no mailbox ticket, stage result, or consume result.  Its
strict observable boundary is therefore:

- staging is attempted only if the captured pair remains the exact live pair;
- staging can still fail for shutdown or an already-pending different identity;
- FSM attempts one exact-identity consume per tick, but P2a has no physical
  dispatch and no P2b owner.

There is no P2a/mailbox/ticket log, ROS topic, or GDB observation in this run,
so actual stage and consume cannot be claimed from this bag.  They must remain
**NOT OBSERVED**, rather than inferred as a successful physical recovery.

From 2.1 ms after the denial until goal arrival 0.631 s later, the bag records
32 `/position_cmd` messages at approximately 50 Hz.  Their position target
changes from `(7.842, 0.038, 1.713)` to `(8.000, 0.000, 2.000)`; the message's
velocity fields are zero.  After the denial the GVF log continues to report
`final_cmd_source=VEL_MATCH_GOVERNOR` and `fallback_reason=none`, not
`GOVERNOR_INVALID_HOLD`.  It also emits later phase/C2 attempts at 8.389–8.637
and a retained phase of 7.953 before goal arrival.  This establishes continued
command/phase activity, though it is not a per-command authoritative phase
time series.

The source explains the distinction: an unselected Adapter output does not
replace the precomputed lifted guidance; it only leaves the governor's reference
offset at its zero-port anchor.  The governor therefore consumes baseline
guidance on the current pair-owned path, but without a selected tube-matched
port.  This is **not** evidence of a completed P2 recovery, nor proof that the
certificate denial was harmless or transient—the archive has no subsequent
normal control snapshot before terminal arrival.  It is a short current-inside
Runtime witness denial followed by the known legacy/baseline-governor fallback
residue that P2b must eventually replace with an authorised recovery owner.

## Stop statement

This dedicated diagnostics stage is complete.  No subsequent architecture or
control stage is authorized by this audit.
