# A6/H2 same-authority timer refresh — self-audit

```text
DOCUMENT_ROLE=STAGE_SELF_AUDIT
STAGE=A6_H2_SAME_AUTHORITY_TIMER_REFRESH_RUNTIME_REBASE
EXECUTION_SPEC=Codex_A6_H2_Timer_Refresh_Runtime_Rebase_Execution_Spec_2026-08-17.md
SOURCE_RESULT=PASS
DYNAMIC_RESULT=NOT_PASS__UNATTRIBUTED_H2_REPLACEMENT_FAILURE
AUTO_ADVANCE=false
```

## Implemented boundary

Only the execution specification's Adapter header/source/test whitelist was
changed, plus this audit.

`TubeBuildRequest` now retains the existing command `gains` and `dt` as
private immutable timer-handoff data.  No ROS message, topic, parameter,
diagnostic schema, Runtime state, path state, gate, mode, reason or latch was
added.

`refreshPairFromTimerEpoch()` is now a two-part private transaction:

1. prepare retains the epoch's original immutable owner/profile/frozen-map
   provenance, captures the newest request only if it still has the exact same
   base pair pointer/generation/session/source/owner, snapshots live Runtime,
   and runs the existing `PhaseOffsetRuntime::dryRun()` exact-PWL witness on
   the candidate profile with that latest command state;
2. finalize rechecks the exact latest request pointer, pair/generation/session,
   Runtime retained delta and previous final-port bits, and absence of an H2
   pin before the single pair CAS.  It writes no Runtime or path state.

The existing latest categorical OCCUPIED/OUT_OF_MAP veto is reused with the
latest immutable command snapshot.  Candidate frozen-map provenance remains
unchanged.

No A5 geometry/Filter/validator/margins, control selection, H2 C2 path
transaction, pin/session contract, launch/configuration or governor code was
changed.

## Evidence interpretation

The earlier active-pair bag does **not** establish that a timer refresh
installed the rev3/map564 profile later seen in `CERTIFICATE_DENIED`.  rev3/map564
is the H2 initial pair; nearby raw timer candidates are map565/map566 and the
archive has no timer-CAS telemetry.  The repaired defect is instead
source-proven old-request Runtime-bit starvation: an ordinary selected command
can advance Runtime during timer construction, while the old completion tested
those later bits against the obsolete request without a rebase/dry-run.

## Focused verification

| Target | Result |
| --- | --- |
| Timer-refresh directed cases | 5/5 PASS |
| `phase_offset_matched_adapter_test` | 63/63 PASS |
| `phase_offset_runtime_test` | 33/33 PASS |
| `phase_offset_tube_epoch_manager_test` | 49/49 PASS |
| `phase_offset_tube_epoch_integration_test` | 10/10 PASS |
| `gvf_switch_policy_test` (H2/pin/session coverage) | 74/74 PASS |
| `continuous_phase_path_test` | 6/6 PASS |

The new directed coverage proves:

- changed Runtime bits during build are rebased and can install only after an
  exact witness;
- a post-prepare Runtime-bit or newest-request change rejects the final CAS
  without authority publication;
- a nominally current-valid candidate without an exact-PWL future witness
  cannot install;
- an explicitly unsafe latest ESDF observation cannot install.

## One private dynamic episode

```text
PRIVATE_MASTER=http://127.0.0.1:11741
ROS_HOME=/tmp/timer_refresh_runtime_rebase_20260817/ros_home
ONLY_OVERRIDE=phase_offset_manual_observe_only:=false
GOAL=(8,0,1)
USER_PROCESSES_TOUCHED=NO
```

The task-owned master/nodes were stopped after the one episode.  The launch
log shows initial pair startup at `[0.050,5.707]`; near the first terminal
handoff (`phase_w=4.508`) it records `replan not installed`, and later enters
`all_candidates_path_end_clamped` at phase `5.693` with goal distance about
`2.21 m`.  Thus dynamic navigation is **NOT PASS**.

This log has no stage/prepare/finalize/timer-CAS telemetry, so it cannot
attribute the failure to timer refresh, a Runtime witness, or another H2
predicate.  The recorder process did not persist beyond its launching shell;
the salvaged bag contains zero messages.  Therefore active map progression,
Candidate→Active→Certified→Selected transitions, tracking samples and
certificate-denial counts are **NOT OBSERVED** for this episode.  This is a
recording failure, not permission to repeat, tune, or modify product logic.

## Stop boundary

The authorized timer-refresh source correction and regressions are complete.
The remaining dynamic H2 replacement failure needs a separately authorized
observability/diagnosis stage before any further product change.  It must not
be represented as an A5/Filter correction or a closed-loop acceptance pass.
