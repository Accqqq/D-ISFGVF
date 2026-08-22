# Codex P1 T1–T3 self-audit

```text
DOCUMENT_ROLE=STAGE_SELF_AUDIT
STAGE=P1_T1_T2_T3_CONTINUOUS_VIABLE_PORT
RESULT=PASS__P1_ONLY
EXECUTION_SPEC=Codex_P1_Continuous_Viable_Port_T1_T3_Execution_Spec_2026-08-17.md
P2A_P2B=NOT_AUTHORIZED__NOT_STARTED
GO=NOT_GRANTED__P2_AND_PHYSICAL_CLOSED_LOOP_REMAIN_OUT_OF_SCOPE
```

## Result

P1 is complete.  Normal path/tube-owner acceptance now uses the Runtime
continuous exact-PWL witness instead of requiring that an unchanged retained
delta remain inside a later future-seam/horizon cross-section.  Current
containment was not relaxed: TubeEpochManager still checks the candidate's
exact current interval, and Runtime dry-run independently checks the same
current retained delta before it constructs a witness.

`buildContinuousExactPwlWitness()` is the existing finite deterministic
witness construction under its explicit name and contract.  Every held port
is checked against its complete exact-PWL path, including crossed knots; the
future evaluator obtains geometry/guidance only from the same immutable owner.
Failure remains existing `CERTIFICATE_DENIED`; it is not recorded, inferred,
or described as a proof that the viable set is empty.  The warmup gate remains
selection/commit-only and has not become a zero-offset normal-owner gate.

## Changes, all inside the P1 whitelist

- `phase_offset_runtime.h/.cpp`: renamed/documented the exact-PWL continuous
  witness operation; no new mode, reason, state, diagnostic, gate, parameter,
  or safety threshold.
- `phase_offset_matched_adapter.cpp`: replaced the future static-retained
  proxy with structural profile coverage.  Existing manager current checks,
  Runtime dry-run, immutable owner/profile/snapshot checks and all H2
  session/pin/CAS checks remain in force.
- `phase_offset_matched_adapter_test.cpp`: added:
  - 99 non-selected cycles keep retained delta and previous port bitwise
    unchanged; cycle 100 selects only a valid Runtime witness;
  - a positive armed fixture: current retained `.15` lies in `[.125,.30]`,
    while the future seam lower bound `.18` excludes unchanged `.15`; the
    Runtime witness accepts and the atomic normal pair commit succeeds;
  - a negative fixture: current raw/filtered interval `[.125,...]` excludes
    retained zero, so prepare/finalize cannot publish a pair and Runtime bits
    stay unchanged.

The existing Adapter regression matrix additionally covers stale/retired
owner-session, captured Runtime-bit changes, pin mismatch, frozen map advance,
and concurrent command/timer interleavings.  No profile/sample/geometry or
witness is reused across owner, generation, session, map sequence or snapshot.

## Verification

All commands were local build/test commands; no ROS master, launch, simulator,
or user process was started, attached, or terminated.

| Target | Result |
| --- | --- |
| `phase_offset_runtime_test` | 33/33 PASS |
| `phase_offset_matched_adapter_test` | 57/57 PASS |
| `phase_offset_port_projector_test` | 24/24 PASS |
| `phase_offset_tube_epoch_manager_test` | 49/49 PASS |
| `phase_offset_tube_epoch_integration_test` | 10/10 PASS |
| `continuous_phase_path_test` | 6/6 PASS |
| `gvf_switch_policy_test` | 68/68 PASS |
| `git diff --check` | PASS |

Dependency/range review confirmed that the P1 edits do not introduce recovery
mailbox/P2 behavior, alter TubeFilter/cross-section/margins, or modify H2
CAS/pin/session/runtime-bit contracts.  The only matching configuration text
found by the prohibited-term search was the pre-existing adapter parameter
read for `boundary_slope_max`; it was not modified.

## Stop statement

Stop after P1.  P2 recovery arbitration/owner work, H2 redesign, physical ROS
closed-loop validation, and any tube-geometry or Filter change remain outside
this authorization and were not started.
