# Z1 Self-Audit: Planner-Authoritative Zero-Connected Tube

Date: 2026-08-20

## Scope

Implemented only Z1 from
`Codex_Planner_Authoritative_Zero_Tube_Simplification_Execution_Spec_2026-08-20.md`.
The planner remains the executable `delta=0` centreline authority.  Tube now
computes only zero-connected offset capacity and represents unavailable
nonzero proof as a valid `[0,0]` planner baseline.

## Whitelist audit

Product and test edits are confined to the Z1 whitelist:

- `phase_offset_navigation` cross-section, builder, filter, surface validator
  call layer, epoch types/manager, and their tests;
- `phase_offset_matched_adapter` and its test;
- clearance audit and its test;
- this self-audit, the Z1 specification, handoff, and the five permitted
  proposal locations.

Two direct API-compile dependants outside the nominal list were minimally
updated after removing `preferred_delta`:

- `phase_offset_raw_candidate_diagnostics.cpp` keeps its fixed-size historical
  slot as `deprecated_current_contains_preferred_delta = 0`;
- `phase_offset_raw_candidate_diagnostics_test.cpp` no longer writes the
  removed field.

No `gvf_manager`, Runtime implementation, C2 connector, planner, launch
parameter values, gradient, pin registry, or mailbox/H2 orchestration file was
modified.

## Semantic checks

- Production cross-sections use `planning/safe_distance`, loaded by the
  adapter, as their sole clearance request.  Legacy robust-margin values are
  diagnostics only.
- Cross-section expansion starts at zero, stops at the first non-safe point,
  and never selects a disconnected nonzero component.
- Inset clamps to zero rather than crossing it.  The exact PWL filter preserves
  zero when every raw knot contains zero.
- `TubeProfileClassification` is the one authority classification:
  `ZERO_ONLY_PLANNER_BASELINE` or `OFFSET_CERTIFIED`.
- Surface validation failure, UNKNOWN, and query-limit failure collapse the
  profile to finite, zero-slope `[0,0]` knots instead of invalidating the
  planner baseline.
- Neutral zero-only observations install as observable Tube profiles but do
  not request bootstrap, create a PathTubePair authority, or arm nonzero
  Runtime offset execution.  A retained nonzero state does not adopt a new
  zero-only profile and is not reset by Z1.
- A production API scan finds no retained preferred-component geometry or
  selection API.  The only identifier match is the intentionally deprecated
  raw-diagnostics slot; the only `components` match is a test comment.

## Verification

Completed focused tests:

- `phase_offset_tube_cross_section_test`: 7/7
- `phase_offset_tube_builder_test`: 9/9
- `phase_offset_tube_filter_test`: 16/16
- `phase_offset_tube_surface_validator_test`: 11/11
- `phase_offset_tube_epoch_manager_test`: 52/52
- `phase_offset_raw_candidate_diagnostics_test`: 8/8
- `phase_offset_clearance_audit_test`: 22/22
- `phase_offset_matched_adapter_test`: 75/75
- `catkin_make -j2`: passed after the final test-source changes.
- `git diff --check`: passed.

The workspace was already substantially dirty before Z1.  The final
`git status --short` confirms the Z1-owned untracked files remain within the
whitelist (plus the two documented direct compile dependants); tracked changes
outside that set predate this stage and were not edited here.

The very first incremental rebuild of the large adapter test target may report
an intermittent `undefined reference to main` immediately after recompiling
its object; `nm` confirms the global `main` is present and a no-edit retry
links successfully.  This is a build-system timing artifact observed before
and after the Z1 changes, not a source failure.

## Explicit follow-up boundary

Z1 deliberately does not solve active-nonzero handoff liveness.  H2 remains a
later stage: deterministic seam selection, asynchronous successor build,
deadline, certified recenter-to-zero fallback, single execution authority, and
pin/mailbox removal remain outstanding.
