# Clearance margin alignment audit — self-audit

Date: 2026-08-18
Stage: `CLEARANCE_MARGIN_ALIGNMENT_AUDIT`
Status: T0–T3 evidence seam and tests complete; planner/tube safety numbers unchanged.

## Scope and files changed

Only the execution-plan whitelist was used:

- `phase_offset_clearance_audit.h/.cpp` now retain a pointwise path/snapshot
  accounting vector while preserving the legacy 60-field payload.
- `phase_offset_clearance_audit_test.cpp` exercises safe, insufficient,
  unknown, snapshot-inflation mismatch, inset zero exclusion, and explicit
  duplicate-evidence cases.
- `tube_surface_validator.h/.cpp` expose observed/min/max cover radius and
  maximum requested clearance as evidence-only fields; validation predicates
  and subdivision limits are unchanged.
- `bspline_traj/CMakeLists.txt` adds the standalone audit library and test.
- `tube_surface_validator_test.cpp` checks the new cover evidence.

No planner parameter, exact-PWL Filter rule, Runtime gate/latch, command path,
map generator, message schema, or swarm behavior was changed.

## T0/T1 evidence

The audit records, per sampled path knot, planner clearance, raw clearance
status/lower bound, snapshot sequence/resolution/included inflation, UAV/map/
localization/tracking margins, full and residual effective radius, continuous
inset, validator cover/requested radius, pre-inset/raw/filtered interval
bounds, and zero containment at each stage.

`same_path_snapshot` and `margin_accounting_complete` require an explicit raw
clearance query, nonzero snapshot sequence, exact source/tube revision match,
and pointwise path/profile agreement. Legacy distance-query diagnostics do not
claim immutable-snapshot provenance.

The existing code paths provide the following accounting:

```text
full radius     = UAV + map uncertainty + localization + tracking
residual radius= full - preincluded map uncertainty
builder inset   = snapshot resolution (offset-interval erosion)
validator query = residual + sampled cover radius + tiny epsilon
```

The snapshot's `included_map_inflation` is recorded separately and is not
assumed to equal map uncertainty. A duplicate margin is reported only when the
caller explicitly declares that equivalence and the values show an excess;
the normal equal-value case is reported aligned, not duplicated. Therefore no
parameter or mapping correction was justified in this stage.

## Verification

Built successfully:

```text
cmake --build build --target phase_offset_clearance_audit_test \
  phase_offset_tube_cross_section_test phase_offset_tube_builder_test \
  phase_offset_tube_filter_test phase_offset_runtime_test \
  phase_offset_matched_adapter_test -j2
```

The complete workspace build also passed:

```text
cmake --build build -j2
```

It rebuilt `formation_planning` successfully.  The emitted warnings were
pre-existing signedness/unused-variable/incomplete-enum-switch warnings in
legacy planner, manager, visualization, and simulator/control sources; none is
in this stage's whitelist or caused by the clearance audit.

Passed: audit 19 tests, cross-section 7, builder 6, Filter 15, surface
validator 8, Runtime 33, raw-candidate diagnostics 8 (its existing 49-field
schema), and matched adapter 63. `git diff --check` passed for the tracked
whitelist edits. Existing unrelated worktree modifications were preserved and
not attributed to this stage.

## Remaining risk / stop statement

The audit seam is standalone and does not publish a new ROS diagnostic schema;
callers must provide the immutable snapshot query and validator result when
they need runtime evidence. No evidence currently proves a physical duplicate
of UAV/map/discretization margins, so the numerical mapping remains frozen.
This stage is complete; do not tune planner safe distance, relax the Filter,
or add nonzero swarm offset integration under this execution specification.
