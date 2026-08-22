# A6-P1R Structural-Seam Preservation Correction — Self Audit

Date: 2026-08-20  
Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
Scope: A6-P1R only.

## Result

`PASS`.

`BuildOwnerAlignedTubePreview()` now applies the required canonical
precedence only while constructing its immutable preview partition:

```text
immutable internal owner seam
  > exact required current phase
  > ordinary supplied/range-neighbour knot
```

The helper identifies internal boundaries of the immutable owner separately
from planner/range hints.  If the exact required current phase is not equal to
such a seam but is within the already-frozen TubeBuilder current-match
tolerance, the exact seam is retained as the one current-match representative.
The near current value is not also inserted, so no sub-epsilon seam/current
cell is created.  If no structural seam is in that direct equivalence class,
the A6-P1 exact-current behavior remains unchanged.  Every retained state is
still evaluated directly from the same immutable owner.

No tolerance, Tube geometry, certificate policy, Runtime, manager, header,
launch, parameter, map/snapshot provenance, ROS schema, topic, or thread was
modified.  No launch was started for A6-P1R.

## Regression coverage

Added `OwnerAlignedTimerPreservesStructuralSeamNearCurrentOnBothSides` using
`MakeCertifiedThreePieceOwner()` and the internal owner seam `w=1.0`.

- `current_w = 1.0 - 5e-10`
- `current_w = 1.0 + 5e-10`

For both sides, `5e-10` is strictly greater than
`kPreparedCoverageTolerance` (`1e-10`) and strictly less than the frozen
TubeBuilder current-match tolerance (`1e-9`).  The test proves candidate
raw/filter/complete, current-found/current-complete diagnostics, exact seam
retention, no simultaneous exact-current knot, one current-match
representative, strictly ordered cells, successful per-cell immutable-owner
certificates, and certified-cell `continuous_inset=0` rather than a fixed
inset fallback.

The full adapter suite also retains the A6-P1 exact seam, near supplied knot,
near preview boundary, three off-grid currents, and prepared Pair coverage.

## Documentation fact correction

The A6-P1 audit now correctly records that
`/tmp/a6_p1_20260820_esdf_active_1` was produced by A6-P1's isolated active
launch.  M3-L1 observe-only evidence remains explicitly inherited.  It also
records the existing `goalCallback()` mapping
`goal_pt.z = msg.z + 1.0`: message `(8,0,1)` therefore maps to internal goal
`(8,0,2)`, consistent with final odometry near `z=2`.

## Verification

Passed commands:

```text
catkin_make -j2 phase_offset_matched_adapter_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test
ctest --output-on-failure -R '(_ctest_phase_offset_navigation_gtest_phase_offset_tube_(cross_section|builder|filter|surface_validator)_test|_ctest_phase_offset_navigation_gtest_phase_offset_(certified_tube_builder|runtime|tube_epoch_manager)_test|_ctest_bspline_race_gtest_(gvf_switch_policy|continuous_phase_path)_test)$'
catkin_make -j2
git diff --check
```

Results:

- targeted new seam regression: 1/1 passed;
- `phase_offset_matched_adapter_test`: 80/80 passed;
- selected CTest regression set: 9/9 passed, 0 failed;
- full `catkin_make -j2`: passed, including `formation_planning`;
- `git diff --check`: passed.

## Static scope checks

- Exactly two production calls invoke `BuildOwnerAlignedTubePreview()`; the
  ordinary timer explicitly passes `request->current_path.w`, and prepared
  construction explicitly passes `captured_w0`.
- `structural_seams` and current-match representative selection appear only in
  the preview partition helper.
- Candidate construction still enters through `TubeEpochManager::update`; no
  legacy Builder/Candidate path was introduced.
- No executable rounding or snapping was added.
- No Tube/manager/Runtime/launch/header/parameter file was modified for this
  stage.  This repository remains extensively dirty with user-owned tracked
  and untracked changes; they were preserved.

## Stop boundary

A6-P1R is complete.  Stop here: do not start A6-P2 bootstrap first-false work,
modify any frozen subsystem, or launch a runtime scenario without a new
execution specification.
