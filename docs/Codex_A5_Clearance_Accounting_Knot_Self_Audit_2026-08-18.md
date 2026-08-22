# A5 future-knot clearance accounting self-audit — 2026-08-18

```text
DOCUMENT_ROLE=PRIVATE_A5_CLEARANCE_ACCOUNTING_KNOT_SELF_AUDIT
STAGE=A5_ACTUAL_KNOT_CLEARANCE_ACCOUNTING
AUTO_ADVANCE_TO_A7=false
PRODUCT_BEHAVIOR_CHANGED=false
CLASSIFICATION=INSET_ZERO_EXCLUSION
```

## K0 provenance and evidence boundary

The fixed episode is the task-owned loopback run recorded in
`/tmp/a5a6_g1b_20260818/evidence/launch.log`.  The first false point is from
the same runtime path revision and immutable map observation identified by
`source_rev=1`, `seq=2`, and `map_seq=92`; the pointwise logs use the same Tube
build and map observation for the adjacent knots.  The checked-out HEAD at
audit start was `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`.  Existing dirty
worktree changes were retained.  The prior dynamic run's formation-planning
binary evidence is recorded in the preceding A5/A6 audit (sha256
`93f7b51c...`, Build-ID `c339b2bd...`); the current rebuilt binary is
`devel/lib/bspline_race/formation_planning`.

The launch configuration fixes the relevant units and layers:

```text
planning/safe_distance                 = 0.40 m (planner cost/query contract)
sdf_map/resolution                     = 0.10 m
sdf_map/obstacles_inflation            = 0.099 m
gvf/gvf_inflation                      = 0.099 m
sdf_map/manual_obstacle_inflate        = 0.10 m
phase_offset/tube/preincluded_map_uncertainty = 0.10 m
phase_offset/tube/uav_radius            = 0.25 m
phase_offset/tube/localization_margin  = 0.05 m
phase_offset/tube/tracking_error_bound = 0.15 m
phase_offset/tube/raw_map_uncertainty  = 0.10 m
```

The planner's ESDF/occupancy side is therefore already inflated by its
configured occupancy layer (approximately 0.099–0.10 m).  The Tube's
immutable cloud snapshot is a separate evidence source.  The dynamic knot
log exposes `map_seq`, but does not expose a categorical
`snapshot_included_map_inflation` value or prove that it is numerically equal
to the map-uncertainty bound.  That field is consequently recorded as
`not observed / not charged`; it is not silently equated to map uncertainty.

## K1 same-path/snapshot pointwise accounting

The current Builder implementation defines `TubeRawSample::pre_inset_*` as
the selected direct-clearance component and defines `raw_*` as
`pre_inset_* + continuous_inset` (the post-inset Filter input).  The runtime
log labels this latter interval `raw=[...]`.  Therefore the pre-inset values
below are the exact inverse of the recorded 0.10 m inset; no map query was
repeated and no legacy ROS payload was used to guess them.

| same `source_rev=1`, `seq=2`, `map_seq=92` | previous knot `w=2.382096295` | target knot `w=2.431715365` | next knot `w=2.481334435` |
|---|---:|---:|---:|
| recorded clearance/status (`min_clearance`, `clearance_recorded`) | 0.527746230 m, recorded=1 | 0.526382393 m, recorded=1 | 0.524984462 m, recorded=1 |
| planner `safe_distance` | 0.40 m | 0.40 m | 0.40 m |
| Tube raw clearance request radius / status | 0.45 m / recorded=1 | 0.45 m / recorded=1 | 0.45 m / recorded=1 |
| planner ESDF/occupancy inflation layer | 0.099–0.10 m configured | 0.099–0.10 m configured | 0.099–0.10 m configured |
| pre-inset direct-clearance bounds | `[-0.150000000, 1.693750000]` | `[-0.081250000, 1.625000000]` | `[-0.037500000, 1.581250000]` |
| pre-inset contains zero | yes | yes | yes |
| continuous inset / snapshot resolution | 0.100000000 m | 0.100000000 m | 0.100000000 m |
| post-inset raw / Filter input | `[-0.050000000, 1.593750000]` | `[0.018750000, 1.525000000]` | `[0.062500000, 1.481250000]` |
| post-inset contains zero | yes | **no** | no |
| Filter output | `[-0.016890512, 1.560640512]` | `[0.022804744, 1.520945256]` | `[0.062500000, 1.481250000]` |
| Filter contains zero | yes | no | no |
| validator cover radius | 0.961717985 m | 0.918075983 m | 0.874436363 m |
| validator requested clearance | 1.411718985 m | 1.368076983 m | 1.324437363 m |
| validator observed / zero surface covered | 1 / 1 | 1 / 0 | 1 / 0 |
| UAV radius | 0.25 m | 0.25 m | 0.25 m |
| map uncertainty / preincluded map uncertainty | 0.10 / 0.10 m | 0.10 / 0.10 m | 0.10 / 0.10 m |
| localization / tracking | 0.05 / 0.15 m | 0.05 / 0.15 m | 0.05 / 0.15 m |
| full effective radius | 0.55 m | 0.55 m | 0.55 m |
| residual effective radius | 0.45 m | 0.45 m | 0.45 m |
| snapshot included inflation/layer mask | not emitted; no equivalence claim | not emitted; no equivalence claim | not emitted; no equivalence claim |

The validator request follows the existing contract
`residual_effective_radius + cover_radius + epsilon`; it is evidence of the
continuous cover check, not an additional erosion of the cross-section.

## K2 first-false classification

```text
INSET_ZERO_EXCLUSION
```

At the target knot, zero is present in the direct pre-inset robust component,
then is removed by the single resolution-derived continuous inset:

```text
pre-inset       [-0.081250000, 1.625000000]  contains 0
inset           0.100000000 m
post-inset/raw  [ 0.018750000, 1.525000000]  excludes 0
Filter          [ 0.022804744, 1.520945256]  excludes 0
Validator       observed=1, zero_surface_covered=0
```

This is not `FILTER_ZERO_EXCLUSION` or `VALIDATOR_ZERO_EXCLUSION`, because
the zero loss precedes those layers.  It is not
`PLANNER_TUBE_ROBUST_CONTRACT_MISMATCH`: the same point's pre-inset component
contains zero, so the Branch-B pre-inset-nonzero predicate is false.  It is
not a proven duplicate-margin case: the only explicit resolution inset is
0.10 m, and the snapshot's included inflation is not numerically identified
with map uncertainty.  The centerline clearance query is recorded as known
and above planner `safe_distance`, but the robust final port still fails at
the inset boundary.  No nonzero offset is installed or used to rescue it.

The preceding knot remains zero-containing after inset and Filter; the next
knot remains zero-excluding.  Thus `w=2.431715365` is the first false knot in
this same path/snapshot episode.

## K3 disposition and scope boundary

No conditional repair was justified.  Changed files in this stage:

```text
docs/Codex_A5_Clearance_Accounting_Knot_Self_Audit_2026-08-18.md  (new)
```

No planner path/source, `planning/safe_distance`, ESDF/map inflation, Tube
margin, continuous inset, cover, Filter, Runtime, launch, map, diagnostic
schema, or CMake file was changed.  The fail-closed result is preserved.  A
future planner-path execution request is required to change the centerline
or planner clearance contract; that work is outside this A5 accounting stage.
Do not advance to A7.

## Verification

Targeted tests passed after the audit build:

```text
phase_offset_clearance_audit_test          22/22 PASS
phase_offset_tube_cross_section_test        7/7 PASS
phase_offset_tube_builder_test              7/7 PASS
phase_offset_tube_filter_test              15/15 PASS
phase_offset_tube_surface_validator_test    9/9 PASS
phase_offset_tube_epoch_manager_test       50/50 PASS
phase_offset_runtime_test                  33/33 PASS
phase_offset_matched_adapter_test          65/65 PASS
phase_offset_raw_candidate_diagnostics_test 8/8 PASS
cmake --build build -j2                         PASS
git diff --check                                PASS
```

The whitelist audit found no stage-product edits outside the self-audit
document; all pre-existing user-owned worktree changes were preserved.
