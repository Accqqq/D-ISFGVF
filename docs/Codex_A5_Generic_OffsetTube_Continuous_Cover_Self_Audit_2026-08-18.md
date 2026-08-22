# A5 generic OffsetTube continuous-cover self-audit — 2026-08-18

```text
DOCUMENT_ROLE=PRIVATE_A5_GENERIC_OFFSET_TUBE_CONTINUOUS_COVER_SELF_AUDIT
STAGE=A5_GENERIC_OFFSET_TUBE_CONTINUOUS_COVER
AUTO_ADVANCE_TO_A7=false
PRODUCT_BEHAVIOR_CHANGED=false
CLASSIFICATION=INSET_ZERO_EXCLUSION__C1_PROOF_BLOCKER
RESULT=NOT_PASS__FAIL_CLOSED_PRESERVED
```

## C0 first-false reproduction

HEAD at audit start: `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`. Existing user-owned dirty and
untracked worktree files were preserved. The prior same-path/snapshot evidence is
`source_rev=1`, `tube seq=2`, `map_seq=92`, target `w=2.431715365`:

```text
pre-inset       [-0.081250000, 1.625000000]   contains zero
continuous inset  0.100000000 m
post-inset/raw  [ 0.018750000, 1.525000000]   excludes zero
Filter          [ 0.022804744, 1.520945256]   excludes zero
```

The first false layer is therefore `INSET_ZERO_EXCLUSION`, as recorded in
`docs/Codex_A5_Clearance_Accounting_Knot_Self_Audit_2026-08-18.md`.

## C1 error-bound audit and blocker

The exact Builder path evaluator does use the planner's `p(w)`, `p_w`, `p_ww`, exact normal,
and curvature at each adaptive sample. A candidate cell can be described with the usual local
bounds:

```text
||p(w_mid) - (p(w0)+p(w1))/2|| <= sup||p''|| * h_w^2 / 8
||N(w)-N(w_mid)|| <= sup||N'|| * h_w / 2
|delta(w)-delta(w_mid)| <= L_filter * h_w / 2
```

and curvature regularity is checked by the cross-section intersection. However, the current
interfaces do not provide a validated cell-wise `sup||p''||`, `sup||N'||`, curvature-variation
bound, or equivalent certificate for all points between the exact samples. `PathStateQuery`
returns only pointwise `p`, `p_w`, and `p_ww`; `TubeSurfaceValidator` observes a deterministic
3x3 tessellation and uses:

```text
cover_radius = 1.1 * sampled_radius + 0.5 * snapshot_resolution
requested = residualEffectiveRadius + cover_radius + epsilon
```

The 3x3 observations and empirical `1.1` factor are not, by themselves, a verifiable upper
bound for an arbitrary C2 cell whose unsampled normal/curvature can vary between those points.
Adaptive subdivision checks midpoint deviation and sampled normal changes, but it does not emit
or validate the required supremum derivative bound. Consequently I cannot safely prove that
SurfaceValidator's cover already subsumes the full Builder `snapshot_resolution` inset for every
planner curve. Per the execution specification, this is a C1 proof blocker: no product
behavior change is authorized.

The fixed inset may be over-conservative in the representative knot, but removing or reducing it
without the missing derivative/curvature certificate would weaken fail-closed safety. The
existing Builder/Validator/Filter responsibilities and all margin values therefore remain
unchanged.

## Scope and changed files

The attempted zero-inset experiment was reverted. Final product behavior changes in this pass:

```text
NONE
```

The only new stage artifact is this self-audit. No planner/Kino/B-spline, safe-distance, SDF
map, Filter mathematics, Runtime/MatchedAdapter semantics, margin numeric, launch/config/map,
message, swarm, or CMake file was changed. The whitelist product sources remain at their
pre-stage semantics (`continuous_inset=snapshot_resolution` on the cloud Builder).

## Verification

The focused tests were rebuilt and passed against the restored semantics:

```text
phase_offset_tube_builder_test              7/7 PASS
phase_offset_tube_surface_validator_test    9/9 PASS
phase_offset_clearance_audit_test          22/22 PASS
phase_offset_tube_cross_section_test        7/7 PASS
phase_offset_tube_filter_test              15/15 PASS
phase_offset_runtime_test                  33/33 PASS
phase_offset_matched_adapter_test          65/65 PASS
cmake --build build -j2                        PASS
git diff --check                               PASS
```

The existing non-whitelist `TubeEpochManagerTest` has nine stale assertions that encode the old
interval/ownership expectations and were not modified:

```text
MaterialBoundChangeAdvancesActiveEpochExactlyOnce
MaterialInstallCommitsOnlyOnce
CloudClearancePathHasNoLegacyQueryInput
CloudOneSidedCandidateContainingRetainedDeltaCannotRescueCenterline
InsetExcludedZeroWithoutContinuousCertificateWaitsFailClosed
CloudGeometricContainmentStaysSeparateFromInteriorMargin
CloudExcludingZeroCandidateDoesNotInstallOrDenyCertificate
CloudExcludingZeroCandidatePreservesPriorActiveProfileAndEpoch
CloudMaterialEnvironmentChangeInstallsButEquivalentRefreshDoesNot
```

They are outside this stage whitelist and are recorded as a test-scope blocker only; no product
change was made to satisfy them.

## Dynamic evidence boundary

All ROS runs used task-owned loopback masters/private `ROS_HOME`; no user master/process was
queried or touched.

Observe-only baseline `/tmp/a5_generic_dynamic_20260818_r8` (master `127.0.0.1:11786`, checked-in
launch/map, goal `(8,0,1)`) produced 185/215/185 epoch/raw/diagnostics lines, zero-containing
early raw knots, C2 planner path replacements, and `[GVF][POINT_GOAL][REACHED] distance=0.198`.
This is baseline/observe-only evidence, not active certified Tube acceptance and not evidence
for changing the inset.

Active ESDF attempt `/tmp/a5_generic_dynamic_20260818_r9_active` (master `127.0.0.1:11787`,
`phase_offset_manual_observe_only=false`) produced no certified/active epoch or active-path
messages and persistent `GOVERNOR_INVALID_HOLD reason=guidance_invalid`; no H2 handoff or goal
was claimed. The known target-knot Validator evidence remains observed clearance
`0.526382393 m` below requested `1.368076983 m`; this is a real fail-closed clearance deficit,
not a justification to weaken the unresolved cover proof.

## Disposition and stop boundary

```text
C0 first-false reproduction: PASS
C1 exact geometry/adaptive-cell/SurfaceValidator formal cover proof: NOT_PASS (missing bound)
C2 generic inset correction: NOT_RUN (blocked by C1)
Product behavior: FAIL-CLOSED PRESERVED
Active ESDF/H2 dynamic acceptance: NOT_PASS (no certified epoch; expected stop)
```

This stage stops at the C1 proof boundary. A future execution specification must first add a
verifiable path/normal/curvature cell-error certificate (or otherwise prove the existing cover
bound) before changing `continuous_inset`. Do not enter A7.
