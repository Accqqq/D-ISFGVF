# A5 certified path-cell geometry bounds — self-audit (2026-08-19)

```text
DOCUMENT_ROLE=PRIVATE_A5_CERTIFIED_PATH_CELL_GEOMETRY_BOUNDS_SELF_AUDIT
STAGE=A5_CERTIFIED_PATH_CELL_GEOMETRY_BOUNDS
AUTO_ADVANCE_TO_A7=false
PRODUCT_BEHAVIOR_CHANGE=CONDITIONAL_CERTIFIED_CELL_ONLY
```

## T0 provenance and scope

The workspace already contained user-owned dirty and untracked prototype files;
they were not reset, staged, or cleaned.  This pass changed only the A5
whitelist sources/tests and this audit.  No planner/Kino/UniformBspline
implementation, SDF map, Filter mathematics, Runtime/MatchedAdapter control
policy, margin, launch/config/map, CMake, message, swarm, or A6 authority file
was changed.

The navigation boundary now carries two pure queries captured from one
immutable owner:

```text
PathStateQuery(w) -> PathDifferentialState
PathCellBoundQuery(w0,w1) -> PathCellGeometryCertificate
```

`ContinuousPhasePath::cellBounds` rejects a non-finite/reversed cell, a cell
crossing two stored segments, an unsupported evaluator, a stationary/low-speed
derivation, or any incomplete/non-finite bound.  Point and cell callbacks in
the matched adapter capture the same `shared_ptr<const ContinuousPhasePath>`;
there is no independent sampled-path certificate fallback.

Certificate production is strictly optional metadata on the evaluator.  In
particular, a mapped B-spline of insufficient degree can retain its original
point evaluator while exposing no cell-bound callback; navigation then keeps
the fixed inset.  A failed cell proof never invalidates an otherwise valid
point evaluator.

If the strict executable Hermite arclength map itself cannot be built (for
example, stationary raw endpoints), the point evaluator uses the legacy linear
`t(w)` mapped B-spline evaluator and exposes no certificate.  This preserves
planner/frontend continuity while remaining fail-closed for cell geometry;
the fallback is never advertised as an arclength certificate.

## T1 exact certificate derivations

For a closed phase cell of width `h = w1-w0`, the certificate records finite
positive `inf ||p_w||` and `inf ||e_z x p_w||`, finite `sup ||p_w||`,
`sup ||p_ww||`, `sup ||p_www||`, `sup ||N_w||`, `sup |kappa|`, normal and
curvature variation bounds, midpoint position variation, and chord deviation.
All scalar outputs use outward floating-point guards.  The generic facts are:

```text
sup ||N_w|| <= sup ||p_ww,xy|| / inf ||p_w,xy||
sup |kappa| <= sup ||p_ww,xy|| / inf ||p_w,xy||^2
|kappa'| <= sup ||p_www,xy|| / inf ||p_w,xy||^2
             + 3 sup ||p_ww,xy||^2 / inf ||p_w,xy||^3
||p(w)-p(w_mid)|| <= sup ||p_w|| h/2
distance to endpoint chord <= sup ||p_ww|| h^2/8
```

### Quintic Hermite C2

The stored six vector coefficients are differentiated in phase, restricted to
the requested subinterval, and converted from power basis to Bernstein basis.
The Bernstein convex hull supplies derivative suprema.  A ball containing the
Bernstein control polygon supplies a conservative positive lower norm; if the
lower bound is not strictly positive, the producer returns no certificate.
The same polynomial coefficient object is captured by both the point evaluator
and its cell-bound evaluator.

### Mapped B-spline

The producer uses the exact representation implemented by
`makeMappedBspline`: the deterministic quintic-Hermite `S(t)` table/cells and
its monotone inverse used by that evaluator.  It does **not** claim that the
Simpson table is an exact or interval-certified integral of the real
`||p_t||`; no such claim is needed or made.

For every map cell intersecting the requested `S` range, positive Bernstein
controls of `S'` give `q_min <= S'(t) <= q_max`; Bernstein controls of `S''`
and `S'''` give `q'_max` and `q''_max`.  Local immutable B-spline derivative
control polygons give bounds on `p_t`, `p_tt`, and `p_ttt`.  The chain rule is
bounded interval-wise:

```text
dt/dw       = A/q
d2t/dw2     = -q' A^2/q^3
d3t/dw3     = A^3 (3 q'^2/q^5 - q''/q^4)
p_w         = p_t dt/dw
p_ww        = p_tt (dt/dw)^2 + p_t d2t/dw2
p_www       = p_ttt (dt/dw)^3 + 3 p_tt dt/dw d2t/dw2 + p_t d3t/dw3
```

Any missing high-order spline derivative, non-positive Bernstein `S'`,
low-speed bound, or invalid map interval fails closed.  Dense sweeps in the
tests are regression oracles only.

## T2 consumer rule and accounting

When every Builder cell in the actual preview has a complete certificate and
the certificate's offset regularity/active-speed preflight succeeds, Builder
records `cell_geometry_certified=true` and applies a proven nonnegative local
inset of exactly zero.  If that eligibility preflight fails, Builder restores
the historical `snapshot_resolution` inset before Filter/Validator; the
certificate layer cannot turn a viable old candidate into a navigation denial.
This is not a special delta-zero branch: the same generic interval `I(w)` is
used for zero and nonzero delta.
The zero local erosion is safe because the certified SurfaceValidator charges
the complete continuous-cover error once.  If any cell lacks a certificate,
the entire Builder preview retains the historical fixed
`snapshot_resolution` inset.

For a certified surface cell, with `D=max |delta|`, PWL boundary slope `L`,
PWL width `W`, and `v` span `dv`, the Validator's cover radius is:

```text
R = midpoint_position_variation
    + (D * normal_variation_bound)/2
    + (L*h)/2
    + (W*dv)/2
    + snapshot_resolution/2
```

The first two terms cover `p` and `N*delta`; the next two cover PWL delta
variation in phase and cross-section; the final term is the one voxel residual
not charged by Builder.  Thus no discretization term is charged twice.  Before
using this smaller cover, Validator also proves the same generic surface is
well-defined throughout the cell:

```text
1 - sup_abs_curvature * D >= regularity_margin
inf ||p_w|| - sup ||N_w|| * D > tangent_epsilon
```

If a profile declares certified cells but any cell query or either invariant
fails, that cell fails closed; it does not fall back to empirical 3x3 cover.
Profiles without a certificate preserve the old fixed-inset/3x3 behavior.

## Verification

### Changed whitelist files

```text
docs/Codex_A5_Certified_Path_Cell_Geometry_Bounds_Self_Audit_2026-08-19.md
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/path_state.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/path_state_query.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/continuous_phase_path.h
src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp
src/swarm_planner/bspline_traj/test/continuous_phase_path_test.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

Focused build and tests run after the implementation:

```text
cmake --build build --target \
  continuous_phase_path_test phase_offset_tube_builder_test \
  phase_offset_tube_surface_validator_test phase_offset_tube_epoch_manager_test \
  phase_offset_matched_adapter_test -j2                         PASS
continuous_phase_path_test                                     10/10 PASS
phase_offset_tube_builder_test                                  9/9 PASS
phase_offset_tube_surface_validator_test                       11/11 PASS
phase_offset_tube_epoch_manager_test                            51/51 PASS
phase_offset_matched_adapter_test                               65/65 PASS
```

The added regressions cover adversarial dense containment for quintic and
mapped-B-spline bounds, unsupported/cross-segment fail-closed behavior,
zero-local-inset straight cells, missing-certificate legacy inset retention,
and certified regularity/offset-speed rejection without sampled fallback.

The owner-wiring audit is structural and executable: the adapter's point and
cell lambdas both capture the same `semantic_path_owner`, while
`ContinuousPhasePath::cellBounds` stamps the certificate with the selected
immutable segment identity.  A synthetic point-only owner intentionally has
no cell callback and therefore remains on the legacy fixed-inset path.

```text
cmake --build build -j2                                      PASS
git diff --check                                              PASS
```

## Dynamic verification

The final binaries were exercised with the task-owned master
`ROS_MASTER_URI=http://127.0.0.1:11789`, the checked-in
`phase_offset_esdf_tube_single.launch`, the original `pillar.pcd`, and the
single goal `(8,0,1)`.  The old task-owned episode was stopped before these
runs; no user master or user process was attached to or terminated.

Observe-only episode (`phase_offset_manual_observe_only:=true`):

```text
bag: /tmp/a5_certified_ros_home_20260819/evidence/observe_final.bag
duration: 48.0 s, 3655 messages
/tube_candidate: 33
/tube: 33
/active_path: 33
/tube_epoch_diagnostics: 28
/move_base_simple/goal: 1
goal result: [GVF][POINT_GOAL][REACHED] distance=0.185
```

This is a valid single-UAV navigation/Tube observe-only PASS.  It is not an
active Tube acceptance claim.

Active episode (`phase_offset_manual_observe_only:=false`):

```text
bag: /tmp/a5_certified_ros_home_20260819/evidence/active_final.bag
duration: 38.5 s, 4838 messages
/tube_candidate: 190
/tube: 190
/active_path: 190
/tube_epoch_diagnostics: 1
first epoch diagnostic: active_profile_complete=1,
  active_obstacle_certified=1, active_display_certified=1,
  certificate_denied=0, current_state_admissible=1
terminal result: H2 terminal nonzero handoff denial latched;
  persistent GOVERNOR_INVALID_HOLD (all_candidates_path_end_clamped);
  no normal goal arrival
```

Thus the certificate/Builder path reaches a certified active profile in the
active run, but the required H2 terminal handoff and normal arrival do not
pass.  This is recorded as an active acceptance NOT_PASS at the A6/H2
handoff boundary; no planner, Filter, margin, velocity, or Tube authority
change is made here to mask it.

## Disposition and A7 stop boundary

```text
C1 analytic/convex-hull path-cell certificate: PASS for quintic Hermite and
    implemented mapped-B-spline S(t) evaluator representation
C1 unsupported/low-speed/cross-segment fail-closed: PASS
C1 generic surface regularity and active-speed interior proof: PASS
C2 fixed-inset replacement: CONDITIONAL; zero only with complete certificate
Static focused regressions: PASS
ROS observe-only dynamic acceptance: PASS (goal reached; evidence above)
ROS active dynamic acceptance: NOT_PASS (terminal H2 nonzero handoff denial;
    persistent GOVERNOR_INVALID_HOLD; no goal arrival)
Overall A5 active acceptance: NOT_PASS until the required task-owned baseline
    and active ESDF episodes satisfy the H2 handoff and normal-arrival clauses
```

Stop here.  Do not enter A7 or change planner/filter/runtime authority.
