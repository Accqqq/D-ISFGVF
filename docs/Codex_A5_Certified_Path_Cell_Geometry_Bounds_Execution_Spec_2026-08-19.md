# A5 certified path-cell geometry bounds — execution specification

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5_CERTIFIED_PATH_CELL_GEOMETRY_BOUNDS
IMPLEMENTATION_AUTHORIZED=true
USER_AUTHORIZATION=explicit Terra/xhigh delegation on 2026-08-19
AUTO_ADVANCE_WITHIN_THIS_SPEC=true
AUTO_ADVANCE_TO_A7=false
PLANNER_OPTIMIZER_CHANGE_ALLOWED=false
FILTER_MATH_CHANGE_ALLOWED=false
MARGIN_NUMERIC_TUNING_ALLOWED=false
```

日期：2026-08-19  
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. Objective

解决上一阶段的 C1 proof blocker：当前 `PathStateQuery` 只提供点值
`p,p_w,p_ww`，无法证明 Tube cell 内任意未采样点的曲线、法向和曲率变化上界；因此
不能安全地用经验 3×3 cover 替换固定 `snapshot_resolution` inset。

本阶段为 `ContinuousPhasePath` 增加可验证的区间几何证书，并将它通过纯查询接口传给
generic TubeBuilder/SurfaceValidator。最终仍只有一个通用：

```text
I(w) = [delta_lower(w), delta_upper(w)]
r(w,delta) = p(w) + N(w)delta
```

`delta=0` 和非零 delta 使用同一证书；不得增加单机特判。

## 2. Fixed prior evidence

权威前序报告：

```text
docs/Codex_A5_Clearance_Accounting_Knot_Self_Audit_2026-08-18.md
docs/Codex_A5_Generic_OffsetTube_Continuous_Cover_Self_Audit_2026-08-18.md
```

固定 first-false：

```text
w=2.431715365
pre-inset       [-0.08125, 1.62500] contains zero
fixed inset      0.10000 m
post-inset      [ 0.01875, 1.52500] excludes zero
```

固定 blocker：当前 3×3 sampled cover 与经验系数 `1.1` 没有 cell-wise supremum derivative
certificate，不能单独承担完整 continuous cover。

## 3. Required proof contract

### B0 — generic cell certificate

设计一个纯数学、无 ROS 的不可变 cell 证书。名称可调整，但至少必须表达：

```text
w0, w1
path-domain/segment identity
certificate valid/complete
inf ||p_w|| over [w0,w1]
sup ||p_ww|| over [w0,w1]
sup ||N_w|| over [w0,w1], or enough facts to derive it safely
sup |curvature| and curvature/normal variation bound needed by the consumer
sup position/chord deviation or a derivable conservative bound
```

All finite/ordering/positive-speed invariants must be explicit. Unknown, segment crossing,
stationary derivative, unsupported evaluator or incomplete bound must return no certificate and
remain fail-closed.

### B1 — path-specific certified producers

`ContinuousPhasePath::Segment` must optionally own a cell-bound evaluator bound to the same
immutable segment as its point evaluator.

At minimum audit and, where safely possible, implement certified bounds for:

1. quintic Hermite C2 connectors using their stored polynomial coefficients and interval-safe
   derivative bounds;
2. mapped B-spline segments using the immutable spline/derivative control points and the
   arclength-map contract;
3. slices/concatenations without crossing source segment boundaries;
4. analytic circle/figure-eight fixtures, or explicit unsupported fail-closed status.

Dense sampling is a regression oracle only; it is not the proof itself. Bounds must arise from
analytic/convex-hull/interval facts belonging to the immutable path representation.

If mapped B-spline arclength inversion cannot yield a rigorous w-domain certificate within this
scope, do not approximate it as certified; report the exact remaining bound.

### B2 — navigation query boundary

Add a pure query type parallel to `PathStateQuery`, for example:

```text
PathCellBoundQuery(w0,w1) -> PathCellGeometryCertificate
```

`phase_offset_navigation` remains independent of concrete B-spline/ROS types. The thin matched
adapter converts `ContinuousPhasePath` ownership into both point and cell-bound queries; both
queries must capture the same immutable owner.

### B3 — generic Tube consumption

TubeBuilder/SurfaceValidator may replace the fixed full-resolution inset only on a cell with a
valid complete certificate. The implemented rule must state and test the conservative error
inequality used for:

```text
p(w) variation
N(w)*delta variation
PWL delta-bound variation
voxel/discretization residual not already charged by the validator
```

Requirements:

- no certificate: preserve existing fixed inset/fail-closed behavior;
- certificate available: compute a nonnegative cell-local residual inset/cover;
- do not subtract a discretization term twice;
- exact PWL Filter mathematics and slope parameter remain unchanged;
- SurfaceValidator must validate the same final generic surface and retain unknown/out-of-map/
  query-limit fail-closed behavior;
- zero remains only when the common certificate proves it, not through a special branch.

## 4. Mandatory execution order

### T0 — provenance and baseline

Record HEAD, existing dirty worktree, related Build-IDs, current focused tests and prior dynamic
evidence. Do not attach to or stop the user ROS master/processes.

### T1 — proof design before product behavior

Write the exact certificate inequalities and path-producer derivations in the self-audit or an
allowed supporting comment/test. Product Tube behavior must remain unchanged until at least the
quintic/mapped-B-spline path types used by the active episode have a valid certificate.

### T2 — conditional implementation

Implement only after T1 passes. If any required executed segment lacks proof, either:

- keep the old full inset on that segment/cell; or
- stop and report without claiming active acceptance.

Do not globally set inset to zero and do not use an empirical dense-sampling margin as proof.

### T3 — verification

Required regressions include:

1. every certified bound conservatively contains dense adversarial evaluations;
2. certificate request crossing two path segments fails or is split exactly;
3. low-speed/stationary/invalid/unsupported paths fail closed;
4. straight line produces zero geometric curve penalty;
5. curved quintic/B-spline produces finite conservative bounds;
6. Tube zero/nonzero delta use the same generic interval and proof;
7. missing certificate preserves the old fixed inset;
8. real between-knot obstacle and unknown/out-of-map remain rejected;
9. no planner/Filter/Runtime/A6 authority regression.

If implementation succeeds, run task-owned observe-only baseline and active
`phase_offset_manual_observe_only=false` ESDF episodes with the checked-in launch, original
`pillar.pcd`, and one `(8,0,1)` goal. Active PASS requires Certified/Active epoch, no semantic
owner gap, no persistent `GOVERNOR_INVALID_HOLD`, and normal goal arrival. Baseline-only arrival
is not active PASS.

## 5. File whitelist

```text
docs/Codex_A5_Certified_Path_Cell_Geometry_Bounds_Execution_Spec_2026-08-19.md
docs/Codex_A5_Certified_Path_Cell_Geometry_Bounds_Self_Audit_2026-08-19.md

src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/path_state.h
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/geometry_types.h
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/geometry.h
src/swarm_planner/phase_offset/phase_offset_core/src/geometry.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/geometry_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/path_state_query.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/continuous_phase_path.h
src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp
src/swarm_planner/bspline_traj/test/continuous_phase_path_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp
```

Do not modify CMake, UniformBspline implementation, planner/Kino/B-spline optimizer sources,
SDF map, launch/config/map, Filter source, Runtime/MatchedAdapter control policy, ROS messages,
swarm or A6 authority semantics. If a proof requires an out-of-whitelist edit, stop and report.

## 6. Verification and stop boundary

Run affected unit/integration tests, completed-stage regressions, full build, dependency-boundary
searches, `git diff --check`, final status and whitelist audit. The self-audit must include:

```text
exact certificate formulas
supported/unsupported path segment types
before/after inset/interval evidence
test and dynamic evidence
changed files
remaining proof gaps
PASS/NOT_PASS
```

Stop after this A5 proof/correction stage. Do not advance to A7.
