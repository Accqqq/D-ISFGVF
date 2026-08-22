# A5 actual future-knot clearance accounting — execution specification

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5_ACTUAL_KNOT_CLEARANCE_ACCOUNTING
IMPLEMENTATION_AUTHORIZED=true
USER_AUTHORIZATION=explicit Terra/xhigh delegation on 2026-08-18
AUTO_ADVANCE_WITHIN_THIS_SPEC=true
AUTO_ADVANCE_TO_A7=false
PLANNER_PATH_OR_PARAMETER_CHANGE_ALLOWED=false
FILTER_RULE_CHANGE_ALLOWED=false
MARGIN_NUMERIC_TUNING_ALLOWED=false
```

日期：2026-08-18  
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. Objective

解释 A5 dynamic audit 中实际失败的 future knot：

```text
w = 2.431715365
raw interval      = [ 0.018750000, 1.525000000 ]
filtered interval = [ 0.022804744, 1.520945256 ]
```

必须回答这条 planner 中心线为何在 Tube robust contract 下排除 `delta=0`：

1. planner 的 ESDF/occupancy 是否已经对 UAV 外壳或其它裕度做了膨胀；
2. Tube raw pre-inset、post-inset、Filter、SurfaceValidator 是否各自只扣除一次物理项；
3. 失败是真实中心线 clearance deficit，还是 planner/Tube 语义错配/重复扣除。

本阶段只解决 accounting 归因。若结论是 planner 路径真实不足，保持 fail-closed 并输出
下一阶段 planner-path execution request；不得在本阶段调参数。

## 2. Fixed evidence and invariants

固定名义中心线语义：

```text
p(w) = planner final C2 path centerline
r(w, delta) = p(w) + N(w) * delta
single-UAV normal reference: delta_ref = 0
```

不得用非零 offset 营救 `delta=0` 不安全的中心线；不得删除 continuous inset、cover、
tracking 或其它安全项来制造零区间。

同一审计行必须绑定：

```text
path source revision
tube/source revision
immutable snapshot sequence/stamp/resolution
current_w
path p/N
```

## 3. Mandatory execution order

### K0 — provenance and read-only baseline

1. 记录 HEAD、既有 dirty worktree、相关 binary Build-ID 和当前 self-audit。
2. 复用已有 `phase_offset_clearance_audit` sidecar；不得通过 legacy ROS payload 猜测
   pre-inset 值。
3. 必要的 ROS 复现只能使用 task-owned loopback master/ROS_HOME；不得接触用户 master
   或进程。
4. 不改 planner、launch、map、Filter、Runtime 或 margin 数值。

### K1 — same-path/snapshot pointwise table

对 `w=2.431715365` 及其前后相邻 knot 记录：

```text
planner point p/N and planner clearance query/status
planner safe_distance definition and units
whether planner ESDF/occupancy is inflated, by which radius/layer
raw pre-inset lower/upper and contains_zero
continuous_inset and snapshot resolution
post-inset raw lower/upper and contains_zero
Filter input/output bounds and contains_zero
SurfaceValidator cover_radius/requested_clearance/observed clearance
UAV radius
map uncertainty and preincluded map uncertainty
localization uncertainty
tracking error bound
full effective radius and residual effective radius
snapshot included inflation/layer mask
```

Every value must state whether it is already included upstream and where it enters the final
inequality. Do not equate `snapshot included inflation` with map uncertainty without explicit
provenance.

### K2 — classification

Classify the first false layer:

```text
INSET_ZERO_EXCLUSION
  pre-inset contains zero, post-inset excludes zero

PLANNER_TUBE_ROBUST_CONTRACT_MISMATCH
  pre-inset excludes zero, planner query says safe under its own contract,
  and same immutable snapshot/path identity is proven

REAL_CENTERLINE_CLEARANCE_DEFICIT
  pre-inset excludes zero and no duplicate/semantic mismatch is proven

FILTER_ZERO_EXCLUSION
  pre/post-inset contain zero but Filter output excludes it

VALIDATOR_ZERO_EXCLUSION
  Filter contains zero but continuous cover/validator is the first false

UNKNOWN
  provenance/query/layer identity is insufficient
```

The observed raw interval is not enough to claim a duplicate margin. The pre-inset stage and
planner query are mandatory.

### K3 — conditional repair only

Only the following behavior changes are authorized, and only after K2 proves the corresponding
implementation defect:

- evidence/profile field alignment in the clearance-audit seam;
- planner/Tube residual-radius mapping when an already-included term is demonstrably charged
  a second time;
- pre/post-inset or validator evidence mapping when the mathematical contract is unchanged.

Not authorized:

- lowering `safe_distance`, Tube margins, continuous inset, cover or tracking bound;
- increasing planner reachability by reducing clearance;
- deleting a failed zero certificate or manufacturing `{delta=0}`;
- changing planner/Kino/B-spline source, map inflation, launch, Runtime gates or Filter math.

If K2 is `REAL_CENTERLINE_CLEARANCE_DEFICIT`, make no product behavior change in this stage.

## 4. File whitelist

```text
docs/Codex_A5_Clearance_Accounting_Knot_Execution_Spec_2026-08-18.md
docs/Codex_A5_Clearance_Accounting_Knot_Self_Audit_2026-08-18.md

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
```

Do not modify CMake, planner files, SDF map implementation, launch/config/map, Runtime,
diagnostic schemas or swarm modules. If the evidence requires an out-of-whitelist change, stop
and report the exact scope boundary.

## 5. Required tests and acceptance

Run the affected audit/Tube tests, the completed-stage regressions, full build, dependency-boundary
searches, `git diff --check`, and whitelist audit.

At minimum preserve tests for:

1. same snapshot/path provenance;
2. pre-inset zero retained after inset;
3. pre-inset zero excluded without unproved repair;
4. duplicate snapshot-inflation evidence classification;
5. unknown/out-of-map remains fail-closed;
6. Filter/Validator zero exclusion categories remain distinct.

Write a self-audit containing the exact pointwise table, classification, any changed files,
tests, and remaining planner/action boundary. Do not advance to A7.
