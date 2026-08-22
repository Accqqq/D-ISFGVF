# Zero-centerline tube installability execution plan

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=ZERO_CENTERLINE_INSTALLABILITY
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=true
USER_AUTHORIZATION=explicit continuous Terra/xhigh delegation
PLANNER_KINO_BSPLINE_CHANGE_ALLOWED=false
FILTER_CHANGE_ALLOWED=false
MARGIN_PARAMETER_TUNING_ALLOWED=false
SWARM_OFFSET_INTEGRATION_ALLOWED=false
```

日期：2026-08-18
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. 固定语义

planner/KinoA*/B-spline 输出的 `p(w)` 是 tube 的名义中心线：

```text
r(w, δ) = p(w) + N(w)δ
```

单机正常执行固定 `δ_ref=0`。集群非零 retained/swarm offset 只能在以该中心线为基础
的安全 tube 内使用，不能营救一个 `δ=0` 已经不安全的 planner 路径。

本阶段不修改 planner 路径生成、`planning/safe_distance`、ESDF/KinoA*/B-spline、
exact PWL Filter、Runtime gate/latch、速度/限幅、map inflation、margin 数值或 swarm
offset。

## 2. 执行顺序

### T0：真实 build 证据

对同一个 path revision、同一个 immutable cloud snapshot、同一个 tube build，记录每个
实际 raw knot：

```text
path.p / N / w
pre-inset obstacle_lower/upper
continuous_inset
post-inset raw_lower/upper
Filter input/output bounds
Validator cover/requested radius
contains_zero at every stage
source/tube revision and snapshot provenance
```

不能用现有 49/83-field ROS diagnostics 反推 pre-inset；需要复用已有 audit seam 或
sidecar/in-memory evidence，不能伪造 planner/tube 同源 snapshot。

### T1：证据判定

1. pre-inset 不包含零：中心线在当前 robust clearance 定义下真实不足，保持 fail-closed；
   不允许用正/负 retained offset 营救。
2. pre-inset 包含零、post-inset 排除零：这是候选的 inset/连续证明问题；不得直接删 inset
   或直接制造 `{0}`，先实现并验证独立的 zero-line 连续 clearance/cover 证明。
3. post-inset 包含零、Filter 排除零：认定为 Filter 回归；保持 exact PWL 数学并修复测试/调用
   链，不调斜率或边界参数。
4. Filter 包含零、Validator 截掉零：修 Validator 的连续 ribbon 证明或证据映射；不能放宽
   required clearance/cover。

### T2：条件性实现

只有 T1 证明实现错误时才修改对应层：

- Builder 可增加 pre/post-inset 与 centerline zero facts；
- 若需要保留单机零中心线，必须有沿 `p(w)` 的连续 clearance/cover certificate；
- 证书不足时保留 fail-closed，不用退化 `{0}` 假装通过；
- Candidate 安装必须要求现有中心线安全事实，不得仅凭 retained delta 在 interval 内就
  ROLLING。

## 3. 文件白名单

```text
docs/Codex_Zero_Centerline_Installability_Execution_Plan_2026-08-18.md
docs/Codex_Zero_Centerline_Installability_Self_Audit_2026-08-18.md

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp
src/swarm_planner/bspline_traj/CMakeLists.txt
```

`tube_filter.cpp`, planner/KinoA*/B-spline sources, launch/config, map snapshot producer,
Runtime source, messages and swarm modules are explicitly out of scope.

## 4. 验收矩阵

- pre-inset 含零且 post-inset 含零：Filter/Validator 保留零；单机 retained/reference 为零。
- pre-inset 含零但 post-inset 排零：有明确 zero-line certificate 才能通过，否则 fail-closed。
- pre-inset 本来排零：记录真实 clearance deficit；非零 retained 不得安装营救。
- unknown/out-of-map：fail-closed。
- retained=0、一侧 tube：单机等待/拒绝，不输出非零营救。
- retained≠0 且 zero 不安全：Candidate 不得 ROLLING。
- 同 path revision、同 snapshot、同 tube revision 的 provenance 必须一致。
- 专项单测、既有 Filter/Runtime/Adapter 回归、完整 build、`git diff --check` 和白名单自审通过。

## 5. 自动推进与停止规则

用户已授权 Terra/xhigh 连续推进，因此 T0→T2 自动执行。若证据不足以证明 zero-line
连续安全，必须保持 fail-closed 并在 self-audit 中报告，不得通过删 inset、放宽 margin、
修改 Filter 或增加 swarm offset 绕过证明缺口。
