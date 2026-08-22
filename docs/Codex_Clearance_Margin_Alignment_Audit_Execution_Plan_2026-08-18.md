# Planner--tube clearance and margin alignment execution plan

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=CLEARANCE_MARGIN_ALIGNMENT_AUDIT
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=true
USER_AUTHORIZATION=explicit continuous Terra/xhigh delegation
PLANNER_PARAMETER_TUNING_ALLOWED=false
FILTER_SEMANTIC_CHANGE_ALLOWED=false
RUNTIME_GATE_OR_LATCH_CHANGE_ALLOWED=false
SWARM_NONZERO_OFFSET_INTEGRATION_ALLOWED=false
```

日期：2026-08-18
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. 目标与不变语义

本阶段处理一个明确的语义问题：planner 输出的路径 `p(w)` 是单机名义中心线，tube
是围绕它生成的允许偏移集合

```text
r(w, delta) = p(w) + N(w) * delta
```

单机正常模式固定 `delta_ref = 0`；集群意图产生的非零 offset 只能在 tube 区间内被
连续投影。不得通过非零 offset 营救一个 `delta=0` 已经不安全的中心线。

本阶段首先冻结 planner 路径、planner `safe_distance`、exact PWL Filter、C2、速度、
governor、Runtime 状态机和所有现有参数。若同一 planner 路径、同一 immutable map
snapshot 下 raw 横截面不包含零，必须先区分真实物理不足与重复/错配裕度；不能用调大
planner 距离或放宽 tube 参数掩盖问题。

## 2. 审计问题

对同一条路径、同一个 map snapshot、同一个 `current_w`，逐点建立以下事实表：

```text
planner clearance 与 planner safe_distance 的定义
raw occupancy/ESDF 的实际 clearance 与是否已膨胀
UAV radius
map uncertainty 与 snapshot.included_map_inflation
localization uncertainty
tracking error bound
continuous inset / snapshot resolution
SurfaceValidator required_clearance 与 cover_radius
raw interval、curvature intersection、Filter 输入/输出
zero 是否进入 raw interval、Filter、Validator
```

每一项必须标注单位、数据来源、是否已经包含在上游数据中，以及在最终不等式中扣除
或增加的位置。`preincluded_map_uncertainty` 只能从 map uncertainty 中扣除一次；
`continuous_inset` 与 `cover_radius` 若证明同一离散化误差，不能重复收费。

## 3. 执行顺序

### T0：基线与审计 seam

1. 保存并报告工作区 `git status --short`、当前相关文件 diff 和测试基线。
2. 复用现有 clearance-audit seam（若其字段或调用位置不足，做最小扩展），使一次
   audit 能关联同一 path revision、snapshot sequence/stamp/resolution、preferred/retained
   delta、current phase 与 tube revision。
3. 审计输出必须能逐点区分：planner clearance、raw clearance、已包含的 map inflation、
   full/residual effective radius、continuous inset、validator cover radius，以及
   `contains_zero`。不得只输出一个汇总 margin。

### T1：证据判定

对每个 raw knot 和同一 snapshot 的 path point，按以下顺序判定：

1. 如果 raw environment interval 在 inset 后包含零，Filter 必须保留零；若不保留，视为
   Filter/调用链回归并修复测试或实现，但不改变 exact PWL 数学约束。
2. 如果 raw interval 不包含零，先检查是否由 UAV 半径、map inflation、uncertainty、
   inset 或 validator cover 重复计算造成。
3. 只有当障碍物到中心线的真实物理距离确实小于 `UAV radius + 真实误差界` 时，才将
   该路径标记为真实不可行；此时保留 fail-closed 行为并记录 planner 重规划需求。

### T2：最小实现修正

只对 T1 证明的重复扣减或口径错配实施代码修正。优先顺序：

1. 修正 planner/tube clearance 的单位与 backing 语义映射；
2. 修正 `preincluded_map_uncertainty`、snapshot inflation、continuous inset、
   validator cover 的单次扣除关系；
3. 保持 `RobustTubeMargins::fullEffectiveRadius()` 与 residual radius 的物理含义
   一致，并保留完整诊断字段；
4. 不修改 planner `safe_distance`、Filter 的 exact PWL 规则、Runtime 的安全拒绝、
   速度/限幅或 swarm offset 逻辑。

### T3：单机语义验收

1. 单机正常模式使用 `delta_ref=0`，完成从 planner、tube、Runtime 到 PositionCommand
   的闭环验证。
2. 在一个中心线真实安全、一个中心线真实不足、一个不确定/未观测的场景中，分别证明：
   - 安全中心线的 raw interval 全程包含零并被 Filter/Validator 保留；
   - 真实不足时 fail-closed，且不会用非零 offset 营救；
   - unknown/out-of-map 不被当作 free。
3. manual 正弦 offset 仅作为几何/控制测试，不得作为单机正常语义的通过依据。

## 4. 允许修改的文件白名单

仓库内只允许修改下列文件；如证据要求新增文件，必须先在本执行单中追加并说明理由：

```text
docs/Codex_Clearance_Margin_Alignment_Audit_Execution_Plan_2026-08-18.md
docs/Codex_Clearance_Margin_Alignment_Audit_Self_Audit_2026-08-18.md

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp
src/swarm_planner/bspline_traj/CMakeLists.txt

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

Launch/config、planner、Filter、Runtime、messages、diagnostics schema、map generator 和
swarm 文件不在白名单内；本阶段不得修改。

## 5. 验收门槛

- 现有 planner 路径和 `safe_distance` 未改变；exact PWL Filter 数学与参数未改变。
- 审计记录能证明同一 snapshot 下每个 margin 的来源和是否已被包含。
- `preincluded_map_uncertainty` 不会使 residual radius 小于其物理 backing 所允许的值，
  也不会再次扣除已经进入 snapshot occupied geometry 的 inflation。
- 安全中心线的 raw interval 满足 `lower <= 0 <= upper`；常数零函数通过 Filter 的
  Lipschitz/PWL 可行性测试。
- 真实物理不足、不确定、出界仍 fail-closed，并提供可区分的 reason。
- 单机正常 Runtime 的 retained/reference delta 保持零；非零 offset 仅在明确的 swarm
  intent 测试中出现，且不改变基础路径安全性。
- 受影响目标 clean/incremental build、专项单测、已完成阶段回归、依赖边界搜索、
  `git diff --check` 全部通过；白名单外既有失败必须单独报告，不得顺手修复。

## 6. 自动推进与停止规则

用户已明确授权 Terra/xhigh 连续调度，因此本执行单 `AUTO_ADVANCE=true`：T0 通过后自动
推进 T1/T2/T3，不等待额外确认。连续推进不等于跳过证据门槛：若发现需要修改白名单外
文件、planner/Filter/Runtime 语义、或外部 ROS 状态无法安全隔离，则保留已完成证据并
停止报告阻塞，不扩大范围。

完成后必须写入对应 self-audit，列出实际修改、证据路径、测试结果、未通过项与剩余风险。
