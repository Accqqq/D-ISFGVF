# Tube V2 Proposal Alignment and Minimal Next-Stage Plan

```text
DOCUMENT_ROLE=READ_ONLY_ALIGNMENT_AND_FUTURE_EXECUTION_PLAN
DOCUMENT_STATUS=DRAFT_NOT_YET_AUTHORIZED_FOR_SOURCE_MUTATION
CURRENT_ACTIVE_STAGE=A6_P2
AUTO_ADVANCE=false
TUBE_V2_SOURCE_MUTATION_ALLOWED=false
```

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
对齐文档：
`/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Latest_Detailed_Proposal_2026-08-20.md`

## 1. 当前决定

最新 proposal 的核心修订可以借鉴，并且确实针对当前 Tube 过度保守与实现臃肿的
主要结构：

```text
geometric safety interval
!=
dynamic offset/phase viability
```

但不得混入当前 A6-P2。A6-P2 正在解决的是：

```text
bootstrap Pair COMMITTED
-> first command capture/selection
-> Runtime executed authority
-> later replan/H2 protection
```

Tube V2 可能扩大 offset-certified 可用窗口，但不能替代 Pair 生命周期正确性。

## 2. 当前代码已经与 proposal 对齐的部分

1. Production cloud cross-section 直接读取 `planning/safe_distance`，planner 与 Tube
   使用同一 clearance 数值。
2. `preincluded_map_uncertainty` 只作为已包含地图不确定性的显式证据，production
   geometry 不再把 UAV/map/localization/tracking 多组 margin 重复叠加到
   `planning/safe_distance` 上。
3. normal Tube 保持 zero-connected / zero-only planner baseline；neutral planner
   不被 Tube 变成导航 gate。
4. Runtime 已有 exact PWL cell、跨 knot 一步 containment、`U+ -> U_safe` 和有限
   refresh-horizon witness。它已经承担了相当一部分“动态可行性”职责。
5. SurfaceValidator 已能按 current-containing 连续 cell 截取，而不是要求整个请求
   preview 所有 cell 同时成功。

因此 Tube V2 不应再新建第二套 Builder、第二套 authority 或第二套 Runtime。

## 3. 当前代码尚未对齐的核心

`TubeFilter` 仍把固定 `boundary_slope_max=0.80` 当成 geometry certificate 的必要条件：

```text
raw samples
-> whole-range exact L-Lipschitz inner envelope
-> choose longest current-containing range
-> truncate when the envelope becomes empty
```

该算法虽然比旧 Hermite repair 正确，但仍会让一个远端窄点或快速边界变化通过全局
Lipschitz envelope 反向收缩其他位置，并把 phase-domain slope 当成几何 Tube 是否存在的
硬条件。这正是 proposal §11--§13 与 §37 T1--T4 要修正的对象。

## 4. proposal 数据的适用边界

proposal 中：

- raw forward first exclusion `0/46`；
- Filter forward first exclusion `45/46`；
- raw slope p90 lower/upper 约 `2.05/2.40`；

来自 A5R-0B frozen corpus，并在
`Codex_A5R_3_Failure_Category_Disposition_Execution_Plan_2026-08-12.md`
中有对应记录。它早于后续 A5F exact Lipschitz、Z1、M3、A6-P1/P1R 与当前 A6-P2。

这些数据足以支持“模型值得重审”，但不能直接写成当前代码的最新 cohort 结论。
进入 Tube V2 前必须按当时最新 main/worktree 重跑同口径 baseline。proposal 的
`first consume p95 ~= 38.887 ms` 也必须绑定原始统计文件与样本数；若只能找到论文文字，
则在新 baseline 前标记为 historical/unverified，不作为当前验收事实。

### 4.1 A6-P2 Run 4 的当前动态证据

A6-P2 完成后的隔离 Run 4 已给出一个与 proposal 核心区分直接一致的
当前例子：

```text
Pair 1 selected, retained delta 0 -> -0.000384455
-> transient Runtime certificate denial
   U+ and U_safe: step 4 joint-port feasible polygon empty
   fatal_control_failure=0
-> Pair 2 later selected, retained delta -0.005596105 -> -0.007772646
```

同次 run 的 raw current anchor `34/34` found/complete，而拒绝理由来自 Runtime
short-horizon exact-port witness，不是 Tube build 或 raw cross-section 失败。这不能替代
V2-0 cohort baseline，但它证明了当前系统中确实可以出现：

```text
geometric/current Tube evidence remains available
while a particular rolling port witness is dynamically unavailable
```

因此后续不得把这类 Runtime viability denial 回写成 geometry/Tube empty，也不得
为了消除短时 denial 而放宽 clearance、slope、速度或 H2 时序。

## 5. 最小实施顺序

### V2-0：冻结当前新基线，只读

必须在 A6-P2 完成后执行，不改产品源码：

- 重放 frozen pillar cohort；
- 分离 raw Builder、Filter、SurfaceValidator 三层 first truncation；
- 记录 raw/local PWL slope distribution，但 slope 只作 diagnostic；
- 记录 current anchor、certified forward horizon、Candidate/Active/Selected；
- 记录 request -> Runtime-ready -> first selected consume latency；
- 记录 margin provenance，证明 planner safe distance 未重复扣除。

只有新基线仍显示 fixed-slope Filter 是主要排除层，才进入 V2-G。

### V2-G：只简化 geometric Filter

本阶段只做 proposal T1/T2 的最小版本：

1. `boundary_slope_max` 从 Tube geometry validity gate 降为兼容性/诊断字段；
2. 不再计算 whole-preview global Lipschitz envelope；
3. 以 raw sampled intervals 构造 cell-local conservative PWL inner ribbon；
4. 每个 PWL endpoint 必须位于对应 raw interval 内；
5. 连续 cell 安全仍由现有 immutable-snapshot SurfaceValidator 证明；
6. validator 失败仍只保留包含 current anchor 的连续安全 segment；
7. true empty/unknown/occupied/out-of-map 与曲率 regularity 继续 fail-closed；
8. normal Tube 继续 zero-connected，不在本阶段引入 disconnected nonzero component；
9. planner clearance、lookahead、`min_certified_forward_w`、速度、timer、launch 参数全部冻结。

本阶段复用现有 Runtime exact-PWL witness 来回答“当前 command 动态上能否执行”，不同时
引入新的 backward envelope。这样可以先证明 geometry false negative 是否消失，避免一次
同时改 Filter、Runtime、状态机和 controller。

### V2-V：仅在证据需要时增加 preview viability

只有 V2-G 后出现以下事实才进入：geometric Tube 明显存在，但 Runtime 因未来快速交替
边界频繁在接近当前点时才拒绝，导致预览决策过晚。

届时新增一个独立、无 authority 的 backward interval 计算：

```text
K_H = I_H
K_k = I_k intersect expand(K_{k+1}, u_delta_max * delta_t_k)
```

要求：

- `interval_nonempty/current_delta_safe` 是 geometry facts；
- `preview_rolling_viable` 是 dynamic fact；
- viability failure 不删除 geometric Tube；
- 先用现有 slowdown/U_safe/replan 接口，不新增 emergency/recovery authority；
- 不用 backward envelope 替代 Runtime 的 exact step containment。

proposal 中 `centerline_safe=false && current_delta_safe=true` 的非零 component 保留属于
active recovery / homotopy 问题，不进入 V2-G/V2-V normal Tube。它需要与 M6 recovery
合同单独对齐，不能破坏当前 zero-connected planner baseline。

## 6. 建议文件边界

V2-G 首轮只允许：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_filter.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
```

只有定向测试证明 production plumbing 需要同步，才单独申请
`certified_tube_builder.*` / `tube_epoch_types.h`；不得顺手修改 manager、planner、C2、
adapter authority、Runtime、launch、ROS schema 或参数。

V2-V 必须另写执行规格和白名单。

## 7. V2-G 必须回归

- steep but continuously safe corridor：raw slope `2--3` 不因 `0.80` 自动截短；
- local bottleneck：窄点只影响局部，不向整个 preview 传播 global contraction；
- true empty cross-section：仍正确 zero-only/fail-closed；
- unsafe intermediate surface：endpoint 安全但中间不安全时仍被 Validator 拒绝；
- current structural seam/off-grid anchor：A6-P1/P1R 行为不退化；
- exact PWL step：跨 knot containment 与 Runtime 既有回归全部通过；
- alternating corridor：geometry 可存在；若当前 Runtime 无动态 witness，必须是
  certificate denial/U_safe/replan 语义，不能伪称 geometry 不存在；
- margin accounting：production query 始终只使用 planner-authoritative clearance；
- Z1/M3/H2/A6 与全量构建保持通过。

## 8. 停止边界

本文件当前只用于 proposal 对齐和后续调度。A6-P2 未完成前不得以本文件修改 Tube
源码。V2-0 完成并由主代理审核新 baseline 后，再形成 V2-G 正式执行规格交给
`gpt-5.6-terra / xhigh` 实施。
