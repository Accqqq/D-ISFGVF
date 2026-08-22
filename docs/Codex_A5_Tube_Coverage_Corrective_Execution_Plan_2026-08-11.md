# Codex A5 Tube Coverage Corrective Execution Plan

```
DOCUMENT_ROLE=EXECUTION_PLAN_DRAFT
DOCUMENT_STATUS=SUPERSEDED
SUPERSEDED_BY=Codex_A5R_Tube_Performance_And_Installability_Recovery_Execution_Plan_2026-08-11.md
EXECUTION_PROHIBITED=true
HISTORICAL_BODY_NON_NORMATIVE=true
IMPLEMENTATION_AUTHORIZED=false
AUTO_ADVANCE=false
```

> 日期：2026-08-11
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`
> 前置状态：S0–S6 已由 Codex 执行完毕。本文件**只针对遗留的 "tube 慢一拍" 问题**。
> 证据基准：`/tmp/claude_s6_esdf_active_67s_JQDxgq/evidence/s6_esdf_active_67s.bag`（2026-08-11 19:14，67 s，ESDF active）
> 所有行号均取自**当前工作区**，已逐条核对。

> **状态声明（2026-08-11）：本执行单已被替代，不得实施。**
> 原 T1 的跨帧差分混合了不同地图 observation、路径 revision 与
> preferred-delta/component 选择，不能据此作模型级 `(a)/(b)` 判定；原 T5
> 依赖的 C2 区间前缀复用前提也已证伪。后续只允许按上方
> `SUPERSEDED_BY` 指向的 A5R 草案重新授权。

> **以下全部正文仅为非规范历史记录。** 旧 §1、T1、T5、成功判据及其因果结论
> 均不得引用为当前事实、验收依据或执行步骤；其中“真实截断者就是斜率包络”、
> 跨帧 T1 和 C2 前缀复用均尚未成立或已被证伪。机器执行者读到本行后必须停止，
> 不得继续解释下方白名单或任务为授权。

---

## -1. 迁移表（历史内容保留，不构成授权）

| 本计划条目 | 新状态 | 迁移去向 |
|---|---|---|
| T1 跨 ROS 帧边界斜率差分 | **撤回** | A5R-0：同一次 build、同一路径、同一 snapshot、同一 preferred delta 内测量 |
| T2 真实截断标签 | **移出 A5R，保留为 diagnostics-only 候选** | 待正确 T1 与失败阶段定位后，另发白名单闭合的独立执行单 |
| T3 horizon/replan 语义 | **保留并扩展** | A5R-3：与 OFFSET_OUTSIDE、unsafe、unknown、incomplete、端口可行性统一分类 |
| T4 `c_plus_raw/c_minus_raw` 命名 | **保留为低优先级候选** | 不得与性能或几何修正混阶段 |
| T5 revision 前缀复用 | **撤回** | 旧 A6 已标记 `WITHDRAWN_INVALID_PREMISE` |

---

## 0. 先纠正三个错误判断

前一轮审查中我提出过三个假设，实测后**全部否定**，不得作为本执行单的依据：

| 假设 | 实测 | 结论 |
|---|---|---|
| `TubeFilter` O(n⁴) 导致节点挂死 | 每次构建采样数 `n = 2…30`（payload idx 11），远低于 `kMaxAdaptiveSamples=4096`（[tube_builder.cpp:331](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp)） | **否定** |
| 节点在 t≈17.7 s 崩溃/死锁 | roslaunch 日志为正常退出；rosout 含 `[GOAL][REACHED] distance=0.198`。之后 47 s 静止是 `receive_goal` 清零后 `cmdCallback` 提前返回 | **否定，任务实际成功完成** |
| 物理走廊窄于裕度预算（c⁺+c⁻ 0.96 < 1.10） | [tube_cross_section.cpp:228-230](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp) 的射线以 `clearance_query(p+Nδ, residualEffectiveRadius)` 前进；[tube_cross_section.cpp:335-336](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp) 的 `c_plus_raw` 由**已腐蚀**的 `upper_final` 反推。我重复扣了一次 `r_eff` | **否定，裕度只扣一次，符合 proposal §10.2** |

## 0.1 Codex 已做对、不得回退的部分

- **S3 成功**：`/position_cmd` 实测 **47.9 Hz**（bag 内 329 帧 / 6.86 s），`/sim/odom` 100 Hz，`/sim/local_map` 10.0 Hz。重计算已移出控制回路。
- **S0 生效**：`residual_effective_radius` 实测恒为 `0.450`，说明 `cloud_obstacle_set_complete=true` / `preincluded_map_uncertainty=0.10` 已正确传入。
- **S4 落地**：[gvf_manager.cpp:455](../src/swarm_planner/bspline_traj/src/gvf_manager.cpp) 在 `reference_delta != 0` 时返回 `geometry.r`；`governor_reference_delta = matched_output.delta`（[gvf_manager.cpp:1182](../src/swarm_planner/bspline_traj/src/gvf_manager.cpp)）→ [gvf_manager.cpp:1311](../src/swarm_planner/bspline_traj/src/gvf_manager.cpp)。
- **S5 落地**：`tube_filter.cpp` 已改用真实 Hermite 值/导数（[tube_filter.cpp:478-489](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp)），原 P1-1「查询斜率在 knot 处恒为 0」**已修复**。

---

## 1. 唯一遗留问题：Candidate 覆盖长度小于一次刷新期的位移

### 1.1 实测数据（同一 bag）

```
tube marker 刷新率           5.9 Hz    gap p50=126ms  p95=429ms  max=714ms
构建尝试 46 次 → 完成 30 次              35% 因 path revision 变化被丢弃
certified 段长度             p50=0.348  max=1.974     (requested = 2.10)
前视余量 certSegEnd - curW   p50≈0.19                 (min_certified_forward_w = 0.40)
UAV 在 ribbon 中的相对位置    p50=100%   p95=100%      每一帧都在最前端或已越过
UAV 到 ribbon 尾端弧长        p50=0.44m  max=1.78m     (设计 back_w = 0.20)
UAV 速度                     p95=1.96 m/s
```

**判据**：UAV 在两次 tube 刷新之间飞过 `1.96 × 0.126…0.429 = 0.25…0.84 m`，而 Candidate 段只有 `0.348 m`。
**覆盖长度 < 刷新期位移 ⇒ 每一帧发布出来时 UAV 都已飞过 ⇒ 视觉上恒定"慢一拍"。**

### 1.2 截断来源（已定位，非猜测）

payload `firstTruncatedReason` 在 **46/46 帧全部为 `REGULARITY`**，`certified_segment_truncated_after` 在 **46/46 帧为 1**。

但该 reason 是**硬编码假标签**：

- [tube_filter.cpp:436](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp)
  `profile.first_truncated_reason = TubeStopReason::REGULARITY;`
  —— 只要 `TubeFilter` 自己发生截断就无条件写 `REGULARITY`，与曲率正则性 `1-κδ≥μ` 毫无关系。

真实截断者是 `TubeFilter` 的**边界斜率包络**：

- 斜率上限 `knot_slope = boundary_slope_max / 1.5 = 0.80 / 1.5 = 0.533`
  （[tube_filter.cpp:268](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp)）
- 逐段 Hermite 导数硬校验
  （[tube_filter.cpp:119-120](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp)）
- `FilterRange` 失败后由 `(first,last)` 双重搜索确定性地缩短区间
  （[tube_filter.cpp:367-368](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp)）

即：**柱子附近走廊闭合速度超过 0.533 δ/w，斜率受限的边界函数无法表达该段，于是后缀被整体丢弃。**

### 1.3 与 proposal 的关系

- §10.5 / §3.5 要求边界经保守平滑、分段 C1、**斜率有界** —— 斜率上限本身是 proposal 要求的，不是 bug。
- §10.4 明确："**某个未来安全横截面为空**" 是**更新中心线**的触发条件之一。
- 当前实现在斜率包络使区间变空时，既不上报也不触发重规划，只是缩短 Candidate 并 `markWaiting`
  （[tube_epoch_manager.cpp:586-588](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp) `FORWARD_HORIZON_SHORT`）。

**所以这既是诊断标签缺陷，也是 hybrid 语义缺失，而不是几何计算错误。**

### 1.4 需要用户决策的建模问题（不得由执行者自行决定）

`boundary_slope_max = 0.80`（[test_gvf.launch:38](../src/swarm_planner/bspline_traj/launch/test_gvf.launch)、[phase_offset_esdf_tube_single.launch:73](../src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch)）
对应 `dδ/dw ≤ 0.533`，约 28°。柱子处走廊闭合接近 90°。

因此存在两种可能，**必须先测量再定性**：

- (a) 原始边界斜率仅在少数采样点越限 ⇒ 属实现问题，按 T2 修；
- (b) 原始边界斜率普遍越限 ⇒ **斜率受限 tube 模型本身不适用于该场景**，属 proposal 级建模问题，需与导师确认后再改，不在本执行单范围内。

T1 的产出就是用来判定 (a)/(b) 的。

---

## 2. 全局禁止项

本执行单下列做法**一律禁止**，违反即视为未完成：

1. **禁止**提高 `boundary_slope_max`、`lookahead_w`、`fixed_delta_max` 或任何 tube 尺寸参数来让 ribbon 变长；
2. **禁止**降低 `uav_radius` / `map_uncertainty` / `localization_uncertainty` / `tracking_error_bound` / `regularity_margin` / `interior_margin`；
3. **禁止**提高 marker 或 diagnostics 发布频率来掩盖覆盖不足；
4. **禁止**降低 `min_certified_forward_w` 让 `FORWARD_HORIZON_SHORT` 不再触发；
5. **禁止**新增 state / mode / certificate / gate / diagnostics schema；
6. **禁止**修改 `v_max`、`a_max`、governor 任何参数、SO3、simulator；
7. **禁止**把 A5 与 A6 混在同一阶段；
8. **禁止** `git reset --hard` / `restore .` / `checkout -- .` / `clean` / `stash pop` / commit / branch / tag / push。

---

## 3. 分阶段执行

### T1 — 只读测量：判定 (a) 还是 (b)

- **单一目标**：测出原始边界斜率 `dδ̄/dw`、`dδ̲/dw` 的真实分布，与 `knot_slope = 0.533` 比较。
- **文件白名单**：**无**。本阶段不修改任何源码、launch、CMake。
- **做法**：离线复用现有 bag。`tube_raw_candidate_diagnostics` 已含逐帧 `CurrentEnvironmentLower/Upper`（idx 43/44）与 `CurrentW`（idx 26）；对相邻帧做差分即可得到沿 w 的原始边界斜率下界估计。若精度不足，允许写**一个** `/tmp` 下的只读分析脚本，不得写入仓库。
- **禁止**：修改任何仓库文件；重新跑 ROS（先用已有 bag）。
- **停止条件**：给出 `|dδ/dw|` 的 p50 / p90 / p99 与超过 0.533 的采样占比，并明确判定 (a) 或 (b)。**判定为 (b) 时立即停止，等待用户与导师决策，不得进入 T2。**

### T2 — 截断语义与标签修正（仅在 T1 判定为 (a) 时执行）

- **单一目标**：让"斜率包络导致的截断"可被诊断，并接入已有的重规划触发；不改变任何几何数值。
- **文件白名单**：
  - `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp`
  - `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h`
  - `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp`
- **改动（穷尽）**：
  1. `TubeStopReason` 增加 `SLOPE_ENVELOPE_INFEASIBLE`（**这是新增枚举值，不是新增 state/mode**，用于替换假标签，允许）；
  2. [tube_filter.cpp:436](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp) 的硬编码 `REGULARITY` 改为写入真实原因，并把"斜率包络使区间变空"的那个 `w` 写进 `first_truncated_w`；
  3. 不改 `knot_slope`、不改 `(first,last)` 搜索策略、不改任何边界数值。
- **禁止**：改变 `filtered_lower/upper` 的任何数值；改 `boundary_slope_max`；改 `FilterRange` 的接受/拒绝判据。
- **停止条件**：
  - 现有 `tube_filter_test` 全绿，且新增一个用例证明"截断 w 与原因被正确上报"；
  - 用同一 bag 离线重放，`firstTruncatedReason` 不再是恒定 `REGULARITY`；
  - **数值回归**：`filtered_lower/upper` 与 T2 前逐点 bitwise 相同。

### T3 — 前视不足接入既有重规划触发（A5 语义，非新增 gate）

- **单一目标**：`FORWARD_HORIZON_SHORT` 由"静默 WAITING"变为"上报中心线前视不足"，接到**已存在**的重规划路径，符合 proposal §10.4。
- **文件白名单**：
  - `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp`（仅 [586-588](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp) 附近）
  - `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`（仅诊断透传）
- **禁止**：新增 `TubeEpochState` / `TubeEpochReason`；直接在 A5 内部调用 planner；改 `min_certified_forward_w`。
- **停止条件**：`observe_only=true` 下 `/position_cmd` 与 T2 结果逐帧 bitwise 相同（本阶段只上报，不得改变控制）。

### T4 — 命名修正（P2，可与 T2 合并评审但独立提交）

- **单一目标**：消除会误导审计者的字段名。我本人就是被它误导的。
- **文件白名单**：`tube_types.h`、`tube_cross_section.*`、`phase_offset_raw_candidate_diagnostics.*` 及其测试。
- **改动**：`c_plus_raw` / `c_minus_raw` 更名为 `c_plus_eroded` / `c_minus_eroded`（或等效名），并在头文件注释写明"已扣除 `residualEffectiveRadius`，非原始物理距离"。**字段顺序与 49 字段总数不得变化。**
- **停止条件**：`static_assert(kRawCandidateDiagnosticCount == 49U)` 保持成立；全部相关测试通过。

### T5 — 35% 构建丢弃（**属于 A6，本执行单不授权**）

[phase_offset_matched_adapter.cpp:681-693](../src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp) 在 `requestSourceStillCurrent` 失败时整段丢弃。
按 proposal §17.1 / 命题 5，C2 connector 已保证 retained phase 处 `p, p_w, p_ww` 连续，因此 join 相位之前的前缀本可保留。
**需要独立的 A6 执行单，禁止在本执行单内实施。**

---

## 4. 每阶段共同验收

1. **单元测试**：本阶段新增用例 + 已完成阶段全部回归；
2. **数值不变性**：T2/T3 必须给出 `filtered_lower/upper` 与 `/position_cmd` 的逐帧 bitwise 对比结果；
3. **ROS A/B**：同图（`pillar.pcd`）、同起点、同目标，A = 阶段前，B = 阶段后，各保存
   完整 roslaunch 日志、rosbag、以及本文件 §1.1 同格式的六项统计
   （刷新率 / gap p50,p95,max / 尝试与完成数 / 段长度 p50,max / UAV 在 ribbon 中的相对位置 / UAV 速度 p95）；
4. **git**：阶段前后记录 `git status --short` 与 tracked diff；跑 `git diff --check`；不 commit / branch / tag / push；
5. **停止**：`AUTO_ADVANCE=false`，每阶段完成后写自审并**停止**，等待新的执行单。

---

## 5. 成功判据

本执行单**不以"ribbon 变长"为成功判据**（那会诱导调参掩盖）。成功判据是：

1. 截断原因可诊断且真实（T2）；
2. 前视不足被显式上报并接入既有重规划（T3）；
3. 诊断字段名不再误导（T4）；
4. 在此基础上，用户与导师能据 T1 的测量结果，**明确判定斜率受限 tube 模型是否适用于当前场景**。

若 T1 判定为 (b)，则"tube 慢一拍"的正确结论是**模型选择问题**，应回到 proposal 层面讨论，而不是继续在实现层修补。
