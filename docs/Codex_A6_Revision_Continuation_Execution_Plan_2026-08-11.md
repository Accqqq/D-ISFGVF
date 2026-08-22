# Codex A6 Revision Continuation Execution Plan

```
DOCUMENT_ROLE=EXECUTION_PLAN_DRAFT
DOCUMENT_STATUS=WITHDRAWN_INVALID_PREMISE
EXECUTION_PROHIBITED=true
HISTORICAL_BODY_NON_NORMATIVE=true
IMPLEMENTATION_AUTHORIZED=false
AUTO_ADVANCE=false
STAGE=A6
A5_STATUS=RETURNED_TO_A5R
```

> 日期：2026-08-11
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`
> 证据基准：`/tmp/claude_s6_esdf_active_67s_JQDxgq/evidence/s6_esdf_active_67s.bag`
> 行号取自当前工作区，已核对。

> **撤回声明（2026-08-11）：本执行单全部阶段均不得实施。**
> `appendSlice(old_path)` 只保证 `[prefix_start, phase_at_switch]` 与旧路径逐点相同；
> `[phase_at_switch, join_w]` 是新构造的五次 Hermite connector，只在切换点匹配
> `p,p_w,p_ww`，区间内部不是旧几何。由于可复用的精确旧前缀位于 UAV 身后约
> `back_w`，它不能提供前向 tube 覆盖。因此 A6-2/A6-3 的事实透传与前缀保留机制
> 没有成立的几何前提。本计划未实施任何 A6 代码改动。

> 后续 A6 必须另写执行单，验证新路径、新 connector 与新 tube 的同版本构建、
> 切换点 continuation 和原子安装；不得把旧 tube profile 的世界坐标采样套到新
> connector 上。

> **以下全部正文仅为非规范历史记录。** 旧 §0、§1.2、§1.3、A6-2、A6-3、
> §4–5 及其白名单/三道门均不得引用为事实、验收依据或执行步骤。机器执行者读到
> 本行后必须停止，不得继续解释下方任务为授权。

---

## 0. 执行顺序（必须准确理解）

```
A5 冻结  →  A6（本执行单）  →  原封不动重测 A5  →  再决定 A5 是否还需要动
```

**不是"先做 A6 再重做 A5"。** A5 的 tube 几何（G2g：自适应 lifted-state 采样 → 净空射线非对称横截面
→ 曲率求交 → C1 斜率滤波 → Lipschitz 覆盖面证明）经审查是**正确的**，本阶段**一行都不许改**。

## 0.1 本执行单的两条已收回判断

写在最前面，防止被当作依据：

| 曾经的说法 | 实测 | 结论 |
|---|---|---|
| 「路径前端只剩 0.98 m，撑不住 lookahead 2.0」 | 中段实测 reqW 跨度 `t=5.40` → 2.18、`t=5.70` → 2.10、`t=5.90` → 1.90，**前端是够的**。0.98 m 只出现在 `t≥6.19` 的接近目标段，属正常终端行为 | **收回。前端长度不是缺陷，不在 A6 范围内** |
| 「走廊窄于裕度预算」 | [tube_cross_section.cpp:228-230](../src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp) 射线以 `clearance_query(p+Nδ, residualEffectiveRadius)` 前进，`c_plus_raw` 由已腐蚀的 `upper_final` 反推。裕度只扣一次 | **收回。符合 proposal §10.2** |

因此 **A6 的范围收窄为唯一一条**：路径 revision 变化时的 tube 连续性。

---

## 1. 唯一问题与量化依据

### 1.1 实测

```
构建尝试 46 次 → 完成 30 次        finalized = 65%
另两次 ESDF 运行                   0/52、0/1040   (0%)
tube marker 刷新                   5.9 Hz   gap p50=126ms  p95=429ms  max=714ms
Candidate 段长度                   p50=0.348 m
UAV 速度                           p95=1.96 m/s
UAV 在 ribbon 中的相对位置          p50=100%  p95=100%
```

### 1.2 为什么修这一条就可能够

覆盖判据：`Candidate 段长度 ≥ 速度p95 × 刷新间隔p95`

```
当前：1.96 × 0.429 = 0.84 m   >  0.348 m   → 不满足，UAV 每帧都飞出 ribbon
修后：构建节奏 46 次 / 6.6 s ≈ 143 ms
      1.96 × 0.143 = 0.28 m   <  0.348 m   → 满足
```

**丢弃率是把 p95 间隔从 ~143 ms 拉到 429 ms 的直接原因。** 修好它，覆盖判据自动满足，
tube 几何、margin、斜率上限、发布频率**都不需要动**。

### 1.3 代码位置与 proposal 依据

- 丢弃点：[phase_offset_matched_adapter.cpp:681](../src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp)
  `if (!requestSourceStillCurrent(*request)) { ... 整段丢弃 ... }`
  （`requestSourceStillCurrent` 定义在 [418](../src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp)）
- revision 判定：[phase_offset_matched_adapter.cpp:378](../src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp)
  `sourceRevision()` —— 指针 identity 或 `start_w` / `end_w` 任一变化即 ++。
  每次重规划都会新建 `ContinuousPhasePath` → 指针必变 → revision 必变 → **整段丢弃**。
- proposal §17.1 / 命题 5：C2 connector 在 retained phase 保证
  `p⁺=p⁻, p_w⁺=p_w⁻, p_ww⁺=p_ww⁻`，因此 `T, N, κ` 连续。
  **⇒ 在 `[w_current, join_w]` 区间内，新旧路径的 tube 几何是同一条，旧 profile 在该区间仍然有效。**

当前实现把「指针变了」等同于「几何变了」，这与 §17.1 直接冲突。

---

## 2. 全局禁止项

1. **禁止**修改 A5 tube 几何：`tube_builder.*`、`tube_cross_section.*`、`tube_filter.*`、`tube_surface_validator.*` 一律不得改动；
2. **禁止**修改任何裕度、`boundary_slope_max`、`lookahead_w`、`back_w`、`min_certified_forward_w`、`interior_margin`、`regularity_margin`；
3. **禁止**提高 marker / diagnostics 发布频率或 tube timer 频率来掩盖；
4. **禁止**新增 `TubeEpochState` / `RuntimeExecutionMode` / `TubeEpochReason` / `ControlFailureReason` / 新 diagnostics schema；
5. **禁止**修改 `v_max`、`a_max`、governor 参数、SO3、simulator、planner（KinoAstar / B-spline）；
6. **禁止**改变 `observe_only` 默认值；本阶段全程 `observe_only=true`；
7. **禁止**动 A5 的 Runtime / PortProjector / MatchedPort；
8. **禁止** `git reset --hard` / `restore .` / `checkout -- .` / `clean` / `stash pop` / commit / branch / tag / push。

---

## 3. 分阶段执行

### A6-1 — 只读：确认 C2 连续性事实可获得

- **单一目标**：确认 `gvf_manager` 在接受新路径时，是否已经持有「C2 成功」与「join 相位」这两个事实，且可无副作用地暴露给 adapter。
- **文件白名单**：**无**（只读）。
- **考察点**：
  - [gvf_manager.cpp:2094](../src/swarm_planner/bspline_traj/src/gvf_manager.cpp) `buildPhaseV2C2Frontend` 内 `best_join_delta_w` / `connector_success`；
  - [gvf_manager.cpp:5603](../src/swarm_planner/bspline_traj/src/gvf_manager.cpp) 点到点重规划调用处，`frontend_ready`（[5617](../src/swarm_planner/bspline_traj/src/gvf_manager.cpp)）与失败分支（[5659](../src/swarm_planner/bspline_traj/src/gvf_manager.cpp)）；
  - `buildMappedPhaseFrontend` 分支（[5610](../src/swarm_planner/bspline_traj/src/gvf_manager.cpp)）**不保证 C2**，必须与 C2 分支区分。
- **停止条件**：书面回答三问 ——
  1. C2 成功/失败是否已有明确布尔？
  2. `join_w` 是否可得？
  3. `buildMappedPhaseFrontend` 路径下是否必须按「不连续」处理？
  任一为否 → 停止并报告，不得自行发明新状态。

### A6-2 — 打通 C2 连续性事实到 adapter

- **单一目标**：让 adapter 知道「本次 revision 变化在 `[w_current, join_w]` 上是 C2 连续的」。
- **文件白名单**：
  - `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
  - `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`（仅 revision 安装处与 `MatchedAdapterInput` 填充处）
  - `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`（仅 `MatchedAdapterInput` 增字段）
- **改动**：`MatchedAdapterInput` 增加两个**纯事实字段**（非状态机）：
  `bool path_c2_continuous_from_previous` 与 `double path_c2_join_w`。
  C2 分支成功时填 `true` + `join_w`；`buildMappedPhaseFrontend` 分支或 C2 失败时填 `false`。
- **禁止**：改变任何控制路径；在 `gvf_manager.cpp` 新增超过约 30 行。
- **停止条件**：`observe_only=true` 下 `/position_cmd` 与 A6-2 前**逐帧 bitwise 相同**（本阶段纯透传，不得改变行为）。

### A6-3 — 用连续性事实保留前缀，替代整段丢弃

- **单一目标**：`finalizeTubeEpoch` 在 revision 变化但 C2 连续时，保留 `[preview_start_w, min(join_w, preview_end_w)]` 的前缀，而不是全丢。
- **文件白名单**：
  - `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`（[418](../src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp) 与 [681](../src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp) 两处）
  - `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
  - 对应测试文件
- **改动**：
  1. `requestSourceStillCurrent` 保持原语义不变（不得放宽），**另加**一个并列判定：revision 变化 + C2 连续 + `join_w` 有效 ⇒ 前缀可保留；
  2. 保留的前缀必须**按 `join_w` 严格截断**，`join_w` 之后一律丢弃并等待新构建；
  3. `buildMappedPhaseFrontend` 分支、C2 失败、`join_w` 非有限 ⇒ **维持原 fail-closed 整段丢弃**。
- **禁止**：放宽 `requestSourceStillCurrent` 本身；保留 `join_w` 之后的任何采样；跳过 cloud usable 契约。
- **停止条件（硬）**：见 §4 三道门，任一不过即停止并报告，不得进入重测 A5。

---

## 4. 验收门（闭环、可证伪、无法靠调参伪造）

| 门 | 判据 | 当前值 |
|---|---|---|
| **门 1** | `finalized / attempted ≥ 90%` | 30/46 = **65%**（另两次运行 0%） |
| **门 2** | `Candidate 段长度 p50 ≥ 速度p95 × marker间隔p95` | 0.348 vs 需 **0.84** |
| **门 3** | `observe_only=true` 下 `/position_cmd` 与 A6 前**逐帧 bitwise 相同** | —— |

**门 1 是关键**：调宽 tube、放宽 margin、提高发布频率**都提高不了完成率**，只有真正修好 revision 连续性才能过。

**门 3 是安全门**：A6 全程不得改变控制输出。若不成立，说明改动越界。

### 采集方式

A6 前后各跑一次同图（`pillar.pcd`）、同起点、同目标的 ESDF active 录包，各自统计：

```
attempted   = /formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics 消息数
finalized   = /formation_planning/phase_offset_manual/diagnostics 消息数
marker gap  = /formation_planning/phase_offset_manual/tube_candidate 相邻间隔 p50/p95/max
段长度      = raw_candidate payload idx21 - idx20  的 p50/max
速度        = /sim/odom twist 模长 p95
ribbon 相对位置 = UAV 到 candidate marker(id=0) 折线的最近点弧长 / 折线总长
```

---

## 5. 成功判据与后续

三道门全过 ⇒ **原封不动重测 A5**（不改任何 A5 文件），再看：

- 若「慢一拍」消失 ⇒ **A5 无需再改**，此前 19 份 A5 执行单追的是 A6 的症状；
- 若仍未消失 ⇒ 才轮到 `Codex_A5_Tube_Coverage_Corrective_Execution_Plan_2026-08-11.md` 的 **T1 只读测量**，
  用它判定斜率受限 tube 模型是否适用于柱子场景。那是 proposal 级建模问题，需与导师确认，
  **不得再由执行者自行修实现**。

`AUTO_ADVANCE=false`：每阶段完成后写自审并停止，等待新的执行单。
