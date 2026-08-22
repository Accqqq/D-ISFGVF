# Codex 执行单 S3.5 — Tube 构造代价与窗口时效性

```
DOCUMENT_ROLE=EXECUTION_ORDER
IMPLEMENTATION_AUTHORIZED=true   (仅限本执行单 S3.5)
AUTO_ADVANCE=false
STAGE=S3.5
PREREQ=S3 已完成并自审通过
NEXT=S4 (未授权，需新执行单)
```

## 0. 本阶段唯一目标

**让 Candidate tube 在时间上对准当前相位。** 不改任何几何、margin、证书语义或控制逻辑。

用户观察到的三个现象，全部来自时效性，与 tube 几何无关（用户明确排除了"形状对新障碍反应迟钝"）：

1. tube 整体落在 UAV 后面；
2. 换路后 tube 消失一段时间才回来；
3. tube 一格一格地跳，不连续。

## 1. 已测量的根因（不要重新论证，直接修）

证据：`/tmp/claude_s3_20260811_TerraMax_H6Pq9/*/tube_due_raw.csv`（S3 自带 instrumentation）

| run | build p50 | p95 | max | finalized |
|---|---|---|---|---|
| `ros_fixed_1` | 2.5 ms | 5.2 ms | 7.6 ms | — |
| `ros_esdf_churn2` | **107.4 ms** | **754.1 ms** | **6368 ms** | 322/399 = 81% |
| `ros_esdf_1` | — | — | — | **0/52 = 0%** |
| `ros_esdf_stress1` | — | — | — | **0/1040 = 0%** |

tube timer 周期 0.10 s（`gvf_manager.cpp:186-189`）。p50 已超周期 ⇒ 定时器长期饱和，`timer_inflight_` 丢弃重叠 tick。

### 根因 A — `TubeFilter::filter` 的 `pass` 循环是冗余的，把 `FilterRange` 变成 O(count²)

`tube_filter.cpp:93-119` `ApplySlopeEnvelope`：

```cpp
for (std::size_t pass = 0U; pass < count; ++pass) {   // <-- 冗余
  ... 一次后向传播 ...
  ... 一次前向传播 ...
}
```

该更新是两个**互相独立**的一维 Lipschitz 包络（`upper` 取 min-envelope，`lower` 取 max-envelope），其精确解为
`u*[i] = min_j (raw_upper[j] + s·|w_i − w_j|)`、`l*[i] = max_j (raw_lower[j] − s·|w_i − w_j|)`。
一维链上标准的**后向一遍 + 前向一遍**顺序算法即为精确解，`pass ≥ 2` 全部是 no-op。

配合 `tube_filter.cpp:243-259` 的 `(first, last)` 双重枚举，总复杂度约 **O(n⁴/11)**。

而 n 不是常数：`tube_builder.cpp:319-326` 的自适应细分判据要求
`normal_deviation × search_extent(3.0) ≤ 0.25 × resolution` 且 `boundary_span ≤ resolution`，
在曲率段上会把 dw 逼到 ~0.008，2.2 的 preview 跨度即产生**数百个采样**，上限 `kMaxAdaptiveSamples = 4096`（`tube_builder.cpp:331`）。
这正好解释 107 ms → 6.4 s 的分布形状：耗时随局部几何爆炸，**恰恰在靠近障碍时最慢**。

### 根因 B — build 比 replan 慢 ⇒ 结果被整体丢弃

`phase_offset_matched_adapter.cpp:681-693` `finalizeTubeEpoch`：`requestSourceStillCurrent` 为假时，
整个 build 连同 `TubeEpochManager` 状态一起重建丢弃。`planInterval = 0.5`，而 p95 build = 754 ms，
必然跨过换路 ⇒ 实测两个 ESDF run **finalized = 0%**。

另外 `phase_offset_matched_adapter.cpp:994-997` 在 revision 变化时**立即清空** candidate 快照，
在新 candidate 就绪前必然出现一个 ≥ 1 个 build 周期的空窗 ⇒ 现象 2。

### 根因 C — 显示窗口冻结在 request 时刻，且 marker 只由 10 Hz timer 发布

`makeBuildRequest` (`phase_offset_matched_adapter.cpp:406`) 把当时的 `phase_w_` 冻进请求；
`makePreview` 用这个冻结的 `w` 开 `[w−back_w, w+lookahead_w]`；
marker 画的是该 profile 的**全部** sample（`phase_offset_tube_markers.cpp:81-99`）。

⇒ ribbon 锚在 `request_age + build_time` 之前：p50 ≈ 0.21 s，@`v_max=2.0` ≈ **0.4 m**；p95 ≈ 0.85 s ≈ **1.7 m**。
而 `back_w = 0.20` ⇒ 滞后量超过整个后向余量 ⇒ 现象 1。

marker 现在只在 `finalizeTubeEpoch:716` 发布，从 50 Hz 掉到 ≤10 Hz ⇒ 现象 3。

### 架构判定

**tube 被当作"冻结的世界坐标带"交付，而 proposal 里 tube 是 w 的边界函数 δ̄(w)、δ̲(w)。**

两者更新频率本应分离（proposal §10.5 / §22.5 / §22.8）：

- **慢（10 Hz）**：在一个 w 域上计算边界函数 —— 需要 clearance 查询、滤波、曲面证明；
- **快（cmd 速率）**：在**当前** w 处开窗/取值 —— `TubeFilter::query` 已是 O(n)，且 `TubeRawSample` 已存了 `p` 与 `N`，画 ribbon **不需要任何几何求值**。

S3 把两者捆在慢路径上，所以窗口跟着慢。本阶段把"开窗与显示"搬回快路径。

## 2. 文件白名单（只允许改这三个文件 + 对应测试）

```
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
```

任何其他文件（含 `tube_builder.cpp`、`tube_surface_validator.cpp`、`port_projector.cpp`、
`phase_offset_runtime.cpp`、`gvf_manager.cpp`、任何 launch、任何 CMakeLists）**不得修改**。
若发现必须改白名单外文件才能达标，**停止并报告**，不得扩大范围。

## 3. 任务

### T1 — 消除 `ApplySlopeEnvelope` 的冗余 `pass` 循环（根因 A）

**文件**：`tube_filter.cpp`

把 `tube_filter.cpp:93` 的 `for (pass = 0; pass < count; ++pass)` 改为固定的
**一次后向传播 + 一次前向传播 + 一次有效性检查**，其余逻辑（斜率 `knot_slope`、
`raw` 夹紧、空区间判定、`DenseSafe`、局部修复预算）**逐字不变**。

**这是行为等价改动，不是近似。** 必须用测试证明等价（见 §5 T1 测试）。

**禁止**：改 `knot_slope` 的定义（`boundary_slope_max / 1.5`）；改 `kEpsilon`；
改 `dense_samples_per_segment`；改 `kLocalRepairBudget`；改 `(first, last)` 的选择规则。

> 预期效果：`FilterRange` 从 O(count²) 降到 O(count)，整体从 O(n⁴) 降到 O(n²·dense)。
> 以 n≈300 估算，p50 应从 107 ms 落到 10 ms 量级。**先只做 T1 并测量**；
> 若 T1 后 p95 仍 > 20 ms，再报告并申请 T1b（把 `(first,last)` 枚举换成单调二分），
> 不要在本执行单内自行扩大。

### T2 — 换路不再产生空窗（根因 B，现象 2）

**文件**：`phase_offset_matched_adapter.cpp`

1. **事件驱动重建**：`sourceRevision()` 变化时，除写入 `latest_build_request_` 外，
   置一个 `rebuild_urgent_` 标志；`timerTick()` 读到该标志时立刻构建，
   不等下一个周期边界。（timer 周期本身不改，仍由 `gvf_manager.cpp` 的 0.10 s 拥有。）
2. **不要提前清空**：删除 `phase_offset_matched_adapter.cpp:994-997` 处
   "revision 变化即清空 candidate 快照"的动作。改为：
   保持既有快照，但把它标记为 `stale_revision = true`；
   `epochMatchesRequest` / Runtime / Certified 对 `stale_revision` 一律**不接受**（与现状同样 fail-closed），
   仅 Candidate marker 在**至多一个 build 周期**内继续显示，之后若仍无新快照则 DELETE。

**严禁**：跨 revision 把旧 tube 当作安全证书；严禁让 `stale_revision` 快照进入
`latest_epoch_snapshot_`、Runtime、Certified 或任何 `active_profile`。
本条只影响 **UNCERTIFIED Candidate 的显示连续性**。

**严禁**：在本阶段实现"保留旧路径前缀几何"（proposal §17.1 的 retained-prefix 复用）。
那属于 A6 continuation，需要独立执行单。

> 预期效果：`finalized` 率从 0%/81% 回到 ≥ 95%（因为 T1 已使 build 远短于 replan 间隔）。

### T3 — 开窗与 ribbon 发布搬回快路径（根因 C，现象 1 与 3）

**文件**：`phase_offset_matched_adapter.{h,cpp}`

1. **构造域留延迟余量**：`makePreview` 的窗口改为
   `[w − back_w − latency_slack, w + lookahead_w + latency_slack]`，
   其中 `latency_slack` 由**已有**参数推导，**不新增 launch 参数**：
   `latency_slack = tube_update_period × u_w_abs_max`（保守上界，当前 = 0.10 × 0.12 = 0.012）。
   若实测仍不足，报告实测值并申请，不要自行加参数。
2. **快路径开窗**：在 `update()`（cmd 线程）中，用**当前** `input.path.w`
   在缓存 profile 上取显示窗口 `[w − back_w, w + lookahead_w] ∩ [preview_start_w, preview_end_w]`，
   只取落在窗口内的 `TubeRawSample` 子序列。
3. **ribbon 在快路径发布**：把 Candidate/Certified 两个 MarkerArray 的发布从
   `finalizeTubeEpoch:716` 移回 `update()`，用 §T3.2 的子序列绘制。
   `TubeRawSample` 已含 `p` 与 `N`，绘制**不得**调用 `GeometryEvaluator`。
4. **base_path / active_path / 四路 diagnostics 保持在 timer 上**（它们才是 S3 要搬走的重负载）。

**禁止**：改 `back_w`、`lookahead_w`、`sample_step_w`、`update_period` 的默认值或 launch 值；
禁止改 `tubeDisplayCertified` 的任何一个条件；禁止改 marker 的颜色、ns、id、类型。

## 4. 全阶段禁止项

- 不得调大 tube、放宽任何 margin（`uav_radius` / `map_uncertainty` / `localization` /
  `tracking_error_bound` / `preincluded_map_uncertainty` / `interior_margin` / `continuous_inset`）。
- 不得新增 gate、mode、state、reason、certificate 或 diagnostics 字段。
- 不得修改 `v_max`、`a_max`、governor 任何参数、`planInterval`。
- 不得触碰 `tube_builder.cpp` 的自适应细分判据（`search_extent=3.0` 的过保守问题**已记录但本阶段不改**）。
- 不得进入 S4/S5/A6。
- 不得 commit / branch / tag / push；不得 `git reset --hard`、`git restore .`、`git clean`、
  不得 pop prototype stash。

## 5. 停止条件（全部为硬条件，任一不满足即停止并报告）

### T1
- **等价性测试**：新增单测，对 ≥ 500 组随机 `(raw_lower, raw_upper, dw, knot_slope)`
  断言"两遍算法"与"原 `count` 遍算法"输出**逐位相同**（`==`，不是容差）。
- 现有 `tube_filter_test.cpp` 全部通过，无用例修改。

### T2 / T3
- `ros_esdf_*` 重跑，`tube_due_raw.csv` 的 **p50 < 20 ms 且 p95 < 50 ms**。
- **`finalized` 率 ≥ 95%**（对比 S3 的 0% / 81%）。
- Candidate marker 的 ADD 帧占比 ≥ S3 基线；换路前后连续 DELETE 不超过 1 帧。

### 回归（必须）
- `phase_offset_manual_observe_only = true` 时，`/position_cmd` 序列与 S3 记录**逐帧 bitwise 相同**。
  （本阶段不碰控制，任何差异都说明改动越界。）
- workspace 全量 ctest 通过（`uav_utils-test` 的历史缺失项除外，不得修复它）。

## 6. 必须提交的证据

1. 阶段前后 `git status --short` 与 `git diff --stat`；`git diff --check`。
2. `tube_due_raw.csv` / `callback_raw.csv`：S3 基线与 S3.5 各一份，附 p50/p95/max/finalized 对比表。
3. `ros_fixed_*` 与 `ros_esdf_*` 各 ≥ 1 组完整 roslaunch 日志 + rosbag。
4. `observe_only=true` 的 `/position_cmd` 逐帧比对结果（哈希或逐帧 diff）。
5. 全部证据的 SHA256。
6. 书面自审 + **显式停止声明**。

## 7. 停止

完成 T1/T2/T3 与 §5 全部停止条件后**停止**。不得开始 S4。
若 T1 后 p95 仍 > 20 ms，**只报告实测的分阶段耗时分解**（TubeFilter / SurfaceValidator / 自适应采样各占多少），
等待新执行单，不得自行优化 SurfaceValidator 或细分判据。
