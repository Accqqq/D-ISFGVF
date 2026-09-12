# M4I：终点附近偏移回中（"意图偏移轨迹接不到终点 / 状态机不进 WAIT_TARGET"）

日期 2026-09-11。状态：已执行。（M4H 的显示层诊断与改动已撤回。）

## 1. 现象与根因（用户指出）

经过集群意图调整后 δ 偏移不为零，而执行参考是

```
r(w) = p(w) + N(w)·δ
```

路径末端 `p(end) ≈ goal`，所以 δ≠0 时参考停在 `goal + N·δ`：飞机也随之停在离目标
`|δ|` 处。而到点判据是"飞机与目标的几何距离 ≤ 0.2 m"：

- 距离永远 > 0.2 m → `terminal_candidate` 永远为假；
- `requestRecenter()` 只在 `terminal_candidate` 为真时才调用 → δ 永远不会被拉回；
- 于是既接不到终点，状态机也永远进不了 `WAIT_TARGET`。

这是一个自锁：**用"已经在目标附近"去触发"把参考拉回目标"**。

## 2. 修法（两处，互补）

1. **终点邻域按当前偏移放大**（`gvf_manager.cpp`）

   在保留 `retainedDelta()` 非零时，把到点邻域放大为
   `0.2 + |δ| + slack`（`slack = gvf/point_goal/terminal_offset_slack`，默认 0.25，
   置 0 即关闭）。δ = 0 时半径仍是严格的 0.2 m，所以没有偏移的普通任务行为不变。
   效果：飞机还在 `|δ|` 之外时就进入 terminal 分支并 `requestRecenter()`，δ 开始
   收敛，收敛到 0 之后严格半径重新生效，按正常到点流程结束。

2. **未完成的回中拥有横向优先权**（`phase_offset_matched_adapter.cpp`）

   回中项 `recenter = -gain·δ·N` 之前是与集群意图 **相加**的，意图可以持续把 δ
   顶回去。现在当 `runtime_->recenterRequested()` 为真时（该标志在 δ 归零的瞬间
   自动清除），本 tick **不叠加** SPH 意图，让 δ 真正回到 0；δ 归零后意图立即恢复。
   效果：偏移轨迹末端重新并回目标点，而不是停在偏移平衡点上。

两者都不改 phase/tube/δ 的物理约束，也不新增门控；`max_terminal_residual_hold`
（有界等待）保留为兜底，但实测已不再被触发。

## 3. 实现范围（白名单）

1. `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
   - 终点邻域放大；新增参数 `gvf/point_goal/terminal_offset_slack`。
2. `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
   - 对应成员默认值。
3. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
   - 回中未完成时 SPH 意图让位。

## 4. 验收（柱阵场景，目标行 -11.2，全程）

| 指标 | 修改前（同一场景） | 修改后 |
|---|---|---|
| `[reach_goal]: EXEC_TRAJ -> WAIT_TARGET` | 依赖 `residual_forced` 兜底 | **3/3 正常进入** |
| `[REACHED]` 形式 | `residual_forced=1`，距离 0.082/0.304/0.246 | **正常到点**，0.188/0.192/0.195 |
| `nonzero Section reference retained` | 多次 | **0 次** |
| 终点位置 | 离目标 0.08~0.30 m | (-1.000, -11.200) / (0.000, -11.200) / (1.000, -11.200) |
| 相位窗口 / slew 拒绝 | 0 | 0 |
| 飞行中 `GOVERNOR_INVALID_HOLD` | 0 | 0（仅起飞前 3 次） |

单元测试：`phase_offset_matched_adapter_test` 10/10、`phase_offset_runtime_test` 15/15、
`phase_offset_allocator_test` 47/47、`phase_offset_tube_viability_test` 42/42、
`phase_offset_section_tube_test` 32/32、`phase_offset_section_input_test` 35/35。
