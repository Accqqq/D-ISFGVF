# M4F：横截面按曲率封顶（相位速率越窗的几何根治）

日期 2026-09-11。状态：已执行。

## 1. 问题

M4E 的实测里，第一次 `Section MatchedPort phase rate is outside the frozen
limits` 紧跟在一条刚装上的 C2 连接段之后（`join_span_ratio=1.18`）。原因不是速度：
对平面曲线，偏移 δ 后的参考 `r = p + N·δ` 满足

```
‖r_w‖ ≈ ‖p_w‖ · |1 − κ(w)·δ|          κ = 局部曲率
```

所以 **δ 越接近局部曲率半径 1/κ，偏移参考的导数越接近 0**，而
`w_dot = k1(α+σ)/‖r_w‖` 会发散。连接段正是 κ 最大的地方，集群意图又在同处把 δ
往外推，两者相乘就出现"刚装好连接段、下一个 tick 就判越窗"。

而当时这条链路上没有任何地方防这件事：

- `phase_offset_section_input.cpp` 的截面扫描只按障碍物自由距离取半宽，**零曲率项**；
- `minimum_reference_speed` 默认 `1e-8`，等于没有下限；
- `regularity_margin`（0.10）在 Section 路径上并未当作下限使用。

## 2. 语义（本批）

横截面的定义增加一条几何约束：**它只包含那些不会让偏移参考退化的 δ**。

```
half_width(w) = min( 障碍物半宽(w), 半宽上限, 曲率上限 )
曲率上限: 折叠侧 |δ| ≤ (1 − μ) / |κ(w)|      μ = min_regularity_ratio
```

- 折叠侧 = 指向曲率中心的一侧：κ>0 时曲线朝 +N 弯，只有 +N 侧会塌；外侧
  （|1−κδ|>1）不受限制，仍由障碍物自由距离决定。
- μ 保证 `‖r_w‖ ≥ μ·‖p_w‖`，于是 `w_dot ≤ k1(α+σ)/(μ‖p_w‖)` 有界。
- μ 取 `[0,1)` 之外即关闭（两侧上限为 +∞），便于对拍。

这不是新门控：它只**收窄**走廊在弯道处能够提供的 δ，直道上完全不生效；不触碰
`upper_nu`、分配器、运行层判据。M4E 的饱和执行继续作为兜底。

默认 μ = **0.35**（`phase_offset_tube/min_regularity_ratio`）。μ 越大越少发生相位
饱和，但弯道可用横向空间越小；μ→0.35 时 `κ=1 1/m`（半径 1 m）处 δ 上限为 0.65 m。

## 3. 实现范围（白名单）

1. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/section_tube.h`
   - `SectionBuildConfig` 增加 `min_regularity_ratio` / `curvature_epsilon`。
2. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_section_input.h`
   - `SectionScanConfig` 同上；
   - 新增纯函数 `sectionCurvatureOffsetCaps()`（折叠侧上限，可单测）。
3. `src/swarm_planner/bspline_traj/src/integration/phase_offset_section_input.cpp`
   - 配置校验增加两个字段；
   - 每个 knot 先算曲率上限，再与障碍物上限取 min（按侧）。
4. `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
   - 把 `section_build` 的这两个值传进 `scan_config`。
5. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
   - 读取 `phase_offset/tube/min_regularity_ratio`（默认 0.35）与
     `phase_offset/tube/curvature_epsilon`。
6. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
   - 构造默认值。
7. `src/swarm_planner/bspline_traj/launch/test_gvf.launch`
   - 新增 arg `phase_offset_tube_min_regularity_ratio`（默认 0.35）并下发。
8. `src/swarm_planner/bspline_traj/test/phase_offset_section_input_test.cpp`
   - 新增 3 条：纯函数折叠侧/镜像/关闭；弯曲路径扫描确实封顶；直道不被封顶。
9. `src/swarm_planner/phase_offset/phase_offset_sim_bringup/config/scenarios/sim_b_formation_3.yaml`
   - 目标行 `y=-10.0 → -11.2`（M4D §4 的目标行问题，见 §5）。

## 4. 验收

单元测试：

| 测试 | 结果 |
|---|---|
| `phase_offset_section_input_test`（含新增 3 条） | 35/35 |
| `phase_offset_matched_adapter_test` | 10/10 |
| `phase_offset_runtime_test` | 15/15 |
| `phase_offset_allocator_test` | 46/46 |
| `phase_offset_tube_viability_test` | 42/42 |
| `phase_offset_section_tube_test` | 32/32 |

柱阵场景（`sim_b_formation_3.yaml` + `pillar.pcd`，目标行 -11.2，30 s+ 采样）：

- `phase rate is outside the frozen limits`：**0 次**；
- `min_regularity_ratio` 运行时确认 0.35；
- 走廊半宽出现明显的**按侧收窄**（例：`left[min=0.150 mean=0.661] right[min=0.650
  mean=1.302]`），开阔段仍是 `max=1.450`（曲率项不生效）；
- 三机位置 x≈-1.16/-0.03/+0.75、y≈-11.10/-11.18/-11.03；uav_0/uav_1
  `[REACHED]`（0.187 / 0.039 m）。

## 5. 仍然存在（下一批）

uav_2 停在离目标 **0.30 m** 处：`no_valid_candidate`，`path_w_start=32.983
path_w_end=33.184`（前端只剩 0.20 w），`path_end_clamped_count=1` —— 前端已耗尽，
所有候选的 look-ahead 都超出路径末端；同时到点判据要求 `dist_xy ≤ 0.2 m`，0.30 m
既不算到点也没有候选 → 永久保持（M4E §7.3 已记录同类）。

建议：让"前端已耗尽"本身成为 terminal 条件（在**有界**半径内，例如
≤ `gvf/cmd/governor_l_min` 量级），复用现有 `residual_forced` 收尾机制，而不是
继续加宽到点半径。
