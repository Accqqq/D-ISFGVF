# M4E：相位速率饱和执行（uav_2 原地保持解冻）

日期 2026-09-11。状态：已执行（本文档含执行结果）。

## 1. 问题（uav_2 停在半路）

柱阵+3 机场景里 uav_2 停在 `phase_w=9.341`、离目标 25.64 m，命令层给出
`final_cmd_source=GOVERNOR_INVALID_HOLD reason=guidance_invalid
fallback_hold_pos=1`。即：不是到点，而是 Section 输出被判无效后 governor 原地保持。

直接原因（同一 tick 紧邻日志）：

```
phase_offset_matched_adapter.cpp: prepare valid=0
  reason='Section MatchedPort phase rate is outside the frozen limits'
```

该判据在 `phase_offset_runtime.cpp` 的 `prepareSection` 内
（`matched_output.w_dot` 必须落在 `[limits.lower_nu, limits.upper_nu]` 且同时落在
`preview.policy` 的同一窗口内）。

## 2. 为什么这是缺陷而不是"该停"

M4C 已冻结的用户语义是：**意图超出可行域时取可行域边界，不丢 tick**。分配器已按此
实现相位轴：`[lower_nu, upper_nu] - f_w0` 与 `[-u_w_abs_max, u_w_abs_max]` 无交集
时，取权限边界（`f_w0 > upper_nu → u_w = -u_w_abs_max`），并置
`output.phase_window_clipped = true`。

但 `prepareSection` 随后用**同一个窗口**再判一次，把这个已被批准的饱和结果重新
判死。结果是：

1. 分配器饱和 → 已选端口 `u_w = ∓u_w_abs_max`；
2. runtime 二次拒绝 → `out.valid = false`；
3. governor 无有效候选 → 原地保持；
4. 相位不再前进 → 同一 w 上永久重演（uav_2 实测 `w` 冻结）。

这同时违反 `AGENTS.md` 核心不变量：
“A Tube, preview, staging, or offset-authority failure alone must not cause
persistent HOLD of an otherwise planner-valid navigation task.”

相位速率窗口本身是**代理约束**：`w_dot = K1*(alpha+sigma)/||r_w||`，参考点的
笛卡尔速度是 `||r_w||·w_dot = K1*(alpha+sigma)`。所以当连接段/弯道上
`||r_w||` 偏离 1 时，`w_dot` 会离开按 `||r_w||≈1` 标定的窗口，而参考点速度仍
正常；此时 `w_dot` 数值本身不构成物理风险，物理速度上限由 governor
（`cmd_vel_max`）单独保证。

## 3. 本批语义（只做两条）

1. **饱和执行**：当且仅当分配器确实无法把基座相位速率压回窗口
   （`phase_window_clipped`），且所选 `u_w` 正落在对应的权限边界上时，
   `prepareSection` 接受该饱和速率，不再二次判死。
2. **非倒退优先**：饱和方向为"速率过低"且饱和后速率仍为负时（参考点想后退，
   说明机体系落后于参考），本 tick 把相位推进率夹到 0：相位坐标保持，横向端口、
   物理指令、`delta` 提交值仍是分配器的饱和端口，整 tick 不丢。

不改：tube 截面构建、`delta` 走廊裁剪、`FinalCommandValid`、
`selectedUConsistent`、slew/幅值、ZOH、provenance、policy、切向速度下限、
V2 链路。

## 4. 实现范围（白名单）

1. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h`
   - `RuntimeSectionPrepareInput` 增加 `bool phase_window_saturated = false;`。
2. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
   - 构造 `RuntimeSectionPrepareInput` 时透传 `allocator.phase_window_clipped`。
3. `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp`
   - `prepareSection`：相位窗口判据在"确认为分配器饱和"时放行；
   - 饱和且速率为负时把执行相位推进率夹到 0，并保持 matched 值自洽。
4. `src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp`
   - 新增两条：上界饱和被接受（`w_dot > upper_nu`、`next_w > w`）；
     下界饱和且速率为负 → 接受且 `next_w == w`、`matched.w_dot == 0`；
   - 反向对照：未标记饱和的越窗输入仍然失败。

## 5. 验收

1. `phase_offset_runtime_test`、`phase_offset_allocator_test`、
   `phase_offset_tube_viability_test`、`phase_offset_section_tube_test`、
   `phase_offset_matched_adapter_test` 全通过（无迁移、无放宽其他判据）。
2. 柱阵场景（`sim_b_formation_3.yaml` + pillar）：
   `prepare valid=0 reason='Section MatchedPort phase rate is outside the frozen
   limits'` 计数为 0；三机穿越柱阵到达目标；无 `GOVERNOR_INVALID_HOLD`。
3. 记录：饱和发生次数、是否长期贴边、飞行连续性。

## 6. 不在本批

- V2 链路（`tube_execution_v2.cpp`）的同名窗口判据保持原样。
- `upper_nu` 参数本身不调（论文值 3.0 不动）。
- 连接段几何（挤压）本身不重构。

## 7. 执行结果

### 7.1 单元测试

| 测试 | 结果 |
|---|---|
| `phase_offset_runtime_test`（含新增用例） | 15/15 |
| `phase_offset_allocator_test` | 46/46 |
| `phase_offset_tube_viability_test` | 42/42 |
| `phase_offset_section_tube_test` | 32/32 |
| `phase_offset_matched_adapter_test` | 10/10 |

附带修复（验证所需，非语义改动）：`runtime_test.cpp` 里只被退役 runtime 用例使用的
`Refresh()`/`Complete()` 两个 helper 原先未跟随 `PHASE_OFFSET_RUNTIME_V2_TEST_ONLY`
守卫，导致 `phase_offset_runtime_test` 在本批之前就已链接失败（未定义
`refreshPreflight`/`complete`，二者已从生产库编译掉）。两个 helper 现在与使用它们的
用例同守卫，V2 目标恢复可构建。

### 7.2 柱阵场景 A/B（`sim_b_formation_3.yaml` + `pillar.pcd`，3 机，无 rviz）

同一场景、同一起点，只切换本批的饱和放行开关：

| 指标 | 放行关闭（= 本批之前） | 放行开启（本批之后） |
|---|---|---|
| `Section MatchedPort phase rate is outside the frozen limits` | 68 / 106 次 | **0 次** |
| `GOVERNOR_INVALID_HOLD ... guidance_invalid` | 持续（相位冻住） | 0 次（诊断类 hold 仅出现在起飞前/到点后） |
| uav_2 位置 | 冻在 y≈+10.5，离目标 20.5 m | 穿到 y≈−9.2 ~ −10.09 |

结论：本批确实解掉了"相位速率越窗 → 二次判死 → 原地保持 → 相位永久冻结"这条链路。

### 7.3 仍然存在的、与本批无关的停住（待下一批）

本批之后 uav_2 偶尔仍停在离目标 0.3~0.8 m 处，机制与本批无关且在本批之前就存在：

- 当前场景目标行为 `y = -10.0`（`x = 0` / `+1` 落在柱体膨胀里），即 M4D §4 已记录的
  旧目标行；实测本批运行 `no path` 268 次、`kinodynamic search fail` 134 次，
  `remaining_w=0.011`，规划器无法把前端补到终点；
- 于是 governor 的 `best_query_w` 已贴住 `path_w_end`（窗口仅 ~0.21 w），36 个候选
  `valid_count=0` → `no_valid_candidate`；
- 而"到点"判据要求 `dist_xy ≤ 0.2 m`，飞机在 0.3~0.8 m 处既不算到点、也没有可用
  候选 → 永久保持。

建议下一批二选一或同时做：

1. 按 M4D §4 的方案把场景目标行改回自由行（`sim_b_formation_3.yaml` → `y=-11.2`）；
2. 让"路径已耗尽"本身成为 terminal 条件：当所有候选都因 `path_end_clamped` 被剔除
   且跟踪误差有限时，用路径末端的 terminal attractor 收尾，而不是无限保持。

## 8. 附带：集群意图权限提升（swarm 链路专用）

用户要求前向与横向意图都要更强。改动只落在 swarm 链路
`phase_offset_agent.launch`（单机 `bspline_race/test_gvf.launch` 的 0.12/0.25/0.6/1.2
保持不变）：

| 参数 | 旧值 | 新值 | 作用 |
|---|---|---|---|
| `phase_offset/manual/u_w_abs_max` | 0.12 | **0.40** | 路径进度速率偏置幅值 [w/s] |
| `phase_offset/manual/u_w_rate_max` | 0.60 | **2.00** | 该偏置的 slew [(w/s)/s] |
| `phase_offset/manual/u_delta_abs_max` | 0.25 | **0.80** | 参考横移速率 [m/s] |
| `phase_offset/manual/u_delta_rate_max` | 1.20 | **3.00** | 横移 slew [m/s²] |

实测（柱阵 3 机，30 s 采样）：`/uav_2/phase_offset_adjusted_path` 与
`/uav_2/optimization_path` 的偏差最大 **1.216 m**（p90 1.11 m），而调整前终点
`delta` 只有 ~0.2 m；同时相位窗口拒绝 0 次、无冻结。物理包络仍由
`gvf/cmd/vel_max=2.0` 与 tube 截面拥有；δ 的**范围**仍由 tube 半宽决定（当前
`environment_search_extent=1.5`、`section_clearance=0.15`）。

## 9. 待办（建议的下一批）：走廊的曲率上限

§7.3 的"预留"与本批的 `‖r_w‖` 塌陷同源，建议单独一批处理：在 section 走廊扫描里
按局部曲率给每侧加一个几何上限 `|δ| ≤ (1-μ)/|κ(w)|`（μ≈0.35~0.4），使 tube 不再
提供会让参考点退化的 δ。理由与实现位置见本批讨论记录；当前
`phase_offset_section_input.cpp` 的截面扫描没有任何曲率项，
`minimum_reference_speed` 仍为 1e-8。
