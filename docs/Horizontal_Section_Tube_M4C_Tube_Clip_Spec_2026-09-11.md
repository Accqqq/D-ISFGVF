# M4C：Tube 约束下的「按边界走」（横向能力不足不得否掉整条路径）

日期 2026-09-11。状态：已冻结待执行，用户已确认语义：

> 我的想法是在 tube 容许范围那就按意图走，如果容不下就按最大边界走。

依据：`AGENTS.md` 核心不变量 “Loss of nonzero transverse Tube capacity must not
by itself invalidate a planner-valid path”，以及 M4A 报告 6.12/6.13 的实测。

## 1. 问题

柱阵场景下三台机会停在 y≈13.8（开放空间则全程可飞）。打点结论：

- `preview` 判 `RATE_INFEASIBLE`（reachable boundary contraction/rate 不可行，
  `max_inward == allowed == 0.1250`，`boundary_tolerance = 0`）；
- 或 `alloc` 判 `NO_ADMISSIBLE_COMMAND`（`phase scalar admissible interval is
  empty`：`f_w0 ≈ 2.3` 超出窗口 `[0.02, 2.0]`，而 `u_w` 权限只有 ±0.12）。

两者都会让 `out.valid = false` → 发布位置保持 → 飞机停住。也就是说：**横向
（或相位）可行域变窄本身，把一条 planner-valid 的路径整条否掉了**，违反上面
那条不变量。

## 2. 目标语义（本批只做两条）

1. **意图在可行域内 → 按意图走**（现有行为，不变）。
2. **意图超出可行域 → 取可行域的边界**（本批新增），而不是失败：
   - 相位轴：`[lower_nu, upper_nu] - f_w0` 与 `[-u_w_abs_max, u_w_abs_max]`
     无交集时，取权限边界里最靠近窗口的那一端：
     `f_w0 > upper_nu → u_w = -u_w_abs_max`；`f_w0 < lower_nu →
     u_w = +u_w_abs_max`。
   - 横向轴：`preview.current_rate_interval` 与 `[-Δ, +Δ]`（`Δ =
     upper_u_delta`）无交集时，取该区间里离 0 最近的端点，再夹进 `[-Δ, +Δ]`。
3. **只有当前状态本身不可行时才保持 fail-closed**：`!preview.valid`、
   `!preview.feasible`（当前点没有走廊）、`!current_delta_inside`（δ 已在走廊
   外）继续失败。

## 3. 实现范围（白名单）

只允许改这两个文件：

1. `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_allocator.cpp`
   - `PreviewValidForAllocation()`：接受 `status != FEASIBLE` 的
     **rate-degraded** preview，前提是 `valid && feasible &&
     current_delta_inside`，且 `current_rate_interval` 的有限性/次序检查仍按
     原样执行（区间为空时允许 `valid=false`，横向轴走第 2 条规则）。
   - `allocate()`：上面两条"无交集即失败"改为按边界取值。
   - 保持不动：`FinalCommandValid`、`selectedUConsistent(0.0)`、
     `BuildSelectedScalar` 内部（static ∩ slew 为空的情形本批仍 fail-closed，
     见第 5 节）。
2. `src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_allocator_test.cpp`
   - 把原先断言 `NO_ADMISSIBLE_COMMAND` / `PREVIEW_INFEASIBLE` 的用例迁移为
     断言"取到边界值"（逐条记录映射），并新增至少两条边界用例。

3. 同一条规则必须在**上游两道同款闸门**上一起放宽，否则分配器的裁剪永远收不到
   rate-degraded 的 preview（实测确认：只改分配器时柱阵仍然停住）：
   - `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
     （`TubeViability::evaluate` 之后那处 `!preview.rate_feasible ||
     !preview.contraction_rate_feasible` 条件）；
   - `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp`
     （`prepareSection` 入口的同款条件）。
   两处都改为：接受 `status == FEASIBLE || status == RATE_INFEASIBLE`，且仍要求
   `valid && feasible && current_delta_inside`；其余检查（provenance、policy、
   section_profile 绑定等）保持不变。

不改：`tube_viability.*`（preview 的判定本身保持原样，只是不再被当作整条命令
的否决条件）、`gvf_manager.cpp`、地图、planner。

## 4. 验收

1. 隔离构建 `phase_offset_allocator_test` 通过；上述迁移用例逐条对应。
2. tube_viability / section_tube / matched adapter 等既有回归不退化。
3. 柱阵场景（`sim_b_formation_3.yaml` + `pillar.pcd`）三机应能通过柱阵段；
   被裁到边界时 δ 应停在走廊边界上（用 `/uav_n/phase_offset_section_tube`
   的左右边界与 odom 对比）。
4. 记录：裁剪发生的次数、δ 是否长期贴边、以及飞行的连续性。

## 5. 不在本批（明确留给下一批）

- `BuildSelectedScalar` 中 static 区间与 slew 区间无交集的情形（走廊收缩快于
  速率限幅）：本批仍 fail-closed。语义上是"安全集"与"参考连续性"冲突，需要
  单独决定哪一个优先（不能两头都保证）。
- preview 的 `RATE_INFEASIBLE` 数值边界（`max_inward == allowed`）本身是否
  存在 1-ULP 舍入问题：本批不改判据，只让它的结果不再否决整条命令。
