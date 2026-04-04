# Iteration Memory

日期：`2026-04-04`

## Iteration Goal
- 稳定当前闭合轨迹实验版
- 保留 `circle/figure8 + auto_start + join + realign + w continuity`
- 减少记忆与当前代码不一致的问题

## Current True State
- 当前代码不是纯 `HEAD` 基线，而是一版恢复后的闭合轨迹实验版。
- 当前闭合轨迹参考点推进不是按时间推进，而是按：
  - `progress_w_`
  - 当前空间位置
  - `JOIN / progress` 规则
- 当前闭合轨迹 nominal reference 仍在使用：
  - `circle_test`
  - `auto_start`
  - `figure8 join mode`
  - `REALIGN`
- 当前闭合轨迹主参考已去掉切向速度：
  - `circle_reference_vel_` 全零
  - `gvf/circle_test/speed` 已删除
- 当前换轨规则：
  - 安全时：新轨迹代价至少优于旧轨迹 `10%` 才切换
  - 但碰撞或旧轨迹后半段时，会直接 `accept_collision&timout`
- 当前 `w` 是连续的：
  - 新轨迹通过 `w_anchor` 续接旧轨迹
  - 不会每次重规划都从 0 开始
- 当前代价时间计算已修正：
  - `J_old` 用旧轨迹时间
  - `J_new` 用新轨迹时间

## Recent Changes In This Iteration
- 恢复了闭合轨迹实验版：
  - `circle_test`
  - `auto_start`
  - `join mode`
  - `REALIGN`
- 修正了轨迹切换代价中的旧/新时间混用问题
- 确认了 `w` 续接链路
- 去掉了闭合轨迹切向速度
- 删除了无效参数：
  - `gvf/circle_test/speed`
  - `collision_replan_cooldown_`

## Active Concerns
- 回轨时虽然会重规划，但如果：
  - 新轨迹被 `reject_worse_new`
  - 或搜索失败 `keep old`
  - 仍可能沿旧轨迹继续推进
- `accept_collision&timout` 目前会弱化“10% 改善才切换”的严格性
- figure8 接轨与回轨时仍需观察是否会贴障

## Current File Focus
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
  - 当前最核心
- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/src/gvf.cpp`
- `src/swarm_planner/bspline_traj/launch/test_gvf.launch`

## Current Effective Params
- `gvf/circle_test/enable`
- `gvf/circle_test/auto_start`
- `gvf/circle_test/shape`
- `gvf/circle_test/radius`
- `gvf/circle_test/figure8_radius`
- `gvf/circle_test/height`
- `gvf/circle_test/points`
- `gvf/circle_test/lookahead_pts`
- `gvf/circle_test/realign_min_progress`
- `gvf/circle_test/join_search_window`
- `gvf/circle_test/join_lookahead_pts`
- `gvf/circle_test/join_exit_dist`
- `gvf/circle_test/join_exit_stable_needed`
- `gvf/circle_test/center_x`
- `gvf/circle_test/center_y`
- `gvf/circle_test/center_z`
- `gvf/collision_threshold`
- `gvf/collision_check_horizon_pts`
- `gvf/collision_consecutive_hits`
- `gvf/planInterval`

## Removed In This Iteration
- `gvf/circle_test/speed`
- `circle_reference_speed_`
- `collision_replan_cooldown_`
- `last_collision_replan_time_`

## Immediate Next Tasks
1. 继续收敛重规划切换规则，明确何时必须切、何时可以 keep old。
2. 继续观察回轨时的贴障行为，必要时加安全优先策略。
3. 确认这版代码要不要提交到 GitHub，并明确提交范围。

## Short Validation Checklist
- `catkin_make --pkg bspline_race -j1`
- 启动 `test_gvf.launch`
- 看 `goal/auto_start`
- 看 `FSM`
- 看 `[GVF][REF] / [GVF][REF][JOIN] / [GVF][SWITCH] / [GVF][W]`
