# gvf_manager FSM Notes

本文档只描述当前代码中 `gvf_manager` 状态机与闭合轨迹相关逻辑的真实生效行为。

## 1. FSM 主状态

状态定义在：
- [gvf_manager.h](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h#L205)

当前状态集合：
- `INIT`
- `WAIT_TARGET`
- `GEN_NEW_TRAJ`
- `REPLAN_TRAJ`
- `EXEC_TRAJ`

主状态机实现在：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2751)

## 2. 状态流转总览

当前主流程：
- `WAIT_TARGET -> GEN_NEW_TRAJ -> EXEC_TRAJ`
- `EXEC_TRAJ -> REPLAN_TRAJ -> EXEC_TRAJ`

触发条件：
- `WAIT_TARGET`
  - `receive_goal == true`
  - 或 `is_first_goal == true`
- `EXEC_TRAJ`
  - `checkCollision() == true`
  - 或达到 `planInterval`

## 3. WAIT_TARGET

位置：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2753)

行为：
- 等待普通目标点或闭合轨迹 auto-start 注入目标
- 一旦 `receive_goal` 或 `is_first_goal` 为真，就进入 `GEN_NEW_TRAJ`

## 4. GEN_NEW_TRAJ

位置：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2761)

行为：
- 调用 `astaropt()` 生成第一条执行轨迹
- 成功时保存：
  - `pm.last_traj`
  - `pm.last_vel`
  - `pm.last_traj_time_`
  - `current_traj_index_ = new_i0`
- 然后进入 `EXEC_TRAJ`

说明：
- 这里已经开始保存“旧轨迹时间”，供后续轨迹切换代价比较使用

## 5. EXEC_TRAJ

位置：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2781)

当前行为分三步：

### 5.1 更新当前轨迹索引

- 从 `current_traj_index_` 往后找当前几何最近点
- 更新 `current_traj_index_`

对应代码：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2782)

说明：
- 这是“前向最近点”策略，不会回退去搜更早索引

### 5.2 点到点终点退出

- 只有在 `circle_test` 关闭时才走普通终点判据
- 如果接近目标点，则退出到 `WAIT_TARGET`

对应代码：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2794)

### 5.3 重规划触发

优先级：
1. 如果起点变化不够大，则直接 return
2. 如果 `checkCollision()` 为真，则进入 `REPLAN_TRAJ`
3. 否则如果达到 `planInterval`，进入 `REPLAN_TRAJ`

对应代码：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2807)

## 6. REPLAN_TRAJ

位置：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2820)

这是当前最核心的状态。

### 6.1 重规划起点索引更新

当前逻辑：
- 若 `sample_w_` 可用，则先用 `progress_w_` 找 `progress_anchor_idx`
- 再从该 anchor 往后找几何最近点
- 用这个点更新 `current_traj_index_`

对应代码：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2827)

说明：
- 这保证重规划起点更贴近当前 lifted progress，而不是单纯几何最近点

### 6.2 旧轨迹锚点提取

辅助函数：
- `computeOldPathAnchor()`
- `computeKeepPathAnchor()`

对应代码：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2859)

作用：
- 接受新轨迹时，从旧轨迹上取 `w_anchor`
- keep old 时保留旧轨迹原有 `start_w`

### 6.3 候选轨迹切换判定

入口：
- `shouldAcceptCandidate(...)`

调用位置：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2896)

当前规则：
- 如果旧轨迹碰撞
  - 或旧轨迹已经走到后半段
  - 则直接 `accept_collision&timout`
- 否则比较代价
  - 只有当新轨迹比旧轨迹至少优 `10%`
  - 才接受新轨迹

### 6.4 接受新轨迹时

行为：
- 保存新轨迹：
  - `pm.last_traj`
  - `pm.last_vel`
  - `pm.last_traj_time_`
- 更新 `current_traj_index_ = new_i0`
- 调用：
  - `pm.gvf_->setNextPathWAnchor(w_anchor)`

对应代码：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2900)

说明：
- 这保证了新轨迹不会从 `w=0` 重新开始，而是延续旧轨迹进度

### 6.5 keep old / 规划失败时

行为：
- 不替换 `pm.last_traj`
- 用旧轨迹 `sample_w_.front()` 作为 keep old 的锚点
- 重新调用 `setNextPathWAnchor(w_anchor)`

对应代码：
- keep old: [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2917)
- 规划失败: [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L2928)

说明：
- keep old 时不会让 `w` 参数系继续漂移

## 7. 闭合轨迹相关状态机外逻辑

### 7.1 auto-start

入口：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L709)

行为：
- 在 `enable_circle_reference_test_ && circle_reference_auto_start_` 条件下
- 自动生成 circle / figure8 nominal reference
- 自动把 `start_pt / goal_pt / is_first_goal / receive_goal` 注入 manager

### 7.2 nominal reference 生成

函数：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L781)
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L817)

当前事实：
- 圆轨和 8 字都只生成位置参考
- `circle_reference_vel_` 当前全为零

### 7.3 nominal reference 点选择

函数：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L856)

当前逻辑：
- figure8 接轨阶段：
  - `JOIN`
- 稳定后：
  - 按 `progress_w_` 选 `phase_idx`
  - 局部搜索 `best_idx`
  - 再加 `lookahead`
- 必要时：
  - `REALIGN`

### 7.4 重要结论

- 闭合轨迹参考点推进不是按时间推进
- 当前是：
  - `progress_w_`
  - 当前空间位置
  - `JOIN / progress / REALIGN`
 共同决定参考点

## 8. checkCollision()

位置：
- [gvf_manager.cpp](/home/cxq/Sim_demo/gvf_4d/gvf_ws/src/swarm_planner/bspline_traj/src/gvf_manager.cpp#L1072)

当前行为：
- 先根据 `current_traj_index_` 与 `progress_w_` 确定检查起点
- 只检查未来 `collision_check_horizon_pts_` 个点
- 连续命中达到 `collision_consecutive_hits_` 才返回碰撞

说明：
- `collision_replan_cooldown_` 已经删除，不在当前行为中

## 9. 当前需要注意的状态机问题

- `accept_collision&timout` 会弱化“10% 改善才切换”的严格性
- 重规划时虽然会尝试回轨，但若：
  - 新轨迹被拒绝
  - 或搜索失败
  - 仍可能 keep old
- `EXEC_TRAJ` 的索引更新是前向最近点，不会向后回退搜索

## 10. 维护原则

以后修改 FSM 相关行为时，优先同步核对：
- `FSMCallback()`
- `shouldAcceptCandidate()`
- `checkCollision()`
- `setNextPathWAnchor()`
- `buildReparamTableFromPathMsg()`

如果文档与代码冲突，以代码为准，并更新本文档。
