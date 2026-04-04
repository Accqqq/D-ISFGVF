# Long-Term Project Memory

## Project Identity
- Name: `Lifted_GVF`
- One-line summary: 基于 `KinoA* + B-spline + GVF` 的无人机局部规划与闭合轨迹跟踪实验项目。
- Primary long-term goal: 在现有规划框架上稳定验证 `lifted progress / closed reference` 的闭合轨迹跟踪能力，尤其是 circle / figure8。
- Current long-term direction:
  - 保留现有 `KinoA* + B-spline` 主体
  - 在此之上逐步稳定闭合轨迹、`w` 连续性与切换逻辑
- Non-goals:
  - 当前不整体重构规划架构
  - 当前不优先引入新依赖

## Repo Map
- Entry point:
  - `src/swarm_planner/bspline_traj/src/formation_planning.cpp`
- Core modules:
  - `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
    - 规划主流程、FSM、闭合轨迹 nominal reference、重规划与切换
  - `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
    - `gvf_manager` 状态、参数、接口
  - `src/swarm_planner/bspline_traj/src/gvf.cpp`
    - GVF / lifted guidance / `sample_w_` 重参数化
  - `src/swarm_planner/path_searching/src/kinodynamic_astar.cpp`
    - KinoA* 搜索与引导点采样
- Runtime config:
  - `src/swarm_planner/bspline_traj/launch/test_gvf.launch`
  - `src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch`

## Architecture Snapshot
- Main flow:
  - `goal/auto-start -> gvf_manager FSM -> KinoA* -> B-spline -> gvf reparam(sample_w_) -> cmdCallback lifted guidance`
- Persistent state:
  - `pm.last_traj / pm.last_vel / pm.last_traj_time_`
  - `pm.gvf_->sample_w_`
  - `circle_reference_traj_ / circle_reference_vel_ / circle_reference_w_`
- Boundaries:
  - `gvf_manager.cpp` 决定何时规划、切不切轨、闭合参考如何选点
  - `gvf.cpp` 决定当前执行轨迹如何转成连续 `w`
  - `kinodynamic_astar.cpp` 负责搜索，不负责 `w` 连续性

## Behavior Anchors
- `formation_planning.cpp -> main()`
  - ROS 节点入口
- `gvf_manager.cpp -> goalCallback()`
  - 点到点目标 / 手动闭合轨迹目标入口
- `gvf_manager.cpp -> odomCallback()`
  - `circle_test + auto_start` 入口
- `gvf_manager.cpp -> generateCircleReference()`
  - circle nominal reference 生成
- `gvf_manager.cpp -> generateFigureEightReference()`
  - figure8 nominal reference 生成
- `gvf_manager.cpp -> getCircleReferenceGoal()`
  - `JOIN / progress / REALIGN` 目标点选择核心
- `gvf_manager.cpp -> checkCollision()`
  - 执行轨迹未来段碰撞检查
- `gvf_manager.cpp -> shouldAcceptCandidate()`
  - 新旧轨迹切换规则
- `gvf_manager.cpp -> FSMCallback()`
  - `WAIT_TARGET / GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ`
- `gvf.cpp -> setNextPathWAnchor()`
  - 为新轨迹注入 `w_anchor`
- `gvf.cpp -> buildReparamTableFromPathMsg()`
  - 构建 `sample_w_`，延续 `w`

## Long-Term Truths
- 闭合轨迹的核心不是“按时间播点”，而是“按 progress 选参考点”。
- `w` 连续性来自：
  - 旧轨迹锚点提取
  - 新轨迹 `sample_w_` 的续接
- figure8 的难点不是生成轨迹，而是：
  - 接轨
  - 交叉点支路选择
  - 重规划切换时的分支稳定性
- 这个项目里需要明确区分：
  - `lifted GVF` 核心能力
  - `join / realign / switch` 工程稳定化

## Effective Parameters
- Closed reference:
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
  - `gvf/circle_test/center_x/y/z`
- Replan / collision:
  - `gvf/collision_threshold`
  - `gvf/collision_check_horizon_pts`
  - `gvf/collision_consecutive_hits`
  - `gvf/planInterval`

## Removed / Deprecated Items
- 已移除：
  - `gvf/circle_test/speed`
  - `circle_reference_speed_`
  - `collision_replan_cooldown_`
  - `last_collision_replan_time_`

## Working Conventions
- Preserve existing architecture unless explicitly asked otherwise.
- Prefer minimal localized diffs.
- Reuse current FSM / replan / GVF patterns before introducing new abstractions.
- Do not add dependencies unless necessary.
- Do not change public interfaces unless explicitly required.

## Known Pitfalls
- `DO NOT RECEIVE GOAL` 只说明 `receive_goal == false`，不等于 topic 连线一定错。
- `rostopic info` 有 publisher/subscriber，不等于 callback 一定执行。
- 之前误用过 `git restore` 覆盖未提交改动；不要依赖“未提交版本还能恢复”。
- 旧文档可能描述的是更早版本，不能直接当当前代码事实。

## Verification Rules
- Primary build command:
  - `catkin_make --pkg bspline_race -j1`
- Manual verification focus:
  1. `goalCallback()` / `auto_start` 是否触发
  2. `FSM` 是否进入 `GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ`
  3. `[GVF][REF] / [GVF][REF][JOIN] / [GVF][SWITCH] / [GVF][W]`

## Git Notes
- 当前项目一旦有可运行版本，应尽快 commit。
- 如果需要保多个实验版本，优先用：
  - branch
  - tag
  - commit
