# Long-Term Project Memory

## Project Identity
- Name: `Lifted_GVF`
- One-line summary: 基于 `KinoA* + B-spline + GVF` 的无人机局部规划与闭合轨迹跟踪实验项目。
- Primary long-term goal: 在现有规划框架上稳定验证 `lifted progress / closed reference` 的闭合轨迹跟踪能力，尤其是 circle / figure8。
- Current long-term direction:
  - 保留现有 `KinoA* + B-spline` 主体
  - 在此之上逐步稳定闭合轨迹、`closed_ref_w_` 相位推进、`w` 连续性与切换逻辑
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
  - `src/swarm_planner/plan_env/src/sdf_map.cpp`
    - 实时感知地图、ESDF、手动地图层、碰撞/边界查询底层实现
- Runtime config:
  - `src/swarm_planner/bspline_traj/launch/test_gvf.launch`
  - `src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch`

## Architecture Snapshot
- Main flow:
  - `goal/auto-start -> closed reference goal candidate -> gvf_manager FSM -> KinoA* -> B-spline -> gvf reparam(sample_w_) -> cmdCallback lifted guidance`
- Persistent state:
  - `pm.last_traj / pm.last_vel / pm.last_traj_time_`
  - `pm.gvf_->sample_w_`
  - `circle_reference_traj_ / circle_reference_vel_ / circle_reference_w_ / circle_reference_total_w_`
  - `closed_ref_w_`
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
  - circle nominal reference 生成，并累计弧长表 `circle_reference_w_`
- `gvf_manager.cpp -> generateFigureEightReference()`
  - figure8 nominal reference 生成，并累计弧长表 `circle_reference_w_`
- `gvf_manager.cpp -> pointFromClosedW() / tangentFromClosedW()`
  - 把连续闭合相位 `closed_ref_w_` 映射回闭合曲线上的插值点和切向
- `gvf_manager.cpp -> getCircleReferenceGoal()`
  - 更新 `closed_ref_w_`，根据当前位置、切向误差和局部投影维护闭合曲线相位
- `gvf_manager.cpp -> selectClosedGoalCandidate()`
  - 基于 `lookahead_min_w / max_w / step_w` 构建候选目标；正常时围绕期望前瞻评分选择，参考线前方有障碍物时可将期望目标推到障碍物后方，再用 KinoA* 选择最低分的完整可达目标
- `gvf_manager.cpp -> buildClosedLookaheadCandidates()`
  - 构建闭合曲线弧长前视候选，不再使用旧 `lookahead_pts`
- `gvf_manager.cpp -> checkCollision()`
  - 执行轨迹未来段碰撞检查
- `sdf_map.cpp -> manualObstacleCallback()`
  - 手动障碍物入口，接收 `/manual_map/add_obstacle_center` 或仿真直接接收 `/clicked_point`
- `sdf_map.cpp -> applyManualLayer()`
  - 将 `manual_occupancy_buffer_` OR 回实时 occupancy / inflated occupancy，保证手动障碍物不被 raycast 或 buffer refresh 清掉
- `sdf_map.cpp -> loadManualMapFile() / saveManualMapFile()`
  - 手动障碍物中心的持久化读写，文件格式为每行 `x y z`
- `gvf_manager.cpp -> shouldAcceptCandidate()`
  - 新旧轨迹切换规则
- `gvf_manager.cpp -> FSMCallback()`
  - `WAIT_TARGET / GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ`
- `gvf.cpp -> setNextPathWAnchor()`
  - 为新轨迹注入 `w_anchor`
- `gvf.cpp -> buildReparamTableFromPathMsg()`
  - 构建 `sample_w_`，延续 `w`

## Long-Term Truths
- 闭合轨迹的核心不是“按时间播点”，而是先把闭合点列参数化成弧长 `w`，再用 `closed_ref_w_` 维护闭合曲线相位，并按弧长前视候选选目标。
- 闭合目标候选不能使用“最远完整成功即接受”：当前规则是延续已接受前瞻或使用 `goal_prefer_lookahead_w`，必要时跨过参考线上的障碍物，再综合前瞻偏差和 Kino 终点误差选最低分候选。这样避免目标过近停在障碍物前，也避免无条件选择远点造成闭合轨迹走捷径。
- `closed_ref_w_` 是连续相位，允许超过一圈；访问闭合曲线点时再通过 `wrapClosedW()` 回绕到当前圈。
- `w` 连续性来自：
  - 旧轨迹锚点提取
  - 新轨迹 `sample_w_` 的续接
- figure8 的难点不是生成轨迹，而是：
  - 闭合相位稳定
  - 交叉点支路选择
  - 重规划切换时的分支稳定性
- 这个项目里需要明确区分：
  - `lifted GVF` 核心能力
  - `closed_ref_w_ / candidate lookahead / switch` 工程稳定化
- 当前飞控命令层是 governor-only position control：
  - 不再使用旧 `CMD_DIRECT / CMD_SPLIT / CMD_POS_FF / CMD_MOTION_LIMIT` fallback
  - `PositionCommand.velocity` 显式保持 0
  - GVF 输出是内部速度意图，不直接发给飞控速度接口
  - governor 从当前执行轨迹的 `sample_w_` 上选前瞻点，把速度意图转换为位置命令
  - 速度转位置关系为 `v_actual ~= K_eq * cmd_dist`
  - 仿真常用 `K_eq ~= 1.65`；实机测得/采用 `K_eq ~= 1.10`
  - 实机若目标速度为 `2.0 m/s`，需要位置前瞻约 `2.0 / 1.10 = 1.82 m`
  - 实机 2m/s 初始 governor 参数应围绕 `governor_l_max / governor_lead_max ~= 1.7~1.9` 调整，而不是使用仿真 `K_eq=1.65` 下的较小前瞻
- `gain_test` 模式用于测位置环等效增益：
  - 打开后 `cmdCallback()` 直接发布 `odom + fixed lead`
  - 用 `odom_vel_lpf dot lead / |lead|^2` 估计 `K_eq`
  - 该模式不走正常 GVF 控制链路
- 手动地图层是 SDFMap 层能力，不属于 GVF 核心算法：
  - 手动障碍物进入 `manual_occupancy_buffer_`
  - 再 OR 到 `occupancy_buffer_ / occupancy_buffer_inflate_`
  - 规划器继续通过 `getInflateOccupancy()`、`getDistance()`、`isInMap()` 感知障碍物和边界
  - 不需要重写 GVF、KinoA* 或 B-spline
- 手动地图层的持久化只保存障碍物中心：
  - `manual_map_file` 是文本文件
  - `#` 开头为注释
  - 每个非注释行是一个障碍物中心 `x y z`
  - 加载时重新按当前 `manual_obstacle_radius / height / inflate` 生成圆柱
  - 该坐标依赖固定 `world` 原点；实机 VIO/map 原点变化时不能直接复用旧文件坐标

## Effective Parameters
- Closed reference:
  - `gvf/circle_test/enable`
  - `gvf/circle_test/auto_start`
  - `gvf/circle_test/shape`
  - `gvf/circle_test/radius`
  - `gvf/circle_test/figure8_radius`
  - `gvf/circle_test/height`
  - `gvf/circle_test/points`
  - `gvf/circle_test/lookahead_min_w`
  - `gvf/circle_test/lookahead_max_w`
  - `gvf/circle_test/lookahead_step_w`
  - `gvf/circle_test/search_back_w`
  - `gvf/circle_test/search_forward_w`
  - `gvf/circle_test/ref_phase_k1`
  - `gvf/circle_test/ref_alpha_rho`
  - `gvf/circle_test/ref_sigma_scale`
  - `gvf/circle_test/ref_wdot_forward_max`
  - `gvf/circle_test/ref_wdot_backward_max`
  - `gvf/circle_test/ref_project_blend`
  - `gvf/circle_test/ref_project_snap_max`
  - `gvf/circle_test/ref_project_boundary_eps`
  - `gvf/circle_test/goal_full_success_tol`
  - `gvf/circle_test/center_x/y/z`
- Control layer:
  - `gvf/cmd/vel_max`
  - `gvf/cmd/pos_gain_equiv`
  - `gvf/cmd/tangent_vel_max`
  - `gvf/cmd/acc_max`
  - `gvf/cmd/governor_l_min`
  - `gvf/cmd/governor_l_max`
  - `gvf/cmd/governor_l_step`
  - `gvf/cmd/governor_l_rate_max`
  - `gvf/cmd/governor_l_ff_weight`
  - `gvf/cmd/governor_lead_max`
  - `gvf/cmd/governor_normal_cross_max`
  - `gvf/cmd/governor_normal_deadband`
  - `gvf/cmd/governor_normal_full_error`
  - `gvf/cmd/governor_normal_max`
  - `gvf/cmd/governor_normal_rate_max`
  - `gvf/cmd/governor_l_rate_weight`
  - `gvf/cmd/governor_normal_weight`
  - `gvf/cmd/governor_normal_rate_weight`
  - `gvf/cmd/governor_tau_vel_weight`
  - `gvf/cmd/governor_normal_vel_weight`
  - `gvf/cmd/governor_normal_vel_error_cap`
  - `gvf/switch/governor_path_margin_w`
  - `gvf/cmd/gain_test_enable`
  - `gvf/cmd/gain_test_lead`
  - `gvf/cmd/gain_test_axis`
- Replan / collision:
  - `gvf/collision_threshold`
  - `gvf/collision_check_horizon_pts`
  - `gvf/collision_consecutive_hits`
  - `gvf/planInterval`
- Manual map:
  - `sdf_map/enable_manual_map`
  - `sdf_map/manual_click_direct`
  - `sdf_map/manual_obstacle_radius`
  - `sdf_map/manual_obstacle_height`
  - `sdf_map/manual_obstacle_inflate`
  - `sdf_map/manual_boundary_padding`
  - `sdf_map/manual_boundary_z_min`
  - `sdf_map/manual_boundary_z_max`
  - `sdf_map/manual_map_file`
  - `sdf_map/manual_map_auto_load`
  - `sdf_map/manual_map_auto_save`

## Manual Map Runtime Notes
- 仿真 `test_gvf.launch`：
  - `manual_click_direct=true`
  - RViz `Publish Point` 可直接生成障碍物
  - 当前默认保存文件为 `src/swarm_planner/bspline_traj/config/manual_obstacles_test_gvf.txt`
- 实机建议：
  - `manual_click_direct=false`
  - `uav_server` 只在 `MANUAL_MAP` 状态下转发 `/clicked_point` 到 `/manual_map/add_obstacle_center`
  - 保存文件建议放在实机 workspace 的 `bspline_traj/config/manual_maps/realflight_obstacles.txt`
- 当前默认圆柱占据半径等于：
  - `manual_obstacle_radius + manual_obstacle_inflate`
  - 仿真示例为 `0.35 + 0.10 = 0.45 m`
- 若局部深度图暂时看不到障碍物，手动障碍物仍会保留在 SDFMap 查询结果里；但如果实机坐标系或 RViz Fixed Frame 不一致，手动点和真实感知点会在 RViz 上错位。

## Removed / Deprecated Items
- 已移除：
  - `gvf/circle_test/speed`
  - `circle_reference_speed_`
  - `collision_replan_cooldown_`
  - `last_collision_replan_time_`
- 当前废弃/旧版残留，不应再作为有效调参：
  - `gvf/cmd/lead_max`
  - `gvf/cmd/use_pos_ff`
  - `gvf/cmd/pos_ff_xy_only`
  - `gvf/cmd/pos_ff_time`
  - `gvf/cmd/pos_ff_max`
  - `gvf/cmd/use_vel_slew_limit`
  - `gvf/cmd/use_switch_tangent_vel_limit`
  - `gvf/cmd/use_split_normal_lead`
  - `gvf/cmd/normal_lead_*`
  - `gvf/cmd/normal_des_boost_*`
  - `gvf/circle_test/lookahead_pts`
  - `gvf/circle_test/realign_min_progress`
  - `gvf/circle_test/join_*`
  - `figure8_join_*`
  - `gvf/kino_sample_ts`
  - `gvf/kino_sample_ts_min`
  - `gvf/kino_max_guide_pts`
  - `gvf/path_pub_from_current`
  - `gvf/path_pub_future_pts`
  - `gvf/replan_stop_radius`
  - `gvf/replan_stop_vel`
  - `gvf/debug_gate`

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
  3. `[GVF][CLOSED_REF] / [GVF][CLOSED_GOAL] / [GVF][KINO_RESULT] / [GVF][SWITCH] / [GVF][CMD_POS_FF]`

## Git Notes
- 当前项目一旦有可运行版本，应尽快 commit。
- 如果需要保多个实验版本，优先用：
  - branch
  - tag
  - commit
