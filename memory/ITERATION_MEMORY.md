# Iteration Memory

日期：`2026-06-02`

## Iteration Goal
- 稳定当前闭合曲线跟踪实验版。
- 当前重点是闭合曲线 `w` 重参数化、候选目标选择、轨迹切换后的 GVF 控制输出。
- 清理旧版 `figure8 join / lookahead_pts / realign_min_progress` 残留，避免参数和代码事实不一致。

## Current True State
- 当前闭合曲线模式仍由 `gvf/circle_test/enable + auto_start + shape` 触发。
- `circle` / `figure8` 参考轨迹仍由 `generateCircleReference()` / `generateFigureEightReference()` 生成。
- 生成出来的闭合参考点会先做弧长参数化，构建：
  - `circle_reference_traj_`
  - `circle_reference_w_`
  - `circle_reference_total_w_`
- 后续闭合参考不直接按点号追踪，而是通过：
  - `wrapClosedW()`
  - `indexFromClosedW()`
  - `pointFromClosedW()`
  - `tangentFromClosedW()`
  把连续弧长相位 `closed_ref_w_` 映射回闭合曲线上的点和切向。
- 当前闭合参考不是旧版按 `lookahead_pts` 点数前视，也不是旧版 `figure8 join mode`。
- 当前闭合参考核心是：
  - `closed_ref_w_` 作为闭合曲线相位/弧长进度
  - 首次进入时 `findInitialClosedPhase()` 根据当前位置投影到闭合曲线，初始化 `closed_ref_w_`
  - `updateClosedRefPhaseByDynamics()` 用真实位置、切向误差和局部投影推进 `closed_ref_w_`
  - `buildClosedLookaheadCandidates()` 用 `lookahead_min_w / max_w / step_w` 构建候选目标
  - `selectClosedGoalCandidate()` 从候选目标里试 kino 规划并选择可达目标
- 当前 `figure8_join_*` 成员已删除，`gvf_manager.cpp` 中对应 reset 引用也已删除。
- 当前 `generateFigureEightReference()` 中旧的未使用 `cx/c2` 也已删除。

## Closed Curve Reparam / Goal Selection
- 当前闭合曲线重参数化链路：
  1. `generateCircleReference()` / `generateFigureEightReference()` 生成固定高度闭合点列。
  2. 累计相邻点距离得到 `circle_reference_w_`，并把首尾闭合段计入 `circle_reference_total_w_`。
  3. `closed_ref_w_` 是连续相位，可以超过一圈；访问点时用 `wrapClosedW()` 回绕到 `[0, total_w)`。
  4. `pointFromClosedW()` 在线段上插值，`tangentFromClosedW()` 给相位推进和日志用。
  5. `getCircleReferenceGoal()` 每次根据真实 odom 更新 `closed_ref_w_`，再取 `closed_ref_w_ + lookahead_w` 作为 nominal 目标。
  6. `selectClosedGoalCandidate()` 会对多个 `lookahead_w` 试 KinoA*，围绕期望前瞻距离对完整成功候选评分，失败时可用 end distance 最小的 partial 备选。

### Closed Goal Candidate Scoring Update (`2026-07-10`)
- `selectClosedGoalCandidate()` 已不再使用旧的 `track_far_first` 策略，也不再找到第一个完整成功候选后立即 `break`。
- 当前闭合轨迹目标选择流程：
  1. 用 `goal_prefer_lookahead_w` 给出正常情况下的期望前瞻距离；已有已接受目标时优先延续上次 `closed_ref_accepted_lookahead_w_`，减少重规划目标跳变。
  2. 将期望前瞻限制在 `lookahead_min_w ~ lookahead_max_w` 候选范围内。
  3. `goal_push_past_obstacle=true` 时，沿闭合参考线向前查询 `SDFMap::getInflateOccupancy()`；若遇到障碍物，将期望前瞻推进到 `obstacle_delta_w + goal_obstacle_pass_margin_w`，但不超过 `lookahead_max_w`。
  4. 候选点按与 `desired_lookahead` 的距离从近到远排序，所有候选都执行 KinoA* 尝试。
  5. 对所有完整成功候选计算：`score = goal_lookahead_weight * abs(lookahead - desired_lookahead) + goal_end_dist_weight * end_to_goal_dist`，选择最低分，而不是最远候选。
  6. 若没有完整成功候选，仍保留 Kino 终点距离目标最近的 partial fallback。
- 新增有效参数：`goal_prefer_lookahead_w / goal_lookahead_weight / goal_end_dist_weight / goal_push_past_obstacle / goal_obstacle_check_step_w / goal_obstacle_pass_margin_w`。
- 当前仿真值：`lookahead_min/max/step = 0.5/3.0/0.25`，`goal_prefer_lookahead_w=1.0`，score 权重为 `5.0/20.0`，障碍物后推开启，检测步长 `0.1 m`，越障碍裕量 `0.8 m`。
- `[GVF][CLOSED_GOAL]` 日志新增 `desired_lookahead / obstacle_delta_w / desired_pushed / selected_score`。
- 该修改只影响闭合轨迹 `selectClosedGoalCandidate()`，不影响普通点到点导航，也不改变 KinoA*、B-spline 或 lifted GVF 核心算法。
- 当前有效闭合曲线参数：
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
  - `gvf/circle_test/center_x/y/z`
- 当前可选/有效的 closed-ref 调参：
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
- 旧版参数当前无效或应清理：
  - `gvf/circle_test/lookahead_pts`
  - `gvf/circle_test/realign_min_progress`
  - `gvf/circle_test/join_search_window`
  - `gvf/circle_test/join_lookahead_pts`
  - `gvf/circle_test/join_exit_dist`
  - `gvf/circle_test/join_exit_stable_needed`

## Control Layer Current State
- 当前真实执行的 `cmdCallback()` 已改为 governor-only position control：
  - 不再使用旧 `CMD_DIRECT / CMD_SPLIT / CMD_POS_FF / CMD_MOTION_LIMIT` fallback
  - 飞控只吃 `PositionCommand.position` 和 yaw
  - `PositionCommand.velocity.x/y/z` 必须显式置 0
  - GVF 输出是内部速度意图，不直接发给飞控速度接口
- 当前主控制链路：
  1. `calcLiftedGuidance3D(pos, progress_w_)` 得到 `out.v_cmd / out.tangent / out.e_perp`
  2. governor 用 `v_tau_intent / v_n_intent` 表示切向与法向速度意图
  3. 在当前执行路径 `pm.gvf_->sample_w_` 上搜索候选前瞻 `L`
  4. 候选点不能越过 `path_w_end`，否则计入 `path_end_clamped_count`
  5. 每个候选用 `K_eq * (cmd_pos - pos)` 估计模型速度，并以速度误差、前瞻偏差、法向偏置、变化率作为代价
  6. 选出 `VEL_MATCH_GOVERNOR` 命令点；无有效候选时进入 `GOVERNOR_INVALID_HOLD`
  7. 发布 `PositionCommand.position = cmd_pos`，`PositionCommand.velocity = 0`
- 速度转位置关系：
  - `v_actual ~= K_eq * cmd_dist`
  - 仿真常用 `K_eq ~= 1.65`
  - 实机测得/采用 `K_eq ~= 1.10`
  - 实机目标速度 `2.0 m/s` 时，理论需要 `L_ff ~= 2.0 / 1.10 = 1.82 m`
- 实机 2m/s 初始建议：
  - `gvf/cmd/vel_max = 2.0`
  - `gvf/cmd/tangent_vel_max = 2.0`
  - `gvf/cmd/pos_gain_equiv = 1.10`
  - `gvf/cmd/governor_l_max = 1.7~1.9`
  - `gvf/cmd/governor_lead_max = 1.7~1.9`
  - `gvf/cmd/governor_l_rate_max ~= 3.0`
  - `gvf/switch/governor_path_margin_w ~= 0.6`
- 实机法向先保守：
  - `gvf/cmd/governor_normal_deadband = 0.06~0.08`
  - `gvf/cmd/governor_normal_full_error ~= 0.30`
  - `gvf/cmd/governor_normal_max ~= 0.15`
  - `gvf/cmd/governor_normal_rate_max ~= 0.3`
- 调参判断：
  - 速度跟不上：若 `tau_vel_error` 长期为负，且 `best_L/cmd_dist` 接近上限，优先加 `governor_l_max / governor_lead_max / governor_l_rate_max`
  - 左右晃：增大 `governor_normal_deadband / governor_normal_full_error`，减小 `governor_normal_max / governor_normal_rate_max`，或增大 `governor_normal_weight / governor_normal_rate_weight`
  - 大弯跟不上：先加 `governor_normal_max / governor_normal_rate_max`；仍外扩再降 `governor_l_max / governor_lead_max`；还不行说明 2m/s 弯里物理半径不够，需要弯里降 `tangent_vel_max`
  - 改 GVF 的 `k1/k2` 后 governor 参数通常不用大改，但要复查 `raw_v_normal_norm / e_perp_norm / normal_state_norm / tau_vel_error`

## Control Layer Effective Params
- 当前直接有效：
  - `gvf/cmd/vel_max`
  - `gvf/cmd/pos_gain_equiv`
  - `gvf/cmd/tangent_vel_max`
  - `gvf/cmd/acc_max`
  - `gvf/cmd/switch_motion_limit_time`
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
- 当前增益测试模式：
  - `gvf/cmd/gain_test_enable`
  - `gvf/cmd/gain_test_lead`
  - `gvf/cmd/gain_test_axis`
- 增益测试打开时，`cmdCallback()` 不走 GVF 正常链路，而是直接发布 `odom + fixed lead` 的位置命令，并用 `odom_vel_lpf dot lead / |lead|^2` 估计 `K_eq`，日志为 `[GVF][GAIN_TEST]`。
- 当前应清理或旧控制残留：
  - `gvf/cmd/speed_max`
  - `gvf/cmd/vel_lpf_hz`
  - `gvf/cmd/pos_time`
  - `gvf/cmd/speed_lag_time_gain`
  - `gvf/cmd/max_track_error`

## Recent Changes In This Iteration
- 已将控制链路从旧位置领先 / split-normal / pos_ff 版本迁移到 governor-only。
- 当前控制链路确定为：`GVF速度意图 -> governor候选前瞻点搜索 -> PositionCommand.position`。
- governor 失败时只允许 `GOVERNOR_INVALID_HOLD`，不回退旧控制算法。
- 已加入 `governor_path_short` 强制切换逻辑，避免规划成功但旧路径被拒切后耗尽。
- 增加/保留 `GAIN_TEST` 模式，用固定位置领先测位置环等效增益 `K_eq`。
- 已在当前仿真 workspace 实现 `SDFMap` 手动地图层：
  - RViz `Publish Point` 或 `/manual_map/add_obstacle_center` 可添加静态圆柱障碍物。
  - 手动障碍物写入 `manual_occupancy_buffer_`，再 OR 到 `occupancy_buffer_ / occupancy_buffer_inflate_`。
  - `getInflateOccupancy()`、`getNearestFreePoint()`、`isInMap()` 已接入手动层。
  - 地图更新、局部清理和 buffer refresh 后会重新 `applyManualLayer()`，手动障碍物不会被实时感知清掉。
- 已加入手动障碍物保存/加载：
  - `sdf_map/manual_map_file`
  - `sdf_map/manual_map_auto_load`
  - `sdf_map/manual_map_auto_save`
  - `test_gvf.launch` 当前保存到 `src/swarm_planner/bspline_traj/config/manual_obstacles_test_gvf.txt`。
  - 文件格式为每行一个障碍物中心：`x y z`，`#` 开头为注释。
  - 当前只持久化手动障碍物中心，不持久化边界点。
- 审计旧参数和旧代码残留：
  - `figure8_join_*` 已从 header 删除
  - cpp 中对应 reset 引用已删除
  - `lookahead_pts / realign_min_progress / join_*` 目前不应再作为有效调参

## Manual Map Layer Notes
- 当前仿真 `test_gvf.launch` 中：
  - `sdf_map/enable_manual_map = true`
  - `sdf_map/manual_click_direct = true`
  - RViz `Fixed Frame=world` 时，`Publish Point` 会直接进入 `SDFMap::manualObstacleCallback()`。
- 当前障碍物尺寸：
  - 实际写入圆柱半径为 `manual_obstacle_radius + manual_obstacle_inflate`
  - `test_gvf.launch` 当前为 `0.35 + 0.10 = 0.45 m`
  - 圆柱高度范围由 `manual_boundary_z_min` 到 `min(manual_obstacle_height, virtual_ceil_height, map_max_boundary.z)`。
- 规划器不需要改 GVF 核心：
  - 碰撞/边界仍通过 `SDFMap::getInflateOccupancy()`、`getDistance()`、`isInMap()` 查询。
  - 手动障碍物进入 SDFMap 后，即使局部感知暂时没有看到，也会被规划查询当作 occupied。
- 手动地图保存/加载：
  - `manual_map_auto_save=true` 时，每次添加障碍物后会重写 `manual_map_file`。
  - `manual_map_auto_load=true` 时，`initMap()` 会读取文件并生成手动圆柱。
  - 依赖固定 `world` 原点；如果实机 VIO/map 原点变化，旧文件里的坐标会整体偏移。
- 实机迁移建议：
  - `manual_click_direct=false`，由 `uav_server` 在 `MANUAL_MAP` 状态下转发 `/clicked_point` 到 `/manual_map/add_obstacle_center`。
  - 实机保存文件建议放在实机 workspace 的 `bspline_traj/config/manual_maps/realflight_obstacles.txt`，并在 launch 中使用绝对路径。

## Active Concerns
- `cmdCallback()` 里还有一些旧控制状态/未调用函数可继续清理：
  - `computePositionCmdOffset()`
  - 与其绑定的 `cmd_speed_max_ / cmd_pos_time_ / cmd_max_track_error_ / cmd_along_ratio_*`
- `closed_ref_dbg_*` 大多只服务日志，可保留到调试稳定后再删。
- 当前还有两个编译 warning：
  - `KinoPathCallback()` 中 signed/unsigned compare
  - `FSMCallback()` 中 `INIT` 未处理

## Current File Focus
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/launch/test_gvf.launch`
- `src/swarm_planner/bspline_traj/src/gvf.cpp`

## Removed / Deprecated In This Iteration
- 已删除或正在删除：
  - `figure8_join_mode_`
  - `figure8_join_idx_`
  - `figure8_join_stable_count_`
  - `generateFigureEightReference()` 未使用的 `cx/c2`
- 已标记废弃：
  - `gvf/circle_test/lookahead_pts`
  - `gvf/circle_test/realign_min_progress`
  - `gvf/circle_test/join_*`
  - `gvf/kino_sample_ts`
  - `gvf/kino_sample_ts_min`
  - `gvf/kino_max_guide_pts`
  - `gvf/path_pub_from_current`
  - `gvf/path_pub_future_pts`
  - `gvf/replan_stop_radius`
  - `gvf/replan_stop_vel`
  - `gvf/debug_gate`

## Immediate Next Tasks
1. 将 Realflight 代码同步为 governor-only 控制层，并保留实机 topic / yaw 约束。
2. Realflight 首飞使用 `K_eq=1.10` 下的 2m/s 保守参数：`l_max/lead_max=1.7~1.9`，法向 deadband `0.06~0.08`。
3. 清理 `test_gvf.launch / gvf.launch` 中旧控制参数，保持 launch 与当前代码事实一致。
4. 清理完成后跑 `catkin_make --pkg bspline_race`。

## Short Validation Checklist
- `catkin_make --pkg bspline_race`
- 启动 `test_gvf.launch`
- 看 `[GVF][CLOSED_REF]`
- 看 `[GVF][CLOSED_GOAL]`
- 看 `[GVF][KINO_RESULT]`
- 看 `[GVF][SWITCH]`
- 看 `[GVF][CMD_VEL_MATCH_GOV]`
- 看 `final_cmd_source=VEL_MATCH_GOVERNOR`
- 看 `fallback_reason=none`
- 看 `path_end_clamped_count / lead_limit_violation / normal_rate_limited / tau_vel_error`
