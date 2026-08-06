# Iteration Memory

日期：`2026-06-02`
最后更新：`2026-07-11`

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

## Fast-Planner-Style B-Spline Parameterization Update (`2026-07-11`)
- 已完成并提交当前仿真 workspace 的 Fast-Planner 风格 B 样条参数化改造，代码提交范围为：
  - `5308b7d feat: add Fast-Planner spline parameterization`
  - `85f0895 feat: initialize optimizer from parameterized spline`
  - `5629ab0 fix: use terminal velocity for partial Kino samples`
  - `0c887ac refactor: use Fast-Planner parameterization in astaropt`
- 旧 active `astaropt()` 链路的问题是：
  - `K` 个 Kino 样本直接生成 `K+4` 个控制点。
  - 中间 Kino 位置样本被直接当作 B 样条控制点，但 cubic B-spline 不会经过所有控制点。
  - B 样条 interval 独立使用 `planning/dist_p / planning/max_vel`，没有使用 Kino `getSamples()` 返回的实际 `ts`。
  - 位置样本曾按 `num_points_to_take_` 截断，但终端导数仍来自未截断路径，可能造成终端状态不一致。
- 当前 active `astaropt()` 数据链路为：
  1. 点到点或闭合候选规划得到同一个 `KinoPlanSamples`。
  2. 使用完整 `point_set`、四个边界导数 `[v0, vT, a0, aT]` 和 `getSamples()` 返回的实际 `ts`。
  3. `UniformBspline::parameterizeToBspline()` 解 `K+4` 个方程、`K+2` 个未知控制点的 Fast-Planner 最小二乘系统。
  4. `bspline_optimizer::setInitialControlPoints()` 用参数化控制点和同一个实际 `ts` 初始化现有 optimizer。
  5. 现有 optimizer 继续只优化内部控制点，首尾各三个 cubic 控制点保持固定。
  6. 优化后用 `getFeasibilityRatio()` 检查速度/加速度；若比例大于 1，只通过 `scaleTime()` 统一增大 interval，不改变空间控制点。
  7. 下游位置/速度采样、`new_i0_out`、FSM、GVF 和 command topic 保持原有接口。
- 新参数语义：
  - `gvf/kino_sample_ts`：Kino `getSamples()` 的初始采样时间，当前 `test_gvf.launch` 设为 `0.15 s`。
  - `gvf/kino_sample_ts_min`：初始采样时间下限，当前为 `0.05 s`。
  - `getSamples()` 会把输入 `ts` 调整成 `T_sum / seg_num`；返回值才是参数化、optimizer 和 spline 使用的权威 interval。
  - `planning/dist_p` 不再决定 active `astaropt()` 的 B 样条 interval，只保留给旧接口/其他链路。
- no-shot / partial 末端速度 bug 已修复：
  - 旧 `getSamples()` 在累计路径时把局部 `node` 回溯到 root，随后错误使用 `node->state.tail(3)` 作为 `end_vel`，实际得到起点速度。
  - 当前使用 `path_nodes_.back()->state.tail(3)`，即 partial 搜索路径真实末端速度。
  - 该修复不改变 `NEAR_END / REACH_HORIZON` 接受策略，只修正传给 B 样条的边界状态。
- 明确保留的行为：
  - `planKinoToGoal()` 仍只拒绝 `NO_PATH`，不新增 `NEAR_END / REACH_HORIZON / partial` 拒绝。
  - 闭合目标候选、`partial_best_end_dist`、碰撞检测、轨迹切换、FSM、GVF、ROS topic 和 command 生成策略未随 B 样条改造改变。
  - 未加入外部 Fast-Planner catkin 依赖，也未替换现有 optimizer。
- 新增测试：
  - `fastplanner_parameterization_test`：覆盖 `K -> K+2`、直线重构、边界速度/加速度、duration、时间缩放保持几何、optimizer 初始化。
  - `kinodynamic_samples_test`：覆盖 no-shot 时 `vT` 来自真实末端节点。
  - 最终验证：`path_searching + bspline_race` 构建成功，`38 tests, 0 errors, 0 failures`。
- B 样条改造后的日志判断：
  - `[GVF][BSPLINE_PARAM] kino_points / control_points / kino_ts / final_interval / ratio` 用于确认 `K+2` 与时间语义。
  - 已观察到的两次失稳日志中，该日志数值有限且关系正常，没有 B 样条参数化 NaN/Inf 或控制点数量异常证据。
  - 已定位的失稳主因在 command governor 和重规划切换：位置指令估算加速度远超 `cmd/acc_max`，碰撞反复触发造成高频强制切轨，fallback 不是固定安全悬停点。这些不属于本轮 B 样条数学改造。
- 当前 B 样条剩余低优先级事项：
  - `UniformBspline::getT()` 对非整采样周期 duration 会遗漏不足一个周期的精确尾点。
  - active pipeline 明确按 cubic 参数化，但 `setControlPointsAndInterval(..., 3, ts)` 与可配置 `planning/traj_order` 之间尚未显式约束。
  - `setInitialControlPoints()` 的非法输入分支缺少逐项回归测试。

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
- 当前仅用于诊断、没有真正执行限幅：
  - `gvf/cmd/acc_max`：只与 `estimated_acc` 一起打印，未限制发布位置指令的加速度。
  - `gvf/cmd/switch_motion_limit_time`：只形成 `switch_active` 日志窗口，未在窗口内限制位置命令跳变。
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

## Closed Kino Forward-Consistency Fix (2026-07-13)
- 本轮修复的现象不是 B 样条重新出现“又字形自交”，而是闭合模式选中了空间上先前进、后绕回的 Kino 完整路径。
- 本机 ROS 日志中的关键反例：
  - `closed_ref_w=71.674`
  - `accepted_goal_w=71.884`
  - 新 `goal_w=72.674`，绝对参考相位仍向前 `0.790 m`
  - 但 `lookahead=1.0 m` 得到 `kino_path_length=5.186 m`
  - 该路径被 `accept_collision&timout` 接受；下一轮 `1.0 m` 候选又得到 `5.139 m` 路径并被接受
  - 随后 `tangent_dot_odom` 从正值翻到 `-0.912`，说明实际执行路径已经转为参考反方向
- 根因：`closed_ref_w_ + lookahead` 只保证目标点在全局参考相位上位于前方；点到点 KinoA* 不知道闭合参考的正向拓扑。旧选择器只检查终点状态/终点进度和最小长度，第一个 `REACH_END` 只要足够长就 early-break，反而会把超长绕回路径当作高质量完整路径。
- 已实施的范围：
  - 保留动态、连续未取模的全局 `closed_ref_w_`
  - 保留 `goal_w = closed_ref_w_ + lookahead`
  - 不增加第二套规划相位，不冻结 `closed_ref_w_`
  - 不保存固定左右绕障决策
  - 不修改 B 样条、governor、碰撞检测、换轨策略或点到点目标选择
- `selectClosedGoalCandidate()` 现在会对每条 Kino `point_set` 做整路径正向性检查：
  1. 从当前连续 `closed_ref_w_` 开始，对 Kino 样本逐点投影到连续未取模参考 `w`
  2. 投影使用专用的局部连续区间，不复用旧固定 `0.3 m` 的 `projectClosedLocal()` 窗口
  3. 投影记录 `trusted / on_boundary / ambiguous / projection_error`
  4. 只有相邻投影都可信时，才累计沿参考切向的连续反向段
  5. `max_backward_w` 与 `reverse_length` 都绑定同一段达到 `min_reverse_run` 的连续可信反向 run，不把孤立噪声和后续小反向段拼接
- 当前方向分类：
  - `FORWARD`：投影可信比例足够，且没有持续明显反向段
  - `TURNBACK`：同一连续可信反向段超过允许尺度
  - `UNKNOWN`：投影可信比例不足、撞局部窗口边界或出现非局部分支歧义
- 当前候选行为：
  - 只有可信且 `FORWARD` 的 usable `REACH_END` 才能以 `first_usable_forward_reach_end` early-break
  - `TURNBACK / UNKNOWN` full 不进入 early-break，也不进入最终 fallback，会继续尝试后面的 `1.25 ... 3.0 m` 候选
  - 可信正向的原生 `REACH_HORIZON / NEAR_END` partial 继续保留
  - 当前优先级为 `forward_full > usable forward_partial`；`usable=false` 的短路径不再有执行 fallback
  - 若所有候选都回绕或投影不可信，规划返回失败；首版不会把完整调头路径伪装成正常 full
- 首版明确没有人工截断 full 的“正向前缀”：
  - 当前 Kino 样本只提供整条路径首末导数，没有截断时刻的真实速度/加速度
  - 直接有限差分重算可能造成末端动力学跳变、B 样条超调或再次短轨迹耗尽
  - 后续若要实现 prefix，必须先由 Kino 输出截断时刻真实状态并重新验证动力学、碰撞和 B 样条正向性
- 投影歧义修正：
  - 960 点密集参考线上的相邻线段属于同一局部分支，不能作为第二投影解
  - ambiguity 只比较与最佳投影相位分离至少 `max(0.5 m, step_window_w)` 的非局部候选
  - 保存 `second_w`，最佳投影变化后重新验证旧 best/second 是否仍满足非局部分离
- 当前首版内部尺度（尚未新增 launch 参数）：
  - `allowed_backward_w = max(0.1, closed_ref_search_back_w_)`，当前约 `0.3 m`
  - `allowed_reverse_length = allowed_backward_w`
  - `min_reverse_run = 3`
  - `min_direction_trusted_ratio = 0.8`
  - `direction_max_projection_error = max(1.5 m, current_local_d + 0.5 m)`
- 新增 `[GVF][CLOSED_GOAL]` 诊断字段：
  - `selected_direction_trusted`
  - `selected_forward_consistent`
  - `selected_max_backward_w`
  - `selected_reverse_length`
  - `selected_direction_trusted_ratio`
  - `selected_turnback_start_idx`
  - `tried_direction`
  - `tried_direction_trust`
  - `tried_max_backward_ws`
  - `tried_reverse_lengths`
  - `tried_direction_trusted_ratios`
  - `tried_direction_boundary_hits`
  - `tried_direction_ambiguous`
- 测试与构建：
  - `gvf_switch_policy_test`：`70/70` 通过
  - `kinodynamic_samples_test`：`3/3` 通过
  - 新测试覆盖单调前进、纯横向、持续回绕、孤立反向噪声、低可信比例、非有限输入、跨圈未取模、终点仍前进但中途回绕、960 点密集参考邻段、方形闭合跨圈与运行时边界投影
  - `path_searching`、`bspline_race` 完整编译通过
  - 独立代码审查确认 success 日志 `72/72`、all-failed 日志 `13/13`，所有 `tried_*` 数组按候选对齐
- 测试目标现在链接 `bspline_gvf`，用于运行时连续投影测试；该 CMake 修改只影响测试构建。
- 点到点导航不进入 `selectClosedGoalCandidate()` 的方向检查，仍走公共 `planKinoToGoal(..., retry_once=true)`，原二次搜索行为保持。
- 尚未完成的唯一关键验证：重新运行原障碍仿真并确认历史 `1.0 m / 5.186 m` 候选显示为 `TURNBACK` 或 `UNKNOWN`，不能成为 `first_usable_forward_reach_end`；同时观察 `planning_duration_ms` 是否持续低于 `0.5 s` 重规划周期。

## Closed Short-Trajectory Exhaustion Root Fix (2026-07-14)
- rosbag `4239` 已确认的停顿链路：`all_candidates_short_fallback -> 几厘米新轨迹被视为成功 -> old_collision/near_end/governor_path_short 强制换轨 -> governor 领先量耗尽 -> 障碍物前明显停顿`。
- `selectClosedGoalCandidate()` 当前只允许 `valid && usable` 的 Kino 候选进入最终执行池：
  - `usable` 同时要求可信正向、真实相位进度达到 `required_progress_w`、实际路径长度达到 `required_execution_length`
  - `REACH_HORIZON / NEAR_END` 的 usable partial 仍可增量执行
  - 所有候选都太短时直接规划失败，生产代码已无 `all_candidates_short_fallback`
- 每次重规划都从配置的 `goal_prefer_lookahead_w` 重新开始，不沿用上一次候选前视，避免前视粘滞，也不保存绕障决策。
- 当前地图的障碍物三状态选点：
  1. 未发现障碍物起点：使用普通配置前视
  2. 已发现起点、尚未确认末端：`desired_lookahead=lookahead_max_w`，遍历全部候选并选实际 `end_delta_w` 最大的 usable 路径
  3. 已确认末端：目标为 `obstacle_end_delta_w + goal_obstacle_pass_margin_w`；优先实际越障且超越量最小的 usable 路径，否则选实际进度最大的 usable partial
- 障碍物模式不会因一个较近的 full 路径提前结束搜索；只有无障碍 full，或已真正越过已确认障碍物末端的 full，才允许 early-break。
- Kino 合格后，`astaropt()` 会从最终 B 样条的实际执行索引重新检查：
  - 剩余实际路径长度
  - 连续未取模闭合参考投影
  - 终点真实相位进度
  - 投影可信度与持续正向性
  - 若选中的 Kino 已越障，则最终 B 样条也必须保持越障
- 若只有未越障但 usable 的 partial，允许先推进并继续增量重规划，不会被 B 样条越障后验误拒。
- 若闭合模式无 usable 新轨迹：
  - 旧轨迹安全：继续旧轨迹
  - 旧轨迹碰撞：锁存当前位置进入 `[GVF][CLOSED_HOLD]`
  - 接受安全新轨迹、旧轨迹恢复安全或退出闭合模式时解除 hold
  - hold 激活和 B 样条后验都只作用于闭合模式；点到点规划路径与换轨策略保持原样
- 新增关键日志：`[GVF][BSPLINE_VALIDATE]`、`[GVF][CLOSED_HOLD]`。
- 本轮未新增 launch 参数，也未修改碰撞检测算法或 `shouldAcceptCandidate()` 的原换轨判据。
- 验证结果：
  - `bspline_race` 全部测试通过，其中 `gvf_switch_policy_test 74/74`
  - `kinodynamic_samples_test 3/3`
  - `catkin_test_results build/test_results`：`182 tests, 0 errors, 0 failures`
  - 独立只读代码审查确认短轨迹执行入口已阻断、点到点模式无 hold 泄漏

## Active Concerns
- `cmdCallback()` 里还有一些旧控制状态/未调用函数可继续清理：
  - `computePositionCmdOffset()`
  - 与其绑定的 `cmd_speed_max_ / cmd_pos_time_ / cmd_max_track_error_ / cmd_along_ratio_*`
- `closed_ref_dbg_*` 大多只服务日志，可保留到调试稳定后再删。
- 当前还有两个编译 warning：
  - `KinoPathCallback()` 中 signed/unsigned compare
  - `FSMCallback()` 中 `INIT` 未处理
- command governor 当前没有真正的发布指令速度/加速度限幅；轨迹切换时日志已出现 `estimated_acc` 数十到数百 `m/s^2`。
- 通用 `GOVERNOR_INVALID_HOLD` 当前每周期使用最新 `odom` 作为命令点；闭合规划失败且旧轨迹碰撞的专用 `[GVF][CLOSED_HOLD]` 已改为锁存进入时的位置。两者语义不同。
- 碰撞检测可能造成连续 `collision -> accept_collision&timout -> switch -> collision` 抖动，但用户当前确认下一项只修“短轨迹耗尽”，不改碰撞检测与切换策略。
- 闭合整路径正向性投影当前复杂度约为 `候选数 × Kino样本数 × 参考线段数`；参考点 `960` 时需要用新仿真日志确认 `planning_duration_ms`，若接近 `0.5 s` 再优化为局部参考索引扫描。
- 所有闭合候选均为 `TURNBACK/UNKNOWN` 时首版会规划失败并保留现有安全行为，不会人工截断 full；后续是否增加带真实 Kino 末端状态的正向 prefix 需要单独设计。

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
  - `gvf/kino_max_guide_pts`
  - `gvf/path_pub_from_current`
  - `gvf/path_pub_future_pts`
  - `gvf/replan_stop_radius`
  - `gvf/replan_stop_vel`
  - `gvf/debug_gate`

## Immediate Next Tasks
1. 重新运行产生过 `1.0 m lookahead / 5.186 m Kino path` 的闭合障碍仿真。
2. 确认坏候选在 `[GVF][CLOSED_GOAL]` 中标记为 `TURNBACK` 或 `UNKNOWN`，并且 `early_break=0`，随后继续尝试更远候选。
3. 确认正常无遮挡 `1.0 m` 候选仍为 `FORWARD` 并优先选择；正向原生 partial 仍能 fallback。
4. 检查 `planning_duration_ms` 的 P95 是否低于 `0.5 s`；若投影耗时过高，再把逐样本全参考段扫描优化成连续局部索引扫描。
5. 仿真稳定后再同步到无人机 `catkin_ws_gvf`，当前本机修改尚未同步无人机。

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
