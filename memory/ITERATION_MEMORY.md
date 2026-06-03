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
  6. `selectClosedGoalCandidate()` 会对多个 `lookahead_w` 试 KinoA*，优先选完整到达的候选，失败时可用 end distance 最小的 partial 备选。
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
- 当前真实执行的 `cmdCallback()` 已经改成“GVF 速度意图转换为位置命令接口”的版本，不是直接给速度接口：
  - `cmd.velocity.x/y/z = 0`
  - 飞控只吃 `cmd.position` 和 yaw
  - `raw_v_gvf / v_gvf` 只作为内部速度意图，用来算位置领先量
- 当前主控制链路：
  1. `calcLiftedGuidance3D(pos, progress_w_)` 得到 `raw_v_gvf`
  2. 用 `gvf/cmd/vel_max` 做 GVF 速度意图幅值限幅
  3. 可选用 `gvf/cmd/use_vel_slew_limit` 做 acc / jerk / offset-rate 限制
  4. `lead_from_gvf = v_gvf / gvf/cmd/pos_gain_equiv`，这是速度意图到位置领先的接口转换
  5. 可选 `pos_ff = pos_ff_time * d(v_gvf)/dt / K_eq`，这是额外前馈位置补偿
  6. 当前 `gvf/cmd/pos_ff_xy_only=true`，所以 `pos_ff.z = 0`
  7. `lead = lead_from_gvf + pos_ff`
  8. `cmd_pos = odom + lead`
  9. 发布 `PositionCommand.position = cmd_pos`，`PositionCommand.velocity = 0`
- `v_gvf / K_eq` 不应再叫前馈；它是位置控制接口所需的速度到位置转换。
- `pos_ff` 才是这次新增/保留的前馈控制项，来源是 `v_gvf` 的差分加速度估计。
- 当前默认测试倾向：
  - `gvf/cmd/use_vel_slew_limit=false`
  - 保留 `vel_max + v/K_eq + pos_ff`
  - `gvf/cmd/use_pos_ff=true`
  - `gvf/cmd/pos_ff_xy_only=true`
- 经验结论：
  - acc/jerk 限幅在换轨时容易导致跟不上。
  - 如果开启 `use_vel_slew_limit`，换轨窗口建议保留：
    - `gvf/cmd/skip_motion_limits_on_switch=true`
  - `pos_ff` 使用 3D 差分时可能导致 z 轴上下抽动，因此当前采用 XY-only 前馈。

## Control Layer Effective Params
- 当前直接有效：
  - `gvf/cmd/vel_max`
  - `gvf/cmd/pos_gain_equiv`
  - `gvf/cmd/lead_max`
  - `gvf/cmd/use_pos_ff`
  - `gvf/cmd/pos_ff_xy_only`
  - `gvf/cmd/pos_ff_time`
  - `gvf/cmd/pos_ff_max`
- 当前可选限幅：
  - `gvf/cmd/use_vel_slew_limit`
  - `gvf/cmd/acc_max`
  - `gvf/cmd/jerk_max`
  - `gvf/cmd/offset_rate_max`
  - `gvf/cmd/use_switch_motion_limits`
  - `gvf/cmd/skip_motion_limits_on_switch`
  - `gvf/cmd/switch_motion_limit_time`
  - `gvf/cmd/switch_acc_max`
  - `gvf/cmd/switch_jerk_max`
  - `gvf/cmd/switch_offset_rate_max`
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
- 加过 acc / jerk / offset-rate 限制，并发现换轨时容易跟不上。
- 增加 `skip_motion_limits_on_switch`，允许换轨窗口跳过这些限幅。
- 当前控制链路确定为：`GVF速度意图 -> vel_max限幅 -> v/K_eq位置领先 -> 可选pos_ff -> PositionCommand.position`。
- 将 position-only 前馈改成可选 XY-only，当前 `test_gvf.launch` 打开。
- 增加/保留 `GAIN_TEST` 模式，用固定位置领先测位置环等效增益 `K_eq`。
- 审计旧参数和旧代码残留：
  - `figure8_join_*` 已从 header 删除
  - cpp 中对应 reset 引用已删除
  - `lookahead_pts / realign_min_progress / join_*` 目前不应再作为有效调参

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
1. 继续清理未调用的 `computePositionCmdOffset()` 及其参数成员。
2. 清理 `test_gvf.launch` 中无效参数，保持 launch 与当前代码事实一致。
3. 处理剩余两个 warning。
4. 清理完成后跑 `catkin_make --pkg bspline_race`。

## Short Validation Checklist
- `catkin_make --pkg bspline_race`
- 启动 `test_gvf.launch`
- 看 `[GVF][CLOSED_REF]`
- 看 `[GVF][CLOSED_GOAL]`
- 看 `[GVF][KINO_RESULT]`
- 看 `[GVF][CMD_DIRECT]`
- 看 `[GVF][CMD_POS_FF] xy_only=1 pos_ff_z=0`
- 若打开限幅，看 `[GVF][CMD_MOTION_LIMIT]`
