# PhaseOffsetSwarm 实施日志

> 日期：2026-08-06
> 工程：/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
> 唯一实施依据：docs/PhaseOffsetSwarm_ROS_Simulation_Implementation_Plan_2026-08-06.md

## 批次 2：命名空间化 test_gvf + 统一 swarm launch

状态：完成（2026-08-06）

### 目标

1. 参数化 test_gvf.launch（uav_ns、robot_id、odom/local_map/global_map/cmd/goal/particle_base/状态话题等）。
2. gvf_manager 去除硬编码话题（/move_base_simple/goal、/path_vis、/goal_vis、/particle0/*、/gvf_force）。
3. sdf_map 修复缺斜杠话题拼接。
4. formation_planning spinner_threads 参数化。
5. 新增 phase_offset_agent.launch / phase_offset_single_sim.launch / phase_offset_swarm_sim.launch / 3、7 机 wrapper / RViz 配置。
6. simulator.launch 增加 legacy 默认开关。
7. 三机、七机 enable_swarm=false 独立规划、控制与统一 RViz 显示。

### 修改文件

- src/swarm_planner/bspline_traj/launch/test_gvf.launch：新增 uav_ns/robot_id/odom_topic/
  local_map_topic/global_map_topic/cmd_topic/goal_topic/particle_base/state_topic/
  path_event_topic/conflict_state_topic/debug_topic/enable_phase_offset/enable_swarm/
  enable_tube/enable_cbf_safety/goal_z_mode/manual_map_auto_save/manual_map_file/
  spinner_threads/start_formation_planning/start_local_sensing 参数；formation_planning
  与 local_sensing 节点带 if 开关；保留用户 circle_test/enable=false、auto_start=false 两行。
- src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h：robot_id/robot_namespace/
  particle_base/各话题参数成员、阶段开关、appendTopic 工具。
- src/swarm_planner/bspline_traj/src/gvf_manager.cpp：构造函数读取新参数；initCallback
  全部话题改参数化；InitGvf 使用 particle_base_；goalCallback 支持 goal_z_mode
  （legacy_offset 默认 msg.z+1，absolute_world 直接使用 msg.z）。
- src/swarm_planner/plan_env/src/sdf_map.cpp：sdf_map/occupancy 等话题改用
  ros::names::append（修复缺斜杠拼接）；带参数 initMap 的 cloud/odom 订阅使用传入话题。
- src/swarm_planner/bspline_traj/src/formation_planning.cpp：spinner_threads 参数化（默认 8）。
- src/swarm_planner/bspline_traj/CMakeLists.txt / package.xml：显式补齐
  geometry_msgs/sensor_msgs/nav_msgs/quadrotor_msgs；新增 phase_offset_swarm_visualizer 目标。
- src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch：start_plant/
  start_controller/start_map/start_rviz/start_odom_visualization 开关（默认 true，legacy 不变）。
- 新增 launch/phase_offset_agent.launch、phase_offset_single_sim.launch、
  phase_offset_swarm_sim.launch、phase_offset_swarm_3/7.launch；
  config/phase_offset_swarm/rviz/phase_offset_swarm.rviz（Fixed Frame=world）；
  config/phase_offset_swarm/scenarios/{none,open_hex,obstacle_split_merge,
  corridor_two_wide,corridor_single_wide}.yaml；
  scripts/swarm_scenario_publisher.py（每机目标只发一次）；
  src/phase_offset_swarm_visualizer.cpp（只读汇总节点：uavs + trajectories marker）；
  so3_quadrotor_simulator/config/initial_states_hex_{1,3,7}.yaml（d=1.0 六边形）。

### 编译

catkin_make -j8：exit 0，无新增警告/错误（既有 sign-compare 警告保留）。

### 单元测试

7 个现有 GTest 全部通过（70 用例）。注意：修改 gvf_manager.h 后必须显式重建测试目标，
否则旧测试二进制按旧类布局链接新库会段错误（已修复并记录为过程教训）。

### ROS 集成测试（test/batch2_swarm_launch_test.py）

N=3：ok=true。1 个 multi simulator 进程/节点；3 planner + 3 local_sensing + 3 controller；
odom 100 Hz；/uav_i/particle0/path、sdf_map/occupancy、position_cmd 全部存在；
无 /particle0sdf_map 等全局残留话题；三机独立到达各自目标（progress=1.0）；
杀掉 visualizer 后 3 planner 与 odom 继续运行。

N=7：ok=true。7 机全部独立飞行（progress 0.74~1.0），7 路 odom ~100 Hz，
节点/进程计数全部符合，visualizer 无关性验证通过。

### Legacy 回归

- 原 simulator.launch（smoke-single）：/sim/odom 100 Hz，全部节点正常。
- 原 test_gvf.launch 默认参数 + /move_base_simple/goal：目标 z=0 → 世界 z=1.0
  （legacy_offset），UAV 精确到达 (5,3,1)。

### 风险与遗留

1. roslaunch 对已死节点（如被 SIGKILL 的 visualizer）的 master 注册清理是惰性的，
   测试断言使用进程检查而非节点列表。
2. RViz 配置已修复（Fixed Frame 层级错误导致黑屏），修复后 visualizer 以 10 Hz
   发布 world 帧 marker，RViz 正常显示网格/点云/UAV/轨迹。
3. 测试脚本与清理命令需避免 pkill/pgrep 自匹配（使用显式 PID 或括号模式）。
4. 目标发布必须在订阅连接建立后发送（queue_size=1 不缓存）。

## 批次 3：phase-offset 几何与活动参考

状态：完成（2026-08-06）

### 修改文件

- include/bspline_race/phase_offset_types.h：计划 5.1 全部结构（PhaseOffsetGeometry、
  TubeBounds、NeighborState、PredictedNeighborState、NeighborSelection、
  NeighborEventState、PairIntentContribution、SwarmIntent、PortCommand、
  SwarmControlMode）。
- include/bspline_race/phase_offset_geometry.h + src/phase_offset_geometry.cpp：
  PhaseOffsetGeometryEvaluator（T/N/曲率/r/r_w、1-kappa*delta>=mu、水平路径防护、
  活动参考误差分解、base_v/base_w_dot；纯计算无 ROS）。
- include/bspline_race/gvf.h + src/gvf.cpp：PhaseOffsetGuidanceResult 与
  calcPhaseOffsetGuidanceAtState（ContinuousPhasePath::evaluate 提供 p/p_w/p_ww，
  fallback 到重参数化采样 + 有限差分）。
- include/bspline_race/gvf_manager.h + src/gvf_manager.cpp：phase_offset_delta_
  状态、delta_init/test_delta/delta_fixed_limit 参数、activeReferencePointAtW、
  governor 候选点改为活动参考 r(w+l,delta)、active path 可视化（5 Hz marker，
  话题 particle_base/phase_offset/active_path_vis）、cmdCallback phase-offset 分支。
- CMakeLists.txt：phase_offset_geometry.cpp 加入 bspline_gvf；新增
  phase_offset_geometry_test。
- launch：test_delta 参数通道（test_gvf/agent/single_sim）。

### 单元测试

phase_offset_geometry_test：8 个用例全部通过（直线 delta=0/±0.5、圆曲率解析、
r_w⊥N、1-kappa*delta 退化、误差分解、FD r_w、8 字分支由 w 区分）。
全部现有 GTest 78 个用例通过。

### ROS 集成测试（test/batch3_phase_offset_test.py，每用例独立进程）

- enable_phase_offset=false：y≈0.0（legacy 中心线），无活动参考可视化。
- delta=0：y≈0.0。
- delta=+0.5：稳态 y=+0.477（目标 +0.5，容差 0.15）。
- delta=-0.5：稳态 y=-0.485。
- 活动参考可视化话题在启用时发布。

结论：delta 改变时 UAV 收敛到活动参考 r=p+N*delta，不会被拉回基础路径 p。

### 过程修复

1. RViz MarkerArray display 必须用 "Marker Topic" 键（"Topic" 键导致话题为空，
   用户看到 display 栏无话题）；已修复 phase_offset_swarm.rviz。
2. visualizer 首次崩溃：deque<Eigen::Vector3d> 在 C++14 下无对齐分配器（改用
   std::array<double,3>）；第二次崩溃：last_pos_ 未初始化大小。
3. 修改 gvf_manager.h 后必须显式重建全部测试目标（旧二进制类布局错位段错误）。

### 风险与遗留

1. 批次 3 的 delta 为固定值（test_delta/初始化值）；u_w/u_delta 注入与内部状态
   同步更新在批次 4 实现。
2. 圆/8 字闭合路径的 phase-offset 飞行验证待批次 10 场景测试补充。

## 批次 4：人工 matched port

状态：完成（2026-08-06）

### 修改文件

- gvf_manager：phase_offset/test_port_enable、test_u_w、test_u_delta、
  test_u_w_amp、test_u_delta_amp、test_port_freq、test_port_phase 参数；
  cmdCallback phase-offset 分支注入人工端口：
  v_match = r_w*u_w + N*u_delta；v_final = base_v + v_match；
  w_dot_final = base_w_dot + u_w；delta_next = delta + u_delta*dt（±限位）；
  governor 使用候选 delta_next；delta 与 w 使用同一最终端口提交；
  epsilon_match 发布到 particle_base/phase_offset/matched_residual 并节流日志。
  振荡相位相对端口启用时刻（绝对时间相位会导致积分漂移，已修复）。
- matched_phase_offset_test.cpp：5 个用例（u_w only / u_delta only / 双端口 /
  限幅端口 / 端口方向分解），验证误差导数与无端口一致（<1e-9）。
- launch 链新增 test_port_* 参数；scripts/generate_phase_offset_maps.py +
  phase_offset_open/split_merge/corridor_two_wide/corridor_single_wide.pcd；
  test_gvf/agent/single_sim 地图尺寸参数化（map_size_x/y/z）。

### 单元测试

matched_phase_offset_test：5/5 通过。全部 GTest 83 个用例通过。

### ROS 集成测试（test/batch4_phase_offset_test.py，开放地图 + map_size_x=40）

- ramp（u_delta=0.2）：delta 0→0.8 限位，物理 y=+0.80，残差 2e-16。
- sine（u_delta_amp=0.3 @0.1Hz，零均值相位）：物理 y 振幅 0.34（position
  governor 跟踪滞后），delta 状态零均值（-0.058），残差 2.3e-16。
- uw（u_w=0.3）：中心线飞行，残差 2.2e-16。
- 全部 epsilon_match < 1e-9（实测 ~2e-16）。

### 过程发现

1. 单机 launch 的 local map 话题是 /sim/local_map（非命名空间）；SDF 地图
   范围 x∈[-10,10]，目标 x=16 出界导致规划卡死——测试需 map_size_x=40。
2. 点路径存在大曲率段（regularity_below_mu）会使几何短暂失效并冻结 delta；
   属于路径性质，allocator/tube（批次 6-8）将正式约束 delta。
3. position-only governor 跟踪振荡横向参考存在放大；计划 7.7 的 velocity
   feedforward 在批次 6/7 处理。

### 风险与遗留

1. 物理 y 对高频 u_delta 的跟踪增益 <1（0.34/0.46），批次 6/7 打开 velocity
   feedforward 后复测。
2. 地图生成脚本已就位，批次 10 补充完整元数据与场景 YAML。

## 批次 5：分布式状态与局部邻居聚合（shadow）

状态：完成（2026-08-06）

### 修改文件

- common_msgs：新增 SwarmState / SwarmPathEvent / SwarmConflictState /
  PhaseOffsetDebug 四个消息（计划 6.1-6.4），更新 CMakeLists 与 common_msgs.h。
- swarm_neighbor_model.h/.cpp：NeighborStateBuffer（mutex + 超时/保留双阈值）、
  SwarmEventBuffer、NeighborSelector（进入/退出滞回、organization/safety 双集合、
  LOS、stale 邻居保留进 safety + communication_fault）、ElasticSwarmIntent
  （紧支撑权重、允许距离带、径向阻尼、有界软安全、beta 只衰减弱凝聚）。
- gvf_manager：状态广播 timer（25 Hz）、共享状态/事件订阅、cmdCallback 内
  shadow 聚合（g_swarm/g_des/u_raw/shadow_v_match/shadow_v_final 计算与 debug
  发布，不驱动真实命令）、interactions marker（org 绿线/safety 红线/g_des 箭头）、
  PhaseOffsetDebug 填充。
- config/phase_offset_swarm/phase_offset_swarm.yaml（计划 9 参数）+ test_gvf
  节点内 rosparam 加载。
- CMakeLists：swarm_neighbor_model.cpp 加入 bspline_gvf；swarm_neighbor_model_test。

### 单元测试

swarm_neighbor_model_test：9/9（七机邻居表与 proposal 完全一致、A 压缩外推、
D 对称抵消、E 不在 C 组织集、距离带行为、beta=0 只关弱凝聚、共同平移零阻尼、
相对伸缩阻尼方向、stale 保留 safety + 通信故障）。全部 GTest 92 个用例通过。

### ROS 集成测试（test/batch5_swarm_shadow_test.py，N=7 六边形 + 开放地图）

- 状态广播：每机 ~22 Hz，7 个 robot_id 齐全，状态有限。
- 端到端邻居一致性：131 个 debug 样本，org 集全部落在
  [enter(1.55), exit(1.65)] 期望区间内，0 失配。
- shadow_mode=true；interactions marker 657 条。
- 故障注入：杀掉 /uav_0/formation_planning 后，uav_1 的 org 剔除 0、
  safety 保留 0（stale 状态不静默消失）；其余 UAV odom 继续。

### 风险与遗留

1. state 实际频率 ~22 Hz（timer 开销），满足计划 20-30 Hz。
2. 事件消息（SwarmPathEvent/SwarmConflictState）的发布端在批次 8 实现。
3. shadow 尚未驱动真实命令；批次 6 由 allocator 接管并激活。

## 批次 6：二维 phase-offset allocator（ACTIVE matched port）

状态：完成（2026-08-06）

### 修改文件

- phase_offset_allocator.h/.cpp：LinearConstraint2D、2D QP active-set 枚举
  （无约束最优 + 单约束投影 + 双约束交点，O(m^2)）、滚动/非负/终端/紧急模式、
  phase 正向、切向裕度、delta 边界、u_w/u_delta 变化率约束；目标函数由
  展开式构造（H/f），无约束解与解析分解一致。
- gvf_manager：swarm 分支接入 allocator（固定 ±limit tube），allocator 端口
  ACTIVE 驱动真实命令（shadow 结束）；TERMINAL 模式（点到点端点 margin）；
  debug 输出 allocator 模式与端口。
- config：phase_speed_max 3.0（W/s），tangent_speed_max 2.0（m/s）。

### 单元测试

phase_offset_allocator_test：10/10（无约束=解析、单约束投影、双约束交点、
多约束最优、rolling 可行、仅 nonneg 可行→SAFETY_PRIORITY、均不可行→EMERGENCY、
phase 不反向、切向裕度、TERMINAL 允许 phase 衰减）。全部 GTest 102 个用例通过。

### ROS 集成测试（test/batch6_swarm_active_test.py，N=3 开放地图）

- swarm on：69 个 debug 样本全部 ROLLING，EMERGENCY 0，残差 1.8e-16，
  delta std=0.0158（邻居意图通过 allocator 驱动 delta 状态），状态有限。
- swarm off：不计算意图（无 debug），delta 不变。

### 过程修复

1. slew-high 约束 b 值符号错误（漏负号）导致上界缺失——已修复并回归。
2. 测试几何 w_dot_base 超过 phase_speed_max 使 rolling 集正确不可行；
   配置与测试参数已对齐（phase_speed_max 3.0、tangent_speed_max 3.0）。

### 风险与遗留

1. position-only governor 的横向响应滞后使物理间距改善有限（计划 7.7）；
   批次 7 的 CBF 提供安全下界，velocity feedforward 待批处理。
2. 物理间距 min 0.45 m（三机汇聚目标）——批次 7 的机间 CBF 将保证 d_rob。

## 批次 7：鲁棒机间与 tube CBF 约束 + velocity feedforward

状态：完成（2026-08-06，形式化 d_rob 保证仍存在跟踪缺口，见遗留）

### 修改文件

- phase_offset_cbf_constraints.h/.cpp：appendTubeConstraints（上下边界 CBF，
  使用 tube 的 upper_dw/lower_dw）、appendPairwiseConstraints（计划 4.5 鲁棒
  距离 d_rob、ρ_rob、线性化 a·u≥b；只读 PredictedNeighborState）。
- swarm_neighbor_model：PredictedNeighborState.velocity_uncertainty_bound 按
  计划 4.5 从消息年龄 + 加速度界 + 执行误差计算（safety 参数）。
- phase_offset_allocator：CBF 约束加入 rolling 与 nonneg 全部模式集；
  phase_max/tangent_max 在 base 已超速时改为 u_w ≤ 0（不再不可行）。
- gvf_manager：CBF 参数读取、swarm 块构建 tube+pair CBF 传入 allocator、
  debug 输出 active_constraints/min_tube_cbf_margin/min_pair_cbf_margin；
  gvf/cmd/use_velocity_feedforward（默认 false）把 v_final 写入
  PositionCommand.velocity（计划 7.7），swarm launch 默认 true。
- config：cbf_gamma_pair 4.0、u_delta_slew_rate 2.5。

### 单元测试

phase_offset_cbf_constraints_test：4/4（tube 上/下界约束形式、pair CBF 系数、
近距离激活）。全部 GTest 106 个用例通过。

### ROS 集成测试（test/batch7_cbf_safety_test.py，N=2 收敛平行道）

- EMERGENCY 5/311（1.6%），残差 2.4e-16。
- CBF 激活（min_pair_cbf_margin −0.24），UAV 间距从无 CBF 的 0.53 m 提高到
  0.69 m；目标间距 0.6 m 不安全时 CBF 保持 UAV 分离（终距 2.7 m）。

### 关键修复

1. CBF 约束最初只进 rolling 集，fallback 到 nonneg 时被丢弃——已加入全部模式。
2. 路径初始化瞬态 w_dot_base≈27 使 phase_max 写成 u_w≤−24（不可行）——
   改为 base 超速时 u_w≤0 的语义。
3. velocity feedforward 对 CBF 物理有效性至关重要（计划 7.7）。

### 风险与遗留（重要）

1. 物理最小间距 0.69 m < 理论 d_rob=0.925——position governor + SO3 跟踪滞后
   超出不确定性界；正式 d_rob 保证需要更激进的 feedforward/增益标定或把实测
   跟踪误差计入 d_rob（计划 7.7 的标定路径）。
2. γ_pair 增大（4.0）改善分离但仍有 1.6% EMERGENCY（CBF+变化率冲突的瞬态）。

## 批次 8：ESDF path tube、channel beta、冲突事件与管道可视化

### 实现

- path_tube_builder.h/.cpp：TubeSample/TubeProfile/TubeBounds、rebuild()
  （沿 ±N 射线步进 + 曲率正则边界 + 保守斜率限幅 + beta=(upper−lower)/open_width）、
  query()/queryConservativeLookahead()（窗口内 lower 取 max、upper 取 min；
  只跳过不可认证样本；unknown/out-of-map 样本不伪造空管道）、lineOfSight()。
- TubeSample.certified：路径点与两条射线均位于地图内且 ESDF 已计算时才认证；
  窄通道的零宽样本是“有效压缩区间”（不能当作 unknown 跳过），未知/越界样本
  才是不可认证。
- gvf_manager：tube timer 10 Hz 重建、tube epoch、tube/conflict 可视化话题、
  SwarmConflictState 发布（channel_beta、conflict_state、valid_until）、
  PhaseOffsetDebug 填充 delta_lower/delta_upper/channel_beta/min_tube_cbf_margin；
  allocator 块内 queryConservativeLookahead 接真实 tube，无效时回退固定
  ±delta_fixed_limit；tube 上下界进 appendTubeConstraints（batch 7 已实现）。
- 地图生成：scripts/generate_phase_offset_maps.py 参数化净宽/墙厚/范围，
  输出 open/split_merge/corridor_two_wide(2.0 m)/corridor_single_wide(1.0 m)
  及元数据 YAML；重新生成全部 PCD（旧文件墙体位置错误）。
- 测试脚本改为每次运行独立日志文件，防止残留进程 FD 交叉污染。

### 关键修复

1. “已观测”判据：本模拟器无深度相机，base occupancy buffer 永远停在
   unknown 初值（isUnknown() 恒真），ESDF（distance_buffer_all_，未更新
   体素=10000 哨兵）才是权威信号；offsetBound 改用 ESDF 距离判断已观测。
2. 零宽窄通道样本与 unknown/越界样本分离：前者压缩 delta 区间（有效），
   后者跳过（不伪造可通过窗口）。
3. 单宽走廊 PCD 旧版本墙体位置错误（自由净宽≈0）——重新生成。

### 单元测试

path_tube_builder_test 4/4：开阔区宽管道（beta≈1）、窄走廊压缩（beta<0.6）、
地图未 ready 无效、blocked lookahead 不伪造空管道。

### ROS 集成测试（test/batch8_tube_test.py，单机单宽走廊 1.0 m）

- ok=true；tube_beta_min=0.094（通道压缩生效）、conflict_events=47、
  conflict_narrow_beta=true、delta_inside_tube=true、emergency_samples=0。
- tube markers 207-209 条消息，10 Hz 重建。
- 双宽走廊（2.0 m）对照：beta_min=0.78（保留横向自由度），唯一 conflict
  active 是启动前 2 s ESDF 未就绪的保守瞬态。
- 全部 GTest：13/14 通过；uav_utils-test 因该包 CMake 缺少 add_library
  （链接 -luav_utils 失败）而无法运行，属历史遗留、未改动该包。

### 风险与遗留

1. 启动期（ESDF 未就绪）tube 重建为不可认证样本 → 回退固定 ±delta_fixed_limit；
   此时 conflict 心跳短暂 active=true（保守，符合“地图未 ready 不接管”）。
2. allocator 在路径起点附近出现过瞬态 ROLLING+SAFETY 不可行（base_w_dot 为负时
   phase_min/phase_max 区间矛盾，batch 6 遗留），EMERGENCY 回退后自恢复；
   批次 8 测试窗口内 emergency_samples=0。批次 9/10 若频繁触发再修。
3. uav_utils-test 无法构建（上游 CMake 缺库目标），与本次改动无关。

## 批次 9：C2 增量更新与 delta continuation

### 实现

- PhaseOffsetDebug 增加 continuation 字段：delta_r_norm/delta_rw_norm/
  delta_error_norm/delta_jump/old_delta/min_connector_tube_margin/
  continuation_sequence。
- gvf_manager：activeReferenceStateAtW()（在 w 处按当前 delta 求 r/r_w/error）、
  recordContinuationBefore/After()；四个安装点（initial closed/point、
  closed REPLAN、point REPLAN）在替换路径前后记录指标；跨目标 INIT 重置基线并
  输出 cross_goal_r_shift；超过阈值（r>1e-3、rw>1e-2、delta>1e-9）打 BAD 告警。
- buildPhaseV2C2Frontend()：每个 join_delta_w 候选增加计划 7.9 检查——
  connector 采样点 1-kappa*delta>=mu、活动参考 r=p+N*delta 的障碍距离
  >=reference_clearance（checkC2ConnectorSamples，phase_offset_geometry 新增，
  可单测）、前视 tube 重建并要求 delta 在 [lower,upper] 且宽度
  >=tube/min_connector_width；min tube margin 进入 connector cost。
  tube 容不下 delta 时拒绝安装（继续尝试其他 join_delta_w）。
- 参数：phase_offset/cont_*_threshold、tube/min_connector_width、
  tube/connector_margin_weight。

### 单元测试

phase_offset_continuation_test 4/4：C2 接缝 p/p_w/p_ww 连续、保留 delta 后
r/r_w/e/V 连续、connector 采样检查拒绝正则/障碍违规、tube 不容纳 delta 拒绝安装。

### ROS 集成测试（batch9_continuation_test.py）

- Part A（单机 open 场，test_delta=0.3，飞行中改目标触发异步重规划）：
  7 个 continuation 事件，max_delta_jump=0、max_delta_r=0、max_delta_rw=0、
  delta 不重置（recenter 衰减后无跳回 0.3）、无 NaN、无 EMERGENCY。
- Part B（N=2 隔离）：只改 uav_0 目标时 uav_1 的 odom 100 Hz 不断、
  自身事件 delta_jump=0、delta_r=0；multi simulator 进程数=1。

### 关键修复

1. 双 spinner 线程下 phase_w_ 在 before/after 之间漂移造成假 delta_r——
   记录使用安装时刻的 w 快照（phase_at_switch）。
2. 跨目标 INIT 必须重置 continuation 基线，否则把“跨任务参考移动”误报为
   C2 跳变（保留 cross_goal_r_shift 诊断）。
3. ABI：gvf_manager.h 布局变化后必须重建全部 *_test，否则段错误。

## 批次 10：场景、消融、rosbag 与数据分析

### 实现

- PhaseOffsetDebug 增加 allocator_ms；gvf_manager 实测 allocator 求解耗时。
- phase_offset_swarm/ablation_mode：0 full、1 phase-only（u_delta=0）、
  2 offset-only（u_w=0）、3 no damping、4 no weak cohesion、
  5 direct-add unmatched baseline（g_des 直接叠加，故意破坏 matched）。
- allocator 速度界修复：phase_min/max、tangent_min/max 改为“包含 u=0 的
  鲁棒区间”（base 超速/负速时不再人为不可行）；CBF 优先级分层求解——
  名义 ROLLING 不可行时放宽 slew（30x）但保留 CBF 再解 SAFETY_PRIORITY，
  再 nonneg 及 relaxed nonneg，最后才 EMERGENCY。
- tube CBF margin 自适应管宽：管宽 <2*margin 时收缩 margin，消除上下界
  约束自相矛盾导致的永久 EMERGENCY。
- launch：phase_offset_single_sim/phase_offset_swarm_sim 增加 closed_shape/
  closed_radius/closed_center、ablation_mode、plan_interval、
  alloc_*_slew_rate、cmd_tangent_vel_max、每机 initN_x/y 参数；
  phase_offset_record.launch（单机场景+rosbag record，修正多行 args 的
  “\”字面量 bug）；generate 脚本重生成地图（批次 8 完成）。
- scripts/analyze_phase_offset_bag.py：读 bag 输出 per-UAV 统计
  （odom 频率、里程、e_perp、matched residual、allocator_ms、delta、
   continuation、tube beta、EMERGENCY）、机间最小距离/碰撞数、通信字节数、
  轨迹 PNG。
- 新初始状态文件：split_merge/corridor_two/circle/figure8/hex_wide 三机布局。

### 场景结果（bag 均在 /tmp/gvf_ws_ros_log/bags/）

| 场景 | 最小机间距离 | <0.6m 碰撞采样 | EMERGENCY | e_perp 均值 | 备注 |
|---|---|---|---|---|---|
| 障碍场 N=3 (split_merge) | 1.61 m | 0 | 0/0/6 | 0.02-0.24 | 中心 UAV 绕障，6 次瞬态 EMERGENCY |
| 双宽走廊 N=3 (2.0 m) | 0.137 m | 94 | 0-8 | 0.01-0.02 | 三机排队飞行，接近瞬态仍偏紧 |
| 单宽走廊 N=1 (1.0 m) | - | 0 | 0 | 0.03 | tube beta 压缩到 0.156 |
| 圆 N=3 | 4.97 m | 0 | 0 | 0.22-0.23 | 全部 ROLLING |
| 8 字 N=3 | 3.09 m | 0 | 0 | 0.12-0.15 | 名义路径，闭环 replan 关闭 |

- matched residual 全部场景 ≤2.5e-16；allocator 均值 ~0.013 ms、最大 0.28 ms
  （目标 2/10 ms）；swarm state 通信 3210-3977 条/20-30 Hz。
- 消融（open_hex N=3，28 s）：mode0 min=0.617/e_perp=0.105/emerg=98；
  mode1(phase-only) min=0.619/e_perp=0.064；mode2(offset-only)
  min=0.599/e_perp=0.198；mode3(no damping) min=0.594；mode4(no cohesion)
  min=0.622；mode5(direct-add) resid=0.505（唯一非零 matched residual，
  证明 matched cancellation 的价值）。ablation_stats.json 已保存。

### 关键修复

1. allocator 速度界错误（max(0,...) 使 u_w<=0 恒成立）导致 phase 无法加速；
   修复后圆场景 EMERGENCY 从 375 降到 0、8 字从 40% 降到 0（配合分层求解）。
2. tube CBF margin 与窄管矛盾 → 永久 EMERGENCY（圆场景修复）。
3. 闭环 replan 的拼接 B 样条在 8 字顶部产生 κ≈3 近奇异弯 → tube 变窄 →
   EMERGENCY/碰撞；8 字场景改用名义解析路径（plan_interval=100）后 0
   EMERGENCY、min 间距 3.09m；拼接路径奇异问题列为遗留。
4. 测试脚本残留进程/消息丢失：目标发布改为 latch+重试；每次运行独立日志；
   run 前清理全部 ROS 残留进程。

### 风险与遗留

1. 三机双宽走廊仍存在接近瞬态（min 0.137 m），单宽 1 m 走廊三机在
   d_rob=0.925 下间距预算不足（结构性不可行），仅 N=1 通过；需要走廊专用
   d_rob 标定或队形调度。
2. 8 字闭环 replan 拼接路径的近奇异弯（κ≈3）未根治，当前以“名义路径+
   不重规划”规避；根治需要 connector 对 B 样条候选的曲率上界约束。
3. CBF 物理保证仍受 position governor/SO3 跟踪滞后限制（batch 7 遗留），
   压力场景（0.6 m 目标间距）改用 1.2 m/s 切向限速后确定性通过。
4. uav_utils-test 无法构建（上游 CMake 缺库目标），与本次改动无关。
