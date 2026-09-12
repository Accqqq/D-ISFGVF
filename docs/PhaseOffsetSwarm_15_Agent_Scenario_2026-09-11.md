# 15 机集群场景（SPH 风格**圆形**编队）

日期 2026-09-11。状态：已执行并实机验证。

## 1. 占位来源

`related_work/SPH-planning` 的 `sph_zhang_3d.cpp::initParticles` 只是把粒子撒在
0.5 m 网格上，真正决定队形的是 SPH 的密度相互作用——它们会散开成一个**圆形团簇**。
所以这里直接复现那个"稳定后的形状"，构造在
`circular_formation_offsets()`：

```
disk（默认）: 同心圆环，环间距 = formation_spacing
              r = 0, s, 2s, ...；每环点数 = floor(2*pi*r/s)，
              最外圈不足时仍然沿整圈均匀铺开（不留缺口）
ring        : 全部 N 台均匀分布在同一个圆周上
```

- 15 台、s=1.5 m 的 disk → 半径集合 **{0, 1.5, 3.0}**（1 个中心 + 内圈 6 + 外圈 8），
  最小相邻间距 **1.50 m**（原来网格只有 0.5 m，太近）；
- 同一组点用 ring → 半径 3.61 m 的单圈，间距 1.50 m；
- 1.5 m 仍小于邻居判定半径 2.45 m，所以协商意图照旧能把它们耦合起来；
- 形状与台数无关：改 `agent_count` / `formation_shape` 就是新队形，不用手写坐标
  （最初那版向日葵圆盘在 N=15 时半径 0.69~3.69 m 乱散，已弃用）。

15 台 disk 的排布（`*` 为一台无人机，x/y 单位 m）：

```
      |            *            |
      |    *               *    |
      |         *     *         |
      |*     *     *     *     *|
      |         *     *         |
      |    *               *    |
      |            *            |
```

## 2. 新场景文件

`src/swarm_planner/phase_offset/phase_offset_sim_bringup/config/scenarios/sim_b_formation_15.yaml`

- 圆盘按质心居中，launch 再用 `init_x/init_y/init_z`（默认 0/20/1）放置；
- 起点包络 `y ∈ [17.0, 23.0]`，点云实测最小净空 **3.95 m**；
- 终点保持同一圆盘、向南穿越柱阵后收在柱阵之外：
  `y ∈ [-21.00, -15.00]`，点云实测最小净空 **2.31 m**（柱阵 y 到 -13.74 为止）；
- 文件模式与数量模式共用同一个生成器（`sph_disk_scenario()`），不会各写一套。

> 说明：4×4、0.5 m 的方阵无法在柱阵**内部**找到 15 个同时净空 >1 m 的格子
> （柱间距约 1 m），所以终点行放在柱阵南侧之外；穿越过程仍然全程与柱阵交互。

## 3. 运行方式

### 3.1 两条命令，数量写在 **swarm** launch 里（本次采用）

world 仍然单独起，它**不知道也不关心**数量：

```bash
roslaunch phase_offset_sim_bringup phase_offset_world.launch
roslaunch phase_offset_sim_bringup phase_offset_swarm.launch              # 默认 15 台
roslaunch phase_offset_sim_bringup phase_offset_swarm.launch agent_count:=8  # 换数量
```

- `phase_offset_swarm.launch` 新增 `agent_count`（默认 **15**）与
  `formation_spacing`（0.5）、`formation_goal_translation_y`（-34.9）、
  `formation_goal_base_y`；`scenario_file` 默认改为空串。
- `agent_count > 0` 时 orchestrator 用 `sph_grid_scenario()` 现算 4 列网格，
  初始位姿质心放到 `init_x/init_y/init_z`（默认 0/20/1），目标保持同形向南平移
  `formation_goal_translation_y`（默认 → 目标行 -15.6…-14.1，与
  `sim_b_formation_15.yaml` 逐点一致）；也可以用 `formation_goal_base_y` 直接钉住
  最低目标行。
- orchestrator 启动时把 agent 列表写到 `/sim_b_world/scenario/agents`。
- `swarm_visualizer.cpp`：等待该参数（`scenario_wait_timeout` 默认 0 = 一直等，
  正值则有界等待），并且**每 1 s 复查**，列表变化时自动重建订阅——所以换数量只需
  重启 swarm launch，world/RViz 不用动。
- `phase_offset_world.launch` 的 `<rosparam command="load" .../>` 改为只在
  `scenario_file != ''` 时执行（默认空，不再预载 3 机场景）。

### 3.2 显式场景文件（可选）

```bash
roslaunch phase_offset_sim_bringup phase_offset_world.launch
roslaunch phase_offset_sim_bringup phase_offset_swarm.launch \
    agent_count:=0 \
    scenario_file:=$(rospack find phase_offset_sim_bringup)/config/scenarios/sim_b_formation_15.yaml
```

`agent_count:=0` 时行为与之前完全一致（数量由文件决定）。

### 3.3 一条命令（可选的等价入口）

```bash
roslaunch phase_offset_sim_bringup phase_offset_sim.launch agent_count:=15
```

只是把上面两条命令包在一起，方便快速起停；日常仍推荐分开启动。

### 3.4 实测

| 流程 | 结果 |
|---|---|
| `world` 单独起 | visualizer 打印 `still waiting for '/sim_b_world/scenario/agents'`（不再报错退出） |
| `swarm`（无参数） | 15 planners、`generated the SPH-style disk formation for 15 agents (spacing 1.50 m, goal rows …)`、visualizer `observing 15 scenario agents`、15/15 目标、**15/15 `REACHED` + `WAIT_TARGET`**、相位窗口/slew 拒绝 0 |
| 队形检查 | 生成点半径集合 `{0, 1.5, 3.0}`、最小相邻间距 1.50 m；到点后最小两两间距 **1.44 m**、均值 3.24 m、y 全在 [-20.99, -14.99] |
| 杀掉 swarm 后 `swarm agent_count:=3` | **3 planners**、`generated ... for 3 agents`、visualizer `observing 3 scenario agents`（world/RViz 未重启，自动跟随） |
| 文件模式 3 机 | 3 planners、参数从文件加载、3/3 目标 |

编队保持：到点后 15 台的**最小两两间距 1.51 m**（均值 3.6 m），终点 y 全部落在
`[-21.03, -14.99]` 目标带内。

### 3.5 附带修复：管廊视图间歇 UNKNOWN_DOMAIN 导致的单机 hold

圆盘编队的一跑里 uav_4 停在半路：日志显示
`local obstacle view unavailable status=2`（UNKNOWN_DOMAIN）48 次，随后
`synchronous profile unavailable; retain current bundle` —— 局部视图短暂不可用时
tube 不再重建，保留的旧 bundle 的域被**相位跑到外面**；governor 的每个候选都因
`query_w` 落在 executed domain 之外被跳过，于是 `no_valid_candidate` 永久保持
（governor 仍显示 `raw_v_norm=1.43`，参考是有效的）。

修法（`gvf_manager.cpp::runVelocityMatchingGovernor`）：把 executed domain 必须
**覆盖当前相位**作为 `executed_domain_ready` 的条件之一；不覆盖时按"没有 executed
域"处理，回退到标称规划路径（仍保留参考偏移 `reference_delta`），于是视图恢复前
照常飞行，不再整任务 hold。

复验（同一圆盘场景）：`local obstacle view unavailable` 仍出现 259 次，但
**15/15 全部到点 + 进入 WAIT_TARGET，0 次相位/slew 拒绝，无一滞留**。

## 4. RViz

两份配置里的 per-UAV 显示块由 3 份机械扩展到 15 份
（`UAV0..14`，含 ESDF / planner path / kino path / goal / GVF 前端 / 向量场 /
Section Tube / 意图偏移轨迹等）：

- `src/swarm_planner/phase_offset/phase_offset_sim_bringup/rviz/phase_offset_sim_b.rviz`
  （swarm world 用的配置）
- `src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz`

两份都通过 YAML 解析校验。机体/标签/轨迹三条显示本来就是 generic-N
（由 `swarm_visualizer` 按场景发布），自动覆盖 15 台。

occupancy 配色对齐单机：15 个 `UAVn ESDF occupancy inflated`
（`/uav_n/particle0sdf_map/occupancy_inflate`）统一改成单机 baseline 的那套设置
——`Channel Name: z`、`Color Transformer: Intensity`、`Min Color 0; 255; 255` →
`Max Color 180; 20; 140`、`Style: Points`、`Size (m): 0.1`、`Value: true`、
`Use rainbow: false`、`Queue Size: 300`、自动强度范围。改前只有 UAV0 是这套，
UAV1…14 还是 RViz 默认的黑白/彩虹。

## 5. 实机验收（柱阵 + 15 机，全程）

| 指标 | 结果 |
|---|---|
| 进程 | 15 × `formation_planning` + 15 × SO3 控制器 |
| 目标发布 | 15/15（场景目标行 -15.6…-14.1） |
| `[REACHED]` | **15/15**（距离 0.008 ~ 0.05 m） |
| `EXEC_TRAJ -> WAIT_TARGET` | **15 次** |
| 三机时代的两条拒绝 | 0 次（相位窗口 / slew） |
| 终点 | 各自目标点（例：uav_0 (-0.694,-15.595)、uav_14 (0.270,-14.090)） |
| 负载 | 12 核，load ≈ 12.5；仿真实时性下降但流程完整 |

注意：15 台 planner 同时跑会吃满 CPU（每台一套 SDF + kino 重规划 + tube），
如果只看效果可以把 `rviz` 关掉、或把 `simulation_rate` 降低；如果要做长时间
统计实验，建议按核数分批或提高 `planInterval`。
