# M4A：把集群接入水平横截面 Tube（执行规格）

日期 2026-09-10。状态：已冻结，待执行。
执行者：Luna（唯一编码代理）。主会话负责规格、审核、独立构建与验收。
前置：M3C 已把单机水平横截面 Section tube 切成生产路径（见
`docs/Horizontal_Section_Tube_M3C_Report_2026-09-10.md`）。

## 1. 本批终点

让通用 N 机 SIM-B/SIM-C bringup 在“新水平横截面 Tube + SPH 集群协调”
配置下真正跑起来，并能在隔离 ROS master 上观测到集群意图通过 Tube 改变
各机横向运动：

1. 每个 agent 用自己的 `local_sensing` 完整局部障碍集构建 Section tube，
   即 bringup 必须声明 `phase_offset_tube_cloud_obstacle_set_complete=true`。
2. SPH 协调链路真实接通：neighbor transport + sph provider + gvf SPH
   bridge 同时启用，`phase_offset/coordination_backend=sph` 生效，
   `/uav_N/phase_offset/sph_beta`、`/uav_N/phase_offset/g_coord` 有真实数据。
3. 每个 agent 的 Tube 可视化 topic 各自独立，互不覆盖。
4. 隔离 ROS master 上可跑 N=3 实机仿真并观测到：逐 agent Tube 发布、
   `g_coord` 非零、UAV 实际横向偏离基路径且未越出走廊。

本批只做“接通与可视化隔离”。不改规划器数学、C2、governor、状态机。
不新增证明、证书、地图 ID、reserve、worker、可行性门控。

## 2. 冻结的运行配置（swarm 默认值）

| 参数 | 旧默认 | 新默认 | 原因 |
| --- | --- | --- | --- |
| `phase_offset_mode` | `active` | `manual` | SPH 只在 MANUAL 生产路径下有意义 |
| `phase_offset_coordination_backend` | `disabled` | `sph` | 集群意图来源 |
| `phase_offset_manual_observe_only` | `true` | `false` | 观察模式不提交偏移 |
| `enable_neighbor_transport` | `false` | `true` | 邻居状态平面 |
| `enable_sph_provider` | `false` | `true` | beta / g_coord 供应 |
| `phase_offset_tube_cloud_obstacle_set_complete` | `false` | `true` | 每机 `local_sensing` 即完整集 |
| `phase_offset_manual_tube_source` | `none` | `esdf` | 兼容占位，MANUAL 生产恒建 Section |
| `phase_offset_normal_preview_horizon_w` | `""` | `1.0` | 见下 |
| `phase_offset_normal_preview_sample_spacing_w` | `""` | `0.25` | 见下 |
| `phase_offset_normal_preview_lower_nu` | `""` | `0.02` | 见下 |
| `phase_offset_normal_preview_upper_nu` | `""` | `2.0` | 见下 |
| `phase_offset_normal_preview_b_tight` | `""` | `0.10` | 见下 |
| `phase_offset_normal_preview_b_open` | `""` | `0.90` | 见下 |

六个 preview 值与 `bspline_traj/launch/test_gvf.launch` 现有默认一致。
`phase_offset_sim_bringup/scripts/swarm_orchestrator.py::_validate_options`
已经要求 `sph ⇒ manual + observe_only=false + neighbor + provider + 六个
preview 非空`。这是既有约束，本批不新增任何校验。

## 3. 白名单

只允许修改以下 8 个文件（相对仓库根 `src/swarm_planner/`）：

1. `phase_offset/phase_offset_sim_bringup/launch/phase_offset_swarm.launch`
2. `phase_offset/phase_offset_sim_bringup/launch/phase_offset_agent.launch`
3. `bspline_traj/src/gvf_manager.cpp`
4. `phase_offset/phase_offset_sim_bringup/rviz/phase_offset_sim_b.rviz`
5. `phase_offset/phase_offset_sim_bringup/config/scenarios/sim_b_conflict_3.yaml`（新增）
6. `phase_offset/phase_offset_sim_bringup/config/scenarios/sim_b_formation_3.yaml`（新增）
7. `phase_offset/phase_offset_sim_bringup/test/sim_b_bringup_test.py`
8. `phase_offset/phase_offset_sim_bringup/launch/phase_offset_world.launch`
   （只改默认 `scenario_file`，使只读可视化器与实际编队一致）

## 4. 逐文件精确改动

### F1 `launch/phase_offset_swarm.launch`

只改第 2 节表内 12 个 `<arg>` 的 `default`。结构与参数转发保持不变
（现有 `<param>` 已把每个 arg 转给 orchestrator 节点）。

### F2 `launch/phase_offset_agent.launch`

同一张表里除 `phase_offset_manual_tube_source` 已存在的部分外，把
`phase_offset_mode`、`phase_offset_coordination_backend`、
`phase_offset_manual_observe_only`、`enable_neighbor_transport`、
`enable_sph_provider`、`phase_offset_tube_cloud_obstacle_set_complete`
和六个 `phase_offset_normal_preview_*` 的 `default` 改成第 2 节值。
`phase_offset_manual_tube_source` 默认改 `esdf`。
保留 `spinner_threads` / `use_velocity_feedforward` 兼容空参、保留
`<param name="phase_offset/mode" value="active" if="...">` 这一行不动。
不新增 include、remap、node、param。

### F3 `bspline_traj/src/gvf_manager.cpp`

初稿把 Section tube 可视化发布者改成相对名。隔离实测证明 `gvf_manager`
使用的是私有 `NodeHandle`，相对名会解析成
`/uav_N/formation_planning/phase_offset_section_tube`，而且会把单机话题
从 `/phase_offset_section_tube` 改掉（破坏用户既有单机 RViz）。已回退成
原来的绝对名，只保留注释说明。因此该文件最终**无行为改动**。

逐机隔离改在 F2 的 agent launch 里：

```xml
<remap from="/phase_offset_section_tube"
       to="/$(arg uav_ns)/phase_offset_section_tube"/>
```

单机 `test_gvf.launch` 仍是 `/phase_offset_section_tube`（不变），SIM-B
下每机为 `/uav_N/phase_offset_section_tube`。

### F4 `rviz/phase_offset_sim_b.rviz`

对 `uav_0/1/2` 三组，把已失效的 5 个 Tube 显示
（`phase_offset_manual/base_path`、`active_path`、`frame`、
`tube_candidate`、`tube`）替换为与单机
`uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz` 完全同一套
字段的显示：`Class: rviz/MarkerArray`、`Enabled: true`、
`Name: PhaseOffset Section Tube (left/right)`、`Namespaces: {}`、
`Queue Size: 100`、`Value: true`，`Marker Topic` 为
`/uav_N/phase_offset_section_tube`。三机都打开。其余显示不动。

### F5 `config/scenarios/sim_b_conflict_3.yaml`（新增）

沿用 `sim_b_open_3.yaml` 的 schema，3 机并排同向飞行，横向间距 1.2 m
（进入 `r_safe=1.5` / `r_conf=1.8` 作用域，SPH 必须产生横向意图）：

- robot 0：初始 `(-9.0, -10.0, 1.0, yaw 0)` → 目标 `(-9.0, 2.0, 0.0)`
- robot 1：初始 `(-7.8, -10.0, 1.0, yaw 0)` → 目标 `(-7.8, 2.0, 0.0)`
- robot 2：初始 `(-6.6, -10.0, 1.0, yaw 0)` → 目标 `(-6.6, 2.0, 0.0)`

`name: sim_b_conflict_3`，`frame_id: world`。不改已有 scenario。

`config/scenarios/sim_b_formation_3.yaml`（新增，并设为默认场景）：按
SPH-planning 的终点处理布置。SPH 侧是 `goal_i = init_i + clicked_goal +
init_bias`，即把起始队形整体平移一个公共向量，各机保持相对间隔：

- 起始 `(x, y, z)`：`(-1, 20, 1)`、`(0, 20, 1)`、`(1, 20, 1)`，
  即绕 `(0, 20)` 的 1 m 一列。
- 终点：`(-1, -10, 0)`、`(0, -10, 0)`、`(1, -10, 0)`，整组向南平移
  30 m，间隔不变。

为让 `y=20` 落在规划地图内，`phase_offset_swarm.launch` 的共享地图默认
从 20×30 提升为 `map_size_x=40 / map_size_y=60`（x∈[-20,20]，
y∈[-30,30]），并把 swarm/world 两个 launch 的默认 `scenario_file` 指向
`sim_b_formation_3.yaml`。

### F6 `test/sim_b_bringup_test.py`

在静态契约测试里新增断言（不新增运行时长、不新增 rostest 依赖）：

1. 解析 `phase_offset_swarm.launch` 的 `<arg default>` 得出一份 options
   字典，喂给 `swarm_orchestrator._validate_options`，必须不抛异常；
   scenario 用 `sim_b_conflict_3.yaml`。这直接证明“默认即 SPH-ready”。
2. `phase_offset_agent.launch` 文本含
   `phase_offset_tube_cloud_obstacle_set_complete` 默认 `true`，
   且六个 `phase_offset_normal_preview_*` 默认非空。
3. `sim_b_conflict_3.yaml` 通过 `validate_scenario`，且 3 机横向间距
   为 1.2 m。

## 5. 禁止事项

- 不新增任何参数、门控、证书、证明、安全账本、地图 ID、reserve、worker。
- 不改规划器 / GVF / C2 / governor / 状态机 / 数学核。
- 不改 `test_gvf.launch` 的 `gvf/circle_test/enable|auto_start`。
- 不删除、不注释掉 M3C 已验收的 Section 构建、发布事务、终端逻辑。
- 编码代理不得运行 `catkin_make` / `cmake` / `make` / `roslaunch` /
  `rostest`；构建与 ROS 验证由主会话在隔离目录完成。
- 不连接、不停止用户 ROS 会话。

## 6. 验收

主会话独立执行：

1. 静态：`python3 test/sim_b_bringup_test.py` 全绿（含新增断言）。
2. 构建：隔离 `supervisor/build`（白名单包）+ `full_devel` 全空间
   需要的目标，`formation_planning` 链接通过。
3. 隔离 ROS master 上跑 `phase_offset_world.launch` + `phase_offset_swarm.launch`
   （scenario `sim_b_conflict_3.yaml`），确认：
   - `/uav_0..2/phase_offset_section_tube` 各自有 MarkerArray、含左右边界
     与填充；
   - `/uav_N/phase_offset/sph_beta`、`/uav_N/phase_offset/g_coord` 有数据，
     `g_coord` 至少有一台非零；
   - 至少一台 UAV 的实际横向偏移相对其基路径非零，且不超过该处走廊半宽；
   - coordination 关闭的同场景对照中该偏移消失或明显更小。
4. 记录到 `docs/Horizontal_Section_Tube_M4A_Report_2026-09-10.md`
   （含原始 topic 采样与日志路径）。

## 7. 回滚

只回滚本批白名单文件。不使用 `git reset --hard` / `checkout --` /
`restore .` / `clean` / stash 操作。改动前保存
`supervisor/m4a_source_before.tgz`。
