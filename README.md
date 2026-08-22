# GVF项目使用说明

本项目是一个基于梯度向量场(Gradient Vector Field, GVF)的无人机集群路径规划系统。

> 备注：本仓库本身就是一个 catkin 工作空间根目录（包含 `src/`、`devel/`、`build/`）。

## 目录结构

```
GVF_close_loop_planning/
├── src/
│   ├── swarm_planner/          # 集群规划器模块
│   │   ├── bspline_traj/       # B样条轨迹规划（GVF核心模块）
│   │   ├── common_msgs/        # 通用消息定义
│   │   ├── path_searching/     # 路径搜索算法
│   │   └── plan_env/           # 规划环境（SDF地图）
│   └── uav_simulator/          # 无人机仿真模块
│       ├── dynamic_map_generator/  # 动态地图生成器
│       ├── mockamap/           # 模拟地图
│       ├── so3_control/        # SO3控制器
│       ├── so3_quadrotor_simulator/  # 四旋翼仿真器
│       └── Utils/              # 工具包
```

## 环境要求

- **ROS版本**: ROS Noetic
- **Ubuntu版本**: Ubuntu 20.04 (ROS Noetic)

## Git拉取和使用

### 1. 从Git仓库拉取代码

```bash
git clone https://github.com/Guangming-Planning-and-Control-Group/GVF_close_loop_planning.git
cd GVF_close_loop_planning
```

### 2. 编译工作空间

```bash
cd GVF_close_loop_planning/
catkin_make
```

### 3. 配置环境变量

```
source ~/GVF_close_loop_planning/devel/setup.bash
```

## 运行说明

### 仿真环境运行（test_gvf.launch）

用于仿真环境中测试和调试GVF算法。

#### 启动步骤：

1. **启动仿真器**（启动无人机仿真器和RViz）
   ```bash
   roslaunch so3_quadrotor_simulator simulator.launch
   ```

2. **启动GVF规划器**
   ```bash
   roslaunch bspline_race test_gvf.launch
   ```

#### test_gvf.launch 配置说明

- **点云话题**: `/sim/local_map` (sensor_msgs::PointCloud2)
- **里程计话题**: `/sim/odom` (nav_msgs::Odometry)
- **控制命令话题**: `/position_cmd` (quadrotor_msgs::PositionCommand)
- **地图尺寸**: 20m × 30m × 2.5m

该launch文件会自动启动：
- `formation_planning` 节点（GVF规划核心）
- `map_generator` 节点（局部感知地图生成）

## 代码逻辑与关键参数（按当前实现）

以下内容对应 `formation_planning` 节点当前实现（见 `src/swarm_planner/bspline_traj/src/gvf_manager.cpp` 和 `src/swarm_planner/bspline_traj/src/gvf.cpp`）。

### Phase-offset 云障碍物完备性约定

当 phase-offset ESDF tube 使用 `obstacle_set_complete=true` 时，这不是“点云质量较好”的提示，而是 producer 对其声明 observed domain 的证明：该 domain 内的全部障碍物必须已被表示，且未占据体素必须可证明为已知自由空间。只有满足这项证明的 producer（例如能对自身观测域给出完整障碍集合的 local-sensing producer）才能置为 true；domain 外始终是 unknown。

稀疏真实深度 hit、传感器 FOV 外空间，以及被遮挡形成的阴影体积都不能证明该完备性，必须置为 `obstacle_set_complete=false`。没有完备 cloud-clearance query 时，ESDF tube 会 fail-closed，不会回退到 legacy SDF distance/ray 路径。

### 规划频率（重规划）

- `gvf/planInterval`：重规划周期（秒）。当前 `execTimerCallback()` 会在以下条件触发 `astaropt()`：
  - 首次收到目标点（`receive_startpt`）
  - 碰撞检测触发（带去抖与冷却）
  - 到达 `planInterval` 定时触发

相关去抖参数：
- `gvf/collision_threshold`：距离场阈值，小于该值认为有碰撞风险（单位：m）。
- `gvf/collision_check_horizon_pts`：只检查未来 N 个轨迹点（降低抖动触发概率）。
- `gvf/collision_consecutive_hits`：连续命中 K 个点才算“碰撞风险”（避免阈值附近抖动）。
- `gvf/collision_replan_cooldown`：碰撞触发重规划冷却时间（秒），避免频繁重规划导致抽动。

### 控制输出平滑（cmdCallback）

`formation_planning` 节点在 `cmdCallback()` 中根据 `GVF(pos)` 输出 `quadrotor_msgs::PositionCommand`。当前实现的核心是：**用真实位置 `pos` 查 GVF 速度场**，再通过“滤波 + 积分参考点 + 前视 + 限速”输出连续的 `cmd_pos`，减少路径更新/障碍附近导致的左右抽动。

涉及的主要参数（launch 里以 `gvf/cmd/*` 配置）：

- `gvf/cmd/vel_max`：GVF 输出速度限幅（m/s）。
- `gvf/cmd/vel_lpf_hz`：对 `vel` 的一阶低通截止频率（Hz）。更小更平滑但响应更慢；常用 `2~8`。
- `gvf/cmd/lookahead_time`：前视时间（s），`cmd_pos = ref_pos + T*vel`。更大更“猛”，更小更稳。
- `gvf/cmd/lookahead_dist`：水平最大前视距离（m），防止前视点离机体太远。
- `gvf/cmd/speed_max`：命令点 `cmd_pos` 的最大速度（m/s），用于把“每周期最大位移”与 `dt` 挂钩。
- `gvf/cmd/max_step`：单周期 `cmd_pos` 最大水平位移（m/tick），用于抑制跳点/抽动（过小会变钝）。
- `gvf/cmd/k_pull`：回拉增益，`ref_pos += k_pull*(pos-ref_pos)*dt`。更大更贴身更稳，但可能更慢。

终点附近策略参数：
- `gvf/slow_radius`：进入减速区半径（m）。
- `gvf/stop_radius`：到达目标判定半径（m）。

## 常见问题：障碍物前左右徘徊（抖动/犹豫）

现象：无人机在障碍物前方、通道两侧代价接近时，会出现左右来回“犹豫/徘徊”。

常见原因：
- A* 在左右两条等价通道间来回切换，`/particle0/path` 频繁更新，导致 GVF 的切向方向反复翻转（`gvf::estimateTangentViaQuadraticFit()` 的切向量跟着变）。
- ESDF/栅格离散带来梯度毛刺，叠加重规划/路径更新更明显。

优先调参建议（不改代码）：
- 适当增大 `gvf/planInterval`（减少路径更新频率）。
- 降低 `gvf/cmd/lookahead_time` / `gvf/cmd/lookahead_dist`（减小横向摆动倾向）。
- 适当降低 `gvf/cmd/max_step` 或 `gvf/cmd/speed_max`（抑制单周期跳变）。
- 如仍有抖动，降低 `gvf/cmd/vel_lpf_hz`（增强滤波，例如 3~6Hz；过低会变慢）。
- 如果是碰撞检测引发的频繁重规划，增大 `gvf/collision_replan_cooldown` 或提高 `gvf/collision_consecutive_hits`。

代码层面根治（建议）：
- 在 `src/swarm_planner/bspline_traj/src/gvf_manager.cpp` 的 `gvf_manager::astaropt()` 增加“路径切换滞回”：切换冷却时间、预警区只在新路径显著更安全/更优时才切换，避免左右通道来回翻。

### 实际飞行运行（gvf.launch）

用于真实无人机飞行场景。

#### 启动步骤：

1. **配置点云话题**
   
   在 `gvf.launch` 文件中，确保点云话题配置正确：
   ```xml
   <param name="gvf/cloud_topic" type="string" value="/drone_1_cloud_registered" />
   ```
   
   根据实际系统配置，可能需要修改为：
   - 点云话题：例如 `/drone_X_cloud_registered` 或 `/camera/depth/points`
   - 话题类型必须是 `sensor_msgs::PointCloud2`

2. **配置深度图话题**（如果使用深度图）
   
   参考swarm_realflight配置

3. **配置里程计话题**
   
   确保里程计话题与SLAM系统输出一致：
   ```xml
   <param name="gvf/odom_topic" type="string" value="/drone_1_visual_slam/odom" />
   ```
   

4. **配置控制命令话题**
   
   确保控制命令话题与飞控系统一致：
   ```xml
   <param name="gvf/cmd_topic" type="string" value="/drone_1_planning/pos_cmd" />
   ```


#### gvf.launch 配置说明

- **点云话题**: `/drone_1_cloud_registered` (实际飞行时需要配置)
- **里程计话题**: `/drone_1_visual_slam/odom` (实际飞行时需要配置)
- **控制命令话题**: `/drone_1_planning/pos_cmd`
- **地图尺寸**: 50m × 50m × 3.85m（可根据实际需求调整）

该launch文件会启动：
- `formation_planning` 节点（GVF规划核心）
- `rviz` 可视化
- `px4ctrl` 飞控节点（通过include启动）

## 点云和深度图配置详解

### 点云配置

#### 1. 点云话题配置

在launch文件中设置点云话题：

```xml
<param name="gvf/cloud_topic" type="string" value="/your_pointcloud_topic" />
```



#### 3. 点云话题验证

启动节点前，检查点云话题是否存在：
```bash
rostopic list | grep cloud
rostopic hz /your_pointcloud_topic
rostopic echo /your_pointcloud_topic -n 1
```

### 配置检查清单

在实际飞行前，请确认以下配置：

- [ ] 点云/深度图话题已正确配置
- [ ] 点云/深度图话题正在发布且频率正常（建议>10Hz）
- [ ] 相机内参已正确配置（如果使用深度图）
- [ ] 里程计话题已正确配置
- [ ] 里程计坐标系与点云坐标系一致或已正确转换
- [ ] 控制命令话题与飞控系统匹配
- [ ] 地图尺寸参数适合实际飞行环境
- [ ] TF变换树配置正确（如果使用多坐标系）

## 常用调试命令

### 查看话题列表
```bash
rostopic list
```

### 查看话题频率
```bash
rostopic hz /sim/local_map
rostopic hz /sim/odom
```

### 查看话题内容
```bash
rostopic echo /sim/local_map -n 1
rostopic echo /sim/odom -n 1
```

### 查看节点信息
```bash
rosnode list
rosnode info /formation_planning
```

### 可视化点云
```bash
rviz
# 添加 PointCloud2 显示，选择对应的点云话题
```

### 查看参数
```bash
rosparam list
rosparam get /formation_planning/gvf/cloud_topic
```

## 故障排查

### 1. 点云/深度图无数据

- 检查话题名称是否正确
- 检查话题是否在发布：`rostopic hz /your_topic`
- 检查坐标系设置是否正确
- 查看节点日志：`rosnode info /formation_planning`

### 2. 里程计数据异常

- 检查里程计话题配置
- 确认TF变换正确
- 检查坐标系一致性

### 3. 规划器无响应

- 检查点云和里程计数据是否正常
- 检查地图参数配置（地图尺寸、分辨率等）
- 查看节点日志输出


## 联系方式

如有问题，请联系项目维护者或提交Issue。

---

**注意**: 在实际飞行前，请务必在仿真环境中充分测试，确保系统稳定可靠。
