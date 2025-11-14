# GVF项目使用说明

本项目是一个基于梯度向量场(Gradient Vector Field, GVF)的无人机集群路径规划系统。

## 目录结构

```
gvfproject/
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
# 克隆仓库（请替换为实际的Git仓库地址）
git clone https://github.com/Guangming-Planning-and-Control-Group/GVF_close_loop_planning.git
cd gvfproject
```

### 2. 编译工作空间

```bash
cd gvfproject/
catkin_make
```

### 3. 配置环境变量

```
source ~/gvfproject/devel/setup.bash
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
- **控制命令话题**: `/position_cmd` (common_msgs::PositionCommand)
- **地图尺寸**: 20m × 30m × 2.5m

该launch文件会自动启动：
- `formation_planning` 节点（GVF规划核心）
- `map_generator` 节点（局部感知地图生成）

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

