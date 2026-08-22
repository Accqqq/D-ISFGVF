# PhaseOffsetSwarm ROS 仿真实施计划

> 日期：2026-08-06  
> 目标工程：/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws  
> 理论基准：/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md  
> 导师稿：/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_8.6.md  
> SPH 参考工程：/home/cxq/ISF-GVF/related_work/SPH-planning  
> 单机回归入口：bspline_race/launch/test_gvf.launch 与 so3_quadrotor_simulator/launch/simulator.launch  
> 多机正式入口：待新增的 multi_quadrotor_simulator_so3 与 phase_offset_swarm_sim.launch  
> 本文面向直接修改代码的执行者，建议 DeepSeek 严格按阶段完成，不要一次性重写 gvf_manager.cpp。

---

## 1. 本次实现的边界和强制原则

第一版 ROS 仿真只实现固定高度二维运动，UAV 的真实动力学仍由现有 SO3 仿真器和控制器完成。新算法只输出水平几何引导与现有 PositionCommand 接口，不修改四旋翼动力学模型。

必须遵守以下原则：

1. 每架 UAV 运行一个独立的 formation_planning 节点、局部感知节点、SDF 地图、A*/B 样条规划器和 phase 状态。
2. 不建立一个节点集中读取全部 UAV 后计算统一集群控制量。
3. 每架 UAV 只广播本机状态，并根据本机当前有效邻居独立计算 g_i^{swarm}。
4. 不保持固定队形、不保持固定 phase 差、不固定 UAV 前后顺序。
5. 集群速度意图不能直接叠加到原 GVF 输出上，必须先变成 u_w 与 u_delta，并通过 matched port 同时作用于物理端和内部端。
6. 路径更新时保持 w 和 delta，不允许在正常 C2 更新中把 delta 瞬时投影或重置为零。
7. 正常模式禁止 phase 反向。
8. 不增加独立的正常 hold 模式。只有几何失效、低层失控或安全集合完全不可行时，才允许使用现有低层 emergency/failsafe 悬停。
9. CBF 只作为底层安全保障层，不作为论文核心创新。管道边界和机间距离 CBF 均直接转化为二维端口 QP 的线性约束，不新增复杂 CBF 框架或外部求解器依赖。
10. 所有新功能必须由参数开关控制，默认关闭时，当前单 UAV test_gvf.launch 的行为必须保持不变。
11. 不直接修改或依赖 SPH-planning 工程运行。该工程只作为算法与工程实现参考。
12. 不改写现有 A*、B 样条和 C2 主体，除非计划中明确指出的接口扩展。
13. 多机正式仿真只启动一个动力学 simulator 进程；该进程维护 $N$ 个互不耦合的四旋翼 plant 状态。每架 UAV 的规划、邻居计算、QP/CBF 和 matched port 仍由独立节点完成。

本文的核心实现链为

$$
\boxed{
\text{局部路径 }p_i(w_i)
\rightarrow
\text{活动参考 }r_i(w_i,\delta_i)
\rightarrow
\text{局部邻居速度意图}
\rightarrow
(u_{w,i},u_{\delta,i})
\rightarrow
\text{matched port}
\rightarrow
\text{现有 SO3 仿真闭环}.
}
$$

---

## 2. 当前代码审计结论

### 2.1 可以直接复用的部分

| 现有能力 | 文件 | 复用方式 |
|---|---|---|
| 单机 SO3 动力学仿真 | src/uav_simulator/so3_quadrotor_simulator | 复用 Quadrotor 动力学类，新增一个进程维护 N 个独立实例 |
| 位置控制器 | src/uav_simulator/so3_control | 保持现有控制器，只参数化话题 |
| 局部点云感知 | src/uav_simulator/dynamic_map_generator/src/local_sensing.cpp | 已支持 odom_topic、global_map_topic、local_map_topic 参数，可直接多实例运行 |
| 局部 SDF/ESDF | src/swarm_planner/plan_env | 每机独立实例；用于管道射线采样和 LOS 检查 |
| A*/Kinodynamic A* | src/swarm_planner/path_searching | 保持单机独立规划 |
| B 样条优化 | src/swarm_planner/bspline_traj | 保持原前端 |
| phase_v2 | gvf_manager.cpp 与 gvf.cpp | 作为唯一权威 w 状态 |
| ContinuousPhasePath | continuous_phase_path.h/.cpp | 提供 p、p_w、p_ww 和五次 Hermite C2 拼接 |
| 现有 command governor | gvf_manager.cpp | 继续把几何速度转成 PositionCommand，但必须改为查询活动参考 r |
| 圆与 8 字生成 | gvf_manager.cpp | 用于闭合任务验证 |
| GTest 框架 | bspline_traj/test 与 CMakeLists.txt | 新增几何、邻居、分配器和 continuation 单元测试 |

### 2.2 当前阻碍多 UAV 仿真的问题

1. test_gvf.launch 中 /sim/odom、/sim/local_map 和 /position_cmd 为绝对话题。
2. 当前 `quadrotor_simulator_so3.cpp` 只有一个 `Quadrotor quad`、一组静态 `command/disturbance` 和一套 odom/cmd 话题；它本身只支持一架 UAV。
3. gvf_manager.cpp 中存在 /particle0、/path_vis、/goal_vis、/move_base_simple/goal 等硬编码绝对话题。
4. InitGvf() 内部把 particle_base 固定为 /particle0。
5. 直接 include 七次 simulator.launch 会产生七个动力学进程、七份地图和多个 RViz，不符合本项目希望采用的 SPH 式单仿真入口。
6. formation_planning 默认 AsyncSpinner 为 8 线程，七机将产生至少 56 个 spinner 线程。
7. 当前状态只有 phase_w_，没有 delta_i、tube epoch、port previous command 和 swarm mode。
8. gvf::calcLiftedGuidanceAtPhase() 只围绕 p(w) 计算误差，不支持 r(w,delta)。
9. runVelocityMatchingGovernor() 的前视点由 pathPointAtW() 查询基础路径 p；如果只在 v_cmd 中加入横向 offset，governor 会继续把 UAV 拉回基础路径。
10. C2 connector 目前只检查中心线碰撞，没有检查保留 delta 后的活动参考和管道正则性。
11. common_msgs 中已有 Particle、Force、Swarm_particles 等 SPH 风格消息，但语义容易被误解为物理力，不适合直接作为新方法的正式接口。
12. test_gvf.launch 开启 manual_map_auto_save 时，七个地图实例可能同时写同一个文件。
13. cmdCallback() 使用固定 dt=0.02，应改成由 timer event 计算并限幅的真实周期。

### 2.3 当前符号约定必须保留

现有代码采用

$$
v_{\mathrm{base}}
=
K_1\alpha T+K_2q e_\perp
$$

并在 launch 中设置 gvf_gain2 为负数，例如 -2.2。proposal 中使用的是

$$
K_1\alpha T-K_2q e_\perp,\qquad K_2>0.
$$

第一版修改代码时不要同时改符号约定。应继续使用现有代码的 K2_ 为负数约定，并用单元测试验证法向项指向活动参考。若未来统一论文和代码符号，必须单独提交修改并同步全部参数。

### 2.4 修改前应定位的现有代码锚点

行号会随修改变化，DeepSeek 应先用 `rg -n` 重新定位，不要按固定行号盲改：

| 位置 | 当前大致锚点 | 修改目的 |
|---|---:|---|
| gvf_manager 构造与订阅发布 | gvf_manager.cpp:5、148 附近 | 参数化 robot/topic，创建通信和 tube timer |
| 主 50 Hz 命令循环 | gvf_manager.cpp:799 附近 | 接入 phase--offset、QP、CBF 与 matched port |
| InitGvf | gvf_manager.cpp:1064 附近 | 去掉 `/particle0` 硬编码 |
| C2 builder | gvf_manager.cpp:1722 附近 | 增加当前 delta、活动参考和 tube 检查 |
| 初始闭合/点到点安装 | gvf_manager.cpp:1913、1982 附近 | 初始化 delta 与 epoch |
| 闭合 C2 切换 | gvf_manager.cpp:5130 附近 | 保留 w、delta 并原子安装 path/tube |
| 点到点 C2 切换 | gvf_manager.cpp:5260 附近 | 同上 |
| phase guidance | gvf.cpp:1398 附近 | 新增活动参考版本，不删除原接口 |
| governor | gvf_manager.cpp:367 附近 | 将候选参考从 p 改成 r |

修改前建议执行：

~~~bash
rg -n "cmdCallback|InitGvf|runVelocityMatchingGovernor|buildPhaseV2C2Frontend|setContinuousPhasePath" src/swarm_planner/bspline_traj/src/gvf_manager.cpp
rg -n "calcLiftedGuidanceAtPhase|LiftedGuidanceResult" src/swarm_planner/bspline_traj/include/bspline_race/gvf.h src/swarm_planner/bspline_traj/src/gvf.cpp
~~~

---

## 3. 目标 ROS 架构

### 3.1 总体节点结构

~~~text
全局只启动一次
├── map_pub                     发布 /mock_map
├── multi_quadrotor_simulator_so3 维护 N 个独立 Quadrotor plant
├── so3_nodelet_manager         装载 N 个命名空间隔离的 SO3Control nodelet
├── swarm_scenario_publisher    发布各机目标或场景事件
├── phase_offset_swarm_visualizer 只读汇总全部状态并发布 MarkerArray
└── rviz                        显示所有命名空间

每架 UAV i 独立启动
/uav_i
├── SO3Control nodelet          逻辑独立，但可共享一个 nodelet manager 进程
├── odom_visualization
├── local_sensing
└── formation_planning
    ├── 本机 SDFMap
    ├── 本机 A*/B 样条
    ├── 本机 ContinuousPhasePath
    ├── PhaseOffsetGeometry
    ├── PathTubeBuilder
    ├── NeighborStateBuffer
    ├── ElasticSwarmIntent
    ├── PhaseOffsetAllocator
    ├── matched port
    └── command governor
~~~

`multi_quadrotor_simulator_so3` 只完成 plant 积分：分别读取 `/uav_i/so3_cmd`，分别更新第 $i$ 个 `Quadrotor`，分别发布 `/uav_i/sim/odom` 和 `/uav_i/sim/imu`。它不得计算邻居、集群意图、路径、QP 或 CBF，也不得给不同 UAV 增加任何隐藏耦合。其角色与 Gazebo/物理世界相同，集中运行不改变控制算法的分布式性质。

### 3.2 建议话题

| 作用 | 话题 |
|---|---|
| UAV i 里程计 | /uav_i/sim/odom |
| UAV i IMU | /uav_i/sim/imu |
| UAV i PositionCommand | /uav_i/position_cmd |
| UAV i SO3 命令 | /uav_i/so3_cmd |
| UAV i 局部点云 | /uav_i/sim/local_map |
| UAV i 独立目标 | /uav_i/goal |
| UAV i 基础路径 | /uav_i/particle0/path |
| UAV i kinodynamic path | /uav_i/particle0/kinopath |
| UAV i 活动参考路径 | /uav_i/phase_offset/active_path |
| UAV i 管道边界 | /uav_i/phase_offset/tube_markers |
| UAV i 邻居边 | /uav_i/phase_offset/neighbor_markers |
| UAV i 意图与端口 | /uav_i/phase_offset/debug |
| 所有 UAV 高频状态 | /phase_offset_swarm/state |
| 所有 UAV 调试状态 | /phase_offset_swarm/debug |
| 8 字/重复访问路径事件 | /phase_offset_swarm/path_event |
| 通道/交叉冲突事件 | /phase_offset_swarm/conflict_state |
| 统一 UAV/编号显示 | /phase_offset_swarm/vis/uavs |
| 统一轨迹显示 | /phase_offset_swarm/vis/trajectories |
| 统一基础/活动路径 | /phase_offset_swarm/vis/paths |
| 统一 tube 显示 | /phase_offset_swarm/vis/tubes |
| 统一邻居边与意图 | /phase_offset_swarm/vis/interactions |
| 全局静态点云 | /mock_map |

### 3.3 分布式含义

ROS 仿真中可以让所有 UAV 在同一个共享 state 话题上广播，因为每条消息只包含发送者本机状态。每个 formation_planning 节点独立接收、过滤、构造邻居集合和计算控制。禁止建立一个 swarm_controller 节点集中输出七架 UAV 的端口命令。

一个总 launch、一个 RViz 或一个只读可视化汇总节点不等于集中式控制。参考 SPH 工程的 MarkerArray 显示方式，`phase_offset_swarm_visualizer` 可以同时读取所有 UAV 的状态、路径和调试量并统一画图，但它不得发布 `/position_cmd`、`u_w`、`u_delta`、CBF 约束或任何控制反馈。关闭该节点后，七架 UAV 的飞行结果必须完全不变。

`/mock_map` 只是仿真环境真值的发布源，不属于算法共享地图。每架 UAV 的 `local_sensing` 必须根据本机 odom 独立截取局部点云并发布 `/uav_i/sim/local_map`；`formation_planning`、本机 SDF 和 tube builder 只能读取该本机局部点云，不能直接订阅 `/mock_map`。因此可以通过不同感知范围、丢帧或延迟，使各机异步发现同一个障碍并独立更新路径。

---

## 4. 数学量到代码状态的映射

### 4.1 每机状态

在 gvf_manager 中新增：

~~~cpp
int robot_id_;
double phase_offset_delta_;
double phase_offset_delta_prev_;
double phase_offset_u_w_prev_;
double phase_offset_u_delta_prev_;
bool phase_offset_initialized_;
uint32_t path_epoch_;
uint32_t tube_epoch_;
SwarmControlMode swarm_mode_;
~~~

SwarmControlMode 建议定义为：

~~~cpp
enum class SwarmControlMode {
  DISABLED = 0,
  ROLLING = 1,
  SAFETY_PRIORITY = 2,
  TERMINAL = 3,
  EMERGENCY = 4
};
~~~

### 4.2 Phase--offset 几何

固定高度二维场景中，根据 ContinuousPhasePathState 计算

$$
v_p=\|p_w\|,
\qquad
T=\frac{p_w}{v_p},
\qquad
N=(-T_y,T_x,0).
$$

曲率为

$$
\kappa
=
\frac{p_{w,x}p_{ww,y}-p_{w,y}p_{ww,x}}
{\|p_w\|^3}.
$$

活动参考为

$$
r=p+N\delta,
$$

$$
r_w=(1-\kappa\delta)p_w.
$$

必须检查

$$
1-\kappa\delta\ge\mu>0.
$$

第一版只允许近似水平路径进入 phase--offset 模式。实现中同时检查

$$
\|(p_{w,x},p_{w,y})\|\ge v_{xy,\min},
\qquad
|T_z|\le \epsilon_z.
$$

若不满足，应请求固定高度重规划或暂时关闭 offset 端口，不要把水平法向公式直接用于明显三维路径。`N_z` 始终为零，offset 不修改飞行高度。

误差使用活动参考：

$$
e=x-r,
\qquad
\tau=\frac{r_w}{\|r_w\|}=T,
$$

$$
e_\parallel=\tau^\top e,
\qquad
e_\perp=e-e_\parallel\tau.
$$

基础 ISF 输出为

$$
v_{\mathrm{base}}
=
K_1\alpha\tau+K_2q e_\perp
$$

以及

$$
\dot w_{\mathrm{base}}
=
\frac{K_1(\alpha+\sigma)}
{\|r_w\|}.
$$

### 4.3 局部集群速度意图

组织邻居集合：

$$
\mathcal N_i^{\mathrm{org}}
=
\{j:d_{ij}<R_c,\ fresh,\ LOS,\ semantic\}.
$$

代码中使用进入/退出滞回：不在集合中的邻居按 $R_c^{\mathrm{enter}}$ 加入，已在集合中的邻居按更大的 $R_c^{\mathrm{exit}}$ 删除，避免距离在边界附近时每帧抖动。

安全邻居集合：

$$
\mathcal N_i^{\mathrm{safe}}
=
\{j:d_{ij}<R_{\mathrm{safe}}\ \text{或}\ TTC_{ij}<T_{\mathrm{safe}}\}.
$$

定义

$$
n_{ij}=\frac{x_j-x_i}{d_{ij}},
\qquad
\dot d_{ij}=(v_j-v_i)^\top n_{ij}.
$$

第一版紧支撑权重可直接使用

$$
a_{ij}=W(d_{ij})=
\begin{cases}
\frac12\left[1+\cos\left(\pi d_{ij}/R_c\right)\right],&d_{ij}<R_c,\\
0,&d_{ij}\ge R_c.
\end{cases}
$$

距离允许带：

$$
\varphi_{ij}(d)=
\begin{cases}
-k_r(d_--d),&d<d_-,\\
0,&d_-\le d\le d_+,\\
\beta_{ij}k_c(d-d_+),&d_+<d<R_c,\\
0,&d\ge R_c.
\end{cases}
$$

$$
g_i^{\mathrm{pos}}
=
\sum_{j\in\mathcal N_i^{\mathrm{org}}}
a_{ij}\varphi_{ij}(d_{ij})n_{ij},
$$

$$
g_i^{\mathrm{damp}}
=
k_d
\sum_{j\in\mathcal N_i^{\mathrm{org}}}
a_{ij}\dot d_{ij}n_{ij}.
$$

第一版保留有界软安全项，但它不代替硬 CBF 安全层。规定 $d_{\mathrm{safe}}<d_{\mathrm{act}}\le d_-$，并使用

$$
d_{ij,\epsilon}=\sqrt{d_{ij}^2+\epsilon_d^2},
\qquad
n_{ij,\epsilon}=\frac{x_j-x_i}{d_{ij,\epsilon}},
$$

$$
\widetilde g_i^{\mathrm{safe}}
=
-\sum_{j\in\mathcal N_i^{\mathrm{safe}}}
a_{ij}^{\mathrm{safe}}k_s
\left[
\frac1{d_{ij,\epsilon}}-\frac1{d_{\mathrm{act}}}
\right]_+
\frac{n_{ij,\epsilon}}{d_{ij,\epsilon}^2},
$$

$$
g_i^{\mathrm{safe}}
=
\operatorname{sat}_{g_{\max}^{\mathrm{safe}}}
\left(\widetilde g_i^{\mathrm{safe}}\right).
$$

然后：

$$
g_i^{\mathrm{swarm}}
=
g_i^{\mathrm{pos}}
+g_i^{\mathrm{damp}}
+g_i^{\mathrm{safe}}.
$$

回中偏好：

$$
g_i^{\mathrm{des}}
=
g_i^{\mathrm{swarm}}-k_{\mathrm{rec}}\delta_iN_i.
$$

### 4.4 无约束端口

$$
J_i=[r_{w,i}\quad N_i].
$$

由于两列正交，

$$
u_{w,i}^{\mathrm{raw}}
=
\frac{r_{w,i}^\top g_i^{\mathrm{des}}}
{\|r_{w,i}\|^2},
$$

$$
u_{\delta,i}^{\mathrm{raw}}
=
N_i^\top g_i^{\mathrm{des}}.
$$

### 4.5 鲁棒机间 CBF 安全约束

只接受消息年龄满足

$$
\tau_{ij}\le\tau_{\max}
$$

的邻居状态。根据时间戳预测邻居当前位置 $\hat x_j$，定义

$$
r_{ij}=x_i-\hat x_j.
$$

鲁棒安全距离至少包含机体距离、跟踪误差、通信时延和采样保持裕度：

$$
d_{\mathrm{safe},ij}^{\mathrm{rob}}
=
d_{\mathrm{safe}}
+\bar e_{x,i}+\bar e_{x,j}
+\frac12\bar a_j\tau_{\max}^2
+\frac12(\bar a_i+\bar a_j)\Delta t_{\mathrm{QP,max}}^2
+\bar e_{\mathrm{comm},ij}.
$$

定义 barrier：

$$
h_{ij}^{\mathrm{rob}}
=
\|r_{ij}\|^2
-\left(d_{\mathrm{safe},ij}^{\mathrm{rob}}\right)^2.
$$

设相对速度、执行和采样误差满足

$$
\|d_{v,ij}\|\le\bar d_{v,ij},
\qquad
\rho_{ij}^{\mathrm{rob}}
=2\|r_{ij}\|\bar d_{v,ij}.
$$

`velocity_uncertainty_bound` 不能简单写死为一个常数。`NeighborSelector` 每周期根据实际消息年龄 $\tau_{ij}$ 计算：

$$
\bar d_{v,ij}
=
\bar e_{v,i}^{\mathrm{est}}
+\bar e_{v,j}^{\mathrm{msg}}
+\bar a_j\tau_{ij}
+(\bar a_i+\bar a_j)\Delta t_{\mathrm{QP,max}}
+\bar d_i^{\mathrm{exec}}
+\bar d_j^{\mathrm{exec}}.
$$

然后把该周期的结果写入不可变 `PredictedNeighborState`。CBF 约束生成器不得重新读取 ROS 时间或原始消息。

候选物理速度为

$$
v_i(u_i)=v_{\mathrm{base},i}+J_i u_i.
$$

每架 UAV 在本机 QP 中加入

$$
\boxed{
2r_{ij}^\top
\left(v_i(u_i)-\hat v_j\right)
\ge
-\gamma_s h_{ij}^{\mathrm{rob}}
+\rho_{ij}^{\mathrm{rob}}.
}
$$

它对 $u_i=(u_{w,i},u_{\delta,i})$ 是线性的。第一版采用“本机对当前邻机预测承担完整鲁棒责任”的约束，不做 pairwise handshake 或 $1/2+1/2$ 责任拆分；这样更保守，但实现清楚。安全结论必须同时注明消息年龄、误差界、QP 周期上界和持续可行性假设。

CBF 只过滤最终端口，不直接修改 `v_cmd`，因此不会破坏 matched cancellation。超过消息时限、CBF 集合不可行或低层执行误差超界时，按 `SAFETY_PRIORITY -> EMERGENCY` 处理。

正式安全实验必须从 $h_{ij}^{\mathrm{rob}}\ge0$ 的初始状态开始，并验证每周期 QP 可行。若仿真一开始就把 UAV 放在鲁棒安全距离以内，CBF 只能尽力恢复，不能声称集合前向不变。

### 4.6 端口约束

Rolling 模式至少包含：

$$
\underline\nu_w
\le
\dot w_{\mathrm{base}}+u_w
\le
\overline\nu_w,
$$

$$
K_1\alpha+\|r_w\|u_w
\ge
v_{\mathrm{tan,min}}>0,
$$

$$
|u_\delta|\le u_{\delta,\max},
$$

$$
|u_w-u_w^{\mathrm{prev}}|
\le a_{u_w}\Delta t,
$$

$$
|u_\delta-u_\delta^{\mathrm{prev}}|
\le a_{u_\delta}\Delta t.
$$

定义带鲁棒收缩裕度的 tube barrier：

$$
h_i^+
=
\overline\delta_i(w_i)-\delta_i-m_\delta,
\qquad
h_i^-
=
\delta_i-\underline\delta_i(w_i)-m_\delta.
$$

将以下 CBF 约束加入同一个二维 QP：

$$
\boxed{
\overline\delta_i'(w_i)
\left(\dot w_{\mathrm{base},i}+u_{w,i}\right)
-u_{\delta,i}
\ge-\gamma_\delta h_i^+,
}
$$

$$
\boxed{
u_{\delta,i}
-\underline\delta_i'(w_i)
\left(\dot w_{\mathrm{base},i}+u_{w,i}\right)
\ge-\gamma_\delta h_i^-.
}
$$

tube 边界需要分段 $C^1$ 且导数有界；不光滑采样点使用保守单侧导数或短前视离散检查。QP 更新周期必须不超过 `dt_qp_max`，采样保持误差计入 $m_\delta$。

点到点任务接近有限路径终端时，`phase_speed_min>0` 不再适用。进入现有 endpoint margin 后切换到 `TERMINAL`：允许总 phase 速度连续下降至零，保留 CBF 和法向误差收敛，随后由现有目标到达逻辑结束任务。该模式只表示有限路径完成，不是狭窄通道中的正常 hold。多 UAV 实验不要让七架 UAV 同时收敛到同一个精确点；应使用不同终端点，或以全部 UAV 越过出口平面作为结束条件。

### 4.7 Matched port

求得最终端口

$$
u_i^\star=
\begin{bmatrix}
u_{w,i}^\star\\
u_{\delta,i}^\star
\end{bmatrix}
$$

后，必须同时执行

$$
v_i^{\mathrm{match}}
=
r_{w,i}u_{w,i}^\star
+N_i u_{\delta,i}^\star,
$$

$$
\dot w_i
=
\dot w_{\mathrm{base},i}+u_{w,i}^\star,
$$

$$
\dot\delta_i=u_{\delta,i}^\star,
$$

$$
v_i^{\mathrm{final}}
=
v_{\mathrm{base},i}
+v_i^{\mathrm{match}}.
$$

禁止只修改 `phase_w_` 和 `phase_offset_delta_` 而不修改物理 `v_cmd`；也禁止只把 $J_i u_i$ 加到 `v_cmd` 而不更新内部 $w,\delta$。

代码审查和单元测试必须使用完整误差导数，而不只检查 `v_match` 的数值拼接。令

$$
e_i=x_i-r_i(w_i,\delta_i),
$$

则同一个最终端口同时进入三个通道后有

$$
\begin{aligned}
\dot e_i
&=
\dot x_i-r_{w,i}\dot w_i-N_i\dot\delta_i\\
&=
\left(
v_{\mathrm{base},i}+r_{w,i}u_{w,i}^\star+N_i u_{\delta,i}^\star
\right)
-r_{w,i}\left(
\dot w_{\mathrm{base},i}+u_{w,i}^\star
\right)
-N_i u_{\delta,i}^\star\\
&=
v_{\mathrm{base},i}-r_{w,i}\dot w_{\mathrm{base},i}.
\end{aligned}
$$

代入当前 ISF 结构得到

$$
\boxed{
\dot e_i
=
K_2q_i e_{\perp i}
-K_1\sigma(e_{\parallel i})\tau_i,
}
$$

其中代码约定 `K2_ < 0`。这就是需要保留的精确误差动力学。无论端口来自人工测试、邻居意图还是约束分配器，只要物理端和内部端使用同一个最终解，端口项都必须代数抵消。

---

## 5. 建议新增的代码文件

### 5.1 Phase--offset 公共类型

新增：

- src/swarm_planner/bspline_traj/include/bspline_race/phase_offset_types.h

建议结构：

~~~cpp
struct PhaseOffsetGeometry {
  Eigen::Vector3d p;
  Eigen::Vector3d dp_dw;
  Eigen::Vector3d d2p_dw2;
  Eigen::Vector3d T;
  Eigen::Vector3d N;
  Eigen::Vector3d r;
  Eigen::Vector3d r_w;
  Eigen::Vector3d e_perp;
  Eigen::Vector3d base_v;
  double curvature;
  double delta;
  double e_parallel;
  double rho;
  double alpha;
  double q;
  double sigma;
  double base_w_dot;
  bool valid;
};

struct TubeBounds {
  double query_w;
  double lower;
  double upper;
  double lower_dw;
  double upper_dw;
  double beta;
  double min_regularity;
  ros::Time stamp;
  uint32_t epoch;
  bool valid;
};

struct NeighborState {
  int robot_id;
  ros::Time stamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  bool fresh;
};

struct PredictedNeighborState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int robot_id;
  ros::Time source_stamp;
  double age;
  Eigen::Vector3d predicted_position;
  Eigen::Vector3d advertised_velocity;
  double velocity_uncertainty_bound;
  bool safety_active;
};

struct NeighborSelection {
  std::vector<NeighborState> organization;
  std::vector<PredictedNeighborState,
              Eigen::aligned_allocator<PredictedNeighborState>> safety;
  bool communication_fault;
};

struct NeighborEventState {
  int robot_id;
  int branch_event_id;
  uint32_t branch_sequence;
  ros::Time branch_valid_until;
  double channel_beta;
  uint32_t conflict_sequence;
  ros::Time conflict_valid_until;
  bool branch_active;
  bool conflict_active;
};

struct PairIntentContribution {
  int robot_id;
  double distance;
  double distance_rate;
  Eigen::Vector3d n_ij;
  Eigen::Vector3d g_pos;
  Eigen::Vector3d g_damp;
  Eigen::Vector3d g_safe;
};

struct SwarmIntent {
  Eigen::Vector3d g_pos;
  Eigen::Vector3d g_damp;
  Eigen::Vector3d g_safe;
  Eigen::Vector3d g_swarm;
  Eigen::Vector3d g_des;
  double min_neighbor_distance;
  int organization_neighbor_count;
  int safety_neighbor_count;
  std::vector<int> organization_neighbor_ids;
  std::vector<int> safety_neighbor_ids;
  std::vector<PairIntentContribution> pair_contributions;
};

struct PortCommand {
  double u_w;
  double u_delta;
  double objective;
  double allocation_residual;
  bool feasible;
  SwarmControlMode mode;
  std::vector<std::string> active_constraints;
};
~~~

### 5.2 Phase--offset 几何模块

新增：

- include/bspline_race/phase_offset_geometry.h
- src/phase_offset_geometry.cpp

建议接口：

~~~cpp
class PhaseOffsetGeometryEvaluator {
public:
  bool evaluate(const ContinuousPhasePathState& path_state,
                const Eigen::Vector3d& position,
                double delta,
                const PhaseOffsetGeometryParams& params,
                PhaseOffsetGeometry& out) const;
};
~~~

职责：

1. 计算 T、N、曲率、r、r_w。
2. 检查 p_w 非退化和 1-kappa*delta。
3. 相对活动参考分解误差。
4. 计算 base_v 和 base_w_dot。
5. 不读取 ROS 话题，不保存控制历史，保证易于单元测试。

### 5.3 路径管道模块

新增：

- include/bspline_race/path_tube_builder.h
- src/path_tube_builder.cpp

建议接口：

~~~cpp
class PathTubeBuilder {
public:
  void setMap(const std::shared_ptr<SDFMap>& map);
  bool rebuild(const ContinuousPhasePath& path,
               double current_w,
               double current_delta,
               TubeProfile& profile);
  bool query(double w, TubeBounds& bounds) const;
  bool queryConservativeLookahead(double w0,
                                  double w1,
                                  TubeBounds& bounds) const;
  bool lineOfSight(const Eigen::Vector3d& a,
                   const Eigen::Vector3d& b) const;
};
~~~

`query()` 与 `queryConservativeLookahead()` 返回的 `TubeBounds` 必须同时填充当前查询相位、上下边界导数、时间戳和 epoch；CBF 层不允许再从离散数组自行差分。分段不光滑点由 tube builder 明确选择保守单侧导数并在 debug 中标记。

第一版实现：

1. 在前视区间按 tube/sample_step_w 采样路径。
2. 对每个样本沿正负 N 方向射线步进。
3. 查询 SDFMap::getDistance() 和 getInflateOccupancy()。
4. 当参考点安全余量小于 reference_clearance 时停止。
5. 将 offset 限制在 max_offset。
6. 合并曲率正则边界 1-kappa*delta>=mu。
7. 对原始上下界做保守移动最小值和斜率限幅。
8. 计算 beta_i=(upper-lower)/open_width 并限制到 0 到 1。

tube builder 只有在以下条件同时满足时才返回 `valid=true`：

1. 本机 odom 已有效；
2. 至少收到一次本机 local map 观测；
3. ESDF 已完成一次对应更新，并具有非零 timestamp/version；
4. 当前路径采样点和法向射线位于地图有效范围内。

当前 SDF 距离缓存初始化可能表现为很大距离，因此“尚未观测、out-of-map、unknown”在 tube 和 LOS 中默认按不可用/阻塞处理，不能当成开阔自由空间。地图尚未 ready 时：

- `TubeBounds.valid=false`；
- offset/swarm 端口不接管控制；
- 只允许运行原有基础初始化/规划或进入安全等待；
- 禁止安装宽度看似很大的假 tube。

`reference_clearance` 不是单纯的机体半径。至少应满足

$$
d_{\mathrm{clear}}
=
r_{\mathrm{uav}}
+\bar e_{\mathrm{track}}
+m_{\mathrm{loc}}
+m_{\mathrm{map}},
$$

其中分别表示机体包络半径、低层跟踪误差界、定位裕度和地图/离散采样裕度。tube 上下边界描述的是活动参考点可取的 offset 区间，真实 UAV 的误差球已经通过上述收缩计入，不能只保证中心参考点不碰障碍。

tube 在线更新必须区分三种情况：

1. 新 tube 仍包含当前 `delta`，并且 rolling 可行集非空：直接增加 `tube_epoch` 并连续使用；
2. 新 tube 不再包含当前误差球，但 nonnegative-progress 可行集仍非空：采用最新安全边界，进入 `SAFETY_PRIORITY`，连续收缩 offset、触发中心线重规划，禁止继续把旧 tube 当作安全依据；
3. nonnegative-progress 也不可行：进入低层 emergency/failsafe。

普通 tube 更新不重置 path epoch，也不对 `delta` 做瞬时 clip。

注意：第一阶段可先用固定对称管道 [-delta_fixed,+delta_fixed] 验证 matched port，再接 ESDF 管道。

### 5.4 邻居状态与集群意图模块

新增：

- include/bspline_race/swarm_neighbor_model.h
- src/swarm_neighbor_model.cpp

建议拆成两个类：

~~~cpp
class NeighborStateBuffer {
public:
  void update(const common_msgs::SwarmState& msg);
  void prune(const ros::Time& now,
             double neighbor_timeout,
             double retention_timeout);
  std::vector<NeighborState> snapshot() const;
};

class SwarmEventBuffer {
public:
  void updatePathEvent(const common_msgs::SwarmPathEvent& msg);
  void updateConflictState(const common_msgs::SwarmConflictState& msg);
  void prune(const ros::Time& now);
  NeighborEventState query(int robot_id) const;
  std::unordered_map<int, NeighborEventState> snapshot() const;
};

class NeighborSelector {
public:
  NeighborSelection select(
      const ros::Time& now,
      const Eigen::Vector3d& self_pos,
      const Eigen::Vector3d& self_vel,
      const std::vector<NeighborState>& raw_neighbors,
      const std::unordered_map<int, NeighborEventState>& events,
      SDFMap* map,
      const NeighborSelectionParams& params) const;
};

class ElasticSwarmIntent {
public:
  SwarmIntent compute(const Eigen::Vector3d& self_pos,
                      const Eigen::Vector3d& self_vel,
                      double self_beta,
                      const NeighborSelection& selection,
                      const std::unordered_map<int, NeighborEventState>& events,
                      SDFMap* map,
                      const SwarmIntentParams& params) const;
};
~~~

职责：

1. 忽略 self robot_id。
2. 使用消息时间戳计算 `age=(now-stamp).toSec()`，名义预测 `predicted_position=position+velocity*age`，未建模加速度只进入鲁棒距离和 `velocity_uncertainty_bound`；
3. `NeighborSelector` 明确返回 `NeighborSelection.organization` 与 `NeighborSelection.safety`；超时邻居立即退出组织凝聚，但不能从安全逻辑中静默消失。若最近一次状态表明其仍可能进入安全半径，则设置 `communication_fault`，由 allocator 进入 `SAFETY_PRIORITY` 或 `EMERGENCY`。
4. 计算紧支撑权重 W。
5. 计算允许距离带、径向阻尼和有界软安全。
6. LOS 只影响组织边，不关闭近距离安全边。
7. beta_ij=min(beta_i,beta_j) 只作用于弱凝聚。
8. `prune()` 只清理超过更长 retention horizon 的历史项；`neighbor_timeout` 与 `retention_timeout` 分开配置，避免消息刚超时就丢失最后已知安全状态；
9. 从 `SwarmEventBuffer` 读取语义兼容性和邻居 `channel_beta`；事件缺失或失效时，开阔区默认 `beta=1`，语义事件默认不激活；
10. `ElasticSwarmIntent` 只从 `selection.organization` 计算位置与阻尼，从 `selection.safety` 计算软安全；
11. 输出逐邻居调试贡献，供七机示例验证。

现有 `SDFMap::getDistance()` 与 `getInflateOccupancy()` 不是 `const` 成员，因此第一版接口使用 `SDFMap*` 或 `SDFMap::Ptr`。不要照抄为 `const SDFMap*` 导致编译失败；若以后需要只读语义，应在 `SDFMap` 中补充经过验证的 const API。

### 5.5 二维端口分配器

新增：

- include/bspline_race/phase_offset_allocator.h
- src/phase_offset_allocator.cpp

不要第一版就引入大型外部 QP 库。由于只有两个变量，可实现一个二维凸 QP active-set 枚举器。

约束统一表示为

~~~cpp
struct LinearConstraint2D {
  Eigen::Vector2d a;
  double b;  // a.dot(u) >= b
  std::string label;
};
~~~

目标为

$$
\frac12u^\top H u+f^\top u.
$$

`H` 和 `f` 不能凭经验硬写，必须由下式展开得到：

$$
\min_u
\frac12\|J_i u-g_i^{\mathrm{des}}\|_{W_g}^2
+\frac12u^\top R_u u
+\frac12(u-u_i^{\mathrm{prev}})^\top R_\Delta
(u-u_i^{\mathrm{prev}}).
$$

由于 $u_w$ 的单位是 phase/s，$u_\delta$ 的单位是 m/s，$R_u$ 和 $R_\Delta$ 必须使用分端口尺度的对角权重，不能用一个无量纲标量同时惩罚两者。也可以用 $S_i=\operatorname{diag}(\|r_{w,i}\|,1)$ 先把两个端口统一映射到 m/s 再构造权重。无约束解应与第 4.4 节解析分解在数值上相符；该测试不通过时，不进入多机控制。

求解候选：

1. 无约束最优解。
2. 每一条约束边界上的最优投影。
3. 任意两条约束边界的交点。
4. 选取全部可行候选中代价最小者。

单机有效线性约束数量 $m$ 通常不超过 20，二维枚举的 $O(m^2)$ 成本足以运行在 50 Hz。必须为求解器写独立单元测试。

分配器接口：

~~~cpp
PortCommand solve(const PhaseOffsetGeometry& geometry,
                  const TubeBounds& tube,
                  const SwarmIntent& intent,
                  const std::vector<PredictedNeighborState,
                      Eigen::aligned_allocator<PredictedNeighborState>>& safety_neighbors,
                  double dt,
                  const PortCommand& previous);
~~~

求解顺序：

1. 尝试 rolling constraints。
2. rolling 不可行时，尝试 nonnegative-progress constraints。
3. 第二次可行则输出 SAFETY_PRIORITY，并请求重规划。
4. 两次均不可行才输出 EMERGENCY。

建议把 CBF 约束生成写成分配器旁边的小型纯 C++ 模块，而不是引入新的控制框架：

- include/bspline_race/phase_offset_cbf_constraints.h
- src/phase_offset_cbf_constraints.cpp

~~~cpp
class PhaseOffsetCbfConstraints {
public:
  void appendTubeConstraints(const PhaseOffsetGeometry& geometry,
                             const TubeBounds& tube,
                             std::vector<LinearConstraint2D>& constraints) const;

  void appendPairwiseConstraints(
      const PhaseOffsetGeometry& geometry,
      const Eigen::Vector3d& self_position,
      const std::vector<PredictedNeighborState,
          Eigen::aligned_allocator<PredictedNeighborState>>& safety_neighbors,
      std::vector<LinearConstraint2D>& constraints) const;
};
~~~

该类只负责把解析 CBF 写成 `a.dot(u) >= b`，不保存 UAV 状态、不发布命令，也不改变 matched port。`appendTubeConstraints()` 必须读取 `query_w、lower/upper、lower_dw/upper_dw、stamp`；`appendPairwiseConstraints()` 只读取已经完成时间预测和不确定性计算的 `PredictedNeighborState`，禁止在 CBF 层再次猜测消息年龄。

### 5.6 可视化与日志模块

新增：

- include/bspline_race/phase_offset_visualizer.h
- src/phase_offset_visualizer.cpp
- scripts/phase_offset_swarm_visualizer.py，或等价的独立 C++ 节点

分成两层：

1. 每机 `PhaseOffsetVisualizer` 产生本机调试消息，并把基础路径、活动路径和 tube marker 直接发布到共享可视化话题；依靠唯一 `ns/id` 区分机器人；
2. 每机同时向共享 `/phase_offset_swarm/debug` 多发布者话题发送 `PhaseOffsetDebug`；
3. 全局只读 `phase_offset_swarm_visualizer` 参考 SPH 的 `particles_vis`/`sph_vis` 方式，订阅共享 state/debug，按 `robot_id` 汇总 UAV 本体、编号、历史轨迹、邻居边和意图箭头。

这样 RViz 只需要订阅少量统一话题，不需要手工添加七套 `/uav_i/...` display，也不要求 ROS1 的 wildcard subscriber。

统一显示内容：

1. 基础路径 p。
2. 活动路径 r。
3. tube 左右边界。
4. 当前 p、r、x。
5. T、N、g_pos、g_damp、g_safe、g_des。
6. v_base、v_match、v_final。
7. organization edges 与 safety edges，使用不同颜色。
8. 文本显示 robot_id、w、delta、beta、mode 和邻居数量。
9. 每架 UAV 的历史轨迹，使用固定且不同的颜色。
10. 当前激活的 pair/tube CBF 约束，使用红色或橙色标记。

Marker 必须满足：

~~~text
header.frame_id = "world"
marker.ns = "uav_<robot_id>/<layer>"
marker.id = robot_id * 1000 + local_marker_id
~~~

这样七架 UAV 即使通过同一个 MarkerArray 话题发布，也不会互相覆盖。轨迹 marker 使用 `LINE_STRIP`，邻居边使用 `LINE_LIST`，UAV 本体使用 `SPHERE` 或 mesh，编号使用 `TEXT_VIEW_FACING`。

该汇总节点是纯观察者。必须用回归测试确认启动或关闭它不会改变任何 UAV 的命令话题和轨迹。

### 5.7 单进程多四旋翼动力学仿真器

保留当前 `quadrotor_simulator_so3` 作为单机回归，不直接把它改成只能多机运行。新增：

- src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/multi_quadrotor_simulator.h
- src/uav_simulator/so3_quadrotor_simulator/src/multi_quadrotor_simulator_so3.cpp
- src/uav_simulator/so3_quadrotor_simulator/launch/multi_simulator.launch

建议先把当前文件中的 `Command、Disturbance、getControl、stateToOdomMsg、quadToImuMsg` 抽成可复用公共代码，再定义：

~~~cpp
struct SimulatedQuadrotorAgent {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int robot_id;
  std::string name;
  QuadrotorSimulator::Quadrotor quad;
  Command command;
  Disturbance disturbance;
  Control last_control;
  ros::Time last_command_stamp;
  bool command_received;
  bool failsafe_active;
  ros::Publisher odom_pub;
  ros::Publisher imu_pub;
  ros::Subscriber so3_cmd_sub;
  ros::Subscriber force_disturbance_sub;
  ros::Subscriber moment_disturbance_sub;
};

class MultiQuadrotorSimulator {
public:
  bool initialize(ros::NodeHandle& nh, int num_agents);
  void step(double dt);
  void publish(const ros::Time& stamp);

private:
  std::vector<std::unique_ptr<SimulatedQuadrotorAgent>> agents_;
};
~~~

一个仿真周期中只做：

~~~cpp
for (auto& agent_ptr : agents_) {
  auto& agent = *agent_ptr;
  const Control control = getControl(agent.quad, agent.command);
  agent.quad.setInput(control.rpm[0], control.rpm[1],
                      control.rpm[2], control.rpm[3]);
  agent.quad.setExternalForce(agent.disturbance.f);
  agent.quad.setExternalMoment(agent.disturbance.m);
  agent.quad.step(dt);
}
~~~

每个 agent 使用独立话题：

~~~text
/uav_i/so3_cmd
/uav_i/sim/odom
/uav_i/sim/imu
/uav_i/force_disturbance
/uav_i/moment_disturbance
~~~

必须满足：

1. `num_agents=1` 时，与原单机 simulator 在相同输入下数值一致；
2. 第 $i$ 架 UAV 的命令只能改变第 $i$ 个 plant；
3. simulator 不订阅 `/phase_offset_swarm/state`、邻居边或集群意图；
4. simulator 内部不进行 UAV 间排斥、碰撞修正或统一规划；
5. 任一 UAV 控制节点退出时，其他 plant 继续积分；失去命令的 plant 采用明确的超时策略；
6. 七机仿真只有一个 `multi_quadrotor_simulator_so3` 进程。

禁止把包含 Eigen 固定尺寸成员和 ROS subscriber 的 agent 直接放入可扩容 `std::vector<Agent>`，也禁止 callback 捕获 vector 元素引用后继续扩容。初始化顺序固定为：

1. 从参数读取 `num_agents`；
2. 创建全部 `unique_ptr<SimulatedQuadrotorAgent>`；
3. 显式清零 `Command/Disturbance/Control`，设置单位四元数、初始时间和 hover/failsafe 状态；
4. 全部对象地址稳定后，再使用 robot_id 绑定 subscriber callback；
5. 最后开始动力学 timer。

初始状态 YAML 通过 launch `<rosparam>` 加载到 ROS 参数服务器，并使用 `XmlRpc::XmlRpcValue` 读取；第一版不要为此新增 `yaml-cpp` 依赖。

SO3 控制器仍然每机逻辑独立，但建议将 7 个 `SO3ControlNodelet` 加载到同一个 `nodelet manager`，减少进程数量。nodelet 共享进程不代表共享控制状态；每个实例使用自己的命名空间、参数和话题。

---

## 6. ROS 消息修改

### 6.1 新增 SwarmState.msg

文件：

- src/swarm_planner/common_msgs/msg/SwarmState.msg

内容：

~~~text
std_msgs/Header header
uint16 robot_id
geometry_msgs/Point position_world
geometry_msgs/Vector3 velocity_world
~~~

高频 20 到 30 Hz 发布。第一版不要把 g_swarm、u_w、u_delta 或完整路径放入该消息。

### 6.2 新增 SwarmPathEvent.msg

文件：

- src/swarm_planner/common_msgs/msg/SwarmPathEvent.msg

建议内容：

~~~text
std_msgs/Header header
uint16 robot_id
int32 branch_event_id
uint32 event_sequence
time valid_until
bool active
~~~

它只用于 8 字、自交或重复访问任务。状态变化时立即发送，事件激活期间以 5 到 10 Hz 心跳重发；点到点和普通圆任务不发布。接收端按 `robot_id + event_sequence` 去重，并丢弃超过 `valid_until` 的旧事件。

### 6.3 新增 SwarmConflictState.msg

文件：

- src/swarm_planner/common_msgs/msg/SwarmConflictState.msg

建议内容：

~~~text
std_msgs/Header header
uint16 robot_id
int32 conflict_region_id
geometry_msgs/Point region_center
geometry_msgs/Vector3 region_axis
geometry_msgs/Vector3 region_extent
float64 channel_beta
float64 eta_to_region
uint8 conflict_state
uint32 event_sequence
time valid_until
bool active
~~~

它用于狭窄通道或交叉区域。状态变化时立即发送，激活期间以 5 到 10 Hz 心跳重发。`channel_beta` 只参与成对弱凝聚系数

$$
\beta_{ij}=\min(\beta_i,\hat\beta_j),
$$

不削弱分离、径向阻尼、软安全和硬 CBF 约束。事件过期后，开阔区默认恢复 `channel_beta=1`。

不要把路径语义和通道状态塞进 20 到 30 Hz 的 `SwarmState`；高频包固定只传 ID、时间戳、世界坐标位置和速度。

### 6.4 新增 PhaseOffsetDebug.msg

文件：

- src/swarm_planner/common_msgs/msg/PhaseOffsetDebug.msg

至少包含：

~~~text
std_msgs/Header header
uint16 robot_id
uint32 path_epoch
uint32 tube_epoch
float64 phase
float64 delta
float64 delta_lower
float64 delta_upper
float64 curvature
float64 regularity
float64 channel_beta
float64 u_w
float64 u_delta
geometry_msgs/Point base_reference
geometry_msgs/Point active_reference
geometry_msgs/Vector3 g_pos
geometry_msgs/Vector3 g_damp
geometry_msgs/Vector3 g_safe
geometry_msgs/Vector3 g_des
geometry_msgs/Vector3 v_base
geometry_msgs/Vector3 v_match
geometry_msgs/Vector3 v_final
geometry_msgs/Vector3 shadow_v_match
geometry_msgs/Vector3 shadow_v_final
uint16[] organization_neighbors
uint16[] safety_neighbors
uint16[] pair_robot_ids
float64[] pair_distances
float64[] pair_distance_rates
geometry_msgs/Vector3[] pair_g_pos
geometry_msgs/Vector3[] pair_g_damp
float64 e_parallel
float64 e_perp_norm
float64 min_neighbor_distance
float64 min_obstacle_distance
float64 min_pair_cbf_margin
float64 min_tube_cbf_margin
float64 allocation_residual
float64 matched_residual
uint16 active_cbf_count
uint8 mode
bool shadow_mode
bool rolling_feasible
bool nonnegative_feasible
string[] active_constraints
~~~

修改：

- common_msgs/CMakeLists.txt
- common_msgs/include/common_msgs/common_msgs.h
- 如需要，common_msgs/package.xml

---

## 7. 现有文件的具体修改

### 7.1 formation_planning.cpp

当前 AsyncSpinner 固定为 8。

修改：

1. 从私有参数 spinner_threads 读取线程数。
2. 默认保持 8，swarm launch 中设置为 2。
3. 节点仍保持一机一个进程。

### 7.2 gvf.h 与 gvf.cpp

修改：

1. 保留现有 LiftedGuidanceResult，避免破坏 legacy。
2. 新增 calcPhaseOffsetGuidanceAtState(pos,w,delta)。
3. 新方法内部调用 ContinuousPhasePath::evaluate() 获取 p、p_w、p_ww。
4. 使用 PhaseOffsetGeometryEvaluator 计算活动参考几何。
5. 返回 r、r_w、N、curvature、base_v 和 base_w_dot。
6. 原 calcLiftedGuidanceAtPhase() 不删除，作为 enable_phase_offset=false 的回归路径。
7. 可视化向量场增加参数，可选择基础路径场或活动参考场。

建议新增结果类型：

~~~cpp
struct PhaseOffsetGuidanceResult {
  PhaseOffsetGeometry geometry;
  Eigen::Vector3d final_v_cmd;
  double final_w_dot;
  bool valid;
};
~~~

### 7.3 gvf_manager.h

新增：

1. robot_id、robot_namespace 和可参数化话题。
2. delta 与 previous port 状态。
3. path_epoch、tube_epoch。
4. NeighborStateBuffer、SwarmEventBuffer 和 NeighborSelector。
5. ElasticSwarmIntent。
6. PathTubeBuilder。
7. PhaseOffsetAllocator 与 PhaseOffsetCbfConstraints。
8. PhaseOffsetVisualizer。
9. swarm state、path event、conflict state 的 publisher 与 subscriber。
10. tube timer、state broadcast timer 和两个事件心跳 timer。
11. 各类功能开关、mutex 和不可变 snapshot 类型。

不要继续把大量成员直接平铺到 gvf_manager。新模块的参数应放在各自 Params 结构中。

### 7.4 gvf_manager.cpp 构造函数与 initCallback()

去除以下硬编码：

- /move_base_simple/goal
- /path_vis
- /goal_vis
- /particle0/path
- /particle0/kinopath
- /particle0/circle_reference
- /gvf_force

全部改为参数读取，默认值仍保持当前单机话题。

新增参数：

~~~text
gvf/robot_id
gvf/robot_namespace
gvf/particle_base
gvf/goal_topic
gvf/path_topic
gvf/kino_path_topic
gvf/path_vis_topic
gvf/goal_vis_topic
gvf/circle_ref_topic
gvf/goal_z_mode
phase_offset_swarm/state_topic
phase_offset_swarm/path_event_topic
phase_offset_swarm/conflict_state_topic
phase_offset_swarm/debug_topic
~~~

InitGvf() 中 particle_base 不再固定为 /particle0。

所有话题拼接统一使用 ROS 名称 API，禁止直接混用 `particle + "sdf_map/..."` 和 `particle + "/path"`：

~~~cpp
static std::string appendTopic(const std::string& base,
                               const std::string& suffix) {
  return ros::names::append(ros::names::clean(base),
                            ros::names::clean(suffix));
}
~~~

例如 `particle_base=/uav_0/particle0` 时，必须得到：

~~~text
/uav_0/particle0/path
/uav_0/particle0/kinopath
/uav_0/particle0/sdf_map/occupancy
/uav_0/particle0/gvf/vector_field
~~~

当前 `sdf_map.cpp` 中存在 `particle + "sdf_map/occupancy"` 的缺斜杠写法，DeepSeek 必须在命名空间阶段统一修复并做话题断言测试，否则会生成 `/uav_0/particle0sdf_map/...`。

当前 `goalCallback()` 会对消息 z 再加 `1.0`。新增 `gvf/goal_z_mode`：

~~~text
legacy_offset   保持现有 msg.z + 1.0，作为旧单机默认
absolute_world  直接使用 msg.z，作为所有新 swarm launch 默认
~~~

禁止让 DeepSeek在场景脚本和 manager 两边同时补高度。

### 7.5 gvf_manager.cpp 的 cmdCallback()

这是最关键的集成点。建议按以下顺序重构，并保留 legacy 分支：

~~~cpp
if (!phase_offset_swarm_enabled_) {
  runLegacyCommandStep();  // 从当前 cmdCallback 原逻辑抽出的等价 helper
  return;
}

dt = boundedTimerDt(event);
const double w_k = phase_w_;
const double delta_k = phase_offset_delta_;

// 1. 用当前 w、delta 计算活动参考几何和基础 ISF 项
geometry = calcPhaseOffsetGuidanceAtState(pos, w_k, delta_k);

// 2. 查询当前管道与前视保守管道
tube = tube_builder_.queryConservativeLookahead(...);

// 3. 读取邻居快照并计算本机局部速度意图
raw_neighbors = neighbor_buffer_.snapshot();
events = event_buffer_.snapshot();
selection = neighbor_selector_.select(
    now, pos, self_velocity, raw_neighbors, events, pm.sdf_map_.get(), ...);
intent = swarm_intent_.compute(
    pos, self_velocity, tube.beta, selection, events, pm.sdf_map_.get(), ...);
intent.g_des = intent.g_swarm - k_rec * delta_k * geometry.N;

// 4. 构造并求解 rolling / safety-priority 端口
port = allocator_.solve(
    geometry, tube, intent, selection.safety, dt, previous_port_);
if (selection.communication_fault) {
  port = enforceCommunicationFaultPolicy(port, ...);
}

// 5. 计算 matched 物理前馈
v_match = geometry.r_w * port.u_w + geometry.N * port.u_delta;
v_final = geometry.base_v + v_match;
w_dot_final = geometry.base_w_dot + port.u_w;
delta_dot_final = port.u_delta;

// 6. 只计算候选状态，暂不提交
const double w_next = w_k + w_dot_final * dt;
const double delta_next = delta_k + delta_dot_final * dt;

// 7. 把 final_v_cmd 交给修改后的 governor
out.v_cmd = v_final;
out.ref_pt = geometry.r;
out.tangent = geometry.T;
out.e_perp = geometry.e_perp;
result = runVelocityMatchingGovernor(..., w_next, delta_next, ...);

// 8. guidance、QP、CBF 与 governor 全部有效后，一次性提交内部状态
if (geometry.valid && port.feasible && result.command_valid) {
  phase_w_ = applyPointEndpointRule(w_next);
  phase_offset_delta_ = delta_next;
  previous_port_ = port;
} else {
  // 不允许只更新 w 或只更新 delta；根据失败原因进入 safety-priority/emergency
  handleInvalidPhaseOffsetStep(...);
}

// 9. 发布 PositionCommand、debug、marker 和本机状态
~~~

必须使用 timer event 计算 dt，并限制在合理区间，例如 0.005 到 0.05 秒。`delta_next` 不得依靠事后 clip 才满足 tube；正常运行时必须由 tube CBF 保证可行。数值误差只允许在 $10^{-9}$ 量级做容差处理。

### 7.6 command governor 必须使用活动参考

当前 runVelocityMatchingGovernor() 通过 pathPointAtW() 查询 p(w)。必须增加活动参考查询：

~~~cpp
bool activeReferencePointAtW(const std::shared_ptr<gvf>& g,
                             double w,
                             double delta,
                             Eigen::Vector3d& r);
~~~

实现：

1. 评估 p、p_w。
2. 计算 N。
3. 返回 r=p+N*delta。

governor 的候选点应从

$$
p(w+l)+n
$$

改为

$$
r(w+l,\delta)+n.
$$

QP 和几何在 $(w_k,\delta_k)$ 上计算；governor 的候选活动参考使用本周期将要提交的 $(w_{k+1},\delta_{k+1})$。只有 governor 有效时才一次性提交该候选状态。这样既不混用已经提交和未提交状态，也不引入一拍 phase/offset 滞后。

如果这一项不改，governor 会把活动参考 offset 当成误差并拉回基础路径，导致 matched port 在真实 SO3 仿真中失效。

### 7.7 PositionCommand 速度前馈

当前 publishGovernorPositionCommand() 将 velocity 全部设为零。第一版先保留位置 governor，确保回归稳定；随后增加参数

~~~text
gvf/cmd/use_velocity_feedforward
~~~

为 true 时将 `v_final` 写入 `PositionCommand.velocity`。该开关在 legacy 单机 launch 中默认 false，以保证回归；在正式 phase--offset 多机 CBF 场景中默认 true。

CBF 约束和 matched cancellation 严格作用在几何命令层 $v_{\mathrm{final}}$。如果 position-only governor 把 velocity 置零，则实际 SO3 闭环只能在经过实验标定的跟踪/执行误差界下主张鲁棒安全，不能声称真实闭环数值精确抵消。正式实验必须：

1. 打开 velocity feedforward，或给出 position-only 模式的保守误差标定；
2. 将实际 $v_{\mathrm{executed}}-v_{\mathrm{final}}$ 上界计入 $\bar d_i^{\mathrm{exec}}$；
3. 区分几何层 matched residual（可到 $10^{-9}$）和 SO3 跟踪 residual（只要求有界）。

### 7.8 C2 更新与 delta continuation

在以下路径安装位置增加 delta 保持和日志：

- installInitialClosedPhaseFrontend()
- installInitialPointPhaseFrontend()
- buildPhaseV2C2Frontend() 成功后的安装逻辑
- 接受新 point/closed frontend 的代码路径

初次初始化：

$$
\delta_i=0.
$$

正常更新：

$$
w_i^+=w_i^-,
\qquad
\delta_i^+=\delta_i^-.
$$

安装前保存旧值：

~~~cpp
old_r
old_r_w
old_error
old_delta
~~~

安装后计算并发布：

~~~cpp
delta_r_norm
delta_rw_norm
delta_error_norm
delta_jump
~~~

正常 C2 更新验收阈值建议：

$$
\|\Delta r\|<10^{-3}\ \mathrm{m},
$$

$$
\|\Delta r_w\|<10^{-2},
$$

$$
|\Delta\delta|<10^{-9}.
$$

### 7.9 C2 connector 的管道检查

当前 buildPhaseV2C2Frontend() 只检查 state.p 的中心线碰撞。对每个 connector 采样点增加：

1. 计算 T、N、kappa。
2. 检查 1-kappa*delta_current>=mu。
3. 检查活动参考 r=p+N*delta_current 的障碍距离。
4. 检查该点左右至少存在最小 tube width。
5. 检查前视管道内存在连续可行的 delta 演化。
6. 将最小 tube margin 加入 connector cost。

若某 connector 不满足，继续尝试其他 join_delta_w。禁止在安装后再把 delta 突然裁剪。

### 7.10 SDFMap 和邻机占用

仿真第一版中，全局 PCD 只包含静态环境，不把其他 UAV 的广播位置写入 SDFMap。邻机始终由 NeighborStateBuffer 与机间 CBF 处理。

这样可避免本地 A* 把前方 UAV 永久当作静态墙体。后续若传感器点云中真实看到机体，再增加基于广播位置的点云 mask。

### 7.11 CMakeLists.txt 与 package.xml

修改 `src/swarm_planner/bspline_traj/CMakeLists.txt`，把以下源文件加入现有 `bspline_gvf` 库：

~~~cmake
src/phase_offset_geometry.cpp
src/path_tube_builder.cpp
src/swarm_neighbor_model.cpp
src/phase_offset_allocator.cpp
src/phase_offset_cbf_constraints.cpp
src/phase_offset_visualizer.cpp
~~~

为所有新 GTest 显式链接：

~~~cmake
bspline_gvf
${catkin_LIBRARIES}
${NLOPT_LIBRARIES}
~~~

检查 `bspline_race` 的 `find_package(catkin REQUIRED COMPONENTS ...)`、`catkin_package()` 和 `package.xml`，显式补齐实际直接使用的 `geometry_msgs`、`nav_msgs`、`common_msgs`、`visualization_msgs` 和 `quadrotor_msgs`，不要依赖其他包的传递依赖。

修改 `common_msgs/CMakeLists.txt`，把四个新消息加入 `add_message_files()`；修改 `common_msgs/include/common_msgs/common_msgs.h` 添加对应生成头文件。

在 `bspline_race/CMakeLists.txt` 显式增加生成消息依赖，避免并行编译时先编译引用头文件的目标：

~~~cmake
add_dependencies(bspline_gvf
  ${catkin_EXPORTED_TARGETS}
  ${common_msgs_EXPORTED_TARGETS})

add_dependencies(formation_planning
  ${catkin_EXPORTED_TARGETS}
  ${common_msgs_EXPORTED_TARGETS})
~~~

修改 `src/uav_simulator/so3_quadrotor_simulator/CMakeLists.txt`，新增 `multi_quadrotor_simulator_so3` executable，并与当前单机 executable 复用同一个 Quadrotor dynamics 库/公共控制转换代码。不要把多机 simulator 链接进 `bspline_gvf`，二者属于不同层。

`so3_quadrotor_simulator` 的 CMake/package.xml 显式声明新 executable 直接使用的 `roscpp、nav_msgs、sensor_msgs、geometry_msgs、quadrotor_msgs`，不能依赖传递依赖。

### 7.12 回调并发、快照和原子安装

仅把 `AsyncSpinner(8)` 改成 2 不能解决线程安全。必须明确以下所有权：

1. `NeighborStateBuffer` 内部持有自己的 mutex，callback 只写缓存；command 周期通过 `snapshot()` 复制不可变快照后立即释放锁；
2. `SwarmEventBuffer` 同样使用独立 mutex；不得在持有 neighbor mutex 时再获取 event mutex；
3. odom 位置、测量速度和时间戳通过 `OdomSnapshot` 在短锁下整体复制，禁止分别读取造成时刻不一致；
4. `phase_w_、phase_offset_delta_、previous_port_、swarm_mode_、governor state` 由一个 `control_state_mutex_` 保护；command callback 在开头复制状态，在末尾短锁一次性提交；
5. 活动 `ContinuousPhasePath` 和 `TubeProfile` 使用不可变

   ~~~cpp
   std::shared_ptr<const ContinuousPhasePath>
   std::shared_ptr<const TubeProfile>
   ~~~

   C2/tube 在锁外构造完成后，通过短锁或 C++11 `std::atomic_load/std::atomic_store(shared_ptr)` 一次性交换；
6. 候选 path/tube 记录构造时的 `path_epoch/tube_epoch`。提交前版本不匹配则丢弃并重建，避免旧 tube 安装到新路径；
7. 当前 `SDFMap` 未证明支持并发读写。第一版必须让同一把 map mutex 同时覆盖 `SDFMap` 内部更新 callback 与外部 LOS/tube 查询，或把二者放入同一个专用 callback queue；仅在 manager 外面加一把查询锁是不够的。该锁绝不能与 control-state 锁同时持有；
8. 禁止在任何上述锁内运行 A*、B 样条优化、C2 搜索、ESDF 整段扫描、Marker 构造或 QP 枚举。

推荐的 command 周期锁顺序不是嵌套加锁，而是依次“复制并释放”：

~~~text
copy odom snapshot -> copy control snapshot -> copy neighbor/event snapshot
-> copy active path/tube shared_ptr -> 无锁计算
-> 短锁校验版本并一次性提交 w/delta/port/governor
~~~

必须增加并发压力测试：在 50 Hz command、10 Hz tube 更新和异步 C2 安装同时运行时，检查无崩溃、无容器迭代失效、无 phase/tube epoch 倒退。

---

## 8. Launch 文件改造

### 8.1 单机 simulator 保留与多机 simulator 新增

保留并只做必要清理：

- src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch

它继续作为当前单机回归入口，默认命令必须保持可运行：

~~~bash
roslaunch so3_quadrotor_simulator simulator.launch
~~~

多机不要重复 include 该文件。新增：

- src/uav_simulator/so3_quadrotor_simulator/launch/multi_simulator.launch

主要 args：

~~~text
num_agents
initial_state_file
topic_prefix
frame_id
simulation_rate
odom_rate
command_timeout
start_at_hover
~~~

`initial_state_file` 建议使用 YAML 定义 `robot_id、name、x、y、z、yaw`。`multi_simulator.launch` 只启动一个 `multi_quadrotor_simulator_so3`，不启动地图、RViz、local_sensing 或规划器。

统一 frame 为 `world`，每条 odom 的 child frame 为 `uav_i/base_link`。七机仿真建议动力学更新 300 到 500 Hz、odom 100 Hz。

当前单机 `simulator.launch` 中地图和两个 RViz 应通过参数开关保留，避免单机回归变化；多机总 launch 不使用这些内置显示，而是在外部统一启动一次地图和 RViz。

必须为原 `simulator.launch` 增加且默认保持 legacy 行为的开关：

~~~text
start_plant:=true
start_controller:=true
start_map:=true
start_rviz:=true
start_odom_visualization:=true
~~~

这样它既能继续一条命令完成旧单机测试，也能被 `phase_offset_single_sim.launch` 以 plant-only 方式复用。

### 8.2 test_gvf.launch

修改文件：

- src/swarm_planner/bspline_traj/launch/test_gvf.launch

新增 args：

~~~text
uav_ns
robot_id
odom_topic
local_map_topic
global_map_topic
cmd_topic
goal_topic
particle_base
state_topic
path_event_topic
conflict_state_topic
debug_topic
enable_phase_offset
enable_swarm
enable_tube
enable_cbf_safety
manual_map_auto_save
manual_map_file
spinner_threads
start_formation_planning
start_local_sensing
~~~

要求：

1. 使用 arg 填充 gvf/cloud_topic、gvf/odom_topic、gvf/cmd_topic。
2. 将当前未使用的 drone_id 真正映射到 gvf/robot_id。
3. particle_base 按 UAV 区分。
4. 多机时 manual_map_auto_save=false，避免七个节点并发写同一文件。
5. local_sensing 显式设置自己的 odom_topic 和 local_map_topic。
6. 保留所有现有 phase_v2 与 C2 参数。
7. `global_map_topic` 可以共同指向 `/mock_map`，但只有 `local_sensing` 订阅；formation_planning 只能订阅本机 `local_map_topic`。
8. 多机默认 `sdf_map/enable_manual_map=false`、`manual_map_auto_save=false`；若必须使用手工障碍，每机使用不同文件名，禁止七个进程写同一个 `manual_obstacles_test_gvf.txt`。
9. 当前 `test_gvf.launch` 同时拥有 `formation_planning` 和 `local_sensing` 两个 node；必须用 `start_formation_planning/start_local_sensing` 的 `if` 开关明确所有权。`phase_offset_agent.launch` 不得在 include 之外再单独启动第二个 local_sensing。

### 8.3 新增单 UAV 实例包装 launch

新增：

- src/swarm_planner/bspline_traj/launch/phase_offset_agent.launch
- src/swarm_planner/bspline_traj/launch/phase_offset_single_sim.launch

`phase_offset_agent.launch` 不启动任何动力学 simulator。它只负责：

1. 在本机命名空间加载一个逻辑独立的 `SO3ControlNodelet`；
2. include 一次 `test_gvf.launch`，并传入 `start_formation_planning:=true、start_local_sensing:=true`；
3. 不在 agent launch 中再次手写 local_sensing 或 formation_planning node；
4. 接收 `robot_id、namespace、nodelet_manager、odom/cmd/map topics`；
5. controller loader 的名称和 remap 必须包含 robot_id，且全部加载到总 launch 提供的同一个 manager。

`phase_offset_single_sim.launch` 的固定所有权为：

1. include 原 `simulator.launch`，但传入 `start_controller:=false、start_map:=false、start_rviz:=false、start_odom_visualization:=false`，只保留单机 plant；
2. 启动一个 nodelet manager；
3. include 一个 `phase_offset_agent.launch`，由 agent 加载唯一 controller 和启动唯一 local_sensing/planner；
4. 在顶层各启动一次 map、visualizer 和 RViz。

### 8.4 新增统一多机总 launch

新增：

- launch/phase_offset_swarm_sim.launch
- config/phase_offset_swarm/rviz/phase_offset_swarm.rviz

可选增加两个很薄的快捷包装：

- launch/phase_offset_swarm_3.launch
- launch/phase_offset_swarm_7.launch

真正的主入口是一个命令：

~~~bash
roslaunch bspline_race phase_offset_swarm_sim.launch num_agents:=7 scenario:=open_hex
~~~

总 launch 的职责与 SPH 工程的总体启动方式相同：一次启动地图、全部机器人和一个 RViz。它内部必须完成：

1. 只启动一个 `map_pub`；
2. 只启动一个 `multi_quadrotor_simulator_so3`，内部创建 `num_agents` 个独立 plant；
3. 只启动一个空的 nodelet manager；总 launch 本身不加载任何 SO3 controller；
4. 根据 `num_agents` include 1 到 7 个 `phase_offset_agent.launch`；
5. 每个 agent 位于 `/uav_i` 命名空间，并向同一 manager 加载一个独立 controller 实例，同时只启动一次 local_sensing 和 formation_planning；agent 不具有独立 simulator 进程；
6. 启动一个只读 `phase_offset_swarm_visualizer` 和一个加载 `phase_offset_swarm.rviz` 的 RViz；
7. 启动场景目标发布器和可选 logger/rosbag；
8. 禁止 agent 内部再次启动 map、RViz 或固定 `world -> base` TF。

`phase_offset_swarm.rviz` 的 Fixed Frame 设置为 `world`，预配置以下 display：

~~~text
/mock_map                                  PointCloud2
/phase_offset_swarm/vis/uavs               MarkerArray
/phase_offset_swarm/vis/trajectories       MarkerArray
/phase_offset_swarm/vis/paths              MarkerArray
/phase_offset_swarm/vis/tubes              MarkerArray
/phase_offset_swarm/vis/interactions       MarkerArray
~~~

ROS1 XML 没有普通循环，第一版可以明确写出 7 个 group，并使用

~~~xml
<group if="$(eval int(arg('num_agents')) > 0)"> ... uav_0 ... </group>
<group if="$(eval int(arg('num_agents')) > 1)"> ... uav_1 ... </group>
...
~~~

统一 launch 只是进程编排，不参与控制计算。关闭 RViz 和 visualizer 后，所有分布式节点仍应继续飞行。

七机初始位置建议按详细 proposal 的六边形：

| UAV | x | y | z |
|---|---:|---:|---:|
| A / 0 | -0.5 | 0.866 | 1.0 |
| B / 1 | 0.5 | 0.866 | 1.0 |
| C / 2 | -1.0 | 0.0 | 1.0 |
| D / 3 | 0.0 | 0.0 | 1.0 |
| E / 4 | 1.0 | 0.0 | 1.0 |
| F / 5 | -0.5 | -0.866 | 1.0 |
| G / 6 | 0.5 | -0.866 | 1.0 |

每个 group 只加载自己的 controller 实例并启动 local_sensing 和 formation_planning。所有 UAV 的动力学由唯一的 multi simulator 统一积分，但状态、命令和 plant 对象按 robot_id 完全隔离。全局 map_pub、multi simulator、nodelet manager、汇总 visualizer 与 RViz 各只启动一次。因此用户只需要运行一个 launch，就能在同一个 `world` 视图中看到七架 UAV、各自轨迹、基础路径 $p_i$、活动路径 $r_i$、tube、邻居边、意图箭头和 CBF 激活状态。

### 8.5 场景发布脚本

新增：

- scripts/swarm_scenario_publisher.py
- config/phase_offset_swarm/scenarios/open_hex.yaml
- config/phase_offset_swarm/scenarios/obstacle_split_merge.yaml
- config/phase_offset_swarm/scenarios/corridor_two_wide.yaml
- config/phase_offset_swarm/scenarios/corridor_single_wide.yaml

脚本读取每架 UAV 的目标并发布到 /uav_i/goal。初期不要在 manager 内部自动人为维持目标槽位；目标文件仅用于可重复实验。

所有新 swarm 场景使用 `gvf/goal_z_mode=absolute_world`，YAML 中的 `z=1.0` 就表示世界高度 1 m。场景发布器只在任务开始或明确任务事件时发送一次目标，并等待确认；禁止周期重发同一目标，否则会反复触发 phase/path 初始化和 C2 更新。

闭合圆和 8 字场景继续使用现有 circle_test auto_start。

### 8.6 地图资源

现有地图可以用于早期检查，但坐标范围不同：

| 文件 | 当前点云范围 |
|---|---|
| pillar.pcd | x 约 [-7.24,7.26]，y 约 [-13.74,13.66]，z 约 [-0.94,2.96] |
| corridor.pcd | x 约 [-0.96,27.94]，y 约 [-10.22,10.18]，z 约 [-0.50,2.40] |
| compare_corridor.pcd | x 约 [0,20]，y 约 [-5.15,5.15]，z 约 [-0.50,3.40] |
| narrow.pcd | x 约 [-15.01,15.62]，y 约 [-0.16,0.18]，z 约 [-0.50,5.50] |

`corridor.pcd` 的 x 上界超过当前 `test_gvf.launch` 的 20 m 地图范围，不能在不调整 map size/origin 的情况下直接使用。

建议新增可复现的专用地图：

- resource/phase_offset_open.pcd
- resource/phase_offset_split_merge.pcd
- resource/phase_offset_corridor_two_wide.pcd
- resource/phase_offset_corridor_single_wide.pcd
- scripts/generate_phase_offset_maps.py

生成脚本必须把通道净宽度、墙体厚度、入口/出口位置和地图范围写成参数，并同时输出元数据 YAML。双机宽通道与单机宽通道不能只靠肉眼命名，应按机体中心安全距离给出横截面必要条件，并在 launch 中读取对应宽度。

---

## 9. 参数文件

新增：

- config/phase_offset_swarm/phase_offset_swarm.yaml

建议第一版参数：

~~~yaml
gvf:
  goal_z_mode: absolute_world
  cmd:
    use_velocity_feedforward: true

phase_offset_swarm:
  enabled: true
  robot_id: 0
  state_topic: /phase_offset_swarm/state
  path_event_topic: /phase_offset_swarm/path_event
  conflict_state_topic: /phase_offset_swarm/conflict_state
  state_publish_rate: 25.0
  path_event_heartbeat_rate: 5.0
  conflict_heartbeat_rate: 10.0
  neighbor_timeout: 0.30
  neighbor_retention_timeout: 1.00

phase_offset:
  enabled: true
  delta_init: 0.0
  delta_fixed_limit: 0.80
  delta_rate_max: 0.60
  delta_rate_accel_max: 1.50
  mu_regular: 0.20
  k_recenter: 0.40

tube:
  enabled: false
  rebuild_rate: 10.0
  sample_step_w: 0.10
  lookahead_w: 2.00
  ray_step: 0.05
  max_offset: 1.20
  reference_clearance: 0.50
  boundary_slope_max: 0.80
  open_width: 1.60

swarm:
  enabled: false
  organization_enter_radius: 1.55
  organization_exit_radius: 1.65
  safety_radius: 1.40
  d_minus: 0.80
  d_plus: 1.20
  ttc_safe: 2.00
  k_repulsion: 1.20
  k_cohesion: 0.25
  k_damping: 0.80
  soft_safety_gain: 0.50
  soft_safety_max: 0.80
  use_los: true
  use_semantic_filter: false

allocator:
  enabled: true
  phase_speed_min: 0.10
  phase_speed_max: 2.00
  tangent_speed_min: 0.10
  tangent_speed_max: 2.00
  u_w_slew_rate: 1.50
  u_delta_slew_rate: 1.50
  cbf_gamma_pair: 2.00
  cbf_gamma_tube: 3.00
  qp_dt_max: 0.03
  weight_world_error: 1.00
  weight_u_w: 0.10
  weight_u_delta: 0.10
  weight_delta_u_w: 0.05
  weight_delta_u_delta: 0.05

safety:
  max_message_age: 0.30
  physical_safe_distance: 0.60
  self_position_error_bound: 0.08
  neighbor_position_error_bound: 0.08
  communication_position_margin: 0.05
  self_velocity_error_bound: 0.10
  neighbor_velocity_error_bound: 0.10
  execution_velocity_error_bound: 0.10
  self_acceleration_bound: 2.50
  neighbor_acceleration_bound: 2.50
  tube_uav_radius: 0.25
  tube_tracking_error_bound: 0.15
  tube_localization_margin: 0.05
  tube_map_margin: 0.05

debug:
  publish_markers: true
  publish_debug_msg: true
  log_rate: 2.0
~~~

`safety/physical_safe_distance` 是唯一权威的 $d_{\mathrm{safe}}$；不要在 `swarm` 和 `safety` 两处各保存一个可能不一致的副本。这些只是启动值，必须通过开阔区七机测试再调整。参数调节时保持

$$
d_{\mathrm{safe}}<d_-<d_+<R_c.
$$

上述半径专门匹配第 8.4 和阶段 4 中边长 $d=1.0$ m 的七机六边形。为了得到 proposal 中的局部邻居图，还必须满足

$$
d<R_c<\sqrt3d.
$$

因此不能把 `organization_enter_radius/organization_exit_radius` 随意设成 2 m 以上，否则七机示例会退化成近似全连接图，`A` 不再只有 `B,C,D` 三个组织邻居。其他尺度的场景应按比例同时调整初始间距、允许距离带和邻居半径。

---

## 10. SPH 工程只借鉴哪些内容

### 10.1 可以借鉴

| SPH 文件与函数 | 可借鉴内容 | 在新工程中的替代实现 |
|---|---|---|
| water_swarm/src/sph_zhang_3d.cpp::findNeighbors() | 半径邻居表与逐邻居遍历 | NeighborStateBuffer 和 organization/safety 两套集合 |
| parallelDensityAndPressures() | 紧支撑核在距离外自动为零 | 单独的 CompactSupportKernel，不计算 SPH 密度和压力 |
| bspline_race_3d.cpp::checkBetweenObstacles() | 沿两机连线采样并检查占据 | 每架 UAV 使用自己的 SDFMap 独立执行 LOS |
| pubroscmd() | 多机器人 marker、编号与 odom 可视化思路 | PhaseOffsetVisualizer |
| collision_matrix 的用途 | 障碍遮挡时释放组织作用 | 本机 LOS 标志，不发布集中式 N 乘 N 矩阵 |

### 10.2 明确不能复制

1. 不复制 SPHSystem 内保存全部 particles 的集中式结构。
2. 不复制 parallelUpdateParticlePositions() 的集中式积分。
3. 不复制 u_den、pressure 和完整 SPH 密度控制。
4. 不复制 absolute friction：u_fri=-k_fri*v_i。新方法只使用邻居径向相对阻尼。
5. 不复制把 SPH 加速度、轨迹 PD 和外部 force 直接相加的结构。
6. 不使用 Particle.msg 和 Force.msg 作为新方法的主接口。
7. 允许一个 multi-plant simulator 发布所有 UAV 的独立 odom，但该节点不得同时计算邻居、规划或集群控制；每条 odom 必须来自对应的独立 Quadrotor plant 状态。

### 10.3 SPH 工程的真实 ROS 数据链与本项目对应关系

需要准确理解 SPH 代码，而不是只看论文中的“distributed”表述：

1. `sph_zhang_3d.cpp` 在启动时创建

   ~~~text
   /particle0/odom
   /particle1/odom
   ...
   ~~~

   并在 `SPHSystem::pubroscmd()` 中遍历内部 `particles`，由同一个节点发布每个粒子的独立 odom；
2. 同一个函数还把全部粒子打包为 `/swarm_particles`；
3. `bspline_race_3d.cpp` 中的单个 `swarm_planning_3d` 进程订阅 `/swarm_particles`，并在一个进程中为所有粒子创建 manager、线程化规划和统一发布 `/swarm_traj`；
4. `OdomBroadcast/OdomWithNeighbors` 在头文件中被 include，并声明了 `odomBroadcastCallback`，但当前主运行链没有看到对应 subscriber/实现接通；
5. 因此该代码的独立 odom 话题是真实存在的，但邻居通信和多粒子规划主要由集中式 ROS 消息与单进程多实例模拟。

本项目借用“一个 plant 节点发布每机独立 odom”的形式，但把控制通信改成真正的每机发布：

~~~text
multi_quadrotor_simulator_so3
    ├── /uav_0/sim/odom ──> formation_planning_0
    ├── /uav_1/sim/odom ──> formation_planning_1
    └── ...

formation_planning_0 ──publish──> /phase_offset_swarm/state
formation_planning_1 ──publish──> /phase_offset_swarm/state
...

每个 formation_planning_i 订阅共享广播话题
    └── 本机按距离、freshness、LOS、semantic 独立筛选邻居
~~~

每架 UAV 必须满足：

- 只订阅自己的 `/uav_i/sim/odom` 作为本体状态；
- 根据自己的 odom 生成并广播 `SwarmState(robot_id,timestamp,position,velocity)`；
- 接收其他 UAV 的状态包，但只在本机形成邻居缓存；
- 不接收其他 UAV 已计算好的 `g_swarm、u_w、u_delta`；
- multi simulator 不发布可供控制直接使用的全体 `Swarm_particles` 数组；若为了 RViz 生成汇总数组，必须放在只读 visualizer 侧。

所以最终既保留 SPH 那种“每机有自己的 odom、一个 launch 全部可见”，又比其现有代码更明确地实现分布式通信和本机决策。

---

## 11. 分阶段实施顺序

任何阶段未通过验收，不进入下一阶段。

### 阶段 0：保存并验证当前单机基线

目标：确保后续能判断是否破坏原 ISF-GVF。

任务：

1. 记录当前分支和未提交修改，不允许 git reset。
2. 编译 common_msgs、plan_env、path_searching、bspline_race 和仿真包。
3. 运行当前 simulator.launch 与 test_gvf.launch。
4. 分别测试点到点、circle 和 figure8。
5. 保存 rosbag、关键日志和 RViz 截图。
6. 记录当前路径误差、最小障碍距离、命令频率和 C2 切换日志。

验收：

- 当前单机仿真能正常运行。
- 现有 GTest 全部通过。
- 后续所有功能关闭时能够复现该结果。

### 阶段 1：单进程 multi-SO3 plant 与多机命名空间

目标：先让三架、再让七架 UAV 在同一 ROS master 中独立飞行，不加入集群算法。

修改：

- 新增 multi_quadrotor_simulator_so3 与 multi_simulator.launch
- 保留原 simulator.launch 做 N=1 回归
- test_gvf.launch
- gvf_manager 的硬编码话题
- formation_planning spinner 线程
- 新增统一 `phase_offset_swarm_sim.launch`、单 agent wrapper 和 RViz 配置

步骤：

1. 令 multi simulator 的 `num_agents=1`，给定与原 simulator 完全相同的 SO3 命令，比较两者状态轨迹。
2. 设置 `num_agents=3`，只启动一个 simulator 进程和三个 controller 实例，检查命令/odom 隔离。
3. 三机各自启动 formation_planning，分别发送独立目标。
4. 将 `num_agents` 改为 7，重复同一总 launch。
5. map_pub、multi simulator、nodelet manager、汇总 visualizer 和 RViz 各只启动一次。
6. 在 RViz 中同时显示全部 UAV、ID 和各自轨迹。

验收：

- /uav_0 到 /uav_6 的 odom 均为 100 Hz 左右。
- `rosnode list` 中只有一个 multi simulator 进程。
- 每机 PositionCommand 经自己的 SO3 controller 后，只改变 multi simulator 中对应的 plant。
- `num_agents=1` 时 multi simulator 与原单机 simulator 的位置、速度和姿态误差在数值容差内。
- 节点名、TF child frame、可视化 marker ID 不冲突。
- 所有 SDF/GVF 话题都位于 `/uav_i/particle0/...`，不存在 `/particle0sdf_map` 一类错误拼接。
- swarm 场景使用 absolute-world z，发送 `z=1.0` 后目标高度确实为 1.0 m，且目标只触发一次路径初始化。
- 七机 enable_swarm=false 时能够独立规划和飞行。
- 一个 `roslaunch` 命令能同时启动并显示全部 UAV。
- 杀掉 RViz/visualizer 后，各 UAV 的 odom、规划和控制继续运行且命令不变。
- CPU 可接受；若过高，先降低统一 simulator rate，不删算法线程安全保护。

### 阶段 2：单机固定管道 phase--offset 几何

目标：不加邻居，只验证 delta 状态和活动参考。

修改：

- phase_offset_types
- phase_offset_geometry
- gvf 新接口
- gvf_manager 中 delta 状态
- active path 可视化

测试：

1. 直线路径，delta=0、+0.5、-0.5。
2. 圆路径，检查 1-kappa*delta。
3. 8 字路径，检查自交点 branch 仍由 w 区分。
4. 手动给定平滑 u_delta 正弦输入。

验收：

- 数值满足 r=p+N*delta。
- r_w 与 N 正交误差小于 1e-6。
- 有限差分 r_w 与解析 r_w 一致。
- delta 改变时 UAV 收敛到活动参考，而不是被拉回 p。
- enable_phase_offset=false 时完全回到当前单机行为。

### 阶段 3：手动端口与 matched cancellation

目标：先不用 flocking，直接注入测试 u_w、u_delta，验证 matched port。

新增参数：

~~~text
phase_offset/test_port_enable
phase_offset/test_u_w
phase_offset/test_u_delta
~~~

测试：

1. 只给 u_w。
2. 只给 u_delta。
3. 两者同时给。
4. 周期性改变端口。

在线计算残差：

$$
\epsilon_{\mathrm{match}}
=
\left\|
(v_{\mathrm{final}}-v_{\mathrm{base}})
-
(r_wu_w+Nu_\delta)
\right\|.
$$

验收：

- epsilon_match 小于 1e-9 的数值误差级别。
- phase 与 delta 使用同一最终端口更新。
- 跟踪误差动力学与无端口时一致。
- 横向端口不改变切向投影。

### 阶段 4：局部状态通信与七机弹性意图

目标：固定宽管道、无障碍开阔区中复现 proposal 的七机例子。

修改：

- SwarmState.msg
- NeighborStateBuffer
- ElasticSwarmIntent
- state broadcast timer
- neighbor marker

测试：

1. 初始七机六边形 d=1.0，验证邻居集合。
2. 单元测试中沿 AB、AC、AD 原方向人工生成三个距离均为 $d_{\mathrm{test}}<d_-$ 的邻居状态，或整体等比例压缩六边形，验证 A 的净意图向外；不要只移动 A 并假定三条边会同时等长变短。
3. 人工生成三条等距 $d_+<d_{\mathrm{test}}<R_c$ 的邻居状态，或整体等比例放大六边形，验证弱凝聚向内。
4. 给 A 一个向外相对速度，验证径向阻尼反向。
5. 让所有 UAV 以同一速度平移，验证阻尼近零。
6. 关闭某条 LOS，验证组织边释放而安全边保留。

必须自动断言初始邻居：

~~~text
A: B C D
B: A D E
C: A D F
D: A B C E F G
E: B D G
F: C D G
G: D E F
~~~

#### UAV A 的最终单周期完整链路，以及阶段 4/5 的启用边界

本节保留从邻居消息到最终命令的完整公式，便于检查最终架构；但实施必须分两步：

- 阶段 4 是 shadow mode：真实 `/uav_i/position_cmd` 仍由基础 ISF-GVF 产生，只计算并记录邻居贡献、`g_des、u_raw、shadow_v_match、shadow_v_final`；
- 阶段 5 在 allocator 和 CBF 测试通过后，才使用 `u_star` 替换 shadow 结果，并让邻居端口进入真实 governor/PositionCommand。

因此阶段 4 不能只打印 `g_swarm`，但也不能提前让未经过 CBF 的集群端口驱动七机。

令六边形边长 $d=1.0$ m，并在该测试中固定

$$
1.0<R_c^{\mathrm{enter}}=1.55
<R_c^{\mathrm{exit}}=1.65
<\sqrt3.
$$

因此 UAV A 的组织邻居快照只能包含 B、C、D。A 在高频通信中收到的是

$$
m_B=(t_B,x_B,v_B),
\quad
m_C=(t_C,x_C,v_C),
\quad
m_D=(t_D,x_D,v_D),
$$

不包含 B、C、D 已经算好的 `g_swarm` 或端口命令。

A 本机逐边计算：

$$
n_{AB}=(1,0),
$$

$$
n_{AC}=\left(-\frac12,-\frac{\sqrt3}{2}\right),
\qquad
n_{AD}=\left(\frac12,-\frac{\sqrt3}{2}\right).
$$

每条组织边的本地贡献为

$$
g_{Aj}^{\mathrm{org}}
=
a_{Aj}
\left[
\varphi_{Aj}(d_{Aj})
+k_d\dot d_{Aj}
\right]n_{Aj}.
$$

于是

$$
\boxed{
g_A^{\mathrm{swarm}}
=
g_{AB}^{\mathrm{org}}
+g_{AC}^{\mathrm{org}}
+g_{AD}^{\mathrm{org}}
+g_A^{\mathrm{safe}}.
}
$$

这里必须在 debug 中保存 AB、AC、AD 三条边的单独贡献，不能只保存最后总和。推荐数据流：

~~~cpp
for (const NeighborState& j : organization_neighbors) {
  PairIntentContribution c;
  c.robot_id = j.robot_id;
  c.distance = (j.position - self.position).norm();
  c.n_ij = (j.position - self.position) / c.distance;
  c.distance_rate = (j.velocity - self.velocity).dot(c.n_ij);
  c.g_pos = weight(c.distance) * phi(c.distance) * c.n_ij;
  c.g_damp = k_d * weight(c.distance) * c.distance_rate * c.n_ij;
  intent.pair_contributions.push_back(c);
  intent.g_pos += c.g_pos;
  intent.g_damp += c.g_damp;
}
intent.g_swarm = intent.g_pos + intent.g_damp + intent.g_safe;
intent.g_des = intent.g_swarm - k_rec * delta_A * geometry.N;
~~~

三个方向检查必须自动化：

1. 三条边都小于 $d_-$ 时，A 的净位置意图沿六边形外法向

   $$
   o_A=\left(-\frac12,\frac{\sqrt3}{2}\right)
   $$

   向外；
2. 三条边都大于 $d_+$ 但小于 $R_c$ 时，净弱凝聚意图沿 $-o_A$ 向内；
3. 七架 UAV 共同平移时，所有 $\dot d_{Aj}=0$，A 的径向阻尼为零；若 A 单独沿 $o_A$ 向外运动，阻尼必须沿 $-o_A$。

得到 `g_des_A` 后，A 使用自己的路径几何，而不是群体公共方向：

$$
J_A=[r_{w,A}\quad N_A],
$$

$$
u_{w,A}^{\mathrm{raw}}
=
\frac{r_{w,A}^\top g_A^{\mathrm{des}}}{\|r_{w,A}\|^2},
\qquad
u_{\delta,A}^{\mathrm{raw}}
=
N_A^\top g_A^{\mathrm{des}}.
$$

阶段 4 用 $u_A^{\mathrm{raw}}$ 计算 shadow matched 结果，但不发布到真实命令。阶段 5 的分配器再加入 phase 正向、物理切向裕度、tube CBF 和 A 对 `selection.safety` 的机间 CBF，输出最终

$$
u_A^\star=
\begin{bmatrix}
u_{w,A}^\star\\
u_{\delta,A}^\star
\end{bmatrix}.
$$

最后必须执行完整 matched 注入：

$$
v_A^{\mathrm{match}}
=
r_{w,A}u_{w,A}^\star
+N_Au_{\delta,A}^\star,
$$

$$
\boxed{
\begin{aligned}
\dot x_A
&=
v_{\mathrm{base},A}+v_A^{\mathrm{match}},\\
\dot w_A
&=
\dot w_{\mathrm{base},A}+u_{w,A}^\star,\\
\dot\delta_A
&=u_{\delta,A}^\star.
\end{aligned}}
$$

对应的 C++ 检查点在阶段 4/5 共用，但由显式开关区分 shadow 与 active：

~~~cpp
const Eigen::Vector3d v_match =
    geometry.r_w * port.u_w + geometry.N * port.u_delta;

guidance.v_cmd = geometry.base_v + v_match;
const double w_dot_total = geometry.base_w_dot + port.u_w;
const double delta_dot_total = port.u_delta;

debug.v_base = geometry.base_v;
debug.v_match = v_match;
debug.v_final = guidance.v_cmd;
debug.u_w = port.u_w;
debug.u_delta = port.u_delta;

if (swarm_control_active) {
  active_guidance = guidance;
} else {
  debug.shadow_v_match = v_match;
  debug.shadow_v_final = guidance.v_cmd;
  active_guidance = base_isf_guidance;  // 阶段 4 真实命令仍使用基础 ISF
}
~~~

阶段 4 只发布 `PhaseOffsetDebug` 中的 shadow 数值，不更新真实 $w_A,\delta_A$，也不替换 `/uav_0/position_cmd`。阶段 5 中只有 guidance、QP、CBF 和 governor 全部有效时，才一次性提交 $w_A^{k+1},\delta_A^{k+1}$，并把最终 guidance 与活动参考交给 governor。B 到 G 分别运行同一条本地链路；D 即使当前有六个邻居，也只计算 `/uav_3` 自己的命令。

验收：

- 每机只发布本机 SwarmState。
- 每机只输出自己的 g_swarm。
- g_C 中没有 E 的直接组织贡献。
- D 虽有六个邻居但不发布其他 UAV 的命令。
- 合成共同平移/相对伸缩样本中，径向阻尼方向和名义耗散计算正确。
- 阶段 4 对 A 能从逐边贡献追踪到 `g_des -> u_raw -> shadow_v_match -> shadow_v_final`，且真实 PositionCommand 与关闭 shadow 计算时一致。

### 阶段 5：二维端口分配器和鲁棒 CBF 安全保障

目标：将邻居意图约束到 phase--offset 两个端口，并保证正常模式不反向。

修改：

- phase_offset_allocator
- 2D QP active-set solver
- rolling 与 safety-priority 模式
- phase_offset_cbf_constraints
- tube CBF 与机间鲁棒 CBF

测试：

1. 无约束时恢复解析 u_raw。
2. u_delta 达到边界时，剩余切向意图仍可由 u_w 实现。
3. phase 下限阻止 u_w 导致反向。
4. 切向裕度阻止物理推进项被抵消。
5. 邻机接近鲁棒安全半径时，机间 CBF 约束激活。
6. rolling 不可行但 nonnegative 可行时进入 SAFETY_PRIORITY。
7. 两者均不可行时才进入 EMERGENCY。
8. 点到点 endpoint margin 内进入 TERMINAL，允许 phase 速度趋于零，但不得错误计为通道 hold。

验收：

- 求解器平均时间小于 2 ms，最大时间小于 10 ms。
- 所有输出满足线性约束容差 1e-6。
- matched residual 不因端口被约束而变化。
- 正常开阔区 phase 反向次数为零。
- 七机从满足 $h_{ij}^{\mathrm{rob}}\ge0$ 的初始状态启动后，才允许设置 `enable_swarm_control=true`。
- 对 A 重新执行完整 active 链 `g_des -> u_raw -> u_star -> v_match -> v_final -> governor -> /uav_0/position_cmd`，并证明实际命令不再是 shadow 值。
- 关闭 CBF 的版本只作为不安全消融，不能用于正式安全验收。
- 实际七机控制中，开启径向阻尼后的邻居距离振荡能量显著低于 no-damping 消融。

### 阶段 6：ESDF 路径管道与狭窄通道

目标：从本机局部地图在线构造 tube，完成开阔区展开、通道前收缩、通道内拉长和出口恢复。

步骤：

1. 单机在静态 PCD 上验证左右边界。
2. 三机通过双机宽通道。
3. 七机通过双机宽通道。
4. 三机通过单机宽通道。
5. 七机通过单机宽通道。
6. 加入 beta_i 与 beta_ij 的弱凝聚衰减。

地图：

- 优先使用 dynamic_map_generator/resource/corridor.pcd。
- 需要更精确宽度时新增 phase_offset_corridor.pcd。
- map_size 必须覆盖 PCD 坐标范围。

验收：

- 首个有效 odom、local map 和 ESDF 之前 `TubeBounds.valid=false`，不得生成假宽管道。
- unknown/out-of-map 不被当作自由空间。
- 当前 delta 始终位于管道内。
- 1-kappa*delta 始终大于 mu。
- 通道越窄，beta 越小。
- 只衰减凝聚，不衰减排斥、阻尼和 CBF 安全约束。
- 同向且正向可行场景中不出现正常 hold。
- 记录横向宽度、纵向长度、最小机间距离和通道吞吐量。

### 阶段 7：异步 C2 更新与双 continuation

目标：利用不同 UAV 的局部感知时刻差，触发异步重规划并保持 w、delta。

步骤：

1. 使用局部感知范围，使前排 UAV 先发现障碍、后排 UAV 后发现。
2. 每机独立接受新 B 样条和 C2 connector。
3. 安装前检查当前 delta 和 connector tube。
4. 保持 w、delta。
5. 记录 r、r_w、e 和 v_cmd 的切换跳变。

验收：

- path_epoch 可不同步增长。
- 任一 UAV 换路不会重置其他 UAV。
- 每次正常更新 delta_jump=0。
- C2 切换位置、活动切向和误差连续。
- 换路期间无错分支、无 phase 反向。

### 阶段 8：圆形和 8 字闭合任务

目标：验证 semantic phase、自交分支和重复访问。

圆：

1. 每机根据自身初始位置初始化 w。
2. delta 允许在管道内横向组织。
3. 不设置固定 phase 差。

8 字：

1. 使用 branch_event_id 标识当前进入的自交分支事件。
2. 组织凝聚只对语义兼容邻居开放。
3. 真实近距离安全不受 semantic filter 影响。
4. 可选增加局部 ETA 冲突调速。

验收：

- 自交中心错分支次数为零。
- lap_id 只保存在本机，不作为固定相位协调输入。
- 没有固定相位差控制。
- 路径更新和过圈时 delta 不被无故重置。

### 阶段 9：实验工具、消融和最终回归

新增：

- scripts/analyze_phase_offset_bag.py
- launch/phase_offset_record.launch
- config/phase_offset_swarm/rviz/phase_offset_swarm.rviz

消融：

1. direct-add flocking：只用于 baseline，故意不 matched。
2. phase-only：u_delta=0。
3. offset-only：u_w=0。
4. no damping。
5. no weak cohesion。
6. no tube lookahead。
7. no matched physical feedforward。
8. hard path switch 替代 C2。
9. pairwise/tube CBF off：只作为不安全消融，不用于正式安全结果。

最终必须重新运行阶段 0 单机回归。

---

## 12. 单元测试清单

### 12.1 phase_offset_geometry_test.cpp

测试：

1. 直线 delta=0。
2. 直线正负 delta。
3. 圆的曲率与解析值。
4. r_w 与 N 正交。
5. 1-kappa*delta 退化检测。
6. e_parallel 与 e_perp 分解。
7. finite-difference r_w。

### 12.2 swarm_neighbor_model_test.cpp

测试：

1. 七机邻居表完全匹配 proposal。
2. A 的三项求和。
3. D 对称位置意图抵消。
4. E 不直接进入 C。
5. d<d_- 时排斥。
6. d 在允许带时位置项为零。
7. d>d_+ 时弱凝聚。
8. beta=0 时只关闭弱凝聚。
9. 共同平移时阻尼为零。
10. 相对伸缩时阻尼方向正确。
11. LOS 关闭组织边。
12. safety neighbor 不受 LOS 和 semantic 关闭。

### 12.3 phase_offset_allocator_test.cpp

测试：

1. 无约束解析解。
2. 单条约束投影。
3. 两条约束交点。
4. 多约束最优候选。
5. rolling 可行。
6. 仅 nonnegative 可行。
7. 完全不可行。
8. phase 不反向。
9. 物理切向裕度。
10. 鲁棒机间 CBF 安全约束。
11. tube 上下边界 CBF 约束。
12. 点到点 TERMINAL 模式解除正向下界但保留安全约束。

### 12.4 matched_phase_offset_test.cpp

测试：

1. 只给 u_w 的精确抵消。
2. 只给 u_delta 的精确抵消。
3. 两个端口同时输入。
4. QP 改写端口后仍抵消。
5. 端口限幅后仍抵消。

### 12.5 phase_offset_continuation_test.cpp

测试：

1. C2 connector 前后 p、p_w、p_ww。
2. 保留相同 delta 后 r 连续。
3. r_w 连续。
4. e 与 V 连续。
5. 新 tube 不容纳 delta 时拒绝安装。

修改 bspline_traj/CMakeLists.txt，把新源文件加入 bspline_gvf 库，并注册上述 GTest。

---

## 13. ROS 集成测试与验收指标

| 类别 | 指标 | 第一版目标 |
|---|---|---:|
| 安全 | 最小机间距离 | 大于等于运行时计算的 `d_safe_rob`；至少不得低于物理 `d_safe` |
| 安全 | 碰撞次数 | 0 |
| CBF | min h_pair_rob | 正式安全场景中大于等于 -1e-6 |
| CBF | min h_tube | 大于等于 -1e-6 |
| CBF | 正常场景 QP 不可行次数 | 0 |
| 障碍 | 最小障碍余量 | 大于设定 reference clearance 减跟踪扰动容差 |
| ISF 跟踪 | e_perp 稳态 | 小于 0.20 m |
| matched | epsilon_match | 小于 1e-6，理想计算层小于 1e-9 |
| 正则性 | min(1-kappa*delta) | 大于等于 mu |
| phase | 正常模式反向次数 | 0 |
| continuation | delta jump | 0 |
| continuation | r jump | 小于 1e-3 m |
| 计算 | 2D allocator 平均耗时 | 小于 2 ms |
| 计算 | 2D allocator 最大耗时 | 小于 10 ms |
| 通信 | state 频率 | 20 到 30 Hz |
| 通信 | 消息超时后退出组织邻居 | 小于 0.35 s |
| 通信 | 最后安全状态保留 | 约 1.0 s，期间触发通信故障策略 |
| 通道 | rolling 主场景正常 hold 比例 | 接近 0 |
| 集群 | 邻居距离落在允许带比例 | 持续提高并稳定 |
| 阻尼 | 距离振荡能量 | 明显低于 no-damping |

---

## 14. 建议运行命令

### 14.1 编译

~~~bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
catkin_make --pkg common_msgs plan_env path_searching bspline_race map_generator so3_quadrotor_simulator so3_control
source devel/setup.bash
~~~

### 14.2 单元测试

~~~bash
catkin_make run_tests_bspline_race
catkin_test_results build/test_results
~~~

### 14.3 单机回归

~~~bash
roslaunch bspline_race phase_offset_single_sim.launch enable_phase_offset:=false
~~~

### 14.4 单机 phase--offset

~~~bash
roslaunch bspline_race phase_offset_single_sim.launch enable_phase_offset:=true enable_swarm:=false
~~~

### 14.5 三机

~~~bash
roslaunch bspline_race phase_offset_swarm_sim.launch num_agents:=3 scenario:=open_hex
~~~

### 14.6 七机

~~~bash
roslaunch bspline_race phase_offset_swarm_sim.launch num_agents:=7 scenario:=open_hex
~~~

### 14.7 通道

~~~bash
roslaunch bspline_race phase_offset_swarm_sim.launch num_agents:=7 scenario:=corridor_single_wide map_file:=$(rospack find map_generator)/resource/phase_offset_corridor_single_wide.pcd
~~~

实际 launch 参数名以实现后的 XML 为准，但多机正式入口统一保持为 `phase_offset_swarm_sim.launch`。快捷的三机/七机 wrapper 只能 include 该主入口，不能维护另一套节点结构。

---

## 15. DeepSeek 修改代码时的执行规则

1. 先阅读本文、详细 proposal、continuous_phase_path.h/.cpp、gvf.h/.cpp、gvf_manager.h/.cpp。
2. 不要一次性替换 gvf_manager.cpp。
3. 每个阶段只修改计划列出的文件。
4. 每完成一个阶段立即编译并运行对应单元测试。
5. 默认开关关闭时必须保持原 test_gvf 行为。
6. 新增模块优先写成无 ROS 纯 C++ 类，再由 manager 包装。
7. 不要把所有邻居状态存进一个中央控制节点；multi simulator 和只读 visualizer 可以保存 plant/显示状态，但不能计算集群控制。
8. 不要使用 common_msgs::Force 作为 g_swarm。
9. 不要把 g_swarm 直接加到 out.v_cmd。
10. 不要只更新 delta 而忘记 N*u_delta 的物理 matched 前馈。
11. 不要只更新 phase 而忘记 r_w*u_w 的物理 matched 前馈。
12. 不要在 governor 中继续查询基础 p 而忽略活动 r。
13. 不要在 C2 更新时 reset delta。
14. 不要为了通过通道而允许 phase 反向。
15. 不要把 safety-priority 的零切向进度称为总物理悬停。
16. 不要删除现有低层 emergency fallback；只需保证它不成为正常集群模式。
17. 不要复制 SPH 的集中式粒子更新、绝对摩擦和密度压力控制。
18. 不要同时修改理论符号和现有 K2 参数符号。
19. 分布式算法层不要用静态全局数组保存七架 UAV 的端口命令。multi simulator 只允许按 plant 保存最后收到的 SO3Command，用于动力学积分。
20. 所有调试信息使用节流日志或 Debug 消息，禁止在 50 Hz 循环中无节制打印。

### 15.1 推荐的修改批次

不要让 DeepSeek 一次完成全部内容。建议逐批给任务，每批验收后再继续：

1. `feat: add single-process multi-quadrotor SO3 simulator`
   - 抽取当前单机 simulator 公共代码；
   - 新增 N 个独立 Quadrotor plant 的单进程节点；
   - 完成 N=1 等价、命令隔离和七 odom 测试。
2. `refactor: namespace test_gvf and add unified swarm launch`
   - 参数化硬编码话题和 spinner；
   - 一个总 launch 启动 multi simulator、N 个 agent、一个 visualizer 和一个 RViz；
   - 三机/七机 `enable_phase_offset=false` 回归。
3. `feat: add phase-offset geometry`
   - 新增几何类型、活动参考查询与单元测试；
   - 不接邻居、不接 QP。
4. `feat: add manual matched port`
   - 人工 $u_w,u_\delta$；
   - 修改 governor 使用活动参考；
   - 验证误差抵消。
5. `feat: add distributed state and local neighbor aggregation`
   - 新消息、缓存、LOS、七机逐边贡献；
   - 先只发布 `g_swarm`，不改变控制。
6. `feat: add constrained phase-offset allocator`
   - 二维 QP、phase/tangent/tube 基本约束；
   - 与 matched port 接通。
7. `feat: add robust pairwise and tube CBF constraints`
   - 只新增解析线性约束和测试；
   - 不改变集群意图模型。
8. `feat: add ESDF path tube and channel events`
   - tube builder、beta、冲突事件与可视化。
9. `feat: preserve offset across C2 updates`
   - 候选 connector/tube 检查和双 continuation 日志。
10. `test: add corridor circle and figure-eight scenarios`
   - 场景、logger、rosbag 分析和消融。

每一批必须返回：修改文件列表、关键 diff 说明、编译输出、单元测试输出、实际启动命令、ROS 话题检查、截图或 rosbag 路径，以及尚未解决的问题。若当前批次验收失败，不允许通过同时修改下一层模块来掩盖问题。

---

## 16. 最终交付物

DeepSeek 完成后应交付：

1. 保持可回归的原单机 simulator.launch。
2. 单进程 `multi_quadrotor_simulator_so3` 和 `multi_simulator.launch`。
3. 参数化后的 test_gvf.launch。
4. 单机 launch、单 agent wrapper 和统一 `phase_offset_swarm_sim.launch`。
5. 四个新 common_msgs 消息：SwarmState、SwarmPathEvent、SwarmConflictState 和 PhaseOffsetDebug。
6. phase_offset_types。
7. phase_offset_geometry。
8. path_tube_builder。
9. swarm_neighbor_model。
10. phase_offset_allocator。
11. phase_offset_cbf_constraints。
12. 每机 phase_offset_visualizer 与全局只读 phase_offset_swarm_visualizer。
13. gvf 与 gvf_manager 的受控集成修改。
14. command governor 活动参考支持。
15. C2 connector 的 delta/tube 检查。
16. 单元测试与 ROS 仿真场景。
17. RViz 配置与 rosbag 分析脚本。
18. 一份实现记录，逐项说明哪些阶段通过、哪些尚未完成、对应测试输出和剩余风险。

第一轮建议只要求完成阶段 0 到阶段 4。确认七机开阔区局部聚合、phase--offset 分解和 matched port 全部正确后，再进入 CBF 安全层、ESDF 管道和狭窄通道。这样可以避免在基础几何与分布式接口尚未验证时，同时调试地图、QP、C2 和多机动力学。

阶段 4 中的 `g_swarm` 只允许计算、记录和可视化，不能驱动七架真实仿真 UAV。只有阶段 5 的机间/tube CBF 通过单元测试和初始安全集检查后，才允许打开 `enable_swarm_control=true` 让邻居意图正式进入多机飞行控制。
