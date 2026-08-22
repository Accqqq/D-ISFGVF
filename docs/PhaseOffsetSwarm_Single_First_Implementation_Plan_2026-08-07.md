# PhaseOffsetSwarm 单机内核优先实施计划

> 日期：2026-08-07  
> 最终论文方向仍是分布式多 UAV PhaseOffsetSwarm。本文只重新规定代码研发顺序，不另立单机论文方向。

```text
DOCUMENT_ROLE=MASTER_ROADMAP
COMPLETED_STAGE=A5.1
BLOCKED_STAGE=A5
CURRENT_AUTHORIZED_STAGE=A5.2
AUTO_ADVANCE=false
```

本总体计划用于说明完整路线，本身不授权执行全部阶段。代码执行者只能执行单独的“当前阶段执行单”；完成当前阶段、自审核并汇报后必须停止。没有新的阶段执行单，不得进入下一阶段。

代码 package、目录、类职责、依赖方向和 CMake target 的详细设计见：

`docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`。

## 0. 是否需要回退：需要，但采用可逆的局部隔离回退

### 0.1 当前判断

当前工作区的多机原型不能直接作为下一阶段的开发基线。原因不是它新增文件太多，而是它已经修改了 16 个原有文件，包括：

- `gvf_manager.h/.cpp`；
- `gvf.h/.cpp`；
- 原 `test_gvf.launch`；
- SDF map；
- 原单机 simulator 源码和 launch；
- 两个包的 CMake/package；
- `common_msgs` 的原有构建入口。

其中 `gvf_manager.cpp` 增加了约 1169 行。即使设置 `enable_swarm=false`，构造函数、订阅发布、timer、编译依赖和单机 simulator 结构也已经改变，因此不能把“开关关闭”当作原单机回归。

所以需要回退原有文件，但不能使用 `git reset --hard`，也不能删除 DeepSeek 新增内容。

### 0.2 回退目标

当前可信单机基线为

```text
branch: main
commit: 9a0e975 feat: integrate unified phase GVF navigation
```

在该基线上只重新应用用户已有的两项 launch 设置：

```xml
<param name="gvf/circle_test/enable" value="false" />
<param name="gvf/circle_test/auto_start" value="false" />
```

### 0.3 建议的可逆回退方法

执行前先停止所有 ROS 进程，并记录当前状态：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git status --short
git diff --stat
git diff --name-only
```

只把“已有跟踪文件的修改”保存到命名 stash，不包含未跟踪的新文件：

```bash
git stash push -m "deepseek-phaseoffset-tracked-prototype-2026-08-07"
```

这里不使用 `-u`。这样：

- 16 个已有文件恢复到 `main` 基线；
- DeepSeek 新增的 phase-offset、swarm、测试、地图和多机 simulator 文件仍保留为未跟踪文件；
- 当前实施计划文档也继续保留；
- 被 stash 的已有文件修改可以随时查看和按文件取回。

随后验证 stash：

```bash
git stash list
git stash show --stat stash@{0}
git status --short
```

禁止直接执行

```bash
git stash pop
```

把整套旧实现重新压回单机基线。后续如果某个独立模块值得复用，只按文件或按代码块人工审核后迁移。

### 0.4 重新应用用户修改

stash 后，使用小范围补丁重新把 `test_gvf.launch` 中以下参数设为 false：

```text
gvf/circle_test/enable=false
gvf/circle_test/auto_start=false
```

除此之外，A0 阶段不修改任何已有源码。

### 0.5 新增文件如何处理

DeepSeek 新增文件先不删除，也不移动。因为原 CMake 和 launch 已恢复，它们不会进入原单机构建和运行链。

暂时保留的内容包括：

- phase-offset 几何、allocator、tube、CBF、neighbor model；
- 新消息；
- 多机 simulator；
- 新 launch、测试、YAML 和 PCD 地图；
- 实施日志和旧计划。

这些文件只能作为参考原型。进入 A1 以后，必须逐个审核，不能整批重新接回 CMake。

### 0.6 回退后的干净构建

当前 build/devel 是在 DeepSeek 修改后的 CMake 和源码上生成的，因此恢复源码后必须重新构建，不能直接复用已有二进制判断单机是否正常。

建议执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
catkin_make clean
catkin_make -j8
source devel/setup.bash
```

如果 clean build 失败，只修复恢复过程中暴露的基线构建问题，不接回任何 phase-offset 或多机模块。

### 0.7 回退完成的判据

完成回退后，`git status --short` 中允许出现：

- `test_gvf.launch` 的两个用户开关修改；
- DeepSeek 新增的未跟踪文件；
- 本实施计划。

不应再出现 `gvf_manager.cpp`、SDF、原 simulator、CMake/package 等已有文件的 DeepSeek 修改。

然后进入 A0 的 ROS 单机回归。A0 没跑通以前，不开始 A1。

## 1. 正确的总体顺序

最终方法仍然是

\[
g_i^{\mathrm{swarm}}
\longrightarrow
(u_{w,i},u_{\delta,i})
\longrightarrow
\text{matched phase--offset port}
\longrightarrow
\text{ISF-GVF}.
\]

当前问题是一次性加入了多机 simulator、通信、邻居模型、tube、QP、CBF、闭合轨迹和多个场景，导致原单机链路被侵入，问题也无法逐层定位。

新的实现顺序固定为

\[
\boxed{
\text{恢复原单机}
\rightarrow
\text{单机 phase--offset 内核}
\rightarrow
\text{单机完整闭环}
\rightarrow
\text{三机通信 shadow}
\rightarrow
\text{三机集群 active}
\rightarrow
\text{复杂集群场景}.
}
\]

单机阶段不是改变论文问题，而是先把最终每架 UAV 都要运行的公共内核调通。

## 2. 原 proposal 中哪些功能必须先做成单机

每架 UAV 的公共内核包括：

1. 原 ISF-GVF、semantic phase 和现有重参数化路径；
2. 现有 \(C^2\) 增量拼接；
3. phase–offset 几何

   \[
   r(w,\delta)=p(w)+N(w)\delta;
   \]

4. 相对活动参考的误差

   \[
   e=x-r(w,\delta);
   \]

5. matched port

   \[
   \dot x=f_x^{\mathrm{ISF}}+r_wu_w+Nu_\delta,
   \]

   \[
   \dot w=f_w^{\mathrm{ISF}}+u_w,
   \qquad
   \dot\delta=u_\delta;
   \]

6. matched cancellation 数值验证；
7. phase 正向、物理切向非零、offset 和正则性约束；
8. 基础路径 \(p\) 与活动路径 \(r\) 的 RViz 显示；
9. 单机安全管道；
10. 路径更新时 \(w,\delta\) continuation；
11. 原 command governor 对活动参考 \(r\) 的正确执行；
12. 点到点终端行为。

这些功能不依赖邻居，应该先在一架 UAV 上验证。

多机阶段才允许加入：

- `SwarmState` 等通信消息；
- 邻居缓存、LOS 和 freshness；
- 分离、弱凝聚和相对径向阻尼；
- 本机局部邻居聚合意图 \(g_i^{\mathrm{swarm}}\)；
- 机间 CBF；
- 多机 simulator、三机/七机场景；
- 狭窄通道、circle 和 figure-eight 集群任务。

## 3. 不可突破的代码保护规则

### 3.1 原单机入口保持不变

原回归入口仍为：

- `so3_quadrotor_simulator/launch/simulator.launch`；
- `bspline_traj/launch/test_gvf.launch`。

默认启动这两个 launch 时，不得加载 phase-offset、多机通信、邻居、CBF 或新场景。

保留用户已有设置：

```xml
<param name="gvf/circle_test/enable" value="false" />
<param name="gvf/circle_test/auto_start" value="false" />
```

### 3.2 单机新功能采用三态模式

```text
phase_offset/mode = disabled | shadow | active
```

- `disabled`：严格走原单机代码路径；
- `shadow`：只计算新几何和诊断，不改变控制；
- `active`：实际使用活动参考与 matched port。

集群另设独立开关：

```text
swarm/enabled = false | true
```

只有单机 active 全部验收后，才允许开启 swarm。

### 3.3 禁止再次重写主文件

`gvf_manager.cpp` 只能增加少量接口调用和状态连接。几何、端口投影、tube 和调试必须在独立模块中实现，不能再次向 manager 堆积上千行代码。

第一阶段不修改 A*、B 样条优化器、原地图格式、SO3 动力学和最大速度参数。降低正常速度只同比例修改 \(K_1,K_2\)。

## 4. A0：恢复原单机基线

### 工作范围

恢复 DeepSeek 对以下原有链路的侵入：

- `gvf_manager.h/.cpp`；
- `gvf.h/.cpp`；
- `formation_planning.cpp`；
- `bspline_traj/CMakeLists.txt` 和 `package.xml`；
- `common_msgs` 原有 CMake/header；
- `sdf_map.h/.cpp`；
- 原 `quadrotor_simulator_so3.cpp`；
- 原 `simulator.launch`；
- simulator 的 CMake/package。

`test_gvf.launch` 恢复原结构，但保留用户的两个 circle 开关。所有新增文件暂时保留，但退出原 CMake 和 launch 链，不直接删除。

### 验收

只使用原 `pillar.pcd`：

1. 原 simulator 正常启动；
2. 原 test_gvf 正常启动；
3. 用户可在 RViz 手工发布目标；
4. 单机生成原局部 A*/B 样条路径；
5. UAV 避开静态障碍并到达目标；
6. 不存在 phase-offset、swarm、CBF timer 或控制输出；
7. 保存基线 rosbag、轨迹和启动日志。

A0 没通过，不进入 A1。

## 5. A1：独立 phase–offset 几何

该阶段只做离线模块和单元测试，不接入 ROS 控制循环。

输入：

\[
p,\quad p_w,\quad p_{ww},\quad w,\quad\delta,\quad x.
\]

输出：

\[
T,\quad N,\quad\kappa,\quad r,\quad r_w,
\quad e_\parallel,\quad e_\perp.
\]

必须验证：

\[
r=p+N\delta,
\]

\[
r_w=(1-\kappa\delta)p_w,
\]

\[
r_w^TN=0,
\qquad
1-\kappa\delta\ge\mu>0.
\]

测试直线、圆和 8 字局部段；验证解析 \(r_w\) 与数值导数一致，且 \(\delta=0\) 时退化为原路径几何。

A1 最初按固定高度 XY 假设完成。A2.1 的真实路径诊断随后证明原规划链会
产生有意义的高度变化，因此 A2.2 获准在不修改 planner 和 C2 的前提下，把
该几何契约推广为重力参考 2.5D。A1 的平面公式保留为推广后公式的严格特例。

## 6. A2：单机 shadow 模式

原 ISF-GVF 继续产生真实控制命令。新模块只旁路计算：

- 固定测试 offset \(\delta_{\mathrm{shadow}}\) 下的候选参考
  \(r^{\mathrm{shadow}}=p+N\delta_{\mathrm{shadow}}\)；
- 当前基础路径状态到 `PathDifferentialState` 的转换；
- 对应的 \(T,N,\kappa,r^{\mathrm{shadow}},r_w^{\mathrm{shadow}}\)；
- 曲率与正则性诊断；
- 基础路径和 shadow 候选参考 Marker。

A2 不实现端口、matched residual 和 tube。这三项分别留给 A4 和 A5。

因此，A2 中的 \(\delta_{\mathrm{shadow}}\) 只是只读几何测试参数，不是已经建立动力学的 \(\delta\) 状态；\(r^{\mathrm{shadow}}\) 也不宣称满足 ESDF tube 安全性，不能作为真实控制参考。真正的活动参考只在后续完成端口状态、约束和 tube 后进入 active 控制。

shadow 模式不得修改 `phase_w_`、`PositionCommand`、governor 输入、B 样条、C2 path 或 SO3 命令。

验收：同一初始状态和目标下，`disabled` 与 `shadow` 的实际控制和轨迹在数值容差内一致。

## 6.1 A2.1：真实路径平面性诊断

A2.1 只增加原始路径 z 范围、z 导数、无效样本和 Marker 删除诊断，不修改
控制、容差、C2 或 core 理论。

真实点到点回程测得约 \(0.451\,\mathrm m\) 的高度跨度，并出现显著非零的
\(p_{w,z}\) 和 \(p_{ww,z}\)。因此固定高度假设不适用于原规划链，不能把这些
分量当作噪声置零。

## 6.2 A2.2：变高度基础路径的重力参考 2.5D 几何

A2.2 保持标量横向状态 \(\delta\)，允许基础路径 \(p(w)\) 完整三维升降，
并以世界重力方向定义水平法向：

\[
e_z=(0,0,1)^T,
\qquad
N=\frac{e_z\times p_w}{\|e_z\times p_w\|}.
\]

定义

\[
N_w
=
\frac{(I-NN^T)(e_z\times p_{ww})}
{\|e_z\times p_w\|},
\]

则

\[
r=p+N\delta,
\qquad
r_w=p_w+N_w\delta.
\]

活动参考切向使用完整三维

\[
T=\frac{r_w}{\|r_w\|}.
\]

要求水平切向非退化，并保持水平非折返正则性。固定高度输入必须严格退化为
A1 原二维公式；\(\delta=0\) 时必须严格满足 \(r=p\)、\(r_w=p_w\)。

A2.2 仍只做 core 和 Shadow 重验，不实现 Bishop frame、二维 offset、active、
matched port 或 tube。A2.2 没通过，不进入 A3。

A2.2 实测 `shadow_delta=0.40` 时有 5 次候选路径因局部曲率违反 offset
正则性而删除；current geometry 始终有效，且没有任何旧平面性拒绝。该结果
不阻塞固定 `delta=0` 的 A3，但必须保留到 A4/A5，禁止把 0.40 m 当作全路径
固定可行偏移。

## 7. A3：单机 active、零端口等价性

设置

\[
\delta=0,
\qquad
u_w=0,
\qquad
u_\delta=0.
\]

使用活动参考接口实际控制 UAV。因为

\[
r(w,0)=p(w),
\]

新接口必须与原 ISF-GVF 等价。

验收：

- \(v_g\) 和 \(\dot w\) 与原结果一致；
- 飞行轨迹、避障结果和终点误差与 A0 一致；
- matched residual 接近机器精度；
- 不出现无原因 EMERGENCY、phase 跳变或重新规划。

A3 没通过，不允许使用非零 offset。

## 8. A4：人工 offset 与 matched port

先不加入任何邻居。使用平滑测试输入，例如

\[
\delta_{\mathrm{ref}}(t)=A\sin(\omega t)
\]

或平滑的左移、回中、右移命令。

实际执行必须同时包含：

\[
\dot x=f_x^{\mathrm{ISF}}+r_wu_w+Nu_\delta,
\]

\[
\dot w=f_w^{\mathrm{ISF}}+u_w,
\qquad
\dot\delta=u_\delta.
\]

RViz 必须同时显示基础路径 \(p\)、活动路径 \(r\)、当前参考点、\(T,N\)、UAV 轨迹以及 \(\delta,u_w,u_\delta\)。

验收：

- UAV 随活动参考平滑横移；
- 原 ISF-GVF 不把 UAV 强行拉回基础路径；
- 撤销输入后 \(\delta\) 平滑回零；
- 误差始终相对 \(r\) 计算；
- matched residual 保持数值精度量级。

## 9. A5：单机端口约束与安全管道

先使用固定管道

\[
-\delta_{\max}\le\delta\le\delta_{\max}.
\]

端口投影只处理：

1. \(\dot w>0\)；
2. 物理切向速度保持正值；
3. \(|u_w|,|u_\delta|\) 和周期变化率；
4. offset 边界；
5. \(1-\kappa\delta\ge\mu\)。

固定管道通过后，再由本机 ESDF 沿基础路径法向采样，得到

\[
\underline\delta(w),\quad\overline\delta(w).
\]

此时只处理静态障碍物和本机活动参考，不处理其他 UAV。CBF 最多作为 tube 边界保障，不作为 matched port 的前提，也不加入机间约束。

验收：活动路径不进入膨胀障碍区；管道变窄时 offset 平滑收回；限幅后的同一端口同时作用于物理端和内部端；不通过修改最大速度掩盖问题。

## 10. A6：复用现有 C2 路径拼接，验证 phase–offset/tube continuation

这一阶段不新增第二套 C2 connector。原代码已经能够对基础路径

\[
p^-(w)\longrightarrow p^+(w)
\]

进行 C2 兼容拼接。这里继续直接复用该结果，不重写原 connector。

路径更新时保留

\[
w^+=w^-,
\qquad
\delta^+=\delta^-.
\]

利用现有 \(C^2\) connector 验证

\[
r^+\approx r^-,
\qquad
r_w^+\approx r_w^-.
\]

原因是：若原 connector 在切换边界保持 \(p,p_w,p_{ww}\) 连续，且 \(p_w\neq0\)，则由它计算的 \(T,N,\kappa\) 也保持兼容。切换瞬间保留同一个 \(\delta\) 后，活动参考

\[
r=p+N\delta
\]

以及固定 \(\delta\) 下的

\[
r_w=(1-\kappa\delta)p_w
\]

不会因为基础路径换版产生硬跳变。

tube 本身不是另一条需要单独进行 C2 拼接的 B 样条。新路径接受后，只需要重新计算新路径对应的

\[
\underline\delta^+(w),\quad\overline\delta^+(w),
\]

并检查当前保留的 \(\delta^-\) 是否仍满足新 tube 边界和正则性约束。若 tube 边界来自离散 ESDF 采样，还需要进行连续插值或低通平滑，避免端口约束突然跳变；这属于 tube 边界更新，不属于新的 C2 路径拼接。

验收：不修改原 C2 connector 主体；path epoch 只在接受新基础路径时变化；正常 phase 前进不 reset；\(\delta\) 不重置；当前 \(\delta\) 对新 tube 可行；更新处没有活动参考或物理命令尖峰；UAV 能继续完成点到点任务。

## 11. A7：原地图单机完整闭环

只使用：

- 原 `pillar.pcd`；
- 原局部地图和 A*/B 样条规划器；
- 用户在 RViz 手工发布目标；
- phase-offset active；
- 本机 tube；
- matched port；
- 原 SO3 执行层。

第一轮降低速度只修改：

```text
gvf_gain1: 2.0 -> 0.8
gvf_gain2: -2.2 -> -0.88
```

二者保持原比例，所有最大速度、搜索速度和饱和边界保持原值。

连续运行至少 10 次，要求每次都能接收手工目标、规划成功、避障到达；无 NaN、无崩溃、无无原因 phase 反向或 EMERGENCY；基础路径和活动路径含义清楚。

只有完成 A7，才开始多机扩展。

## 12. B0：隔离式多机仿真基础

多机仿真使用独立 executable 和 launch，不再次修改原单机 simulator 的默认行为。

要求：

- 原 `simulator.launch` 不变；
- 全局地图和 RViz 只启动一次；
- 每架 UAV 有独立 odom、SO3 command 和 local sensing；
- multi simulator 只积分动力学，不计算集群控制；
- `num_agents=1` 时与原单机动力学等价。

## 13. B1：三机独立导航，不开集群

三架 UAV 都运行已经通过 A7 的单机内核，但设置

\[
g_i^{\mathrm{swarm}}=0.
\]

在原 `pillar.pcd` 上验证每机话题隔离、独立局部地图、独立规划器和独立点到点导航。这一步只证明多实例没有破坏单机内核，不证明集群效果。

## 14. B2：通信和邻居聚合 shadow

加入高频状态通信和邻居缓存，但集群意图只计算、不执行。

每架 UAV 独立计算

\[
g_i^{\mathrm{swarm}}
=
\sum_{j\in\mathcal N_i}g_{ij}^{\mathrm{pos}}
+
\sum_{j\in\mathcal N_i}g_{ij}^{\mathrm{damp}}
+g_i^{\mathrm{safe}}.
\]

每架 UAV 只聚合当前有效局部邻居，不存在中心节点的全局集群力。关闭 shadow 后，三机轨迹必须不变。

## 15. B3：三机集群 active

把本机局部集群意图分解为

\[
u_{w,i}^{\mathrm{raw}}
=
\frac{T_i^Tg_i^{\mathrm{swarm}}}{\|r_{w,i}\|},
\qquad
u_{\delta,i}^{\mathrm{raw}}
=N_i^Tg_i^{\mathrm{swarm}}.
\]

经过 A5 已验证的端口投影器后，调用 A4–A7 已验证的 matched port。多机阶段不能再修改 matched cancellation 公式。

第一版只做三架 UAV 在原地图上从左向右点到点：保持松散、无固定槽位的集群；各自沿局部路径绕开静态障碍；不保持固定队形和固定前后顺序；三架 UAV 都到达各自目标。

## 16. B4：机间 CBF 保障

只有三机低速集群闭环稳定后，才加入机间 CBF。CBF 只过滤最终端口可行集，不改邻居意图、phase–offset 分解、matched port 或 ISF-GVF 误差动力学。

第一轮先做两机同步低速测试，再进入三机原地图。异步 reciprocal handshake、严格 sampled-data 证明和复杂冲突调度不与第一轮实现同时进行。

## 17. 最后才做的扩展

以下内容全部排在 B4 之后：

1. 狭窄通道和 channel beta；
2. circle 集群；
3. 8 字集群和 branch event；
4. 七机 ACTIVE；
5. 多机异步增量换路；
6. 消融与论文统计。

## 18. 给代码执行者的停止规则

每个阶段必须单独汇报修改文件、编译结果、运行命令和验收指标。只有当前阶段通过并得到确认后，才能进入下一阶段。

遇到失败时：

- 只修当前阶段；
- 不提前实现后续功能；
- 不增加场景专用模式；
- 不关闭重规划或放宽安全指标伪造通过；
- 不通过降低最大速度解决正常速度问题；
- 不大规模重写原文件；
- 不自动连续完成 A0–B4。

当前只允许执行 A0。原单机重新跑通并保存基线以前，不实施 A1，也不继续任何多机代码。
