# DeepSeek A3 单机 Active 零端口等价执行单

```text
DOCUMENT_ROLE=STAGE_EXECUTION_SPEC
STAGE=A3
PREVIOUS_STAGE=A2.2
AUTO_ADVANCE=false
ALLOWED_TO_EXECUTE=A3_ONLY
```

## 1. 唯一目标

本阶段只完成：

1. 把现有 `calcLiftedGuidanceAtPhase()` 中的 authoritative-phase ISF 参考
   几何公式机械抽取为该路径模式唯一的 `ISFReferenceKernel`；
2. 使用 A2.2 已通过的三维变高度几何，在
   `delta=0, u_w=0, u_delta=0` 时生成活动参考；
3. 先并行比较原路径输出和零端口活动参考输出；
4. 连续满足严格等价门槛后，才允许活动参考输出进入原 governor；
5. 证明实际 UAV 控制、phase 更新、避障和到达行为与原系统等价。

本阶段不实现非零 offset，不实现 matched port、PortProjector、tube、CBF 或
swarm。

## 2. 理论与代码边界

固定：

\[
\delta=0,
\qquad
u_w=0,
\qquad
u_\delta=0.
\]

A2.2 必须给出：

\[
r(w,0)=p(w),
\qquad
r_w(w,0)=p_w(w),
\qquad
T(w,0)=\frac{p_w}{\|p_w\|}.
\]

因此新入口必须满足：

\[
v_g^{active}=v_g^{legacy},
\qquad
\dot w^{active}=\dot w^{legacy}.
\]

A3 中所谓 `zero-port residual` 只用于记录上述几何与 guidance 等价残差。
不得借此提前实现 A4 的一般 `MatchedPort`。

## 3. A2.2 已知结论

开始前确认：

- 2.5D core 11/11；
- Shadow adapter 13/13；
- 原 bspline_race 64 项通过；
- `catkin_test_results` 仅有既存 `uav_utils` 缺失 XML；
- 真实路径不再因 z 导数被拒绝；
- current geometry invalid 为 0；
- `r.z-p.z=0`；
- `/position_cmd` 唯一发布者仍是 `/formation_planning`。

A2.2 在 `shadow_delta=0.40` 时记录到 5 次合法的
`offset regularity margin is violated`。这不阻塞 A3，因为本阶段固定
`delta=0`、`regularity=1`；但禁止通过修改正则性掩盖该历史结果。

A2.2 整个阶段实际修改了白名单中的 8 个文件。若先前汇报只写最后修复的
2 个文件，应在 A3 开始状态中更正，不得把 untracked 文件误当成未修改。

## 4. 开始前阅读与 Git 自检

完整阅读：

- 根目录 `AGENTS.md`；
- 总体计划和代码架构；
- A2、A2.1、A2.2 执行单；
- `phase_offset_core` 当前 geometry、README 和测试；
- `gvf::calcLiftedGuidanceAtPhase()`；
- `gvf_manager::cmdCallback()`；
- command governor 相关测试。

执行并记录：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git stash list
catkin_test_results --verbose
```

必须确认：

- branch=`main`；
- HEAD=`9a0e975`；
- prototype stash 仍存在；
- tracked diff 仍只有 A2 的 5 个授权文件；
- A2.2 没有修改 manager、launch、CMake、planner、C2 或控制链；
- `circle_test/enable=false`、`auto_start=false`；
- 当前没有 A3 代码或 active 控制接入。

不满足则停止汇报。

## 5. 文件白名单

### 5.1 允许新增

```text
src/swarm_planner/bspline_traj/include/bspline_race/guidance/isf_reference_kernel.h
src/swarm_planner/bspline_traj/src/guidance/isf_reference_kernel.cpp
src/swarm_planner/bspline_traj/test/isf_reference_kernel_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_active_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_active_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_active_adapter_test.cpp
src/swarm_planner/bspline_traj/launch/phase_offset_active_zero_single.launch
```

### 5.2 允许修改

```text
src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/src/gvf.cpp
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/launch/test_gvf.launch
src/swarm_planner/bspline_traj/test/gvf_lifted_visualization_test.cpp
```

共 13 个文件。若需要其他文件，停止并汇报，不得扩展白名单。

禁止修改：

- `phase_offset_core` 全部文件；
- Shadow adapter 三个文件；
- `gvf.h`；
- `continuous_phase_path.*`；
- A*、B-spline、C2、SDF、governor 算法和 simulator；
- `package.xml`、`common_msgs` 和旧 prototype。

## 6. ISFReferenceKernel

### 6.1 依赖边界

Kernel 位于 `bspline_race/guidance`，只允许依赖 Eigen 和 C++ 标准库。
禁止包含 ROS、`gvf.h`、地图、B-spline、manager、phase-offset、neighbor、CBF
或消息头。

建议等价类型：

```cpp
struct ReferenceGeometry {
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  double derivative_norm = 0.0;
  bool valid = false;
};

struct IsfGains {
  double k1 = 0.0;
  double k2 = 0.0;
  double convergence_bandwidth = 0.0;
  double progress_rho0 = 0.0;
  double progress_delta = 0.0;
  double alpha_min = 0.0;
};

struct IsfGuidance {
  Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
  double w_dot = 0.0;
  double e_parallel = 0.0;
  Eigen::Vector3d e_perp = Eigen::Vector3d::Zero();
  Eigen::Vector3d ref_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  bool valid = false;
  std::string invalid_reason;
};
```

命名可小幅调整，但不能加入 ROS 和控制模式状态。

### 6.2 Authoritative-phase 唯一公式

Kernel 必须机械保持当前 authoritative phase 公式和现有 K2 符号约定：

\[
e=x-r,
\qquad
e_\parallel=T^Te,
\qquad
e_\perp=e-e_\parallel T,
\qquad
\rho=\|e_\perp\|.
\]

\[
q(\rho)=
\begin{cases}
\tanh(\rho/b)/\rho,&\rho>10^{-6},\\
1/b,&\rho\le10^{-6},
\end{cases}
\]

其中：

\[
b=\max(10^{-6},\text{convergence bandwidth}).
\]

\[
\alpha
=
\alpha_{min}
+\frac{1-\alpha_{min}}
{1+(\rho/\rho_0)^2},
\qquad
\rho_0=\max(10^{-6},\text{progress rho0}).
\]

\[
\sigma=\tanh(e_\parallel/d),
\qquad
d=\max(10^{-6},\text{progress delta}).
\]

\[
v_g=K_1\alpha T+K_2q e_\perp,
\]

\[
\dot w
=
\frac{K_1(\alpha+\sigma)}{\|r_w\|}.
\]

注意当前 launch 中 `K2` 本身为负。禁止把代码改写成新的正增益减号形式，
禁止修改增益、clamp、阈值或误差定义。

输出失败时所有数值字段 finite、`valid=false`、原因非空。

### 6.3 原入口机械迁移

`gvf::calcLiftedGuidanceAtPhase()` 只保留：

1. 查询当前 `p(w)`、`p_w(w)`；
2. 构造 `ReferenceGeometry` 和 `IsfGains`；
3. 调用唯一 Kernel；
4. 映射回原 `LiftedGuidanceResult`，并保持 `w_proj=w`。

不得在 `gvf.cpp` 中保留第二份 alpha、sigma、q 或 `v_cmd/w_dot` 公式。

`calcLiftedGuidance3D()` 的局部投影和 progress 语义不是 A3 的活动模式，
其历史公式允许继续留在 `gvf.cpp`，本阶段禁止顺手修改。单一公式源要求只
针对 `calcLiftedGuidanceAtPhase()`、active adapter 和 active manager 链。

## 7. PhaseOffsetActiveAdapter

### 7.1 职责

Active adapter 只做：

```text
ContinuousPhasePathState + semantic w
    -> PathDifferentialState
    -> A2.2 GeometryEvaluator(delta=0)
    -> ReferenceGeometry(r,T,||r_w||)
    -> ISFReferenceKernel
    -> zero-port geometry residual
```

它不得：

- 接收非零 delta；
- 保存或更新 delta；
- 定义 `u_w`、`u_delta`；
- 实现 matched physical term；
- 访问 governor、PositionCommand、phase 状态或 ROS publisher；
- 查询地图或路径最近点。

建议输出：

```text
guidance
geometry
||r-p||
||r_w-p_w||
||T-p_w/||p_w||||
valid / invalid_reason
```

所有比较使用同一个输入 position、同一个 semantic phase 和同一组 gains。

### 7.2 单一 phase 来源

Active 只允许在 `unifiedPhaseV2Active() && phase_initialized_` 且当前
`ContinuousPhasePath` 可在 `phase_w_` evaluate 时运行。

禁止：

- 使用 `projectToPathLocal()`；
- 新建第二个 phase 状态；
- 重置或重对齐 `phase_w_`；
- 在 active adapter 内推进 phase。

## 8. Mode 与安全接管

### 8.1 模式

保留：

```text
phase_offset/mode = disabled | shadow | active
```

- `disabled`：完全保持原控制链；
- `shadow`：保持 A2/A2.2 行为；
- `active`：只允许 A3 零端口 active；
- 其他字符串：报错并退回 disabled。

Active 模式没有 delta 参数。即使 launch 或参数服务器存在
`phase_offset/shadow_delta`，active 也必须内部固定 `delta=0`。

### 8.2 零 offset Shadow 预检

在任何 active ROS 验收之前，先运行同一路径：

```bash
roslaunch bspline_race phase_offset_shadow_single.launch shadow_delta:=0.0
```

完整往返必须满足：

- 所有成功路径快照 `invalid_sample_count=0`；
- `candidate_path_complete=true`；
- current invalid=0；
- 没有 planar、horizontal-degenerate 或 regularity warning；
- `r=p`、`r_w=p_w`、T 为三维原路径切向；
- UAV 正常到达，Shadow 无控制 publisher。

预检失败立即停止，不得进入 active。

### 8.3 并行等价门

Active 启动后，每个可评估周期同时获得：

```text
legacy = gvf::calcLiftedGuidanceAtPhase(pos, phase_w)
active = PhaseOffsetActiveAdapter(delta=0)
```

记录至少：

```text
||v_active-v_legacy||
abs(w_dot_active-w_dot_legacy)
abs(e_parallel_active-e_parallel_legacy)
||e_perp_active-e_perp_legacy||
||ref_active-ref_legacy||
||T_active-T_legacy||
zero-port geometry residuals
```

默认等价容差建议 `1e-10`，允许作为参数集中解析，但不得在运行中自动放宽。

只有连续至少 100 个有效控制周期全部满足容差，才允许：

```text
selected guidance = active
```

门开启前实际控制继续使用 legacy。

### 8.4 失败回退

若发生以下任一情况：

- active/legacy 任一无效；
- 路径或 semantic phase 不可用；
- 任一残差超过容差；
- 出现非 finite；

则该周期使用 legacy。若已经接管，则锁存 active failure，本次进程后续保持
legacy，并明确 throttle error；禁止在 active/legacy 间频繁切换。

回退不得：

- 修改 phase；
- 触发额外 EMERGENCY；
- 生成 hold command；
- 清空原路径；
- 改变 governor 状态。

## 9. manager 接入限制

`gvf_manager` 只允许：

1. 解析 active mode、容差和连续周期数；
2. 构造 active adapter；
3. 在现有 authoritative phase 分支准备输入；
4. 并行获得 legacy/active 输出并记录残差；
5. 根据等价门选择 active 或 legacy；
6. 将选中的原 `LiftedGuidanceResult` 交给未修改的 phase 更新和 governor。

所有 ISF 公式、几何公式和比较公式放在独立模块；callback 只编排。

A2+A3 对 `gvf_manager.cpp` 的累计新增应尽量不超过约 150 行。若明显超过，
停止并重新拆分，不得把算法堆进 callback。

不得修改：

- `phase_w_` 更新公式；
- endpoint clamp；
- initial acquisition；
- governor 候选、代价、饱和和状态；
- PositionCommand 生成和发布；
- path install、C2 和 replan 状态机。

## 10. ROS 诊断

可以使用一个 `std_msgs::Float64MultiArray` private topic：

```text
~phase_offset_active/diagnostics
```

字段顺序至少记录：

```text
mode_active
gate_open
gate_consecutive_count
active_failure_latched
v_cmd_residual
w_dot_residual
e_parallel_residual
e_perp_residual
reference_residual
tangent_residual
r_minus_p_norm
r_w_minus_p_w_norm
geometry_tangent_residual
selected_source        # 0 legacy, 1 active
```

不新增自定义消息。所有字段 finite，并在代码注释中写清顺序。

诊断不得被 governor 或控制决策读取；只有等价门读取 adapter 的直接比较结果。

## 11. 单元测试

### 11.1 Kernel golden regression

在修改 `calcLiftedGuidanceAtPhase()` 前，先从当前公式建立硬编码数值基准。
最终测试不得再复制一遍完整公式，而应比较固定期望值。

至少覆盖：

1. 三维斜切向、零误差；
2. 正/负沿程误差；
3. 非零法向误差；
4. `rho<=1e-6` 分支；
5. 当前负 K2 符号；
6. 非单位切向、零 derivative norm、非 finite 输入拒绝；
7. bandwidth/rho0/delta 的现有 `1e-6` clamp；
8. 输出始终 finite。

### 11.2 原 gvf 回归

扩展 `gvf_lifted_visualization_test.cpp`：

- 原现有测试全部保持；
- hard-coded representative input 的 `v_cmd/w_dot/e/ref/T` 保持；
- authoritative phase 不进行最近点投影；
- 变高度 `ContinuousPhasePath` 使用完整三维 tangent。

### 11.3 Active adapter

至少覆盖：

1. field-by-field path 转换；
2. 斜直线 `delta=0` 的 `r=p`、`r_w=p_w`、三维 T；
3. 变高度曲线输出与直接 Kernel 完全一致；
4. zero-port 三类几何残差为机器精度；
5. 与 legacy representative guidance 的所有残差满足 `1e-12`；
6. 近竖直、非法路径、非法 gains 安全失败；
7. API 不存在非零 delta、u_w、u_delta 和控制写回。

## 12. CMake

新增独立最小 library：

```text
isf_reference_kernel
phase_offset_active_adapter
```

要求：

- kernel test 只链接 kernel；
- active adapter test 只链接 adapter、kernel、phase_offset_core 和必要 catkin；
- `bspline_gvf` 链接 kernel 和 active adapter；
- 不把新源码直接继续堆进 `bspline_gvf` source list；
- 不修改 package 依赖。

## 13. 构建与测试

执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
catkin_make -j8
catkin_make run_tests_phase_offset_core
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

要求：

- A2.2 core 11/11；
- Shadow adapter 13/13；
- 新 kernel 和 active adapter 测试全部通过；
- 原 bspline_race 回归不减少；
- 仅允许既存 `uav_utils` XML 缺失；
- 无新增编译警告和失败。

## 14. ROS 运行验收

检查现有 ROS master，不连接、停止或复用用户进程。使用隔离 master，只清理
本阶段启动的进程。

### 14.1 Disabled 回归

原 simulator + `test_gvf.launch`：

- mode 默认 disabled；
- 不存在 active topic；
- 原目标完整到达；
- 行为与 A0/A2 回归一致。

### 14.2 Shadow delta=0 预检

按第 8.2 节完整执行，满足零 invalid 和完整 candidate 后才继续。

### 14.3 Active 零端口

启动：

```bash
roslaunch bspline_race phase_offset_active_zero_single.launch
```

使用与 A2.2 同类的往返目标，覆盖：

- 首次规划；
- 多次重规划；
- C2 connector；
- governor；
- 终点段和最终到达。

必须报告：

```text
active gate 开启时间和所需周期数
active selected 周期数
legacy fallback 周期数和原因
六类 guidance 最大残差
三类 zero-port geometry 最大残差
active invalid 次数
phase 跳变次数
无原因 EMERGENCY 次数
replan/C2 结果
最终 odom 和目标误差
/position_cmd publisher
active diagnostics 频率
```

验收：

- gate 正常开启；
- 接管后持续使用 active；
- 接管后 residual 超限和 fallback 均为 0；
- guidance 残差不超过配置容差；
- geometry 残差为机器精度量级；
- UAV 正常避障和到达；
- phase 无额外 reset/jump；
- 不出现 A3 引入的 EMERGENCY；
- `/position_cmd` 仍只有 `/formation_planning`；
- active adapter 不发布控制命令；
- 原 governor 和 SO3 链未改变。

## 15. 依赖与单一公式源审核

执行：

```bash
rg -n "tanh\(|alpha_min|progress_rho0|progress_delta|convergence_bandwidth|w_dot" \
  src/swarm_planner/bspline_traj/src/guidance \
  src/swarm_planner/bspline_traj/src/integration \
  src/swarm_planner/bspline_traj/src/gvf.cpp \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp

rg -n "PositionCommand|cmd_pub|Governor|SDFMap|CBF|Neighbor|Swarm|u_delta|u_w" \
  src/swarm_planner/bspline_traj/include/bspline_race/guidance \
  src/swarm_planner/bspline_traj/src/guidance \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_active_adapter.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_active_adapter.cpp

rg -n "projectToPathLocal|phase_w_\s*=|progress_w_\s*=|PositionCommand" \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_active_adapter.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_active_adapter.cpp

git diff --check
git status --short
git diff --stat
```

确认：

- `calcLiftedGuidanceAtPhase()`、active adapter 和 active manager 使用的
  authoritative-phase ISF 公式只有 kernel 一个实现源；
- 搜索仍可命中未授权修改的历史 `calcLiftedGuidance3D()`，但不得在 active
  adapter、active manager 或 `calcLiftedGuidanceAtPhase()` 中出现第二份公式；
- active adapter 无 ROS、控制写入、地图、governor 和 phase 写回；
- 只修改白名单 13 个文件；
- planner、C2、governor、simulator、core、Shadow adapter 未改变；
- tracked diff 只在 A2 既有 5 文件中增加 A3 授权内容；
- prototype stash 保持；
- 无 commit、branch、tag 或 push。

## 16. 严格禁止

1. 非零 delta；
2. 定义或执行 `u_w`、`u_delta`；
3. 实现 MatchedPort 或 PortProjector；
4. 实现 tube、ESDF、CBF、neighbor、swarm；
5. 修改 phase 更新律、endpoint clamp 或 phase reset；
6. 修改 governor、PositionCommand、SO3；
7. 修改 A*、B-spline、C2、SDF 或 simulator；
8. 修改 K1/K2、带宽、alpha、sigma 或速度参数；
9. 修改 `phase_offset_core` 或 Shadow adapter；
10. 用 active 失败触发 hold 或 EMERGENCY；
11. 恢复旧 prototype；
12. 创建 commit；
13. 自动进入 A4。

## 17. 最终汇报格式

1. 开始 Git/测试状态；
2. 新增与修改文件；
3. Kernel API、公式和依赖；
4. 原 `calcLiftedGuidanceAtPhase` 的机械迁移；
5. Active adapter API 和零端口限制；
6. 单元测试、原回归和总测试；
7. Shadow delta=0 预检；
8. Active 等价门和接管结果；
9. 全部最大残差；
10. UAV 到达、phase、replan/C2、EMERGENCY；
11. `/position_cmd` 和 ROS graph；
12. 单一公式源与禁止依赖搜索；
13. 最终 Git 状态和白名单审核；
14. A2.2 的 0.40 regularity 结果仍保留；
15. 明确写出：`A3 完成后已停止，未进入 A4`。

任何验收需要非零 offset、matched port、tube 或修改原控制链时，停止并汇报，
不得扩大范围。
