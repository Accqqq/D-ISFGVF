# DeepSeek A4 单机人工 Offset 与 Matched Port 执行单

```text
DOCUMENT_ROLE=STAGE_EXECUTION_SPEC
STAGE=A4
PREVIOUS_STAGE=A3
AUTO_ADVANCE=false
ALLOWED_TO_EXECUTE=A4_ONLY
```

## 1. 唯一目标

本阶段只完成单机非零 phase-offset 端口闭环：

1. 在纯 Eigen/C++14 的 `phase_offset_core` 中加入 `port_types`、
   `MatchedPort` 和不依赖安全管道的基础 `PortProjector`；
2. 将 A3 已验证的活动参考从固定 `delta=0` 推广为持续积分的标量内部状态
   `delta`；
3. 使用人工、平滑、可重复的 `delta_ref(t)` 和小幅 `u_w` 测试输入；
4. 将同一组最终实际执行的 `u_w`、`u_delta` 同时作用于物理通道和内部
   `w`、`delta` 通道；
5. 证明非零端口下 matched cancellation、正向 phase、正物理切向裕度和
   2.5D 高度保持成立；
6. 让 UAV 随活动参考平滑横移，并在人工输入撤销后平滑回到 `delta=0`。

本阶段仍是最终分布式 PhaseOffsetSwarm 的单机内核验证，不是新的研究方向。

本阶段不实现固定 tube、ESDF tube、tube preview、neighbor、swarm、机间 CBF、
分布式意图、continuation checker 或新的 C2 connector。

## 2. 与详细 Proposal 和现有结论的关系

详细 proposal 的核心状态保持为

\[
z=(x,w,\delta),
\qquad
r(w,\delta)=p(w)+N(w)\delta.
\]

`delta` 的严格含义是活动参考点 `r` 的横向坐标，不是每周期从 UAV 位置重新
投影得到的测量值。真实 UAV 状态满足

\[
x=r+e.
\]

因此：

- `delta` 必须持续积分；
- 禁止用最近点距离覆盖 `delta`；
- 禁止在路径重规划或普通控制周期中把 `delta` 突然置零；
- A4 的跟踪误差必须始终相对 `r` 计算。

A2.2 已证明真实基础路径会有完整三维高度变化。本阶段继续使用重力参考 2.5D
几何：

\[
e_z=(0,0,1)^T,
\qquad
N=\frac{e_z\times p_w}{\|e_z\times p_w\|},
\]

\[
N_w=
\frac{(I-NN^T)(e_z\times p_{ww})}
{\|e_z\times p_w\|},
\]

\[
r=p+N\delta,
\qquad
r_w=p_w+N_w\delta,
\qquad
T=\frac{r_w}{\|r_w\|}.
\]

不得退回固定高度 XY 假设，不得丢弃或置零 `p_w.z()`、`p_ww.z()`。

A2.2 在 `shadow_delta=0.40` 时出现过 5 次合法
`offset regularity margin is violated`。A4 必须保留该事实：

- 禁止把 `0.40 m` 作为默认全路径可行幅值；
- 人工幅值必须经过真实当前路径的双侧正则性预检；
- 禁止放宽 `GeometryParams::regularity_margin` 来使测试通过。

## 3. A3 已验收基线

开始 A4 前必须确认：

- branch=`main`；
- HEAD=`9a0e975`；
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` 仍存在；
- `git diff --check` 通过；
- A3 Kernel 6/6；
- A3 Active adapter 6/6；
- lifted guidance 9/9；
- phase-offset core 11/11；
- Shadow adapter 13/13；
- `catkin_test_results --verbose` 只有既存 `uav_utils` 缺失 XML；
- A3 ROS 诊断 303 周期中 legacy 99、active 204；
- 第 100 个连续周期、约 1.979 s 开门；
- 开门后 fallback=0、failure latch=0；
- 六类 guidance 和三类 zero-port residual 最大值均为 0；
- `/position_cmd` 唯一发布者仍为 `/formation_planning`；
- A3 没有非零 `delta`、`u_w`、`u_delta`、matched port 或 phase 写回。

A4 开始时 tracked diff 必须仍为 A3 收口后的 7 个文件：

```text
src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/launch/test_gvf.launch
src/swarm_planner/bspline_traj/package.xml
src/swarm_planner/bspline_traj/src/gvf.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_lifted_visualization_test.cpp
```

其中 `package.xml` 只允许保留 A2 已有的 `phase_offset_core` 依赖，不得在 A4
继续修改。旧 prototype 的 untracked 文件仍只作参考，不能因其存在而扩大 A4
白名单。

若任一前置条件不满足，停止并报告，不得开始 A4。

## 4. A4 与 A5 的边界消歧

较新的代码架构要求 A4 在 core 中创建 `MatchedPort` 和 `PortProjector`；总路线
把固定/ESDF tube 放在 A5。A4 按以下边界执行。

### 4.1 A4 中允许的 PortProjector 职责

只处理不依赖 tube 或地图的单机运动学可行性：

1. 输入有限性；
2. `u_w`、`u_delta` 的幅值限制；
3. 两个端口相对上一周期最终值的变化率限制；
4. 最终 phase 速度严格正向；
5. 最终物理切向速度保持正裕度；
6. 当前局部曲率下，下一步 `delta` 不越过已有正则性裕度。

这些约束使用解析区间求交和 clamp 即可。禁止引入 QP solver、OSQP、CVX、
CBF 或优化器依赖。

### 4.2 必须留到 A5 的内容

- 固定 `[-delta_max, delta_max]` tube；
- 随 `w` 变化的 `delta_lower(w)`、`delta_upper(w)`；
- tube 边界导数和前向不变约束；
- ESDF/占据地图采样；
- tracking-error inflation；
- tube preview、窄通道收缩和管道过滤；
- `phase_offset_navigation` package。

A4 的 `PortProjector` API 可以在 A5 扩展，但不得提前创建空 tube 字段、地图
接口或 navigation package。

## 5. Matched Port 理论不变量

令 A3 Kernel 相对当前活动参考输出基础 ISF 项：

\[
f_x^{\mathrm{ISF}},
\qquad
f_w^{\mathrm{ISF}}.
\]

`PortProjector` 输出最终实际执行端口：

\[
u^\star=(u_w^\star,u_\delta^\star).
\]

`MatchedPort` 必须且只能组合：

\[
\dot x=
f_x^{\mathrm{ISF}}+r_wu_w^\star+Nu_\delta^\star,
\]

\[
\dot w=f_w^{\mathrm{ISF}}+u_w^\star,
\qquad
\dot\delta=u_\delta^\star.
\]

定义

\[
e=x-r(w,\delta).
\]

则 matched residual 必须按完整误差动力学计算：

\[
\epsilon_{\mathrm{matched}}
=
\left(\dot x-r_w\dot w-N\dot\delta\right)
-
\left(f_x^{\mathrm{ISF}}-r_wf_w^{\mathrm{ISF}}\right).
\]

验收要求

\[
\|\epsilon_{\mathrm{matched}}\|
\]

保持双精度机器精度量级。

禁止只检查 `r_w u_w-r_w u_w` 或 `N u_delta-N u_delta` 的单独代数项而不检查
完整组合残差。

同一最终端口必须满足：

- physical channel 使用 `u_w_final`、`u_delta_final`；
- phase update 使用同一个 `u_w_final`；
- delta integration 使用同一个 `u_delta_final`；
- diagnostics 报告的 final port 与上述三处完全一致。

禁止 physical channel 使用 raw port，而内部状态使用 projected port，或反之。

## 6. 正向运动条件

`u_w` 可以是负修正，但最终 phase 禁止反向。要求：

\[
\dot w=f_w^{\mathrm{ISF}}+u_w^\star
\ge \dot w_{\min}>0.
\]

phase 正向本身不足以自动证明物理切向正向，因此还必须显式保持：

\[
T^T\dot x
=
T^Tf_x^{\mathrm{ISF}}
+\|r_w\|u_w^\star
\ge v_{\mathrm{tan,min}}>0.
\]

由于 2.5D 几何仍满足 `T^T N=0`，`u_delta` 不进入物理切向投影。

不得通过修改 K1、K2、带宽、planner 速度、最大速度或原饱和界使本阶段通过。
只能限制新端口。

正常人工 profile 中禁止 commanded hold、停车排队或 phase 反向。已有紧急安全
兜底可以保留，但验收运行中不得触发。

## 7. 开始前阅读与 Git 自检

完整阅读：

- 根目录 `AGENTS.md`；
- `PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`；
- `PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`；
- A1、A2、A2.1、A2.2、A3 执行单；
- `phase_offset_core` 当前全部源码、README 和测试；
- `ISFReferenceKernel` 和 Active adapter 全部源码、测试；
- `gvf_manager::cmdCallback()` 的 A3 接入；
- 原 phase 更新、governor 和 C2 测试。

执行并记录：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git stash list
git diff --numstat 9a0e975 -- \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp
catkin_test_results --verbose
```

必须记录 A4 开始前 `gvf_manager.cpp` 相对基线累计新增 151 行。A4 不允许继续
把大段状态机、公式或 profile 生成堆入 callback。

## 8. 文件白名单

### 8.1 允许新增

```text
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_types.h
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/matched_port.h
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_projector.h
src/swarm_planner/phase_offset/phase_offset_core/src/matched_port.cpp
src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/matched_port_test.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/port_projector_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/launch/phase_offset_manual_single.launch
```

### 8.2 允许修改

```text
src/swarm_planner/phase_offset/phase_offset_core/CMakeLists.txt
src/swarm_planner/phase_offset/phase_offset_core/README.md
src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

共 17 个白名单文件。若实际实现需要修改任何其他文件，立即停止并报告，不得
扩大范围。

特别禁止修改：

- `geometry.*`、`geometry_types.h`、`path_state.h`；
- `ISFReferenceKernel`；
- A3 `phase_offset_active_adapter.*` 及其测试；
- A2 Shadow adapter；
- `gvf.cpp`、`gvf.h`；
- `continuous_phase_path.*`；
- planner、A*、B-spline、C2、SDF、governor、simulator、SO3；
- 任意 `package.xml`；
- `common_msgs`；
- 旧 prototype 文件。

## 9. Core 类型设计

`port_types.h` 只表达端口、投影和 matched 输出，不混入 ROS、路径消息、地图、
邻居或 swarm 类型。

建议至少包含等价概念：

```cpp
struct PortCommand {
  double u_w = 0.0;
  double u_delta = 0.0;
};

struct PortProjectionLimits {
  double u_w_abs_max = 0.0;
  double u_delta_abs_max = 0.0;
  double u_w_rate_max = 0.0;
  double u_delta_rate_max = 0.0;
  double phase_dot_min = 0.0;
  double tangent_speed_min = 0.0;
  double regularity_margin = 0.1;
};

struct PortProjectionInput {
  PortCommand raw;
  PortCommand previous_final;
  double dt = 0.0;
  double delta = 0.0;
  double curvature = 0.0;
  double r_w_norm = 0.0;
  double base_w_dot = 0.0;
  double base_tangent_speed = 0.0;
};

struct PortProjectionResult {
  PortCommand final_port;
  double final_w_dot = 0.0;
  double final_tangent_speed = 0.0;
  double next_delta = 0.0;
  double next_regularity = 0.0;
  bool u_w_limited = false;
  bool u_delta_limited = false;
  bool valid = false;
  std::string invalid_reason;
};
```

具体命名可调整，但必须保持一个头文件一个概念，输出始终有限，失败时
`valid=false` 且原因非空。

不得在 core 类型中保存 ROS 时间、node、publisher、topic、robot ID、path epoch
或动态参数服务器对象。

## 10. MatchedPort 职责

`MatchedPort` 必须是无状态纯数学模块。它只：

1. 接收有效 geometry；
2. 接收 Kernel 的基础 `v_cmd`、`w_dot`；
3. 接收最终 port；
4. 计算 physical port `r_w*u_w + N*u_delta`；
5. 计算最终 `v_cmd`、`w_dot`、`delta_dot`；
6. 计算完整 matched residual；
7. 检查所有输出有限。

它不得：

- 生成 raw port；
- 限幅或投影 port；
- 积分 `w` 或 `delta`；
- 读取时间；
- 读取参数；
- 访问地图、邻居、CBF、ROS 或 governor；
- 重新实现 ISF 公式。

`u_w=u_delta=0` 时必须严格退化到 A3 基础输出。

## 11. PortProjector 职责

`PortProjector` 必须是无状态或显式 previous-final 输入的纯数学模块。禁止在
对象内部隐藏时间和历史状态。

推荐投影顺序：

1. 验证输入、limits 和 `dt`；
2. 对 raw port 做幅值区间裁剪；
3. 与 previous-final 的变化率区间求交；
4. 从 `base_w_dot + u_w >= phase_dot_min` 得到 `u_w` 下界；
5. 从 `base_tangent_speed + r_w_norm*u_w >= tangent_speed_min` 得到另一
   `u_w` 下界；
6. 对全部 `u_w` 区间求交；
7. 使用 `delta_next=delta+dt*u_delta` 和当前局部 curvature 保持
   `1-curvature*delta_next >= regularity_margin`；
8. 对全部 `u_delta` 区间求交；
9. 输出唯一、有限的 final port 和限制原因。

如果可行区间为空，返回 invalid；禁止偷偷把正向裕度降为零，禁止改变
regularity margin，禁止生成 hold。

本阶段的局部一步 regularity 只用于人工小幅端口的运动学保护，不等价于 A5
的全路径安全 tube 或 preview。

## 12. 人工输入与 Delta 状态

新增 `PhaseOffsetMatchedAdapter`，作为 `ContinuousPhasePathState`、A3 Kernel、
core geometry、PortProjector、MatchedPort 与 manager 之间的薄适配层。

它可以拥有 A4 运行状态：

- 当前 `delta`；
- previous final port；
- profile elapsed time；
- startup gate 状态；
- failure latch；
- 当前路径双侧正则性预检结果。

不得把这些状态放进 core。

人工 profile 必须：

- 从 `delta=0`、port=0 开始；
- 使用 C1 或更高连续的平滑左移、回中、右移、回中；
- 或使用有平滑启停包络的正弦 `delta_ref(t)`；
- 包含至少一个小幅非零 `u_w` 正修正和负修正窗口；
- 最终回到 `delta_ref=0`、`u_w_raw=0`；
- 不使用阶跃 offset；
- 不使用邻居或地图输入生成命令。

`delta` 必须由最终 `u_delta` 积分：

\[
\delta_{k+1}=\delta_k+\Delta t\,u_{\delta,k}^{\star}.
\]

允许 raw offset port 使用平滑 feedforward 加误差反馈，例如

\[
u_\delta^{raw}=\dot\delta_{ref}+k_\delta(\delta_{ref}-\delta),
\]

但参数只属于 A4 adapter/launch，不得进入 Kernel 或 geometry。

禁止直接执行 `delta=delta_ref`，禁止从 UAV 位置重新估计并覆盖 `delta`。

## 13. 双侧正则性预检

人工 profile 启动前，必须对当前完整 semantic path 以不大于 `w=0.1` 的采样
步长分别检查 `delta=+A` 和 `delta=-A`：

- 所有 path state 有效；
- 两侧 geometry 均有效；
- candidate path complete；
- 最小 regularity 不低于 core 既有 margin；
- `r.z-p.z` 为机器精度量级；
- 不出现近竖直水平切向退化。

只有 A3 零端口 gate 已打开且双侧预检通过，人工 profile 才能启动。

配置幅值大于当前路径安全幅值时必须拒绝启动并报告，不得自动放宽 margin。
允许 adapter 将实际幅值保守缩小到配置上限以下，但必须：

- 发布 configured amplitude；
- 发布 accepted amplitude；
- 发布缩小原因；
- accepted amplitude 仍足以形成可测横移；
- 若无法得到至少 `0.05 m` 的双侧可行幅值，停止 ROS 验收并报告。

路径更新后重新执行预检。若新路径不支持下一次 excursion，则保持或平滑返回
`delta=0`，不得突然 reset。若当前 geometry 已无效，则锁存失败并使用既有紧急
安全处理；验收中该情况必须为 0 次。

不得修改原 C2 connector 或路径安装逻辑。

## 14. A3 Startup Gate 与模式

保留现有模式：

```text
disabled
shadow
active
```

新增：

```text
manual
```

语义：

- `active` 必须保持 A3 零端口行为和 14 字段 diagnostics 完全不变；
- `manual` 启动后先固定 `delta=0`、port=0，复用 A3 等价比较连续至少
  100 个周期；
- gate 打开且双侧预检通过后才允许非零人工 port；
- gate 前使用 legacy 输出；
- 人工非零阶段不再要求与 legacy 等价，因为活动参考已故意改变；
- 人工阶段不得在 manual 和 legacy 间周期性切换；
- 非零阶段发生 invalid 时锁存失败，禁止静默继续或频繁切换。

A3 active 模式的回归是 A4 的硬性验收项。

## 15. Manager 接入限制

`gvf_manager` 仍只允许：

1. 解析 mode 和 A4 新参数；
2. 构造 adapter；
3. 准备路径状态、position、gains 和 `dt`；
4. 调用 adapter；
5. 将 adapter 选择的最终 guidance 交给原 phase 更新和 governor；
6. 发布 adapter 已生成的 diagnostics/Marker 数据。

callback 中禁止出现：

- matched 公式；
- port 投影公式；
- profile 公式；
- `delta` 积分公式；
- regularity 区间计算；
- residual 公式。

A4 开始时 `gvf_manager.cpp` 已相对基线新增 151 行。A4 应通过把 A3 gate 和
A4 端口编排封装进 integration adapter，保持 manager 净增量仍接近约 150 行。

目标：

```text
git diff --numstat 9a0e975 -- gvf_manager.cpp
```

新增行最好不超过约 160。若超过 170 行，视为明显超过架构目标，必须停止并
重新拆分；不得以“只多几十行”为理由把状态机留在 callback。

不得修改原 phase 积分、endpoint clamp、phase reset、governor 或
`PositionCommand` 发布函数。

## 16. 参数纪律

允许新增的参数仅属于 manual port：

```text
phase_offset/manual/amplitude
phase_offset/manual/profile_period
phase_offset/manual/warmup_cycles
phase_offset/manual/delta_tracking_gain
phase_offset/manual/u_w_amplitude
phase_offset/manual/u_w_abs_max
phase_offset/manual/u_delta_abs_max
phase_offset/manual/u_w_rate_max
phase_offset/manual/u_delta_rate_max
phase_offset/manual/phase_dot_min
phase_offset/manual/tangent_speed_min
phase_offset/manual/preflight_sample_step_w
```

命名可小幅调整，但禁止散落到 core。

默认人工 amplitude 不得为 `0.40`。推荐从 `0.10 m` 或更小值开始，并以真实
双侧预检结果为准。

不得修改：

- K1、K2；
- convergence bandwidth；
- progress rho/delta、alpha；
- planner 速度、加速度和饱和；
- governor 参数；
- C2 参数；
- geometry epsilon 和 regularity margin。

## 17. Diagnostics

manual diagnostics 使用 `std_msgs/Float64MultiArray`，不新增消息。字段顺序必须
固定并在代码注释、测试和最终报告中一致。至少发布：

```text
mode_manual
zero_gate_open
zero_gate_consecutive_count
failure_latched
profile_active
preflight_complete
configured_amplitude
accepted_amplitude
delta_ref
delta
delta_tracking_error
u_w_raw
u_delta_raw
u_w_final
u_delta_final
u_w_limited
u_delta_limited
base_w_dot
final_w_dot
base_tangent_speed
final_tangent_speed
current_regularity
next_regularity
matched_residual_norm
physical_port_norm
r_minus_p_norm
r_z_minus_p_z
selected_manual
```

可以增加必要字段，但不得删除上述语义。

所有字段必须 finite。invalid 时不得把 NaN 发布为 0 来伪装正常；必须同时通过
valid/failure 字段和明确日志报告原因。仅用于消息数值安全的 finite fallback
不得改变控制判定。

## 18. RViz 可视化

manual 模式至少显示：

- 基础路径 `p`；
- 当前 `delta` 对应的完整活动路径 `r`；
- 当前活动参考点；
- 当前 `T`、`N`；
- UAV 既有轨迹；
- `delta`、`u_w_final`、`u_delta_final` 的文字 Marker。

要求：

- active path 高度逐点等于 base path 高度；
- Marker 使用固定 namespace 和 ID；
- candidate 不完整时 active path 发布 DELETE；
- current geometry 无效时当前参考、T、N、文字 Marker 均 DELETE；
- visualization 只读，不发布控制命令。

禁止修改旧 Shadow Marker 语义。

## 19. Core 单元测试

### 19.1 MatchedPort

至少覆盖：

1. `u_w=u_delta=0` 严格退化到 A3；
2. 固定高度直线，两个端口同时非零；
3. 曲线和非零 `delta`；
4. 三维变高度路径；
5. 多组正负 `u_w`、`u_delta` 的完整 matched residual；
6. physical/internal 使用同一 final port；
7. invalid geometry、invalid base guidance、NaN/Inf 安全失败；
8. 所有失败输出 finite。

### 19.2 PortProjector

至少覆盖：

1. raw port 在全部限制内时保持不变；
2. `u_w` 幅值上下限；
3. `u_delta` 幅值上下限；
4. 两类 rate limit；
5. phase 正向下界激活；
6. physical tangent margin 激活；
7. phase 和 tangent 下界同时激活；
8. 正、负 curvature 下的下一步 regularity；
9. 零 curvature；
10. 空可行区间 invalid；
11. invalid `dt`、limit、NaN/Inf；
12. 输出和 diagnostics finite。

core 测试只链接 `phase_offset_core`，不得链接 ROS 或 `bspline_race`。

## 20. Integration Adapter 单元测试

至少覆盖：

1. `ContinuousPhasePathState` 转换保留完整三维导数；
2. startup gate 前 `delta=0`、port=0；
3. 100 周期严格等价后才打开 gate；
4. gate 前失败重置连续计数；
5. 双侧 amplitude preflight；
6. `0.40 m` 在构造的高曲率路径上被拒绝；
7. 平滑 profile 产生正负 offset；
8. `delta` 只由 final `u_delta` 积分；
9. raw port 被限制时 physical/internal 使用同一个 final port；
10. 非零 `u_w` 和 `u_delta` 的 matched residual；
11. 变高度路径下 `r.z-p.z=0`；
12. profile 结束后平滑回零；
13. 路径预检失败时不启动新的 excursion；
14. invalid 后 failure latch，不在 manual/legacy 间抖动；
15. diagnostics 字段数量、顺序和 finite；
16. invalid Marker DELETE 行为。

禁止通过测试访问 manager private state。

## 21. CMake 与依赖

`phase_offset_core` library 加入：

```text
src/matched_port.cpp
src/port_projector.cpp
```

新增两个独立 core gtest target。

在 `bspline_race` 中新增独立最小 library：

```text
phase_offset_matched_adapter
```

它只链接必要的：

- `phase_offset_core`；
- `isf_reference_kernel`；
- A3 active adapter（仅用于 startup zero gate）；
- 必要 catkin/ROS visualization 依赖。

不得把新源码继续直接堆进 `bspline_gvf` source list。不得新增 package 依赖，
不得修改任何 `package.xml`。

## 22. 构建与回归测试

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

- A1/A2.2 geometry 11/11 保持；
- Shadow adapter 13/13 保持；
- A3 Kernel 6/6 保持；
- A3 Active adapter 6/6 保持；
- lifted guidance 9/9 保持；
- 新 MatchedPort、PortProjector、matched adapter 全部通过；
- 原 bspline_race 回归项不减少；
- 只允许既存 `uav_utils` 缺失 XML；
- 无 A4 新增 warning、failure 或 skipped test。

## 23. ROS 隔离验收

先检查现有 ROS master 和进程。不得连接、停止或复用用户进程。使用新的隔离
master 和独立 `ROS_LOG_DIR`，只清理本阶段启动的进程。

保持：

- 原 `pillar.pcd`；
- 原 simulator；
- 原 planner、C2、governor 和 SO3；
- 原速度和控制参数。

### 23.1 Disabled 回归

- mode 默认 disabled；
- 无 phase-offset active/manual topic；
- 原目标正常到达；
- `/position_cmd` 唯一 publisher 为 `/formation_planning`。

### 23.2 A3 Active 零端口回归

重新运行 `phase_offset_active_zero_single.launch`：

- 100 周期开门；
- 开门后 fallback=0、failure latch=0；
- A3 九类残差保持机器精度；
- UAV 到达；
- 不出现 A4 topic 或非零 port。

### 23.3 Manual 非零端口

启动：

```bash
roslaunch bspline_race phase_offset_manual_single.launch
```

运行至少一个完整的：

```text
zero warmup
→ left offset
→ center
→ right offset
→ center
```

并包含小幅正、负 `u_w` 修正。使用真实点到点任务，覆盖首次规划、重规划、
已有 C2、governor 和最终到达，但不得修改这些模块。

必须记录：

```text
startup gate 开启周期和时间
configured/accepted amplitude
双侧 preflight 样本数、invalid 数、最小 regularity
delta_ref、delta 的 min/max 和最大跟踪误差
u_w_raw/final、u_delta_raw/final 的 min/max
两类 projection 激活次数和原因
base/final w_dot 最小值
base/final physical tangent speed 最小值
matched residual 最大值
physical port 最大值
r-p 横向一致性残差
r.z-p.z 最大值
manual selected 周期
manual invalid、failure latch、fallback 次数
profile 回零后的 delta 和 final port
已有 replan/C2 次数及观察到的 w/delta 跳变
最终 odom 和目标误差
/position_cmd publisher
diagnostics 频率
EMERGENCY/hold 次数
```

验收阈值：

- accepted amplitude 双侧可行且不小于 `0.05 m`；
- 实际 `delta` 正负两侧都达到 accepted amplitude 的至少 80%；
- `delta` 跟踪连续，无阶跃 reset；
- profile 结束后 `|delta| <= 1e-3 m`；
- profile 结束后 `|u_w_final|, |u_delta_final| <= 1e-3`；
- `r.z-p.z` 为机器精度量级；
- matched residual 不超过 `1e-10`；
- `final_w_dot >= phase_dot_min > 0`；
- `final_tangent_speed >= tangent_speed_min > 0`；
- current/next regularity 不低于既有 margin；
- manual 非零阶段 fallback=0、failure latch=0；
- commanded hold=0、无 A4 引入的 EMERGENCY；
- UAV 正常避障和到达；
- `/position_cmd` 仍只有 `/formation_planning`；
- matched adapter 不发布控制命令。

对 C2 只记录观察结果，不修改 connector，不宣称完成 A6 continuation 验收。

## 24. 依赖与禁止内容审核

执行：

```bash
rg -n "ros|SDF|Map|B[Ss]pline|Neighbor|Swarm|CBF|tube|robot_id" \
  src/swarm_planner/phase_offset/phase_offset_core

rg -n "tanh\(|alpha_min|progress_rho0|progress_delta|convergence_bandwidth" \
  src/swarm_planner/phase_offset/phase_offset_core \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

rg -n "PositionCommand|cmd_pub|publishGovernorPositionCommand|SDFMap|CBF|Neighbor|Swarm" \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

rg -n "phase_w_\s*=|progress_w_\s*=|projectToPathLocal" \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

rg -n "delta_lower|delta_upper|ESDF|DistanceQuery|tube|preview" \
  src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_types.h \
  src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_projector.h \
  src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp

git diff --check
git status --short
git diff --stat
git diff --numstat 9a0e975 -- \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp
```

允许搜索命中注释或测试名称时必须逐项解释，不能只报告“有命中”。

## 25. 自审核要求

最终必须确认：

- core 仍只有 Eigen/STL；
- `MatchedPort` 不生成、不投影 port；
- `PortProjector` 不包含 tube、地图、neighbor、swarm、CBF；
- matched adapter 不重新实现 ISF 或 geometry 公式；
- `delta` 是持续内部状态，不由 UAV 投影覆盖；
- 同一 final port 用于 physical、phase、delta 三个通道；
- active reference error 相对 `r`；
- A3 active 零端口行为未改变；
- A2 Shadow 行为未改变；
- `shadow_delta=0.40` 的 5 次 regularity violation 历史仍保留；
- manager 仍是薄编排层；
- planner、C2、governor、PositionCommand、SO3、simulator 未改；
- 只修改白名单文件；
- branch、HEAD、stash 保持；
- 无 staged 内容、commit、branch、tag、push；
- `git diff --check` 通过。

## 26. 严格禁止

1. neighbor、swarm、robot ID 或多实例；
2. 固定 tube、ESDF tube、tube preview 或 navigation package；
3. CBF、安全 QP 或任何外部优化器；
4. 修改 geometry 公式、epsilon、regularity margin；
5. 把 z 导数投影或置零；
6. 修改 Kernel/ISF 公式或增益；
7. 修改 phase 更新律、endpoint clamp 或 phase reset；
8. 修改 planner、A*、B-spline、C2、SDF、governor、SO3、simulator；
9. 直接把人工速度加到原 ISF 输出而不经过 matched port；
10. physical 和 internal 使用不同 port；
11. 用 raw port 更新一个通道、projected port 更新另一个通道；
12. 每周期从 UAV 位置重算 `delta`；
13. 突然 reset 或 clip `delta`；
14. 正常模式 commanded hold、phase 反向或零切向裕度；
15. 将 `0.40 m` 当作默认可行幅值；
16. 修改 `common_msgs` 或创建新消息；
17. 恢复或连接旧 prototype；
18. 创建 commit；
19. 自动进入 A5。

## 27. 停止条件

出现以下任一情况立即停止并报告：

- branch、HEAD、stash 或 A3 基线不符；
- 需要修改白名单外文件；
- 真实路径无法提供至少 `0.05 m` 双侧正则人工幅值；
- nonzero manual 必须依赖 tube/ESDF 才能安全验收；
- final phase 或物理切向无法保持严格正裕度；
- matched residual 不能达到数值精度；
- manager 无法保持薄适配和行数目标；
- A3 active 零端口回归失败；
- ROS runtime 不可用；
- 工作区测试被无关历史缺陷阻塞。

不得为绕过停止条件而修改理论、参数、planner、C2 或控制链。

## 28. 最终汇报格式

1. 开始 branch、HEAD、stash、tracked/untracked 状态；
2. A3 基线复核；
3. 实际新增/修改文件；
4. `port_types` API；
5. MatchedPort 公式、API 和 residual；
6. PortProjector A4 边界和解析投影；
7. 人工 profile、delta 持续积分和双侧预检；
8. manager 行数和薄接入；
9. core、adapter、A1-A3 和原回归测试；
10. Disabled 和 A3 active ROS 回归；
11. Manual profile、幅值、port、正向裕度和回零结果；
12. matched residual、三通道同 port 证据；
13. 变高度和 `r.z-p.z` 结果；
14. replan/C2 仅观察结果；
15. `/position_cmd`、EMERGENCY、hold；
16. 依赖搜索和禁止内容审核；
17. Git diff、白名单和 stash 审核；
18. A2.2 的 `0.40 m` regularity 结果仍保留；
19. 明确写出：`A4 完成后已停止，未进入 A5`。

本执行单授权 A4 代码实施。完成、测试、自审核和汇报后必须停止。
