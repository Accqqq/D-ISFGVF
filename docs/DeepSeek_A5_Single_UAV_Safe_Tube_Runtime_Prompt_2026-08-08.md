# DeepSeek A5 单机安全 Tube 与 Navigation Runtime 执行单

工作目录：

`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

阶段控制：

`AUTO_ADVANCE=false`

本轮只执行 A5。完成构建、测试、ROS 验收、自审核和汇报后立即停止，禁止进入
A6。

## 1. 唯一目标

在 A4 已验证的单机 matched port 上完成以下闭环：

1. 创建独立的 `phase_offset_navigation` package；
2. 先实现并验收固定 offset tube；
3. 再通过抽象 `DistanceQuery` 接入本机 ESDF，构造随 phase 变化的双侧安全
   tube；
4. 将 A4 integration adapter 中的 `delta`、previous final port、人工 profile
   状态、tube 状态和单机运行时编排迁入 `PhaseOffsetRuntime`；
5. 扩展 `PortProjector`，联合处理 `u_w`、`u_delta` 与随 phase 变化的 tube
   前向不变约束；
6. 继续使用同一组最终端口同时驱动物理、phase 和 delta 三通道；
7. 对 UAV 尺寸、定位误差、跟踪误差、地图误差和离散采样误差做保守 erosion，
   保证安全对象是实际 UAV，而不只是活动参考点；
8. 保持原 planner、B-spline、C2 connector、governor、`PositionCommand`、SO3
   和 simulator 工作不变。

本阶段不实现 neighbor、swarm、多机消息、机间 CBF、continuation checker、新
C2 connector、路径 phase 重映射或 A6 内容。

## 2. 与详细 Proposal 的关系

A5 是详细 Proposal 中 phase-shape safe envelope 的单机一维 2.5D 验证，不是
新的研究方向，也不改变最终多机目标。

本阶段对应：

\[
r(w,\delta)=p(w)+N(w)\delta ,
\]

\[
d_{\mathrm{req}}
=r_{\mathrm{UAV}}
+m_{\mathrm{localization}}
+m_{\mathrm{tracking}}
+m_{\mathrm{map}}
+m_{\mathrm{extra}} .
\]

详细 Proposal 后续允许更高维局部 B-spline shape 或多机 intent；A5 只验证
每个 agent 最底层必须具备的：

- 活动参考与基础路径分离；
- matched phase-shape port；
- 静态障碍允许的横向 envelope；
- 端口投影后的前向不变性；
- 实际跟踪误差 erosion。

因此 A5 不修改基础 B-spline 控制点，不实现局部 shape QP，也不偏离 Proposal。
后续 swarm 只会把人工 raw intent 替换为本地 `g_swarm`，不能改变本阶段验证的
最终端口和安全 tube 链路。

## 3. A4 已验收基线

开始前必须确认：

- branch 为 `main`；
- HEAD 为 `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`；
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` 仍存在；
- 无 staged 内容、commit、tag 或 push；
- A4 tracked diff 仍只有既有 7 个文件；
- `gvf_manager.cpp` 相对基线累计新增 166 行；
- `phase_offset_matched_adapter.cpp` 当前为 677 行；
- A4 core、Shadow、Kernel、A3 Active、Matched adapter 与 lifted guidance
  回归均通过；
- `catkin_test_results --verbose` 仅有已知无关的 `uav_utils` 缺失 gtest XML；
- A4 ROS matched residual 最大值约 `4.58e-16`；
- A4 中 `r.z-p.z=0`，phase 与物理切向速度严格为正；
- `/position_cmd` 唯一发布者为 `/formation_planning`。

A4 另有一个必须保留到 A6 的观察：

- 已接入 C2 切换日志中曾出现 `phase_before != phase_after`；
- 最大观察差约 `0.040469`；
- `delta` 未 reset。

该观察不阻塞 A5，但 A5 禁止宣称 continuation 已通过，也不得修改 C2 来消除
该观察。

如果 branch、HEAD、stash、A4 diff 或测试前提不同，立即停止并报告。

## 4. 开始前完整阅读

完整阅读：

- 根目录 `AGENTS.md`；
- `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`；
- `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`；
- A1、A2、A2.1、A2.2、A3、A4 的全部专用执行单；
- `phase_offset_core` 当前全部源码、README、CMake、package 和测试；
- A2 Shadow、A3 Active、A4 Matched adapter 全部源码和测试；
- `ISFReferenceKernel` 全部源码和测试；
- `ContinuousPhasePath` 的公开接口和采样实现；
- `gvf_manager` 中 A2–A4 的参数解析与 `cmdCallback()` 接入；
- `SDFMap` 的 `isInMap`、`getInflateOccupancy`、`getDistance`、buffer 初始化、
  cloud callback、ESDF update 与 sentinel 语义；
- 旧 untracked `path_tube_builder.*`、相关测试和配置，只作逐行参考。

旧 prototype 只能参考，禁止修改、编译、整体复制或连接回 CMake。

开始前执行并保存原始输出：

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
wc -l \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
catkin_test_results --verbose
```

另外保存开始时所有 tracked 与 untracked 路径清单，结束时逐项比较。

## 5. A5 与 A6 的严格边界

A5 允许：

- 对当前 semantic path 的局部 preview 样本构造 tube；
- 路径对象或定义域变化时使旧 tube cache 失效；
- 对新路径同步重建当前 preview tube；
- 检查当前保留的 `delta` 是否在新 tube 内；
- 记录 C2/replan 发生时的只读诊断。

A5 禁止：

- 创建 `ContinuationChecker`；
- 比较旧路径和新路径的 `p,p_w,p_{ww},r,r_w` 连续性；
- 修改或包装原 C2 connector；
- phase remap、phase reset 或 path epoch 语义；
- 在切换时重置 `delta`；
- 宣称 A6 continuation 验收完成。

可在 adapter 内维护一个仅用于 tube cache invalidation 的不透明
`source_revision`。它不是论文语义中的 path epoch，不得发布成 continuation
结论。

## 6. 文件白名单

### 6.1 允许新增

```text
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/offset_constraint.h

src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
src/swarm_planner/phase_offset/phase_offset_navigation/package.xml
src/swarm_planner/phase_offset/phase_offset_navigation/README.md
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/distance_query.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_filter.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp

src/swarm_planner/bspline_traj/launch/phase_offset_fixed_tube_single.launch
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
```

### 6.2 允许修改

```text
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_types.h
src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/port_projector_test.cpp
src/swarm_planner/phase_offset/phase_offset_core/README.md

src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/package.xml
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

共 29 个白名单文件。需要修改任何其他文件时立即停止并报告，不得扩大范围。

特别禁止修改：

- `geometry.*`、`geometry_types.h`、`path_state.h`；
- `matched_port.*`；
- `ISFReferenceKernel`；
- A2 Shadow 和 A3 Active adapter；
- `gvf.cpp`、`gvf.h`；
- `ContinuousPhasePath`；
- `SDFMap`、`plan_env`；
- planner、A*、B-spline、C2、governor、simulator、SO3；
- `common_msgs`；
- 旧 prototype 的 tube、allocator、CBF、swarm 文件；
- 路线文档和本执行单。

## 7. 依赖方向

必须保持：

```text
Eigen/STL
    ↓
phase_offset_core
    ↓
phase_offset_navigation
    ↓
bspline_race integration adapter
    ↓
原 governor / PositionCommand / SO3
```

`phase_offset_navigation`：

- 只依赖 C++14、Eigen 和 `phase_offset_core`；
- 不依赖 ROS、ROS time、消息、`SDFMap`、`plan_env`、B-spline、manager、
  robot ID、neighbor、swarm、CBF、publisher、subscriber 或 SO3；
- 不保存具体地图对象；
- 不生成最终 ROS 控制命令。

`phase_offset_core` 仍为纯数学库。新增 generic offset constraint 后仍不得知道
TubeProfile、ESDF 或地图。

## 8. DistanceQuery 必须显式表达状态

禁止只使用：

```cpp
std::function<double(const Eigen::Vector3d&)>
```

因为单个 `double` 无法可靠区分：

- 查询源尚未准备；
- 点在地图外；
- ESDF 未计算或未知；
- 已知自由；
- 已占据。

在 `distance_query.h` 中定义类似：

```cpp
enum class DistanceStatus {
  UNAVAILABLE,
  OUT_OF_MAP,
  UNKNOWN,
  KNOWN_FREE,
  OCCUPIED
};

struct DistanceQueryResult {
  DistanceStatus status = DistanceStatus::UNAVAILABLE;
  double signed_distance = 0.0;
};

using DistanceQuery = std::function<
    DistanceQueryResult(const Eigen::Vector3d&)>;
```

具体字段名可小幅调整，但五种语义必须保留。

navigation 中只有 `KNOWN_FREE` 且 finite 的距离可用于扩大安全 tube。
`UNAVAILABLE`、`OUT_OF_MAP`、`UNKNOWN` 和 `OCCUPIED` 绝不能当作无限自由空间。

## 9. SDFMap 薄适配规则

`SDFMap` 只允许在 `bspline_race` integration adapter 中出现。adapter 将
`SDFMap` 包装为 `DistanceQuery`，navigation 不得 include `plan_env`。

当前 simulator 的 cloud-only 地图中，occupancy unknown flag 不能单独作为 ESDF
可用性的判据；已有 distance buffer 在未计算时使用约 `10000` sentinel。

adapter 必须按现有代码事实构造状态：

1. map 指针为空：`UNAVAILABLE`；
2. `isInMap(p)==false`：`OUT_OF_MAP`；
3. distance 非 finite 或达到配置的 unobserved sentinel 阈值：`UNKNOWN`；
4. inflated occupancy 非零或 signed distance 非正：`OCCUPIED`；
5. 其余 finite、已计算值：`KNOWN_FREE`。

unobserved threshold 必须显式配置并验证，默认可按现有 `10000` 初始化值选择
保守阈值，例如 `1000`。禁止通过把 sentinel 当作大净空来生成假安全 tube。

不得修改 `SDFMap` 来迎合 A5，也不得把旧 prototype 的 `mapReady()` 假设复制
进来。

## 10. Tube 类型

`tube_types.h` 至少拆分：

- tube source：`NONE`、`FIXED`、`ESDF`；
- raw sample；
- filtered profile；
- 当前 query bounds；
- build/filter diagnostics；
- runtime tube status。

每个 phase sample 至少包含：

- `w`；
- raw lower/upper；
- filtered lower/upper；
- lower/upper 对 `w` 的导数；
- sample 是否完整认证；
- regularity intersection；
- unknown、out-of-map、occupied 等停止原因。

profile 必须明确：

- preview 起止 phase；
- sample 数；
- invalid/unknown/out-of-map 数；
- source revision 和 tube revision；
- raw/filtered 是否有效；
- obstacle certification 是否成立；
- 最小宽度和最小安全余量。

禁止在 generic global types header 中混入 neighbor、swarm 或 CBF 类型。

## 11. 固定 Tube

第一步只构造：

\[
-\delta_{\max}\le\delta\le\delta_{\max}.
\]

固定 tube：

- 不需要地图；
- 必须与完整 preview 中每个 sample 的 signed regularity bound 求交；
- 必须经过同一个 TubeFilter 和 PortProjector 接口；
- 必须明确标记 `obstacle_certified=false`；
- 只能证明 tube/port 机制，不能宣称静态障碍安全。

固定 tube 单元测试和 ROS 验收全部通过后，才能继续 ESDF tube。

## 12. ESDF Tube 构造

对 preview 中的每个 `PathDifferentialState`：

1. 使用 A2.2 已验证的重力参考水平法向
   \[
   N=\frac{e_z\times p_w}{\|e_z\times p_w\|};
   \]
2. 保持 `N.z=0`，不得投影或清零基础路径真实 z 导数；
3. 从 `p(w)` 沿 `+N` 和 `-N` 分别作射线采样；
4. 只接受已知、finite、ESDF 已计算的点；
5. 将未知、越界和占据作为保守边界，不得跳过后继续寻找更宽区域；
6. 与固定最大 offset 和 signed regularity bound 求交。

活动参考仍为：

\[
r=p+N\delta ,
\qquad r.z-p.z=0 .
\]

不得切换回固定高度 XY 假设。

## 13. 安全 Erosion

ESDF tube 的参考点 clearance 至少使用：

\[
d_{\mathrm{req}}
=r_{\mathrm{UAV}}
+m_{\mathrm{localization}}
+m_{\mathrm{tracking}}
+m_{\mathrm{map}}
+m_{\mathrm{extra}}
+m_{\mathrm{discretization}} .
\]

参数必须独立发布和记录，禁止合成一个来源不明的 magic margin。

要求：

- `r_UAV` 来自明确的仿真/机体保守半径；
- tracking bound 是配置的可验证上界；
- 每周期测量 `||x-r||`，超过 tracking bound 时 tube safety claim 失效并报告；
- 实际 UAV 位置还要直接查询 ESDF；
- 若当前 SDF 已基于 inflated occupancy，A5 仍可对该集合再使用完整 erosion，
  这会更保守；
- 禁止未经证明就扣除已有 map inflation；
- ray step、map resolution 和 phase sample step 的离散误差必须进入
  `m_discretization` 或等价保守检查。

旧 prototype 中 `0.25 m` UAV radius、`0.15 m` tracking bound 等数值只可作为
参考。必须结合当前 simulator、planner safe distance、map inflation 和实际 A4
tracking 数据给出依据；不得为了通过测试随意减小。

## 14. 未知、越界与 ESDF 未就绪

以下规则是硬约束：

- 基础点 `p(w)` 自身不是 `KNOWN_FREE` 或 clearance 不足时，该 sample invalid；
- preview 中任一必需 sample invalid 时，profile 不完整；
- 不得从统计中删除 invalid sample 后仍将 `candidate/profile complete` 置 true；
- 射线遇到未知或地图边界时，只能保留最后一个已认证的内侧位置；
- 若第一步射线就未知，可得到零宽或极窄的保守边界，但不能得到开放边界；
- ESDF source 未就绪时不得保留旧 profile 并继续声称安全；
- profile 不完整时 tube Marker 必须发布 DELETE；
- profile 不完整时不得启动非零 profile。

禁止复制旧 prototype 中“跳过 uncertified sample 后对剩余 sample 求 lookahead”
的行为。

## 15. Preview 范围

本机 ESDF 是局部地图，禁止要求一次认证远超本机地图的整条 semantic path。

ESDF tube 使用当前位置附近的 phase preview：

\[
[w_{\mathrm{current}}-w_{\mathrm{back}},
  \min(w_{\mathrm{current}}+w_{\mathrm{lookahead}},w_{\mathrm{end}})] .
\]

要求：

- preview 必须包含精确当前 phase；
- sample step 不大于 `0.10 w`；
- ray step 建议不大于 `0.05 m`；
- 前向 certified horizon 小于配置下限时不得开始非零 offset；
- adapter 负责从 `ContinuousPhasePath` 生成中立
  `PathDifferentialState` samples；
- manager 不得继续承担全路径采样循环。

## 16. TubeFilter

`TubeFilter` 必须输出安全保持的、分段 C1、斜率有界的 profile。

硬性质：

1. filtered lower 不得小于安全要求，即
   \[
   \underline\delta_{\mathrm{filtered}}
   \ge\underline\delta_{\mathrm{raw}};
   \]
2. filtered upper 不得大于 raw upper；
3. lower 始终不大于 upper；
4. knot 两侧 value 和 first derivative 连续；
5. `|lower_w|`、`|upper_w|` 不超过配置上限；
6. 窄区必须通过提前向前扩展的保守 envelope 形成 preview，而不是到边界才瞬间
   clip；
7. 对 raw profile 做 dense certification，任何 cubic overshoot 都必须向内收缩
   或判 invalid；
8. outward expansion 可以限速，inward safety contraction 不得被低通过滤成超出
   最新 raw safe set；
9. 若最新安全 contraction 已使当前 `delta` 位于 tube 外，禁止修改
   `delta` 伪装可行，必须报告 tube violation。

可使用 Lipschitz envelope、shape-preserving Hermite 或其他纯 C++14 解析方法。
禁止引入优化器、OSQP、CVX 或 QP solver。

## 17. Core Offset Constraint

`phase_offset_core/offset_constraint.h` 只定义 generic scalar envelope，
例如：

```cpp
struct OffsetConstraint {
  double lower = 0.0;
  double upper = 0.0;
  double lower_w = 0.0;
  double upper_w = 0.0;
  double invariant_gain = 0.0;
  double interior_margin = 0.0;
  bool enabled = false;
  bool valid = false;
};
```

core 不得 include navigation 的 TubeProfile，也不得出现 ESDF、地图、neighbor、
swarm 或 CBF 类型。

## 18. Tube 前向不变约束

定义：

\[
h^+=\overline\delta(w)-m_{\mathrm{tube}}-\delta ,
\]

\[
h^-=\delta-\underline\delta(w)-m_{\mathrm{tube}} .
\]

使用最终：

\[
\dot w=f_w^{\mathrm{ISF}}+u_w ,
\qquad
\dot\delta=u_\delta .
\]

则：

\[
\dot h^+
=\overline\delta_w(f_w^{\mathrm{ISF}}+u_w)-u_\delta ,
\]

\[
\dot h^-
=u_\delta-\underline\delta_w(f_w^{\mathrm{ISF}}+u_w) .
\]

PortProjector 至少约束：

\[
\dot h^+ + \alpha h^+ \ge 0 ,
\qquad
\dot h^- + \alpha h^- \ge 0 .
\]

这些是 `u_w` 与 `u_delta` 的联合线性约束。禁止先独立 clamp `u_w`、再独立
clamp `u_delta` 后声称满足 tube。

## 19. PortProjector 二维联合投影

保留 A4 全部约束：

- 输入 finite；
- amplitude；
- previous-final rate；
- phase 正向；
- 物理切向正裕度；
- signed regularity。

在其上加入 tube half-planes。

最终可行域是二维凸多边形。实现必须：

1. 解析构造所有 box/half-plane；
2. 对 raw port 做确定性的最近可行投影；
3. 可通过 polygon clipping、边界投影和顶点枚举实现；
4. raw port 已可行时保持不变；
5. 可行域为空时返回 invalid；
6. 不放宽 phase、tangent、regularity 或 tube margin；
7. 不引入 QP solver；
8. 输出每类 active/limited constraint 与 residual。

投影后必须再次计算：

- final `w_dot`；
- final tangent speed；
- next `delta`；
- next regularity；
- upper/lower invariant residual。

Runtime 还必须在精确 filtered profile 上查询 `w_next`，检查
`delta_next` 仍位于 next bounds。失败时 invalid，禁止只相信一阶局部近似。

当 OffsetConstraint disabled 时，A4 PortProjector 输出必须逐项回归等价。

## 20. PhaseOffsetRuntime

`PhaseOffsetRuntime` 是 ROS-free 单机状态门面。

它拥有：

- 当前 `delta`；
- previous final port；
- manual profile elapsed/start/complete/recenter 状态；
- fixed/ESDF tube profile；
- tube source revision 与 tube revision；
- TubeBuilder、TubeFilter；
- preflight 与 runtime failure diagnostics。

它不拥有：

- ROS publisher/subscriber/topic/time；
- `ContinuousPhasePath` 或 B-spline 对象；
- `SDFMap`；
- `ISFReferenceKernel`；
- manager、robot ID、neighbor、swarm、CBF；
- `PositionCommand` 或 SO3 command。

推荐采用两段式纯数据接口：

1. runtime 根据 neutral path samples、position、DistanceQuery 和内部 `delta`
   准备 geometry/tube/reference；
2. adapter 调用现有唯一 `ISFReferenceKernel` 得到 base guidance；
3. runtime 接收 base `v_cmd/w_dot` 与 raw intent，联合投影、MatchedPort、积分
   最终 `delta`，返回 StepOutput。

具体 API 可调整，但不得把 ISF 公式复制进 navigation，也不得在 adapter 中保留
第二套 delta 积分和端口历史。

## 21. 状态更新唯一性

Runtime 只允许在完整 step 成功后：

\[
\delta_{k+1}
=\delta_k+\Delta t\,u_{\delta,k}^{\star}.
\]

要求：

- 只由 final `u_delta` 更新；
- previous port 只保存 final port；
- projection、MatchedPort 和积分使用同一 final port；
- invalid step 不得部分提交；
- 不得 `delta=delta_ref`；
- 不得由 UAV 位置覆盖 `delta`；
- 不得在 path revision 时 reset `delta`；
- 不得在 tube 收缩时直接 clip `delta`。

## 22. 人工 Profile 迁移

A4 的平滑人工 profile 数学和状态迁入 navigation Runtime。

保留 A4 行为：

- 从 `delta=0`、port=0 开始；
- C1 或更高平滑左移、回中、右移、回中；
- 包含正负小幅 `u_w`；
- 最终 `delta_ref=0`、raw `u_w=0`；
- raw `u_delta` 可为 feedforward 加 tracking feedback；
- 不使用阶跃；
- 不使用 neighbor 或地图生成 intent。

adapter 只解析参数、转换路径/地图、调用 Kernel/Runtime 和发布 diagnostics。

## 23. Startup Gate

保留：

- `disabled`；
- `shadow`；
- `active`；
- `manual`。

不新增混乱的顶层控制模式。A5 在 `manual` 下增加：

```text
phase_offset/manual/tube_source = none | fixed | esdf
```

语义：

- `none`：A4 manual 回归，行为不变；
- `fixed`：A5 固定 tube；
- `esdf`：A5 obstacle-certified tube。

非零 profile 启动前必须同时满足：

- A3 零端口等价 gate 连续至少 100 周期；
- current geometry valid；
- A4 bilateral regularity preflight valid；
- tube profile complete；
- 当前 `delta` 位于 tube；
- ESDF 模式下 source ready、preview certified、tracking error 在 bound 内；
- profile 已稳定至少配置的连续 rebuild 数。

gate 前继续 legacy。gate 后不得周期性在 manual 与 legacy 间切换。

## 24. 路径更新

发现 semantic path identity 或定义域变化时：

1. 使旧 tube cache 失效；
2. 对新路径当前 preview 同步重建；
3. 保留 `w` 和 `delta`；
4. 不修改 C2；
5. 若新 tube 完整且包含当前 `delta`，允许 Runtime 继续；
6. 若不支持下一次 excursion，使用 final port 平滑回中；
7. 若当前 `delta` 已位于新认证 tube 外，锁存并报告 tube violation；
8. 不得 reset/clip `delta`；
9. 只记录 phase/delta/path switch 观察，不做 A6 连续性结论。

A5 ROS 验收中第 7 类事件必须为 0。

## 25. Adapter 与 Manager 限制

`PhaseOffsetMatchedAdapter` 仍是 ROS/路径/地图薄适配层：

- 从 `ContinuousPhasePath` 生成 preview samples；
- 把 `SDFMap` 包装成 DistanceQuery；
- 调用 A3 Kernel 和 navigation Runtime；
- 发布 diagnostics 与 Marker；
- 不保留 delta、previous port、profile 数学或 tube 算法。

硬性代码规模：

- `phase_offset_matched_adapter.cpp` 完成后不得超过 500 行；
- 每个 navigation implementation 文件建议低于 500 行。

`gvf_manager.cpp` 当前相对基线为 `+166`。A5：

- 不得超过 `+166`；
- 目标通过迁移参数解析/采样编排净减少到约 `+150` 或更低；
- callback 中不得出现 map ray、tube、filter、profile、projector、matched、
  delta 积分或 barrier 公式；
- manager 最多准备 position、path pointer、map pointer、gains、legacy、dt，
  调用 adapter，再交回原 governor。

不得修改原 phase 积分、endpoint clamp、governor、`PositionCommand` 发布函数。

## 26. 参数

只在 adapter/launch 解析，navigation 使用参数结构体。

至少包括：

```text
phase_offset/manual/tube_source
phase_offset/tube/fixed_delta_max
phase_offset/tube/sample_step_w
phase_offset/tube/lookahead_w
phase_offset/tube/back_w
phase_offset/tube/min_certified_forward_w
phase_offset/tube/ray_step
phase_offset/tube/max_offset
phase_offset/tube/boundary_slope_max
phase_offset/tube/invariant_gain
phase_offset/tube/interior_margin
phase_offset/tube/stable_rebuild_cycles
phase_offset/tube/esdf_unobserved_threshold
phase_offset/tube/uav_radius
phase_offset/tube/localization_margin
phase_offset/tube/tracking_error_bound
phase_offset/tube/map_margin
phase_offset/tube/extra_margin
phase_offset/tube/discretization_margin
```

固定 tube launch 和 ESDF tube launch 必须显式给出参数。

禁止修改：

- 最大速度；
- planner velocity/acceleration limits；
- `K1`、`K2`；
- governor saturation；
- C2 参数；
- map inflation；
- simulator dynamics。

不得靠速度或增益调参使 A5 通过。

## 27. Diagnostics

A3 active 的原 14 个 diagnostics 字段和值序必须完全不变。

A4 manual diagnostics 可在末尾追加 A5 字段，但原字段索引和语义不得改变。

A5 至少发布：

- tube source；
- source ready；
- raw/filtered/profile complete；
- obstacle certified；
- source/tube revision；
- sample/invalid/unknown/out-of-map/occupied count；
- preview start/end 与 certified forward horizon；
- raw/filtered current lower、upper、lower_w、upper_w；
- current/next `h^-`、`h^+`；
- upper/lower invariant residual；
- fixed/regularity/ESDF/erosion 各边界贡献；
- UAV radius 与各 margin；
- required reference clearance；
- current reference ESDF distance；
- current actual-UAV ESDF distance；
- tracking error norm 与 bound；
- current/next inside tube；
- tube-limited flags/reasons；
- rebuild/reject/violation counts；
- delta、raw/final port；
- matched residual；
- phase/tangent margins；
- `r.z-p.z`。

所有发布数值必须 finite。无效值通过独立 bool/count/reason 表达，不能用 NaN。

## 28. Marker

保留 A4 base path、active path 和 frame。

新增只读：

- filtered lower boundary LINE_STRIP；
- filtered upper boundary LINE_STRIP；
- 可选 raw boundary 辅助线；
- 当前 preview 区间状态。

规则：

- profile 不完整或任一边界不完整时，对相应既有 ID 发布 DELETE；
- 禁止发布残缺 LINE_STRIP；
- active path 仍保持真实 base height；
- visualization 不发布控制命令。

## 29. Core 单元测试

扩展 `port_projector_test.cpp`，至少覆盖：

1. tube disabled 时与 A4 结果逐项相同；
2. constant fixed lower/upper；
3. upper boundary active；
4. lower boundary active；
5. sloped upper 将 `u_w/u_delta` 联合约束；
6. sloped lower 联合约束；
7. raw feasible 时不变化；
8. amplitude、rate、positive phase、positive tangent、regularity、tube 同时成立；
9. 可行多边形为空返回 invalid；
10. projection 结果 deterministic；
11. invariant residual 非负；
12. input/constraint non-finite 拒绝；
13. 不通过放宽 margin 伪造可行。

## 30. TubeBuilder 单元测试

使用解析 DistanceQuery，不依赖 ROS 或真实地图。

至少覆盖：

1. fixed tube；
2. open area；
3. 对称走廊；
4. 非对称走廊；
5. 变高度基础路径；
6. `r.z-p.z=0`；
7. UAV radius 增大时 tube 单调缩小；
8. tracking margin 增大时 tube 单调缩小；
9. map/discretization margin；
10. obstacle hit；
11. OUT_OF_MAP；
12. UNKNOWN；
13. UNAVAILABLE；
14. sentinel 不得成为大净空；
15. invalid path sample；
16. 近竖直退化；
17. signed regularity intersection；
18. preview 中任一 invalid sample 使 complete=false；
19. 不跳过 uncertified sample；
20. current base point clearance 不足时拒绝。

## 31. TubeFilter 单元测试

至少覆盖：

1. constant profile 保持；
2. asymmetric profile；
3. 窄区提前收缩；
4. filtered 始终为 raw 的子集；
5. knot value 连续；
6. knot derivative 连续；
7. slope bound；
8. cubic 不 overshoot；
9. lower/upper 不交叉；
10. impossible profile invalid；
11. outward expansion rate limit；
12. inward contraction 不生成 unsafe overshoot；
13. query value/derivative finite；
14. exact next-step bounds 可查询。

## 32. Runtime 单元测试

至少覆盖：

1. tube source none 保持 A4 zero/manual 行为；
2. final `u_delta` 唯一积分 delta；
3. previous port 只保存 final port；
4. fixed tube 收缩；
5. ESDF tube 收缩；
6. 同一 final port 进入 MatchedPort；
7. matched residual 为数值精度；
8. tracking error 不超过 bound；
9. tracking bound 超限失效；
10. ESDF unavailable 时 profile 不启动；
11. source revision 变化使 cache 失效并重建；
12. revision 变化不 reset delta；
13. current delta outside new tube 报 violation，不 clip；
14. exact next-step tube check；
15. invalid step 不部分提交；
16. profile 平滑回零；
17. 端口 amplitude/rate 和正向裕度；
18. deterministic；
19. 无 neighbor/swarm/CBF 输入。

## 33. Adapter 回归测试

扩展 A4 adapter test：

- A3 active 原 14 字段不变；
- A4 `tube_source=none` 全部原测试通过；
- fixed tube Marker 和 diagnostics；
- ESDF query 五状态转换；
- sentinel 映射为 UNKNOWN；
- profile invalid 时 boundary/active Marker DELETE；
- runtime owns delta 与 previous port；
- adapter 不再自行积分 delta；
- 路径 preview 采样含当前 phase；
- source revision 变化；
- path change 不 reset delta；
- current tube violation 明确失败；
- `r.z-p.z=0`；
- matched residual；
- 无 control publisher。

## 34. CMake 与 package

新增 `phase_offset_navigation` 独立 library：

```text
libphase_offset_navigation
```

其测试只链接：

- `phase_offset_navigation`；
- `phase_offset_core`；
- Eigen/gtest。

`bspline_race` 的 matched adapter 链接：

- `phase_offset_navigation`；
- `phase_offset_core`；
- A3 Kernel/Active adapter；
- 既有 ROS/plan_env 依赖。

禁止把 navigation 源文件直接编入 `bspline_gvf`，禁止统一链接旧 prototype
模块。

## 35. 构建与测试顺序

按顺序执行：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
catkin_make -j8
catkin_make run_tests_phase_offset_core
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

必须明确记录：

- geometry；
- MatchedPort；
- 扩展 PortProjector；
- TubeBuilder；
- TubeFilter；
- Runtime；
- Shadow adapter；
- A3 Active；
- A4/A5 Matched adapter；
- ISFReferenceKernel；
- lifted guidance 与原 bspline_race 回归。

仅允许保留已知的 `uav_utils` 缺失 XML。出现其他失败时停止，不得修无关包。

## 36. ROS 隔离规则

先检查现有 ROS master 和用户进程。

- 不连接已有 master；
- 不停止用户进程；
- 使用新的隔离 master，例如 `11325`；
- 记录当前端口 owner；
- 只清理本轮启动的 roscore、simulator、planner、RViz/record 节点；
- 结束后确认隔离端口释放。

地图只使用原 `pillar.pcd`。禁止新增或切换 corridor、circle、figure-eight、
split/merge 等地图或场景。

## 37. ROS 验收顺序

必须依次完成：

1. disabled baseline；
2. A3 active zero-port；
3. A4 manual `tube_source=none`；
4. A5 fixed tube；
5. A5 ESDF tube。

前一项失败不得进入后一项。

每项均检查：

- `/position_cmd` 唯一发布者是 `/formation_planning`；
- Shadow/navigation/adapter 无控制 publisher；
- 原 planner、C2、governor、SO3 仍工作；
- 最终 UAV 到达；
- 无 EMERGENCY；
- 本轮进程可清理。

## 38. Fixed Tube ROS 验收

使用 `phase_offset_fixed_tube_single.launch`。

配置人工 amplitude 明显大于 fixed tube 内边界，但仍处于 A4 已验证的小幅范围，
用于产生真实 projection；不得增大速度或端口上限。

必须观察：

- zero gate 至少 100 周期后打开；
- fixed profile complete；
- obstacle_certified 明确为 false；
- 至少一次 tube-limited event；
- `delta` 始终在 fixed filtered bounds 内；
- current/next `h^-,h^+` 非负；
- invariant residual 不低于 `-1e-6`；
- final port amplitude/rate 满足配置；
- phase 与物理切向速度严格为正；
- matched residual 最大值不高于 `1e-10`；
- `r.z-p.z` 不高于 `1e-10 m`；
- profile 最终平滑回零；
- failure/fallback/tube violation 为 0；
- UAV 正常到达。

固定 tube 不能用于声明 obstacle safety。

## 39. ESDF 未就绪预检

在 ESDF active profile 前必须记录至少一个启动期诊断：

- source not ready 或 profile incomplete；
- selected nonzero=false；
- final port 为零；
- tube boundary Marker DELETE；
- 没有假 wide tube；
- ESDF ready 后才开始 stable rebuild 计数。

如果环境启动太快无法在 ROS 中稳定捕获，必须用 unit/integration test 明确覆盖，
并在汇报中说明。

## 40. ESDF Tube ROS 验收

使用 `phase_offset_esdf_tube_single.launch` 和原 `pillar.pcd`。

先复用 A4 已通过的点到点目标；如该路线没有产生可测 tube 变化，可在原地图内
选择另一个安全点到点目标，但不得修改地图、planner、C2、速度、增益或 safety
margin。

必须记录完整往返或至少两个独立目标，包含 replan/C2 观察。

验收：

- source ready 后才建立 obstacle-certified profile；
- 所有 active preview sample complete；
- unknown/out-of-map sample 没有被跳过；
- filtered bounds 是 raw safe bounds 的子集；
- tube width 随路径/障碍发生可测变化；
- 至少出现一次比 `max_offset` 更窄的 ESDF 边界；
- 至少一次 raw intent 因 tube 被联合投影；
- tube 收缩时 delta 通过 final `u_delta` 平滑回收；
- 不发生 delta reset 或 clip；
- `delta` 始终在 filtered bounds 内，容差不超过 `1e-4 m`；
- current/next invariant residual 不低于 `-1e-6`；
- reference ESDF distance 不低于 configured `d_req`，允许的数值容差必须小于
  map/ray discretization margin；
- actual UAV ESDF distance 不低于不含 tracking bound 的实体 clearance；
- `||x-r||` 不超过 configured tracking bound；
- `r.z-p.z` 不高于 `1e-10 m`；
- matched residual 最大值不高于 `1e-10`；
- final phase speed 和 physical tangent speed 严格为正；
- port amplitude/rate 全部满足；
- no profile invalid、no tube violation、no failure latch、no fallback；
- UAV 正常到达。

若原 `pillar.pcd` 中无法产生可测 tube narrowing，A5 ESDF 动态验收为
inconclusive，必须停止报告，禁止用新地图或减小 margin 伪造通过。

## 41. Replan/C2 只读记录

ROS 中记录：

- replan 次数；
- C2 accepted/rejected 次数；
- 每次日志中的 phase before/after；
- tube source revision；
- tube rebuild；
- delta before/after；
- current delta 是否位于新 tube；
- 是否出现 command spike。

但最终汇报必须写：

`A5 仅记录路径更新观察，未完成 A6 continuation 验收。`

不得把 A4 观察到的最大 phase 差 `0.040469` 删除、改写或解释为已修复。

## 42. 依赖边界审核

执行：

```bash
rg -n "ros|SDFMap|plan_env|ContinuousPhasePath|B[Ss]pline|Manager|Neighbor|Swarm|CBF|robot_id|PositionCommand|SO3" \
  src/swarm_planner/phase_offset/phase_offset_navigation

rg -n "ros|SDF|Map|TubeProfile|DistanceQuery|Neighbor|Swarm|CBF|robot_id" \
  src/swarm_planner/phase_offset/phase_offset_core

rg -n "Neighbor|Swarm|pairwise|inter_uav|common_msgs|CBF|OSQP|CVX|nlopt" \
  src/swarm_planner/phase_offset/phase_offset_navigation \
  src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/offset_constraint.h \
  src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp

rg -n "PositionCommand|cmd_pub|publishGovernorPositionCommand|SO3|uav_cmd" \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

rg -n "Continuation|continuation|phase remap|path epoch|C2 connector" \
  src/swarm_planner/phase_offset/phase_offset_navigation \
  src/swarm_planner/phase_offset/phase_offset_core \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

每个命中逐项解释。注释和合理类型名不自动算失败，但不得有真实禁止依赖。

## 43. 代码规模与公式唯一性审核

执行：

```bash
wc -l \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/*.cpp

git diff --numstat 9a0e975 -- \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp

rg -n "r_w.*u_w|N.*u_delta|matched_residual|delta.*dt|profile_elapsed|sin\\(|cos\\(" \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp \
  src/swarm_planner/phase_offset/phase_offset_navigation
```

要求：

- matched adapter implementation 不超过 500 行；
- manager 相对基线新增不超过 166 行，目标约 150 或更低；
- manager/adapter 无 delta 积分、profile 或 projector 公式；
- ISF guidance 仍只有 Kernel 一个 authoritative implementation；
- MatchedPort 公式仍只有 core 一个实现；
- Runtime 调用 MatchedPort，不复制 matched cancellation。

## 44. Git 与白名单自审核

执行并记录：

```bash
git diff --check
git status --short
git diff --stat
git diff --name-only
git stash list
git diff --numstat 9a0e975 -- \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp
```

逐项确认：

- branch/HEAD/stash 未变；
- 无 staged 内容；
- 无 commit/tag/push；
- A5 新增/修改只在 29 项白名单内；
- A2–A4 既有 tracked diff 保留；
- 旧 untracked prototype 未修改、删除、移动、rename 或接入；
- `test_gvf.launch` 中用户保留的 circle settings 仍为 false；
- `git diff --check` 通过。

## 45. 严格禁止

1. 自动进入 A6；
2. neighbor、swarm、robot ID 或多机消息；
3. `phase_offset_msgs`、`phase_offset_swarm`；
4. 修改 `common_msgs`；
5. 机间 CBF 或 pairwise safety；
6. obstacle QP/OSQP/CVX；
7. 新 C2 connector；
8. continuation checker；
9. phase remap/reset；
10. delta reset/clip；
11. 修改 planner、A*、B-spline、C2、SDFMap、governor、SO3、simulator；
12. 修改最大速度、增益或 saturation；
13. 新地图、corridor、circle、figure-eight、split/merge；
14. 把 UNKNOWN/OUT_OF_MAP/sentinel 当自由；
15. 跳过 invalid sample 后声称 profile complete；
16. 仅保证参考点、不检查实际 UAV tracking erosion；
17. adapter 保存具体 `SDFMap` 到 navigation Runtime；
18. navigation include ROS/plan_env；
19. core include navigation tube 类型；
20. 复制 ISF 或 MatchedPort 公式；
21. 恢复或 pop prototype stash；
22. 修改旧 prototype 文件；
23. 创建 commit、branch、tag 或 push；
24. 将 A5 验收结果解释为 A6 continuation 已完成。

## 46. 停止条件

出现以下任一情况立即停止并报告：

- branch、HEAD、stash 或 A4 baseline diff 不符；
- 需要修改白名单外文件；
- fixed tube 无法通过；
- DistanceQuery 无法可靠区分 sentinel/unknown；
- ESDF source 未就绪却只能靠假设继续；
- erosion 后原路径中心本身无法认证；
- tracking error 超过配置 bound；
- current delta 落在新 tube 外；
- PortProjector 联合可行域为空；
- tube narrowing 需要修改速度、增益、map 或 planner 才能通过；
- A5 需要 continuation/C2 修改；
- ROS runtime 不可用；
- workspace-wide test 仅因新的无关历史缺陷失败。

保留 worktree，不越权修复。

## 47. 最终汇报格式

最终汇报必须包含：

1. branch、HEAD、stash；
2. A5 实际修改/新增文件；
3. navigation 依赖边界；
4. DistanceQuery 五状态和 SDF sentinel 适配；
5. fixed tube 结果；
6. ESDF raw/filter/profile 结果；
7. erosion 各参数和依据；
8. PortProjector 联合约束与 invariant residual；
9. Runtime 状态迁移和 delta 更新唯一性；
10. matched residual；
11. phase/tangent 正裕度；
12. tracking error 与 actual/reference ESDF clearance；
13. tube narrowing 和平滑回收；
14. unknown/out-of-map/profile completeness；
15. Marker DELETE；
16. A3/A4 回归；
17. build/test 总数与已知 `uav_utils` 例外；
18. ROS disabled/active/manual/fixed/esdf 五组结果；
19. `/position_cmd` publisher；
20. replan/C2 只读观察；
21. manager 和 adapter 行数；
22. dependency search、`git diff --check`、白名单、stash；
23. 本轮进程清理；
24. 明确写出：
    `A5 仅记录路径更新观察，未完成 A6 continuation 验收。`
25. 最后一行明确写出：
    `A5 完成后已停止，未进入 A6。`
