# PhaseOffsetSwarm 代码架构设计

> 日期：2026-08-08  
> 目标：保护原 ISF-GVF，先完成单机公共内核，再通过单向接口扩展分布式集群。

## 1. 总体结论

需要新建目录和独立 library，不能继续把 phase-offset、tube、邻居、CBF 和可视化全部堆进 `gvf_manager.cpp` 或同一个 `bspline_gvf` 库。

推荐结构为：

```text
纯数学核心
    ↓
单机导航扩展
    ↓
原 ISF-GVF 薄适配层
    ↓
后续独立集群包
```

当前只创建单机需要的包。消息和集群包等单机 A7 通过以后再创建。

## 2. 当前结构存在的问题

原始代码已经较大：

```text
gvf_manager.cpp：约 5454 行
gvf_manager.h：约 1077 行
```

DeepSeek 修改后进一步增至：

```text
gvf_manager.cpp：约 6601 行
gvf_manager.h：约 1257 行
```

此外还存在以下耦合：

1. `phase_offset_types.h` 同时定义几何、tube、邻居、集群意图和控制模式；
2. 纯几何类型直接依赖 `ros::Time`；
3. neighbor header 同时依赖 ROS 消息、SDF map 和控制算法；
4. tube builder 直接持有具体 `SDFMap`；
5. geometry、allocator、CBF、tube 和 neighbor 全部编入 `bspline_gvf`；
6. manager 同时承担规划、phase、tube、通信、QP、CBF、可视化和调试；
7. 单机编译也被迫依赖多机消息；
8. 任何多机修改都可能破坏原单机。

因此，单纯把几个 `.cpp` 放进新文件夹还不够，必须同时划分 package、library 和依赖方向。

## 3. 推荐目录树

在 `src/swarm_planner` 下新建逻辑目录：

```text
src/swarm_planner/phase_offset/
├── phase_offset_core/          # A1 创建：纯数学、无 ROS
├── phase_offset_navigation/    # A5 创建：单机 tube 与 runtime
├── phase_offset_msgs/          # B2 创建：多机消息
└── phase_offset_swarm/         # B2 创建：邻居、集群意图与安全
```

`phase_offset/` 本身不放 `package.xml`；下面每个子目录是独立 catkin package。

阶段 A1–A4 只创建 `phase_offset_core`。阶段 A5 才创建 `phase_offset_navigation`。多机两个包不提前创建和编译。

## 4. phase_offset_core：纯数学内核

### 4.1 目录

```text
phase_offset_core/
├── CMakeLists.txt
├── package.xml
├── include/phase_offset_core/
│   ├── path_state.h
│   ├── geometry_types.h
│   ├── geometry.h
│   ├── port_types.h
│   ├── matched_port.h
│   ├── port_projector.h
│   └── continuation.h
├── src/
│   ├── geometry.cpp
│   ├── matched_port.cpp
│   ├── port_projector.cpp
│   └── continuation.cpp
└── test/
    ├── geometry_test.cpp
    ├── matched_port_test.cpp
    ├── port_projector_test.cpp
    └── continuation_test.cpp
```

### 4.2 依赖限制

只允许依赖 Eigen 和 C++ 标准库。禁止包含：

- ROS header 和 `ros::Time`；
- `common_msgs` 或其他消息；
- `SDFMap`；
- `gvf_manager`；
- `ContinuousPhasePath`；
- Marker、topic 和 robot ID；
- neighbor、swarm、CBF 概念。

### 4.3 中立路径输入

核心层不认识 B 样条，只接受中立微分路径状态：

```cpp
struct PathDifferentialState {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ww = Eigen::Vector3d::Zero();
  double w = 0.0;
  bool valid = false;
};
```

原工程适配层负责将 `ContinuousPhasePathState` 转换为该结构，避免 core 反向依赖 `bspline_race`。

### 4.4 类型拆分

不要继续保留一个混合的 `phase_offset_types.h`。至少拆成：

```text
path_state.h       基础路径微分状态
geometry_types.h   T、N、N_w、kappa、r、r_w、误差
port_types.h       u_w、u_delta、投影和约束结果
```

### 4.5 类职责

`PhaseOffsetGeometry` 只负责：

- 计算重力参考的 \(T,N,N_w,\kappa_{xy}\)；
- 计算 \(r=p+N\delta\)；
- 计算 \(r_w=p_w+N_w\delta\)；
- 计算相对活动参考的误差；
- 检查完整切向、水平切向非退化和
  \(1-\kappa_{xy}\delta\ge\mu\)。

A2.2 后，基础路径允许完整三维高度变化，但标量 \(\delta\) 仍只沿重力参考
水平法向作用。固定高度路径时，上述公式严格退化为
\(r_w=(1-\kappa\delta)p_w\)。完整 Bishop 法平面和二维 offset 不属于当前
架构阶段。

`MatchedPort` 只负责：

- 使用最终实际执行的 \(u_w,u_\delta\)；
- 计算 \(r_wu_w+Nu_\delta\)；
- 组合 \(\dot x,\dot w,\dot\delta\)；
- 计算 matched residual。

`PortProjector` 只负责单机端口可行域：

- phase 正向；
- 物理切向非零；
- 端口限幅和变化率；
- 固定 tube 边界；
- 曲率正则性。

它不包含机间 CBF。

`ContinuationChecker` 只比较路径更新前后的 \(p,p_w,p_{ww},r,r_w\)，不管理 path epoch，也不修改原 C2 connector。

## 5. phase_offset_navigation：单机 tube 与 runtime

### 5.1 目录

```text
phase_offset_navigation/
├── CMakeLists.txt
├── package.xml
├── include/phase_offset_navigation/
│   ├── distance_query.h
│   ├── tube_types.h
│   ├── tube_builder.h
│   ├── tube_filter.h
│   └── phase_offset_runtime.h
├── src/
│   ├── tube_builder.cpp
│   ├── tube_filter.cpp
│   └── phase_offset_runtime.cpp
└── test/
    ├── tube_builder_test.cpp
    ├── tube_filter_test.cpp
    └── runtime_test.cpp
```

该包依赖 `phase_offset_core`，但不依赖 swarm、邻居消息和机间 CBF。

### 5.2 地图查询解耦

Tube builder 不直接保存 `SDFMap*`，使用抽象距离函数：

```cpp
using DistanceQuery =
    std::function<double(const Eigen::Vector3d&)>;
```

ROS 适配层用 lambda 绑定 `sdf_map->getDistance(p)`；单元测试使用解析圆、矩形或走廊距离。这样 tube 算法可以脱离 ROS 和真实地图测试。

### 5.3 Runtime 门面

`PhaseOffsetRuntime` 组合：

```text
Geometry
MatchedPort
PortProjector
TubeBuilder / TubeFilter
ContinuationChecker
```

建议接口：

```cpp
class PhaseOffsetRuntime {
 public:
  void configure(const RuntimeParams& params);
  void reset(double w, double delta);
  ShadowOutput evaluateShadow(const StepInput& input) const;
  StepOutput step(const StepInput& input);
  ContinuationResult checkPathUpdate(
      const PathUpdateInput& input) const;
};
```

Runtime 保存 \(\delta\)、上一周期端口、tube profile、mode 和 continuation 诊断。Runtime 不保存 publisher、subscriber、topic、B 样条对象、邻居和 SO3 command。

## 6. 原 bspline_traj 只增加薄适配层

建议在原包中只新增：

```text
bspline_traj/
├── include/bspline_race/guidance/
│   └── isf_reference_kernel.h
├── include/bspline_race/integration/
│   └── phase_offset_adapter.h
├── src/guidance/
│   └── isf_reference_kernel.cpp
└── src/integration/
    └── phase_offset_adapter.cpp
```

### 6.1 ISFReferenceKernel

原始 ISF 公式不能在 `gvf.cpp` 和新模块中各写一份。抽取一个对任意参考几何求值的通用入口：

```cpp
struct ReferenceGeometry {
  Eigen::Vector3d point;
  Eigen::Vector3d tangent;
  double derivative_norm = 0.0;
};

LiftedGuidance evaluateIsfGuidance(
    const Eigen::Vector3d& position,
    const ReferenceGeometry& reference,
    const IsfGains& gains);
```

原路径模式传入 \(p,T,\|p_w\|\)，phase-offset 模式传入 \(r,T,\|r_w\|\)。这样 ISF 公式只有一个实现源。

抽取之前先为现有 `calcLiftedGuidanceAtPhase` 建立回归测试；抽取过程只做机械迁移，不同时修改公式。

### 6.2 PhaseOffsetAdapter

Adapter 只做转换和调用：

```text
ContinuousPhasePathState
        ↓
PathDifferentialState
        ↓
PhaseOffsetRuntime
        ↓
ReferenceGeometry / matched guidance
        ↓
原 command governor
```

Adapter 不实现几何、tube、QP、邻居和 CBF。

### 6.3 manager 修改上限

`gvf_manager` 只允许：

1. 读取参数并构造 adapter；
2. 在 command callback 中准备 `StepInput`；
3. 调用 shadow 或 active；
4. 把输出交给原 governor；
5. 发布少量调试信息；
6. path update 时调用 continuation check。

建议将 phase-offset 对 `gvf_manager.cpp` 的新增控制在约 150 行以内。所有 \(\delta\)、端口历史、tube 和约束状态封装进 Runtime。

## 7. B2 后再建 phase_offset_msgs

目录建议：

```text
phase_offset_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── AgentState.msg
    ├── PathEvent.msg
    ├── ConflictState.msg
    └── DebugState.msg
```

不要继续修改原 `common_msgs`，避免原单机因为新论文实验增加不必要的消息依赖。

## 8. B2 后再建 phase_offset_swarm

```text
phase_offset_swarm/
├── CMakeLists.txt
├── package.xml
├── include/phase_offset_swarm/
│   ├── neighbor_types.h
│   ├── neighbor_buffer.h
│   ├── neighbor_selector.h
│   ├── elastic_intent.h
│   ├── pairwise_safety_filter.h
│   └── swarm_coordinator.h
├── src/
│   ├── neighbor_buffer.cpp
│   ├── neighbor_selector.cpp
│   ├── elastic_intent.cpp
│   ├── pairwise_safety_filter.cpp
│   ├── swarm_coordinator.cpp
│   ├── swarm_coordinator_node.cpp
│   └── swarm_visualizer_node.cpp
├── config/
├── launch/
└── test/
```

Swarm 包只输出本机世界坐标协调意图

\[
g_i^{\mathrm{swarm}},
\]

不能直接发布最终 `PositionCommand`。单机 Runtime 继续负责意图分解、端口投影和 matched 注入。

可视化节点必须只读；关闭它后飞行结果必须不变。

## 9. 依赖方向

```text
Eigen / STL
    ↓
phase_offset_core
    ↓
phase_offset_navigation
    ↓
bspline_race integration adapter
    ↓
formation_planning / governor / SO3

phase_offset_msgs
    ↓
phase_offset_swarm
    ↓  输出 g_swarm
bspline_race integration adapter
```

禁止出现：

```text
core -> ROS messages
core -> SDFMap
core -> gvf_manager
navigation -> swarm
swarm -> gvf_manager 内部状态
visualizer -> 控制输出
```

## 10. CMake target 划分

建议目标：

```text
phase_offset_core:
  libphase_offset_core

phase_offset_navigation:
  libphase_offset_navigation

bspline_race:
  libbspline_gvf
  libphase_offset_isf_adapter
  formation_planning

phase_offset_swarm:
  libphase_offset_swarm_core
  phase_offset_swarm_coordinator_node
  phase_offset_swarm_visualizer_node
```

测试只链接最小目标。例如 geometry test 只链接 `phase_offset_core`，不能再统一链接完整 `bspline_gvf`。

## 11. 每周期数据流

单机阶段：

```text
gvf_manager 获得 odom 和 ContinuousPhasePathState
        ↓
adapter 转换 PathDifferentialState
        ↓
runtime 计算 geometry 和 tube
        ↓
runtime 读取零输入或人工 external intent
        ↓
PortProjector 生成最终 u_w、u_delta
        ↓
ISFReferenceKernel 计算相对 r 的基础 ISF
        ↓
MatchedPort 加入 r_w u_w + N u_delta
        ↓
adapter 返回 guidance output
        ↓
原 governor 和 SO3
```

多机阶段只把“零输入或人工输入”替换为 `phase_offset_swarm` 输出的 \(g_i^{\mathrm{swarm}}\)，其余链路不改变。

## 12. DeepSeek 文件迁移表

| 当前文件 | 新架构处理 |
|---|---|
| `phase_offset_types.h` | 不直接复用；拆成 core、tube、swarm 类型 |
| `phase_offset_geometry.*` | 去除 ROS 和 ContinuousPhasePath 依赖后迁入 core |
| `phase_offset_allocator.*` | 去除 swarm/CBF 语义，改为单机 PortProjector |
| `phase_offset_cbf_constraints.*` | A 阶段不接；B4 迁入 swarm safety |
| `path_tube_builder.*` | 用 DistanceQuery 解耦 SDFMap 后迁入 navigation |
| `swarm_neighbor_model.*` | A 阶段不接；B2 拆成 buffer、selector、intent |
| `phase_offset_swarm_visualizer.cpp` | B2 后改为独立只读节点 |
| 四个新消息 | 不进入 common_msgs；B2 迁到 phase_offset_msgs |
| manager 中新增的 1169 行 | 不整体复用；按 adapter 接口重新接入 |
| 当前 CMake 大改 | 不整体复用；按阶段逐个 target 加入 |

旧代码只作为公式和测试参考，不能整批接回。

## 13. 分阶段创建顺序

- A0：不创建新包，只恢复原单机；
- A1：创建 `phase_offset_core`，只做 geometry；
- A2：在原包增加 shadow adapter；
- A2.1：量化真实路径高度变化和无效 Marker；
- A2.2：推广为变高度基础路径的重力参考 2.5D geometry，并重验 shadow；
- A3：抽取 `ISFReferenceKernel`，完成零端口等价；
- A4：在 core 加入 MatchedPort 和 PortProjector；
- A5：创建 `phase_offset_navigation`，加入固定 tube 和 ESDF tube；
- A6：加入 continuation checker，复用原 C2 connector；
- A7：完成原地图单机验收；
- B0–B1：增加隔离式 multi simulator 和多实例运行；
- B2：创建 `phase_offset_msgs` 和 `phase_offset_swarm`，先 shadow；
- B3–B4：依次开启集群意图和机间安全过滤。

## 14. 维护规则

1. 一个头文件只表达一类概念；
2. 单个算法 `.cpp` 建议不超过约 500 行；
3. ROS callback 不写数学公式，只转换消息和调用接口；
4. 算法层不保存 topic 名；
5. 参数解析集中在 adapter/node；
6. 每个 package 有 README，写清输入、输出和依赖；
7. 每阶段只增加必要的 CMake target；
8. 单元测试链接最小库；
9. 控制模式使用一个明确 enum，不组合大量布尔开关；
10. 单机 core 不出现 swarm、neighbor 和 robot_id；
11. 多机包不拥有 \(w,\delta\) 的权威状态；
12. 最终实际端口只由单机 Runtime 决定。

## 15. 最终建议

需要开新目录，而且应该通过 package 和 library 边界解决维护问题，而不是只给当前 `.cpp` 分类。

最重要的边界是：

\[
\boxed{
\text{数学核心不依赖 ROS},
\quad
\text{原 manager 只做适配},
\quad
\text{集群层只输出意图}.
}

按照该结构，单机 A1–A7 调通后，多机扩展只增加 \(g_i^{\mathrm{swarm}}\) 的来源，不再修改 matched port 和 ISF-GVF 主体。
