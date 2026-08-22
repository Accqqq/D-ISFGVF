# Planner-Authoritative Zero-Connected Tube Simplification

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行授权：用户已明确要求由主代理调度并由 `gpt-5.6-terra / xhigh` 实现  
`AUTO_ADVANCE=true`，但只限本文件定义的 Z1 阶段；不得提前进入 H2/authority/gradient 重构。

## 1. 阶段目标

把当前 Tube 从“重新认证并可能否决 planner 中心线”的独立安全系统，收缩为“围绕 planner 最终 C2 路径计算额外横向偏移空间”的模块。

本阶段必须建立以下产品语义：

```text
planner 最终 C2 路径 p(w) = 权威可执行中心线

TubeBuilder:
  从 delta=0 向左右扩展
  只保留包含 0 的连通区间
  非零空间无法证明时退化为 [0,0]

neutral / single-UAV:
  delta=0
  planner/C2 安装不等待 OffsetTubeCertificate

active nonzero:
  只有非零 offset 区域获得证书后才允许执行
```

本阶段不是取消 Tube。Tube 仍负责：

- 计算每个 `w` 上可用的左右 offset；
- 保持曲率 regularity；
- 生成连续 exact-PWL 边界；
- 对非零 swept ribbon 做连续障碍验证；
- 供 Runtime 投影 `delta` 和 `u_delta`。

## 2. 不可改变的基线

严禁修改或规避：

- Kino A*；
- B-spline optimizer 的规划逻辑；
- 原有五次 Hermite C2 connector；
- global semantic `w`；
- planner 重规划触发和候选接受策略；
- Governor、SO3、原始单机控制链；
- `test_gvf.launch` 中用户保留的 circle-test 开关；
- active nonzero 正常交接的 `w+=w-`、`delta+=delta-` 理论要求。

不得通过修改速度、planner 限制、地图、控制增益或放宽碰撞检测让测试通过。

## 3. 当前问题与代码证据

当前生产链存在四个与目标语义冲突的行为：

1. `TubeCrossSectionSolver` 扫描整个 `[-search_extent,+search_extent]`，枚举所有安全分量，再用 `preferred_delta` 选择分量；
2. Tube 使用独立的 UAV/map/localization/tracking 裕度组合，而 planner 使用 `planning/safe_distance`；
3. `continuous_inset` 可把一个原本包含零的窄区间收缩到零的同一侧，从而排除 `delta=0`；
4. Filter/SurfaceValidator/epoch 安装把“没有非零 offset 证书”升级为 Candidate 失败，而不是正常的零宽退化。

相关位置：

- `phase_offset_navigation/src/tube_cross_section.cpp`
- `phase_offset_navigation/src/tube_builder.cpp`
- `phase_offset_navigation/src/tube_filter.cpp`
- `phase_offset_navigation/src/tube_surface_validator.cpp`
- `phase_offset_navigation/src/tube_epoch_manager.cpp`
- `bspline_traj/src/integration/phase_offset_matched_adapter.cpp`

## 4. 强制设计决定

### Z1.1 Planner clearance 是生产 Tube 的唯一 clearance 尺寸

ESDF/cloud Tube 的生产横截面必须读取同一个 ROS 参数：

```text
planning/safe_distance
```

并在与 planner 相同膨胀 backing 的 immutable snapshot 上使用该 required clearance。

不得在生产横截面上继续独立求和：

```text
uav_radius
+ map_uncertainty
+ localization_uncertainty
+ tracking_error_bound
```

旧字段可在迁移期只保留为兼容/诊断输入，但不得再决定生产 Tube 几何或安装结果。若保留，必须在注释和测试中明确 deprecated/non-authoritative。

### Z1.2 只构造 zero-connected component

删除生产几何中的：

- `TubeCrossSectionInput::preferred_delta`；
- `contains_preferred_delta`；
- 完整 disconnected `components` 输出；
- `DistanceToComponent`；
- preferred component selection；
- Builder 中只为 preferred component 服务的 overload/字段/诊断。

新横截面算法：

1. `delta=0` 是 planner-authoritative baseline；
2. 查询零点只用于判断是否允许产生非零 offset，以及记录地图变化；它不拥有 planner 路径安装否决权；
3. 若零点无法由 Tube snapshot 证明，横截面返回 zero-only `[0,0]`，不得搜索或选择断开的非零分量；
4. 若零点可用，从零分别沿 `+N`、`-N` 找第一个边界；
5. 与 curvature regularity 区间相交；
6. 最终必须满足 `lower <= 0 <= upper`；
7. 任一侧失败只把该侧收缩到零；两侧失败得到 `[0,0]`。

初版可保留安全 bracket + 二分边界细化。immutable-snapshot gradient 不属于本阶段。

### Z1.3 显式区分 zero-only 与 offset-certified

产品状态必须有一个简单、稳定的内部分类，名称可按现有类型风格调整，但语义只能是：

```text
ZERO_ONLY_PLANNER_BASELINE
OFFSET_CERTIFIED
```

要求：

- `ZERO_ONLY_PLANNER_BASELINE` 是有效退化结果，不是 fatal、certificate denial 或 candidate incomplete；
- 它可用于 candidate 诊断和 RViz 表示；
- 它不得激活非零 offset authority；
- 它不得使 `requiresPathTubePairBootstrap()` 或 authoritative handoff 捕获 neutral planner；
- `OFFSET_CERTIFIED` 才能授权 manual/swarm 非零 offset。

不要再增加大量布尔门控。优先使用一个枚举/分类，并逐步映射旧诊断字段。

### Z1.4 inset 和 Filter 必须保持零

对包含零的区间做 inset 时：

```cpp
lower = std::min(0.0, lower + inset);
upper = std::max(0.0, upper - inset);
```

或使用数学等价且更清晰的实现。

要求：

- inset 最差把区间收缩为 `[0,0]`；
- 不允许 inset 把零推到区间外；
- 所有 raw knots 包含零时，exact-PWL Filter 必须保留零；
- exact-PWL Filter 可以保留，不要重新引入 cubic/C1 smoothing。

### Z1.5 SurfaceValidator 只决定非零容量

SurfaceValidator 或其调用层必须区分：

```text
非零 ribbon 连续证明成功 -> OFFSET_CERTIFIED
证明失败/UNKNOWN/OOM/limit -> ZERO_ONLY_PLANNER_BASELINE
```

不得因为非零 ribbon 证明失败而让 planner 中心线 Candidate 整体无效。

可以在本阶段采用最小实现：一旦非零 ribbon 连续验证失败，就把整个候选收缩为全 preview `[0,0]`，而不是尝试复杂的逐 cell 最大宽度修复。安全性优先于保留非零宽度，简单性优先于利用率。

### Z1.6 Epoch/Runtime/adapter 语义

`TubeEpochManager` 必须做到：

- zero-only 是正常候选状态；
- zero-only 不安装为非零 OffsetAuthority；
- retained `delta=0` 时不触发 `BASE_CENTERLINE_*` 等等待门控；
- 旧 `BASE_CENTERLINE_CLEARANCE_INSUFFICIENT`、`BASE_CENTERLINE_INDETERMINATE`、`BASE_CENTERLINE_CONTINUITY_UNCERTIFIED` 不再拥有 planner path veto 语义；可删除或降级为观察诊断；
- active nonzero 若最新候选只能 zero-only，保留旧 authority 只为后续受证回零；本阶段不得瞬时 reset `delta`。

`PhaseOffsetMatchedAdapter` 必须做到：

- observe-only 仍构建并发布 Tube；
- neutral `delta=0` 不因 zero-only 候选进入 bootstrap/handoff；
- 只有具有非零容量的 certified profile 才能触发首次 offset activation；
- 不修改现有 direct active-nonzero continuation/H2 实现，本阶段只保证它不会被 zero-only 候选误激活。

## 5. Proposal 与 handoff 同步

修改：

- `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`
- `/home/cxq/ISF-GVF/handoff.md`

Proposal 只修改与职责边界冲突的内容：

- §21：Tube 不再重新否决 neutral planner；无非零证书时使用 zero-only；
- §22.5：Tube 输入 planner 最终 C2 路径并使用同一 planner clearance，只计算额外 offset；
- §22.7：Tube feasibility 只约束 active nonzero continuation；neutral connector 安装不等待 Tube；
- §28.1：正常 Tube 只保留 zero-connected component，断开的非零分量忽略；
- 命题 5 保留，但明确适用于 active nonzero 正常交接；neutral replan 不需要 OffsetTubeCertificate。

不得改写 proposal 的 matched port、ISF-GVF、C2、global phase、swarm organization 或 CBF 主体。

## 6. 当前阶段文件白名单

除非编译错误由本阶段 API 改动直接引起，否则只允许修改：

```text
/home/cxq/ISF-GVF/handoff.md
/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md

docs/Codex_Planner_Authoritative_Zero_Tube_Simplification_Execution_Spec_2026-08-20.md
docs/Codex_Planner_Authoritative_Zero_Tube_Simplification_Self_Audit_2026-08-20.md

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp
```

明确禁止本阶段修改：

```text
gvf_manager.cpp / gvf_manager.h
continuous_phase_path.*
Kino A* / B-spline planner
launch 参数数值
cloud_occupancy_snapshot.*
phase_offset_runtime.*
pin registry / pending-completed mailbox / H2 seam orchestration
```

若必须修改禁止文件才能完成 Z1，停止并报告，不得扩大白名单。

## 7. 必须新增或改写的测试

### Cross-section

1. planner zero 可用、两侧自由：返回包含零的双侧区间；
2. 单侧立即失败：返回 `[0,+b]` 或 `[-a,0]`；
3. 两侧立即失败或零点 snapshot 无法证明：返回 zero-only `[0,0]`；
4. 存在断开的非零安全分量：忽略该分量；
5. 任意 retained/preferred delta 不改变 Tube 几何（production API 中应已删除）。

### Builder / Filter / Validator

1. `continuous_inset` 大于某侧宽度时仍包含零；
2. 所有 raw knots 含零时 Filter 后全程含零；
3. nonzero ribbon 的 between-knot obstacle/UNKNOWN/limit 使结果退化 zero-only，而不是整个 Candidate 失败；
4. zero-only profile 的边界与斜率均为零且数值有限。

### Epoch / Adapter

1. zero-only 是正常候选，不是 fatal/certificate denial；
2. zero-only 不产生 active nonzero profile/authority；
3. observe-only 继续发布 zero-only candidate；
4. neutral Runtime 不触发 `requiresPathTubePairBootstrap()`；
5. 真正有非零 certified width 时原有 bootstrap 行为仍有效；
6. active nonzero 的 retained state 语义不被删除或瞬时清零。

### 旧测试语义反转

至少审查并改写：

- `CloudOneSidedCandidateContainingRetainedDeltaCannotRescueCenterline`
- `InsetExcludedZeroWithoutContinuousCertificateWaitsFailClosed`
- `CloudExcludingZeroCandidateDoesNotInstallOrDenyCertificate`
- preferred-component 相关 CrossSection/Builder/Audit 测试。

## 8. 验收标准

必须全部满足：

1. production Tube 几何不再由独立 robust margin 求和决定；
2. production CrossSection API 中无 `preferred_delta` 和 disconnected components selection；
3. 任意有效 planner preview 生成的候选每个 knot 满足 `lower <= 0 <= upper`；
4. 不能证明非零空间时得到明确 zero-only，而不是 candidate failure；
5. zero-only 不激活非零 offset authority；
6. exact-PWL Filter 与 between-knot 非零安全验证仍存在；
7. 原有 planner/C2/global-w/Runtime retained-delta 代码未修改；
8. proposal、handoff、代码语义一致；
9. 定向单元测试全部通过；
10. `phase_offset_matched_adapter_test` 与已完成阶段回归通过；
11. `catkin_make -j2` 或等价 workspace build 通过；
12. `git diff --check` 通过；
13. 自审记录白名单、测试结果、未解决的 active-nonzero H2 liveness 后续工作。

## 9. 本阶段明确不解决的问题

完成 Z1 后仍可能存在 active nonzero direct handoff 的生命周期复杂度。本阶段不得顺手修改。

后续独立阶段应处理：

```text
single deterministic seam
+ async successor build
+ handoff deadline
+ certified recenter-to-zero fallback
+ atomic ExecutionAuthority
+ removal of pin/double mailbox
```

immutable snapshot gradient 和 Builder/Validator 合并也属于后续阶段。

## 10. 停止条件

出现以下情况立即停止并向主代理报告：

- 需要修改白名单外产品文件；
- 发现 planner/C2 本身不能保证当前中心线语义；
- zero-only 会在现有 Runtime 中意外激活非零 authority 且无法在白名单内修正；
- 测试要求改变 planner 参数、速度或安全阈值；
- 工作区出现与本阶段重叠的用户新改动。

