# M3-L1 CertifiedTubeBuilder Pipeline Consolidation

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行授权：用户已明确要求由主代理负责调度和写计划，由
`gpt-5.6-terra / xhigh` 实现，并在完成后执行静态与 launch 动态测试。  
`AUTO_ADVANCE=true`，但只限本文件定义的 M3-L1；不得提前进入 M3-L2
旧实现删除、M4 atomic authority、M6 recenter 或 M7 pin/mailbox 删除。

## 1. 阶段目标

当前 production candidate 构建在 `TubeEpochManager::update()` 中手工串联：

```text
TubeBuilder raw/adaptive knots
-> attach immutable map provenance
-> TubeFilter exact-PWL inner envelope
-> zero-only classification/collapse
-> TubeSurfaceValidator continuous swept-ribbon proof
-> validation failure collapse to planner zero-only
-> final classification
```

这使 EpochManager 同时承担 epoch/installation policy 与构建证明流水线，且
`HasNonzeroCapacity()`、`CollapseToPlannerZeroBaseline()` 和失败诊断归并散落在
manager source 中。

M3-L1 新增一个纯 C++ `CertifiedTubeBuilder`，成为 production candidate 的唯一
构建入口。它组合并复用现有 `TubeBuilder`、`TubeFilter`、
`TubeSurfaceValidator`，一次返回完整 immutable candidate 及分层结果。

```text
TubeEpochManager
-> CertifiedTubeBuilder::build(input, result)  // one call
-> current-state/install/epoch policy
```

本阶段是证明保持的 ownership 收敛，不重写几何算法，不删除已经验证的内部
Builder/Filter/Validator。M3-L2 只有在新旧差分和动态验收完成后，才可授权删除
旧 production 入口或继续拆分 1200 行 raw Builder。

## 2. 必须冻结的已完成基线

### Z1

- planner 最终 C2 路径是 `delta=0` neutral 权威中心线；
- production Tube 使用 `planning/safe_distance`；
- 横截面只保留 zero-connected interval；
- 无非零证书时是有效 zero-only `[0,0]`；
- 只有 `OFFSET_CERTIFIED` 能激活非零 offset authority；
- retained delta 仍属于 Runtime/handoff containment，不返回 Tube 分量选择。

### H2-L1

- 每次 active replan 只选择一个 deterministic future seam；
- 每次 callback 最多一次 C2 build 和一次 Tube stage；
- retryable prepare/final failure 保留同一 pending identity 到 seam；
- session stale/expiry 无副作用丢弃；
- CAS 成功才 pending -> completed；
- frontend mirror 成功后才消费 completed。

不得修改上述语义或恢复 multi-seam、preferred component、zero-only gate。

## 3. 不可改变的导航与安全语义

严禁修改：

- Kino A*、B-spline optimizer；
- 五次 Hermite C2 connector 的几何、cost 和接受条件；
- global semantic `w`；
- planner 重规划触发、候选接受、速度/加速度参数；
- Governor、SO3、matched-port、Runtime 数学；
- current reference/actual clearance predicates；
- exact-PWL slope/inner-envelope 规则；
- certified-cell cover、between-knot ribbon proof、UNKNOWN/OOM fail-closed；
- snapshot identity/provenance；
- H2 PathTubePair CAS、pin 和双 mailbox。

不得通过增大 planner clearance、缩小 offset、缩短目标或切换地图来让测试通过。

## 4. 强制设计

### M3-L1.1 新的单一构建入口

新增：

```text
phase_offset_navigation/certified_tube_builder.h
phase_offset_navigation/src/certified_tube_builder.cpp
```

建议接口（字段名可按现有风格微调）：

```cpp
struct CertifiedTubeBuildInput {
  TubeSource source;
  TubeEpochPathSamples preview_path; // 或不依赖 epoch types 的等价 aligned vector
  DistanceQuery distance_query;
  ClearanceQuery cloud_clearance_query;
  PathStateQuery path_state_query;
  PathCellBoundQuery path_cell_bound_query;
  double cloud_snapshot_resolution;
  double current_w;
  std::uint64_t path_source_revision;
  std::uint64_t tube_revision;
  std::uint64_t map_observation_sequence;
  bool map_observation_is_snapshot;
};

struct CertifiedTubeBuildResult {
  TubeProfile profile;
  bool raw_complete;
  bool filtered_complete;
  bool complete;
  bool surface_validation_attempted;
  TubeSurfaceValidationResult surface_validation;
};

class CertifiedTubeBuilder {
 public:
  CertifiedTubeBuilder(const TubeBuilderConfig&,
                       const TubeFilterConfig&,
                       const TubeSurfaceValidatorConfig&);
  bool configurationValid() const;
  bool build(const CertifiedTubeBuildInput&,
             CertifiedTubeBuildResult&) const;
};
```

新 header 不得依赖 `tube_epoch_types.h`，避免 builder 反向依赖 epoch policy。
若需要 preview typedef，应在新 header 内使用与现有完全相同的 aligned vector，
或移到不含 manager/status 的低层 types header；本阶段优先前者，避免扩大修改。

### M3-L1.2 完整复刻现有 production 顺序

`CertifiedTubeBuilder::build()` 必须逐步保持当前顺序和结果：

1. FIXED/legacy source 调用现有 `TubeBuilder::build()`；
2. ESDF 调用现有 certified-cell `buildCloudClearance()` overload；
3. ESDF raw profile 在过滤/验证前附上完全相同的：
   - `snapshot_sequence`；
   - `snapshot_resolution`；
   - `snapshot_provenance_is_immutable`；
4. raw 成功后恰好调用一次现有 `TubeFilter::filter()`；
5. 过滤后无非零容量：生成 planner-authoritative zero-only；
6. ESDF 且存在非零容量：恰好调用一次现有 certified-cell
   `TubeSurfaceValidator::validate()`；
7. surface proof 失败：保留当前 first-failure diagnostics，并降级为完整
   zero-only，而不是 incomplete candidate；
8. 成功后分类只能为：
   - `OFFSET_CERTIFIED`；或
   - `ZERO_ONLY_PLANNER_BASELINE`；
9. raw/filter 真正不完整时仍返回 incomplete，不得伪装 zero-only；
10. 不在 builder 中检查 retained delta、actual position、Runtime state 或 active
    profile；这些仍由 EpochManager installation/current-state policy 负责。

### M3-L1.3 EpochManager 只消费构建结果

`TubeEpochManager` 必须：

- 将 `TubeBuilder builder_ + TubeFilter filter_ + TubeSurfaceValidator
  surface_validator_` 收敛为一个 `CertifiedTubeBuilder certified_builder_`；
- configuration validity 只查询这个组合入口及 manager 自己的 tolerance；
- `update()` 对 candidate 构建只调用一次 `certified_builder_.build()`；
- 从 result 复制 raw/filtered/complete/profile 状态；
- 删除 manager source 中只服务旧串联的：
  - `HasNonzeroCapacity()`；
  - `CollapseToPlannerZeroBaseline()`；
  - surface failure collapse/classification 分支；
- 保留 `TubeFilter::query()` 作为现有 immutable profile 查询 API；Runtime 和
  EpochManager current bounds 本阶段不迁移；
- 不改变 build 完成后的 current geometry、reference/actual clearance、retained
  containment、forward horizon、install/equivalent/reject/wait policy。

### M3-L1.4 不删除经过验证的内部证明组件

本阶段保留并继续直接测试：

- `TubeBuilder`；
- `TubeFilter`；
- `TubeSurfaceValidator`；
- 其公开配置和单元测试；
- Runtime 使用的 `TubeFilter::query()`。

它们在 M3-L1 中变为 `CertifiedTubeBuilder` 的内部组合件。不得复制其几何、
filter 或 cover 算法到新文件；新类只拥有 orchestration、classification、
provenance 和 zero-only fallback。

### M3-L1.5 结果与诊断兼容

对相同 input，迁移前后的 production pipeline 必须保持：

- profile knots、raw/filtered bounds；
- preview/certified ranges 和 truncation；
- classification、complete flags、obstacle certificate；
- snapshot provenance；
- first invalid/stop diagnostics；
- continuous zero-centerline evidence；
- EpochManager state/reason/disposition/counters；
- active install/equivalent behavior。

不新增 ROS message/schema、launch 参数、Tube epoch reason 或 Runtime mode。

## 5. 明确不做

本阶段不实现：

- immutable snapshot gradient/witness 扩展（M2，可独立按性能需要决定）；
- 删除/合并 raw Builder、Filter、SurfaceValidator 的内部算法（M3-L2）；
- atomic `ExecutionAuthority`（M4）；
- recenter/recovery（M6）；
- pin registry 或 pending/completed 删除（M7）；
- planner/C2/H2/Runtime/adapter 改动；
- proposal 理论修改；
- 新线程、异步 worker、参数或 ROS schema。

## 6. 文件白名单

只允许修改：

```text
docs/Codex_M3_L1_Certified_Tube_Builder_Pipeline_Consolidation_Execution_Spec_2026-08-20.md
docs/Codex_M3_L1_Certified_Tube_Builder_Pipeline_Consolidation_Self_Audit_2026-08-20.md

src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp

/home/cxq/ISF-GVF/handoff.md
```

只读但不得修改：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_filter.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/bspline_traj/**
src/swarm_planner/plan_env/**
Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md
```

若不能在白名单内保持行为，停止并报告，不得扩大到 adapter、Runtime、planner、
C2 或 launch/config。

## 7. 必须新增的测试

### CertifiedTubeBuilder 定向测试

1. invalid sub-config 使组合配置 fail closed；
2. FIXED 输入与迁移前 `TubeBuilder + TubeFilter` 结果等价；
3. ESDF open ribbon 与迁移前
   `buildCloudClearance + filter + certified-cell validate` 结果等价；
4. snapshot sequence/resolution/immutable provenance 原样保留；
5. zero-connected 无非零容量输出完整 zero-only；
6. surface proof failure 输出完整 zero-only，并保留 first failure diagnostics；
7. raw incomplete 仍 incomplete，不升级 zero-only；
8. 每个阶段每次 build 最多调用一次；若现有纯接口难以注入计数，使用结果与
   production source structural scan 双重证明，不得为了计数修改冻结类。

### EpochManager 回归/结构测试

1. 原 52 个 tests 全部保留；
2. 增加至少一项断言 manager 的 candidate raw/filtered/complete 状态来自统一
   result，zero-only 与 offset-certified installation 行为不变；
3. production source scan 必须显示 `TubeEpochManager::update()` 不再直接调用：
   - `TubeBuilder::build*`；
   - `TubeFilter::filter()`；
   - `TubeSurfaceValidator::validate()`；
4. manager source 不再包含旧 collapse/capacity helper。

### 必须复跑

- `certified_tube_builder_test`；
- `phase_offset_tube_epoch_manager_test`；
- `phase_offset_tube_builder_test`；
- `phase_offset_tube_filter_test`；
- `phase_offset_tube_surface_validator_test`；
- `phase_offset_tube_cross_section_test`；
- `phase_offset_runtime_test`；
- `phase_offset_matched_adapter_test`；
- `gvf_switch_policy_test`；
- `continuous_phase_path_test`。

## 8. 构建与静态验收

必须通过：

```text
catkin_make -j2
git diff --check
```

并记录：

- branch/HEAD；
- dirty worktree；
- 阶段前后白名单文件；
- dependency scans；
- 新旧 pipeline 等价测试；
- 未触及冻结文件。

## 9. Launch 动态验收

静态测试完成后执行动态验收。必须先检查现有 ROS master；不得连接、关闭或复用
用户进程。使用独立 `ROS_MASTER_URI`、独立 `ROS_HOME`、任务专属 PID/log 目录，
只清理由本阶段启动的进程。

按顺序执行：

1. neutral baseline：使用现有单机启动链，PhaseOffset disabled，目标
   `(8, 0, 1)`；验证 planner/C2/Governor/SO3 正常且到达；
2. ESDF observe-only：
   `phase_offset_esdf_tube_single.launch` 默认参数，目标 `(8, 0, 1)`；验证
   candidate 构建持续、neutral 不被 Tube gate、目标到达；
3. ESDF active nonzero：同一 launch，仅覆盖
   `phase_offset_manual_observe_only:=false`，目标 `(8, 0, 1)`；验证：
   - 初始 PathTubePair 激活；
   - candidate/epoch 持续更新；
   - 出现 replan 时使用单一 seam；
   - retryable failure 不丢 pending；
   - 不出现 `new path + old/null Tube`；
   - 若成功 successor 存在，观察至少一次 H2 replacement；
   - 最终是否到达必须如实记录。

动态 active 若仍因“没有任何可在 deadline 前提交的 successor”进入已知 M6
边界，不得在本阶段修改 recenter、planner、Tube margin 或目标来掩盖。记录为
`STATIC_PASS_DYNAMIC_M6_BOUNDARY` 并停止；若失败发生在新统一 pipeline 与旧结果
不一致，则 M3-L1 不通过，必须在白名单内修正或报告 blocker。

## 10. 验收标准

1. production candidate 构建只有一个 `CertifiedTubeBuilder` 入口；
2. EpochManager 不再手工拥有/串联三个构建证明对象；
3. 构建顺序、分类、zero-only fallback、diagnostics 和 provenance 不变；
4. current-state/retained/install/epoch policy 仍只在 EpochManager；
5. standalone Builder/Filter/Validator 和 Runtime query API 不变；
6. 所有定向与回归测试通过；
7. 完整构建和 diff-check 通过；
8. 三类 launch 动态结果有隔离日志和准确结论；
9. 无参数、线程、schema、planner/C2/Runtime/H2 理论变化；
10. self-audit 明确 M3-L2/M4/M6/M7 尚未完成。

## 11. 停止条件

遇到以下任一项停止并报告：

- 需要修改白名单外源码；
- 新旧 pipeline 对相同输入产生无法解释的 profile/diagnostic 差异；
- 需要削弱 continuous ribbon proof 或 UNKNOWN/OOM fail-closed 才能通过；
- 需要修改 Runtime、adapter、planner、C2、launch 参数或安全裕度；
- 动态失败只能由 M6 recenter/physical recovery 解决。

完成本阶段后写 self-audit、更新 handoff 并停止。不得自动进入 M3-L2、M4、M6
或 M7。
