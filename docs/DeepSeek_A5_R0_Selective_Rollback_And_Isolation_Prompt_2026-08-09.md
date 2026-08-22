# DeepSeek A5-R0 选择性撤回、诊断隔离与 Tube Epoch 重构前置执行规范

日期：2026-08-09

工作目录：

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

## 0. 文档性质与授权边界

本文件定义 A5-R 系列修订的第一步：`A5-R0`。

本轮目标只有两个：

1. 精确撤回已经越过 A5 模块边界的 A5.2.3 planner-clearance-contract；
2. 将 A5.2.2 clearance audit 从在线控制适配器和正式构建链中隔离，同时保留其
   源文件和历史证据作为只读诊断参考。

本文件还在后半部分记录 A5-R1/A5-R2 的目标架构，但这些章节只用于后续设计，
不授权本轮实现。

```text
CURRENT_AUTHORIZED_STAGE=A5-R0
AUTO_ADVANCE=false
BLOCKED_STAGE=A5
```

完成 A5-R0 的构建、测试、ROS baseline 回归、自审核和汇报后立即停止。

严禁自动进入：

- A5-R1 TubeEpochManager；
- A5-R2 Runtime/ROS 集成；
- A5.3 snapshot/coherence；
- A6 C2 phase-offset/tube continuation；
- swarm、CBF、多机、场景扩展或论文统计。

生成或读取本文件不等于已经执行 A5-R0。只有用户明确要求“执行 A5-R0”后，
才允许修改本文件白名单中的源码。

## 1. 修订依据

详细 proposal 的主链为：

```text
低频：A*/B-spline + existing C2
  -> accepted base centerline p^kappa(w)

中频：accepted path + latest local SDF
  -> candidate robust tube
  -> hybrid installation check
  -> active tube epoch ell

高频：query active tube only
  -> project (u_w, u_delta)
  -> same final port in physical/internal channels
```

基础路径和活动参考定义为：

```text
r(w, delta) = p(w) + N(w) delta
```

tube 是：

```text
delta_lower(w) <= delta <= delta_upper(w)
```

即规划中心线 `p(w)` 在 `delta=0` 处是 lifted strip 的中心切片；允许的标量
offset 沿 `N(w)` 扫出 reference ribbon。tube 不是第二个 planner，也不是通过
高频修改 B-spline 控制点产生。

建议频率：

```text
A*/B-spline/C2        2-5 Hz or event driven
local map/tube       10-20 Hz
phase-offset port    30-50 Hz
ISF/matched          50-100 Hz
```

## 2. 已确认的当前问题

### 2.1 正确且应保留的数学件

以下主线与 proposal 一致：

```text
ContinuousPhasePath
  -> PathDifferentialState
  -> GeometryEvaluator
  -> p, N, N_w, curvature
  -> TubeBuilder ray search along +/-N
  -> obstacle/regularity intersection
  -> TubeFilter
  -> PortProjector
  -> MatchedPort
```

因此 A5-R0 禁止把 A1-A4 和这些纯算法件作为“回滚对象”一起删除。

### 2.2 当前 A5 Runtime 的结构性问题

当前 control loop 以 `dt=0.02` 调用 adapter；adapter 每周期采样路径并调用
`PhaseOffsetRuntime::prepare()`；`prepare()` 在 tube source 非 NONE 时每周期执行
`rebuildProfile()`。

当前 `rebuildProfile()`：

```text
build candidate into local variable
filter candidate
profile_ = built
```

它没有独立的：

```text
candidate_profile
active_profile
active_tube_epoch
hybrid_install_result
```

因此 `tube_revision` 是 rebuild counter，不是论文语义中的 tube epoch；一次临时
incomplete rebuild 还能覆盖上一次 complete profile。

这一问题属于 A5-R1/A5-R2，A5-R0 只记录并隔离，不得顺手修改 Runtime。

### 2.3 A5.1 的准确定位

A5.1 只处理原 simulator RViz：

- base/active/frame/tube display；
- lower/upper boundary；
- translucent ribbon；
- invalid/incomplete 时 IDs 0/1/2 同步 DELETE。

A5.1 未修改 tube 数学和控制，因此本轮保留。

### 2.4 A5.2 的准确定位

A5.2 完成了有价值的确定性修复：

- 裸 launch 恢复 `disabled/none`；
- refresh 默认保持 `3.0`；
- failure/unsafe 时 certified Marker fail closed；
- readiness/first-invalid/CSV schema 诊断更准确。

但 A5.2 明确没有实现 candidate/installed tube epoch。A5-R0 保留其 baseline 修复
和 83 字段兼容状态，不宣称这些字段代表最终 epoch 设计。

### 2.5 A5.2.1 的准确定位

A5.2.1 抽取了 `phase_offset_tube_markers` helper，并增加两套只读 Marker topic。
这是可复用的显示实现，本轮保留。

但当前 Candidate 和 Certified 都读取同一个 Runtime `TubeProfile`。它们只是两种
显示 gate，不是真正的 candidate/active profile。该语义在 A5-R2 重接，本轮不改。

### 2.6 A5.2.2 的准确定位

A5.2.2 的 60 字段 clearance audit 成功证明 planner centerline clearance 与 tube
reference clearance 不一致，是有价值的诊断证据。

但它：

- 不属于最终控制闭环；
- 重复采样 `ContinuousPhasePath` 和查询 SDF；
- 增加 adapter 参数、publisher 和发布路径；
- 不应继续作为正式 runtime architecture。

A5-R0 将其从 launch、adapter 和 CMake 正式接线中移除，但不删除 audit 源文件、
测试源码或 `/tmp` 证据。

### 2.7 A5.2.3 的准确定位

A5.2.3 修改了：

- Kinodynamic A* margin schedule 和 shot clearance；
- B-spline optimizer obstacle activation threshold；
- `gvf_manager` initial/replan/C2 candidate installation；
- planner-contract diagnostics。

这同时违反原 A5 执行单的 planner/A*/B-spline/C2 禁改边界，并把“中心线
`d(p(w)) >= 0.70`”误当成 proposal 所要求的 candidate-tube feasibility。

真正的 candidate-path tube 检查必须包括：

- 候选路径对应的 lateral bounds；
- retained `delta`；
- current reference/tracking ball；
- forward offset evolution；
- existing C2 continuation。

这些属于后续 A6，不属于 A5-R0。

## 3. A5-R0 完成后的目标状态

A5-R0 完成后，工作区必须满足：

```text
Original baseline launch semantics preserved
A1-A4 preserved
A5 TubeBuilder/TubeFilter/PortProjector preserved
A5.1 RViz/ribbon preserved
A5.2 baseline/display fail-closed fixes preserved
A5.2.1 marker helper/topics preserved

A5.2.2 online audit disconnected, sources retained as reference
A5.2.3 planner enforcement completely removed

A5 remains BLOCKED
ESDF tube is not accepted as final safe runtime
No A5-R1/R2/A6 code exists yet
```

## 4. 开始前只读检查

执行者必须先完整读取：

```text
AGENTS.md
docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md
docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md
docs/DeepSeek_A5_Single_UAV_Safe_Tube_Runtime_Prompt_2026-08-08.md
docs/DeepSeek_A5_1_RViz_Tube_Visualization_Repair_Prompt_2026-08-08.md
docs/DeepSeek_A5_2_ESDF_Readiness_And_Static_Refresh_Isolation_Prompt_2026-08-08.md
本执行单
```

记录：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git stash list
```

预期：

```text
branch = main
HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
stash contains deepseek-phaseoffset-tracked-prototype-2026-08-08
```

工作树现有内容全部视为用户所有。禁止清理或覆盖。

开始前还必须记录以下文件的 SHA-256，作为“只反向撤指定增量”的依据：

```text
kinodynamic_astar.h/.cpp
bspline_opt_3d.h/.cpp
gvf_manager.h/.cpp
test_gvf.launch
bspline_traj/CMakeLists.txt
path_searching/CMakeLists.txt
phase_offset_matched_adapter.h/.cpp
```

若用户在本执行单生成后继续修改其中任何重叠区域，必须停止并重新审计，不得
覆盖用户新改动。

## 5. Git 与编辑安全

严格禁止：

```text
git reset --hard
git checkout -- .
git restore .
git clean
git stash pop
```

禁止创建 commit、branch、tag 或 push。

所有反向修改必须使用小范围 `apply_patch`，逐个符号和逐个 CMake/launch 接线
撤回。不得整文件替换。

## 6. A5-R0 文件白名单

只允许修改以下既有文件：

```text
src/swarm_planner/path_searching/CMakeLists.txt
src/swarm_planner/path_searching/include/path_searching/kinodynamic_astar.h
src/swarm_planner/path_searching/src/kinodynamic_astar.cpp

src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/include/bspline_race/bspline_opt_3d.h
src/swarm_planner/bspline_traj/src/bspline_opt_3d.cpp
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/launch/test_gvf.launch
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

只允许删除以下 A5.2.3 新增文件，并且删除前必须确认内容仍与 A5.2.3 实现一致、
没有后续用户修改：

```text
src/swarm_planner/path_searching/include/path_searching/clearance_margin_policy.h
src/swarm_planner/path_searching/test/clearance_margin_policy_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_planner_clearance_contract.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_planner_clearance_contract.cpp
src/swarm_planner/bspline_traj/test/phase_offset_planner_clearance_contract_test.cpp
src/swarm_planner/bspline_traj/test/bspline_optimizer_clearance_contract_test.cpp
```

以下 A5.2.2 文件必须保留且本轮不得修改，只从在线接线中隔离：

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp
```

## 7. 本轮明确禁止修改的保留模块

```text
src/swarm_planner/phase_offset/phase_offset_core/**
src/swarm_planner/phase_offset/phase_offset_navigation/**

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_markers.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_markers.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp

src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz
plan_env/**
SDFMap
ContinuousPhasePath
gvf.cpp/gvf.h
governor
PositionCommand
SO3
simulator
messages
swarm
CBF
```

特别说明：A5-R0 已知当前 `PhaseOffsetRuntime` 仍存在 50 Hz rebuild 和单 profile
问题，但本轮禁止修改它。该问题必须留给独立 A5-R1/R2 执行单，避免 rollback、
状态机重构和 ROS 集成混在一轮。

## 8. A5.2.3 精确撤回要求

### 8.1 Kinodynamic A*

从 `kinodynamic_astar.h/.cpp` 精确移除：

```text
#include <path_searching/clearance_margin_policy.h>
KinoClearanceReport
clearance_contract_enabled_
clearance_contract_required_
current_attempt_margin_
clearance_report_
setClearanceContract()
clearClearanceContract()
resetClearanceReport()
clearanceReport()
contract-only shot getDistance/current margin checks
contract margin schedule orchestration
```

恢复 A5.2.3 开始前的 legacy search 行为：

- 原 `margin_` schedule；
- 原 retry 次数；
- 原 search return code；
- 原 shot 只使用既有 occupancy 语义；
- 不顺手修复任何历史 planner 行为。

`path_searching/CMakeLists.txt` 中移除 `clearance_margin_policy_test` target 和链接。

### 8.2 B-spline optimizer

精确移除：

```text
effective_safe_distance_
beginClearanceContract()
endClearanceContract()
effectiveSafeDistance()
ScopedClearanceSafeDistance
contract-only activation distance selection
```

`calcEsdfCost()` 恢复只使用原 `safe_distance_`。

必须保持：

```text
planning/safe_distance = original configured behavior
lambda3 = 10
max velocity/acceleration unchanged
all unrelated optimizer formulas unchanged
```

### 8.3 gvf_manager

精确移除 A5.2.3 引入的：

```text
phase_offset_planner_clearance_contract include
planner_clearance_contract_* members
planner_contract_attempt_sequence_
last_kino_report_
planner_contract_pub_
KinoPlanSamples::kino_report

planner contract parameter parsing
planner contract publisher
plannerContractApplicable()
plannerContractRequiredClearance()
runPlannerContractValidation()
publishPlannerContractDiagnostics()

Kino set/clear/reset contract calls
ScopedClearanceSafeDistance use
initial frontend hard validator
C2 trial validator
replan install validator
48-field diagnostic publication
```

要求：

- 只反向撤 A5.2.3；
- 不改已有 path phase、C2、candidate cost、install 顺序或 governor；
- 不重置 `phase_w_`；
- 不修改 A1-A4/A5 adapter 调用；
- `gvf_manager.cpp` 中不留下 planner-tube clearance 公式或诊断组装。

### 8.4 Launch

只移除：

```text
phase_offset_planner_clearance_contract_enable
phase_offset_planner_clearance_contract_sample_step_w
phase_offset_planner_clearance_contract_max_chord_m

phase_offset/planner_clearance_contract/enable
phase_offset/planner_clearance_contract/sample_step_w
phase_offset/planner_clearance_contract/max_chord_m
```

必须保留：

```text
phase_offset_mode default=disabled
phase_offset_manual_tube_source default=none
sdf_map_buffer_refresh_period default=3.0
gvf/circle_test/enable=false
gvf/circle_test/auto_start=false
```

### 8.5 CMake 和新文件

从 `bspline_traj/CMakeLists.txt` 精确移除：

```text
phase_offset_planner_clearance_contract.cpp
phase_offset_planner_clearance_contract_test
bspline_optimizer_clearance_contract_test
相关 target_link_libraries
```

删除第 6 节列出的六个 A5.2.3 新文件。

## 9. A5.2.2 在线 audit 隔离要求

### 9.1 Adapter

从 `phase_offset_matched_adapter.h/.cpp` 移除在线接线：

```text
#include phase_offset_clearance_audit.h
clearance_audit_enable
clearance_audit_publish_rate
clearance_audit_sample_step_w
clearance_audit_lateral_probe_half_width
audit parameter parsing
audit configuration validation
clearance_audit_pub_
last_audit_publish_stamp_
publishClearanceAudit()
publishManual() 中的 audit call
```

禁止修改：

- 83 字段 manual diagnostics 的现有索引和数值；
- candidate/certified Marker helper；
- gate、selected、delta、port 或 Runtime 调用；
- TubeBuilder/TubeFilter；
- A5.1/A5.2.1 topic。

### 9.2 Launch

移除：

```text
phase_offset_clearance_audit_enable
phase_offset_clearance_audit_publish_rate
phase_offset_clearance_audit_sample_step_w
phase_offset_clearance_audit_lateral_probe_half_width

phase_offset/clearance_audit/enable
phase_offset/clearance_audit/publish_rate
phase_offset/clearance_audit/sample_step_w
phase_offset/clearance_audit/lateral_probe_half_width
```

### 9.3 CMake

从正式构建接线中移除：

```text
phase_offset_clearance_audit.cpp
phase_offset_clearance_audit_test target
相关 target_link_libraries
```

audit 的 header/source/test 文件仍保留在工作树，作为用户拥有的只读诊断参考；
禁止删除、移动、重命名或继续扩展。

## 10. 必须保留的 A5.1/A5.2/A5.2.1 内容

### 10.1 A5.1

- 原 `swarm_rviz.rviz` 中 A5 displays；
- lower/upper line；
- triangle ribbon；
- invalid/incomplete 三 DELETE；
- 不自动启动第二个 RViz。

### 10.2 A5.2

- baseline defaults；
- fixed/ESDF source enum；
- readiness evaluated 和 first-invalid 诊断；
- failure/unsafe certified Marker fail closed；
- header/row 数量严格检查原则；
- manual diagnostics 暂时保持 83 字段。

### 10.3 A5.2.1

- `phase_offset_tube_markers.h/.cpp`；
- Candidate 和 Certified 两个独立 topic；
- 独立 namespace、颜色、alpha；
- Candidate 明确标记 UNCERTIFIED；
- helper 不包含 Runtime、ESDF、control 或 planner 逻辑。

本轮必须在最终报告中注明：

```text
这些 Marker 仍读取同一个 legacy Runtime TubeProfile；
它们尚未代表 proposal 中真正的 candidate_profile/active_profile。
```

## 11. A5-R0 单元与构建验证

### 11.1 静态符号检查

以下 active source/build 文件中必须无命中：

```text
planner_clearance_contract
PlannerClearanceContract
clearance_margin_policy
KinoClearanceReport
ScopedClearanceSafeDistance
effective_safe_distance_
beginClearanceContract
endClearanceContract
phase_offset_clearance_audit
publishClearanceAudit
clearance_audit_pub_
```

允许 `phase_offset_clearance_audit.*` 三个隔离参考文件自身命中 audit 名称；它们
不得出现在 CMake、launch 或 adapter 中。

### 11.2 构建

```bash
catkin_make -j8
```

必须通过。

### 11.3 回归测试

至少运行：

```text
phase_offset_core full tests
phase_offset_navigation full tests
bspline_race guidance/active/matched/marker tests
path_searching existing tests
```

要求：

- A1-A4 回归不减少；
- A5 TubeBuilder/TubeFilter/Runtime 现有测试仍通过；
- A5.1/A5.2.1 Marker 测试仍通过；
- 不再构建 A5.2.2 audit test；
- 不再构建 A5.2.3 contract tests；
- `catkin_test_results --verbose` 仅允许既有 `uav_utils` 缺 XML 历史项。

## 12. A5-R0 ROS baseline 回归

运行 ROS 前：

1. 检查是否已有 ROS master；
2. 不连接或终止用户进程；
3. 使用隔离 ROS port；
4. 只清理本轮启动的进程。

### 12.1 ROS-0 裸 baseline

裸 `test_gvf.launch` 实际参数必须为：

```text
phase_offset mode = disabled
tube source = none
refresh = 3.0
circle enable = false
circle auto_start = false
```

验收：

- manual candidate/certified/diagnostics 无 publisher；
- clearance audit 无 publisher；
- planner clearance contract 无 publisher；
- `/position_cmd` 唯一 publisher 仍为 `/formation_planning`；
- 原 pillar 点到点 `(8,0,0)` 或原已验证等价目标正常到达；
- planner、ISF-GVF、governor、SO3 正常。

### 12.2 ROS-1 A4 manual/none 回归

显式：

```text
phase_offset mode = manual
tube source = none
```

只验证 A4：

- 100 周期 gate 语义不变；
- same final port；
- delta 只由 final `u_delta` 积分；
- manual 83 字段长度保持；
- 无 audit/planner-contract publisher。

### 12.3 可选 fixed 显示回归

可显式运行 fixed tube，仅验证 A5.1/A5.2.1 Marker 几何未被 R0 破坏。

本轮禁止运行 ESDF 长窗口并宣称 A5 安全验收；R0 没有修复 Runtime epoch。

## 13. 自审核

完成后必须运行：

```bash
git diff --check
git status --short
git diff --stat
```

并逐项确认：

1. 只修改第 6 节白名单；
2. 只删除六个明确列出的 A5.2.3 文件；
3. A5.2.2 audit 三个源/测试文件仍存在且未修改；
4. A5.1 RViz/ribbon 未修改；
5. A5.2.1 marker helper 未修改；
6. phase_offset_core/navigation 未修改；
7. planner 恢复 A5.2.3 前行为；
8. no staged/commit/tag/push；
9. prototype stash 未触碰；
10. 用户其他 dirty 内容未触碰。

## 14. A5-R0 停止条件

发生以下任一情况立即停止：

- 需要修改 `phase_offset_navigation` 才能完成 R0；
- 需要修改 A5.1/A5.2.1 helper；
- 无法区分 A5.2.3 hunks 与用户后续修改；
- 撤 contract 后原 planner 无法构建，且修复需要扩大范围；
- baseline ROS 不可用；
- workspace-wide test 被无关历史缺陷阻塞；
- 验收需要调速度、margin、lambda、tracking bound、地图或 C2。

不得以“顺便修好 A5”为理由扩大范围。

## 15. A5-R0 最终报告格式

最终报告必须包含：

1. 执行前 branch/HEAD/stash/status；
2. 精确修改和删除文件；
3. A5.2.3 每一层撤回结果；
4. A5.2.2 audit 隔离结果；
5. A5.1/A5.2/A5.2.1 保留结果；
6. static symbol search；
7. build/test 结果；
8. ROS-0/ROS-1/可选 fixed 结果；
9. final diff/status/self-audit；
10. 明确停止语句。

最终必须原样包含：

```text
A5-R0 只完成选择性撤回与诊断隔离；未修改原 A5 Runtime，未实现
TubeEpochManager，未进入 A5-R1、A5-R2、A5.3 或 A6。A5 继续 blocked。
```

---

## 16. 后续目标架构（只读设计，不授权本轮实现）

以下内容用于约束后续独立执行单。

### 16.1 A5-R1：纯 C++ TubeEpochManager

建议在 `phase_offset_navigation` 中增加单一职责组件：

```text
TubeEpochManager
```

职责：

```text
input:
  accepted path preview
  path source revision
  map observation token
  DistanceQuery
  current w/delta
  current reference and tracking state

build:
  TubeBuilder -> candidate raw profile
  TubeFilter  -> candidate filtered profile

hybrid check:
  current reference/tracking ball safe
  retained delta inside candidate current bounds
  current geometry regular
  forward rolling feasibility status

state:
  candidate_profile
  active_profile
  candidate_sequence
  active_tube_epoch
  install result/status
```

重要语义：

- build candidate 不得覆盖 active profile；
- active epoch 只在 hybrid installation 时变化；
- 相同 path/map observation 和等价 profile 不得无意义增加 epoch；
- `w`、`delta` 在合法 tube update 时保持不变；
- current safe 但 forward infeasible 时，最新安全 tube 仍可安装，同时输出
  `REPLAN_REQUIRED`；
- latest observation 已否定 current safety 时，旧 active tube 不得继续作为安全证书，
  输出 `SAFETY_PRIORITY`；
- UNKNOWN/OUT_OF_MAP/UNAVAILABLE 不得扩大 candidate bounds；
- 不使用 permanent failure latch 表达普通 transient map/readiness 状态。

建议状态枚举：

```text
NO_ACTIVE_TUBE
WAITING_FOR_CANDIDATE
ROLLING
REPLAN_REQUIRED
SAFETY_PRIORITY
CONFIGURATION_ERROR
```

R1 必须是 ROS-free，不修改 planner、C2、SDFMap、adapter 或 manager。

若 integration 无法提供真实 map revision，不得把本地计数命名为 `map_revision`。
只能命名为 `map_observation_sequence`，并明确它不构成 SDF snapshot 证明。

若实现安全 epoch 必须修改 SDFMap 才能获得一致读取，应停止并申请独立阶段，
不得在 R1 中偷偷进入 A5.3。

### 16.2 A5-R2：多频率 Runtime/ROS 集成

目标调用链：

```text
control callback 50 Hz
  if tube update due (10-20 Hz or path event):
      adapter samples accepted path once
      adapter supplies DistanceQuery + observation sequence
      TubeEpochManager builds/checks candidate

  Runtime high-rate step:
      read immutable active profile snapshot
      query current/next bounds
      PortProjector
      MatchedPort
      integrate final delta
```

R2 完成后，`PhaseOffsetRuntime` 不再拥有：

```text
DistanceQuery
TubeBuilder
TubeFilter
rebuild_count as tube epoch
stable_rebuild_cycles
```

Runtime 只拥有：

```text
delta
previous final port
manual test intent state
active tube snapshot/reference
step-local projection/matched result
```

Marker 重新接线：

```text
Candidate Marker <- candidate_profile
Certified/Active Marker <- active_profile + active_tube_epoch
Execution status <- separate state/diagnostic
```

100 周期 zero-port gate 只决定 guidance selection，不决定 tube 是否在几何上完成或
是否已经安装为 active epoch。

R2 验收必须证明：

- 50 Hz control step 不会触发 50 Hz tube rebuild；
- stable static path/map 下 active epoch 至少 15 s 不变化；
- rejected candidate 不覆盖 active profile；
- installed candidate 原子增加 epoch；
- install 前后 `w`、`delta` 不 reset/clip；
- failure/transient 状态不会造成 stale Certified ADD；
- disabled baseline 等价；
- fixed tube 和 ESDF tube 分别验收；
- planner/A*/B-spline/C2 仍未修改。

### 16.3 A6：现有 C2 connector 周围的 candidate-path tube 检查

A6 才允许：

```text
new planner candidate
  -> existing C2 connector candidate
  -> construct candidate path tube
  -> retained delta feasibility
  -> forward offset reachability
  -> install path with retained w/delta
```

禁止重新引入 A5.2.3 的简单中心线 `distance >= 0.70` gate 代替完整 tube 检查。

## 17. 给执行者的启动指令

当用户未来明确授权执行时，只发送：

```text
严格执行：
docs/DeepSeek_A5_R0_Selective_Rollback_And_Isolation_Prompt_2026-08-09.md

本轮只执行 A5-R0。AUTO_ADVANCE=false。完成选择性撤回、诊断隔离、构建、
回归、自审核和报告后立即停止。禁止修改 phase_offset_core/navigation，禁止进入
A5-R1、A5-R2、A5.3 或 A6。
```
