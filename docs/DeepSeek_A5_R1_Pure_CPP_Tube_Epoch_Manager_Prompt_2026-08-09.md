# DeepSeek A5-R1 纯 C++ TubeEpochManager 执行规范

日期：2026-08-09

工作目录：

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

阶段控制：

```text
DOCUMENT_ROLE=STAGE_EXECUTION_SPEC
CURRENT_AUTHORIZED_STAGE=A5-R1
AUTO_ADVANCE=false
BLOCKED_STAGE=A5
```

本文件是 A5-R1 的独立执行规范。读取或生成本文件不等于开始执行；只有用户后续
明确要求“严格执行本执行单”后，才允许修改本文件白名单中的源码。

完成 A5-R1 的实现、构建、单元测试、回归、自审核和汇报后立即停止。

严禁自动进入：

- A5-R2 Runtime/ROS 多频率集成；
- A5.3 SDF snapshot/coherence；
- A6 existing C2 周围的 candidate-path tube continuation；
- A7 单机完整闭环；
- swarm、CBF、多机、复杂地图或论文统计。

---

## 1. 本轮唯一目标

在 `phase_offset_navigation` 中新增一个 ROS-free、单一职责的：

```text
TubeEpochManager
```

它必须把当前 A5 legacy Runtime 中混在一起的两种对象明确分开：

```text
candidate_profile
active_profile
```

并建立真正的：

```text
candidate_sequence
active_tube_epoch
hybrid installation result
```

本轮只实现和单元测试该纯 C++ 组件，不把它接入 `PhaseOffsetRuntime`、adapter、
manager、ROS topic、Marker 或真实 SDFMap。

本轮结束时，当前线上 Runtime 行为必须完全不变；A5 仍然 blocked。

---

## 2. 与详细 proposal 的严格对应关系

proposal 的正确低频/中频/高频链路是：

```text
低频：A*/B-spline + existing C2
  -> accepted base centerline p^kappa(w)

中频：accepted path preview + latest local distance observation
  -> PathDifferentialState
  -> GeometryEvaluator gives p(w), N(w), N_w(w), regularity
  -> TubeBuilder ray search along +/-N
  -> TubeFilter
  -> candidate robust tube
  -> hybrid installation check
  -> active tube epoch ell

高频：query active tube only
  -> PortProjector
  -> MatchedPort
  -> same final port in physical / phase / delta channels
```

活动参考仍为：

\[
r(w,\delta)=p(w)+N(w)\delta.
\]

tube 仍为 lifted strip：

\[
\underline\delta(w)\le\delta\le\overline\delta(w).
\]

因此：

- tube 必须建立在已经接受的规划中心线及其 lifted geometry 上；
- `p(w)` 是 `delta=0` 的中心切片；
- 沿 `N(w)` 扫描允许的标量 offset 才形成 ribbon/tube；
- tube 不是第二个 planner；
- tube 不通过修改 A*、B-spline 控制点或 C2 connector 产生；
- A5-R1 不检查新 planner candidate，也不决定 planner path 是否安装。

---

## 3. 已确认的当前结构问题

当前 `PhaseOffsetRuntime::prepare()` 每个 50 Hz control step 都会调用
`rebuildProfile()`。

当前 `rebuildProfile()` 的核心语义是：

```text
build local TubeProfile
filter local TubeProfile
profile_ = built
```

由此产生以下问题：

1. 没有独立 candidate profile；
2. 没有独立 active profile；
3. incomplete candidate 可以覆盖之前的 complete profile；
4. `rebuild_count` 被写入 `tube_revision`，但它只是构建尝试计数；
5. 没有真正的 active tube epoch；
6. 普通 transient readiness 会和 permanent `failure_latched_` 混在一起；
7. Candidate/Certified Marker 当前仍读取同一个 legacy `output.tube_profile`；
8. 当前 100-cycle startup gate 被错误地卷入 tube 构建/显示时序理解。

A5-R1 只新增正确的数据模型和纯 C++ 状态机，不修改上述线上 Runtime。

---

## 4. A5-R0-r1 前置状态

执行前必须确认：

```text
branch = main
HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
stash contains deepseek-phaseoffset-tracked-prototype-2026-08-08
no staged content
```

并确认 A5-R0-r1 已完成：

- A5.2.3 planner-clearance-contract 已撤回；
- A5.2.2 audit 已从在线链路/CMake 隔离；
- `bspline_opt_3d.cpp` 不再有 A5.2.3 残留 diff；
- `gvf_manager.cpp` 不再存在 `prospective_phase_w`；
- 当前正式 CTest 为 19 个 target；
- 旧 audit/contract/prototype test XML 已清理；
- A5.1/A5.2/A5.2.1 保留；
- Candidate/Certified Marker 仍共用 legacy profile，尚未重接。

若任一前置不符，停止并报告，不得一边修 R0 一边实现 R1。

---

## 5. A5-R1、A5-R2、A5.3 与 A6 的边界

### 5.1 A5-R1 本轮允许

- 在 `phase_offset_navigation` 中新增纯 C++ TubeEpochManager；
- 复用现有 `TubeBuilder`、`TubeFilter`、`GeometryEvaluator` 和
  `DistanceQuery`；
- 构建 latest candidate profile；
- 检查 current geometry、retained delta、tracking/current clearance；
- 计算纯几何 forward status；
- 原子决定是否替换 active profile；
- 维护 candidate sequence 和 active tube epoch；
- 对 transient invalid/unavailable 实现可恢复状态，而不是永久 latch；
- 为全部语义添加 ROS-free gtest。

### 5.2 A5-R2 才允许

- 把 manager 接入 adapter/Runtime；
- 以 10-20 Hz 或 path event 构建 candidate；
- 让 50 Hz control step 只读取 active profile；
- 从 legacy Runtime 移除 TubeBuilder/TubeFilter/rebuild state；
- Candidate Marker 接 candidate profile；
- Certified/Active Marker 接 active profile 和 active epoch；
- 增加 ROS diagnostics；
- 验证 15 s stable active epoch；
- 验证 rejected candidate 不覆盖高频 active profile。

### 5.3 A5.3 才允许

- 修改 SDFMap 或其 callback/read API；
- 实现一致的 map snapshot/revision；
- 证明一次 tube build 中所有查询来自同一个 ESDF 版本；
- map mutex/callback queue/coherent read transaction；
- `MapSnapshot`、`SafeLiftedTubeSnapshot` 或等价对象。

### 5.4 A6 才允许

- 对 planner/C2 candidate path 构建 candidate-path tube；
- 检查 retained delta 在新 path/tube 上的 continuation；
- forward offset reachability；
- new path install decision；
- 比较更新前后 `p,p_w,p_ww,r,r_w`；
- 修改或包装 existing C2 install 路径。

A5-R1 禁止把上述后续内容提前塞进 `TubeEpochManager`。

---

## 6. 开始前必须完整读取

执行者必须完整读取：

```text
AGENTS.md
docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md
docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md
docs/DeepSeek_A5_Single_UAV_Safe_Tube_Runtime_Prompt_2026-08-08.md
docs/DeepSeek_A5_R0_Selective_Rollback_And_Isolation_Prompt_2026-08-09.md
本执行单
```

并完整读取当前：

```text
phase_offset_navigation/distance_query.h
phase_offset_navigation/tube_types.h
phase_offset_navigation/tube_builder.h/.cpp
phase_offset_navigation/tube_filter.h/.cpp
phase_offset_navigation/phase_offset_runtime.h/.cpp
phase_offset_navigation 全部现有 tests
phase_offset_core/path_state.h
phase_offset_core/geometry.h/.cpp
phase_offset_core/port_types.h
```

旧 prototype 只能只读参考，禁止修改、编译或接线。

---

## 7. 开始前记录

执行并保存原始输出：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git stash list
git diff --cached --name-only
ctest --test-dir build -N
catkin_test_results --verbose build/test_results
```

对本轮禁止修改的 navigation 文件保存 SHA-256：

```text
include/phase_offset_navigation/distance_query.h
include/phase_offset_navigation/tube_types.h
include/phase_offset_navigation/tube_builder.h
include/phase_offset_navigation/tube_filter.h
include/phase_offset_navigation/phase_offset_runtime.h
src/tube_builder.cpp
src/tube_filter.cpp
src/phase_offset_runtime.cpp
test/tube_builder_test.cpp
test/tube_filter_test.cpp
test/runtime_test.cpp
```

结束时这些文件的 SHA-256 必须与开始前一致。

---

## 8. A5-R1 文件白名单

### 8.1 只允许新增

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
```

### 8.2 只允许修改

```text
src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
src/swarm_planner/phase_offset/phase_offset_navigation/README.md
```

共 6 个白名单路径。

需要修改任何其他源码、launch、CMake、package 或文档时立即停止并报告。

特别禁止修改：

```text
phase_offset_core/**
phase_offset_navigation/phase_offset_runtime.*
phase_offset_navigation/tube_builder.*
phase_offset_navigation/tube_filter.*
phase_offset_navigation/tube_types.h
phase_offset_navigation/distance_query.h
bspline_traj/**
gvf_manager.*
adapter/marker helper
planner/A*/B-spline/C2
SDFMap/plan_env
governor/PositionCommand/SO3/simulator
messages/swarm/CBF
launch/RViz/map/config
```

---

## 9. 依赖边界

新增代码只允许依赖：

```text
C++14 / STL / Eigen
phase_offset_core
phase_offset_navigation existing tube types/builder/filter/distance query
```

禁止包含或出现真实依赖：

```text
ROS / ros::Time / messages
SDFMap / plan_env
ContinuousPhasePath / B-spline / manager
publisher / subscriber / topic / Marker
robot_id / neighbor / swarm / CBF
PositionCommand / SO3
mutex / callback queue / AsyncSpinner
MapSnapshot / SDF snapshot/coherence
ContinuationChecker / C2 connector
```

`map_observation_sequence` 只是调用方传入的不透明 observation token；它不证明
DistanceQuery 在整个 build 期间来自同一个 SDF 版本。

禁止把它命名为：

```text
map_revision
sdf_revision
snapshot_revision
```

---

## 10. 新增类型职责

`tube_epoch_types.h` 只表达 candidate/active tube installation 概念，不混入
Runtime manual profile、ROS、planner 或 swarm 类型。

至少定义等价语义的状态：

```cpp
enum class TubeEpochState {
  NO_ACTIVE_TUBE,
  WAITING_FOR_CANDIDATE,
  ROLLING,
  REPLAN_REQUIRED,
  SAFETY_PRIORITY,
  CONFIGURATION_ERROR,
};
```

建议另外定义：

```cpp
enum class TubeInstallDisposition {
  NONE,
  INITIAL_INSTALL,
  REPLACED_ACTIVE,
  EQUIVALENT_REFRESH,
  SAFETY_REPLACEMENT,
  REJECTED_CANDIDATE,
};

enum class CurrentSafetyStatus {
  NOT_EVALUATED,
  SAFE,
  INDETERMINATE,
  UNSAFE,
};
```

名称可小幅调整，但状态语义不得减少或混在一个 bool 中。

---

## 11. 推荐纯数据接口

可采用等价于以下结构的 API：

```cpp
struct TubeEpochManagerConfig {
  TubeBuilderConfig builder;
  TubeFilterConfig filter;
  double profile_equivalence_tolerance = 1e-10;
  double inside_tolerance = 1e-10;
};

struct TubeEpochUpdateInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TubeSource source = TubeSource::NONE;
  PathDifferentialState current_path;
  PathSamples preview_path;
  Eigen::Vector3d actual_position = Eigen::Vector3d::Zero();
  DistanceQuery distance_query;
  double retained_delta = 0.0;
  std::uint64_t path_source_revision = 0U;
  std::uint64_t map_observation_sequence = 0U;
};
```

输出/状态至少提供：

```text
latest candidate profile
active profile
candidate_sequence
active_tube_epoch
active path source revision
active map observation sequence
candidate complete/filter result
current geometry valid
current bounds valid
retained delta current-inside
current reference/actual clearance status
tracking-within-bound
certified forward horizon
retained-offset forward-contained
installed / equivalent-noop / rejected
TubeEpochState
TubeInstallDisposition
finite reason/status code
```

允许具体 API 调整，但禁止返回裸指针或让调用者直接修改 manager 内部 profile。

访问 active/candidate profile 时只提供 const 引用、值拷贝或其他只读接口。

---

## 12. Candidate 构建语义

每次有效 update attempt：

1. `candidate_sequence` 单调增加；
2. 在局部变量中调用现有 `TubeBuilder`；
3. raw complete 后调用现有 `TubeFilter`；
4. 构建结果保存为 latest candidate；
5. candidate 不得直接覆盖 active；
6. candidate 的 `TubeProfile::tube_revision` 可暂时记录 candidate sequence；
7. 最终报告必须明确该字段仍不是 active tube epoch；
8. `active_tube_epoch` 只能由 hybrid installation 决定。

构建失败、filter 失败、UNKNOWN、OUT_OF_MAP、UNAVAILABLE 或 invalid path 时：

- latest candidate 必须保留完整失败诊断；
- active profile 不得被该 incomplete candidate 覆盖；
- 不得产生假 complete/obstacle-certified profile；
- 不得永久 latch，下一次有效 candidate 可以恢复。

---

## 13. Current hybrid installation check

对 complete candidate，安装判断至少检查：

1. `current_path` finite、valid；
2. `GeometryEvaluator(current_path, actual_position, retained_delta)` valid；
3. 完整 2.5D geometry 与 signed regularity valid；
4. candidate 在精确 `current_path.w` 可查询；
5. retained delta 相对 interior margin 位于 current bounds；
6. ESDF source 下 current reference query 为 finite `KNOWN_FREE`；
7. reference clearance 不低于 `requiredReferenceClearance()`；
8. actual UAV query 为 finite `KNOWN_FREE`；
9. actual clearance 不低于 `requiredActualClearance()`；
10. `||x-r||` 不超过 tracking bound；
11. fixed source 明确保持 `obstacle_certified=false`，不得伪装 ESDF safety。

禁止只检查 base centerline `p(w)` 的 clearance 后就安装。

禁止用 retained delta 的 clip/reset 让检查通过。

---

## 14. Forward 几何状态

A5-R1 只允许计算几何层 forward status，不得宣称已经证明动态 PortProjector 或
C2 continuation 可行。

至少计算：

```text
certified_forward_w
forward_horizon_sufficient
retained_offset_forward_contained
```

其中：

```text
certified_forward_w = max(0, candidate.preview_end_w - current_w)
```

`forward_horizon_sufficient` 使用现有：

```text
TubeBuilderConfig::min_certified_forward_w
```

不得新增一个重复的安全 horizon 参数。

`retained_offset_forward_contained` 必须检查：

- exact current w；
- `[current_w, current_w + min_certified_forward_w]` 内所有 profile knots；
- exact forward interval end；
- 每次使用 `TubeFilter::query()` 和同一 interior margin。

它只表示“保持当前 retained offset 的几何前视包含性”，不表示 rate-limited
offset dynamics、base `w_dot`、PortProjector convex feasibility 或 A6 candidate-path
reachability 已通过。

---

## 15. 安装决策矩阵

### 15.1 无 active，candidate incomplete

```text
state = WAITING_FOR_CANDIDATE
active profile unavailable
active epoch remains 0
```

### 15.2 complete、current safe、current inside、forward sufficient

```text
install candidate
state = ROLLING
```

### 15.3 complete、current safe/current inside，但 forward 不足

包括：

- certified forward horizon 小于要求；或
- retained offset 不在完整 forward interval 内。

此时允许安装最新 current-safe profile，但必须：

```text
state = REPLAN_REQUIRED
```

该状态只是向未来 integration 输出 replan intent；A5-R1 不发布 ROS replan 请求，
也不调用 planner。

### 15.4 complete，但 retained delta/current error ball 不再可接受

若 candidate 本身是完整的最新安全 envelope，但 current retained delta 已在当前
bounds 外，或最新明确查询证明 current reference/actual safety 不满足，则：

- 旧 active profile 不得继续作为 current safety certificate；
- 若 complete candidate 可作为最新 recovery boundary，允许以
  `SAFETY_REPLACEMENT` 安装；
- `state = SAFETY_PRIORITY`；
- `current_state_admissible=false`；
- 不 reset/clip retained delta；
- 不在 A5-R1 中计算 emergency command。

### 15.5 candidate incomplete/indeterminate，已有 active

若最新 observation 为 UNAVAILABLE/UNKNOWN/构建 incomplete，但没有明确得到新的
current-safe candidate：

- latest candidate 保存失败；
- active profile 几何对象保持不变；
- active epoch 不变；
- active current-validation 标志必须变为 false；
- `state = WAITING_FOR_CANDIDATE`；
- 禁止继续宣称 active profile 已被 latest observation 认证；
- 下一次完整 candidate 可恢复，不得 permanent latch。

### 15.6 配置无效

```text
state = CONFIGURATION_ERROR
no installation
```

不得用普通 source readiness transient 表达 CONFIGURATION_ERROR。

---

## 16. Active epoch 语义

`active_tube_epoch`：

- 初始为 0；
- 第一次 material install 后为 1；
- 只在 active profile 的安全关键内容或 path provenance 实际变化时增加；
- rejected candidate 不增加；
- incomplete candidate 不增加；
- equivalent refresh 不增加；
- control step/query 不增加；
- 不因 candidate build counter 增加；
- 不因单纯 Marker/diagnostics 发布增加；
- 不 reset 或倒退。

`candidate_sequence`：

- 每个 candidate build attempt 增加；
- 可以远高于 active epoch；
- 不得发布或命名成 active tube epoch。

---

## 17. Profile 等价规则

必须实现明确、可测试、finite 的安全关键 profile 等价比较。

等价至少要求：

- 相同 tube source；
- 相同 path source revision；
- 相同 preview start/end（容差内）；
- 相同 sample count；
- 每个 sample 的 `w`、filtered lower/upper、lower_w/upper_w 在容差内；
- 相同 complete/raw/filtered/obstacle-certified 状态。

不得把以下构建计数作为几何不等价依据：

```text
candidate_sequence
TubeProfile::tube_revision
rebuild/reject counters
map observation sequence alone
```

`ROLLING`、`REPLAN_REQUIRED`、`SAFETY_PRIORITY` 等 execution state 由 current
state 和 forward check 决定，不属于 profile 几何身份。若 profile/provenance
完全等价而 execution state 改变，只更新 state/status，不增加 active epoch。

若 path source revision 改变，即使数值 bounds 相同，也必须视为新的 active
provenance，并在成功安装时增加 active epoch。

若只有 `map_observation_sequence` 改变但安全关键 profile 等价：

- active epoch 保持；
- 允许更新“最近等价验证 observation sequence”诊断；
- 不宣称这构成 SDF snapshot/coherence。

---

## 18. Transient 与 failure 语义

TubeEpochManager 不得包含：

```text
permanent failure latch
one-way failed state
manual fallback counter
control selection
```

普通 transient sequence 必须可恢复：

```text
ROLLING
 -> WAITING_FOR_CANDIDATE (temporary unavailable)
 -> ROLLING (next valid equivalent/new candidate)
```

或：

```text
ROLLING
 -> SAFETY_PRIORITY (latest complete boundary excludes current state)
 -> ROLLING/REPLAN_REQUIRED (state becomes admissible again)
```

状态恢复不得 reset active epoch、retained delta 或 candidate sequence。

---

## 19. 原子状态提交

A5-R1 的“原子”指单线程纯 C++ 状态语义：

1. candidate 在局部对象中完整构建；
2. filter 与 hybrid checks 全部完成；
3. 形成一个 installation decision；
4. 最后一次性更新 active profile、epoch、provenance 和 state；
5. 任一早退都不得产生 half-installed active state。

必须用单元测试证明：

- rejected candidate 不改变 active profile；
- rejected candidate 不改变 active epoch；
- equivalent candidate 不改变 active profile identity/epoch；
- material candidate 只增加一次 epoch；
- failed check 不会只更新一半 metadata。

A5-R1 不实现线程同步、atomic shared_ptr 或 mutex。并发安装和 immutable snapshot
属于 A5-R2/A5.3。

---

## 20. Source-specific 语义

### 20.1 NONE

- 不构建 candidate；
- state 为 `NO_ACTIVE_TUBE`；
- 不宣称 fixed/ESDF safety；
- 若 API 支持从 active 切为 NONE，必须明确 active unavailable，但 epoch 计数不得
  倒退或复用。

### 20.2 FIXED

- 复用 existing TubeBuilder/TubeFilter；
- 可安装 active fixed profile；
- `obstacle_certified=false`；
- 只证明 epoch/hybrid/profile 机制；
- 不执行 DistanceQuery；
- 不宣称静态障碍安全。

### 20.3 ESDF

- 只有 finite `KNOWN_FREE` 可建立/验证安全；
- UNKNOWN/OUT_OF_MAP/UNAVAILABLE 不得扩大 bounds；
- required reference/actual clearance 使用 existing erosion 配置；
- current tracking ball 必须检查；
- observation sequence 只是 token，不是 snapshot revision。

---

## 21. 纯数据 diagnostics

本轮不发布 ROS diagnostics，但 manager result/status 必须足够支持 A5-R2 后续发布。

至少保留：

```text
state
install disposition
candidate sequence
active tube epoch
candidate/active path source revision
candidate/active map observation sequence
candidate complete
active available
active current-validation valid
current geometry valid
current bounds valid
current inside
current safety status
tracking error and bound
reference/actual distance and required clearance
certified forward horizon
forward horizon sufficient
retained offset forward-contained
equivalent refresh count
install/reject/wait/safety-priority counts
reason code/string
```

所有数值必须 finite；不可用值用 bool/status 表达，不使用 NaN sentinel。

---

## 22. 单元测试要求

新增 `tube_epoch_manager_test.cpp`，至少覆盖以下测试。

### 22.1 基本安装与 epoch

1. invalid config -> CONFIGURATION_ERROR；
2. first valid fixed candidate -> epoch 1；
3. first valid ESDF candidate -> epoch 1；
4. active profile 与 candidate profile 是独立对象；
5. candidate sequence 单调增加；
6. active epoch 永不倒退。

### 22.2 等价与 material update

7. 完全相同 input 重建，candidate sequence 增加、active epoch 不变；
8. map observation sequence 改变但 profile 等价，active epoch 不变；
9. bounds material change，active epoch 恰好增加 1；
10. path source revision 改变，即使 bounds 数值相同，成功安装后 epoch 增加；
11. build/reject counters 不影响 profile 等价；
12. equivalence tolerance deterministic，non-finite 拒绝。

### 22.3 Candidate/active 解耦

13. complete active 后出现 incomplete candidate，active profile/epoch 保持；
14. latest candidate 保存 incomplete diagnostics；
15. rejected candidate 不能部分覆盖 active metadata；
16. candidate sample 内容修改不会通过可写引用修改 active；
17. source NONE 后 active 不再可用，epoch 不复用。

### 22.4 Hybrid current check

18. retained delta current-inside + geometry valid；
19. retained delta current-outside -> SAFETY_PRIORITY，不 clip；
20. regularity invalid -> no normal install；
21. current reference insufficient clearance；
22. actual UAV insufficient clearance；
23. tracking error over bound；
24. UNKNOWN/OUT_OF_MAP/UNAVAILABLE 不产生 current safety claim；
25. fixed source obstacle_certified=false。

### 22.5 Forward status

26. sufficient horizon + retained delta forward-contained -> ROLLING；
27. short terminal horizon -> install + REPLAN_REQUIRED；
28. current inside but future knot excludes retained delta -> install +
    REPLAN_REQUIRED；
29. exact forward interval end 被检查；
30. forward result 不宣称 PortProjector/C2 dynamic feasibility。

### 22.6 Transient recovery

31. valid -> UNAVAILABLE -> valid 可恢复，无 permanent latch；
32. valid -> incomplete -> equivalent valid 可恢复，epoch 无无意义增加；
33. ROLLING -> SAFETY_PRIORITY -> ROLLING 可恢复；
34. WAITING 时 active current-validation=false；
35. safety-priority 不 reset retained delta。

### 22.7 Atomicity 与确定性

36. failed build 不 half-install；
37. failed current check 不 half-update epoch/provenance；
38. material install 只 commit 一次；
39. repeated sequence 输出 deterministic；
40. 所有公开 status 数值 finite。

测试使用解析 DistanceQuery，不依赖 ROS、bag、真实地图或 wall time。

---

## 23. 现有回归不得修改或减少

A5-R1 禁止修改 existing：

- TubeBuilder tests；
- TubeFilter tests；
- Runtime tests；
- core geometry/matched/projector tests；
- bspline_race adapter/marker tests。

必须证明新增 manager 不改变 legacy Runtime 行为。

当前 `PhaseOffsetRuntime` 仍保留 50 Hz rebuild 和 single profile 问题，这是本轮
刻意保留的 integration gap，不得为了让新测试“更完整”而修改 Runtime。

---

## 24. CMake 与 README

只在 `phase_offset_navigation/CMakeLists.txt`：

- 将 `src/tube_epoch_manager.cpp` 加入 `phase_offset_navigation` library；
- 新增 `phase_offset_tube_epoch_manager_test`；
- 测试只链接 `phase_offset_navigation`、`phase_offset_core` 和 gtest/Eigen 的现有
  最小依赖；
- 不增加 ROS 或 bspline_race 依赖。

README 只追加：

- candidate/active profile 区分；
- candidate sequence 与 active epoch 区分；
- R1 不等于 Runtime/ROS integration；
- `map_observation_sequence` 不等于 coherent map snapshot。

禁止顺手改写 README 的既有理论描述。

---

## 25. 构建与测试顺序

先清除测试结果，避免再次复用旧 XML：

```bash
catkin_make clean_test_results
catkin_make -j8
```

然后至少运行：

```text
phase_offset_core full tests
phase_offset_navigation full tests
bspline_race current guidance/active/matched/marker regressions
path_searching existing test
plan_env existing test
```

最后：

```bash
ctest --test-dir build -N
catkin_test_results --verbose build/test_results
```

要求：

- 新 manager tests 全部通过；
- existing core/navigation tests 不减少；
- adapter/marker 回归不减少；
- 不存在旧 audit/contract/prototype XML；
- 仅允许明确记录既有 `uav_utils` 链接/缺 XML历史项；
- 报告实际执行的 testcase 数，不复用旧 547。

本轮不要求 ROS launch，因为 manager 尚未接入线上链路。

---

## 26. 依赖边界审核

执行：

```bash
rg -n "ros|ros::Time|SDFMap|plan_env|ContinuousPhasePath|B[Ss]pline|Manager|Publisher|Subscriber|Marker|PositionCommand|SO3" \
  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_* \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp

rg -n "Neighbor|Swarm|CBF|pairwise|robot_id|common_msgs|OSQP|CVX|nlopt" \
  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_* \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp

rg -n "MapSnapshot|SafeLiftedTubeSnapshot|snapshot_revision|map_revision|sdf_revision|mutex|atomic|AsyncSpinner|callback_queue" \
  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_* \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp

rg -n "Continuation|continuation|C2|planner|replan publisher|PositionCommand|cmd_pub" \
  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_* \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
```

`REPLAN_REQUIRED` 状态名本身允许，但不得存在 planner 调用、ROS publisher 或 C2
逻辑。每个命中必须解释。

---

## 27. 代码规模与职责审核

执行：

```bash
wc -l \
  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h \
  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp \
  src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
```

要求：

- implementation 建议不超过 500 行；
- types header 只放 epoch/install 数据类型；
- manager header 只放 manager API/ownership；
- manager source 不复制 TubeBuilder/TubeFilter/Geometry 公式；
- 不实现 PortProjector 或 MatchedPort；
- 不含 manual sine profile；
- 不含 ROS diagnostics 组装。

---

## 28. Git 与白名单自审核

结束时执行并保存：

```bash
git diff --check
git status --short
git diff --stat
git diff --name-only
git diff --cached --name-only
git stash list
```

逐项确认：

1. branch/HEAD/stash 未变；
2. 无 staged 内容；
3. 无 commit/branch/tag/push；
4. 只有第 8 节 6 个路径新增/修改；
5. 第 7 节记录的禁止修改文件 SHA-256 全部不变；
6. adapter、manager、Runtime、TubeBuilder、TubeFilter 未修改；
7. launch/RViz/map/参数未修改；
8. A5-R0-r1 状态未回退；
9. prototype stash 和旧 untracked prototype 未触碰；
10. `git diff --check` 通过。

---

## 29. 严格禁止

1. 修改 legacy Runtime；
2. 把 manager 接入 adapter/manager；
3. 新 ROS topic、publisher、diagnostic payload 或 Marker；
4. 修改 83 字段 schema；
5. 修改 100-cycle gate；
6. 修改 TubeBuilder/TubeFilter/PortProjector 数学；
7. 修改 erosion 数值、0.70 m required clearance 或其他 safety 参数；
8. 修改 planner、A*、B-spline、C2；
9. 修改 SDFMap 或实现 map snapshot；
10. 修改速度、K1/K2、governor 或 saturation；
11. reset/clip retained delta；
12. permanent failure latch；
13. neighbor、swarm、CBF、多机消息；
14. 新地图、corridor、circle、figure-eight、split/merge；
15. ROS ESDF 长窗口验收；
16. 把 candidate sequence 命名为 tube epoch；
17. 把 map observation sequence 命名为 map revision；
18. 宣称 R1 已解决 50 Hz rebuild；
19. 宣称 R1 已完成 active Marker 重接；
20. 自动进入 A5-R2/A5.3/A6。

---

## 30. 停止条件

发生以下任一情况立即停止并报告：

- branch/HEAD/stash/precondition 不符；
- R0-r1 残留重新出现；
- 需要修改白名单外文件；
- 需要修改 TubeBuilder/TubeFilter 才能表达 candidate/active；
- 需要修改 Runtime 才能通过 manager unit test；
- current safety check 只能通过具体 SDFMap 完成；
- 无法区分 transient/unsafe/configuration error；
- profile 等价规则需要忽略 path provenance；
- epoch 会因等价 rebuild 持续增长；
- rejected candidate 会覆盖 active；
- 实现需要 mutex/atomic/map snapshot；
- 测试失败需要调整 safety 参数或 planner；
- workspace-wide test 被无关历史缺陷阻塞；
- 实现需要进入 A5-R2/A5.3/A6。

保留工作树并报告，不得扩大范围。

---

## 31. 最终报告格式

最终报告必须包含：

1. branch、HEAD、stash、开始前 status；
2. 实际新增/修改的 6 个白名单路径；
3. candidate profile 与 active profile 的所有权；
4. candidate sequence 与 active epoch 的区别；
5. hybrid current installation checks；
6. forward geometric status 及其非动态含义；
7. installation decision matrix；
8. equivalent refresh 规则；
9. path provenance 与 map observation token 语义；
10. transient recovery、无 permanent latch；
11. rejected candidate 不覆盖 active 的测试；
12. safety-priority 不 reset/clip retained delta；
13. unit test 和全部回归统计；
14. dependency searches；
15. 禁止修改文件的 SHA-256 前后对比；
16. code line counts；
17. `git diff --check`、最终 status/stash；
18. no staged/commit/tag/push；
19. 明确说明未运行 ROS；
20. 明确说明 legacy Runtime/Marker 仍未重接。

最终必须原样包含：

```text
A5-R1 只完成纯 C++ candidate/active TubeEpochManager；未修改 legacy
PhaseOffsetRuntime，未把 manager 接入 ROS，未实现 SDF snapshot/coherence，未进入
A5-R2、A5.3、A6 或后续阶段。A5 继续 blocked。
```

---

## 32. A5-R1 完成后的预期状态

```text
Existing online chain:
  adapter -> legacy Runtime(single profile, 50 Hz rebuild)
  remains unchanged

New tested offline component:
  accepted neutral path preview
    -> TubeEpochManager
      -> latest candidate profile
      -> hybrid installation result
      -> active profile
      -> active tube epoch

No ROS wiring yet
No snapshot/coherence claim
No planner/C2 candidate-path check
A5 remains blocked
```

---

## 33. 给执行者的启动指令

用户后续若决定执行，只发送：

```text
严格执行：
docs/DeepSeek_A5_R1_Pure_CPP_Tube_Epoch_Manager_Prompt_2026-08-09.md

本轮只执行 A5-R1。AUTO_ADVANCE=false。完成纯 C++ TubeEpochManager、单元测试、
回归、自审核和报告后立即停止。禁止修改 legacy Runtime、adapter、manager、
TubeBuilder、TubeFilter、SDFMap、planner 或 C2；禁止进入 A5-R2、A5.3 或 A6。
```
