# DeepSeek A5-R2 多频率 Runtime/ROS 集成执行规范

日期：2026-08-09

工作目录：

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

阶段控制：

```text
DOCUMENT_ROLE=STAGE_EXECUTION_SPEC
CURRENT_AUTHORIZED_STAGE=A5-R2
AUTO_ADVANCE=false
BLOCKED_STAGE=A5
```

本文件是 A5-R2 的独立执行规范。只有用户明确要求严格执行本文件后，才允许修改
第 9 节白名单源码。

本轮完成实现、构建、单测、ROS 回归、自审核和汇报后立即停止。

严禁自动进入：

- A5.3 SDF snapshot/coherence；
- A6 existing C2 周围的 candidate-path tube continuation；
- A7 单机完整闭环；
- 多机、swarm、CBF、复杂场景或论文统计。

---

## 1. 本轮唯一目标

把 A5-R1 已完成并测试的纯 C++ `TubeEpochManager` 接入当前单机 ROS 闭环，形成
明确的多频率数据链：

```text
50 Hz control callback
  |
  +-- 10 Hz or path event:
  |     sample accepted path once
  |     create DistanceQuery
  |     increment map_observation_sequence
  |     TubeEpochManager builds latest candidate
  |     hybrid install -> immutable active profile + active_tube_epoch
  |
  +-- every 50 Hz:
        read immutable active profile only
        query current/next bounds
        PortProjector
        MatchedPort
        integrate final delta
```

本轮必须结束当前 legacy 行为：

- Runtime 每 50 Hz 重建 tube；
- Runtime 同时拥有 builder/filter/map query；
- incomplete candidate 覆盖 active profile；
- Candidate/Certified Marker 读取同一个 profile；
- rebuild counter 被误当 tube revision/epoch；
- 100-cycle gate 决定 tube 是否构建/安装。

本轮不修改 tube 几何、erosion、PortProjector、MatchedPort、planner、C2 或 SDFMap。

---

## 2. 与 proposal 的严格数据流

基础路径与活动参考仍为：

\[
r(w,\delta)=p(w)+N(w)\delta.
\]

tube 仍为规划中心线 lifted geometry 上的 strip：

\[
\underline\delta(w)\le\delta\le\overline\delta(w).
\]

正确的数据流必须是：

```text
accepted ContinuousPhasePath
  -> PathDifferentialState samples
  -> GeometryEvaluator gives p,N,N_w,regularity
  -> TubeBuilder +/-N ray search
  -> TubeFilter
  -> candidate_profile
  -> TubeEpochManager hybrid installation
  -> active_profile + active_tube_epoch
  -> Runtime high-rate bounds query and matched port
```

因此：

- tube 仍基于已经接受的规划轨迹；
- tube 不是 planner，也不修改 B-spline 控制点；
- candidate 是最新构建尝试；
- active 是当前安装给 Runtime 的 immutable profile；
- candidate rejection 不能改变 active object 或 epoch；
- active epoch 不是 control cycle counter 或 rebuild counter。

---

## 3. A5-R1-r1 前置状态

执行前必须确认：

```text
branch = main
HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
stash contains deepseek-phaseoffset-tracked-prototype-2026-08-08
no staged content
```

A5-R1-r1 必须保持：

- `TubeEpochManager` 已存在；
- manager 43/43 tests；
- navigation 65/65 tests；
- candidate/active 独立；
- candidate sequence/active epoch 独立；
- reference/actual indeterminate reason 已修正；
- `update()==true` 不等于 ROLLING/safe 的注释存在；
- protected navigation 旧文件 SHA 与 A5-R1 报告一致；
- no ROS/SDFMap/planner/C2 dependency in TubeEpochManager；
- A5-R0-r1 planner contract 仍未恢复。

若任一前置不符，立即停止，不得同时修 R1 和实施 R2。

---

## 4. 当前必须修复的线上问题

当前 `PhaseOffsetMatchedAdapter::update()` 每个 50 Hz 周期：

1. 调用 `collectSamples()`；
2. 可能调用 `sample_path()` 采样完整 accepted path；
3. 构造 preview；
4. 构造 DistanceQuery；
5. 调用 `PhaseOffsetRuntime::prepare()`；
6. Runtime 调用 `rebuildProfile()`；
7. builder/filter 结果覆盖单一 `profile_`。

当前 `buildMarkers()` 又调用一次 `collectSamples()`，所以在未预填 sampled path 时，
Marker 路径也可能触发 50 Hz 全路径采样。

当前 Candidate/Certified Marker 均读取：

```text
output.tube_profile
```

这不符合 proposal，也不符合 A5-R1 已建立的 candidate/active ownership。

---

## 5. A5-R2 与后续阶段边界

### 5.1 A5-R2 允许

- adapter 内建立 deterministic 10 Hz tube update schedule；
- path event 立即触发 candidate build；
- adapter 持有 `TubeEpochManager`；
- adapter 将 manager active profile 转成 immutable shared ownership；
- Runtime 高频读取 installed active view；
- Runtime 移除 builder/filter/distance query/rebuild state；
- Candidate Marker 接 latest candidate；
- Certified/Active Marker 接 active profile 和 epoch state；
- 新增独立 strict tube-epoch diagnostics topic；
- 保持 manual 83 字段索引和长度不变；
- fixed/ESDF/none/disabled ROS 回归；
- 证明 control rate 与 build rate 解耦。

### 5.2 A5.3 才允许

- 修改 SDFMap；
- 真实 map/SDF revision；
- coherent SDF snapshot；
- 查询事务、map mutex、callback queue；
- 证明一次 build 中所有 query 来自同一 ESDF version；
- `MapSnapshot`、`SafeLiftedTubeSnapshot` 等对象。

本轮只能使用：

```text
map_observation_sequence
```

它只是 adapter 每次 ESDF candidate attempt 的本地序号，不是 map revision。

### 5.3 A6 才允许

- planner/C2 candidate path tube；
- retained delta 对 new path 的 continuation；
- forward offset dynamic reachability；
- 修改 existing path install decision；
- replan publisher/callback；
- 比较 path update 前后 `p,p_w,p_ww,r,r_w`。

A5-R2 的 `REPLAN_REQUIRED` 只作为状态/诊断，不调用 planner。

---

## 6. 开始前必须完整读取

执行者必须完整读取：

```text
AGENTS.md
docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md
docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md
docs/DeepSeek_A5_Single_UAV_Safe_Tube_Runtime_Prompt_2026-08-08.md
docs/DeepSeek_A5_2_ESDF_Readiness_And_Static_Refresh_Isolation_Prompt_2026-08-08.md
docs/DeepSeek_A5_R0_Selective_Rollback_And_Isolation_Prompt_2026-08-09.md
docs/DeepSeek_A5_R1_Pure_CPP_Tube_Epoch_Manager_Prompt_2026-08-09.md
本执行单
```

并完整读取：

```text
phase_offset_navigation 全部源码、README、CMake 和 tests
phase_offset_core port/geometry/matched interfaces and tests
phase_offset_matched_adapter.h/.cpp/test
phase_offset_tube_markers.h/.cpp/test
gvf_manager 中 matched adapter 的构造和 cmdCallback 接入
test_gvf.launch 全部参数和 node 数量
bspline_traj CMake/package
SDFMap public query semantics，只读
```

旧 prototype 只能只读参考，禁止修改或接线。

---

## 7. 开始前记录

保存原始输出：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git diff --cached --name-only
git stash list
find build -name CTestTestfile.cmake -print0 | xargs -0 rg -n "add_test\\(" | wc -l
catkin_test_results --verbose build/test_results
```

记录开始前行数：

```bash
wc -l \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp
```

记录第 10 节保护文件 SHA-256，结束时必须逐项一致。

---

## 8. 目标依赖方向

必须保持：

```text
Eigen/STL
  -> phase_offset_core
  -> phase_offset_navigation
       TubeEpochManager (medium-rate candidate/install)
       PhaseOffsetRuntime (high-rate active execution)
  -> bspline_race integration adapter
       accepted path sampling + SDFMap bridge + ROS publishers
  -> original governor / PositionCommand / SO3
```

Runtime 不得知道：

```text
ROS
SDFMap
DistanceQuery
TubeBuilder object
TubeFilter object/config
candidate build scheduling
map observation sequence generation
publisher/Marker
planner/C2
```

Runtime 允许调用 existing static `TubeFilter::query()` 查询 immutable active profile；
不得持有或调用 filter construction/filtering。

---

## 9. A5-R2 文件白名单

### 9.1 只允许新增

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_epoch_diagnostics.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_epoch_diagnostics.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_diagnostics_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

### 9.2 只允许修改

```text
src/swarm_planner/phase_offset/phase_offset_navigation/README.md
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp

src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

共 13 个白名单路径。

需要修改其他文件时立即停止，不得扩大范围。

---

## 10. 本轮禁止修改并需保存 SHA 的文件

```text
phase_offset_core/**

phase_offset_navigation/distance_query.h
phase_offset_navigation/tube_types.h
phase_offset_navigation/tube_builder.h/.cpp
phase_offset_navigation/tube_filter.h/.cpp
phase_offset_navigation/tube_epoch_types.h
phase_offset_navigation/tube_epoch_manager.h/.cpp
phase_offset_navigation/test/tube_builder_test.cpp
phase_offset_navigation/test/tube_filter_test.cpp
phase_offset_navigation/test/tube_epoch_manager_test.cpp

phase_offset_tube_markers.h/.cpp/test
gvf_manager.h/.cpp
gvf.h/.cpp
ContinuousPhasePath
plan_env/SDFMap
planner/A*/B-spline/C2
governor/PositionCommand/SO3/simulator
RViz/map/config/messages/swarm/CBF
```

Marker helper 已经可以接收任意 TubeProfile，本轮只改变调用者传入的 profile，禁止
为方便而修改 helper。

---

## 11. Adapter 的新状态所有权

`PhaseOffsetMatchedAdapter` 在 manual 模式下新增/拥有：

```text
TubeEpochManager
latest TubeEpochStatus
latest candidate immutable profile pointer
installed active immutable profile pointer
tube update elapsed accumulator
last tube update path revision
map_observation_sequence
cached full path samples for preflight/visualization
last preflight path revision
```

Adapter 不拥有：

```text
delta
previous final port
manual elapsed/profile state
PortProjector formulas
MatchedPort formulas
tube ray/filter formulas
```

这些仍分别属于 Runtime、core 或 manager。

---

## 12. Tube update schedule

新增单一参数：

```text
phase_offset/tube/update_period
```

launch arg：

```text
phase_offset_tube_update_period
```

默认：

```text
0.10 s  # 10 Hz
```

有效范围：

```text
0.05 <= update_period <= 0.10
```

禁止通过改成 control `dt=0.02` 恢复 50 Hz rebuild。

update due 条件：

```text
first manual path context
OR accumulated dt >= update_period
OR accepted path source revision changed
OR tube source changed/cleared
```

要求：

- 使用 input `dt` 的 deterministic accumulator；
- 不创建 ROS timer/thread；
- path event 立即触发，不等待下一个 period；
- due 时只调用一次 `collectSamples()`；
- due 时只调用一次 `TubeEpochManager::update()`；
- `buildMarkers()` 不得再次调用 `sample_path()`；
- candidate sequence 的增长频率不得等于 control cycle 频率；
- fixed 和 ESDF 都走相同 scheduler；
- source NONE 不做 periodic tube build，只在 path event 更新 preflight/cache。

---

## 13. Path sampling cache

当前 `collectSamples()` 生成：

```text
full preflight path
current local preview
```

R2 后：

- 仅在 update due/path event 调用；
- full path 缓存供 manual preflight 和 base/active Marker；
- preview 只供当前 candidate build；
- Marker 每 50 Hz 可以用 cached samples 重算 active line，但不得重新 sample path；
- cached path 只在 path source revision 改变或 sampling 成功时原子替换；
- sampling 失败不得清空上一份 active profile；
- sampling 失败必须产生 incomplete candidate/waiting status，不能假 profile complete。

单元测试必须用 callback counter 证明 50 control calls 不会产生 50 次 sample callback。

---

## 14. Path source revision 与 map observation sequence

保留当前不透明 path source revision：

```text
semantic path identity + start_w + end_w
```

它只表示 accepted path source change，不是 A6 path epoch/continuation 证明。

ESDF candidate attempt 每次 due：

```text
++map_observation_sequence
```

fixed/none 可以保持 map observation sequence 为 0 或最后值，但不得伪造 map
revision。

必须在 diagnostics 中明确：

```text
map_observation_sequence_is_snapshot = false
```

不得新增 `map_revision`、`sdf_revision` 或 snapshot 字段名。

---

## 15. Immutable candidate/active ownership

Adapter 保存：

```cpp
std::shared_ptr<const TubeProfile> candidate_profile;
std::shared_ptr<const TubeProfile> active_profile;
```

规则：

1. 每个 candidate attempt 后更新 candidate pointer；
2. incomplete candidate 也可保存用于 diagnostics，但 Marker helper会 DELETE；
3. active epoch 增加时才创建新的 active profile pointer；
4. equivalent refresh 时 active pointer identity 必须保持不变；
5. rejected candidate 时 active pointer identity 和 active epoch 都保持；
6. source NONE 时 active/candidate pointer 置空，但 epoch counter 不倒退；
7. Runtime 只能得到 `shared_ptr<const TubeProfile>`；
8. Runtime 不得修改 profile samples；
9. 不使用 atomic shared_ptr/mutex；当前 adapter update 在同一 control callback 中顺序执行；
10. 真并发 snapshot 属于后续阶段。

---

## 16. Runtime 配置重构

从 Runtime 配置移除：

```text
TubeBuilderConfig as builder ownership
TubeFilterConfig as filter ownership
stable_rebuild_cycles
```

Runtime 只保留执行需要的 tube 参数，例如等价结构：

```cpp
struct RuntimeTubeExecutionConfig {
  double invariant_gain = 1.0;
  double interior_margin = 0.0;
  double tracking_error_bound = 0.15;
};
```

这些值必须直接来自现有 adapter tube config，不新增重复 ROS safety 参数。

Runtime 仍保留 `TubeSource`，用于 NONE/fixed/ESDF execution policy。

---

## 17. Runtime API 重构

把 full-path preflight 和 high-rate step 分开。

建议接口：

```cpp
struct RuntimePreflightInput {
  PathSamples path;
  Eigen::Vector3d position;
  std::uint64_t path_source_revision;
};

bool refreshPreflight(const RuntimePreflightInput& input);

struct RuntimeInstalledTubeView {
  std::shared_ptr<const TubeProfile> active_profile;
  TubeEpochStatus epoch_status;
};

struct RuntimePrepareInput {
  PathDifferentialState current_path;
  Eigen::Vector3d position;
  RuntimeInstalledTubeView tube_view;
  double dt;
  bool zero_gate_open;
};
```

名称可小幅调整，但必须满足：

- high-rate input 没有 preview path；
- high-rate input 没有 DistanceQuery；
- high-rate input 没有 builder/filter config；
- preflight 只在 accepted path revision 变化时刷新；
- install/update active view 不 reset Runtime state。

---

## 18. Runtime 必须移除的 state

从 `PhaseOffsetRuntime` 移除：

```text
TubeBuilder tube_builder_
TubeFilter tube_filter_
TubeProfile profile_
rebuild_count_
reject_count_ as candidate-build counter
violation_count_ as rebuild/tube-install counter
stable_rebuild_count_
last_source_revision_ for tube build
have_source_revision_ for tube build
rebuildProfile()
DistanceQuery handling
source readiness queries
candidate build/filter diagnostics ownership
```

Runtime 可以保留：

```text
delta_
previous_final_port_
manual profile elapsed/start/completed/recenter state
manual preflight state and its path revision
current immutable active profile pointer/view
step-local geometry/bounds/projection/matched
fatal control result classification
```

普通 epoch WAITING/REPLAN_REQUIRED/SAFETY_PRIORITY 不得写入 permanent failure latch。

---

## 19. Zero gate 与 tube epoch 解耦

100-cycle zero-port gate 只控制：

```text
whether manual matched guidance may be selected
```

它不得控制：

```text
path sampling due
candidate build
candidate marker
active profile installation
active tube epoch
active current validation
```

因此固定 tube 单测/ROS 中允许出现：

```text
gate closed
candidate installed
active epoch > 0
Candidate Marker ADD
Certified/Active Marker ADD if current certificate is valid
selected = false
```

gate 第 100 周期打开后才允许 Runtime matched guidance 接管。

---

## 20. Runtime execution mode

新增纯 C++ Runtime execution enum，至少区分：

```text
NO_TUBE_REQUIRED       # source NONE/A4
NORMAL                 # ROLLING
RECENTERING            # validated REPLAN_REQUIRED
UNCERTIFIED_RECENTER   # transient WAITING with executable retained active profile
BLOCKED                # no executable active or SAFETY_PRIORITY
FATAL_CONTROL_FAILURE  # geometry/kernel/projector/matched invalid
```

名称可调整，但不得只用 `valid`/`failure_latched` 两个 bool 混合所有语义。

---

## 21. Epoch state 到高频执行的决策表

### 21.1 TubeSource NONE

- A4 manual 行为逐项不变；
- gate 后允许平滑 manual profile；
- 不需要 active profile；
- candidate/active Marker DELETE；
- epoch state 为 NO_ACTIVE_TUBE；
- 不能因 manager 没 active 而 invalid。

### 21.2 ROLLING

- active profile 必须 available/complete；
- active current validation 必须 true；
- high-rate current/next bounds 必须 valid；
- high-rate tracking error 必须在 bound；
- gate closed：只计算状态，不 selected；
- gate open：允许 normal manual profile/matched port。

### 21.3 REPLAN_REQUIRED

- active profile/current validation 仍有效；
- Certified/Active Marker 可以保持 ADD；
- 禁止开始新的 excursion；
- 若已经 nonzero/profile active，Runtime 使用 final port 平滑 recenter；
- 若已 centered，保持零 offset matched state；
- 输出 Runtime mode `RECENTERING`；
- 只发布 replan-required 状态，不调用 planner。

### 21.4 WAITING_FOR_CANDIDATE

- active current validation 已失效，因此 Certified Marker DELETE；
- 禁止开始新 excursion；
- 若已有 active profile，且 current retained delta 仍在其 bounds 内，允许
  `UNCERTIFIED_RECENTER`，raw intent 只向 `delta=0` 回收；
- 该 recovery 不得被报告为 obstacle-certified；
- 若 active 不可用/current bounds 不含 retained delta，Runtime `BLOCKED`；
- BLOCKED 不提交 delta/previous port；
- 下一次 valid candidate 可恢复，不 permanent latch。

### 21.5 SAFETY_PRIORITY

- Certified Marker DELETE；
- Runtime 不得把 `update()==true` 误解为 safe；
- 不 reset/clip delta；
- 当前 PortProjector 若无法从 outside set 证明单步可行，则 `BLOCKED`；
- 不在本轮实现 emergency/CBF/replan command；
- ROS acceptance window 中 SAFETY_PRIORITY 事件必须为 0，否则停止。

### 21.6 CONFIGURATION_ERROR / NO_ACTIVE_TUBE（source fixed/esdf）

- no new excursion；
- no certified marker；
- selected=false；
- no partial state commit；
- configuration error 可以由 adapter permanent disable；普通 no-active 不 latch。

---

## 22. High-rate current checks

Runtime 每个 50 Hz step 只允许：

1. evaluate current active geometry；
2. query installed active profile current bounds；
3. compute tracking error `||x-r||`；
4. check current delta inside；
5. after projection query exact next bounds；
6. PortProjector；
7. MatchedPort；
8. final delta integration。

禁止每个 high-rate step：

- query SDFMap；
- call DistanceQuery；
- build candidate；
- filter profile；
- increment candidate sequence/active epoch；
- call full-path sample callback。

reference/actual ESDF clearance 使用最近一次 TubeEpochManager update 的 status，必须
明确它只对应最近 observation sequence。

---

## 23. Runtime state update uniqueness

继续严格保持：

\[
\delta_{k+1}=\delta_k+dt\,u_{\delta,k}^{\star}.
\]

要求：

- 只使用 final projected `u_delta`；
- previous port 只保存 final port；
- projection/MatchedPort/delta 使用同一 final port；
- active epoch install 不 reset/clip delta；
- path revision 不 reset delta；
- transient waiting 不 reset delta；
- blocked/fatal step 不部分提交 delta 或 previous port；
- active pointer replacement 在 Runtime step 前完成，step 内不可变。

---

## 24. Failure latch 语义收缩

Adapter 现有 permanent `failure_latched_` 只允许用于：

- zero-port equivalence 在 gate 后失败；
- current geometry/kernel/projector/MatchedPort 的不可恢复 control invariant failure；
- invalid configuration。

不得用于：

- periodic candidate rejection；
- ESDF unavailable/unknown/out-of-map；
- WAITING_FOR_CANDIDATE；
- REPLAN_REQUIRED；
- equivalent refresh；
- short forward horizon；
- ordinary map observation transient。

Runtime 不再提供 public `latchFailure()` 给 tube readiness 调用。

---

## 25. Candidate/Certified Marker 重新接线

保持 topic 和 namespace 不变：

```text
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/tube
```

调用关系必须变为：

```text
Candidate Marker <- latest candidate_profile
Certified/Active Marker <- installed active_profile
```

Candidate predicate：

```text
manual mode
&& candidate profile geometry displayable
```

Certified/Active predicate 必须由单一 helper/方法生成，并同时驱动 83 字段
`kTubeDisplayCertified` 和 Marker action：

```text
manual mode
&& active profile available/complete
&& active epoch > 0
&& active_current_validation_valid
&& Runtime current geometry/bounds/inside/tracking valid
&& !fatal adapter failure latch
&& epoch state in {ROLLING, REPLAN_REQUIRED}
&& (source FIXED || active obstacle_certified)
```

特别要求：

- 不包含 zero gate；
- 不包含 control selected；
- gate 只控制 guidance，不控制 active certificate；
- WAITING/SAFETY_PRIORITY/CONFIGURATION_ERROR/NO_ACTIVE -> 三 DELETE；
- failure latch 后首个 Marker 周期三 DELETE；
- candidate rejection 不得让 Certified 使用 candidate；
- Marker helper 文件本身不修改。

---

## 26. Base/active/frame Marker

- base/active path 使用 cached full path samples；
- `buildMarkers()` 不再调用 `collectSamples()`；
- active path 仍使用 current Runtime delta；
- invalid cache/geometry 时保持原 DELETE 语义；
- frame Marker 语义和 IDs 不变；
- 不增加第二个 RViz；
- RViz config 不修改。

---

## 27. Manual 83 字段兼容

`ManualDiagnosticIndex` 0-82 的索引和数组长度必须完全不变：

```text
kManualDiagnosticCount == 83
```

禁止追加到 manual array；epoch 语义放独立 topic。

legacy tube 字段改为读取 installed active/execution view，映射必须文档化：

```text
kTubeSource                  active/source config
kTubeSourceReady             active current validation
kTubeRaw/Filtered/Profile    active profile flags
kTubeObstacleCertified       active profile flag
kTubeSourceRevision          active path source revision
kTubeRevision                active profile legacy candidate build sequence;
                             NOT authoritative active epoch
kTubeCurrent/Next bounds     high-rate Runtime active bounds
kTubeReference/Actual        latest epoch observation values
kTubeTrackingNorm            high-rate tracking value
kTubeViolation               epoch state == SAFETY_PRIORITY
kTubeRebuildCount            candidate_sequence compatibility count
kTubeRejectCount             manager reject_count
kTubeViolationCount          manager safety_priority_count
kTubeDisplayCertified        same active marker predicate
```

必须在新 epoch diagnostics 中提供 authoritative active epoch。

`kManualLegacyDiagnosticCount` 仍是 alias/sentinel，不得导出两次。

---

## 28. 独立 tube epoch diagnostics

新增 topic：

```text
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
```

消息仍可使用：

```text
std_msgs/Float64MultiArray
```

但 schema 必须由独立 helper 定义、严格 static_assert、严格单测。

建议固定 49 字段：

| index | field |
|---:|---|
| 0 | schema_version |
| 1 | tube_source |
| 2 | epoch_state |
| 3 | install_disposition |
| 4 | reason |
| 5 | candidate_sequence |
| 6 | active_tube_epoch |
| 7 | candidate_path_source_revision |
| 8 | active_path_source_revision |
| 9 | candidate_map_observation_sequence |
| 10 | active_map_observation_sequence |
| 11 | map_observation_is_snapshot (=0) |
| 12 | candidate_raw_complete |
| 13 | candidate_filtered_complete |
| 14 | candidate_complete |
| 15 | active_available |
| 16 | active_current_validation_valid |
| 17 | current_geometry_valid |
| 18 | current_bounds_valid |
| 19 | retained_delta_current_inside |
| 20 | current_state_admissible |
| 21 | current_safety_status |
| 22 | reference_clearance_sufficient |
| 23 | actual_clearance_sufficient |
| 24 | tracking_within_bound |
| 25 | forward_horizon_sufficient |
| 26 | retained_offset_forward_contained |
| 27 | dynamic_feasibility_evaluated (=0) |
| 28 | retained_delta |
| 29 | tracking_error_norm |
| 30 | tracking_error_bound |
| 31 | reference_signed_distance |
| 32 | actual_signed_distance |
| 33 | required_reference_clearance |
| 34 | required_actual_clearance |
| 35 | certified_forward_w |
| 36 | equivalent_refresh_count |
| 37 | install_count |
| 38 | reject_count |
| 39 | wait_count |
| 40 | safety_priority_count |
| 41 | candidate_sample_count |
| 42 | active_sample_count |
| 43 | active_profile_complete |
| 44 | active_obstacle_certified |
| 45 | active_display_certified |
| 46 | tube_update_due_this_cycle |
| 47 | runtime_execution_mode |
| 48 | control_selected |

字段名可小幅调整，但数量、单一 alias 规则和核心语义不得减少。

所有值 finite；enum 用整数 double；不可用值用 bool/status 表达，禁止 NaN。

---

## 29. Diagnostics helper 边界

`phase_offset_tube_epoch_diagnostics.*`：

- 只依赖 STL、navigation types 和 adapter-facing plain data；
- 不 include ROS publisher、SDFMap、gvf_manager；
- 不拥有状态；
- 不决定 control；
- 只把 status/profile/runtime summary 映射为 fixed array；
- header/row/payload test 严格 49；
- 不复制 tube display predicate；adapter 先计算 bool 再传入 helper。

---

## 30. Launch 参数

在 `test_gvf.launch`：

新增：

```xml
<arg name="phase_offset_tube_update_period" default="0.10" />
<param name="phase_offset/tube/update_period"
       value="$(arg phase_offset_tube_update_period)" />
```

移除已经失去架构含义的：

```text
phase_offset_tube_stable_rebuild_cycles arg
phase_offset/tube/stable_rebuild_cycles param
```

必须保持：

```text
phase_offset_mode default=disabled
phase_offset_manual_tube_source default=none
sdf_map_buffer_refresh_period default=3.0
gvf/circle_test/enable=false
gvf/circle_test/auto_start=false
```

不得增加新的 launch/node/RViz/map。

---

## 31. Runtime 单元测试

更新 `runtime_test.cpp`，既有 A4/A5 数学回归不得减少。

至少覆盖：

1. source NONE 与 A4 final-port/delta 行为等价；
2. Runtime class 不再持有 builder/filter/rebuild members；
3. high-rate input 没有 DistanceQuery/preview；
4. preflight 只在 explicit refresh 调用；
5. active profile immutable view 可执行；
6. gate closed 仍评估 active geometry/bounds，但不 selected；
7. gate open ROLLING 正常 selected；
8. active epoch install 不 reset delta；
9. active pointer replacement 不 reset previous final port；
10. equivalent status refresh 不改变 state；
11. rejected candidate 不影响 Runtime active pointer；
12. REPLAN_REQUIRED 不开始新 excursion；
13. REPLAN_REQUIRED active excursion 平滑 recenter；
14. WAITING before start 不 latch；
15. WAITING with executable active profile only recenter；
16. WAITING recovery to ROLLING；
17. SAFETY_PRIORITY blocked、delta 不 clip；
18. CONFIGURATION_ERROR blocked；
19. blocked step 不提交 delta/previous port；
20. high-rate tracking over bound deletes executable certificate；
21. current/next exact bounds；
22. same final port drives physical/phase/delta；
23. matched residual numerical precision；
24. fatal geometry/projector/matched classification；
25. no permanent latch for epoch transient；
26. deterministic。

---

## 32. Adapter/多频率单元测试

现有 adapter tests 必须更新并保持 A3/A4 回归。

新增 focused `phase_offset_tube_epoch_integration_test.cpp`，至少覆盖：

1. 50 control cycles at 0.02 s, update period 0.10 s -> candidate attempts about 10，
   not 50；
2. sample callback count equals update attempts/path events，not Marker cycles；
3. first path context forces immediate build；
4. path source revision forces immediate build；
5. map observation sequence only increments per ESDF attempt；
6. fixed does not query SDFMap；
7. manager builds before zero gate；
8. active epoch can be installed before gate while selected=false；
9. gate opens only cycle 100；
10. gate opening does not increment active epoch；
11. candidate pointer updates on candidate attempt；
12. active pointer only changes on material epoch；
13. equivalent refresh preserves active shared_ptr identity；
14. rejected candidate preserves active pointer/epoch；
15. candidate and active geometry can differ；
16. Candidate Marker uses candidate；
17. Certified Marker uses active；
18. gate closed does not itself delete valid active marker；
19. WAITING causes Certified DELETE but Candidate may remain ADD；
20. SAFETY_PRIORITY causes Certified DELETE；
21. failure latch first cycle DELETE；
22. source NONE keeps A4 behavior；
23. manual diagnostics exactly 83；
24. epoch diagnostics exactly 49；
25. no control publisher；
26. no sample callback in buildMarkers；
27. path update does not reset delta；
28. active install does not reset/clip delta；
29. Runtime and Marker see same active epoch/status；
30. all values finite。

---

## 33. Existing adapter tests 调整原则

必须保留：

- A3 active zero-port；
- gate 1-99 closed / 100 opens；
- A4 smooth bilateral manual profile；
- final port delta integration；
- matched residual；
- 2.5D z semantics；
- five-state DistanceQuery mapping；
- Marker geometry/IDs/ribbon；
- 83 field legacy index order。

必须更新旧语义：

- `CandidateAddsBeforeGateWhileCertifiedDeletes` 不再成立；
- gate closed 但 active profile 已安装且 current certificate valid 时，Certified
  可以 ADD，selected 仍 false；
- Candidate/Certified 不再保证同 sample geometry；
- candidate rejection 不再令 active Marker 跟随删除；只有 active validation/state
  决定 Certified；
- old `tube_profile` single object assertion 必须删除。

禁止为了让旧测试通过而继续共用 profile。

---

## 34. Diagnostics helper tests

至少覆盖：

1. static_assert count=49；
2. all enum/status mappings；
3. candidate/active sample counts；
4. active epoch authoritative；
5. map observation snapshot flag always 0；
6. Runtime mode；
7. display certificate；
8. selected/gate independent；
9. all numbers finite；
10. no alias duplicate；
11. exact payload length；
12. deterministic。

---

## 35. Marker helper 回归

现有 `phase_offset_tube_markers` 10 个测试必须原样通过。

本轮不修改 helper；adapter/integration tests 证明传入对象已经分离。

---

## 36. CMake

只在 `bspline_traj/CMakeLists.txt`：

- 将 `phase_offset_tube_epoch_diagnostics.cpp` 加入 matched adapter 所属 library；
- 新增 diagnostics test target；
- 新增 focused integration test target；
- 使用现有最小依赖；
- 不连接 audit/contract/prototype targets；
- 不新增 package dependency。

navigation CMake 不需要修改；manager 已在 library 中。

---

## 37. 构建与测试顺序

先：

```bash
catkin_make clean_test_results
catkin_make -j8
```

然后至少运行：

```text
phase_offset_core full
phase_offset_navigation full
bspline_race full
path_searching existing
plan_env existing
```

必须明确记录：

- manager 43 tests 不减少；
- Runtime tests；
- adapter existing tests；
- new integration tests；
- diagnostics tests；
- marker 10 tests；
- guidance/active/shadow/kernel tests；
- clean XML only；
- actual testcase count；
- existing uav_utils exception if encountered。

最后：

```bash
find build -name CTestTestfile.cmake -print0 | xargs -0 rg -n "add_test\\(" | wc -l
catkin_test_results --verbose build/test_results
```

---

## 38. ROS 隔离规则

运行前：

1. 检查用户 ROS master/process；
2. 不连接/终止用户图；
3. 使用新的隔离端口；
4. 只清理由本轮启动的进程；
5. 保存 owned PID 清单；
6. 结束确认端口释放；
7. 地图只用原 `pillar.pcd`；
8. 不修改目标、速度、增益、安全 margin、refresh period 来规避失败。

每组都保留原始 bag、CSV、JSON、分析器和 SHA-256。

---

## 39. ROS-0 disabled baseline

裸 `test_gvf.launch`：

```text
mode=disabled
tube source=none
update period=0.10
refresh=3.0
circle flags=false
```

验收：

- manual diagnostics/epoch diagnostics/candidate/active Marker 无 publisher；
- `/position_cmd` 唯一 publisher `/formation_planning`；
- original pillar point-to-point `(8,0,0)` 或等价已验证目标到达；
- planner/governor/SO3 正常；
- no phase-offset control side effect。

---

## 40. ROS-1 manual/none A4 回归

显式：

```text
mode=manual
tube_source=none
```

验收：

- cycle 1-99 gate closed，100 opens；
- nonzero smooth manual profile；
- same final port；
- delta recurrence exact；
- matched residual <=1e-10；
- manual diagnostics every frame exactly 83；
- epoch diagnostics every frame exactly 49，state NO_ACTIVE_TUBE；
- candidate/certified three DELETE；
- no failure/fallback；
- `/position_cmd` unique publisher；
- A4 result unchanged。

---

## 41. ROS-2 fixed multirate/epoch 回归

显式：

```text
mode=manual
tube_source=fixed
tube_update_period=0.10
```

必须记录启动前至 profile 完成和静态 hold。

验收：

- candidate/active epoch 在 gate 前可以建立；
- gate 1-99 selected=false；
- gate 100 后才 selected=true；
- gate opening 不改变 active epoch；
- candidate sequence 约 10 Hz，明显低于 50 Hz control；
- sample callback 次数与 candidate attempts 同量级；
- fixed active obstacle_certified=false；
- Candidate Marker 读取 candidate；
- Certified Marker 读取 active；
- valid active marker 与 gate 独立；
- manual 83 / epoch 49 strict；
- delta 始终由 final port 更新；
- no reset/clip at install；
- matched residual <=1e-10；
- failure/fallback/safety-priority=0；
- profile 完成并回中；
- 到达目标。

### 41.1 15 s stable epoch window

在 accepted path、current phase 和 fixed profile 几何稳定的静态 hold 窗口中：

- 连续至少 15 s；
- candidate sequence 继续按约 10 Hz 增加；
- active_tube_epoch 不变化；
- active shared profile geometry 不变化；
- Marker 不闪烁；
- control selected/centered 状态可记录但不作为 epoch identity。

若 current phase 仍明显推进导致 preview material change，不得把该窗口冒充静态
epoch window；应等待真正静态 hold 或报告 inconclusive。

---

## 42. ROS-3 ESDF multirate integration

显式：

```text
mode=manual
tube_source=esdf
tube_update_period=0.10
refresh=3.0
```

使用原 pillar 地图和 A4 已验证安全目标，不改 safety 参数。

验收：

- DistanceQuery 只在 candidate update due 路径调用；
- map_observation_sequence 约 10 Hz，不是 control rate；
- epoch diagnostics snapshot flag=0；
- candidate profile 与 active profile 可独立观察；
- rejected/incomplete candidate 不覆盖 active pointer/epoch；
- active install 原子增加 epoch；
- equivalent refresh 不增加 epoch；
- current valid window state 仅 ROLLING/REPLAN_REQUIRED；
- acceptance window WAITING/SAFETY_PRIORITY/CONFIGURATION_ERROR=0；
- active certificate false 后首个 Marker 周期三 DELETE；
- no stale Certified ADD；
- high-rate tracking/current/next bounds valid；
- reference/actual clearance 满足 existing config；
- delta no reset/clip；
- matched residual <=1e-10；
- manual 83 / epoch 49 strict；
- no failure/fallback；
- UAV 到达。

### 42.1 静态 epoch 观察

在目标到达后或其他合法的 static path/current-phase hold 中，尝试取得 15 s：

- candidate sequence 继续更新；
- equivalent candidate 不增加 active epoch；
- active profile/Marker 不闪烁。

若当前 local SDF 无 coherent snapshot 导致无法取得稳定窗口，只能报告：

```text
A5-R2 integration complete but ESDF coherence remains blocked for A5.3
```

禁止将 refresh 改为 0、修改地图或缩小 margin 伪造稳定。

---

## 43. ROS 原始证据与分析器

bag 至少记录：

```text
/formation_planning/phase_offset_manual/diagnostics
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/tube
/position_cmd
/sim/odom
/move_base_simple/goal
/rosout
```

分析器必须：

- manual payload strict 83；
- epoch payload strict 49；
- Marker IDs 0/1/2 同步；
- diagnostics/Marker 按 bag message time 对齐；
- 报告最大配对间隔；
- candidate sequence rate；
- active epoch change times；
- gate transition；
- selected state；
- delta recurrence；
- matched residual；
- candidate/active Marker action；
- stale certificate mismatch count；
- state counts；
- static 15 s window；
- output CSV header/row exact；
- analyzer SHA-256；
- bag/CSV/JSON SHA-256。

不得复用旧 A5.2 CSV exporter。

---

## 44. Disabled/fixed/ESDF 结果边界

A5-R2 可以声明：

- candidate/active ownership 已接入；
- 50 Hz control 与 10 Hz build 解耦；
- active epoch/Marker/Runtime 一致；
- fixed/ESDF integration 行为得到证据；
- transient 不再直接永久 latch；
- legacy Runtime 50 Hz rebuild 已移除。

A5-R2 不得声明：

- coherent SDF snapshot 已实现；
- map revision 与 tube epoch 原子绑定已证明；
- A5.3 完成；
- C2/path continuation 完成；
- SAFETY_PRIORITY recovery 理论完成；
- A5/A6/A7 已全部验收。

因此本轮结束后 A5 仍 blocked。

---

## 45. Dependency boundary audit

执行：

```bash
rg -n "DistanceQuery|TubeBuilder|TubeFilter[[:space:]]+[a-zA-Z_].*_;|rebuildProfile|stable_rebuild|map_observation" \
  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp

rg -n "ros|SDFMap|plan_env|ContinuousPhasePath|Publisher|Subscriber|Marker|PositionCommand|SO3" \
  src/swarm_planner/phase_offset/phase_offset_navigation

rg -n "Neighbor|Swarm|CBF|pairwise|robot_id|common_msgs|OSQP|CVX|nlopt" \
  src/swarm_planner/phase_offset/phase_offset_navigation \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_epoch_diagnostics.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_epoch_diagnostics.cpp

rg -n "MapSnapshot|SafeLiftedTubeSnapshot|snapshot_revision|map_revision|sdf_revision|mutex|atomic|AsyncSpinner|callback_queue" \
  src/swarm_planner/phase_offset/phase_offset_navigation \
  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

rg -n "PositionCommand|cmd_pub|publishGovernorPositionCommand|SO3|uav_cmd" \
  src/swarm_planner/bspline_traj/include/bspline_race/integration \
  src/swarm_planner/bspline_traj/src/integration
```

合理的 static `TubeFilter::query` 命中允许，但 Runtime 不得持有 filter object。

---

## 46. 公式唯一性与代码规模

执行：

```bash
wc -l \
  src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_epoch_diagnostics.cpp

rg -n "r_w.*u_w|N.*u_delta|matched_residual|delta.*dt|sin\\(|cos\\(" \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp \
  src/swarm_planner/bspline_traj/src/integration \
  src/swarm_planner/phase_offset/phase_offset_navigation/src
```

要求：

- Runtime implementation <=500 行；
- matched adapter implementation <=500 行；
- diagnostics helper focused；
- gvf_manager 行数和 diff 不增加；
- ISF formula 仍只有 Kernel；
- MatchedPort formula 仍只有 core；
- manual profile 只在 Runtime；
- no duplicate delta integration；
- no tube builder/filter formulas in adapter。

若 adapter 无法在 500 行内完成，停止并申请新增单一职责 helper，禁止擅自扩大
白名单或压缩成不可读单行。

---

## 47. Git 与白名单自审核

结束时：

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
2. no staged/commit/tag/push；
3. 只修改/新增第 9 节 13 个路径；
4. 第 10 节 protected SHA 前后一致；
5. manager 43 tests/source 未修改；
6. marker helper 未修改；
7. gvf_manager/gvf/planner/SDFMap/C2 未修改；
8. launch baseline defaults 保持；
9. no audit/contract/prototype CMake targets；
10. user other dirty state 未触碰；
11. `git diff --check` 通过；
12. ROS ports released、user processes untouched。

---

## 48. 严格禁止

1. 修改 TubeEpochManager/R1 tests；
2. 修改 TubeBuilder/TubeFilter 数学；
3. 修改 PortProjector/MatchedPort/core；
4. 修改 gvf_manager 或增加 manager lines；
5. 修改 planner/A*/B-spline/C2；
6. 修改 SDFMap 或实现 snapshot；
7. map observation counter 命名为 revision；
8. 50 Hz candidate build；
9. buildMarkers 中重新 sample path；
10. candidate rejection 覆盖 active；
11. gate 控制 candidate/active installation；
12. Candidate/Certified 共用同一个 profile；
13. Runtime 保存 DistanceQuery/builder/filter object；
14. transient readiness permanent latch；
15. delta reset/clip；
16. 修改 safety margin、0.70 m clearance、speed、K1/K2 或 saturation；
17. 修改 refresh=3.0 规避；
18. 新地图/场景；
19. ROS timer/thread/mutex/atomic snapshot；
20. replan publisher、ContinuationChecker、A6；
21. swarm/CBF/multi-UAV；
22. 修改 83 字段长度；
23. 复用旧 CSV exporter；
24. 自动进入 A5.3/A6/A7。

---

## 49. 停止条件

发生任一立即停止：

- precondition/R1 SHA 不符；
- 需要修改白名单外文件；
- Runtime 无法移除 50 Hz build 而不改 core；
- adapter 需要改 gvf_manager；
- adapter >500 行且需要新增未授权 helper；
- fixed regression 失败；
- manual none A4 不等价；
- candidate build rate 仍接近 50 Hz；
- buildMarkers 仍调用 sample path；
- rejected candidate 改变 active pointer/epoch；
- active install reset/clip delta；
- transient 仍 permanent latch；
- Marker 与 diagnostics certificate 不一致；
- manual/epoch schema 长度错误；
- ESDF acceptance 出现 SAFETY_PRIORITY/failure/fallback；
- 15 s static epoch window 无法取得；
- 需要 refresh=0、改地图/目标/safety 参数；
- 实现需要 SDF snapshot/coherence；
- ROS unavailable；
- workspace-wide test 被新无关问题阻塞；
- 需要进入 A5.3/A6。

保留工作树并报告，不得扩大范围。

---

## 50. 最终报告格式

最终报告必须包含：

1. branch/HEAD/stash/start status；
2. 13 个白名单实际变更；
3. adapter/Runtime/manager ownership；
4. 10 Hz schedule 和 path event；
5. sample callback count vs control count；
6. candidate sequence vs active epoch；
7. immutable active pointer identity tests；
8. rejected candidate preservation；
9. Runtime removed state list；
10. zero gate decoupling；
11. epoch-to-execution decision table；
12. transient/no permanent latch；
13. delta/previous final port uniqueness；
14. Candidate/Certified Marker source；
15. certified predicate；
16. manual 83 and epoch 49 schema；
17. unit/regression test totals；
18. ROS-0/1/2/3 raw evidence；
19. build rate and epoch stability metrics；
20. matched residual/tracking/clearance；
21. Marker/diagnostic alignment；
22. dependency/formula/line audit；
23. protected SHA；
24. git/status/stash/process cleanup；
25. explicit A5.3 limitation。

最终必须原样包含：

```text
A5-R2 只完成多频率 candidate/active TubeEpochManager 与 legacy Runtime/ROS
集成；未修改 SDFMap，未实现 coherent SDF snapshot/revision，未完成 A5.3，未进入
A6、A7 或多机阶段。A5 继续 blocked。
```

---

## 51. A5-R2 完成后的预期状态

```text
50 Hz control:
  current path state
  + immutable installed active profile
  -> Runtime bounds/projector/matched/delta

10 Hz/path event:
  accepted path sampling
  + DistanceQuery
  + map_observation_sequence (not snapshot)
  -> TubeEpochManager candidate/install

Candidate Marker <- latest candidate
Certified Marker <- installed active + current certificate
Execution state <- separate 49-field diagnostics
Manual compatibility <- unchanged 83 fields

No 50 Hz rebuild
No candidate overwrite of active
No planner/C2/SDFMap changes
A5 still blocked by A5.3/A6 boundaries
```

---

## 52. 给执行者的启动指令

用户明确授权后，只发送：

```text
严格执行：
docs/DeepSeek_A5_R2_Multirate_Runtime_ROS_Integration_Prompt_2026-08-09.md

本轮只执行 A5-R2。AUTO_ADVANCE=false。完成多频率 Runtime/ROS 集成、单元测试、
fixed/ESDF 隔离 ROS 回归、自审核和报告后立即停止。禁止修改 TubeEpochManager、
TubeBuilder、TubeFilter、phase_offset_core、gvf_manager、SDFMap、planner 或 C2；
禁止进入 A5.3、A6、A7 或多机阶段。
```
