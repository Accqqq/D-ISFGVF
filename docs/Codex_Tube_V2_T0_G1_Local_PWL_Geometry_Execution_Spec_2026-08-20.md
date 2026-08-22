# Tube V2 T0--G1 Current Baseline and Local-PWL Geometry Execution Spec

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_T0_G1
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=T0_TO_G1_TO_VALIDATION_ONLY
EXECUTOR=LUNA_MAX_SINGLE_AGENT
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、调度、T0/G1 gate 审核、独立复核和最终验收；
**Luna max 单代理**负责执行。Luna max 不得调用、创建、委派或请求任何子代理。  
前置：A6-P2 已完成；manager `48/48`、adapter `82/82`、规定 CTest `10/10`、
全量 `catkin_make -j2` 与 initial-offset dynamic liveness 已通过。

## 0. 阶段目标

保留 planner-authoritative clearance、zero-connected normal Tube、唯一
`CertifiedTubeBuilder` production pipeline、SurfaceValidator 和 Runtime exact-PWL
动态约束；只移除 Filter 对固定 `boundary_slope_max=0.80` 的全 preview 几何硬门槛：

```text
raw current-containing geometric samples
-> cell-local conservative PWL representation
-> existing continuous SurfaceValidator
-> existing Epoch / Runtime / H2
```

本阶段不实现 backward viability，不改 Runtime、planner、H2、launch 或参数。

## 1. 必须保留的既有合同

1. Production cross-section 与 planner 共用 `planning/safe_distance`；margin 不重复扣。
2. normal Tube 只保留包含 `delta=0` 的 connected component；不跨 disconnected branch。
3. neutral planner 不等待 Tube；Tube 只授权 offset，不成为导航 gate。
4. `TubeBuilder -> TubeFilter -> TubeSurfaceValidator -> CertifiedTubeBuilder ->
   TubeEpochManager` 仍是唯一 production candidate pipeline。
5. Runtime 的 one-sided PWL、exact crossed-knot containment、`U+ -> U_safe` 和
   refresh-horizon witness 不改。
6. A6-P2 的 Pair `COMMITTED -> selected -> nonzero`、pending activation/H2 保护不退化。

当前 `tube_filter.cpp` 仍用 fixed `L=boundary_slope_max` 做 whole-range forward/backward
envelope、枚举 anchored `[first,last]`，并会产生 slope-induced `REGULARITY` 截短。G1
只删除这层 geometry/viability 耦合。

历史 `45/46` 与 slope p90 `2.05/2.40` 来自 2026-08-12 A5R-0B，不能冒充当前基线。
A6-P2 Run 4 还证明：raw current anchors 可完整时，Runtime 仍可能短暂出现 `U+` 与
`U_safe` joint-port polygon 同时为空，随后又恢复 selected；这属于 dynamic viability，
不得回写成 geometry empty。

## 2. T0：最终源码的 before baseline

T0 只读，产物只放 `/tmp`。

### 2.1 身份冻结

记录：

```text
git rev-parse HEAD
git status --short
sha256sum:
  tube_types.h, tube_filter.h/cpp, tube_surface_validator.cpp,
  certified_tube_builder.cpp, tube_epoch_manager.cpp,
  phase_offset_runtime.cpp, phase_offset_matched_adapter.cpp, gvf_manager.cpp
```

工作区高度脏；禁止 reset/clean/checkout/stash/restore/rebase，保留全部用户资产。

### 2.2 静态快照

输出 `static_model_snapshot.txt`，确认：

- `boundary_slope_max` 参数/default/launch 值；
- Filter configuration validity、global envelope、range enumeration、slope assertion；
- Filter 是否写 `first_truncated_reason=REGULARITY`；
- Validator current-connected cell 截取；
- Runtime 从 PWL endpoints 重建 slope；
- production clearance 只用 `planning/safe_distance`；
- production candidate 只经 `CertifiedTubeBuilder`。

### 2.3 当前动态 capture

固定配置：

```text
phase_offset_esdf_tube_single.launch
pillar.pcd
goal=(8,0,1), existing internal target=(8,0,2)
phase_offset_manual_observe_only:=false
all other parameters unchanged
```

使用 fresh private master、fresh `ROS_HOME`、独立 evidence dir。启动前确认端口空闲；
记录 launcher/recorder PID/PGID；只清理该 run；final process/port check 必须为空。

至少一次有效 run。若 source-current raw cohorts 少于 30，允许第二次完全同配置重复 run；
不得延迟 goal、改 goal、减速或改 timer 获取样本。

### 2.4 T0 报告字段

尽现有诊断能力逐 cohort 报告：

- run identity、path/source revision、candidate/build/map sequence；
- raw current anchor found/complete；
- raw sample count、requested/retained preview；
- Filter complete/classification/certified segment；
- truncated before/after、first w/reason、certified forward；
- Validator attempted/complete/truncated；
- Candidate/Active/current-validation/selected；
- identity 能正确关联时才报告 Runtime-ready/first selected latency；marker/DUE 不得替代。

若现有 raw schema 可恢复相邻 bounds，计算 lower/upper raw slope count/p50/p90/p99/max；
否则明确写 `CURRENT_SCHEMA_CANNOT_RECONSTRUCT_RAW_ADJACENT_SLOPES`，不得新增 ROS 字段。

T0 输出示例：`/tmp/tube_v2_t0_20260820_<id>/`，包含 source identity、manifest、
before summary、bags/logs 与 cleanup proof。

只要 current raw anchor 可建立且 fixed-slope Filter 仍在 production 路径，自动进入 G1。
若 production 已不使用该 Filter，或 margin/Builder authority 已破坏，则停止报告。

## 3. G1：cell-local conservative PWL

### 3.1 精确 Filter 算法

输入严格递增 knots `w_0 < ... < w_n` 与完整有限 raw intervals：

```text
I_k = [raw_lower_k, raw_upper_k]
raw_lower_k <= raw_upper_k
```

输出：

```text
filtered_lower_k = raw_lower_k
filtered_upper_k = raw_upper_k
lower_w_k = (lower_{k+1}-lower_k)/(w_{k+1}-w_k)
upper_w_k = (upper_{k+1}-upper_k)/(w_{k+1}-w_k)
```

内点继续线性插值；interior knot 返回右单侧导数；最终 knot 返回左单侧导数。corner 合法，
不要求 global C1。不得 global intersection、经验平滑或远端 bottleneck 反向收缩。

### 3.2 必须删除的旧机制

从 production Filter 删除，而不是留 dead code：

- global exact L-Lipschitz forward/backward envelope；
- anchored `[first,last]` enumeration；
- `abs(slope) <= boundary_slope_max` validity assertion；
- slope-induced Filter truncation；
- Filter 自己生成的 `TubeStopReason::REGULARITY`。

`boundary_slope_max` 字段和 ROS/launch 参数暂留作 compatibility/roughness diagnostic，
但不得影响 configuration validity、success、samples、preview、bounds、slopes 或 classification。

### 3.3 输入拒绝条件

仍拒绝：raw incomplete/empty、少于两个 knots、unordered/duplicate/nonfinite `w`、
sample incomplete、nonfinite bounds、`lower>upper`、nonfinite current、缺少 A6 exact current
anchor。不得用 nearest knot 替代 current，不改 structural seam 优先级。

### 3.4 成功输出

- 保留 Builder 提供的全部 current-containing samples；
- 不新增 Filter truncation；保留 Builder 已有 truncation provenance；
- 设置 filtered bounds、one-sided slopes、`filtered_contains_zero`；
- 重算 `diagnostics.min_width`；
- `filtered_complete=true`、`complete=true`；
- 不改 raw bounds/raw_build_samples/snapshot/revision/current anchor。

### 3.5 连续安全与动态边界

ESDF nonzero profile 必须继续过现有 immutable-snapshot SurfaceValidator。不得放宽 cover、
subdivision、query limit、clearance、UNKNOWN/OUT_OF_MAP 或 regularity。Validator 只保留包含
current anchor 的连续成功 cells；zero-only baseline 不变。

G1 不新增 viability 类，不改 Runtime/Projector/QP。陡峭 geometry 若无动态 witness，继续由
`U+ -> U_safe -> certificate denial` 处理，不能重新缩窄 geometry 掩盖。

## 4. 文件白名单

允许：

```text
docs/Codex_Tube_V2_T0_G1_Local_PWL_Geometry_Execution_Spec_2026-08-20.md
docs/Codex_Tube_V2_T0_G1_Local_PWL_Geometry_Self_Audit_2026-08-20.md
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_filter.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
/home/cxq/ISF-GVF/handoff.md
```

优先只改 `tube_types.h` comment、`tube_filter.cpp`、`tube_filter_test.cpp`。

禁止修改：Builder/Validator/CertifiedBuilder/EpochManager/Runtime/Core production 源码、
全部 `bspline_traj` 源码、planner、map、simulator、launch、CMake、package、ROS schema、
Paper proposal。若需要白名单外修改，停止报告。

## 5. 必须覆盖的测试

### 5.1 TubeFilter

1. high slope `>0.80` 不截短，filtered endpoints 等于 raw，query slope 为真实 local slope；
2. distant bottleneck 不收缩 earlier cells，不缩短 preview；
3. PWL corner 的 right/left one-sided derivative 正确；
4. `boundary_slope_max` 为 `0.80`、极小正值、0、负值时 geometry 相同；
5. Filter 不发明 `REGULARITY` truncation，保留 Builder reason；
6. dense queries 等于 raw endpoint PWL interpolation 且 `lower<=upper`；
7. unordered/duplicate/incomplete/nonfinite/empty/missing-anchor/single-knot fail-closed；
8. zero-connected/zero-only 与 `filtered_contains_zero` 一致。

旧 slope-cap/global-envelope 测试必须改为新 oracle，不得只删除。

### 5.2 Validator/Certified/Epoch

- high-slope but continuously safe ribbon 通过；
- endpoints safe、between-knot unsafe 仍拒绝/截取；
- UNKNOWN/OUT_OF_MAP/query limit/current-anchor-cell failure 仍 fail-closed；
- distant unsafe cell 由 Validator 截 current-connected segment，不由 Filter global shrink；
- zero-only fallback/classification/current-delta/forward horizon 不变。

### 5.3 全回归

重跑 Runtime exact-PWL/alternating-corridor、Tube cross-section/builder、manager 48、adapter 82、
continuous path 与 A6 structural seam/off-grid current anchor 回归。

## 6. 构建和静态验收

```text
catkin_make -j2 phase_offset_tube_filter_test
./devel/lib/phase_offset_navigation/phase_offset_tube_filter_test

catkin_make -j2 \
  phase_offset_tube_surface_validator_test \
  phase_offset_certified_tube_builder_test \
  phase_offset_tube_epoch_manager_test \
  phase_offset_runtime_test

catkin_make -j2 gvf_switch_policy_test phase_offset_matched_adapter_test
./devel/lib/bspline_race/gvf_switch_policy_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test

cd build
ctest --output-on-failure -R \
'(_ctest_phase_offset_navigation_gtest_phase_offset_tube_(cross_section|builder|filter|surface_validator)_test|_ctest_phase_offset_navigation_gtest_phase_offset_(certified_tube_builder|runtime|tube_epoch_manager)_test|_ctest_bspline_race_gtest_(gvf_switch_policy|continuous_phase_path|phase_offset_matched_adapter)_test)$'
cd ..
catkin_make -j2
git diff --check
```

静态证明：Filter 不再用 slope 作 decision；无 global envelope/range dead code；不生成
slope-induced REGULARITY；禁止文件无改动；production Builder 唯一；无 retry/sleep/tuning/
second builder/geometry widening。

## 7. G1 after launch

与 T0 完全同配置，用新 private master/ROS_HOME/evidence dir。记录 finalized bag、logs、
goal/final odom、raw/filter/epoch/cloud、Pair selected/nonzero、Runtime denial、PID/PGID 与
cleanup proof。

动态 acceptance：

1. 到达 internal `(8,0,2)`；
2. raw current anchors 全 found/complete；
3. 至少一次 Pair selected 且 retained delta 显著非零；
4. Filter 不因 `abs(slope)>0.80` 截短/删除；
5. truncation 明确归属 Builder 或 Validator，不再归属 fixed-slope Filter；
6. unsafe/unknown/validator failure 仍 fail-closed；
7. Runtime viability denial 可存在，但 `fatal=0` 且不计为 geometry empty；
8. 无 H2/A6 authority 退化，无 `all_candidates_path_end_clamped`；
9. task processes 与端口完全清理。

before/after 报告 raw/filter/validator cohorts、preview/forward horizon、Filter/Validator
truncation、可比 build latency、selected/nonzero 与 Runtime denial。不得要求真实 Builder/
Validator 安全截取计数归零。

## 8. 失败处置

- Filter test fail：只修 local-PWL 实现/oracle，不恢复 fixed slope/global envelope。
- Validator 负载/query limit：报告 cell/slope/cover/subdivision/query；不提高 limit、不降
  clearance。无法证明则 fail-closed 并停止，申请独立 Validator capacity 阶段。
- Runtime denial 增加：记录 geometry 与 `U+/U_safe` reason；不改 Runtime/rate/speed/timer。
- A6/H2 regression：立即停止，不得 observe-only、固定 delta=0 或 neutral retire 规避。
- 白名单外需求：停止报告。

## 9. Self-audit、handoff 与停止

写：

```text
docs/Codex_Tube_V2_T0_G1_Local_PWL_Geometry_Self_Audit_2026-08-20.md
```

包含 exact files、old/new algorithm、T0 与历史数据区别、测试/全构建、before/after 表、
margin/zero-connected/Builder/Validator/Runtime/H2 frozen audit、cleanup、limitations。
更新 handoff 顶部并保留历史。

完成 G1、全部验证、一次 after launch、自审和 handoff 后停止。不得自动进入 backward
viability、Runtime/QP、centerline-unsafe recovery、H2 stage 修复、recenter/M4/M6、planner/
参数调优、多机或论文实验。

