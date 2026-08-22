# Tube V2 G5 Exact Current-Anchor Sampled-Cover Execution Spec

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=COMPLETE
STAGE=TUBE_V2_G5
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=G5_T0_TO_MINIMAL_BRANCH_CORRECTION_TO_REGRESSION_TO_ONE_READINESS_MATCHED_ROS_RUN
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
FINAL_CLASSIFICATION=G5_IMPLEMENTATION_COMPLETE_NEW_FIRST_FALSE
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、调度、边界审阅和最终验收；GPT-5.6 Luna Max 单代理负责
T0 冻结、最小实现、测试、独立 ROS、self-audit 和 handoff。

Luna Max 禁止创建、调用、委派、请求或等待任何子代理。不得把阅读、编码、测试、ROS、
bag 解码、证据整理或文档工作交给其他模型。连续执行本规格，不在小步骤等待确认。

## 0. 前置事实和剩余工作

G3 已冻结：

```text
G3_IMPLEMENTATION_COMPLETE_QUERY_BUDGET_RESOLVED_DYNAMIC_ACCEPTANCE_UNMET
CURRENT_ANCHOR_INSUFFICIENT_CLEARANCE_REFINED_TO_MAX_DEPTH
```

G4 只读审计已完成，主分类：

```text
B_DUPLICATE_CLEARANCE_ACCOUNTING_PROVEN
```

同一 candidate 7 / map 919 / snapshot `1787292633.0334985` 的 exact evidence：

```text
current_w=3.1851342137905054
current base=(3.286887963172443,0.34280071140075674,1.0002026651862932)
snapshot_resolution=0.1
planner_safe_distance=0.4
nearest occupied closed-voxel AABB distance=0.44280071140075605

R0=0.4                                  PASS
R1=0.4+1e-6                             PASS
R2=0.4+0.5*resolution+1e-6=0.450001     FAIL
```

失败 cell 是 Validator 在 `validate()` 开头构造的精确 current-anchor cell：

```text
{w0=current_w, w1=current_w, v0=0, v1=1, depth=0}
```

其 inward descendants 只切分 v，始终保持 `w0==w1==current_w`。13 层均执行相同 9 点
closed-voxel clearance query；sampled geometry cover 从约 `1.6e-12` 趋近于 0，真正导致
拒绝的是 sampled fallback 额外加入的固定 `0.5*resolution=0.05`。query 本身已经量到
closed occupied voxel AABB，而不是 voxel centre，因此该固定项在这个精确分支没有独立
证明对象。

G5 的唯一代码任务：

> 仅在精确 `w0==w1==current_w` 的 sampled fallback 中，使用纯 sampled geometry cover
> `1.1*sampled_radius`；不加固定 half-voxel。所有其他 cover、查询、状态和层保持不变。

当前已知剩余量就是该单一实现点、定向测试、完整 G3 回归和一次 readiness-matched 独立
ROS 验收。若 ROS 暴露新的跨层 first-false，只取证并收尾，不在 G5 扩改。

## 1. 不可改变的安全和架构合同

必须全部保留：

1. `planner_safe_distance=0.4` 及其 production authority 不变；
2. `cover_epsilon=min(1e-6,0.01*snapshot_resolution)` 不变；
3. accepted current-anchor leaf 仍执行 row-major 3x3、准确 9 次 query；
4. `KNOWN_FREE && clearance_certified && finite(clearance) && clearance>=requested_radius`
   的接受谓词不变；
5. UNKNOWN、OUT_OF_MAP、OCCUPIED、UNAVAILABLE、未认证 clearance 和非有限 clearance
   全部 fail-closed；
6. current anchor 仍先于所有 path cells 验证；
7. current-connected segment、zero-centreline evidence 和 knot evidence 语义不变；
8. sampled factor `1.1` 不变；
9. 非退化 sampled fallback 仍为
   `1.1*sampled_radius + 0.5*snapshot_resolution`；
10. certified nondegenerate cover 仍为
    `fixed_half_voxel + w_reducible + v_reducible`；
11. PathCellGeometryCertificate 的 speed/regularity checks 不变；
12. G3 leaf-only scheduling、proof-derived anisotropic split、计数和默认 limits 不变；
13. `max_subdivision_depth=12`、`max_query_samples=250000` 不变；
14. G2 inward families、选择、Builder attribution 和 zero-only fallback 不变；
15. Filter、Epoch、Runtime、Pair、H2、planner、map、adapter、launch 和 schema 不变。

本阶段不是降低 planner clearance，不是用 categorical free 代替 clearance，也不是删除通用
voxel support。它只删除 G4 已证明在 exact degenerate current-anchor sampled branch 中重复的
一次 accounting。

## 2. G5-T0：修改前冻结

修改前完整读取：

```text
AGENTS.md
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Self_Audit_2026-08-21.md
docs/Codex_Tube_V2_G4_Current_Anchor_Clearance_Accounting_Read_Only_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md 顶部 G4/G3 段
本 G5 execution spec
```

建立：

```text
/tmp/tube_v2_g5_t0_20260821_<id>/
```

保存：

```text
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
G4 source_hashes_normalized.tsv
G4 current_anchor_query_sequence.tsv
G4 counterfactuals.tsv
```

记录至少以下文件 SHA-256：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/plan_env/src/cloud_occupancy_snapshot.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_cloud_occupancy_query.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
```

工作区高度脏，所有已有 tracked/untracked 文件均为用户资产。禁止 reset、clean、checkout、
restore、stash、rebase、commit 或 push。只允许碰 §5 白名单。

## 3. G5-T1：先建立精确 boundary tests

先在现有 `tube_surface_validator_test.cpp` 添加针对 exact branch 的 deterministic tests；不得
另建测试二进制、测试框架、fixture module 或 CMake target。

### 3.1 Exact current-anchor narrow-clearance pass

构造完整、有限、含 exact current knot 的 zero-width 或可解析极窄 sampled profile，使
current-anchor `sampled_radius=0`（或明确小于 `1e-12`）。query 回调：

- 对 current-anchor 首 9 次调用返回 `KNOWN_FREE`、`clearance_certified=true`、
  clearance 严格位于 `0.400001` 与 `0.450001` 之间，例如 `0.425`；
- 后续非 current-anchor cells 可返回充分大的 certified clearance，使整体验证完成；
- 保存每次 requested radius 和 point。

必须断言：

```text
validate=true
result.current_anchor_valid=true
first 9 callback calls are the current-anchor row-major 3x3 set
first 9 requested radii == 0.400001 within 1e-12
exactly 9 current-anchor queries occur before path-cell validation
all first 9 returned clearances are <0.450001 and still pass
```

若 profile 令 9 个几何点重合，也必须保留 9 次调用，禁止去重、cache 或提前成功。

### 3.2 True insufficient clearance remains fail-closed

使用同一 exact current-anchor setup，返回
`KNOWN_FREE + clearance_certified=true + clearance<0.400001`，例如 `0.4000005`。
把 test config 的 depth 设为 0 以隔离该 cell，但不改 production default。

必须断言：

```text
validate=false
result.current_anchor_valid=false
query_sample_count=9
first_failure_reason=INSUFFICIENT_CLEARANCE
profile.obstacle_certified=false
```

### 3.3 Categorical and certification failures remain fail-closed

在同一 exact branch 上至少逐项覆盖：

```text
UNKNOWN
OUT_OF_MAP
OCCUPIED
UNAVAILABLE
KNOWN_FREE but clearance_certified=false
KNOWN_FREE but clearance=NaN
```

每项不得因移除 fixed cover 而通过。保留现有 StopReason 映射；不新增状态或门控。

### 3.4 Nondegenerate sampled fallback numerical freeze

构造 `w1>w0`、无 cell certificate、cover 可在 depth 0 查询的 deterministic cell；记录
current-anchor 前 9 次之后的 path-cell requested radius。断言非退化 sampled cell 仍精确为：

```text
1.1*sampled_radius + 0.5*snapshot_resolution
```

容差不超过 `1e-12`。不得把 G5 结论扩展到 `w1-w0<=kEpsilon`、near-equal w 或任意
nondegenerate sampled cell。

### 3.5 Certified nondegenerate cover numerical freeze

保留并必要时细化现有 `CertifiedCoverBreakdownKeepsLegacyTotal`：

```text
certified nondegenerate cover = fixed + w_reducible + v_reducible
```

其 fixed half-voxel 仍存在。若该 test 同时观察 current-anchor sampled fallback 的 max cover，
只更新 exact current-anchor oracle 为 `1.1*sampled_radius`；不得改 certified cell 的 expected
cover。测试名称/注释必须清楚区分两类分支。

### 3.6 G3 deterministic accounting freeze

G3 wide-ribbon test必须继续断言：

```text
query_sample_count=144
limit_exceeded=false
split_w_count=0
split_both_count=0
split_v_count>0
anisotropic_split_count>0
all accepted leaves still 9 queries
```

若 max cover 因 exact current-anchor branch 从 `1.345` 变为 `1.32`，只更新这一条已授权
oracle；leaf/query/split/depth 安全语义不能变化。

## 4. G5-T2：唯一允许的 production 修改

修改位置限定在 `tube_surface_validator.cpp` 的 file-local validation context 和
`CollectCellPoints()` sampled fallback。

### 4.1 精确 branch identity

把调用方 `validate(..., current_w, ...)` 的 exact `current_w` 仅作为 file-local context 值传给
`CollectCellPoints()`，或者采用同等但更小、且能证明不会命中其他 cell 的 file-local实现。

资格条件必须等价于：

```text
cell.w0 == current_w && cell.w1 == current_w
```

要求 exact equality；禁止使用 `kEpsilon`、`kAnchorTolerance`、相对 tolerance、
`w1-w0<=...` 或“接近退化”判断。理由是 production current-anchor cell 直接用同一个
`current_w` 构造，v-only descendants 原样复制 w；任何非退化或 near-degenerate path cell 均
不在 G4 证明范围。

不得通过 profile classification、candidate number、map sequence、width threshold、clearance
值或 ROS state 判断该分支。

### 4.2 Cover formula

sampled fallback 只允许如下二选一：

```text
exact current anchor:
  cover_radius = 1.1 * sampled_radius

all other sampled fallback cells:
  cover_radius = 1.1 * sampled_radius + 0.5 * snapshot_resolution
```

保持 finite/nonnegative 校验、result min/max accounting、target cover、subdivision和 query
调用位置不变。不要添加参数、enum、公共 API、cache、日志、topic、diagnostic field 或新的
validator class。

注释必须明确：closed-voxel query 已拥有 exact current-anchor sampled branch 的 voxel support；
非退化 fallback 没有被 G4 证明可删除 fixed inset，因此保留。

### 4.3 绝对禁止触碰

不得修改：

- `CertifiedCellCoverRadius()` 的公式、breakdown 或 regularity/speed checks；
- `CellClearancePasses()` 的 requested radius、9 点循环、状态谓词或计数；
- `ChooseProofDerivedSplit()`、`ValidateCell()` 的 G3 scheduling/split/depth；
- public header/config/defaults；
- Builder、Filter、Epoch、Runtime、Pair、H2、planner、map query、launch 或 schema。

实现目标是一个局部条件和一处 cover 选择；若发现需要新增跨模块状态、参数、门控或第二套
流程，停止实现并报告，不能把 Tube 做复杂。

## 5. 文件白名单

允许修改：

```text
docs/Codex_Tube_V2_G5_Exact_Current_Anchor_Sampled_Cover_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G5_Exact_Current_Anchor_Sampled_Cover_Self_Audit_2026-08-21.md
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
/home/cxq/ISF-GVF/handoff.md
```

只有现有 CertifiedBuilder test 因 exact G5 oracle 直接失败、且无法由 Validator test 完整表达
时，才条件式允许：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
```

不得修改 `tube_surface_validator.h`，除非现有编译边界证明 file-local context 无法完成；按当前
源码结构预期不需要。任何白名单外 source/test 需求都停止并报告。

## 6. Focused、完整回归和静态审计

按顺序执行并保存完整日志：

```text
catkin_make -j2 \
  phase_offset_tube_surface_validator_test \
  phase_offset_certified_tube_builder_test \
  cloud_occupancy_snapshot_test \
  phase_offset_cloud_occupancy_query_test

./devel/lib/phase_offset_navigation/phase_offset_tube_surface_validator_test
./devel/lib/phase_offset_navigation/phase_offset_certified_tube_builder_test
./devel/lib/plan_env/cloud_occupancy_snapshot_test
./devel/lib/bspline_race/phase_offset_cloud_occupancy_query_test
```

然后完整 G3 regression：

```text
catkin_make -j2 \
  phase_offset_tube_filter_test \
  phase_offset_tube_epoch_manager_test \
  phase_offset_runtime_test \
  gvf_switch_policy_test \
  continuous_phase_path_test \
  phase_offset_matched_adapter_test

./devel/lib/phase_offset_navigation/phase_offset_tube_filter_test
./devel/lib/phase_offset_navigation/phase_offset_tube_epoch_manager_test
./devel/lib/phase_offset_navigation/phase_offset_runtime_test
./devel/lib/bspline_race/gvf_switch_policy_test
./devel/lib/bspline_race/continuous_phase_path_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test

cd build
ctest --output-on-failure -R \
'(_ctest_phase_offset_navigation_gtest_phase_offset_tube_(cross_section|builder|filter|surface_validator)_test|_ctest_phase_offset_navigation_gtest_phase_offset_(certified_tube_builder|runtime|tube_epoch_manager)_test|_ctest_bspline_race_gtest_(gvf_switch_policy|continuous_phase_path|phase_offset_matched_adapter)_test)$'
cd ..

catkin_make -j2
git diff --check
```

若实际 binary 路径与上文不同，使用现有 CMake 生成路径，不修改 CMake。

静态审计必须证明：

```text
one local exact-current-anchor condition only
no tolerance-based widening of branch
no new gate/parameter/cache/retry/timer/thread/module/schema
accepted current-anchor still exactly 9 queries
nondegenerate sampled fixed half-voxel unchanged
certified nondegenerate fixed half-voxel unchanged
planner clearance and epsilon unchanged
query/depth defaults unchanged
Builder/Filter/Epoch/Runtime/Pair/H2/planner/map/launch hashes unchanged
```

## 7. G5-T3：same-snapshot replay

在启动新 ROS 前，复用 G4 证据和 shadow replay，不修改产品源码或 workspace CMake，重算：

```text
point=(3.286887963172443,0.34280071140075674,1.0002026651862932)
snapshot=candidate 7/map 919/stamp 1787292633.0334985
exact nearest closed AABB distance=0.44280071140075605
new exact-current-anchor request=0.4+1.1*sampled_radius+1e-6
```

必须保存 new request、returned certified lower bound、exact nearest 和 pass/fail。期望约
`0.4000010000016`，应 PASS。该 replay 是 accounting 验证，不替代 production ROS acceptance。

## 8. G5-T4：readiness-matched 独立 ROS

测试全通过后，运行一次与 G3 完全相同的独立 ROS 验收：

```text
fresh private roscore port
fresh ROS_HOME
new /tmp/tube_v2_g5_readiness_20260821_<id>/
unchanged absolute phase_offset_esdf_tube_single.launch
unchanged pillar.pcd
unchanged parameters and observe_only=false
Phase A actual /sim/local_map nonempty updates >=900
Phase B exactly one goal publication: (8,0,1)
no fixed sleep used as a product workaround
```

先确认目标端口无 listener；只记录并清理本任务 PID/PGID，绝不附着或终止用户已有 ROS。
录制与 G3 相同的必要 topics，并保存 launcher/rosout、bag info、effective params、goal/final
odom、process/port cleanup proof。

必须解码并关联同一 cohort 的：

```text
candidate_sequence / map_observation_sequence / snapshot stamp
raw current_w and found+complete
first_stop_reason / first_stop_w / last_queries / limit_exceeded
active obstacle/display certification
zero gate
Pair COMMITTED
selected_runtime_executed
retained nonzero abs(delta)>1e-6
Runtime denial/fatal
goal reached internal (8,0,2)
all_candidates_path_end_clamped
```

期望动态 acceptance：

1. G3 的 current-anchor `0.450001` false 消失；
2. 至少一个关键 candidate 在同一安全参数下 current anchor 通过；
3. active profile `OFFSET_CERTIFIED` / obstacle certified；
4. Pair `COMMITTED`；
5. selected runtime authority；
6. retained nonzero delta；
7. goal 达到 internal `(8,0,2)`；
8. fatal=0；
9. 无 `all_candidates_path_end_clamped`；
10. cleanup 为空。

如果第一 run 仅因 task-owned recorder/launch/端口等非产品原因无效，允许一次完全同配置 rerun；
不得因产品 first-false 反复 launch。

## 9. 新 first-false 处置：只取证，不扩改

如果 G5 current anchor 已通过，但动态链路在非退化 Surface、Builder、Filter、Epoch、Runtime、
Pair、H2、planner 或其他层出现新的 first-false：

1. 精确记录 candidate/map/path owner、状态、数值和第一失败层；
2. 完成已有 tests、self-audit、handoff 和 cleanup；
3. 状态记为 `G5_IMPLEMENTATION_COMPLETE_NEW_FIRST_FALSE`；
4. 不修改该新层，不写临时门控，不调参数，不继续叠加 Tube 机制。

特别禁止：

- 再删非退化 half-voxel；
- 降低 `planner_safe_distance` 或 epsilon；
- 提高 query/depth limits；
- 增加 cache、retry、warmup、timer、sleep、状态 gate；
- 修改 inward families、Builder、Runtime、Pair、H2 或 planner；
- 用 observe-only、zero-only、减速、延迟 goal 或改 map 制造通过。

## 10. Self-audit、handoff 和最终状态

写：

```text
docs/Codex_Tube_V2_G5_Exact_Current_Anchor_Sampled_Cover_Self_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
```

self-audit 必须包含：

- exact changed files 和 before/after hashes；
- old/new 两行公式和 exact branch predicate；
- G4 same-snapshot replay；
- 所有新增/既有 test 数量及日志；
- G3 deterministic 144-query freeze；
- complete CTest、full build、diff-check；
- protected hashes；
- 独立 ROS identity、动态结果和 cleanup；
- no-bloat/no-new-gate/no-parameter-change 审计；
- 若有新 first-false，明确只记录未修复。

最终状态只能选择：

```text
G5_EXACT_CURRENT_ANCHOR_ACCOUNTING_CORRECTED_DYNAMIC_ACCEPTANCE_PASS
G5_IMPLEMENTATION_COMPLETE_NEW_FIRST_FALSE
G5_IMPLEMENTATION_COMPLETE_DYNAMIC_REPRODUCTION_UNMET
G5_BLOCKED_OUTSIDE_WHITELIST_OR_WORKSPACE
```

## 11. 给 Luna Max 的连续执行指令

```text
你是本阶段唯一执行代理 GPT-5.6 Luna Max，reasoning=max。
禁止创建、调用、委派、请求或等待任何子代理。

完整读取 AGENTS.md、G3 execution spec/self-audit、G4 read-only audit、handoff 顶部和本 G5
spec。连续执行 G5-T0 -> exact boundary tests -> 唯一局部分支修改 -> focused tests -> 完整
G3 regression/full build -> same-snapshot replay -> readiness-matched isolated ROS -> self-audit/
handoff。不要在小步骤停下来确认。

只处理精确 w0==w1==current_w sampled fallback 的 fixed half-voxel 重复计费。保持 1.1
sampled factor、epsilon、9 点 query、fail-closed 状态、v subdivision、非退化 sampled cover 和
certified cover不变。禁止新增门控、模块、cache、参数、第二套 Validator，禁止改 Builder、
Filter、Epoch、Runtime、Pair、H2、planner、map、launch 或 schema，禁止调 depth/query/
clearance/速度/timer。

如果动态 first-false 移到其他层，只取证、测试、自审和 handoff，不扩改。持续执行到 G5
授权工作全部完成并清理本任务 ROS 进程。
```
