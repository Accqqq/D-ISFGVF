# Tube V2 G3 SurfaceValidator Proof-Capacity Execution Spec

~~~text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_G3
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=READ_ONLY_FREEZE_TO_DETERMINISTIC_REPRODUCTION_TO_PROOF_PRESERVING_CAPACITY_CORRECTION_TO_FULL_VALIDATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
~~~

日期：2026-08-21  
工作区：/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws  
执行分工：主代理负责规格、调度、复杂度/安全 gate 审核、白名单复核与最终验收；
Luna max 单代理负责证据冻结、实现、测试、独立 ROS 运行、自审和 handoff。

Luna max 禁止创建、调用、委派、请求或等待任何子代理；不得把分析、编码、测试、
benchmark、launch 或证据处理交给其他模型。

## 0. 前置事实与本阶段唯一目标

G2 最终 readiness-matched run：

/tmp/tube_v2_g2_readiness_final_20260821_025413/

已精确证明：

~~~text
candidate_seq=7
map_seq=928
first_stop_reason=INSUFFICIENT_CLEARANCE
first_stop_w=4.989959832
query_sample_count=249993
max_query_samples=250000
limit_exceeded=1
classification=ZERO_ONLY_PLANNER_BASELINE
-> no Pair COMMITTED / selected / retained nonzero
~~~

goal 正常到达 internal (8,0,2)，raw current anchors 完整，zero gate 正常打开；
Runtime、Pair、H2 不是该 run 的 first-false。

G3 的唯一目标：

> 在不提高 250000 query budget、不改变 max subdivision depth、不降低 clearance/cover/
> regularity、不添加门控或参数的前提下，让现有同一 SurfaceValidator 连续覆盖证明避免
> 对不可接受父 cell 的无效查询和不必要的各向同性四叉展开，从而在现有固定预算内完成。

本阶段优化的是证明执行顺序与重复工作，不是减少证明义务。

## 1. 必须冻结的安全合同

以下合同全部不变：

1. 每个最终接受的 nonzero ribbon cell 都必须有完整 continuous cover；
2. required clearance 仍为 planner_safe_distance + cell cover + epsilon；
3. cover 的几何项、snapshot half-voxel 项和 margin accounting 数值不变；
4. PathCellGeometryCertificate 的 speed/regularity 条件不变；
5. UNKNOWN、OUT_OF_MAP、OCCUPIED、UNAVAILABLE、insufficient clearance、
   regularity failure 全部 fail-closed；
6. max_subdivision_depth=12 和 max_query_samples=250000 默认值不变；
7. current anchor 必须首先可证明；
8. 只保留 current-connected、在 w 上连续的 certified cells；
9. G1 local-PWL、G2 inward correction、zero-only fallback 不回滚；
10. Runtime exact-PWL、U+ -> U_safe、Pair bootstrap/H2/planner 不改。

禁止用“少查一些点”替代覆盖证明。唯一允许跳过的是：当前 cell 的 cover 已经大于接受
阈值、因此该 parent cell 无论 clearance query 结果如何都不可能被直接接受时的 parent
clearance query。G3 不新增 cache，只修正 parent-query scheduling 与 proof-derived split。

## 2. 当前查询爆炸的静态路径

当前 src/.../tube_surface_validator.cpp 的 ValidateCell() 顺序为：

~~~text
CollectCellPoints + compute cover
-> CellClearancePasses: query all 9 points
-> only then check cover_radius <= target_cover
-> if not accepted and both w/v spans splittable: always split 4 children
~~~

因此存在两类证明冗余：

### 2.1 不可接受 parent 的提前查询

若 cover_radius > target_cover，该 parent 本身不可能被接受；它的 9 次 clearance calls
不能替代所有 child 的最终证明。先查询 parent 再 split 会消耗 budget，却没有减少最终
leaf 证明义务。

### 2.2 各向同性四叉展开

CertifiedCellCoverRadius 的 cover 可分成：

~~~text
fixed = 0.5 * snapshot_resolution
w_reducible =
    midpoint_position_variation_bound
  + 0.5 * maximum_delta * normal_variation_bound
  + 0.5 * delta_slope * h
v_reducible =
    0.5 * maximum_width * v_span
cover = fixed + w_reducible + v_reducible
~~~

当只有 w 或只有 v 项主导时，当前实现仍同时 split w 和 v，形成不必要的 4^depth
leaf 数量。G3 只允许根据上述已有证明项选择 split dimension；不得发明经验阈值。

## 3. G3-T0：只读冻结

任何源码/test 修改前：

1. 完整读取 AGENTS.md、G2 execution spec/self-audit、handoff 顶部；
2. 记录 HEAD、git status --short、git diff --stat、git diff --check；
3. 记录以下文件 sha256：

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
~~~

4. 只读总结 G2 final 的 map maturity、raw/current facts、query terminal、goal、cleanup；
5. 新建 /tmp/tube_v2_g3_t0_20260821_<id>/ 保存 identity、hashes、static model 和 evidence
   manifest。

工作区高度脏；禁止 reset/clean/checkout/restore/stash/rebase。所有已有文件均为用户资产。

## 4. G3-T1：deterministic query-capacity reproduction

在 production 修改前，先只修改测试，建立可计算的复杂度基线。

### 4.1 Certified wide-ribbon fixture

在 tube_surface_validator_test.cpp 中构造：

- strictly increasing local-PWL knots；
- complete current anchor；
- valid PathCellGeometryCertificate；
- same required clearance、snapshot resolution 和 regularity contract；
- all-known-free immutable ClearanceQuery；
- deliberately large but finite PWL width/slope，使 current implementation 在小于 production
  的测试 budget 下确定性达到 query limit；
- fixture 的 cover decomposition 必须能从参数解析计算，不得通过随机搜索凑数。

pre-change test/log 必须证明：

~~~text
geometry/path valid
all queried points KNOWN_FREE
failure only because query_sample_count reaches configured budget
limit_exceeded=1
first_failure_reason=INSUFFICIENT_CLEARANCE
~~~

保存 pre-change 日志到 /tmp。

### 4.2 Callback accounting

query callback 自己计数，断言：

~~~text
callback_call_count == result.query_sample_count
~~~

不得用 wall time 作为主要容量 oracle；query calls、leaf count 和 split count才是 oracle。

### 4.3 Existing negative controls

在改 production 前复跑并冻结：

- between-knot obstacle；
- endpoints safe/interior unsafe；
- remote unsafe suffix；
- UNKNOWN edge；
- sample query limit；
- current-anchor cell failure；
- regularity certificate failure。

## 5. G3-T2：第一优先修正——leaf-only clearance queries

### 5.1 精确算法

ValidateCell() 必须先完成 geometry/cover 计算，再决定是否 query：

~~~text
geometry invalid
  -> preserve existing failure/subdivision semantics

geometry valid && cover_radius > target_cover
  -> this parent cannot be accepted
  -> do not call ClearanceQuery for this parent
  -> split using §6 policy

geometry valid && cover_radius <= target_cover
  -> execute the existing 9-point clearance queries
  -> if all safe, accept
  -> if any unsafe/unknown/OOM, split or fail exactly as before
~~~

这不放宽证明：最终 accepted leaf 仍执行同样 9-point query 和同样 requested radius。

### 5.2 计数语义

TubeSurfaceValidationResult 至少区分：

~~~text
query_sample_count                 # actual ClearanceQuery calls, existing meaning
geometry_cell_count                # CollectCellPoints attempts
clearance_leaf_cell_count          # cells that actually called ClearanceQuery
prequery_cover_split_count         # cover too large, skipped parent query
max_depth_observed
~~~

这些只是在内存中的 proof-capacity evidence，不是 ROS schema、gate、limit 或参数。

现有 query_sample_count 外部语义保持“实际底层 query call 数”。

### 5.3 T2 gate

实现 leaf-only 后：

- 新 wide-ribbon fixture 必须在相同 budget 下完成；
- query count 必须显著低于 pre-change；
- 所有 negative controls 结果不变；
- 若 production-like stress 仍接近/达到 budget，自动进入 T3；
- 若已留出明确余量，可不实现 T3，直接进入回归和 ROS。

“明确余量”定义为 deterministic stress 的 query_sample_count <= 相同 test budget 的 50%。
该 50% 只用于决定是否继续做性能实现，不参与 production acceptance，也不得写进
Validator 决策。

## 6. G3-T3：条件式各向异性 split

只有 T2 deterministic stress 仍超过 50% budget 或仍 limit-exceeded 时授权。

### 6.1 Cover breakdown

CertifiedCellCoverRadius 同时返回 proof breakdown：

~~~text
fixed_cover
w_reducible_cover
v_reducible_cover
decomposable=true
~~~

总 cover 计算必须与修改前 bitwise/数值等价（允许 1e-12 浮点 tolerance），不能漏项、
重复项或改变 snapshot half-voxel charge。

无完整 certified-cell breakdown 的 sampled-cover fallback：

- decomposable=false；
- 保持既有 conservative split 行为；
- 不用经验方式猜维度。

### 6.2 Dimension choice

只在 cover too large 且 decomposable=true 时：

~~~text
allowable_reducible = target_cover - fixed_cover
~~~

- 若 w_reducible > allowable_reducible 且 v_reducible > allowable_reducible：
  split both；
- 若只有 w 超过：split w only；
- 若只有 v 超过：split v only；
- 若二者单独均不超过但和超过：split 较大的项；
- 绝对相等时固定优先 split w，child 会重新计算，不新增 hysteresis；
- 选中维度不可 split 时退回另一个可 split 维度；
- 两者均不可 split 则按原 max-depth/insufficient-clearance fail-closed。

若 clearance query 已执行且失败，不根据 cover breakdown 猜障碍方向；保持原 conservative
both-dimension split（或仅剩可 split 的维度）。

### 6.3 Depth

不提高 max_subdivision_depth。每产生一层 child 仍 depth+1。若各向异性顺序导致在既有
depth 内无法完成，fail-closed；不得用隐藏的 w/v 独立 depth 绕过上限。

### 6.4 T3 gate

- deterministic stress query count 进一步下降；
- cover breakdown sum 等于原 cover；
- w-only、v-only、both、tie、unsplittable fallback 都有单测；
- negative controls 全不变；
- 若仍达到 query budget，停止并报告；不得叠加 cache 或其他证明层。

## 7. G3-T4：禁止继续叠加机制

若 T2 leaf-only 与必要的 T3 anisotropic split 后仍达到 query budget：

- 保存 deterministic query/leaf/split/depth evidence；
- 不新增 exact/approximate cache；
- 不新增 spatial index、第二套 Validator 或新的 proof pipeline；
- 不调 query/depth limit；
- 完成测试、自审和 handoff，状态为
  G3_IMPLEMENTATION_COMPLETE_DYNAMIC_QUERY_BUDGET_UNMET。

## 8. 文件白名单

始终允许：

~~~text
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Self_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
~~~

T1/T2 允许：

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
~~~

只为 in-memory result evidence / final attribution 且确有需要时允许：

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
~~~

限制：不得改变 G2 inward families、selection、raw preservation、query-budget terminal
semantics；只允许读取新的 Validator evidence并写准确 diagnostics/tests。

全部其他文件永久只读，包括：

~~~text
TubeBuilder production
TubeFilter production
TubeEpochManager production
PhaseOffsetRuntime / PortProjector / Core
phase_offset_matched_adapter production
gvf_manager / H2 / planner / C2
plan_env / CloudOccupancySnapshot
launch / config / parameters
CMakeLists.txt / package.xml
ROS msg/topic/schema
Paper proposal
~~~

G2 adapter attribution log保留但不得修改或增加 command-loop 日志。

## 9. 强制禁止事项

1. 不提高 max_query_samples；
2. 不提高 max_subdivision_depth；
3. 不降低 planner_safe_distance、cover_epsilon、snapshot resolution charge、
   regularity margin；
4. 不减少 accepted leaf 的 9-point surface set；
5. 不新增 readiness/control/navigation/profile gate；
6. 不新增 launch 参数或 tunable split heuristic；
7. 不用 fixed sleep、retry launch、减速、observe-only、固定 delta=0制造通过；
8. 不恢复 Filter slope cap/global envelope；
9. 不改 G2 inward算法去绕过真实 query limit；
10. 不进入 Runtime/QP/Projector、H2、recenter、planner、backward viability、多机；
11. 不用 wall-time benchmark替代 deterministic query-count proof；
12. 不实现任何 exact/approximate cache。

## 10. 必须覆盖的 tests

### 10.1 Leaf-only scheduling

- cover too large parent executes zero ClearanceQuery calls before split；
- accepted leaf still executes exact 9 calls；
- callback count equals query_sample_count；
- low max_query_samples still fails closed；
- max depth reached before acceptable cover remains limit_exceeded。

### 10.2 Anisotropic split（若 T3 实施）

- width-only pressure produces v-only split；
- path/slope-only pressure produces w-only split；
- both individually exceed allowable produces four children；
- combined-only excess chooses larger term；
- exact tie chooses w deterministically；
- sampled fallback remains conservative；
- depth accounting unchanged。

### 10.3 Safety regressions

- high-slope continuously safe ribbon passes；
- between-knot obstacle rejects；
- endpoints safe/interior unsafe rejects；
- remote unsafe suffix truncates current-connected segment；
- UNKNOWN observed edge fails；
- OUT_OF_MAP fails；
- current anchor unsafe fails；
- regularity/cell certificate failure fails；
- query budget terminal remains zero-only in CertifiedBuilder；
- G2 inward deterministic repair tests 11/11 remain。

## 11. 构建与静态验收

至少执行：

~~~text
catkin_make -j2 \
  phase_offset_tube_surface_validator_test \
  phase_offset_certified_tube_builder_test \
  phase_offset_tube_filter_test \
  phase_offset_tube_epoch_manager_test \
  phase_offset_runtime_test \
  gvf_switch_policy_test \
  phase_offset_matched_adapter_test

./devel/lib/phase_offset_navigation/phase_offset_tube_surface_validator_test
./devel/lib/phase_offset_navigation/phase_offset_certified_tube_builder_test
./devel/lib/phase_offset_navigation/phase_offset_tube_filter_test
./devel/lib/phase_offset_navigation/phase_offset_tube_epoch_manager_test
./devel/lib/phase_offset_navigation/phase_offset_runtime_test
./devel/lib/bspline_race/gvf_switch_policy_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test

cd build
ctest --output-on-failure -R \
'(_ctest_phase_offset_navigation_gtest_phase_offset_tube_(cross_section|builder|filter|surface_validator)_test|_ctest_phase_offset_navigation_gtest_phase_offset_(certified_tube_builder|runtime|tube_epoch_manager)_test|_ctest_bspline_race_gtest_(gvf_switch_policy|continuous_phase_path|phase_offset_matched_adapter)_test)$'
cd ..

catkin_make -j2
git diff --check
~~~

静态证明：

- accepted cell clearance predicate和9点集合不变；
- cover公式和margin数值不变；
- only impossible parent queries skipped；
- split policy只来自proof breakdown；
- query/depth defaults unchanged；
- no new gate/parameter/schema/thread；
- no protected source changes；
- unique CertifiedTubeBuilder production path不变。

## 12. G3-T5：readiness-matched ROS acceptance

使用 G2 已冻结的两阶段 harness：

~~~text
Phase A:
  fresh private master/ROS_HOME/evidence
  actual nonempty map messages
  /sim/local_map count=900
  mock/occupancy/esdf/update持续有效
  no fixed sleep

Phase B:
  publish goal (8,0,1) exactly once
  pure observation
~~~

场景、launch、map、observe_only和所有参数与 G2 相同。使用新端口和 evidence dir，只清理
本次 PID/PGID。

必须记录：

- Validator query_sample_count；
- geometry/leaf/prequery-split/anisotropic counts（已实施者）；
- candidate/map sequence；
- raw current anchors；
- active obstacle/display certification；
- zero gate；
- Pair COMMITTED；
- selected_runtime_executed；
- retained nonzero；
- Runtime denial/fatal；
- goal result；
- cleanup。

动态 acceptance：

1. goal 到达 internal (8,0,2)；
2. raw current anchors 全 found+complete；
3. 至少一个关键 candidate 在 250000 内完成，不能再出现 query-budget terminal；
4. active profile OFFSET_CERTIFIED/obstacle_certified；
5. Pair COMMITTED；
6. selected_runtime_executed；
7. retained delta abs(after)>1e-6；
8. fatal=0；
9. 无 all_candidates_path_end_clamped；
10. process/port cleanup为空。

如果 query budget已解决但 first-false移到真实 UNKNOWN/OOM/clearance/regularity或 Runtime
denial：

- 精确记录；
- 不继续修改其他层；
- 不调参数；
- 完成本阶段自审并停止在新 first-false。

若第一 run 因录包/进程/端口非产品原因无效，允许一次完全同配置 rerun；不得因产品失败
重复 launch。

## 13. Self-audit、handoff 与最终状态

写：

docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Self_Audit_2026-08-21.md

更新 /home/cxq/ISF-GVF/handoff.md 顶部，保留历史。

必须包含：

- exact stage status；
- exact changed files；
- before/after hashes；
- pre-change deterministic query reproduction；
- T2/T3各自是否实施及 gate 证据；
- old/new proof execution，不得写成安全条件变化；
- query count before/after；
- all tests/build/diff-check；
- readiness ROS evidence；
- cleanup；
- limitations；
- no-new-gate/no-limit-change audit；
- explicit stop boundary。

最终状态选择：

~~~text
G3_PROOF_CAPACITY_CORRECTED_DYNAMIC_ACCEPTANCE_PASS
G3_PROOF_CAPACITY_CORRECTED_NEW_FIRST_FALSE
G3_DETERMINISTIC_CAPACITY_REPRODUCTION_UNMET
G3_IMPLEMENTATION_COMPLETE_DYNAMIC_QUERY_BUDGET_UNMET
G3_BLOCKED_OUTSIDE_WHITELIST_OR_WORKSPACE
~~~

不得自动进入下一层。

## 14. 给 Luna max 的执行指令

~~~text
你是本阶段唯一执行代理 gpt-5.6-luna，reasoning=max。
禁止创建、调用、委派、请求或等待任何子代理。

完整读取 AGENTS.md、本 G3 execution spec、G2 execution spec/self-audit、
/home/cxq/ISF-GVF/handoff.md，以及本规格点名源码和 final evidence。

连续执行 G3-T0 -> deterministic pre-change reproduction -> T2 leaf-only ->
按 gate 必要时 T3 anisotropic -> focused/full tests ->
readiness-matched isolated ROS -> self-audit/handoff。

不需要每个小步骤等待确认；但任何 production 修改必须满足前一 gate。不得提高 query/
depth limit、降低 clearance/cover、添加门控/参数、改 Runtime/H2/planner/launch/schema。

如果 query budget解决后 first-false移到其他安全或动态层，记录并收尾，不得跨层修复。
~~~
