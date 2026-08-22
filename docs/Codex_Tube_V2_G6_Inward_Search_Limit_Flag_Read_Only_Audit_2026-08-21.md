# Tube V2 G6 Inward-Search Limit-Flag Read-Only Audit

```text
DOCUMENT_ROLE=READ_ONLY_AUDIT_EXECUTION_SPEC
DOCUMENT_STATUS=COMPLETE
STAGE=TUBE_V2_G6
IMPLEMENTATION_AUTHORIZED=false
SOURCE_OR_TEST_EDITS_AUTHORIZED=false
AUTO_ADVANCE=FREEZE_TO_STATIC_ATTEMPT_RECONSTRUCTION_TO_EXACT_RUNTIME_TERMINAL_CAPTURE_TO_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、调度、分类边界和下一阶段判断；GPT-5.6 Luna Max 单代理负责
只读源码核算、既有 bag 解码、必要的一次 unchanged-binary gdb 独立 ROS、离线 shadow replay、
self-audit 和 handoff。

Luna Max 禁止创建、调用、委派、请求或等待任何子代理。不得把源码阅读、数学核算、bag
解码、gdb、ROS、shadow replay、证据处理或文档工作交给其他模型。连续执行本规格，不在小
步骤等待确认。

## 0. G5 已冻结事实和 G6 唯一问题

G5 最终状态：

```text
G5_IMPLEMENTATION_COMPLETE_NEW_FIRST_FALSE
```

G5 只改变 exact sampled current-anchor branch：

```text
cell.w0 == current_w && cell.w1 == current_w:
  cover = 1.1 * sampled_radius
all other sampled fallback cells:
  cover = 1.1 * sampled_radius + 0.5 * snapshot_resolution
```

G5 focused tests、G3 regression、CTest、full build 和 same-snapshot replay 已通过；最终只改
`tube_surface_validator.cpp` 和 `tube_surface_validator_test.cpp`。非退化 sampled cover、
certified cover、planner clearance、epsilon、9 点 query、depth/query defaults 和所有下游层
均未改变。

纠正后的 readiness run：

```text
/tmp/tube_v2_g5_readiness_20260821_154000/
private port=12911
fresh ROS_HOME
observe_only=false
pillar.pcd
pre-goal local map=900
one goal=(8,0,1), internal=(8,0,2)
goal reached
cleanup empty
```

G6 只研究 corrected run 的 candidate 8 / map 919 / snapshot
`1787298468.9096804`：

```text
current_w=3.113361964553144
raw current sample found=1 complete=1
current filtered/raw interval evidence approximately [-0.01875, 1.4875]
first_stop_reason=INSUFFICIENT_CLEARANCE
first_stop_w=3.113361965
invalid_reason="inward search attempted but no candidate certified; attempts=101
last_reason=insufficient_clearance last_queries=2484 last_limit_exceeded=1;
using planner zero baseline"
```

已确定：

```text
2484 < 250000 - 9
QUERY_BUDGET_EXHAUSTION=false
```

`limit_exceeded` 在 Validator 中还可能来自：

1. `cell.depth >= max_subdivision_depth`；
2. `!split_w && !split_v`，即两个轴均不可切；
3. query budget（已用数值排除）。

G6 的唯一目标是精确回答：

> 101 次 inward search 的最后/最有代表性失败到底在哪个 cell、哪条 limit 分支、哪类证明
> 或真实 clearance 上发生？它是 depth/unsplittable proof terminal、真实 occupied-volume
> clearance不足、geometry/regularity失败、还是 snapshot/path/point 身份错位？

G6 不写任何实现，不调 depth/query/clearance，不添加 observability 到产品源码。

## 1. 永久冻结边界

G6 不得修改任何 workspace source、test、launch、config、parameter、CMake、package、message、
schema 或已有 G1--G5 execution/self-audit 文档。允许写入的只有：

```text
docs/Codex_Tube_V2_G6_Inward_Search_Limit_Flag_Read_Only_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g6_*
```

允许在 `/tmp` 创建：

- gdb command files 和输出；
- bag topic slices、TSV/CSV/JSON、field maps；
- 只读解码脚本；
- 独立 shadow diagnostic source/binary；
- unchanged-binary 独立 ROS 的 logs/bag/manifest。

这些文件不得进入 workspace/CMake，也不得成为 production runtime 依赖。

禁止：

1. 修改 Validator/Builder 或临时加 production log/topic/field；
2. 调高 `max_subdivision_depth=12` 或 `max_query_samples=250000`；
3. 降低 `planner_safe_distance=0.4`、epsilon、cover、regularity 或 map inflation；
4. 删除任何非退化 half-voxel 或 certified cover term；
5. 添加 cache、retry、gate、warmup、sleep、timer、parameter 或第二套 Validator；
6. 修改 inward family、scale、attempt count、selection tolerance 或 family order；
7. 修改 Filter、Epoch、Runtime、Pair、H2、planner、map、adapter、launch 或 schema；
8. 用 observe-only、zero-only、减速、延迟 goal、改 map 或改 scenario 制造通过；
9. 把 `limit_exceeded=1` 自动等同于 query budget；
10. 在没有同一 candidate/map/snapshot/path owner 证据时混用 G4/G5 不同 run 的点或 clearance。

## 2. G6-T0：身份、hash 和现有证据冻结

先完整读取：

```text
AGENTS.md
docs/Codex_Tube_V2_G2_Offset_Authority_First_False_And_Minimal_Correction_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G2_Offset_Authority_First_False_And_Minimal_Correction_Self_Audit_2026-08-21.md
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Self_Audit_2026-08-21.md
docs/Codex_Tube_V2_G4_Current_Anchor_Clearance_Accounting_Read_Only_Audit_2026-08-21.md
docs/Codex_Tube_V2_G5_Exact_Current_Anchor_Sampled_Cover_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G5_Exact_Current_Anchor_Sampled_Cover_Self_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md 顶部 G5 段
本 G6 spec
```

建立：

```text
/tmp/tube_v2_g6_t0_20260821_<id>/
```

保存 HEAD、`git status --short`、`git diff --stat`、`git diff --check`、G5 T0/T1 hashes、
corrected readiness `run_identity.txt`、`bag_info.txt`、`identity_chain.tsv`、
`new_first_false.tsv` 和 external cleanup proof。

记录并冻结至少以下 SHA-256：

```text
tube_surface_validator.h/.cpp/test
certified_tube_builder.h/.cpp/test
tube_builder.h/.cpp/test
tube_filter.h/.cpp/test
tube_epoch_manager.h/.cpp/test
phase_offset_runtime.h/.cpp/test
distance_query.h
tube_types.h
cloud_occupancy_snapshot.h/.cpp/test
phase_offset_cloud_occupancy_query.h/.cpp/test
phase_offset_matched_adapter.cpp/test
phase_offset_esdf_tube_single.launch
```

G6 结束时全部 source/test/launch hash 必须与 T0 完全一致。任何变化都视为边界违规；保留
用户文件、停止实现，不得自行 restore/reset。

## 3. G6-T1：静态重建 101 次 inward attempt

逐行追踪：

```text
CertifiedTubeBuilder::build
-> full-width SurfaceValidator result
-> RetryableValidationFailure
-> BOTH_SIDED / POSITIVE_ONLY / NEGATIVE_ONLY
-> PrepareInwardCandidate
-> SurfaceValidator::validate
-> last_inward_validation
-> InwardSearchFailureReason
```

建立 `static_inward_attempt_schedule.tsv`，每行至少包括：

```text
attempt_index
family
family_order
level
scale exact hexadecimal/decimal
scale > kCapacityTolerance
current raw lower/upper
expected filtered lower/upper at current_w
expected current width
has_positive/has_negative eligibility
whether current width > kCapacityTolerance
```

必须证明 101 的精确组成，不可只引用 log：

```text
BOTH_SIDED: scale=2^-1 ... 2^-33       -> 33 attempts
POSITIVE_ONLY: scale=2^0 ... 2^-33    -> 34 attempts
NEGATIVE_ONLY: scale=2^0 ... 2^-33    -> 34 attempts
total=101
```

若源码/浮点实际比较导致边界不同，以 exact IEEE-754 replay 为准，并解释差异。

明确最后 attempt 身份：预期是 `NEGATIVE_ONLY, level=33, scale=2^-33`；使用 candidate 8
current interval复算其 current bounds。还要记录全 profile 其他 knots 是否仍有宽度
`>kCapacityTolerance`，解释为什么 `PrepareInwardCandidate` 仍返回 true。

静态枚举 Validator 的三个 `limit_exceeded=true` owner，建立：

| Owner | Source line/function | Predicate | Can produce 2484? | Already excluded? |
|---|---|---|---|---|

不得把 Builder 的文本 `last_limit_exceeded=1` 当作 owner。

## 4. G6-T2：完整解码 corrected G5 bag，不先开新 ROS

只读使用：

```text
/tmp/tube_v2_g5_readiness_20260821_154000/evidence/g5_readiness.bag
```

解码并保留 field name/index/raw message：

```text
/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
/formation_planning/phase_offset_manual/tube_cloud_snapshot_diagnostics
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
/formation_planning/phase_offset_manual/diagnostics
/formation_planning/phase_offset_manual/base_path
/formation_planning/phase_offset_manual/active_path
/formation_planning/phase_offset_manual/tube
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/frame
/sim/odom
/particle0/gvf/occupancy
/manual_map/occupancy
/rosout
```

对 candidate 7、8、9 建 cohort table；candidate 8 至少恢复：

```text
candidate/map/snapshot stamp
path/tube revision and owner if available
current_w and current sample identity
sample count=48 and requested/certified preview
all raw/current bounds and slopes exposed by markers/schema
cross-section reason and ray terminations
c_plus_raw/c_minus_raw
base/actual/+/- categorical status
current base point and normal if markers allow
all PWL knots/bounds represented by candidate marker
snapshot point cloud identity/resolution/inflation/domain
full-width failure and preserved provenance
terminal active/Pair/selected facts
```

区分：

- collapsed zero-baseline display profile；
- retained `raw_build_samples`/raw diagnostic；
- original filtered profile used by all 101 attempts；
- transient earlier active profile；
- candidate 8/map 919 terminal profile。

不得用 collapsed zero bounds冒充 inward input。

如果 bag 已能精确确定 limit owner、terminal cell 和 failed queries，可跳过新 ROS；必须写
`UNCHANGED_BINARY_GDB_NOT_NEEDED_EXISTING_EVIDENCE_SUFFICIENT`。只要缺 terminal cell/depth/
query values 任一项，进入 T3。

## 5. G6-T3：unchanged-binary gdb 独立 ROS 只读捕获

### 5.1 运行合同

只在 T2 不足时运行一次：

```text
fresh private roscore port
fresh ROS_HOME
new /tmp/tube_v2_g6_gdb_20260821_<id>/
unchanged built formation_planning binary
unchanged phase_offset_esdf_tube_single.launch
phase_offset_manual_observe_only:=false
unchanged pillar.pcd and parameters
actual /sim/local_map >=900 before goal
exactly one goal=(8,0,1)
```

gdb 是 read-only observation；不得 rebuild with changed optimization and call it production
acceptance。只清理本任务 PID/PGID/port，不触碰用户已有 ROS。

### 5.2 Builder attempt capture

在 unchanged binary 的 `CertifiedTubeBuilder::build` inward loop 中，捕获与 candidate 8
同类型的第一个 terminal cohort。每个 attempt至少输出：

```text
map_observation_sequence/current_w
attempt_index/family/family_index/level/scale
current filtered lower/upper/width
inward_complete
first_failure_reason/w
query_sample_count/limit_exceeded
current_anchor_valid
geometry_cell_count/clearance_leaf_cell_count/prequery_cover_split_count
max_depth_observed
split_w/v/both/anisotropic counts
min/max cover/max requested clearance/min clearance margin
retryable result
```

输出 full width + 101 attempts；若 run 的 candidate/map sequence 不等于 G5 corrected run，
使用同一结构 first-false并明确 `STRUCTURALLY_MATCHED_NEW_RUN`，不得混数值。

### 5.3 Limit owner capture

分别在三个 owner 位置设置只读断点或等效 line break：

```text
CellClearancePasses query-budget predicate
ValidateCell depth predicate
ValidateCell !split_w&&!split_v predicate
```

必须输出实际命中的 owner。对最后 inward attempt 的 terminal cell记录：

```text
cell depth,w0,w1,v0,v1
w_span/v_span
can_split_w/can_split_v
geometry_valid
certificate_attempted if recoverable
breakdown fixed/w/v/decomposable
sampled_radius/cover_radius/target_cover
clearance_safe
query count before/after cell
first failure reason/w before/after
```

若 depth owner命中，确认是否 depth exactly 12；若 unsplittable owner命中，精确给出哪个 span
`<=1e-10`。禁止由 query count猜 owner。

### 5.4 Failed 9-point queries

对 terminal cell 本层的全部 9 点保存：

```text
row/column index
w/v/delta
point xyz
cover/requested radius
DistanceStatus
clearance_certified
returned clearance
pass/fail
```

如果 terminal 发生在 cover-too-large prequery branch，明确本层执行 0 query，并追踪最近的
ancestor/child query failure；不能伪造 9 点。

如果 optimization 使关键变量不可见，使用 `/tmp` 独立 shadow replay；不得改 workspace。

## 6. G6-T4：same-snapshot shadow replay 和不等式分类

使用与 terminal cohort 相同 snapshot/path/point。优先从新 gdb run 的录包恢复；若使用 G5
candidate 8/map919，则必须保持 stamp `1787298468.9096804`，不能借用 G4 map919 的不同 stamp。

对每个 failed query计算到 occupied closed voxel AABB union 的 exact nearest distance，并记录
nearest voxel index/AABB。比较：

```text
R_planner = 0.4
R_request = 0.4 + exact branch cover + epsilon
D_closed  = exact nearest occupied closed-volume distance
```

必须分类每点：

```text
D_closed < R_planner                         -> true planner residual violation
R_planner <= D_closed < R_request            -> distinct continuous geometry cover insufficiency
D_closed >= R_request but production failed  -> query/identity/status mismatch
no query because cover>target/depth           -> proof geometry terminal, not clearance fact
```

若 returned clearance 被 requested radius cap住，不能把 cap 当 exact nearest；shadow replay 必须
输出 exact distance。

对 last negative-only `2^-33` attempt，复算 current anchor和 terminal path-cell bounds；明确
它是否已接近 zero centreline，及 failure 是否仍发生在非 current w variation/其他 knots。

## 7. 强制主分类

G6 最终必须选择一个主分类：

### A. TRUE_PLANNER_RESIDUAL_CLEARANCE_INSUFFICIENT

同一 snapshot/point 的 exact closed-volume distance小于 `0.4+epsilon`。这是物理/地图安全
不足；不写实现，不调 margin/depth/budget。

### B. DISTINCT_CONTINUOUS_SURFACE_COVER_CLEARANCE_INSUFFICIENT

centreline residual安全，但 terminal surface/cell 的必要纯几何 cover 后 request大于 exact
clearance。不是重复 half-voxel；不删 cover。

### C. MAX_DEPTH_PROOF_TERMINAL_WITH_UNRESOLVED_CELL

实际命中 depth=12；本层没有足够证据接受，且查询/几何混合仍未消解。报告 terminal cell 和
safe/unsafe点；G6 不提高 depth。

### D. UNSPLITTABLE_NUMERIC_TERMINAL

实际命中 `!split_w&&!split_v`；给出 exact spans 和浮点中点退化。只建议未来最小数值审计，
G6 不改 epsilon/depth。

### E. GEOMETRY_OR_REGULARITY_TERMINAL

Collect/Evaluate/certificate speed/regularity首先失败；给出 exact invariant，不把它写成
clearance不足。

### F. QUERY_OR_IDENTITY_MISMATCH_PROVEN

证明 snapshot/path/current point/radius/status对应错误。只建议下一阶段最小 owner correction。

### G. PROOF_CAPACITY_TERMINAL_WITH_ALL_REQUIRED_CLEARANCE_SAFE

证明所有 required terminal surface点 exact clearance都满足且失败仅来自不必要 proof
scheduling/cell topology；只有此分类才值得建议新的最小 proof-capacity implementation spec。

### H. OBSERVABILITY_INSUFFICIENT

仅在 bag、unchanged-binary gdb 和 shadow replay 全部不能恢复 exact owner/cell/query时允许；
必须列出缺失字段和已穷尽手段。

如果多个现象同时存在，主分类选择最早导致最后有效 nonzero inward candidate被拒绝的事实；
其他作为 secondary。`limit_exceeded` owner和 safety classification必须分别写清，不能混为一个词。

## 8. 验收矩阵

至少完成：

```text
G5 source/test hashes unchanged                         PASS
all protected source/test/launch hashes unchanged       PASS
101-attempt static schedule exact                       PASS
query-budget owner numerically excluded                 PASS
actual limit owner captured                             PASS or H with exhausted evidence
terminal cell identity captured                         PASS or H
terminal query/cover table captured                     PASS or H
same-snapshot exact closed-volume replay                PASS or H
git diff --check                                        PASS
workspace source/test edits during G6                   NONE
task-owned ROS cleanup                                  EMPTY
```

复跑 existing tests只为 no-change smoke，不得修改 oracle：

```text
SurfaceValidator 22/22
CertifiedBuilder 11/11
CloudOccupancySnapshot 14/14
Cloud occupancy query 4/4
```

若 gdb run不需要，不强制 full build；若运行 unchanged binary，记录其 G5 build hash/mtime和
与 G5 full build一致的 identity。

## 9. 文档、handoff 和连续执行边界

完成后把本文件 `DOCUMENT_STATUS` 改为 `COMPLETE`，追加完整 audit record，并更新：

```text
/home/cxq/ISF-GVF/handoff.md
```

必须包含：

1. exact 主分类和 secondary；
2. 101 attempt schedule与最后 attempt identity；
3. actual limit owner源码分支；
4. terminal cell w/v/depth/spans/cover；
5. 9点或 prequery-zero-query事实；
6. exact closed-volume replay；
7. query budget排除不等式；
8. candidate/map/snapshot/path/current identity；
9. protected hashes/no-source-edit；
10. evidence dirs和 cleanup；
11. 是否值得下一阶段最小 implementation spec。

最终状态：

```text
G6_READ_ONLY_AUDIT_COMPLETE_<A_TO_H_CLASSIFICATION>
G6_BLOCKED_OUTSIDE_READ_ONLY_BOUNDARY
```

若分类 A/B/C/D/E，禁止通过乱删 margin、加门控或调 limits绕过；若 F/G，只能建议主代理
另写新的最小 implementation spec，Luna 不得自行实现。完成审计、文档、handoff和 cleanup前
不要停止；也不要跨过只读边界。

## 10. 给 Luna Max 的连续执行指令

```text
你是本阶段唯一执行代理 GPT-5.6 Luna Max，reasoning=max。
禁止创建、调用、委派、请求或等待任何子代理。

完整读取 AGENTS.md、G2--G5点名 specs/self-audits、handoff 顶部和本 G6 spec。连续执行
G6-T0 -> 101 attempt静态重建 -> corrected G5 bag完整解码 -> 必要时 unchanged-binary gdb
独立 ROS -> terminal cell/9点/limit owner捕获 -> same-snapshot closed-volume shadow replay ->
强制分类 -> smoke tests/hash/diff/cleanup -> audit/handoff。不要在小步骤停下来确认。

全程只读产品源码/test/launch。不要修改 Validator/Builder，不要调 depth/query/clearance，
不要添加 log/topic/schema/cache/gate/parameter/second Validator。必须明确区分 query budget、
depth、unsplittable、真实 clearance、geometry和identity；2484 queries明确不是 query-budget
exhaustion。如果出现下一层事实，只取证分类，不实现。

```

## 11. G6 final audit record

Audit date: 2026-08-21. No workspace source, test, launch, configuration,
parameter, schema, CMake, or product-runtime file was changed during G6.

### 11.1 Classification and boundary

Primary classification:

```text
C_MAX_DEPTH_PROOF_TERMINAL_WITH_UNRESOLVED_CELL
```

Secondary classifications:

```text
STRUCTURALLY_MATCHED_NEW_RUN
G6_OBSERVABILITY_LIMITATION_EXACT_G5_TERMINAL_NOT_RECOVERED
```

The primary owner was recovered in an unchanged-binary GDB run: the
`ValidateCell` depth guard at
`tube_surface_validator.cpp:513` (`cell.depth >=
config.max_subdivision_depth`). The captured terminal cell reached depth 12,
the unchanged configured maximum. No `!split_w && !split_v` owner hit was
observed. The GDB run did not recover the exact G5 candidate-8 final
validation/9-point table, so G6 does not classify the unresolved cell as
clearance-safe or clearance-unsafe and does not claim a closed-volume safety
result.

### 11.2 Exact G5 identity and static inward schedule

The corrected G5 evidence remains the authoritative exact identity:

```text
candidate=8
map_observation_sequence=919
snapshot_stamp=1787298468.9096804
current_w=3.113361964553144
current_interval≈[-0.01875,1.4875]
```

The exact static schedule is preserved in
`/tmp/tube_v2_g6_t0_20260821_161359/static_inward_attempt_schedule.tsv`:

```text
BOTH_SIDED       33 attempts, levels 0..32, scales 2^-1..2^-33
POSITIVE_ONLY    34 attempts, levels 0..33, scales 2^0..2^-33
NEGATIVE_ONLY    34 attempts, levels 0..33, scales 2^0..2^-33
total            101 attempts
last             NEGATIVE_ONLY, level 33, scale 0x1.0000000000000p-33
```

At the last attempt, the current bounds are
`[-2.182787284255028e-12, 0.0]`, width
`2.182787284255028e-12 < kCapacityTolerance (1e-10)`. The corrected bag does
not expose every other knot's width; however, the 101 successful
`PrepareInwardCandidate` entries establish that an eligible nonzero knot
remained for each attempt.

The last G5 Validator record reports `last_queries=2484`. This is not query
budget exhaustion: the budget predicate requires
`query_count + 9 > 250000`, while `2484 < 250000 - 9`.

### 11.3 Captured limit owner and terminal cell

The one allowed independent unchanged-binary run is recorded in
`/tmp/tube_v2_g6_gdb_20260821_171500/`. It used private ROS port 12917,
fresh `ROS_HOME`, `observe_only=false`, the unchanged
`phase_offset_esdf_tube_single.launch`, `pillar.pcd`, at least 900 local-map
observations before one `(8,0,1)` goal, and reached the goal. Its dynamic
candidate/current values are not substituted for the exact G5 candidate-8
values above; the run is evidence of the same terminal structure only.

The final depth-owner row in
`gdb_depth_owner_compact.tsv` is:

```text
current_w              2.1092223659487712
depth                  12
w0,w1                  4.0386765291196118, 4.0386937522243063
w_span                 1.7223104694430447e-05
v0,v1                  0, 0.000244140625
v_span                 0.000244140625
query_count            22374
geometry_cell_count    2513
clearance_leaf_cells   2486
prequery_cover_splits  27
max_depth_observed     12
limit_exceeded         true
first_failure_reason   INSUFFICIENT_CLEARANCE
```

The depth owner is distinct from the query-budget owner at
`CellClearancePasses` (line 390) and the unsplittable owner at line 540.
The latter had no observed hit. Cover radius, requested radius, and the
terminal nine-point rows were not recoverable without adding production
observability or perturbing the run; they are intentionally reported as
unresolved rather than reconstructed or inferred.
Path/tube revision-owner fields were likewise not exposed at the exact
terminal, and no prequery-zero-query fact is claimed.

### 11.4 Replay, hashes, tests, and cleanup

Because the exact G5 terminal cell/points were not recovered, G6 does not
claim an exact same-snapshot closed-volume replay or a true clearance
classification. The G5 snapshot stamp above was not mixed with the
structurally matched GDB run. The evidence directories are:

```text
/tmp/tube_v2_g6_t0_20260821_161359/
/tmp/tube_v2_g6_gdb_20260821_171500/
/tmp/tube_v2_g6_t1_smoke_20260821_171500/
```

Protected-hash comparison: 28 matching files, 0 mismatches, with one
genuinely missing pre-existing test file
`src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_runtime_test.cpp`.
The smoke tests passed: SurfaceValidator 22/22, CertifiedBuilder 11/11,
CloudOccupancySnapshot 14/14, and cloud occupancy query 4/4.
`git diff --check` passed. The independent cleanup proof
`/tmp/tube_v2_g6_gdb_20260821_171500/cleanup_external_check.txt` has empty
task-owned process and port sections.

### 11.5 Final stop decision

```text
G6_READ_ONLY_AUDIT_COMPLETE_C_MAX_DEPTH_PROOF_TERMINAL_WITH_UNRESOLVED_CELL
G6_BLOCKED_OUTSIDE_READ_ONLY_BOUNDARY
```

No implementation recommendation is made. Do not increase subdivision depth
or query budget, alter clearance/margins, remove cover terms, add production
instrumentation, or run another ROS/GDB session under G6. Any continuation
requires a new bounded specification from the main agent.
