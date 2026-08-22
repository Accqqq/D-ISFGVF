# Tube V2 G4 current-anchor clearance-accounting read-only audit

```text
DOCUMENT_ROLE=READ_ONLY_AUDIT_EXECUTION_SPEC
DOCUMENT_STATUS=COMPLETE
STAGE=TUBE_V2_G4
CLASSIFICATION=B_DUPLICATE_CLEARANCE_ACCOUNTING_PROVEN
IMPLEMENTATION_AUTHORIZED=false
SOURCE_OR_TEST_EDITS_AUTHORIZED=false
AUTO_ADVANCE=FREEZE_TO_STATIC_ACCOUNTING_TO_EXACT_RUNTIME_QUERY_REPLAY_TO_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、调度、证据边界和下一阶段判断；Luna max 单代理负责只读审计、
离线/独立 ROS 取证、自审和 handoff。

Luna max 禁止创建、调用、委派、请求或等待任何子代理。不得把代码阅读、数学核算、
bag 解码、调试器运行、ROS launch、证据处理或文档交给其他模型。

## 0. G3 已冻结事实

G3 状态：

```text
G3_IMPLEMENTATION_COMPLETE_QUERY_BUDGET_RESOLVED_DYNAMIC_ACCEPTANCE_UNMET
CURRENT_ANCHOR_INSUFFICIENT_CLEARANCE_REFINED_TO_MAX_DEPTH
canonical=G3_PROOF_CAPACITY_CORRECTED_NEW_FIRST_FALSE
```

G3 readiness evidence：

```text
/tmp/tube_v2_g3_readiness_20260821_145100/
candidate_seq=7
map_seq=919
current_w=3.1851342137905054
first_stop_w=3.185134214
last_queries=117=13*9
max_subdivision_depth=12
query_budget_terminal=false
unsplittable_terminal=false
goal_reached=true
Pair/selected/retained_nonzero=false
```

G3 已证明查询容量不再是 first-false。G4 不继续优化 subdivision，也不尝试让动态验收通过；
唯一目标是精确回答：

> 当前锚点的 `INSUFFICIENT_CLEARANCE` 是真实物理/地图安全余量不足，还是 Builder、
> SurfaceValidator 与 closed-voxel clearance query 之间存在重复或错位的 clearance/cover
> accounting？

在这个问题得到可复核答案前，禁止写任何实现。

## 1. 不可改变的边界

G4 全程只读。禁止修改任何工作区源码、测试、launch、config、参数、CMake、package、消息、
schema 或已有 execution spec。允许写入的只有：

```text
docs/Codex_Tube_V2_G4_Current_Anchor_Clearance_Accounting_Read_Only_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g4_*/
```

允许在 `/tmp` 创建一次性脚本、gdb command file、CSV、bag slice、独立编译的诊断程序和日志；
它们不得被加入 workspace/CMake，不得成为运行时依赖，并须在 evidence manifest 中逐项列出。

永久禁止：

1. 调低 `planner_safe_distance`、cover、epsilon、regularity 或任何 margin；
2. 调高 `max_subdivision_depth` 或 `max_query_samples`；
3. 添加 cache、门控、retry、warmup、timer、sleep、参数或第二套 Validator；
4. 修改 TubeBuilder、Filter、EpochManager、Runtime、Pair、H2、planner、adapter 或 map production；
5. 用 zero-only、observe-only、减速或延迟 goal 制造通过；
6. 把 categorical KNOWN_FREE 当成 clearance 足够；
7. 用当前飞机实际位置的安全代替 base/current-anchor surface 的安全；
8. 在没有同一 snapshot、同一 path owner、同一 current anchor 身份证据时混用不同 run 的数值。

G3 的四个源码/测试文件立即冻结，不得继续修改。

## 2. G4-T0：身份与证据冻结

先完整读取：

```text
AGENTS.md
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Self_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md 顶部 G3 段
docs/Codex_A5_Clearance_Accounting_Knot_Execution_Spec_2026-08-18.md
docs/Codex_A5_Clearance_Accounting_Knot_Self_Audit_2026-08-18.md
docs/Codex_Clearance_Margin_Alignment_Audit_Execution_Plan_2026-08-18.md
docs/Codex_Clearance_Margin_Alignment_Audit_Self_Audit_2026-08-18.md
docs/Codex_A5_Generic_OffsetTube_Continuous_Cover_Execution_Spec_2026-08-18.md
docs/Codex_A5_Generic_OffsetTube_Continuous_Cover_Self_Audit_2026-08-18.md
```

建立：

```text
/tmp/tube_v2_g4_t0_20260821_<id>/
```

保存 HEAD、`git status --short`、`git diff --stat`、`git diff --check`、G3 status、G3 evidence
manifest，并记录下列文件 SHA-256：

```text
tube_cross_section.h/.cpp
tube_builder.h/.cpp
tube_surface_validator.h/.cpp
certified_tube_builder.h/.cpp
tube_epoch_manager.h/.cpp
distance_query.h
tube_types.h
phase_offset_cloud_occupancy_query.h/.cpp
cloud_occupancy_snapshot.h/.cpp
sdf_map.cpp
phase_offset_matched_adapter.cpp
phase_offset_clearance_audit.h/.cpp
phase_offset_esdf_tube_single.launch
```

保存 G3 四个 changed files 的 final hashes。后续任何源码 hash 变化都视为 G4 边界违规，立即
停止写实现并报告；不得自行恢复用户文件。

## 3. G4-T1：解码既有 G3 run，不启动新 ROS

只针对必要 topic 使用 bag index 解码，不全量回放 4.4 GB bag。把字段名、字段序号、原始行和
解析结果同时保存，禁止只留手工摘要。

### 3.1 Candidate/current-anchor 身份

从以下 topic 解码 candidate 5、6、7：

```text
/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
/formation_planning/phase_offset_manual/tube_cloud_snapshot_diagnostics
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
/formation_planning/phase_offset_manual/diagnostics
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/active_path
/sim/odom
/rosout
```

至少输出：

```text
candidate_sequence / map_observation_sequence / snapshot observation stamp
requested preview / current_w / current sample found+complete
current c_plus_raw / c_minus_raw
cross-section reason / positive and negative ray termination
current base categorical status / +/- ray-step status / actual-position status
residual/full effective radius / preincluded map uncertainty
candidate classification/completeness/obstacle certification
first_stop_reason / first_stop_w / last_queries / last_limit_exceeded
base-current point and actual-position separation（若 marker 足以恢复）
```

先确认 candidate 7 的 raw diagnostic 和 zero-baseline log 属于同一个 candidate/map cohort；
不允许把 candidate 7 的 current_w 与 candidate 5/6 的 snapshot 或 bounds 混合。

### 3.2 初始数值边界

对 candidate 7 建立只读 accounting 表：

```text
R_cross_section = planner_safe_distance
R_surface       = planner_safe_distance + cover_radius + cover_epsilon
cover_radius    = fixed_cover + w_reducible_cover + v_reducible_cover
fixed_cover     = 0.5 * snapshot_resolution（当前 production 定义，仅记录）
```

明确区分：

- categorical occupancy status；
- clearance query 的 nearest occupied closed-voxel-volume distance；
- Builder ray boundary `c_plus_raw/c_minus_raw`；
- filtered/inward delta width；
- Surface 的 Euclidean geometry cover；
- snapshot 中已包含的 map inflation；
- planner authoritative safe distance。

不得把 `c_minus_raw=0.025` 直接冒充 nearest clearance；只能把它作为在固定
`R_cross_section` 下沿 normal 的 safe-boundary 证据，并保留 `ray_step` 与
`boundary_tolerance` 误差。

## 4. G4-T2：静态 clearance/cover accounting 审计

逐行追踪同一个 occupied evidence 从 PointCloud2 到最终 Surface query：

```text
SDFMap cloud callback
-> CloudOccupancySnapshot build（voxelization + included inflation）
-> queryCloudOccupancySnapshotClearance
-> nearest distance to closed occupied voxel AABB
-> makeCloudOccupancyClearanceQuery
-> TubeCrossSectionSolver centre/ray query at planner_safe_distance
-> Builder continuous inset/cell geometry certificate
-> SurfaceValidator cover_radius
-> requested_radius = planner_safe_distance + cover + epsilon
```

必须回答并引用源码行/历史规格：

1. `queryCloudOccupancySnapshotClearance` 返回到 occupied voxel centre 还是 closed voxel
   volume 的距离？
2. snapshot `included_map_inflation` 已包含什么，不包含什么？
3. `planner_safe_distance=0.4` 是唯一 production residual radius，还是又与 margins 重复？
4. Builder 的 fixed continuous inset、cell-certificate inset 和 Surface fixed half-voxel 分别
   证明什么误差？
5. G1 local-PWL certificate 的 midpoint/normal/slope/width 项是否已经是纯 Euclidean surface
   cover？
6. 当 `w0=w1=current_w` 且 inward width 接近零时，w/v reducible cover 各自应是多少？
7. `0.5*snapshot_resolution` 是对 surface sampling、voxel occupancy support、Builder
   interpolation，还是其他误差的唯一一次 charge？
8. closed-voxel-volume distance 与 fixed half-voxel 同时出现是否是两个不同证明义务，还是同一
   voxel support 被重复 charge？
9. 如果去掉任一项，是否会对任意未采样 surface point 或 occupied voxel volume 产生真实漏证？

必须画出一张 exact accounting ledger，形式至少为：

| Term | Numerical value | First introduced at | Physical/proof meaning | Already represented upstream? | Duplicate? |
|---|---:|---|---|---|---|

结论不能依赖“看起来保守”。每个 term 必须有单位、几何对象和不等式方向。

## 5. G4-T3：精确 runtime query 数值取证

若 T1 现有 bag 已经包含 exact `point / requested_radius / returned status / returned clearance /
cover breakdown / snapshot resolution`，直接使用，不新开 ROS。若缺任一关键量，运行一次新的
独立 ROS 观察，不修改产品源码。

### 5.1 新 run 基本合同

```text
fresh private roscore port
fresh ROS_HOME
fresh evidence directory /tmp/tube_v2_g4_readiness_20260821_<id>/
unchanged phase_offset_esdf_tube_single.launch
unchanged pillar.pcd
unchanged parameters
Phase A actual local-map messages >= 900
one and only one goal (8,0,1)
```

启动前记录 `rosparam` effective values，包括 map resolution、obstacle inflation、
`planner_safe_distance`、ray step、boundary tolerance、preincluded uncertainty 和所有相关
Tube margins。结束后必须有 task-owned process/port 空证明。

### 5.2 观察手段优先级

只允许以下非产品观察手段，按顺序尝试：

1. 使用现有 diagnostics/markers/bag 精确恢复；
2. 使用 `/tmp` gdb command file 对现有带符号二进制做 one-run 只读断点，捕获
   `CellClearancePasses` 当前锚点调用和 `queryCloudOccupancySnapshotClearance` 返回；
3. 若编译优化使变量不可见，在 `/tmp` 创建独立 shadow diagnostic harness，从同一 run 的
   PointCloud2、odom、path/tube marker 和 effective params 重建 snapshot/query；不得改 workspace
   源码或 CMake。

禁止添加 production ROS log、topic、schema 或临时源码 instrumentation。禁止为了方便调试修改
优化级别后把该二进制当 readiness 结果；shadow harness 只用于数值复算，并须与 production
categorical/status/boundary facts交叉验证。

### 5.3 每个 current-anchor leaf 必须记录

至少记录第一次 full-width current-anchor failure和最后一次 inward current-anchor failure：

```text
candidate/map/snapshot identity
depth
w0,w1,v0,v1
all 9 point coordinates and equality/near-equality关系
filtered lower/upper at current_w
fixed/w/v cover breakdown and total cover
cover_epsilon
requested_radius
query status
nearest occupied closed-voxel-volume distance
clearance_certified
query pass/fail
nearest occupied voxel AABB/index（若现有 query 可得；否则 shadow replay 得出）
observed/grid box domain checks
```

验证 `117=13*9` 的每层 requested radius、point 和 returned clearance 是否不变；若因 v interval
缩小而变化，给出完整序列，不能只报告首尾。

## 6. G4-T4：counterfactual 仅离线审计

在完全相同 snapshot 与 point 上，仅离线比较以下半径，不改变 production：

```text
R0 = planner_safe_distance
R1 = planner_safe_distance + pure geometric cover + epsilon
R2 = planner_safe_distance + current production cover + epsilon
```

其中 `pure geometric cover` 是否包含 half-voxel，必须由 T2 证明决定，不能预设答案。

输出每个半径的：status、certified lower bound、nearest occupied voxel-volume distance、pass/fail。
若 query API 在未发现障碍时把 clearance cap 到 requested radius，要明确“返回值相等”只证明
lower bound，不等于真实最近距离。

严禁把离线 counterfactual 的通过直接当成修改授权。

## 7. 强制分类

G4 最终必须且只能选择一个主分类：

### A. TRUE_CURRENT_ANCHOR_CLEARANCE_INSUFFICIENT

同一 snapshot/point 下，nearest occupied voxel-volume distance 确实小于无重复 accounting 的
required radius。结论：真实安全不足；不写实现、不调参数。

### B. DUPLICATE_CLEARANCE_ACCOUNTING_PROVEN

用不等式和 exact source ownership 证明同一物理误差被 charge 两次，且删除重复项不会减少任一
continuous surface/occupied-volume 证明义务。只写下一阶段最小 correction 建议，不在 G4 实现。

### C. QUERY_OR_IDENTITY_MISMATCH_PROVEN

证明 Surface query 使用了错误 snapshot、path owner、current anchor、point 或 radius。只记录最小
owner/identity correction 边界，不在 G4 实现。

### D. CONSERVATIVE_BUT_DISTINCT_OBLIGATIONS

各项均有不同且必要的证明对象；当前 clearance 位于 CrossSection 可用而 Surface continuous cover
不可用的窄区间。结论：不是 bug，也不是容量问题；不实现。

### E. OBSERVABILITY_INSUFFICIENT

只在上述 read-only/gdb/shadow replay 全部无法恢复 exact 数值时允许。必须列出缺失字段和已穷尽
手段；不得用猜测进入实现。

若证据同时支持多个现象，主分类按最早导致 current-anchor rejection 的因果项选择，其他作为次级
发现。

## 8. 验收矩阵

G4 至少完成：

```text
G3 source hashes unchanged                         PASS
all protected source hashes unchanged              PASS
G3 SurfaceValidator 18/18                          PASS（只复跑，不重编实现）
CertifiedBuilder 11/11                             PASS
CloudOccupancySnapshot focused tests               PASS
phase_offset_cloud_occupancy_query focused tests   PASS
git diff --check                                   PASS
workspace source/test modified by G4               NONE
task-owned ROS cleanup                             EMPTY
```

若新 ROS 未运行，明确写 `NOT_NEEDED_EXISTING_EVIDENCE_SUFFICIENT`，不要伪造 cleanup run。

## 9. 文档和 handoff

最终写：

```text
docs/Codex_Tube_V2_G4_Current_Anchor_Clearance_Accounting_Read_Only_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
```

文档必须包含：

1. exact 主分类；
2. accounting ledger；
3. candidate/map/snapshot/current-anchor 身份链；
4. exact runtime query values和 13 层序列；
5. closed voxel volume 与 half-voxel charge 的判定；
6. counterfactual 结果；
7. 所有 source hashes与 no-source-edit证明；
8. ROS 或 existing-bag evidence目录；
9. cleanup proof；
10. 下一阶段是否值得写 implementation spec。

若分类为 B 或 C，只能建议主代理另写新的最小 implementation execution spec；Luna 不得自行实施。
若分类为 A 或 D，明确禁止通过调 clearance/depth/budget 或新增机制绕过真实安全事实。

持续执行到审计、文档和 handoff 全部完成；不要在小步骤停下来确认。仅当继续需要修改 workspace
源码/测试或触碰未授权外部进程时，保留证据并报告边界，不得自行扩大权限。

---

## 10. G4 completed audit record

The execution above is complete. The exact classification is:

```text
B. DUPLICATE_CLEARANCE_ACCOUNTING_PROVEN
```

The scope is only the exact current-anchor sampled fallback where
`w0 == w1 == current_w`, including its inward-halving descendants. At fixed
`w`, `r(v)=p+N*delta(v)` is affine in `v`; the nine sampled points already
bound that cross-section variation. The immutable cloud query measures distance
to closed occupied-voxel volumes, so the additional `0.5*resolution=0.05 m`
fixed term has no independent owner in this branch. Removing that one term in
the same point/snapshot counterfactual passes. This does not generalize to
nondegenerate certified cells or non-certified nondegenerate fallback cells.

### 10.1 Identity chain

Evidence: `/tmp/tube_v2_g4_t1_20260821_144853/identity_chain.tsv`.

| Field | Value |
|---|---|
| G3 runtime/bag | `/tmp/tube_v2_g3_readiness_20260821_145100` |
| Candidate sequence | `7` |
| Map observation sequence | `919` |
| Snapshot stamp | `1787292633.0334985` |
| Current `w` | `3.1851342137905054` |
| Preview | `[2.9851342137905053,5.185134213790505]` |
| Current sample | found `1`, complete `1` |
| Current base point | `(3.286887963172443,0.34280071140075674,1.0002026651862932)` |
| Base/actual separation | `0.026347543858429273 m` |
| Cloud points | `43600` |
| Resolution | `0.1 m` |
| Included inflation | `0.1 m` |
| Planner radius | `0.4 m` |
| Raw rays | `c_plus_raw=1.475`, `c_minus_raw=0.025` |
| Categorical statuses | base/actual/+/− all `KNOWN_FREE` |
| Zero-baseline log | `attempts=101`, `last_queries=117=13*9`, `last_limit_exceeded=1` |

The shadow replay reconstructed occupied voxel `(132,148,10)` with closed AABB
lower `(3.2,-0.2,0.99)`, upper `(3.3,-0.1,1.09)`, and exact nearest closed-
volume distance `0.44280071140075605 m`.

### 10.2 Accounting ledger and ownership

| Term | Value | Owner/meaning | Already represented upstream? | Duplicate in exact degenerate branch? |
|---|---:|---|---|---|
| `planner_safe_distance` | `0.4 m` | Sole production residual Euclidean radius | No | No |
| Included map inflation | `0.1 m` | Voxelized occupied-set support | Yes, once in snapshot | No |
| Closed voxel-volume distance | `0.44280071140075605 m` exact nearest | Point-to-union-of-closed-AABB distance | Query itself | No; it owns voxel support |
| Sampled radius | `1.4551915228366853e-12 m` at depth 0, tending to `3.552713678800501e-16 m` | `p+N*delta(v)` variation over 3x3 samples | No | No |
| Sampled factor | `1.1` | Conservative multiplier on sampled radius | No | No |
| Fixed half voxel | `0.05 m` | Legacy discretization/support charge | Closed-volume query already includes voxel support here | **Yes, only here** |
| Cover epsilon | `1e-6 m` | Numerical strictness | No | No |

Source ownership is visible at `cloud_occupancy_snapshot.h:96-101`,
`cloud_occupancy_snapshot.cpp:372-407,440-547`, `tube_builder.cpp:1054-1084`,
and `tube_surface_validator.cpp:303-413,642-643`. The query scans occupied
voxel AABBs using `pointToClosedVoxelDistance` and caps free results at the
requested radius while retaining `clearance_certified`; it does not measure to
voxel centres. The Builder ray values are boundary evidence, not nearest
clearance. The Builder continuous inset remains a separate geometry contract
and is not removed by this audit.

### 10.3 Exact 13-layer current-anchor sequence

Evidence: `current_anchor_query_sequence.tsv`. Every row has the same nine
points `p+N*delta(v)`, status `KNOWN_FREE`, certified returned lower bound
`0.44280071140075605 m`, and query failure because the request is larger.

| Depth | `v1` | Sampled radius (m) | Requested radius (m) | Returned (m) | Result |
|---:|---:|---:|---:|---:|---|
| 0 | 1 | `1.4551915228366853e-12` | `0.4500010000016007` | `0.44280071140075605` | fail, split v |
| 1 | 0.5 | `7.275957614183426e-13` | `0.45000100000080034` | `0.44280071140075605` | fail, split v |
| 2 | 0.25 | `3.637978807091713e-13` | `0.45000100000040016` | `0.44280071140075605` | fail, split v |
| 3 | 0.125 | `1.8189894035458566e-13` | `0.4500010000002001` | `0.44280071140075605` | fail, split v |
| 4 | 0.0625 | `9.094947017729283e-14` | `0.45000100000010007` | `0.44280071140075605` | fail, split v |
| 5 | 0.03125 | `4.5474735088646414e-14` | `0.45000100000005` | `0.44280071140075605` | fail, split v |
| 6 | 0.015625 | `2.2737367544323207e-14` | `0.450001000000025` | `0.44280071140075605` | fail, split v |
| 7 | 0.0078125 | `1.1368683772161604e-14` | `0.4500010000000125` | `0.44280071140075605` | fail, split v |
| 8 | 0.00390625 | `5.684341886080802e-15` | `0.45000100000000626` | `0.44280071140075605` | fail, split v |
| 9 | 0.001953125 | `2.8421709430404014e-15` | `0.45000100000000315` | `0.44280071140075605` | fail, split v |
| 10 | 0.0009765625 | `1.4210854715202005e-15` | `0.45000100000000154` | `0.44280071140075605` | fail, split v |
| 11 | 0.00048828125 | `7.105427357601002e-16` | `0.45000100000000076` | `0.44280071140075605` | fail, split v |
| 12 | 0.000244140625 | `3.552713678800501e-16` | `0.4500010000000004` | `0.44280071140075605` | fail, depth>=12 |

The sequence is exactly `117=13*9`; no layer is a query-identity mismatch.

### 10.4 Counterfactuals

Evidence: `counterfactuals.tsv`, `shadow_replay_results.json`, and
`shadow_replay_table.tsv`.

| Radius | Requested | Returned | Exact nearest | Status/certified | Pass |
|---|---:|---:|---:|---|---|
| `R0=0.4` | `0.4` | `0.4` (cap) | `0.44280071140075605` | `KNOWN_FREE` / yes | **yes** |
| `R1=0.4+1e-6` | `0.400001` | `0.400001` (cap) | `0.44280071140075605` | `KNOWN_FREE` / yes | **yes** |
| `R2=0.4+0.05+1e-6` | `0.4500010000000004` | `0.44280071140075605` | `0.44280071140075605` | `KNOWN_FREE` / yes | **no** |

The cap in R0/R1 is only a certified lower-bound result; it is not the exact
nearest distance. R2 fails against the same exact closed AABB distance.

### 10.5 Verification, hashes, and cleanup

The four focused binaries were rebuilt only through existing CMake targets and
then run unchanged:

```text
phase_offset_tube_surface_validator_test       18/18 PASS
phase_offset_certified_tube_builder_test       11/11 PASS
cloud_occupancy_snapshot_test                  14/14 PASS
phase_offset_cloud_occupancy_query_test          4/4 PASS
git diff --check                                 PASS
source/test edits during G4                    NONE
```

Cleanup proof: `/tmp/tube_v2_g4_t1_20260821_144853/cleanup.txt` is empty for
ports `12892-12901` and task-owned processes. No new ROS run was needed.

Every row in `source_hashes_normalized.tsv` matches (`25/25`). The complete
protected hash list is retained in `g4_final_source_hashes.txt`; the four G3
test hashes are:

```text
b2f622613587e48bfc6156d02be530e8adfb57dbe2b4f7248caca168967d7c1e  src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
7ffa20fdc5ad252674675dda3dd1b34c12fccb5a236e0c5bf11ed7d671a3fcf3  src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
```

The 21 protected production/launch hashes are recorded verbatim in the same
evidence file and were unchanged from T0; no G4 source/test hash changed.

```text
ff732941a651ab161ae3edb99feeffd31f7c31f6671c6c7f388efc90e0ad5d40  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h
7682c0fcb503a5e7be8c78eae4151c7daecd5821c4854e3b10114ae000d8f8fa  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp
5e73f36372e17d45ad9beadace81829a6ba9c6b6417bd195b8ee8f5f2aa9647a  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
d9248801e9f34629c269f0bb11dcf0daa698cded03dec2f59ca4db3fdd7893f3  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
577bb4bb1da19a9f0d7ad61c2811d0c5588181af0477795215f406c759cff9c4  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
6d5b641dbfa13df5c98915d2cf234c75f0349754e6a306c63ede5b38931cc21a  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
6dcf9856d8f3aeaa459ea5b76f151346e6e7dec5453007cef64e5aa8357f475e  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h
475fc672178758cad17fa9c2099c462e1feca2cd5ca6f7154766025884e5a1ad  src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
1dac43a00e6dc18f9e6eaf956c7ed0f0672bc997b1a8a69fc5df4022576c3cce  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
2a0af9e58dd7d77e89676e916f4daf4a15a63b9ee148fff51b1d34c95dc34d8a  src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
8e3d45092c82117da98e11b8cc7049da98105dba5312493f46493051d1b2ccea  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/distance_query.h
e11dd5777f542cdd147bd05bb564ea0afa3c6a314510b26b1164c2ea55d12e7b  src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
2ba7afba129c93b7b09369b7e180bbb99a12053d592f72e78f5db5ca0c14ccde  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_cloud_occupancy_query.h
a80f7a4470e94548bf103e4ae4253e829f2a62c83b820f44b5e6a4deea871094  src/swarm_planner/bspline_traj/src/integration/phase_offset_cloud_occupancy_query.cpp
6249c03ad460f55cf4f58360d96082b5d9959d8e51bcdee25e3c9bf5a82ea9f9  src/swarm_planner/plan_env/include/plan_env/cloud_occupancy_snapshot.h
06902ef0eef8fc8871999d55531802ccb2b076f332e5aaf5a8d40a9e396c468e  src/swarm_planner/plan_env/src/cloud_occupancy_snapshot.cpp
1c981450292795244a63be56aae8b18bd33a331aa00c0d84aadb5cd083a1fc40  src/swarm_planner/plan_env/src/sdf_map.cpp
4f324cb9f83fc7d60fbea9bc32338244cfcc00292660165d9ac6f930ca93442a  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
e4aa0089276e7109592045d303ae9d9fc86f268abaf421e11218847040c1983f  src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
d9eb73d08fd7893af82214d3a07734cf6c118a7ef7d41c7ce1dec0853dfbfc13  src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
334d807751ad6aa1739c241f07b5e22c554a3ff2470838e975a9cad8009e3813  src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
```

### 10.6 Next-stage boundary

G4 authorizes no implementation. A new G5 execution specification is justified
only for the exact `w0==w1==current_w` sampled-fallback accounting branch:

1. preserve planner residual `0.4`, epsilon, closed-volume query semantics,
   nine-point sampling, fail-closed statuses, and current v subdivision;
2. remove or reassign only the fixed half-voxel charge in that branch after a
   proof-oriented unit test and exact same-snapshot replay;
3. do not globally remove half-voxel accounting from certified or other
   fallback cells;
4. do not alter depth, query budget, map inflation, raw ray rules, Builder,
   Runtime, Pair, H2, planner, parameters, schema, or launch behavior.
