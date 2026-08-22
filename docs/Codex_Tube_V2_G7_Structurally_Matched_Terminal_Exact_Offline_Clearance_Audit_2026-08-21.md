# Tube V2 G7 structurally matched terminal exact offline clearance audit

```text
DOCUMENT_ROLE=READ_ONLY_OFFLINE_AUDIT_EXECUTION_SPEC
DOCUMENT_STATUS=COMPLETE
STAGE=TUBE_V2_G7
IMPLEMENTATION_AUTHORIZED=false
SOURCE_OR_TEST_EDITS_AUTHORIZED=false
ROS_EXECUTION_AUTHORIZED=false
GDB_EXECUTION_AUTHORIZED=false
AUTO_ADVANCE=FREEZE_TO_COHORT_SELECTION_TO_GEOMETRY_RECONSTRUCTION_TO_EXACT_CLOSED_VOXEL_CLASSIFICATION_TO_SELF_AUDIT
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、边界、调度和后续实现判断；GPT-5.6 Luna Max 单代理连续完成
既有证据的只读离线恢复、精确性审计、closed-voxel 距离核算、分类、自审计和 handoff。

Luna Max 禁止创建、调用、委派、请求或等待任何子代理。不得将源码阅读、bag 解析、数学
重建、PointCloud2 解码、距离计算、证据审计或文档工作交给其他模型。本阶段不在小步骤等待
确认；在本规格边界内持续执行到完成或形成精确的离线不可恢复证明。

## 0. G6 冻结结论与 G7 唯一目标

G6 已完成，主分类为：

```text
C_MAX_DEPTH_PROOF_TERMINAL_WITH_UNRESOLVED_CELL
```

G6 已经排除：

```text
QUERY_BUDGET_OWNER=false
UNSPLITTABLE_OWNER=false
```

unchanged-binary、独立 ROS、结构匹配 run 的实际 owner 是
`TubeSurfaceValidator::ValidateCell` 的 max-depth guard，terminal cell 为：

```text
evidence=/tmp/tube_v2_g6_gdb_20260821_171500/
bag=/tmp/tube_v2_g6_gdb_20260821_171500/evidence/g6_targeted.bag
current_w=2.1092223659487712
depth=12
w0=4.0386765291196118
w1=4.0386937522243063
w_span=1.7223104694430447e-05
v0=0
v1=0.000244140625
v_span=0.000244140625
query_count=22374
geometry_cell_count=2513
clearance_leaf_cell_count=2486
prequery_cover_split_count=27
max_depth_observed=12
limit_exceeded=true
first_failure_reason=INSUFFICIENT_CLEARANCE
```

两个 span 均仍大于 `1e-10`，只是 depth guard 先终止。G6 没有恢复该 cell 的 exact path/tube
owner、exact nine surface points、cover/requested radius 或同 snapshot 的 closed-voxel distance，
因此没有把它判成真实不安全，也没有判成纯 proof topology false negative。

G7 唯一目标是：

> 只使用已经存在的 G6 targeted bag、G6 GDB 文本、G5/G6 解码证据、当前只读源码和 `/tmp`
> 离线工具，尽最大可验证程度恢复结构匹配 terminal cell 的 exact 3x3 surface table、
> cover/requested radius、对应 immutable cloud cohort，并计算每点到 occupied closed voxel AABB
> union 的精确最近距离；若现有证据不足以做到 exact，必须精确证明缺的是哪个生产状态、为何
> marker/diagnostics 不能无损反演，以及下一次最小只读捕获必须捕获什么。

G7 不是实现阶段，不改 Tube，不开 ROS，不运行 gdb，不提高 depth/query limit，也不弱化安全
裕度。

## 1. 永久冻结边界

不得修改任何 product source、header、test、launch、config、parameter、CMake、package、message、
schema 或 G1--G6 execution/self-audit 文档。允许写入的仅有：

```text
docs/Codex_Tube_V2_G7_Structurally_Matched_Terminal_Exact_Offline_Clearance_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g7_*
```

允许在 `/tmp/tube_v2_g7_*` 创建：

- rosbag 只读导出、JSONL/TSV/CSV；
- PointCloud2 binary 解码结果；
- 离线 Python/C++ 数学脚本和编译产物；
- marker/raw/cloud cohort manifests；
- exact closed-AABB nearest-distance tables；
- hash、diff、process/port 只读审计结果。

这些文件不得进入 workspace/CMake，不得成为 production runtime 依赖。

禁止：

1. 启动 `roscore`、`roslaunch`、ROS node、bag play 或新仿真；
2. 启动/attach gdb、lldb、strace、perf 或修改正在运行的进程；
3. 修改 Validator、Builder、Filter、Epoch、Runtime、Pair、H2、planner、map、adapter、launch；
4. 调整 depth、query budget、clearance、epsilon、cover、regularity、voxel inflation；
5. 删除非退化 half-voxel、certified cover 或 `1.1` sampled factor；
6. 添加 gate、cache、retry、timer、sleep、parameter、schema、module 或第二套 Validator；
7. 把 marker 的线性显示 ribbon 自动当作 production continuous surface；
8. 把 clearance query 返回的 request-cap 值自动当作 exact nearest distance；
9. 混用 corrected G5 candidate 8/map919/snapshot `1787298468.9096804` 与 G6 结构匹配 run；
10. 为了得到“安全”结论使用邻近 message、邻近 map frame 或不同 snapshot，而不写误差/身份证明；
11. reset、restore、checkout、clean、stash、rebase 或改动任何用户已有 dirty worktree 内容。

## 2. G7-T0：规则、证据、源码和 hash 冻结

完整读取：

```text
AGENTS.md
/home/cxq/ISF-GVF/handoff.md 顶部 G6 完成段
docs/Codex_Tube_V2_G6_Inward_Search_Limit_Flag_Read_Only_Audit_2026-08-21.md
docs/Codex_Tube_V2_G5_Exact_Current_Anchor_Sampled_Cover_Self_Audit_2026-08-21.md
本 G7 spec
```

只读追踪至少以下生产路径：

```text
TubeSurfaceValidator::ValidateCell
CollectCellPoints
EvaluateSurfaceRow
CertifiedCellCoverRadius
CellClearancePasses
TubeFilter::query
GeometryEvaluator::preparePath/evaluatePreparedReference
ContinuousPhasePath::evaluate/cellBounds
MakeTimerPathStateQuery/MakeTimerPathCellBoundQuery
MakeCandidateTubeMarkers/AppendBoundaryPoints
raw candidate diagnostics schema/producer
cloud snapshot diagnostics schema/producer
CloudOccupancySnapshot construction and closed-volume query semantics
```

建立：

```text
/tmp/tube_v2_g7_t0_20260821_<id>/
```

保存 HEAD、`git status --short`、`git diff --stat`、`git diff --check`，以及 G6 targeted evidence
目录的递归 file manifest、size、mtime、SHA-256。冻结至少：

```text
tube_surface_validator.h/.cpp/test
tube_filter.h/.cpp/test
certified_tube_builder.h/.cpp/test
cloud_occupancy_snapshot.h/.cpp/test
phase_offset_cloud_occupancy_query.h/.cpp/test
phase_offset_raw_candidate_diagnostics.h/.cpp/test
phase_offset_tube_markers.h/.cpp/test
phase_offset_matched_adapter.h/.cpp/test
continuous_phase_path.h/.cpp/test
geometry.h/.cpp/test
phase_offset_esdf_tube_single.launch
```

复制为只读工作输入或记录 hash：

```text
/tmp/tube_v2_g6_gdb_20260821_171500/evidence/g6_targeted.bag
/tmp/tube_v2_g6_gdb_20260821_171500/gdb_depth_owner_compact.tsv
/tmp/tube_v2_g6_gdb_20260821_171500/gdb.log
/tmp/tube_v2_g6_gdb_20260821_171500/run_identity.txt
/tmp/tube_v2_g6_gdb_20260821_171500/cleanup_external_check.txt
```

G7 结束时所有 product/hash 必须与 T0 一致。若发现差异，不得自行恢复；只报告用户已有变化。

## 3. G7-T1：targeted bag 全量 cohort 和消息身份审计

先用只读 rosbag API 或离线解析，不启动 ROS master。对 G6 targeted bag 建立：

```text
bag_info.txt
topic_manifest.tsv
connection_manifest.tsv
message_manifest.tsv
```

每条相关 message 至少记录：

```text
topic
bag_time
header_stamp if present
frame_id
connection_id
message_index_on_topic
payload_count/point_count
action/ns/id for Marker
raw SHA-256 of serialized payload if feasible
```

必须解析全部已有相关 topics：

```text
/formation_planning/phase_offset_manual/base_path
/formation_planning/phase_offset_manual/active_path
/formation_planning/phase_offset_manual/frame
/formation_planning/phase_offset_manual/tube
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/diagnostics
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
/formation_planning/phase_offset_manual/tube_cloud_snapshot_diagnostics
/sim/local_map
/rosout
/move_base_simple/goal
```

对 3 个 control publication cohort 分别输出 `cohort_<n>.json`，以相同 header stamp、同一 publish
tick、candidate/map sequence 和 diagnostics 的一致性关联 base path、candidate markers、raw/cloud
diagnostics、epoch diagnostics。不能只按“时间最近”合并；若 header stamp 不同，必须列出 join key
和容许误差。

从 GDB 文本中恢复 terminal owner 出现的 wall/bag 时间窗口；在 rosout、raw/cloud diagnostics 中
建立该 terminal validation 对应哪个 build/candidate/map cohort 的证据链。输出：

```text
terminal_cohort_identity.tsv
terminal_cohort_competing_candidates.tsv
```

若 bag 只包含 terminal validation 之后的 collapsed zero-baseline candidate，必须明确写：

```text
DISPLAY_PROFILE_IS_POST_FAILURE_COLLAPSED_BASELINE
```

不得将它冒充 Validator 的 inward input。

## 4. G7-T2：生产变量到 bag 字段的可逆性审计

建立 `production_state_recoverability.tsv`，逐项判断下列 exact 状态能否从 bag 无损恢复：

```text
terminal TubeProfile.samples 的 w
terminal TubeProfile.samples 的 filtered_lower/filtered_upper
terminal raw_build_samples
terminal cell_geometry_certified
terminal current_w
terminal snapshot_resolution
required_clearance
cover_epsilon
cell w0/w1/v0/v1/depth
path owner segment identity/type/w0/w1
path p,p_w,p_ww at terminal w0/wmid/w1
normal N at terminal w0/wmid/w1
cell certificate values if used
sampled_radius
certified/sampled cover branch
cover_radius
requested_radius
nine point xyz
immutable occupied point set and resolution
voxel index anchoring/rounding rule
```

每项必须给出：

```text
RECOVERABLE_EXACT
RECOVERABLE_WITH_EXPLICIT_BOUND
DISPLAY_ONLY_NOT_PRODUCTION_EQUIVALENT
NOT_RECORDED
AMBIGUOUS_COHORT
```

并附 source symbol、bag topic/field、反演公式、数值条件和误差。

必须显式审计 marker 的信息损失：

1. `base_path` 只发布 `full_path_samples[i].p`，不发布 `w/p_w/p_ww`；
2. `tube_candidate` 的 lower/upper line 只发布 sample knot 的
   `p + N*filtered_lower/upper`；
3. ribbon 是显示三角形，不等于 `ContinuousPhasePath::evaluate(w)` 的生产曲面；
4. 若 raw diagnostics 只含 current sample，不能由此恢复远端 `w≈4.03868` 的所有 bounds；
5. 若 lower/upper width 非零且同 knot 的 base point、numeric bounds 可得，可复算 `N`，但必须
   检查 sign、unit norm 和 float64 round-trip；
6. 即使 sample knot 可恢复，也不能未经证明将 path/normal 在 cell 内线性插值代替 semantic owner。

这一节的目的不是预设“不可恢复”，而是防止用 RViz 几何近似伪造 exact 安全结论。

## 5. G7-T3：优先恢复 exact terminal profile/path 几何

按以下优先级执行，前一级 exact 成功才进入下一项计算；失败则记录为何不足并继续检查下一级：

### 5.1 直接保存的 terminal data

搜索 targeted bag、GDB log、G6 `/tmp` 产物是否已保存：

```text
profile.samples/raw_build_samples
path state/certificate
cell points
cover breakdown
clearance requests/results
```

不要仅按文件名判断；检查 payload 内容和 GDB convenience-variable 输出。

### 5.2 同 cohort raw diagnostics + marker 的无损反演

若 numeric `w/lower/upper` 和同 knot `base/lower/upper` 全部存在，恢复：

```text
N = (upper_point - lower_point) / (upper_bound - lower_bound)
p = lower_point - N * lower_bound
```

并核验：

```text
norm(N)≈1
p + N*lower_bound == lower_point
p + N*upper_bound == upper_point
base_path point == p
```

误差必须使用实际 double ULP/serialization 误差，不得随意用厘米级 tolerance。

### 5.3 semantic owner 的可重建性

若 bag 含足以唯一恢复 mapped B-spline/quintic segment 的控制点、时间域、phase mapping、端点导数
和 segment seam，则允许在 `/tmp` 构建独立 replayer，调用当前只读 production math 或逐式等价
实现，在 `w0/wmid/w1` 求 exact `p,p_w,p_ww,N`。必须证明参数集合唯一。

若只有 sampled `p` 或显示 ribbon，不得拟合一条“看起来接近”的 B-spline 后称 exact。拟合只可
作为误差探索，分类必须保持 unresolved。

### 5.4 terminal profile 的 PWL bounds

仅在 terminal inward input 的相邻 exact sample knots/bounds 已恢复时，按生产 `TubeFilter::query`
的 exact cell selection 和 double interpolation规则计算 w0/wmid/w1 的 lower/upper。记录：

```text
w
left/right sample index and w
alpha exact decimal/hex
lower/upper exact decimal/hex
```

不得用 collapsed zero baseline、G5 candidate 8 或另一个 build profile 替代。

输出：

```text
terminal_path_states.tsv
terminal_profile_bounds.tsv
terminal_geometry_reconstruction_proof.md
```

## 6. G7-T4：exact 3x3 point、cover 和 request 重建

只有 T3 已证明 terminal path/profile exact 时执行 exact 重建。使用生产 row-major 顺序：

```text
w_values = [w0, (w0+w1)/2, w1]
v_values = [v0, (v0+v1)/2, v1]
delta = lower(w) + clamp(v,0,1) * (upper(w)-lower(w))
r = p + N * delta
```

输出 `terminal_nine_points.tsv`，每行至少含：

```text
row col index
w decimal/hex
v decimal/hex
lower upper delta decimal/hex
p xyz
N xyz
point xyz
distance_to_center
```

按 production 顺序判定 cover branch：

```text
certificate attempted?
certificate complete/regularity/speed valid?
certified geometric cover and fixed/w/v breakdown
or sampled_radius=max(norm(point-center))
sampled cover=1.1*sampled_radius+0.5*snapshot_resolution for this nondegenerate cell
target_cover=max(required_clearance,snapshot_resolution)
cover_too_large=(cover_radius>target_cover)
requested_radius=required_clearance+cover_radius+min(1e-6,0.01*snapshot_resolution)
```

注意该 terminal cell `w0!=w1`，因此绝不是 G5 exact current-anchor branch；若使用 sampled
fallback，必须保留 `0.5*snapshot_resolution`。不得再删除安全项。

输出：

```text
terminal_cover_reconstruction.tsv
terminal_branch_trace.txt
```

若 `cover_radius > target_cover`，本 terminal cell 是 prequery depth terminal，本层没有执行
9 个 clearance query。此时要写：

```text
TERMINAL_CELL_PREQUERY_DEPTH_OWNER_ZERO_TERMINAL_QUERIES
```

并把安全分类限制为 proof geometry terminal，不能虚构 query failure。仍可对 9 个几何点做
offline closed-distance 测量，但必须标记为 shadow safety evidence，不得称 production query table。

## 7. G7-T5：immutable cloud cohort 和 closed-voxel exact distance

### 7.1 snapshot identity

从 cloud diagnostics、build/map sequence、bag time 和 `/sim/local_map` 建立
`cloud_cohort_identity.tsv`。必须回答：

```text
哪个 /sim/local_map message 构成或等价于本 build 的 immutable snapshot？
是否记录了 snapshot sequence/stamp/resolution/inflation/point_count/domain？
build snapshot 是收到该 message 后立即冻结，还是经过筛选/去重/voxelization？
bag 中 PointCloud2 是否包含 snapshot 使用的全部 occupied support？
```

只有 source code + diagnostics + timestamps 能唯一选中一个 PointCloud2，并证明 snapshot
construction 的 deterministic transform 后，才标为 `EXACT_SNAPSHOT_RECONSTRUCTED`。

若多个 50 Hz map frames 在 join window 内内容完全相同，可计算 canonical occupied voxel set hash；
只有 hash、frame、resolution、transform 均一致才可合并。不能仅因 point_count 同为 900 就合并。

### 7.2 voxel set reproduction

按 production `CloudOccupancySnapshot` 的 exact rounding/index key、duplicate handling、finite
filter、frame contract 和 voxel closed AABB 定义，离线重建 occupied voxel set。输出：

```text
snapshot_manifest.json
occupied_voxels.tsv or compact binary + schema
occupied_voxel_set_sha256.txt
```

不得把 point center Euclidean distance 当 closed voxel distance。每个 voxel 的 closed AABB：

```text
[cx-r/2,cx+r/2] x [cy-r/2,cy+r/2] x [cz-r/2,cz+r/2]
```

以生产实际 center/index 规则为准；若 production 定义不同，以源码为准并写出公式。

### 7.3 exact nearest distance

对每个成功重建的 terminal point，计算到所有 occupied closed AABB union 的 exact nearest
distance，保存最近 voxel index、center、AABB、axis gaps、squared distance 和 distance。至少用两种
独立离线实现或 brute force + indexed implementation 交叉核验。

输出 `terminal_closed_voxel_distance.tsv`：

```text
point_index
point xyz
nearest voxel index/center/AABB
dx dy dz
D_closed
R_planner=0.4
R_request
D_closed-R_planner
D_closed-R_request
classification
```

分类：

```text
D_closed < R_planner + epsilon
  TRUE_PLANNER_RESIDUAL_CLEARANCE_INSUFFICIENT
R_planner + epsilon <= D_closed < R_request
  DISTINCT_CONTINUOUS_SURFACE_COVER_CLEARANCE_INSUFFICIENT
D_closed >= R_request
  REQUIRED_POINT_CLEARANCE_SAFE
```

边界比较必须复用 production epsilon/request 公式并记录 strict/non-strict 运算。若 snapshot 或
point 不 exact，禁止输出上述 definitive 分类，只能输出带误差区间的探索性结果。

## 8. G7-T6：强制结论树

G7 最终必须且只能选择下列主分类之一：

### A. TRUE_CLEARANCE_INSUFFICIENT_AT_STRUCTURALLY_MATCHED_TERMINAL

exact terminal point、exact snapshot 和 closed-volume distance 已恢复，至少一点低于 planner
residual 或 required request。Tube 对该 nonzero surface 的拒绝有真实安全依据；不写实现、不调
margin/depth，下一步仅需最终文档/独立验收范围判断。

### B. PROOF_TOPOLOGY_ONLY_ALL_REQUIRED_TERMINAL_POINTS_SAFE

exact terminal nine points、cover/request 和 exact snapshot 已恢复，所有 required points 均满足
`D_closed >= R_request`，而 actual owner 仍仅为 max-depth proof scheduling/topology。只有这个结论
才允许主代理随后写一个极小 implementation spec；G7 自己不得修改代码。

### C. PREQUERY_MAX_DEPTH_GEOMETRY_TERMINAL_WITH_SHADOW_CLEARANCE_RESULT

证明 terminal cell `cover_radius>target_cover`，本层 production query count 为 0；可附 exact
shadow closed-distance safe/unsafe 结果，但必须把 proof owner 与未执行 query 分开。

### D. EXACT_OFFLINE_RECONSTRUCTION_BLOCKED_BY_MISSING_PRODUCTION_STATE

现有 bag/marker/diagnostics 无法无损恢复 exact path owner、terminal inward profile、certificate、
snapshot identity 或其他必要状态。必须：

1. 列出每个 missing state；
2. 证明为什么现有 marker/diagnostics 不可逆；
3. 列出已经穷尽的所有证据和尝试；
4. 给出下一次 unchanged-binary targeted capture 的最小字段表和断点位置；
5. 不提出 production logging/schema 修改；
6. 不把近似回放称为 safety proof。

### E. QUERY_OR_COHORT_IDENTITY_MISMATCH_PROVEN

证明 G6 terminal cell 与 bag marker/cloud cohort 不是同一 build/snapshot/path/profile，或已有证据
内部矛盾。精确指出 mismatch owner；不做实现。

若 A/B/C 的 exact 前提任一不满足，必须选择 D 或 E。宁可保持 unresolved，也不能用显示近似
给出虚假的安全结论。

## 9. 若选择 D，必须产出下一阶段最小捕获合同

写入本 G7 文档附录，不另改 product。`next_capture_contract.tsv` 至少列出：

```text
capture_site/source line/function
condition identifying final structurally matched validation
current_w
profile pointer and sample count
all sample w/filtered lower/upper needed around terminal cell
raw/candidate/snapshot/source identity
path owner pointer/segment identity/type/w0/w1
path state p/p_w/p_ww at cell w0/wmid/w1
cell w0/w1/v0/v1/depth
certificate attempted/complete and full breakdown
nine point xyz/w/v/delta
sampled_radius/cover_radius/target_cover/requested_radius
prequery vs CellClearancePasses fact
per-point query status/certified/returned clearance
immutable snapshot resolution and occupied voxel set export or exact source message identity
```

要求下一次只读捕获：

- unchanged binary；
- 不改 product source/schema；
- 一个私有 ROS master、fresh ROS_HOME、pillar.pcd、observe_only=false、一个 goal；
- 只用 GDB command/script 或 core dump 将值写到 `/tmp`；
- capture condition 必须避免输出数万个 cell，只捕获 first structurally matched depth terminal 的
  local parent/terminal context；
- 结束清理仅本任务 PID/PGID/port。

G7 不执行该合同。主代理会在 G7 完成后决定是否立即发 G8 read-only capture spec。

## 10. 验收矩阵

必须输出并填完：

| 项目 | 要求 |
|---|---|
| workspace product/source/test/launch hash | 与 G7-T0 完全一致 |
| G6 targeted bag hash | 与 T0 一致 |
| bag topic/message census | PASS |
| three publication cohorts separated | PASS |
| terminal cohort identity | PASS 或精确 AMBIGUOUS |
| production-state recoverability table | PASS |
| collapsed/display profile distinction | PASS |
| exact path/profile reconstruction | PASS 或 D 中精确缺失证明 |
| exact nine points | PASS 或 D 中精确缺失证明 |
| exact cover/prequery branch | PASS 或 D 中精确缺失证明 |
| exact snapshot identity | PASS 或 D 中精确缺失证明 |
| closed-voxel replay two-way cross-check | PASS、NOT_RUN_BECAUSE_EXACT_INPUT_MISSING 或明确 FAIL |
| definitive classification discipline | PASS |
| no ROS/gdb/process launch | PASS |
| `git diff --check` | PASS |
| unauthorized changed files | 0 |

不得为了“填满矩阵”把缺失证据写成 PASS。`NOT_RUN_BECAUSE_EXACT_INPUT_MISSING` 是允许且正确的
fail-closed结果。

## 11. 文档、自审计和 handoff

直接在本文件末尾追加完成区，至少包含：

1. `DOCUMENT_STATUS=COMPLETE`；
2. 主分类 A/B/C/D/E；
3. terminal cohort identity；
4. recoverability table 摘要；
5. exact/approximate/not-recorded 的严格边界；
6. nine-point、cover、snapshot、closed-distance 产物路径；
7. 若 D，missing-state proof 和 `next_capture_contract.tsv`；
8. source/test/launch hash 对比；
9. `git diff --check`；
10. no-ROS/no-gdb/no-new-process 证明；
11. 明确 `NO_PRODUCT_IMPLEMENTATION_PERFORMED`。

更新 `/home/cxq/ISF-GVF/handoff.md` 顶部，保留 G6 历史，写入 G7 完成结果和证据路径。不得在
G7 内自行授权代码修改。

## 12. 连续执行与停止边界

Luna Max 在本规格内部连续工作，不为每个 T0--T6 小步骤请求确认。只有以下情况终止本阶段：

1. A/B/C/D/E 已按 exact 证据选定；
2. 文档、自审计、hash、diff 和 handoff 完成；
3. 未启动任何 ROS/gdb，未修改任何 product 文件。

G7 完成后停止在只读边界。若主分类为 D，等待主代理的新 G8 捕获规格；若为 B，等待主代理的
最小 implementation spec；若为 A/C，等待主代理决定是否只做最终独立 ROS 验收或收尾。

## 13. G7 completion and self-audit

Audit date: 2026-08-21. No product source, header, test, launch, config,
schema, CMake, or G1--G6 document was modified. No ROS, GDB, bag-play, or
debug process was started.

### 13.1 Mandatory classification

```text
D_EXACT_OFFLINE_RECONSTRUCTION_BLOCKED_BY_MISSING_PRODUCTION_STATE
```

The G6 structurally matched terminal identity is recoverable to this point:

```text
candidate_sequence=4
current_w=2.109222365948771
map_observation_sequence=902
snapshot_stamp=1787303242.5718198
PointCloud2_header_seq=930
PointCloud2_point_count=50320
```

The exact GDB owner cell is unchanged from G6:

```text
depth=12
w0=4.0386765291196118  w1=4.0386937522243063
v0=0                 v1=0.000244140625
query_count=22374
geometry_cell_count=2513
clearance_leaf_cell_count=2486
prequery_cover_split_count=27
first_failure_reason=INSUFFICIENT_CLEARANCE
```

The terminal candidate publication is a marker DELETE. The only nonempty
candidate marker cohort is an earlier 12-knot display profile, and its marker
tick competes with a different raw/cloud diagnostic tick. Markers contain
only sampled `p+N*filtered_lower/upper` points (plus a display ribbon), not
`w`, bounds, `N`, derivatives, semantic owner, or certificate data. The raw
diagnostic schema contains only aggregate/current-sample facts; it does not
serialize `raw_build_samples`, per-knot bounds, or Validator cell points.
The GDB text contains cell/counter owner output but no terminal path state,
cover breakdown, requested radius, or nine-point table.

The exact cloud message can be selected by the diagnostic stamp and is saved,
with raw and XYZ hashes, in `snapshot_manifest.json`. However,
`CloudOccupancySnapshotBuildInput.camera_position` is read from mutable SDFMap
state and `/sim/odom` is absent from the targeted bag. Therefore the exact
production occupied voxel set, observed AABB, and closed-volume distances are
not reconstructible. No approximate marker geometry or guessed camera was used
as a safety substitute.

### 13.2 Recoverability and replay matrix

The complete state audit is in
`/tmp/tube_v2_g7_t0_20260821_172710/production_state_recoverability.tsv`.
Exact: current `w`, cell bounds/depth, resolution, cover-epsilon formula,
PointCloud2 message identity, snapshot domain/rounding rule, and diagnostics
sequence/stamp. Display-only: base/tube marker geometry and current-only raw/
cloud arrays. Not recorded: terminal profile vectors, path owner/derivatives,
normal, certificate, cover/request radius, nine points, and per-point query
results. Explicitly bounded but not exact: the occupied set because camera
position is missing.

The exact reconstruction outputs are fail-closed artifacts:

```text
terminal_path_states.tsv
terminal_profile_bounds.tsv
terminal_geometry_reconstruction_proof.md
terminal_nine_points.tsv
terminal_cover_reconstruction.tsv
terminal_branch_trace.txt
terminal_closed_voxel_distance.tsv
```

They are under `/tmp/tube_v2_g7_t0_20260821_172710/` and state
`NOT_RUN_BECAUSE_EXACT_INPUT_MISSING`; no closed-voxel classification is
claimed. The minimum unchanged-binary capture contract is
`next_capture_contract.tsv`, covering Builder line 425, Validator lines 513,
540, and 401, path-owner evaluation, cloud callback/snapshot metadata, and
the exact nine query rows.

### 13.3 Evidence, hashes, and no-change proof

The G6 targeted bag has 1,640 messages across 12 recorded topics plus rosout
connections, and its SHA-256 is recorded in the T0 evidence. Topic, connection,
message, raw-payload, marker, diagnostic, cohort, and map manifests are under:

```text
/tmp/tube_v2_g7_t0_20260821_172710/
```

All 30 required product files in the G7 hash manifest are present and frozen
at T0; `git diff --check` passes. Existing dirty-worktree changes were only
observed and preserved. The no-ROS/no-GDB proof is
`no_ros_gdb_process_proof.txt`; an observational scan saw a pre-existing
user-owned roscore on port 39979, which G7 did not touch. No G7-owned process
was launched.

### 13.4 Final boundary

```text
G7_READ_ONLY_OFFLINE_AUDIT_COMPLETE_D_EXACT_OFFLINE_RECONSTRUCTION_BLOCKED_BY_MISSING_PRODUCTION_STATE
NO_PRODUCT_IMPLEMENTATION_PERFORMED
WAIT_FOR_NEW_G8_READ_ONLY_CAPTURE_SPEC
```

Any next action must be a new bounded unchanged-binary capture specification.
G7 does not authorize production instrumentation, schema changes, parameter
changes, depth/query changes, or safety-margin changes.
