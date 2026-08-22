# Tube V2 G8 unchanged-binary exact terminal/snapshot capture and replay

```text
DOCUMENT_ROLE=READ_ONLY_RUNTIME_CAPTURE_AND_OFFLINE_REPLAY_EXECUTION_SPEC
DOCUMENT_STATUS=COMPLETE
STAGE=TUBE_V2_G8
IMPLEMENTATION_AUTHORIZED=false
SOURCE_OR_TEST_EDITS_AUTHORIZED=false
UNCHANGED_BINARY_PRIVATE_ROS_AUTHORIZED=true
READ_ONLY_GDB_AUTHORIZED=true
PRODUCT_LOGGING_OR_SCHEMA_CHANGE_AUTHORIZED=false
AUTO_ADVANCE=FREEZE_TO_ONE_PRIVATE_RUN_TO_ONE_SHOT_DEPTH_TERMINAL_CAPTURE_TO_EXACT_SNAPSHOT_DUMP_TO_OFFLINE_CLOSED_VOXEL_REPLAY_TO_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、边界、调度和后续代码判断；GPT-5.6 Luna Max 单代理连续完成
unchanged-binary 私有 ROS/GDB 一次性只读捕获、精确 snapshot dump、离线 closed-voxel replay、
分类、自审计和 handoff。

Luna Max 禁止创建、调用、委派、请求或等待任何子代理。不得把源码阅读、GDB、ROS harness、
bag、snapshot dump、距离计算、测试或文档交给其他模型。本阶段在规格内连续执行，不为小步骤
等待确认。

## 0. G7 冻结结论与 G8 唯一目标

G7 已完成：

```text
D_EXACT_OFFLINE_RECONSTRUCTION_BLOCKED_BY_MISSING_PRODUCTION_STATE
```

G7 从既有 G6 bag 恢复了结构匹配 terminal cohort：

```text
candidate_sequence=4
map_observation_sequence=902
snapshot_stamp=1787303242.5718198
PointCloud2_header_sequence=930
PointCloud2_point_count=50320
current_w=2.109222365948771
depth=12
w=[4.0386765291196118,4.0386937522243063]
v=[0,0.000244140625]
query_count=22374
prequery_cover_split_count=27
owner=ValidateCell max-depth guard
```

但 terminal candidate marker 是 DELETE，bag 没有保存 terminal inward profile/path、9 点、
cover/request、certificate branch；同时没有 `/sim/odom`，无法从 PointCloud2 单独重建
`CloudOccupancySnapshot` 的 observed box/occupied set。因此 G7 正确地没有做近似安全结论。

G8 唯一目标：

> 在一次新的、独立的、unchanged-binary pillar 场景中，以条件断点捕获第一个结构匹配的
> `depth==12`、w/v 均仍可切分的 Validator terminal；在同一断点栈中直接保存 production
> `SurfacePointArray`、cover/target/request、profile 和 Builder identity，并从同一
> `TubeBuildRequest::cloud_snapshot` 直接保存 immutable snapshot metadata 与 authoritative
> dense `occupied` vector。随后离线按 production closed-voxel 语义精确重放 9 个请求并判定：
> 真实安全/观测域不足，还是仅 proof scheduling/topology 在深度 12 拒绝了实际可证安全 cell。

G8 不改任何产品实现，不增加 observability，不调 depth/query/clearance/cover，不加 gate/cache/
retry/timer/parameter/module/第二 Validator。

## 1. 永久冻结边界

不得修改任何 product source、header、test、launch、config、parameter、CMake、package、message、
schema 或 G1--G7 execution/self-audit 文档。允许写入的仅有：

```text
docs/Codex_Tube_V2_G8_Unchanged_Binary_Exact_Terminal_Snapshot_Capture_And_Replay_Spec_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g8_*
```

允许在 `/tmp/tube_v2_g8_*` 创建：

- fresh `ROS_HOME`、PID/PGID/port manifests；
- GDB command/Python scripts 与 log；
- one-shot terminal TSV/JSON/binary dump；
- targeted rosbag 和只读解码；
- immutable occupied vector binary；
- `/tmp` 独立离线 replay source/binary；
- test logs、hashes、diff、cleanup proof。

禁止：

1. 修改/重编译 workspace product 后称 unchanged binary；
2. 临时向 product 加 log、topic、field、friend、getter、debug API 或 schema；
3. 调高 `max_subdivision_depth=12` 或 `max_query_samples=250000`；
4. 降低 `planner_safe_distance=0.4`、epsilon、cover、regularity 或 map inflation；
5. 删除任何 nondegenerate half-voxel、certified cover 或 `1.1` sampled factor；
6. 修改 inward family/order/scale/attempt count/capacity tolerance；
7. 添加任何产品 gate、cache、retry、timer、sleep、parameter、module 或第二 Validator；
8. 使用 observe-only、zero-only、改 map、改 goal、减速、延迟逻辑制造通过；
9. attach 或终止用户已有 ROS/GDB/process；
10. reset、restore、checkout、clean、stash、rebase 或覆盖用户 dirty worktree；
11. 将新 run 数值冒充 G5 candidate8/map919；本阶段只声称结构匹配新 run；
12. 若捕获不完整，使用 marker 插值、猜测 camera 或 request-capped clearance 补洞。

## 2. G8-T0：规则、binary、证据和运行前冻结

完整读取：

```text
AGENTS.md
/home/cxq/ISF-GVF/handoff.md 顶部 G7 段
docs/Codex_Tube_V2_G7_Structurally_Matched_Terminal_Exact_Offline_Clearance_Audit_2026-08-21.md
/tmp/tube_v2_g7_t0_20260821_172710/next_capture_contract.tsv
docs/Codex_Tube_V2_G6_Inward_Search_Limit_Flag_Read_Only_Audit_2026-08-21.md
本 G8 spec
```

建立：

```text
/tmp/tube_v2_g8_capture_20260821_<id>/
```

保存：

```text
HEAD
git status --short
git diff --stat
git diff --check
ulimit
date/timezone
ROS_DISTRO/ROS_PACKAGE_PATH
existing ROS masters/processes/listeners
```

必须识别并避开用户已有 roscore（G7 曾观察到 port 39979/PID 1071735；以本次实际扫描为准）。
不得向用户 master 查询节点、发布 topic、attach gdb 或发送 signal。

冻结 G7 要求的 30 个 product source/test/launch hash，并额外冻结实际执行 binary 及依赖：

```text
devel/lib/bspline_race/formation_planning
devel/lib/libphase_offset_navigation.so if present
devel/lib/libphase_offset_core.so if present
devel/lib/libplan_env.so or linked equivalent if present
ldd formation_planning
readelf -n/-Ws summary
```

记录 binary build-id、SHA-256、mtime。G8 禁止执行 build。若 binary hash 在运行前后变化，结果
无效，保留证据并停止，不得自行 rebuild/restore。

先完整阅读 G6 的 successful private-run harness/log/cleanup 方法；只复用其私有进程管理，不复用
G6 缺字段的 GDB command。

## 3. G8-T1：先写并静态审计 one-shot GDB capture

在启动 ROS 前先写完 `/tmp` GDB command/Python script，并进行 `gdb -batch` symbol preflight，
不得启动节点。脚本必须是 one-shot、低输出、条件捕获，不能在每个 cell 打印。

### 3.1 terminal match predicate

在 `tube_surface_validator.cpp:513` 的 depth guard 设置条件断点。目标是该 run 第一个满足：

```text
cell.depth == context.config->max_subdivision_depth == 12
cell.w1 - cell.w0 > 1e-10
cell.v1 - cell.v0 > 1e-10
cell.w0 != context.current_w || cell.w1 != context.current_w
context.result->query_sample_count + 9 <= context.config->max_query_samples
```

这定义 `STRUCTURALLY_MATCHED_NEW_RUN`，不要求复现 G6 exact doubles。命中后脚本把全局
`terminal_captured` 标为 true；同一 run 后续 depth terminal 只 continue，不重复 dump。

断点必须确认当前 source line 确实是 `if (cell.depth >= ...)`；不能盲信旧行号。

### 3.2 line-513 当前 frame 必捕获值

命中时直接从 `ValidateCell` frame 写出 exact decimal 和 hexadecimal double：

```text
context.current_w
context.snapshot_resolution
context.required_clearance
context.regularity_margin
context.cover_epsilon
context.config max depth/query
cell w0/w1/v0/v1/depth and spans
geometry_valid
clearance_safe
cover_radius
target_cover
cover_radius <= target_cover
breakdown.fixed_cover
breakdown.w_reducible_cover
breakdown.v_reducible_cover
breakdown.decomposable
result counters/reason/w/limit before guard
```

直接遍历 `points[0..8]`，每点保存：

```text
index/row/column
point.w
point.point.x/y/z
distance to points[4]
```

计算并保存：

```text
requested_radius = context.required_clearance + cover_radius + context.cover_epsilon
terminal_prequery = geometry_valid && cover_radius > target_cover
terminal_query_branch = geometry_valid && cover_radius <= target_cover
```

这些值来自 production stack local，不得由 marker 重建。

### 3.3 profile/Builder identity 捕获

在同一 breakpoint stack 中查找 `CertifiedTubeBuilder::build` frame，捕获：

```text
inward_attempt_count
family/family_index
level
scale decimal/hex
input.current_w
input.source_revision/tube/map/snapshot identity fields available
input.cloud_snapshot_resolution
inward profile pointer
inward.complete/filtered_complete/cell_geometry_certified/source/classification
inward.samples.size
all inward.samples: w, raw lower/upper, filtered lower/upper, p xyz, N xyz
raw_build_samples equivalent fields when retained
inward_validation counters if in scope
```

如果 optimized binary 中某个变量 unavailable，逐字段标记 `OPTIMIZED_OUT`，但 line-513 的
points/cover/context 和同栈 snapshot dump 是安全分类的硬要求。

不得在 Builder line 425 对 101 attempts 全量停止。只有若 stack frame 无法访问 family/scale，
才允许在 line 425 设置静默 bookkeeping breakpoint，只保存当前 attempt identity 到 GDB
convenience variables，不输出 profile vectors。

### 3.4 从同一调用栈捕获 immutable snapshot

在 line-513 breakpoint 向上遍历 frame，定位
`PhaseOffsetMatchedAdapter::buildTubeEpoch` 或仍持有同一 `request` 的等价 frame。读取：

```text
request pointer
request source_revision/current_path.w/map_observation_sequence
request cloud_snapshot shared_ptr raw pointer
snapshot valid
observation_sequence
observation_stamp sec/nsec
map_min/map_max
observed_min/observed_max
grid_origin
voxel_count
resolution
included_map_inflation
occupied.size/capacity/data pointer
```

从该 snapshot 的 authoritative `std::vector<uint8_t> occupied` data pointer 到 size 精确执行
`dump binary memory`，生成：

```text
terminal_snapshot_occupied.bin
```

脚本必须保存 start/end pointer 和 byte count；dump 后在 shell 计算 SHA-256。`occupied.size`
必须等于 `voxel_count.x*y*z`。不要尝试读取 private column index；dense vector 是权威。

若 `buildTubeEpoch` frame 访问不到 `request`，允许在函数内 snapshot 已形成但 `manager->update`
调用前设置静默 breakpoint，以 `current_w/map_observation_sequence/snapshot pointer` 建立最多几十条
小型 pointer registry；line-513 terminal 再通过 `context.current_w` 和当前线程调用栈选择同一
request。不得把 registry 变成产品 cache，也不得输出每帧 occupied vector。

### 3.5 可选 per-query capture

若 `terminal_query_branch=true`，line 401 的 production query 已在 depth guard 前执行。为了避免
错过，不强制依赖该断点；exact snapshot + points + request 足以离线重放同一纯函数。

可以设置低成本条件 breakpoint 记录当前 terminal cohort 的 9 个 per-query result，前提是条件
能在查询前识别同一 cell，且不会打印其他数万个 query。做不到则跳过，不影响 exact offline
replay 验收。

### 3.6 capture 结束行为

完成 terminal + snapshot dump 后：

1. 写 `CAPTURE_COMPLETE` sentinel；
2. 让被调试进程继续，而不是在断点挂死；
3. harness 看到 sentinel 后结束本任务 run；
4. 只向本任务 PID/PGID 发正常退出，必要时再升级信号；
5. 不触碰用户进程。

GDB preflight 必须证明：symbol/function/source line 可解析，脚本语法有效，输出目录可写，且没有
`set variable` 修改 inferior production state。允许设置 GDB convenience variables 和 Python
host-side state；禁止改被调试进程内存。

## 4. G8-T2：一次独立 unchanged-binary ROS/GDB 运行

### 4.1 私有运行合同

只运行一次：

```text
fresh private roscore port, confirmed unused
fresh ROS_HOME under G8 evidence dir
unchanged absolute phase_offset_esdf_tube_single.launch
phase_offset_manual_observe_only:=false
unchanged pillar.pcd
unchanged parameters
formation_planning launched under gdb with unchanged binary
targeted rosbag starts before goal
actual /sim/local_map message count >=900 before goal
exactly one external goal=(8,0,1), internal=(8,0,2)
```

禁止连接用户 ROS master。所有 ROS 命令显式带本任务 `ROS_MASTER_URI`/`ROS_HOME`。

targeted bag 至少记录：

```text
/sim/local_map
/sim/odom
/move_base_simple/goal
/formation_planning/phase_offset_manual/base_path
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
/formation_planning/phase_offset_manual/tube_cloud_snapshot_diagnostics
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
/formation_planning/phase_offset_manual/diagnostics
/rosout
```

bag 是身份交叉证据；精确 terminal geometry 和 occupied vector 以 GDB dump 为权威。

### 4.2 运行中的成功条件

成功只定义为：

```text
one goal published
one structurally matched line-513 terminal captured
terminal_points/context/cover files complete
same-stack snapshot metadata complete
occupied binary byte count exact
CAPTURE_COMPLETE sentinel written
```

不要求 goal reached、Pair committed 或 nonzero offset accepted；本阶段是诊断捕获，不是动态功能
验收。不得因为 goal 已 reached 而在 sentinel 前停止。

若当前场景没有命中结构匹配 terminal，保留完整证据并分类 `G8_TARGET_NOT_REPRODUCED`；本规格
不授权改 map/goal/parameters 或自动跑第二次。若只是 GDB 脚本语法/optimized variable导致捕获不
完整，保留错误并精确列出，不得通过修改 product 解决。

### 4.3 清理

保存所有任务 PID/PGID/port。结束时只清理本任务启动的：

```text
gdb/inferior
roslaunch/roscore
rosbag
harness/watcher
task-owned child processes
private port listener
```

独立于 harness 再执行 `/proc` + `ss -ltnp` 外部扫描，生成
`cleanup_external_check.txt`。不得把 harness 自己的命令文本误判为残留进程。用户已有 port/PID
必须在前后均存在或至少没有被本任务发送 signal。

## 5. G8-T3：capture 完整性和身份核对

在离线 replay 前先验收：

```text
terminal context current_w == Builder/input current_w
snapshot observation_sequence == request map_observation_sequence
snapshot resolution == context.snapshot_resolution
occupied.size == voxel_count product == binary byte count
snapshot consistent metadata
all 9 points finite
cover/request finite and request >= required_clearance
cell depth == 12
w_span/v_span > 1e-10
query count remains below budget owner
```

与 bag raw/cloud diagnostics 按 observation sequence/stamp/current_w 建立 identity chain；若 bag
因 shutdown 没收到最后 diagnostic，不否定同-stack GDB pointer identity，但要如实记录。

输出：

```text
terminal_identity.tsv
terminal_context.tsv
terminal_points.tsv
terminal_profile.tsv
terminal_snapshot_metadata.tsv
occupied_dump_manifest.json
identity_chain.tsv
```

任一硬条件不满足，不运行 definitive replay，分类为 `G8_CAPTURE_INCOMPLETE`。

## 6. G8-T4：exact offline production-equivalent replay

只在 T3 完整时，在 `/tmp` 编写独立 replay。优先直接链接/调用当前 unchanged
`plan_env::queryCloudOccupancySnapshotClearance`；如果私有 vector 装载困难，允许逐式等价实现，
但必须再用 brute-force 全 occupied voxel closed AABB 扫描交叉验证。

重建 snapshot 使用 GDB 捕获的 exact：

```text
valid/sequence/stamp
map_min/map_max
observed_min/observed_max
grid_origin
voxel_count
resolution
included_map_inflation
occupied bytes
```

对 9 点使用同一个：

```text
R_planner = context.required_clearance
R_cover = cover_radius
epsilon = context.cover_epsilon
R_request = R_planner + R_cover + epsilon
```

每点输出：

```text
index/row/column
point xyz/w
production-equivalent status
clearance_certified
request-capped returned clearance
exact nearest occupied closed-voxel distance from brute force
nearest voxel index/address/lower/upper
axis gaps and squared distance
closed request ball inside observed box?
closed request ball inside grid box?
D_closed-R_planner
D_closed-R_request
```

必须区分：

```text
KNOWN_FREE and D_closed >= R_request
OCCUPIED / D_closed == 0
KNOWN_FREE but D_closed < R_request
UNKNOWN because request ball exceeds observed box
OUT_OF_MAP because point/request ball exceeds grid/map
UNAVAILABLE metadata/arithmetic failure
```

production query 在无 occupied voxel 进入 request window 时返回 capped `R_request`；brute-force
必须另算全 occupied set 的 exact nearest distance，不能把 cap 冒充 exact distance。

两种 replay 的 status/certification/capped distance 必须逐点一致；brute-force exact nearest 的
double 误差记录 ULP/absolute tolerance。

## 7. G8-T5：terminal branch 与安全分类

先确定 production terminal branch：

```text
PREQUERY_DEPTH_TERMINAL:
  geometry_valid && cover_radius > target_cover

POSTQUERY_DEPTH_TERMINAL:
  geometry_valid && cover_radius <= target_cover && clearance_safe == false

GEOMETRY_DEPTH_TERMINAL:
  geometry_valid == false
```

若 geometry invalid，不得用 9 点 clearance 覆盖 geometry/regularity failure；单独分类。

在 exact metadata/points/snapshot/replay 完整的前提下，最终主分类必须选一个：

### A. TRUE_PLANNER_RESIDUAL_CLEARANCE_INSUFFICIENT

至少一点 exact `D_closed < R_planner` 或 occupied。真实 planner residual 不满足；Tube 拒绝正确，
不改 margin/depth。

### B. CONTINUOUS_COVER_REQUEST_CLEARANCE_INSUFFICIENT

所有点 `D_closed >= R_planner`，但至少一点 `D_closed < R_request`。centreline residual可能安全，
但为覆盖连续 cell 所需的 request 不满足；不得删除 cover。

### C. OBSERVED_DOMAIN_OR_MAP_SUPPORT_INSUFFICIENT

至少一点 query 为 UNKNOWN/OUT_OF_MAP/UNAVAILABLE，或完整 request ball 不在 observed/grid/map box。
这是 fail-closed 观测支持不足，不等于 occupied obstacle，也不授权弱化安全。

### D. ALL_NINE_REQUESTS_EXACTLY_SAFE_MAX_DEPTH_PROOF_ONLY

必须同时满足：

```text
geometry_valid=true
all 9 production-equivalent statuses KNOWN_FREE
all 9 clearance_certified=true
all 9 exact D_closed >= R_request
snapshot/query identity exact
owner only max-depth with both spans splittable
```

这证明该 terminal cell 已可由现有 3x3+cover 事实安全接受，失败来自 proof scheduling/topology/
prequery policy与 max-depth交互。只有 D 才允许主代理随后写极小 implementation spec；G8 自己不
改代码。

### E. GEOMETRY_OR_REGULARITY_DEPTH_TERMINAL

`geometry_valid=false`；记录 first invalid path/regularity fact。不得归咎 clearance。

### F. QUERY_OR_SNAPSHOT_IDENTITY_MISMATCH_PROVEN

同 stack request/context/snapshot、bag diagnostics 或 replay 产生不可解释矛盾。列出 exact mismatch。

### G. G8_TARGET_NOT_REPRODUCED_OR_CAPTURE_INCOMPLETE

没有命中结构 terminal，或 hard capture字段/occupied dump不完整。列出已完成项与精确缺失；不得
近似分类，不改产品重跑。

主分类选择最早令该 cell不能安全接受的事实。branch 类型作为独立 secondary，不能把
`PREQUERY` 自动等同于 unsafe，也不能把 `depth` 自动等同于 proof-only。

## 8. G8-T6：只读验证与不变性

G8 不 build、不改代码。capture/replay 后运行已有已构建 focused tests：

```text
tube_surface_validator_test
certified_tube_builder_test
cloud_occupancy_snapshot_test
phase_offset_cloud_occupancy_query_test
```

记录各自 case count 和 PASS/FAIL。再运行 `git diff --check`。

对 T0 hash 进行 final comparison：所有 product source/test/launch 和 formation_planning binary 必须
一致。允许差异仅为 G8 spec、handoff 和 `/tmp` artifacts。用户已有其他 dirty 内容不得修改、
归因或清理。

## 9. 验收矩阵

| 项目 | 要求 |
|---|---|
| unchanged binary hash/build-id before/after | MATCH |
| product source/test/launch hashes | MATCH |
| private master/ROS_HOME/port | PASS |
| user ROS untouched | PASS |
| unchanged launch/map/params/observe_only=false | PASS |
| one goal only | PASS |
| local map >=900 before goal | PASS |
| structurally matched depth-12 terminal | PASS 或 G |
| line-513 exact 9 points | PASS 或 G |
| exact cover/target/request/branch | PASS 或 G |
| same-stack snapshot identity | PASS 或 G |
| occupied dense vector exact dump | PASS 或 G |
| production-equivalent replay vs brute force | PASS 或 NOT_RUN_FOR_G |
| definitive A--F only with exact inputs | PASS |
| task-owned cleanup | PASS |
| focused tests | PASS |
| `git diff --check` | PASS |
| unauthorized product changes | 0 |

不得把 optimized-out、missing、近似或 marker-derived 填成 PASS。

## 10. 文档、自审计和 handoff

直接在本文件末尾追加完成区，至少包含：

1. `DOCUMENT_STATUS=COMPLETE`；
2. 主分类 A--G 和 branch secondary；
3. run identity/port/goal/map count；
4. exact terminal cell/context/9点/cover/request 摘要；
5. snapshot identity、occupied size/hash；
6. replay逐点结果和最小 margin；
7. 是否存在真实 residual/cover/domain violation；
8. 是否满足 D 的所有条件；
9. focused tests、hash、diff；
10. task-owned cleanup和用户 ROS 未触碰证明；
11. `NO_PRODUCT_IMPLEMENTATION_PERFORMED`；
12. 下一阶段建议只能是：代码已正确拒绝并收尾，或 D 后等待主代理最小 implementation spec，
    或 G 后精确报告捕获问题。

更新 `/home/cxq/ISF-GVF/handoff.md` 顶部，保留 G7/G6 历史。G8 不自行授权代码修改。

## 11. 连续执行和停止边界

Luna Max 在 T0--T6 内连续执行，不为中间步骤请求确认。一次 run 完成后必须清理并完成离线
replay/文档；不得因已经得到一个初步数值而省略 identity/hash/cleanup。

G8 只在以下状态之一结束：

```text
A/B/C/E/F: exact failure owner classified, no code change
D: exact proof-only false negative established, wait for main-agent implementation spec
G: one authorized run exhausted but target/capture incomplete, exact report only
```

无论哪种结果，G8 不修改 product、不调参数、不启动第二次场景、不增加门控。

## G8 completion — 2026-08-21

```text
DOCUMENT_STATUS=COMPLETE
IMPLEMENTATION_AUTHORIZED=false
PRIMARY_CLASSIFICATION=G8_TARGET_NOT_REPRODUCED_OR_CAPTURE_INCOMPLETE
CAPTURE_SUBCLASSIFICATION=G8_CAPTURE_INCOMPLETE
SECONDARY_CLASSIFICATION=STRUCTURALLY_MATCHED_NEW_RUN
TERMINAL_BRANCH=UNRESOLVED_BECAUSE_COVER_UNAVAILABLE
NO_PRODUCT_IMPLEMENTATION_PERFORMED
```

The one authorized private unchanged-binary run is recorded under:

```text
/tmp/tube_v2_g8_capture_20260821_181934/
```

Run identity and boundary checks:

```text
private_port=13017
launch=phase_offset_esdf_tube_single.launch
map=pillar.pcd
phase_offset_manual_observe_only=false
goal_count=1
goal=(8,0,1)
internal_goal=(8,0,2)
pre_goal_local_map_count=900
```

The first structurally matched line-510 Validator terminal captured directly
from the production stack was:

```text
context.current_w=0.11256160363570508
depth=12
w0=0.050000000000000003
w1=0.050015273829012628
v0=0
v1=0.000244140625
w_span=1.5273829012625584e-05
v_span=0.000244140625
query_count_before_guard=225
geometry_cell_count=36
clearance_leaf_cell_count=25
prequery_cover_split_count=11
max_depth_observed=12
first_failure_reason=INSUFFICIENT_CLEARANCE
```

The Builder input identity was also captured:

```text
input.current_w=0.11256160363570508
input.path_source_revision=1
input.tube_revision=1
input.map_observation_sequence=1107
input.cloud_snapshot_resolution=0.10000000000000001
input.map_observation_is_snapshot=1
```

The bag-side cloud diagnostic agrees on observation sequence `1107`, stamp
`1787308478.379691`, and resolution `0.1`; this is an identity cross-check,
not a substitute for the same-stack snapshot dump.

Capture completeness failed at the required exact-evidence boundary.  At the
optimized line-510 frame, `cover_radius` and `target_cover` were optimized out;
the GDB Eigen accessors returned `UNAVAILABLE` for all nine point xyz values
and profile `p/N` vectors; Builder family/level/scale were unavailable; and the
same-stack request/snapshot pointers were found but their fields and dense
`occupied` vector could not be dereferenced.  Therefore
`terminal_snapshot_occupied.bin` and `CAPTURE_COMPLETE` were not produced.
The complete field-by-field audit is
`/tmp/tube_v2_g8_capture_20260821_181934/capture_completeness.tsv`.
The pointer/evaluator failure details are recorded in
`/tmp/tube_v2_g8_capture_20260821_181934/capture_failure_root_cause.txt`.

No exact production-equivalent clearance replay was run.  In particular, no
cover/request radius, exact point geometry, observed/grid bounds, or occupied
vector was guessed or reconstructed from markers/diagnostics.  Consequently
none of classifications A–F is claimed; the result is precisely
`G8_CAPTURE_INCOMPLETE`.

Focused existing tests passed without a build:

```text
tube_surface_validator_test                 22/22 PASS
certified_tube_builder_test                 11/11 PASS
cloud_occupancy_snapshot_test               14/14 PASS
phase_offset_cloud_occupancy_query_test      4/4 PASS
```

The 30 frozen product source/test/launch hashes and the formation-planning,
navigation, core, and plan-env binary SHA-256/build IDs match T0.  Launch/map
hashes match.  `git diff --check` passes.  The independent cleanup proof shows
all task PIDs dead and private port `13017` not listening; reference user-owned
processes remained present.  No user ROS master/process was attached to,
signalled, or otherwise touched.

Next action is only to report the exact capture failure and stop.  G8 does not
authorize a second run, a production observability change, or an
implementation; any continuation requires a new bounded specification.
