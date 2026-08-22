# Tube V2 G9 unchanged-binary callee-boundary exact capture and replay

```text
DOCUMENT_ROLE=READ_ONLY_RUNTIME_CAPTURE_AND_OFFLINE_REPLAY_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_G9
IMPLEMENTATION_AUTHORIZED=false
SOURCE_OR_TEST_EDITS_AUTHORIZED=false
UNCHANGED_BINARY_PRIVATE_ROS_AUTHORIZED=true
READ_ONLY_GDB_PYTHON_AUTHORIZED=true
PRODUCT_LOGGING_OR_SCHEMA_CHANGE_AUTHORIZED=false
AUTO_ADVANCE=FREEZE_TO_CALLEE_BOUNDARY_PREFLIGHT_TO_ONE_PRIVATE_RUN_TO_EXACT_OUTPUT_REFERENCE_CAPTURE_TO_DIRECT_SNAPSHOT_REFERENCE_DUMP_TO_REPLAY_TO_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、调度、边界和后续代码判断；GPT-5.6 Luna Max 单代理连续完成
callee-boundary GDB Python 只读捕获、一次 private unchanged-binary ROS、exact snapshot dump、
closed-voxel replay、分类、自审计和 handoff。

禁止 Luna Max 创建、调用、委派、请求或等待任何子代理，不得并行代理工作。本阶段不在小步骤
等待确认。

## 0. G8 失败根因与 G9 的窄修正

G8 完成了一个结构匹配新 run，但安全证据不完整：

```text
PRIMARY=G8_TARGET_NOT_REPRODUCED_OR_CAPTURE_INCOMPLETE
SECONDARY=STRUCTURALLY_MATCHED_NEW_RUN
current_w=0.11256160363570508
depth=12
w=[0.050000000000000003,0.050015273829012628]
v=[0,0.000244140625]
query_count=225
map_observation_sequence=1107
```

G8 在 `ValidateCell` depth guard 之后读取局部值时，optimized binary 已令以下值失活或不可由
GDB expression evaluator展开：

```text
points xyz
cover_radius/target_cover/requested_radius
profile p/N
shared_ptr request/snapshot fields
occupied vector
```

G9 不重复这种读取方式。唯一观测修正是把断点移到值的真实 ABI/内存所有者边界：

1. 在 `CollectCellPoints` 入口记录 `points`、`cover_radius`、`breakdown` 输出引用的地址；
2. 用 GDB Python `FinishBreakpoint` 在该函数返回时从这些地址直接读取值；
3. 在 `queryCloudOccupancySnapshotClearance` 入口直接记录其
   `const CloudOccupancySnapshot& snapshot` 参数地址，而不是反解 shared_ptr；
4. 在 `ValidateCell` depth guard 命中失败 cell 时，将刚完成且 cell identity 相同的 candidate
   output 与当前 direct snapshot address配对，直接 dump dense `occupied` vector；
5. 离线精确 replay。

这只是 GDB host-side 观测方法修正，不修改 production memory、code、schema、build flags 或行为。

## 1. 永久冻结边界

不得修改任何 product source/header/test/launch/config/parameter/CMake/package/message/schema 或
G1--G8 文档。只允许写：

```text
docs/Codex_Tube_V2_G9_Unchanged_Binary_Callee_Boundary_Exact_Capture_And_Replay_Spec_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g9_*
```

允许 `/tmp` GDB Python/command、private ROS_HOME、targeted bag、binary dumps、offline replay、
logs/hashes/tests/cleanup artifacts。

禁止：

1. 修改或重编译 product；
2. 修改 compiler flags、生成 debug variant 后称 production evidence；
3. 使用 LD_PRELOAD/interposition/ptrace writer/hot patch；
4. `set variable`、call inferior function、写 inferior memory；
5. 加 product log/topic/getter/friend/schema；
6. 调 depth/query/clearance/cover/epsilon/regularity/map inflation；
7. 加 gate/cache/retry/timer/sleep/parameter/module/第二 Validator；
8. 改 inward schedule、map、goal、observe_only 或场景；
9. attach/kill用户进程或连接用户 ROS master；
10. reset/restore/checkout/clean/stash/rebase；
11. 在 exact capture 不完整时用 marker/odom timing/猜测补值；
12. G9 内做任何 product implementation。

## 2. G9-T0：冻结与前置核对

完整读取：

```text
AGENTS.md
/home/cxq/ISF-GVF/handoff.md 顶部 G8 段
docs/Codex_Tube_V2_G8_Unchanged_Binary_Exact_Terminal_Snapshot_Capture_And_Replay_Spec_2026-08-21.md 完成区
/tmp/tube_v2_g8_capture_20260821_181934/capture_failure_root_cause.txt
/tmp/tube_v2_g8_capture_20260821_181934/capture_completeness.tsv
/tmp/tube_v2_g8_capture_20260821_181934/gdb_capture.py
/tmp/tube_v2_g8_capture_20260821_181934/gdb_capture.cmd
本 G9 spec
```

建立 `/tmp/tube_v2_g9_capture_20260821_<id>/`，保存 HEAD/status/diff、时区、现有 ROS/process/
listeners、G8 binary build-id/hash、当前 binary/build-id/hash、30 个 product hashes、launch/map hash。

必须确认执行 binary 与 G8/G7 冻结一致；禁止 build。避开用户已有 master/port/PID。

## 3. G9-T1：GDB Python callee-boundary 静态设计

启动 ROS 前完成脚本和 symbol/disassembly preflight。

### 3.1 `CollectCellPoints` entry breakpoint

在真实 symbol（含 anonymous namespace mangling）上设置 Python breakpoint。每次 entry 只在：

```text
cell.depth == 12
cell.w1-cell.w0 > 1e-10
cell.v1-cell.v0 > 1e-10
not exact current anchor
```

时创建一个 candidate record，内容：

```text
thread/inferior/frame identity
context address
cell exact w0/w1/v0/v1/depth
points output object address
cover_radius output double address
breakdown output object address
```

不得依赖 caller 中同名 local 的 DWARF location。

用 `gdb.FinishBreakpoint` 或等价 return breakpoint。return 时：

1. 读取函数 bool return (`geometry_valid`)；
2. 从保存的 output address 直接读取 9 个 `SurfacePoint`；
3. 从保存地址读取 cover double；
4. 从保存地址读取 breakdown 4 fields；
5. 将完整 candidate 保存在 GDB Python host memory 和 `/tmp` candidate 文件；
6. 不写 inferior。

### 3.2 raw-memory fallback for Eigen/STL pretty-printer failure

若 `points[i].point.x()` 等表达式不可用，必须使用 DWARF type layout + `inferior.read_memory`：

```text
sizeof(SurfacePoint)
offsetof(SurfacePoint, point)
offsetof(SurfacePoint, w)
Eigen::Vector3d storage = 3 contiguous doubles
```

在 preflight 记录 `ptype/o`、field bitpos、sizeof/alignment。对每个点按 byte offset读 3 个 IEEE
754 doubles和 w。禁止假设 stride=32 而不由 type layout验证。

同样对 `CellCoverBreakdown` 用 field bitpos读取：

```text
fixed_cover
w_reducible_cover
v_reducible_cover
decomposable
```

对 double 同时输出 decimal、`float.hex()` 和 raw 8-byte hex。

### 3.3 direct snapshot reference breakpoint

在 `plan_env::queryCloudOccupancySnapshotClearance(const CloudOccupancySnapshot&, const
Eigen::Vector3d&, double)` function entry 设置静默 Python breakpoint。直接读取 ABI function arg：

```text
&snapshot
&point
required_radius
thread id
```

对每个线程只保存 latest direct snapshot object address、latest query point/radius和时间序号；不
打印每个 query，不 dump vector。

如果 debug name `snapshot` optimized out，使用 function ABI/disassembly + architecture calling
convention读取 reference argument register，仅限读取；必须在 preflight 证明寄存器到 object pointer
映射。不得猜寄存器。

### 3.4 `ValidateCell` depth-owner breakpoint

在 `if (cell.depth >= max_subdivision_depth)` 的实际 source/instruction位置设置 breakpoint，只在：

```text
depth==12
both spans >1e-10
non-current-anchor
query_count+9 <= max_query_samples
```

时处理。命中时：

1. 从 host candidate ring选择同 thread、同 context address、cell四个 exact doubles全相等的最近
   `CollectCellPoints` completed record；
2. 若无 exact match，不得取“最近 cell”；记录 mismatch并继续寻找下一个 terminal；
3. 读取仍可用 context/result scalars；
4. 计算 `target_cover=max(required_clearance,snapshot_resolution)`；
5. 计算 `requested_radius=required_clearance+captured_cover+cover_epsilon`；
6. 判定 PREQUERY/POSTQUERY/GEOMETRY branch；
7. 选择同 thread latest direct snapshot address；
8. 验证该 snapshot resolution 与 context一致，sequence 与 Builder/bag identity随后交叉。

本 run 只正式接受第一个 terminal + candidate + snapshot 三者完整配对的 structurally matched cell。
前面不完整的 hit 可记录但不能写 sentinel。

### 3.5 direct snapshot raw dump

从 typed direct `CloudOccupancySnapshot*` 读取：

```text
valid
observation_sequence
observation_stamp sec/nsec
map_min/map_max
observed_min/observed_max
grid_origin
voxel_count
resolution
included_map_inflation
occupied vector start/finish/end_of_storage
```

如果 STL field expression失败，使用 DWARF layout读取 `std::vector<uint8_t>` 的三个 pointer；必须
验证：

```text
finish-start == voxel_count.x*voxel_count.y*voxel_count.z
end_of_storage >= finish
byte count > 0 and bounded by expected map size
```

用 `inferior.read_memory(start,size)` 写 host-side
`terminal_snapshot_occupied.bin`；不是 `dump binary memory` expression。计算 SHA-256。

完成 exact terminal TSV + snapshot metadata + occupied binary 后写 `CAPTURE_COMPLETE` sentinel并
continue inferior。

### 3.6 preflight acceptance

必须在不启动 ROS node的 `gdb -batch`/symbol preflight中证明：

```text
CollectCellPoints symbol resolved and not wholly unavailable
FinishBreakpoint can be created
SurfacePoint/CellCoverBreakdown/CloudOccupancySnapshot/vector type layouts known
queryCloudOccupancySnapshotClearance symbol/arg ABI known
ValidateCell depth-owner source/instruction known
all memory actions are read-only
```

若 symbol 被完全内联/删除，G9 run不启动；完成精确静态 blocker报告。不能用 product rebuild解决。

## 4. G9-T2：一次 private unchanged-binary run

运行合同与 G8 一致：

```text
fresh unused private port
fresh ROS_HOME
unchanged phase_offset_esdf_tube_single.launch
phase_offset_manual_observe_only:=false
unchanged pillar.pcd/parameters
unchanged formation_planning under read-only GDB Python
targeted bag before goal
/sim/local_map >=900 before goal
exactly one goal=(8,0,1), internal=(8,0,2)
```

bag 至少记录 `/sim/local_map`、`/sim/odom`、goal、base/tube candidate、raw/cloud/epoch/manual
diagnostics、rosout。所有命令显式使用本任务 master/ROS_HOME，不连接用户 master。

成功条件只有：

```text
one structurally matched depth terminal
exact 9 points from output memory
exact cover/breakdown/request
direct snapshot metadata
exact occupied dense vector dump
CAPTURE_COMPLETE
```

不要求 goal reached或 nonzero offset accepted。sentinel后正常结束任务run并只清理任务PID/PGID/port。

本规格只授权一次 run。若 preflight通过但 runtime仍不能完整捕获，分类 G；不改产品、不自动第二跑。

## 5. G9-T3：identity 与完整性验收

必须输出：

```text
terminal_identity.tsv
terminal_context.tsv
terminal_points.tsv
terminal_cover.tsv
terminal_snapshot_metadata.tsv
occupied_dump_manifest.json
identity_chain.tsv
capture_completeness.tsv
```

硬条件：

```text
9*xyz/w finite
cover/breakdown finite
request finite >= required clearance
depth==12 and both spans splittable
context address/cell exact match between Collect and Validate
snapshot direct reference captured on same thread/build
snapshot resolution == context snapshot resolution
snapshot sequence == Builder/bag map observation sequence
occupied bytes == voxel product
snapshot metadata passes cloudOccupancySnapshotConsistent semantics
```

任何硬条件失败，不做 definitive replay。

## 6. G9-T4：exact replay

按 G8 T4 要求重建 snapshot。两路交叉：

1. production-equivalent `queryCloudOccupancySnapshotClearance`；
2. brute-force full occupied closed AABB nearest distance。

对 9 点记录 status/certified/capped clearance、exact nearest voxel/address/AABB、observed/grid ball
containment、`D-R_planner`、`D-R_request`。不得把 capped request当 exact distance。

还要复算 captured sampled radius：

```text
max_i ||points[i]-points[4]||
```

若 `breakdown.decomposable=false`，核对 captured cover是否等于：

```text
1.1*sampled_radius + 0.5*snapshot_resolution
```

因为 terminal w0!=w1，绝不能使用 exact current-anchor omission。若 decomposable=true，核对
fixed/w/v breakdown sum与 cover。

## 7. G9-T5：主分类

exact inputs完整时必须选择：

```text
A_TRUE_PLANNER_RESIDUAL_CLEARANCE_INSUFFICIENT
B_CONTINUOUS_COVER_REQUEST_CLEARANCE_INSUFFICIENT
C_OBSERVED_DOMAIN_OR_MAP_SUPPORT_INSUFFICIENT
D_ALL_NINE_REQUESTS_EXACTLY_SAFE_MAX_DEPTH_PROOF_ONLY
E_GEOMETRY_OR_REGULARITY_DEPTH_TERMINAL
F_QUERY_OR_SNAPSHOT_IDENTITY_MISMATCH_PROVEN
```

D 必须满足 geometry valid、9点 KNOWN_FREE/certified、每点 exact distance >= request、snapshot
identity exact、owner only max depth且两个 span仍可切。只有 D 才允许主代理之后写极小 implementation
spec；G9 本身不改代码。

若 preflight或一次run仍不完整：

```text
G_G9_CALLEE_BOUNDARY_CAPTURE_INCOMPLETE
```

不得近似归入 A--F。

## 8. 测试、hash、diff、cleanup

不 build。运行已有已构建：

```text
tube_surface_validator_test
certified_tube_builder_test
cloud_occupancy_snapshot_test
phase_offset_cloud_occupancy_query_test
```

要求 22/22、11/11、14/14、4/4 PASS。比较 T0/final product和binary hashes，运行
`git diff --check`。独立 `/proc`+`ss` cleanup proof必须显示任务PID/port为空，用户进程未触碰。

## 9. 文档与 handoff

在本文件末尾追加 `DOCUMENT_STATUS=COMPLETE` completion，包含 run identity、主分类/branch、exact
terminal/cover/snapshot/replay摘要、证据路径、tests/hash/diff/cleanup和
`NO_PRODUCT_IMPLEMENTATION_PERFORMED`。

更新 `/home/cxq/ISF-GVF/handoff.md` 顶部，保留 G8历史。G9不自行授权实现。

## 10. 连续执行与停止边界

Luna Max 连续执行 preflight、一次run、replay、自审计和handoff，不中途等确认。只在以下之一结束：

```text
A/B/C/E/F exact failure classification
D exact proof-only classification
G one authorized callee-boundary capture exhausted
```

无论结果如何，不改product、不跑第二次、不加门控。

## G9 completion — 2026-08-21

```text
DOCUMENT_STATUS=COMPLETE
IMPLEMENTATION_AUTHORIZED=false
PRIMARY_CLASSIFICATION=G_G9_CALLEE_BOUNDARY_CAPTURE_INCOMPLETE
SECONDARY_CLASSIFICATION=PREFLIGHT_BLOCKED_NO_CONCRETE_COLLECTCELLPOINTS_ABI
TERMINAL_BRANCH=NOT_REACHED
ROS_PRIVATE_RUN_STARTED=false
NO_PRODUCT_IMPLEMENTATION_PERFORMED
```

The G9 T0 directory is:

```text
/tmp/tube_v2_g9_capture_20260821_190320/
```

The unchanged binary preflight was completed before any ROS launch.  The
anonymous-namespace `CollectCellPoints` has only an inline DWARF instance at
`ValidateCell+0x65` (`0x14285`) and no concrete out-of-line symbol in
`objdump -tC`.  A GDB `FinishBreakpoint` created on that inline frame resolves
back to the same entry address rather than a distinct return site.  The
`points` output object has a readable stack address, but `cover_radius` and
`breakdown` are scalarized/implicit: taking either address at entry fails and
the DWARF location is not an ABI-owned memory object.  Therefore the exact
callee-boundary contract cannot be met without an unauthorized binary or
product change, and the G9 stop boundary forbids starting the private ROS run.

The preflight did establish the remaining layouts and boundary:

```text
SurfacePoint sizeof=32; point offset=0; w offset=24
CellCoverBreakdown sizeof=32; fields fixed=0, w=8, v=16, decomposable=24
CloudOccupancySnapshot sizeof=216; occupied offset=176
std::vector<uint8_t> _M_start=0, _M_finish=8, _M_end_of_storage=16
queryCloudOccupancySnapshotClearance address=0x27660
query ABI: entry rdi is snapshot reference pointer, rsi is point reference pointer, xmm0 is radius
ValidateCell depth owner: source line 513 at 0x1485a
```

No terminal cell, exact nine points, cover, breakdown, request radius, direct
snapshot metadata, occupied vector, `CAPTURE_COMPLETE`, or replay exists;
these are explicitly `NOT_CAPTURED`/`NOT_RUN`, not approximated.  No A--F
classification is claimed.  The run contract retained the unchanged launch,
`pillar.pcd`, `observe_only=false`, and goal `(8,0,1)` (internal `(8,0,2)`),
but private port `13019` was never started or listened on.

Existing built tests were run without a build and passed:

```text
tube_surface_validator_test                 22/22 PASS
certified_tube_builder_test                 11/11 PASS
cloud_occupancy_snapshot_test               14/14 PASS
phase_offset_cloud_occupancy_query_test      4/4 PASS
```

All frozen product hashes, launch/map hashes, and formation-planning binary
SHA-256/build ID match T0; `git diff --check` passes.  The cleanup proof shows
no G9 task process and no listener on port `13019`; existing user ROS
processes were not attached to, signalled, or touched.

Evidence index:

```text
/tmp/tube_v2_g9_capture_20260821_190320/g9_preflight_acceptance.tsv
/tmp/tube_v2_g9_capture_20260821_190320/g9_navigation_preflight.log
/tmp/tube_v2_g9_capture_20260821_190320/g9_static_preflight_navigation.log
/tmp/tube_v2_g9_capture_20260821_190320/g9_plan_env_preflight.log
/tmp/tube_v2_g9_capture_20260821_190320/g9_addr_probe.log
/tmp/tube_v2_g9_capture_20260821_190320/g9_finish_probe.log
/tmp/tube_v2_g9_capture_20260821_190320/run_status.tsv
/tmp/tube_v2_g9_capture_20260821_190320/focused_tests.tsv
/tmp/tube_v2_g9_capture_20260821_190320/binary_identity_compare.tsv
/tmp/tube_v2_g9_capture_20260821_190320/launch_map_hash_status.tsv
/tmp/tube_v2_g9_capture_20260821_190320/cleanup_external_check.txt
```

Next action is only to report this exact preflight capture blocker.  G9 does
not authorize a second run, a rebuild, an instrumentation change, or an
implementation.
