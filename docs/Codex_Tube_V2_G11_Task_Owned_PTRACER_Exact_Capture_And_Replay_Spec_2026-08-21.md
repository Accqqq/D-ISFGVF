# Tube V2 G11 task-owned ptracer exact capture and replay

```text
DOCUMENT_ROLE=READ_ONLY_TASK_OWNERSHIP_CORRECTION_AND_EXACT_REPLAY_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_G11
IMPLEMENTATION_AUTHORIZED=false
SOURCE_HEADER_TEST_LAUNCH_CONFIG_PARAMETER_SCHEMA_EDITS_AUTHORIZED=false
PRODUCT_REBUILD_AUTHORIZED=false
UNCHANGED_BINARY_PRIVATE_ROS_AUTHORIZED=true
READ_ONLY_GDB_PYTHON_AUTHORIZED=true
EXISTING_PTRACER_SHIM_AUTHORIZED_FOR_TASK_LAUNCH_ONLY=true
PRODUCT_MEMORY_WRITE_AUTHORIZED=false
AUTO_ADVANCE=G10_FREEZE_TO_TASK_OWNERSHIP_PROOF_TO_ONE_PRIVATE_RUN_TO_EXACT_CAPTURE_TO_REPLAY_TO_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

本阶段由同一个 GPT-5.6 Luna Max 单代理连续执行。禁止创建、调用、委派、请求或等待任何子代理，
不得把任一步交给其他模型。完成 G11 的捕获、重放、分类、测试、自审计和 handoff 前，不要中途等待
用户确认。

## 0. G10 结论、G11 唯一问题和产品状态

G10 的 product/unit machine map 已经完整通过：

```text
ValidateCell ELF start                       0x14220
cover/target compare                         ValidateCell+0x1338 / ELF 0x15558
compare instruction                          comisd %xmm3,%xmm9
xmm3                                         cover_radius
xmm9                                         target_cover
SurfacePoint[9]                              rsp+0x310, stride 32
SurfacePoint xyz/w offsets                   0/8/16/24
depth owner                                  ValidateCell+0x63a / ELF 0x1485a
query hidden-sret ABI                        rdi=&result,rsi=&snapshot,rdx=&point,xmm0=radius
CloudOccupancySnapshot size/occupied offset  216/176
occupied vector start/finish/end offsets     0/8/16
```

现有 unit runtime evidence：

```text
SurfaceValidator tests                       22/22 PASS
compare hits                                 1102
sampled nondegenerate                        990
exact current anchor                         112
certified                                    14
cover<=target                                927
cover>target                                 175
actual branch match                          1102/1102
snapshot suite                               14/14 PASS
snapshot ABI/metadata valid population       6187/6187
known snapshot hashes                        14/14
cloud bridge                                 4/4 PASS
```

G10 private run没有进入动态验收。它通过全局 `ps|awk` 的第一个匹配误选了 T0 前已存在的用户 PID
`627777`，而不是本任务 launch 创建的 PID `1105587`；kernel拒绝了 attach。rosbag、harness、900-map
条件、goal、terminal capture和replay均未启动。G10分类为：

```text
G_G10_INSTRUCTION_SITE_CAPTURE_INCOMPLETE
PRIVATE_ROS_ATTACH_PRECONDITION_FAILED_NO_CAPTURE
```

G11 唯一变化是修复任务运行编排：严格证明 formation PID 属于本次 fresh private launch，并只对该
PID使用已存在的只读 ptracer shim和GDB。G11不修改 Tube 生产逻辑，不添加门控、cache、retry、timer、
parameter、module、第二 Validator，也不改变安全裕度、cover、epsilon、depth或query limit。

## 1. 必读输入和冻结证据

执行前完整读取：

1. `AGENTS.md`；
2. `/home/cxq/ISF-GVF/handoff.md` 顶部 G10 final；
3. 本 G11 spec；
4. G10 spec completion；
5. G10 artifacts：
   - `/tmp/tube_v2_g10_capture_20260821_192802/machine_map_runtime_preflight.tsv`
   - `/tmp/tube_v2_g10_capture_20260821_192802/query_machine_map_runtime_preflight_cloud.tsv`
   - `/tmp/tube_v2_g10_capture_20260821_192802/query_machine_map_runtime_preflight_bridge.tsv`
   - `/tmp/tube_v2_g10_capture_20260821_192802/g10_capture.gdb`
   - `/tmp/tube_v2_g10_capture_20260821_192802/g10_map_harness.py`
   - `/tmp/tube_v2_g10_capture_20260821_192802/ros_run/run_status.tsv`
   - `/tmp/tube_v2_g10_capture_20260821_192802/ros_run/cleanup_external_check.txt`
6. G8成功 attach 的只读编排证据，只提取方法，不复用其 PID/run目录：
   - `/tmp/tube_v2_g8_capture_20260821_181934/gdb_attach_ready.txt`
   - `/tmp/tube_v2_g8_capture_20260821_181934/launch_map_ptracer_hash_t0.tsv`
7. 当前产品源码仅用于离线语义核对：
   - `tube_surface_validator.cpp`
   - `cloud_occupancy_snapshot.h/.cpp`

G11 必须建立新的：

```text
/tmp/tube_v2_g11_capture_20260821_<id>/
```

不得向 G8/G9/G10 目录追加或覆盖文件。

## 2. 严格写入白名单和禁止项

只允许写：

```text
docs/Codex_Tube_V2_G11_Task_Owned_PTRACER_Exact_Capture_And_Replay_Spec_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g11_*
```

不得修改或重编译任何 product source/header/test/launch/config/CMake/package/schema/binary；不得修改
G1--G10文档。不得修改 proposal、planner、map、simulator、Builder/Filter/Epoch/Runtime/Pair/H2或
SurfaceValidator production。

禁止：

```text
set variable
inferior function call
GDB memory write
hot patch
debug rebuild
product instrumentation/log/schema
parameter tuning
max_subdivision_depth increase
max_query_samples increase
clearance/cover/epsilon/regularity reduction
gate/cache/retry/timer/module/second Validator
git reset/clean/checkout/restore/stash/rebase
signal/attach/inspect-memory of any non-task PID
```

`/tmp`中的只读 GDB Python、PID resolver、ROS harness和offline replay helper不属于产品模块，run结束后
不得连接进CMake、launch或production。

## 3. G11-T0：冻结产品、shim和外部进程边界

### 3.1 product identity

直接复用 G10 已通过的 machine-map结果，不重复探索寄存器或offset，不重新跑大规模 unit GDB
preflight。先重新计算并比较 G10 冻结的 31 个 product hash、formation binary、navigation/core/plan_env
library、absolute launch和pillar map hash/build-id。任何产品或二进制 identity 不一致：禁止 ROS，分类 G。

### 3.2 ptracer shim identity

本阶段明确授权只对本次 roslaunch environment使用现有：

```text
/tmp/tube_v2_g6_ptracer.c
/tmp/tube_v2_g6_ptracer.so
```

要求：

```text
source sha256 = 3d1cde49f16eac236f70e0962f6e185a957614338b5b535c93dabdd49da9c299
shared-object sha256 = 087d8dedd5f555469f472fbd37b0d5cdf9b9bd49e7d91f4e43b463beb90046ae
```

源码必须只有 constructor 中的：

```c
prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0);
```

不得重编译、替换或复制另一个shim。hash/source不匹配则禁止 ROS，分类 G。该shim只改变本任务进程的
ptrace许可，不是产品代码，不得进入仓库或安装产物。

### 3.3 private run preflight

记录以下只读基线，但不得把完整 `/proc/<pid>/environ` 写入日志：

```text
all numeric PIDs existing at G11 T0
all exact formation_planning executable PIDs existing at T0
their PID/starttime/cmdline/log path only
current listeners
chosen unused private port
boot_id and clock tick rate
git status --short and tracked diff
```

选择一个 fresh unused private port和fresh `ROS_HOME`。已有listener一律视为外部资产，不得终止。

## 4. G11-T1：任务 PID resolver——attach 前必须全部通过

禁止使用全局 `ps|awk ... {print first}`、`pgrep` first match或仅靠进程名选择PID。创建一个仅位于
G11 `/tmp`目录的 resolver，直接读取 `/proc`，从本任务记录的 `ROSLAUNCH_PID` 向下遍历 parent chain。

候选必须同时满足下列全部硬条件：

1. PID不在G11 T0的pre-existing PID集合；
2. `/proc/<pid>/stat` starttime晚于G11 T0，也晚于或等于本任务roslaunch starttime；
3. 沿 `/proc/<pid>/stat` PPID链可精确到达本任务 `ROSLAUNCH_PID`，中间链完整且不经过PID 1；
4. `/proc/<pid>/exe` realpath严格等于：
   `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/devel/lib/bspline_race/formation_planning`；
5. executable SHA-256等于G11 T0冻结hash；
6. NUL分隔cmdline的 `argv[0]` 为上述exact binary，且存在exact `__name:=formation_planning`；
7. 唯一 `__log:=...` 参数的real parent路径位于G11 fresh `ROS_HOME/log/`；
8. 只从NUL分隔environ提取并核对以下三项，不得打印其他环境项：
   - `ROS_MASTER_URI=http://127.0.0.1:<G11_PORT>`
   - `ROS_HOME=<G11_RUN>/ros_home`
   - `LD_PRELOAD=/tmp/tube_v2_g6_ptracer.so`
9. `/proc/<pid>/maps`中存在同一realpath ptracer shim；
10. UID等于当前任务用户；
11. `TracerPid: 0`，尚未被其他调试器附着；
12. 所有条件通过后恰好只剩一个candidate。

resolver必须为每个被检查的 formation-like PID写 `pid_candidate_audit.tsv`，逐项标出PASS/FAIL和失败
原因；为唯一candidate写：

```text
task_ownership_proof.tsv
task_parent_chain.tsv
selected_pid_identity.tsv
```

在实际 `gdb -p` 前再次读取 selected PID的 `/proc/<pid>/stat` starttime、exe、cmdline、三个环境项、
parent chain和TracerPid，防止PID复用或身份漂移。两次identity必须精确一致。

若candidate为0个或多于1个、任一字段无法读取、或二次核对不一致：不得attach，不得启动bag/harness，
只清理本任务进程并分类 G。严禁尝试任何失败candidate，更严禁尝试T0已有PID。

## 5. G11-T2：private ROS启动和ptracer隔离

启动顺序固定：

```text
fresh roscore
-> unchanged absolute roslaunch with task-only LD_PRELOAD
-> exact task PID ownership proof
-> read-only GDB attach to selected PID
-> attach/inferior identity proof
-> targeted rosbag
-> 900 local-map harness
-> exactly one goal
-> exact capture or bounded timeout
-> task-only cleanup
```

环境限制：

- roscore、GDB、rosbag、harness命令显式 `env -u LD_PRELOAD`；
- 只有本次 unchanged absolute roslaunch命令显式设置
  `LD_PRELOAD=/tmp/tube_v2_g6_ptracer.so`，其task descendants可继承；
- launch仍为
  `src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch`；
- map仍为`pillar.pcd`；
- `phase_offset_manual_observe_only:=false`；
- 其他参数完全不变；
- GDB attach参数只能来自已写完ownership proof的一个 selected PID。

GDB Python启动后必须验证：

```text
gdb.selected_inferior().pid == selected PID
formation executable/object files match frozen binary
selected PID stat starttime still matches
G11_GDB_STATE_MACHINE_ATTACHED emitted
```

只有上述attach gate通过，才允许启动bag和harness。

## 6. G11-T3：基于 G10 的精确 GDB state machine

复制 G10 GDB脚本到新的G11 `/tmp`目录后再修改；不得覆盖G10 artifact。保持已验证的instruction map，
只做以下明确修正。

### 6.1 post-goal capture gate

harness在满足 `/sim/local_map >= 900`、确认goal subscriber存在并且只发布一次 `(8,0,1)` 后，原子写
`GOAL_PUBLISHED` sentinel，包含map count/sequence/time和publish count。GDB只允许在该sentinel存在后
激活compare/depth candidate capture；不得把pre-goal build误当目标。

### 6.2 Builder/query ownership

在每个 post-goal `CertifiedTubeBuilder::build` entry按thread建立新的build record：

```text
builder/input pointer
current_w
path_source_revision
tube_revision
map_observation_sequence
cloud_snapshot_resolution
map_observation_is_snapshot
build serial
```

每个Builder entry只启用一次query breakpoint。第一次
`queryCloudOccupancySnapshotClearance` entry使用已验证hidden-sret ABI读取：

```text
rdi=&return_result
rsi=&snapshot
rdx=&point
xmm0=requested radius
```

只保存direct snapshot pointer、metadata、vector start/finish/end和first-query identity；此时不得每个build
dump完整occupied vector。保存成功后立即禁用query breakpoint，下一Builder entry再重新启用，避免数万次
无意义trap。

### 6.3 compare candidate

在 `ValidateCell+0x1338` 仅处理：

```text
GOAL_PUBLISHED exists
cell.depth == 12
w_span > 1e-10
v_span > 1e-10
not exact current-anchor
query_count + 9 <= max_query_samples
same thread has active post-goal Builder and direct snapshot
```

立即读取：

```text
context = rbp
cell = r12
points raw bytes = rsp+0x310, exactly 9*32
cover = xmm3 low double
target = xmm9 low double
required_clearance = context+48
cover_epsilon = context+64
requested = required_clearance + cover + cover_epsilon
result counters and config limits
```

验证所有9点xyz/w finite、cover/target/request finite，并验证：

```text
target == max(required_clearance, snapshot_resolution)
requested >= required_clearance
host_prequery == (cover > target)
```

candidate key必须为：

```text
thread + context pointer + cell pointer + exact bit patterns of w0/w1/v0/v1 + build serial
```

禁止nearest/epsilon cell配对。

### 6.4 depth owner exact pairing and terminal dump

在 `ValidateCell+0x63a` 使用同一structural predicate和exact key。若没有exact compare candidate，只记录
unpaired原因并继续找下一个，不得猜测。

exact pair成功时，在inferior仍暂停于同一build期间：

1. 再次读取direct snapshot object metadata和vector pointers；
2. 要求snapshot valid、sequence等于Builder map sequence、resolution bitwise等于Builder/context
   resolution；
3. 要求occupied byte count等于 `voxel_count.x*y*z`，finish-start精确等于count，end>=finish；
4. 从start到finish一次性只读dump exact occupied bytes；
5. 计算SHA-256；
6. 写 exact terminal/cell/context/points/cover/snapshot/identity TSV和binary；
7. 写 `CAPTURE_COMPLETE`；
8. 禁用高频breakpoints并continue inferior。

最少必须生成：

```text
terminal_identity.tsv
terminal_context.tsv
terminal_cell.tsv
terminal_points.tsv
terminal_cover.tsv
terminal_result.tsv
terminal_snapshot_metadata.tsv
terminal_snapshot_occupied.bin
occupied_dump_manifest.tsv
identity_chain.tsv
capture_completeness.tsv
CAPTURE_COMPLETE
```

若snapshot identity/count或任何hard field失败，不得写成功sentinel，不得做definitive replay。

## 7. G11-T4：一次 unchanged private run

bag至少记录：

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

硬合同：

```text
private master and fresh ROS_HOME
unchanged absolute launch/map/parameters
observe_only=false
bag active before harness goal
/sim/local_map count >= 900 before goal
exactly one goal publish
goal=(8,0,1), internal=(8,0,2)
only post-goal terminal eligible
```

不要求goal reached、Pair commit或nonzero offset accepted；本阶段只需要一个完整structural terminal bundle。
本spec只授权一次private run。若runtime identity/capture仍不完整，分类 G，不改product，不在同一spec
自动第二跑。

## 8. G11-T5：production-equivalent + brute-force exact replay

仅在 `CAPTURE_COMPLETE` 和所有 completeness硬项PASS后重放。可以在G11 `/tmp`中创建standalone replay
source/binary，但不得写入仓库或CMake。优先链接当前已冻结的 `libplan_env`，重建捕获的
`CloudOccupancySnapshot`（private acceleration pointer保持null），对9点分别调用真实：

```text
queryCloudOccupancySnapshotClearance(snapshot, point, required_clearance)
queryCloudOccupancySnapshotClearance(snapshot, point, requested)
```

同时独立遍历完整dense occupied vector，对每个occupied voxel的closed AABB计算exact nearest Euclidean
distance和nearest voxel `(x,y,z,address,lower,upper)`。不得用voxel centre距离，不得用bag cloud或ESDF替代
captured occupied vector。

逐点输出：

```text
planner query status/certified/capped clearance
request query status/certified/capped clearance
brute-force exact nearest closed-AABB distance
nearest occupied voxel identity/AABB
point-in-map
planner ball in observed/grid boxes
request ball in observed/grid boxes
D_exact - required_clearance
D_exact - requested
production-vs-bruteforce consistency
```

重算：

```text
sampled_radius = max_i ||point[i]-point[4]||
sampled_nondegenerate_cover = 1.1*sampled_radius + 0.5*snapshot_resolution
```

若captured cover按production浮点顺序与该式匹配，标为sampled nondegenerate；否则只标为
certificate/other route，不得伪造breakdown。route未知不阻止基于exact points/cover/request/snapshot的
clearance分类。

## 9. G11 主分类和判定优先级

只有 exact capture identity、occupied dump、production replay和brute-force replay均完整时，才可选择
A--F。按以下优先级选最早阻止安全接受的事实：

### F. QUERY_OR_SNAPSHOT_IDENTITY_MISMATCH_PROVEN

Builder/context/snapshot sequence或resolution不一致、occupied length/hash不稳定、selected inferior不是
proved task PID、production replay与captured object出现不可解释identity矛盾。若仅是capture缺字段而未
证明矛盾，归G而不是F。

### E. GEOMETRY_OR_REGULARITY_DEPTH_TERMINAL

compare-site未覆盖且depth terminal有明确geometry invalid/regularity evidence。命中compare-site本身证明
该candidate的`geometry_valid=true`，不得再把它猜为E。

### C. OBSERVED_DOMAIN_OR_MAP_SUPPORT_INSUFFICIENT

任一点在planner或request半径查询为UNAVAILABLE/OUT_OF_MAP/UNKNOWN，或对应closed ball不完整包含于
observed/grid/map支持。C是fail-closed support不足，不等于障碍物，也不授权弱化安全。

### A. TRUE_PLANNER_RESIDUAL_CLEARANCE_INSUFFICIENT

domain完整时，任一点 exact closed-AABB distance `< required_clearance`，或点位于occupied voxel。

### B. CONTINUOUS_COVER_REQUEST_CLEARANCE_INSUFFICIENT

所有点 exact distance均 `>= required_clearance`，但至少一点 exact distance `< requested`。这表示planner
centre/reference residual本身安全，但完整连续cell cover request不安全；不得删除cover。

### D. ALL_NINE_REQUESTS_EXACTLY_SAFE_MAX_DEPTH_PROOF_ONLY

必须同时满足：

```text
geometry_valid=true from compare-site
all 9 request queries KNOWN_FREE
all 9 request queries clearance_certified=true
all 9 exact distances >= requested
snapshot/build/context identity exact
owner is max-depth
depth==12
w_span>1e-10 and v_span>1e-10
query budget not owner
unsplittable not owner
```

D才证明该terminal是现有9点+exact cover/request都安全但被max-depth proof scheduling拒绝。只有D允许
主代理随后编写极小implementation spec；G11本身不改任何代码。

若attach、goal、terminal、snapshot、dump或replay任一不完整：

```text
G_G11_TASK_OWNED_CAPTURE_INCOMPLETE
```

必须给出精确secondary，不得近似归入A--F。

## 10. cleanup、测试、hash和不变性

### 10.1 cleanup

清理只能寻址本任务manifest中记录的master/launch/formation/gdb/bag/harness PID或其已证明descendants。
先INT、等待，再仅对仍存活的task PID TERM。禁止按名称、全局pattern或端口杀进程，禁止signal T0已有
PID。run后独立验证：

```text
all task PIDs dead
all task PGIDs have no remaining members
private port has no listener
selected formation PID dead or TracerPid=0
all T0 external formation PIDs unchanged/present status recorded
user_processes_signalled=0
user_processes_attached=0
```

注意：G10曾对用户PID发起过一次被kernel拒绝的attach尝试；G11必须做到对任何T0/external PID连
attach尝试都不发生。

### 10.2 tests and identity

run结束后不build，运行现有已构建focused tests：

```text
tube_surface_validator_test                 expected 22/22
certified_tube_builder_test                 expected 11/11
cloud_occupancy_snapshot_test               expected 14/14
phase_offset_cloud_occupancy_query_test     expected 4/4
```

重新比较31个product hash、binary build-id/hash、launch/map/shim hash，运行 `git diff --check`。允许变化
只能是本G11 spec、handoff和G11 `/tmp` artifacts；不得归因、清理或覆盖用户已有dirty内容。

## 11. completion 和 handoff

在本spec末尾追加completion，并更新 `/home/cxq/ISF-GVF/handoff.md` 顶部，至少包括：

1. `DOCUMENT_STATUS=COMPLETE`；
2. A--G primary和secondary/branch；
3. private port、run dir、T0 external PID集合摘要；
4. exact selected PID ownership proof和attach inferior identity；
5. map count、goal count/time；
6. exact terminal cell/context/9点/cover/target/request；
7. snapshot sequence/resolution/voxel count/occupied hash；
8. replay逐点结果、minimum planner margin、minimum request margin；
9. sampled/certificate route判定；
10. tests、product/binary/launch/map/shim hash、diff；
11. task cleanup、private port和external user process proof；
12. `NO_PRODUCT_IMPLEMENTATION_PERFORMED`。

## 12. 连续执行边界

Luna Max连续执行 G11-T0--T5、分类、自审计和handoff，不中途等确认，不创建子代理。G11无论A--G均
不修改product、不调参数、不增加任何门控/模块、不运行第二次private场景。完成后立即把完整结果报告
主代理；主代理继续调度下一最小阶段。

## 13. G11 completion — one authorized run stopped at second identity gate

```text
DOCUMENT_STATUS=COMPLETE
PRIMARY_CLASSIFICATION=G_G11_TASK_OWNED_CAPTURE_INCOMPLETE
SECONDARY_CLASSIFICATION=SECOND_IDENTITY_RECHECK_LOG_PATH_PARSE_FAILURE__ROS_ARG_IS___LOG_COLON_EQUALS__NO_RUNTIME_IDENTITY_DRIFT
NO_PRODUCT_IMPLEMENTATION_PERFORMED
NO_SECOND_PRIVATE_RUN_AUTHORIZED
```

G11-T0 froze the unchanged product and binary set. The fresh private master
used port `13021` and run directory
`/tmp/tube_v2_g11_capture_20260821_200450/ros_run/`; the T0 external formation
PIDs were `627777` and `716149`. The unchanged launch created roslaunch PID
`1113576` (starttime `28996540`) and formation PID `1113645`
(starttime `28996593`). The resolver examined only new task candidates (T0
PIDs were excluded without opening their `/proc` entries), and its first
complete proof for PID `1113645` passed all twelve ownership checks: fresh
PID/starttime, exact roslaunch descendant chain, executable and SHA-256,
argv0/name, unique `__log:=` path under the fresh ROS_HOME, three selected
environment values, mapped ptracer shim, UID, and `TracerPid=0`.

Immediately before the authorized `gdb -p`, the second read proved the same
PID, starttime, executable, environment, shim mapping, UID, tracer state, and
parent chain. Its only failed field was the helper's log-argument parser,
which did not recognize the ROS spelling `__log:=` even though the resolver's
proof had already recorded the exact fresh log path. This is a capture
orchestration identity-gate defect, not runtime PID/executable identity drift.
The gate therefore refused attach; no GDB attach, ptrace of any external PID,
rosbag, harness, map precondition, goal, terminal, occupied dump, or replay
occurred. The task-owned master/launch/formation processes were cleaned, port
`13021` is free, and the external PIDs remained untouched.

No A--F classification is claimed. There are no terminal cell/context/point,
cover/target/request, snapshot, occupied hash, or replay margins because the
post-goal capture phase was never entered. Focused tests after cleanup passed:
`tube_surface_validator_test` 22/22,
`phase_offset_certified_tube_builder_test` 11/11,
`cloud_occupancy_snapshot_test` 14/14, and
`phase_offset_cloud_occupancy_query_test` 4/4. All 31 product hashes match
T0; formation/navigation/plan_env hashes and build IDs, absolute launch/map
hashes, and the ptracer source/shared-object hashes remain unchanged; `git
diff --check` is clean.

Evidence:

```text
/tmp/tube_v2_g11_capture_20260821_200450/pid_candidate_audit.tsv
/tmp/tube_v2_g11_capture_20260821_200450/task_ownership_proof.tsv
/tmp/tube_v2_g11_capture_20260821_200450/selected_pid_identity.tsv
/tmp/tube_v2_g11_capture_20260821_200450/selected_pid_identity_second.tsv
/tmp/tube_v2_g11_capture_20260821_200450/ros_run/cleanup_external_check.txt
/tmp/tube_v2_g11_capture_20260821_200450/tests/focused_tests.tsv
```
