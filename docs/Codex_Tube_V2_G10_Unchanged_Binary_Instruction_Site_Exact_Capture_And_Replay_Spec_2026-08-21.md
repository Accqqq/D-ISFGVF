# Tube V2 G10 unchanged-binary instruction-site exact capture and replay

```text
DOCUMENT_ROLE=READ_ONLY_MACHINE_INSTRUCTION_CAPTURE_AND_OFFLINE_REPLAY_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_G10
IMPLEMENTATION_AUTHORIZED=false
SOURCE_OR_TEST_EDITS_AUTHORIZED=false
UNCHANGED_BINARY_PRIVATE_ROS_AUTHORIZED=AFTER_UNIT_PREFLIGHT_PASS
READ_ONLY_GDB_PYTHON_AUTHORIZED=true
PRODUCT_MEMORY_WRITE_AUTHORIZED=false
AUTO_ADVANCE=FREEZE_TO_DISASSEMBLY_MAP_TO_UNIT_RUNTIME_PREFLIGHT_TO_ONE_PRIVATE_RUN_TO_EXACT_REGISTER_STACK_SNAPSHOT_CAPTURE_TO_REPLAY_TO_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

GPT-5.6 Luna Max 单代理连续执行。禁止创建、调用、委派、请求或等待子代理，不得并行代理工作。

## 0. G9 结论和 G10 唯一变化

G9 证明 `CollectCellPoints` 被完全内联，无法使用 callee/return boundary；因此未启动 ROS：

```text
G_G9_CALLEE_BOUNDARY_CAPTURE_INCOMPLETE
PREFLIGHT_BLOCKED_NO_CONCRETE_COLLECTCELLPOINTS_ABI
```

G9 同时冻结了当前 `libphase_offset_navigation.so` 的机器布局：

```text
ValidateCell symbol start = 0x14220 in ELF
points stack object initialized at rsp+0x310
SurfacePoint sizeof=32, point offset=0, w offset=24
depth owner = ValidateCell+0x63a / ELF 0x1485a
cover-vs-target compare = ValidateCell+0x1338 / ELF 0x15558
instruction = comisd %xmm3,%xmm9
at compare: xmm3=cover_radius, xmm9=target_cover
context=%rbp, cell=%r12
queryCloudOccupancySnapshotClearance ABI after hidden struct-return lowering:
  rdi=&return_result, rsi=&snapshot, rdx=&point, xmm0=radius
CloudOccupancySnapshot sizeof=216, occupied offset=176
std::vector<uint8_t> start/finish/end offsets=0/8/16
```

G10 唯一观测变化：不再要求不存在的 C++ output address，而是在已经验证的 production machine
instruction 上读取 live stack/register值。先用现有 unit test runtime preflight证明映射，再允许一次
private ROS。binary、代码和产品内存均不改变。

## 1. 冻结边界

不得修改/重编译任何 product source/header/test/launch/config/parameter/CMake/schema/binary 或 G1--G9
文档。只允许写：

```text
docs/Codex_Tube_V2_G10_Unchanged_Binary_Instruction_Site_Exact_Capture_And_Replay_Spec_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g10_*
```

禁止 `set variable`、inferior function call、memory write、LD_PRELOAD、hot patch、debug rebuild、
product log/schema、depth/query/clearance/cover/epsilon/regularity/map/goal/observe_only变更，以及任何
gate/cache/retry/timer/module/第二 Validator。不得触碰用户 ROS/process，不得做 git destructive操作。

## 2. G10-T0：hash/build-id/disassembly 冻结

完整读取 AGENTS、handoff顶部G9、G8/G9 completion、G9 preflight artifacts和本spec。建立
`/tmp/tube_v2_g10_capture_20260821_<id>/`。

冻结 30 个 product hash、formation_planning、libphase_offset_navigation、libplan_env、launch、map、
build-id/ELF section/ASLR relocation。禁止 build。

用 `objdump/readelf/gdb info line/disassemble` 独立确认上述 offsets和指令；若任一 byte/hash/offset
不一致，禁止 ROS，分类 G。

## 3. G10-T1：现有 unit test 下的 machine-map runtime preflight

在已构建 `tube_surface_validator_test` 下运行只读 GDB Python。breakpoint使用 symbol-relative
`ValidateCell+0x1338`，不能写死 runtime absolute address。

每次 compare-site hit读取：

```text
context pointer = rbp
cell pointer = r12
points base = rsp+0x310
cover = low double of xmm3
target = low double of xmm9
9 SurfacePoint records, stride 32, xyz offsets 0/8/16, w offset 24
```

unit preflight必须至少覆盖：

1. sampled nondegenerate cell；
2. exact current anchor；
3. certified cell（若现有test命中）；
4. cover<=target query branch；
5. cover>target prequery branch。

用 test source期望、result `min/max_cover_radius/max_requested_clearance`、query captured radius或已知
analytic geometry交叉验证：

```text
xmm3 == expected cover
xmm9 == max(required_clearance,snapshot_resolution)
points memory == query points/order
host comparison target>=cover matches actual next control-flow branch
```

同时在 `queryCloudOccupancySnapshotClearance` unit test入口验证 hidden-sret ABI：
`rdi=&return_result`、`rsi=&snapshot`、`rdx=&point`、`xmm0=radius`。必须把 `rsi` 作为 direct
snapshot reference，验证 metadata offsets和occupied vector pointers；将同一 snapshot dump后与
test-built known occupancy/hash核对。不得把 `rdi` 的返回对象缓冲区误当 snapshot。

preflight输出 `machine_map_runtime_preflight.tsv`。所有硬项 PASS 才允许 private ROS。任何寄存器/offset/
branch不一致，禁止 ROS，不得猜测。

## 4. G10-T2：GDB Python host-side state machine

### 4.1 Builder activation

在 `CertifiedTubeBuilder::build` entry建立 current build record：thread、input current_w、map observation
sequence、snapshot resolution。每个 build entry重新允许一次 direct snapshot reference capture。

### 4.2 direct snapshot capture

在该 build 的第一次 `queryCloudOccupancySnapshotClearance` entry：

```text
return-result address = rdi
snapshot address = rsi
point address = rdx
requested radius = xmm0
```

读取 snapshot metadata并保存 object address；同一 build后续 query breakpoint静默禁用，避免数万次
trap。下一 Builder entry再启用。不得在此时每次dump occupied vector。

### 4.3 compare-site candidate capture

在 `ValidateCell+0x1338` 只处理：

```text
cell.depth==12
w_span>1e-10
v_span>1e-10
non-current-anchor
query_count+9<=max_query_samples
```

立即从 `rsp+0x310` raw read 288 bytes，解码9点；读取xmm3/xmm9和context scalars。保存candidate：

```text
thread/context/cell exact identity
points
cover/target
requested=required+cover+epsilon
host branch prequery=(cover>target)
snapshot object address from same thread/current build
```

### 4.4 depth-owner pairing

在 `ValidateCell+0x63a` 只处理相同 structural predicate。按同 thread、context pointer和cell四个 exact
doubles选择 compare candidate；不允许 nearest匹配。

若 exact pair成功：

1. 捕获 result counters/reason；
2. 从 direct snapshot object读取完整 metadata；
3. 从 occupied vector start/finish读取 exact bytes；
4. 验证 resolution、sequence/build identity和byte count；
5. 写 terminal TSV、snapshot binary/hash、`CAPTURE_COMPLETE`；
6. continue inferior。

若 depth hit没有 compare candidate，说明 geometry invalid或machine path未覆盖；记录 E/G候选并继续
寻找下一个完整structural terminal，不得错误配对。

## 5. G10-T3：一次 private ROS

仅在 unit runtime preflight全 PASS 后运行一次：fresh unused private master/ROS_HOME、unchanged
absolute launch、pillar.pcd、observe_only=false、unchanged params、targeted bag、`/sim/local_map>=900`
后 exactly one goal `(8,0,1)`。

成功条件是 exact terminal 9点/cover/target/request + same build direct snapshot metadata + occupied binary
+ sentinel。无需goal reached/nonzero accepted。

本spec只授权一次run；失败不改product、不自动第二跑。仅清理任务PID/PGID/port，独立检查用户进程
未触碰。

## 6. G10-T4：exact replay

按 production semantics和brute-force closed occupied AABB两路重放9点。输出 status/certified/capped
clearance、exact nearest voxel/AABB/distance、observed/grid request-ball containment、`D-Rplanner`、
`D-Rrequest`。

复算 sampled radius。若captured route为sampled nondegenerate，必须验证：

```text
cover = 1.1*sampled_radius + 0.5*snapshot_resolution
```

若certificate route，则从machine path/available breakdown/result明确标识；即使breakdown未捕获，exact
cover+points+snapshot仍可做clearance分类，但不得伪称sampled公式。

## 7. 主分类

exact inputs完整时选：

```text
A_TRUE_PLANNER_RESIDUAL_CLEARANCE_INSUFFICIENT
B_CONTINUOUS_COVER_REQUEST_CLEARANCE_INSUFFICIENT
C_OBSERVED_DOMAIN_OR_MAP_SUPPORT_INSUFFICIENT
D_ALL_NINE_REQUESTS_EXACTLY_SAFE_MAX_DEPTH_PROOF_ONLY
E_GEOMETRY_OR_REGULARITY_DEPTH_TERMINAL
F_QUERY_OR_SNAPSHOT_IDENTITY_MISMATCH_PROVEN
```

D要求9点全部KNOWN_FREE/certified且exact distance>=request，snapshot identity exact，owner仅max-depth、
spans仍可切。只有D允许主代理后续写极小implementation spec；G10不改代码。

若unit map preflight或一次run/capture不完整：

```text
G_G10_INSTRUCTION_SITE_CAPTURE_INCOMPLETE
```

## 8. 测试、不变性和文档

不build。运行现有focused tests 22/22、11/11、14/14、4/4。比较product/binary/launch/map hashes，
`git diff --check`，独立cleanup proof。

在本spec末尾追加 completion并更新handoff顶部：分类、branch、run/preflight、exact points/cover/snapshot/
minimum margin、tests/hash/diff/cleanup、`NO_PRODUCT_IMPLEMENTATION_PERFORMED`。

## 9. 连续执行边界

Luna Max连续完成T0--T4、分类、自审计和handoff，不中途等确认。无论A--G，不改product、不调参数、
不加门控、不跑第二次。

## G10 completion — 2026-08-21

```text
CLASSIFICATION=G_G10_INSTRUCTION_SITE_CAPTURE_INCOMPLETE
SECONDARY=PRIVATE_ROS_ATTACH_PRECONDITION_FAILED_NO_CAPTURE
BRANCH=unchanged-worktree-read-only-preflight
NO_PRODUCT_IMPLEMENTATION_PERFORMED
NO_SECOND_RUN_AUTHORIZED
```

The corrected hidden-struct-return query ABI was frozen and validated as
`rdi=&return_result, rsi=&snapshot, rdx=&point, xmm0=radius`; the direct
snapshot reference is therefore `rsi`.  Navigation runtime preflight passed
under the existing binary: 1,102 compare-site hits, 414 depth-owner hits, 990
sampled nondegenerate rows, 112 exact-anchor rows, 14 certified rows, 927
`cover<=target` rows, 175 `cover>target` prequery rows, and 1,102/1,102
actual-branch matches.  Query preflight passed for the 14-test snapshot suite
(6,190 calls, 6,187/6,187 valid-population metadata/vector reads, 14 known
fixture hashes, two explicitly isolated expected-invalid negative fixtures)
and the 4-test cloud bridge (3 calls, one known fixture hash).  Evidence is
under `/tmp/tube_v2_g10_capture_20260821_192802/`, including
`machine_map_runtime_preflight.tsv`, the labeled query preflight TSVs, and the
read-only GDB logs.

The one authorized private attempt used fresh ROS master/ROS_HOME on port
13020 and the unchanged absolute pillar launch.  Launch created task-owned
formation PID 1105587, but the generic PID discovery selected pre-existing
user-owned PID 627777; read-only GDB correctly refused ptrace on that target.
The attach gate failed before rosbag/harness startup, before the 900-map
precondition, and before any goal publication.  No terminal candidate,
points, cover, request, snapshot, occupied dump, sentinel, or replay exists;
no A--F safety classification or minimum-margin claim is made.  Only the
task-owned master/launch/formation processes were cleaned, port 13020 is free,
and user-owned formation PIDs 627777 and 716149 received no successful attach,
signal, memory write, or process mutation (the attempted ptrace was refused).
The run proof is in `ros_run/run_status.tsv` and
`ros_run/cleanup_external_check.txt`.

Focused tests passed: SurfaceValidator 22/22, CertifiedTubeBuilder 11/11,
CloudOccupancySnapshot 14/14, and cloud query 4/4.  All 31 frozen product
hashes match T0, unchanged binary hashes were recorded, and `git diff --check`
passes.  No product source, test, launch, parameter, map, binary, or memory
was modified or written.
