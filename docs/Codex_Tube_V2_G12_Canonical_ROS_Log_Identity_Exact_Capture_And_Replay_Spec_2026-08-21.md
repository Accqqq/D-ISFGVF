# Tube V2 G12 canonical ROS-log identity exact capture and replay

```text
DOCUMENT_ROLE=READ_ONLY_IDENTITY_PARSER_CORRECTION_AND_EXACT_CAPTURE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_G12
IMPLEMENTATION_AUTHORIZED=false
PRODUCT_SOURCE_HEADER_TEST_LAUNCH_CONFIG_PARAMETER_SCHEMA_EDITS_AUTHORIZED=false
PRODUCT_REBUILD_AUTHORIZED=false
UNCHANGED_BINARY_PRIVATE_ROS_AUTHORIZED=true
READ_ONLY_GDB_PYTHON_AUTHORIZED=true
EXISTING_PTRACER_SHIM_AUTHORIZED_FOR_TASK_LAUNCH_ONLY=true
PRODUCT_MEMORY_WRITE_AUTHORIZED=false
AUTO_ADVANCE=G11_PARSER_FAILURE_TO_SHARED_CANONICAL_IDENTITY_PROOF_TO_ONE_PRIVATE_RUN_TO_CAPTURE_TO_REPLAY_TO_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

同一个 GPT-5.6 Luna Max 单代理连续执行。禁止子代理、委派和并行代理工作；不得中途等待确认。

## 0. G11精确结论和G12唯一修正

G11创建了一次fresh private ROS，第一层resolver正确选中本任务：

```text
roslaunch PID=1113576
formation PID=1113645
formation executable/hash=PASS
not pre-existing=PASS
starttime=PASS
roslaunch descendant chain=PASS
argv0/name=PASS
fresh ROS_HOME log path=PASS
private ROS_MASTER_URI/ROS_HOME/LD_PRELOAD=PASS
ptracer maps=PASS
UID=PASS
TracerPid=0 PASS
selected count=1
```

外部用户 formation PID `627777/716149` 被明确排除且未attach/未signal。G11随后在第二次identity读取处
失败：当时的第二脚本没有按ROS canonical remap参数 `__log:=<path>` 解析日志参数，因而把正确cmdline
误读为空日志路径。失败发生在GDB前：无attach、无bag、无harness、无goal、无capture；端口13021和
任务进程已清理。分类：

```text
G_G11_TASK_OWNED_CAPTURE_INCOMPLETE
SECOND_IDENTITY_MISMATCH
actual root cause=SECOND_IDENTITY_LOG_ARGUMENT_PARSER_DID_NOT_ACCEPT___log_COLON_EQUALS
```

这不是 Tube 或运行身份实际失败。G12唯一修正是让第一/第二identity使用同一个canonical cmdline解析
helper，并对第一次读取的raw cmdline做bitwise一致性核对。G10 machine map、G11 ownership合同、GDB
capture、snapshot replay和A--G分类保持不变。

## 1. 边界与白名单

完整读取：`AGENTS.md`、handoff顶部G11、G10/G11 spec completion、本G12 spec，以及：

```text
/tmp/tube_v2_g11_capture_20260821_200450/task_ownership_proof.tsv
/tmp/tube_v2_g11_capture_20260821_200450/selected_pid_identity.tsv
/tmp/tube_v2_g11_capture_20260821_200450/selected_pid_identity_second.tsv
/tmp/tube_v2_g11_capture_20260821_200450/ros_run/run_status.tsv
/tmp/tube_v2_g11_capture_20260821_200450/ros_run/process_scan_after.txt
/tmp/tube_v2_g11_capture_20260821_200450/ros_run/listeners_after.txt
```

只允许写：

```text
docs/Codex_Tube_V2_G12_Canonical_ROS_Log_Identity_Exact_Capture_And_Replay_Spec_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g12_*
```

禁止修改/重编译任何产品代码、test、launch、参数、map、CMake、schema或binary；禁止修改G1--G11
文档。禁止任何gate/cache/retry/timer/parameter/module/第二Validator、安全裕度削弱、depth/query limit
提高、product log/instrumentation和GDB memory write。禁止git destructive操作和触碰用户ROS。

## 2. G12-T0：冻结和preflight

建立全新 `/tmp/tube_v2_g12_capture_20260821_<id>/`。重新比较G10/G11冻结的31个product hash、binary
hash/build-id、absolute launch、pillar map和ptracer shim：

```text
/tmp/tube_v2_g6_ptracer.c  sha256=3d1cde49f16eac236f70e0962f6e185a957614338b5b535c93dabdd49da9c299
/tmp/tube_v2_g6_ptracer.so sha256=087d8dedd5f555469f472fbd37b0d5cdf9b9bd49e7d91f4e43b463beb90046ae
```

任何identity变化禁止ROS、分类G。不重新build，不重复G10 unit machine-map探索。

记录G12 T0全部numeric PID、exact formation PID/starttime/cmdline log path、listeners和fresh private port；
不得输出完整process environ。

## 3. G12-T1：共享canonical identity parser

在G12 `/tmp`中建立一个共享helper，first resolver和second identity必须import同一实现，禁止各写一套
`__log`解析逻辑。

NUL分隔cmdline解析合同：

```text
argv[0] == exact formation binary
exactly one b'__name:=formation_planning'
exactly one canonical b'__log:=<nonempty path>'
```

可只为审计兼容识别 `__log=<path>`，但当前ROS实际输入必须记录为
`LOG_ARGUMENT_FORM=COLON_EQUALS`。对`__log:=`必须以字节前缀 `b'__log:='` 匹配并从 `:=` 后截取，
不得用模糊substring或错误的`__log=`前缀。

启动ROS前对共享parser运行纯 `/tmp` fixture：

1. canonical `__log:=/tmp/run/ros_home/log/id/formation_planning-4.log` -> exact path PASS；
2. empty `__log:=` -> FAIL；
3. two log args -> FAIL；
4. missing name -> FAIL；
5. NUL-separated real argv roundtrip -> PASS。

输出 `identity_parser_preflight.tsv`，全部PASS才允许ROS。

## 4. G12-T2：task PID ownership与二次bitwise identity

沿G11的12项ownership proof，仅从本任务roslaunch `/proc` descendant中选择exact formation PID；禁止
全局first-match/pgrep。T0 PID全部排除，不读取/attach其内存。

每次resolver polling写独立 `resolver_attempt_<n>/`，不得让早期“尚未出现candidate”的failure sentinel
残留并污染最终成功状态。成功时必须只有一个candidate，并将成功attempt原子复制/汇总为：

```text
pid_candidate_audit.tsv
task_ownership_proof.tsv
task_parent_chain.tsv
selected_pid_identity.tsv
selected_cmdline.bin
selected_cmdline.sha256
```

第一次identity必须保存selected PID的exact NUL cmdline bytes，但只把安全的argv0/name/log摘要写TSV。

紧接GDB attach前，second identity必须：

1. 重新读取 `/proc/<pid>/stat`，PID/starttime与第一次精确一致；
2. exe realpath/hash精确一致；
3. raw `/proc/<pid>/cmdline` bytes和第一次 `selected_cmdline.bin` bitwise一致，SHA一致；
4. 用同一共享parser再次得到exact canonical log path；
5. ROS_MASTER_URI、ROS_HOME、LD_PRELOAD三项一致；
6. shim maps、UID、parent chain一致；
7. `TracerPid=0`；
8. log path位于G12 fresh `ROS_HOME/log/`。

输出 `selected_pid_identity_second.tsv`。任何一项失败：不得attach/bag/harness/goal，清理任务PID并分类G。

## 5. G12-T3：attach和private run

启动顺序固定：

```text
fresh roscore without LD_PRELOAD
-> unchanged absolute roslaunch with exact existing ptracer shim
-> first task ownership proof
-> second bitwise identity proof
-> read-only GDB attach to that one PID
-> GDB inferior PID/starttime/exe/object identity
-> targeted bag without LD_PRELOAD
-> harness without LD_PRELOAD
-> /sim/local_map>=900
-> exactly one goal (8,0,1), internal (8,0,2)
-> post-goal exact capture or bounded timeout
-> task-only cleanup
```

只有roslaunch及其task children继承 `/tmp/tube_v2_g6_ptracer.so`；master/GDB/bag/harness显式
`env -u LD_PRELOAD`。launch、pillar map、`observe_only=false`和全部参数不变。

GDB attach gate必须写：

```text
selected_inferior_pid == proved PID
selected starttime == second identity starttime
formation executable/object == frozen binary
G12_GDB_STATE_MACHINE_ATTACHED
```

未通过前不得启动bag/harness。

## 6. G12-T4：GDB capture hard contract

从G11新脚本复制到G12新目录并进行静态自审，不覆盖G11 artifacts。启动前必须通过：

```text
embedded GDB Python compile PASS
resolver/shared-parser/second-identity/harness py_compile PASS
all paths and markers contain G12, no G10/G11 output path
DepthBP key_for(context,cell_ptr,cell,build) signature PASS
terminal snapshot_info(...,dump=True) PASS
snapshot_identity_ok called before any terminal file/sentinel PASS
G12 inferior identity marker PASS
```

capture semantics沿用G11并明确：

- `ValidateCell+0x1338`：`xmm3=cover`、`xmm9=target`、9点=`rsp+0x310`；
- `ValidateCell+0x63a`：depth owner；
- only post-`GOAL_PUBLISHED` build；
- depth=12、两个span可切、非current-anchor、query budget尚可容纳9点；
- first query per Builder uses hidden-sret ABI `rdi/rsi/rdx/xmm0`；
- query breakpoint first hit后立即disable，下一post-goal Builder再enable；
- exact key=`thread/context/cell_ptr/w0bits/w1bits/v0bits/v1bits/build_serial`；
- terminal时dump same-build direct snapshot occupied vector，验证sequence/resolution/count/pointers/hash；
- 所有hard项PASS后才写 `CAPTURE_COMPLETE`。

最少输出G11规定的terminal identity/context/cell/points/cover/result/snapshot/occupied/identity/completeness
文件。不得用marker/bag近似替代live exact fields。

## 7. G12-T5：exact replay

仅在capture完整后，于G12 `/tmp`中使用standalone helper链接冻结的`libplan_env`，重建captured snapshot，
对9点分别以planner residual和captured request调用真实
`queryCloudOccupancySnapshotClearance`。同时独立遍历dense occupied bytes，按occupied closed voxel AABB
计算exact nearest distance/voxel/address/AABB。

逐点输出status/certified/capped clearance、observed/grid ball containment、exact distance、
`D-required_clearance`、`D-requested`和production/brute-force一致性。

复算 sampled radius。nondegenerate sampled route必须满足：

```text
cover = 1.1*sampled_radius + 0.5*snapshot_resolution
```

不匹配则标certificate/other，不能伪造breakdown，但不妨碍exact clearance分类。

## 8. A--G分类

exact输入完整时按G11同一优先级：

```text
F_QUERY_OR_SNAPSHOT_IDENTITY_MISMATCH_PROVEN
E_GEOMETRY_OR_REGULARITY_DEPTH_TERMINAL
C_OBSERVED_DOMAIN_OR_MAP_SUPPORT_INSUFFICIENT
A_TRUE_PLANNER_RESIDUAL_CLEARANCE_INSUFFICIENT
B_CONTINUOUS_COVER_REQUEST_CLEARANCE_INSUFFICIENT
D_ALL_NINE_REQUESTS_EXACTLY_SAFE_MAX_DEPTH_PROOF_ONLY
```

D硬条件不变：geometry valid、9点request全部KNOWN_FREE/certified、exact D>=request、snapshot identity、
owner仅max-depth、depth12、两个span可切、非query-budget/unsplittable。只有D允许主代理后续极小代码spec。

若attach/goal/capture/snapshot/replay不完整：

```text
G_G12_CANONICAL_IDENTITY_CAPTURE_INCOMPLETE
```

## 9. cleanup、测试与文档

本spec只授权一次private run。只清理manifest记录的task PID/证明后的descendants；不得按全局名称/pattern
杀进程。验证private port空、task PID空、T0 external PID未attach/未signal。

不build，运行已有：22/22、11/11、14/14、4/4 focused tests。比较31 product hashes、binary/launch/map/
shim hashes，运行`git diff --check`。

在本spec末尾追加completion并更新handoff顶部，完整写primary/secondary、ownership/bitwise identity、
attach、map/goal、terminal、snapshot/replay margins、tests/hash/diff/cleanup和：

```text
NO_PRODUCT_IMPLEMENTATION_PERFORMED
```

Luna Max连续完成全部步骤，不中途停下等待确认；无论A--G，G12不改产品、不加门控、不跑第二次。

## 10. G12 completion — canonical identity and attach passed; rosbag gate stopped run

```text
DOCUMENT_STATUS=COMPLETE
PRIMARY_CLASSIFICATION=G_G12_CANONICAL_IDENTITY_CAPTURE_INCOMPLETE
SECONDARY_CLASSIFICATION=ROSBAG_START_READINESS_FALSE_NEGATIVE_AFTER_ATTACH
NO_PRODUCT_IMPLEMENTATION_PERFORMED
NO_SECOND_PRIVATE_RUN_AUTHORIZED
```

G12-T0 froze the same 31 product files, formation/navigation/plan_env
binaries/build IDs, absolute launch/map, and existing ptracer shim. The fresh
run directory was `/tmp/tube_v2_g12_capture_20260821_203231/` with private
port `13022`; T0 external formation PIDs `627777` and `716149` were excluded
without attach or signal.

The shared `g12_identity_parser.py` passed all five pre-ROS fixtures,
including canonical `__log:=` extraction, empty/duplicate/missing-name
rejection, and NUL argv roundtrip. First ownership succeeded on resolver
attempt 2 for roslaunch PID `1119698` and formation PID `1119767`
(starttime `29077991`); all twelve ownership fields passed and the exact raw
NUL cmdline was saved. The immediate second read imported the same parser and
passed PID/starttime, executable/hash, canonical log path, ROS environment,
shim map, UID, parent chain, `TracerPid=0`, and raw cmdline byte-for-byte and
SHA equality (`4cf3affe7245c3cce4851bbae8840a5f2334ce4eddc2bd9d083bf8eaca90d403`).

The read-only GDB inferior gate passed for the same formation PID and frozen
object (`G12_GDB_STATE_MACHINE_ATTACHED`). The run then launched the targeted
bag, but the bounded readiness poll expired before observing its startup line;
the post-run `rosbag.log` contains the valid `Recording to .../g12_targeted.bag`
line and an active bag file was present at the gate timeout. Because the gate
was conservatively false, the harness was not started: `/sim/local_map` never
reached the 900 precondition, no goal was published, and no post-goal exact
capture or replay occurred. This is a G12 orchestration incompleteness, not an
identity or product failure. No A--F classification is claimed.

Task-owned processes (roscore `1119542`, roslaunch `1119698`, formation
`1119767`, GDB `1119819`, bag `1119860`) were cleaned; port `13022` is free.
External PIDs remained untouched. Focused tests passed 22/22, 11/11, 14/14,
and 4/4; all product hashes, binary/build IDs, launch/map/shim hashes, and
`git diff --check` match the frozen baseline.

Evidence:

```text
/tmp/tube_v2_g12_capture_20260821_203231/identity_parser_preflight.tsv
/tmp/tube_v2_g12_capture_20260821_203231/resolver_success.tsv
/tmp/tube_v2_g12_capture_20260821_203231/task_ownership_proof.tsv
/tmp/tube_v2_g12_capture_20260821_203231/selected_cmdline.bin
/tmp/tube_v2_g12_capture_20260821_203231/selected_cmdline_second.bin
/tmp/tube_v2_g12_capture_20260821_203231/selected_pid_identity_second.tsv
/tmp/tube_v2_g12_capture_20260821_203231/gdb_attach_gate.tsv
/tmp/tube_v2_g12_capture_20260821_203231/ros_run/cleanup_external_check.txt
/tmp/tube_v2_g12_capture_20260821_203231/tests/focused_tests.tsv
```
