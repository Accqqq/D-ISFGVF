# Tube V2 G13 ROS-graph recorder readiness exact capture and replay

```text
DOCUMENT_ROLE=READ_ONLY_RECORDER_READINESS_CORRECTION_AND_EXACT_CAPTURE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_G13
IMPLEMENTATION_AUTHORIZED=false
PRODUCT_EDITS_OR_REBUILD_AUTHORIZED=false
UNCHANGED_BINARY_PRIVATE_ROS_AUTHORIZED=true
READ_ONLY_GDB_PYTHON_AUTHORIZED=true
EXISTING_PTRACER_SHIM_AUTHORIZED_FOR_TASK_LAUNCH_ONLY=true
PRODUCT_MEMORY_WRITE_AUTHORIZED=false
AUTO_ADVANCE=G12_ATTACH_SUCCESS_TO_ROS_GRAPH_RECORDER_READY_TO_ONE_GOAL_TO_EXACT_CAPTURE_TO_REPLAY_TO_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

同一个 GPT-5.6 Luna Max 单代理连续执行；禁止子代理、委派和并行代理工作，不中途等确认。

## 0. G12结论和G13唯一变化

G12已经证明全部task identity和attach链：

```text
shared canonical __log:= parser fixtures       PASS
resolver unique task PID                       1119767
12 task ownership checks                       PASS
first/second raw cmdline bitwise                PASS
cmdline SHA-256                                 4cf3affe7245c3cce4851bbae8840a5f2334ce4eddc2bd9d083bf8eaca90d403
second identity all checks                      PASS
GDB selected inferior PID/starttime/exe/object  PASS
G12_GDB_STATE_MACHINE_ATTACHED                  PASS
external PID 627777/716149                      excluded/untouched
```

G12随后只因rosbag readiness脚本使用20秒内grep `Recording to`而退出。cleanup后
`rosbag.log`显示recorder实际已订阅全部目标topic并开始记录，最终bag大小约62 MB；日志是缓冲后才可见。
没有启动harness、没有发布goal、没有Tube terminal capture/replay。G12分类：

```text
G_G12_CANONICAL_IDENTITY_CAPTURE_INCOMPLETE
ROSBAG_START_FAILED
actual root cause=LOG_FLUSH_BASED_READINESS_FALSE_NEGATIVE
```

G13唯一变化：recorder readiness不再依赖stdout/stderr日志刷新；改用本次private ROS graph中的固定
recorder node、process liveness、expected subscriptions和`.bag.active`文件。其他identity、GDB capture、
goal、snapshot replay和A--G合同完全继承G12。

## 1. 冻结边界和写入白名单

完整读取：`AGENTS.md`、handoff顶部G12、G10--G12 completion、本spec和G12下列证据：

```text
/tmp/tube_v2_g12_capture_20260821_203231/identity_parser_preflight.tsv
/tmp/tube_v2_g12_capture_20260821_203231/task_ownership_proof.tsv
/tmp/tube_v2_g12_capture_20260821_203231/selected_pid_identity_second.tsv
/tmp/tube_v2_g12_capture_20260821_203231/gdb_attach_identity.tsv
/tmp/tube_v2_g12_capture_20260821_203231/ros_run/rosbag.log
/tmp/tube_v2_g12_capture_20260821_203231/ros_run/run_status.tsv
```

只允许写：

```text
docs/Codex_Tube_V2_G13_ROS_Graph_Recorder_Readiness_Exact_Capture_And_Replay_Spec_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
/tmp/tube_v2_g13_*
```

禁止修改/重编译产品source/header/test/launch/config/parameter/CMake/schema/binary/map。禁止修改G1--G12
文档。禁止任何Tube gate/cache/retry/timer/parameter/module/第二Validator、安全裕度削弱、depth/query
limit提高、product instrumentation/log和GDB memory write。禁止git destructive操作和触碰用户ROS。

## 2. G13-T0：identity和脚本冻结

建立fresh `/tmp/tube_v2_g13_capture_20260821_<id>/`。比较31个product hash、binary hash/build-id、
absolute launch、pillar map和现有ptracer shim hash；任何变化禁止ROS并分类G。不build、不重复G10
machine-map探索。

记录T0 numeric PID、external exact formation PID/starttime/log path、listeners、fresh private port。不得输出
完整environ。

复制G12已通过的shared parser/resolver/second identity/harness/GDB state machine到G13新目录并只替换
stage/path/marker。run前硬性静态验收：

```text
shared parser five fixtures PASS
all Python py_compile PASS
embedded GDB Python compile PASS
all output paths point to G13
no G10/G11/G12 marker/env/output path remains
DepthBP exact key signature PASS
terminal snapshot dump=True + identity gate PASS
G13_GDB_STATE_MACHINE_ATTACHED marker PASS
```

## 3. G13-T1：task ownership和attach

完全复用G12通过的合同：

- resolver polling使用attempt-specific directories；
- T0 PID排除；
- exact roslaunch descendant、binary/hash、argv0/name、canonical `__log:=` fresh path；
- exact private ROS_MASTER_URI/ROS_HOME/LD_PRELOAD；
- shim maps、UID、TracerPid=0；
- selected candidate恰好1个；
- first raw cmdline保存；
- second raw cmdline bitwise/SHA、PID/starttime/exe/env/maps/parent/log全部一致；
- GDB inferior PID/starttime/exe/object与proof一致。

只有全部PASS才可进入recorder。只对proved task formation PID做read-only attach。

## 4. G13-T2：ROS-graph recorder readiness——唯一编排修正

在private master上以固定node name启动targeted recorder：

```text
node name=/tube_v2_g13_recorder
output=/tmp/tube_v2_g13_capture_.../ros_run/evidence/g13_targeted.bag
```

topics保持：

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

recorder和检查命令显式`env -u LD_PRELOAD`。不得使用`rg Recording to`作为readiness gate；日志只作事后
证据。

recorder ready必须同时满足：

1. recorded BAG PID仍存活；
2. private master `rosnode ping -c 1 /tube_v2_g13_recorder`成功；
3. `rosnode info /tube_v2_g13_recorder`可读取并保存；
4. node info中至少明确包含上述10个expected subscriptions，不能只检查一个topic；
5. exact `g13_targeted.bag.active`存在；
6. active文件属于当前用户且位于G13 evidence目录；
7. private master/ROS_HOME环境仍与run一致。

输出：

```text
recorder_readiness.tsv
recorder_rosnode_info.txt
recorder_active_file.tsv
```

全部PASS才启动harness。若真实recorder process/node/active file不成立则分类G；不得因日志未flush误判。

## 5. G13-T3：900 map、one goal和exact capture

harness等待 `/sim/local_map>=900`，确认goal subscriber后exactly once发布 `(8,0,1)`，internal
`(8,0,2)`，原子写G13 `GOAL_PUBLISHED`。GDB只捕获post-goal Builder。

GDB machine map不变：

```text
ValidateCell+0x1338  xmm3=cover, xmm9=target, points=rsp+0x310 (9*32)
ValidateCell+0x63a   depth owner
query ABI            rdi=&result,rsi=&snapshot,rdx=&point,xmm0=radius
```

只处理depth12、两个span可切、非current-anchor、query budget可容纳9点的same-build structural terminal。
first query per Builder后禁用query breakpoint；compare/depth按
`thread/context/cell_ptr/w0bits/w1bits/v0bits/v1bits/build_serial` exact pair。

terminal必须在same-build direct snapshot上验证sequence/resolution/vector pointers/voxel product/count/hash，
dump exact occupied bytes，全部hard项PASS后才写`CAPTURE_COMPLETE`。不得用bag/marker近似terminal fields。

## 6. G13-T4：exact replay和分类

仅在capture完整后，于G13 `/tmp`创建standalone replay helper并链接冻结`libplan_env`：对9点用
required planner radius和captured request分别调用真实snapshot clearance query；另独立遍历dense
occupied bytes，以closed voxel AABB计算exact nearest distance/voxel/AABB。

输出逐点status/certified/capped distance、observed/grid ball containment、exact distance、planner/request
margin和production/bruteforce一致性。复算sampled formula：

```text
1.1*sampled_radius + 0.5*snapshot_resolution
```

分类优先级继承G12：

```text
F_QUERY_OR_SNAPSHOT_IDENTITY_MISMATCH_PROVEN
E_GEOMETRY_OR_REGULARITY_DEPTH_TERMINAL
C_OBSERVED_DOMAIN_OR_MAP_SUPPORT_INSUFFICIENT
A_TRUE_PLANNER_RESIDUAL_CLEARANCE_INSUFFICIENT
B_CONTINUOUS_COVER_REQUEST_CLEARANCE_INSUFFICIENT
D_ALL_NINE_REQUESTS_EXACTLY_SAFE_MAX_DEPTH_PROOF_ONLY
```

D仍要求9点request全部KNOWN_FREE/certified且exact D>=request、identity exact、owner仅max-depth、两个span
可切、非query-budget/unsplittable。只有D允许主代理后续极小implementation spec。

任一attach/recorder/goal/capture/snapshot/replay不完整：

```text
G_G13_ROS_GRAPH_RECORDER_CAPTURE_INCOMPLETE
```

## 7. cleanup、bag验收、测试和handoff

本spec只授权一次private run。只清理manifest task PID/proved descendants；不按全局名称/pattern杀进程。
cleanup后要求`.bag.active`正常finalize为`.bag`。若goal已发布，用`rosbag info`核对goal topic message count
exactly 1；保存topic/count摘要。若capture在goal后完成，不要求goal reached。

验证private port空、task PID空、external PID未attach/未signal。不build，运行focused tests
22/22、11/11、14/14、4/4；比较product/binary/launch/map/shim hash并运行`git diff --check`。

在本spec末尾追加completion并更新handoff顶部：primary/secondary、identity/attach、recorder readiness、
map/goal、terminal/snapshot/replay margins、bag counts、tests/hash/diff/cleanup，并写：

```text
NO_PRODUCT_IMPLEMENTATION_PERFORMED
```

Luna Max连续完成所有步骤，不中途停止；无论A--G，G13不改产品、不加门控、不跑第二次。

## 8. G13 completion — recorder graph ready, exact capture/replay classified B

```text
DOCUMENT_STATUS=COMPLETE
PRIMARY_CLASSIFICATION=B_CONTINUOUS_COVER_REQUEST_CLEARANCE_INSUFFICIENT
SECONDARY_CLASSIFICATION=ALL_NINE_KNOWN_FREE_AND_CERTIFIED_BUT_REQUEST_MARGIN_NEGATIVE
NO_PRODUCT_IMPLEMENTATION_PERFORMED
NO_SECOND_PRIVATE_RUN_AUTHORIZED
```

G13-T0 froze the unchanged 31 product files, formation/navigation/plan_env
binaries/build IDs, absolute launch/map, and ptracer source/shared-object.
The fresh run was `/tmp/tube_v2_g13_capture_204422/` on private port `13023`;
T0 external formation PIDs `627777` and `716149` remained excluded and
untouched.

The five shared canonical `__log:=` parser fixtures passed. Resolver attempt 2
proved formation PID `1123810` (roslaunch PID `1123740`, formation starttime
`29133419`) with all twelve task-ownership checks PASS. First and second raw
NUL-separated cmdline bytes were saved and bitwise/SHA equal
(`b17e37a32f4f58f4dcca5c82384a61e679b0e864ac2de929a6cea39c47269e43`); the
canonical log path, ROS environment, shim map, UID, parent chain, and
`TracerPid=0` all matched. The read-only GDB inferior/object gate passed with
`G13_GDB_STATE_MACHINE_ATTACHED`.

The fixed recorder node `/tube_v2_g13_recorder` passed the graph readiness
contract: recorder PID `1123909` alive, `rosnode ping` and `rosnode info`
successful, all ten expected subscriptions present, exact
`/tmp/tube_v2_g13_capture_204422/ros_run/evidence/g13_targeted.bag.active`
present under the current user and evidence directory, and private master/
ROS_HOME environment correct. The bag finalized to
`g13_targeted.bag` (276.3 MB, 10172 messages). Topic counts included
`/sim/local_map=914`, `/sim/odom=9140`, `/move_base_simple/goal=1`,
`/rosout=115`, and one message each on the two cloud diagnostic topics; the
goal publish sentinel records map count `900`, sequence `934`, and exactly one
publish of `(8,0,1)` (internal `(8,0,2)`).

The post-goal exact terminal captured at depth 12 is build serial 2, context
`0x7f40b66a03e0`, cell `0x7f40b669cd90`, with
`[w0,w1]=[0.050000000000000003,0.05001077002768612]`,
`[v0,v1]=[0,0.000244140625]`, cover
`0.050705847943061204`, target/required `0.40000000000000002`, and request
`0.45070684794306121`. The direct snapshot identity is sequence `911`,
resolution `0.10000000000000001`, voxel count `200,300,25`, occupied count
`1500000`, and occupied SHA-256
`a6bbdcca2b8ffadfbbaab361953e88af7edd19695121307b4100f345cdf2df53`.

Standalone production replay through the frozen `libplan_env` and independent
dense closed-AABB traversal agree for all 9 points. All 9 required-radius and
all 9 request-radius queries are `KNOWN_FREE` and clearance-certified. The
minimum exact planner margin is `+0.021131738630078112`; the minimum exact
request margin is `-0.029575109312983072`. Captured cover matches
`1.1*sampled_radius + 0.5*resolution` exactly (`sampled_radius=
0.00064167994823745398`), so this is the sampled nondegenerate route. Because
the planner residual is safe but the complete continuous request is not,
G13's priority classification is B, not D.

Task-owned PIDs (roscore `1123599`, roslaunch `1123740`, formation `1123810`,
GDB `1123861`, recorder `1123909`, harness `1123994`) were cleaned; the bag
active file finalized, port `13023` is free, and external processes were not
signalled or attached. Focused tests passed 22/22, 11/11, 14/14, and 4/4; all
product/binary/launch/map/shim hashes match T0 and `git diff --check` is clean.

Evidence:

```text
/tmp/tube_v2_g13_capture_204422/identity_parser_preflight.tsv
/tmp/tube_v2_g13_capture_204422/recorder_readiness.tsv
/tmp/tube_v2_g13_capture_204422/recorder_rosnode_info.txt
/tmp/tube_v2_g13_capture_204422/recorder_active_file.tsv
/tmp/tube_v2_g13_capture_204422/task_ownership_proof.tsv
/tmp/tube_v2_g13_capture_204422/selected_cmdline.bin
/tmp/tube_v2_g13_capture_204422/selected_cmdline_second.bin
/tmp/tube_v2_g13_capture_204422/gdb_attach_gate.tsv
/tmp/tube_v2_g13_capture_204422/terminal_identity.tsv
/tmp/tube_v2_g13_capture_204422/terminal_snapshot_occupied.bin
/tmp/tube_v2_g13_capture_204422/replay_points.tsv
/tmp/tube_v2_g13_capture_204422/replay_summary.tsv
/tmp/tube_v2_g13_capture_204422/classification.tsv
/tmp/tube_v2_g13_capture_204422/ros_run/bag_info.txt
/tmp/tube_v2_g13_capture_204422/ros_run/bag_topic_counts.tsv
/tmp/tube_v2_g13_capture_204422/ros_run/cleanup_external_check.txt
```
