# Luna Max Tube V2 T0--G1 New-Conversation Handoff

```text
DOCUMENT_ROLE=NEW_CONVERSATION_EXECUTOR_HANDOFF
DATE=2026-08-20
WORKSPACE=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
REQUESTED_EXECUTOR=LUNA_MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
```

## 1. 新对话的唯一任务

让 **Luna max 单代理**完整执行：

`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_Tube_V2_T0_G1_Local_PWL_Geometry_Execution_Spec_2026-08-20.md`

执行顺序固定：

```text
完整读取 execution spec
-> T0 当前代码/动态 before baseline
-> T0 gate 审核
-> G1 cell-local conservative PWL Filter
-> 定向测试与全回归
-> 隔离 after launch
-> self-audit + handoff
-> 停止
```

Luna max 必须自己完成全部工作，**不得创建、调用、委派或请求任何子代理**，也不得将
任何子任务交给其他模型。

## 2. 当前完成状态

上一阶段 A6-P2 已完成，不要重做，也不要回滚：

```text
same-owner Pair COMMITTED
-> command captures pending activation Pair
-> Runtime selected/executed
-> retained delta becomes significantly nonzero
-> accepted replan uses H2 branch
```

最终验证：

```text
gvf_switch_policy_test             48/48 PASS
phase_offset_matched_adapter_test  82/82 PASS
required CTest set                 10/10 PASS
catkin_make -j2                    PASS
git diff --check                   PASS
```

A6-P2 Run 4 证据：

`/tmp/a6_p2_20260820_acceptance_4/`

关键动态顺序：

```text
Pair 1 COMMITTED, generation=1, session=3
-> pending activation capture
-> selected/runtime executed
-> retained delta 0 -> -0.000384455
-> transient certificate denial, fatal=0
   U+ and U_safe step-4 joint-port polygon empty
-> Pair 2 retained delta -0.005596105 -> -0.007772646
-> H2 branch attempted, stage_success=0
-> POINT_GOAL REACHED, distance=0.196
```

Run 4 的 raw current anchors 为 `34/34 found+complete`。这证明当前系统中 geometry evidence
与短时 dynamic viability denial 可以同时存在；G1 不得把 Runtime denial 当成 geometry
empty。

A6-P2 自审：

`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_A6_P2_Initial_Offset_Bootstrap_First_False_And_Liveness_Self_Audit_2026-08-20.md`

总 handoff：

`/home/cxq/ISF-GVF/handoff.md`

## 3. 为什么现在做 Tube V2 G1

基础架构已经完成：

- planner/Tube 共用 `planning/safe_distance`；
- margin accounting 已避免重复扣除；
- normal Tube 保持 zero-connected/zero-only planner baseline；
- production candidate 统一经 `CertifiedTubeBuilder`；
- Runtime 已有 exact-PWL crossed-knot witness、`U+ -> U_safe`；
- Pair bootstrap、selected/nonzero、pending activation/H2 protection 已恢复。

当前主要过度保守点仍在 `TubeFilter`：

```text
fixed boundary_slope_max=0.80
-> global L-Lipschitz forward/backward envelope
-> anchored range enumeration
-> distant bottleneck may contract earlier preview
-> slope-induced REGULARITY truncation
```

G1 的唯一核心修改是把它变成：

```text
filtered knot bounds = raw knot bounds
local PWL interpolation
one-sided local derivatives
continuous safety still proved by existing SurfaceValidator
dynamic reachability still decided by existing Runtime
```

## 4. 必须首先读取的文件

Luna max 开始操作前必须完整读取：

1. 主执行规格：
   `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_Tube_V2_T0_G1_Local_PWL_Geometry_Execution_Spec_2026-08-20.md`
2. proposal 对齐审查：
   `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_Tube_V2_Proposal_Alignment_And_Minimal_Next_Stage_Plan_2026-08-20.md`
3. 最新 proposal：
   `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Latest_Detailed_Proposal_2026-08-20.md`
   重点读取 §11--§14、§37，但不得修改 proposal。
4. A6-P2 self-audit 与 `/home/cxq/ISF-GVF/handoff.md` 顶部。
5. 当前源码：
   - `tube_types.h`
   - `tube_filter.h`
   - `tube_filter.cpp`
   - `tube_surface_validator.cpp`
   - `certified_tube_builder.cpp`
   - `tube_epoch_manager.cpp`
   - `phase_offset_runtime.cpp`

## 5. 强制工作区规则

工作区高度脏，已有 tracked/untracked 改动都是用户资产。

禁止：

```text
git reset
git clean
git checkout --
git restore
git stash / stash pop
rebase
覆盖或删除不相关文件
```

不得假设 untracked adapter/A6 文件可以删除。不得为了得到干净 diff 回退已有工作。

T0 evidence 只能写入新的 `/tmp/tube_v2_t0_*` 目录。每次 launch 必须使用新 private ROS
master、新 `ROS_HOME`、新 evidence directory，只清理该 run 的 PID/PGID。

## 6. 白名单摘要

完整白名单以主 execution spec 为准。优先只修改：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
```

必要时只允许修改执行规格列出的 Validator/Certified/Epoch test 文件以及 self-audit/handoff。

严禁修改：

```text
TubeBuilder production
TubeSurfaceValidator production
CertifiedTubeBuilder production
TubeEpochManager production
PhaseOffsetRuntime / PortProjector / core production
bspline_traj source (manager/adapter/H2/A6)
planner / map / simulator
launch / parameters / CMake / package
ROS messages/topics/schema
proposal
```

若实现需要白名单外修改，停止并报告，不得自行扩权。

## 7. G1 不可误解的算法合同

对于严格递增且完整有效的 raw knots：

```text
filtered_lower[k] = raw_lower[k]
filtered_upper[k] = raw_upper[k]
```

cell slope：

```text
lower_w[k] = (lower[k+1]-lower[k]) / (w[k+1]-w[k])
upper_w[k] = (upper[k+1]-upper[k]) / (w[k+1]-w[k])
```

interior knot query 返回右单侧导数；末端返回左单侧导数。PWL corner 合法。

必须删除 production Filter 中的：

- global Lipschitz forward/backward envelope；
- anchored range enumeration；
- slope cap assertion；
- slope-induced truncation；
- Filter 生成的 `REGULARITY` reason。

`boundary_slope_max` 字段和 ROS 参数暂时保留兼容，但它不得再决定 Filter geometry、
configuration validity、success、preview、bounds、slopes 或 classification。

G1 不是放宽连续安全：所有 ESDF nonzero ribbon 仍须通过现有 SurfaceValidator。

## 8. T0 与 G1 动态验收

T0 和 after 都使用：

```text
phase_offset_esdf_tube_single.launch
pillar.pcd
goal=(8,0,1), internal=(8,0,2)
phase_offset_manual_observe_only:=false
其他参数不变
```

T0 先冻结当前 Filter/Validator/candidate facts。历史 `45/46` 只能当历史动机，不能写成
当前 baseline。

after 至少证明：

- goal 到达；
- raw current anchors 完整；
- Pair selected 且 retained delta 显著非零；
- fixed slope `0.80` 不再导致 Filter truncation；
- 真实 truncation 只能归属 Builder 或 Validator；
- unsafe/unknown/Validator failure 仍 fail-closed；
- Runtime dynamic denial 可存在，但 `fatal=0` 且不计 geometry empty；
- H2/A6 无退化；
- 无 `all_candidates_path_end_clamped`；
- run processes 和 port 完全清理。

## 9. 停止边界

完成 T0、G1、全部测试/全量构建、一次 after launch、自审与 handoff 后停止。

不得继续：

- backward viability envelope；
- Runtime/QP/PortProjector；
- disconnected/current-delta component recovery；
- H2 `stage_success=0` 修复；
- recenter/M4/M6；
- planner/参数调优；
- 多机或论文实验。

## 10. 可直接复制给新对话 Luna max 的启动指令

```text
工作区是 /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws。

你是本阶段唯一执行代理 Luna max。禁止创建、调用、委派或请求任何子代理，禁止把任务
交给其他模型；所有分析、实现、测试和 launch 都由你单代理完成。

先完整读取：
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_Luna_Max_Tube_V2_T0_G1_New_Conversation_Handoff_2026-08-20.md
以及其中指定的主执行规格：
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_Tube_V2_T0_G1_Local_PWL_Geometry_Execution_Spec_2026-08-20.md

严格按规格连续执行 T0 -> G1 -> tests/build -> isolated after launch -> self-audit/handoff，
不要停止等待我确认；只有命中规格中的真实停止条件或白名单外需求时才报告阻塞。

工作区高度脏，保护所有已有改动；禁止 reset/clean/checkout/restore/stash/rebase。
严格白名单，不得修改 Runtime、Builder/Validator/Epoch production、bspline_traj、planner、
launch、参数、ROS schema 或 proposal。

完成后停止在 G1，不进入 backward viability 或其他后续阶段。
```

