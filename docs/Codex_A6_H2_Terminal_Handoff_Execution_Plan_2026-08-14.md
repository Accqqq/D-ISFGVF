# Codex A6-H2：末端 handoff 调度修正执行单

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=DIAGNOSIS_IN_PROGRESS
STAGE=A6_H2_TERMINAL_HANDOFF_SCHEDULING
AUTO_ADVANCE=false
```

## 1. 已证实事实与根因边界

用户运行的停机日志给出：`phase_w=5.704`、旧 path `end_w=5.706`；governor 的
36 个前视候选全部 `path_end_clamped`，故保持当前位置。与此同时 KINO 成功、switch
判据接受，但 H2 没有安装 replacement：`POINT_PHASE_V2 replan not installed`。

这不是 tube 几何、Filter、SurfaceValidator、margin、slope、lookahead、速度或
Runtime exact-PWL 的失败证据。此时旧 pair 已没有一个同时满足以下条件的 H2 seam：

1. 来自旧 pair 的 immutable certified sample；
2. `seam_w >= captured_w0 + existing min_certified_forward_w`；
3. 提交时严格仍有 `current_w < seam_w`。

因此末端 fail-closed HOLD 是正确行为，但它本身不能证明缺少 replan trigger。

静态追踪还确认：

- `shouldAcceptCandidate()` 已复用 `shouldForceAcceptForGovernorPathShort()`；在
  `remaining_w <= cmd_governor_l_max + switch_governor_path_margin_w` 时，候选确实
  被强制接受。但是这个调用位于 `REPLAN_TRAJ` 内，意味着它只能决定**已经发生的**
  replan 是否接受 candidate。`EXEC_TRAJ` 原先只会因 collision 或 `planInterval`
  进入 `REPLAN_TRAJ`，所以没有把相同的 existing path-short 事实用作提前调度信号。
  这只是静态调用位置，不能单独推出真实失败原因。
- H2 的 `stageFutureSeamPathTubeTransaction()` 仅在 candidate 已获接受后执行；它先
  pin exact old pair，随后依次完成 immutable seam、C2、new tube、runtime dry-run，并把
  成功结果放入现有的唯一 `pending_path_tube_handoff_` slot。
- 唯一允许完成 pending 的路径是既有 command-boundary
  `prepareAndCommitPendingPathTubeHandoff()`；它随后通过现有 completed mailbox 交给 FSM。
  不能在 FSM、timer 或一个新 callback 内另行 commit。
- attachment 只记录了不可恢复的终端时刻，不能反推出更早一次 H2 attempt 的失败层；旧
  `stageFuture...` 的 bool-only 返回没有把 rejected seam / C2 / prepared tube / runtime
  / pending slot 分开留存。不得为此新增 product diagnostic schema。

既有 command-boundary `prepareAndCommitPendingPathTubeHandoff()` 已是唯一 commit 路径，
且已有 H2 动态验收已证明它可在 50 Hz command 节奏完成 exact commit；本单不得改动
CAS/mailbox/pin。下一步是通过 task-owned private GDB 将 first false return 固定在
`certifiedFutureSeamCandidates()`、C2、`stagePathTubePair()`、prepared Runtime 或 manager
transaction slot 的其中一层。不得为此新增 product diagnostics、重试或 gate。

## 2. 白名单与禁止项

仅允许改动：

```text
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
docs/Codex_A6_H2_Terminal_Handoff_Execution_Plan_2026-08-14.md
```

允许：只读 GDB/私有日志分层定位；若定位证据支持，才在本白名单内使用现有
`pending_path_tube_handoff_` / completed mailbox、exact pin/CAS 和既有
`min_certified_forward_w` 作最小修复与 focused regression。

禁止：新增 product gate/mode/state/reason/enum/diagnostic schema/ROS parameter；改动 tube
geometry、TubeFilter、SurfaceValidator、margin、slope、lookahead、speed、rate、tracking
bound、AGENTS.md；放宽 `current_w < future_seam_w`、exact pair/session/runtime-bit CAS，或在
new C2 connector 上复用 old tube/profile/sample 几何。

## 3. 最小实施假设与停止条件

只读诊断必须先回答：在旧 horizon 尚可用的第一个 replan 中，到底是 future candidates、
C2、prepared tube、Runtime dry-run、exact pin/CAS，还是 transaction slot 拒绝。若答案是
证书/几何/Runtime 本身的 fail-closed 拒绝，则不得改调度；若要继续飞，需要明确的
emergency/replan bridge 或更上游 path/tube availability 架构，超出本单。

## 4. 验收

1. first false return 的动态证据必须明确唯一失败层；
2. terminal / insufficient-horizon 仍必须拒绝，不产生 unsafe install；
3. 每个尝试保持 exact old-pair pin、session/runtime-bit final CAS；
4. 只有诊断定位支持的补丁才新增 focused regression 并运行既有 H2 focused suites、
   `git diff --check` 与范围审计；
5. 如运行动态验收，只使用本任务独立 ROS master/ROS_HOME，且不接入、终止或影响用户
   进程 `1746447`、`1746448`、`1746461`。

## 5. 当前状态

`DIAGNOSIS_IN_PROGRESS`：执行过的 early-scheduling 接线已经撤回，不能保留。

### 5.1 已撤回的 early-scheduling 尝试（2026-08-14）

task-owned private master `11561` 的运行目录：

```text
/tmp/a6_h2_terminal_private_20260814_9TnXAv
```

其关键时间线反驳了“缺少提前 trigger”假设：

```text
1786696391.284  plan_interval, remaining_w=1.824
1786696391.305  existing REPLAN accepted near_end, remaining_w=1.786
1786696391.305–1.512  C2 connector success for seams 4.028 ... 4.426
1786696391.444 onward  experimental early wiring repeatedly entered REPLAN
1786696392.084  remaining_w=0.391; still no install
```

实验 wiring 使用既有 `governor_path_short` 直接从 `EXEC_TRAJ` 进入 `REPLAN_TRAJ`，但在
stage failure 时会每个 FSM tick 重跑 KINO（约 50 Hz），并没有产生 H2 install。这不是可接受
的修复，已用 `apply_patch` 精确撤回 helper、EXEC 接线和两个测试；没有保留新 gate、状态、
参数或 reason。

结论：普通 periodic replan 已在 certificate 完全耗尽前执行过，且 C2 已成功。first false
return 位于 C2 之后，必须继续定位 `stagePathTubePair()` / prepared tube / Runtime / slot，
而不是修改 replan 调度或放宽 H2 合同。
