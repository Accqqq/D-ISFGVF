# A6/H2 跨目标新任务 neutral rebase 执行规范

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DOCUMENT_STATUS=AUTHORIZED_BY_USER_2026-08-19
STAGE=A6_H2_CROSS_GOAL_NEUTRAL_REBASE
IMPLEMENTATION_AUTHORIZED=true
EXECUTOR=Terra_xhigh
AUTO_ADVANCE=false
A7_NOT_AUTHORIZED=true
```

日期：2026-08-19  
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. 目标

恢复可信单机导航基线 `main@9a0e975` 的已到达跨目标语义：目标 A 已声明
`POINT_GOAL REACHED`、FSM 已进入 `WAIT_TARGET` 后，用户发送目标 B，planner 从当前
odom 状态开始一项新的导航任务；它不与上一任务做跨任务 C2，也不继承上一任务路径
坐标系中的 phase offset。用户已明确裁定：`EXEC_TRAJ` 中途直接改目标不在本阶段范围内。

修复以下已动态复现的永久阻塞：

```text
goal A: POINT_GOAL REACHED
goal B: KinoA* success + BSPLINE_PARAM success
        -> initial finite frontend failed
        -> persistent GOVERNOR_INVALID_HOLD
```

固定诊断是：goal reset 先 retire 旧 `PathTubePair`，但 Runtime 故意保留旧任务的
`retained_delta`、previous final port 和未完成 profile；随后 null-pair bootstrap 正确拒绝把
已执行的非零 authority 绑定到新 planner owner。安全拒绝本身正确，错误在于“新导航任务”
复用了“同任务 authority retirement”语义。

本阶段只新增一个明确的新任务 neutral rebase。完成后：

```text
新目标到来
  -> 旧任务 authority/session 失效
  -> Runtime 以新任务语义回到 delta=0 neutral
  -> planner 从当前 odom 构建并安装新路径
  -> Tube 保持 sidecar；新路径的 Pair 完整证明后才重新允许非零 offset
```

## 2. 不可混用的两类生命周期

### 2.1 同一个目标内的 replan/handoff

保持现有 A6/H2 合同不变：

```text
old exact PathTubePair pin
future certified seam
existing base-path C2 composite owner
new-owner Tube/profile
Runtime exact-PWL dry-run
generation/session/runtime-bit final CAS
completed frontend mailbox
```

不得 reset `delta`，不得 path-only 切换，不能复用旧 Tube 证明新 C2 几何。

### 2.2 用户发送新目标

这里专指上一点目标已经到达、FSM 处于 `WAIT_TARGET` 后发送的新目标。这是任务边界，
不是同任务 replacement：

- 新 planner 起点来自当前实际 odom 状态；
- 新路径的 `delta=0` 表示以该新路径为新坐标基准，不要求飞机回到旧路径中心线；
- 旧路径中的 `delta`、profile elapsed、previous port 和 preflight revision 不得跨任务继承；
- 旧 pair/session、pending/completed handoff、deferred tail、timer/control snapshots 必须一并退休；
- 新任务重新执行 zero-port warm-up；没有新 matching Pair 时只能运行 planner 的 neutral
  `delta=0` 路径，不能发出非零 port。

新任务 rebase 是控制坐标和 authority 生命周期重置，不是把一个未经证明的 `{0}` 区间写进
Tube，也不是把飞行器物理位置瞬移到旧中心线。

## 3. 实施要求

### A. Runtime 的任务级 neutral reset

在 `PhaseOffsetRuntime` 增加一个窄接口，例如：

```cpp
void resetForNewNavigationTask();
```

它只重置任务级值状态：

```text
delta = 0
previous_final_port = zero/default
profile_elapsed = 0
profile_started = false
profile_completed = false
returning_to_center = false
preflight and preflight revision = empty
```

配置与构造时 configuration validity 保持不变。普通 H2 replacement、timer refresh、
certificate denial、fatal handling 不得调用该接口。

### B. Adapter 的原子新任务 reset

增加与 `retirePathTubeAuthority()` 明确不同的接口，例如：

```cpp
bool resetForNewNavigationTask(expected_session, retired_session);
```

在既有 `runtime_command_mutex_` 边界内完成：

1. 验证 expected authority session，拒绝 stale reset；
2. 使 pin registry 中旧 lease/identity 失效；
3. 增加 authority session；
4. 清空 authoritative pair、build/candidate/active/control snapshots 和旧 source/preflight
   bookkeeping；
5. 调用 Runtime task reset；
6. 关闭并清零 zero-port gate/warm-up 计数，使新任务重新 warm-up；
7. 不把旧 Pair、旧 retained delta 或旧 previous port映射到新 owner。

不得由这个接口清除 genuine fatal adapter/configuration failure。若现有 `failure_latched_`
混合了可重试任务失败和 genuine fatal，先在自审中记录其精确语义；本阶段不得用新目标作为
通用故障清除按钮。

### C. Manager 只在新目标入口使用任务 reset

拆分当前含混的 reset 路径：

```text
goalCallback/new-navigation-task
    -> task-level neutral reset

same-goal H2/replan/current-state denial
    -> existing fail-closed retirement/handoff semantics
```

Manager 必须继续保持固定锁序：

```text
frontend_apply -> path_tube_handoff -> authoritative_phase -> Adapter Runtime
```

新目标 reset 在一个 manager ownership transaction 中：

- 取消 pending/completed H2 frontend 和 deferred tail；
- 调用 Adapter task reset 并同步 manager session；
- invalidates authoritative phase；
- 通过现有 FSM clear mailbox 清理旧 planner/GVF mirror；
- goal callback 只提交新任务目标和 reset 请求，不在 command callback 中安装新 frontend。

随后沿原导航流程进入 `GEN_NEW_TRAJ`，从当前 odom 运行 `astaropt()`。初始 PointPhaseV2
frontend 在 Runtime neutral 且 gate 已关闭时必须能以 planner-only owner 安装；新 Tube/Pair
随后通过现有 activation boundary 建立。

### D. 保持原导航行为

- 规划失败仍在 `GEN_NEW_TRAJ` fail-closed 重试；
- 允许新目标刚收到后的短暂初始化 HOLD，但不得形成永久 HOLD；
- 不要求旧路径提供 future seam；不沿旧 Tube 物理回中；
- 不改变 goal tolerance、planner、KinoA*、B-spline、C2、Governor 或 SO3 算法；
- `observe_only`、manual amplitude `0` 和 phase-offset disabled 的原行为保持。
- `EXEC_TRAJ` 中途 retarget 不在本阶段验收范围内；本阶段既不为它新增 FSM 跳转，也不
  声称已经验证该行为。

### E. 诊断必须准确

不要再把所有初始化失败统一报告成 `initial finite frontend failed`。至少在内部/日志中区分：

```text
planner/bspline candidate failure
mapped ContinuousPhasePath failure
new-task authority reset/session failure
neutral frontend install failure
optional Tube/Pair bootstrap failure
```

不得为此新增 ROS message/topic/schema 或参数；使用现有日志即可。

## 4. 白名单

仅允许修改：

```text
docs/Codex_A6_H2_Cross_Goal_Neutral_Rebase_Execution_Spec_2026-08-19.md
docs/Codex_A6_H2_Cross_Goal_Neutral_Rebase_Self_Audit_2026-08-19.md

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
```

如实现需要修改 CMake、launch/config、message、TubeBuilder、TubeEpochManager、Filter、
SurfaceValidator、ContinuousPhasePath、planner、map、速度/rate/lookahead/tracking/margin 或
AGENTS，停止并报告，不扩展范围。

## 5. 明确禁止

- 不降低或绕过 `min_certified_forward_w`；
- 不制造 `{0}` Tube，不把 planner safety 伪装成 Tube certificate；
- 不允许已执行非零 authority 走普通 null-pair bootstrap；
- 不在普通同目标 H2 handoff 中清零 Runtime；
- 不复用旧 Pair/profile/snapshot 到新任务 owner；
- 不通过重启节点、amplitude=0、observe-only 或参数调小声称修复；
- 不改变 Tube geometry、Filter、clearance accounting 或 planner 参数；
- 不进入 A7、swarm intent 或新 recovery controller。

## 6. 必须新增的测试

### Runtime

1. 已启动 profile 且 `delta!=0` 后 task reset 回到 neutral；
2. `hasExecutedOffsetAuthority()==false`，previous port/profile/preflight 不跨任务；
3. reset 幂等且不改变配置有效性；
4. 普通 Runtime/H2 路径不会隐式调用 task reset。

### Adapter

1. exact session 的 task reset 原子退休旧 Pair/lease/snapshots 并 neutralize Runtime；
2. stale session reset 零修改；
3. reset 后 gate 关闭并重新 warm-up；
4. reset 后可先安装 planner-only neutral owner，未建 Pair 前不能选择非零 port；
5. ordinary `retirePathTubeAuthority()` 仍保留已执行 Runtime state 并 fail-closed；
6. pending/active H2 transaction 在 task reset 后不能 final CAS；
7. failure/fatal latch 不被新任务错误清除。

### Manager

1. goal A 的非零 authority 不阻塞 goal B 初始 planner frontend；
2. new owner 与 old Pair 永不同时可见；
3. new goal reset 后旧 completed/pending/deferred mailbox 不可消费；
4. 同一目标内非零 H2 replacement 仍走原 future-seam/pin/CAS；
5. phase-offset disabled/observe-only/neutral baseline 回归；
6. 初始化失败日志可以区分 planner 与 authority 层。

## 7. 动态验收

使用 task-owned ROS master/ROS_HOME，不接入或终止用户进程。保持当前验收参数与原
`pillar.pcd`，不得调参。

必须在同一次 `formation_planning` 进程中完成：

```text
launch once
goal A = (8, 0, 1) 或等价可达点
observe POINT_GOAL REACHED
不重启任何节点
goal B = 反向可达点
observe POINT_PHASE_V2 INIT / planner frontend installed
observe goal B POINT_GOAL REACHED
```

验收条件：

- 两个目标均到达；
- 第二目标不持续 `GOVERNOR_INVALID_HOLD`；
- 第二目标 Kino/B-spline 成功后不循环 `initial finite frontend failed`；
- 新任务非零 offset 只能在新 owner 的 fresh Pair 安装后启动；
- 没有 new path + old Pair、stale session commit 或跨任务 old delta；
- 记录 bag、rosout、参数、二进制/hash 和进程清理证据。

建议再增加一组 A→B→A 三目标重复验收，证明 reset 幂等且无一次性偶然通过。

## 8. 构建、自审和停止

至少运行：

```text
phase_offset_runtime_test
phase_offset_matched_adapter_test
gvf_switch_policy_test
现有 H2/P1/Tube focused regressions
full workspace build/test
git diff --check
白名单与 dependency-boundary audit
```

自审必须记录修改前后 `git status --short`，说明 dirty worktree 中哪些文件为既有用户改动，
并只归属本阶段实际 diff。

完成实现、测试、动态验收、Terra 自审后停止；不得自动进入 A7。
