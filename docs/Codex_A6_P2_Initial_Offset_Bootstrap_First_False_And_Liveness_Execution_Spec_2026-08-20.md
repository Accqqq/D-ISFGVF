# A6-P2 Initial Offset Bootstrap First-False and Liveness

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、调度、独立审查和最终动态验收；
`gpt-5.6-terra / xhigh` 负责白名单内实现、首轮测试和隔离 launch。  
前置条件：A6-P1R 已通过并停止。  
`AUTO_ADVANCE=true`，但只允许本文件的“结果可判定化 -> 单次诊断 launch ->
命中层最小修正 -> 同配置复验”。不得进入 M6 recenter、M4 authority 重构、M7
pin/mailbox 删除或多机阶段。

## 1. 已知事实

A6-P1 active 隔离 capture 已证明：

- 37/37 raw candidate 完整且有 current anchor；
- 23 个 active profile/current validation/admissible 事实有效；
- warmup gate 后期开启；
- 但 manual/epoch 中 `control_selected=0`、`delta=0` 全程；
- 没有 PathTubePair bootstrap/stage/dry-run/CAS 或 H2 mailbox 事件。

因此当前飞机是 neutral-only 到达。下一 first-false 位于：

```text
phaseOffsetTubeTimerCallback
  -> activatePendingOffsetAuthority
  -> prepareBootstrapPathTubeTransaction
  -> PhaseOffsetMatchedAdapter::stagePathTubePair
  -> live-phase/latest-map prepare
  -> final CAS
```

现有两个 `first_false` lambda 丢弃原因。已有 manager bootstrap 单测主要使用
`TubeSource::FIXED`；实际失败是 `ESDF` active，不能用 FIXED 成功代替动态证据。

## 2. 阶段目标

先把一次 initial bootstrap 尝试归约为唯一、内部、可测试的结果，然后用一次隔离
active launch 找到真实 first-false。只修该层的确定性 liveness 缺陷，并用同一个
launch 证明：

```text
same planner owner + certified Tube -> committed PathTubePair
-> Runtime selected -> delta != 0
-> later accepted replan uses H2 single-seam retry-safe handoff
```

本阶段不重新设计 Tube，也不把 bootstrap 成功变成新的导航 gate。

## 3. 内部结果合同

在 manager 内新增一个非 ROS schema 的结果值，例如
`OffsetBootstrapAttemptResult`，至少可区分：

```text
NOT_REQUIRED
ENTRY_OR_SLOT_PRECONDITION
OWNER_OR_SAMPLE
STRUCTURAL_SEAM
STAGE_<existing PathTubePairStageFailure>
LIVE_PHASE_STATE
LIVE_PHASE_WINDOW
POST_STAGE_OWNER_OR_SESSION
LIVE_PREPARE_OR_LATEST_MAP
FINAL_CAS
COMMITTED
```

允许用一个小 struct 携带：

```text
outer result
existing PathTubePairStageFailure
captured_w0 / live_wc / future_seam_w
authority_session
map observation sequence
```

要求：

1. 它只描述本次函数调用结果，不是 authority、gate、latch 或 mailbox；
2. `activatePendingOffsetAuthority()` 和
   `prepareBootstrapPathTubeTransaction()` 的现有 bool 成功语义不变；
3. adapter 已有 `PathTubePairStageFailure` 必须完整上传，禁止再压成
   `stage_internal_emitted` 或无信息字符串；
4. production timer 只在真正进入 bootstrap 尝试时记录：
   - failure：同一结果节流日志；
   - success：一次 `COMMITTED` 日志，带 pair generation/session/owner；
5. `NOT_REQUIRED` 不刷日志；
6. 不新增 topic、msg、diagnostic array 字段、参数、文件输出或线程；
7. 不把临时结果写入 Runtime/TubeEpoch/Pair authority。

日志必须足以离线区分上述结果，但不得打印指针地址作为唯一身份事实。

## 4. 诊断后最小修正规则

先完成结果枚举和 deterministic tests，再启动一次固定 active launch。只有动态证据
命中的以下层允许修正：

### 4.1 owner/sample 或 structural seam

- 修正 bootstrap 对 immutable planner owner 的 sampling/seam 选择；
- 不构造 C2，不改变 planner path，不枚举多个 seam；
- 每次 timer bootstrap 最多一个 seam、一次 stage；
- seam 必须有足够 construction lead，并保持 `captured_w0 < live_wc < seam` 的
  commit 窗口；不得降低 `min_certified_forward_w`。

### 4.2 stage Tube / coverage / validator

- 只修同 owner、同 frozen snapshot、同 M3 unified builder 的输入/范围 plumbing；
- 不放宽 zero-connected、clearance、Filter、Validator 或 current validation；
- 不新增第二套 Builder，也不得复用 provenance 不匹配的普通 sidecar epoch。

### 4.3 live phase / latest-map prepare

- 允许按 live `wc` 重新执行现有短 prepare/dry-run；
- frozen snapshot 继续证明构建 provenance；latest snapshot 只执行既有显式
  OCCUPIED/OUT_OF_MAP 否决；
- 若根因是 build 后 commit window 确定性过短，只能扩大本次 prepared horizon/seam
  lead 到由现有 `lookahead_w`/`min_certified_forward_w` 推导的范围；不得延时 command、
  降速、调 timer 或降低安全阈值；
- 不允许在 timer callback 中重跑无界循环或多个 candidate。

### 4.4 final CAS

- 只修被证据证明的过时比较或错误 session/phase capture；
- 最终仍必须在短锁内验证 live owner/session/phase/Runtime bits 后做一次 CAS；
- 不允许 unconditional store，不允许把新 Path 与旧/null Tube 分开发布。

### 4.5 committed-before-first-command activation protection

Run 1 已证明 bootstrap 可以 `COMMITTED`，但现有 replan 分支只用
`requiresAuthoritativeOffsetHandoff()` 判定 H2。该 predicate 只在 Runtime 已经
executed offset authority 后为 true，因此漏掉了一个真实生命周期状态：

```text
certified same-owner Pair committed
-> pending first command activation
-> Runtime has not executed offset authority yet
```

该状态已可由 `hasPendingOffsetActivationPair(captured_pair)` 识别。本阶段允许的
最小修正为：

- point/closed C2 replan 各自只 capture 一次 Pair；
- `executed authority || matching pending activation Pair` 任一成立时，都必须走
  已有 H2 single-seam transaction，不得走 neutral direct install；
- `retirePathTubeAuthorityIfNeutral()` 必须在现有 `runtime_command_mutex_` 内同时
  拒绝 matching pending activation Pair，防止外层 check 与 timer final CAS 之间的
  check-then-retire race；
- 不新增 authority/gate/mailbox，不改 Runtime 选择逻辑，不改 Tube 或 planner；
- 修正后的第一条 command 仍必须通过现有 Runtime prepare/complete 进入
  `selected`，不得由 replan/FSM 预先标记 executed authority。
- `RuntimeStepOutput::delta` 是该 command 的 pre-step retained value；selected 后真正
  提交到 Runtime 的值是 `projection.next_delta`。若第一条 selected command
  正好位于 profile 起点且 pre-step `delta=0`，允许记录一条按 Pair
  generation/session 去重的
  `selected && projection.valid && abs(projection.next_delta) > 1e-6`
  一次性日志，同时输出 `retained_delta_before` 和
  `retained_delta_after=projection.next_delta`。它只是验收证据，不得成为
  profile gate/latch/authority 或改变 command 输出。

Run 1 现有 bag 只证明 Pair commit 早于后续 command/replan，不能单凭
`replan not installed` 声称该次 replan 已实际退休 Pair。修正的依据是可达的
静态生命周期缺口，Run 2 必须用一次性内部日志/测试证明真实分支。

若 first-false 位于白名单外，或修复需要更改 Runtime/Tube 算法、planner、C2、launch
参数、ROS schema 或线程，立即停止并报告；不得猜测性扩展。

## 5. 必须冻结

不得修改：

- planner safe distance、Kino A*、B-spline、C2 接受条件和 replan 触发；
- TubeBuilder/CertifiedTubeBuilder/Filter/SurfaceValidator/EpochManager/Runtime 算法；
- zero-connected/zero-only 合同和 margin 数值；
- warmup、manual amplitude/profile、速度/饱和、timer period；
- H2-L1 single-seam、pending/completed mailbox、pin lease、consume-after-mirror 语义；
- recenter、physical recovery、swarm、CBF；
- launch 文件、ROS messages/topics/schema、参数或线程。

不得通过关闭 active、固定 delta=0、observe-only、延迟发 goal、减速或改地图制造通过。

## 6. 文件白名单

允许修改：

```text
docs/Codex_A6_P2_Initial_Offset_Bootstrap_First_False_And_Liveness_Execution_Spec_2026-08-20.md
docs/Codex_A6_P2_Initial_Offset_Bootstrap_First_False_And_Liveness_Self_Audit_2026-08-20.md

src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

/home/cxq/ISF-GVF/handoff.md
```

adapter 三个文件只有在 first-false 命中 stage/prepare 内层时才允许修改；若只需
manager 结果传递或 seam/horizon 修正，不得顺手改 adapter。

全部其他文件只读，包括：

```text
src/swarm_planner/phase_offset/**
src/swarm_planner/plan_env/**
src/swarm_planner/bspline_traj/launch/**
src/swarm_planner/bspline_traj/CMakeLists.txt
Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md
```

## 7. 定向测试

### Result attribution

对每个 outer result 至少有 deterministic assertion；stage 层可用 table-driven 映射
验证所有现有 `PathTubePairStageFailure` 不丢失。必须覆盖：

- not required 与真实 attempt 区分；
- owner/sample、seam、stage、live window、post-stage recheck、prepare、final CAS；
- committed 返回 pair 且 owner/session/generation 一致；
- failure 不发布 pair、不改变 Runtime retained delta/previous port；
- repeated transient attempt 不产生 pending/completed H2 mailbox。

### ESDF bootstrap

新增与 active 参数口径一致的 immutable all-known-free snapshot fixture：

- `TubeSource::ESDF`；
- off-grid captured current；
- current 可在 stage 期间单调前进；
- snapshot sequence/provenance 有效；
- 成功时 commit same-owner Pair；
- 明确 OCCUPIED/OUT_OF_MAP latest snapshot 仍拒绝；
- frozen snapshot 与 later safe snapshot sequence 变化不凭空否决；
- phase 到达/越过 seam 必须拒绝，不安装过期 Pair。

### H2 regressions

现有 H2-L1 tests 必须保持：每 callback 单 stage、retry retains same pending identity、
CAS 后 completed、mirror 成功后 consume、expired/stale 正确 drop。

新增 pending-activation 回归：

- commit 一个 certified same-owner bootstrap Pair，但故意不调用 Runtime update；
- 断言该 Pair 为 pending activation，replan 分类必须进 H2；
- neutral direct commit 必须被拒绝，Pair identity/session/owner 保持；
- 再执行一次携带该 Pair 的 command update，断言 Runtime selected/executed；
- executed 后的 replan 仍继续走 H2，不得回落 neutral direct install。

## 8. 构建与静态验收

必须通过：

```text
catkin_make -j2 gvf_switch_policy_test phase_offset_matched_adapter_test
./devel/lib/bspline_race/gvf_switch_policy_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test
ctest --output-on-failure -R '(_ctest_phase_offset_navigation_gtest_phase_offset_tube_(cross_section|builder|filter|surface_validator)_test|_ctest_phase_offset_navigation_gtest_phase_offset_(certified_tube_builder|runtime|tube_epoch_manager)_test|_ctest_bspline_race_gtest_(gvf_switch_policy|continuous_phase_path|phase_offset_matched_adapter)_test)$'
catkin_make -j2
git diff --check
```

静态 scan：

- 每 timer bootstrap 最多一次 `stagePathTubePair()`；
- 无 loop/retry/sleep/parameter tuning；
- 无 ROS schema/topic/launch/Runtime/Tube 算法修改；
- Pair 仍只有 finalize CAS 一个 authority publication；
- H2 pending/completed/consume seam 未旁路；
- M3 unified builder 仍是唯一 production candidate build 入口。

## 9. 隔离 launch 顺序

每次 launch 前：

1. 检查目标端口无人监听；
2. 新建独立 `ROS_HOME` 和证据目录；
3. 记录本次启动 PID/PGID；
4. 只清理由本次 PID 清单启动的进程；
5. 绝不 attach/kill 环境中既有 ROS master/node。

固定：

```text
launch: phase_offset_esdf_tube_single.launch
map: existing pillar.pcd
goal message: (8,0,1)
internal goal semantics: (8,0,2), because goalCallback adds +1.0 to z
active override: phase_offset_manual_observe_only:=false
all other launch parameters unchanged
```

### Run 1: attribution

结果日志必须证明至少一次真实 bootstrap attempt，并确定唯一 first-false。若结果已是
`COMMITTED`，不得臆造修复；继续检查 command 是否捕获 pair、Runtime 为什么未 selected。

### Run 2: same-config acceptance

最小修正后重新构建，用新端口/ROS_HOME、完全相同 launch/goal/参数复验。必须记录：

- bootstrap `COMMITTED`；
- command 捕获同 generation/session 的 Pair；
- warmup 后 `selected=1`；
- `delta` 至少一个样本显著非零；
- Runtime valid，无 fatal invariant；
- raw candidate current anchor 持续完整；
- 至少一次 accepted replan 后出现 H2 single-seam stage；
- 若暂时 prepare 失败，same pending identity 被 retry；
- 最终 commit/consume 顺序正确，或在整次运行没有 H2 replacement 时准确标注未证明；
- 任意时刻不存在 new path + old/null Tube；
- 无 `all_candidates_path_end_clamped`；
- 到达日志与 internal `(8,0,2)` 目标一致。

本阶段只有在 Pair authority、selected 和 nonzero delta 三项都被动态证据证明时，才能
声明 initial offset bootstrap liveness 已恢复。H2 若因该次 planner 时序未触发，不能
伪称已动态证明，但现有 deterministic H2 regressions 必须全过。

## 10. 停止条件

完成单一 first-false 的最小修正、全部测试、构建、隔离 launch、自审和 handoff 后停止。
不得以本规格继续清理 temporary pin/mailbox、做 recenter、atomic authority 大改或多机。
