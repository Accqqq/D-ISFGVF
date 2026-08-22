# H2-L1 Single-Seam Retry-Safe Active Handoff

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行授权：用户已明确要求由主代理持续调度、由 `gpt-5.6-terra / xhigh` 实现  
`AUTO_ADVANCE=true`，但只限本文件定义的 H2-L1；不得提前进入 recenter、atomic authority 或 pin/mailbox 删除。

## 1. 阶段目标

本阶段只解决 active nonzero Path--Tube 交接中的两个确定性生命周期缺陷：

```text
同一次 replan callback 同步枚举多个 future seams
+ pending/completed mailbox 在一次失败前被 move 掉
```

目标行为：

```text
每次 replan 最多选择一个 deterministic future seam
→ 最多构建一次 C2 successor 和一次 successor Tube
→ 成功后形成一个 pending transaction
→ command callback 在 seam 截止前可重复 prepare/finalize
→ 一次可重试失败不丢 pending
→ 成功 CAS 后 completed frontend 保留到 FSM 成功应用
→ stale/expired transaction 无副作用丢弃
→ 后续 periodic replan 可从新的 live w 再选择一个 seam
```

这里的“异步”沿用现有架构的跨 callback 边界：FSM 负责一次 staging，command callback 负责 prepare/CAS，FSM 再消费 completed frontend。本阶段不新建后台线程、线程池或第二套 planner。

## 2. 已完成且必须冻结的 Z1 基线

必须保留：

- planner 最终 C2 路径是 neutral `delta=0` 权威中心线；
- Tube 只构造 zero-connected offset capacity；
- 无非零证书时为有效 zero-only `[0,0]`；
- 只有 `OFFSET_CERTIFIED` 可激活非零 offset authority；
- production Tube 使用 `planning/safe_distance`；
- active nonzero 正常交接仍要求完整 successor Path--Tube pair 和 `w+=w-`、`delta+=delta-`。

不得修改 Z1 Tube 几何、clearance、Filter、Validator 或分类语义。

## 3. 不可改变的导航基线

严禁修改：

- Kino A*；
- B-spline optimizer；
- 五次 Hermite C2 connector 的几何公式、连接条件和 cost；
- global semantic `w`；
- planner 重规划触发、候选接受策略和速度参数；
- Governor、SO3 和原始单机控制链；
- Runtime 的 matched-port/retained-delta 数学；
- neutral planner direct install；
- proposal 的理论命题。

本阶段不得新增导航门控、不得调参数、不得用 zero-only 作为 active nonzero successor。

## 4. 强制设计决定

### H2-L1.1 单一 deterministic seam

删除生产路径中的：

- `structuralFutureSeamCandidates()` 返回完整 vector；
- `tryFutureSeamCandidatesInOrder()`；
- 同一 replan callback 内“第一个 C2/Tube 失败后继续尝试后续 seam”的循环。

替换为单值选择接口，名称可按现有风格调整：

```cpp
bool selectStructuralFutureSeam(
    const std::shared_ptr<const PathTubePair>& old_pair,
    double captured_w0,
    double min_construction_lead_w,
    double& seam_w);
```

规则：

1. 只从 old pair 的 immutable owner-aligned structural samples 选择；
2. 选择第一个满足 `w >= captured_w0 + min_construction_lead_w` 且与 owner state 一致的 sample；
3. 不查询旧 Tube 是否覆盖 seam；
4. 不合成新的 seam phase；
5. 一次 replan callback 最多调用一次 `buildPhaseV2C2Frontend()`；
6. 一次 replan callback 最多调用一次 `stagePathTubePair()`；
7. 此 seam 的 staging 失败后立即返回，不能在同一次 callback 尝试后续 seam；
8. 后续 periodic/collision replan 使用新的 captured live `w` 重新选择 seam。

这减少单 callback 最坏构建量，不改变 C2 几何或 periodic retry 语义。

### H2-L1.2 Pending 不得在 prepare 前被消费

当前 `prepareAndCommitPendingPathTubeHandoff()` 在 expensive prepare 前执行：

```cpp
handoff = std::move(pending_path_tube_handoff_);
```

必须删除这种所有权转移。

新行为：

1. 在 handoff mutex 下只复制 pending `shared_ptr` identity；
2. malformed payload、session mismatch 或 transaction/pin 已失效属于 permanent stale，可清除；
3. `current_w >= future_seam_w` 是明确 expiry deadline，清除 pending 并释放 pin；
4. 在 `current_w < future_seam_w` 时，`preparePathTubePairCommit()` 返回 false 属于 retryable attempt；只要同一 pending identity/session 仍 live，就保留 mailbox；
5. lock-free prepare 后 phase generation 改变、live command phase 不再等于 capture、或 final short predicate 暂时失败时，同样保留 pending，下一 command callback 用新的 capture 重试；
6. 只有成功完成 adapter CAS 并在同一 manager critical section 发布 completed mailbox 后，才清除 pending；
7. stale/expired guard 的析构必须发生在 manager/Runtime locks 外；
8. 任一 callback 最多对同一 pending 做一次 prepare/finalize attempt。

不得新增无限内部 while-loop、sleep 或阻塞等待。

### H2-L1.3 Pending 的截止点就是 future seam

本阶段不新增 ROS 参数或经验性 guard。

```text
deadline_w = candidate_pair.future_seam_w
```

只要 `current_w < deadline_w`，pending 可由后续 command callback 重试；到达或越过 deadline 后必须作为 expired 无副作用释放。

expiry 后 periodic replanning仍保持可用，并从新的 live phase 选择下一个 deterministic seam。不得恢复旧的 terminal retry suppression。

### H2-L1.4 Completed mailbox 只在成功应用后清除

当前 `consumeCompletedPathTubeHandoff()` 在检查和应用之前 move completed mailbox。必须改为：

1. under lock 捕获 completed pointer identity，但不清空；
2. malformed payload、session mismatch 或 live authority 已明确不再匹配时，清除 stale completed；
3. frontend payload validation/application 失败时不得留下“已消费但 mirror 未安装”的静默状态；
4. 只有 `applyPathTubeFrontendMirrorLocked()` 与 `installAuthoritativePathMirrorLocked()` 成功后，才在同一 serialization boundary 清除同一 completed identity；
5. 成功发布只发生一次；后续 FSM tick 不得重复应用或发布；
6. completed frontend 不得回退或修改已经 CAS 成功的 PathTubePair authority。

如果现有 apply 顺序使失败发生在部分写入之后，先完成无副作用的 payload prevalidation，再执行 mirror mutation；不要通过新增恢复状态机补救部分写入。

### H2-L1.5 最小生命周期结果分类

为测试和稳定日志增加一个小型内部枚举，避免继续只返回无语义 `false`。名称可调整，但最多表达：

```text
NONE
RETRY_PENDING
COMMITTED
DROPPED_EXPIRED
DROPPED_STALE
CONSUMED
```

可以分别为 pending commit 与 completed consume 使用更小的枚举。它们是 manager 内部生命周期结果：

- 不新增 ROS message/schema；
- 不成为导航门控；
- 不成为 Runtime mode；
- 不增加 Tube epoch reason；
- 日志必须节流，不能每个 50 Hz retry 刷屏。

### H2-L1.6 Pin 与双 mailbox 暂时保留

本阶段不删除：

- `PathTubePairPinRegistry`；
- `PathTubePairPin`；
- pending/completed 两个槽；
- adapter 的 PathTubePair CAS；
- timer refresh ownership。

只修正它们的生命周期。atomic `ExecutionAuthority` 和旧机制删除属于后续独立阶段。

## 5. 明确不做的内容

本阶段不实现：

- 默认或 fallback recenter；
- `delta` reset；
- swarm intent pause；
- terminal no-successor 的物理 recovery；
- background worker/thread；
- atomic ExecutionAuthority；
- pin registry 删除；
- pending/completed 合并；
- gradient Tube；
- CertifiedTubeBuilder；
- proposal 理论修改。

因此 H2-L1 完成后，若物理上或证书上始终不存在任何 active-nonzero successor，系统仍需要后续 M6 certified recenter/recovery 阶段。自审必须明确这一边界。

## 6. 当前阶段文件白名单

只允许修改：

```text
docs/Codex_H2_Single_Seam_Retry_Safe_Handoff_Execution_Spec_2026-08-20.md
docs/Codex_H2_Single_Seam_Retry_Safe_Handoff_Self_Audit_2026-08-20.md

src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp

/home/cxq/ISF-GVF/handoff.md
```

只读回归但不得修改：

```text
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/phase_offset/**
src/swarm_planner/plan_env/**
Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md
```

若无法在白名单内完成，停止并报告，不得扩大到 adapter、Runtime、Tube 或 planner。

## 7. 必须新增或改写的测试

### Single seam

1. 选择 earliest eligible owner-aligned structural sample；
2. exact construction-lead boundary 被保留；
3. 非单调或 owner mismatch 输入 fail closed；
4. 一次 staging 失败不会调用第二个 seam；
5. 后续以更大的 captured `w` 调用时可选择自然更晚的 seam。

旧 `AttemptsLaterImmutableCandidateAfterFirstC2Failure` 必须删除或反转；产品不再允许 same-callback fallback。

### Pending lifecycle

1. 第一次 prepare false 后 pending identity 仍存在；
2. 下一 command capture 可对同一 pending 重试；
3. phase generation drift 不消耗 pending；
4. `current_w == future_seam_w` 清除 expired pending；
5. session/reset drift 清除 stale pending；
6. success 恰好从 pending 转换为 completed 一次；
7. pin 在 retry 期间仍绑定原 pair，在 expired/stale/success publication 后释放；
8. 多线程/R2 stale completion 不改变 live authority。

### Completed lifecycle

1. valid completed frontend 成功应用一次并清空；
2. transient pre-application failure 不静默丢 payload；
3. stale session/authority payload 被无副作用丢弃；
4. 成功后下一 FSM tick 不重复发布；
5. 任意时刻不出现 `new path + old/null Tube`。

### 回归

必须复跑：

- `gvf_switch_policy_test`；
- `phase_offset_matched_adapter_test`；
- `phase_offset_runtime_test`；
- `phase_offset_tube_epoch_manager_test`；
- `continuous_phase_path_test`；
- Z1 CrossSection/Builder/Filter/SurfaceValidator tests。

## 8. 验收标准

1. production code 无完整 seam vector 枚举与 same-callback multi-attempt helper；
2. 每次 replan callback 最多一次 C2 build、一次 Tube stage；
3. pending 不再在 prepare 前 move；
4. retryable prepare/final predicate failure 保留 pending 到下一 command callback；
5. seam 到期或 session stale 时明确释放，无副作用；
6. completed 只在成功 mirror application 后清除；
7. active PathTubePair CAS 仍是原子完整 pair；
8. neutral planner/C2/Z1 行为不变；
9. 无新参数、线程、ROS schema 或导航门控；
10. 定向与回归测试全部通过；
11. `catkin_make -j2` 通过；
12. `git diff --check` 通过；
13. 自审记录 dirty worktree、白名单和仍未解决的 recenter/recovery 边界。

## 9. 停止条件

出现以下情况立即停止并向主代理报告：

- 需要修改 adapter/Runtime/Tube/planner 才能完成；
- 需要新线程或 ROS 参数才能获得可重试语义；
- retry retention 会破坏 current-state safety 或 PathTubePair CAS；
- 测试要求默认回零或瞬时修改 `delta`；
- 工作区出现与 H2-L1 白名单重叠的用户新改动。

