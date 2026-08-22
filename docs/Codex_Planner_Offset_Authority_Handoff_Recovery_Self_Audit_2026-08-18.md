# Planner/offset authority handoff recovery self-audit

日期：2026-08-18  
阶段：`PLANNER_OFFSET_AUTHORITY_HANDOFF_RECOVERY`

## 完成内容

本阶段把 Tube 的观测能力与 planner 路径安装权威拆开：

```text
requiresTubeTimer()
    MANUAL Tube timer/diagnostics，仍可在 observe-only 下运行

requiresAuthoritativeOffsetHandoff()
    仅当 MANUAL、非 observe-only，且 Runtime 已执行的 profile 尚未完成
    或 retained delta 非零时为真
```

因此在 `observe_only=true` 或中性 `delta=0` 时：

```text
最终 C2 planner path 可以按 baseline/current-seam 逻辑安装
Tube 继续异步构建 candidate/diagnostics
没有匹配 Tube 时 command guidance 回退新 planner owner
```

neutral direct C2 install 通过 `commitNeutralPlannerFrontend()` 在同一个
`frontend_apply -> handoff -> Runtime` 事务内完成最终 neutral 检查、废止旧
PathTubePair/mailbox/deferred tail，并安装新的 planner/GVF mirror；ROS topic 在解锁后发布。
这保证 command callback 不会观察到 `new planner path + old PathTubePair`。

初始 point/closed bootstrap 也显式携带独立的 `planner_path_owner`。因此在
observe-only/neutral bootstrap 没有 `PathTubePair` 时，frontend mirror 仍会安装最终
`ContinuousPhasePath`；command callback 不会在已经打印 `POINT_PHASE_V2 INIT` 后退化为
`current semantic path unavailable -> guidance_invalid`。

后续 neutral C2 direct install 同时通过 manager-owned authoritative mirror 更新
`gvf::last_path_` 和显式全局 `w`。`/particle0/gvf/traj_vis` 的渐变 C2 路径由该 mirror
定时生成；不能把新 frontend 送回会在 authoritative mode 下拒绝写入的 legacy
`pathCallback()`，否则控制路径已切换但 RViz 会继续显示旧路径。

非零 retained delta 仍保留旧的 H2 合同：future seam、C2 composite path、new-owner
Tube、Runtime dry-run、pin、generation/session 和最终 CAS 均未放宽。

## A6 lifecycle activation correction

动态复核发现，`delta=0` 不能单独说明 manual profile 已经结束：它也可能是 profile
尚未启动前的中性值。旧实现把“已配置 amplitude”提前作为 bootstrap Pair 的必要条件，
使 planner 中性 owner、zero-port warm-up 与 H2 Pair 生命周期相互耦合；在短局部 path
上，gate 可能直到 path 末端才真正进入 Runtime 的可执行分支。

现改为明确的 activation boundary：

```text
neutral Runtime (delta=0, profile not started)
    planner C2 owner controls navigation; Tube remains sidecar evidence

zero gate open + pending manual intent
    manager rebuilds/proves the current immutable planner owner as a fresh
    PathTubePair, executes Runtime dry-run, then final-CAS publishes the pair

next command
    Runtime receives that pair and may commit the first nonzero offset port
```

Pair 建立不构造新 C2 path、不复用 stale Tube，也不制造 `{0}` Tube。Pair 尚未存在或
activation proof 失败时，Runtime 不会从 sidecar epoch 发出首个非零 port；planner
continues as the neutral owner. 一旦 profile 已启动，原有 H2 future-seam/pin/session/CAS
合同保持不变；profile 完成且 recenter 到零后，又回到 neutral planner authority。

## Terra 终审后的 authority/并发补强

Terra xhigh 的独立终审发现，pending intent 与 executed authority 拆分后还需要三个
并发边界。现均已修正并加入回归：

1. `requiresPathTubePairBootstrap()` 只允许“尚未执行、retained `delta=0` 的 pending
   intent”。已执行 profile（包括瞬时过零）或 retained `delta!=0` 在 Pair 丢失后保持
   unpaired/fail-closed，不能通过 `expected_pair=null` 绑定到新 planner owner。
2. null-pair 的 `stage -> prepare -> final CAS` 三个边界都在 Runtime mutex 下重新拒绝
   executed authority；planner-only direct install 也通过同一 mutex 做最终 neutral 检查。
3. neutral retire、manager frontend mirror 和 GVF authoritative mirror 现在位于同一个
   `frontend_apply -> handoff -> Runtime` 事务中，ROS path 发布在解锁后。activation 同时
   捕获 `{planner_owner, authority_session}`，并在 final CAS 前复核当前 owner、session 和
   mailbox，不能再组合出 `{old owner, new session}`。

终端 suppression 也只作用于重复 periodic replan；collision-triggered replan 保持最高
优先级，不再被该 latch 拦截。

## 非零末端 recovery 边界

本阶段没有找到现有且可证明的物理 recenter/recovery owner。因而没有：

- 降低 `min_certified_forward_w`；
- 制造 `{0}` Tube；
- path-only 切换仍保持非零 reference；
- 发明 emergency/hover 控制命令。

当非零 retained delta 的旧 path 已不足以提供 future seam 时，代码只增加一次内部分类日志：

```text
NONZERO_OFFSET_HANDOFF_RECOVERY_REQUIRED
```

并继续现有 fail-closed handoff 语义。该分支仍需要后续授权阶段提供当前状态 recovery
owner，不能声称已经解决非零末端物理交接。

另外，`FSMCallback` 现在把该终端分类位作为当前 authority session 的一次性周期
replan 抑制：命令侧继续保持既有 `all_candidates_path_end_clamped` HOLD，碰撞触发的
显式 replan、goal/reset 和 owner retirement 不受抑制；没有生成 recovery command，也
没有放宽 seam 合同。这样旧 owner 无 certified future seam 时不会在每个
`planInterval` 无限重复 `REPLAN_TRAJ -> HOLD`。抑制位由 reset/retirement 清除；任何
成功的 H2 transaction 也会清除它。

## 修改文件

- `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp`

没有修改 planner 算法、TubeBuilder/Filter/Validator、margin、launch/config、message、
CMake、速度/rate/lookahead 或安全阈值。

## 验证

```text
cmake --build build -j2                              PASS
phase_offset_matched_adapter_test                   67/67 PASS
gvf_switch_policy_test                               83/83 PASS
phase_offset_runtime_test                            33/33 PASS
phase_offset_clearance_audit_test                    22/22 PASS
phase_offset_tube_epoch_manager_test                 51/51 PASS
phase_offset_tube_filter_test                        15/15 PASS
phase_offset_tube_surface_validator_test             11/11 PASS
```

已在 task-owned ROS master `127.0.0.1:11793` 使用原 ESDF 单机 launch 运行最终动态验收，
证据包为：

```text
/tmp/a6_h2_authority_final7_20260819/evidence/active_final.bag
goal=(8,0,1), phase_offset_manual_observe_only=false
duration=6.2 s, size=912.9 KiB, messages=1070
```

该 run 记录到：

```text
POINT_PHASE_V2 INIT: path_end_w=5.684
C2 planner install: phase=3.706, path_end_w=8.368, accepted_new=1
POINT_GOAL REACHED distance=0.190
tube / tube_candidate / active_path: 35 messages each
tube diagnostics / epoch diagnostics: 26 messages each
```

记录中没有 `all_candidates_path_end_clamped`、没有 `planner-only switch denied`。仅在 planner
frontend 尚未初始化、goal 刚收到的第一个控制周期记录两条既有 `guidance_invalid` HOLD；
后续控制为正常 governor/C2 路径。

实际 manual profile 已启动后，临近目标出现一次（且仅一次）合法的：

```text
NONZERO_OFFSET_HANDOFF_RECOVERY_REQUIRED
current_w=8.247777 required_seam_w=8.647777 old_path_end=8.631957
```

这是执行计划保留的真正 nonzero recovery 边界：旧 owner 只差约 `0.016 m` 无法提供所需
future seam，系统没有 path-only 切换或制造 recovery command；一次性分类后仍在约
`0.16 s` 内正常满足 point-goal tolerance。它与原 bug 不同：原 bug 是仅因 pending
`amplitude=0.1` 就永久阻塞 neutral planner C2。

新增 focused regression
`GvfH2RecoverySuppression.TerminalNonzeroDenialSuppressesPeriodicRetryUntilAuthorityReset`
验证一次性分类位只在 reset/authority retirement 后清除；
`GvfPlannerAuthority.NeutralFrontendInstallsIndependentOwnerWithoutManufacturingTubePair`
验证 neutral bootstrap 不依赖 `PathTubePair` 也能保留 planner owner。

另有 `GvfPlannerBootstrapPolicy.NeutralRetireAndPlannerMirrorInstallShareOneManagerTransaction`
验证 neutral retire 与新 owner mirror 为同一 manager 事务，并确定性拒绝
`{old owner, new session}` 的 activation 混合快照；collision-priority 回归证明 terminal
periodic suppression 不影响碰撞重规划。

Terra xhigh 在最终 diff 上完成第三次独立复核，结论为 `PASS`：未发现剩余
authority-bypass、data race、TOCTOU 或 lock-order defect。其唯一保留意见是仓库本身已有
大量阶段外 dirty/untracked 文件，因此无法仅凭当前 worktree 对历史改动做全局归属证明；
本阶段实际修改仍限制在执行规范白名单内。

另新增/更新 Adapter 回归，验证 gate 到达 100 时未配对的 complete sidecar Tube 不能选择
manual port；通过现有 `stagePathTubePair -> preparePathTubePairCommit -> final CAS` 建立同-owner
Pair 后才允许选择。这覆盖首个非零 port 不得在无 Pair 分支执行的 activation 边界。

## 结论

单机/observe-only/neutral planner 导航不再被 Tube H2 handoff 作为前置条件阻塞；命令侧
使用新的 planner owner，并在 owner 发布前废止旧 offset authority。manual offset 只在
同一 planner owner 的 Pair 被完整证明并 CAS 后激活。非零 offset 的终端交接仍明确
fail-closed，等待后续授权的物理 recovery owner。

阶段完成，停止于本执行计划范围内。
