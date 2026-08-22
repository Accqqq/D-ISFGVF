# A6/H2 overlap-certificate Path–Tube handoff liveness — execution specification

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A6_H2_OVERLAP_CERTIFICATE_HANDOFF_LIVENESS
IMPLEMENTATION_AUTHORIZED=true
USER_AUTHORIZATION=explicit request to schedule Terra/xhigh implementation
AUTO_ADVANCE=true
```

日期：2026-08-19  
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. Objective

修复 active nonzero phase-offset 下，中途重规划虽然已经得到安全可达的 KinoA*/B-spline
候选和 C2 connector，却因旧 `PathTubePair` 的有限证明域无法提供
`current_w + min_certified_forward_w` 的 future seam，最终进入永久
`GOVERNOR_INVALID_HOLD` 的问题。

本阶段必须同时保持：

1. 原始单机导航仍使用 retained-current-phase C2，并保持全局 `w`；
2. `delta=0`、尚未执行 nonzero offset authority 或 `observe_only=true` 时，H2 不得阻塞
   planner frontend；
3. active nonzero offset 时仍只允许原子 `{path, tube, runtime, map provenance}` 交接；
4. future geometric seam 保留；现有 `min_certified_forward_w` 可以继续提供确定性的异步
   construction lead，但不得再解释成旧 Tube 必须 certified 到该 seam；
5. 不通过调小安全裕度、强塞 `{0}` Tube、重置 `delta`、降低速度或关闭重规划恢复导航。

## 2. Fixed proposal and audit conclusions

本阶段以详细 proposal 的以下语义为准：

```text
w+ = w-
delta+ = delta-
C2 matches p, p_w, p_ww
new Tube is rebuilt on the new C2 composite owner
connector selection checks Tube and offset reachability
```

相关条款：

- 详细 proposal §8.2–8.3；
- 详细 proposal §17.1–17.2；
- 详细 proposal §21、§22.7；
- 命题 5：C2 path 与 Tube 更新下的 double continuation；
- `AGENTS.md` §9：复用原 C2 connector，不建立第二套 Tube C2。

SOL/max 审核固定以下结论：

- baseline path-only C2 不需要 future seam；
- future seam 不是全局 `w` continuation 的语义，而是异步构建新 C2+Tube 时防止
  capture seam 在提交前被 live `w` 越过的工程事务机制；
- 不能简单把异步 H2 改回 stale current-seam；
- 当前错误耦合是：以 `min_certified_forward_w=0.4` 选择未来结构 seam 的同时，又要求
  旧 Pair 的 active Tube certificate 覆盖到该 seam。动态复现进一步证明，异步 C2+Tube
  staging 仍需要确定性 future lead；应删除旧 certificate-to-seam 义务，而不是删除 lead；
- 正确方向是 future geometric seam + copied old prefix + new-owner Tube + live-current
  atomic CAS，而不是让旧 Tube 一直证明到 geometric seam。

## 3. Fixed runtime evidence

用户本次日志：

```text
current_w       = 12.006205
old_path_end    = 12.015170
required_seam_w = 12.406205
goal_distance   > 2.7 m

Kino replan success
C2 connector success at several earlier phases
NONZERO_OFFSET_HANDOFF_RECOVERY_REQUIRED
all_candidates_path_end_clamped
terminal nonzero handoff denial latched
```

该事件不是 global `w` 重置，也不是 Tube clearance/Filter 的直接拒绝。它是旧 Pair 的
有限 path/certificate horizon 与 H2 seam 选择耦合产生的 terminal liveness failure。

但是 terminal empty seam 只是最终直接失败。本阶段必须先找到同一次运行中、terminal
之前第一个未安装 accepted planner candidate 的 exact first-false；不得假设所有早期失败都
与 terminal seam 相同。

## 4. Required semantic split

必须明确区分四个对象：

```text
global w                     persistent semantic coordinate
old path object domain       finite old ContinuousPhasePath interval
old Tube certificate horizon finite proof on the old owner
new composite path domain    same global w, extended by copied prefix + C2 + new tail
```

### 4.1 Baseline/neutral branch

满足任一条件时，继续走原始 current-phase C2 安装：

```text
observe_only
no executed nonzero offset authority
authoritative neutral state that has been explicitly released by an existing contract
```

该分支：

- 不调用 H2 future-seam selector；
- 不消费旧 `PathTubePair`；
- 不等待 Tube；
- 保持 `phase_before == phase_after`；
- 新 Tube 作为新 planner owner 的异步 sidecar 重新构建；
- 不得制造未经证明的 `{0}` Tube。

manual warm-up 后第一次取得 offset authority 的 bootstrap 也属于 Tube timer 的异步所有权：

- `phaseOffsetTubeTimerCallback` 可以在普通 `timerTick()` 后，针对当前 planner owner 调用既有
  `activatePendingOffsetAuthority()`；
- 50 Hz `cmdCallback` 只捕获/消费已经完成的 pair，不得调用会进入
  `buildPreparedTubeEpoch()` 的 bootstrap 重活；
- bootstrap 使用和 H2 相同的 overlap 时序：timer 捕获 `w0/owner/session/generation`，以
  `ws >= w0 + min_certified_forward_w` 的 structural seam 异步构建；构建完成后重新捕获
  live `wc` 并在该点执行 new-pair Runtime dry-run；staging 不得生成绑定 `w0` 的
  `PathTubePairCommitPreparation`，唯一 preparation 只能绑定该 live `wc`；
- 正常 command 推进 `w0 < wc < ws` 必须允许，不能要求提交时 `wc == w0`；短 CAS 边界只
  要求 planner owner/session/phase generation 未漂移且 phase 等于刚刚用于 dry-run 的 live
  snapshot；
- `wc` dry-run 同时重捕当前 `odom` 与最新 map snapshot；staging 的 position/snapshot 只保留
  construction provenance，不能代替 live actual/reference categorical check；
- reset、planner owner/session 变化或 `wc >= ws` 时无副作用失败并由 timer 重试，不得阻塞或
  改写 baseline command。

仅仅在历史上配置过 manual amplitude，不等于当前拥有 nonzero execution authority。

### 4.2 Active nonzero branch

定义：

```text
w0 = transaction capture phase
wc = live phase at short commit/CAS boundary
ws = future geometric C2 seam
```

要求：

```text
w0 <= wc < ws
new path == old path on the copied prefix containing wc
new path enters its C2 connector only at ws
```

`ws` 用于几何异步连续性；CAS 发生在 live `wc`。二者不得混为同一个时刻。

## 5. Overlap-certificate contract

### 5.1 Old Pair responsibility

旧 Pair 只负责：

- 从 capture 到实际 CAS 期间继续执行；
- 在 live `wc` 对 current reference、retained `delta` 和实际 tracking state 保持有效；
- 保持 exact pointer/revision/generation/session/map provenance，直到 CAS 或 transaction
  discard。

不得仅因为旧 Tube 没有证明到 `ws` 就提前拒绝；旧 Tube 不得被复制或解释到新 connector。

### 5.2 New Pair responsibility

新 immutable pair 必须完全绑定同一个 new C2 composite owner，并重新构建自己的 Tube，覆盖：

```text
capture/current overlap
copied old-prefix remainder
future seam ws
complete C2 connector
required new-owner forward execution horizon
```

新 Pair 的每个 Tube sample、`p/N`、map snapshot、profile 和 Runtime dry-run 都必须来自
new owner，不能复用旧 Tube 几何。

### 5.3 Seam candidate source

future geometric seam 候选来自 pinned old path owner 的 immutable structural sample/domain，
并保持现有确定性 construction lead：

```text
ws >= w0 + min_certified_forward_w
```

该不等式只约束 structural seam 与异步构建余量；不得查询或要求 old active Tube profile、
`certified_segment_end_w`、old Tube bounds 已经覆盖 `ws`。旧 Tube 仍只负责 live CAS phase
`wc`。`min_certified_forward_w` 同时继续保持 new Tube 安装 horizon 的既有含义，但本阶段不
调整其数值，也不新增 lead 参数。

不得增加新的 launch lead 参数。每个 staged candidate 在 commit 时必须重新验证
`live wc < ws`；已过期 candidate 无副作用丢弃。

### 5.4 Short commit boundary

CAS 前必须在既有固定锁序内只读验证：

1. exact old pair pin/identity/session 仍为 current；
2. `w0 <= wc < ws`；
3. old pair 在 `wc` 当前有效；
4. new owner copied prefix 在 `wc` 与 old owner 的 `p,p_w,p_ww` 一致；
5. new Tube 在 `wc` 有效并包含 retained `delta`；
6. current reference、actual state 和 tracking-error ball 满足最新 map categorical contract；
7. copied Runtime/previous final port 在 new pair 上 non-mutating dry-run 成功；
8. new Tube 对 copied prefix、connector 和其 required forward horizon 的覆盖完整；
9. 一次 atomic CAS 只暴露完整 old pair 或完整 new pair。

任何失败都不得修改 live `w`、`delta`、previous port、preflight、path mirror 或 authority。

## 6. Connector/Tube joint selection

不得继续把“中心线 C2 已接受”当作 PathTubePair 已接受。对每个结构候选 seam/connector，按序：

1. 构造 new composite owner；
2. 在 new owner 上构造 isolated prepared Tube；
3. 验证 current retained `delta`；
4. 使用现有 exact PWL/profile 与 non-mutating Runtime 能力验证 connector 上连续可执行的
   offset/port witness；
5. 验证正 phase、`u_delta`/rate、正则性和最新 map；
6. 成功才进入 pending transaction；失败继续尝试既有其他 connector 长度或 planner
   candidate。

优先复用现有 Filter、Runtime dry-run、C2 enumeration 和 transaction primitive。不得新增另一套
certificate 类型、第二套 C2 connector 或特定地图分支。

若现有 primitive 不能证明 proposal §17.2 的完整 connector 可达性，必须在 self-audit 中精确
标出缺口；不得用一次单步 query 冒充全段证明。

## 7. First-false requirement

实现前先在 private ROS master 复现 long point-to-point manual-active ESDF run。临时证据必须
为 terminal HOLD 前每个 accepted planner candidate 给出且仅给出第一个失败层：

```text
old pair pin/identity
structural seam enumeration
C2 connector
new-owner Tube build/coverage
retained-delta/current-state check
connector viability/dry-run
transaction slot
live-w expiry
final CAS/mailbox
```

临时 instrumentation 在归因完成后移除，不新增 ROS schema、topic、参数或永久 gate。

只有 exact first-false 与本执行单架构一致时才实施对应修复；若 earliest false 属于不同层，先按
相同 invariants 修正该层，再继续 overlap handoff 验收，不能只修 terminal 日志。

## 8. Authorized files

```text
docs/Codex_A6_H2_Overlap_Certificate_Handoff_Liveness_Execution_Spec_2026-08-19.md
docs/Codex_A6_H2_Overlap_Certificate_Handoff_Liveness_Self_Audit_2026-08-19.md

src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
```

若完整 connector viability 确实需要修改 `TubeFilter`、`TubeBuilder`、`ContinuousPhasePath` 或
新增 navigation primitive，先停止产品修改并在 self-audit 报告 exact interface blocker；不得自行
扩展白名单。

## 9. Explicit prohibitions

- 不修改 planner/A*/B-spline optimizer/C2 公式和选择参数；
- 不修改 Tube geometry、Filter 数学、SurfaceValidator、margin、map inflation；
- 不修改 `min_certified_forward_w`、lookahead/back、sample step、速度、加速度、governor
  saturation 或 replan period；
- 不通过 `observe_only=true`、`mode=disabled` 或 amplitude=0 伪造 active nonzero 验收；
- 不强制 `delta=0`，不加入特定终点 recenter，不恢复 F1a/F1b/recenter 实验代码；
- 不允许 new path + old Tube、new path + null Tube 或旧 Runtime state 重绑新 owner；
- 不因 transaction 失败推进 `w/delta`、修改 preflight 或永久占用 mailbox；
- 不处理飞行中用户显式发送另一个目标；本阶段只处理同一目标的 planner replan；
- 不推进 A7、swarm allocator/CBF 或多机场景。

## 10. Required focused regressions

至少覆盖：

1. baseline/observe-only/current-neutral replan 使用原 current-phase C2，`w` 不重置且不触发
   future-seam H2；
2. active nonzero：old Tube certificate 不覆盖 `ws`，但 old current state 与 new overlap Tube
   都有效时允许准备并原子提交；
3. old pair 在 live `wc` 已失效时拒绝；
4. new Tube 在 `wc` 不包含 retained `delta` 时拒绝；
5. live `wc >= ws` 的 expired candidate 无副作用拒绝；
6. new Tube 未覆盖 copied prefix、connector 或 required new horizon 时拒绝；
7. exact pin/session/generation/map provenance 任一漂移时拒绝；
8. 失败 transaction 不改变 live Runtime bits、`w/delta`、previous port 或 pair；
9. 成功 transaction 只产生一次 generation advance 和一次 frontend consume；
10. structural seam 保持 `min_certified_forward_w` construction lead；old certificate 明确不
    需要覆盖 seam，且 new Tube 安装域检查保持；
11. 多个 connector candidate 中前一个 Tube/viability 失败、后一个成功时能够继续选择；
12. 既有 Runtime、Adapter、GVF switch 全部回归通过。

## 11. Dynamic acceptance

在 private ROS master 上至少完成：

### R0 — baseline navigation

```text
phase_offset disabled or observe-only
long point-to-point run
multiple planner C2 replans
goal reached
```

必须无 H2/future-seam blocker。

### R1 — active nonzero overlap handoff

```text
mode=manual
observe_only=false
tube_source=esdf
nonzero offset actually selected
same long point-to-point target
```

必须出现：

```text
at least two planner C2 acceptances
matching joint PathTubePair generation advances
phase_before == phase_after within tolerance
delta continuity without forced reset
continued VEL_MATCH_GOVERNOR execution after each handoff
goal reached / WAIT_TARGET
```

不得出现持续：

```text
NONZERO_OFFSET_HANDOFF_RECOVERY_REQUIRED
all_candidates_path_end_clamped
terminal handoff denial retry suppression
new path + old/null Tube authority
```

### R2 — expiry/race stress

重复 active run 或使用既有 test hook 使至少一个 staged candidate 在 CAS 前过期。必须只丢弃
expired candidate，旧完整 pair 继续执行；后续较晚有效 transaction 能成功，不死锁、不永久
HOLD、不发生 stale completion 覆盖。

## 12. Build and audit gates

```bash
catkin_make --pkg phase_offset_navigation --make-args phase_offset_runtime_test -j2
catkin_make --pkg bspline_race --make-args phase_offset_matched_adapter_test gvf_switch_policy_test -j2

./devel/lib/phase_offset_navigation/phase_offset_runtime_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test
./devel/lib/bspline_race/gvf_switch_policy_test

catkin_make -j2
git diff --check
```

还必须：

- `rg` 确认没有恢复 `PendingNeutralRecenter`、F1a/F1b、same-goal recenter 实验；
- 记录前后 `git status --short`，保留所有用户已有改动；
- 只清理本阶段 private ROS master/process；
- 写 self-audit，逐项对应本执行单和详细 proposal。

## 13. Completion boundary

只有同时满足以下条件才可标记完成：

1. terminal 前 exact first-false 已被动态证明并修复；
2. baseline current-phase C2 完全不受 H2 影响；
3. active nonzero 至少两次真实 joint PathTubePair handoff 后继续导航并到达目标；
4. 没有参数/安全裕度放宽，没有强制 `delta=0`；
5. focused/full build、tests、dynamic acceptance 和 self-audit 全部通过。

若 connector 全段 viability 缺少现有可证明接口，按 §8 报告 blocker，不得用新的宽松条件、
单步 dry-run 或 terminal 特例宣称完成。
