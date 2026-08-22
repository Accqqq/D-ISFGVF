# Codex P1/P2：连续可行 port 与当前状态 recovery 正式执行单

```text
DOCUMENT_ROLE=FORMAL_FUTURE_EXECUTION_SPECIFICATION
DOCUMENT_STATUS=CONTRACT_FROZEN__ALL_PLANNED_PRODUCT_MODIFICATIONS_NOT_AUTHORIZED
STAGE=P1_CONTINUOUS_VIABLE_PORT__P2_CURRENT_STATE_RECOVERY
REPLACES_DIRECTION=A6_H2_ACTIVATION_PATCH_AS_THE_REMEDY
AUTO_ADVANCE=false
GO_CONDITION=P1_ACCEPTANCE_PASS && P2_ACCEPTANCE_PASS
```

## 1. 决策与范围

本单把问题从 A6/H2 的 activation、timer、C2 seam 或原子 handoff 修补中移出。A6/H2
仍须保持其既有的 immutable owner、future seam、pin、session/runtime-bit 与 final-CAS
合同；它不是 current-state 可行性或逃逸控制的 owner。

拟议的正确路线是：对单一 immutable tube owner 构造 **tube-aware、连续的 viable
port**，并用从真实 current state 可达的连续 connector/witness 决定 normal owner 是否
可接受；当该证明不可用或 current state 已不在 tube 中时，必须由明确的 recovery owner
接管。它不是把 `delta=0` 强行塞回 tube，也不是把 non-selected 静默交给 base governor。

本文件只冻结未来 P1/P2 的合同、顺序、白名单和验收方法。除本文件外，**所有计划性产品、
测试、构建、参数、启动和 AGENTS 修改均为 NOT AUTHORIZED**。本单不授权实现，也不使
旧 A6/H2 单中的产品白名单重新生效。

P1 与 P2 是一个 GO 闸门的两个必要部分：**P1/P2 不同时通过，不得 GO、不得把任一
normal/recovery 路线视为可发布。**

## 2. 证据基线和必须纠正的时间线

### 2.1 可复核证据

本单只以以下现有证据为输入，不重新解释其已撤回的实验：

| 证据 | 可用事实 | 本单采用的边界 |
| --- | --- | --- |
| `Codex_A6_H2_Prepared_Epoch_Activation_Execution_Plan_2026-08-14.md` §9（P0） | 在 old path 仍有 future certified horizon、C2 成功、exact pair/session/pin 成立时，fresh staged `TubeEpochManager::update()` first false；`reason=CURRENT_OFFSET_OUTSIDE`、`retained_delta=0`、`current_bounds_valid=true`、`forward_horizon_sufficient=true`。 | 排除 H2 scheduling、C2、pin/CAS、warmup 与前向 horizon 为首因。 |
| 同文 §12（P0.5 run3） | capture phase 的第一个 exact-PWL sample：raw `[+0.125,+1.325]`、filtered `[+0.125,+0.864691]`；raw 已排除 zero，且 immutable snapshot current clearance 不足。 | Filter 不是 first exclusion；不得放宽 `CURRENT_OFFSET_OUTSIDE` 或安装该 candidate。 |
| `Codex_A6_H2_Upstream_Current_State_Compatibility_Read_Only_Audit_2026-08-16.md` §2–§3.4 | Runtime 在 retained offset 不在 active PWL interval 时会给出既有 `WAITING_FOR_CANDIDATE`/`CURRENT_OFFSET_OUTSIDE`；bounded witness 无法给出时会给出既有 `CERTIFICATE_DENIED`。`selected=false` 后 manager 仍可能发布 `VEL_MATCH_GOVERNOR`。 | 当前缺的是 path/tube/execution 合同和 recovery bridge，不是 Filter 或 H2 activation 修补。 |
| 同审计 §2 map49→54 复现 | 对同一 decisive current owner point 与 actual point，map49 和 map54 的实测 clearance 分别相同；old active 与 new candidate 的相邻 profile sample 都是 raw `[+0.125,+1.325]`。 | 只可称为**该两个决定性 current points 的查询结果未变**；不得泛称完整 map snapshots 相同，也不得把地图编号变化说成首因。 |

### 2.2 强制采用的动态时间线

以下时间顺序是 P1/P2 的设计输入，不能再被折叠成“tube set 为空”或“仅一个
`CURRENT_OFFSET_OUTSIDE` 事件”。它与 P0/P0.5 的 H2 capture 拒绝相容，但描述的是
Runtime/command 链路中更早出现的缺口。

1. 在时间 `t0`，active exact-PWL bounds **含有 zero**，而有限 DFS 没有找到 bounded
   witness。Runtime 因而给出既有 `CERTIFICATE_DENIED`，`selected=false`（并且该次
   matched port 不可用）。
2. 同一 command cycle 没有 recovery bridge。因 command callback 只在 `selected=true`
   时替换输出，base guidance 仍有效，最终来源仍为 `VEL_MATCH_GOVERNOR`。
3. baseline 随后继续推进至 `t1`；此后 retained `delta=0` 才进入不含 zero 的 active PWL
   section，并出现 `CURRENT_OFFSET_OUTSIDE`。

因此，**finite DFS no-witness 只表示该有限搜索没有交付证书；绝不等于数学可行集合
`U` 为空。**只有一个具有完整连续语义的 oracle 在其明确的 domain、assumptions 和证明
义务下给出“空”的结论时，才可记录 `U=∅`；本单不允许既有 `CERTIFICATE_DENIED` 被
改名、推断或外推为这种结论。

P0/P0.5 则记录另一条同样必须保留的边界：在其真实 H2 capture，new owner 的 raw current
corridor 已经不含 retained zero，故 H2 的 fail-closed 拒绝正确。两种事实共同要求连续
可行 port 与 recovery，而非 H2 侧绕过。

### 2.3 明确排除的旧方向

- 不把 A6/H2 “再激活”、timer retry、提前 replan 或 future-seam 调度当作修复。
- 不改 `TubeFilter`、cross-section、SurfaceValidator、A5 geometry、margins、速度、
  lookahead、rate、H2、CAS、pin、session 或 runtime bit。
- 不新增 gate、latch、ROS parameter、产品 enum/mode/reason/diagnostic schema；也不以
  新名字包装既有拒绝。
- 不以 `.40/.45/.55` 改数值、重算 margin 或调 planner 约束作为通行办法。
- 不跨 owner、path revision、generation、session、map sequence 或 immutable snapshot
  复用 profile、cross-section、tube sample、clearance 或 witness 几何。

## 3. T0 合同冻结（D0；仅 docs；NOT AUTHORIZED）

T0 是 P1/P2 的先决条件。它只可产生 D0 文档修订；没有签核的 T0 不得开始任何产品或测试
修改。

### 3.1 Pre-gate / post-gate 策略

1. **Pre-gate / unarmed 保守特例。**在 normal owner 尚未 armed、没有已提交可行 port
   历史时，`delta=0` 的 robust 可行性只可作为保守起始假设之一。此时必须明确零初值、
   exact-PWL domain 与从该初值到首段 witness 的连续 connector；无法证明则不 arm。
2. **Post-gate / armed 策略。**一旦 owner 已以真实 retained port armed，normal acceptance
   的主判据是“真实 current state 到连续 viable port 的可达 witness”，而不是每周期重新
   要求 zero robust，亦不是把 reference 强制归零。可行 witness 可以是非零且随 phase
   连续变化的 port。
3. **Cert unavailable 的语义。**证书不可用、有限 DFS no-witness、domain 不足、未知
   clearance 或计算资源不足，均不得被写成 set empty。它们只会禁止 normal acceptance，
   并依 T4/T6 选择 fail-closed recovery；唯一可记录的“空”必须来自 T1 定义的完备 oracle
   证明。

T0 停止条件：若任何参与者要求把 zero robust 扩展成 armed normal owner 的通用 gate、要求
用现有 `CERTIFICATE_DENIED` 表示 `U=∅`，或需要一个新产品 mode/reason 来表达这些差异，
立即停止；不以代码填补政策空白。

### 3.2 Clearance provenance：统一语义，不统一数值

D0 必须固定每一个判断所读取的来源和语义，保持以下名称互不替代：

| 名称 | 当前证据中的语义 | 在 P1/P2 中的规则 |
| --- | --- | --- |
| `planning/safe_distance = .40` | B-spline planner cost 的配置语义。 | 仅记录 planner provenance；不是 tube certificate。 |
| residual effective radius = `.45` | immutable snapshot query 时已扣除 preincluded map uncertainty 的 tube 检查半径。 | 仅作为该 snapshot clearance contract 的输入。 |
| full tube radius = `.55` | UAV、map、localization、tracking 的完整 tube accounting。 | 仅作为完整 robust-tube 语义与报告字段。 |

`.40/.45/.55` 的“统一”是 provenance 与语义可追溯，不是令三者相等、调任何一个数值，或用
一个来源替代另一个。每份 P1/P2 测试记录都必须写入：owner identity、map/snapshot identity、
clearance source、preincluded accounting、full/residual 语义、所判定的 exact PWL domain。

T0 停止条件：若要使测试通过必须改上述值、margin composition、Filter 或 query source，停止
并回报为研究/安全合同变更。

### 3.3 Recovery owner 决策点

在 P2a 任何接线前，D0 必须选择且写明一个已有所有者：

- 若 current state 仍可由既有 FSM replan 合同处理，指定该 FSM 的现有 recovery entry，
  以及其对 unsafe/current start 的证明责任；或
- 若需要物理 recovery，指定已有 physical recovery owner、控制输入合同、合法起点和
  退出条件。

generic position HOLD 不能自动成为该 owner，也不能被称为碰撞恢复。old C2 prefix、旧
certificate 或 old tube 不能伪造 current-safe start。若未能从现有 owner 中作出有理论支持
的选择，则 P2b 无白名单、P2 不得继续。

### 3.4 同步例外

唯一可能的同步例外只限 P2a 的专用、进程内 recovery mailbox。它只携带已有 contract 所需
输入，不能改变 H2 handoff CAS，也不能产生可观察的新 enum/mode/reason/diagnostic。

T0 必须固定以下二选一实现并冻结其线性化点：

- producer/consumer 的 exactly-once CAS；或
- `std::mutex` 保护的单槽 mailbox 和不可重复消费的内部 ticket。

无论选择何者，FSM 是唯一 `exec_state` writer；adapter、timer、command callback 和
recovery producer 只能投递，不得直接写 FSM state。该内部同步不能成为新 gate/latch。

### 3.5 T0 最后决策（2026-08-16；只读）

最后 inventory 的结论是：**没有可复用的现有 physical recovery owner**。因此不得选择候选，
`P2b=STOP`，且 P2a 也不得接线或以 test double 以外的方式假装 production route 已存在。
这个决定适用于 `CERTIFICATE_DENIED`（即使 current actual point 仍 `KNOWN_FREE`）和
`CURRENT_OFFSET_OUTSIDE`；前者不是 `U=∅`，后者也不是自动等同硬碰撞。二者都必须阻断
`VEL_MATCH_GOVERNOR`，但在新 owner 获授权、给出 current-state dynamics/snapshot 合同前均不能
发起真实恢复。

T0 冻结的同步建议为 `std::mutex` 单槽 mailbox + 私有 monotonic ticket/consumed 标记，而非 CAS。
producer 的线性化点为 identity-bound request 入槽，FSM consumer 的线性化点为持锁验证并移出/标记
已消费；stale owner/session 只能零次消费，且 recovery work 必须在解锁后执行。详细候选、输入合同、
provenance/退出/线程、测试证据、两类拒绝的目标路由和未来最小架构边界见下文的“**T0 inventory
evidence（supporting record）**”。这不是产品授权。

## 4. 严格文件白名单与阶段依赖

所有下列列表都是**候选白名单而非当前授权**。没有一份按阶段另行签署的执行授权，任何
非 docs 写入都是 NOT AUTHORIZED。

| 阶段 | 可触及的唯一文件 | 前置依赖 | 不得触及 |
| --- | --- | --- | --- |
| D0 / T0 | `docs/Codex_P1_P2_Continuous_Viable_Port_Recovery_Execution_Plan_2026-08-16.md` 及未来明确列名的 D0 docs | P0/P0.5/audit evidence | 全部产品、测试、参数、launch、AGENTS。 |
| P1 / T1–T3 | `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h`；`src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp`；`src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp`；`src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`；`src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`；D0/P1 docs | T0 签核、理论 witness 定义、P1 单独授权 | Filter/cross-section/margins/H2/CAS/pin/session/runtime bit、manager、参数、CMake/launch、AGENTS。 |
| P2a / T4–T5 | `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`；`src/swarm_planner/bspline_traj/src/gvf_manager.cpp`；`src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`；D0/P2a docs；仅在 T0 选定时的专用 mailbox/mutex 定义 | P1 完整通过；T0 recovery owner 和同步选择已冻结；P2a 单独授权 | P1 以外 runtime/adapter 文件、Filter/cross-section/margins、H2/CAS/pin/session/runtime bit、产品 enum/mode/reason/diag/ROS param。 |
| P2b / T6 | **由 T0 所选 recovery owner 的单独、逐文件白名单定义**，外加对应 focused test 与 docs | P2a 通过；recovery route 的理论/安全合同和 P2b 单独授权 | 除 P2b 独立白名单外的任何文件；尤其不能借 manager 直接重写实际 recovery 行为。 |

跨 owner/map/profile/sample/witness 复用在所有阶段一律禁止。一个 pending 或 committed
witness 的 owner、path revision/generation/session、map sequence、snapshot 与 exact phase
domain 必须完整一致；任何不一致只能拒绝，不得 remap、pin 旧几何或 silent fallback。

## 5. P1：continuous viable port（T1–T3；NOT AUTHORIZED）

### T1 — Runtime exact-PWL viability/oracle

**目的。**在一个 immutable owner、profile、snapshot 与准确 current state 内，定义连续
viability oracle。令 `U(w)` 表示 exact-PWL bounds 与既有 physical/non-reverse/slew 约束
共同允许的 port 集；oracle 必须在所有 knot 间连续检查，而非只看 sample 或 DFS 节点。

**拟议实现边界。**Runtime 只可：

1. 从 exact current phase 开始，以同一 owner 的 exact-PWL bounds 构造连续 segment；
2. 从真实 retained delta、已提交 port 和既有速率/方向限制构造可达 connector；
3. 在 current phase 到下一个合法 replacement boundary（或已定义 terminal boundary）上
   给出连续 `delta*(w)` / executed-port witness；
4. 仅输出三种逻辑事实：有连续 witness、证书不可用、或在 oracle 已证明完备的明确 domain
   上可证明不可行。不得通过新增产品 reason/enum 表示它们；existing status 的 fail-closed
   含义保持不变。

有限 DFS 可保留为测试中的 counterexample finder 或性能诊断，但不能单独决定数学空集，也
不能是 armed normal owner 的最终 accept/reject oracle。T1 不改变 reference path、tube
生成、Filter、margin、snapshot 或 H2 事务。

**必备 unit fixtures 与观测字段。**

- knot 内收缩、crossed-knot、endpoint、nonzero initial port、速率受限 connector；
- bounds 含 zero 但有限 DFS no-witness：结果必须是“certificate unavailable”，不可记录
  `U=∅`；
- raw `+0.125` lower bound 的 P0.5 类型：已证明该 owner/current-domain 不可从 retained
  zero 连通，normal acceptance 必须拒绝；
- unknown/domain truncated/owner mismatch：fail-closed，且不写入任何 committed state；
- 每项记录 `owner_revision/generation/session`、`map_sequence/snapshot`、`current_w`、
  PWL domain、raw/filtered bounds、initial retained port、witness knot/segment coverage、
  certificate kind 与 clearance provenance。

**停止条件。**如连续验证必须读取 Filter 内部状态、改 margin、跨 map 查询、改 existing
runtime bit，或无法区分 unavailable 与 proved-infeasible，停止。不得用 sampling density 或
DFS depth 调参掩盖语义缺口。

### T2 — Adapter 100-cycle ownership / state-commit

**目的。**证明 T1 的计算不会在 owner 更换、candidate 拒绝或 callback interleave 中污染
live state，并在 100 个连续 command cycles 中保持单一 provenance。

**拟议合同。**Adapter 对每 cycle 先用 local candidate 运行 oracle，随后才可完成一次 normal
state commit；任何 reject、unavailable、map/profile/owner mismatch 都不得更新 retained port、
normal owner、active profile 或 last accepted witness。所有被发布的 normal port 必须可回溯到
同一 immutable owner/snapshot，不能拼接 old profile 和 new path，也不能复用 H2 old pair 的
tube/sample 几何。

**100-cycle fixture。**在 test-owned A/B owners、不同 revision/map/snapshot、accepted /
unavailable / denied / replacement 交错的 100 cycles 中，断言：

1. 每个 selected/valid normal output 的 `owner`, `profile`, `snapshot`, `phase domain` 一致；
2. 恰好一次提交或零次提交；拒绝周期对 live execution state 的差为零；
3. 连续 witness 覆盖本 cycle executed phase，没有 owner 或 map 混搭；
4. 所有 non-selected case 都携带既有 status 语义，尚不由 Adapter 发布 base fallback；
5. test 记录 commit attempt/commit result、cycle index、input/output provenance、retained
   port before/after 与 exact PWL coverage，而非增加产品 diagnostic。

**停止条件。**若需让 Adapter 直接写 FSM `exec_state`、扩展 H2 CAS/pin、添加 latch 或跨 owner
缓存来通过 100-cycle fixture，停止。

### T3 — Normal owner continuous-witness acceptance

**目的。**把 owner acceptance 从“current zero 恰好在 bounds 中”推广为“真实 current state
可达连续 viable port”。这是 planner/connector acceptance 的合同升级；本阶段不修改 base
planner，而是在 Runtime/Adapter owner 边界拒绝任何未证明可执行的 owner。

一个 normal owner 仅在以下全部为真时可被 accepted/selected：

1. T2 的 single-owner provenance 成立；
2. exact current state 可接入 witness 的第一段，且现有 port-rate、non-reverse 与 matched
   physical/internal port 合同都成立；
3. witness 在完整 required phase interval（至 next replacement boundary）连续落在 exact-PWL
   viable set 内，不只在 knots 内；
4. clearance 证据的 source/accounting 与 D0 一致；
5. 已 armed 的 owner 不因“zero 不在 bounds”被错误拒绝，也不因“zero 在 bounds”而绕过
   connector 证明；unarmed 的保守 zero 特例仅按 §3.1 处理。

**P1 通过条件。**所有 normal selected/valid ports 都有连续可复核 witness；P0.5 类型
owner 保持拒绝；“bounds contains zero + finite DFS no-witness”不会产生虚假的空集结论或
silent normal acceptance；100-cycle provenance/state-commit fixture 全部通过。

**T3 停止条件。**若只能靠再次引入 zero universal gate、放松 `CURRENT_OFFSET_OUTSIDE`、
变更 tube/Filter/margin 或把 certificate unavailable 当作 set empty 获得通过，P1 失败并停止。

## 6. P2：同周期 recovery（T4–T6；NOT AUTHORIZED）

P2 只能消费已有 Runtime/TubeEpoch/selected/valid 事实；不能重新定义、扩展或发布这些
状态。它的目标不是使不可执行 owner 变为正常，而是保证 rejected current-state case 不会
在同一 cycle 无声地回落至 legacy baseline governor。

### T4 — 同周期禁止 denied/outside 静默 baseline；调度已选 recovery

**目的。**当已有输入表明 `CERTIFICATE_DENIED`、`CURRENT_OFFSET_OUTSIDE` 或同等
non-selected/invalid current-state 拒绝时，command path 必须在**同一 cycle**把控制权交给
T0 选定的 recovery owner/request，而不是保留 `VEL_MATCH_GOVERNOR`。

P2a 只做 arbitration / dispatch：它不产生 recovery trajectory、不发明 HOLD、不把
non-selected 伪装为 selected，也不改变 H2。T6 的 route-specific owner 尚未实际可用时，
T4 只能用 test double 验证 dispatch 合同；production GO 必须等待 T6。

**必测时间线。**用 test-owned sequence 复现 §2.2：

```text
t0: bounds_contains_zero=true; finite_dfs=no_witness
    -> existing CERTIFICATE_DENIED; selected=false; valid=false
    -> recovery request selected in this same command cycle
    -> final_cmd_source != VEL_MATCH_GOVERNOR

t1: if baseline would have advanced into a single-sided PWL interval
    -> no silent baseline publication is permitted
    -> selected recovery route remains the only next action
```

对 P0/P0.5 的 actual outside fixture，`CURRENT_OFFSET_OUTSIDE` 也必须产生同周期 recovery
request；绝不能为了避免 dispatch 而安装 H2 candidate、复用旧 tube 或放宽 current check。

**停止条件。**若同周期封堵需要新增 mode/reason/gate/latch、修改 Runtime/Adapter 以外的
P2a 白名单、或替换 H2 CAS/commit 路径，停止。

### T5 — Thread-safe mailbox；FSM 为唯一 exec_state writer

**目的。**使 T4 的 request 在 timer/command/FSM 交错下 exactly once 地送达 selected owner，
而不允许 producer 直接写 FSM state。

实施只能采用 T0 冻结的专用 CAS 或 mutex 单槽 variant；若采用 mutex，ticket/consumed 标记
仅是私有同步数据，不是 product state/mode/reason/diagnostic。其线性化点、reset 条件、
owner/session binding、stale request discard 和 shutdown 行为都必须写入 test assertion。

**必备并发 tests。**至少覆盖两个 producer attempt、重复 command callback、FSM delay、
stale owner/session、normal acceptance 与 recovery request 竞争、shutdown；断言：

- FSM 是唯一 `exec_state` writer；
- 每个已接受 request 恰好消费一次，或在 owner/session 不一致时零次消费；
- 没有 duplicate recovery、deadlock、lost request、base-governor fallback 或 H2 authority
  mutation；
- `exec_state`、mailbox ticket、source owner/session、consume cycle 都由 test-only observation
  捕获，产品 schema 不增加字段。

**停止条件。**如需要新增线程、全局队列、ROS parameter、状态 enum，或把 mailbox 接到
H2 pin/CAS/session，停止。

### T0 inventory evidence（supporting record；2026-08-16；结论优先）

```text
INVENTORY_STATUS=COMPLETE_READ_ONLY
T0_DECISION=NO_EXISTING_REUSABLE_PHYSICAL_RECOVERY_OWNER
P2B_STATUS=STOP__NO_OWNER_SELECTED__NO_PRODUCT_AUTHORIZATION
```

**结论。**仓库中没有一个现有、已接入当前 `formation_planning` 实际控制链的 owner 同时满足：

1. 能从“current robust certificate 已失效、但 actual point 仍 `KNOWN_FREE`”的真实 current
   state 起步；
2. 对所执行的刹车 / hold / escape 轨迹使用同一 immutable owner、map snapshot 与 clearance
   provenance；
3. 有明确退出条件，且不会由旧 path、H2、base governor 或任意新 SO3 消息悄悄夺回控制；以及
4. 有覆盖上述入口的测试。

因此 T0 **不能**把下列任何候选写成“已选 / 已授权 recovery owner”。P2b 没有现有 owner
白名单，P2 在此停止。这个结论不把 `CERTIFICATE_DENIED` 推断为 `U=∅`，也不把
`CURRENT_OFFSET_OUTSIDE` 重命名为硬碰撞。

当前实际 normal command 链已核实为：

```text
formation_planning (AsyncSpinner(8))
  -> gvf_manager::cmdCallback (50 Hz; captures Runtime result)
  -> quadrotor_msgs/PositionCommand
  -> SO3ControlNodelet
  -> quadrotor_msgs/SO3Command
  -> single / multi SO3 simulator plant
```

来源为 `src/swarm_planner/bspline_traj/src/formation_planning.cpp:13-17`、
`src/swarm_planner/bspline_traj/src/gvf_manager.cpp:449-457,1358-1411,1538-1542,1606-1609`、
`src/uav_simulator/so3_control/src/so3_control_nodelet.cpp:96-119,136-153` 及
`src/uav_simulator/so3_quadrotor_simulator/src/multi_quadrotor_simulator_so3.cpp:177-201`。
尤其是，manager 只有在 `matched_output.selected` 时才替换 base guidance；non-selected 会继续
进入 governor。这正是 T4 要封堵的当前缺口，而不是现有 recovery route。

| 候选（活性） | 实际输入 / current-unsafe 起点 | provenance、退出、线程 owner | 现有测试 / T0 处置 |
| --- | --- | --- | --- |
| `gvf_manager::FSMCallback` 的 `REPLAN_TRAJ`（**活代码**） | 入口只来自 `checkCollision()` 的旧轨迹未来点或 `planInterval`（`gvf_manager.cpp:6342-6349`），不是 Runtime denial。非首次 `astaropt()` 从 `pm.last_traj[i0]` 而非 actual current state 开始（`:6058-6081`）；`checkCollision()` 也只读旧轨迹的未来 horizon（`:3983-4038`）。故不允许 current robust certificate 已失效的起点。 | 读取 mutable `pm.sdf_map_`，没有 immutable snapshot/tube provenance；规划失败同样回 `EXEC_TRAJ`（`:6804-6820`）。FSM timer 是 50 Hz、与 command 同处 `AsyncSpinner(8)`；FSM 虽是其自身 state writer，却没有同周期 command arbitration。 | `gvf_switch_policy_test` 测的是 switch/H2 policy，不覆盖 unsafe-current start、同周期封堵或 recovery。**拒绝为 owner。** |
| `makeGovernorInvalidHold()`（**活代码**） | 只由 `missing_gvf`、invalid guidance/path 或无 governor candidate 调用；它把目标直接设为当前 `pos`（`:729-741`），没有从 Runtime denial 的桥接。 | 没有 map/profile/snapshot 输入；下一次 valid governor 即退出。owner 是 command callback。 | `GvfAuthoritativePhaseCommit.HoldsForEveryGovernorInvalidReason` 只断言 phase 不提交（`gvf_switch_policy_test.cpp:679-694`），不证明到 hold 的制动轨迹安全。**generic HOLD，禁止称为 recovery。** |
| `closed_ref_recover_`（**活代码但默认关闭的 circle/figure-8 参考逻辑**） | 由 `closed_ref_enable_recover_` 与到 nominal reference 的距离进入（`gvf_manager.cpp:3837-3863`）；它重选 reference phase/goal，不接收 Runtime/tube failure。 | 无 immutable map/profile；距 reference 小于 recover radius 即退出；经 planner/FSM 继续同一 normal command 链。 | `gvf_switch_policy_test` 的 closed-goal tests 只覆盖 nominal goal ordering。**不是物理 recovery。** |
| `KinoPathCallback`（**活 timer，通常由 `use_kinopath_` 关闭**） | 首次取 odom、之后又取旧 kino path 最近点（`:4040-4081`）；仅发布 `/particle0/kinopath`，不控制 `/position_cmd`。 | 无 Runtime/tube snapshot；无 recovery entry/exit contract；timer 也是 AsyncSpinner callback。 | 无 recovery test。**不是 command owner。** |
| `PhaseOffsetAllocator::EMERGENCY`（**死代码 / test-only**） | 只是 `PortCommand.mode=EMERGENCY` 的数学返回（`phase_offset_allocator.cpp:287,325`）。该 `.cpp` 不在 `bspline_traj/CMakeLists.txt:51-107` 的 production library sources，也没有非测试调用。 | 没有 ROS、PositionCommand、SO3 或 FSM 调用链，故无 current-state/provenance/exit。 | 仅 `phase_offset_allocator_test.cpp:208-213` 断言 enum。**不得把 enum 当 actuator owner。** |
| `SO3ControlNodelet`、`motors` topic（**活低层 tracking controller，非 recovery**） | 接收任意 `PositionCommand` 并覆盖 `des_pos/des_vel/des_acc`（`so3_control_nodelet.cpp:96-119`）。收过一次 command 后，odom callback 会重发最后 target（`:136-146`）；所谓“stale command hover”只是一条 TODO（`:142-143`）。 | 无 ESDF/tube/snapshot/owner-session。`motors` 只改变 SO3 message 的 auxiliary bit（`:159-166`）；本 simulator 的 SO3 consumer 并不读取该 bit（`multi_quadrotor_simulator_so3.cpp:177-201`）。下一 PositionCommand 任意覆盖。 | 无 SO3 recovery/timeout unit test。**不是 mux/override，也不是安全 stop。** |
| multi SO3 simulator command-timeout hover（**活，但 simulator-only**） | 只有某 agent 已收过 SO3 command、随后超过 `command_timeout` 没有任何 SO3 command 才 `applyHoverFailsafe()`（`multi_quadrotor_simulator_so3.cpp:130-169,347-359`）。当前 SO3 nodelet 的 stale target 重发阻止此条件，因此不能由 P2 same-cycle 调用。single simulator 仅有 startup `start_at_hover`（`quadrotor_simulator_so3.cpp:226-249`），没有 timeout failsafe。 | hover 只使用 plant 当前 yaw/mass/gravity；没有 owner/map snapshot、tube 或 obstacle query。任意下一 SO3 command 立即清 `failsafe_active`（`multi...cpp:177-201`），没有 accepted-normal exit contract。 | `so3_quadrotor_simulator/test/multi_sim_tests.py:817-913` 覆盖“静默后 hover、下一 raw SO3 command 恢复”，不覆盖 planner/SO3-nodelet/ESDF/current-unsafe 情形。**不能复用。** |
| `drone_control/px4ctrl` 的 STOP/LAND/RETURN 与 `flight_manager`（**已编译的外部 PX4 支路，未接入当前 sim chain**） | `px4ctrl` 接收外部 `/control`；仓库没有该 topic publisher。cmd 5 把目标固定到 0.2 s 前的 odom，cmd 2 直接给 `z=-1`，cmd 3 只是 return flag（`px4ctrl.cpp:428-503`）；`flight_manager` 的 return/continue 只发布无人订阅的 `/goal_with_id`（`flight_manager.cpp:16-68`）。 | 无 Runtime/tube/snapshot/session；STOP 被任何非 5 command 取消，LAND/RETURN 均不证明 unsafe-current trajectory。其 ROS timer/`ros::spin()` 不属于 `formation_planning` 的 FSM。 | 无 tests；`px4ctrl.launch` 仅由旧 `gvf.launch` include，且其 `position_cmd` remap 不属于当前 phase-offset simulator launch。**不可作为 T0 owner。** |
| `control_keyboard` / `control_example` 与历史 `switch_emergency` 注释（**operator utility / 注释死代码**） | 前两者是人工或示例 PositionCommand publisher（`so3_control/src/control_keyboard.cpp:20-76`、`control_example.cpp:5-64`），没有自动 recovery contract；后者整段已注释（`gvf_manager.cpp:4845-5150`）。 | 无 snapshot、session、exit 或 arbitration；同时发布会与 manager 争抢同一 topic。 | 无 recovery tests。**明确排除。** |

#### 两类既有拒绝的推荐路由（推荐，不是授权）

| 既有输入事实 | 不得做的事 | 所需 route / 允许退出 |
| --- | --- | --- |
| `CERTIFICATE_DENIED`，且 current actual point 仍为 `KNOWN_FREE`、retained port 当前仍 inside | 不得把有限 DFS no-witness 写成 `U=∅`；不得继续 `VEL_MATCH_GOVERNOR`，也不得立即让 FSM 用 old-path replan 或 generic HOLD 冒充证明。 | P2a 应同 cycle 投递携带既有 immutable authority/snapshot 的 **certificate-unavailable recovery request**，以阻断 base governor。它随后只能由新的 route-specific current-state owner 完成“制动/短连接器仍安全”的证明；仅当新的 P1 continuous viable witness 与同一 live owner/session/snapshot 重新成立才退出 normal。当前没有该 owner，故此路由 **STOP**。 |
| `CURRENT_OFFSET_OUTSIDE`，包括 P0/P0.5 的 actual point `KNOWN_FREE` 但 residual robust certificate 已失效 | 不得安装 H2 candidate、复用 old tube/prefix 或放宽 current interval；不得把 raw current PWL 排除了 retained port 的事实交回 baseline governor。 | P2a 应同 cycle 投递 **unsafe-current recovery request**。route-specific owner 必须以真实 current position/velocity 为起点，在同一 immutable snapshot 下证明有限制动/escape/connector；只有 fresh P1 normal witness 才可退出。现有 FSM、HOLD、SO3 timeout、PX4 STOP/LAND 均不满足，故此路由同样 **STOP**。 |

#### 同步选择：`std::mutex` 单槽 mailbox（推荐，不是授权）

T0 的同步建议是 **`std::mutex` 保护的单槽 mailbox + 私有单调 ticket/consumed 标记**，而不是
CAS。理由和冻结要求如下：

- 当前 `formation_planning` 使用 `AsyncSpinner(8)`，而 command 与 FSM timer 均为 50 Hz；并非
  高频无锁数据面。已有 H2 mailbox 已采用 mutex（`gvf_manager.h:369-386`）并有单槽/session
  tests（`gvf_switch_policy_test.cpp:834-904`）。Runtime 本身也以
  `runtime_command_mutex_` 串行（`phase_offset_matched_adapter.h:671-716`）。
- recovery request 必须作为一个整体带上 pair/path owner、revision/generation、authority session、
  map sequence、immutable snapshot、actual state、retained port、既有 Runtime/epoch status 与
  producing command cycle。CAS 只能安全交换 immutable heap payload 指针，仍须另行解决 claim、
  ABA、ticket、stale discard 与内存序；在此频率下只增加 exactly-once 测试矩阵。
- mutex variant 的 producer linearization point 是“在 slot 为空或为同一 identity 的幂等重试时，
  写入 request 并分配/保留 ticket”；FSM consumer linearization point 是“持锁验证 identity/session，
  将 slot 移出并标为 consumed，随后**解锁**才做 recovery work”。owner/session 不匹配的 request
  只可零次消费；不得被新 owner remap。所有 producer（command、任何未来 timer/adapter）只能
  投递；FSM 仍是唯一 `exec_state` writer，且 mailbox mutex 不得与 H2 pin/CAS/session lock
  嵌套。
- 现有 adapter 已有 command/timer 并发 stress test
  （`phase_offset_matched_adapter_test.cpp:2574-2624`），但它不测试 recovery mailbox。未来 focused
  test 必须额外覆盖：两个 producer attempt、重复 command、FSM delay、normal/recovery 竞争、
  stale owner/session、shutdown，以及 exactly once 或 stale zero-consume。此项不授权实现。

#### STOP 后的最小架构边界（未来独立 stage；不属于 P2b 当前白名单）

若后续仍要求真正的 physical recovery，最小合理边界不是复用上表任一候选，而是先签署一份
新的 recovery-architecture 规格，并至少显式列出：

```text
docs/<new-recovery-architecture-and-dynamic-contract>.md
src/swarm_planner/phase_offset/phase_offset_navigation/
  include/phase_offset_navigation/current_state_recovery_owner.h       (new)
  src/current_state_recovery_owner.cpp                                 (new)
  test/current_state_recovery_owner_test.cpp                           (new)
  CMakeLists.txt                                                       (target/test registration)
src/swarm_planner/bspline_traj/
  include/bspline_race/gvf_manager.h
  src/gvf_manager.cpp
  test/gvf_switch_policy_test.cpp
  CMakeLists.txt                                                       (link registration if required)
```

新 owner 必须把 actual pose/velocity、已存在 Runtime/epoch facts 和**同一** immutable snapshot
作为显式输入，并输出经过动态/clearance 证明的 recovery command sequence；manager 只做
same-cycle arbitration、mailbox 与唯一 command publication。若多周期执行需要新的 internal
owner state/transition，这本身就是对本单 §2.3、§3.3、§6 禁止新增 state/mode 的架构变更，必须
先修订本正式单并获得新授权，不能伪装为 P2a 接线。除非新规格另外证明当前 PositionCommand→SO3
单 publisher 合同已足够，否则 SO3 mux/watchdog、simulator 或真实飞控接口的改动也必须被单独
列入，不能隐含加入。

### T6 — Route-specific recovery owner 与真实动态/H2 验收

T6 只能在 P2b 的独立白名单、所选 recovery owner 的安全合同和对应 unit/integration tests
明确后实施。它必须给出对 current unsafe start 有效的 route-specific behavior；generic HOLD、
old C2 prefix、旧 certificate 和重试 H2 都不是替代品。

真实动态验收必须使用 task-owned private ROS master、唯一 ROS_HOME、独立端口和 task-owned
process cleanup；不得接入、终止或影响用户 ROS/process。它至少要在真实 ESDF 单侧 tube 中
证明：

1. 正常场景的 selected/valid port 持续来自连续 witness；
2. phase 越过 old path end，并发生真实 replacement H2；H2 identity/pin/CAS 原合同未变；
3. §2.2 的 denied/outside 注入在同 cycle 进入已选 recovery，而非 `VEL_MATCH_GOVERNOR`；
4. 不存在 silent baseline fallback、跨 owner/map reuse、或伪 normal selection；
5. negative fixtures（unknown、domain不足、证书不可用、raw current interval 真正排除 retained
   port、stale mailbox）全部 fail-closed，并且不把 certificate unavailable 报成 set empty。

**P2 通过条件。**T4 同周期 arbitration、T5 exactly-once ownership、T6 route-specific
recovery 与真实动态/H2 验收均通过。任一缺失即 P2 未通过，系统不得 GO。

**T6 停止条件。**若所选 owner 不能对 current unsafe start 给出 route-specific 安全合同，
若真实 ESDF/H2 运行需要调参、改 launch 或触及 P2b 白名单外文件，若任一 negative fixture
仍发布 baseline，或若 H2 identity/pin/CAS 被改变，立即停止；不得以 generic HOLD、重试或
跨 owner 复用取代 recovery。

## 7. 验证命令与必须保存的字段（未来授权后）

本节是未来执行的命令/证据模板，不是当前执行命令。当前文档阶段不运行 build、ROS 或产品
测试。

### 7.1 静态范围与构建/单测

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git diff --name-only
git diff --check
catkin_make
ctest --test-dir build --output-on-failure \
  -R '^(phase_offset_runtime_test|phase_offset_matched_adapter_test|gvf_switch_policy_test)$'
rg -n 'TubeFilter|cross-section|RobustTubeMargins|safe_distance|search/margin|H2|CAS|pin|session|runtime_bit' \
  src/swarm_planner/phase_offset/phase_offset_navigation \
  src/swarm_planner/bspline_traj
git status --short
```

每阶段必须将 `git diff --name-only` 与该阶段白名单逐项比较；T1–T3 只能修改 P1 行所列文件，
T4–T5 只能修改 P2a 行所列文件。若 existing build/test harness 使用不同的 target invocation，
只可在 D0 文档中替换为其等价的既有命令，不可趁机改 CMake、launch 或参数。

### 7.2 动态验收命令形态

```bash
export TASK_ROOT=/tmp/p1_p2_current_state_$(date +%Y%m%d_%H%M%S)
export ROS_HOME="$TASK_ROOT/ros_home"
export ROS_MASTER_URI="http://127.0.0.1:<task-owned-free-port>"
mkdir -p "$ROS_HOME"
roscore >"$TASK_ROOT/roscore.log" 2>&1 &
roslaunch <D0-frozen-existing-launch-and-unchanged-arguments> \
  >"$TASK_ROOT/roslaunch.log" 2>&1
```

`<task-owned-free-port>` 与 `<D0-frozen-existing-launch-and-unchanged-arguments>` 必须在 T0
以 read-only inventory 固定；不得通过新 launch/param 调整制造 acceptance。测试结束只清理
本任务启动的 process，并保存 pre/post process、port、loaded-library map、build-id、SHA256、
GDB/test transcript 和 shutdown trace。

### 7.3 每次动态/100-cycle 记录字段

```text
cycle / monotonic time / command-cycle identity
old and candidate owner revision,generation,session,path domain
map sequence / immutable snapshot identity / profile source-revision-domain
current w / actual state / retained and executed port before-after
raw and filtered exact-PWL bounds / contains-zero
oracle domain / continuous witness coverage / certificate kind
finite DFS result (if run) -- explicitly non-equivalent to U empty
clearance source / preincluded accounting / .40,.45,.55 semantic labels
selected / valid / existing runtime and tube-epoch status
final_cmd_source / recovery owner / request ticket / consume cycle
FSM exec_state writer identity / mailbox exactly-once outcome
H2 old-pair pin / future seam / final CAS result (observation only)
```

禁止为收集这些字段增加产品 diagnostic；可使用 test assertion、private GDB、private ROS
logs 或 test-owned trace。

## 8. 总停止规则与交付物

立即停止并回报，而不是扩大范围，若发生任一情况：

- P1/P2 的理论选择仍不明确，或 T0 未选 recovery owner/同步机制；
- witness/recovery 需要改 Filter、cross-section、margins、A5 geometry、H2、CAS、pin、
  session、runtime bit、参数、launch、AGENTS 或不在白名单的文件；
- 要靠 new enum/mode/reason/diagnostic/gate/latch 表达合同；
- 需要跨 owner/map/snapshot 复用几何或用旧 certificate 覆盖 current state；
- negative case 仍发布 `VEL_MATCH_GOVERNOR`，或 certificate unavailable 被记录为 `U=∅`；
- P1 或 P2 任一验收未通过。

阶段交付物只能是：按授权的 focused tests、动态证据根目录、白名单/差异审计、`git diff --check`
结果、`git status --short`、written self-audit 与明确 stop/GO 判定。只有 P1 与 P2 都通过，
才可由新的独立授权考虑 GO；本单本身不作 GO。

## 9. 当前停止点

本正式单只新增本 docs 文件。当前没有产品代码、测试、参数、launch、CMake 或 `AGENTS.md`
修改；所有后续计划性修改保持 **NOT AUTHORIZED**。下一动作是对 T0 的理论路线、recovery
owner 和同步 variant 作明确授权决策，而不是继续 A6/H2 修补。
