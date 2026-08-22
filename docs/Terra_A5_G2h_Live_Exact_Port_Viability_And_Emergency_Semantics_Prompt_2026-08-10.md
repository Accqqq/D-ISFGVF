# A5-G2h：Live Exact-Port 连续可行性与 Emergency 语义修订规范

## 0. 授权与停止边界

本执行单只授权 **A5-G2h**。目标是修复已经通过连续 tube 几何验证后的高率
执行层：正确区分 WAITING、显式地图 UNSAFE、`U+` 不可行、`U>=0` 不可行，
并在当前 tube 状态可执行时选择一条满足 slew 和 exact next-envelope 的连续端口
witness，避免“单帧 selected / 多帧 emergency”抖动。

`AUTO_ADVANCE=false`。完成实现、构建、单测、隔离 ROS 验收、自审核和报告后
立即停止。

本阶段禁止：

- 修改 A5-G2g 已通过的 PointCloud2 clearance、cross-section、surface validator；
- 修改 residual/full margin `0.45/0.55`；
- 让普通单机 `delta=0` 自动侧移；
- 修改 planner/Kino/B-spline optimizer、地图、目标、速度或 saturation 参数；
- 修改 `gvf_manager.cpp`、`gvf.cpp`、原 C2 connector、phase revision/A6；
- 新增普通 Runtime gate/mode/profile 副本；
- 通过隐藏 Marker/diagnostics/emergency 或放宽安全约束让测试通过；
- 进入 A6、A5.3、多机、CBF。

如果证据证明剩余不可执行主要来自 base path 的 `delta=0` clearance mismatch，
或必须修改 planner/A6 才能运行，按事实停止并报告；不得越权。

## 1. 执行前置条件

开始前只读核对并记录：

- branch=`main`；
- HEAD=`9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`；
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` 存在且不得
  restore/pop；
- staged 为空；
- `git status --short`、tracked diff、白名单内 untracked 摘要；
- 当前 dirty worktree 全部视为用户内容；
- 不连接、复用或终止用户 11311 ROS；
- ROS 使用新隔离端口和独立 `ROS_HOME`。

任一前置异常时停止报告，不得 reset/restore/clean/stash/commit/branch/tag/push。

## 2. 已完成且不得回退的事实

### A5-G2g 连续几何

证据根目录：

`/tmp/a5_g2g_acceptance_20260810/`

已确认：

- immutable occupied-voxel-volume Euclidean clearance；
- direct center-clearance cross-section；
- adaptive lifted-state sampling；
- continuous ribbon surface validation；
- exact snapshot audit 41/41 due Candidate ADD 通过；
- min voxel-volume clearance=`0.4668870179 m`，要求=`0.45 m`；
- 旧 bag 反例=`0.1652957454 m`；
- 普通单机 retained delta/intent 不自动侧移；
- fixed 325/325 Candidate ADD、0 emergency；
- 666 tests、0 failures/errors。

不得修改或弱化这些实现、测试和证据。

## 3. 当前 117 帧 emergency 不能混为一个问题

正式 ROS-2 summary：

`/tmp/a5_g2g_acceptance_20260810/esdf/evidence/esdf_summary.json`

总计 172 epoch frames：

- Runtime emergency=117；
- selected=2；
- fatal/latch=0；
- retained outside=118；
- Candidate ADD/DELETE=167/5；
- Certified ADD/DELETE=2/170。

117 帧 context 已分为：

```text
73 frames: epoch WAITING / CURRENT_OFFSET_OUTSIDE,
           active_current_validation=0, retained_inside=0

15 frames: epoch EMERGENCY / ACTUAL_CLEARANCE_INSUFFICIENT
           （显式 current map unsafe）

4 frames : WAITING / FORWARD_HORIZON_SHORT,
           active_current_validation=1, retained_inside=1

25 frames: ROLLING / reason=NONE,
           active_current_validation=1, retained_inside=1,
           dynamic_class=INFEASIBLE
```

此外有 54 帧 candidate/active revision mismatch、42 个 emergency 与 mismatch
重合。A6 事件必须单独标记，不能在 A5 添加 revision gate/workaround。

因此本阶段不得以“把 117 全部变成 selected”为验收目标：

- `delta=0` 不在最新安全 corridor 时，正确语义是 WAITING，不是自动侧移；
- actual/reference 明确 unsafe 时，正确语义仍是 emergency；
- path revision mismatch 若导致旧 profile 域/参考不连续，属于 A6；
- A5-G2h 只应消除错误分类和在 current admissible、revision-compatible 上下文中的
  假性 exact-port 空集/贪心失效。

## 4. 第一阶段必须先做只读失败分类

在修改选择算法前，必须让每个 live projection attempt 能回答：

```text
U+ core polygon 是否为空？为什么？
U+ 选中的 port 是否仅因 exact next-envelope 失败？
U>=0 core polygon 是否为空？为什么？
U>=0 选中的 port 是否仅因 exact next-envelope 失败？
profile next_w 是否越域？
current bounds/inside/map/geometry 是否有效？
previous_final slew box 是什么？
哪个 half-plane/constraint 形成冲突？
是否存在另一个 current polygon 内 port 能通过 exact next-envelope？
```

不得继续把所有失败压成字符串：

`"current U+ and U>=0 port sets are empty"`

因为当前 `project_exact()` 的 false 同时可能表示：

1. core PortProjector polygon 真空；
2. core projection 成功，但唯一选中的 nearest-to-raw point 不在 exact next bounds；
3. `next_w` 超出 active profile；
4. next bounds query indeterminate；
5. exact next delta outside；
6. 输入/geometry/math invalid。

只有第 1 类在对整个可行集合证明后，才能称为 port set empty。

## 5. PortProjector focused diagnostics

### 5.1 不改变 approved half-plane 公式

继续复用唯一 `phase_offset_core::PortProjector` 内核。不得复制以下公式到 Runtime
或 ROS adapter：

- amplitude box；
- slew box；
- phase/tangent progress；
- regularity；
- current offset-step；
- upper/lower invariant half-plane。

允许在 pure core 中新增 focused result/diagnostics：

- failure stage enum；
- each scalar interval；
- active/infeasible constraint labels；
- feasible polygon vertex/edge representation；
- exact projection cost/witness；
- deterministic constraint residuals。

如果暴露 feasible polygon，必须由 PortProjector 自己构造并返回，navigation 不得
重新实现 half-plane intersection。

### 5.2 真实集合 vs 单点投影

当前 `PortProjector::project` 只返回离 raw 最近的一个 port。随后 Runtime 查询：

\[
w^+=w+dt(\dot w_{base}+u_w),
\qquad
\delta^+=\delta+dt\,u_\delta
\]

并检查 `delta+` 是否在 exact `bounds(w+)` 内。

若这个单点失败，不代表整个 current feasible polygon 与 exact next envelope 的交集
为空。本阶段必须建立确定性的 **exact tube port selector**，在同一个 core polygon
上寻找满足 exact next envelope 的 port。

## 6. ExactTubePortSelector

建议新增 navigation 单一职责模块：

```text
ExactTubePortSelector
```

输入：

- core PortProjector feasible polygon/constraints；
- current raw intent；
- current `w,delta`；
- current path geometry/base guidance；
- active continuous TubeProfile；
- dt；
- U+ 或 U>=0 limits；
- previous final port。

输出：

- feasible/infeasible/indeterminate；
- selected port；
- exact next `w,delta,bounds`；
- deterministic witness；
- failure stage/reason；
- minimum constraint residual。

### 6.1 Exact constraint

selector 必须直接验证：

\[
\underline\delta(w^+)+m
\le \delta^+
\le \overline\delta(w^+)-m.
\]

这对 `u_w,u_delta` 通常不是线性 half-plane，不能简单把 nearest point 失败当成
集合空。

### 6.2 Deterministic bounded search

由于 port 是二维，允许使用确定性有界算法：

- 检查 raw projection、polygon vertices；
- 检查 polygon edges 上 exact-envelope residual 的根/极值；
- 必要时对 edges 做有界 subdivision/bisection；
- 使用 TubeProfile 连续 bounds query；
- 对所有候选使用统一 cost 和 deterministic tie-break；
- max subdivisions/queries 必须固定并 fail closed；
- 禁止引入通用 QP/NLP solver 或 ROS dependency。

若采用同等严格的解析/interval 方法也可，但必须证明不会漏掉 polygon 内存在的
exact-next feasible port。

### 6.3 选择目标

在 exact feasible set 非空时：

- NORMAL/U+：选择离 manual raw intent 最近的可行 port；
- RECOVERY/U>=0：选择能够回到 `delta_ref=0`、并具有最大 future viability 的
  deterministic port；
- 不允许通过选择 port 改变正常单机的 delta intent；
- matched physical/internal channel 使用同一个 final port。

## 7. Slew-continuous short-horizon viability

单周期 exact feasible 不保证下一周期仍有解。现有 dynamic rollout 已认识到 slew
连续性，但当前实现仍有局限：

- 使用一个 frozen base-motion fact 覆盖整个 horizon；
- 每步贪心调用单点 PortProjector；
- positive/nonnegative classification 可能有 current witness，但后续贪心失败；
- Runtime 只把 current nonnegative witness 当 advisory；
- live current facts 与 install-time result 可能不同。

### 7.1 本阶段目标

在每个 live admissible cycle，用当前 base guidance 和 active profile 做一个短、确定性
receding viability search：

1. 先求连续可达的 U+ witness；
2. U+ 无连续 witness 时求 U>=0 witness；
3. 使用 witness 的第一个 port；
4. 下一周期用新 live facts 重算；
5. 两者集合均被严格证明为空时才 emergency。

这不是新的 gate，而是当前控制输出的 selector。

### 7.2 Horizon

搜索 horizon 必须覆盖至少：

- 当前到下一 tube refresh 的时间/phase 进度；
- slew 从当前 previous port 进入 recovery 所需的最短步数；
- active profile 的剩余 certified segment。

不得新增 launch 调参。使用已有：

- dt；
- tube update period；
- min certified forward w；
- u_w/u_delta rate limits；
- profile bounds。

### 7.3 不得贪心误判

若一个 first port 虽然更接近 raw intent，但会让下一步 slew box 与 tube empty，而
另一个 port 可以保持连续 viability，必须选择后者。

允许对 feasible polygon 的少量 deterministic candidates 做 bounded tree/DP 搜索，
但：

- 不复制 core constraints；
- 不创建 persistent recovery profile；
- 不创建新 Runtime mode；
- 不把 rollout 变成 Candidate/Active/Marker hard gate；
- 超限 fail closed 并明确 diagnostics。

## 8. CBF/invariant 与 exact next-envelope 重复约束审查

用户明确要求不要继续叠加 gate。本阶段必须审查：

- current offset-step box；
- upper/lower invariant half-plane；
- exact next-envelope；
- multi-step viability；

是否存在重复/矛盾导致假性空集。

如果证据证明：

- current/next exact envelope 有连续 witness；
- multi-step selector 能保持 tube；
- 但 invariant half-plane 单独造成 polygon empty；

则允许将该 invariant 从 hard execution plane 降为 diagnostics **仅在有严格 exact
current+next+short-horizon witness 替代时**。不得直接删除后不补安全证明。

主 proposal 中 CBF 不是 matched-port 前提。最终 hard execution constraints 应保持
最小集合：

- amplitude/slew；
- non-reversing phase/tangent；
- regularity；
- exact current/next tube；
- slew-continuous short-horizon viability。

任何 invariant 语义改变必须：

- focused unit test；
- before/after failure classification；
- 无越界 trajectory；
- fixed/A4 zero-port regression；
- 在报告中明确说明。

## 9. Runtime 最小状态语义

控制 arming 后只保留：

```text
NORMAL: exact slew-continuous U+ witness
SAFETY_PRIORITY: exact slew-continuous U>=0 witness
EMERGENCY: active/current map/geometry admissible，但 U>=0 集合严格为空；
           或最新明确 map unsafe
FATAL: math/software invariant contradiction
```

兼容 enum 可保留，但不得增加新 mode/gate。

### WAITING

以下应为 WAITING/uncertified、selected=false，而不是 live exact-port emergency：

- Candidate/active 不存在；
- current retained delta 不在最新 safe corridor；
- active current validation 已撤销但没有明确 map unsafe；
- forward horizon 不足且当前没有 authoritative active domain；
- latest evidence UNKNOWN/UNAVAILABLE；
- A6 revision mismatch 导致 profile ownership 不对应。

WAITING 不得改变 delta/previous final port，不得自动侧移。

### Explicit map unsafe

actual/reference OCCUPIED、OUT_OF_MAP 或 clearance insufficient 仍可请求 emergency。
不得隐藏。

## 10. Gate 与 state commit

100-cycle gate 仍只决定是否接管：

```text
selected = gate_open && exact_witness_valid && !fatal
```

gate 前允许 shadow 计算/发布 diagnostics/Certified，但：

- 不更新 delta；
- 不更新 previous_final_port；
- 不推进 manual profile elapsed；
- 不发布 active emergency command；
- 不将 pre-gate infeasible 统计为执行 emergency。

## 11. A6 revision 隔离

本阶段不得修改：

- path revision identity；
- C2 switch/install；
- phase clamp/advance；
- old/new path continuation；
- active tube reparameterization across path update。

所有 ROS 分析必须把 frames 分为：

```text
revision-compatible
revision-mismatch/A6
```

A5-G2h 验收只对 revision-compatible 且 current admissible frames 宣称 exact-port
修复。A6 frames 原样报告，禁止 workaround。

## 12. Base-path delta=0 mismatch 隔离

A5-G2g 已证明部分 safe geometric component 不包含 0。普通单机不能自动侧移。

因此：

- retained delta outside -> WAITING；
- Candidate 可显示；
- Active/Certified 不得宣称有效；
- 不要求 selected；
- 不计入“admissible exact-port false emergency”验收分母；
- 记录为未来 planner/tube clearance integration blocker。

本阶段不得修改 planner 或把 manual profile 当作自动避障器。

## 13. 文件白名单

Pure core：

```text
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_types.h
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_projector.h
src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/port_projector_test.cpp
```

Navigation：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_dynamic_feasibility.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_dynamic_feasibility.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_dynamic_feasibility_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
```

允许新增单一职责模块：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/exact_tube_port_selector.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/exact_tube_port_selector.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/exact_tube_port_selector_test.cpp
```

Integration/diagnostics：

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_epoch_diagnostics.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_epoch_diagnostics.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_diagnostics_test.cpp
src/swarm_planner/bspline_traj/CMakeLists.txt
```

允许新增最多一个独立 exact-port diagnostics header/source/test/topic。旧 manual=83、
epoch=60、raw=49、cloud=18 索引不得移动。优先独立 schema，必须 static_assert 和
严格列数测试。

禁止修改：

- G2g cloud snapshot/clearance/cross-section/builder/filter/surface validator；
- `gvf_manager.*`、`gvf.*`；
- planner/Kino/B-spline optimizer；
- C2/governor/SO3/simulator；
- launch 参数/默认值；
- margins、速度、amplitude/rate/abs limits；
- RViz、messages、swarm、CBF。

若必须修改白名单外文件，停止报告，不得扩权。

## 14. 必须新增的单元测试

### PortProjector diagnostics/set

1. core polygon truly empty 的具体原因；
2. polygon nonempty、nearest raw point exact-next 失败，但另一个 polygon point
   成功；
3. deterministic polygon vertices/edges/tie-break；
4. amplitude/slew/progress/regularity/invariant residual；
5. no duplicated half-plane formula；
6. disabled A4 exact equivalence 保持。

### ExactTubePortSelector

1. nonlinear next bounds 下 interior feasible point；
2. only edge root feasible；
3. U+ empty、U>=0 feasible；
4. both strictly empty；
5. next_w profile end/out-of-domain；
6. current/next query indeterminate；
7. upper/lower narrowing corridor；
8. slew prevents immediate raw but alternative viable；
9. deterministic bounded subdivision；
10. no solution missed in analytic fixtures；
11. selected final port exact next inside；
12. same final port passed to matched channel。

### Short-horizon viability

1. greedy nearest port leads to next-step empty，alternative first port survives；
2. U+ future-infeasible -> early U>=0 recovery；
3. U>=0 legal hold/recenter；
4. frozen install facts differ but live re-evaluation succeeds；
5. profile horizon short -> early safe slowdown/recovery or correct WAITING；
6. slew history propagated exactly；
7. max search limit fail closed；
8. no persistent hidden profile/state；
9. revision mismatch excluded/diagnosed；
10. current delta outside -> WAITING, not auto-side-step。

### Runtime/adapter

1. gate closed does not commit state；
2. admissible U+ witness -> NORMAL/selected after gate；
3. admissible U>=0 witness -> SAFETY_PRIORITY/selected after gate；
4. strict U>=0 empty -> EMERGENCY；
5. retained outside/validation false -> WAITING, not emergency；
6. explicit map unsafe -> emergency；
7. Candidate/Certified semantics unchanged；
8. no fatal/latch for transient geometry/map/port；
9. matched residual/same-final-port；
10. diagnostics exact reason and schema。

不得删除或弱化现有测试。

## 15. 构建与回归

依次执行：

1. `catkin_make -j8`；
2. core PortProjector tests；
3. exact selector/dynamic/runtime/epoch tests；
4. G2g geometry全部回归；
5. adapter/epoch/marker/cloud 回归；
6. fixed/manual/zero-port/matched 回归；
7. `catkin_test_results --verbose`；
8. dependency-boundary search；
9. `git diff --check`；
10. staged/status/whitelist audit。

历史 `uav_utils -luav_utils` 只报告，不得修。

## 16. 隔离 ROS 验收

### ROS-0 baseline

- disabled/none/refresh=3.0；
- no manual tube publishers；
- sole `/position_cmd` publisher；
- goal reached。

### ROS-1 fixed

- Candidate continuous ADD；
- gate 后 selected 连续；
- 0 emergency/latch/fatal；
- exact selector 不改变 fixed zero-port/matched 行为。

### ROS-2 ESDF/cloud

使用与 G2g 完全相同的参数、pillar、目标、速度、margin、cloud contract，不调参。
保存 bag/CSV/JSON/analyzer/SHA-256。

分析必须按以下互斥类别统计：

```text
A. explicit current map unsafe
B. retained delta outside / active validation false
C. forward horizon/profile unavailable
D. revision mismatch / A6
E. revision-compatible + map safe + active/current valid + retained inside
```

验收重点是 E 类：

- 每帧记录 U+ core polygon、exact selector、short-horizon witness；
- 若 U+ 有连续 witness -> NORMAL selected；
- 否则若 U>=0 有连续 witness -> SAFETY_PRIORITY selected；
- emergency 仅允许在独立 verifier 也证明 U>=0 exact set/viability empty；
- 不允许 nearest-point false negative；
- 不允许 selected 单帧高频抖动；
- final port slew、amplitude、nonreverse、regularity、current/next tube 全部满足；
- same final port/matched residual 合格；
- no latch/fatal。

A/B/C/D 类：

- 按正确语义 WAITING/emergency/blocked 分类；
- 不自动侧移；
- 不作为 E 类 false emergency；
- D 类不得修改 A6。

G2g clearance 继续严格通过：

- Candidate ADD surface min clearance >=0.45-epsilon；
- exact snapshot match；
- no raw-storage/self-free；
- margins unchanged。

若 E 类没有足够帧形成可核查窗口，必须另选 **同一原 pillar 地图、同一正式目标
流程中自然出现的 eligible window**；不得换地图/目标/参数。若真实路径长期 B/D，
停止报告 planner/A6 blocker。

## 17. 最终报告与停止

报告必须给出：

- 实际修改文件和职责；
- 117 帧 before 分类和 after 分类；
- core polygon empty vs nearest-point exact failure 数量；
- selector/viability 算法与不会漏解的依据；
- invariant hard-plane 是否变化及安全替代证明；
- E 类 selected/emergency 连续窗口；
- A/B/C/D 正确语义统计；
- G2g clearance 回归；
- fixed/baseline 回归；
- tests、dependency、diff/status/staged；
- 无 commit/tag/push/stash；
- 未修改白名单外文件；
- 明确停止：未进入 planner integration、A6、A5.3、多机。

只有当 revision-compatible、map-safe、active/current-valid、retained-inside 的 E 类
不存在假性 live emergency，A5-G2h 才能标记完成。
