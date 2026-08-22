# A5-G2e-r1：最小 Runtime 状态语义与可恢复执行修订规范

执行模型：`gpt-5.6-terra`

推理强度：`max`

日期：2026-08-10

`AUTO_ADVANCE=false`

本执行单只授权 A5-G2e-r1。目标是对当前 G2e 做减法修订，不得继续叠加 gate、状态或参数。完成实现、构建、单测、隔离 ROS 验收、自审核和报告后必须停止。禁止进入 A5.3、A6 或多机阶段。

---

## 1. 当前事实与本阶段结论

G2e 已修复：

- 动态不可行 candidate 不再先安装后触发 permanent latch；
- `U+`、`U>=0`、emergency、genuine fatal 已初步分开；
- transient tracking/map rejection 不再直接 permanent latch；
- ESDF ROS 中 `failure_latched=0`。

但当前实现仍不合格，且继续增加 gate 会使系统更难实际运行。

固定 tube 正式证据：

```text
/tmp/a5_g2e_ros1c/evidence/fixed.bag
/tmp/a5_g2e_ros1c/evidence/fixed.json
```

29 帧 emergency 的精确构成：

1. 24 帧发生在现有 100-cycle gate 尚未打开时；phase-offset 没有接管控制，却已被 tracking monitor 标为 active emergency；
2. 2 帧发生在 path revision 2 -> 3 后，tracking 在约 1 ms 内 `0.016 -> 0.176 m`；
3. 3 帧发生在 path revision 3 -> 4 后，tracking 在约 1 ms 内 `0.077 -> 0.189 m`。

后两次不是 UAV 物理跟踪在 1 ms 内突然恶化，而是 path/reference update continuation 尚未在 A6 验证。A5 不得通过新增 gate、调大 tracking bound 或修改路径代码掩盖它。

当前动态检查还有两个缺口：

- `U>=0` 只逐 knot 做局部 probe，没有构造连续可达 witness，可能 false positive；
- horizon certificate 使用安装时冻结的 base-motion facts，Runtime 状态变化后只在 projector 失败时才发现 certificate 过期。

本阶段唯一目标：

> 将 G2e 简化为一个现有一次性 control-arming gate、candidate/active 两个 tube ownership、以及每周期 `U+ / U>=0 / emergency` 三类执行结果；tracking 短暂超界只撤销安全显示证书并进入可恢复的非反向回中控制，不再直接退出控制或 permanent latch。

---

## 2. 不增加新门控的硬约束

禁止新增：

- `PREARM` 状态；
- 新 readiness counter；
- 新 debounce/hysteresis 参数；
- 新 consecutive-cycle gate；
- 新 map-ready/display-ready/control-ready 布尔链；
- 新 tracking threshold；
- 新 launch feature flag；
- 新 recovery timeout；
- 新 certificate age 参数。

现有 100-cycle zero-port equivalence gate 保留，但职责必须缩减为：

```text
control_armed = zero_gate_open && !genuine_failure_latched
```

它只决定 phase-offset output 是否允许接管原 guidance。

它不得决定：

- candidate 是否构建；
- tube 是否几何/地图有效；
- dynamic feasibility 是否计算；
- gate 关闭期间是否 emergency；
- Candidate Marker 是否显示；
- map observation 是否可用。

gate 关闭时允许 shadow 构建和诊断，但必须满足：

```text
selected = false
active emergency output = false
failure_latched = false
delta and previous final port unchanged
```

不需要新增 PREARM enum；直接由现有 `zero_gate_open`/selected 表达未接管状态。

---

## 3. 最小 ownership

只保留两个 tube 对象：

```text
candidate_profile:
  最新 observation/path 上构建出的完整 finite 几何候选；

active_profile:
  最近一次成功安装、且未被最新明确 unsafe evidence 否定的 tube。
```

禁止再创建：

- recovery profile；
- display profile；
- shadow active profile；
- fallback certificate；
- pending/committed 多份 profile 副本。

candidate build incomplete/UNKNOWN/UNAVAILABLE 时：

- Candidate 根据自身完整性 ADD/DELETE；
- 不自动覆盖 active；
- 不因“缺少新证据”把旧 active 当成已被障碍否定；
- 不 permanent latch；
- active 是否还能执行由当前 exact bounds/port 检查决定。

只有最新 observation 明确证明以下任一情况时，才否定旧 active 的当前安全声明：

- actual/reference categorical OCCUPIED；
- current actual/reference OUT_OF_MAP；
- current retained offset 不在最新明确 tube 内；
- `U>=0` exact current set 为空；
- geometry/math invalid。

UNKNOWN/UNAVAILABLE 是证据不足，不等于明确 UNSAFE。它可以撤销 Certified Marker，但不得伪装成永久软件失败。

---

## 4. Runtime 只保留三类控制结果

控制 arming 后，每个高率周期只允许以下三类安全/执行结果：

```text
1. U+ nonempty
   -> normal rolling

2. U+ empty, U>=0 nonempty
   -> nonnegative recovery / safety priority

3. U>=0 empty, or latest evidence explicitly unsafe
   -> emergency required
```

另有与安全状态正交的：

```text
genuine fatal invariant
```

它不是第四个普通 tube gate，而是 NaN/数学合同破坏的永久故障分类。

现有 enum 可为 ABI/诊断兼容保留，但不得通过更多中间状态组成复杂布尔链。`WAITING_FOR_CANDIDATE`/`BLOCKED` 只能表示没有可执行 active 数据，不能成为 tracking 抖动时反复退出控制的路径。

---

## 5. Tracking 语义：证书监控，不是立即关控制开关

`tracking_error_bound=0.15` 保持不变，仍是鲁棒 tube margin 的组成部分。禁止调大。

但：

```text
tracking_error_norm > tracking_error_bound
```

不得单独导致：

- permanent latch；
- 立即退出 matched controller；
- fallback 到普通 base guidance；
- `U>=0` 未检查就 emergency；
- delta reset/clip。

原因：关闭 matched recovery 会失去 `N*u_delta` 和一致 internal delta 演化，可能使 tracking 更难恢复。

### 5.1 fixed tube

fixed tube 不是 obstacle certificate。tracking 超界时：

- 记录 `tracking_within_bound=false`；
- Certified Marker DELETE；
- 禁止启动新的 manual excursion；
- 使用 `U>=0` 非反向回中端口继续 matched control；
- 只有 `U>=0` 为空或 geometry/math invalid 才 emergency。

### 5.2 ESDF/cloud tube

tracking 超界表示原鲁棒 tracking-ball certificate 暂时失效，因此：

- Certified Marker DELETE；
- 不再宣称 obstacle-certified rolling；
- 禁止新 excursion；
- 若最新 categorical snapshot 对 actual/reference 没有明确 OCCUPIED/OUT_OF_MAP，并且 `U>=0` 有解，则继续 nonnegative recenter/recovery；
- 该 recovery 必须明确诊断为 uncertified recovery，不得把 Marker 重新 ADD；
- 若 actual/reference 明确 OCCUPIED/OUT_OF_MAP，或 `U>=0` 为空，才 emergency。

本阶段没有低层 hover override；报告必须明确 recovery 不是新的安全定理，Certified DELETE 表示鲁棒证明暂时不成立。

不得新增 recovery mode enum；可复用现有 SAFETY_PRIORITY/Runtime SAFETY_PRIORITY，并通过已有 tracking field 表明证书失效。

---

## 6. `U+` 与 `U>=0`：必须有连续 witness

继续复用同一个 `PortProjector` 半平面内核。不得复制 invariant 公式。

### 6.1 `U+`

保留正：

- phase progress margin；
- physical tangent progress margin；
- amplitude；
- slew；
- current/next envelope；
- boundary invariants；
- regularity。

沿 existing certified horizon 构造确定性的连续 rollout witness。每步必须传播：

```text
w
delta
previous_final_port
```

并查询 exact next bounds。

### 6.2 `U>=0`

不得继续只逐 knot 使用同一个 retained delta/previous port 做互不连接的 probe。

必须同样构造连续 witness：

- lower phase/tangent margins 设为零；
- 其他 amplitude/slew/tube/invariant/regularity 不变；
- 每步传播 `w/delta/previous_final_port`；
- 若 projected total `w_dot > 0`，继续 exact next-envelope rollout；
- 若得到合法 `w_dot == 0` 的 hold port，且当前 offset/invariant/regularity 均成立，则该 hold 本身是合法的 `U>=0` witness，可终止为 feasible；
- 不允许因 rollout 不前进而无限循环；
- 不允许仅凭各 knot 独立有解判 feasible。

允许 conservative false negative，不允许 false positive。

必须新增反例测试：

```text
every knot locally feasible
but propagated delta/previous-port cannot reach the next knot
```

预期 `U>=0` 不得判 feasible。

---

## 7. 消除过期 certificate gate

不得再把安装时的：

```text
w/delta/previous_final_port/base_w_dot/base_tangent_speed
```

保存为一个需要以后“完全相等”的永久 certificate key。这些量按设计每周期变化，完全相等检查只会把正常状态演化变成 emergency/fatal 分类问题。

要求改为：

1. candidate/active install 只保存 profile/map/path epoch ownership；
2. high-rate Runtime 使用本周期真实 current geometry、delta、previous port、base guidance，重新检查 current exact `U+`；
3. `U+` 不可行时立即检查 current exact `U>=0`；
4. projector 成功后执行同一个 final port；
5. projector 空集按本周期两个集合的真实结果分类，不依赖与旧 frozen facts 的 equality；
6. horizon check 属于 receding certificate：至少在 tube update 时检查，Runtime 每周期 exact current/next-envelope 检查；
7. 若无法在当前 adapter 两阶段调用中获得 base guidance，使用小型 prepare/evaluate/complete 接口调整，不得复制 IsfReferenceKernel 公式。

删除或停止使用：

- `dynamic_certified_w`；
- `dynamic_certified_delta`；
- `dynamic_certified_previous_final_port`；
- `dynamic_certified_base_w_dot`；
- `dynamic_certified_base_tangent_speed`；
- `CertificateStateMatches()`；

除非字段必须为 ABI 保留；若保留，必须明确 deprecated、永远不参与控制/安全决定。

Runtime projector 空集不再自动 genuine fatal：

- 本周期 `U+` 空、`U>=0` 非空 -> safety priority；
- 本周期 `U>=0` 也空 -> emergency；
- 只有 projector/helper 对完全相同本周期输入给出自相矛盾结果，才 genuine fatal。

---

## 8. Path revision：不在 A5 增加 workaround gate

本阶段禁止修改：

- `ContinuousPhasePath`；
- phase mapping；
- C2 connector；
- path install；
- `gvf_manager`；
- planner replan。

path revision 时：

- 保持 retained `delta`；
- 不 permanent latch；
- tracking 瞬时超界按第 5 节进入 nonnegative matched recovery；
- Certified Marker DELETE，直到 tracking 和当前证书恢复；
- 不新增 `WAITING_PATH_CONTINUATION` gate；
- diagnostics 记录 source/path revision 即可。

报告必须明确：reference jump 的根治属于 A6，本阶段只保证 A5 不因该事件反复退出控制或永久锁死。

---

## 9. Marker 与 diagnostics

Candidate Marker：

- 只由 candidate finite/complete geometry 决定；
- 不依赖 100-cycle gate、tracking、selected、emergency 或 failure latch；
- 三个 ID 同步 ADD/DELETE。

Certified Marker：

- 表示“当前鲁棒 tube certificate 有效”，不是“控制器正在输出”；
- 要求 active map/profile certificate、current bounds、tracking within bound、当前 dynamic classification 至少 `U>=0`；
- tracking recovery、UNKNOWN/UNAVAILABLE latest evidence、explicit unsafe、emergency、fatal 时 DELETE；
- 不要求 `zero_gate_open`，因为 gate 只决定是否接管控制，不决定 tube 证书本身；
- 三个 ID 同步 ADD/DELETE。

Control selected：

```text
selected = zero_gate_open && !failure_latched &&
           (normal rolling || nonnegative recovery)
```

在 tracking recovery 中 selected 可以保持 true，但 Certified 必须 false。

Manual diagnostics 保持严格 83 字段，旧索引不动。

Epoch diagnostics 现有 0–59 索引不动；优先不追加字段。用现有字段表达：

- state；
- runtime mode；
- tracking within bound；
- positive/nonnegative feasible；
- emergency；
- transient blocked；
- genuine fatal；
- selected；
- display certified。

如当前字段无法区分 uncertified recovery，可只追加一个有限枚举字段；禁止增加一组新的 bool gate。

Publisher queue 不属于本阶段核心。保留当前 queue=10 只有在能证明不改变控制/语义且 Marker/diagnostic 对齐改善时才允许；否则恢复原值。不得用 queue 掩盖生成时刻不一致。

---

## 10. 文件白名单

只允许修改以下当前 G2e 文件中确有必要者：

### phase_offset_core

```text
src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/port_projector_test.cpp
```

只有修复共享 nonnegative projector 语义需要时才改；不得改 matched port 或 geometry。

### phase_offset_navigation

```text
src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_dynamic_feasibility.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_dynamic_feasibility.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_dynamic_feasibility_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
```

### bspline_race thin integration

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_epoch_diagnostics.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_epoch_diagnostics.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_diagnostics_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

若 CMake 不需要变化，不修改。

任何正确实现若需要修改白名单外文件，立即停止并报告。尤其禁止修改：

- `gvf_manager`；
- launch；
- planner/Kino/B-spline/C2；
- SDFMap/cloud snapshot；
- tube builder/filter/cross-section geometry；
- Marker helper geometry；
- PositionCommand/governor/SO3；
- parameters/messages/swarm/CBF。

---

## 11. 单元测试要求

### Gate/lifecycle

- gate 关闭 99 周期：selected=false、emergency=false、failure=false；
- candidate 和 tube certificate 可在 shadow 中计算；
- 第 100 周期打开后才允许 selected；
- gate 不参与 candidate completeness；
- gate 不参与 certificate geometric truth。

### Tracking recovery

- fixed tracking `0.16 > 0.15`：Certified false、selected 在 gate open 且 `U>=0` 时仍 true、safety-priority、no emergency、no latch；
- ESDF tracking exceed + current evidence not explicitly unsafe + `U>=0`：uncertified recovery、selected true、Marker DELETE、no latch；
- explicit OCCUPIED/OUT_OF_MAP：emergency、selected false；
- tracking 恢复后自动回到 rolling，无 reset/delta jump。

### Dynamic feasibility

- G2d steep boundary 在 install/execution 前识别；
- positive continuous rollout；
- nonnegative continuous rollout；
- legal zero-progress hold；
- all knots local-feasible but no continuous propagated witness -> infeasible；
- slew/amplitude/regularity/next-envelope hard；
- no frozen-fact equality gate；
- deterministic finite/no input mutation。

### Ownership

- incomplete/unknown candidate 不覆盖 active；
- explicit latest unsafe invalidates current certificate；
- active epoch 只 material install 时增长；
- no extra profile objects；
- retained delta/previous final port only successful selected final port updates。

### Fatal

- tracking/map/candidate/path revision/U+ empty/U>=0 empty 均不 permanent latch；
- NaN/geometry/matched mathematical contradiction 可 latch；
- same-current-input helper/projector contradiction 可 latch；
- failure reason retained。

### Marker/diagnostics

- Candidate and Certified each no mixed action；
- tracking recovery: Candidate may ADD, Certified DELETE, selected may remain true；
- gate closed: certificate may ADD but selected false and emergency false；
- manual=83、epoch=60（或仅追加一个字段后的精确新值）；
- old diagnostic indices unchanged；
- generated Marker/diagnostic same-frame agreement。

---

## 12. 构建与 ROS 验收

### ROS-0 baseline

裸 launch：

- disabled/none/refresh=3.0；
- manual topics no publisher；
- `/position_cmd` 唯一 publisher `/formation_planning`；
- point-to-point baseline 到达；
- no new params/publishers。

### ROS-1 fixed

使用与 G2e 相同 fixed 参数和目标，不调参。

验收：

- gate 关闭期间 emergency=0、selected=0；
- gate 打开后 rolling/recovery 可以连续 selected；
- G2e 中 24 帧 pre-gate emergency 必须为 0；
- path revision 引起的 5 帧 tracking spike 可产生 Certified DELETE/recovery，但不得 selected 掉线、不得 emergency、不得 latch；
- tracking 恢复后自动 Certified ADD/rolling；
- delta 无 reset；
- matched residual 合格；
- Candidate 连续完整；
- Marker/diagnostics 逐帧一致。

### ROS-2 ESDF/cloud snapshot

沿用 G2d/G2e 正式 PointCloud2 参数、地图和目标，不调速度/threshold/map。

验收：

- raw access=0、self-free=0；
- margins `0.55/0.10/0.45`；
- no permanent latch；
- tracking transient 进入 uncertified nonnegative recovery，而不是直接退出控制；
- explicit `U>=0` empty/explicit map unsafe 才 emergency；
- recovery 时 Certified DELETE，Candidate 可继续；
- 恢复后自动 rolling；
- active install 有连续 `U+` 或 `U>=0` witness；
- no stale frozen-fact equality decision；
- no Marker mixed actions or diagnostic mismatch。

保存新 bag、CSV、JSON、分析脚本和 SHA-256。旧 G2e bag 只作对比，不能作为 r1 正式证据。

若真实证据显示 continued matched recovery 会产生明确 map collision risk，立即停止并报告；不得通过隐藏 Marker/diagnostic 使测试通过。

---

## 13. Git、自审核与停止

执行前后记录：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git diff --cached --name-only
git stash list
```

预期：

- `main`；
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`；
- prototype stash 保持；
- no staged/commit/tag/push；
- 用户既有 dirty 内容保持。

最终报告必须列出：

1. 删除/停止使用了哪些多余 gate/frozen certificate facts；
2. gate 现在唯一职责；
3. tracking recovery 与 Certified safety claim 的区别；
4. `U>=0` 连续 witness；
5. path revision 仍留给 A6 的内容；
6. ROS-1 原 29 emergency 的新分类；
7. ROS-2 rolling/recovery/emergency 统计；
8. 所有证据路径/hash；
9. 白名单和依赖审计；
10. 未修改项。

完成 A5-G2e-r1 后立即停止。

禁止进入 A5.3、A6、多机、CBF、planner tuning、速度调参或新场景。

