# Codex A5G Runtime Exact-Set 与 A6 H2 Refresh-Pin 执行单

**日期：** 2026-08-13  
**状态：** `IMPLEMENTED_VALIDATED_S4_NOT_PASS`  
**授权范围：** 仅限本执行单列出的 Runtime 精确可行集修正与 A6 H2 同权威刷新事务保护。A5G 的 tube 几何、滤波器、裕度、路径相位域与既有安全语义均冻结，不在本单内修改。

## 0. 结论与执行顺序

本单处理两个已经由证据定位、彼此独立的问题，顺序不可颠倒：

1. **阶段 1 — Runtime exact-PWL 可行集：** 以精确当前/下一步 PWL 约束及有界短时域可行性取代旧连续 invariant 平面作为运行时硬约束的角色；旧 invariant 平面仅保留为诊断量。
2. **阶段 2 — A6 H2 old-pair refresh pin：** 从 seam 选择到最终 CAS 固定同一个旧 path/tube pair，禁止同 owner 的 timer 刷新将其重绑为不同 provenance 的 pair。

阶段 2 不得以阶段 1 的失败为由绕过任何安全检查；阶段 1 也不得改变 manager 的安装判据。

## 1. 已批准的事实基础

### 1.1 Runtime invariant 冲突

`/tmp/a5g_h2_invariant_cf_20260813` 中的 counterfactual 证据显示：准备阶段已经通过 current-bound、tracking、base guidance 与正常性检查；在 `PhaseOffsetRuntime::complete()` 中，`U+` 与 `U_safe` 均因 joint-port feasible polygon 为空失败。旧连续 invariant 半平面与精确 PWL terminal/crossed-knot 约束同时作为硬约束，是待验证的冲突来源。

本单不假定删去 invariant 平面天然安全。它们只能在以下替代证明全部成功时失去“硬执行约束”地位。

### 1.2 H2 unsafe rebind 已撤回

下列旧方案已明确撤回，严禁恢复：

`docs/Codex_A6_H2_Same_Authority_Timer_Refresh_Correction_Execution_Plan_2026-08-13.md`  
`DOCUMENT_STATUS=WITHDRAWN_UNSAFE_REBIND_CONTRACT`

实测同 revision 下仍可能出现 `live_gen != expected_gen`。这不是纯指针问题：timer 刷新的 pair 同时携带 map/profile/sample provenance 的变化。因此 pointer-only rebind 或“同 owner 可以替换”的任何合同均不安全。

### 1.3 保持不变的 manager 拒绝语义

以下合法 fail-closed 情况必须保留，禁止通过自动横移、自动接管、降级认证或任何隐式豁免绕过：

- `CURRENT_OFFSET_OUTSIDE`（reason 6）：当前保留的 `delta=0` 不在安全连通分量内；
- `FORWARD_HORIZON_SHORT`（reason 11）：认证前向覆盖不足。

`candidate_complete=1` 仍仅表示构建/过滤/表面验证完成，绝不等价于可安装的 Active tube。

## 2. 阶段 1：Runtime exact-PWL 可行集修正

### 2.1 目标

消除“精确一步 containment 存在、但遗留 invariant 平面使 joint feasible polygon 错误为空”的冲突，同时保持运行时控制输入的全部实际安全约束。

### 2.2 唯一允许的硬执行约束集

Runtime 的可选控制输入必须同时满足下列**精确、可计算**约束；任一项失败即不可选：

1. **精确当前 containment：** 当前相位/偏移处于已安装 tube 的有效安全区间；
2. **精确 PWL terminal containment：** 一个控制步结束时的预测状态位于对应 PWL segment 的闭安全区间；
3. **精确 crossed-knot containment：** 若该步跨越一个或多个 PWL knot，穿越点及每个相关 piece 的约束均逐一满足，不能只验证末端；
4. **既有运动学边界：** amplitude、`u_delta` slew、non-reverse、regularity 和已批准的 base-guidance 有效性保持原样；
5. **确定性 bounded short-horizon viability：** 在固定、有限、确定性的短时域内，以相同的精确 PWL containment 和既有运动学边界前向检查，证明至少存在一条连续可行控制序列。不得用随机采样、软惩罚、参数放宽或“本步可行即可”代替。

旧 continuous invariant 半平面可以继续计算、记录并在测试中对照，但在且仅在上述 1–5 项均通过时，不能再作为额外的硬执行不等式加入 projection polygon。它们的状态为 **diagnostic-only**，不是删除证据，也不是安全豁免。

### 2.3 先决合同：generic future-step evaluator

现有 frozen-base counterfactual 的作用仅是**隔离旧 invariant 半平面与 exact-PWL 约束的冲突**。它不能、也绝不能被表述或使用为正式的 bounded short-horizon viability 证明：冻结当前 base guidance 后外推，无法证明未来每一步在真实路径几何、参考状态和 ISF guidance 下仍有可行控制。

因此，在任何把旧 invariant 平面降为 diagnostic-only 的产品修改之前，必须先增加或复用一个 generic future-step evaluator 合同。该合同在 rollout 的**每一个**预测 step 都必须：

1. 从 immutable path owner 与对应 immutable sample 读取该未来 phase 的路径/剖面来源，而不是复用当前 step 的冻结结果；
2. 根据 rollout 的控制历史，预测该 step 的 matched physical state 与 matched reference state；
3. 用与真实 Runtime 完全相同的既有 ISF gains 和既有 guidance 合同，重新计算该未来 step 的 path geometry 与 base guidance；
4. 在重新计算的 future state/guidance 上，检查 §2.2 的 exact current、terminal、crossed-knot containment 及 amplitude、slew、non-reverse、regularity；
5. 返回可审计的可行/不可行结果以及最早失败 step/既有约束类别；不得用 frozen guidance、单步投影成功或诊断 invariant 的结果替代。

该 evaluator 必须保持分层：navigation/runtime 只依赖 generic evaluator 及已有的匹配量/不可变量，不能引入 ROS 依赖，也不能把 B-spline 公式或 path-owner 内部实现搬入 navigation。具体路径几何与 matched reference 的计算仍属于 adapter/path-owner 边界。

short-horizon 的步数和覆盖范围只可由现有合同确定性推导：已有 `dt`、`tube_update_period`、`min_certified_forward_w`、既有 slew limits 与 profile domain。不得新增可配置 horizon、采样间隔、容忍量或任何数值参数；若这些既有量无法推出一个同时覆盖下一次 tube 更新前的控制演化、且不越过 profile domain 的有限 rollout，则停止并报告合同缺口。

### 2.4 实施约束

- 复用已有 exact terminal/crossed-knot 数据与投影器；不得改写 tube/filter/geometry，也不得改动边界、margin、slope、lookahead、速度、rate 或 tracking bound；
- generic future-step evaluator 是实施前置条件。若 navigation/adapter 边界无法在本单白名单内表达其输入、输出、immutable provenance 或预测 matched state，停止并报告接口缺口；不得用 frozen guidance 冒充正式证明；
- bounded short-horizon viability 的长度、离散化和失败语义必须由上述既有已批准量推导；不得发明新产品参数；
- 必须保存“旧 invariant 拦截但精确集合可行”的反例诊断，以及“精确短时域不可行”时仍 fail-closed 的反例；
- 禁止将 candidate 成功、一次 projection 成功或 invariant 诊断通过误当作 selected 的替代条件。

### 2.5 阶段 1 验收与停止条件

先进行 focused unit/projection 验证，再进入私有 ROS dry-run；不得接入或影响用户 ROS。

**必须通过：**

- 代表性 counterfactual 中，移除旧 invariant 硬平面后，精确 current、terminal、crossed-knot、运动学边界和 bounded short-horizon viability 共同给出可行 witness；
- 该 witness 的每一个 rollout step 都由 generic future-step evaluator 用该 step 的 immutable owner/sample、预测 matched state、重新计算的 geometry/base guidance 得出；frozen-base counterfactual 仅可作为冲突定位对照；
- 对每个 crossed-knot、末端收缩和无短时域延续的 fixture，Runtime 仍拒绝并输出已有失败语义；
- 现有 regularity、non-reverse、amplitude、slew 与 current-containment 回归保持 fail-closed；
- `phase_offset_runtime_test`、`phase_offset_port_projector_test` 及当前 CMake focused tests 全部通过。

**立即停止并回报，不得继续猜测：**

- 无法构造确定性 bounded short-horizon viability 证明；
- generic future-step evaluator 需要向 navigation 泄露 ROS/B-spline/path-owner 实现，或现有 adapter 边界不能安全表达所需合同；
- 仅靠改变任意数值参数才能获得 witness；
- 精确约束在跨 knot 或预测末端出现未覆盖的安全空洞；
- 发现证据表明 invariant 平面实际表达了精确 PWL 集未覆盖的安全性质。

## 3. 阶段 2：A6 H2 exact old-pair transaction-scoped pin

### 3.1 目标

实现 H2 零空窗事务所需的 authority 一致性：seam/C2/new-tube/runtime dry-run/final CAS 全过程只使用同一个、不可变的旧 path/tube pair。H2 的目标不是复用新 connector 上旧 tube 几何，而是确保构造和提交基于同一对旧对象及其 provenance。

### 3.2 Pin 合同

1. 必须在 `certifiedFutureSeamCandidates()` 之前、以及任何 seam 选择、C2 connector 构造、new-tube 构建或 runtime dry-run 之前，经 adapter 的**原子 capture+acquire** 取得并 pin **精确旧 pair**；不得先读 candidate 再延迟 acquire；
2. capture 返回的 transaction snapshot 至少绑定 path pointer、tube pointer、revision、generation、session/epoch 与 map/profile/sample provenance；adapter 必须负责把这些身份作为同一原子合同捕获，调用者不得拼接多个非原子读；
3. pin 由 RAII guard 或明确的 manager transaction owner 持有，并持续至新 path、新 tube、runtime dry-run 与最终 compare-and-swap 成功或失败为止；其析构/完成路径必须只释放自己所持有的精确 capture；
4. 重活（seam 枚举、C2 构造、new-tube 构建、runtime dry-run）期间不得持有 runtime lock 或 manager lock。pin 是事务所有权，不是长时间锁；
5. 最终 CAS 必须继续验证 exact pointer/session/Runtime bits，且验证的正是被 pin 的 pair，不得接受“同 revision”或“同 owner”的替代对象；
6. 所有 success、failure、reset、retire、shutdown、expired-seam、pending cancel、异常返回和早退路径均必须精确释放 pin；manager 持锁时不得调用 release，以避免锁反转、重入或 timer deadlock；
7. 无法取得完整 exact pair、pin 已过期、或最终 CAS 发现任何身份不一致时，事务 fail-closed，不安装新 pair。

锁序必须固定并有测试覆盖。adapter capture/acquire、manager transaction ownership、runtime validation 与 timer 协调只能遵循一个文档化顺序；任何反向获取均为实现错误。release 必须在 manager lock 外进行，并对 reset/retire/shutdown/pending-cancel 等异步路径保持幂等。

### 3.3 精确 API 职责

- **Adapter：** 提供原子 `capture+acquire`，产出不可变 old-pair transaction snapshot/guard；负责携带 path/tube identity 与 provenance，不向 navigation 暴露 ROS 或 B-spline 实现；
- **Manager transaction owner：** 在获得 guard 后编排 seam、C2、new tube、dry-run 与 final CAS；不在计算重活期间持 manager/runtime lock；在锁外释放 guard；
- **Runtime：** 仅消费被 guard 绑定的 immutable pair 做 dry-run/验证，不得发起 timer refresh、pointer rebind 或降级身份比对；
- **Timer refresh：** 在同一协调点使用非阻塞 `try_lock` 检查 transaction pin；pin 存在时跳过 pair-authority publication，其他允许的构建/证据工作继续；不得等待 guard、重绑或改写其 expected provenance；
- **Final CAS：** 比对 guard capture 的 exact identity/session/Runtime bits，成功后按现有路径发布；失败不安装并由 guard 生命周期释放。

### 3.4 Timer 行为

timer 仍可照常构建候选、采集 raw/cloud 证据和完成非 pair-authority 工作。它不得在 pin 存在时改变被 pin 事务所见的 pair。

- `refreshPairFromTimerEpoch()` 采用非阻塞 `try_lock`；
- 若 lock 不可得或存在 transaction pin，timer **仅跳过 pair-authority publication/refresh**；
- 它不得重绑 pointer，不得改写 expected generation/provenance，不得将新 pair 塞入正在进行的 H2 事务；
- timer 跳过本次 pair publication 不是产品 gate、状态、mode、reason 或 schema，也不是对构建结果的拒绝；下一次无 pin 时可正常刷新；
- pin 仅解决同权威 refresh race，不放宽 revision 不匹配、reason 6/11、runtime 不可行或最终 CAS 失败。

### 3.5 阶段 2 验收与停止条件

**必须通过：**

- 复现 `live_gen=12 / expected_gen=11 / live_rev=expected_rev=1` 类竞争时，H2 事务不发生 pointer/provenance rebind；
- timer 在 pin 期间继续完成允许的构建/诊断工作，但不发布 pair-authority refresh；
- `certifiedFutureSeamCandidates()` 与所有 seam/C2 重活开始前已经持有同一 atomic capture guard；测试不得只覆盖最终 CAS；
- future seam 仍只来自被 pin 的旧 pair 的 immutable certified samples，并且相对 capture 至少保留既有 `tube.min_certified_forward_w`；不得新增 lead/time 参数或放宽提交时的 `current_w < future_seam_w`；
- reset、retire、shutdown、expired-seam、pending cancel 及每个早退分支精确释放同一 guard；在 manager lock 持有期间调用 release 的测试必须失败；
- 固定锁序下，timer try-lock、transaction cancel 与 release 的并发压力测试无死锁、无锁反转、无 stale authority，也不发生 pointer/provenance rebind；
- 成功路径完成 exact pair CAS 后，FSM mirror 可消费，并按照既有 Candidate → Active → Certified → Selected 链路工作；
- reason 6 和 reason 11 的 fixture 仍拒绝；
- success/failure/reset/expired-seam 全部分支无 pin 泄漏、死锁或 stale pair；
- `continuous_phase_path_test`、`gvf_switch_policy_test`、`phase_offset_matched_adapter_test`、`phase_offset_tube_epoch_integration_test`、`phase_offset_tube_epoch_manager_test` 及当前 CMake focused tests 全部通过。

**立即停止并回报：**

- pin 需要扩大为全局锁、阻塞 timer 或改变路径/tube publication 的正常所有权；
- 无法证明每个事务退出分支释放 pin；
- 无法在 seam 枚举前完成 adapter 原子 capture+acquire，或必须在重活期间持 runtime/manager lock；
- 不能定义固定锁序，或 release 必须在 manager lock 内执行；
- exact identity 不足以表达 provenance，必须靠宽松的同 revision/同 owner 判断；
- H2 成功依赖复用 connector 区间上的旧 profile/sample 几何。

## 4. 严格白名单与禁止项

本单允许修改的文件穷尽如下；任何其他产品文件需要修改时立即停止并回报：

```text
# 阶段 1：pure core / navigation exact-set 与 future-step 合同
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_types.h
src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/port_projector_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp

# 阶段 1 adapter binding 与阶段 2 exact-pair pin
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp

# 执行单
docs/Codex_A5G_Runtime_Exact_Set_And_A6_H2_Refresh_Pin_Execution_Plan_2026-08-13.md
```

白名单内也只允许：表达 generic future-step evaluator 的最小接口、Runtime 中 invariant 的硬/诊断职责转换、确定性 exact-PWL short-horizon viability、old-pair pin 生命周期、原子 capture+acquire、timer 非阻塞协作、精确身份验证及相应 focused fixtures。不得把 ROS 或 B-spline 公式放进 navigation。

以下全部禁止：

- 新增任何 product gate、runtime mode、epoch state、reason、枚举、诊断 schema 或隐藏 fallback；
- 改动 tube 几何、TubeFilter、surface validator、margin、boundary slope、lookahead、路径速度、rate、tracking bound、A5G phase-domain 合同；
- 修改 `AGENTS.md`；
- 自动 side-step、自动重规划、自动认证、失败时复用旧 connector 几何、pointer-only rebind；
- 连接用户 ROS、杀死用户进程、用参数调优掩盖失败；
- 以测试通过替代真实安全证明，或以单步 witness 代替短时域可行性。

## 5. 聚焦测试、私有 ROS 与最终停止条件

代码完成后，顺序运行：

1. `phase_offset_runtime_test`
2. `phase_offset_port_projector_test`
3. `continuous_phase_path_test`
4. `gvf_switch_policy_test`
5. `phase_offset_matched_adapter_test`
6. `phase_offset_tube_epoch_integration_test`
7. `phase_offset_tube_epoch_manager_test`
8. 当前 CMake focused tests 与 `git diff --check`

仅在以上通过后，才可使用独立、任务自有的私有 ROS 运行 H2 acceptance：真实可行 connector 必须经 staged new tube/runtime、exact pair CAS 与 FSM mirror，到达 Candidate → Active → Certified → Selected。合法 reason 6/11 与 exact-set 不可行必须保持拒绝。

真实物理闭环 `/position_cmd -> /sim/odom` 和 S4 不属于本单的先验通过条件；只有在 sustained Selected interval 后才能测量。S4 只有实测最大 tracking error `< 0.05 m` 才能标记 PASS，其他结果均为 WAIVER/NOT PASS，不能反向驱动本单调整参数。

若任一阶段达到其停止条件、focused tests 失败、私有 ROS 出现安全语义回退或证据不足，本执行单停止在该阶段，保留 A5G/A5F 与既有 fail-closed 行为，提交事实报告而不扩大改动范围。

## 6. 2026-08-14 最终执行记录

### 6.1 范围审计

- 阶段 1 exact-PWL：`PASS`。
- 阶段 2 H2 exact old-pair pin：`PASS`。
- 动态验收暴露的首个 future seam 过近问题已按最小范围修正：候选仍只从旧 pair 的 immutable certified samples 中按序选择，但必须至少领先 capture 既有 `tube.min_certified_forward_w`。该修正没有新增参数、gate、mode、state、reason 或诊断字段，也没有改动 tube 几何、Filter、SurfaceValidator、margin、slope、lookahead、速度、rate 或 tracking bound。
- §4 原白名单漏列了 §3.5 与 §5 已明确要求执行、且承载 manager focused fixtures 的 `gvf_switch_policy_test.cpp`。本记录将其补为**测试文件**白名单；这不扩大产品代码范围。
- `AGENTS.md` 未修改。

### 6.2 聚焦回归

2026-08-14 使用当前工作区二进制重新执行，结果如下：

```text
phase_offset_port_projector_test             24/24 PASS
phase_offset_runtime_test                    33/33 PASS
continuous_phase_path_test                    6/6  PASS
gvf_switch_policy_test                       68/68 PASS
phase_offset_matched_adapter_test            54/54 PASS
phase_offset_tube_epoch_integration_test     10/10 PASS
phase_offset_tube_epoch_manager_test         49/49 PASS
TOTAL                                       244/244 PASS
git diff --check                                   PASS
```

### 6.3 H2 私有动态验收

补丁后私有 GDB 运行：

`/tmp/a5g_a6_h2_private_gdb_parent_20260814/run_20260814_043249_12601`

观测到同一次 replacement 事务完整通过：

```text
MANAGER_STAGE_RETURN=true
ADAPTER_PREPARE_RETURN=true
ADAPTER_FINALIZE_RETURN=true
MANAGER_COMMIT_RETURN=true
FSM_CONSUME_RETURN=true
```

普通私有运行：

`/tmp/a5g_a6_h2_private_final_acceptance_20260814/run_20260814_043540_12602`

观测到初始前端 `[REPARAM] start_w=0.050 ...`、C2 connector success，以及 replacement 前端 `[REPARAM] start_w=1.452 end_w=7.159 ...`。因此 H2 exact-pair CAS、前端发布与 FSM 接管标记为 `PASS`。

### 6.4 Candidate 到 Selected 与 S4

低扰动内存计数运行：

`/tmp/a5g_a6_h2_private_gdb_parent_20260814/run_20260814_044904_12609`

```text
SELECTED_SUMMARY count=40
min=0.034709699268125796
max=0.10569159001824309
revisions=[2]
```

- replacement revision 2 到达 Candidate → Active → Certified → Selected：`PASS`。
- sustained Selected 区间得到 40 个 tracking 样本：`PASS`。
- S4 `max tracking error < 0.05 m`：`NOT PASS`；实测最大值为 `0.10569159001824309 m`。

按本执行单约束，S4 结果不得反向驱动 tube 参数、tracking bound、速度或额外门控修改。后续若要处理 S4，必须另开专门的物理控制/tracking 诊断执行单。

### 6.5 隔离与停止

- 三次私有运行的 `user_processes_pre.txt` 与 `user_processes_post.txt` 无差异。
- 用户进程 `1746447`、`1746448`、`1746461` 在最终检查时仍存活。
- 本执行单在 H2 与 Candidate → Selected 验收通过、S4 明确记录为 `NOT PASS` 后停止；不继续调参，也不新增机制。
