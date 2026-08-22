# Codex A6-H2-R1：同 owner timer refresh 与 future-seam handoff 权威一致性修正执行单

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=WITHDRAWN_UNSAFE_REBIND_CONTRACT
STAGE=A6-H2-R1
IMPLEMENTATION_AUTHORIZED=false
AUTO_ADVANCE=false
AUTHORIZATION_SCOPE=SAME_AUTHORITY_TIMER_REFRESH_HANDOFF_STARVATION_CORRECTION
PRODUCT_GATE_STATE_MODE_REASON_CERTIFICATE_LATCH_SCHEMA_CHANGE_ALLOWED=false
LAUNCH_CONFIG_PARAMETER_CHANGE_ALLOWED=false
TUBE_GEOMETRY_FILTER_VALIDATOR_RUNTIME_THEORY_CHANGE_ALLOWED=false
MARGIN_SLOPE_LOOKAHEAD_BACK_SPEED_SATURATION_RATE_TRACKING_BOUND_CHANGE_ALLOWED=false
AGENTS_MD_CHANGE_ALLOWED=false
```

> 日期：2026-08-13  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 前置执行单：`Codex_A6_H2_Atomic_Path_Tube_Handoff_Execution_Plan_2026-08-12.md` 与
> `Codex_A5G_Executable_Phase_Parameterization_Execution_Plan_2026-08-13.md`。

> **撤回说明（2026-08-13）**：后续独立审查证明本文原 2.2 的“same-owner refresh 后
> 重绑 `transaction.expected_pair`”不安全。timer refresh 不只替换指针，也可能替换 frozen map
> provenance、full samples 与 active profile；而 future seam、existing horizon 和 C2 composite
> 已在 manager 中基于旧 pair 选择。旧 transaction 不能只换 expected pointer 后继续提交。
> 正确机制必须二选一：从 capture 到 finalize pin/串行化 base pair refresh，或 refresh 后从最新
> live pair 完整重做 seam selection、C2、new tube 与 dry-run。两者都需要新的 manager+adapter
> 执行单。本文件不再授权任何产品修改。

## 0. 结论与范围

A5G 已消除 raw B-spline 静止端点导致的相位爆速；当前动态日志中 C2 connector 已能构建，
但 H2 transaction 在 staging 期间受到同 owner timer refresh 饥饿：

```text
FSM capture old pair P(generation=g)
-> outside locks 构建 C2/new-owner tube
-> 10 Hz timer 对同 path/revision 原子 refresh 为 P'(generation=g+1)
-> stagePathTubePair 要求 live pointer == P
-> false，整次 replan not installed
```

本单只修正该 ownership 竞态。不改变 C2、A5G 映射、tube 几何、Filter、SurfaceValidator、
Runtime port 选择或任何控制参数；不新增 gate/state/mode/reason/certificate/latch/计数器/topic/
diagnostics schema。

## 1. 冻结证据

1. `refreshPairFromTimerEpoch()` clone 当前 pair、递增 generation 并 CAS 发布一个新指针；现有测试
   `PairTimerRefreshAtomicallyReplacesSamePathAuthorityWithNewEpoch` 明确断言 same owner/revision 但
   pair pointer 改变。
2. `stagePathTubePair()` 在锁内以 `live != expected_pair` 直接拒绝；现有测试
   `PreparedPathTubePairRejectsChangedExpectedAuthority` 只覆盖任意 authority 变化，没有覆盖
   same-owner timer refresh。
3. private ROS 日志在同一次 replan 中连续出现多个 `connector_success`，随后
   `replan not installed`；A5G endpoint/preflight 已通过，符合重活期间 timer 刷新的时序。
4. `samePathTubeAuthority()` 目前只保护 commit 后至 FSM mirror 消费之间的 same-owner refresh，
   不覆盖 staging/prepare/finalize transaction。

## 2. 已撤回的修正契约（禁止实施）

### 2.1 什么可以被视为同一 base authority

timer refresh 后的 live pair 只有在下列事实全部成立时，才可作为 transaction 捕获 base pair 的
等价后继：

```text
same nonzero authority_session
same source_revision
same immutable path_owner pointer
same semantic path start/end
live generation >= captured generation
same Runtime retained_delta bits
same Runtime previous_final_port bits
```

这不是放宽为“same revision 即可”。path owner、session、revision 与 Runtime bits 任一变化都必须
拒绝。map/profile generation 可以因 same-owner timer refresh 前进；new transaction 必须以锁内
重读到的最新等价 live pair 作为实际 CAS expected base，不能再对已被 refresh 替换的旧指针做
CAS。

### 2.2 stage / prepare / finalize

- `stagePathTubePair()` 捕获锁内 live pair；若它与调用者捕获 pair 指针相同，保持现有语义。
- 若指针不同，只允许按 2.1 验证为 timer-refreshed equivalent base；transaction 的
  `expected_pair` 必须记录这个锁内最新 live pair。
- staged revision 必须从锁内实际 base pair/source revision 派生，不能从过时 captured pointer
  派生。
- prepare/finalize 继续严格比较 transaction 记录的实际 expected pair，并继续逐 bit 校验 Runtime
  state；不得把任意后续 refresh 默认为可接受。
- 如果 prepare/finalize 前又发生一次 same-owner timer refresh，可在同一个短 Runtime/CAS 边界按
  2.1 重新绑定一次实际 expected base，或让 transaction 安全失败后下次 replan 重建；验收要求
  不能因稳定的 10 Hz refresh 形成永久饥饿。
- latest explicit map unsafe、authority retirement、path owner/revision 改变仍沿用现有 fail-closed。

### 2.3 明确禁止的伪修

禁止：

1. 暂停 timer、降低 timer rate、冻结 phase 或增加 retry/warm-up/lead gate；
2. 删除 generation/CAS、放宽为 same revision 或忽略 Runtime bits；
3. 复用旧 path 的 tube/profile 到新 C2 connector；
4. 调整 margin、slope、lookahead、速度、rate、tracking bound；
5. 修改 A5F/A5G tube/filter/runtime/ISF 理论以掩盖 ownership 竞态；
6. 新增产品 diagnostics reason/schema 来代替修复。

## 3. 穷尽白名单

### 3.1 产品

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

只允许增加无状态 authority-equivalence helper、transaction captured-base 字段，或最小修改现有
stage/prepare/finalize CAS 逻辑；不得增加 lifecycle 状态、ROS 字段或参数。

### 3.2 测试

```text
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

### 3.3 执行单与证据

```text
docs/Codex_A6_H2_Same_Authority_Timer_Refresh_Correction_Execution_Plan_2026-08-13.md
```

动态探针、runner、bag、日志、自审写入 `/tmp/a6h2_r1_*`，不得接入产品 CMake。

## 4. 必须测试

1. 安装 old pair，产生同 owner/revision timer refresh；用 refresh 前捕获的 pair 调用 stage，必须
   形成 transaction，且 `transaction.expected_pair` 是锁内最新 refreshed pair。
2. 上述 transaction prepare/finalize 成功，new owner/new tube 同 revision 原子接管；跨边界的
   `w`、retained delta、previous final port bits 不变。
3. stage 后、prepare 前再次发生 same-owner timer refresh：不得形成永久饥饿；允许安全重绑定或
   一次失败后由明确的重建路径成功，但不能新增 retry gate。
4. same revision 但不同 path owner 必须拒绝。
5. same owner 但不同 authority session、retired session、source revision 或 Runtime bits 必须拒绝。
6. stale old timer completion 不得覆盖 new pair。
7. production quintic connector interior 的 tube samples 仍来自 new owner，旧 profile 不复用。
8. 重建并运行：

```text
phase_offset_matched_adapter_test
phase_offset_tube_epoch_integration_test
gvf_switch_policy_test
continuous_phase_path_test
phase_offset_runtime_test
phase_offset_tube_epoch_manager_test
```

随后运行 current-CMake focused 16 suite、`git diff --check`、白名单与依赖边界审计。

## 5. 动态验收

使用 task-owned private ROS master/ROS_HOME、原 launch/map/参数，无 `LD_PRELOAD`/bypass：

1. A5G init 仍为 semantic endpoint，不出现 `initial finite frontend failed`；
2. C2 connector success 后形成 pending transaction；
3. command 原子提交后 FSM 消费 completed mailbox，新 owner/path revision 接管；
4. 不再因持续 same-owner timer refresh 永久出现 `replan not installed`；
5. Candidate -> Active -> Certified -> Selected；
6. 仅 Selected 后计算 `/position_cmd -> /sim/odom` tracking，max `<0.05 m` 才算 S4 PASS；
7. 若还有失败，按真实 stage/prepare/finalize 层报告，不增加 gate 或调参数。

## 6. 完成标准

只有 unit/integration、focused suite、private ROS H2 handoff 与白名单审计全部通过，才可写：

```text
A6_H2_R1_SAME_AUTHORITY_TIMER_REFRESH_CORRECTION_PASS
```

Filter 参数调优、emergency/replan bridge、多机与论文实验不属于本单。
