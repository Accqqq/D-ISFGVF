# Codex P1 T1–T3：continuous viable port 执行规范

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DOCUMENT_STATUS=AUTHORIZED_BY_USER_2026-08-17
STAGE=P1_T1_T2_T3_CONTINUOUS_VIABLE_PORT
AUTO_ADVANCE=false
SUPERSEDES_FOR_THIS_STAGE=Codex_P1_P2_Continuous_Viable_Port_Recovery_Execution_Plan_2026-08-16.md §5 only
NEXT_STAGE=P2a/P2b NOT AUTHORIZED
```

## 1. 已签署目标

本阶段只实现并验证 normal owner 的 continuous viable port 合同。对于一个唯一的 immutable
path owner、tube profile 与 map/snapshot provenance，Runtime 必须以真实 retained delta、已提交
previous final port、既有 amplitude/rate/non-reverse/regularity 约束和 exact-PWL bounds 构造一个
连续、可执行的 port witness。所有 normal selected/valid 输出必须使用同一 witness 的第一端口。

有限、确定性的候选搜索只能作为 witness finder：未找到 witness 一律保持既有
`CERTIFICATE_DENIED` fail-closed 语义，绝不声称数学可行集合为空。已 armed owner 的 normal
acceptance 不得重新把 `delta=0` 当作通用 gate；零偏移 robust 检查只允许是尚无 committed
history 的 unarmed pre-gate 保守起点。

## 2. 范围和唯一白名单

允许修改的产品/测试文件仅为：

- `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
- 本执行规范及其 P1 self-audit 文档。

允许的最小行为变更：Runtime 的 exact-PWL witness construction，以及 Adapter 在 normal owner
安装前后以**同一** Runtime dry-run witness 取代“retained delta 静态覆盖未来 seam/horizon”的
错误代理条件。任何 commit 前仍必须保留 exact owner/profile/snapshot identity，以及既有
session/CAS/pin/runtime-bit checks；不改变它们。

## 3. 明确禁止

- 不修改 Filter、cross-section、SurfaceValidator、A5 geometry、margin 或任何数值参数；
- 不修改 TubeEpochManager、gvf_manager、H2、CAS、pin、session、runtime bit、CMake、launch 或 AGENTS；
- 不新增 gate、mode、state、reason、enum、diagnostic schema、ROS parameter 或 latch；
- 不跨 owner/path revision/generation/session/map sequence/immutable snapshot 复用 profile、sample、
  geometry、clearance 或 witness；
- 不把 `CERTIFICATE_DENIED`、unknown、domain 不足或有限搜索 no-witness 改称 `U=empty`；
- 不进入 P2a/P2b，不启动、连接、终止或影响用户 ROS/process。

## 4. T1–T3 实现合同

### T1 — Runtime oracle

1. 建立的 witness 从 exact current phase 的 retained delta 和 previous final port 出发；
2. 每段使用 exact PWL current/terminal/crossed-knot containment，因而在 knot 之间也连续成立；
3. 每段均受现有 port projection 的 amplitude、slew、non-reverse、tangent、regularity 合同约束；
4. future geometry/guidance 只从同一 immutable owner 的 evaluator 取得，且 Runtime 交叉检查；
5. 任何 contract/profile/domain/owner failure 或没有构造出 witness，只产生既有
   `CERTIFICATE_DENIED`，并且不 commit retained delta 或 previous port。

### T2 — 100-cycle ownership/state commit

测试需以 test-owned A/B owner、不同 revision/map/snapshot 和 accepted/unavailable/denied/
replacement 交错输入，证明 cycle 1–99 没有 selected commit 时 retained delta 与 previous port
逐 bit 不变；cycle 100 只有 T1 witness 有效才 normal selected 并完成一次 commit。owner/map/
session 交错必须零 commit，不能污染 live Runtime。

### T3 — normal owner acceptance

Adapter 的 normal owner acceptance 必须使用上述同一 dry-run continuous witness，不能要求
retained delta 在未来 seam/horizon 的静态截面仍原样出现。必测：

1. one-sided `[+0.125, ...]`，但真实 retained port 已为正且能连续执行，允许 normal acceptance；
2. raw current interval 排除实际 retained port，拒绝且不污染；
3. profile domain、unknown/current categorical evidence 或 owner/map/session provenance 不一致，
   fail-closed 且零 commit。

## 5. 必须验证和停止条件

完成后运行 focused build/tests（至少 `phase_offset_runtime_test`、
`phase_offset_matched_adapter_test`，以及既有 P1/H2 focused regressions 可运行部分）、dependency
searches、`git diff --check`、白名单 diff 审计和 `git status --short`。写入 self-audit 后立即停止。

若完成本阶段需要触及禁止文件/行为、引入任何新 gate/schema、修改安全数值、放宽 current retained
containment、跨 owner/map reuse，或任一 P1 fixture 不通过，停止并报告；不得推进 P2。

## 6. 执行前自审

```text
SOURCE_AUDIT_COMPLETE=yes
EXISTING_DFS_IS_NOT_A_SET_EMPTY_ORACLE=yes
EXACT_PWL_HELD_STEP_CHECKS_EXIST=yes
STATIC_RETAINED_AT_FUTURE_SEAM_PROXY_IDENTIFIED=yes
RUNTIME_DRY_RUN_IS_NON_MUTATING=yes
WHITELIST_CONFIRMED=yes
P2_NOT_AUTHORIZED=yes
```

该自审完成后，产品修改仅限本规范第 2 节。
