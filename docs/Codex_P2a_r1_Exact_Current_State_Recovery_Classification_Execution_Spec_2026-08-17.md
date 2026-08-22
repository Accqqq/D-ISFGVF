# Codex P2a-r1：精确 current-state recovery 分类执行规范

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DOCUMENT_STATUS=AUTHORIZED_BY_USER_2026-08-17
STAGE=P2A_R1_EXACT_CURRENT_STATE_RECOVERY_CLASSIFICATION
AUTO_ADVANCE=false
```

## 已签署目标

本阶段只修正 P2a producer 对既有 Runtime/TubeEpoch current-state denial
事实的分类。P1 Runtime 在已存在 valid current executable port 后出现有限
witness 无法构造时，按既有合同给出 `CERTIFICATE_DENIED` 并保持
`executable=true`；P2a 必须对该明确 denial 投递 recovery request，而不能把
`!executable` 误当作它的必要条件。

`CURRENT_OFFSET_OUTSIDE` 仍只在 `selected=false`、`valid=false`、
`executable=false` 的 current-invalid 输入时可投递。任何 selected/valid、
observe-only、pre-gate、candidate unavailable 或旧 current certificate 仍可执行
的输入均不得投递。矛盾的 outside+executable 输入 fail-closed 且不得投递。

## 唯一白名单

- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- 本执行规范及本阶段 self-audit。

header 只有在无法实现上述最小分类修正时才可修改；本阶段预期无需修改。

## 冻结与禁止

只消费既有 `CERTIFICATE_DENIED` / `CURRENT_OFFSET_OUTSIDE` 事实；不新增
gate、mode、reason、diagnostic、参数或状态。不改 Runtime、Adapter、A5/Filter、
TubeEpochManager、H2、CAS、pin、session 或 mailbox 的同步结构、身份合同和
ticket 语义。不得改变恢复 dispatch 或制造 physical recovery command。

## 验收

新增/更新 `gvf_switch_policy_test` fixtures，至少证明：

- certificate-denied + executable=true 对同一 identity 恰好 staging 一次；
- normal/pre-gate/observe-only 不 staging；
- outside + non-executable staging；outside + executable 矛盾输入不 staging；
- duplicate、stale owner/session 与 H2 identity 合同保持。

运行 GVF focused test、P1/H2 回归、`git diff --check`、白名单审计和最终
`git status --short`，写 self-audit 后停止本小阶段。
