# Codex P2a-r1：精确 current-state recovery 分类自审

```text
DOCUMENT_ROLE=STAGE_SELF_AUDIT
STAGE=P2A_R1_EXACT_CURRENT_STATE_RECOVERY_CLASSIFICATION
EXECUTION_SPEC=Codex_P2a_r1_Exact_Current_State_Recovery_Classification_Execution_Spec_2026-08-17.md
CLASSIFICATION_FIX=PASS
AUTO_ADVANCE=false
P2B_PHYSICAL_RECOVERY_OWNER=NOT_IMPLEMENTED
```

## 实施事实

`gvf_manager::requiresCurrentStateRecovery()` 仍先要求既有
`selected=false && valid=false`。其后精确分流：

- 既有 `CERTIFICATE_DENIED`（Runtime mode、Runtime flag 或 epoch flag）直接
  投递，即使 Runtime 保留 `executable=true`；
- `CURRENT_OFFSET_OUTSIDE` 仍额外要求 `executable=false`；
- 没有明确 denial 的 observe-only、pre-gate/blocked、waiting candidate 和
  selected normal 结果不投递；outside+executable 的矛盾输入也不投递。

没有改动 request 的 mutex 单槽、ticket、owner/revision/generation/session identity、
FSM consumer，亦没有改变 command publication、P2b dispatch、Runtime、Adapter、
TubeEpoch/H2、CAS/pin/session、A5/Filter 或任何参数。

## 测试证据

`gvf_switch_policy_test` 由 73 增至 74 项。新增的真实 Runtime 输出形态是
`CERTIFICATE_DENIED + executable=true`；它针对同一 immutable identity 只保留一个
ticket，恰好消费一次。分类 fixture 同时覆盖 normal、pre-gate、waiting certificate、
outside non-executable 和 outside+executable。

本地构建与回归结果：

| Target | Result |
| --- | --- |
| `cmake --build build --target gvf_switch_policy_test -j2` | PASS |
| `gvf_switch_policy_test` | 74/74 PASS |
| `phase_offset_runtime_test` | 33/33 PASS |
| `phase_offset_matched_adapter_test` | 57/57 PASS |
| `phase_offset_tube_epoch_manager_test` | 49/49 PASS |
| `phase_offset_tube_epoch_integration_test` | 10/10 PASS |
| `continuous_phase_path_test` | 6/6 PASS |
| `git diff --check` | PASS |

既有 mailbox tests 继续验证 duplicate producer 共享一个 ticket、stale
owner/session 零消费、H2 authority session 不变、normal 不投递和 shutdown 清槽。

## 白名单与停止点

本小阶段编辑仅限：

- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- 本 execution spec 与本 self-audit。

未编辑 header；没有新增 gate/mode/reason/diagnostic、mailbox 结构或产品参数。工作区在
开始前已有用户/既有阶段的广泛 dirty changes；本阶段未清理、还原或改动白名单以外文件。

P2a-r1 的分类缺口已修复并通过验证。按 `AUTO_ADVANCE=false`，本小阶段在此停止；P2b
physical recovery owner 仍需由主调度的独立授权阶段处理，不能把本 r1 当作 recovery
trajectory 或发布闭环的完成。
