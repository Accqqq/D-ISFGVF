# Codex P2a T4–T5：recovery arbitration 与 mailbox 自审

```text
DOCUMENT_ROLE=STAGE_SELF_AUDIT
STAGE=P2A_T4_T5_RECOVERY_ARBITRATION_MAILBOX
EXECUTION_SPEC=Codex_P2a_T4_T5_Recovery_Arbitration_Mailbox_Execution_Spec_2026-08-17.md
T5_MAILBOX_INFRASTRUCTURE=PASS
T4_SAME_CYCLE_NON_FALLBACK=NOT_PASS__P2B_OWNER_NOT_AUTHORIZED
P2A_OVERALL=NOT_GO
P2B=NOT_STARTED
```

## 实施事实

`gvf_manager` 现在有一个与 H2 mailbox 完全分离的、`std::mutex` 保护的单槽
current-state recovery request。它只在 command callback 已捕获且仍持有同一 immutable
`PathTubePair` 时，消费既有 Adapter output：

- `selected=false`、`valid=false`、`RuntimeExecutionStatus::executable=false`，且已有
  `CERTIFICATE_DENIED` 事实；或
- 同一 non-executable 前提下已有 `TubeEpochReason::CURRENT_OFFSET_OUTSIDE`。

observe-only、pre-gate、waiting candidate 和 normal selected output 不会触发。请求携带 exact
pair pointer、source revision、generation、authority session 与私有 monotonic ticket。相同 identity
的重复 producer 保留原 ticket；不同 identity 不能覆盖 slot；FSM consumer 只对 exact live
identity 消费，stale request 被丢弃且 consumed ticket 不前进。shutdown 清空 slot 并停止 intake。

FSM 是唯一生产 consumer。P2b 物理 owner 尚不存在，因此当前 FSM consume 后**不改变**
`exec_state_`，不发布 PositionCommand，也不把 generic HOLD 或 legacy replan 冒充 recovery。
同理，P2a 没有改变 H2 CAS/pin/session/runtime-bit 或任何 tube/filter/margin 参数。

## 测试证据

`gvf_switch_policy_test` 由 68 增至 73 项，其中新增 P2a fixtures 覆盖：

- 仅现有 `CERTIFICATE_DENIED` / `CURRENT_OFFSET_OUTSIDE` 分类；
- 同周期 request staging、identity binding 与对 `exec_state_` 的零写入；
- 两个并发且重复的 producer 共用一个 ticket，H2 session 不变；
- FSM delay 后的 exactly-once consume、stale owner/session 的零次 consume、无覆盖；
- normal acceptance 与 denied request 竞争，以及 shutdown 清理。

本阶段运行的本地验证：

| Test | Result |
| --- | --- |
| `gvf_switch_policy_test` | 73/73 PASS |
| `phase_offset_runtime_test` | 33/33 PASS |
| `phase_offset_matched_adapter_test` | 57/57 PASS |
| `phase_offset_port_projector_test` | 24/24 PASS |
| `phase_offset_tube_epoch_manager_test` | 49/49 PASS |
| `phase_offset_tube_epoch_integration_test` | 10/10 PASS |
| `continuous_phase_path_test` | 6/6 PASS |
| `git diff --check` | PASS |

`ctest --test-dir build` 发现该 catkin workspace 没有注册可由 ctest 枚举的 tests，因此以上使用该
workspace 已构建的既有 gtest binaries。没有启动或连接 ROS master、launch、simulator，亦未接触
用户进程。

## 明确未通过项与停止点

P2a 不可能使 T4 的最终命令合同通过：无 P2b route-specific recovery owner 时，cmd callback
仍可能将 rejected matched output 保留为 legacy guidance 并经 `runVelocityMatchingGovernor()` 发布
`VEL_MATCH_GOVERNOR`。将该行为改为 generic HOLD、旧 path replan 或未证明的 emergency 会伪造
物理恢复，因此没有实施。

故 T5 的 mailbox infrastructure 通过，但 T4 same-cycle non-fallback、P2 全部 GO、真实 ROS
闭环及 P2b 都是 **NOT PASS / NOT STARTED**。下一阶段必须先获得独立 P2b recovery-owner 动态/净空
合同与逐文件授权；不得用本 P2a 作为飞行安全或最终效果验证通过的依据。

## 白名单审计

本阶段只编辑：

- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- 本 P2a execution spec 与 self-audit。

未编辑 Runtime/P1、Adapter、Filter、cross-section、SurfaceValidator、margin、TubeEpochManager、
H2 CAS/pin/session/runtime-bit、CMake、launch、AGENTS 或参数。所有编辑使用 `apply_patch`。
