# Codex P2a T4–T5：recovery arbitration 与 mailbox 执行规范

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DOCUMENT_STATUS=AUTHORIZED_BY_USER_2026-08-17
STAGE=P2A_T4_T5_RECOVERY_ARBITRATION_MAILBOX
AUTO_ADVANCE=false
P2B=NOT_AUTHORIZED__NO_PHYSICAL_RECOVERY_OWNER
GO=NOT_GRANTED
```

## 1. 已签署目标

本阶段只消除已有 Runtime/epoch 已明确给出的、**当前状态不可执行**拒绝在同一
command cycle 悄悄回落到 `VEL_MATCH_GOVERNOR` 的缺口。它实现一个进程内、
`std::mutex` 保护的单槽 recovery-request mailbox；FSM 是唯一 `exec_state_` writer。

P2a 不产生任何 recovery trajectory 或 PositionCommand，不把 generic HOLD 称为
recovery，也不使 P2b 物理 recovery route 可用。生产代码只形成不可观察的内部
dispatch seam；在没有 P2b owner 的情况下，production 仍然不是 GO。

## 2. 唯一白名单

- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- 本执行规范与本阶段 self-audit。

仅当现有 Adapter 输出无法无猜测地区分 T4 输入时，才可额外修改
`phase_offset_matched_adapter_test.cpp`；实施前必须在本文档和自审中记录这一必要性。

## 3. 冻结合同

- 仅已有 `CERTIFICATE_DENIED`、`CURRENT_OFFSET_OUTSIDE` 或等价的
  `selected=false && valid=false` current-state failure 才可投递 request；observe-only、
  pre-gate、candidate 暂缺而旧 current certificate 仍有效不得触发。
- 同一 command cycle 对可触发拒绝不得继续发布 `VEL_MATCH_GOVERNOR` baseline。
- request 身份绑定既有 authority/owner/session；stale owner/session 零次消费，绝不 remap。
- producer（command、future timer、adapter）只投递。FSM 持锁验证并移出 request；解锁后
  才调用内部 test/dispatch seam。它是唯一可能写 `exec_state_` 的地方。
- 同步只用 `std::mutex`、一个 slot、私有 monotonic ticket 与 consumed marker；不接入或
  改动 H2 CAS/pin/session/runtime-bit。

## 4. 明确禁止

- 不实现 P2b，不新增 physical recovery owner、trajectory、HOLD/emergency 语义、ROS 参数、
  gate/latch、mode/state/reason/enum/diagnostic schema。
- 不改 Runtime/P1、Adapter、TubeEpochManager、Filter/cross-section/margins、安全数值、
  H2、CAS、pin、session、runtime bit、CMake、launch 或 AGENTS。
- 不跨 owner/path revision/generation/session/map/snapshot 复用任何 tube/profile/sample/witness。
- 不修改用户已有 dirty changes；所有编辑使用 `apply_patch`。

## 5. T4–T5 实施与验收

T4 从既有 Adapter output 精确分类并在同 cycle 仲裁：`CERTIFICATE_DENIED`/
`CURRENT_OFFSET_OUTSIDE` 的 invalid current-state case 投递 request，并使 final command
不来自 `VEL_MATCH_GOVERNOR`。P2b 未接入时，不发布伪 recovery command；仅由 test seam
验证 dispatch。其它 non-selected case 保持原有行为。

T5 mailbox 的 producer linearization point 是持锁写入/保持同 identity request 并分配/保留
ticket；consumer linearization point 是持锁 identity/session 验证、移出并标记 consumed。
recovery work 在锁外。测试必须覆盖：两个 producer、重复 command、FSM delay、stale
owner/session、normal-acceptance 竞争、shutdown；断言 exactly once 或 stale zero-consume，
无 deadlock/lost/duplicate/base fallback/H2 authority mutation。

完成后运行 focused build/tests、P1/H2 regressions、dependency search、`git diff --check`、
白名单审计和 `git status --short`。写 self-audit 并停止 P2a；主调度可另行启动 P2b。
