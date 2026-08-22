# A6/H2 跨目标新任务 neutral rebase 自审

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_SELF_AUDIT
DATE=2026-08-19
STAGE=A6_H2_CROSS_GOAL_NEUTRAL_REBASE
IMPLEMENTATION_STATUS=STATIC_AND_FOCUSED_TESTS_PASS
DYNAMIC_STATUS=B_FRONTEND_RECOVERS__A_TO_B_END_TO_END_NOT_CLAIMED
A7_STATUS=NOT_ENTERED__NOT_AUTHORIZED
```

## 结论

本阶段完成了新导航任务的 neutral rebase：A 的已执行非零 offset 不再阻塞 B 的初始 planner frontend。静态测试、相关回归测试和完整编译均通过。一次不重启 `formation_planning` 的动态运行中，A 到达后 B 成功规划、安装 PointPhaseV2 frontend，并产生正常 `VEL_MATCH_GOVERNOR` 命令。

本阶段边界是 A 已经 `POINT_GOAL REACHED`、FSM 已进入 `WAIT_TARGET` 后再发送 B。用户已
明确裁定不考虑 `EXEC_TRAJ` 中途直接 retarget；该行为没有在本阶段增加状态机跳转，也不在
本文验收声明内。

但该长反向 B 随后在既有的同目标 nonzero H2 future-seam/local-path-exhaustion 处进入 fail-closed HOLD，未到达 B。因此不得声称 A->B 动态验收 PASS，也没有以缩短 B、调参、amplitude=0、observe-only 或重启节点来掩盖此边界。

终审发现的 P1 已补修：此前 command-side reset 没有退休 timer-owned `TubeEpochManager`、active profile 和 path cache，且把 `source_revision_` 清为零，理论上可令 B 的 revision 1 与 A 的 active profile 等价。现在 reset 只原子发布新的 private task generation；下一次 timer tick 独占消费它、重建其持久状态后才接受 B request。source revision 也跨任务单调递增。该 token 不改变 ROS schema、参数或 TubeEpochManager 本身。

## 实现核对

修改仅归因于本阶段白名单中的下列文件；工作树本来就包含大量其他用户改动和未跟踪文件，未重置、清理或修改它们。

- `phase_offset_navigation/phase_offset_runtime.{h,cpp}` 和 `test/runtime_test.cpp`
  - 新增 `PhaseOffsetRuntime::resetForNewNavigationTask()`：清除 `delta`、previous final port、profile elapsed/start/completed/recentering、preflight data/revision；不改变已验证的 configuration validity。
  - 普通 prepare/H2 路径不调用此接口。
- `bspline_traj/integration/phase_offset_matched_adapter.{h,cpp}` 和对应测试
  - 新增 exact-session `resetForNewNavigationTask(expected, retired)`；在 `runtime_command_mutex_` 内拒绝 stale session（零变更），退休 pair/pin/control evidence、清空旧 source identity/preflight bookkeeping、关闭并清零 warm-up gate，然后 neutralize Runtime。
  - P1：reset 只发布 atomic private `task_generation`，不从 command thread 写 timer-owned manager。每个 request/epoch/control snapshot 带该 token；timer tick 首先消费新 token，清空 cache/profile/one-shot state 和 old epoch slots，并重建 `TubeEpochManager`。`source_revision_` 不再归零，仍以新 source identity 递增，避免跨任务 profile-equivalence collision。
  - P1 TOCTOU：专用 `task_publication_mutex_` 是 reset/finalize 的线性化 barrier；二者均按 `publication -> runtime_command` 顺序取得锁。finalize 在 barrier 内做 generation revalidation、diagnostic/marker publication、candidate/epoch store 和 pair refresh；reset 只有在已开始的旧 finalize 完成后才清槽并返回。昂贵 tube build 在 barrier 外，reset 不在该锁内调用 ROS publish。
  - P1 inactive TOCTOU：timer inactive request 的 cache/profile/epoch cleanup、DELETE publication 和 one-shot counter 也在该 barrier 内，并在锁内再次验证 generation；故 reset 后暂停恢复的 A inactive tick 不能清 B slots 或发出 A DELETE。
  - ordinary `retirePathTubeAuthority()` 保持同目标语义，仍保留 Runtime state；task reset 不清除 `failure_latched_` 或 fatal reason。
- `bspline_traj/gvf_manager.{h,cpp}` 和 `test/gvf_switch_policy_test.cpp`
  - 新增 manager task reset，锁序为 `frontend_apply -> path_tube_handoff -> authoritative_phase -> Adapter Runtime`。
  - 新目标会取消 pending/completed/deferred handoff，令旧 frontend 通过既有 clear mailbox 失效，重新同步 session，并使 authoritative phase 无效。
  - `goalCallback` 和 auto-start 使用此新任务路径；原 `resetUnifiedPhaseV2()` 继续保留给同目标 H2 retirement。初始化诊断现在区分 planner B-spline、mapped path、optional Tube/Pair bootstrap、neutral frontend install 和新任务 authority reset。

没有改动 TubeBuilder、TubeEpochManager、Filter、SurfaceValidator、ContinuousPhasePath、planner、launch/config/CMake/parameters、ROS message/schema 或 A7。Runtime source 的扫描也没有 ROS include 或 `ros::` 依赖。

## 静态验证

在此工作树、已完成的 build 上重新执行，全部通过：

| Binary | Result |
| --- | --- |
| `phase_offset_runtime_test` | 35/35 |
| `phase_offset_matched_adapter_test` | 73/73 |
| `gvf_switch_policy_test` | 84/84 |
| `phase_offset_tube_epoch_manager_test` | 51/51 |
| `continuous_phase_path_test` | 10/10 |
| `phase_offset_tube_filter_test` | 15/15 |
| `phase_offset_tube_builder_test` | 9/9 |

特别覆盖：Runtime 任务 reset 的 neutral/idempotence/ordinary-prepare 不重置；Adapter 的 atomic reset、stale no-op、fresh warm-up/bootstrap、fatal latch 保留；Manager 的新任务 mailbox/deferred/session/phase 失效。新增 `NewTaskResetDefersTimerStateRetirementAndPreventsRevisionCollision` 先建立真实 A timer manager/profile/cache，再持有 old H2 pin/prepared CAS，验证 reset 后 stale token 是 no-op、old timer completion 和 old final CAS 均失败；B 的 first command 没有旧 profile，first timer tick 才重建 manager，并以新的 source revision 从 epoch/install count 1 开始。`NewTaskResetLinearizesAgainstPausedOldTimerPublication` 在 finalize generation check 后确定性暂停、并发执行 reset，证明 reset 被 barrier 阻塞；释放 completion 后 reset 清空全部旧 slots，旧 completion 无法在 reset 返回后回填或发布。

`NewTaskResetRejectsPausedOldInactiveCleanupAfterBSlotsAppear` 在旧 A inactive tick 读取 request 后暂停，完成 reset 并设置 B slots 后恢复；旧 tick 因 barrier 内 generation recheck 返回 false，B candidate/epoch/control slots 保持原指针。

`NewTaskResetRejectsPausedOldDeactivateProducerAfterBUpdate` 覆盖 public `deactivate()`：A 在捕获旧 generation 后暂停，reset 与 B update 完成后恢复；其 locked recheck 拒绝 A，B 的 latest build/control slots 和 `command_active_` 保持不变。

`source /opt/ros/noetic/setup.bash && catkin_make -j2` 已通过。全工作树的 `catkin_make run_tests -j2` 不能作为总测试门槛：它在开始执行测试前重建既有 `uav_utils-test` 时失败，链接器报 `/usr/bin/ld: cannot find -luav_utils`。这不是本阶段文件或链接依赖的改动。

工作树根目录的 `ctest --output-on-failure` 正常结束但报告 `No tests were found!!!`；本仓库的 ROS tests 由 catkin targets 承载，故上述直接执行的 binaries 是本阶段可执行的测试证据。

`phase_offset_tube_epoch_integration_test` 当前为 6/10，四个已存在的失败已直接复现：

| Test | Assertion |
| --- | --- |
| `ArmedTrackingRecoveryKeepsSelectedThenReturnsToRolling` | line 204: `adapter.update(...99*kDt)` expected `true`, actual `false` |
| `ArmedEsdfTrackingRecoveryUsesLatestFreeSnapshotWithoutCertificateDenial` | line 265: `adapter.update(armed, output)` expected `true`, actual `false` |
| `PathRevisionTrackingRecoveryKeepsSelectedWithoutAttemptingA6Continuation` | line 309: `adapter.update(...99*kDt)` expected `true`, actual `false` |
| `GateOpeningDoesNotAdvanceAnEquivalentActiveEpoch` | line 351: `output.selected` expected `true`, actual `false` |

该 test 未调用 `resetForNewNavigationTask` 或 `retirePathTubeAuthority`，且其对 unpaired epoch/gate selection 的预期与既有 H2 的 Pair-before-nonzero 合同冲突。修复它需要超出本阶段白名单的 H2/Tube 行为更改，因此未作修改。

`git diff --check` 通过。因为基线工作树很脏，白名单审计按本阶段归因完成，而不是将无关的 `git status` 项误归入此提交。

## 动态证据与准确边界

隔离环境：

```text
ROS_MASTER_URI=http://127.0.0.1:11355
ROS_HOME=/tmp/codex_a6_h2_cross_goal_20260819_11355/ros_home
launch=phase_offset_esdf_tube_single.launch
override=phase_offset_manual_observe_only:=false
formation_planning SHA-256=a49f9fa57213efdccab40b91b974e5b1e864fe1008463a24965b5159fee3b013
```

有效参数记录在 `roslaunch.log`：manual mode、`observe_only=False`、amplitude `0.1`、warmup cycles `100`、tube source `esdf`、`min_certified_forward_w=0.4`。验收时没有改变任何参数。

同一个 `formation_planning` 进程的日志顺序如下（行号以 `/tmp/codex_a6_h2_cross_goal_20260819_11355/roslaunch.log` 为准）：

1. A `(8, 0, 1)` 的 frontend 已运行，line 417 记录 `[GVF][POINT_GOAL][REACHED] distance=0.187`，随后进入 `WAIT_TARGET`。
2. line 458 收到 B `(-8, 0, 1)`，无需节点重启即从 `WAIT_TARGET` 转入 `GEN_NEW_TRAJ`。
3. Kino/B-spline 成功后，line 466 的 `[GVF][POINT_PHASE_V2][INIT] ... exact_path=1` 和 line 468 的 `point_phase_v2 init ... EXEC_TRAJ` 表明 B 的初始 frontend 被安装。
4. line 470 已是 `final_cmd_source=VEL_MATCH_GOVERNOR fallback_reason=none`，所以原来的 “initial finite frontend failed -> permanent initialization HOLD” 阻塞已消失。
5. B 运行至 line 594 时发生 `NONZERO_OFFSET_HANDOFF_RECOVERY_REQUIRED current_w=9.323485 required_seam_w=9.723485 old_path_end=9.435256`；这是同目标 nonzero H2 的 future seam 不足，而非 B 初始化或跨目标旧 authority 泄漏。line 599 随后为 `GOVERNOR_INVALID_HOLD fallback_reason=all_candidates_path_end_clamped`。

因此本运行只证明本阶段的跨目标初始化恢复，不能证明 B 到达。继续解决该 terminal same-goal H2 recovery 需要 planner/H2/recovery-owner 方面的授权，不属于 A6/H2 cross-goal neutral rebase；本阶段在此停止，未进入 A7。

### P1 timer-generation 复跑

P1 修补后的 binary 在一个新的、隔离的 master 上复跑了同一观察（没有改参数或缩短 B）：

```text
ROS_MASTER_URI=http://127.0.0.1:11356
ROS_HOME=/tmp/codex_a6_h2_p1_cross_goal_20260819_11356/ros_home
formation_planning SHA-256=b42ff2759548b327cd710fcad64b6e6f8901772075d04b4f09db9cf9c4ada913
roslaunch.log SHA-256=66b806ff17c5895248cb3131613ca3821c25e96f3c141aa2e035e5f671dbb5b1
```

同一进程中，line 314 接收 A，line 433 为 A `POINT_GOAL][REACHED]`；line 479 接收 B，line 487 为 B 的 `POINT_PHASE_V2][INIT]`，line 491 已为 `VEL_MATCH_GOVERNOR fallback_reason=none`。这直接确认 P1 timer reset 后 B 不继承导致初始 frontend/HOLD 的旧 timer state。B 仍在 line 614 出现既有 `NONZERO_OFFSET_HANDOFF_RECOVERY_REQUIRED`，line 619 为 `all_candidates_path_end_clamped`；因此 P1 复跑同样不构成 B arrival 或 A->B PASS。

原始证据保存在：

- `/tmp/codex_a6_h2_cross_goal_20260819_11355/roslaunch.log`
  - SHA-256: `4af9777fa23449269409fd0aaea990efd25b5826bb92af4d7c5e857e4a3734f7`
- `/tmp/codex_a6_h2_cross_goal_20260819_11355/cross_goal_runtime.bag`
  - SHA-256: `ffc6881f8ea30ff70f38760d513af32c93974e4be231c85df77498009004b2c2`
- `/tmp/codex_a6_h2_cross_goal_20260819_11355/roscore.log` and `rosbag_runtime.log`

## Cleanup

Only task-owned PID records were used: roscore `579073`, roslaunch `579196`, rosbag `579400`, and runtime rosbag `579603`. `579603` received SIGINT and exited with its child; the runtime bag was finalized. The roslaunch wrapper had already exited but its isolated nodes and master were still registered on port 11355; they were shut down through that task-owned ROS master using `rosnode kill -a`, then the master received SIGINT. A subsequent process scan found no surviving task-owned ROS node or master.

`cross_goal.bag.active` (609050 bytes) predates this cleanup as an already-dead recorder (`579400` was absent); it remains an unindexed recovery artifact (`rosbag info` requests `rosbag reindex`). It was not mistaken for a live process and no unrelated user ROS process was terminated.

P1 复跑的 11356 master 使用 `rosnode kill -a` 关闭其专属节点后退出，随后确认该 port 无 listener。清理时一个辅助 shell 的 PID 筛选错误地匹配并中断了该 shell 自身；没有影响任何 ROS node 或用户进程。最终 process/port scan 仅见该检查命令本身，不见 11356 master/node。
