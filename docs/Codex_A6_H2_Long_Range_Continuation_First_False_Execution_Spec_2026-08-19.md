# A6/H2 长距离同目标 continuation first-false 执行规范

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DOCUMENT_STATUS=AUTHORIZED_BY_USER_2026-08-19
STAGE=A6_H2_LONG_RANGE_CONTINUATION_FIRST_FALSE
IMPLEMENTATION_AUTHORIZED=ONLY_AFTER_UNIQUE_FAILURE_LAYER_IS_PROVEN
EXECUTOR=Terra_xhigh
AUTO_ADVANCE=false
A7_NOT_AUTHORIZED=true
```

日期：2026-08-19  
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. 前置状态

上一阶段 `A6_H2_CROSS_GOAL_NEUTRAL_REBASE` 已通过独立终审：A 到达并进入
`WAIT_TARGET` 后，不重启节点发送 B，B 能安装新的 PointPhaseV2 frontend，并恢复
`VEL_MATCH_GOVERNOR`。跨任务 Runtime、Pair、timer evidence、DELETE、deactivate producer
均已按 task generation/session 退休。

本阶段不重做该 reset，也不考虑用户已排除的 `EXEC_TRAJ` 中途 retarget。

## 2. 固定动态证据

证据：

```text
/tmp/codex_a6_h2_p1_cross_goal_20260819_11356/roslaunch.log
```

B `(-8,0,1)` 的同目标时间线：

```text
phase_w=0.050   initial path_end=5.715       INIT success
phase_w=0.571   replacement path_end=5.892  C2/H2 install success
phase_w=3.605   replacement path_end=8.927  C2/H2 install success
phase_w=4.403   replacement path_end=9.413  C2/H2 install success

phase_w=7.421   old path_end=9.413
                remaining_w≈1.99
                required future seam=7.821
                Kino/B-spline success
                accepted/near_end replan
                replan not installed

phase_w=9.404   old path_end=9.413
                required future seam=9.804
                NONZERO_OFFSET_HANDOFF_RECOVERY_REQUIRED
                all_candidates_path_end_clamped HOLD
```

因此最终 terminal HOLD 只是后果。真正要定位的是 `phase_w≈7.421` 时尚有足够结构路径
余量的 replacement first false。不得用最终 `w≈9.404` 的空 seam 代替该定位，也不得通过
缩短 B、重启节点、amplitude=0、observe-only 或调参声称修复。

## 3. 唯一目标

把 `phase_w≈7.421` 的未安装唯一归因到以下一层：

```text
F0 exact old Pair/pin/session capture unavailable
F1 certifiedFutureSeamCandidates empty
   F1a terminal structural exhaustion
   F1b certificate horizon lag with structural headroom
F2 all immutable future seam C2 connectors fail
F3 new-owner stagePathTubePair/prepared Tube fails
F4 Runtime exact-PWL dry-run/current retained state rejects
F5 pending/completed transaction slot or manager session rejects
F6 command-boundary prepare/latest categorical/final CAS rejects
F7 completed mailbox consumption/frontend mirror install rejects
```

只有动态证据确定唯一 first-false 层后，才允许在该层做最小修复。

## 4. 诊断要求

优先使用 task-owned private GDB、现有 test hooks、现有日志和 rosbag，不新增 product ROS
message/topic/schema/parameter。若优化构建使断点无法区分，可在白名单内增加临时纯内部
test hook/counter，取得证据后移除；不得留下新的产品 gate/reason enum 或高频日志。

必须记录：

- exact old pair pointer/revision/generation/session/path owner；
- captured `w0`、old path end、certified segment start/end、required seam；
- immutable seam candidate 列表和逐候选 C2 结果；
- `stagePathTubePair`、prepared Tube、Runtime dry-run、pin/slot、prepare/final CAS 的首个 bool；
- 若为 certificate lag，deferred tail 是否保存、timer same-owner refresh 是否发生、为何未消费；
- current retained delta/previous port 是否位于 new exact-PWL profile；
- map snapshot identity/latest categorical veto。

## 5. 允许的修复边界

### 若为 F1b certificate lag

修正现有 deferred planner-tail/refresh 消费生命周期，使它在 exact same owner/session/generation
条件下等待 fresh certified horizon 后只消费一次。不得合成 seam、降低
`min_certified_forward_w` 或复用旧 profile 到新 owner。

### 若为 F2 C2

仅修现有 base-path C2 connector 的通用候选/参数域错误；不得增加 Tube C2，不得为该地图或
目标写特例。必须保持 C2 continuity 和 collision checks。

### 若为 F3/F4

只修 proven ownership/domain/runtime mismatch。不得放宽 Tube、Filter、SurfaceValidator、
margin、tracking bound 或 exact-PWL witness；若 new owner 的真实 current state 不可执行，保持
拒绝并报告需要的新 recovery owner。

### 若为 F5/F6/F7

修正 mailbox/pin/session/CAS 的具体生命周期或 TOCTOU；不得增加非原子 path-only fallback。

### 若只能证明真正 terminal nonzero exhaustion

停止实施并给出 current-state/physical recenter owner 的新阶段边界；不得强制清零 delta、制造
`{0}` Tube 或沿未经证明的新路径切换。

## 6. 白名单

诊断与证据支持的最小修复仅允许：

```text
docs/Codex_A6_H2_Long_Range_Continuation_First_False_Execution_Spec_2026-08-19.md
docs/Codex_A6_H2_Long_Range_Continuation_First_False_Self_Audit_2026-08-19.md

src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
```

若需要修改 TubeBuilder、TubeEpochManager、Filter、SurfaceValidator、ContinuousPhasePath、planner、
launch/config/CMake/message/map/速度/rate/lookahead/tracking/margin 或 AGENTS，停止并报告，不扩展。

## 7. 禁止项

- 不给 planner 增加新的 Tube gate；
- 不降低安全半径或 future seam 距离；
- 不强行把 zero 写入 raw/filtered Tube；
- 不把 planner candidate acceptance 当作 H2 install；
- 不在非零 authority 下 path-only switch；
- 不使用 timer retry storm、无限 replan 或 generic emergency HOLD 作为修复；
- 不进入 A7、swarm intent、CBF 或多机阶段。

## 8. 回归与动态验收

按唯一失败层增加确定性回归，并重跑：

```text
phase_offset_runtime_test
phase_offset_matched_adapter_test
gvf_switch_policy_test
H2/P1/Tube focused suites
catkin_make -j2
git diff --check
白名单/dependency audit
```

动态硬验收必须使用独立 ROS master/ROS_HOME、当前参数和 `pillar.pcd`，同一个
`formation_planning` 进程：

```text
A=(8,0,1) reached
WAIT_TARGET 后发送 B=(-8,0,1)
B frontend INIT
至少一次在 w≈7.4 或等价非终端位置成功 replacement install
B POINT_GOAL REACHED
```

不得出现持续 `all_candidates_path_end_clamped`、持续 `GOVERNOR_INVALID_HOLD`、stale Pair、
old/new owner 混合或重复 retry storm。保存 bag、日志、参数、binary hash、失败层证据和进程清理。

## 9. 停止条件

完成唯一 first-false 归因、证据支持的最小修复、focused/full build、A→B 到达动态验收、
Terra 自审后停止；不得自动进入 A7。若真实边界需要新的 physical recovery owner，写明 blocker
并停止在该边界，不能弱化安全合同。
