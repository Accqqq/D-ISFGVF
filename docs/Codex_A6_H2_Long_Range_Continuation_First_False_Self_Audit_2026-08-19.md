# A6/H2 长距离 continuation first-false 自审

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_SELF_AUDIT
DATE=2026-08-19
STAGE=A6_H2_LONG_RANGE_CONTINUATION_FIRST_FALSE
UNIQUE_FIRST_FALSE=F1B_CERTIFICATE_HORIZON_LAG
LIFECYCLE_RESULT=WORKING_WHEN_FRESH_CERTIFICATE_EXISTS
IMPLEMENTATION_RESULT=NO_PRODUCT_FIX_AUTHORIZED
DYNAMIC_ACCEPTANCE=A_REACHED__B_NOT_REACHED
STOP_REASON=FRESH_TUBE_CERTIFICATE_REJECTED__OUTSIDE_WHITELIST
A7_STATUS=NOT_ENTERED
```

## 结论

这不是 deferred-tail 的保存、同 owner refresh、consume、C2、Runtime、mailbox 或
CAS 生命周期故障。动态实测显示：当 timer 能生成新的 active certificate 时，Pair 以
相同 owner/revision/session 从 `G` 刷新到 `G+1`，deferred artifact 恰好消费一次，随后
正常 H2 事务继续。因此 F2--F7 均不是该层的首个 `false`。

B 的首个非终端 `false` 是 F1b：旧 Pair 结构路径仍超过 required future seam，但 active
certificate 尚未覆盖它。其后 timer 生成 fresh Tube epoch 的本体拒绝（base-centreline
continuity、candidate completeness 或 reference clearance），不能发布新的 same-owner
Pair，所以 deferred artifact 正确地不消费。让这些 epoch 变为 active 需要修改
`TubeEpochManager`、TubeBuilder 或 SurfaceValidator 的证书合同，均不在本阶段白名单。
在没有新 certificate 的情况下强制消费、复用旧 profile、降低 horizon 或 path-only
切换都违反 H2 合同，未实施。

## 动态证据

GDB attach 被此主机的 yama `ptrace_scope` 拒绝；因此按执行规范使用了白名单内、仅 task
私有、无参数/无 topic/无 schema 的临时 hook。hook 在证据取得后已从源文件移除。

隔离运行：

```text
ROS_MASTER_URI=http://127.0.0.1:11403
ROS_HOME=/tmp/codex_a6_h2_long_false_20260819d/ros_home
launch=phase_offset_esdf_tube_single.launch
override=phase_offset_manual_observe_only:=false
map=pillar.pcd
```

同一个 `formation_planning` 进程先完成 A `(8,0,1)`，进入 `WAIT_TARGET`，再接收 B
`(-8,0,1)`；没有重启、缩短目标、amplitude=0 或 observe-only。

用户已明确排除旧任务仍在 `EXEC_TRAJ` 时直接发送新目标的 mid-flight replacement。本自审、
本阶段实现和动态验收仅覆盖 A 已到达并已进入 `WAIT_TARGET` 后的 B；没有为 mid-flight
replacement 新增状态机分支、测试或 recovery 行为。

### F1b 的 exact capture

本次 B 路径的首次可复现非终端样本为：

```text
Pair: revision=8 generation=8 authority_session=5
w0=2.534224021
required seam=2.934224021
path end=7.105367688
immutable samples=[0.497494798, 7.105367688]
active certificate=[2.339292249, 2.739292249]
terminal_exhaustion=false
canDeferPlannerTailForCertificateLag=true
saveDeferredPlannerTailOnNoCertifiedFutureSeam=true
```

所以候选列表为空的唯一原因是 certificate horizon lag，而不是 terminal structural
exhaustion。已保存的 artifact 保持原 Pair/revision/generation/session；它没有被错误
清除、改 owner 或进入 pending/completed slot。

### Refresh first false

timer request 仍精确引用同一 Pair（revision `8`、generation `8`、session `5`）。其首个
fresh build 在 `w=2.495143818` 返回：

```text
built_ok=false
candidate_complete=true
active_available=false
active_current_validation_valid=false
state=WAITING_FOR_CANDIDATE
reason=BASE_CENTERLINE_CONTINUITY_UNCERTIFIED
certified_forward_w=1.947349720
```

随后该 path 的 fresh attempts 又分别记录为 `CANDIDATE_INCOMPLETE`、
`REFERENCE_CLEARANCE_INSUFFICIENT`。因为没有 active epoch，
`prepareTimerPairRefresh` 和 final pair CAS 并未成为首个失败层；保持 `G=8` 是正确的
fail-closed 行为。

### 生命周期正向对照

同一进程中的 A 给出直接的 F1b 正向控制样本：

```text
w0=7.099889950; required seam=7.499889950; path end=8.357977528
certificate end=7.175028557; terminal=false; defer save=true
timer same-owner Pair commit: generation 5 -> 6
new certificate=[6.414619361, 8.357977528]
deferred tail consume: generation 5 -> 6, exactly once
```

随后 C2 connector 成功、A 到达。此对照排除了“F1b artifact 没保存、timer 不 refresh、G+1
比较错误、consume/slot/prepare/final-CAS 固有故障”的归因。

原始执行规范中固定的 B `w≈7.421` run 也满足同一 F1b 判别式（`path_end=9.413` 大于
`required=7.821`）；其原日志没有 Pair/epoch 内部字段。上述私有 run 补全了所需动态字段，
但其 Kino path 在该非确定性 replanning 条件下取得了不同的等价非终端 phase，未把终点 HOLD
错误当成 first false。

## 范围、清理和停止

- 临时 hook 只在 `gvf_manager.cpp` 与 `phase_offset_matched_adapter.cpp` 中存在，记录后已
  移除；`rg TEMP_A6_H2_FF src/swarm_planner/bspline_traj` 无输出。
- 未修改 TubeBuilder、TubeEpochManager、Filter、SurfaceValidator、planner、launch、map、
  速度、rate、margin、tracking bound 或 ROS schema。
- 私有 11400--11403 masters/nodes 均通过各自 ROS master 的 `rosnode kill -a` 停止；未终止
  用户 ROS 进程。
- 因 B fresh certificate 的首拒位于禁止修改的 Tube certificate owner，本阶段未作产品代码
  修复，也不能声称 A→B arrival 通过。下一步必须由新的、明确允许 Tube certificate/physical
  recovery owner 的执行规范决定；本阶段在此停止。

## 回归

临时 hook 移除后重新完成了 `catkin_make -j2`；其中只有既有的 `gvf_manager.cpp` 编译警告
（signedness 与未覆盖 `INIT` enum），没有错误。下列 focused binaries 均通过：

```text
phase_offset_runtime_test: 35/35
phase_offset_matched_adapter_test: 73/73
gvf_switch_policy_test: 84/84
git diff --check: PASS
```
