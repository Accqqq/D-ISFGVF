# A5 generic OffsetTube continuous-cover correction — execution specification

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5_GENERIC_OFFSET_TUBE_CONTINUOUS_COVER
IMPLEMENTATION_AUTHORIZED=true
USER_AUTHORIZATION=explicit Terra/xhigh delegation on 2026-08-18
AUTO_ADVANCE_WITHIN_THIS_SPEC=true
AUTO_ADVANCE_TO_A7=false
PLANNER_CHANGE_ALLOWED=false
FILTER_MATH_CHANGE_ALLOWED=false
MARGIN_NUMERIC_TUNING_ALLOWED=false
```

日期：2026-08-18  
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. Objective

修正通用 Tube 构造中把 planner 中心线 `delta=0` 挤出的问题。当前已知 first-false：

```text
w = 2.431715365
pre-inset raw       = [-0.08125, 1.62500]
continuous_inset    = 0.10000 m
post-inset raw      = [ 0.01875, 1.52500]
filtered            = [ 0.022804744, 1.520945256]
```

`delta=0` 必须是通用 `OffsetTubeCertificate` 的普通可行点；不能为单机另造一套安全
语义，也不能让 Tube 自己选择或制造 delta。

目标是让 Tube 使用精确 planner 曲线、法向、曲率和连续 cell cover 计算尚未被其它
证据覆盖的离散化误差，而不是对每个横截面无条件执行固定全分辨率 inset。

## 2. Fixed semantics

```text
p(w) = planner final C2 nominal centerline
r(w,delta) = p(w) + N(w) * delta
I(w) = one generic certified offset interval
```

- 单机的 matched port 零意图自然产生 `delta=0`；它不是一个特殊选择分支。
- 集群 matched port 产生非零期望，Runtime 只能在同一 `I(w)` 内连续投影。
- TubeBuilder/Filter/SurfaceValidator 不拥有意图，也不输出最终控制 delta。
- 真实障碍、unknown、out-of-map、无法证明连续 cover 的情况必须 fail-closed。
- 不得把旧 Tube、planner path-only 或非零偏移作为零点营救。

## 3. Mandatory execution order

### C0 — reproduce and prove current geometry

1. 读取 `AGENTS.md`、本规范和以下审计：
   - `docs/Codex_A5_Clearance_Accounting_Knot_Self_Audit_2026-08-18.md`
   - `docs/Codex_A5_A6_Active_PathTubePair_Continuity_Recovery_Self_Audit_2026-08-18.md`
2. 记录 HEAD、既有 dirty worktree、相关 binary hashes/build IDs。
3. 在 task-owned loopback ROS master/ROS_HOME 中复用原 `pillar.pcd` 和 checked-in launch；
   不接触用户 ROS master/process。
4. 复现目标 knot 及相邻 knot，保存同一 path revision、same snapshot、exact path state、
   normal/curvature、pre-inset interval、current inset、Filter 和 Validator evidence。

### C1 — cell geometry/accounting design

在实现前明确写出一个可验证的 cell 误差上界，至少说明：

```text
曲线 p(w) 的中点偏差上界
法向 N(w) 的变化上界
曲率/delta 对 r(w,delta) 的影响
cell 内横截面边界变化
snapshot voxel/discretization 误差
已经由 SurfaceValidator cover 计入的部分
仍需由 Tube interval inset 扣除的 residual 部分
```

禁止以“经验上改成 0.05”代替证明。若无法得到安全上界，保持现有 fail-closed 并在
self-audit 中报告，不实施行为修复。

### C2 — conditional generic implementation

只有 C1 证明当前固定 inset 过度计费或责任重复，才允许：

- 把 `continuous_inset` 改为按曲线 cell/局部几何计算的 residual inset；或
- 将连续 cover 责任集中到 SurfaceValidator，并让 TubeBuilder 保留未侵蚀 raw interval；
- 保持 exact PWL Filter 的数学和参数不变；
- 保证最终 `I(w)` 对所有 delta 使用同一套证书；
- 保证 `delta=0` 只有在 pre-inset 和连续 cover 都通过时才保留。

如果 C1 证明 `0.10 m` 是真实且必要的未覆盖误差，不修改产品行为，报告 planner/path
或地图分辨率边界。

### C3 — regression and dynamic evidence

至少覆盖：

1. 精确曲线、pre-inset 含零、连续 cover 通过 → 最终 interval 仍含零；
2. 曲率/法向变化大的曲线 → 不得因折线近似伪造安全；
3. 真实中间 cell 障碍 → 仍排除对应区间或 fail-closed；
4. unknown/out-of-map → fail-closed；
5. 非零 delta 与零 delta 使用同一 generic certificate；
6. Filter 未被放宽，旧 Tube/Runtime/Adapter 回归保持通过。

若 C2 成功，使用 task-owned active ESDF episode 验证 Candidate/Certified epoch、zero
containment、matched-port output 和 A6 pair handoff；不得把 observe-only 到达算作 Tube
动态通过。

## 4. File whitelist

```text
docs/Codex_A5_Generic_OffsetTube_Continuous_Cover_Execution_Spec_2026-08-18.md
docs/Codex_A5_Generic_OffsetTube_Continuous_Cover_Self_Audit_2026-08-18.md

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp
```

Do not modify CMake, planner/Kino/B-spline, SDF map implementation, launch/config/map,
Filter implementation, Runtime/MatchedAdapter control semantics, ROS message schemas or swarm
modules. If a necessary edit is outside the whitelist, stop and report the exact boundary.

## 5. Required verification and stop boundary

Run affected Tube/clearance tests, all completed-stage regressions, full build, dependency-boundary
searches, `git diff --check`, final status and whitelist audit. Write a self-audit with the proof,
changed files, exact interval before/after, test results and dynamic evidence.

This stage must stop after A5 generic Tube evidence. It must not advance to A7 or change planner
clearance parameters without a new execution specification.
