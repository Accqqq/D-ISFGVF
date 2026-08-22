# Continuous-inset and planner/tube contract resolution plan

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=CONTINUOUS_INSET_CONTRACT_RESOLUTION
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=true
USER_AUTHORIZATION=explicit continuous Terra/xhigh delegation
PLANNER_PATH_CHANGE_ALLOWED=false
PLANNER_PARAMETER_TUNING_ALLOWED=false
FILTER_CHANGE_ALLOWED=false
MARGIN_TUNING_ALLOWED=false
SWARM_CHANGE_ALLOWED=false
```

日期：2026-08-18
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. 固定架构

planner/KinoA*/B-spline 的输出 `p(w)` 是 tube 名义中心线：

```text
r(w, δ) = p(w) + N(w)δ
```

单机执行固定 `δ_ref=0`。非零 retained/swarm offset 只能在一个已经证明中心线安全的
tube 内使用，不能营救 `δ=0` 不安全的路径。

禁止修改 planner 路径生成、`planning/safe_distance`、exact PWL Filter、Runtime、
margin 数值、map inflation、launch、消息或 swarm。

## 2. T0：真实同-build 证据

使用实际一次 TubeBuilder/Filter/Validator 链，关联同一个：

```text
path revision
tube revision
immutable snapshot sequence/stamp/resolution
current_w
preferred/retained delta
```

每个实际 knot 必须记录：

```text
p(w), N(w)
pre-inset lower/upper
continuous_inset
post-inset raw lower/upper
Filter input/output lower/upper
Validator cover/requested radius
contains_zero at each stage
```

优先复用现有 `TubeProfile::raw_build_samples`、`validator_knot_evidence` 和 clearance
audit sidecar。若需要 ROS 采集，只能使用 task-owned private master/ROS_HOME 和 `/tmp`
证据；不得新增持久 ROS diagnostics schema，也不得把 standalone synthetic test 当作
实际 planner 证据。

## 3. T1：分支判定

### 分支 A：inset 误删零

若同一个实际 knot 满足：

```text
pre-inset contains zero = true
post-inset contains zero = false
```

才允许修改 Builder/Validator 的误差职责。不得直接删除 inset 或制造无证明的 `{0}`；
必须利用完整 Validator ribbon cover 证明 `δ=0` 沿保留 w 段连续安全，并增加窄/宽 tube
测试。

### 分支 B：pre-inset 已排除零

若：

```text
pre-inset contains zero = false
```

则认定当前 planner nominal clearance 与 tube robust clearance contract 不一致或
路径真实不足。保持 fail-closed；不得通过降低 residual、tracking bound、map margin、
提高 planner 可达性参数或选择非零 retained offset 绕过。

### 分支 C：Filter/Validator 排除

若 pre/post-inset 都含零而 Filter 排除，保持 exact PWL 数学并修复调用链测试；若 Filter
含零而 Validator 排除，修正连续证明/证据映射，不放宽 required clearance 或 cover。

## 4. 条件性实现

只有 T1 证明存在实现错误时才落代码修复。单机成功条件仍为：

```text
current base centerline clearance safe
full retained segment zero-line certificate true
retained/reference delta = 0
```

真实不足、unknown、out-of-map 和无连续证书必须等待或 fail-closed。

## 5. 文件白名单

```text
docs/Codex_Continuous_Inset_Contract_Resolution_Execution_Plan_2026-08-18.md
docs/Codex_Continuous_Inset_Contract_Resolution_Self_Audit_2026-08-18.md

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_clearance_audit.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp
src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/CMakeLists.txt
```

Adapter changes, if needed, are measurement-only sidecar plumbing: no ROS schema, gate, mode,
planner algorithm, or control behavior changes.

## 6. 验收与自动推进

- 实际同-build provenance 可复核；不能用旧 binary 或 synthetic-only evidence 冒充。
- 分支 A 才允许修 inset/cover；分支 B 明确记录 robust contract mismatch。
- 单机 `δ=0`、unknown、真实 deficit、Filter、Validator、EpochManager 回归通过。
- 完整 workspace build、`git diff --check`、白名单审计和 self-audit 完成。

用户已授权 `AUTO_ADVANCE=true`，T0→T1→T2 自动推进；证据不足时保持 fail-closed 并记录边界，
不扩大到 planner 参数或 swarm。
