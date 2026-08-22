# Tube V2 G14 用户分段 launch 双 owner 只读审计执行规范

```text
DOCUMENT_ROLE=READ_ONLY_OWNER_AUDIT_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_G14
IMPLEMENTATION_AUTHORIZED=false
AUTO_ADVANCE=READ_ONLY_EVIDENCE_TO_DEFINITIVE_OWNER_CLASSIFICATION
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
REASONING_EFFORT=MAX
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
PRODUCT_SOURCE_EDITS_ALLOWED=false
PRODUCT_PARAMETER_EDITS_ALLOWED=false
```

日期：2026-08-21  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

执行分工：主代理负责本规范、调度、证据边界、复杂度/安全审核和最终验收；
同一个 Luna Max 单代理负责连续执行全部只读审计、private ROS 复现、证据整理、自审和
handoff 更新。

Luna Max 禁止创建、调用、委派、请求或等待任何子代理；不得把日志分析、GDB、ROS、
测试、脚本、分类或文档交给其他模型。执行中不得停下来等待用户确认；某一种只读测量
方式不可用时，按本规范规定的 fallback 顺序继续，不能自行转入产品实现。

## 0. 本阶段结论边界

G5 已完成且仍成立的安全修正只有：

```text
exact current-anchor sampled fallback 不再重复计入 half voxel
nondegenerate sampled cover 不变
certified cover 不变
clearance + cover + epsilon 不变
9-point query、regularity、UNKNOWN/OOM/OCCUPIED fail-closed 不变
```

G13 对 unchanged binary 的 exact terminal capture/replay 证明：

```text
classification=B_CONTINUOUS_COVER_REQUEST_CLEARANCE_INSUFFICIENT
planner minimum margin=+0.021131738630078112
request minimum margin=-0.029575109312983072
production query == independent closed-AABB replay, 9/9
```

因此不得删除 cover、epsilon、9 点或 fail-closed。G13 在 goal 后约 0.3 秒捕获 terminal
即结束，没有覆盖完整导航、后续 Tube 构建容量或 H2 liveness；它不是完整任务到达验收。

用户本次真实分段 launch 运行同时暴露两个待归因现象：

1. Tube 多次变成 `ZERO_ONLY_PLANNER_BASELINE`，RViz 中 nonzero ribbon 基本不可见；
2. 第五个目标中 nonzero authority 已经执行，但 H2 replacement staging 返回 false，旧路径
   耗尽后进入 `GOVERNOR_INVALID_HOLD/all_candidates_path_end_clamped`。

G14 的唯一任务是把这两个 owner 分开并确定 exact first-false。G14 不修代码。

## 1. 冻结的用户真实运行

### 1.1 原始证据

用户粘贴日志：

```text
/home/cxq/.codex/attachments/5c774867-f1ff-44d2-8171-4432b517c7aa/pasted-text.txt
```

对应完整 ROS 日志：

```text
/home/cxq/.ros/log/eea738ea-9d60-11f1-8034-f759afa1ebcc/rosout.log
```

只允许读取该目录；不得删除、移动、压缩、改写或把它当成 task-owned cleanup 目标。
读取 roslaunch 日志时只提取第一条 `roslaunch starting with args` 和进程/Build-ID 所需字段；
禁止复制或发布完整 environment，因为其中可能包含无关凭据。

### 1.2 两段启动

真实运行使用：

```bash
roslaunch so3_quadrotor_simulator simulator.launch

roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_amplitude:=0.10 \
  phase_offset_manual_observe_only:=false \
  phase_offset_manual_profile_period:=2.0 \
  phase_offset_manual_tube_source:=esdf \
  phase_offset_tube_fixed_delta_max:=0.04 \
  phase_offset_tube_cloud_obstacle_set_complete:=true \
  phase_offset_tube_preincluded_map_uncertainty:=0.10
```

未提供的 launch 参数保持 checked-in 默认值。不得额外启动
`phase_offset_esdf_tube_single.launch`；否则会产生重名节点和不同 orchestration。

### 1.3 五个目标的真实序列

目标均由 RViz 发布，消息 `z=0`，现有 callback 生成实际飞行高度语义：

```text
G1 = ( 7.381,   0.378, 0.0) -> REACHED distance=0.184
G2 = (-6.823,  -0.685, 0.0) -> REACHED distance=0.194
G3 = ( 0.537,  12.745, 0.0) -> REACHED distance=0.184
G4 = (-0.301, -13.714, 0.0) -> REACHED distance=0.189
G5 = (-7.913,  13.439, 0.0) -> NOT REACHED; terminal hold about 4.96 m from goal
```

复现必须按状态推进：只有观察到前一目标的 `[POINT_GOAL][REACHED]` 后才发布下一目标；
不得用固定短 sleep 在飞行中覆盖目标。G5 后运行到首次 stable path-end HOLD 证据完整，再清理。

### 1.4 已知 Tube 现象

完整日志表明 zero baseline 不止一种：

```text
full-width query-budget exhaustion:
  queries=249993, max_queries=250000, limit_exceeded=1

inward search attempted but no candidate certified:
  attempts commonly 99 or 101
  last_queries ranges from 117 to tens of thousands
  last_limit_exceeded=1 is not by itself query-budget proof；它也可能表示 depth exhaustion
```

所以 G14-A 不能把所有不可见 Tube 都粗略写成“250000 不够”。必须逐个 zero event 分类：

```text
Z1 FULL_WIDTH_QUERY_BUDGET_EXHAUSTION
Z2 INWARD_SEARCH_NO_CERTIFIED_CANDIDATE
Z3 RAW_OR_FILTER_ZERO_CAPACITY
Z4 OTHER_FAIL_CLOSED_TERMINAL
```

### 1.5 已知卡死链

第五目标的冻结链路：

```text
OFFSET_BOOTSTRAP COMMITTED
pair_generation=1 session=43
selected_runtime_executed
retained_delta: 0 -> -0.000384455

command activation later denied:
execution_mode=5
certificate_denied=1
fatal_control_failure=0
invalid_reason=short-horizon exact-port witness unavailable
U+ first_failure_step=4 category=projector
detail=constraint=joint port polygon; joint port feasible polygon is empty
U_safe same first-failure category/detail

H2 replan:
reason=executed_authority
stage_success=0
replan not installed

old path:
path_w_end=24.464
remaining_w -> 0
goal distance remains about 4.96

governor:
candidate_count=36
valid_count=0
path_end_clamped_count=36
fallback_reason=all_candidates_path_end_clamped
```

`stage_success=0` 说明该次 H2 在 `stageFutureSeamPathTubeTransaction()` 内失败；该次没有进入
pending handoff，因此 prepare/finalize/consume 不是这一个 replan 的 first-false。仍需检查同一
generation 前后的完整生命周期，防止把别的 callback 的事实混进来。

## 2. 不可改变的合同与明确禁区

以下全部不变：

1. `max_query_samples=250000`；
2. `max_subdivision_depth=12`；
3. planner safe distance、cover、half-voxel、epsilon、regularity margin；
4. current anchor first、current-connected certified segment；
5. UNKNOWN、OUT_OF_MAP、OCCUPIED、UNAVAILABLE、insufficient clearance fail-closed；
6. local-PWL geometry certificate、PathTubePair identity/session/generation/map provenance；
7. exact retained delta、previous final port、U+/U_safe 和 CAS 原子性；
8. planner、A*/B-spline、C2、governor、速度、加速度、lookahead、replan period；
9. 用户现有 launch、map 和 dirty worktree。

禁止：

- 提高 query/depth limit；
- 降低 clearance、cover、epsilon、regularity 或 tracking bound；
- 加 gate、cache、retry、timer、parameter、module、第二 Validator、第二 C2；
- 为 pillar map、第五目标或某个 phase 写特例；
- 把 101 次 inward search 简单改成“多试/少试几次”；
- 新增永久 ROS 日志、topic、message/schema；
- 修改任何产品源码、test、launch、config、CMake 或生成产品 binary；
- reset/clean/checkout/restore/stash/rebase；
- attach、signal、kill 或清理用户 ROS/PID；
- 用 `pkill`、按名字全局 kill 或占用 11311 的用户 master。

## 3. G14-T0：只读冻结与证据清单

执行前完整读取：

```text
AGENTS.md
handoff.md 顶部 G13
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G3_Surface_Validator_Proof_Capacity_Self_Audit_2026-08-21.md
docs/Codex_Tube_V2_G5_Exact_Current_Anchor_Sampled_Cover_Self_Audit_2026-08-21.md
docs/Codex_Tube_V2_G6_Inward_Search_Limit_Flag_Read_Only_Audit_2026-08-21.md
docs/Codex_Tube_V2_G13_ROS_Graph_Recorder_Readiness_Exact_Capture_And_Replay_Spec_2026-08-21.md
docs/Codex_A6_H2_No_Certified_Future_Seam_Findings_Read_Only_2026-08-17.md
docs/Codex_A6_H2_Clean_Active_Acceptance_Read_Only_2026-08-17.md
docs/Codex_A6_H2_Overlap_Certificate_Handoff_Liveness_Execution_Spec_2026-08-19.md
```

建立：

```text
/tmp/tube_v2_g14_owner_audit_20260821_<id>/
```

保存：

- `git rev-parse HEAD`、`git status --short`、`git diff --stat`、`git diff --check`；
- formation binary SHA-256、Build-ID、ELF debug/symbol可用性；
- simulator/test launch、pillar map、下列源码的 SHA-256；
- 用户 ROS 现存 PID、task 前后 starttime 和“不触碰”清单；
- evidence manifest，标明 USER_READ_ONLY、TASK_OWNED、DERIVED 三类。

关键源码：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
```

## 4. G14-T1：先解析用户原始 clean run

在启动任何新 ROS 前，先从原始日志生成确定性 TSV。

### 4.1 goal timeline

`goal_timeline.tsv` 每行：

```text
goal_index
goal_stamp
x y z
initial_path_start_w initial_path_end_w
point_goal_reached_stamp/reached_distance 或 NOT_REACHED
last_installed_path_start_w last_installed_path_end_w
last_phase_w
last_d_goal
```

证明前四目标已结束后再发送下一目标；第五目标才是当前 liveness failure。

### 4.2 zero baseline timeline

`zero_baseline_timeline.tsv` 每行：

```text
goal_index
stamp
candidate_seq
map_seq
first_stop_reason
first_stop_w
invalid_reason
primary_classification Z1/Z2/Z3/Z4
attempt_count if present
last_queries if present
last_limit_exceeded if present
```

分别统计：

- 每个 goal 的 zero event 数量；
- Z1/Z2/Z3/Z4 数量；
- G5 首个 zero event 与 nonzero Pair commit 的先后关系；
- RViz nonzero Tube 缺失是否由持续 zero baseline 直接解释；
- 不得从 `last_limit_exceeded=1` 推断 query cap，除非 queries 落在 budget terminal 区间。

### 4.3 H2 causal timeline

`h2_user_timeline.tsv` 以 exact stamp 排序：

```text
G5 init/replan installs
full-width/inward zero events
bootstrap stage attempts
Pair COMMITTED generation/session
first selected+valid nonzero command
first retained_delta != 0
first certificate_denied/fatal flag
first accepted planner candidate after executed authority
H2 stage_success
pending/committed/consumed lifecycle if any
last installed frontend range
first all_candidates_path_end_clamped
stable HOLD and d_goal
```

不能把 `command_activation=not_selected_or_invalid` 当成 H2 first-false；它是同一 authority
的 command-side fact。必须进一步判断 H2 staging 是否因同一个 dry-run/Tube 事实失败。

## 5. G14-A：Tube proof-capacity / zero-baseline owner

### 5.1 必须捕获的每次 Validator 证据

对目标 zero event 的 full-width attempt，以及其存在时的每个 inward attempt，记录：

```text
goal_index
candidate_seq / tube_revision
source_revision
snapshot_sequence / resolution
current_w
requested preview start/end
profile sample count
cell_geometry_certified
certified_cell_count
attempt_kind = FULL_WIDTH | INWARD
family = BOTH_SIDED | POSITIVE_ONLY | NEGATIVE_ONLY | NA
scale
level
inward_attempt_ordinal
validate_return
classification
first_failure_reason / first_failure_w
limit_exceeded
query_sample_count
geometry_cell_count
clearance_leaf_cell_count
prequery_cover_split_count
max_depth_observed
split_w_count
split_v_count
split_both_count
anisotropic_split_count
min/max_cover_radius
max_requested_clearance
min_clearance_margin
certified_start/end_w
```

校验：

```text
query_sample_count % 9 == 0
query_sample_count == 9 * clearance_leaf_cell_count
geometry_cell_count >= clearance_leaf_cell_count
query budget terminal iff next 9-point leaf would exceed 250000
```

若 budget check 在调用 query 前拒绝该 leaf，`clearance_leaf_cell_count` 仍应只表示实际执行了
ClearanceQuery 的 leaf；不要把拒绝 leaf 计入等式。

### 5.2 route 与递归 owner 判定

必须区分：

```text
A_SAMPLED_NONDECOMPOSABLE_BOTH_SPLIT
  profile/cell 没有可用 certified decomposition，cover-too-large 后 w+v 同时 split

B_CERTIFIED_DECOMPOSITION_BOTH_REQUIRED
  certificate 有效，proof terms 同时要求 w+v split

C_CERTIFICATE_ATTEMPT_FAILED_GEOMETRY_BOTH_SPLIT
  profile 声明 certificate，但 cell certificate/speed/regularity proof 失败，随后保守 both split

D_CLEARANCE_FAILURE_CONSERVATIVE_BOTH_SPLIT
  cover 已可接受并执行 9 点，但 request clearance 不足；失败后历史策略同时 split w+v

E_BUILDER_REPEATED_INWARD_SEARCH
  单次 Validator 未达到 query cap，但 99/101 个确定性 inward candidates 全部失败

F_CURRENT_ANCHOR_TERMINAL
  exact current anchor 自身失败

G_REMOTE_OR_CONNECTOR_CELL_TERMINAL
  current anchor 通过，失败来自远端/connector/new-owner horizon
```

一个 zero event 可以有 primary + secondary owner，例如
`Z2 / E_BUILDER_REPEATED_INWARD_SEARCH / D_CLEARANCE_FAILURE_CONSERVATIVE_BOTH_SPLIT`。

### 5.3 unchanged-binary 只读 GDB 方法

优先使用当前 unchanged formation binary 的自动 continue、无内存写入 GDB Python：

- full-width `surface_validator_.validate()` 返回点捕获 `result.surface_validation`；
- inward `surface_validator_.validate()` 返回点捕获 `inward_validation`、family/scale/level；
- `CertifiedCellCoverRadius()` 的失败分支只对目标 candidate 条件触发，用 source-line hit
  区分 certificate query/invariant/regularity failure；
- `ValidateCell()` 对目标 terminal 的 cover-too-large、clearance failure 和 split decision
  做计数级捕获，不逐点打印 250000 次 query；
- 日志写 `/tmp` TSV，breakpoint command 必须 silent/continue，不在飞行线程人工停留。

GDB 只能 attach 已完成 G13 同等级 task ownership proof 的本次 private formation PID。
不得 attach 用户 PID。不得 `set var`、call mutating function、写内存或发送 signal。

若 source local 被优化掉，fallback 顺序：

1. 使用 DWARF + machine instruction site 读取已有 result 对象；
2. 使用现有 focused test 验证 ABI/field offsets 后再读取；
3. 使用 `/tmp` 只读解析/重放工具处理已捕获值；
4. 仍不足时，完成一份后续 bounded temporary measurement spec，但 G14 不修改产品源码。

### 5.4 Tube-A 最终问题必须回答

1. 为什么用户看到的是几乎没有 nonzero Tube，而不是仅一次短暂 fallback？
2. Z1 query cap 的主要递归 owner 是 A/B/C/D 中哪一个？
3. Z2 的 99/101 attempts 是否是独立复杂度问题，还是同一 clearance/cover事实在不同 scale
   上重复出现？
4. full-width 与 inward budget 是否分别计数，是否存在 Builder 跨 attempt 累计误判？
5. first failure 是 current anchor、current附近、远端还是 H2 connector/new horizon？
6. G3 的 leaf-only query 和 proof-derived split 是否仍按设计生效；若生效，为什么此路径仍爆炸？
7. 不提高 limit、不加 cache、不减少证明义务时，下一阶段最小允许修正边界是什么？

G14 只写修正边界，不实施。

## 6. G14-B：H2/path-end exact first-false owner

### 6.1 manager staging 层级

对 `pair_generation=1 session=43` 后首次 accepted planner candidate 的
`stageFutureSeamPathTubeTransaction()`，只记录第一个 false：

```text
H0 ADAPTER_OR_PIN_UNAVAILABLE
H1 OLD_PAIR_IDENTITY_OR_MANAGER_SLOT
H2 STRUCTURAL_FUTURE_SEAM
H3 C2_FRONTEND_OR_TRIAL_PATH
H4 CONNECTOR_END_OR_REQUIRED_HORIZON
H5 TRIAL_OWNER_SAMPLE_EVALUATION
H6 ADAPTER_STAGE_PATH_TUBE_PAIR
H7 TRANSACTION_CANDIDATE_OR_SESSION
H8 PENDING_SLOT_FINAL_RECHECK
H9 STAGE_SUCCESS
```

每次 accepted replan 一行，不得为同一个 callback 报多个 first-false。

捕获：

```text
stamp
phase/captured_w0
old pair generation/session/source revision/map seq
old owner start/end
old profile certified start/end
old full-sample first/last/count
minimum structural seam
selected seam
trial path start/end
connector_end_w
new_owner_required_horizon_end
snapshot seq
retained delta
previous final port
manager pending/completed slot state
first_false layer
```

### 6.2 adapter stage 层级

若 manager first-false 为 H6，进一步唯一分类现有
`PathTubePairStageFailure`：

```text
INPUT_PRECONDITION
TRANSACTION_PRECONDITION
PAIR_SESSION_RUNTIME_SNAPSHOT
OWNER_EVALUATE
TUBE_BUILD_PRECONDITION
TUBE_RAW_BUILD
TUBE_FILTER
TUBE_SURFACE_VALIDATOR
TUBE_COVERAGE_OR_VALIDATOR
STAGING_DRY_RUN
```

使用 unchanged-binary source-line GDB 捕获 `stagePathTubePair()` 每个 return-false call site；
在 `tube_update_status` 分支记录 candidate raw/filtered/complete、active availability、current
validation、classification、owner coverage 和 map provenance。不得通过加永久 log 获得这些值。

若为 `STAGING_DRY_RUN`，必须记录：

```text
dry_run.valid
execution mode
certificate_denied
fatal_control_failure
genuine_fatal_invariant
failure reason
retained delta and current candidate bounds
tracking error/bound
U+ first failure step/category/detail
U_safe first failure step/category/detail
```

然后与 1787317583.096 的 command-side denial 做 exact identity 对比：同 generation/session、
同 retained delta、同 owner/profile/map、同 projector empty-polygon 事实，还是独立的新-owner
staging failure。

### 6.3 lifecycle 排除法

对该 stage-false callback 验证：

```text
pending handoff created = false
prepare called = false
finalize/CAS called = false
completed mailbox created = false
FSM consume/install = false
old frontend retained = true
```

若其他 callback 曾经 stage success，必须以 generation/session/candidate stamp 分开记录，不能
用它覆盖当前 failure。

### 6.4 H2-B 最终问题必须回答

1. `stage_success=0` 的 exact first-false 是 H0-H8 哪层？
2. 若为 H6，adapter 的 exact enum 是哪一个？
3. 它是否由 Tube Z1/Z2 同一构建失败直接造成？
4. 它是否由 command-side `joint port feasible polygon is empty` 同一事实造成？
5. `fatal_control_failure=0` 为什么仍正确拒绝 nonzero authority/replacement？
6. 卡死直接 owner 是否为：安全拒绝 replacement + 无新 frontend + 旧 path end；还是另有更早
   owner？
7. governor HOLD 是安全后果还是 first-false？
8. 下一阶段最小修正应属于 SurfaceValidator/Builder、Runtime projector、H2 staging，还是
   planner continuation；不得把不同层混成一个“大改 Tube”。

## 7. G14-R：一次 private 用户场景复现

### 7.1 隔离规则

- 先只读记录现有 ROS master/PID；
- 选 fresh unused loopback port；
- fresh `ROS_HOME`、`ROS_LOG_DIR`、evidence root；
- 显式启动 task-owned `roscore`，再执行 §1.2 两段 launch；
- 只清理本任务记录的 process group/PID 和 private port；
- 不触碰用户 11311 master、用户 RViz 或其他 ROS 进程；
- 不使用全局 node name kill。

### 7.2 readiness

发布 G1 前必须通过：

```text
private master reachable
formation_planning executable/hash/Build-ID matches frozen baseline
sim odom present
local map received and usable
expected phase-offset topics present
recorder subscriptions ready
GDB inferior/object identity gate passed
```

不得把固定 log flush 字符串作为 recorder readiness gate。

### 7.3 目标驱动

用一个 task-owned harness 顺序发布 §1.3 五个目标；每个目标只发一次。前四个目标由
`[POINT_GOAL][REACHED]` 驱动下一步。G5 终止条件：

```text
success: G5 reached
or evidence-complete blocker:
  pair selected/executed
  one accepted H2 replan classified
  old remaining_w=0
  all 36 candidates path-end-clamped
  HOLD stable >= 2 s
```

为了保持证据有界，不录完整 `/sim/local_map` 大流。记录：

```text
/rosout
/move_base_simple/goal
/sim/odom
/position_cmd
/particle0/path
phase-offset diagnostics/epoch/raw/cloud（按现有低率语义）
```

local map 只做 readiness/count/hash/provenance；目标 Validator/snapshot值由 GDB `/tmp` TSV
捕获。不得生成无必要的多 GB bag。

### 7.4 复现不完全时

现有用户 clean run 已是动态事实。若 private GDB run 因调试开销没有复现相同时间顺序：

1. 不调参数；
2. 不增加 retry；
3. 用用户 clean log 的 causal timeline + private run 已成功捕获的同层结构证据完成归因；
4. 必要时对目标函数运行现有 unit fixture 的 unchanged-binary machine capture；
5. 仍无法得到 exact first-false 时，只写后续 temporary measurement spec，不能猜 owner。

## 8. 测试、hash、diff 和清理

G14 不改产品；仍需验证当前 binary/测试基线：

```text
TubeSurfaceValidator focused suite
CertifiedTubeBuilder focused suite
CloudOccupancySnapshot suite
cloud occupancy query bridge suite
phase_offset_matched_adapter focused suite
gvf switch/H2 focused suite
git diff --check
```

记录测试前后源码 SHA、formation binary SHA/Build-ID。任何产品 hash 变化都使 G14 动态证据
失效，必须先查明 task 是否意外写入；不得 reset 用户 worktree。

清理证据：

- recorder/goal harness/GDB/launch/roscore 的 task-owned PID 逐个退出；
- private port free；
- task 前用户 PID 的 PID/starttime 仍一致；
- 不删除 `/tmp` evidence；
- 不清理用户 `.ros/log`。

## 9. 必须生成的输出

```text
/tmp/tube_v2_g14_owner_audit_20260821_<id>/manifest.tsv
/tmp/tube_v2_g14_owner_audit_20260821_<id>/goal_timeline.tsv
/tmp/tube_v2_g14_owner_audit_20260821_<id>/zero_baseline_timeline.tsv
/tmp/tube_v2_g14_owner_audit_20260821_<id>/tube_validator_attempts.tsv
/tmp/tube_v2_g14_owner_audit_20260821_<id>/tube_owner_classification.tsv
/tmp/tube_v2_g14_owner_audit_20260821_<id>/h2_user_timeline.tsv
/tmp/tube_v2_g14_owner_audit_20260821_<id>/h2_first_false.tsv
/tmp/tube_v2_g14_owner_audit_20260821_<id>/causal_classification.md
/tmp/tube_v2_g14_owner_audit_20260821_<id>/tests.tsv
/tmp/tube_v2_g14_owner_audit_20260821_<id>/cleanup_external_check.txt
```

更新：

```text
docs/Codex_Tube_V2_G14_User_Split_Launch_Dual_Owner_Read_Only_Audit_Spec_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
```

文档最终必须明确：

```text
TUBE_VISIBILITY_DIRECT_OWNER=<Z + A-G exact class>
TUBE_QUERY_BUDGET_DIRECT_OWNER=<A-D exact class or NOT_APPLICABLE>
INWARD_SEARCH_DIRECT_OWNER=<E plus underlying A-D/F/G class>
H2_STAGE_FIRST_FALSE=<H0-H8>
H2_ADAPTER_FIRST_FALSE=<enum or NOT_APPLICABLE>
H2_AND_TUBE_RELATION=INDEPENDENT | SHARED_TUBE_BUILD_OWNER | SHARED_RUNTIME_WITNESS_OWNER | PARTIALLY_COUPLED
FINAL_HOLD_DIRECT_OWNER=<exact causal statement>
GOVERNOR_HOLD_ROLE=FAIL_CLOSED_CONSEQUENCE | FIRST_FALSE
NEXT_MINIMAL_CORRECTION_BOUNDARY=<one subsystem and invariant boundary>
NO_PRODUCT_IMPLEMENTATION_PERFORMED
```

## 10. 完成条件

G14 只有在以下全部成立时才 COMPLETE：

1. 用户五目标 clean run 已精确重建；
2. zero baseline 每个事件按 Z1-Z4 分类；
3. 至少一个 full-width query-budget terminal 的 Validator 计数 owner 确定；
4. 至少一个 99/101 inward failure 的 attempt/底层 owner 确定；
5. `pair_generation=1 session=43` 的 H2 `stage_success=0` exact first-false 确定；
6. Tube 显示 owner 与最终卡死 owner 的关系确定；
7. 未修改产品、未调参数、未削弱安全证明；
8. 测试/hash/diff/cleanup 通过；
9. spec completion 和 handoff 写完。

```text
DOCUMENT_COMPLETION_ALLOWED=DEFINITIVE_DUAL_OWNER_CLASSIFICATION_ONLY
NO_PRODUCT_IMPLEMENTATION_PERFORMED
```
