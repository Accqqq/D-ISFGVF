# Tube V2 G2 Offset Authority First-False and Minimal Correction Execution Spec

~~~text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=TUBE_V2_G2
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=READ_ONLY_FREEZE_TO_MATCHED_BASELINE_TO_ATTRIBUTION_TO_HIT_LAYER_CORRECTION_TO_VALIDATION_ONLY
EXECUTOR=GPT_5_6_LUNA_MAX_SINGLE_AGENT
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
~~~

日期：2026-08-21  
工作区：/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws  
执行分工：主代理负责规格、调度、关键 gate 审核、白名单复核和最终验收；
Luna max 单代理负责读取证据、实现、测试、隔离 ROS 运行、自审和 handoff。
Luna max 禁止创建、调用、委派、请求或等待任何子代理；不得把分析、编码、测试、
launch 或证据处理交给其他模型。

前置阶段：

- A6-P2 已证明 Pair COMMITTED -> Runtime selected/executed -> retained delta 显著非零；
- Tube V2 G1 已把 Filter 改为 raw-knot-preserving cell-local PWL；
- G1 focused tests、required CTest、全量 catkin_make -j2 和 git diff --check 已通过；
- G1 两次 after run 均到达目标并有完整 raw current anchors，但没有观察到 Pair authority；
- G1 停止状态为 G1_IMPLEMENTATION_COMPLETE_DYNAMIC_ACCEPTANCE_UNMET。

本文件授权同一阶段内连续执行，不要求每个小步骤等待用户确认；但连续执行不扩展文件
白名单、不授权下一架构阶段，也不允许通过新增门控或参数调优制造通过。

## 0. 本阶段唯一目标

先消除 T0 before 与 G1 after 的运行成熟度差异，再找到 offset authority 未出现的唯一
first-false。若未修改代码的 readiness-matched run 已恢复 Pair authority，停止于
“时序不可比已确认”，不得为追求代码改动而修改 production。

只有 readiness-matched 基线仍未恢复，并且证据精确命中现有
SurfaceValidator/CertifiedTubeBuilder 语义时，才允许做对应层的最小安全修正：

~~~text
complete raw current-connected local-PWL profile
-> preserve a current-connected, continuously certified nonzero ribbon segment
-> active profile classification OFFSET_CERTIFIED
-> existing bootstrap predicate opens
-> existing Pair / Runtime / H2 lifecycle proceeds unchanged
~~~

本阶段不是 backward viability、Runtime/QP、H2、planner 或参数阶段。

## 1. 已冻结的事实与未决因果

### 1.1 T0 before

证据目录：

/tmp/tube_v2_t0_20260820_225509/

已知：

- 40/40 raw current anchors found+complete；
- goal 到达 internal (8,0,2)；
- map observation sequence 在关键 bootstrap 附近约 932--939；
- 先有 TUBE_BUILD_PRECONDITION，随后 Pair COMMITTED；
- command selected_runtime_executed；
- retained delta 0 -> -0.000384455，后续多次 selected nonzero；
- launch 后约 80 秒以上才发送 goal。

### 1.2 G1 after

证据目录：

- /tmp/tube_v2_g1_after_20260820_232029/
- /tmp/tube_v2_g1_after_20260820_232633/

已知：

- 两次均到达 internal (8,0,2)；
- raw current anchors 分别 22/22、20/20 found+complete；
- 无 all_candidates_path_end_clamped；
- 无 Pair COMMITTED、selected 或 retained nonzero；
- Run 2 有 certificate_denied=1、transient_blocked=2、fatal=0；
- map observation sequence 约 129--158；
- launch 后约 19 秒发送 goal；
- zero gate 最终已打开，active profile complete/current validation 正常；
- 但 active profile 长时间 obstacle_certified=0、
  active_display_certified=0，未成为 OFFSET_CERTIFIED。

### 1.3 当前代码因果

现有 bootstrap predicate 位于：

src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

PhaseOffsetMatchedAdapter::requiresPathTubePairBootstrapLocked() 要求：

1. active profile classification 是 OFFSET_CERTIFIED；
2. zero gate open；
3. 无 failure latch；
4. 无现有 Pair；
5. Runtime 有 pending/active offset intent；
6. Runtime 尚未执行 offset authority。

现有 certified pipeline 位于：

src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp

当前语义为：

~~~text
SurfaceValidator false
-> CollapseToPlannerZeroBaseline()
-> obstacle_certified=false
-> classification=ZERO_ONLY_PLANNER_BASELINE
-> bootstrap predicate remains false
~~~

这条静态链只说明可能的 first-false，不证明 G1 local-PWL 是动态根因。T0 与 after 的
goal 时机、map sequence 和运行成熟度不同；在完成 readiness-matched 基线前，禁止写出
“G1 导致 authority 消失”的因果结论。

## 2. 强制工作区与执行纪律

工作区高度脏，所有现有 tracked/untracked 文件都是用户资产。

禁止：

~~~text
git reset
git clean
git checkout
git restore
git stash / stash pop
rebase
删除、移动、覆盖或格式化不相关文件
创建 commit、branch、tag 或 push
~~~

开始和结束均记录：

~~~text
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
~~~

每次修改前后记录白名单文件 sha256。若发现其他活动任务正在写同一文件，停止写入并向
主代理报告；不得回滚或覆盖其他任务的改动。

ROS 运行必须使用 fresh private master、fresh ROS_HOME、fresh evidence directory。
启动前检查端口；记录 PID/PGID；只清理由本次任务启动的进程；不得 attach、kill 或
重用用户已有 ROS master/node。

## 3. G2-T0：只读证据冻结

在任何 production 或 test 源码修改前完成。

### 3.1 身份与源码快照

记录 HEAD、status、diff stat 和以下文件 sha256：

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_filter.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
~~~

### 3.2 历史 evidence 时间线

只读解析 T0 与两次 G1 after。每个 run 生成一行统一时间线，时间均同时报告：

- absolute ROS timestamp；
- relative-to-roslaunch-start 秒；
- relative-to-first-map-observation 秒；
- relative-to-goal-publish 秒。

至少提取：

1. roscore/roslaunch ready；
2. first diagnostics；
3. first usable immutable map snapshot；
4. first map observation sequence；
5. first active profile complete；
6. first active obstacle_certified=1；
7. first active classification OFFSET_CERTIFIED；
8. zero gate count 开始与 first zero_gate_open=1；
9. goal publish；
10. first real bootstrap requirement；
11. first bootstrap attempt及其 result/stage failure；
12. first Pair COMMITTED；
13. first command selected；
14. first retained nonzero；
15. goal reached；
16. final map observation sequence。

若历史 schema 没有某个字段，写 UNOBSERVABLE_IN_EXISTING_EVIDENCE，不得猜测，不得用
marker、topic 存在或 command delta=0 替代。

输出到新的：

/tmp/tube_v2_g2_t0_20260821_<id>/

至少包括：

~~~text
source_identity.txt
source_hashes.txt
historical_timeline.csv
historical_timeline.md
historical_first_false.md
evidence_manifest.txt
~~~

### 3.3 历史对比结论格式

只能使用以下之一：

~~~text
HISTORICAL_RUNS_NOT_READINESS_MATCHED
HISTORICAL_EVIDENCE_ALREADY_PROVES_SAME_READINESS
HISTORICAL_SCHEMA_INSUFFICIENT_FOR_READINESS_COMPARISON
~~~

当前预期为 HISTORICAL_RUNS_NOT_READINESS_MATCHED，但 Luna 必须从证据重新计算，不得
直接复制本文件结论。

## 4. G2-T1：readiness-matched 未修改代码基线

这一 run 在任何源码修改前执行。它是产品未修改的公平基线，不是修复后验收。

### 4.1 固定场景

~~~text
launch=phase_offset_esdf_tube_single.launch
map=existing pillar.pcd
phase_offset_manual_observe_only:=false
goal message=(8,0,1)
existing internal target semantics=(8,0,2)
all other launch parameters unchanged
~~~

不得修改 launch、参数、速度、warmup、timer、clearance、query limit、goal、地图或
controller。不得使用 observe-only、固定 delta=0、手工减速或重复发 goal。

### 4.2 readiness 协调合同

测试 harness 只能读取现有 topics/diagnostics；它不是 product gate，不得写入仓库
production 逻辑。禁止用固定 sleep 作为 readiness 判据。

首次只读 harness run 已证明：这个 launch 在第一个 goal 前没有 planner owner，因此
phase-offset diagnostics/epoch/raw/cloud profile 均无消息，active profile 和 zero gate
在结构上不可能成为 pre-goal 条件。该 run 的证据必须保留为：

/tmp/tube_v2_g2_readiness_baseline_20260821_015000/

它属于 HARNESS_PRECONDITION_MISMATCH，不是产品、geometry 或 bootstrap failure，也不算
readiness-matched baseline acceptance。

正式 G2-T1b 使用两阶段 readiness，不修改任何 product 行为。

#### Phase A：goal 前 map maturity

只有现有 map topics 的实际消息同时表明以下事实后，才发送一次 goal：

1. private master 与目标 nodes 存活；
2. /sim/local_map PointCloud2 连续非空；
3. /mock_map PointCloud2 连续非空；
4. /particle0sdf_map/occupancy、/particle0sdf_map/esdf 和
   /particle0sdf_map/update_range 均持续发布非空/有效消息；
5. 从本次 launch 开始累计的 /sim/local_map 或等价 10 Hz map update 事件达到 900，
   用于匹配历史 T0 first immutable map sequence 约 902 的成熟度，而不是用 wall-clock
   sleep 近似；
6. 期间目标 nodes 未重启，map header stamp/sequence 未倒退。

900 是历史 evidence-matching harness 阈值，不是 launch 参数、产品 gate 或安全阈值。
禁止把它写入 production。若某 topic 的 ROS message 没有可用 sequence，harness 只能用
本次订阅实际收到的非空消息计数，并在证据中明确写 sequence_unobservable。

Phase A timeout 固定为 launch 后 150 秒。若超时，记录 exact blocking map topic/count、
正常 finalize 和 cleanup；不改 map rate、不伪造计数、不发送 goal。

#### Phase B：goal 后 phase-offset readiness/lifecycle

Phase A 满足后只发送一次原 goal。随后连续记录：

1. first phase-offset diagnostics/raw/cloud/epoch message；
2. first immutable map observation sequence；
3. first complete candidate profile；
4. first active profile present+complete；
5. first active obstacle_certified/display_certified；
6. first zero gate open；
7. failure latch；
8. first bootstrap requirement/attempt；
9. Pair COMMITTED、selected、retained nonzero；
10. goal reached。

Phase B 不等待某个条件后再改变控制；它只观察产品原有状态机。不得在 goal 后补发 goal、
暂停 command、冻结 planner 或人为等待 OFFSET_CERTIFIED。

若 profile/zero gate/Pair 未出现，按 G2-T2/T3 报唯一 first-false；不得把 Phase A map
maturity 写成新的 bootstrap conjunct。

Phase B 最多采集 180 秒，或在 goal reached 且生命周期证据完成后收尾。不得仅因
飞机到达就提前丢失尚在写盘的 rosbag/log；先正常 finalize recorder，再清理进程。

### 4.3 readiness run 输出

使用新目录：

/tmp/tube_v2_g2_readiness_baseline_t1b_20260821_<id>/

必须保存：

~~~text
private_env.txt
run_identity.txt
source_hashes.txt
readiness_events.csv
pre_goal_map_counts.csv
goal_publish.txt
rosbag + bag_info
rosout/launch logs
raw.csv
epoch.csv
manual diagnostics csv
cloud snapshot csv
odom.csv
commands.csv
first_false.md
acceptance_summary.md
PID/PGID manifest
processes_final_check.txt
port_final_check.txt
~~~

### 4.4 未修改代码 gate

若此 run 同时满足：

- goal 到达；
- raw current anchors 全部 found+complete；
- active profile 至少一次 OFFSET_CERTIFIED；
- Pair COMMITTED；
- command selected；
- retained delta 至少一次 abs(delta_after) > 1e-6；
- fatal_control_failure=0；
- 无 all_candidates_path_end_clamped；
- cleanup proof 为空；

则结论为：

~~~text
G2_READINESS_MISMATCH_CONFIRMED
NO_PRODUCTION_CORRECTION_REQUIRED
~~~

此时不得改 production。继续做静态回归、self-audit、handoff 后停止。

若 readiness 超时或 run 未满足 Pair lifecycle，进入 G2-T2 first-false attribution。

## 5. G2-T2：bootstrap predicate first-false

### 5.1 必须区分的结果

对于每个 zero gate 已打开后的 timer opportunity，必须能区分：

~~~text
ZERO_GATE_CLOSED
NO_ACTIVE_PROFILE
ACTIVE_NOT_OFFSET_CERTIFIED
FAILURE_LATCHED
NO_PENDING_INTENT
PAIR_ALREADY_PRESENT
EXECUTED_AUTHORITY_ALREADY_TRUE
REQUIRED
~~~

这些名字只用于 attribution，不得成为新的运行 gate。现有
requiresPathTubePairBootstrapLocked() 的布尔合同和所有原有 conjunct 必须保持。

优先从既有 diagnostics、Pair lifecycle logs 和离线关联恢复结果。只有现有 schema
无法唯一恢复时，才允许加入一个内部、只读、可测试的 evaluation enum/struct：

- 它必须由现有 predicate 的同一组事实计算；
- 现有 bool 结果必须严格等价于 evaluation==REQUIRED；
- 不得持久化，不得 latch，不得影响 Runtime/epoch/Pair；
- 不新增 ROS msg、topic、diagnostic array field、参数、线程或文件输出；
- production 最多允许节流或一次性 lifecycle log；
- NOT_REQUIRED/普通 warmup 不得刷屏。

若加入此可观测性，先完成 deterministic tests，再用新 private run 复验 attribution。
最多允许一次同配置 attribution rerun；不得反复 launch 猜原因。

### 5.2 predicate gate

若 first-false 不是 ACTIVE_NOT_OFFSET_CERTIFIED：

- 只允许修被证据命中的既有生命周期错误；
- 不得顺手改 Tube geometry；
- 若命中层超出本规格条件白名单，停止报告；
- 不得新增 readiness/control gate。

若 first-false 是 ACTIVE_NOT_OFFSET_CERTIFIED，进入 G2-T3。

## 6. G2-T3：OFFSET_CERTIFIED 内层 first-false

必须对同一个 candidate/build/map sequence 关联以下事实，不得混用不同 epoch 或不同
snapshot：

### 6.1 Raw / Builder

- raw_complete；
- current_sample_found/current_sample_complete；
- raw sample count；
- requested and retained preview；
- current knot raw lower/upper；
- 是否存在 abs(bound)>1e-10 的 nonzero raw capacity；
- Builder truncation before/after、first w/reason；
- immutable snapshot sequence/resolution/provenance；
- planner_safe_distance 与实际 requested clearance；
- cell geometry certificate coverage/count。

### 6.2 Filter / G1

- filtered_complete；
- filtered endpoints 是否逐 knot 等于 raw endpoints；
- local PWL lower_w/upper_w；
- boundary_slope_max 是否完全不参与 decision；
- Filter 是否保持 Builder truncation provenance；
- Filter 不得生成 REGULARITY；
- Filter 后是否仍有 nonzero capacity；
- current anchor 是否仍完整。

### 6.3 SurfaceValidator

- surface_validation_attempted；
- current_anchor_valid；
- first_failure_reason 和 first_failure_w；
- query_sample_count；
- limit_exceeded；
- min/max cover radius；
- max requested clearance；
- min clearance margin；
- zero-centerline knot evidence；
- certified start/end；
- truncated before/after；
- Validator 结束后是否仍有 nonzero capacity。

failure 必须精确归入：

~~~text
RAW_NO_NONZERO_CAPACITY
RAW_OR_CURRENT_INCOMPLETE
FILTER_INVALID
FILTER_LOST_NONZERO_CAPACITY
VALIDATOR_CURRENT_ANCHOR_FAILURE
VALIDATOR_REGULARITY
VALIDATOR_UNKNOWN
VALIDATOR_OUT_OF_MAP
VALIDATOR_OCCUPIED
VALIDATOR_CLEARANCE
VALIDATOR_QUERY_LIMIT
VALIDATOR_NO_CURRENT_CONNECTED_CELL
VALIDATOR_REMOTE_TRUNCATION_ONLY
CERTIFIED_BUILDER_WHOLE_PROFILE_COLLAPSE
OTHER_EXACTLY_NAMED
~~~

不得把 Runtime certificate denial、selected=0 或 Pair absence写成 geometry empty。

### 6.4 精确复现

动态 first-false 必须被一个 deterministic C++ test 复现，优先在
tube_surface_validator_test.cpp 或 certified_tube_builder_test.cpp 中使用：

- same-shaped raw local-PWL bounds；
- same current-anchor topology；
- controlled immutable clearance query；
- same failure reason class；
- assertion before repair 说明为何 classification 变为 ZERO_ONLY。

若无法用确定性 test 复现，停止在诊断，不得修改 production 算法。

## 7. G2-T4：条件式最小修复授权

### 7.1 总原则

只有 first-false 和 deterministic reproduction 均命中的层可修改。修复必须满足：

~~~text
never widen beyond raw local-PWL ribbon
never lower required clearance
never reinterpret UNKNOWN/OUT_OF_MAP/OCCUPIED as safe
never raise query/subdivision limits
never restore global envelope or slope cap
preserve only a current-connected continuously certified nonzero segment
otherwise remain ZERO_ONLY_PLANNER_BASELINE
~~~

修复不是为了“让 bootstrap 一定成功”，而是为了避免把已经能由同一 immutable
snapshot、同一 clearance、同一 continuous-cover 证明的非零子 ribbon 因整条 full-width
ribbon 的单次失败而全部丢弃。

### 7.2 若命中 Validator/CertifiedBuilder whole-profile collapse

只在证据证明以下全部条件时授权：

1. raw/filter profile 完整且含 current anchor；
2. Filter 后有非零容量；
3. full-width ribbon 的某个明确 cell/side 失败；
4. 使用同一 snapshot、同一 required clearance、同一 cover/regularity 规则时，存在
   严格位于 raw ribbon 内、包含 delta=0 连通边界的非零子 ribbon；
5. 该子 ribbon 可在 deterministic test 中被现有或修正后的 continuous validator
   完整认证。

允许的最小语义：

- 只向 delta=0 方向收缩 lower/upper，不得向外扩；
- 可分别收缩正负侧，保留一侧非零也是合法 OFFSET_CERTIFIED；
- 只保留 current anchor 所在、在 w 上连续的已认证 cells；
- 远端失败只能截断远端，不得反向清空已认证 current-connected segment；
- 任何最终输出必须再次通过完整 continuous SurfaceValidator；
- 若最终少于两个严格递增 knots、无 nonzero capacity、current cell 无法认证或 final
  revalidation 失败，继续 collapse 为 ZERO_ONLY_PLANNER_BASELINE；
- provenance、first failure、truncation reason、snapshot identity 必须保留；
- OFFSET_CERTIFIED 只能由最终实际非零且 obstacle_certified=true 的 profile 获得。

算法不得引入 launch 参数、调参旋钮、retry loop、sleep 或 second production builder。
若需要 deterministic inward search，其范围必须固定在 raw bound 与 zero 之间，并以
最终 continuous validation 成功为唯一接受条件；搜索中间失败不能被当成安全。

### 7.3 若命中其他层

- Raw/Builder 输入错误：只修 exact current/sample/provenance plumbing；不得改 planner、
  launch 或 Builder clearance semantics。
- Filter 错误：只修 G1 合同回归；不得恢复 slope cap/global envelope。
- Predicate observability错误：只修结果映射，不改变 bool。
- Pair/manager 生命周期错误：只有现有 A6-P2 first-false 明确命中且在条件白名单内才
  可修；不得进入 H2/recenter/authority 重构。
- Runtime denial：记录并停止 Tube 修改；本规格禁止改 Runtime/QP/Projector。

## 8. 文件白名单

### 8.1 始终允许

~~~text
docs/Codex_Tube_V2_G2_Offset_Authority_First_False_And_Minimal_Correction_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G2_Offset_Authority_First_False_And_Minimal_Correction_Self_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
~~~

所有 /tmp/tube_v2_g2_* evidence 与一次性 harness 只放 /tmp，不纳入仓库。

### 8.2 只允许测试修改

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
~~~

只修改与 first-false、最小修复和回归直接相关的测试；不得删除旧 fail-closed 测试来制造
通过。

### 8.3 条件式 production 白名单

只有相应 gate 命中后才允许：

Bootstrap attribution 不足：

~~~text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
~~~

限制：只允许只读 attribution/evaluation/logging，或被精确 first-false 命中的既有
predicate/lifecycle bug；禁止新 gate、authority、thread、schema 或 timer 行为。

Validator/CertifiedBuilder first-false：

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
~~~

优先不改 header/tube_types；只有 deterministic result/evidence 无法表达时才添加最小内部
字段。不得新增 ROS-visible schema。

G1 Filter 回归：

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
~~~

只有证据证明 G1 合同实现错误才允许；不得以“Validator 难认证”为由恢复 global
envelope、slope cap 或 Filter REGULARITY。

### 8.4 永久禁止

~~~text
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_core/**
src/swarm_planner/phase_offset/phase_offset_navigation/src/port_projector.cpp
src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp
planner / Kino A* / B-spline / C2 production
src/swarm_planner/plan_env/**
src/uav_simulator/**
all launch/config/parameter files
CMakeLists.txt / package.xml
ROS msg/topic/schema
Paper proposal
~~~

若真正修复需要上述文件，停止并报告 OUTSIDE_G2_WHITELIST。

## 9. 不可违反的算法与安全边界

1. G1 raw-knot-preserving cell-local PWL 保持；不恢复 fixed slope 0.80 决策。
2. planner 仍拥有 delta=0 neutral baseline；zero-only 不阻止 neutral navigation。
3. OFFSET_CERTIFIED 必须代表非零 ribbon 的独立 continuous obstacle certificate。
4. UNKNOWN、OUT_OF_MAP、OCCUPIED、regularity failure 和资源上限继续 fail-closed。
5. clearance、continuous cover、snapshot resolution 和 margin accounting 不降低、不重复
   扣除，也不调数值。
6. Runtime exact-PWL crossed-knot witness、U+ -> U_safe、fatal semantics 不改。
7. Pair bootstrap、pending activation、H2 single-seam、mailbox/pin/CAS 不重构。
8. 不添加 readiness gate、navigation gate、warmup gate、profile gate 或 control gate。
9. 不以 observe-only、delta=0、延迟 command、减速、改 goal 或改地图规避。
10. 不进入 backward viability、disconnected component recovery、recenter、M4/M6/M7、
    multi-UAV 或论文实验。

## 10. 必须新增或更新的 deterministic tests

具体数量由实际 first-false 决定，但以下合同必须被覆盖。

### 10.1 Predicate attribution

若新增内部 evaluation：

- 每个枚举 reason 都有表驱动测试；
- evaluation==REQUIRED 与原 bool true 完全等价；
- reason 只改变可观测性，不改变 Pair/Runtime/epoch；
- zero gate closed、active absent、zero-only、pending intent absent、Pair present、
  executed authority 各自唯一归因；
- warmup/NOT_REQUIRED 不产生新 authority。

### 10.2 SurfaceValidator

保持并复跑：

- high-slope continuously safe ribbon passes；
- between-knot obstacle rejects；
- interior unsafe with safe endpoints rejects；
- far unsafe suffix truncates but retains current segment；
- UNKNOWN/OUT_OF_MAP/current-anchor failure fail closed；
- query limit fail closed；
- regularity certificate failure fail closed。

若命中 whole-profile collapse，新增：

1. full-width local-PWL ribbon fails at an outer side, but a strict inward
   current-connected nonzero sub-ribbon passes with the same clearance/cover；
2. asymmetric case preserves only the certifiable side；
3. remote bad cell truncates without erasing a safe current-connected segment；
4. current-anchor nonzero ribbon genuinely unsafe still yields zero-only；
5. UNKNOWN/OUT_OF_MAP cannot be rescued by inward search unless the final
   actually queried/certified sub-ribbon contains no such unsafe evidence；
6. query limit is never raised and exhaustion remains zero-only；
7. final accepted ribbon is revalidated end-to-end and every bound is inside
   the original raw/filter bound。

### 10.3 CertifiedTubeBuilder

- raw incomplete never masquerades as baseline；
- true no-capacity remains complete zero-only；
- validator failure provenance is retained；
- certifiable inward nonzero current segment yields
  classification=OFFSET_CERTIFIED、obstacle_certified=true；
- non-certifiable result remains ZERO_ONLY_PLANNER_BASELINE、
  obstacle_certified=false；
- no second Builder invocation or different snapshot query；
- boundary_slope_max compatibility value remains irrelevant。

### 10.4 Epoch / Adapter / Manager

- OFFSET_CERTIFIED candidate can become active under existing install policy；
- zero-only remains observable but cannot bootstrap nonzero authority；
- existing bootstrap Pair COMMITTED path remains unchanged；
- pending intent, Pair present and executed authority semantics remain；
- manager 48/48 and adapter 82/82 baseline tests may increase in count but none
  may be deleted or weakened。

## 11. 构建、回归和静态审计

至少执行：

~~~text
catkin_make -j2 \
  phase_offset_tube_filter_test \
  phase_offset_tube_surface_validator_test \
  phase_offset_certified_tube_builder_test \
  phase_offset_tube_epoch_manager_test \
  phase_offset_runtime_test \
  gvf_switch_policy_test \
  phase_offset_matched_adapter_test

./devel/lib/phase_offset_navigation/phase_offset_tube_filter_test
./devel/lib/phase_offset_navigation/phase_offset_tube_surface_validator_test
./devel/lib/phase_offset_navigation/phase_offset_certified_tube_builder_test
./devel/lib/phase_offset_navigation/phase_offset_tube_epoch_manager_test
./devel/lib/phase_offset_navigation/phase_offset_runtime_test
./devel/lib/bspline_race/gvf_switch_policy_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test

cd build
ctest --output-on-failure -R \
'(_ctest_phase_offset_navigation_gtest_phase_offset_tube_(cross_section|builder|filter|surface_validator)_test|_ctest_phase_offset_navigation_gtest_phase_offset_(certified_tube_builder|runtime|tube_epoch_manager)_test|_ctest_bspline_race_gtest_(gvf_switch_policy|continuous_phase_path|phase_offset_matched_adapter)_test)$'
cd ..

catkin_make -j2
git diff --check
~~~

静态审计必须证明：

- Filter 无 global envelope、range enumeration、slope cap decision 或 Filter REGULARITY；
- no new gate/latch/parameter/thread/topic/schema；
- no clearance/query-limit/warmup/timer/speed change；
- production candidate 仍只有 CertifiedTubeBuilder 一条入口；
- final nonzero profile 必须经过 SurfaceValidator；
- unsafe/UNKNOWN/OUT_OF_MAP/regularity/limit 仍 fail-closed；
- Runtime/QP/Projector/H2/planner hashes 不变；
- 未授权文件无修改。

若 full workspace build 被无关历史缺陷阻塞，保存完整日志并先复跑已授权 target；不得修
白名单外历史缺陷。

## 12. G2-T5：readiness-matched 修复后独立 ROS 验收

只有以下两种情况运行：

1. readiness baseline 已直接通过，用它作为最终 acceptance；
2. 完成命中层最小修复后，用新 private master 做 after acceptance。

after 必须与 G2-T1 使用同一 readiness harness 和固定场景；不得改 readiness 条件或
timeout 来偏袒 after。

最终动态 acceptance：

1. readiness event 完整，goal 只发送一次；
2. goal 到达 internal (8,0,2)；
3. 所有 raw cohorts 的 current anchor found+complete；
4. Filter 后 local-PWL facts 完整，无 fixed-slope truncation；
5. active profile 至少一次 OFFSET_CERTIFIED；
6. obstacle_certified=1 与 active_display_certified=1 可关联到同一 profile/epoch；
7. existing bootstrap predicate 到 REQUIRED；
8. Pair COMMITTED，generation/session/owner 可关联；
9. first command selected_runtime_executed；
10. retained delta 至少一次 abs(after)>1e-6 且 before/after 可关联；
11. Runtime denial 可出现，但 fatal_control_failure=0；
12. 无 all_candidates_path_end_clamped；
13. H2/A6 lifecycle 无新退化；
14. bag/log 正常 finalize；
15. processes_final_check.txt 与 port_final_check.txt 为空。

必须做 before/readiness/after 表格，至少比较：

- goal relative time；
- first map/goal map sequence；
- zero gate open；
- first OFFSET_CERTIFIED；
- validator failure distribution；
- current-connected certified horizon；
- Pair/selected/nonzero；
- Runtime denial/fatal；
- goal result；
- cleanup。

不得把一次偶然未出现 Runtime denial 写成安全改进，也不得把 Runtime denial 写成 Tube
geometry failure。

## 13. 失败处置与停止条件

### 13.1 直接停止

- required edit outside conditional whitelist；
- deterministic reproduction 不成立；
- fix 需要降低 clearance、提高 query limit、调 timer/warmup/speed；
- fix 需要 Runtime/QP/Projector/H2/planner/launch/schema；
- active profile 已 OFFSET_CERTIFIED，但 Pair failure 位于本规格外；
- ROS runtime unavailable；
- 无法隔离用户 ROS 进程；
- unrelated dirty-worktree conflict 无法安全绕过。

### 13.2 允许连续执行

在同一规格内可连续完成：

~~~text
read-only freeze
-> readiness baseline
-> exact first-false
-> deterministic reproduction
-> hit-layer minimal correction
-> focused/full tests
-> readiness-matched after
-> self-audit/handoff
~~~

“连续执行”不等于进入 backward viability、recenter、authority rewrite 或后续 Tube V2
阶段。

### 13.3 最终状态名称

按证据选择唯一状态：

~~~text
G2_READINESS_MISMATCH_CONFIRMED_NO_PRODUCTION_CORRECTION_REQUIRED
G2_FIRST_FALSE_CORRECTED_DYNAMIC_ACCEPTANCE_PASS
G2_DIAGNOSTIC_COMPLETE_FIX_OUTSIDE_WHITELIST
G2_DETERMINISTIC_REPRODUCTION_UNMET
G2_IMPLEMENTATION_COMPLETE_DYNAMIC_ACCEPTANCE_UNMET
G2_BLOCKED_BY_ROS_OR_UNRELATED_WORKSPACE
~~~

## 14. Self-audit 与 handoff

写：

docs/Codex_Tube_V2_G2_Offset_Authority_First_False_And_Minimal_Correction_Self_Audit_2026-08-21.md

必须包含：

- exact final status；
- exact files changed；
- before hashes / after hashes；
- historical readiness comparison；
- readiness baseline timeline；
- predicate first-false；
- Validator/CertifiedBuilder inner first-false；
- deterministic reproduction；
- old/new algorithm only if production changed；
- proof that no new gate was added；
- focused tests、CTest、full build、diff-check；
- readiness-matched dynamic acceptance；
- cleanup proof；
- limitations；
- explicit stop boundary。

更新 /home/cxq/ISF-GVF/handoff.md 顶部，保留全部历史内容。不得修改 Paper proposal。

## 15. 给 Luna max 的执行指令

~~~text
你是本阶段唯一执行代理 gpt-5.6-luna，reasoning=max。
禁止创建、调用、委派或请求任何子代理；所有工作由你单代理完成。

完整读取本 execution spec、AGENTS.md、G1 execution spec、G1 self-audit、
/home/cxq/ISF-GVF/handoff.md，以及本规格点名的当前源码。

严格连续执行 G2-T0 -> G2-T1 -> 必要时 G2-T2/T3 -> 命中层最小修复 ->
tests/build -> readiness-matched ROS acceptance -> self-audit/handoff。

不要停下来等待逐步确认；只有命中本规格真实停止条件、白名单外需求、ROS 隔离不可用
或工作区冲突时才报告阻塞。

不要新增任何产品门控，不调 launch/参数/速度/timer/warmup/clearance/query limit，
不要改 Runtime/QP/Projector/H2/planner，不进入 backward viability。
如果 readiness-matched 未修改代码 run 已通过，禁止为了“有改动”而修改 production。
~~~
