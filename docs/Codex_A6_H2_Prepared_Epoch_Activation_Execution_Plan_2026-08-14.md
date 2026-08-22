# Codex A6-H2：Prepared Epoch 激活执行单

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=P0_5_CURRENT_OFFSET_CLASSIFICATION_ACTIVE
STAGE=A6_H2_PREPARED_EPOCH_REJECTION_CLASSIFICATION
AUTO_ADVANCE=false
```

## 1. 目标

本阶段只分类 H2 future-seam transaction 中 fresh staged
`TubeEpochManager::update()` 的真实拒绝原因。目标不是实施 activation 修补，
也不是让 `candidate_complete` 被强制升级为 Active。

若 new Candidate 不满足既有 current admissibility、map evidence 或
forward-horizon 条件，必须继续不安装；不得手工 promote profile、合成 status、
复用 old profile/sample，或改动 H2 的 commit/pin/CAS 合同。

## 2. 已证实事实

private GDB trace `/tmp/a6_h2_terminal_trace_20260814` 已在一次正常 periodic
replan 中记录：

```text
captured_w0                  3.592115443
old certified end            4.326637835
C2 success seam              4.028267753
certified future seams       7
first false return           buildPreparedTubeEpoch()
```

该 Candidate 的 source revision、map sequence 和 immutable snapshot 都与 request
一致，且 `candidate_complete=true`；但 fresh staged manager 返回
`active_available=false`、`active_current_validation_valid=false`、empty Active
profile。故首个失败在 Adapter 的 prepared-epoch activation，不在 C2、seam
scheduler、old-pair pin、pending slot、CAS 或 Runtime dry-run。

## 3. 已审计接口结论

`buildPreparedTubeEpoch()` 已创建 stack-local `TubeEpochManager` 并调用既有
`update()`。fresh manager 没有 warmup、连续帧数或隐藏 activation gate：

- `FirstEsdfCandidateInstallsEpochOne` 证明 fresh ESDF Candidate 可立即安装；
- `PreparedTubeEpochOwnsVerifiedPathTubeAndRuntimeDryRunDoesNotMutateLiveState`
  证明 prepared local build 不会污染 live manager/Runtime；
- complete Candidate 仍不 Active 的现有分支只有：
  1. current state 不 admissible 或 map indeterminate（通常已有
     `CURRENT_OFFSET_OUTSIDE`）；或
  2. forward certificate 不足（已有 `FORWARD_HORIZON_SHORT`）。

因此目前证据**不能**授权一个“激活修补”：直接 promotion 必然绕过既有安全判据。
必须先证明 Adapter 向 local manager 交付了错误 input/domain，而不是真实拒绝。

## 4. 严格白名单和禁止项

本单仅在 P1 的 input 错误被动态证实后，允许：

```text
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
docs/Codex_A6_H2_Prepared_Epoch_Activation_Execution_Plan_2026-08-14.md
```

不需要、也不允许修改 Adapter header、CMake、TubeEpochManager、TubeBuilder、
TubeFilter、SurfaceValidator、gvf_manager、AGENTS.md、launch/config 或 ROS
diagnostics/message。

禁止新增 gate/mode/state/reason/enum/latch/parameter；禁止放宽
`current_w < future_seam_w`、exact pair/session/runtime-bit CAS、current
admissibility、map evidence 或 forward horizon；禁止复用 old tube/profile/raw
sample/world clearance/bounds；禁止用 live manager 做 staged build。

## 5. 已撤销的 left-prefix / sliced-preview 假设

本文件先前的 P1/P2 假设称：将 staged Candidate 的 preview 从
`[captured_w0, existing_future_horizon_end_w]` 改为
`[prepared_start_w, existing_future_horizon_end_w]` 可能解决 Active 缺失。
该假设现已**撤销且禁止实施**。

独立审核指出 production timer-H2 replacement 调用传入的 sampled new owner 的
起点就是 transaction capture phase：`prepared_start_w == captured_w0`。因此先前
提议的左侧 prefix 在生产 H2 路径为空，P2 是 no-op，不能作为失败解释、更不能作为
产品补丁。此前关于“Filter 左侧样本历史导致本问题”的推断不是动态事实，全部撤回。

不得改 `buildPreparedTubeEpoch()` 的 preview slicing，除非未来另有独立执行单和
production 动态证据证明非空、错误的 Adapter 输入域；本执行单不授权这种修改。

## 6. P0：补全动态拒绝事实（只读）

在任务自有 private ROS/GDB 中，在 `staged_manager.update()` 返回后、聚合 false
判断前读取已有 `TubeEpochStatus` 与 aggregate 每个既有谓词：

```text
state, disposition, reason,
current_state_admissible, current_bounds_valid,
retained_delta_current_inside,
reference/actual clearance status,
forward_horizon_sufficient, certified_forward_w,
candidate_complete, active_available, active_current_validation_valid,
candidate/active source revision and map sequence,
ProfileCoversPreparedRange(), ProfileSamplesMatchOwner()
```

同时保存同一 transaction 的 captured phase、new-owner sample range、frozen snapshot
identity、retained delta 与 C2 seam。不得增加产品日志/diagnostics schema。

## 7. P0 evidence capture、验收与停止

必须保存可以独立复核的：GDB transcript、loaded library 和 build identity、private ROS
logs、复现命令/环境及 task-owned process cleanup evidence。不得把 GDB print 变成产品
diagnostic/log/schema。

P0 成功只表示准确归类 first false；它不表示 H2 已修复。P0 完成后只可写一份基于
该具体 reason 的下一阶段最小执行建议，不实施产品修改。

立即停止并报告：无法在旧 path 尚有 future certified seam 时复现；C2 未成功；first
false 在 `buildPreparedTubeEpoch()` 之前；或要得到答案必须改产品源码、A5 geometry/
Filter/SurfaceValidator、H2 CAS/pin、参数或安全合同。

## 8. 保持的 failure semantics

所有 staged values 保持 local。任一 build/validation/dry-run/pin/CAS 失败时，不创建
pending handoff、不发布 path-only frontend、不改变 live manager 或 Runtime；RAII pin
按既有路径释放，old pair 保持唯一 authority。不得以参数、门控、重试或自动 replan
掩盖失败。

## 9. P0 private-GDB 完成记录（2026-08-16）

唯一作为 P0 结论的运行是 task-owned private master `11696`：

```text
evidence root  /tmp/a6_h2_p0_20260816/run6
adapter build-id d6bb62f6f96ba4389ce8f6abe1e9072f87c7c558
adapter sha256   e0cd24ef443d3c83706292f40caad665132c2b6c44cae26d75fad18a8380ef94
```

`loaded_adapter_maps.txt` 记录 debug inferior 实际映射的是 workspace 的
`devel/lib/libphase_offset_matched_adapter.so`；`gdb.transcript.txt`、
`adapter_build_id.txt`、`binary_sha256.txt`、private ROS logs、pre/post process
snapshots 和 `shutdown_trace.log` 均保存在该目录。只停止了 task-owned GDB、roslaunch
和 roscore；用户 ROS/process 未接入或终止。

在旧 path 尚有 certified future horizon 时，运行完整复现了 periodic replan、C2 和 H2
staging：

```text
captured_w0                  3.4867096991473838
old certified horizon end    4.2785922517897506
future seam                  3.9303552428188300
new-owner samples            40
stagePath input contract     passed
```

真实 first false 位于 `buildPreparedTubeEpoch()` 的 local
`TubeEpochManager::update()` 返回之后，而非 H2 pin/CAS/scheduler：

```text
update_ok                         false
state                             WAITING_FOR_CANDIDATE
disposition                       REJECTED_CANDIDATE
reason                            CURRENT_OFFSET_OUTSIDE
candidate_raw_complete/filter/complete true / true / true
current_bounds_valid              true
retained_delta                    0
retained_delta_current_inside     false
current_state_admissible          false
forward_horizon_sufficient        true
certified_forward_w               0.79188255264236673
active_available/current-valid    false / false
candidate revision/map            2 / 57 (matches request)
active revision/map               0 / 0 (empty fresh-manager slot)
snapshot provenance               matches request
```

该次 status 同时报出 `CurrentSafetyStatus::UNSAFE`，reference/actual clearance
sufficient 均为 false；但 final manager reason 是既有的
`CURRENT_OFFSET_OUTSIDE`。不得把这个 status 误写为“缺少 activation gate”或
“forward horizon short”。

### 9.1 aggregate 条件的精确解释

1362--1383 的左到右 `||` 第一个 true 是
`!staged_update_ok`。因此 C++ short-circuit 后的 coverage/owner predicates 并未在
该 transaction 中执行；不可伪称它们已经动态通过或失败。基于在断点读取的 raw state，
各项为：

```text
!staged_update_ok                         true   <-- first false
!candidate_complete                       false
!active_available                         true
!active_current_validation_valid          true
candidate revision mismatch                false
active revision mismatch                   true  (no Active installed)
candidate map-sequence mismatch            false
active map-sequence mismatch               true  (no Active installed)
snapshot-bit mismatch                      false
active profile revision/source mismatch    true / true (default empty profile)
ProfileCoversPreparedRange                 not evaluated (short-circuit)
ProfileSamplesMatchOwner                   not evaluated (short-circuit)
```

前面 P0 试验中将 stagePath 的 success continuation PC 误标为 `live_pair` failure 的
临时 GDB trace 已撤回；后续 run 的 raw pointer 复核表明
`live.get()==expected_pair.get()`、authority session `1`、pin lease `1`。run6 使用真实
failure-target/aggregate PC，才是本节结论。

## 10. 下一阶段建议：只读 current-offset 分类

P0 已经排除 H2 scheduling、seam、exact pair identity、fresh-manager warmup 和
forward-horizon 作为本 transaction 的首因，但尚未证明“为什么 full new-owner Candidate
在 captured phase 排除了 retained delta=0”。下一阶段只能是独立、evidence-first 的只读
执行单，且至少要在同一 immutable new owner/snapshot 下区分：

1. candidate 的 captured-phase interval 是否确实不含 `delta=0`，以及其上下界、raw/
   filtered provenance；
2. reference、actual 和 base-centerline 的 categorical map result；
3. old authoritative profile 与 new Candidate 在同一 capture phase 的 profile/map
   provenance 差异；
4. 该事实是 new C2 owner 的真实几何/安全走廊不含当前状态，还是 Adapter 为既有 manager
   提供了错误的 immutable input。

该阶段不得调 margin、slope、Filter、SurfaceValidator、lookahead、速度或 map 参数，
不得自动 side-step，也不得令 H2 安装一个 `delta=0` 不可行的 Candidate。若它确认 new
Candidate 的 current interval 本身排除 0，则这是 A5/path-current-state compatibility
或更上游规划问题，不是 A6 prepared-epoch activation 可修的问题；任何理论/模型变更必须
另行授权。当前单到此停止，未实施产品补丁。

## 11. P0.5 current-offset classification（只读授权）

P0 已证实 `CURRENT_OFFSET_OUTSIDE`，但它不是根因分类。本阶段只在同类真实
replacement H2 私有复现中定位 `delta=0` 首次离开区间的层级；不得修改产品源码。

### 11.1 必须记录的同一 transaction 事实

```text
captured_w0 / future_seam_w / prepared profile domain
retained_delta 与其 Runtime/capture 来源
new owner current p, N, r 与 actual position
candidate raw and filtered sample bounds around captured_w0
TubeFilter exact query bounds and query phase/domain behavior
map/clearance categorical result 与完整 margin decomposition
profile source/revision/map provenance
```

必须确定 profile query 使用的是 exact sample、PWL interpolation、endpoint clamp，还是
已经发生 phase/domain mismatch；任何 GDB 只读调用不得变为 product diagnostic/log/schema。

### 11.2 唯一允许的分类结果

```text
A  raw geometry interval 已不含 delta=0
B  raw 含 0，但 exact PWL/Lipschitz filter envelope 首次排除 0
C  raw/filter 均含 0，但 profile query/domain/interpolation 给出错误区间
D  retained_delta 或 captured phase 映射错误
E  current-state contract 对 new owner 的真实要求不是当前 retained state
```

如果结果是 A、B 或 E，H2 不得安装；这不是通过调整参数、自动横移或放宽
`CURRENT_OFFSET_OUTSIDE` 可以解决的问题。如果结果是 C 或 D，后续才可写最小修复
计划，且必须保留 exact pair/session/runtime-bit CAS、pin、future seam 和所有 A5 几何
合同。

### 11.3 P0.5 产物与停止条件

保存 private GDB transcript、loaded-library map/build-id/sha、ROS logs 与 task-owned
cleanup evidence。只在 evidence 明确落入 C 或 D 时，才写下一阶段最小修复计划及文件
白名单；不实施修复。若要获得事实必须改 geometry/Filter/SurfaceValidator、参数、gate、
reason/schema 或用户 ROS，立即停止。

## 12. P0.5 完成记录（2026-08-16）：A，new raw current interval 已排除零偏置

```text
DOCUMENT_STATUS=P0_5_CURRENT_OFFSET_CLASSIFICATION_COMPLETE
CLASSIFICATION=A_RAW_GEOMETRY_INTERVAL_EXCLUDES_RETAINED_DELTA
PRODUCT_CHANGE_AUTHORIZED=false
```

唯一有效证据是 task-owned private master `11697` 的 run3：

```text
evidence root  /tmp/a6_h2_p05_20260816/run3
adapter build-id d6bb62f6f96ba4389ce8f6abe1e9072f87c7c558
adapter sha256   e0cd24ef443d3c83706292f40caad665132c2b6c44cae26d75fad18a8380ef94
formation sha256 0c7c9f6e07b4b3743e507c2a04d87420134b9c4d03a870051048aaebeb004a13
```

`loaded_adapter_maps.txt` 确认 inferior 映射 workspace 的 production adapter；该目录还
保存 transcript、private ROS/ROS_HOME logs、private-master pre/post port/process snapshots
和 cleanup trace。只向任务自有 GDB、roslaunch、roscore 发送 SIGINT；没有接入、终止或
影响用户 `11311` ROS/process。

run1 与 run2 均明确作废，不能作为分类证据：run1 的临时 GDB probe 对
`gdb.Breakpoint.stop` 使用了无效的 monkey patch，第一次 `stagePathTubePair()` 命中后
异常退出；run2 已到达 H2，但其 vector dereference/Eigen decode 失败，故没有得到 raw
interval。两者均未修改产品文件、参数或运行时合同。run3 修正为具名 GDB breakpoint
subclass，并直接从 `staged.candidate_profile.samples` 读取。

### 12.1 同一真实 H2 replacement 的精确事实

```text
captured_w0                 3.5282144601532073
future seam                 3.9291527605033800
old certified horizon end   4.2772818543947091
prepared profile domain     [3.5282144601532073, 4.2772818543947091]
profile source/revision     ESDF / 2
profile raw/filter/complete true / true / true
retained delta              0 (request and status)
new nearest sample          index 0, w=3.5282144601532073, abs error=0
```

因此 `TubeFilter::query()` 的 current query 在 candidate 的第一个精确 sample；既没有
phase/domain mismatch，也没有跨 cell interpolation 或 endpoint clamp。它是既有 exact PWL
query 的 endpoint value（`tube_filter.cpp` 的 domain check、clamp 和 PWL query 语义均保持
未改）。该 query 的实际 bounds 为：

```text
raw interval                [+0.12500000000000014, +1.3250000000000002]
filtered PWL interval       [+0.12500000000000014, +0.86469108791324978]
```

raw lower 已严格大于 retained `delta=0`；Filter 未改变 lower，仅收紧 upper，故
**首次排除发生在 raw geometry（A）而不是 Filter（B）**。同时 raw sample 的
`environment=[+0.12500000000000014,+1.3250000000000002]`、
`environment_contains_zero=false`、`cross_section_reason=NONE`、两个 ray stop 均为
`NONE`，排除“filter/unknown gap 造成排零”的解释。

### 12.2 owner/current/reference/actual、query 与 margin evidence

同一 captured sample 的 new owner 状态为：

```text
p = (3.4985402338025264, 0.32677807850896001, 1.0191386618154241)
N = (0.03374408847510417, 0.99943050608483253, 0)
r = p                         (retained delta = 0)
actual position = (3.4809729189504965, 0.31913753876174461, 1.0379197054193889)
```

实际 map contract 是 immutable cloud-snapshot clearance query（`TubeSource::ESDF`、
snapshot=true、resolution `0.1`），不是 mutable SDF/raw occupancy query。margin
decomposition 为：

```text
full radius = uav 0.25 + map 0.10 + localization 0.05 + tracking 0.15 = 0.55
preincluded map uncertainty = 0.10
residual radius queried against snapshot = 0.45
```

Manager status 从这同一 query 记录 reference/actual clearance 分别为
`0.42677807850895932` / `0.41913753876174387`，均小于 residual `0.45`；故两个
`*_clearance_sufficient=false` 且 `CurrentSafetyStatus::UNSAFE`。`CheckClearance()` 只会
在 categorical `KNOWN_FREE`、certified、finite 后写入这两个非零 clearance 值，故这是
known-free-but-insufficient 的 snapshot result，不是 UNKNOWN/OUT_OF_MAP/SDF fallback。
status 对外保留的 full required clearance 是 `0.55`，但该 snapshot query 的实际输入按既有
preincluded-map accounting 为 residual `0.45`；这里仅记录语义，未改任何 margin。

### 12.3 old/new provenance 和结论边界

```text
old authoritative pair      revision=1, generation=11, session=1, map=51, snapshot=true
new prepared candidate      revision=2, map=57, snapshot=true, ESDF, tube_revision=1
```

P0 已证明 pin/pair/session 与 staged provenance 合同成立；P0.5 又证明 captured phase、
retained delta 和 exact profile query 均一致。因此 C（query/domain/interpolation）和 D
（retained delta/captured phase mapping）不是本 transaction 的解释；E 也不是在 A6
activation 层可修的异常合同。根因事实是：**new C2 owner 的真实 raw current corridor
不包含仍被保留的零偏置状态**。

这不是 A6/H2 prepared activation、TubeFilter 或 margin 参数的修复事项。H2 必须继续拒绝，
不得安装此 Candidate；严禁任何 activation promotion、A5 geometry/Filter/
SurfaceValidator/margin/slope/lookahead/speed/parameter 改动，亦不得放宽
`CURRENT_OFFSET_OUTSIDE`。

### 12.4 仅可建议的后续边界（未授权、不得实施）

若另有新执行单，下一阶段只能做上游 **path/current-state compatibility** 的只读调查。其
read-only inspection whitelist 仅为：

```text
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
docs/Codex_A6_H2_Prepared_Epoch_Activation_Execution_Plan_2026-08-14.md
/tmp/a6_h2_p05_20260816/*
```

产品修改 whitelist 为 **none**。该调查必须保留 exact pair/session/runtime CAS、pin、future
seam 和所有 A5 safety contracts；本单至此停止，没有实施任何产品改动。
