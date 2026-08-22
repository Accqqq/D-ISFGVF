# Codex A5R-0B Cohort-Consistent Raw Profile Measurement Execution Plan

```
DOCUMENT_ROLE=MEASUREMENT_ONLY_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5R-0B
IMPLEMENTATION_AUTHORIZED=true
AUTHORIZATION_SCOPE=GO_TMP_ONLY
AUTO_ADVANCE=false
REPOSITORY_SOURCE_MUTATION_ALLOWED=false
ROS_DIAGNOSTICS_SCHEMA_CHANGE_ALLOWED=false
RUNTIME_GATE_STATE_MODE_REASON_CHANGE_ALLOWED=false
PARAMETER_GEOMETRY_CONTROL_CHANGE_ALLOWED=false
```

> 日期：2026-08-11  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 前置：`Codex_A5R_Tube_Performance_And_Installability_Recovery_Execution_Plan_2026-08-11.md` §4 A5R-0B。  
> 本文件是 A5R-0B 所需的独立 measurement-only 执行单。用户已授权仅在 `/tmp`
> 创建外部 probe、运行本阶段拥有的私有 ROS episode 并生成 sidecar 证据；**不授权**
> 修改源码、参数、launch、几何、控制或 ROS schema。

---

## 0. 结论与授权建议

**GO（仅限外部 `/tmp` 采集）。** 现有构建足以用一个外部 `LD_PRELOAD`
pass-through probe 采集同一次 `TubeEpochManager::update` 的完整 raw
cross-section records、Filter 前后 profile 和 Validator 前后 profile。无需新增 ROS
topic、message、diagnostics schema、参数、state、mode、reason 或任何仓库源码。

本执行单的唯一授权是：

```
ALLOWED_WORK=/tmp/a5r_0b_<run-id> external probe + private owned ROS episode
```

它**不**授权 A5R-1 性能修改、A5R-2/3、A6、A7 或任何仓库实现修改。完成采集、分析和
自审后必须停止。

---

## 1. 已核对的可行性事实

1. 当前动态库导出了以下非 inline C++ 边界，且 ELF relocation 为
   `R_X86_64_JUMP_SLOT`，已有 `/tmp/codex_tube_hotspot_profile_20260811_223738/`
   的同类 profiler 实测使用它们：

   - `FLAG_Race::PhaseOffsetMatchedAdapter::buildTubeEpoch`；
   - `phase_offset_navigation::TubeEpochManager::update`；
   - `TubeBuilder::buildCloudClearance`；
   - `TubeCrossSectionSolver::solve`；
   - `TubeFilter::filter`；
   - `TubeSurfaceValidator::validate`。

2. `TubeBuildRequest` 已持有一个 immutable
   `shared_ptr<const CloudOccupancySnapshot>`；同一 request 建立
   `ClearanceQuery` 后才调用 manager。因此 request 的 snapshot address token、
   observation sequence、stamp 和 resolution 可作为该 build 的 provenance。
3. manager input 已含 `path_source_revision`、`map_observation_sequence`、
   `retained_delta` 和 `current_path.w`。生产调用把 `retained_delta` 原样传给
   `buildCloudClearance(... preferred_delta, current_w, ...)`。
4. `TubeRawSample` 已保留 selected component 的 obstacle/curvature、inset 后
   `raw_lower/raw_upper` 和 `filtered_*` 字段。但 builder、Filter 和 Validator 会在
   成功返回时裁掉不保留的 samples；只在 wrapper 返回后复制 profile 不足以恢复全部
   adaptive raw samples。
5. 为补足第 4 点，probe 必须在同一 builder 调用内部同时：

   - pass-through 包装传入的 `PathStateQuery`，记录它实际返回的
     `(w, PathDifferentialState)`；
   - pass-through 包装 `TubeCrossSectionSolver::solve`，记录每一次实际 solve 的
     input 和 result（包括全部 post-curvature components）；
   - 用原调用次序和 path-state `(p,N)` 单调匹配为每个 solve 赋予实际 adaptive `w`。

这不是重放、二次 query 或重建几何；所有数据都来自原 build 的同一调用链。

### 1.1 已知可观测性边界

- `TubeCrossSectionResult::components` 是**曲率求交之后**的 components。当前 ABI
  不暴露 solver 内部的 pre-curvature raw-component vector。因此 `EMPTY_AFTER_CURVATURE_INTERSECTION`
  可以被准确标为 curvature 排除，但不得伪造其 pre-curvature component bounds。
- Validator 成功并裁剪时，可由其输入/输出 profile 精确定位保留段外的第一个 knot
  和首个被拒 segment。Validator 整体返回 `false` 时，它不会导出 `certified[]`；只能
  报告 `TubeSurfaceValidationResult.first_failure_w/reason` 为“首次报告失败”，不能谎称
  它是已证明的第一个被裁掉 knot。
- components 没有跨 `w` 的持久 identity。相邻 selected components 不相交时必须写为
  `DISJOINT_COMPONENT_SWITCH_OR_IDENTITY_UNRESOLVED` 并从斜率统计剔除；不得把它当连续
  导数。相交也只标为 `OVERLAP_CONTINUITY_CANDIDATE`。

---

## 2. 范围、禁止项与穷尽白名单

### 2.1 唯一目标

在每一个 cohort 内导出并关联：

```
one manager update
  = one path revision
  = one immutable map snapshot
  = one retained_delta / builder preferred_delta
  = one current_w
```

并定位 raw cross-section、curvature/inset、Filter、Validator 四层各自在 current
anchor 前/后的首次排除事实。T1 只使用同一 `cohort_id` 的相邻 raw samples。

### 2.2 明确禁止

- 不修改 `src/`、`CMakeLists.txt`、`package.xml`、launch、test、config、`AGENTS.md` 或
  已有 diagnostics schema；
- 不新增 ROS publisher/subscriber、topic、message、gate、state、mode、reason、证书字段或
  参数；
- 不改变 margin、slope、lookahead/back、rate、速度、饱和、map、path、C2、governor、simulator
  或 SO3；
- 不使用 probe 结果作性能 PASS、ROLLING/Active/Certified PASS 或 `(a)/(b)` 模型结论；
- 不跨 build、path revision、snapshot 或 preferred delta 配对；
- 不 reset/restore/clean/stash-pop/commit/branch/tag/push。

“stage exclusion”是离线证据列，不是新增的运行时 reason/state。

### 2.3 文件白名单

仓库内本阶段唯一可写文件是本执行单：

```
docs/Codex_A5R_0B_Cohort_Consistent_Raw_Profile_Measurement_Execution_Plan_2026-08-11.md
```

获授权的实际采集只可写入一次性根目录：

```
/tmp/a5r_0b_<UTC-run-id>/
  probe/a5r_0b_tube_profile_probe.cpp
  probe/a5r_0b_tube_profile_probe.so
  probe/a5r_0b_analyze.py
  evidence/*.csv
  evidence/*.json
  analysis/*.txt
  analysis/*.sha256
  ros_home/**
  ros_logs/**
```

允许只读的仓库输入仅为：当前 `devel/lib/bspline_race/formation_planning`、
`devel/lib/libphase_offset_matched_adapter.so`、`devel/lib/libphase_offset_navigation.so`，
它们的公开 headers，以及现有、未改参数的
`launch/phase_offset_esdf_tube_single.launch`。不得使用 `catkin_make` 或写入 `build/`、
`devel/`；本阶段的适当 build 仅为 `/tmp` 内 probe 的 `g++ -fPIC -shared` 编译。

---

## 3. 采集机制

### 3.1 透明 wrapper 链

probe 在一个新建、私有、loopback ROS master 下只 preload 到本阶段启动的
`formation_planning`。它不读取或写入 ROS 参数，不广告 ROS 通信接口。

| 边界 | probe 动作 | 不允许的动作 |
|---|---|---|
| `buildTubeEpoch` | 创建 thread-local `cohort_id`；复制 request 的 control/path/snapshot provenance | 修改 request、snapshot、runtime 或 timer |
| `TubeEpochManager::update` | 核对并补齐 revision/map sequence/retained delta/current_w | 改变 input、result 或 manager |
| `buildCloudClearance` | 以调用一次的 logging `PathStateQuery` 转发给原符号；记录其返回的 path states | 二次 path/clearance query、改变参数或 profile |
| `TubeCrossSectionSolver::solve` | 原符号返回后复制 input/result/components | 改 result 或 selected component |
| `TubeFilter::filter` | 原调用前后复制 `TubeProfile` | 改 profile 或返回值 |
| `TubeSurfaceValidator::validate` | 原调用前后复制 profile 和 validation result | 改 profile、result、query 或返回值 |

每个 wrapper 通过 `dlsym(RTLD_NEXT, exact_mangled_symbol)` 找到原符号，并且只调用一次。
若符号缺失、递归解析、TLS 嵌套不平衡、capture overflow 或任何 provenance 不一致，probe
只写失败证据并使本次 measurement **NO-GO**；不能 fallback 到改仓库插桩。

`PathStateQuery` wrapper 只能把每一个原始 query 原样转发一次，再复制其返回值。其记录
用于给 `solve` 记录匹配 `w`，不是重新计算 adaptive sampler。

### 3.2 关联 ID

`cohort_id` 的格式为：

```
<run_uuid>:<pid>:<thread_id>:<monotonic_build_sequence>
```

它由外部 probe 的 `buildTubeEpoch` 入口生成；同一 ID 必须出现在所有 CSV 行。每个
`cohort_id` 的不可变 key 为：

```
(path_source_revision, map_observation_sequence, map_snapshot_instance_token,
 observation_stamp_ns, snapshot_resolution, retained_delta, preferred_delta,
 current_w, tube_revision)
```

其中 `map_snapshot_instance_token` 仅是该 process 内 `shared_ptr.get()` 的十六进制 token，
不能跨运行比较；跨运行 provenance 使用 sequence/stamp/resolution/输入二进制 SHA-256。
`preferred_delta` 必须 bitwise 等于本次 builder 入参，`retained_delta` 必须 bitwise 等于
manager input；两者不等即 cohort 无效。

### 3.3 CSV/JSON 字段定义

所有 CSV 先有 `schema_version,run_uuid,cohort_id` 三列；浮点以 round-trip 精度写出，NaN
显式写 `nan`，不用 `0` 伪装缺失。

| 文件 | 每行对象 | 必填字段 |
|---|---|---|
| `build_context.csv` | 一个 manager update | ID key、`control_sequence`、source、request/completion time、snapshot valid metadata、`manager_update_return`、candidate sequence/status（仅事实，不作 PASS） |
| `path_state_calls.csv` | 一个 builder 内实际 PathStateQuery 调用 | `path_call_index,w,p[3],p_w[3],p_ww[3],valid,returned` |
| `raw_samples.csv` | 一个实际 `TubeCrossSectionSolver::solve` | `solver_call_index,w,w_match_status,p[3],N[3],curvature,preferred_delta,cross_section_reason,result_valid,lower_curvature,upper_curvature,selected_lower_final,selected_upper_final,inset_lower,inset_upper,inset_nonempty,contains_zero_after_inset,contains_preferred_after_inset,positive/negative_termination,full/preincluded/residual_radius` |
| `raw_components.csv` | 一个 post-curvature component | `solver_call_index,component_index,lower,upper,contains_zero,contains_preferred,is_selected` |
| `stage_samples.csv` | 一个 profile snapshot 内 knot | `stage` (`RAW_PROFILE`,`FILTER_INPUT`,`FILTER_OUTPUT`,`VALIDATOR_INPUT`,`VALIDATOR_OUTPUT`)、`stage_index,w,raw_lower,raw_upper,filtered_lower,filtered_upper,lower_w,upper_w,complete` |
| `stage_exclusions.csv` | 一层、一个方向的结论 | `stage,direction,stage_return,input_count,output_count,first_excluded_sample_w,first_excluded_segment_w0,first_excluded_segment_w1,reported_failure_w,reported_failure_reason,certainty` |
| `validator.csv` | 一个 Validator 调用 | `return,complete,current_anchor_valid,truncated_before,truncated_after,limit_exceeded,query_sample_count,certified_start_w,certified_end_w,min_clearance_margin,first_failure_w,first_failure_reason` |
| `t1_pairs.csv` | 一对相邻、允许分析的 raw knots | `left/right_solver_call_index,left/right_w,dw,d_lower_dw,d_upper_dw,abs_d_lower_dw,abs_d_upper_dw,component_relation,accepted,rejection_code` |

`selected_lower_final/upper_final` 是当前 solver 返回的 **selected post-curvature** component，
不能依字段名 `lower_obstacle/upper_obstacle` 误称为 pre-curvature raw obstacle bounds。
`inset_lower = selected_lower_final + snapshot_resolution`，
`inset_upper = selected_upper_final - snapshot_resolution`，必须与实际 builder 的
`raw_lower/raw_upper` bitwise 相同（正常有限值时）。

---

## 4. 四层首次排除的离线定义

所有“首次”均以 `current_w` 为 anchor 分成 `BACKWARD`（递减 `w`）与 `FORWARD`（递增
`w`）两行，避免把 UAV 身后失败误写成前向失败。

1. **raw cross-section**：在完整、按实际 adaptive `w` 排序的 `raw_samples.csv` 中，从
   anchor 向相应方向寻找第一条 `cross_section_reason != NONE` 或 `result_valid=false` 的
   row。写已有 enum reason；若 path-state 映射不唯一，不判定。
2. **curvature/inset**：
   - `CURVATURE_NUMERICAL_FAILURE` 或 `EMPTY_AFTER_CURVATURE_INTERSECTION` 直接记为
     curvature 排除；
   - 否则若 result valid 而 `inset_lower > inset_upper`，记为 inset 排除；
   - 否则该 row 通过此层。不能由此推断未导出的 pre-curvature components。
3. **raw profile 连通截取**：`RAW_PROFILE` 是 builder 围绕 current anchor 保留的连续
   complete segment；完整 raw 表中不在该 segment 的 row 必须标出，不得从 CSV 静默删除。
4. **Filter**：比较 `FILTER_INPUT` 和 `FILTER_OUTPUT` 的有序 `w` 子序列。
   成功时，输出首/尾外紧邻的 input knot 是精确 first excluded sample；前向首个被拒
   segment 为 `[output.back().w, input[next].w]`。若 filter 返回 false，只记
   `FILTER_FAILED_NO_MEMBERSHIP_WITNESS`，不得编造 first knot。
5. **Validator**：同样比较 `VALIDATOR_INPUT/OUTPUT`。成功裁剪时按 knot/segment 精确
   标记；失败时只记录 `first_failure_w/reason`，`certainty=REPORTED_NOT_PROVEN_KNOT`。

每层还分别列出 zero 与 retained/preferred delta 的 containment：raw 使用 inset 后 bounds；
Filter/Validator 只对实际保留 knots 使用其输出 bounds。被阶段排除的 knot 不重新查询来
制造“仍然可行”的结果。

### 4.1 component 分类与正确 T1

对同一 cohort 的相邻 raw knots：

- 只有两个 `w` 均已一一映射、均通过 cross-section/curvature/inset、且二者 selected
  components 都包含同一个 preferred delta、并且 closed intervals 有交集时，写
  `OVERLAP_CONTINUITY_CANDIDATE`；
- selected intervals 不相交、selected component 缺失、或一一映射不成立时，写
  `DISJOINT_COMPONENT_SWITCH_OR_IDENTITY_UNRESOLVED` 或 `UNRESOLVED`；该 pair
  `accepted=false`；
- 不设事后 `(a)/(b)` cutoff。T1 仅报告已接受 pair 的 lower/upper signed 和 absolute
  slope 的 p50/p90/p99/max、样本数和所有 rejection 分类。

这样 component switch/identity ambiguity 永远不会伪装成连续边界导数。

---

## 5. 执行顺序

1. **冻结和预检。** 记录 `git status --short`、tracked diff、`git diff --check`；记录上述
   三个运行时二进制 SHA-256、probe source SHA-256 和完整命令行。已有脏工作树是用户所有，
   只比较本阶段前后，不要求 clean。
2. **ABI self-test。** 在 `/tmp` 编译 probe；以 `nm -D`/`readelf -rW` 和 probe 的
   `dlsym` self-test 验证六个 exact symbol、`RTLD_NEXT` 非空及 pass-through call counter。
   不通过即停止。
3. **纯 C++ 回归。** 在不写仓库的前提下运行已存在的、当前二进制对应的
   `phase_offset_tube_cross_section_test`、`phase_offset_tube_builder_test`、
   `phase_offset_tube_filter_test`、`phase_offset_tube_surface_validator_test`、
   `phase_offset_tube_epoch_manager_test`。这是回归证据，不修改或新建 repository test。
4. **私有 observe-only episode。** 先确认目标 loopback port 未被占用且不会附着用户 ROS
   master；用现有 `phase_offset_esdf_tube_single.launch` 的默认参数（默认
   `phase_offset_manual_observe_only=true`）启动本阶段拥有的进程，并只向
   `formation_planning` preload probe。不得传入参数 override。
5. **采集与停止。** 至少得到一个有完整 raw/Filter/Validator trace 的 cohort；是否重复
   由数据覆盖决定，不得为了 ROLLING 计数、性能或漂亮分位数延长/调参。只清理本阶段启动的
   process；立即写出 CSV、analysis、hash manifest。
6. **离线分析。** 首先执行 cohort integrity、mapping 和 stage-snapshot checks；通过后才
   写 T1 分位数与阶段定位报告。数据无效时只报告 NO-GO，不补跑跨 cohort 配对。
7. **自审并停止。** 重做 git/status/diff checks，确认除本执行单和 `/tmp` run root 外无
   改动；报告证据路径、SHA-256、明确停止。不得自动进入 A5R-1。

---

## 6. 验收与等价性检查

以下都是 measurement evidence 的有效性条件，**不是**对产品新增的 runtime gate。

### 6.1 cohort/数据验收

- 每一个 `cohort_id` 的所有 row 都具有同一 immutable key；
  `retained_delta == preferred_delta`、manager revision == builder revision、request snapshot
  sequence == manager map sequence；
- 每个实际 `solve` 都有一个且仅一个单调 path-state `w` 映射；`raw_samples` 总数等于
  scoped solver call count；无 overflow、丢行或跨 TLS context 行；
- `RAW_PROFILE` 的每个 knot 能匹配完整 raw 表；Filter 和 Validator 的输入/输出是各自
  有序 input 的子序列，且复制 profile 的 source/tube revision 不变；
- 成功 Validator 时 `validator.csv` 的 certified start/end 与 `VALIDATOR_OUTPUT` 首/尾相等；
  失败时不输出伪造的 membership exclusion；
- T1 每个 accepted pair 的两个 row 共享相同 `cohort_id`，并且 `dw > 0`。

### 6.2 透明等价性

- 每个 interposed method 解析到原符号一次并调用一次；wrapper 返回原 bool/result，不修改
  input/reference output；
- path query log 的 `returned` count 必须等于实际转发 count，且 probe 不发起额外的
  path/clearance query；
- wrapper 记录的 `inset_lower/upper` 必须与 raw profile 对应 sample 的
  `raw_lower/raw_upper` 完全一致；Filter output 不得宽于同 knot raw bounds；
- 二进制 SHA-256、仓库 tracked diff、参数快照、launch 文件哈希在 run 前后不变；
- 不要求、也不得把独立 ROS 运行的 `/position_cmd` 做 bitwise 对比。probe run 不能用作
  未插桩性能门、安装门或控制等价 PASS。

### 6.3 不属于验收门的事项

不要求 `ROLLING`、Active、Certified、`source_current_finalized`、OFFSET_OUTSIDE、
HORIZON_SHORT、validator failure、performance p95 或任何失败计数改善。它们是待定位事实，
不是此 measurement 阶段的通过条件。

---

## 7. 停止条件

立即停止并报告，不扩大白名单或改代码，当且仅当出现以下任一项：

- ABI/符号/运行时二进制 hash 与 preflight 不同，或 `RTLD_NEXT` 不能证明透明转发；
- 没有私有 ROS master，或只能附着/影响用户进程；
- snapshot/revision/preferred-delta key 不一致，path-state 映射多义，capture overflow，或
  stage snapshots 不能验证；
- 要取得 pre-curvature component identity、validator 全部 cell witness 或更多字段必须修改
  仓库源码；
- 需要改变参数、geometry、控制、launch、ROS schema，或问题实质进入 A5R-1/A6；
- 完成一份有效 raw corpus 与方法报告。

前五项为本次数据 **NO-GO**；最后一项是正常完成，必须停止而不是继续优化或实现。

---

## 8. 交付物与最终报告格式

一次有效运行的证据根为：

```
/tmp/a5r_0b_<UTC-run-id>/
  evidence/build_context.csv
  evidence/path_state_calls.csv
  evidence/raw_samples.csv
  evidence/raw_components.csv
  evidence/stage_samples.csv
  evidence/stage_exclusions.csv
  evidence/validator.csv
  evidence/t1_pairs.csv
  analysis/A5R_0B_METHOD_AND_RESULTS.md
  analysis/manifest.sha256
  analysis/pre_post_git_and_binary_integrity.txt
```

最终 handoff 必须只回答：

1. cohort count、完整率、每层 first exclusion（前/后分开）和 component ambiguity count；
2. 同 cohort T1 样本覆盖与分位数；
3. `GO`（数据可用于 A5R-3 分类/后续获授权设计）或 `NO-GO`（证据无效及原因）；
4. 证据目录、hash、前后 git status，以及“未改源码/参数/控制，停止于 A5R-0B”。

不得在 A5R-0B 报告中写性能优化通过、A5 固定 revision 完成、A6 continuation 完成，或
任何跨 cohort 的模型级结论。
