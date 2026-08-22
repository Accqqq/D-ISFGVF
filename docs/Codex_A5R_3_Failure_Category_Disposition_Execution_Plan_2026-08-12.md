# Codex A5R-3 失败类别处置执行单

```
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5R-3
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=false
AUTHORIZATION_SCOPE=TMP_ONLY_FAILURE_CATEGORY_DISPOSITION_AUDIT
REPOSITORY_SOURCE_MUTATION_ALLOWED=false
ROS_PARAMETER_LAUNCH_MUTATION_ALLOWED=false
PRODUCT_GATE_STATE_MODE_REASON_CERTIFICATE_LATCH_SCHEMA_CHANGE_ALLOWED=false
```

> 日期：2026-08-12  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 前置：`Codex_A5R_Tube_Performance_And_Installability_Recovery_Execution_Plan_2026-08-11.md`
> §4 A5R-3，A5R-0B final_v3，A5R-2 final self-audit，以及 corrected A6 只读审查。  
> 本单只授权一个 `/tmp` 内的、只读的失败类别处置审计。**不授权任何产品源码、测试、
> CMake、launch、参数、tube 几何或控制语义修改。** 其目的不是让失败计数消失，而是
> 将每个类别绑定到已有事实，并决定它是“验证/无需代码”、“模型或理论升级”，还是
> “需要另一份独立的最小实现单”。

---

## 0. Findings-first 决定

### 0.1 当前已能决定的结论

1. **raw 不是主导排除层。** A5R-0B final_v3 的 46 个同 cohort build、2278 个完整
   raw solver record 中，raw cross-section 的正、反向都是
   `NO_EXCLUSION_OBSERVED`；所有 raw cross-section reason 都是 `NONE`。不能把后续
   `CURRENT_OFFSET_OUTSIDE` 叙述成已证明的 raw-clearance 缺陷。
2. **curvature/inset 与 raw-connectivity 仅是少数前向事实。** 两层各只有 3/46 个
   `EXACT_RAW_ROW` / `EXACT_KNOT` 前向 witness，不支持修改全局几何、margin 或
   component 选择策略。
3. **Filter 是主导的前向收缩层，但这不是实现 bug 的证据。** Filter 的 forward
   first-exclusion 为 45/46 cohort（backward 为 29/46）；可接受的、同 cohort raw
   相邻对的绝对斜率 p90 为 lower `2.0512715744464032`、upper
   `2.396319471217999`，大于当前 `boundary_slope_max=0.80`。这说明当前柱场
   corpus 与该斜率受限 tube 模型之间存在待决的模型/理论问题；它**不**授权调大
   slope、改变 repair/truncation、放宽 filter 或重写 tube 几何。
4. **Validator 的少数拒绝必须保留 fail-closed。** A5R-0B 记录 44/46 return true、
   2/46 return false；没有“同一 immutable snapshot 上所有验证 clearance 均满足，
   而 Validator 错误拒绝”的反证。不能因为只占两次而绕过或调小 surface cover、递归
   深度、query limit 或 clearance 要求。
5. **A5R-2 已证明固定 semantic revision 的安全/拒绝分层，而不是所有动态闭环。**
   安全正样本完成 Candidate、ROLLING、Active/current-valid、Certified、Selected 的
   固定 revision 交接；explicit unsafe、unknown/incomplete、short horizon、retained
   offset 与 revision-negative control 都有既有 oracle。它不构成 A6 connector
   continuation、物理 active ROS、A7 动态闭环或 S4 tracking PASS。
6. **现有证据没有发现可安全、局部修复的产品缺陷。** 因而本单的产品源码白名单为空。
   特别是不得把“计数多”、“Candidate 未安装”、“U+ 为空”或“Filter 缩短 profile”
   本身当作代码修改授权。

### 0.2 本单唯一要回答的问题

对每一个已有失败/拒绝类别，当前实现是否按其已有契约作出正确的分层处置：

```text
raw/current exclusion
filter / surface validation
explicit unsafe
unknown / incomplete
short forward horizon
source-stale revision
U+ empty -> U>=0 attempt -> both empty
```

这里的“处置”只描述现有 Candidate、Active/current-valid、Certified、Selected 与最终端口
是否正确分开；它不是新增 runtime state、gate、reason 或 replan 协议。

---

## 1. 绝对边界与禁止项

### 1.1 不得改动的东西

- 不新增、删除、重命名或复用任何 product gate、state、mode、reason、certificate、
  latch、counter、ROS topic、diagnostics field/schema；`/tmp` CSV/JSON 的文字列不属于
  产品 schema。
- 不改变 tube geometry、cross-section component、margin、`boundary_slope_max`、
  filter repair/truncation、surface cover、query limit、lookahead/back、
  `min_certified_forward_w`、timer/publish rate、速度、saturation、planner/C2、governor、
  simulator、SO3 或 map。
- 不把 A5R-0B 的 raw slope 统计当作“调大 0.80”的授权，也不把 Filter/Validator
  rejection 数归零当作验收。
- 不添加自动 side-step；retained `delta` 被最新 Candidate 排除时，不能用旧 Active
  或 baseline 假装它仍可执行。
- 不把 latest explicit unsafe、unknown、incomplete、short horizon、source-stale
  混成同一个失败类别；也不把 `source_current_finalized`、Candidate、ROLLING、Active、
  Certified、Selected 合并成一个 pass bit。
- 不恢复旧 A6 prefix/profile reuse。新 quintic connector 内部不是旧 path；A5R-3 不得
  使用旧 tube/profile 覆盖它。
- 不接入、查询、停止、复用或清理用户默认 ROS master/process；本单没有 ROS 运行授权。
- 不 reset/restore/clean/stash-pop/commit/branch/tag/push，不修改 `AGENTS.md`。

### 1.2 `U+` 与 `U>=0` 的术语绑定

本单不改名字或实现，只在报告中作以下一一对应：

| 报告术语 | 当前实现中的既有事实 |
|---|---|
| `U+` | `PhaseOffsetRuntime::complete()` 对 `PortProjector::project()` 的第一次调用；`phase_dot_min` 与 `tangent_speed_min` 保持正值。 |
| `U>=0` | 仅在 `U+` 失败后，同一 projector、同一 raw port、同一 bounds/previous port 的第二次调用；只将以上两个 progress lower bound 置零。当前源码注释称它为 `U_safe`。 |
| 两者都空 | 当前既有 Runtime 输出 certificate denial、`selected=false`，且不提交 delta/previous final port；不得借本单把它另造为新的 mode/latch。 |

是否存在一个能把“两者都空”连接到物理 emergency/replan 的既有上游接口，是本单的
**只读接口边界审计项**。若没有此接口，结论只能是
`NEEDS_SEPARATE_EMERGENCY_REPLAN_POLICY_AUTHORIZATION`，绝不能临时让 tube 层发明它。

---

## 2. 穷尽白名单与写入边界

### 2.1 仓库白名单

本阶段仓库内唯一允许存在的本阶段文件是本执行单：

```text
docs/Codex_A5R_3_Failure_Category_Disposition_Execution_Plan_2026-08-12.md
```

执行过程中不得写入仓库的任何文件，包括 `src/**`、`test/**`、`CMakeLists.txt`、
`package.xml`、`launch/**`、`config/**`、`AGENTS.md`、`build/**`、`devel/**`、消息定义和
既有计划。若发现需要修改本计划本身或任何其他 repository 文件，立即停止并另发新单。

### 2.2 外部工作根目录

一次执行只可创建一个新的根目录：

```text
/tmp/a5r_3_<UTC-run-id>/
  manifest/
  corpus/
  probe/a5r3_failure_disposition.cpp
  probe/CMakeLists.txt
  probe/build/
  probe/a5r3_failure_disposition
  trace/
  regression/
  static_audit/
  report/
```

`/tmp` probe 是外部 C++14 fixture，只能链接当前已有 workspace artifact；它不能复制回
仓库、不能成为产品构建输入，不能 interpose/patch/LD_PRELOAD 或修改产品对象内存布局。
若当前 header/library ABI、链接或现有 gtest artifact 不可复核，写
`EVIDENCE_INCOMPLETE`，**不得**以 rebuild workspace、改 CMake、改测试或修改产品代码继续。

### 2.3 允许只读的输入

- 当前 workspace 已有 headers、`devel/lib` libraries、现有 gtest binaries；
- A5R master、A5R-0B final_v3 evidence、A5R-2 report/trace、corrected A6 只读报告；
- 当前 `tube_epoch_manager.cpp`、`tube_filter.cpp`、`tube_surface_validator.cpp`、
  `phase_offset_runtime.cpp`、`phase_offset_matched_adapter.cpp`、`gvf_manager.cpp` 与其
  对应 tests，仅用于 source hash、静态审计和外部 fixture oracle；
- 已有 `/tmp/a5r_2_20260811T210415Z/` evidence，尤其 revision negative control。

不运行 `catkin_make`，不重建 `build/` 或 `devel/`，不启动 ROS。

---

## 3. 失败类别与预先确定的处置

下表是本单的判读标准。它不是“新的状态机”；表中状态/字段均为当前已有对象的读取结果。

| 类别 | 已有证据 | A5R-3 必须核对的现有处置 | 当前代码结论 |
|---|---|---|---|
| Raw/current exclusion | raw cross-section 46/46 两向无 first exclusion；`PREFERRED_DELTA_OUTSIDE_SELECTED_COMPONENT=731` 是 T1 pair 关系，不能偷换成 raw current witness | Candidate 必须保留为可观察证据；exact current retained delta 不在 Candidate 时 `CURRENT_OFFSET_OUTSIDE`，不得自动横移、错误 Certified 或让旧 Active 授权 Runtime | **验证/无需代码**；只有同一 profile/current-w/delta bits 显示 raw/filtered 都包含 delta 而 manager 错报 outside，才可能是实现 defect |
| Curvature/inset/connectivity | 各只有 3 个前向 witness | 报告其准确 stage，不能归因给 raw 或 Filter；不以 3 次为由修改几何 | **验证/无需代码** |
| Filter | forward 45/46、backward 29/46 first exclusion；同 cohort raw abs-slope p90 约 2.05/2.40，当前 limit 0.80 | 验证 filtered bounds 从不比 raw-safe 更宽、且现有 C1/斜率约束生效；不把收缩称为 code failure | **模型/理论升级**，需用户/导师决定，A5R-3 不改代码 |
| Surface Validator | 44 true、2 false；无同 snapshot 的 false-rejection 反证 | 验证 rejected profile 不成为 complete/certified Active；Validator 不得跳过 continuous obstacle certificate | **验证/无需代码**；保守性讨论不等于 bug |
| Explicit unsafe | A5R-2 oracle：occupied/out-of-map current reference/actual -> certificate denial | `active_current_validation_valid=false`、Certified/Selected false、delta/previous port 不被拒绝步部分提交；不得留旧证书作为当前安全证据 | **验证/无需代码** |
| Unknown/incomplete | A5R-2 fail-closed oracle；现有 code 区分无 Active cold start 与同 source 已装 Active 的 sample-and-hold | incomplete Candidate 不成为新 certificate；cold start waits；仅没有 explicit unsafe 时，已有同 source Active ownership 的保持必须被如实单列，不能误报为新 Candidate install | **验证/无需代码**，除非出现 incomplete Candidate 成为新的 Active/Certified 或跨 revision 运行 |
| Short horizon | existing manager oracle 产生 `FORWARD_HORIZON_SHORT` 并拒绝新 install | forward certificate 不足不得安装为新的 ROLLING/current-valid；不得降低 threshold 或 lookahead | **验证/无需代码**；若需要 planner replan 信号，属于另一个接口设计单 |
| Source-stale | A5R-2 negative control：旧 build finalize false、旧 Candidate/Runtime exposure=0；corrected A6 明确禁止旧 profile 到 new connector | semantic identity/revision 变化时 stale Candidate/Runtime/Certified 都不得暴露；该空窗保留 fail-closed，不能以 reuse 填补 | **验证/无需代码**；只作为 corrected A6 的前置压力，不改 A5R-3 |
| `U+` empty、`U>=0` nonempty | existing Runtime test `LivePositiveFailureReprojectsSameRawPortAsSafe` | 第二次 exact projector 必须成功；仅放松两个非倒退下界；输出 `SAFETY_PRIORITY`、可选中且 final phase/tangent speed 非负 | **验证/无需代码** |
| `U>=0` 也 empty | existing Runtime tests show certificate denial, no latch, no delta/previous port mutation | 记录 Runtime/adapter/caller 的 exact output 与上游分支；不得把基准 guidance 当作已验证的 emergency/replan 处置 | **只读接口缺口审计**；没有既有明确 replan/emergency actuation contract 时，不授权产品 patch |

### 3.1 Filter/Validator 的特别判定规则

以下都**不是** implementation defect：

- raw bound 的相邻斜率大于 `0.80`；
- Filter 收缩或截断一个原始安全 component；
- Validator 因 cover、未知或 query-limit 保守拒绝；
- Candidate 因当前 retained delta、前向长度或 surface certificate 而不安装。

只有下列可复核的同输入反例才可写 `SAFETY_IMPLEMENTATION_DEFECT_PROVEN`：

1. Filter 的任意 dense query 在同一 raw-safe profile 外却被标为 valid；
2. Validator 将无连续 certificate 的 profile 标为 complete/obstacle-certified，或它在同一
   immutable snapshot、同一 path query、同一 profile 下与逐点完整 clearance oracle 发生
   明确矛盾；
3. manager/runtime 在 explicit unsafe、incomplete、insufficient horizon 或 stale revision
   下把新的不合格 Candidate 当作 current Active/Certified/Selected；
4. `U+` 失败但 `U>=0` 成功时未做第二次既有 projector 尝试，或两者均失败后仍提交
   delta/previous final port/selected output。

即便任一反例成立，本单仍然只输出最小、可复现的 defect trace 和涉及文件的只读
dependency graph；**不会**在本单直接改产品。随后必须有单独白名单执行单，且仅可列出
由该 trace 证明必需的文件。

---

## 4. 必须产生的只读 evidence

### 4.1 provenance 与 corpus 复核

先写入 `/tmp/.../manifest/`：

1. `git status --short`、`git diff --name-only`、`git diff --check` 的前后结果；只用于
   验证，不得清理脏工作树；
2. A5R-0B 的 `A5R_0B_METHOD_AND_RESULTS.md`、`a5r_0b_summary.json`、`t1_pairs.csv`、
   `stage_exclusions.csv` 的 SHA-256；
3. A5R-2 report、`trace/build_layers.csv`、`regression/fail_closed_manager.txt`、
   `regression/fail_closed_runtime.txt` 的 SHA-256；
4. 所有读取的 product source/header、外部 fixture、gtest binary 和共享 library 的
   SHA-256、`ldd`、编译器版本、完整 CMake configure/build/link 命令；
5. `/tmp` tree 起止 manifest。

由 A5R-0B 的原始 CSV 生成一个只读 `corpus/stage_summary.csv`。它必须逐行复核并保留：

```text
cohort_count=46
raw_records=2278
raw_cross_section NONE=2278
raw first exclusion forward/backward=0/0
curvature/inset forward exact witness=3
raw-connectivity forward exact witness=3
filter forward/backward exact witness=45/29
validator returns true/false=44/2
accepted T1 pairs=1468
raw lower/upper abs-slope p90=2.0512715744464032/2.396319471217999
boundary_slope_max=0.80
```

组件切换、`PREFERRED_DELTA_OUTSIDE_SELECTED_COMPONENT`、`OUTSIDE_RAW_PROFILE` 和
`UNRESOLVED` 必须独立计数；不得把它们塞进连续斜率统计或 current-exclusion 分母。

### 4.2 现有 gtest 回归

不改测试，只运行当前 artifact 中实际存在的测试；保存完整 stdout/stderr/exit code 与
`--gtest_list_tests`。至少尝试以下既有 oracle（artifact 名称不同则记录实际名称）：

```text
phase_offset_tube_epoch_manager_test
  CurrentRetainedDeltaOutsideDoesNotInstallOrLatch
  CloudExcludingZeroCandidateDoesNotInstallOrDenyCertificate
  CloudExcludingZeroCandidatePreservesPriorActiveProfileAndEpoch
  ExplicitCloudUnsafeDeniesCertificate
  UnknownCloudEvidenceWaitsFailClosed
  IncompleteCandidatePreservesActiveProfileAndEpoch
  ShortForwardHorizonWaitsWithoutInstalling
  CloudFutureUnknownRetainsCandidateAndWaitsForShortHorizon

phase_offset_runtime_test
  LatestCandidateExcludingRetainedDeltaBlocksOldActiveExecution
  IncompleteCandidateDoesNotDropRetainedActiveControl
  CertificateDenialDoesNotMutateDeltaOrPreviousPort
  LivePositiveFailureReprojectsSameRawPortAsSafe
  CurrentEmptyPortDeniesCertificateWithoutLatch
  ChangedLiveBaseFactsUseCurrentExactCertificateDenialWithoutLatch

phase_offset_tube_filter_test
  DenseQueriesNeverOvershootRawProfileOrSlopeLimit
  NarrowFutureRegionContractsBeforeTheBoundary
  RemoteSuffixConflictTruncatesAtTheCurrentLocalSegment

phase_offset_tube_surface_validator_test
  OpenRibbonIsCertifiedAndQueriesResidualPlusCover
  BetweenKnotObstacleRejectsContinuousRibbon
  UnknownObservedEdgeFailsClosed
  SampleLimitFailsClosed
```

如果 A5R-2 已记录的 adapter-test stale ABI 再次出现，只能写
`EVIDENCE_INCOMPLETE_STALE_ABI` 并把 A5R-2 external replay 的 revision negative control
作为 source-stale authority；不重建 workspace，也不因测试 ABI 旧而改源码。

### 4.3 `/tmp` external probe

外部 probe 可沿用 A5R-2 的 C++14 fixture 方法、链接当前已有 artifacts，并只读取/调用
当前公开接口；必要时可在其**自身 translation unit**使用已有 test 同样的 private-public
可见性技巧。它必须不伪造 profile、revision、snapshot 或修改对象内存，并以如下 trace
记录已有行为：

```text
scenario, semantic_identity_token, source_revision, map_snapshot_sequence,
current_w_bits, retained_delta_bits, candidate_complete, candidate_raw_complete,
candidate_filtered_complete, obstacle_certified, candidate_first_stop_reason,
active_available, active_current_validation_valid, active_source_revision,
epoch_state, epoch_reason, install_disposition, certificate_denied,
runtime_mode, runtime_valid, control_selected,
u_plus_attempted, u_plus_feasible, u_nonnegative_attempted, u_nonnegative_feasible,
delta_before_bits, delta_after_bits, previous_port_before_bits, previous_port_after_bits,
caller_selected_branch, outcome_note
```

所有浮点值同时写 decimal 和 IEEE-754 hexadecimal bits；缺失/不适用不得伪造为 `0`。
这些字段仅存在于 `/tmp` trace，绝不写入 ROS diagnostics。

probe 至少包含以下 deterministic scenario：

1. complete Candidate 排除 retained delta，含“无 Active”和“已有 Active”两种前史；
2. current explicit `OCCUPIED` 与 `OUT_OF_MAP`；
3. current `UNKNOWN`/`UNAVAILABLE` 与 incomplete Candidate，含 cold start 和已有同 source
   Active；
4. complete Candidate 但 forward horizon 不足；
5. 固定 identity 正样本与一次 semantic-identity revision negative control；
6. `U+` failure 后 `U>=0` feasible；
7. `U+` 和 `U>=0` 均 infeasible。

任何 scenario 若无法忠实使用已有 unit-test fixture/oracle，只能标
`EVIDENCE_INCOMPLETE`；不得自行调参数或做“近似”输入以制造一个结论。

### 4.4 调用链静态审计

`static_audit/` 必须以 source SHA 和行号记录以下现有链条，而不是改动它：

```text
sourceRevision -> makeBuildRequest -> timerTick/buildTubeEpoch
               -> finalizeTubeEpoch -> latest Candidate/Runtime atomic exposure
               -> update/runtime prepare+complete -> matched_output.selected
               -> gvf_manager selected branch
```

特别记录：

- stale request 在 `finalizeTubeEpoch()` 被拒绝并清空 Candidate/Runtime exposure 的位置；
- `U+` 首次 projector 和 `U>=0` 第二次 projector 的相同输入/仅两项 lower-bound 差异；
- both-empty 后 Runtime 不提交 delta/previous port 的位置；
- `gvf_manager` 仅在 `matched_output.selected` 时替换 base guidance 的位置。

最后一项只能说明当前调用分支，不能未经物理协议证明就把 base guidance 标成
“已验证 emergency/replan”。

---

## 5. 分类输出与停止判定

### 5.1 每个类别只能写下列之一

```text
VALIDATED_NO_CODE
MODEL_OR_THEORY_DECISION_REQUIRED
SEPARATE_EMERGENCY_REPLAN_POLICY_AUTHORIZATION_REQUIRED
EVIDENCE_INCOMPLETE
SAFETY_IMPLEMENTATION_DEFECT_PROVEN
```

- Filter 的当前预期类别是 `MODEL_OR_THEORY_DECISION_REQUIRED`。
- `U>=0` 也 empty 的预期是确认现有 certificate-denial/selected-false 无状态提交，并对
  其上游物理处置写 `SEPARATE_EMERGENCY_REPLAN_POLICY_AUTHORIZATION_REQUIRED`，除非
  已有可复核接口另有明确动作。
- `SAFETY_IMPLEMENTATION_DEFECT_PROVEN` 只能满足 §3.1 的明确反例条件；它不是本单的
  修改许可，而是下一份最小实现单的证据前置。

### 5.2 最终报告必含表格

`report/A5R_3_FINAL_SELF_AUDIT.md` 必须把下列层次分列，不能合并：

| scenario/category | raw/current stage | candidate | ROLLING/Active/current-valid | Certified | Selected/final-port commit | disposition | code authorized |
|---|---|---|---|---|---|---|---|

同时必须单列：

1. Filter 和 Validator 证据与为何不能改参数；
2. source-stale negative control 与“无旧 connector profile reuse”；
3. `U+`/`U>=0` 两次 exact attempt 的输入、返回值及 port/delta 不变性；
4. 上游 selected branch 的静态边界，和没有物理 emergency/replan contract 时的明确
   未完成项；
5. 全部 gtest、probe、hash、白名单审计、结束 `git diff --check` / `git status --short`；
6. 逐字保留：

```text
criterion: S4 tracking < 0.05
status:    WAIVER / NOT PASS
```

`0.057762750070442376` 仍不得称为 tracking 证据；其已知 provenance 是 `bbox_density`。

### 5.3 立即停止条件

发生任一项立即停止，不扩展为“顺手修复”：

- 需要任何 repository source/test/CMake/launch/parameter/ROS/schema 修改；
- `/tmp` fixture 无法链接/复核当前 artifact；
- fixed-revision positive scenario 的 semantic identity/domain/source revision 改变；
- 需要调 margin、slope、lookahead、timer、速度、饱和或 surface 参数才会“通过”；
- 发现只靠新增 gate/state/mode/reason/certificate/latch 才能表达建议；
- 要求旧 profile/tube 覆盖 new quintic connector；
- 需要触及用户 ROS master/process；
- gtest ABI 或环境问题使 evidence 不完整。

停下后仅报告证据、缺口和最小 dependency graph。任何产品修复、replan/emergency 协议、
Filter 模型重审或 corrected A6 都必须另发独立执行单。

---

## 6. 精确执行顺序

1. 读取本单、AGENTS.md、A5R master、A5R-0B final_v3、A5R-2 final audit 与 corrected
   A6 review；创建 provenance manifest，确认除本文件外没有本阶段 repository 写入。
2. 在新 `/tmp/a5r_3_<UTC-run-id>/` 复算并核对 A5R-0B stage/T1 summary；不重新跑 ROS
   episode，不改 probe，也不把 raw slope 变成参数建议。
3. 运行 §4.2 的已有 gtest oracle，保存完整输出；如果 artifact 不可用，记录而非重建。
4. 仅在当前 artifact 可复核时构建并运行 §4.3 external probe，逐 scenario 输出既有
   Candidate/Active/Runtime/Selected 层次和 `U+`/`U>=0` attempts。
5. 进行 §4.4 静态调用链审计；尤其区分 source-stale fail-close 与 map-only
   sample-and-hold，区分 Runtime certificate denial 与物理 emergency/replan。
6. 写最终 self-audit、trace、hash、白名单检查和 explicit stop statement；复查
   `git diff --check` 与 `git status --short`。

本单完成后**立即停止**。它不自动进入 Filter 理论重审、任何 replan/emergency 产品实现、
corrected A6 或 A7。
