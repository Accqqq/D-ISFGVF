# Codex A6-V：真实 C2 connector 的 same-revision handoff 验证执行单

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A6-V
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=false
AUTHORIZATION_SCOPE=TMP_ONLY_REAL_C2_H1_VALIDATION
REPOSITORY_PRODUCT_CODE_MUTATION_ALLOWED=false
REPOSITORY_TEST_CMAKE_LAUNCH_PARAMETER_MUTATION_ALLOWED=false
PRODUCT_GATE_STATE_MODE_REASON_CERTIFICATE_LATCH_SCHEMA_CHANGE_ALLOWED=false
OLD_A6_PREFIX_REUSE=WITHDRAWN_AND_PROHIBITED
H2_PATH_TUBE_JOINT_COMMIT_CLAIM=PROHIBITED
```

> 日期：2026-08-12  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 本单是 corrected A6 的窄验证单。起草本单不等于已经执行它；执行前仍须记录当前
> provenance，并且执行后必须在本单边界内停止。

## 0. 结论与唯一验证对象

本单只验证当前实现已经具备的较弱语义 **H1**：

```text
new immutable ContinuousPhasePath
  + request-captured immutable CloudOccupancySnapshot
  + current retained delta
  -> build a new-revision Candidate
  -> expose Candidate / Runtime only while that source revision remains current
  -> otherwise discard stale work fail-closed
```

它不实现、也不宣称以下更强的 **H2**：

```text
one indivisible path + tube transaction
at the C2 seam, with no new-path / no-new-tube interval observable by command
```

H1 中在新 path 已发布而新 epoch 尚未完成时的空窗，必须保持现有 fail-closed 语义。
不能以旧 path 的 tube、旧 profile 的 `p/N`、旧 clearance，或任何“前缀复用”填补该空窗。
若目标改为 H2，立即停止本单并另立跨 `gvf`、adapter、epoch-manager ownership/transaction
架构执行单；不得把它伪装成 A6-V 的小补丁。

本单不会新增产品 gate、state、mode、reason、certificate、latch、计数器、topic 或
diagnostics schema。`/tmp` trace 的列只属于外部证据，绝不写回产品对象。

---

## 1. 已冻结事实与前置边界

### 1.1 旧 A6 已撤回，不能作为依据

`docs/Codex_A6_Revision_Continuation_Execution_Plan_2026-08-11.md` 的
`DOCUMENT_STATUS=WITHDRAWN_INVALID_PREMISE`。其 A6-2/A6-3 曾错误假定：C2 connector
在 `[phase_at_switch, join_w]` 内与旧 path 相同，因而可保留旧 tube 前缀。

实际拼接是：

```text
[prefix_start, phase_at_switch]    appendSlice(old path)
[phase_at_switch, join_w]          newly evaluated quintic Hermite connector
[join_w, path_end_w]               new mapped B-spline
```

五次 Hermite 仅在端点匹配 `p, p_w, p_ww`。connector 内点是新几何，旧
`TubeRawSample::p/N`、旧环境 clearance 和旧 bounds 都不适用。本单不得重提旧 A6
的 C2 fact 透传、`join_w` 字段、prefix retain、relaxed stale 判定或旧 profile fallback。

### 1.2 proposal 的准确含义

- proposal §17.1 / 命题 5：在 retained switch phase，若 C2 seam 匹配
  `p,p_w,p_ww`，且**新** tube 在该 phase 容纳同一 `delta`，可保持
  `w^+=w^-`、`delta^+=delta^-`，所以瞬时 `r,r_w,e,V` 连续。
- proposal §17.2：new connector 的未来部分可能变窄；它要求 new path/new tube 的
  前向可行性，而没有授权 old profile 跨 connector 复用。
- proposal §10.6：候选 tube 的现有安装/Runtime 检查是 hybrid update 的一部分；这不自动
  提供 path+tube joint commit。

因此本单的 real-C2 fixture 必须同时证明「seam 连续」和「connector interior 与旧 path
不同」。只证明前者不足以排除旧 A6 前提复活。

### 1.3 A5R-2 / A5R-3 的可用事实和不可跨越边界

可作为前置只读证据：

1. A5R-2 已得到 `FIXED_REVISION_POSITIVE_REPLAY_PASS`：固定 semantic revision 的
   正样本已有 Candidate → Active/current-valid → Runtime first consume 的 replay 证据；
   revision negative control 中 stale old build 没有 Candidate/Runtime exposure。
2. A5R-3 已得到 `A5R_3_FAILURE_CATEGORY_DISPOSITION_COMPLETE_NO_PRODUCT_CODE`：
   Filter 是模型/理论待决项，explicit unsafe、unknown/incomplete、short horizon、stale 和
   projector 分支均保持原有 fail-closed/disposition，不授权 A6 修改。

以下边界必须逐字保留在 A6-V 最终报告：

```text
A5R2_FAILURE_SCENARIO_ADAPTER_REPLAY=EVIDENCE_INCOMPLETE_UNIT_ORACLE_PASS
PHYSICAL_ACTIVE_ROS=NOT_RUN / NOT_PASS
LIVE_CALLBACK_JITTER=NOT_DIRECTLY_MEASURED
S4_TRACKING_LT_0_05=WAIVER / NOT PASS
0.057762750070442376 is bbox_density provenance, not tracking evidence
```

A6-V 不得把 real-C2 的 safe/stale external fixture 说成上述 failure category 的完整
adapter replay，也不得将其升级成 physical closed-loop 或 S4 tracking PASS。

---

## 2. 写入边界、禁止项与外部工作根

### 2.1 仓库产品代码白名单

**为空。** 执行 A6-V 时不允许写入 repository 的任何产品文件：

```text
src/**
test/**
CMakeLists.txt / package.xml
launch/** / config/**
build/** / devel/**
AGENTS.md
message definitions / diagnostics schemas
any existing execution plan
```

本文件是本次起草产生的唯一 repository 文档；实际验证也不应回写或修订它。需要新
repository 文件、workspace rebuild、参数/launch 编辑或产品源码改动时，一律写
`OUT_OF_SCOPE_REQUIRES_NEW_EXECUTION_SPEC` 并停止。

### 2.2 明确禁止的伪修复

- 不改 `tube_builder`、`tube_cross_section`、`tube_filter`、`tube_surface_validator`，不改
  geometry、margin、slope、sample/cover、lookahead/back、速度、saturation、timer 或
  publish rate；
- 不修改 `sourceRevision()`、`requestSourceStillCurrent()`、`epochMatchesRequest()`、
  `finalizeTubeEpoch()` 或 Runtime/Projector；
- 不增加 C2 flag、`join_w` handoff、pending/hold/retry mode、replan/emergency bridge、
  新的 latch/reason/certificate/schema；
- 不从 old Candidate/Active/Certified profile 复制 samples、bounds、world `p/N`、map
  clearance 或 display 到 new connector；
- 不接入、查询、复用、停止或清理用户默认 ROS master/process。本单不启动 ROS；
- 不 `reset --hard`、`restore`、`checkout`、`clean`、`stash pop`、commit、branch、tag 或 push。

Fixture 中读取已有 private snapshot 的 `#define private public` 约定，仅可存在于 `/tmp`
的单一 C++ translation unit，和 A5R-2 既有 fixture 相同；它不能改变任何 header、library
对象布局、内存、ABI 或运行时行为。

### 2.3 外部根目录

每次只创建一个新的根目录：

```text
/tmp/a6v_real_c2_<UTC-run-id>/
  fixture/a6v_real_c2_h1.cpp
  fixture/CMakeLists.txt
  fixture/build/
  fixture/a6v_real_c2_h1
  manifest/
  trace/
  regression/
  static_audit/
  report/A6_V_FINAL_SELF_AUDIT.md
```

外部 CMake 只能链接当前 workspace 的已存在 libraries，且须记录完整 configure/build/link
命令、`ldd`、绝对 library 路径和 header/library SHA-256。它不得 `LD_PRELOAD`、interpose、
patch、手工构造 `TubeEpochSnapshot`/`ControlPublishSnapshot`，或复制结果回 repository。
若 ABI、链接、library provenance 或 gtest artifact 不可核对，结论只能是
`EVIDENCE_INCOMPLETE_EXTERNAL_FIXTURE_UNAVAILABLE`；不得以重建 workspace 或修改 CMake
继续。

---

## 3. 执行前只读 provenance

在创建 fixture 前，写入 `manifest/`：

1. `git status --short`、`git diff --name-only`、`git diff --check` 的前后结果。它们只用于
   证明边界，脏工作树完全视为用户所有，绝不清理。
2. 本单、撤回 A6、A5R-2/A5R-3 final audit、A6 corrected read-only design、proposal
   §10.6/§17.1/§17.2/命题 5 的 SHA-256 或可复查摘录。
3. 当前 `continuous_phase_path.cpp`、`phase_offset_matched_adapter.{h,cpp}`、
   `phase_offset_runtime.cpp`、`gvf_manager.cpp`、相关 existing test source，以及 fixture
   所链接 header/shared library/gtest binary 的 SHA-256。
4. `gvf_manager` C2 install branch 的静态摘录（带 line number 与 source hash）：证明 C2
   replan branch 不给 `phase_w_` 赋 reset 值。该审计只证明当前源码的局部事实；它不能冒充
   live multi-callback interleaving 或 H2 transaction 证据。
5. `/tmp` tree 前后 manifest，以及外部 fixture source/hash/compile command/`ldd`。

preflight 若发现仓库白名单外的本阶段写入、产品文件 hash 在本阶段中改变，立即停止并标记
`REPOSITORY_MUTATION_OUT_OF_SCOPE`。不尝试恢复这些文件。

---

## 4. `/tmp` real-C2 H1 fixture 的固定设计

### 4.1 使用真实的 production C2 evaluator，而非伪造“连续”路径

fixture 必须直接使用现有
`ContinuousPhasePath::makeQuinticHermite(w_s, w_j, old_state, new_state)`。
不得以一条已经与 old path 相同的 lambda、线性插值或手工预采样路径替代 connector。

构造顺序必须是：

```text
old_owner:
  a non-degenerate immutable old ContinuousPhasePath

new_owner:
  appendSlice(old_owner, prefix_start, w_s)
  appendSegment(w_s, w_j, "c2_quintic", makeQuinticHermite(...))
  appendSegment(w_j, path_end_w, new-tail evaluator)
```

fixture 要选择已知温和、all-free 可建 tube 的 old/new endpoint data；`w_j-w_s` 至少包含一
个不会落在端点的 tube sampling phase。数值是 fixture path 的确定性几何定义，不是产品
参数或 safety tuning。若该 fixture 不能得到完整 new Candidate，只能报告
`EVIDENCE_INCOMPLETE_FIXTURE_NOT_INSTALLABLE`，不得放宽 tube 参数、margin、slope 或
lookahead 来让它通过。

### 4.2 seam 和 connector interior 的两条独立记录

fixture 对 `old_owner` 和 `new_owner` 记录：

```text
at w_s:
  ||p_old - p_new||, ||p_w_old - p_w_new||, ||p_ww_old - p_ww_new||

at one or more strict interior w_m in (w_s, w_j):
  p_old(w_m), p_new(w_m), ||p_old(w_m) - p_new(w_m)||
```

seam 的三个差必须满足现有 unit-test 所用的 `1e-9` numerical comparison；fixture path
的定义必须预先选择到使 interior position 差有清楚、非 round-off 的正量（例如
`>1e-4 m`）。`1e-9`/`1e-4` 仅为外部数值等价/非等价断言，绝非产品安全门或新的控制阈值。

同时逐个检查 new Candidate 的 connector-interior `TubeRawSample`：

```text
sample.p == new_owner evaluated p at sample.w       (existing numeric tolerance)
sample.p != old_owner evaluated p at the same w     (non-round-off difference)
```

至少一条已完成、位于 `(w_s,w_j)` 的 Candidate sample 必须写入 trace；若 sampling grid
未命中 connector interior，固定 fixture 的 path domain/connector length应在**不改变产品
config**的前提下重新定义一次。不得以比较 seam 本身代替 interior 证据。

### 4.3 immutable owner、frozen map 和 retained state

fixture 使用 A5R-2 的现有 `MANUAL + ESDF` test configuration 常量和 production
`buildCloudOccupancySnapshot()`，创建一个 self-consistent、all-free 的 immutable
`shared_ptr<const CloudOccupancySnapshot>`。将**同一个 pointer instance**冻结并传给 old
request 和 new C2 request；构建中不改变其内容或 observation sequence。

每一行 trace 至少记录：

```text
old_owner_address, new_owner_address,
request_owner_address, request_source_revision,
frozen_snapshot_address, frozen_snapshot_observation_sequence,
request_snapshot_address, request_map_observation_sequence,
request_retained_delta_bits,
request_current_w_bits
```

new request 必须由当前 adapter 的 `update()`/`makeBuildRequest()` 创建，不能手写 revision
或 `TubeBuildRequest`。它必须满足：

```text
request.semantic_path_owner.get() == new_owner.get()
request.cloud_snapshot.get() == frozen_snapshot.get()
request.map_observation_sequence == frozen_snapshot->observation_sequence
request.source_revision == old_request.source_revision + 1
request.retained_delta bits == pre-handoff Runtime retained delta bits
request.current_path.w bits == captured seam w_s bits
```

其中 owner pointer change 是**真实 revision**来源；不允许通过只改
`semantic_path_identity` 模拟 revision。

为避免“保持”为零值的空洞证明，fixture 先使用既有 active test selector（仅 fixture
in-memory `observe_only=false`）按 A5R-2 已有 warmup 语义跑到 selected command，记录非平凡
`retained_delta` 和 previous final port。若现有不变 config 无法产生非平凡状态，写
`EVIDENCE_INCOMPLETE_NONTRIVIAL_CONTINUATION_NOT_REACHED`，不能调整 selector 之外的产品
参数、warmup、速度、saturation 或 tube 配置。

### 4.4 必须按顺序发生的 H1 trace

以下是 fixture event schedule，不是产品状态机，也不创建新的 runtime gate：

```text
O0  old immutable path request; build old TubeEpochSnapshot, but do not finalize it
R1  update with new_owner at the same captured seam w_s and frozen snapshot
S1  finalize old built snapshot after revision changes
N1  build new request from new_owner + same frozen snapshot + retained delta
F1  finalize new built snapshot while new source revision is current
C1  first later command update consumes the new matching Runtime epoch
```

每个 event 都写 `steady_before/after`、synthetic stamp、revision、build sequence、pointer
addresses、current `w`/delta/final-port IEEE-754 bits，以及 Candidate/Runtime atomic presence。
这里的 time 只用于顺序和可重现性；本单不添加任何 timing/pass-rate/marker cadence 验收。

#### S1：stale old build 必须保持 fail-closed

在 `R1` 已发布 new request 后调用现有 `finalizeTubeEpoch(old_request, old_built)`。必须记录：

```text
finalize return == false
old build sequence is absent from latest_candidate_epoch_snapshot_
old build sequence is absent from latest_epoch_snapshot_
old Candidate/Active/Certified is never presented to C1
```

raw/cloud build diagnostics 的既有一次性记录可以存在，但绝不能被写成 Candidate、Runtime、
Certified 或 control consume。不得将 old profile 包装为 C2 compatible fallback。

#### F1/C1：new Candidate 的 same-revision exposure

在 `N1`/`F1`，fixture 通过当前 `buildTubeEpoch()` 和 `finalizeTubeEpoch()` 生成新 epoch，
不伪造结果。它分别记录而不合并：

```text
Candidate atomic: latest_candidate_epoch_snapshot_
Runtime-ready atomic: latest_epoch_snapshot_
first command consume: latest_control_snapshot_ / command output
```

new Candidate/active provenance 必须与 `new_request.source_revision` 一致；仅在当前现有
`epochMatchesRequest()` contract 成立时才可记为 Runtime-ready。`C1` 的 first consume 必须
引用 `F1` 的 new build sequence，且不得引用 `O0`。Candidate profile、full path samples 和
connector-interior `TubeRawSample` 都必须来自 `new_owner`；old profile pointer/geometry 不得
参与 new epoch。

### 4.5 `w`、delta、previous final port 的准确证明范围

在 O0 selected command 后、R1 后、F1 后（且 C1 normal step 之前）分别记录：

```text
request/current path w bits
runtime_->retainedDelta() bits
runtime_->previousFinalPort().u_w bits
runtime_->previousFinalPort().u_delta bits
```

R1/F1 不得把 captured `w` 改为 path start、零或其他相位；也不得把 retained delta 或
previous final port 清零。C1 是正常 command step，之后 state 可以依现有控制律演化，因此
不能把它的合法积分变化错报为 reset。C1 同时记录已有 `projection.final_port` 与
`matched.matched_residual_norm`，只证明其沿用既有 physical/internal matched-port contract。

adapter 本身不拥有 `gvf_manager::phase_w_`。故本单对 live `phase_w_` 的结论仅限
§3 静态 source audit 加 fixture seam input bit preservation；它明确**不**声称已测试 8-thread
ROS callback 下的 manager-level atomic C2 install，也不将此限制隐藏成 H2 已完成。

---

## 5. Existing-test 回归与 evidence 分类

无需、也不得向 repository 增加 test。外部 fixture 优先；已有 tests 仅作不修改源码的
回归佐证：

| 既有证据 | 要求的作用 | 不能据此声称 |
|---|---|---|
| `phase_offset_continuation_test` 的 `C2ConnectorSeamIsContinuous` | 对 seam `p,p_w,p_ww` 与同 delta 的局部 `r,r_w` 连续性作回归 | connector interior 等于 old path，或 new tube 已 joint-commit |
| `phase_offset_matched_adapter_test` 的 `PathStaleBuildEmitsRawAndCloudExactlyOnceWithoutCandidateOrRuntimeExposure` | 对 stale fail-close 的现有单元语义作回归；若 ABI 可运行 | real-C2 geometry 组合已由 identity-only test 覆盖 |
| `runtime_test` 的 `ActivePointerReplacementDoesNotResetDeltaOrPreviousPort` | 对现有 pointer replacement state persistence 作回归 | manager `phase_w_` live atomicity 或 H2 |
| A5R-2 frozen replay/A5R-3 audit | 既有 fixed revision 和 failure-category evidence 的引用 | failure adapter replay 完整、physical ROS 或 S4 PASS |

若现有 adapter gtest 与 current header/library 出现 A5R-2 已记录的 stale ABI，记录
`EVIDENCE_INCOMPLETE_STALE_ABI` 并保留 external fixture 的直接当前-library evidence；不
rebuild workspace、不修改 test/CMake。每个实际运行的 gtest binary/filtered case、退出码和
SHA-256 都存入 `regression/`。

---

## 6. 结果写法、停止条件与最终报告

### 6.1 H1 可写出的最强结论

只有下列全部外部证据成立时，最终报告可写：

```text
A6_V_REAL_C2_H1_VALIDATED_NO_PRODUCT_CODE
```

该结论的含义严格限于：真实 production quintic C2 seam 在 fixture 中连续、其 interior
确实是新 path；new immutable owner/same frozen map snapshot/retained delta 生成了 new revision
Candidate；stale old build fail-closed；new Candidate/Runtime first consume 的 provenance 同属
new revision；handoff 本身未 reset captured w/delta/previous final port。

以下任何一种情况都只能写 `EVIDENCE_INCOMPLETE` 或具体失败 trace，不能改产品：

- seam 不连续、interior 与 old path 未区分、或 new Candidate sample 不能对应 new owner；
- old stale build 有 Candidate/Runtime/Certified exposure；
- new Candidate 以 old revision/profile/map provenance 暴露，或 retained state 被 reset；
- fixture/library ABI/provenance 不可重现，existing tests 不能执行；
- fixture 因既有 config 的安全拒绝而未能产生 safe new Candidate。

这些是外部证据判读条件，不是新增产品 gate 或 acceptance state。

### 6.2 明确禁止的结论

最终报告不得出现下列任何说法：

- “H2/path+tube zero-gap joint commit 已验证”；
- “旧 tube 在 C2 connector 上仍有效”或任何 old-prefix reuse；
- “A5 geometry/filter/validator 已因此通过/应调整”；
- “A5R-2 所有 failure scenario adapter replay 已 PASS”；
- “physical ROS、live callback jitter、S4 tracking <0.05 已 PASS”；
- “marker/83/50 diagnostics timing 证明 install 或 command consume”。

### 6.3 必交付的报告与停止声明

最终 self-audit 唯一路径为：

```text
/tmp/a6v_real_c2_<UTC-run-id>/report/A6_V_FINAL_SELF_AUDIT.md
```

报告至少列出：环境/库 provenance、fixture C2 numerical trace、immutable owner/map/delta
provenance、O0/R1/S1/N1/F1/C1 event trace、sample geometry comparison、w/delta/port bits、
existing-test results、A5R-2/A5R-3/physical/S4 boundary，以及 repository pre/post integrity。

报告末尾必须写：

```text
STOP: A6-V ends after H1 evidence.  No H2 transaction, no A7 dynamic
closed-loop, no physical ROS acceptance, and no product-code change was entered.
```

本单到此结束。它不授权后续 A7、H2 architecture、emergency/replan policy、Filter model
change 或任何 A5 tube 几何工作。
