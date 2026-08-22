# Codex A6-H2：future-seam 原子 Path–Tube Handoff 执行单

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A6-H2
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=false
AUTHORIZATION_SCOPE=ATOMIC_FUTURE_SEAM_PATH_TUBE_HANDOFF
PRODUCT_GATE_STATE_MODE_REASON_CERTIFICATE_LATCH_SCHEMA_CHANGE_ALLOWED=false
LAUNCH_CONFIG_PARAMETER_CHANGE_ALLOWED=false
TUBE_GEOMETRY_MARGIN_SLOPE_LOOKAHEAD_BACK_SPEED_SATURATION_CHANGE_ALLOWED=false
OLD_A6_PREFIX_REUSE=WITHDRAWN_AND_PROHIBITED
H1_PATH_ONLY_PUBLICATION=PROHIBITED
```

> 日期：2026-08-12  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 前置证据：A5R-1D、A5R-2、A5R-3、A6-V H1 及
> `/tmp/abi_rebuild_20260812T055905Z/report/ABI_REBUILD_FOCUSED_REGRESSION_AUDIT.md`。

## 0. 目标和严格含义

本单实现 **H2**，但只实现下列精确定义的原子交接：

```text
old immutable {path, Runtime-eligible tube epoch}
  -- outside locks: construct one future-seam new path owner and one wholly-new tube owner
  -- short command/control boundary: revalidate then atomic-exchange one immutable pair
new immutable {path, Runtime-eligible tube epoch}
```

一个命令周期只能观察到完整的 old pair 或完整的 new pair。不得观察到：

```text
new path + old tube
new path + null tube
old path + new tube
same path revision + unrelated epoch
```

H2 不把 old `TubeProfile`、`TubeRawSample::p/N`、world clearance、bounds、filter
结果、Candidate、Active 或 Certified profile 复制、裁剪或包装到 new connector。新 tube 的
每一个样本均由 **new immutable path owner** 和其 frozen map snapshot 重新构建。

这不是新 gate、state、mode、reason、certificate、latch、topic 或 diagnostics schema。
`PathTubePair` / `HandoffEpoch` 仅是不可观察、短生命周期、不可变的 ownership value；它没有
ROS 参数、发布接口或可持久化状态机语义。

## 1. 已证实的问题和不可违反的边界

### 1.1 当前 H1 的空窗和并发风险

- 点到点 C2 replan 当前在 `gvf_manager.cpp:5602-5641` 先发布新的
  `ContinuousPhasePath`；adapter 在后续 command 才发现新 revision。
- `PhaseOffsetMatchedAdapter::update()` 在 revision 改变时清空 Candidate / Runtime
  exposure（`phase_offset_matched_adapter.cpp:988-998`）。
- stale timer complete 在 `phase_offset_matched_adapter.cpp:673-693` 会 reset live
  `TubeEpochManager` 并清空 slots；这可擦除已安装的新 epoch。
- command 的 legacy guidance 在 `gvf_manager.cpp:1138` 取 path，adapter input 在
  `1153-1167` 再取 path；`gvf::calcLiftedGuidanceAtPhase()` 又间接 atomic-load path
  （`gvf.cpp:1399-1438`）。在 `formation_planning.cpp:15` 的 `AsyncSpinner(8)` 下，这可
  组成 old guidance / new adapter path 的混搭。

### 1.2 真实 C2 的几何事实

`buildPhaseV2C2Frontend()` 在 `gvf_manager.cpp:2125-2195`：

```text
[prefix_start, w_s]       appendSlice(old owner)
(w_s, join_w)             newly evaluated quintic Hermite connector
[join_w, end_w]           mapped new B-spline
```

五次 Hermite 只在端点匹配 `p,p_w,p_ww`。A6-V 的 production fixture 已在 strict connector
interior 测到 `||p_new-p_old|| > 1e-4 m`。因此 old tube 不能在 connector interior 使用。

### 1.3 明确禁止项

下列内容一律不在本单范围：

- 不新增或修改 gate、state、mode、reason、certificate、latch、计数器、topic、消息、
  diagnostics schema 或 launch/config 参数；
- 不改变 tube geometry、margin、slope、sample/cover、lookahead/back、速度、饱和、
  timer/publish rate、planner limit、governor、SO3、simulator 或 C2 选择参数；
- 不在控制/ownership lock 内运行 A*、B-spline 优化、C2 搜索、整段 tube build、ESDF
  扫描、Marker 构造或 QP 枚举；
- 不提前 path-only publish，不冻结/hold `phase_w_`，不增加 time/lead/retry 阈值；
- 不放宽 `requestSourceStillCurrent()` 或以 old certificate 掩盖 latest explicit map unsafe；
- 不改 AGENTS，不 reset/restore/checkout/clean/stash-pop/commit/branch/tag/push；
- 不启动、接入、停止或清理用户默认 ROS master/process。运行验证必须使用 private master。

## 2. future switch phase 的确定性候选

### 2.1 为什么不能使用当前-seam 延迟提交

当前 builder 以构建当时 `phase_at_switch` 调用
`old_path->evaluate(phase_at_switch, ...)`（`gvf_manager.cpp:2125-2128`）并把 quintic
起点固定为该值（`2165-2166`）。一次 isolated tube build 的实际成本约 35--40 ms，50 Hz
command 可在这段时间推进 `phase_w_`。因此完成后在另一个 phase 上安装 current-seam path
既不保持实际 seam C2，也不能通过早发 path 规避空窗。

### 2.2 唯一允许的 future seam 来源

replan 捕获 old immutable pair、old owner、frozen map snapshot 和 retained Runtime state 后，
只能从 **old owner 既有 immutable sample/domain** 确定性枚举有限的 future `w_s`：

```text
w_s in old path's existing sampled phases
w_s > captured current w
w_s lies in the old pair's currently executable/certified forward coverage
```

这不是新的可调 lead、time budget、hold 或 launch parameter。候选顺序必须由既有 immutable
sample order决定，且每个候选都使用已有 C2 connector length enumeration / acceptance logic；不得
发明新的 connector cost、阈值或路径选择政策。

为支持该事实，现有 C2 builder 可扩展为接收明确 `future_switch_w`，在该 phase 从 old owner
计算 Hermite start state，并构造：

```text
new owner = appendSlice(old, prefix_start, future_switch_w)
          + quintic(old(future_switch_w), mapped_new(join_w))
          + mapped new tail
```

它不得以 command 实时 phase 覆盖 `future_switch_w`，也不得改变已有 connector 搜索范围、
sample step、裕度或成本。

## 3. immutable transaction 与 ownership 设计

### 3.1 单一 pair slot

adapter/manager 层引入一个内部不可变 value（可命名 `PathTubePair`）：

```text
path_owner                 shared_ptr<const ContinuousPhasePath>
source_revision            monotonically assigned provenance
frozen map snapshot + sequence
full new-owner tube epoch snapshot / active profile
precommit [w0,w_s] coverage facts
transaction generation
```

该 value 的 path 和 Runtime epoch 必须互相匹配：

```text
epoch.source_revision == pair.source_revision
epoch.active_path_source_revision == pair.source_revision
epoch.active profile is complete/current-valid
epoch frozen map provenance is internally consistent
```

`shared_ptr<const PathTubePair>` 的单一 atomic slot 是唯一 control authority。command、adapter
和 governor 只能从同一次 captured pair 获取 path owner、epoch/profile 与 provenance；禁止分别
读取 gvf path slot、adapter epoch slot 或 governor path slot，再试图按顺序对齐。若保留
`gvf::continuous_phase_path_`，它只可作为非权威显示/legacy mirror，绝不可由 control consumer
独立读取。不得先交换 `gvf::continuous_phase_path_` 再交换 adapter slot，或反向顺序交换。

`gvf` 需要一个接受 captured `shared_ptr<const ContinuousPhasePath>` 的 pure guidance
evaluator（或等价 static helper）。command cycle 必须：

```text
load one pair once
-> evaluate legacy/base lifted guidance from pair.path_owner
-> evaluate adapter Runtime from the same pair + same path state
-> governor path lookahead uses that captured owner for this cycle
```

任何 later global path read 只能用于下一命令周期，不能改变本周期 pair。

### 3.2 stage 的无副作用构建

每个 future-seam transaction 记录 generation 及 old pair identity。所有重活均在 lock 外：

1. 从 old immutable owner 和 frozen map 选一个 `w_s`；
2. 构造完整 new path owner；
3. 在 local/temporary `TubeEpochManager` 上构造 full new-owner tube，绝不调用/改变 live
   manager；当前 `TubeEpochManager::update()` 会改 Candidate、Active 和 counters
   （`tube_epoch_manager.cpp:290-603`），因此不得直接拿 live owner build；
4. 以 transaction 显式准备域构建同一条 new tube，覆盖

   ```text
   [w0, w_s] + existing required future horizon
   ```

   其中 `w0` 是 transaction capture 的 current phase。此 internal prepared preview domain 是
   transaction input，而非配置修改；不得借此调整 `back_w` / `lookahead_w` 或任一 tube geometry
   参数。precommit copied prefix 的 tube samples 仍全部来自 new owner evaluation；
5. 仅当 new profile complete、retained `delta` at `w_s` 可行、connector + future horizon
   满足现有 manager 安装条件时，形成 prepared immutable pair。A6-V 里 old prefix geometry
   identity 不能替代该完整 new tube 构建。

### 3.3 precommit revalidation 和 exact port dry-run

在 command/control boundary 的短临界区，重新读取并校验：

1. live pair identity / generation 仍等于 transaction captured old pair；
2. actual `phase_w` 仍满足 `w0 <= phase_w < w_s`，且 new owner's copied prefix covers it；
3. same staged new profile 可只读 query current `w`，当前 retained `delta` 在 bounds 内；
4. frozen/latest map contract 可用；若 latest categorical map 明确 OCCUPIED/OUT_OF_MAP
   否定当前 old reference/actual safety，则不得靠 old certificate继续执行；
5. exact Runtime evaluation先按当前既有语义尝试 `U+`，失败才尝试 `U_safe`；两者均空则
   不提交；
6. current `w`、retained `delta`、previous final port 的值均保持，且 Runtime preflight
   不因 staging/failed candidate 提前写入 live Runtime。

上述 port 检查必须使用 copied/temporary Runtime state 或 non-mutating dry-run API；不得调用
live `complete()`，因为它在 selected 时会写 `delta_`、`previous_final_port_` 等状态
（`phase_offset_runtime.cpp:404-418`）。也不得对 live runtime 调用 revision-keyed
`refreshPreflight()`，否则 failed stage 会污染 old pair 的 preflight。

所有条件通过时才在同一短临界区一次 atomic exchange **唯一的** pair authority；gvf published
path slot 如保留只在该 exchange 后写入非权威 mirror。若不能使所有 control consumer 从 pair
查询，停止而不是保留两个可独立消费的 authority。交换后 command 继续从已捕获的完整 old pair
或完整 new pair运行，没有 path-only frame。

### 3.4 stale completion / map provenance guard

- 每个 planner/timer/map transaction 携带 immutable old-pair identity、generation、path owner、
  retained-delta bits 和 frozen map provenance；任一不匹配即 discard；
- stale completion 可以发布其已有 raw/cloud construction diagnostics 的一次性原始证据，但
  绝不能 reset live manager、clear current pair、发布 Runtime/Candidate/Certified authority 或
  覆盖 newer generation；
- timer build 必须在 isolated manager 中进行。commit 后旧 timer 只能发现 generation mismatch
  并丢弃，不能执行当前 `finalizeTubeEpoch()` 的 reset branch；
- map-only advance维持既有 frozen provenance/sample-and-hold contract；但 **每一个 command 的
  selection 前** 都必须用 latest immutable snapshot 对 current reference/actual 执行既有
  categorical check。若任一为 `OCCUPIED` 或 `OUT_OF_MAP`，立即走现有
  certificate-denied/fail-closed mask，old pair 不得 `selected`，即使没有 prepared transaction。
  这不是新 mode/reason/schema，且不能为连续性而保留旧 certificate。

### 3.5 fail/skip 语义

future candidates 枚举、C2、new tube、precommit coverage、current delta、`U+`、`U_safe`、
generation 或 map revalidation 任一失败时：整 transaction 丢弃，不发生 partial install，old
pair 保持原有语义。错过 `w_s` 仅导致 discard/rebuild；绝不 freeze phase。

若 latest map 明确否定 old pair：H2 不增加 emergency/replan bridge、mode 或 reason。保持现有
certificate-denial/fail-closed exposure，并在最终报告标记
`SEPARATE_EMERGENCY_REPLAN_POLICY_AUTHORIZATION_REQUIRED`。

### 3.6 H2 不作的理论主张

H2 不宣称 connector 全区间存在完整的连续 `delta(w)` 正速度可达性定理，也不以此替代后续
proposal/theory验证。本单的可执行验收严格限于：new owner 的重建 tube、所需 horizon、precommit
coverage，以及 commit instant 的 current retained delta 和既有 exact `U+ -> U_safe` non-mutating
检查。

## 4. 修改白名单

仅在本单各前置测试通过后，允许修改：

```text
src/swarm_planner/bspline_traj/include/bspline_race/gvf.h
src/swarm_planner/bspline_traj/src/gvf.cpp
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
docs/Codex_A6_H2_Atomic_Path_Tube_Handoff_Execution_Plan_2026-08-12.md
```

`CMakeLists.txt`、package、launch、config、message/diagnostic schema、tube builder/filter/
cross-section/surface-validator 均不在白名单。若 current tests cannot host the required test
without CMake mutation，停止并报告；不得绕开白名单。

## 5. 实施顺序

### H2-0 — provenance 与 baseline

记录 current `git status --short`、`git diff --name-only`、`git diff --check`，及
`/tmp/abi_rebuild_20260812T055905Z` 的 manifests。现有脏改动均为用户所有；不得清理或修改。
确认 current-CMake focused suite 是 16 binaries / 202 tests PASS，且 legacy
`phase_offset_continuation_test` 为 unresolved-symbol / non-current-CMake artifact，不能作
H2 evidence。

### H2-1 — pair-captured command semantics

先让 gvf/base guidance、adapter Runtime 和 governor query 消费同一次 immutable pair capture。
在此阶段不改变 tube geometry、map policy或 state selection。验证 interleave 下 legacy guidance
owner、adapter current-path owner、Runtime epoch source revision和 governor owner全部相同。

### H2-2 — isolated prepared new-owner tube

实现 temporary `TubeEpochManager` / prepared result API 和 temporary Runtime exact-port dry-run。
它们不能改 live Candidate/Active/counters/preflight/delta/previous port。实现 explicit prepared
preview range，在同一 new owner 上重建 prefix-to-seam + existing future horizon 的 tube，保留
所有现有 builder/filter/validator configuration。

### H2-3 — future-seam builder和transaction commit

扩展 existing C2 builder 接收 deterministic old-path future `w_s`。在 old pair仍可执行时构建
候选，完成 revalidation 后一次交换 pair；删除/替换现有
`PathRevisionFailsClosedUntilTimerBuildIsConsumed` 的空窗期望，不能让它继续把 H1 空窗当作
正确行为。所有 stale finalize reset path改为 generation-discard。

### H2-4 — scoped test与审计

完成下列测试后才运行 build；任何失败不调参数，而是保留 trace、停止并报告。

## 6. 必须新增或替换的验收测试

### 6.1 deterministic real-C2 future-seam handoff

使用 production `makeQuinticHermite`，old/new connector interior 必须可区分。断言：

1. new owner 在 `w_s` C2 seam 满足 `p,p_w,p_ww` 连续；
2. strict connector interior 的 new tube sample `p/N` 来自 new owner，且不等于 old owner；
3. new tube 覆盖 `w0..w_s` 和原有 future certified horizon，不使用 old profile/sample；
4. commit 前 command只消费 old pair；commit 后第一个 command只消费同 revision new pair；
5. 每个 observed tuple皆为 `{path owner, epoch revision, profile owner}` 同一 pair；
6. `w`、retained `delta`、previous final port 的 IEEE-754 bits 在 successful commit 边界不变。

### 6.2 exact current installability

对 staged pair 的当前 `w/delta/position` 做 non-mutating preflight，分别覆盖：

- retained delta outside：no commit，old pair不被污染；
- `U+` empty、`U_safe` feasible：可按既有 safety-priority result prepare，但不新增 mode；
- `U+` 和 `U_safe` 都 empty：no commit，old Runtime state bits不变；
- incomplete new tube / short horizon / invalid current geometry：no commit；
- failed stage 后 old Runtime preflight、delta、previous port、manager counters均不变。

### 6.3 concurrent ownership and stale race

以 command、timer/build、replan/map interleave 构造测试，至少验证：

- no command sees mixed pair，source/tube epochs不倒退；
- stale old build完成于 new pair commit 后，不清除/replace new pair、也不 reset live manager；
- changed old owner/generation/map/delta 或 `phase_w >= w_s` 使 prepared transaction discard；
- latest snapshot `OCCUPIED` / `OUT_OF_MAP` 对 reference 或 actual 命中时，即使无 staged
  candidate也在该 command 前 mask old pair，断言 `selected=false`；
- command capture之后的 global path update不改变该 command's guidance/adapter/governor owner。

### 6.4 baseline regressions

至少运行 rebuilt current-CMake targets：

```text
phase_offset_matched_adapter_test
phase_offset_tube_epoch_integration_test
phase_offset_runtime_test
phase_offset_tube_epoch_manager_test
gvf_switch_policy_test
```

并运行适当的 package rebuild / focused test target build、boundary searches、`git diff --check`
和 postflight `git status --short`。不得引用 non-current-CMake
`phase_offset_continuation_test` 作成功证据。

## 7. ROS / S4 后续验证（本单完成后单独记录）

仅在 H2 unit/integration tests 全通过后，以私有 ROS master/port 运行 active ESDF。这不是默认
master 的操作。记录每个 command 的 pair revision/owner、Candidate/Active/Certified/Selected、
tracking，并只在连续 `selected=1` 至少 3 s、same revision pair、`max tracking < 0.05 m` 时声明
S4 pass。否则如实标记 `NOT PASS`，不得调 tube geometry、Filter 或 emergency policy 来制造通过。

Filter 和 emergency/replan bridge 的优先级须依据 H2 后真实数据另行决定；本单不实现两者。

## 8. 完成标准和停止条件

只有以下全部成立才能写 `A6_H2_ATOMIC_FUTURE_SEAM_HANDOFF_PASS`：

- future seam 来自既有 old immutable path samples，且 new C2 path/new tube同 owner；
- full new tube 覆盖 precommit region和existing future horizon；
- command只消费完整 old/new pair，无 path-only或profile混搭；
- live `w/delta/previous final port`跨成功 commit保持；
- stale/map/delta/port failure无半提交，latest explicit unsafe保持 fail-closed；
- current-source focused regressions及 `git diff --check` 通过。

以下任一情况必须停止并报告，不能通过放宽条件继续：无 safe future `w_s`、无法在白名单内建立
single pair authority、需要 phase hold/new parameter/new state、new tube无法覆盖 precommit区间、
latest map明确 unsafe、或任何 test/ABI/provenance failure。

```text
STOP: H2 does not authorize Filter tuning, an emergency/replan bridge,
planner/governor/simulator changes, or dynamic A7 claims.
```
