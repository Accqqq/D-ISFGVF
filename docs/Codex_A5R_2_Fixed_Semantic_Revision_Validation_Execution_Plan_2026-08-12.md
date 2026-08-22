# Codex A5R-2 固定 semantic-revision 验证执行单

```
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5R-2
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=false
AUTHORIZATION_SCOPE=TMP_ONLY_FROZEN_SEMANTIC_REVISION_REPLAY
REPOSITORY_SOURCE_MUTATION_ALLOWED=false
ROS_PARAMETER_LAUNCH_MUTATION_ALLOWED=false
PRODUCT_GATE_STATE_MODE_REASON_CERTIFICATE_LATCH_SCHEMA_CHANGE_ALLOWED=false
```

> 日期：2026-08-12  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 前置：`Codex_A5R_Tube_Performance_And_Installability_Recovery_Execution_Plan_2026-08-11.md`
> §4 A5R-2；`/tmp/a5r_2_readonly_design_20260811T185046Z/`
> `A5R_2_FIXED_REVISION_FINDINGS_FIRST_DESIGN.md`。  
> 本单只授权 A5R-2 的外部、冻结 semantic-path-revision 回放验证。它不授权任何产品
> 源码、测试、CMake、launch、参数、tube 几何、控制参数或 ROS diagnostics 的修改。

---

## 0. Findings-first 决定与停止边界

### 0.1 本阶段要回答的唯一问题

在一个始终相同的 semantic path identity、相同 path 起止值和预先冻结的输入事件表中，
当前实现是否按既有语义完成以下**不同层次**的交接：

```text
command request
  -> TubeEpochManager::update 的已有 install/disposition
  -> finalizeTubeEpoch 的 source-current atomic exposure
  -> 第一条消费该 epoch 的 command update / ControlPublishSnapshot
  -> 后续 timer-owned marker + 83-field + 50-field publication opportunity
```

最后一项是外显发布节奏，不能倒推为 manager install、atomic exposure 或 command consume
的时刻。A5R-1C 的 request-to-display 数值因此只保留为发布链证据；它不是 A5R-2 的
安装延迟，也不触发本阶段的代码优化。

### 0.2 已冻结的结论

1. A5R-1C 已完成其单热点等价优化；其 endpoint performance 仍是 **NO-PASS**。本单
   不重开性能优化，也不将其显示时延改写成 install 失败。
2. A5R-2 只检查固定 semantic revision。改变 identity 的 case 只作负对照，不能混入
   固定 revision 正样本，更不能修改 `sourceRevision` 或 revision 处理来压制 churn。
3. old A6 的 prefix reuse 已撤回：五次 connector 只在端点 C2，内部不是旧路径；本单
   **禁止**复用旧 path/tube/profile 到新 connector，也不声称 A6 continuation 通过。
4. active replay 可在纯 fixture 中把既有 `observe_only` selector 设为 `false`；那只是
   测试对象的现有配置值，不是 ROS 参数、launch override、新 mode 或新 gate。物理 ROS
   不运行 active selector。
5. 沿用 A5R-1D 已修正的**既有 100 ms 工程预算**，且只应用于 request 到 Candidate
   atomic exposure、Runtime-ready atomic exposure、first command consume；不创建第二个
   deadline、p95 门、marker 间隔门、覆盖长度门或新的 acceptance gate。marker/83/50 的
   外显时延单独报告，永不计入上述执行预算。

### 0.3 绝对禁止项

- 不新增或修改任何 product gate、state、mode、reason、certificate、latch、ROS topic、
  diagnostics field/schema、计数器或参数；离线 CSV/JSON 的列不是产品 schema。
- 不改 margin、slope、lookahead/back、`min_certified_forward_w`、timer/publish rate、
  velocity、saturation、planner/C2、governor、simulator、SO3 或 map。
- 不把 `source_current_finalized`、`ROLLING`、Active、Certified、Selected 合并为一个
  “tube pass”；也不因真实 unsafe/unknown/incomplete 而强行安装。
- 不把 10 Hz marker、83-field 或 50-field topic 的时间当作 `finalizeTubeEpoch` 安装或
  command-consume 的时间。
- 不接入、查询、停止或复用用户默认 ROS master/process；不 reset/restore/clean/stash-pop/
  commit/branch/tag/push。

完成本单的 evidence 和自审后立即停止。A5R-3 与 corrected A6 需要各自新的执行单。

---

## 1. 穷尽白名单与写入边界

### 1.1 仓库白名单

本阶段仓库内唯一允许写入的文件是本执行单：

```text
docs/Codex_A5R_2_Fixed_Semantic_Revision_Validation_Execution_Plan_2026-08-12.md
```

特别地，以下均为**禁止写入**：`src/**`、`test/**`、`CMakeLists.txt`、`package.xml`、
`launch/**`、`config/**`、`AGENTS.md`、`build/**`、`devel/**`、ROS 参数服务器、消息定义与
任何已有执行单。不得用“只改测试”或“只加 measurement field”绕开此白名单。

### 1.2 外部工作根目录

每次执行只可创建一个新的根目录：

```text
/tmp/a5r_2_<UTC-run-id>/
  replay/a5r2_frozen_replay.cpp
  replay/CMakeLists.txt
  replay/build/**
  replay/a5r2_frozen_replay
  manifest/**
  trace/**
  regression/**
  ros_home/**
  ros_logs/**
  corroboration/**
  report/**
```

`/tmp` 内的源文件、构建系统、trace 和报告都是本阶段的外部 test fixture，不得复制回
仓库或成为产品构建输入。若外部 replay 不能以当前既有库构建/链接，立即报告
`EXTERNAL_REPLAY_UNAVAILABLE`，不得扩展白名单修改 repository test、adapter 或 CMake。

### 1.3 可只读使用的输入

- 当前 workspace 的已存在 headers、`devel/lib` libraries 和既有 gtest binaries；
- A5R master、A5R-2 findings-first design、A5R-1C final self-audit；
- 现有的 `phase_offset_matched_adapter_test.cpp`、`tube_epoch_manager_test.cpp`、
  `runtime_test.cpp`，仅用于复核 fixture 常量/既有语义；
- 若进行可选物理佐证，仅可读取并原样启动现有
  `launch/phase_offset_fixed_tube_single.launch`。

不运行 `catkin_make`，不重建或覆盖 workspace artifact。外部 CMake 只能在 `/tmp` build
目录生成其 own executable；它须链接当前已有 artifact，并在 manifest 记录绝对库路径和
`ldd` 结果。

---

## 2. 执行前 provenance 与 artifact preflight

在创建 replay 前，将以下只读结果写入 `/tmp/.../manifest/`：

1. `git status --short`、`git diff --name-only` 和 `git diff --check`；它们仅用于证明未越界，
   不得据此清理用户工作树。
2. 当前 replay 所包含 header、相关 adapter/navigation/core source、当前
   `libphase_offset_matched_adapter.so`、`libphase_offset_navigation.so`、
   `libphase_offset_core.so`、`formation_planning`（仅在可选 ROS 佐证时）和被执行 gtest
   binary 的 SHA-256。
3. 外部 fixture 的编译器版本、完整 CMake configure/build 命令、链接命令、`ldd` 和
   `CMAKE_PREFIX_PATH`。replay 必须解析到本 workspace 的既有 `devel/lib`，不能悄然链接
   系统中另一份 phase-offset library。
4. 运行开始和结束的 `/tmp` tree manifest；仓库白名单以外出现任何新/改文件即停止并报告。

若 header/library provenance 不能解释、库缺失、ABI/linker 失败或现有 gtest binary 缺失，
本次写 `EVIDENCE_INCOMPLETE`。这不是修改产品代码的授权。

---

## 3. `/tmp` frozen replay 的精确形状

### 3.1 编译与可见性约束

`a5r2_frozen_replay.cpp` 是一个独立 C++14 fixture translation unit。它可以像既有 adapter
test 一样在**该 `/tmp` translation unit 内**对 adapter header 使用
`#define private public`，以读取已有 test 已读取的 snapshots/manager fields；宏不得进入
任何 repository source、installed header 或 product library 编译。

外部 CMake 只链接既有：

```text
phase_offset_matched_adapter
phase_offset_active_adapter
isf_reference_kernel
continuous_phase_path
phase_offset_core
phase_offset_navigation
plan_env
catkin libraries
```

它不得 interpose、patch、LD_PRELOAD、修改对象内存布局或伪造 `TubeEpochSnapshot` /
`ControlPublishSnapshot`。它只按当前 unit-test 可见的接口构造输入、调用当前方法，并在
调用边界复制观察值。

### 3.2 固定 semantic path 约束

replay 使用当前 adapter test 的 deterministic `SyntheticPath` 等价路径和相同的 existing
manual configuration contract。正样本的所有 command/timer event 必须同时满足：

```text
same semantic_path_identity address
same semantic_path_start_w IEEE-754 bits
same semantic_path_end_w IEEE-754 bits
same supplied path samples / path-state query function
same source revision observed from the unmodified adapter
```

每条 event 都写入 identity token、path endpoint bits、input/current source revision、
request source revision、candidate source revision、active source revision 和 build sequence。
不得替换 shared identity、修改 `sourceRevision`、手工写 revision 值或依赖 wall-clock race。

每个 ESDF build 自己绑定一个 immutable snapshot；快照间变化只能作为冻结 map evidence，
不得改变正样本的 semantic path identity。对 fixed tube scenario，同样明确记录
`TubeSource::FIXED` 和无 map snapshot 的事实。

### 3.3 声明式 event schedule

replay 以 event ordinal 和合成 ROS stamp 驱动，不等待真实 timer。对每一个 build `i`，至少
按以下顺序执行并记录：

```text
R_i   command update creates/stores TubeBuildRequest
M_i   buildTubeEpoch returns; observe TubeEpochManager result/status
F_i   finalizeTubeEpoch returns; observe candidate/eligible epoch atomics
C_i   first later command update that reads build i; observe output/runtime/control snapshot
P_i   a later timer tick's publication opportunity; record the ControlPublishSnapshot it owns
```

对至少三组独立、measurement-only 的 frozen replay，按 A5R-1D 的既有定义报告：从
冻结 `TubeBuildRequest::stamp` 到 Candidate atomic exposure、Runtime-ready atomic exposure、
first command consume 的 p50/p95/max 与既有 100 ms miss。只有原本可安装、source-current
且 epoch/request contract 匹配的 row 进入 Runtime-ready/usable-install 时效统计；
unsafe、unknown、incomplete、short-horizon、source-stale 必须保留并按原类别单列。

fixture 的 `steady_clock` 只能证明确定性调用链；它必须在报告中明确写
`LIVE_CALLBACK_JITTER_NOT_DIRECTLY_MEASURED`，不得由 nominal 50 Hz 推算 live p95。`P_i`
的外显时延和 cadence 仍分开报告，不进入 100 ms 执行预算。

`P_i` 的记录规则尤其必须遵守当前所有权：在每个 timer tick 进入前读取
`latest_control_snapshot_` 的 `epoch_build_sequence`；timer 内 `finalizeTubeEpoch()` 调用的
`publishManual()` 所拥有的是该**先前 command snapshot**。因此一个 publication opportunity
只有在其 `ControlPublishSnapshot::epoch_build_sequence == i` 时，才可关联到 epoch `i`；否则
写出实际所属 build sequence。它绝不能被标为 build `i` 的 install 或 consume 时间。

这是离线 trace 的 event 名称，不是新增 runtime state/reason/diagnostic。

### 3.4 每条 trace 的字段

`trace/events.jsonl` 每行记录一个 event，不省略失败行。至少包括：

```text
schema_version, run_uuid, scenario, event_kind, event_ordinal,
synthetic_stamp_ns, steady_before_ns, steady_after_ns,
semantic_identity_token, semantic_start_w_bits, semantic_end_w_bits,
request_control_sequence, request_source_revision, request_map_sequence,
build_sequence, candidate_sequence, active_tube_epoch,
manager_state, install_disposition,
candidate_complete, active_available, active_current_validation_valid,
candidate_source_revision, active_source_revision,
source_current_finalized,
candidate_atomic_build_sequence, runtime_atomic_build_sequence,
command_control_sequence, command_consumed_build_sequence,
control_snapshot_build_sequence, control_snapshot_candidate_only,
display_certified, control_selected,
current_w_bits, certified_segment_start_w_bits, certified_segment_end_w_bits,
retained_delta_bits, final_port_u_w_bits, final_port_u_delta_bits,
outcome_note
```

所有浮点关键字段须写 IEEE-754 hexadecimal bits 并另写可读 decimal；`NaN`、缺失和不适用
不能伪装为 `0`。`outcome_note` 是 `/tmp` 报告文字，不能映射或写回任何 product reason。

另生成：

- `trace/build_layers.csv`：每一个 build 的 R/M/F/C/P 行和实际 build/epoch ownership；
- `trace/coverage.csv`：每个 command snapshot 的 `w_publish`、active certified endpoint、
  下一 usable release 的实际 phase 和原始差值；
- `trace/publication_ownership.csv`：每次 timer publication opportunity 的 timer ordinal、
  published control snapshot build sequence 与正在完成 build sequence；
- `trace/negative_control.csv`：revision negative control 独立输出，绝不合入正样本统计。

### 3.5 既有层次的读取方式

必须分别报告，不可互相代替：

| 层次 | 只读证据 |
|---|---|
| `source_current_finalized` | `finalizeTubeEpoch()` 返回后对应 candidate/eligible atomic snapshot 的 source/build provenance |
| Candidate | `TubeEpochSnapshot::candidate_profile`、candidate sequence、complete/status facts |
| `ROLLING` | 既有 `TubeEpochStatus::state == ROLLING` |
| Active/current-valid | `active_available && active_current_validation_valid`，连同 active source revision |
| Certified | 既有 `tubeDisplayCertified(output)` / existing display fact，不把它等同于 Active 或 Selected |
| control-selected | 当前 command output 的 `selected` 与 `ControlPublishSnapshot::output.selected` |

`M_i` 是 manager update 的已有 install/disposition 返回点，`F_i` 是 source-current
atomic exposure 点，`C_i` 才是 command side first consume 点。即使它们落在同一 fixture
event loop 中，也必须保留为三条不同 trace 行。

---

## 4. 冻结场景与既有 oracle

所有场景由 `/tmp` fixture 的固定输入构造，不修改 live planner、C2、launch 或 ROS 参数。
每个 scenario 保存 input fixture 的 SHA-256 和完整 event schedule。现有 unit tests 是 oracle
来源；replay 只把已有行为串成可审计的端到端事件链。

| ID | 固定输入和执行方式 | 必须原样观察的既有语义 |
|---|---|---|
| `FR-safe-observe` | fixed tube；`observe_only=true`；恒定 semantic identity；R/M/F/C/P schedule | Candidate/ROLLING/Active 可以可见，精确 port 可求值；`selected=0`，retained delta 和 previous final port 不提交。 |
| `FR-safe-active` | 同一 fixture，**仅 fixture object** 的既有 `observe_only=false`；不设置 ROS 参数 | 既有 zero-port warmup 自然打开后，valid ROLLING/current-valid profile 可由 Runtime 使用；记录 exact final port、matched output、state commit 和 `selected`。不改 warmup 值，也不把它当新 gate。 |
| `FR-explicit-unsafe` | 先冻结 safe ESDF epoch，再给同 semantic revision 的冻结 OCCUPIED/OUT_OF_MAP current evidence | existing certificate denial/current invalid/Selected false；拒绝步不能部分改写 runtime delta 或 previous final port。 |
| `FR-unknown-or-incomplete` | 同 semantic revision 的 UNKNOWN/UNAVAILABLE/missing snapshot evidence | cold start waits；已有 active 时可按当前语义保留旧 Active owner，但 incomplete Candidate 不成为新的 certificate。 |
| `FR-short-horizon` | current-containing、未来不足的冻结 profile/evidence | Candidate 可见，existing waiting semantics；不安装新的 current certificate。 |
| `FR-retained-offset-outside` | complete latest corridor 排除 retained delta 的冻结输入 | Candidate 可见但 current validation false；Runtime 不进行隐式 side-step 或错误 Certified/Selected 使用。 |
| `FR-revision-negative-control` | 仅此场景更换 semantic identity，一次且可审计 | 既有 source gate fail-closed，旧 exposure 不被静默继承；单独报告，不能进入 positive fixed-revision 分母。 |

`FR-explicit-unsafe`、`FR-unknown-or-incomplete`、`FR-short-horizon` 和
`FR-retained-offset-outside` 的 builder/manager/runtime input 应复制当前既有
`tube_epoch_manager_test` 与 `runtime_test` 中的 deterministic fixture 做法。若外部 replay
无法忠实构造某一既有 fixture，不得近似替换它；该 scenario 写 `EVIDENCE_INCOMPLETE`，并以
现有未修改 gtest 的指定回归输出作为单元级佐证。

不允许把 tracking excursion 改写成 unsafe/unknown，也不改写现有“tracking 可 revoke display
certificate 而不新建 manager mode”的语义。

---

## 5. 覆盖与 certificate 边界：只读计算

对 `F_i` 后 first command-consumed snapshot，记录：

```text
w_publish_i       = ControlPublishSnapshot::output.geometry.w
cert_end_w_i      = active/candidate profile 的现有 certified_segment_end_w
available_w_i     = cert_end_w_i - w_publish_i
```

若之后存在同一 semantic revision 的下一条既有 usable release `j`，记录其实际 command
snapshot phase：

```text
delta_w_to_next_usable_i = w_publish_j - w_publish_i
coverage_slack_i         = available_w_i - delta_w_to_next_usable_i
```

“usable”仅转录当前 manager 输出的 profile complete、`ROLLING`、Active/current-valid、
forward-horizon 与 install disposition/provenance 事实；本单不向产品加入这个判断，也不为其
增加新阈值。没有 `j` 时，只能按预声明的 fixture normal path-end event 收尾；任意停止、
trace 丢失、revision change 或 recorder failure 都不能伪装成 normal terminal。

对于每个 interval 的 command snapshot，写出 raw `w - cert_end_w`、Certified 和 Selected。
报告只区分既有语义所呈现的三种事实：

```text
continuous existing coverage observation
safe fail-closed observation after endpoint
observed Certified/Selected beyond endpoint
```

这不是新增产品 acceptance gate，也不使用自定义 epsilon 或事后删帧。所有固定输入断言
沿用当前链接 artifact 的既有 unit-test/oracle 比较方式和既有数值容差；replay 必须在 manifest
中记录所复用的 test/oracle、常量/比较表达式和 source SHA，不能在 `/tmp` 另发明 tolerance。
coverage 行仍保留 IEEE-754 bits 和原始差值。100 ms 仅按 A5R-1D 的三个执行端点计量，
不用于 marker/diagnostics。

---

## 6. 外显发布与可选物理 ROS 佐证

### 6.1 fixture 内的 publication 语义

纯 fixture 的 publication trace 只验证 timer-owned ownership/cadence：它记录每次
`timerTick()` 前后哪个 immutable `ControlPublishSnapshot` 可由 `publishManual()` 使用，及其
epoch build sequence。它不需要真实 ROS subscriber 才能判定 ownership，也不把 synthetic
event spacing称为真实 10 Hz latency。

因此，A5R-2 不能从 fixture 宣称 marker/83/50 已在物理 topic 上按时间送达；它只能证明
这些外显 payload 在当前 architecture 中位于 command-consume 之后的 timer-owned 链。

### 6.2 可选、非权威的物理 ROS corroboration

只有 frozen fixture evidence 已完整时，才可进行一次可选佐证。它仅说明未修改物理链是否
运行，不替代 fixture 的 exact internal join，也不影响 fixture 结果。

1. 启动本次运行自有的 loopback master：独立 `ROS_MASTER_URI=http://127.0.0.1:<owned-port>`、
   `ROS_IP=127.0.0.1`、`ROS_HOME=/tmp/.../ros_home`、logs 在本 `/tmp` root。
2. 不读取或写入默认 `ROS_MASTER_URI`；不对用户 master 执行 `rosnode`、`rosparam`、
   `rosbag`、kill 或 cleanup 操作。
3. 不传任何 launch argument，不设任何 ROS param，不编辑 launch/config；如启动，只能原样
   使用现有 `phase_offset_fixed_tube_single.launch`。它不是 frozen semantic revision 的证明。
4. 因本单不允许参数覆写，物理运行保持现有 observe-only 默认。`observe_only=false` active
   只允许在 §4 的纯 fixture 中出现；物理 active 结果固定标为 `NOT RUN / NOT PASS`，不得
   从 observe-only 物理 topic 推断 active physical PASS。
5. 若环境具备，记录现有 frame、manual diagnostics、tube epoch diagnostics、candidate/tube
   marker、`/position_cmd`、`/so3_cmd`、`/sim/odom` 与 path topic。它们只能佐证物理执行链；
   由于没有共同 exact command sequence/full matched-port/absolute phase，reference/port 到
   `/position_cmd` 的精确 join 必须标为 `NOT DIRECTLY OBSERVABLE`。
6. 仅清理本次记录的 PID/roscore/roslaunch；用户进程永不触碰。ROS 不可用时写
   `CORROBORATION_NOT_RUN`，不影响纯 fixture 的完成度，但也不得产生 physical PASS。

---

## 7. 已有回归、结果布局与 S4

### 7.1 不改源码的回归

在当前已存在 build artifacts 上运行并原样保存 stdout/stderr/exit code：

```text
phase_offset_matched_adapter_test
phase_offset_tube_epoch_manager_test
phase_offset_runtime_test
phase_offset_tube_epoch_diagnostics_test
phase_offset_tube_markers_test  (仅当当前 artifact 实际名称存在)
```

其中至少保留既有 fixed install、observe-only、selected fixed snapshot、path revision
fail-closed、explicit unsafe、unknown/incomplete、short horizon、retained-offset 和
runtime exact-final-port 相关 test 的过滤结果。若 artifact 名称不同，记录实际名称和
`--gtest_list_tests`，不得为此编辑 CMake/test。回归通过只证明已有 unit contract；不代替
frozen replay 的层次 trace，也不宣称 A6/A7。

### 7.2 最终 `/tmp` 报告必须分表给出

1. fixture source/binary/header/library hash、toolchain、event schedule、identity/revision
   assertion；
2. 每一 build 的 R/M/F/C/P 原始 trace，且 Candidate、ROLLING、Active/current-valid、
   Certified、Selected 各自独立；
3. manager install 与 finalize atomic exposure 的对应关系；
4. first command consume 和 control snapshot ownership；
5. timer publication ownership/cadence，与 install/consume 明确分栏；
6. 每个 certificate endpoint/phase coverage 原始行，不删除 terminal 或失败帧；
7. unsafe、unknown/incomplete、short horizon、retained-offset 和 revision negative control 的
   oracle 对照；
8. observe-only 对 retained delta/previous final port 不提交的记录；active fixture 对 final
   port/matched output/selected 的记录；
9. 可选物理佐证及其 `NOT DIRECTLY OBSERVABLE` 边界；
10. gtest results、白名单审计、结束 git status/diff check 和 explicit stop statement。

结果只可写 `OBSERVED`、`NOT OBSERVED`、`EVIDENCE_INCOMPLETE`、`NOT RUN` 等 evidence
描述，并按 A5R-1D 报告既有 100 ms execution-budget 统计；不可用显示 topic 延迟或新阈值
把它们改写为性能/闭环 PASS。

### 7.3 S4 文字必须逐字保留

每一最终 A5R-2 报告都必须包含：

```text
criterion: S4 tracking < 0.05
status:    WAIVER / NOT PASS
```

`0.057762750070442376` 不能被称为 tracking proof；在现有证据中它来自 `bbox_density` mean。
本单无权修改 tracking 标准、豁免或其 provenance。

---

## 8. 精确执行顺序与停止条件

1. 读取本单、A5R master、A5R-2 findings-first design 和 A5R-1C self-audit；记录 preflight
   manifests，确认仓库只有本单可写。
2. 在新的 `/tmp/a5r_2_<UTC-run-id>` 创建 external C++14 replay/CMake，并以现有 workspace
   artifact build；记录所有 source/header/library hash 与 `ldd`。不写 workspace build/devel。
3. 先运行 `FR-safe-observe` 和 `FR-safe-active` 的固定 revision schedule，输出每一 R/M/F/C/P
   event；active 仅在 in-memory fixture config 中使用 existing selector false。
4. 依次运行四类 existing fail-closed fixture 和 revision negative control，永不把 negative
   control 混入正样本或以 mutation suppress revision。
5. 计算并保存 raw coverage/publication ownership tables；按 A5R-1D 对 R->Candidate atomic、
   R->Runtime-ready atomic、R->first consume 报告既有 100 ms budget/p50/p95/max/miss。不得
   定义第二个 deadline、将 display latency 纳入该预算、或另设人工 tolerance。
6. 运行当前 artifact 的相关 gtest regressions。可选 ROS 佐证仅按 §6.2 在 owned loopback
   master 进行；失败/不可用时保留 evidence，不改代码。
7. 生成 `/tmp/.../report/A5R_2_FINAL_SELF_AUDIT.md`、SHA-256 manifest、白名单审计和
   explicit stop statement；复查 `git diff --check` 与 `git status --short`。

立即停止并报告，而不是扩大范围，若发生任一项：

- workspace 白名单外文件被修改；
- external replay 不能忠实链接当前 artifact、任何 hash/provenance 不可复核，或 trace 缺失；
- positive corpus 内 semantic identity/start/end/source revision 改变；
- 需要修改 test/source/CMake/launch/parameter 才能继续；
- ROS master 不属于本次 run，或清理范围无法确认；
- 任何人要求将旧 tube/profile 复用到 new quintic connector。

本单完成后不自动进入 A5R-3 或 corrected A6。
