# A6 Owner-Aligned Current-Phase Preview Correction

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行授权：M3-L1 动态验收发现 adapter preview plumbing 的确定性缺陷；用户已要求
主代理持续调度、写计划，并由 `gpt-5.6-terra / xhigh` 修改和执行 launch 验收。  
`AUTO_ADVANCE=true`，但仅限本文件定义的 A6-P1；不得借此修改 Tube 几何、
Runtime、planner/C2、H2 lifecycle、recenter 或 authority 架构。

## 1. 动态证据与问题定义

M3-L1 active ESDF 隔离运行中：

```text
raw candidate seq=1:
  current_w=0.05
  state=ROLLING, reason=NONE
  raw/filter/complete=1/1/1
  current sample found/complete=1/1

raw candidate seq>=2:
  current_w=0.398624...（以及后续连续值）
  state=WAITING_FOR_CANDIDATE
  reason=CANDIDATE_INCOMPLETE
  sample_count>0, invalid_count=1
  current sample found/complete=0/0
```

根因位于 adapter anonymous helper：

```text
BuildOwnerAlignedTubePreview()
```

它当前只加入：

- preview `start_w/end_w`；
- supplied planner samples；
- immutable owner segment endpoints。

但 ordinary timer preview 的范围是：

```text
start = current_w - back_w
end   = current_w + lookahead_w
```

因此 `current_w` 通常既不是 start/end，也不是 planner knot/segment seam。
`TubeBuilder` 要求 preview 中存在精确 current-phase anchor；缺失时返回：

```text
adaptive preview does not contain current phase sample
```

首帧成功只是 `w=0.05` 偶然与 planner sample 对齐，不代表持续可用。

这不是 clearance、zero-connected、Filter、SurfaceValidator 或 M3
`CertifiedTubeBuilder` 的失败，而是进入它们之前的 preview partition 漏点。

## 2. 阶段目标

令 owner-aligned preview 明确携带一个 `required_current_w`，并从同一 immutable
owner 精确求值后插入排序分区：

```text
{range endpoints, required current phase, supplied knots, owner seams}
-> sort/deduplicate
-> owner evaluate every knot
-> assert required current anchor exists exactly once
```

修复 ordinary timer 和 prepared/PathTubePair 两条调用路径，使连续推进的任意合法
current phase 都能成为 Tube candidate anchor，同时保留 segment-aligned cell proof。

## 3. 必须冻结的行为

不得修改：

- `TubeBuilder`、`CertifiedTubeBuilder`、`TubeFilter`、
  `TubeSurfaceValidator`、`TubeEpochManager`；
- planner safe distance、Tube offset/ray/filter/cover 参数；
- zero-connected 与 zero-only 语义；
- current phase 的数值或 global `w`；
- Kino A*、B-spline、C2、replan 触发和候选接受；
- Runtime matched-port、retained delta、warmup、dry-run/CAS；
- H2-L1 single seam 与 retry-safe mailbox；
- snapshot、map revision、path owner/provenance；
- ROS schemas、parameters、topics、threads；
- recenter/physical recovery。

不得用 round-to-grid、修改 `current_w`、放宽 Builder anchor tolerance 或额外 margin
来修复。当前 phase 必须作为其原始 double 值由 immutable owner 求值。

## 4. 强制实现

### A6-P1.1 helper contract

修改 anonymous helper signature：

```cpp
bool BuildOwnerAlignedTubePreview(
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const MatchedAdapterPathSamples& supplied_samples,
    double start_w,
    double end_w,
    double required_current_w,
    MatchedAdapterPathSamples& output);
```

输入必须 fail closed：

- owner 有效且非空；
- start/end/current finite；
- `end_w > start_w`；
- `required_current_w` 位于 `[start_w,end_w]`，只允许既有
  `kPreparedCoverageTolerance` 的边界 roundoff；
- owner 能在 required current phase 求值为 finite/valid state。

### A6-P1.2 partition insertion

- 在 supplied samples 和 segment endpoints 之外，显式 append
  `required_current_w`；
- 若 current 与 start/end/seam/sample 在 tolerance 内，dedupe 为一个 knot；
- 不将 current snap 到其他 knot；tolerance 内 canonical 值必须保证最终输出能被
  `TubeBuilder` 以其现有 current-anchor tolerance 找到；最稳妥方式是让
  `required_current_w` 本身成为该等价类的 canonical knot；
- 所有输出 state 仍只通过 `owner->evaluate(w, ..., false)` 构造；不得线性插值；
- 构造完成后明确验证 output 单调，并存在恰好一个
  `abs(sample.w-required_current_w) <= current-anchor tolerance` 的 sample；
- 不合成新 owner segment，不改变 cell endpoints 或 certificate query。

注意：当前 `sort + std::unique` 保留等价组中的最小值，可能把 required current
丢给一个相近 planner knot。实现必须显式处理 canonical ownership，而不是只在
旧 append 列表里多 push 一次 current。

### A6-P1.3 两个 production 调用点

ordinary timer：

```text
required_current_w = request->current_path.w
```

prepared/PathTubePair：

```text
required_current_w = captured_w0
```

两条路径均不得退回 mutable/sampled interpolation。prepared 路径即使 current
等于 range start，也必须走同一显式 contract，防止未来调用改变 range 后复发。

### A6-P1.4 fallback 边界

ordinary timer 中若 owner-aligned helper 因真正 owner/range invalid 失败，保留现有
`makePreview()` fallback；但不得因遗漏 current 而失败。prepared PathTubePair
继续 fail closed，不新增 fallback。

本阶段不改变 fallback 本身或 Builder 对 current anchor 的要求。

## 5. 文件白名单

只允许修改：

```text
docs/Codex_A6_Owner_Aligned_Current_Phase_Preview_Correction_Execution_Spec_2026-08-20.md
docs/Codex_A6_Owner_Aligned_Current_Phase_Preview_Correction_Self_Audit_2026-08-20.md

src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

/home/cxq/ISF-GVF/handoff.md
```

只读回归，不得修改：

```text
src/swarm_planner/phase_offset/**
src/swarm_planner/bspline_traj/include/**
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/plan_env/**
src/swarm_planner/bspline_traj/launch/**
Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md
```

若需要修改 header/API、Tube、manager、Runtime 或 launch，停止并报告。

## 6. 必须新增的测试

### Ordinary timer

1. 构造 immutable owner 和 supplied samples，使 current phase 严格位于两个
   planner knots 之间，且不是 owner segment endpoint；
2. `timerTick()` 后 candidate：
   - raw/filter/complete 均 true；
   - samples 中包含原始 current double；
   - current sample complete；
   - no `CANDIDATE_INCOMPLETE`；
3. 连续使用至少三个不同的 off-grid current phases，每次都保持 current anchor；
4. current 恰好/近似等于 supplied knot 或 segment endpoint 时只输出一个 anchor，
   profile 仍严格单调；
5. current 在 range 外或 owner evaluate 失败时 fail closed/fallback 行为与当前合同
   一致，不产生伪造 sample。

### Prepared/Pair

6. prepared samples 不显式包含 captured current，但 owner 覆盖它；prepared build
   成功且 active profile 包含 captured current；
7. captured current 与 start boundary 相同仍只有一个 anchor；
8. owner mismatch、range/provenance/coverage 的既有拒绝测试保持通过；
9. `stagePathTubePair()` 对 off-grid captured current 能形成完整 candidate pair，
   不改变 dry-run/CAS 语义。

### 回归

必须复跑：

- `phase_offset_matched_adapter_test`；
- `phase_offset_certified_tube_builder_test`；
- `phase_offset_tube_epoch_manager_test`；
- CrossSection/Builder/Filter/SurfaceValidator；
- `phase_offset_runtime_test`；
- `gvf_switch_policy_test`；
- `continuous_phase_path_test`。

## 7. 构建与静态验收

必须通过：

```text
catkin_make -j2
git diff --check
```

静态 scan 确认：

- 两个 production helper call 都显式传 current phase；
- 没有 round/snap current phase 的新代码；
- 没有 Tube/Runtime/manager/launch/parameter 修改；
- M3 unified pipeline 仍是 production 唯一 candidate build 入口。

## 8. Launch 动态验收

继续遵守隔离 ROS master/ROS_HOME/PID/log 规则，绝不触碰环境中既有 ROS 进程。
固定地图、目标、参数，不以延时或调参制造成功。

按顺序：

1. ESDF observe-only：默认
   `phase_offset_esdf_tube_single.launch`，目标 `(8,0,1)`；确认到达且 raw
   candidate 在连续 off-grid `current_w` 上保持完整；
2. ESDF active：同一 launch，仅覆盖
   `phase_offset_manual_observe_only:=false`，同一目标；必须记录：
   - raw seq1 与 seq2+ complete 状态；
   - current sample found/complete；
   - initial PathTubePair/offset authority 是否激活；
   - Runtime selected/valid；
   - H2 single-seam replacement/retry；
   - path-end-clamped 与最终目标距离。

若 preview 修复后 raw candidate 持续完整，但 Pair 激活仍失败，则准确定位下一层
bootstrap/stage/dry-run/CAS blocker，写入 self-audit 并停止；不得增加临时日志、
修改 manager/adapter header 或越权修复，除非另写执行规格。

## 9. 验收标准

1. owner-aligned preview 永远显式包含合法 current phase；
2. current 使用原始 global `w`，不 snap、不 reset；
3. continuous off-grid timer candidates 不再因 missing current anchor incomplete；
4. prepared/Pair 路径共享同一 contract；
5. segment seam/cell certificate 行为不变；
6. 单测、回归、完整构建、diff-check 通过；
7. observe-only 动态通过；
8. active 动态至少证明 candidate 持续完整，并准确报告 Pair/H2 后续层；
9. 无安全参数、Tube 几何、Runtime、planner/C2、schema 或 thread 变化。

## 10. 停止边界

完成后写 self-audit、更新 handoff 并停止。不得自动进入 bootstrap observability、
M6 recenter、M4 authority 或 M7 删除阶段。
