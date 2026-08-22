# Planner/offset authority and handoff recovery — execution plan

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=PLANNER_OFFSET_AUTHORITY_HANDOFF_RECOVERY
IMPLEMENTATION_AUTHORIZED=true
USER_AUTHORIZATION=explicit request to schedule Terra/xhigh implementation
AUTO_ADVANCE=true
PLANNER_GEOMETRY_CHANGE_ALLOWED=false
PLANNER_PARAMETER_TUNING_ALLOWED=false
FILTER_CHANGE_ALLOWED=false
MARGIN_TUNING_ALLOWED=false
```

日期：2026-08-18
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. Objective

修正两层 authority 混用造成的故障，同时保持既有安全合同：

1. planner 的最终 C2 composite path 是基础导航 authority；
2. Tube 是绑定 path revision 的可选 OffsetAuthority，只决定是否允许非零 phase
   offset；
3. `observe_only=true` 或 retained `delta=0` 时，Tube 构建/验证失败不得阻塞原始
   planner 导航；
4. retained `delta!=0` 时，仍必须保持 matching path/tube/snapshot/runtime owner、
   exact pin/CAS/session 和 future-seam 合同；不得用 path-only 切换伪装非零 offset 安全；
5. 局部 planner path 末端但 goal 仍很远时，不得无限重复
   `all_candidates_path_end_clamped -> replan not installed -> HOLD`。

## 2. Fixed evidence and diagnosis

本阶段固定以下已复核事实：

```text
current_w = 5.112
old_path_end = 5.135
min_certified_forward_w = 0.400
required old future seam >= 5.512
goal distance = 8.20 m
```

因此 `certifiedFutureSeamCandidates()` 在 C2、Tube build、Runtime dry-run、CAS 之前
就返回空。KinoA*/B-spline candidate acceptance 成功不能证明 H2 handoff 成功。该事件是
局部 path exhaustion，不是真正 goal terminal，也不是 Tube geometry/Filter 失败。

另有非 terminal `replan not installed` 证据（约 `w=4.45`），其 first-false 层尚未动态
归因；不得用 terminal 证据替代该定位，必须保持 CAS/pin/session/runtime-bit 合同并单独
分类。

## 3. Required architecture

内部可使用一个不可观察的执行快照概念：

```text
ExecutionAuthority {
  planner_path_owner,
  planner_path_revision,
  optional matching tube/offset authority
}
```

允许的控制语义：

```text
matching Tube + valid offset authority:
    r = p + N*delta; nonzero offset may be selected

no matching Tube / observe-only / neutral delta:
    planner C2 path remains executable as the baseline
    phase-offset authority is disabled
    Tube continues as asynchronous observation/build evidence
```

不得制造一个未经证明的 `{0}` Tube。回到 baseline 必须表示为退出 offset authority、
使用原始 planner/GVF 控制路径；不是把 `{0}` 写入 Tube interval。

For `delta!=0`, path replacement is allowed only after a continuous recenter-to-zero proof
within the live old authority, or through an explicitly proven current-state recovery/bootstrap
owner. If neither exists, fail closed once with a classified recovery-required result; do not
weaken `min_certified_forward_w`, do not retry forever, and do not path-only switch while the
nonzero reference remains active.

## 4. Authorized files

```text
docs/Codex_Planner_Offset_Authority_Handoff_Recovery_Execution_Plan_2026-08-18.md
docs/Codex_Planner_Offset_Authority_Handoff_Recovery_Self_Audit_2026-08-18.md

src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
```

No CMake, launch, message, planner, TubeBuilder, Filter, SurfaceValidator, TubeEpochManager,
margin, map inflation, speed, rate, lookahead, tracking bound, or safety threshold edits are
authorized.

## 5. Implementation stages

### A. Observation/baseline authority split

- Keep the existing Tube timer and diagnostics for manual ESDF observation.
- Add an internal distinction between observation-timer availability and authoritative offset
  handoff.
- In `observe_only` and neutral `delta=0` cases, let the final C2 planner frontend install using
  the existing baseline/current-seam path logic.
- Ensure command guidance falls back to the newly installed planner owner when no matching
  offset authority exists; it must never use an old `PathTubePair` after a new planner owner is
  installed.
- Feed the adapter a null/non-authoritative pair for the sidecar build so it can publish Tube
  evidence without controlling the base path.

### B. Terminal handoff liveness

- Detect local path exhaustion separately from true goal completion.
- For neutral delta, install the accepted C2 planner path and invalidate the stale offset
  authority; rebuild Tube on the new final C2 owner.
- For nonzero delta, use only an existing/provable recenter or recovery contract. If source
  audit shows no safe owner, do not invent one; classify once and stop the stage with the exact
  blocker in the self-audit.
- Do not convert a terminal fail-closed hold into a generic emergency command.

### C. Preserve authoritative nonzero handoff

- Keep exact old/new owner matching, future seam, new-owner Tube build, Runtime dry-run, pin,
  generation/session and final CAS for nonzero offset transitions.
- Do not reuse old Tube/profile/sample/clearance on a new C2 connector.
- Do not treat planner candidate acceptance as Tube handoff acceptance.

### D. Separate nonterminal first-false diagnosis

- Add only test/internal evidence needed to distinguish future-seam empty, C2 failure,
  `stagePathTubePair`, prepared Tube, Runtime dry-run, transaction-slot, pin/CAS, and final
  mailbox failure.
- Do not claim the `w≈4.45` failures are fixed by the terminal repair without a reproducer.

## 6. Required regressions

At minimum:

```text
1. observe_only + ESDF Tube failure does not alter baseline planner navigation;
2. terminal local path end with goal far: accepted neutral candidate installs once;
3. true goal terminal remains normal WAIT/stop;
4. planner revision with no matching Tube rejects nonzero offset but keeps δ=0 baseline;
5. neutral delta path switch never consumes an old PathTubePair;
6. nonzero delta with valid future seam still uses atomic H2 handoff;
7. nonzero delta with insufficient horizon never path-only switches or retries forever;
8. command/timer/replan interleave never exposes new path + old Tube;
9. nonterminal first-false categories are not conflated with terminal seam exhaustion.
```

## 7. Acceptance and stop conditions

- Focused builds/tests for all modified targets pass.
- Full workspace build, `git diff --check`, whitelist audit and self-audit pass.
- No planner/Filter/margin/launch parameter is changed.
- The user-visible terminal episode no longer loops when `delta=0` or observe-only.
- If a safe nonzero recenter/recovery owner cannot be established from existing contracts,
  report that exact boundary instead of manufacturing a recovery command or weakening H2.

Stop after this stage. Do not advance to swarm or new recovery policy without a new execution
specification.
