# A5/A6 Active PathTubePair continuity recovery — execution specification

```text
DOCUMENT_ROLE=CURRENT_STAGE_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5_A6_ACTIVE_PATHTUBEPAIR_CONTINUITY_RECOVERY
IMPLEMENTATION_AUTHORIZED=true
USER_AUTHORIZATION=explicit Terra/xhigh delegation on 2026-08-18
AUTO_ADVANCE_WITHIN_THIS_SPEC=true
AUTO_ADVANCE_TO_A7=false
PLANNER_KINO_BSPLINE_CHANGE_ALLOWED=false
FILTER_CHANGE_ALLOWED=false
MARGIN_OR_LAUNCH_TUNING_ALLOWED=false
SAFETY_FALLBACK_BYPASS_ALLOWED=false
```

日期：2026-08-18  
仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 1. Objective

修正当前 A5/A6 active manual ESDF episode 中的两个连续阻塞，但不得把二者混为一个
“终点条件太多”问题：

1. A5：Candidate 完整且初始 current raw interval 包含 `delta=0`，却因
   `BASE_CENTERLINE_CONTINUITY_UNCERTIFIED` 无法安装 Active/Certified Tube；
2. A6：Kino/B-spline 与 planner switch policy 已接受新路径，但联合
   `PathTubePair` 没有安装或没有持续存活，command callback 捕获不到
   `path_owner`，最终 `guidance_invalid -> GOVERNOR_INVALID_HOLD`。

本阶段必须先定位每条链的精确 first-false，再进行最小修复。不得凭现象直接删除
continuity、clearance、pin、session、Runtime dry-run 或 CAS 条件。

## 2. Fixed evidence

### 2.1 Isolated A5/A6 episode

权威报告：

`docs/Codex_A5_A6_Dynamic_Acceptance_Self_Audit_2026-08-18.md`

固定事实：

```text
raw/cloud Candidate builds present
Candidate Marker ADD present
Certified Marker ADD count = 0
active_tube_epoch = 0
initial current raw bounds = [-1.93125, 2.90000]
initial raw contains_zero = true
earliest epoch reason = BASE_CENTERLINE_CONTINUITY_UNCERTIFIED (13)
later reason = BASE_CENTERLINE_CLEARANCE_INSUFFICIENT (11)
planner C2 accepted replacements present
joint Active/Certified PathTubePair replacement absent
```

### 2.2 User terminal reproduction

The later active reproduction stopped near the goal with:

```text
best_query_w = 6.394
planner rows = 57
planner remaining_w = 1.252
Kino replan = success
B-spline parameterization = success
GVF switch policy = accept=1
POINT_PHASE_V2 = replan not installed
command path_w_start/path_w_end = 0/0
command candidate_count = 0
current semantic path unavailable
d_goal ~= 0.63 m
stop_radius = 0.30 m
```

This proves a missing authoritative command path while a planner frontend still exists. It does
not by itself prove whether the new Tube failed construction, failed certification, failed
transaction preparation/final CAS, or whether a previously live pair was retired too early.

## 3. Fixed semantics and invariants

```text
planner final C2 path p(w) = base navigation authority
tube = safety/offset authority bound to exactly one immutable planner path owner
r(w,delta) = p(w) + N(w)delta
```

- Neutral `delta=0`/observe-only baseline fallback remains as already implemented.
- Active manual offset may be nonzero. It must never path-only switch or execute without a
  matching certified Tube.
- A physically unsafe centerline or offset remains fail-closed.
- A failed replacement must not create an authority gap by retiring a still-valid old pair
  before a new pair commits.
- A new planner path must never execute with an old Tube.
- No old profile, snapshot, revision, generation, session or pin may be rebound to a new owner.
- Planner candidate acceptance is not H2 success.

## 4. Mandatory execution order

### G0 — provenance and non-mutating reproduction

Before product edits:

1. record `git status --short`, tracked diff names, HEAD and relevant binary Build-IDs;
2. run the existing focused unit/integration tests;
3. reproduce on a task-owned loopback ROS master and `ROS_HOME`, using the checked-in
   `phase_offset_esdf_tube_single.launch`, original `pillar.pcd`, and the only override
   `phase_offset_manual_observe_only:=false`;
4. do not attach to or terminate the user's ROS master/processes;
5. publish one `(8,0,1)` goal and record raw/cloud/epoch/Candidate/Certified/path/command/odom/
   rosout evidence from before the goal until arrival or stable blocker.

### G1 — A5 continuity-certificate first-false

For the first build carrying reason 13, obtain exact same-profile facts:

```text
source revision / authority session / map observation sequence
current_w / profile preview range / retained range
raw and filtered bounds at every retained knot
contains_zero at every retained knot
validator knot-evidence count and w values
first retained knot with no evidence
first retained knot with filtered_contains_zero=false
first retained knot with zero_surface_covered=false
cover radius / requested clearance / observed clearance at that knot
validator first_failure_w/reason and truncation range
```

Classify exactly one first-false:

1. a retained filtered knot truly excludes zero;
2. zero is present but physical continuous clearance/cover fails;
3. zero and clearance facts pass but knot evidence is lost/misaligned after validation or
   truncation;
4. current-w/profile query or immutable provenance mismatch;
5. another explicitly evidenced layer.

Only cases 3–5 authorize a behavior fix. Cases 1–2 remain fail-closed unless the evidence proves
an existing accounting implementation error. Do not reduce required clearance or cover radius.

### G2 — A5 conditional minimal repair

If G1 proves an implementation defect:

- repair only evidence/profile alignment, current-w querying, or the proven certificate mapping;
- the certificate must still cover every retained zero-centerline knot and every continuous cell;
- do not manufacture `{0}` or mark a failed cover as certified;
- add a regression reproducing the exact first-false and a negative physical-clearance case.

If G1 proves true physical insufficiency, make no product behavior change and report the exact
deficit and knot.

### G3 — A6 PathTubePair lifecycle first-false

Correlate one accepted replan from planner acceptance through command capture:

```text
old pair pointer/generation/session/revision and live/retired state
old pin lease/capture state
candidate path owner/revision and prepared Tube identity
stagePathTubePair result
preparePathTubePairCommit result and exact failed predicate
Runtime dry-run result
latest categorical safety result
finalize/CAS result
completed mailbox/FSM consume result
any retireOffsetAuthorityForPlannerOwner call and cause
command-callback captured pair/path owner immediately before/after the attempt
```

Classify whether:

1. no valid old pair ever existed because A5 never installed one;
2. a valid old pair was retired before a replacement was committed;
3. the new pair failed a legitimate nonzero-offset safety/recenter contract;
4. stage/prepare/finalize/mailbox failed due implementation identity, session, pin or lifecycle
   mismatch;
5. a neutral baseline branch was incorrectly treated as authoritative nonzero handoff.

### G4 — A6 conditional minimal repair

- If no old pair existed because A5 failed, do not invent lifecycle fallback; rerun after the A5
  repair/evidence disposition.
- If a valid old pair was retired early, preserve it until the replacement final CAS succeeds;
  on replacement failure keep the old pair only while its existing certified range remains valid.
- If stage/prepare/finalize/mailbox has an identity/session/pin implementation defect, repair the
  exact predicate and preserve all other H2 gates.
- If nonzero recenter/current-state recovery lacks a proof owner, keep fail-closed and report it;
  do not silently fall back to planner-only guidance.
- Correct the log text `baseline guidance retained` if active authoritative mode actually has no
  permitted baseline fallback; diagnostics must state the real fail-closed condition.

## 5. File whitelist

Only the following files may be modified:

```text
docs/Codex_A5_A6_Active_PathTubePair_Continuity_Recovery_Execution_Spec_2026-08-18.md
docs/Codex_A5_A6_Active_PathTubePair_Continuity_Recovery_Self_Audit_2026-08-18.md

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_diagnostics_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

Do not edit CMake unless an already-whitelisted existing test target cannot express the required
regression; in that case stop and request scope expansion rather than changing it.

## 6. Explicit prohibitions

Do not modify:

- planner/KinoA*/B-spline geometry, switch costs or safe distance;
- TubeBuilder ray geometry or exact PWL Filter;
- Tube margins, UAV radius, tracking/localization/map uncertainty, inset or cover parameters;
- speed, acceleration, lookahead, forward horizon, stop radius or update periods;
- launch/config/map files;
- observe-only defaults or manual amplitude;
- goal override or generic Governor HOLD policy;
- swarm/CBF/messages;
- old Tube reuse across a new path owner.

Do not delete a condition merely because many predicates exist. Every change must name the
specific false predicate and prove why it contradicts the intended existing contract.

## 7. Required tests

At minimum:

```text
phase_offset_tube_surface_validator_test
phase_offset_tube_epoch_manager_test
phase_offset_runtime_test
phase_offset_matched_adapter_test
phase_offset_tube_epoch_diagnostics_test
phase_offset_tube_epoch_integration_test
gvf_switch_policy_test
```

Required regressions:

1. all retained zero-containing knots with valid continuous cover produce a true centerline
   certificate;
2. a missing knot cover or a genuinely zero-excluding knot remains rejected;
3. failed replacement does not retire a still-valid old pair;
4. successful replacement changes path/tube owner atomically with generation advance;
5. command capture never sees new planner owner + old Tube;
6. command capture never sees a null owner gap caused solely by a failed replacement;
7. nonzero offset without a valid Tube/recenter proof remains fail-closed;
8. neutral/observe-only baseline behavior remains unchanged.

Then run an incremental/full build as appropriate, regression tests for completed stages,
dependency-boundary searches, `git diff --check`, final `git status --short`, and whitelist audit.

## 8. Dynamic acceptance

Use a new task-owned loopback ROS master. Preserve the same launch/map/override and publish one
`(8,0,1)` goal. Record a bag and provenance hashes.

PASS requires, in one joint episode:

```text
A5:
  at least one complete Candidate ADD
  at least one Certified/Active ADD
  active_tube_epoch > 0
  no false centerline certificate when physical evidence fails

A6:
  at least one planner C2 acceptance
  corresponding joint PathTubePair commit/generation advance
  no command-owner null gap between valid old and committed new pair
  no persistent GOVERNOR_INVALID_HOLD before true goal terminal
  final distance < stop_radius and normal goal completion
```

If the same physical route genuinely cannot certify the active nonzero manual offset, report that
physical blocker; do not label planner-path-only arrival as A5/A6 PASS.

## 9. Stop boundary

Complete this specification, write the self-audit, and stop. Do not begin A7, swarm intent,
parameter tuning, new recovery policy or scenario expansion.
