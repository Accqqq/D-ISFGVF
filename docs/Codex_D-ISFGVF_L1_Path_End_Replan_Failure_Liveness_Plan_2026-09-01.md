# D-ISFGVF L1 Path-End Replan-Failure Liveness Plan

Date: 2026-09-01  
Mode: plan only  
Repository: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
Authoritative baseline: `6549f2eb238ff17790e92069adb087b7bb5024e6`

## 1. Decision

The direct stop is owned by `gvf_manager::runVelocityMatchingGovernor()`.
It rejects every candidate when every query is classified as path-end clamped,
then `makeGovernorInvalidHold()` publishes the current position with:

```text
final_cmd_source=GOVERNOR_INVALID_HOLD
fallback_reason=all_candidates_path_end_clamped
```

The first false liveness decision is not the Kino failure itself. It is the
Governor rule that rejects an exact, in-domain path endpoint together with the
strictly out-of-domain candidates. That converts a recoverable failed optional
replan into a command-invalid tick, prevents authoritative phase commit, and
allows the same old frontend and phase to recur.

The minimal repair has two inseparable parts:

1. Treat an exact endpoint query as inside the authoritative domain. Continue
   rejecting every query strictly beyond `path_end_w`; do not clamp an
   out-of-domain command into validity and do not extrapolate geometry.
2. When a periodic point-to-point replan has explicitly failed in the planner,
   the global mission goal is not reached, and the authoritative phase has
   consumed the retained frontend through its endpoint, set one planner-owned
   `PLANNER_INVALID` terminal classification for that retained frontend. Keep
   the frontend and command authority current, but suppress further optional
   `plan_interval` retries. Collision priority and the existing owner-bound
   current-state recovery route remain able to request `REPLAN_TRAJ`.

This pair is required. Endpoint acceptance alone would replace the invalid
HOLD loop with an endpoint-command/replan-failure loop. A terminal
classification alone would strand the remaining `0.012` of planner-authorized
support behind the existing invalid-HOLD policy.

The terminal classification does not declare the retained path geometry
invalid, does not retire it, and does not classify the local endpoint as the
mission goal. Its precise meaning is:

```text
the current planner-valid frontend has no remaining forward support,
and the periodic planner attempt produced no continuation
```

## 2. Frozen scope and non-goals

This plan does not modify or reinterpret:

- Kinodynamic A* search, heuristic, collision model, margin, or `check_num`;
- `astaropt()` planner logic or its success/failure contract;
- the B-spline optimizer, timing, or feasibility semantics;
- ESDF, map representation, `planning/safe_distance`, or collision thresholds;
- TubeBuilder, TubeCrossSectionSolver, SurfaceValidator, Tube epoch state, or
  the accepted R1 clearance-query repair;
- C2 geometry, planner-path validity, candidate switch scoring, or the C3
  successful-candidate force-accept rule;
- launch files, parameters, command rates, or controller gains;
- mission-goal tolerance or the existing goal-position override;
- command geometry beyond the installed path endpoint.

No Luna execution, source edit, test edit, launch edit, stage, commit, or push
is authorized by this document.

## 3. Read-only source audit

### 3.1 Periodic replan trigger owner

Owner: `gvf_manager::FSMCallback()` in
`src/swarm_planner/bspline_traj/src/gvf_manager.cpp`.

In `EXEC_TRAJ`:

```cpp
const bool periodic_due =
    (current_time - last_replan_time_).toSec() >= planInterval;
```

`selectReplanTrigger(collision_detected, periodic_due,
periodic_phase_ready, false)` selects `PERIODIC`, after which
`logReplanReason("plan_interval")` and
`changeFSMExecState(REPLAN_TRAJ, "planInterval reached")` run.

Properties:

- It is periodic/time-driven, not an edge tied to remaining path support.
- It has no near-end predicate.
- It is not suppressed or remembered after a failed replan because the fourth
  selector input is always `false`.
- Goal distance participates only in the preceding, distinct mission
  completion check `dist_xy < 0.2`; it is not part of `plan_interval`.
- `logReplanReason()` reports diagnostics only. `curr_i` is the clamped
  `current_traj_index_`; `progress_i` is the closest/lower-bound index derived
  from the reparameterization cache and `progress_w_`; `start_i` is the later
  of those indices; `end_i` is the collision-horizon end; `remaining_w` is
  `cache.w.back() - progress_w_`. None of these values gates the periodic
  trigger.

### 3.2 Failed point Phase V2 replan owner

Owner: the point Phase V2 `REPLAN_TRAJ` branch in
`gvf_manager::FSMCallback()`.

When `astaropt()` returns false:

- `installed` remains false;
- `pm.last_traj`, `pm.last_vel`, `pm.last_traj_time_`, the current continuous
  path owner, GVF mirror, Path+Tube pair, and `current_traj_index_` remain the
  old frontend state;
- `phase_w_` remains the captured `fsm_phase.w`;
- the old anchor is not replaced;
- `last_replan_time_` is reset to the current time;
- the FSM unconditionally returns to `EXEC_TRAJ`;
- no failure cause, edge, or terminal state is remembered.

The resulting retry schedule is another periodic attempt after the next
`planInterval`, with collision/current-state recovery triggers retaining their
existing independent priority.

### 3.3 Path-end candidate owner

Owner: `gvf_manager::runVelocityMatchingGovernor()`.

Candidates are generated from the configured lookahead interval plus feedforward,
previous, and rate-limit boundary candidates. After deduplication each
candidate uses:

```text
query_w = progress_w_after + L * phase_per_meter
```

For the active executed-reference query, the immutable domain is read from the
same current authority. For the neutral path, `pathPointAtW()` exposes the
installed `ContinuousPhasePath` domain and reports whether the original query
was beyond its end.

The current rule rejects a candidate when:

```cpp
clamped_to_end || query_w > candidate_path_w_end - 1e-6
```

Therefore an exact endpoint query is rejected despite being supported by both
`ContinuousPhasePath::evaluate(end_w, ..., false)` and
`PhaseOffsetExecutedReferenceQuery::query(end_w, ...)`. With the active launch
setting `gvf/cmd/governor_l_min=0.0`, the zero-lookahead candidate can be the
only in-domain candidate after `phase_candidate` is clamped to `path_end_w`,
but it is rejected by the epsilon rule. Every positive-lookahead candidate is
strictly beyond the domain. All candidates can consequently become invalid
solely because of path-end proximity.

### 3.4 Governor fallback and command authority

If no best candidate exists and
`path_end_clamped_count == candidate_count`, the Governor calls:

```text
makeGovernorInvalidHold(pos, "all_candidates_path_end_clamped", dbg)
```

The result:

- commands the current odometry position;
- sets `command_valid=false`;
- sets `selected_valid_for_state=false`;
- resets Governor state after successful publication;
- does not retain a previous valid forward command;
- is still published through the normal local command/adapter publication
  boundary.

`decideAuthoritativePhaseCommit()` requires `command_valid=true`. The invalid
HOLD therefore prevents `phase_w_` from advancing. The captured Path+Tube pair
and frontend remain current; no stale/currentness rejection caused the stop.

### 3.5 Mission-goal owner

`gvf_manager::FSMCallback()` owns mission completion using `pm.goal_pt` and the
point-goal condition `dist_xy < 0.2`. `cmdCallback()` also knows
`real_dis_to_goal` for the existing final goal-position override and logging.

The Governor path-end rule knows only the local installed path domain. It does
not distinguish:

- local frontend exhausted while `d_goal=8.64`, from
- the actual mission goal reached.

The plan preserves the FSM goal owner and adds no mission-completion inference
to the Governor.

### 3.6 Initial planning and frontend retirement audit

Point Phase V2 `GEN_NEW_TRAJ` retries a failed `astaropt()` while remaining in
`GEN_NEW_TRAJ`. It is an initial-acquisition retry loop, not a planner terminal
state. Routing L1 Case 4 into it would merely replace the observed periodic
loop with a new high-rate planning loop, so this plan does not use
`GEN_NEW_TRAJ` as the repair.

The existing authoritative frontend clear/retirement path is task/reset-owned:

- `resetForNewNavigationTask()` retires the task authority session and queues
  `pending_frontend_clear_session_`;
- `consumeFrontendClearMailbox()` is the FSM-owned writer that clears
  `pm.last_*`, the continuous path/GVF mirror, and Governor state;
- successful same-task replans replace the frontend only through the existing
  neutral commit or immutable Path+Tube handoff transaction.

A failed optional replan has no authority to invoke those clear/retirement
operations. The L1 repair will not add such authority.

### 3.7 ExecutionAuthority and currentness

Each command captures one immutable current path owner and, where active, one
Path+Tube pair. Guidance, the adapter, the executed-reference query, and the
Governor use that same owner. A successful PositionCommand publication and
authority transaction precede the no-fail authoritative phase commit.

The exact endpoint candidate will use this same transaction boundary. It does
not bypass `ExecutionAuthority`, install a second reference, or read a newer
frontend midway through the command. The planner terminal classification is
FSM-owned and does not retire or rewrite ExecutionAuthority state.

## 4. First liveness false and ownership

```text
FIRST_LIVENESS_FALSE_FUNCTION=gvf_manager::runVelocityMatchingGovernor
FIRST_LIVENESS_FALSE_CONDITION=all admissible lookahead candidates, including the exact in-domain zero-lookahead endpoint candidate, are rejected solely by clamped_to_end || query_w > path_end_w - 1e-6
FIRST_LIVENESS_FALSE_OWNER=command governor path-end candidate policy
```

The failed planner call is necessary input to the observed loop but is not the
first false command decision. The failed-replan branch retains an already
planner-valid path; the Governor then makes that path non-commandable before
its exact endpoint is consumed.

## 5. C3 relationship

The accepted C3 repair remains valid and unchanged:

- `shouldDeclarePointGoalReached()` removed the former `0.2 m` to `2.0 m`
  replan suppression;
- `shouldForceAcceptForGovernorPathShort()` and `shouldAcceptCandidate()`
  force acceptance of a successful planner candidate when the old frontend
  cannot support the configured Governor horizon.

L1 is outside C3's covered state. C3 has a successful candidate to accept. In
the frozen L1 trace, `astaropt()` fails, so no candidate reaches the C3 switch
policy. The missing behavior is:

```text
failed periodic planner continuation
+ retained frontend reaches exact end
-> one planner-owned terminal classification
```

No C3 threshold, score, goal policy, or successful-switch path changes.

## 6. Frozen repair design

### 6.1 Exact endpoint is in-domain

In `runVelocityMatchingGovernor()`:

- keep every strict out-of-domain query invalid;
- keep `clamped_to_end=true` invalid;
- remove the rule that rejects an otherwise valid query merely because it is
  within `1e-6` of the endpoint;
- allow `query_w == candidate_path_w_end` when the immutable path/query returns
  a valid finite reference and tangent;
- do not change candidate costs, lookahead generation, rate limits, lead
  limits, normal constraints, or selection order.

For the frozen state, `phase_candidate` is clamped by the existing
`clampPointPhaseCandidate()` to `16.549`. The `L=0` candidate queries exactly
`16.549`, commands that existing endpoint, returns `command_valid=true`, and
permits the existing post-publication phase transaction to commit
`phase_w=16.549`. Every `L>0` query remains invalid because it is strictly
beyond the path end.

No candidate is fabricated. No point outside the existing owner is evaluated
or commanded.

### 6.2 Remember only the active replan source

Add one narrow FSM-owned value that records whether the currently executing
`REPLAN_TRAJ` transition came from the periodic selector. It is set before the
existing `EXEC_TRAJ -> REPLAN_TRAJ` transition and cleared on every replan
exit/reset.

- `plan_interval` sets `active_replan_is_periodic=true`;
- collision and owner-bound current-state recovery set it to `false`;
- no counter, retry budget, timer, or generic failure history is added.

This source tag prevents a collision/current-state recovery failure or a
Tube/handoff denial from being relabelled as the L1 planner terminal.

### 6.3 Planner-owned terminal classification

Add one FSM-owned per-frontend state:

```text
point_path_end_planner_invalid_terminal=false/true
```

It may transition `false -> true` only when all conditions are true:

1. point Phase V2 is active;
2. the active replan source is `plan_interval`;
3. `astaropt()` explicitly returned false, so no planner candidate exists;
4. no candidate reached Tube/C2/handoff/install logic;
5. the retained authoritative path owner is finite and current;
6. authoritative `phase_w >= path_end_w - 1e-6`;
7. `shouldDeclarePointGoalReached(false, dist_xy, 0.2)` is false.

It must not transition to true for:

- an `astaropt()` success whose candidate is later not installed;
- Tube build/filter/validator/profile/staging failure;
- C2 or handoff failure;
- candidate score rejection;
- collision-triggered replan failure;
- owner-bound current-state recovery replan failure;
- actual mission completion;
- closed-reference mode.

When true:

- `pm.last_*`, the continuous path, GVF mirror, current pair, phase, progress
  index, and ExecutionAuthority remain current;
- no frontend clear or path retirement occurs;
- `EXEC_TRAJ` remains the command-producing FSM state, augmented by this
  explicit planner terminal classification;
- the Governor continues to publish the exact valid endpoint reference inside
  the retained domain;
- optional `plan_interval` selection is suppressed through the existing fourth
  `selectReplanTrigger()` input;
- collision remains higher priority in `selectReplanTrigger()`;
- the existing owner-bound current-state recovery mailbox may still route to
  `REPLAN_TRAJ` independently of periodic selection;
- a successful later safety/recovery replan or a new navigation task clears
  the terminal state only together with a new frontend/task owner.

The selector parameter/comment should be renamed from the obsolete
nonzero-handoff-specific wording to the exact general meaning
`periodic_replan_suppressed`. The nonzero H2 diagnostic remains excluded from
this gate; only the new planner-owned terminal state is passed to it.

### 6.4 State reset rules

Clear the terminal state and the active periodic-source tag on:

- accepted initial point frontend installation;
- successful point replan installation/staging;
- `resetForNewNavigationTask()` / new goal;
- unified phase reset/frontend retirement;
- actual mission completion.

Do not clear the terminal state merely because time passes. Do not retry it
with a counter. A safety/recovery replan that fails returns to the retained
terminal classification; a successful replacement changes the frontend and
then clears it.

### 6.5 Why this is not a new HOLD/retry/gate stack

The repair adds no command HOLD path. The endpoint command is an ordinary
valid Governor command using the current immutable reference query. It adds no
retry loop; it stops the optional periodic loop after planner-authorized
support is consumed. It adds no planner candidate gate; successful candidates
follow the existing C3/C2/handoff/install path unchanged.

The single terminal bit is the planner result required by the task's allowed
`PLANNER_INVALID` ownership. It is not Tube-owned and has no Tube,
visualization, clearance, or certificate input.

## 7. Required safety/liveness matrix

| Case | Active frontend | FSM state | Governor behavior | Command authority | HOLD allowed | Replan allowed | Path retirement allowed |
|---|---|---|---|---|---|---|---|
| 1. Forward support, replan succeeds | New accepted frontend after the existing atomic install/handoff | `EXEC_TRAJ`, planner terminal false | Existing normal candidate selection on new owner | Existing ExecutionAuthority/current planner owner | No new HOLD | Yes, existing collision/periodic/recovery rules | Only the existing planner-owned successful replacement of the old owner |
| 2. Forward support, replan fails | Old frontend retained | `EXEC_TRAJ`, planner terminal false because `phase_w < end_w` | Continue normal commands within old support | Existing current owner | Only pre-existing explicit safety invalidity; not L1 path end | Yes, existing triggers | No |
| 3. Nearly exhausted, goal not reached, replan succeeds | New candidate is force-accepted by existing C3 when applicable, then installed normally | `EXEC_TRAJ`, planner terminal false | Normal commands on new owner | Existing atomic handoff/commit | No new HOLD | Yes | Existing successful replacement only |
| 4. Nearly exhausted, goal not reached, periodic replan fails, current state not unsafe | Old frontend retained. First consume its exact endpoint; after a planner failure at committed end, retain it as terminal endpoint owner | `EXEC_TRAJ + PLANNER_INVALID terminal` after `phase_w=end_w` | Exact endpoint is a valid in-domain command; no `GOVERNOR_INVALID_HOLD` for path-end-only exhaustion | Same current owner and normal publication/phase transaction | `GOVERNOR_INVALID_HOLD` is not allowed for the endpoint-only condition. Natural convergence to the commanded endpoint is allowed | Optional periodic replan: no after terminal. Existing collision/current-state recovery routes: yes | No |
| 5. Nearly exhausted, explicit `CURRENT_STATE_UNSAFE=true` | Current owner retained by existing fail-closed semantics | Existing safety/recovery path; L1 periodic source tag is false | Existing unsafe-state invalid command/HOLD remains authoritative; endpoint acceptance is not used when guidance/authority is invalid | Existing CURRENT_STATE_UNSAFE owner | Yes, only under the existing explicit unsafe condition | Existing owner-bound recovery replan remains allowed, including while periodic replans are suppressed | No L1 retirement |
| 6. Actual mission goal reached | Final frontend retained for terminal visualization until the next goal reset | `WAIT_TARGET` | Existing mission completion behavior; active-goal command publication stops | Mission-goal FSM owner | Normal mission stop allowed | No, until a new goal | Only the existing next-goal/reset retirement, not local path-end logic |

Case 4 and Case 6 are deliberately different. Case 4 keeps
`pm.receive_goal=true`, retains the local endpoint owner, and records planner
continuation failure. Case 6 sets `pm.receive_goal=false` and is owned only by
the `dist_xy < 0.2` mission-completion rule.

## 8. Loop-freedom proof

Use the lexicographic progress value for the current frontend:

```text
P = (authoritative phase_w,
     point_path_end_planner_invalid_terminal ? 1 : 0)
```

For the frozen state:

```text
phase_w=16.537
path_end_w=16.549
d_goal=8.64
periodic astaropt failure
36/36 currently path-end rejected
```

the repaired transition is:

1. The exact endpoint `L=0` query is inside the immutable domain and is a valid
   Governor candidate. Every query beyond `16.549` remains invalid.
2. After successful command publication, the existing phase transaction
   commits `phase_w=16.549`. Thus `P` strictly increases in its first
   component. The old tuple with `phase_w=16.537` cannot recur through the
   path-end-only invalid-HOLD path.
3. If the next periodic planner attempt again returns no candidate while the
   mission goal remains unreached, the terminal bit changes `false -> true`.
   Thus `P` strictly increases in its second component.
4. While the terminal bit is true, the periodic selector cannot return
   `PERIODIC`. Therefore the same retained frontend cannot execute the same
   optional periodic failure transition again.
5. The bit can clear only with a successful replacement frontend or a new
   navigation task/reset. Either event changes the authoritative owner/task,
   so it cannot recreate the same frozen state.

An explicit collision or owner-bound current-state recovery event is a
different safety-owned transition and is not the observed optional
`plan_interval` loop. Command publication failure is likewise a separate
transport/transaction failure, not the frozen L1 path-end decision.

No sleep, timeout, retry counter, or infinite retry assumption appears in the
proof.

## 9. Deterministic test plan

All deterministic tests belong in the existing
`src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`; no new target
or CMake edit is required.

### 9.1 Reproduce and repair all-candidates-path-end-clamped

Extend the existing Governor test access to return command source, fallback
reason, selected query, path-end count, candidate count, and valid count.

Construct a finite path and valid guidance with:

- `governor_l_min=0.0` as in the active launch;
- `progress_w_after=path_end_w`;
- global goal represented as not reached in the policy fixture;
- all positive lookaheads strictly beyond the endpoint.

Baseline characterization must show the old rule would reject the endpoint.
The repaired assertion must show:

- one exact endpoint candidate is valid;
- `best_query_w == path_end_w`;
- `command_valid=true`;
- `final_cmd_source=VEL_MATCH_GOVERNOR`;
- commanded point equals the authoritative endpoint;
- every strictly beyond-end query remains rejected;
- no evaluation or command exceeds `path_end_w`.

Run this once through the neutral path query and once through an immutable
executed-reference query so active ExecutionAuthority semantics are covered.

### 9.2 Failed local replan policy without Kino changes

Add a pure policy helper/fixture input for:

- point mode active;
- active replan is periodic;
- `planner_candidate_found=false`;
- goal not reached;
- phase before end versus phase at end.

Assert failure before the endpoint keeps the current frontend and leaves the
terminal false. Assert failure at the endpoint sets the planner terminal.
This simulates planner failure without mocking or changing Kino internals.

### 9.3 Exact loop-freedom sequence

Drive the pure sequence:

```text
(16.537, terminal=false)
-> endpoint Governor command valid
-> authoritative phase commit decision to 16.549
-> periodic planner failure at 16.549
-> terminal=true
-> periodic selector returns NONE
```

Assert the exact original tuple cannot be selected again. Also assert collision
still wins over periodic suppression and the owner-bound recovery route remains
eligible independently.

### 9.4 Successful replan unchanged

Assert `planner_candidate_found=true` never sets the terminal, including:

- normal successful installation;
- C3 governor-path-short force acceptance;
- candidate found but not installed because a later C2/Tube/handoff step was
  unavailable.

The last case is mandatory proof that Tube state cannot create
`PLANNER_INVALID`.

### 9.5 Mission goal unchanged

Assert `shouldDeclarePointGoalReached(false, dist_xy, 0.2)` continues to own
mission completion and prevents the L1 terminal classification when true.
Closed-reference mode remains excluded.

### 9.6 Unsafe-state behavior unchanged

Assert collision/current-state recovery replan sources never set the L1
terminal. Existing invalid-command phase-commit tests must continue to show no
phase commit when command authority reports invalid/unsafe.

### 9.7 Reset and owner replacement

Assert a new task/reset or successful new frontend clears both the terminal and
active periodic-source tag. Elapsed time alone must not clear either state.

### 9.8 Tube independence

The terminal policy helper must have no Tube, clearance, visualization,
certificate, or R1 query input. A planner candidate found but later denied by
Tube/handoff must retain the old frontend and must not set the planner terminal.

## 10. Build and regression verification for future implementation

The implementation execution specification must require, at minimum:

```bash
catkin_make --pkg bspline_race
catkin_make run_tests_bspline_race
catkin_test_results build
git diff --check
git status --short
```

It must also verify that only the implementation whitelist changed and that the
accepted R1 file hashes remain identical to their pre-implementation values.

## 11. Same-scene ROS acceptance

Replay the same point-to-point scene that produced:

```text
remaining_w=0.012
d_goal=8.64
path_end_clamped_count=36
valid_count=0
repeated Kino failure
```

Required evidence:

1. The exact endpoint candidate is selected without any query beyond the
   authoritative path domain.
2. The authoritative phase commits from the pre-end value to the endpoint
   after successful command publication.
3. A later periodic `astaropt()` failure at the consumed endpoint produces one
   explicit planner terminal log containing the trigger source, phase, path
   end, remaining support, and mission-goal distance.
4. No later `reason=plan_interval` transition occurs for that same retained
   frontend while the terminal is set.
5. No `GOVERNOR_INVALID_HOLD/all_candidates_path_end_clamped` occurs solely
   because the exact endpoint is in range.
6. No command/reference query exceeds the installed path end.
7. A successful planner continuation still installs and resumes normal
   navigation.
8. A new navigation goal clears the terminal through the existing task reset.
9. Explicit current-state unsafe behavior can still HOLD and request its
   existing owner-bound recovery replan.
10. Local endpoint exhaustion is never logged or treated as mission goal
    completion while `d_goal=8.64`.
11. ZERO_ONLY, Tube invalidity, clearance diagnostics, and visualization state
    do not set or clear the planner terminal.

Acceptance is navigation through a valid planner-owned continuation or one
explicit planner-owned terminal after authorized support is consumed. A silent
zero-progress periodic loop is not acceptable.

## 12. R1 isolation

`R1_DIRECTLY_IN_LIVENESS_CHAIN=NO`.

The accepted R1 production changes are in the occupancy snapshot/query and
Tube geometry evidence path. `gvf_manager.cpp` may pass a cloud snapshot into
adapter/Tube preparation on successful candidate or active-offset paths, but
the frozen L1 chain does not depend on an R1 decision:

- `plan_interval` selection reads timer/collision/phase readiness only;
- the observed `astaropt()` returns false before point candidate Tube/C2/handoff
  staging is entered;
- old frontend retention is the direct `installed=false` FSM behavior;
- Governor candidate generation and
  `all_candidates_path_end_clamped` do not call an R1-modified function;
- the final command source proves the command reached the Governor path-end
  rule rather than stopping at Tube ZERO_BASELINE.

No R1 file may be modified or reverted by L1.

`TUBE_ZERO_BASELINE_DIRECT_STOP_OWNER=NO` for the frozen event.

## 13. Exact implementation whitelist

Only these code/test files are required by the frozen design:

```text
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
```

Why each file is necessary:

- `gvf_manager.h`: declare the two narrow FSM-owned state values and pure
  policy/test helpers; rename the periodic-suppression selector input semantics.
- `gvf_manager.cpp`: accept exact in-domain endpoint candidates, record the
  periodic replan source, classify the exact planner failure at consumed path
  end, maintain reset rules, and emit terminal diagnostics.
- `gvf_switch_policy_test.cpp`: deterministic endpoint, failure ownership,
  loop-freedom, safety-priority, reset, and Tube-independence coverage.

No CMake edit is needed because the test target already exists. Any required
change outside this whitelist is a scope expansion and must stop implementation
for a new plan review.

## 14. Rejected alternatives

- **Modify Kino/search/margin/safe distance:** violates the planner freeze and
  does not repair failed-replan ownership.
- **Clamp every beyond-end command to the endpoint:** turns out-of-domain
  geometry into a command and violates command-domain safety.
- **Endpoint candidate only:** still permits an endless endpoint-command plus
  failed-periodic-replan cycle.
- **Terminal classification only:** preserves the current false invalid HOLD
  before the remaining support is consumed.
- **Clear or retire the old frontend on failure:** gives an optional failed
  replan task/reset authority it does not own and can strand ExecutionAuthority.
- **Route to `GEN_NEW_TRAJ`:** replaces the loop with the existing initial-plan
  retry loop and supplies no legitimate terminal.
- **Set `WAIT_TARGET` or `receive_goal=false`:** falsely declares local frontend
  exhaustion to be mission completion.
- **Retry counter, delay, sleep, or timeout:** does not prove progress and adds
  forbidden retry machinery.
- **Tube-owned stop/replan/retirement or clearance certificate gate:** violates
  authority and could reject a planner-valid path because of Tube state.
- **Reuse the last forward command:** can outlive the domain/owner from which it
  was produced and breaks current command authority.

## 15. Main review answers

| Question | Answer | Evidence |
|---|---|---|
| Q1. Direct runtime stop owner proven? | YES | Governor returns `GOVERNOR_INVALID_HOLD/all_candidates_path_end_clamped` and publishes current position. |
| Q2. ZERO_BASELINE excluded as direct owner? | YES | The final command source is Governor path-end fallback; Tube diagnostics are not consulted by that branch. |
| Q3. Failed-replan/old-frontend/path-end loop proven? | YES | Point REPLAN retains old state, resets the timer, returns EXEC; invalid command prevents phase commit. |
| Q4. First liveness-false owner identified? | YES | `runVelocityMatchingGovernor()` rejects the exact in-domain endpoint candidate. |
| Q5. Kino search unchanged? | YES | Planner failure is treated as an input. |
| Q6. Planner validity semantics unchanged? | YES | No candidate validity gate changes; the retained path remains valid and current. |
| Q7. Tube/R1 geometry and clearance unchanged? | YES | No Tube/R1 file or predicate is in the implementation whitelist. |
| Q8. Repair inside planner-authorized support? | YES | Endpoint equality is allowed; every strict beyond-end query remains invalid. |
| Q9. Local end distinguished from mission goal? | YES | Planner terminal keeps the goal active; only `dist_xy < 0.2` enters `WAIT_TARGET`. |
| Q10. Case 4 deterministic and non-looping? | YES | Phase commits to end, then the per-frontend terminal bit changes once and suppresses periodic retries. |
| Q11. CURRENT_STATE_UNSAFE preserved? | YES | Non-periodic recovery causes cannot set L1 terminal; existing unsafe HOLD/recovery remains authoritative. |
| Q12. Successful replan preserved? | YES | Candidate-found/install paths and C3 acceptance are unchanged and clear terminal state only with a new frontend. |
| Q13. Can the repair create a new HOLD/retry/gate stack? | NO | It adds no HOLD or retry loop and uses one planner-owned terminal state with no Tube/planner-candidate gate. |

```text
MAIN_PLAN_REVIEW=PASS
```

## 16. Authorization state

```text
PLAN_MODE=L1_PATH_END_REPLAN_FAILURE_LIVENESS
IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
KINO_SEARCH_CHANGED=false
PLANNER_VALIDITY_CHANGED=false
TUBE_GEOMETRY_CHANGED=false
R1_CLEARANCE_CHANGED=false
SAFE_DISTANCE_CHANGED=false
DISPLAY_ONLY_P1_IMPLEMENTED=false
D3_STARTED=false
COMMIT_PUSH=false
```

