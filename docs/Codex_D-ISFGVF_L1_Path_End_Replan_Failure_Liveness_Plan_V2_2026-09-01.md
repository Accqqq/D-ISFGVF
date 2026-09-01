# D-ISFGVF L1 Path-End Replan-Failure Liveness Plan V2

Date: 2026-09-01  
Mode: plan correction only  
Repository: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
Baseline: `6549f2eb238ff17790e92069adb087b7bb5024e6`

## 1. Version and audit history

V1 remains preserved at:

```text
docs/Codex_D-ISFGVF_L1_Path_End_Replan_Failure_Liveness_Plan_2026-09-01.md
```

V1 passed Main review, but the exactly-one Sol/max review returned:

```text
SOL_FIRST_QUESTION_ANSWER=NO
SOL_BLOCKER_COUNT=0
SOL_MAJOR_COUNT=1
SOL_VERDICT=L1_PATH_END_REPLAN_FAILURE_LIVENESS_PLAN_NOT_ACCEPTABLE
```

The single MAJOR was that V1 allowed a point-replan “installation/staging”
success to clear the path-end planner terminal. In asynchronous H2,
`installed=true` can mean only that a candidate transaction was staged; it
does not prove that the exhausted old frontend has lost authoritative
execution ownership.

V2 corrects only that ownership/clear timing. All other L1 source findings,
authority boundaries, endpoint policy, case distinctions, non-goals, and
implementation scope remain frozen.

```text
V1_SOL_MAJOR_FIXED=TERMINAL_CLEAR_ONLY_AFTER_AUTHORITATIVE_OWNER_REPLACEMENT
```

## 2. Frozen L1 behavior retained from V1

The following V1 conclusions remain normative and unchanged:

- Direct stop owner:
  `gvf_manager::runVelocityMatchingGovernor()`.
- Direct stop reason:
  `GOVERNOR_INVALID_HOLD/all_candidates_path_end_clamped`.
- First liveness false:
  the Governor rejects the exact in-domain endpoint together with strictly
  beyond-end candidates.
- Exact endpoint equality is valid inside both `ContinuousPhasePath` and the
  immutable executed-reference query.
- Every query strictly beyond the authoritative endpoint remains invalid.
- No geometry is extrapolated or synthesized past the endpoint.
- A periodic `astaropt()` failure before frontend exhaustion retains the old
  frontend and returns to normal execution.
- A periodic `astaropt()` failure after the retained frontend is consumed and
  while the global goal is not reached may set one planner-owned terminal for
  that exact frontend.
- Collision and owner-bound current-state recovery are not periodic failures
  and cannot set the L1 terminal.
- C3 successful-candidate force acceptance is unchanged.
- Mission completion remains exclusively owned by the point-goal
  `dist_xy < 0.2` rule.
- Kino, optimizer, planner validity, ESDF, safe distance, Tube/R1 geometry,
  clearance, visualization, launch, and parameters remain unchanged.
- No new HOLD command, retry counter, timeout, generic retry state machine, or
  planner candidate gate is introduced.

The V1 implementation concept remains:

1. permit the exact valid endpoint candidate;
2. bind the optional-periodic suppression to one planner terminal belonging to
   the exhausted current frontend;
3. keep collision and existing current-state recovery priority available.

## 3. Authoritative replacement source audit

### 3.1 Staging is not frontend replacement

In the point Phase V2 H2 replan branch,
`stageFutureSeamPathTubeTransaction()` success causes:

```cpp
if (frontend_ready && !install_w.empty() && path_tube_handoff_required) {
    installed = true;
}
```

This `installed` value means only that the immutable transaction is pending.
The branch comment explicitly says the command boundary applies the frontend
only after the same pair CAS succeeds. Neither `pm.last_*` nor the GVF
frontend mirror is replaced at this point.

Therefore no terminal clear may be connected to:

- candidate acceptance;
- candidate/C2 preparation;
- Tube preparation;
- `stageFutureSeamPathTubeTransaction()` returning true;
- `frontend_ready=true`;
- `installed=true` in the asynchronous H2 branch;
- pending handoff publication;
- command-side prepare success.

### 3.2 H2 `COMMITTED` mailbox is still not FSM owner replacement

`prepareAndCommitPendingPathTubeHandoff()` performs final pair validation/CAS,
moves the transaction from `pending_path_tube_handoff_` to
`completed_path_tube_handoff_`, and sets lifecycle `COMMITTED`.

That operation intentionally does not write `pm.last_*` or the legacy/current
GVF frontend mirror. The FSM remains the sole frontend writer. Consequently:

```text
PathTubeHandoffLifecycleResult::COMMITTED
```

is not a terminal-clear event. A completed mailbox that has not yet been
consumed must retain the terminal of the old FSM frontend.

### 3.3 H2 authoritative frontend replacement linearization

The H2 terminal-clear point is inside
`gvf_manager::consumeCompletedPathTubeHandoff()` and only on its successful
consume path.

The function holds the existing frontend-apply and handoff serialization,
then proves:

- the completed mailbox exists;
- candidate pair and payload are valid;
- authority session matches;
- the frontend payload is structurally valid;
- the live pair has the same existing authority identity as the completed
  candidate using session, source revision, and immutable path-owner identity;
- `applyPathTubeFrontendMirrorLocked()` installs the candidate into
  `pm.last_*` and the current continuous path owner;
- `installAuthoritativePathMirrorLocked()` succeeds;
- the same completed mailbox is still current;
- the mailbox is retired and lifecycle becomes `CONSUMED`.

Only after these facts are true has the new frontend become current from the
FSM/command frontend boundary. `DROPPED_STALE`, `DROPPED_EXPIRED`,
`RETRY_PENDING`, `COMMITTED`, missing consumer, or failed consume are not
replacement events and cannot clear the old-owner terminal.

### 3.4 Neutral authoritative replacement linearization

The neutral/synchronous clear point is the successful true-return path of
`gvf_manager::commitNeutralPlannerFrontend()`.

Under the existing frontend-apply and handoff locks, the function:

1. rejects a pending/completed H2 handoff;
2. atomically retires any safely neutral old offset authority;
3. applies the new planner frontend/current continuous owner;
4. installs the authoritative path mirror;
5. returns success only for the completed direct replacement path.

Preparation or an attempted neutral commit that returns false cannot clear the
terminal. The clear must be executed at the successful replacement
linearization, after the current owner is confirmed to be the new owner, not in
the caller merely because it intended or attempted an install.

### 3.5 Existing identity is sufficient

No new ownership architecture is needed. The manager already has all identity
facts required to bind a terminal to the exact authoritative frontend:

- immutable `ContinuousPhasePath` owner identity;
- `path_tube_authority_session_`;
- H2 `PathTubePair::source_revision`;
- H2 `PathTubePair::path_owner`;
- `samePathTubeAuthority()`, whose equivalence is authority session, source
  revision, and immutable path-owner identity;
- existing frontend-apply/handoff locks that serialize replacement and mailbox
  consumption.

V2 will store only a small value-semantic terminal owner identity copied from
these existing facts. It is not a new path authority, does not own a command,
and cannot install or retire a frontend.

Conceptually:

```text
PointPathEndPlannerTerminalOwner {
    active
    immutable_path_owner_identity
    authority_session
    owner/source_revision
}
```

For an H2 current owner, the identity is derived from the current live pair and
the matching current frontend path. For a neutral frontend, it is derived from
the current continuous planner owner plus the current manager session/revision
facts. Exact immutable-owner identity remains the primary discriminator; the
existing session/revision values prevent cross-task or stale completion reuse.

## 4. Corrected terminal invariant

The V2 invariant is:

```text
TERMINAL_CLEAR
    iff
THE_TERMINAL'S_OLD_OWNER_HAS_LOST_AUTHORITATIVE_FRONTEND_OWNERSHIP
AND
A_DIFFERENT_NEW_FRONTEND_OWNER_IS_CURRENT

or

THE_EXISTING_TASK/MISSION_OWNERSHIP_IS_TORN_DOWN
```

It is never:

```text
candidate prepared
candidate accepted
candidate staged
handoff requested
handoff pending
pair CAS completed but FSM frontend unconsumed
installed/staged Boolean true
```

The terminal remains active whenever its recorded identity still matches the
current exhausted frontend identity. Periodic suppression is evaluated from
this owner-bound match, not from an unqualified staging result.

## 5. Corrected terminal lifecycle

### 5.1 Terminal creation remains unchanged

The terminal is created only when all frozen V1 predicates hold:

1. point Phase V2 is active;
2. the active replan source is `plan_interval`;
3. `astaropt()` explicitly returns false;
4. no planner candidate enters Tube/C2/handoff logic;
5. the current authoritative frontend identity is valid and captured;
6. authoritative phase has reached its path end within the existing numerical
   endpoint tolerance;
7. the mission goal is not reached.

Creation records the exact exhausted owner identity. Candidate-found but
Tube/C2/handoff-denied outcomes cannot create a terminal.

### 5.2 Non-clear events

The terminal must survive all of the following while the recorded owner remains
the current frontend:

- elapsed time and repeated FSM ticks;
- candidate preparation or acceptance;
- C2 construction;
- Tube/profile preparation;
- asynchronous H2 staging success;
- `pending_path_tube_handoff_` presence;
- command-side handoff preparation;
- `RETRY_PENDING`;
- pair CAS and lifecycle `COMMITTED`;
- `completed_path_tube_handoff_` presence without FSM consumption;
- a temporarily unavailable completed-mailbox consumer;
- stale, expired, rejected, cancelled, superseded, or failed handoff;
- neutral replacement preparation or a neutral commit returning false.

No `installed`, `frontend_ready`, staging-success, pending, or completed-mailbox
Boolean may call the terminal-clear helper.

### 5.3 Neutral clear rule

At entry to `commitNeutralPlannerFrontend()`, capture the currently installed
frontend identity as the expected old owner. On the successful replacement
path, after the new path owner and authoritative mirror are current, clear the
terminal only if:

1. the terminal is active;
2. its identity exactly matches the captured old owner;
3. the current frontend identity exactly matches the requested new owner;
4. the new owner differs from the old owner;
5. the function is on its successful authoritative commit path.

If any condition fails or the commit returns false, retain the terminal.

The clear occurs inside the existing serialized replacement operation or in an
immediately adjacent no-fail step while the same locks still prove the owner
transition. It is not deferred to a caller that sees only `installed=true`.

### 5.4 Asynchronous H2 clear rule

Staging and pair CAS leave the terminal unchanged.

At entry to `consumeCompletedPathTubeHandoff()`, retain the current FSM
frontend identity as the expected old owner. Clear only on the path that:

1. validates the completed candidate/session/current live authority;
2. applies the completed frontend mirror;
3. successfully installs the authoritative path mirror;
4. confirms the same completed mailbox is still current;
5. retires that mailbox with lifecycle `CONSUMED`;
6. confirms the new current frontend identity matches the consumed candidate;
7. confirms the terminal identity matched the replaced old owner and the new
   owner differs from it.

The old-owner terminal is retired/cleared as part of this exact owner
replacement linearization. It is not cleared for lifecycle `COMMITTED`, because
`COMMITTED` means only that a completed mailbox is available.

### 5.5 Ownership teardown clear rule

The terminal may also clear when the old task/mission ownership itself ends:

- `resetForNewNavigationTask()` / new-goal authority-session teardown;
- unified frontend/task reset that retires the current owner;
- actual point mission completion entering `WAIT_TARGET`.

These are existing ownership boundaries, not staging or replacement guesses.
The active periodic-source tag is cleared at the same teardown boundaries.

### 5.6 Successful replan callers

The point replan branch continues to use its existing `installed` value for its
current FSM/handoff behavior, but it must not use that value to clear the
terminal.

- Neutral success clears inside the successful neutral commit linearization.
- H2 staging success returns to `EXEC_TRAJ` with the terminal still active and
  periodic replans still suppressed.
- The independent command callback continues processing the pending handoff;
  periodic suppression does not prevent prepare/CAS/mailbox publication.
- The FSM clears only when its later completed-mailbox consume makes the new
  frontend current.

Thus delaying terminal clear does not reject or block a successful H2
candidate.

## 6. Corrected behavior matrix

| Case | Active frontend and terminal | FSM / replan behavior | Governor and command authority | Clear/retirement rule |
|---|---|---|---|---|
| 1. Forward support, neutral replan succeeds | Old frontend until `commitNeutralPlannerFrontend()` completes; then new owner current and old-owner terminal, if any, clears | Existing `REPLAN_TRAJ -> EXEC_TRAJ` | Existing command path on the authoritative owner | Clear only after successful neutral authoritative replacement |
| 1-H2. Forward support, H2 staging succeeds | Old FSM frontend remains current; any old-owner terminal remains through pending and `COMMITTED`. New frontend becomes current only at successful FSM consume | Return to `EXEC_TRAJ`; command callback advances the pending transaction independently | Existing old/current authority until H2 transaction and frontend consume complete | Clear only at lifecycle `CONSUMED` plus verified new current owner |
| 2. Forward support, replan fails | Old frontend retained; terminal remains false because support remains | Existing `EXEC_TRAJ`; ordinary triggers remain | Existing normal commands inside old support | No clear or retirement |
| 3. Nearly exhausted, replan succeeds | C3 acceptance remains unchanged. Neutral/H2 clear timing follows Case 1/1-H2 | Existing successful pipeline | No new HOLD or domain change | Never clear on staging alone |
| 4. Exhausted, goal not reached, periodic planner failure, safe current state | Old owner remains current with `terminal(old)=true` | `EXEC_TRAJ`; optional periodic selection suppressed | Exact endpoint command remains inside old owner domain | No path retirement. Terminal survives until actual replacement or task teardown |
| 4-H2. Terminal(A)=true and candidate B stages | A remains FSM frontend owner; terminal(A) remains true while B is pending or merely completed | No periodic reopen. Command callback may complete B; FSM later consumes | Existing authority transaction only; no new command owner | Clear terminal(A) only after consumed B is the verified current frontend |
| 5. Explicit current state unsafe | Current owner retained; its terminal, if present, is not cleared by safety replan staging/failure | Existing owner-bound safety recovery may enter `REPLAN_TRAJ` despite periodic suppression | Existing CURRENT_STATE_UNSAFE HOLD remains authoritative | Only an actual successful owner replacement clears; safety failure does not |
| 6. Actual mission goal reached | Mission/task ownership ends; final visualization remains per existing behavior | `WAIT_TARGET`, no optional replan | Existing mission stop | Terminal clears at mission ownership teardown, not because local endpoint equals goal |
| 7. New task/reset | Old owner/session retired through existing reset | Existing new-task flow | New task obtains a distinct authority session/owner | Old terminal clears with task teardown |

Local frontend exhaustion and mission completion remain distinct. Case 4 keeps
the goal active and never sets `receive_goal=false`; Case 6 alone uses the
mission-completion rule.

## 7. Corrected loop-freedom proof

Define the terminal as an owner-bound value:

```text
T(A) = terminal active for exact authoritative frontend identity A
```

Optional periodic replanning is suppressed exactly while `T(A)=true` and A is
still the current frontend.

### 7.1 Frozen path-end transition

For the original state:

```text
phase_w=16.537
path_end_w=16.549
d_goal=8.64
periodic astaropt failure
all candidates previously path-end rejected
```

the exact endpoint candidate becomes valid, remains inside the immutable
domain, and permits the existing command transaction to commit phase to
`16.549`. A later periodic planner failure at the consumed endpoint creates
`T(A)=true`. Optional periodic selection for A is then suppressed.

### 7.2 Required asynchronous timeline

The authoritative H2 timeline is:

```text
old frontend A exhausted
T(A)=true
    -> periodic suppressed
candidate B prepared by an existing non-periodic/safety recovery path
    -> async H2 staging succeeds
    -> A remains current FSM frontend
    -> T(A) remains true
    -> periodic remains suppressed
pending B may remain pending for any number of FSM ticks
    -> T(A) remains true
pair CAS may publish completed mailbox B (COMMITTED)
    -> B is still unconsumed by FSM
    -> T(A) remains true
FSM validates and consumes B
    -> B becomes current authoritative frontend
    -> lifecycle=CONSUMED
    -> terminal identity matches replaced A
    -> retire/clear T(A)
```

No staging-success Boolean changes periodic suppression.

### 7.3 Failure and stale outcomes

If B is pending, retry-pending, stale, expired, cancelled, rejected, failed,
superseded without owner replacement, or completed but never consumed:

```text
A remains current
T(A) remains true
periodic remains suppressed
```

Therefore the original `EXEC_TRAJ -> plan_interval -> failed replan ->
EXEC_TRAJ` zero-progress loop cannot reopen merely because B was prepared,
staged, committed to a mailbox, or later dropped.

If a different valid owner B actually becomes current, the old tuple is no
longer reproducible because the authoritative frontend identity has changed.
Clearing `T(A)` at that exact transition cannot reactivate retries on A.

### 7.4 Neutral timeline

Neutral preparation and failed commit keep A and `T(A)`. A successful neutral
commit changes the current owner from A to B under the existing serialized
linearization, then retires `T(A)`. The old owner cannot become current again
without a separate authority transition.

### 7.5 Monotonic measure

For a given task, use:

```text
P = (frontend_owner_epoch,
     authoritative_phase_w,
     terminal_for_current_owner ? 1 : 0)
```

- exact endpoint acceptance strictly advances phase to the old frontend end;
- terminal creation changes the third component `0 -> 1` for that owner;
- staging/pending/COMMITTED do not decrease it;
- successful authoritative replacement strictly changes the first component
  before the old terminal is retired;
- task reset changes the authority session/task epoch.

No permitted clear returns to the same owner, same phase, terminal false state.
The proof uses no retry count, timeout, wait, or assumed eventual handoff.

```text
LOOP_FREEDOM_PROVEN=YES
```

## 8. Deterministic test plan

All tests remain in the existing
`src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp` target.

The V1 exact-endpoint, strict-beyond-end, periodic-failure classification,
goal distinction, current-state safety, command-domain, C3 success, reset, and
Tube-independence tests remain required.

Add the following V2 transaction tests using the existing mailbox, lifecycle,
neutral-commit, frontend-owner, authority-session, and test-access fixtures.

### A. Staging success without consume

Arrange `terminal(A)=true`, stage candidate B successfully, and leave the
handoff pending.

Assert:

- current frontend identity remains A;
- terminal identity remains A and active;
- periodic selector is suppressed;
- no terminal-clear call is derived from `installed=true` or staging success.

### B. Pending across multiple FSM ticks

Keep B pending across multiple consume/FSM observations without successful
completion.

Assert A and `terminal(A)` remain bitwise/identity stable and periodic remains
suppressed on every tick.

### C. Stale, failed, cancelled, and expired handoff

Cover `DROPPED_STALE`, `DROPPED_EXPIRED`, prepare failure with
`RETRY_PENDING`, explicit cancellation/supersession that does not replace A,
and rejected final CAS.

Assert:

- A remains current;
- `terminal(A)=true`;
- periodic remains suppressed;
- no failed lifecycle outcome clears terminal.

### D. Completed but unconsumed

Move B to `completed_path_tube_handoff_` with lifecycle `COMMITTED`, but do not
call successful FSM consumption. Also cover a temporarily unavailable consumer
that retains the completed mailbox.

Assert A remains the FSM frontend, `terminal(A)=true`, and periodic remains
suppressed.

### E. Valid completed handoff consumed

Start with A current and `terminal(A)=true`, then validate and consume completed
B through `consumeCompletedPathTubeHandoff()`.

Assert atomically at the successful boundary:

- B is the current continuous/frontend owner;
- mailbox is empty;
- lifecycle is `CONSUMED`;
- B identity differs from A;
- `terminal(A)` clears exactly once;
- a repeated consume cannot clear or affect a newer terminal.

Add an owner-mismatch variant proving completion for B cannot clear a terminal
whose recorded old identity does not match the frontend being replaced.

### F. Neutral preparation/commit failure

Arrange A current and `terminal(A)=true`; prepare neutral B but force the
existing commit precondition to fail or prevent the commit call.

Assert A and its terminal remain current/active and periodic stays suppressed.

### G. Neutral authoritative commit succeeds

Commit B successfully through `commitNeutralPlannerFrontend()`.

Assert:

- function returns true;
- B is the current owner and differs from A;
- old pair/session retirement semantics remain unchanged;
- `terminal(A)` clears only after the current-owner observation is B;
- no caller-side staging/installed Boolean performs the clear.

### H. New task/reset and mission teardown

Assert new-task reset changes the authority session/task identity and clears the
old terminal through existing teardown. Separately assert true mission
completion clears the terminal while local path-end exhaustion alone does not.

### I. Owner binding and same-path refresh

Prove a same-authority timer/profile refresh that changes only the pair object
but preserves session, source revision, and immutable path owner does not clear
the terminal. Prove a stale completion from another session/revision cannot
clear it.

### J. Successful candidate pipeline unchanged

Assert H2 staging is still accepted and progressed by the independent command
callback while periodic selection is suppressed. The delayed clear must not
block prepare, final CAS, completed mailbox publication, or successful FSM
consume.

## 9. Same-scene ROS acceptance

Replay the same L1 scene and retain all V1 acceptance requirements:

```text
remaining_w=0.012
d_goal=8.64
path_end_clamped_count=36
valid_count=0
repeated Kino failure
```

Required V2 lifecycle evidence:

1. The exact endpoint command is valid and never exceeds the authoritative
   domain.
2. A consumed-end periodic planner failure sets terminal identity A.
3. Logs include A's owner/session/revision identity and show periodic
   suppression while A remains current.
4. If candidate B stages, the terminal remains active; no terminal-clear event
   accompanies staging or `installed=true`.
5. Lifecycle `RETRY_PENDING`, `DROPPED_STALE`, `DROPPED_EXPIRED`, and
   `COMMITTED` do not clear A's terminal.
6. A completed but unconsumed B leaves A's terminal active.
7. The terminal-clear log occurs only with either:
   - successful neutral authoritative commit with current owner B; or
   - successful H2 lifecycle `CONSUMED` with current owner B.
8. The clear log records both replaced owner A and new current owner B and
   proves they differ.
9. If B never becomes current, A's terminal and periodic suppression persist
   without reopening the original loop.
10. A new task/reset clears through authority-session teardown.
11. Actual mission completion remains distinct from local frontend exhaustion.
12. No Tube, clearance, ZERO_ONLY, visualization, or R1 state sets or clears
    the terminal.

## 10. Exact implementation whitelist

The correction remains fully implementable inside the V1 whitelist:

```text
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
```

Planned responsibilities:

- `gvf_manager.h`
  - define the value-semantic terminal owner identity;
  - declare exact owner capture/match/clear helpers;
  - keep the periodic-source and terminal policy testable.
- `gvf_manager.cpp`
  - preserve endpoint acceptance and terminal creation from V1;
  - remove every terminal clear derived from replan `installed/staged` state;
  - clear neutral terminal only inside successful
    `commitNeutralPlannerFrontend()` replacement;
  - clear H2 terminal only inside successful
    `consumeCompletedPathTubeHandoff()` replacement/lifecycle `CONSUMED`;
  - retain terminal for all pending/failure/stale/unconsumed outcomes;
  - clear on existing task/reset/mission teardown.
- `gvf_switch_policy_test.cpp`
  - add mandatory A–J lifecycle, owner-binding, and loop-freedom tests using
    existing fixtures.

No adapter, ExecutionAuthority, Tube, planner, CMake, launch, parameter, or R1
file is required. If implementation proves otherwise, stop for scope expansion
and mark the plan not accepted.

## 11. Planner, command, Tube, and safety non-interference

- Successful candidate construction and H2 staging remain unchanged.
- Delayed terminal clear cannot block the command callback's independent
  pending-handoff prepare/CAS path.
- ExecutionAuthority remains the sole accepted command transaction owner.
- The FSM remains the sole frontend mirror writer.
- Endpoint equality remains in-domain; strict beyond-end remains invalid.
- The terminal identity is observational policy state only and cannot reject a
  planner-valid path or a Tube candidate.
- Tube/R1/clearance/visualization state cannot create, match, or clear the
  terminal.
- Existing explicit CURRENT_STATE_UNSAFE HOLD/recovery remains able to request
  replan independently of optional periodic suppression.
- Collision priority in `selectReplanTrigger()` remains unchanged.
- No path is cleared or retired merely because an optional replan failed.

## 12. Main V2 review

| Question | Answer | Evidence |
|---|---|---|
| Q1. Can async staging success clear terminal before owner replacement? | NO | No replan `installed/frontend_ready/staged` Boolean is a clear site. |
| Q2. Can pending/stale/failed/unconsumed H2 clear terminal? | NO | Only the successful `CONSUMED` frontend-replacement path clears; all other lifecycle states retain. |
| Q3. Does terminal remain attached to the exhausted owner until replacement? | YES | Terminal stores the existing immutable owner/session/revision identity and suppression requires that owner to remain current. |
| Q4. Does neutral commit clear only after authoritative replacement? | YES | Clear occurs only inside the successful `commitNeutralPlannerFrontend()` linearization after new owner/mirror currentness. |
| Q5. Does H2 clear only after completed handoff consumption and owner replacement? | YES | Lifecycle `COMMITTED` retains terminal; successful FSM `CONSUMED` plus new-owner verification clears. |
| Q6. If replacement never completes, does suppression remain active? | YES | Pending, retry, stale, failed, cancelled, expired, and unconsumed outcomes retain A and `terminal(A)`. |
| Q7. Can staging alone reopen the original periodic loop? | NO | Staging cannot change terminal identity or periodic suppression. |
| Q8. Are planner/Kino/Tube/R1/C3 semantics unchanged? | YES | No affected file or decision lies outside the same three manager/test paths. |
| Q9. Is exact endpoint acceptance unchanged from V1? | YES | Endpoint equality remains valid; strict beyond-end rejection remains. |
| Q10. Is loop freedom proven across async staging/replacement? | YES | Terminal remains monotonic for A through pending/COMMITTED and clears only after owner epoch changes to B. |

Successful replan behavior is preserved more precisely than V1: the candidate
pipeline may stage and commit normally, but the old terminal clears only at the
existing authoritative owner replacement point. This timing change neither
rejects nor delays the transaction itself.

```text
MAIN_V2_REVIEW=PASS
```

## 13. Authorization state

```text
PLAN_MODE=L1_PATH_END_REPLAN_FAILURE_LIVENESS_V2
IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
KINO_SEARCH_CHANGED=false
PLANNER_VALIDITY_CHANGED=false
TUBE_GEOMETRY_CHANGED=false
R1_CLEARANCE_CHANGED=false
SAFE_DISTANCE_CHANGED=false
CODE_MODIFIED=false
COMMIT_PUSH=false
```

