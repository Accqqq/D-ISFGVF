# Tube V2 G16 D/E construction and neutral-liveness execution spec

```text
DOCUMENT_ROLE=PRODUCT_CORRECTION_EXECUTION_SPEC
DOCUMENT_STATUS=PARTIALLY_FIXED
DATE=2026-08-22
BASELINE=G15_STATIC_COMPLETE_DYNAMIC_CONSTRUCTION_BLOCKED
PLAN_OWNER=PRIMARY_CODEX_AGENT
EXECUTOR=LUNA_MAX_SINGLE_AGENT
SUBAGENT_CREATION_ALLOWED=false
DELEGATION_ALLOWED=false
PARALLEL_AGENT_WORK_ALLOWED=false
NO_NEW_GATE_CACHE_RETRY_TIMER_WATCHDOG_MODULE_QUEUE=true
NO_LIMIT_OR_SAFETY_MARGIN_CHANGE=true
```

G15 Phase A marker ownership is frozen.  G16 has two bounded owners:

- D: remove only mathematically identical repeated construction proof work;
- E: use the existing return-to-center and neutral commit path after a failed
  nonzero Tube/Pair stage, before the old executable owner expires.

G16 must not restore the old Filter/global envelope, increase query/depth/
attempt limits, relax clearance/margins, promote a failed Tube, or directly
install a neutral planner frontend while nonzero authority is still active.

The G15 visibility correction, marker lifecycle, exact-knot reuse, and neutral
commit correction are frozen unless a G16 focused test proves a direct defect.
Before editing, record their current source hashes and focused test results.
Do not mix D and E into one patch: finish and test D, then finish and test E.

## Execution protocol

1. Preserve the dirty worktree.  Do not reset, restore, checkout, clean, stash,
   rebase, overwrite an untracked file, or edit an unrelated user asset.
2. Create one fresh evidence directory `/tmp/tube_v2_g16_<id>/` containing
   baseline hashes, commands, test logs, dynamic logs/bag, and cleanup proof.
3. Do not add production diagnostics merely to obtain evidence.  Test-only
   counters/fakes are allowed; dynamic evidence must use existing ROS output.
4. A source-proven invariant may remove repeated work.  A statistical pattern,
   repeated log string, or equal rounded value is not sufficient proof.
5. Continue automatically through a phase when its focused tests pass.  If a
   proposed simplification lacks a safety proof, do not improvise; record the
   missing contract and continue with the independently safe E work.

## D — certified construction

1. Use only `/tmp/tube_v2_g15_dynamic_20260822_021000` plus current source.
   Do not start a new diagnostic launch before the source classification. Produce
   a per-attempt table for one 101-attempt Z2 build: requested lower/upper and
   amplitude, first failure `w`/`v`/reason, `current_anchor_valid`, query count,
   cell count, and split counters.  If the run lacks per-attempt fields, record
   that absence explicitly and derive only source-provable fields; do not invent
   Validator results.
2. Trace `CertifiedTubeBuilder::build()` and `TubeSurfaceValidator::validate()`
   to determine whether one invocation already exposes a current-connected
   certified/truncated interval or a reusable exact proof for the inward loop.
3. If an exact-current/zero-line failure is invariant across all attempts,
   compute that fact once per immutable build and reuse it only for the same
   snapshot/profile identity before producing the existing zero-only result.
4. If Validator output already contains a certified current-connected interval,
   let the Builder consume that exact output once; never interpolate or infer a
   ribbon from failed samples.
5. Otherwise merge only byte-for-byte identical exact Validator work.  Preserve
   fail-closed `INSUFFICIENT_CLEARANCE`, UNKNOWN, OOM, and query-terminal
   outcomes and report the smallest missing proof interface instead.
6. Add focused tests proving repeated invariant failure is not re-run 101 times,
   while true nonzero certification, UNKNOWN/OOM, query terminal, and Filter /
   Runtime semantics remain unchanged.  Record before/after attempt/query/
   split counts.

Candidate9's query terminal is separate: only same immutable snapshot and exact
query-key structural duplicates may be merged; no cache or Validator shortcut.

### D source classification and permitted edits

Classify the 101-attempt path into exactly one of these cases before product
editing:

- D0: the first failure is at the exact current/zero line and is independent of
  requested inward width.  One exact proof may terminate all narrower attempts.
- D1: the failure is outside zero and Validator already returns a continuously
  certified current-connected inward region.  Builder may consume that returned
  proof exactly once.
- D2: failures move with the inward boundary, but the existing Validator output
  does not certify a replacement region.  Do not invent one; retain fail-closed
  behavior and report the missing minimal proof interface.
- D3: candidate9 query explosion contains source-identical duplicated cells or
  queries within one invocation.  Only remove duplicate construction of the
  same proof obligation; do not add memoization or skip a distinct cover cell.

Preferred write scope for D:

- certified_tube_builder.cpp/.h and its focused test;
- tube_surface_validator.cpp/.h only if an already-computed certified result is
  inaccessible because of a narrow return-type defect;
- no TubeFilter, Runtime, adapter, manager, launch, parameter, or ROS schema
  changes.

D must preserve full-width-first semantics, zero-connectedness, exact current
anchor policy, Builder truncation provenance, Validator continuous cover, and
zero-only planner baseline.  It may delete the outer 99/101 whole-Validator
loop only when D0 or D1 proves the same safe result without it.

### D focused acceptance

- a D0 fixture performs one invariant proof, returns the same failure enum and
  zero-only result, and never enters 100 narrower validations;
- a D1 fixture consumes only Validator-certified current-connected bounds and
  produces the same or narrower safe ribbon;
- an outer-only obstacle with a genuinely safe inward ribbon still certifies;
- exact-current unsafe, UNKNOWN, OOM, query terminal, incomplete cover, and
  disconnected-current cases remain fail-closed;
- a safe full-width ribbon still certifies without inward work;
- no query/depth/attempt limit, tolerance, clearance, or margin changes;
- before/after evaluator/Validator invocation counts are asserted in tests,
  not inferred from wall-clock time.

## E — safe neutral liveness

### E root cause fixed by the primary agent

The existing source does not contain a production H2 return-to-center caller.
The only assignment of Runtime returning_to_center_ is a failed manual
preflight refresh.  H2 stage failure currently attempts neutral commit, which
correctly refuses live nonzero authority, then keeps the old frontend.  A later
exact-port denial does not mutate delta.  This is the observed G15 chain.

Historical H2 specifications explicitly left this owner to the later M6
certified recenter/recovery stage.  Therefore E is not allowed to expose the
boolean alone or pretend that makeManualRawPort u_w=0 freezes path progress.
E must implement the minimum M6 contract below using the existing old active
PathTubePair as the sole recenter authority.

1. Trace the existing point and closed-loop replan branches, Runtime denial,
   return-to-center, neutral-retirement predicate, and
   `commitNeutralPlannerFrontend()` using the G15 gen1/H2 failure timeline.
2. On Tube/Pair stage failure with active nonzero authority, revoke further
   nonzero continuation through the existing offset-denial/return-to-center
   semantics, without directly changing the planner frontend.
3. Once the existing neutral predicate is true, install the already accepted
   callback-local planner payload through the existing neutral commit path.  If
   that callback has ended, use only the next existing replan callback; add no
   cache, queue, timer, retry scheduler, or new state.
4. Prove this occurs before the old executable horizon ends.  Governor HOLD
   remains the final fail-closed result when no safe return or neutral commit
   exists.
5. Add focused tests for nonzero+Tube-fail refusal of direct neutral switching,
   return-to-zero command, neutral accepted install, atomic successful H2 Pair,
   genuine no-return HOLD, and equivalent point/closed-loop behavior.

### E exact lifecycle requirements

The existing active certified Pair remains the only authority while returning
to zero.  On replacement Tube failure:

1. deny extension of nonzero authority using the existing certificate-denial /
   return-to-center command path;
2. keep commands inside the old active Tube while delta converges toward zero;
3. prove from existing state that no active or pending nonzero Pair remains and
   the existing neutral tolerance/predicate is satisfied;
4. only then install the accepted neutral planner frontend through
   commitNeutralPlannerFrontend;
5. never carry a stale accepted payload across callbacks in new storage.  If
   the original callback has ended, the next ordinary accepted replan supplies
   the payload.

Preferred write scope for E:

- gvf_manager.cpp/.h and gvf_switch_policy_test;
- adapter only if invoking an existing return-to-center/denial operation
  requires a direct existing API correction;
- no new FSM state, latch, pending slot, deadline, wakeup, retry callback, or
  governor exception.

E focused evidence must include a deterministic state timeline:

    nonzero Pair active
    -> replacement Tube stage false
    -> existing return-to-center command active
    -> executed/retained delta converges to existing neutral tolerance
    -> nonzero authority retired by existing predicate
    -> accepted neutral frontend installed

Tests must also prove a nonzero direct switch remains refused and that an
actually impossible return still ends in the existing HOLD rather than being
reported successful.

### E authorized implementation -- minimum certified recenter

The certified geometric fact is already available: a valid normal active Tube
is connected to delta=0, and the retained delta is inside its current bounds.
The old active PathTubePair remains the only path/Tube owner until neutral
retirement.  E may move the reference toward zero only while every executed
step remains certified by that old Pair and the existing Runtime exact-PWL
witness.

Implement exactly these responsibilities:

1. Runtime request operation
   - add a small command-thread operation on PhaseOffsetRuntime which requests
     return-to-center using the existing returning_to_center_ and
     profile_completed_ lifecycle;
   - it must not change delta, previous_final_port, owner, Tube, preflight,
     limits, or safety status at request time;
   - expose only the minimum read-only state needed to know that certified
     return is active; do not add another state enum or flag.
2. Pair-serialized adapter bridge
   - add one adapter operation guarded by runtime_command_mutex_;
   - accept only the exact currently authoritative Pair pointer, generation,
     authority session, and still-executed nonzero Runtime authority;
   - reject stale callbacks, pending bootstrap, retired Pair, null owner/profile,
     or session drift with no mutation;
   - call the Runtime request operation only after the exact Pair check;
   - no ROS service/topic/message, mailbox, queue, timer, or retry is allowed.
3. H2 failure owner
   - in the existing point and closed accepted-replan branches, after the one
     H2 stage attempt returns false, request certified return for the exact Pair
     captured by that replan decision;
   - request at most once for that state transition; repeated ordinary replans
     while Runtime is already returning must not start another H2 transaction;
   - this suppression is existing authority-mode routing, not a new success
     gate: neutral commit still refuses until Runtime is truly neutral.
4. Recenter command
   - while returning, suppress the manual nonzero profile and set delta_ref=0;
   - after base guidance is known, form a recenter raw port whose requested
     phase rate cancels base_w_dot and whose delta rate uses the existing
     delta_tracking_gain toward zero;
   - pass that port through the existing PortProjector and continuous exact-PWL
     witness with the existing nonnegative-progress safety policy;
   - do not bypass rate limits, tangent/phase constraints, Tube bounds,
     crossed-knot checks, refresh-horizon witness, or map/current safety;
   - if the projector cannot immediately freeze phase because of existing rate
     limits, only its certified projected command may execute.  No direct w or
     delta assignment is allowed.
5. Neutral completion
   - update delta and previous port only through the normal selected complete
     path;
   - use the existing kOffsetAuthorityNeutralTolerance unchanged;
   - once selected execution reaches that existing tolerance, mark the existing
     return lifecycle complete and clear returning_to_center_;
   - do not use kProfileCompletionDeltaTolerance as a new handoff tolerance and
     do not snap a materially nonzero delta to zero;
   - hasExecutedOffsetAuthority and the existing neutral-retirement predicate
     remain the sole authority release proof.
6. Planner installation
   - no planner frontend changes during nonzero return;
   - after neutral retirement succeeds, the next ordinary accepted replan
     callback installs through the existing commitNeutralPlannerFrontend;
   - do not store the preceding callback's accepted frontend in new state.

### E fail-closed boundaries

- If the old active Pair is no longer current-valid, the retained delta is
  outside its certified bounds, latest categorical evidence is explicitly
  unsafe, or the exact-PWL recenter witness is unavailable, no recenter step may
  mutate Runtime.  The existing CERTIFICATE_DENIED/WAITING/HOLD result remains.
- If existing rate limits cannot stop phase before old owner exhaustion, the
  dynamic stage is NOT_FIXED.  Do not change rates, horizon, replan period,
  governor, or Tube length to make it pass.
- Never create a zero-only Tube as recenter authority and never bind the old
  Tube to the accepted new planner path.

### E file scope

Only the following product files may be changed for E:

- phase_offset_runtime.h/.cpp and runtime_test.cpp;
- phase_offset_matched_adapter.h/.cpp and phase_offset_matched_adapter_test.cpp;
- gvf_manager.h/.cpp and gvf_switch_policy_test.cpp.

TubeBuilder, TubeFilter, SurfaceValidator, TubeEpochManager, planner/C2 math,
governor, launch, parameters, CMake, ROS schema, and map code are forbidden.

### E required focused tests

Runtime tests:

- request at a selected nonzero state changes no delta/previous port immediately;
- next valid old-Pair command requests zero phase progress and delta convergence,
  then executes only the projected exact-PWL-certified result;
- repeated valid commands monotonically reduce absolute delta and do not
  advance beyond the certified projected phase behavior;
- neutral tolerance completion clears existing return state and makes
  hasExecutedOffsetAuthority false;
- explicit unsafe/current invalid/witness unavailable leaves Runtime unchanged
  and returns the original fail-closed mode.

Adapter tests:

- exact live Pair/session request succeeds once;
- stale pointer/generation/session, pending activation, retired Pair, and
  already-neutral request are no-op failures;
- returning command continues to use the old Pair's path/profile/epoch only;
- no new H2 Pair is staged while certified return is active.

Manager tests:

- accepted replan plus nonzero H2 false requests return but does not install the
  neutral frontend in that callback;
- point and closed branches behave identically;
- after Runtime neutral retirement, a later ordinary accepted replan installs
  through commitNeutralPlannerFrontend;
- successful H2 still commits atomically without entering return;
- impossible certified return retains the existing HOLD outcome.

## E-R1 mandatory safety correction after independent review

The first E implementation is not dynamically acceptable.  Independent
read-only review and the G1/G2-reached, G3-timeout run proved that it can keep
positive phase progress during return and that return is not serialized against
pending H2/timer Pair replacement.  E-R1 supersedes any earlier conflicting E
wording and must be completed before another dynamic acceptance run.

### E-R1.1 Authority lifecycle at zero crossings

- PhaseOffsetRuntime::requestReturnToCenter must set only the existing
  returning_to_center_ state.
- It must not set profile_completed_ at request time.  A started manual profile
  at an instantaneous delta zero crossing therefore continues to hold executed
  authority until one selected certified return command completes the existing
  neutral lifecycle.
- profile_completed_=true and returning_to_center_=false are written only after
  a selected complete step observes the existing
  kOffsetAuthorityNeutralTolerance.  Do not change that tolerance or snap a
  materially nonzero delta.
- Request remains lifecycle-only: no delta, previous port, Pair, epoch,
  preflight, or owner mutation.

### E-R1.2 Recenter witness priority

- When the prepared command is returning, do not call the ordinary U+ witness.
- Call the existing nonnegative-progress U_safe exact-PWL witness directly with
  the recenter raw port u_w=-base_w_dot and
  u_delta=-delta_tracking_gain*delta.
- The existing PortProjector rate, phase/tangent, regularity, Tube-bound,
  crossed-knot, and future-step checks remain unchanged.
- A projected positive phase rate caused by an existing rate bound is allowed
  only when it is the exact certified U_safe result.  Tests must use production
  rate limits and prove phase progress reaches zero before the old owner end in
  the passing fixture.
- U_safe witness failure returns the original CERTIFICATE_DENIED result and
  mutates no retained Runtime state.

### E-R1.3 Exact old-Pair exclusivity without new state

Use only the existing returning_to_center_ state and existing locks.  Do not
add a lifecycle token, generation, gate, mailbox, queue, or retry.

Under runtime_command_mutex_ and the existing pin lock order:

- requestReturnToCenter on the adapter rejects an active PathTubePairPin lease;
  a busy/pending H2 transaction is not a failed successor and must not trigger
  recenter;
- captureAndAcquirePathTubePairPin rejects while Runtime is returning;
- stagePathTubePair entry and its Runtime snapshot reject returning;
- preparePathTubePairCommit rejects returning;
- commit/final CAS rechecks returning under the same command lock immediately
  before replacing the authoritative Pair;
- timer Pair refresh prepare and final CAS perform the same returning checks;
- a transaction prepared before the request must fail final CAS without
  side-effects when request wins the lock;
- if Pair replacement wins the lock first, a request carrying the old exact
  Pair must fail without changing Runtime.  The new Pair then remains the valid
  continuation.

No old Pair pointer is rebound to a new planner owner.  Candidate/timer evidence
may remain observable, but it cannot replace the active Pair during return.

### E-R1.4 Manager routing and completed frontend

- capturePathTubeReplanHandoffRequirement must expose or separately read the
  existing returning state once for the decision.
- While returning, point and closed branches do not start a new H2 transaction.
  They may build an ordinary accepted planner payload and call the existing
  neutral commit; that commit must continue to refuse until neutral retirement.
- A false stage result caused by an already-active pin/lease must not request
  return.  The adapter exact request operation is the final serialized owner
  and rejects that case.
- Do not perform the adapter return request directly from the point/closed
  branch.  Add a manager helper with no stored state which acquires the existing
  locks in the established order frontend_apply_mutex_ then
  path_tube_handoff_mutex_, rejects pending/completed handoff slots, and only
  then calls the adapter exact-Pair request under its existing Runtime lock.
  consumeCompletedPathTubeHandoff, neutral commit, and return request therefore
  share one linearization boundary.  If consume wins, the stale old-Pair
  request fails; if return wins, consume observes returning and cannot install.
- prepareAndCommitPendingPathTubeHandoff and consumeCompletedPathTubeHandoff
  must not install a replacement frontend after return has won serialization.
  Prefer making the adapter final CAS rejection authoritative; add a manager
  defensive no-install check using the same existing returning state, without
  creating another lifecycle flag.
- A pending or completed mailbox observed defensively after returning is true
  is stale and must be removed with its pin released outside manager locks.  It
  must not remain RETRY_PENDING and later install after returning becomes false.
  After neutral completion, only a new ordinary accepted replan may install
  through commitNeutralPlannerFrontend.
- Once selected return completes and neutral authority retirement succeeds, a
  later ordinary accepted replan installs through commitNeutralPlannerFrontend.

### E-R1.5 Required race and production-flow tests

Runtime:

- request at a profile zero crossing retains executed authority until selected
  neutral completion;
- returning bypasses U+ and directly exercises U_safe;
- production rate limits show certified phase deceleration/freeze and eventual
  delta convergence;
- existing u_delta slew limits may cause a finite zero crossing or a temporary
  increase in absolute delta when the request arrives with a large previous
  port.  This is not failure if every selected step remains inside the old
  certified Tube, uses the exact U_safe witness, stops consuming owner phase,
  and reaches the unchanged neutral tolerance within a bounded test horizon;
- do not add a new monotonic-delta projector constraint, change rate limits, or
  rewrite previous_final_port merely to satisfy the test;
- genuine U_safe joint-polygon failure returns CERTIFICATE_DENIED with identical
  delta/previous port/profile lifecycle.

Adapter:

- active pin makes request fail with no mutation;
- request before H2 final CAS makes CAS fail with no Pair/generation/session
  change;
- H2 CAS before request makes stale old-Pair request fail;
- timer refresh prepared before request fails final CAS;
- returning rejects new stage/prepare/refresh while candidate evidence remains
  displayable;
- selected neutral completion permits existing retirement exactly once.

Manager:

- real point and closed stage-false flow requests return only after the stage's
  own lease is released;
- pin-busy false does not request return;
- returning suppresses new H2 and refuses neutral install while nonzero;
- pending/completed frontend cannot install after return serialization;
- pending/completed frontend present in a defensive returning-state fixture is
  dropped and cannot revive after neutral completion;
- a deterministic hook between consume's returning observation and mirror
  application proves the manager-serialized return request cannot enter that
  window; the opposite ordering proves consume wins and stale request fails;
- later neutral ordinary replan installs;
- successful concurrent Pair replacement wins cleanly and prevents stale
  recenter request.

All earlier E tests that directly write private flags remain supplemental and
cannot substitute for these production-flow/race tests.

## Verification and dynamic boundary

Run D/E focused tests, related regressions, `catkin_make -j2`, and
`git diff --check`.  Then use a fresh private split launch with the existing
five-goal harness.  Report candidate geometry, certified success/attempt/query
counts, and each G1–G5 outcome honestly.  The only acceptable final labels are
`PARTIALLY_FIXED` or a more specific blocked/partial substatus; never `FIXED`
unless all five goals and both D/E owners pass dynamically.

Run static verification in this order:

1. CertifiedTubeBuilder and SurfaceValidator focused tests for D;
2. marker 13 and adapter 83 regression tests;
3. GVF switch-policy and continuous-path tests for E;
4. EpochManager and Runtime regressions;
5. required focused CTest set;
6. full catkin_make -j2;
7. git diff --check and source/binary hash comparison.

Dynamic acceptance uses a fresh private ROS master and the user's unchanged,
separate simulator.launch and test_gvf.launch commands.  Start recorder before
publishers, send the known five goals in order, and clean only task-owned PIDs.

Required dynamic facts:

- candidate geometry remains visible and certified never displays uncertified
  geometry;
- at least one real nonzero certified Pair commits and executes;
- the repeated-failure cohort no longer performs 101 equivalent validations if
  D0/D1 was source-proven;
- query terminal remains fail-closed and is not hidden;
- after a nonzero H2 failure, return-to-center begins before the old path end;
- neutral frontend installs only after the existing neutral predicate is true;
- G1 through G5 all reach for FIXED status;
- no stable all_candidates_path_end_clamped HOLD occurs when a safe return and
  accepted neutral continuation exist.

If certified Tube remains zero-ADD, G2 or any later goal times out, or the old
HOLD chain remains, label the result PARTIALLY_FIXED or NOT_FIXED and retain the
shortest direct evidence.  Candidate visibility alone is not D success;
navigation survival alone is not certified-construction success.
