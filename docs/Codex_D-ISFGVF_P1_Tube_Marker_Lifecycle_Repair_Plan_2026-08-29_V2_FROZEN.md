# D-ISFGVF P1 bounded Tube marker-lifecycle repair plan V2

    DOCUMENT_ROLE=BOUNDED_REPAIR_EXECUTION_PLAN
    DOCUMENT_STATUS=V2_FROZEN_AFTER_FRESH_SOL_XHIGH_ACCEPTANCE
    DATE=2026-08-29
    PRODUCTION_BASELINE=0fb998966bab78f8e555aca49e1b4189519462be
    ROOT_CAUSE=PATH_FRAME_OWNER_REPLACEMENT_WITHOUT_OLD_MARKER_RETIREMENT
    PREVIOUS_SOL_VERDICT=P1_TUBE_MARKER_LIFECYCLE_REPAIR_PLAN_NOT_ACCEPTABLE
    HALF_VOXEL_REVERT=false
    STAGE1A_CURRENTNESS_CHANGED=false
    BATCH_C_D=PAUSED

## 1. Purpose and revision boundary

This V2 plan supersedes the stopped candidate only for its three audited
blockers:

1. owner replacement was not serialized through actual relevant Marker
   publication;
2. the only DELETE obligation could be discarded or deferred by authority or
   lifecycle transitions;
3. deterministic tests did not cover either unsafe interleaving or a delayed
   stale retirement.

The accepted root cause, Stage 1A currentness, R3 eligibility, marker geometry,
marker lifetime, Tube mathematics, Runtime, Recovery, H2, and Batch C/D are not
reopened.

This planning turn changes no production or test source, launches no Luna or
ROS process, and performs no stage, commit, or push.

## 2. Fixed source facts

The implementation shall rely only on these existing facts:

- `PhaseOffsetMatchedAdapter::finalizeTubeEpoch()` already takes
  `task_publication_mutex_` before `runtime_command_mutex_`, performs the final
  `requestSourceStillCurrent()` check, and retains the publication mutex while
  calling `publishManual()`.
- ordinary MANUAL `update()` currently takes only `runtime_command_mutex_`, so
  it can replace an owner after finalization releases the runtime mutex but
  before `publishManual()` completes its ROS publications;
- `resetForNewNavigationTask()` already takes
  `task_publication_mutex_ -> runtime_command_mutex_` and emits a broader full
  MANUAL DELETE;
- `deactivate()` and `retirePathTubeAuthority()` currently do not share the
  publication boundary;
- `gvf_manager::phaseOffsetTubeTimerCallback()` calls
  `PhaseOffsetMatchedAdapter::scheduleTubeBuild()` every configured Tube timer
  period, including when the latest build request is empty or inactive;
- all three relevant marker topics use stable namespaces/IDs and zero lifetime.

## 3. Frozen V2 publication linearization boundary

### 3.1 One existing mutex is the marker-authority barrier

No new mutex is authorized. `task_publication_mutex_` becomes the sole
linearization barrier for both:

- any operation that changes or retires the owner permitted to publish the
  three relevant Tube marker topics; and
- the final eligibility recheck plus every actual ADD/DELETE publication on
  those topics.

The frozen nested lock order remains:

    task_publication_mutex_
      -> runtime_command_mutex_
        -> worker_state_mutex_ (only where already required)

No path may take `task_publication_mutex_` while already holding
`runtime_command_mutex_`.

### 3.2 Ordinary owner/source/frame replacement

For MANUAL mode, `PhaseOffsetMatchedAdapter::update()` shall acquire
`task_publication_mutex_` before its existing `runtime_command_mutex_` lock.
It retains the publication lock through the complete command update. ACTIVE
mode need not take this Marker-specific barrier.

For an ordinary same-task source/path/frame/semantic-owner/frame-owner
replacement, while both locks are held, the update shall:

1. construct the immutable replacement request;
2. queue the durable retirement obligation for the currently published marker
   owner, if one exists;
3. preserve the existing Candidate/Epoch clears;
4. atomically store the replacement in `latest_build_request_`;
5. complete the current control snapshot update;
6. release `runtime_command_mutex_`, then `task_publication_mutex_`.

The replacement linearization point is the store of the replacement
`latest_build_request_` while the durable obligation already exists and both
locks are held.

The command thread publishes no ROS Marker message at this ordinary
replacement seam.

### 3.3 Completion publication

`finalizeTubeEpoch()` keeps its existing outer `task_publication_mutex_` from
function entry through the return of `publishManual()`.

Its final eligibility validation remains the unchanged
`requestSourceStillCurrent()` call while it also holds
`runtime_command_mutex_`. After that validation, it may release only the
runtime mutex. It must retain `task_publication_mutex_` through:

- Candidate/Epoch exposure;
- any matching retirement DELETE required before the new owner ADD;
- construction of the relevant marker decision;
- the actual ROS publication calls for `/tube_candidate`, `/tube`, and
  `/tube_certified_geometry`.

Because every MANUAL owner replacement must first acquire the same publication
mutex, replacement cannot linearize between final eligibility and actual
publication.

The legal ordering is therefore exactly one of:

    B final eligibility -> B relevant publications -> release -> C replacement

or:

    C replacement -> release -> B final currentness rejection

The forbidden ordering:

    B final eligibility -> C replacement -> stale B ADD

cannot acquire the required locks.

### 3.4 Other owner-loss paths

`deactivate()`, `retirePathTubeAuthority()`, and
`retirePathTubeAuthorityIfNeutral()` shall use the same
`task_publication_mutex_ -> runtime_command_mutex_` order through their owner
retirement/request invalidation decision.

`resetForNewNavigationTask()` already has the required order and retains it.
`requestShutdown()` already owns the publication mutex and retains it through
the stronger shutdown DELETE described below.

This serialization changes no Stage 1A predicate and does not accept an old
worker completion.

## 4. Durable marker-retirement obligation

### 4.1 Minimal state

Add one adapter-local marker lifecycle ledger, protected exclusively by
`task_publication_mutex_`:

    CURRENT_PUBLISHED_MARKER_OWNER=
      valid
      publication_generation
      TubeWorkIdentity
      publication_stamp

    PENDING_MARKER_RETIREMENT=
      valid
      target_publication_generation
      target_TubeWorkIdentity
      retirement_stamp

`publication_generation` is adapter-local, monotonic, nonzero, and advances
only after an actual relevant ADD publication. The identity is the existing
`TubeWorkIdentity`; no second source-revision model is introduced.

This is not a naked pending boolean: the obligation names the exact actual
publication generation and immutable owner it must retire. It stores no marker
geometry, Tube profile, map, callback, worker state, or Runtime state.

### 4.2 Creation

While holding `task_publication_mutex_`, an owner-loss operation creates an
obligation before it invalidates the owner/request if
`CURRENT_PUBLISHED_MARKER_OWNER` is valid.

Creation applies to:

- ordinary same-task source/path/frame/owner replacement;
- `deactivateLocked()` before installing the inactive request;
- `retirePathTubeAuthorityLocked()` before clearing the request/authority.

If an obligation already targets the same current publication generation,
another replacement or retirement leaves it intact. B-to-C coalescing cannot
overwrite an A-publication obligation with an identity that was never
published.

Same-owner command or map refreshes create no obligation.

### 4.3 Exact consumer and publication helper

Add adapter-local helpers in `phase_offset_matched_adapter.cpp`; callers must
already hold `task_publication_mutex_`.

The retirement consumer performs one indivisible publication transaction:

1. recheck that the pending target generation and `TubeWorkIdentity` exactly
   equal `CURRENT_PUBLISHED_MARKER_OWNER`;
2. construct the existing all-DELETE bundles for IDs 0/1/2 on only
   `/tube_candidate`, `/tube`, and `/tube_certified_geometry`;
3. perform all three ROS DELETE publications without releasing the mutex;
4. invalidate the current published-owner record;
5. consume the matching obligation.

It may not copy the obligation, release the mutex, and publish later. A target
mismatch must never publish DELETE; it is a fail-closed lifecycle invariant
failure and must not mutate a newer marker owner.

All actual publication sites for the three relevant topics shall be routed
through a small adapter-local lifecycle-aware publication helper. Marker
arrays continue to be built by the existing marker builders. Base-path,
active-path, frame, diagnostics, topic names, namespaces, IDs, colors, and
geometry remain unchanged.

Before that helper publishes any ADD for an owner different from a pending
retired owner, it consumes the matching retirement transaction under the same
mutex. It then publishes the current bundle and records a new nonzero
`publication_generation` with the exact current `TubeWorkIdentity` only after
the actual ADD calls. An all-DELETE bundle clears the current owner and
consumes a matching obligation.

R3 eligibility is computed exactly as before. The helper observes only the
resulting R3 ADD/DELETE action and records its current request owner; it does
not modify Candidate/request eligibility or introduce fallback.

### 4.4 Timer-owned bounded consumption

The normal no-completion retirement owner remains:

    gvf_manager::phaseOffsetTubeTimerCallback()
      -> PhaseOffsetMatchedAdapter::scheduleTubeBuild()
      -> consume matching marker-retirement obligation

`scheduleTubeBuild()` shall acquire `task_publication_mutex_`, consume a
matching obligation before loading/rejecting/scheduling any worker request,
and release the publication mutex before taking `worker_state_mutex_` for the
ordinary latest-only scheduling decision.

Consumption does not require a current, active, or non-null build request and
does not build Tube geometry. Therefore authority retirement or deactivation
cannot make the DELETE wait behind an inflight worker or a replacement build.

### 4.5 Lifecycle supersession rules

An obligation may be removed only by an actual matching-or-broader DELETE
while `task_publication_mutex_` remains held:

- timer/scheduler retirement emits the three-topic all-DELETE transaction;
- a current replacement completion emits that transaction immediately before
  its first ADD;
- inactive processing may emit the same or broader MANUAL DELETE and consume
  the obligation;
- `resetForNewNavigationTask()` emits its existing broader MANUAL DELETE and
  consumes every relevant obligation before clearing lifecycle state;
- `requestShutdown()` emits the same broader MANUAL DELETE, when advertised
  marker state or an obligation exists, before setting the shutdown flag and
  before publishers are released.

The following must never clear or overwrite the ledger without such a DELETE:

- `retirePathTubeAuthorityLocked()`;
- `deactivateLocked()`;
- ordinary owner replacement;
- `consumeTimerTaskGeneration()`;
- timer cache/manager cleanup;
- worker-slot invalidation;
- `shutdown()` state cleanup after `requestShutdown()`.

No lifecycle cleanup is allowed to infer that clearing an in-memory Candidate
also clears lifetime-zero RViz marker state.

## 5. Safety proofs for the three audited blockers

### 5.1 No stale ADD after replacement

Completion publication and owner replacement both require
`task_publication_mutex_`. If completion owns it first, every relevant ADD is
finished before replacement linearizes. If replacement owns it first, the
unchanged currentness gate observes the new request and rejects the old
completion. There is no unlocked interval between the final identity decision
and the ROS ADD.

### 5.2 No retirement loss

The obligation targets an actual publication generation rather than the
latest request. Replacement, authority retirement, deactivation, timer reset,
and worker cleanup cannot erase it. The next Tube timer consumes it before any
worker decision. Reset and shutdown can consume it only by publishing a
matching-or-broader DELETE under the same publication mutex.

Consequently:

    RETIREMENT_LATENCY
      <= one configured tube_update_period
         + normal callback scheduling/lock-acquisition jitter

Successful construction or worker completion is not part of this bound.

### 5.3 No stale DELETE of a newer owner

A consumer holds `task_publication_mutex_` continuously from target recheck
through the actual DELETE and ledger update. A newer ADD cannot interleave.
Conversely, every newer ADD first consumes a matching older obligation and
then records its new publication generation before releasing the mutex. A
delayed timer/retirement attempt later finds no matching pending target and
cannot DELETE the newer publication.

## 6. Topic classification

    /formation_planning/phase_offset_manual/tube_candidate=NEEDS_REPAIR
    /formation_planning/phase_offset_manual/tube=NEEDS_REPAIR
    /formation_planning/phase_offset_manual/tube_certified_geometry=NEEDS_REPAIR

All three share the same stable-ID, zero-lifetime publication namespace and
must pass through the V2 publication/retirement boundary. Their construction
and eligibility semantics remain unchanged. In particular, R3 retains stable
current-request authority, Candidate/request identity checks, immutable
provenance, all-ADD/all-DELETE bundles, and no Pair/Active fallback.

## 7. Exact implementation whitelist

Future implementation may modify exactly:

1. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
2. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
3. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

The header is authorized only for the minimal publication ledger, private
locked helper declarations, and bounded passive test hooks needed by Tests
A-C.

No fourth implementation or test path is authorized.

## 8. Protected scope

The following remain unchanged:

- `phase_offset_tube_markers.cpp` and its header;
- `gvf_manager.cpp` and timer wiring;
- `tube_surface_validator.cpp` and all half-voxel mathematics/tests;
- TubeBuilder, TubeFilter, CertifiedTubeBuilder, SurfaceValidator;
- `requestSourceStillCurrent()` and `epochMatchesRequest()` predicates;
- Stage 1A worker/latest-only scheduling/currentness architecture;
- R3 eligibility, provenance, request/Candidate/request protocol, and
  no-fallback rule;
- Runtime, Recovery, Handoff, H2, C2, planner, map, launch/config files;
- marker lifetime, namespace, IDs, colors, and geometry;
- Batch C/D.

No command-side ordinary replacement ROS publication, new mutex, timer,
worker, FSM, retry, cache architecture, topic, or marker lifetime is allowed.

## 9. Exactly three deterministic concurrency regressions

Use only existing bounded condition-variable/test-hook patterns. Every wait is
finite and every thread is joined. No new concurrency framework is authorized.

### TEST A — replacement versus stale ADD

Establish current B and pause B completion at the last point immediately before
the actual relevant Marker publication while it owns
`task_publication_mutex_`. Start C owner replacement and prove it is blocked at
the same mutex. Release B and assert:

    B ADD fully publishes
      -> B releases publication barrier
      -> C replacement linearizes and queues B retirement

Then execute the complementary C-first ordering by pausing C inside the
publication barrier after its replacement store, starting B finalization, and
releasing C. Assert B fails unchanged currentness and publishes no ADD.

There must be no trace:

    C replacement -> stale B ADD

### TEST B — durable retirement across lifecycle transition

Establish `A_ADD`, replace A with B, and verify the obligation targets A's
actual publication generation. Before any B completion, run bounded subcases
through the same test body for:

- authority retirement;
- deactivation while an old worker occupies the worker path;
- task reset;
- shutdown publication retirement.

For authority retirement and deactivation, invoke the next scheduler decision
and assert the three-topic DELETE occurs without waiting for worker completion.
For task reset and shutdown, assert the synchronous existing-or-stronger
DELETE occurs before ledger cleanup. In every subcase the obligation may
disappear only with observed DELETE of A's namespace/IDs.

### TEST C — stale retirement cannot delete a newer owner

Create an A retirement obligation. Pause a timer retirement attempt before it
acquires `task_publication_mutex_`. Complete current B: B's publication path
must first emit A DELETE, consume A's obligation, then publish B ADD and record
B's newer publication generation. Release the delayed retirement attempt and
assert it emits no DELETE, B remains the current visible owner, and the marker
trace is exactly:

    A_ADD -> A_DELETE -> B_ADD

Never:

    B_ADD -> stale_A_DELETE

No other new concurrency regression is authorized. Existing Stage 1A, R3,
lifecycle, Runtime, Recovery, and Validator regressions continue unchanged.

## 10. Future implementation review and verification

Main shall verify only:

1. owner replacement and all three relevant actual publications share
   `task_publication_mutex_` continuously through the defined boundaries;
2. the identity/generation obligation survives until observed DELETE;
3. a delayed mismatched retirement emits no DELETE;
4. changed paths are exactly the three whitelist paths.

The future isolated-worktree build/test protocol remains:

```bash
source devel/setup.bash
catkin_make -j2
catkin_make run_tests_phase_offset_core
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_bspline_race
devel/lib/bspline_race/phase_offset_matched_adapter_test
```

The existing five-goal real ROS acceptance remains unchanged except that the
required marker trace is now explicitly:

    OLD ADD -> DELETE within one Tube period plus jitter -> NEW ADD

or `OLD ADD -> DELETE` when no replacement completes. No old hash may reappear
and no delayed retirement may remove a newer hash.

## 11. Stop conditions

Stop without redesign if implementation requires:

- a new mutex or a fourth changed path;
- command-thread ordinary replacement publication;
- releasing `task_publication_mutex_` between final identity validation and
  actual relevant publication;
- clearing an obligation without matching-or-broader DELETE;
- worker completion for the latency bound;
- stale completion acceptance or a currentness predicate change;
- R3 eligibility/fallback, marker construction/lifetime, safety mathematics,
  Runtime, Recovery, H2, or Batch C/D changes.

## 12. V2 frozen review status

The existing publication barrier can serialize both sides without a new mutex.
The generation-qualified obligation is independent of the request/worker slot
and has an actual-DELETE-only consumption rule. The delayed-retirement path
cannot retain a target across a newer ADD.

    MAIN_PLAN_REVIEW=PASS
    SOL_Q1=PROVEN_SAFE
    SOL_Q2=PROVEN_SAFE
    SOL_Q3=PROVEN_SAFE
    SOL_PLAN_VERDICT=P1_TUBE_MARKER_LIFECYCLE_REPAIR_PLAN_V2_ACCEPTABLE
    DOCUMENT_STATUS=V2_FROZEN_AFTER_FRESH_SOL_XHIGH_ACCEPTANCE

The independent audit was deliberately limited to the three V1 blockers. It
found the replacement/publication boundary, durable DELETE obligation, and
generation-qualified stale-retirement rejection proven safe. No implementation
is authorized by this planning turn.

    P1_TUBE_MARKER_LIFECYCLE_REPAIR_PLAN_V2_FROZEN
