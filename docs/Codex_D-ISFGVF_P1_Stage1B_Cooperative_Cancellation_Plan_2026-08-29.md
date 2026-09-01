# D-ISFGVF P1 Stage1B obsolete-build cooperative-cancellation plan

    DOCUMENT_ROLE=BOUNDED_STAGE1B_PLAN
    DOCUMENT_STATUS=REJECTED_AFTER_FRESH_SOL_XHIGH_AUDIT
    DATE=2026-08-29
    IMPLEMENTATION_AUTHORIZED=false
    LUNA_AUTHORIZED=false
    ROS_RERUN_AUTHORIZED=false
    COMMIT_AUTHORIZED=false
    PUSH_AUTHORIZED=false

## 1. Accepted input and fixed diagnosis

This plan starts from the accepted V2 marker-lifecycle checkpoint in:

    /tmp/d_isfgvf_p1_tube_marker_lifecycle_v2_20260829

The production base of that isolated checkpoint is:

    0fb998966bab78f8e555aca49e1b4189519462be

The accepted real-ROS evidence is not reopened:

    REAL_ROS_LIFECYCLE_CONTRACT=PASS
    REAL_ROS_VISUAL_AVAILABILITY=FAIL
    VISIBILITY_DUTY_CYCLE=0.129835
    DELETE_TO_NEXT_ADD_GAP_MEDIAN_MS=5081.410
    DELETE_TO_NEXT_ADD_GAP_MAX_MS=13536.758
    PATH_REVISION_INTERVAL_MEDIAN_MS=1035.666
    WORKER_BUILD_LATENCY_MEDIAN_MS=78.646
    STALE_INFLIGHT_BLOCKING_TIME_MEDIAN_MS=2999.463
    CURRENT_COMPLETION_RATIO=0.595238
    FIRST_LIMITING_SEAM=STALE_INFLIGHT_WORK_BLOCKS_LATEST
    STAGE1A_NO_CANCELLATION_INVOLVED=true
    STAGE1B_CANCELLATION_NOW_JUSTIFIED=true
    TEST_ONLY_REPAIR_PRIORITY=DEFERRED

Stage1B changes only how an already-obsolete worker computation exits. It does
not change the V2 rule that an old visible marker is retired promptly and is
never retained on an authoritative/current topic to conceal a gap.

## 2. Goal and preserved architecture

When request A is running and a genuinely superseding request B replaces its
Tube work identity, A shall stop at the next safe cooperative checkpoint,
publish and commit nothing, and release the sole worker so the existing newest
pending request can run.

The following remain unchanged:

- one joined worker;
- at most one inflight build;
- one latest-only pending slot, no FIFO;
- Tube-timer permits as the only production scheduling permits;
- immutable `TubeBuildRequest` handoff;
- job-local transactional Tube state;
- `requestSourceStillCurrent()` as the unchanged final fail-closed gate;
- V2 marker publication/retirement linearization and ledger;
- R3 publication eligibility and no-fallback rule;
- half-voxel tightening and every Tube/Validator predicate;
- Runtime, Recovery, Handoff, H2, C2, planner, map, and control semantics.

No second worker, pool, timer, retry, historical queue, stale-marker fallback,
or speculative parallel Tube build is introduced.

## 3. Source facts controlling the design

The current worker already owns a `SchedulePermit` containing an immutable
`TubeWorkIdentity`. That identity includes:

    task_generation
    active state
    source_revision
    path_revision
    frame_revision
    authority_session
    semantic_path_owner pointer
    frame_owner pointer
    base_path_tube_pair pointer/generation
    shutdown invalidation

It intentionally excludes ordinary `control_sequence`, command refreshes, and
`map_observation_sequence`. Therefore same-source command/map refreshes remain
sample-and-held and must not cancel a valid inflight build.

The expensive construction path already crosses adapter-owned abstract query
boundaries:

    PathStateQuery
    PathCellBoundQuery
    ClearanceQuery
    RawOccupancyQuery (diagnostic tail only)

These callbacks cover adaptive raw sampling, certified-cell construction,
cross-section clearance work, recursive SurfaceValidator cells/queries, and
the diagnostic tail. `TubeFilter` is a finite in-memory pass over the already
bounded sample set and contains no external query loop.

The long stale real-ROS attempts contained approximately 139477 to 279784
Tube-construction queries and lasted approximately 1.958 to 4.063 seconds.
Their observed duration/query ratios were 0.01334 to 0.01455 ms/query, with a
median of 0.01423 ms/query. The multi-second delay is therefore accumulated
query work, not one indivisible mathematical primitive.

## 4. Frozen cancellation identity predicate

Add one private, lock-free adapter predicate conceptually equivalent to:

    obsolete(permit):
      shutdown requested
        OR task_generation no longer equals permit.work_identity
        OR authority_session no longer equals permit.work_identity
        OR latest_build_request is null
        OR makeTubeWorkIdentity(latest_build_request)
             != permit.work_identity

The comparison uses the existing `TubeWorkIdentity`; no new revision or
currentness model is allowed.

Consequences:

- B with a changed path/source/frame/owner/pair/task/lifecycle identity cancels
  A;
- deactivate, reset, authority retirement, and shutdown invalidate A;
- a new immutable request pointer carrying the same Tube work identity does
  not cancel A;
- a newer map snapshot or control observation alone does not cancel A;
- the predicate never takes `task_publication_mutex_`,
  `runtime_command_mutex_`, or `worker_state_mutex_`.

The predicate is only an early obsolete-work test. It does not replace or
weaken `requestSourceStillCurrent()`.

## 5. Cooperative unwinding mechanism

Use one adapter-private cancellation sentinel type in the `.cpp` anonymous
namespace. The sentinel is not a ROS/public navigation type and is not a
validation reason.

For one worker job, wrap the existing adapter-created `PathStateQuery`,
`PathCellBoundQuery`, `ClearanceQuery`, and diagnostic `RawOccupancyQuery`.
Each wrapper performs the identity predicate immediately before delegating to
the unchanged underlying query. If obsolete, it throws only the private
cancellation sentinel. Otherwise it returns the underlying result unchanged.

The outer adapter build boundary catches only that sentinel and reports the
private execution outcome:

    COMPLETED_BUILD_PATH
    CANCELLED_OBSOLETE

All other Builder/Manager status and return values retain their current
meaning. In particular, the sentinel is never converted to:

    validator rejection
    invalid geometry
    unavailable/unknown map
    incomplete Candidate
    current successful completion

The sentinel cannot escape the worker thread entry. Existing C++ exception
support is already required by the adapter's `std::bad_alloc` handling. The
heavy path holds none of the worker/publication/Runtime mutexes, and all local
objects are RAII values, so unwinding abandons only job-local state.

## 6. Exact checkpoints

Poll at these bounded seams only:

1. immediately before preparing/copying job-local build state;
2. before and after `collectSamples()`;
3. before and after owner-aligned preview construction;
4. before every invocation of the four wrapped abstract query callbacks;
5. immediately after `TubeEpochManager::update()` returns;
6. before and after the raw/cloud diagnostic tail;
7. once immediately before handing a completed local snapshot to finalization.

No cancellation checks are added inside geometry algebra, TubeFilter arithmetic,
marker construction, Runtime, or every mathematical primitive.

The query wrappers give the needed fine granularity without modifying
`TubeBuilder`, `TubeFilter`, `CertifiedTubeBuilder`, `TubeEpochManager`,
`TubeSurfaceValidator`, `ValidationContext`, or `ValidateCell()`: every costly
recursive/sample path reaches an existing wrapped callback. The only
non-query interval is one bounded in-memory filter/assembly phase.

## 7. Worker outcome and transaction rules

`runTubeBuildJob()` shall handle the two private outcomes as follows.

For `CANCELLED_OBSOLETE`:

- do not call `finalizeTubeEpoch()`;
- do not call `publishBuildDiagnostics()`;
- do not publish Candidate, Epoch, marker, raw, or cloud authority/evidence;
- do not commit `TubeJobLocalState`;
- do not mutate the V2 marker ledger;
- record only attempt-local timing/cancellation evidence;
- return to `tubeWorkerMain()`, which clears the existing running slot and
  immediately consumes the already-populated latest-only pending slot, if any.

If no pending request exists yet, the worker remains idle until the next
existing Tube-timer permit. The command path still does not directly schedule
work.

For `COMPLETED_BUILD_PATH`, preserve the present behavior exactly, including
finalization of an incomplete but current Candidate for existing diagnostic
semantics and the final `requestSourceStillCurrent()` gate.

## 8. Distinguishable cancellation evidence

Extend only the existing adapter-local tube-due measurement row with one
boolean field:

    cancelled_obsolete

A cancelled attempt is recorded with:

    source_current_finalized=0
    cancelled_obsolete=1

An ordinary current or ordinary rejected/stale completion uses:

    cancelled_obsolete=0

This field is observational only. It changes no control, certification,
publication, scheduler, or worker decision. The existing timing row remains
attempt-only evidence.

## 9. Safety proof obligations

### 9.1 No publication or partial commit

The sentinel is caught before `finalizeTubeEpoch()`. Candidate/Epoch stores,
job-state commit, R3/marker publication, pair refresh, and diagnostic
publication all remain downstream of finalization. The local manager/profile
objects are destroyed during unwind. Therefore a cancelled A has no authority
or committed side effect.

### 9.2 No spurious cancellation of sample-held work

Cancellation compares semantic `TubeWorkIdentity`, not request pointer,
control sequence, request stamp, map sequence, or latest cloud pointer.
Ordinary same-owner map/control refresh preserves the identity and cannot
trigger the sentinel.

### 9.3 Only superseding work/lifecycle identity cancels

The only data read by the predicate are the existing immutable latest request
and the existing atomic task/authority/shutdown invalidators. A source, path,
frame, owner, pair, task, active-state, authority-session, or shutdown change
is already an accepted Stage1A stale-work boundary.

### 9.4 TubeEpochManager remains transactional

Production construction continues on the copied/fresh job-local manager.
Unwinding cannot reach `commitTubeJobLocalState()`. No partially built profile
can enter the committed manager, cache, active profile, installed epoch,
Candidate slot, or pair.

### 9.5 Lifecycle and join remain safe

Shutdown/reset/deactivate use their existing invalidation order and worker
notification. A blocked test checkpoint is always released before join.
Production callbacks do not block. Shutdown still sets stop, wakes, and joins
the single worker without holding a worker-needed lock. Reset/deactivate keep
the worker alive as Stage1A requires; only their obsolete running job exits.

### 9.6 Latest-only pending remains unchanged

Cancellation does not populate or reorder the pending slot. Timer permits
continue to replace the single slot with the newest eligible request.

### 9.7 Final currentness remains the second defense

A supersession can occur after the final cooperative checkpoint. The existing
unchanged `requestSourceStillCurrent()` inside the V2 publication barrier must
still reject that completion.

## 10. Release-latency expectation and acceptance bounds

Structurally:

    stale release latency
      <= remaining time of one already-entered abstract query
         + one bounded non-query phase (worst case: TubeFilter/assembly)
         + stack unwind/bookkeeping jitter

The existing long stale runs average approximately 0.01423 ms per query. A
query-boundary check removes the remaining 139k-280k obsolete-query tail.
Allowing far more than the observed average for a single callback and normal
scheduling, the future implementation gate is:

    EXPECTED_WORST_STALE_RELEASE_LATENCY_MS=250
    STALE_INFLIGHT_BLOCKING_TIME_MEDIAN_MS <= 100
    STALE_INFLIGHT_BLOCKING_TIME_P95_MS <= 250
    STALE_INFLIGHT_BLOCKING_TIME_MAX_MS <= 250

This is respectively at least a 96.7% median reduction from 2999.463 ms and a
bound below one quarter of the 1035.666 ms median path-revision interval.

The end-to-end future real-ROS availability gates are deliberately looser
because a genuinely current Tube build may still be expensive:

    DELETE_TO_NEXT_ADD_GAP_MEDIAN_MS <= 1000
    LONGEST_NO_TUBE_INTERVAL_MS < 13536.758
    VISIBILITY_DUTY_CYCLE > 0.129835
    CURRENT_COMPLETION_RATIO > 0.595238

The primary Stage1B acceptance is removal of multi-second stale-worker
occupation, not a guarantee that all current geometry builds are fast.

## 11. Exact implementation whitelist

Future Stage1B implementation may modify exactly these three existing files in
the isolated V2 worktree:

1. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
2. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
3. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

The header is limited to the private execution outcome, private probe/helper
declarations, one passive bounded test hook, and the observational tube-due
field.

No navigation API or fourth implementation/test file is authorized.

## 12. Protected files and semantics

Do not modify:

- `gvf_manager.cpp` or timer wiring;
- `phase_offset_navigation` headers, sources, or tests;
- TubeBuilder, TubeFilter, CertifiedTubeBuilder, TubeEpochManager,
  TubeSurfaceValidator, ValidationContext, or ValidateCell;
- half-voxel, v-span, cell partition, clearance, comparison, tolerance, query,
  or safety mathematics;
- `requestSourceStillCurrent()` or `epochMatchesRequest()` predicates;
- Stage1A one-worker/latest-only scheduling semantics;
- V2 marker ledger, retirement latency, publication barrier, or topic actions;
- the separately deferred V2 marker test-only repair or existing Tests A/B/C;
- R3, Runtime, Recovery, Handoff, H2, C2, planner, map, launch/config, Batch C/D.

## 13. Minimum deterministic regressions

All waits use bounded timeouts and all threads are joined.

### TEST 1 — obsolete A cooperatively exits and B runs

Start A and pause it at an adapter query checkpoint. Replace A with genuinely
different work identity B and give B the normal next Tube-timer permit. Release
A's checkpoint.

Assert:

- A reports `CANCELLED_OBSOLETE`;
- A publishes no raw/cloud/Candidate/Epoch/marker data and commits no job state;
- A does not traverse a test-controlled simulated remainder of its build;
- B starts within the bounded cancellation-release timeout without waiting for
  A's full completion path;
- B completes current and publishes only B authority;
- pending depth remains at most one.

### TEST 2 — sample-held refresh does not cancel

Start A and publish newer immutable requests that change only ordinary
map/control/sample-held fields while preserving `TubeWorkIdentity`.

Assert the cancellation predicate remains false, A reaches the normal current
finalization path, and no `cancelled_obsolete=1` row is produced.

### TEST 3 — cancellation races with shutdown/deactivate/reset

Use bounded subcases with A paused at a checkpoint.

- shutdown: invalidate, release checkpoint, prove sentinel is caught, no
  publication follows shutdown, and the worker is joined;
- deactivate: prove A exits, the existing inactive/pending semantics remain,
  and the worker remains healthy;
- reset: prove A exits, no old-task state is committed, and the joined-worker
  architecture remains usable for the next task; the persistent worker stays
  alive after reset and is joined exactly once by the later normal shutdown.

No lifecycle operation waits for A's full synthetic build remainder.

### TEST 4 — supersession after the final checkpoint

Pause A immediately after the last cooperative checkpoint but before
`finalizeTubeEpoch()`. Replace it with B, then release A.

Assert no cancellation sentinel is required at this late boundary, the
unchanged final `requestSourceStillCurrent()` rejects A, A publishes/commits
nothing, and B remains eligible.

No fifth new deterministic cancellation test is authorized.

## 14. Future build and regression gate

The future implementation must run at minimum:

    source devel/setup.bash
    catkin_make -j2
    catkin_make run_tests_phase_offset_core
    catkin_make run_tests_phase_offset_navigation
    catkin_make run_tests_bspline_race
    devel/lib/bspline_race/phase_offset_matched_adapter_test
    git diff --check

It must leave the separately deferred V2 test-only repair untouched and retain
the current Stage1A, V2 Tests A/B/C, R3, Runtime, Recovery, H2/C2, marker, and
Validator regression behavior. Stage1B may add only Tests 1-4 above; it may not
repair or rewrite an existing V2 test in the same file.

Instrumentation/cancellation disabled or never triggered must be behaviorally
identical to the current V2 checkpoint.

## 15. Future real-ROS acceptance

Use the unchanged five-goal V2 acceptance scenario and parameters. Record:

    owner/path/frame replacement timestamp
    worker request/start/completion identity
    cancelled_obsolete outcome/timestamp
    next pending/current worker start
    DELETE and NEW ADD timestamps
    marker geometry hash
    goal success and Runtime/control health

Verify the bounds in Section 10, no old-hash resurrection, no delayed stale
DELETE of newer geometry, unchanged prompt retirement, and no navigation or
control regression.

Do not retain stale geometry or weaken currentness to meet availability.

## 16. STOP conditions

Stop without redesign if:

- any fourth implementation/test path is required;
- an abstract query callback swallows the private sentinel;
- exceptions are disabled for the target or the sentinel can escape the worker;
- cancellation requires representing obsolete work as map/geometry/validator
  failure;
- a valid same-work-identity map/control refresh cancels;
- cancellation requires holding a publication/Runtime/worker mutex during
  heavy work;
- cancelled work can reach diagnostics, Candidate/Epoch, pair, marker, or
  committed job state;
- latest-only scheduling, final currentness, V2 marker lifecycle, R3, safety
  mathematics, Runtime/Recovery/H2/C2, or Batch C/D must change;
- an existing unrelated failure requires an out-of-scope repair.

## 17. Fresh Sol xhigh bounded plan audit

The exactly-one fresh Sol xhigh audit returned:

    Q1=BLOCKER
    Q2=PROVEN_SAFE
    Q3=PROVEN_SAFE
    Q4=PROVEN_SAFE
    P1_STAGE1B_COOPERATIVE_CANCELLATION_PLAN_NOT_ACCEPTABLE

Because Q1 did not prove that a cancelled stale build cannot publish or
partially commit, this candidate is rejected and is not frozen. No bounded
repair, second audit, implementation, or scope expansion is authorized in this
turn.

## 18. This turn

This document is plan-only. It authorizes no implementation, Luna, ROS run,
commit, or push. Exactly one fresh Sol xhigh shall audit only Q1-Q4. The plan
may be frozen only if all four are `PROVEN_SAFE` and the exact acceptance
verdict is returned.

    P1_STAGE1B_COOPERATIVE_CANCELLATION_PLAN_REJECTED
