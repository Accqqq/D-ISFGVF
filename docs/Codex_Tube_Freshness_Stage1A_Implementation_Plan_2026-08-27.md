# Tube Freshness Stage 1A — Authoritative Implementation Plan

DOCUMENT_ROLE=AUTHORITATIVE_STAGE_1A_IMPLEMENTATION_PLAN
DOCUMENT_STATUS=FROZEN
MAIN_FINAL_REVIEW=PASS
SOL_REQUIRED_CORRECTIONS=RESOLVED
IMPLEMENTATION_READY=true
IMPLEMENTATION_STARTED=false
AUTHORIZED_IMPLEMENTATION_AGENT=LUNA_MAX_ONLY
AUTHORIZED_SCOPE=STAGE_1A_ONLY
STAGE_1B_AUTHORIZED=false
BATCH_C_AUTHORIZED=false
BATCH_D_AUTHORIZED=false
COMMIT_AUTHORIZED=false
PUSH_AUTHORIZED=false
BASE_SHA=5355be736339a650432280deec53362f61749a43

## 1. Purpose and exact problem

The production Tube build currently runs synchronously inside the ROS Tube
timer callback. Build duration ranges from approximately 45 ms to 2.186 s,
while Candidate and Certified publications are observed near 2 Hz. A slow
build can become stale during C2/path replacement and is rejected only after
the expensive computation returns.

Stage 1A changes scheduling, freshness, and lifecycle ownership only. It does
not change Tube geometry, certification, Runtime, Recovery, H2, C2, planner,
safety, or visualization semantics.

Implementation is not started or authorized by this document's existence.

## 2. Authoritative baseline

Repository: Accqqq/D-ISFGVF
Workspace: /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
Branch: main
Frozen base SHA: 5355be736339a650432280deec53362f61749a43
Remote: d_isfgvf/main at the same accepted baseline

The work is forward-only from this SHA. Existing user-owned worktree changes
must remain untouched.

## 3. Accepted historical checkpoints

9a0e975  trusted original single-UAV PhaseOffset/GVF baseline
cda7e50  phase-offset Tube Batch A checkpoint
1902766  phase-offset Tube Batch B checkpoint
7f162b3  Batch-B post-acceptance Tube repairs
5355be7  accepted R3 + Horizontal-N frozen baseline

## 4. Existing worktree exclusions

The following entries predate this implementation plan and are not Stage 1A:

    M  AGENTS.md
    ?? .codex
    ?? Testing/
    ?? docs/D-ISFGVF_PhaseOffset_Tube_Architecture_Refactor_Brief_2026-08-23.md
    ?? docs/D-ISFGVF_Tube_Freshness_Stage1_Handoff_2026-08-26.md
    ?? src/swarm_planner/bspline_traj/test/__pycache__/
    ?? src/uav_simulator/so3_quadrotor_simulator/test/__pycache__/

They must not be cleaned, restored, stashed, staged, committed, or attributed
to Luna's Stage 1A work.

## 5. Baseline runtime evidence

    /particle0/gvf/traj_vis
        approximately 10.004 Hz, maximum interval approximately 113 ms

    local_map
        approximately 10.002 Hz, maximum interval approximately 108 ms

    phase_offset_manual/tube_candidate
        approximately 1.997 Hz
        p95 interval approximately 1.748 s
        maximum interval approximately 2.315 s

    phase_offset_manual/tube_certified_geometry
        approximately 1.997 Hz
        p95 interval approximately 1.749 s
        maximum interval approximately 2.316 s

    buildTubeEpoch
        count 9
        minimum 45.196 ms
        median 75.609 ms
        mean 450.068 ms
        p90 949.708 ms
        p95 1567.921 ms
        maximum 2186.134 ms
        source_current_finalized 7/9

Known expensive case: inward attempts 101, last queries 1629, and
last_limit_exceeded 1. These are measurements, not new safety thresholds.

## 6. Exact current architecture

formation_planning
  -> ros::AsyncSpinner(8)
     -> 50 Hz cmd_timer
        -> gvf_manager::cmdCallback()
           -> PhaseOffsetMatchedAdapter::update()
              -> Runtime/gate/recovery/authority command path

     -> Tube timer, currently literal ros::Duration(0.10)
        -> gvf_manager::phaseOffsetTubeTimerCallback()
           -> PhaseOffsetMatchedAdapter::timerTick()
              -> timer_inflight_ CAS
              -> latest_build_request_ load
              -> buildTubeEpoch()
                 -> TubeEpochManager::update()
                 -> CertifiedTubeBuilder::build()
                 -> TubeBuilder/filter/SurfaceValidator/inward search
              -> finalizeTubeEpoch()
                 -> Candidate/epoch/diagnostic/publication
           -> activatePendingOffsetAuthority()

     -> exec_fsm_timer 20 ms
     -> test_cmd_timer 20 ms
     -> kino_timer 200 ms

The relevant current adapter functions are makeBuildRequest(),
sourceRevision(), requestSourceStillCurrent(), epochMatchesRequest(),
buildTubeEpoch(), finalizeTubeEpoch(), consumeTimerTaskGeneration(),
timerTick(), advertise(), requestShutdown(), shutdown(),
resetForNewNavigationTask(), and deactivate().

## 7. Exact target architecture

The 50 Hz command path continues to replace the immutable
latest_build_request_ only. It never directly starts a Tube build.

Each ROS Tube timer event calls scheduleTubeBuild():

    create one monotonic SchedulePermit
    atomically load the latest immutable request
    replace one pending slot with the newest eligible request
    notify the worker
    return quickly

A single joined worker consumes the pending slot, creates job-local state,
runs the shared runTubeBuildJob primitive, applies stale/shutdown/currentness
gates, and commits/publicizes only current results.

There is exactly one running job and exactly one pending request slot. There
is no FIFO. If A is running and B, C, and D arrive, only D remains pending.

## 8. Scheduler permit contract

The implementation must define:

    schedule_permit_id
    pending_request and pending_permit_id
    running_request and running_permit_id
    last_started_request_identity
    last_completed_request_identity
    worker_started
    worker_stop_requested

Request instance identity is the immutable request pointer plus task
generation. Tube work identity is the semantic identity in Section 11.

For each Tube timer permit:

1. Increment the permit ID.
2. Atomic-load latest_build_request_.
3. Reject null, shutdown-invalid, and task-invalid requests.
4. Under the worker-state mutex, compare request identity with running and
   pending identities.
5. Replace the one pending slot with the newest eligible request.
6. Never append to a queue.
7. Notify the worker and return.

A repeated permit carrying the exact same immutable request pointer must not
rebuild that request repeatedly. A new immutable request may be selected on a
later permit, including a same-source request carrying newer map provenance.

After a worker job completes:

1. Finalize or abandon the running request.
2. Record its last-completed identity.
3. Consume an already populated pending slot, if one exists.
4. Do not blindly load latest_build_request_ and immediately rebuild.
5. If a request arrived after the final pending check and no new permit has
   populated the slot, wait for the next configured Tube timer permit.

Therefore ordinary 50 Hz command traffic cannot produce 50 Hz Tube builds:
command updates replace an immutable slot, while only Tube timer permits allow
worker selection.

The 0.05 and 0.10 values limit permit cadence, not completed Tube rate.

## 9. One authoritative execution path

All construction and finalization must be implemented in one shared primitive:

    runTubeBuildJob(request, permit, job_local_state)
        -> precondition/currentness checks
        -> job-local buildTubeEpoch
        -> post-build stale/shutdown checks
        -> transactional commit/finalization

Production path:

    phaseOffsetTubeTimerCallback
        -> scheduleTubeBuild
        -> joined worker
        -> runTubeBuildJob

Compatibility path:

    timerTick
        -> the same runTubeBuildJob primitive synchronously
        -> only while worker_started is false

Once the production worker is started, timerTick must not execute a second
synchronous Tube implementation. It must reject or perform only the explicitly
defined bounded scheduler behavior. There must be one writer for every
former timer-owned mutable field.

## 10. Worker state machine

STOPPED
  -> advertise/start -> IDLE

IDLE
  -> timer permit with eligible request -> PENDING
  -> shutdown/reset/deactivate -> INVALIDATED or STOPPING

PENDING
  -> worker takes slot -> RUNNING
  -> later timer permit -> replace pending with newest request
  -> identity invalidation -> INVALIDATED

RUNNING
  -> semantic identity change -> stale cancellation epoch
  -> current build return -> FINALIZING
  -> stale build return -> ABANDONED
  -> shutdown -> STOPPING after the current build returns in Stage 1A

FINALIZING
  -> currentness and shutdown pass -> COMMITTED
  -> failed gate -> ABANDONED

COMMITTED or ABANDONED
  -> pending slot exists -> RUNNING
  -> otherwise -> IDLE

STOPPING
  -> worker return -> JOINED

The worker-state mutex protects only worker lifecycle and the pending/running
bookkeeping. It is never held during heavy construction or finalization.

## 11. Currentness and obsolescence identity

Tube work identity includes:

    task_generation
    active/inactive state
    source_revision
    path revision
    frame revision
    authority_session
    frame_owner pointer identity
    base_path_tube_pair pointer identity
    base_path_tube_pair_generation
    shutdown/reset/deactivate invalidation state

Semantic owner invariant:

sourceRevision() already derives identity from semantic_path_owner or the
semantic path identity and semantic path start/end bounds. Stage 1A must
preserve and test:

    semantic owner replacement or semantic bounds change
        -> sourceRevision increments source_revision

No independent semantic-owner revision scheme may be invented.

Frame-owner invariant, selected four-file solution:

    for one source_revision, frame_owner pointer and frame revision are fixed

The implementation must retain the existing makeBuildRequest() reuse/create
behavior, record immutable frame-owner identity/revision in the epoch snapshot,
and explicitly compare frame-owner identity/revision in
requestSourceStillCurrent() and epochMatchesRequest(). On frame-owner
replacement, stale Candidate/epoch exposure must be cleared or fail closed,
even if numeric source_revision is unchanged.

Ordinary control_sequence updates, ordinary command updates, and ordinary
newer map_observation_sequence are not unconditional cancellation causes. An
immutable same-source cloud snapshot may legitimately complete while a newer
map observation exists. Existing source/currentness/provenance gates remain
authoritative.

## 12. Complete ownership model

latest_build_request_: immutable cross-thread snapshot, atomic command-side
publication and scheduler/worker atomic load.

latest_candidate_epoch_snapshot_: immutable worker publication snapshot,
updated only after currentness checks.

latest_epoch_snapshot_: immutable Runtime-eligible worker publication snapshot.

latest_control_snapshot_: command-owned immutable snapshot, read by worker only
at the short finalization boundary.

authoritative_path_tube_pair_: immutable H2 snapshot under the existing CAS and
Runtime lock protocol.

authority_session_ and task_generation_: atomic invalidation boundaries.

task_publication_mutex_: short reset/finalization publication barrier; never
held across heavy Tube construction.

runtime_command_mutex_: command-owned Runtime/recovery/authority state; worker
may use it only for existing short pair/publication operations.

tube_epoch_manager_, cached samples, cache revision/flag, active profile,
installed active epoch, latest cloud status, timer task generation, and
publication/delete counters: worker committed state with job-local copies.

timer_build_sequence_: worker attempt sequence, incremented once per actual
Tube-build attempt.

timer_inflight_: compatibility guard for the unadvertised synchronous test
path only; it must not drop production timer permits.

tube-due timing samples: attempt-only measurement state under the existing
measurement mutex, with no control semantics.

Runtime, recovery, pending authority, and gate fields remain command-owned and
unchanged.

## 13. Transactional job-local state

Define committed worker state and job-local state.

Committed worker state contains the last committed ordinary manager, cached
samples/revision/flag, active profile/installed epoch, cloud status, timer
task-generation marker, and publication/delete bookkeeping.

A job-local state is copied from committed state before an ordinary build and
contains its request, permit, work identity, attempt sequence, local manager,
local cache/profile, local cloud status, and local diagnostics.

Every mutable write currently performed by buildTubeEpoch(),
finalizeTubeEpoch(), or diagnostic publication must belong to one of:

    committed worker state
    job-local state
    publication-barrier state
    attempt-only measurement state
    immutable cross-thread snapshot
    command-owned state

A stale job may retain attempt timing and the existing same-task raw/cloud
provenance evidence where current semantics allow it. It must not commit
manager, cache, active profile, epoch, cloud status, publication ownership,
Candidate, Certified, or new-task DELETE state.

Heavy computation holds none of task_publication_mutex_,
runtime_command_mutex_, or the worker-state mutex.

Finalization acquires task_publication_mutex_ first, briefly commits worker
state, releases the worker-state mutex, then performs only the existing short
Runtime/pair operation. The worker-state mutex must not be held while entering
finalizeTubeEpoch().

## 14. Ordinary job versus H2 pair refresh

Ordinary request:

    copy the committed ordinary TubeEpochManager into job-local state

Pair-refresh request:

    construct a fresh config-only TubeEpochManager

The pair-refresh path must not inherit ordinary Candidate/Active history.
The existing behavior of
StaleBaseTimerEpochCannotChangeNewPairOrPersistentTimerManager must remain
unchanged. A stale pair refresh cannot alter the ordinary manager, profile,
cache, or newer PathTubePair.

## 15. Candidate sequence semantics

Stage 1A freezes the four-file-compatible interpretation:

    candidate_sequence
        = committed Candidate sequence

    timer_build_sequence
        = every actual worker Tube-build attempt

A stale job may consume timer_build_sequence without committing
candidate_sequence. A discarded local manager sequence may therefore appear
again in the next committed Candidate. This is an explicit diagnostic
distinction, not an accidental promise of unique sequence numbers for all
attempts.

Every committed Candidate must preserve:

    profile.tube_revision
        == profile.profile_revision
        == status.candidate_sequence

Do not change fixed public diagnostic schemas. Attempt identity is represented
by worker timer_build_sequence and acceptance evidence; candidate_sequence
remains the existing committed compatibility field.

If this contract cannot be truthful using the four files, stop. Do not modify
phase_offset_navigation for sequence reservation.

## 16. Reset, deactivate, and shutdown

Reset/new task remains the task-boundary linearization point. It invalidates the
task/cancellation identity, notifies the worker, prevents old jobs from
committing, and leaves running job-local objects untouched. Existing Runtime
reset semantics remain unchanged.

Deactivate publishes the immutable inactive request, invalidates active work,
notifies the worker, performs the existing timer-side clear/DELETE transition,
and preserves Runtime retained delta and previous-final-port history. An old
DELETE may not erase a newer task visualization.

The manager destructor order remains:

    adapter->requestShutdown()
    stop all manager timers
    adapter->shutdown()

requestShutdown() must set shutdown_requested_, invalidate pending/running
work identity, set worker stop state, and wake the condition variable.

The worker may finish one already-running build in Stage 1A, but post-build and
publication-barrier checks must reject shutdown-invalidated work.
finalizeTubeEpoch() must check shutdown before diagnostics, Candidate,
Certified, or DELETE publication.

shutdown() must join the one worker with no worker/publication/Runtime lock
held. Only after join may it clear worker state, snapshots, and publishers.
It must be idempotent, leave no detached worker or UAF, and publish nothing
after the shutdown ownership flip.

## 17. Bootstrap and authority activation

Actual production advertise() clears the test-only Runtime owner capability
before the Tube timer is created. Therefore advertised production
requiresPathTubePairBootstrap() is expected to be NOT_REQUIRED.

This fact must be asserted/tested. When NOT_REQUIRED, scheduler-only callback
behavior may retain the existing no-op/not-required activation path.

If a bootstrap-required path is ever reachable, activation must be coupled to
successful worker completion/finalization, not to scheduler enqueue alone.
H2 authority timing and ownership semantics must not change.

## 18. Timer-period wiring

matched_config_ is assigned before the manager creates the Tube timer. The only
authorized wiring change is:

    ros::Duration(0.10)
        -> ros::Duration(matched_config_.tube_update_period)

No launch/configuration change is authorized. The acceptance must verify a
0.05 configuration produces approximately 50 ms scheduling permits and a
0.10 configuration produces approximately 100 ms permits. This does not
require completed Tube builds at 20 Hz or 10 Hz.

## 19. Exact four-file whitelist

Luna may modify exactly:

1. src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
2. src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
3. src/swarm_planner/bspline_traj/src/gvf_manager.cpp
4. src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

No fifth file is authorized. In particular, do not modify phase_offset_navigation
production files, TubeEpochManager, CertifiedTubeBuilder, TubeBuilder,
TubeSurfaceValidator, ValidationContext, gvf_switch_policy_test.cpp,
CMakeLists.txt, launch/config files, Runtime, Recovery, H2, C2,
Horizontal-N, or R3 production semantics.

## 20. Function-level instructions

phase_offset_matched_adapter.h:
- add worker lifecycle, permit, request identity, committed/job-local state,
  scheduler, worker-main, shared run, and join declarations;
- retain immutable request/snapshot semantics;
- add only internal frame-owner identity needed for currentness;
- do not alter Runtime or public diagnostic schemas.

phase_offset_matched_adapter.cpp:
- start the worker only after advertise setup;
- implement scheduleTubeBuild(), worker main, runTubeBuildJob(), and join;
- refactor buildTubeEpoch() to use job-local state;
- keep ordinary manager-copy and pair-refresh fresh-manager branches;
- update requestSourceStillCurrent() and epochMatchesRequest() with explicit
  frame-owner identity/revision checks;
- clear/fail-closed Candidate exposure on frame replacement;
- add shutdown checks before and inside finalization publication;
- make reset/deactivate/shutdown invalidate and notify safely;
- keep timerTick() as a guarded shared-primitive compatibility entry;
- preserve all existing finalization, source, map, Candidate, Certified, Pair,
  Runtime, Recovery, H2, and R3 gates.

gvf_manager.cpp:
- use matched_config_.tube_update_period for the Tube timer duration;
- call scheduler-only behavior from the production Tube callback;
- preserve bootstrap ordering and test the production NOT_REQUIRED invariant;
- do not change command/FSM/kino timers, AsyncSpinner, planner, C2, or H2.

phase_offset_matched_adapter_test.cpp:
- add only the listed scheduler, transaction, lifecycle, sequence,
  owner/frame, R3, period, and regression tests;
- keep existing expected semantic behavior unchanged.

## 21. Frozen and forbidden semantics

Stage 1A must not change:

- Horizontal-N specialization;
- N or N_w mathematics;
- r = p + N delta;
- r_w;
- exact-zero or tiny-nonzero authority semantics;
- Tube geometry or certification predicates;
- SurfaceValidator acceptance predicates;
- CertifiedTubeBuilder inward-search policy;
- TubeBuilder algorithm;
- ESDF safety thresholds;
- back_w, lookahead_w, sample_step_w;
- environment_search_extent, ray_step, min_certified_forward_w;
- R3 Candidate meaning or Certified Geometry eligibility;
- Runtime selected-u/control law;
- Recovery;
- H2 authority;
- C2 geometry;
- planner cadence or command cadence;
- AsyncSpinner thread count;
- topics/namespaces;
- marker geometry/colors;
- Stage 1B cancellation;
- Batch C/D, NoQP, or gradient-representation work.

## 22. Exact required tests

The existing adapter test target must include:

1. SchedulerReturnsWithoutSynchronousTubeBuild
2. LatestOnlyCoalescesAThroughDToD
3. PendingSlotNeverExceedsOne
4. NoDuplicateRebuildWithoutNewPermitOrTubeIdentity
5. Normal50HzCommandTrafficCannotCause50HzTubeBuildChurn
6. RequestArrivingAtWorkerCompletionWaitsAtMostNextPermitAndDoesNotFIFO
7. StaleOrdinaryOwnerOrFrameCompletionCannotPublish
8. StaleOrdinaryJobCannotMutateCommittedWorkerState
9. StalePairRefreshCannotMutatePersistentManagerOrNewPair
10. TaskGenerationResetInvalidatesRunningJob
11. DeactivateDuringBuildProducesCorrectLifecycle
12. ShutdownJoinsAndPreventsPostShutdownPublication
13. TimerTickCannotSynchronouslyBuildWhenWorkerStarted
14. OrdinaryJobCopiesCommittedManagerButPairRefreshUsesFreshManager
15. CandidateSequenceTracksCommittedCandidateAndBuildSequenceTracksAttempts
16. R3RequestCandidateRequestProtocolUnchanged
17. Existing exact-zero and tiny-nonzero regressions unchanged
18. Configured scheduling-period behavior for 0.05 and 0.10
19. SourceRevisionSemanticOwnerInvariant
20. FrameOwnerReplacementWithUnchangedSourceRevisionIsRejected
21. AdvertisedProductionBootstrapRemainsNotRequired

Required assertions include pending depth <= 1, no repeated unchanged request,
no B/C FIFO, no stale ordinary cache/profile/cloud/counter mutation, no stale
Pair replacement, no post-shutdown publication, and committed profile/status
sequence equality.

After implementation, run:

    catkin_make -j2
    catkin_make run_tests_phase_offset_core
    catkin_make run_tests_phase_offset_navigation
    catkin_make run_tests_bspline_race

Also run the accepted Horizontal-N, Runtime, MatchedAdapter, Recovery, R3,
C2/H2, exact-zero, tiny-nonzero, and original planner regressions.

## 23. Real ROS acceptance

Use:

    roslaunch so3_quadrotor_simulator simulator.launch

    roslaunch bspline_race test_gvf.launch
      phase_offset_mode:=manual
      phase_offset_manual_observe_only:=false
      phase_offset_manual_tube_source:=esdf
      phase_offset_tube_cloud_obstacle_set_complete:=true
      phase_offset_tube_preincluded_map_uncertainty:=0.10

Measure:

- traj_vis Hz;
- tube_candidate Hz;
- tube_certified_geometry Hz;
- command callback Hz;
- local_map Hz;
- scheduling permit cadence;
- worker build-start cadence;
- completed publication cadence;
- pending depth;
- timer_build_sequence order;
- committed candidate_sequence order;
- request age at worker start;
- candidate age at publication;
- current_w minus candidate_build_w;
- stale abandonment count;
- source-current rejection count;
- shutdown join latency.

Acceptance does not require completed Tube rate to equal 10 Hz. It requires no
FIFO, no 50 Hz worker churn, bounded scheduler callback time, pending depth
never above one, stale state/publication rejection, non-accumulating request
age, materially improved temporal staleness, healthy command/planner/map
behavior, and unchanged Frozen semantics.

ROS logs, bags, CSV evidence, /tmp directories, build artifacts, and temporary
files must not be added to Git.

## 24. Stage 1A limitation and Stage 1B boundary

Stage 1A does not interrupt a build already deep inside TubeBuilder,
SurfaceValidator, or inward-search validation. One obsolete job may still run
for approximately 2.186 s before post-build abandonment.

Stage 1A is nevertheless correct and worthwhile: it removes FIFO backlog, keeps
the ROS callback fast, and prevents stale completion from corrupting committed
state or publishing current Candidate/Certified data.

Stage 1B remains separately unauthorized. Future work may add an independent
CANCELLED status and cooperative checks through CertifiedTubeBuilder,
TubeBuilder, TubeSurfaceValidator, ValidationContext, ValidateCell(), and
query loops. Cancellation must never be represented as ordinary validation
failure.

## 25. Future paper and Batch C/D reconciliation

This note does not amend the Frozen Plan or authorize later work.

Before Batch C authorization, Main must reconcile the latest paper's:

    N = (e_z x p_w) / ||e_z x p_w||
    retained raw branch as the connected component containing delta = 0
    arc-length s(w) -> approximately uniform Delta-s cross sections
    backward feasible recursion K_k -> inner envelope K(s)

The latest paper describes soft spacing regulation using g_sep, g_coh,
g_damp, g_des, analytic phase/transverse decomposition, scalar projections,
and matched injection. It does not authorize the older hard pairwise-CBF
Batch C design. Batch D remains future cleanup/observability work.

## 26. Luna execution sequence and rules

Only Luna Max may be authorized in a later explicit turn.

Required sequence:

    read this frozen plan completely
        ->
    verify branch, HEAD, worktree, and staged paths
        ->
    report the start snapshot
        ->
    modify only the four authorized files
        ->
    run focused build/tests
        ->
    run full regressions
        ->
    run git diff --check
        ->
    verify whitelist and protected semantics
        ->
    STOP and return the report

Luna must not redesign this plan, edit the handoff, edit the Frozen Plan or
AGENTS.md, modify a fifth file, add Stage 1B cancellation, change Batch C/D,
stage, commit, push, clean, reset, restore, stash, checkout, rewrite history,
or start another production writer/sub-agent.

## 27. STOP conditions

Stop and report if:

- branch, HEAD, or baseline differs materially;
- an edit is required outside the four-file whitelist;
- TubeEpochManager copyability is not compile-safe;
- candidate-sequence semantics cannot be truthful within four files;
- sourceRevision semantic-owner invariant fails;
- frame-owner identity cannot be validated;
- worker state cannot have one writer;
- a worker/publication/runtime lock inversion appears;
- join requires holding a worker-needed lock;
- bootstrap-required production behavior is reachable;
- any Frozen/R3/Runtime/Recovery/H2/C2/Horizontal-N semantic change appears;
- existing unrelated worktree content must be touched;
- acceptance requires cadence, safety-threshold, or topic changes;
- Stage 1B cancellation is introduced.

## 28. Repository hygiene expectations

Before and after implementation Luna must report:

    branch
    HEAD
    git status --short
    git diff --check
    git diff --cached --name-only

The pre-existing user-owned entries must remain untouched. Stage 1A changes may
appear only in the four authorized files. No path may be staged by Luna.

## 29. Final state

STAGE_1A_IMPLEMENTATION_PLAN_FROZEN_READY_FOR_LUNA

This plan is frozen and may be handed to Luna in a future turn. Luna has not
been started in this turn. No implementation, staging, commit, push, or
next-stage action follows this document.

STOP.

