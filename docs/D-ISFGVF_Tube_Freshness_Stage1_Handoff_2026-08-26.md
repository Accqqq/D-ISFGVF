# D-ISFGVF Tube Freshness — Stage 1 Handoff

```text
DOCUMENT_ROLE=FRESH_MAIN_CONTEXT_HANDOFF
DOCUMENT_STATUS=CORRECTED STAGE 1A SPECIFICATION / IMPLEMENTATION NOT AUTHORIZED
DATE=2026-08-26
REPOSITORY=Accqqq/D-ISFGVF
BRANCH=main
FROZEN_ROLLBACK_SHA=5355be736339a650432280deec53362f61749a43
SOL_VERDICT=STAGE_1A_PLAN_ACCEPTABLE_WITH_REQUIRED_CORRECTIONS
IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
STAGE_1B_AUTHORIZED=false
COMMIT_AUTHORIZED=false
PUSH_AUTHORIZED=false
```

## 1. Purpose and status

This document is the self-contained handoff and corrected execution
specification for the Tube freshness/performance repair known as Stage 1.
It incorporates the independent read-only Sol review and resolves its
blocking findings.

This document is not an implementation authorization. No production code,
test code, launch/configuration, CMake, Frozen Plan, or AGENTS file may be
changed merely because this handoff exists.

The current recommendation is:

```text
Stage 1A:
  one joined Tube worker
  latest-only scheduling with one pending slot
  transactional job-local Tube state
  stale completion abandonment
  reset/deactivate/shutdown lifecycle
  tube_update_period wiring

Stage 1B:
  cooperative cancellation inside expensive Tube computation
  separately unauthorized and separately reviewable
```

The corrected Stage 1A specification is ready for final Main review, but
implementation remains blocked until that review explicitly releases it.

## 2. Repository, branch, and frozen rollback SHA

Repository:

```text
Accqqq/D-ISFGVF
```

Actual workspace:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

Authoritative baseline:

```text
branch: main
HEAD:   5355be736339a650432280deec53362f61749a43
remote: d_isfgvf/main at the same accepted baseline
```

The SHA above is the rollback and planning baseline for this handoff. It is
not permission to rewrite history or to commit Stage 1A.

## 3. Accepted historical checkpoints

The repository history records these earlier milestones:

```text
9a0e975  trusted original single-UAV PhaseOffset/GVF baseline
cda7e50  phase-offset Tube Batch A checkpoint
1902766  phase-offset Tube Batch B checkpoint
7f162b3  Batch-B post-acceptance Tube repairs
5355be7  accepted R3 + Horizontal-N frozen baseline
```

The current Stage 1 work is forward-only from
`5355be736339a650432280deec53362f61749a43`. Existing accepted Horizontal-N,
R3, Runtime, Recovery, H2, and Frozen-plan content is user-owned baseline
material and must not be reconstructed, rolled back, or silently rewritten.

## 4. Current working-tree exclusions

At handoff creation, existing unrelated worktree entries are:

```text
 M AGENTS.md
?? .codex
?? Testing/
?? docs/D-ISFGVF_PhaseOffset_Tube_Architecture_Refactor_Brief_2026-08-23.md
?? src/swarm_planner/bspline_traj/test/__pycache__/
?? src/uav_simulator/so3_quadrotor_simulator/test/__pycache__/
```

They are not Stage 1A content. They must remain untouched and must never be
staged, committed, or attributed to the Tube repair.

The only new path permitted by the current user authorization is this
untracked handoff document:

```text
docs/D-ISFGVF_Tube_Freshness_Stage1_Handoff_2026-08-26.md
```

No staging, commit, push, stash, reset, restore, clean, checkout, branch
creation, or history rewrite is authorized in this handoff turn.

## 5. Current Tube-lag symptom

The real ROS experiment established approximately:

```text
/particle0/gvf/traj_vis       ~10 Hz
phase_offset_manual/tube_candidate       ~2 Hz
phase_offset_manual/tube_certified_geometry ~2 Hz
```

The Tube build is highly variable:

```text
minimum       ~45 ms
median        ~76 ms
mean         ~450 ms
p95         ~1.57 s
maximum      ~2.186 s
```

Some slow builds became stale during C2/path replacement and were rejected
only after the expensive computation completed.

## 6. Runtime measurements preserved for A/B comparison

The known baseline evidence is:

```text
traj_vis:
    ~10.004 Hz
    max interval ~113 ms

local_map:
    ~10.002 Hz
    max interval ~108 ms

tube_candidate:
    ~1.997 Hz
    p95 interval ~1.748 s
    max interval ~2.315 s

tube_certified_geometry:
    ~1.997 Hz
    p95 interval ~1.749 s
    max interval ~2.316 s

buildTubeEpoch timing:
    count 9
    min 45.196 ms
    median 75.609 ms
    mean 450.068 ms
    p90 949.708 ms
    p95 1567.921 ms
    max 2186.134 ms
    >100 ms: 4/9
    >=500 ms: 2/9
    >=1 s: 1/9
    source_current_finalized: 7/9
```

Known expensive case:

```text
inward attempts = 101
last_queries = 1629
last_limit_exceeded = 1
```

These values are evidence, not new safety thresholds.

## 7. Confirmed root cause

The heavy Tube build currently runs synchronously in the ROS Tube timer
callback. ROS timer scheduling is therefore delayed by the build duration;
the callback's `timer_inflight_` guard prevents overlap but does not provide
freshness or cancellation.

The current ordinary build also mutates persistent timer-side state before
the final source/currentness decision. A stale build can therefore affect a
newer request unless all build state is transactional.

## 8. Exact current code path

```text
formation_planning
└── ros::AsyncSpinner(8)
    ├── cmd_timer: 20 ms
    │   └── gvf_manager::cmdCallback()
    │       └── PhaseOffsetMatchedAdapter::update()
    │           ├── Runtime/gate/recovery/authority command state
    │           └── PositionCommand staging/publication
    │
    ├── phase_offset_tube_timer_: literal ros::Duration(0.10)
    │   └── gvf_manager::phaseOffsetTubeTimerCallback()
    │       ├── PhaseOffsetMatchedAdapter::timerTick()
    │       │   ├── timer_inflight_ CAS
    │       │   ├── consumeTimerTaskGeneration()
    │       │   ├── latest_build_request_ load
    │       │   ├── inactive transition / DELETE, or
    │       │   └── active synchronous build:
    │       │       ├── buildTubeEpoch()
    │       │       ├── TubeEpochManager::update()
    │       │       ├── CertifiedTubeBuilder::build()
    │       │       ├── TubeBuilder / cloud-clearance path
    │       │       ├── filtering
    │       │       └── SurfaceValidator + inward search
    │       │   └── finalizeTubeEpoch()
    │       │       ├── task_publication_mutex_
    │       │       ├── diagnostics
    │       │       ├── requestSourceStillCurrent()
    │       │       ├── Candidate/epoch slots
    │       │       ├── same-owner Pair refresh
    │       │       └── publishManual()
    │       └── activatePendingOffsetAuthority()
    │
    ├── exec_fsm_timer: 20 ms
    ├── test_cmd_timer: 20 ms
    └── kino_timer: 200 ms
```

`gvf_manager::phaseOffsetTubeTimerCallback()` also contains the H2 bootstrap
bridge. Production `advertise()` clears the test-only Runtime owner
capability, so the bridge is normally `NOT_REQUIRED` in the production path.

## 9. Why RViz, local map, and the command loop are not the main cause

The baseline shows that trajectory visualization and local-map publication
remain close to 10 Hz while Candidate/Certified Tube publications fall to
about 2 Hz. The measured Tube build itself includes multi-hundred-millisecond
and multi-second outliers. Therefore the primary bottleneck is the synchronous
Tube computation and its delayed timer scheduling, not RViz rendering, the
local-map producer, or the 50 Hz command callback.

The Stage 1A design must not compensate by changing command cadence,
planner cadence, map semantics, or visualization geometry.

## 10. Stage 1A goal

The target is:

```text
ROS Tube timer / scheduler
    -> capture newest immutable request
    -> overwrite one pending slot
    -> wake worker
    -> return quickly

single joined Tube worker
    -> consume newest permitted request
    -> build in job-local state
    -> stale abandonment/currentness checks
    -> existing finalization gates
    -> Candidate / Certified publication
```

Forbidden behavior:

```text
A -> B -> C -> D FIFO backlog
```

Required behavior:

```text
worker building A
B arrives
C arrives
D arrives
pending slot contains only D
```

## 11. Final corrected Stage 1A execution specification

### 11.1 Single worker and one-slot scheduling

Production `advertise()` starts exactly one Tube worker. The worker is joined
by `shutdown()` and is never detached.

The scheduler owns no heavy build. Each ROS Tube timer callback creates one
monotonic scheduling permit and performs only bounded capture/coalescing work.

Ordinary 50 Hz command/request updates may replace the immutable
`latest_build_request_`, but they do not themselves create a scheduling permit
and do not wake a build loop that bypasses the Tube timer period.

### 11.2 Scheduling concepts

The implementation must define the following distinct concepts:

```text
SchedulePermit
    monotonic timer-produced permit; one permit per Tube timer event

RequestInstanceIdentity
    immutable request pointer identity, with task generation and request
    metadata used to prevent duplicate processing

TubeWorkIdentity
    semantic identity used for staleness/cancellation; not just sequence

running request
    the one request currently owned by the worker job

pending request
    the one request slot waiting behind the running job

last-started request
    identity of the most recently started job

last-completed request
    identity of the most recently completed/abandoned job

stale request
    a request whose task/source/owner/frame/authority/pair/shutdown identity
    no longer matches the current owner state
```

### 11.3 Exact scheduler algorithm

For each timer callback:

1. Increment `schedule_permit_id`.
2. Atomically load `latest_build_request_`.
3. Reject null, shutdown, or task-invalid requests.
4. Under the worker-state mutex, compare the immutable request identity with
   the running and pending identities.
5. If it is eligible and not already represented by the same pending/running
   request, replace the pending slot with this newest request and record the
   permit ID.
6. Never append to a queue.
7. `notify_one()` and return.

When the worker completes a job:

1. It does not blindly rebuild whatever `latest_build_request_` currently
   contains.
2. It may consume a request already captured by a later timer permit.
3. It may coalesce a newer request only if that request has an outstanding
   scheduling permit represented by the pending slot.
4. If a request arrives after the final pending check and no permit exists,
   the worker waits for the next configured timer permit.
5. The same immutable request pointer is never rebuilt repeatedly merely
   because the worker returned to its wait loop.

This gives a bounded extra delay of at most one configured Tube period for
the check/completion race, while preserving the configured scheduling rate.

Normal 50 Hz traffic cannot create 50 Hz Tube builds because:

```text
50 Hz update -> latest immutable request replacement only
10/20 Hz timer permit -> at most one worker request selection per permit
worker -> one running job, one pending slot, no FIFO
```

The `0.05` period permits 20 scheduling events per second, and `0.10` permits
10 scheduling events per second; neither means that completed heavy builds
must reach those rates.

### 11.4 One authoritative build path

The implementation must extract one authoritative execution primitive,
conceptually:

```text
runTubeBuildJob()
    -> create job-local state
    -> perform heavy build
    -> post-build stale/currentness checks
    -> commit state only if current
    -> finalize/publication
```

Production uses:

```text
phaseOffsetTubeTimerCallback()
    -> scheduleTubeBuild()
    -> joined worker
    -> runTubeBuildJob()
```

`timerTick()` may synchronously invoke the same primitive only when
`worker_started_ == false`, preserving existing unadvertised deterministic
unit fixtures. Once the production worker has started, `timerTick()` must not
perform a second synchronous build path; it must reject or act only as a
bounded scheduler wrapper according to the explicit guard.

## 12. Worker/scheduler state machine

```text
STOPPED
  --advertise/start--> IDLE

IDLE
  --timer permit with eligible request--> PENDING
  --shutdown/reset/deactivate--> INVALIDATED or STOPPING

PENDING
  --worker takes slot--> RUNNING
  --new timer permit--> PENDING(newest request only)
  --shutdown/reset/deactivate--> INVALIDATED

RUNNING
  --new timer permit--> RUNNING + replace PENDING slot
  --semantic identity change--> stale token set
  --build returns current--> FINALIZING
  --build returns stale--> ABANDONED
  --shutdown--> STOPPING after build returns (Stage 1A)

FINALIZING
  --publication barrier + currentness pass--> COMMITTED
  --currentness/shutdown fail--> ABANDONED

COMMITTED / ABANDONED
  --pending permit exists--> RUNNING
  --otherwise--> IDLE

STOPPING
  --worker exits--> JOINED
```

The worker-state mutex protects only pending/running/lifecycle bookkeeping.
It is never held across heavy construction or finalization.

## 13. Complete ownership/state table

| State | Classification | Required ownership/guard |
|---|---|---|
| `latest_build_request_` | immutable shared snapshot | command/planner atomic publication; scheduler/worker atomic load |
| `latest_candidate_epoch_snapshot_` | immutable shared snapshot | worker publication after currentness; atomic readers |
| `latest_epoch_snapshot_` | immutable shared snapshot | worker publication only for Runtime-eligible current epoch |
| `latest_control_snapshot_` | command-owned immutable snapshot | command path constructs; worker reads at short finalization boundary |
| `authoritative_path_tube_pair_` | immutable H2 snapshot | existing CAS/runtime lock and exact pair identity |
| `authority_session_` | atomic shared invalidation token | command/reset/retire writes; worker reads |
| `task_generation_` | atomic task boundary | command reset writes; worker reads |
| `task_publication_mutex_` | publication barrier state | reset/finalize linearization; never held during build |
| `runtime_command_mutex_` | command-owned Runtime/authority state | command writer; worker only short pair/publication access |
| `timer_inflight_` | compatibility guard | synchronous unadvertised test path only |
| `timer_task_generation_` | worker committed state | worker-only writer |
| `timer_build_sequence_` | attempt-only/worker state | worker assigns every actual build attempt |
| `tube_epoch_manager_` | worker committed state | worker-only committed owner; never command-mutated |
| `cached_full_path_samples_` | worker committed state | job-local copy, commit only when current |
| `cached_path_source_revision_` | worker committed state | job-local copy, commit only when current |
| `have_cached_path_` | worker committed state | job-local copy, commit only when current |
| `timer_active_profile_` | worker committed state | job-local copy, commit only when current |
| `timer_installed_active_epoch_` | worker committed state | job-local copy, commit only when current |
| `latest_cloud_occupancy_query_status_` | worker committed diagnostic state | job-local/attempt evidence; no stale committed overwrite |
| `timer_last_published_epoch_build_sequence_` | publication-barrier state | worker-only after successful publication |
| `timer_last_raw_diagnostic_build_sequence_` | publication-barrier state | worker-only under existing diagnostic rules |
| `timer_last_cloud_diagnostic_build_sequence_` | publication-barrier state | worker-only under existing diagnostic rules |
| `timer_last_deactivate_sequence_` | publication-barrier state | worker-only inactive transition |
| Candidate/Certified/DELETE dedupe fields | publication-barrier state | worker publication path; task barrier protects reset |
| tube-due timing samples | attempt-only measurement state | mutex-protected measurement vector; no control semantics |
| Runtime/recovery/pending authority fields | command-owned | `runtime_command_mutex_`; unchanged |
| `shutdown_requested_` | atomic lifecycle state | requestShutdown/shutdown/reset/worker checks |

The key single-writer rule is that every former timer-owned mutable field is
either worker committed state or job-local state. No command callback may
mutate those objects directly.

## 14. Currentness and obsolescence identity

`TubeWorkIdentity` must include the complete semantic set:

```text
task_generation
active/inactive state
source_revision
path revision
frame revision
authority_session
semantic_path_owner identity
frame_owner identity
base_path_tube_pair identity
base_path_tube_pair_generation
shutdown/reset/deactivate invalidation state
```

The semantic owner/frame identities must be compared directly, or the
repository must explicitly prove and test that every such replacement changes
source revision. An undocumented assumption is insufficient.

The following are not unconditional cancellation identities:

```text
ordinary control_sequence updates
ordinary command updates
ordinary newer map_observation_sequence
```

A Tube build may legitimately complete against the immutable cloud/map
snapshot carried by its request while a newer observation exists. The newer
observation becomes provenance for a later request; it does not automatically
invalidate the completed same-source epoch. Existing
`requestSourceStillCurrent()` and `epochMatchesRequest()` remain authoritative
fail-closed gates.

Owner/frame replacement is different: an old Candidate/Certified exposure may
not survive a replacement that leaves source revision numerically unchanged.
The implementation must invalidate or clear such exposure under the existing
publication boundary.

## 15. Ordinary job versus H2 pair refresh

The current distinction is intentional and must remain exact.

### Ordinary request

The job-local `TubeEpochManager` begins as a copy of the current committed
ordinary manager state. This preserves Candidate/Active history for ordinary
rolling epochs while preventing the running job from mutating committed state.

### Pair-refresh request

When `request->base_path_tube_pair` is present, the job must construct a fresh
config-only `TubeEpochManager`, matching current `buildTubeEpoch()` behavior.
Candidate/Active history must not be copied into pair refresh.

The existing regression must remain true:

```text
StaleBaseTimerEpochCannotChangeNewPairOrPersistentTimerManager
```

A stale pair-refresh completion must not replace a newer Pair, change the
persistent ordinary manager, or alter the persistent ordinary active profile.

## 16. `candidate_sequence` and `timer_build_sequence`

Stage 1A adopts the minimal four-file-compatible interpretation:

```text
candidate_sequence
    = committed Candidate sequence

timer_build_sequence
    = every actual worker Tube-build attempt
```

Consequences are explicit:

- a stale job may consume `timer_build_sequence`;
- a stale job-local manager increment may not commit `candidate_sequence`;
- a later committed Candidate may reuse the numeric candidate sequence that
  appeared only inside discarded job-local state;
- published committed profiles must continue to satisfy:

```text
profile.tube_revision
== profile.profile_revision
== status.candidate_sequence
```

Attempt identity and committed Candidate identity must be distinguishable in
diagnostics. `timer_build_sequence` is the measurement/attempt identity;
`candidate_sequence` remains the committed Candidate compatibility field.

If truthful diagnostics or existing external contracts require a unique
candidate sequence for every attempted build, implementation must stop and
request a separate `phase_offset_navigation` whitelist/API. Stage 1A must not
expand that scope silently.

## 17. Transactional job-local state and lifecycle

At worker start:

```text
committed worker state
    -> job-local copy
        -> heavy build mutates only job-local state
            -> currentness/shutdown check
                -> commit or discard
```

The job-local state must include at least:

```text
tube_epoch_manager_
cached_full_path_samples_
cached_path_source_revision_
have_cached_path_
timer_active_profile_
timer_installed_active_epoch_
timer_build_sequence allocation/result
latest_cloud_occupancy_query_status_
timer task-generation marker
timer diagnostic last-build/publish/delete counters
Candidate/Certified/DELETE dedupe state where touched by the job
```

Attempt timing and explicitly permitted raw/cloud provenance may be recorded,
but stale work may not mutate committed manager/profile/cache/epoch ownership
or publish Candidate/Active/Certified/new-task DELETE.

Heavy computation must hold none of:

```text
task_publication_mutex_
runtime_command_mutex_
worker-state mutex
```

The worker must release its worker-state mutex before entering finalization.
Finalization acquires `task_publication_mutex_` first, briefly commits worker
state, releases that state mutex, and only then takes any short Runtime/pair
lock required by the existing protocol. This avoids a worker-state to
publication inversion against reset/publication to Runtime ordering.

### Lifecycle transitions

### Reset/new navigation task

`resetForNewNavigationTask()` remains the task-boundary linearization point.
It must:

1. invalidate the task generation and worker identity;
2. notify the worker;
3. prevent old pending/running jobs from committing;
4. preserve existing Runtime reset/authority semantics;
5. allow the worker to rebuild clean task-local manager/profile/cache state.

It must not directly mutate a running job-local object.

### Deactivate

Deactivation remains semantically identical:

1. publish an immutable inactive request;
2. invalidate active work identity;
3. notify the worker;
4. let the worker perform the existing timer-side clear/DELETE transition;
5. gate DELETE by task/control identity so an old DELETE cannot erase a new
   task's visualization;
6. preserve Runtime retained delta/previous-final-port semantics.

### Shutdown

The conceptual `gvf_manager` destructor order remains:

```text
adapter->requestShutdown()
stop manager timers
adapter->shutdown()
```

`requestShutdown()` must set `shutdown_requested_`, invalidate pending work,
set worker stop state, and wake the condition variable.

`finalizeTubeEpoch()` must recheck shutdown under
`task_publication_mutex_` before diagnostics, Candidate, Certified, or DELETE
publication. A worker that began before shutdown may finish its current
computation in Stage 1A, but it cannot publish after the shutdown ownership
flip.

`shutdown()` must:

- never join while holding worker/publication/Runtime locks;
- join the one worker;
- only after join clear worker-facing state, snapshots, and publishers;
- be idempotent;
- leave no detached thread, UAF, or post-shutdown publication.

Because Stage 1A has no mid-SurfaceValidator cancellation, join latency may
approach one already-running build, currently approximately 2.186 s. This
latency must be measured rather than hidden.

## 18. Bootstrap and authority activation ordering

The frozen production fact is:

```text
advertise()
    -> execution_authority_.setTestOnlyRuntimeOwnerAllowed(false)
    -> requiresPathTubePairBootstrap() normally returns false
```

This must be asserted/tested explicitly.

When bootstrap is `NOT_REQUIRED`, a scheduler-only timer callback may retain
the existing no-op/not-required activation call without changing production
H2 behavior.

If a bootstrap-required path is ever active, activation must be coupled to a
successful worker completion/finalization, not run immediately after merely
queuing an asynchronous build. The callback must not silently change H2
activation timing.

## 19. Timer-period wiring

`PhaseOffsetMatchedAdapter::loadConfig()` already loads and validates:

```text
phase_offset/tube/update_period
```

`matched_config_` is assigned before the timer is created in the manager
constructor path. The minimum correction is:

```text
ros::Duration(matched_config_.tube_update_period)
```

in place of the current literal `ros::Duration(0.10)`.

This changes only scheduling configuration wiring. It does not require Tube
completion at 20 Hz or 10 Hz.

Required acceptance:

```text
configured 0.05 -> approximately 50 ms scheduling permit interval
configured 0.10 -> approximately 100 ms scheduling permit interval
```

Invalid configuration must retain the existing adapter-disabled behavior.

## 20. Strict four-file Stage 1A whitelist

The corrected Stage 1A implementation whitelist is exactly:

1. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
2. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
3. `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
4. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

No other file is authorized.

In particular, Stage 1A must not modify:

```text
phase_offset_navigation production files
TubeEpochManager source/header
CertifiedTubeBuilder
TubeBuilder
TubeSurfaceValidator
ValidationContext
gvf_switch_policy_test.cpp
CMakeLists.txt
launch/config files
Runtime
Recovery
H2
C2
Horizontal-N
R3 production semantics
```

If candidate-sequence reservation cannot be truthful within these four files,
stop rather than widening the whitelist.

## 21. Frozen semantics and forbidden changes

Stage 1A must not change:

- Horizontal-N specialization;
- `N` / `N_w` mathematics;
- `r = p + N delta`;
- `r_w`;
- exact-zero and tiny-nonzero authority semantics;
- Tube geometry or certification predicates;
- SurfaceValidator acceptance predicates;
- CertifiedTubeBuilder inward-search policy;
- ESDF safety thresholds;
- `back_w`, `lookahead_w`, `sample_step_w`;
- `environment_search_extent`, `ray_step`;
- `min_certified_forward_w`;
- R3 Candidate meaning;
- R3 Certified Geometry eligibility protocol;
- Runtime allocation/control law;
- Recovery;
- H2 authority;
- C2 construction;
- planner cadence;
- command cadence;
- `AsyncSpinner(8)`;
- topic namespaces;
- visualization geometry/colors.

Within the whitelisted adapter files, only worker scheduling, lifecycle,
transactional state plumbing, stale abandonment, period wiring, and their
tests may change.

## 22. Exact unit-test plan

Tests belong in the existing `phase_offset_matched_adapter_test.cpp` target.
No new target or CMake edit is authorized.

Required tests:

1. `SchedulerReturnsWithoutSynchronousTubeBuild`
2. `LatestOnlyCoalescesAThroughDToD`
3. `PendingSlotNeverExceedsOne`
4. `NoDuplicateRebuildWithoutNewPermitOrTubeIdentity`
5. `Normal50HzCommandTrafficCannotCause50HzTubeBuildChurn`
6. `RequestArrivingAtWorkerCompletionWaitsAtMostNextPermitAndDoesNotFIFO`
7. `StaleOrdinaryOwnerOrFrameCompletionCannotPublish`
8. `StaleOrdinaryJobCannotMutateCommittedWorkerState`
9. `StalePairRefreshCannotMutatePersistentManagerOrNewPair`
10. `TaskGenerationResetInvalidatesRunningJob`
11. `DeactivateDuringBuildProducesCorrectLifecycle`
12. `ShutdownJoinsAndPreventsPostShutdownPublication`
13. `TimerTickCannotSynchronouslyBuildWhenWorkerStarted`
14. `OrdinaryJobCopiesCommittedManagerButPairRefreshUsesFreshManager`
15. `CandidateSequenceTracksCommittedCandidateAndBuildSequenceTracksAttempts`
16. `R3RequestCandidateRequestProtocolUnchanged`
17. Existing exact-zero and tiny-nonzero regressions unchanged
18. Config period `0.05` / `0.10` scheduler behavior

Additional required assertions:

- owner/frame replacement with unchanged numeric source revision is rejected;
- unchanged latest request is not rebuilt repeatedly;
- stale ordinary jobs do not alter cache/profile/cloud status/counters;
- shutdown prevents all worker-side publication after invalidation;
- existing `StaleBaseTimerEpochCannotChangeNewPairOrPersistentTimerManager`
  semantics remain unchanged;
- `profile.tube_revision == profile.profile_revision ==`
  `status.candidate_sequence` for every committed Candidate;
- `timer_build_sequence` and committed Candidate sequence are distinguishable.

Regression targets after implementation, but not during this plan-only turn:

```text
catkin_make -j2
catkin_make run_tests_phase_offset_core
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_bspline_race
```

The complete Horizontal-N, Runtime, MatchedAdapter, Recovery, R3, C2/H2,
exact-zero, tiny-nonzero, and original planner regressions remain mandatory.

## 23. Real ROS acceptance plan

Use the known experiment:

```bash
roslaunch so3_quadrotor_simulator simulator.launch

roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_observe_only:=false \
  phase_offset_manual_tube_source:=esdf \
  phase_offset_tube_cloud_obstacle_set_complete:=true \
  phase_offset_tube_preincluded_map_uncertainty:=0.10
```

Measure:

```text
/particle0/gvf/traj_vis Hz
phase_offset_manual/tube_candidate Hz
phase_offset_manual/tube_certified_geometry Hz
command callback Hz
local_map Hz
```

Also measure:

```text
scheduling permit cadence
actual worker build-start cadence
completed Tube publication cadence
pending depth
attempted timer_build_sequence order
committed candidate_sequence order
request age at worker start
candidate age at publication
current_w - candidate_build_w
stale abandonment count
source-current rejection count
shutdown join latency
```

Acceptance is not “Tube must always be exactly 10 Hz.” It requires:

- pending depth never exceeds one;
- no FIFO B/C/D backlog;
- no 50 Hz worker rebuild churn;
- scheduler callback remains bounded and fast;
- stale work cannot mutate committed state;
- stale work cannot publish current Candidate/Certified;
- request age does not monotonically accumulate;
- Candidate/Certified temporal staleness materially improves over the ~2 Hz
  baseline;
- trajectory, command, and map behavior remains healthy;
- all Frozen semantics remain unchanged.

ROS logs, bags, CSV evidence, and `/tmp` directories are not repository
artifacts and must not be added to Git.

## 24. Known Stage 1A limitation

Stage 1A performs post-build abandonment and does not interrupt a build that
is already deep inside:

```text
TubeBuilder
SurfaceValidator
inward-search validation
```

One obsolete in-flight build may therefore still consume approximately the
current worst-case 2.186 s before being abandoned. Stage 1A is still
materially useful because it removes FIFO backlog, keeps the ROS callback
fast, and prevents stale completion from corrupting or publishing state.

Stage 1A can be checkpointed only after the corrected unit tests, regressions,
and real ROS acceptance pass.

## 25. Deferred Stage 1B

Stage 1B remains separately unauthorized.

Potential later scope:

```text
independent CANCELLED status
CertifiedTubeBuilder cancellation boundaries
TubeBuilder cancellation boundaries
TubeSurfaceValidator cancellation boundaries
ValidationContext propagation
ValidateCell recursion/query-loop checks
```

Cancellation must never be represented as ordinary validation failure. The
current `bool validate() == false` means geometry/safety failure and must not
be overloaded.

Stage 1B may require a separate navigation whitelist and separate Sol audit.

## 26. Latest-paper / future Batch C-D reconciliation note

This section is a future-plan note only. It does not amend the Frozen Plan,
authorize Batch C/D, or authorize implementation.

The latest finalized paper differs from the older Frozen Batch C/D concept.
Before Batch C authorization, Main must reconcile the following:

1. The paper uses the Horizontal-N specialization:

   ```text
   N = (e_z x p_w) / ||e_z x p_w||
   ```

   preserving planner altitude and allowing horizontal transverse motion.

2. The paper defines the retained raw Tube branch as the connected component
   containing `delta = 0`. This must be reconciled with older
   current-delta/Frozen wording before Batch C.

3. Preview is explicitly:

   ```text
   centerline arc-length s(w)
       -> approximately uniform Delta-s cross sections
       -> backward feasible-set recursion K_k
       -> conservative piecewise-C1 inner envelope K(s)
   ```

4. Batch C must be redesigned before authorization. The old joint allocator /
   moving-CBF / hard pairwise-CBF concept is not automatically valid.

5. The latest paper describes inter-agent interaction as soft spacing
   regulation, not a hard pairwise safety guarantee. Do not implement the old
   production pairwise-CBF Batch C without a new Main-window decision.

6. Batch D remains Cleanup / Observability / Experiment Interfaces, but future
   diagnostics should emphasize:

   ```text
   I(w), K(s), b_pre, beta,
   g_sep, g_coh, g_damp, g_des,
   u_w_nom, u_delta_nom,
   admissible U_w / U_delta^K,
   final u_w / u_delta,
   delta, dot w, v_s, matched feedforward
   ```

No Batch C, NoQP, gradient-representation, or Batch D implementation belongs
in Stage 1A.

## 27. Development governance and agent roles

```text
Main/Codex
    orchestration, specification, review, acceptance, stop decisions

Sol Max/xhigh
    independent read-only audit; no code or plan rewriting

Luna Max
    sole implementation writer, only after explicit authorization
```

Governance rules:

- never use multiple production writers;
- no implementation merely because a plan exists;
- no stage/commit/push before acceptance unless explicitly authorized;
- no silent whitelist expansion;
- preserve all user-owned worktree changes;
- any post-freeze technical change must be deliberate and independently
  reviewed.

## 28. Exact next action

```text
main window reviews this corrected specification;
if all required Sol corrections are resolved,
obtain/perform final readiness confirmation;
only then authorize exactly one Luna implementation writer for Stage 1A.
```

Implementation is not authorized by this document.

## 29. STOP conditions

Stop and report without expanding scope if any of the following occurs:

- current branch, HEAD, or baseline differs materially;
- an edit is required outside the four-file whitelist;
- `TubeEpochManager` copyability is not compile-safe;
- candidate-sequence semantics cannot be truthful within four files;
- owner/frame identity cannot be validated;
- worker state cannot be made single-writer;
- a lock inversion or join-with-lock dependency appears;
- bootstrap-required production behavior is discovered;
- any Frozen/R3/Runtime/H2/C2/Horizontal-N semantic change is needed;
- existing unrelated worktree content would need to be touched;
- ROS acceptance requires changing cadence, safety thresholds, or topics;
- Stage 1B cancellation is being smuggled into Stage 1A.

## 30. Repository hygiene snapshot

The expected post-handoff state is:

```text
branch: main
HEAD:   5355be736339a650432280deec53362f61749a43
```

Existing entries remain:

```text
 M AGENTS.md
?? .codex
?? Testing/
?? docs/D-ISFGVF_PhaseOffset_Tube_Architecture_Refactor_Brief_2026-08-23.md
?? src/swarm_planner/bspline_traj/test/__pycache__/
?? src/uav_simulator/so3_quadrotor_simulator/test/__pycache__/
```

The only new entry from the authorized handoff turn is:

```text
?? docs/D-ISFGVF_Tube_Freshness_Stage1_Handoff_2026-08-26.md
```

Required hygiene checks:

```text
git diff --check: must pass
git diff --cached --name-only: empty
```

The handoff remains untracked and uncommitted. No implementation, staging,
commit, push, or next-stage action follows this document.

STOP.
