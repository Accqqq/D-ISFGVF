# D-ISFGVF P1 bounded Tube marker-lifecycle repair plan

    DOCUMENT_ROLE=BOUNDED_REPAIR_EXECUTION_PLAN
    DOCUMENT_STATUS=CANDIDATE_PENDING_FRESH_SOL_XHIGH_AUDIT
    DATE=2026-08-29
    PRODUCTION_BASELINE=0fb998966bab78f8e555aca49e1b4189519462be
    ROOT_CAUSE=PATH_FRAME_OWNER_REPLACEMENT_WITHOUT_OLD_MARKER_RETIREMENT
    HALF_VOXEL_REVERT=NOT_AUTHORIZED
    HALF_VOXEL_CHANGE_RELATION=PREEXISTING_VISUALIZATION_OR_LIFECYCLE_BUG
    BATCH_C_D=PAUSED

## 1. Purpose and stop boundary

This plan authorizes one future bounded implementation that repairs only the
same-task MANUAL Tube marker lifecycle after an ordinary path/source/frame
owner replacement.

The repair ends when the old owner has lost marker publication authority and
the timer/publication owner has emitted the required all-DELETE bundles. It
does not redesign Stage 1A scheduling, currentness, Candidate construction,
Runtime, Recovery, H2, R3 eligibility, Tube geometry, or certification.

This planning turn modifies no production or test source. It launches no Luna
and performs no stage, commit, or push.

## 2. Accepted runtime evidence

The accepted live event is:

| Time | Planner revision | Worker attempt | Final currentness | Visible candidate |
|---|---:|---:|---|---|
| 1787933228.374 | 30 | 63 | accepted | ADD `224513ce08f963ad` |
| 1787933228.616 | 31 | - | - | old revision 30 remains |
| 1787933229.575 | 31 | 64 | rejected | old revision 30 remains |
| 1787933230.190 | 32 | - | - | old revision 30 remains |
| 1787933230.618 | 32 | 65 | rejected | old revision 30 remains |
| 1787933231.670 | 32 | 66 | accepted | ADD `2121a9138b7c59d6` |

The stale visible interval was 3.296 seconds. Map and planner state advanced.
ZERO_BASELINE was not involved. Stage 1A correctly rejected obsolete worker
completions.

## 3. Source facts controlling the repair

The existing ordinary command branch in
`PhaseOffsetMatchedAdapter::update()` detects a changed source/path/frame owner
and clears `latest_candidate_epoch_snapshot_` and `latest_epoch_snapshot_`.
It does not establish a marker-retirement publication.

Production `gvf_manager::phaseOffsetTubeTimerCallback()` is the 10 Hz timer
owner and calls `PhaseOffsetMatchedAdapter::scheduleTubeBuild()`. Heavy Tube
construction remains on the single joined worker.

`finalizeTubeEpoch()` owns completion publication under
`task_publication_mutex_`, then takes `runtime_command_mutex_` for the final
source/currentness decision. `requestSourceStillCurrent()` must remain
unchanged and fail closed.

`publishManual()` returns without ordinary `/tube` or `/tube_candidate`
publication when its command snapshot is not exact-current. Its stale branch
may make one R3 decision only after a worker completion; it does not provide a
bounded no-completion retirement path.

All three Tube marker namespaces use stable IDs 0/1/2. Their ADD markers leave
`lifetime` at the ROS default zero. This is correct only when loss of owner
authority explicitly produces DELETE.

## 4. Topic classification

### `/formation_planning/phase_offset_manual/tube_candidate`

    CLASSIFICATION=AFFECTED_BY_SAME_DEFECT

This is the reproduced gray marker. Source/frame replacement clears the
in-memory Candidate but does not publish DELETE. Stale completion rejection
then leaves the last ADD visible.

### `/formation_planning/phase_offset_manual/tube`

    CLASSIFICATION=AFFECTED_BY_SAME_DEFECT

It uses the same ordinary completion-owned publication boundary, stable IDs,
and zero lifetime. The stale `publishManual()` branch deliberately does not
publish this topic. Therefore a previously certified ADD can remain visible
after the same owner replacement until another qualifying publication.

### `/formation_planning/phase_offset_manual/tube_certified_geometry`

    CLASSIFICATION=AFFECTED_BY_SAME_DEFECT

Its request/Candidate/request eligibility predicate is already fail-closed and
must not change. However its all-DELETE bundle is still sent only by a later
`publishManual()` or full lifecycle delete. If no worker completes after an
ordinary owner replacement, the last R3 ADD has the same unbounded lifetime.
The repair therefore includes only an owner-retirement all-DELETE decision for
this topic; the R3 eligibility predicate and ADD path remain unchanged.

No base-path, active-path, frame, map, trajectory, or new topic is included.

## 5. Frozen repair mechanism

### 5.1 Command-side responsibility

The ordinary no-pair branch of `PhaseOffsetMatchedAdapter::update()` remains
the source/frame replacement linearization point.

When its existing replacement predicate is true, while
`runtime_command_mutex_` is already held, it shall:

1. preserve the existing Candidate/Epoch atomic clears;
2. replace the single pending marker-retirement token with the identity of the
   newly current request;
3. publish no ROS message;
4. perform no worker scheduling or construction;
5. retain all currentness and Runtime behavior.

The pending token has exactly one meaning:

> The current ordinary MANUAL request has replaced the owner that could have
> published the currently visible Tube marker namespaces, so those old
> namespaces require retirement before any ADD owned by the replacement.

The token is latest-only. Several replacements before the timer consumes it
may overwrite it with the newest owner identity because one DELETE bundle
retires every older visible owner using the stable namespace/ID set.

### 5.2 Token identity

Reuse the existing `TubeWorkIdentity` fields rather than inventing a second
revision system. The queued identity must cover:

- task generation;
- active state;
- source revision;
- path revision;
- frame revision;
- authority session;
- semantic path owner pointer;
- frame owner pointer;
- base PathTubePair pointer/generation where applicable;
- shutdown invalidation state.

The stored state is one `TubeWorkIdentity` plus one pending boolean, protected
by `runtime_command_mutex_`. It contains no profile, marker cache, map object,
worker handle, or Runtime state.

Control sequence and map sequence must not be used to invalidate this pending
retirement. Same-owner 50 Hz command/map refreshes after replacement must not
starve the DELETE. A later owner replacement overwrites the identity.

### 5.3 Publication ownership

The exact production owner of the bounded no-completion retirement is:

    gvf_manager::phaseOffsetTubeTimerCallback()
      -> PhaseOffsetMatchedAdapter::scheduleTubeBuild()
      -> pending marker-retirement consumption/publication

`scheduleTubeBuild()` shall perform the small marker-retirement decision before
the worker-slot scheduling decision. It must not build a Tube synchronously.

The retirement decision takes locks in the frozen order:

    task_publication_mutex_
      -> runtime_command_mutex_

Under the runtime lock it loads the latest request and consumes the token only
if the token matches the latest active work identity. It then releases the
runtime lock while retaining `task_publication_mutex_`, and publishes the
three all-DELETE bundles. It releases the publication lock before taking
`worker_state_mutex_` for ordinary scheduling.

Therefore command-thread direct DELETE is not required.

### 5.4 Completion-before-timer ordering

A valid replacement worker completion may win the race against the next timer
callback. `finalizeTubeEpoch()` already owns `task_publication_mutex_` and
`runtime_command_mutex_` at its final currentness boundary.

After `requestSourceStillCurrent()` succeeds, and before any replacement ADD
can be published, finalization shall consume a matching pending retirement
token. After the Candidate/Epoch stores and release of
`runtime_command_mutex_`, but while retaining `task_publication_mutex_`, it
shall publish DELETE before its normal `publishManual()` ADD decision.

This path is an ordering optimization, not the latency guarantee. The timer
path remains independently sufficient when no worker succeeds or completes.

### 5.5 Retirement marker construction

Add one adapter-local helper in `phase_offset_matched_adapter.cpp` that
publishes only:

- `MakeCandidateTubeMarkers(..., empty_profile, false)`;
- `MakeCertifiedTubeMarkers(..., empty_profile, false)`;
- `MakeCertifiedGeometryTubeMarkers(..., empty_profile, false)`.

All bundles must contain DELETE for IDs 0/1/2 in their existing namespaces.
The helper must not delete base_path, active_path, or frame markers and must not
call `publishManualDelete()`, whose scope is intentionally wider.

Use the latest matching request stamp selected at retirement consumption. Do
not change marker lifetime, IDs, namespaces, colors, geometry construction, or
`phase_offset_tube_markers.cpp`.

### 5.6 Lifecycle clearing

Pending ordinary retirement state must be cleared, without a new ordinary
publication, by lifecycle paths that already own broader deletion or no
longer allow publication:

- `resetForNewNavigationTask()` after/beside its existing full DELETE;
- `retirePathTubeAuthorityLocked()`;
- `deactivateLocked()` because the inactive timer request owns the full
  DELETE;
- `consumeTimerTaskGeneration()`;
- `shutdown()` / shutdown invalidation.

Existing reset/deactivate/task-generation publication behavior remains the
authority. The new token must never survive into a later task or reactivation.

## 6. Concurrency proof obligations

### A. Old owner retirement

Once replacement update has stored the new request and queued its matching
retirement token, the old Candidate/Epoch pointers are cleared. No old owner
can remain marker-authoritative. The next timer publication decision consumes
the token without requiring a build.

### B. Stale completion

An old completion fails the unchanged `requestSourceStillCurrent()` gate
before Candidate/Epoch storage and normal publication. It is not allowed to
consume the new owner's retirement token and cannot ADD old geometry.

### C. No stale DELETE of the new owner

Retirement consumption is serialized with completion publication by
`task_publication_mutex_` and requires equality with the latest current work
identity. If the current replacement completion wins, it consumes and emits
DELETE before its ADD. No matching token remains after the ADD. A delayed old
completion therefore cannot delete the new marker.

### D. New owner ADD

After retirement consumption, a valid current replacement completion follows
the unchanged Candidate/Epoch store and `publishManual()` logic and may ADD
normally.

### E. No resurrection

Old completion is rejected by the unchanged source/path/frame/owner checks.
The only later ADD is based on a current immutable Candidate. No retry, stale
fallback, Pair fallback, or cached marker geometry is introduced.

### F. Reset/deactivate/shutdown

Existing task publication ownership is retained. The token is cleared at
those lifecycle boundaries and cannot cross generation/session ownership.

## 7. Latency contract

For an ordinary same-task source/path/frame replacement:

    OLD_MARKER_RETIREMENT_LATENCY
      <= configured tube_update_period + normal ROS callback scheduling jitter

For the accepted configuration this is normally at most one 10 Hz timer
period plus jitter. A valid worker completion may retire sooner, but success or
completion is not required.

No command-thread zero-latency promise is made.

## 8. Exact implementation whitelist

Future Luna may modify exactly:

1. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
2. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
3. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

The header is authorized only for:

- the pending retirement identity/boolean;
- small private queue/consume/publish helper declarations;
- at most one passive test-only retirement-publication observer hook if the
  deterministic tests cannot otherwise observe the exact DELETE bundle.

No fourth implementation/test path is authorized.

## 9. Protected files and semantics

The following remain unchanged:

- `phase_offset_tube_markers.cpp` and its header;
- `gvf_manager.cpp` and timer period wiring;
- `tube_surface_validator.cpp` and its tests;
- all `phase_offset_navigation` production sources;
- TubeBuilder, TubeFilter, CertifiedTubeBuilder, SurfaceValidator;
- `requestSourceStillCurrent()` and `epochMatchesRequest()` predicates;
- Stage 1A worker, latest-only slot, currentness, task generation, and
  lifecycle architecture;
- R3 eligibility, immutable provenance, request/Candidate/request protocol,
  all-ADD/all-DELETE construction, and no-fallback rule;
- Runtime, Recovery, Handoff, H2, C2, planner, map, launch files, parameters,
  diagnostic schemas, and ROS topic names;
- marker lifetime, ID, namespace, color, and geometry;
- the accepted half-voxel tightening at `0fb998...`;
- Batch C/D.

Do not accept stale completions, add a finite marker lifetime, publish from the
command update, add a timer/worker/FSM/retry/cache/topic, or alter safety
mathematics.

## 10. Deterministic regression tests

All tests belong in `phase_offset_matched_adapter_test.cpp` and must run without
a ROS master.

### 10.1 Exact A_ADD -> A_DELETE -> B_ADD trace

Create a current owner A and complete a Candidate whose marker bundle is ADD.
Record `A_ADD` only after verifying the exact candidate geometry is A-owned.

Replace A with a distinct owner/frame/source B. Do not build or complete B.
Invoke one timer/scheduler publication decision. Assert:

- the pending token matched B and was consumed;
- candidate, certified, and R3 retirement bundles are all DELETE for IDs
  0/1/2;
- the DELETE occurred before any B completion;
- no base/active/frame delete is part of this helper;
- a second timer decision does not duplicate the same retirement.

Finalize a captured stale A build and assert false, no A epoch store, no ADD,
and no second retirement publication.

Then complete current B and verify its immutable B marker bundle is ADD. The
test-owned logical trace must be exactly:

    A_ADD
    A_DELETE
    B_ADD

It must reject:

    A_ADD -> silence -> B_ADD

and:

    A_ADD -> A_DELETE -> stale_A_ADD -> B_ADD

### 10.2 Forced inflight-A replacement race

Reuse the existing finalization hooks and bounded condition-variable pattern:

1. A has a visible ADD and a second A job is inflight;
2. pause A before the final source/currentness decision;
3. update B so the replacement linearizes and queues retirement;
4. start the timer retirement decision;
5. release A;
6. assert A finalization fails currentness and cannot consume B's token;
7. assert the timer publication owner emits DELETE independently of B build;
8. complete B and verify B ADD.

The test must have finite condition-variable waits consistent with existing
race fixtures and must join every thread.

### 10.3 B-before-delayed-A safety

After B replacement, allow the current B completion path to consume the token,
emit DELETE, and establish B Candidate ADD before invoking a delayed stale-A
completion/publication attempt. Assert:

- the token is already consumed;
- stale A returns false;
- no additional DELETE occurs;
- the latest Candidate/Epoch still belongs to B;
- B marker geometry remains ADD and bitwise/identity-distinct from A.

### 10.4 Rapid B-to-C coalescing

Replace A with B and then B with C before a timer decision. Assert the single
pending token names C, one DELETE retires the visible namespace, no B/C build
is required for DELETE, and only a later valid C completion can ADD.

### 10.5 Lifecycle and R3 regressions

Retain and extend focused assertions that:

- reset/deactivate emits its existing full delete only and clears the token;
- task-generation reset cannot leak a token;
- shutdown prevents post-shutdown publication;
- stale old inactive/delete paths cannot remove a reactivated owner;
- all existing R3 eligibility and final-request race tests remain unchanged;
- retirement produces one R3 all-DELETE bundle without changing the R3 ADD
  predicate;
- same-owner command/map updates never queue retirement;
- frame-owner replacement with unchanged numeric source revision does queue
  retirement.

If exact publication observation requires a passive test hook, it may observe
only the retirement bundle and work identity. It must not influence production
control flow, locking, eligibility, or marker contents.

## 11. Main implementation review gate

After future Luna implementation, Main must independently verify:

- changed paths are exactly the three whitelist paths;
- command/update performs no ROS publication;
- `requestSourceStillCurrent()` is byte/semantic unchanged;
- the timer callback remains scheduler/publication-only and performs no Tube
  build;
- lock order remains publication -> runtime -> worker where nested;
- retirement token has only the frozen meaning and no marker/profile cache;
- DELETE cannot follow a newer ADD for the same current owner;
- stale A cannot ADD or DELETE B;
- R3 eligibility and ADD logic are unchanged;
- marker lifetime and marker construction source are unchanged;
- half-voxel tightening remains present;
- `git diff --check` passes;
- no stage, commit, or push occurs.

## 12. Build and regression protocol

Use a new isolated worktree based exactly on
`0fb998966bab78f8e555aca49e1b4189519462be`. Do not implement in production
main.

From the isolated workspace root:

```bash
source devel/setup.bash
catkin_make -j2
catkin_make run_tests_phase_offset_core
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_bspline_race
```

Run the named adapter binary directly and preserve its complete result:

```text
devel/lib/bspline_race/phase_offset_matched_adapter_test
```

Required focused status:

- all new marker-lifecycle tests pass;
- all existing Stage 1A scheduler/currentness/race tests pass;
- all existing R3 tests pass;
- no new failure in Runtime, Recovery, H2/C2, TubeBuilder, Validator, or the
  accepted half-voxel tests;
- inherited unrelated baseline failures remain classified separately and are
  not repaired outside the whitelist.

## 13. Real ROS acceptance protocol

Repeat the accepted split launch using the isolated repaired build:

```bash
roslaunch so3_quadrotor_simulator simulator.launch
```

```bash
roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_observe_only:=false \
  phase_offset_manual_tube_source:=esdf \
  phase_offset_manual_tube_cloud_obstacle_set_complete:=true \
  phase_offset_tube_preincluded_map_uncertainty:=0.10 \
  phase_offset_measurement_enable:=true \
  phase_offset_measurement_tube_due_csv_path:=<run>/tube_due.csv
```

Use the same five-goal natural replan scenario that produced the 3.296 second
window. Record at minimum:

- `/particle0/path`;
- `/particle0/gvf/traj_vis`;
- `/sim/local_map`;
- `/formation_planning/phase_offset_manual/tube`;
- `/formation_planning/phase_offset_manual/tube_candidate`;
- `/formation_planning/phase_offset_manual/tube_certified_geometry`;
- raw/cloud/epoch/manual diagnostics;
- `/rosout`;
- `tube_due.csv`.

For every source/frame replacement following a visible Tube ADD, record:

- replacement timestamp and planner path revision;
- old marker namespace/IDs/action/hash;
- first DELETE action and timestamp;
- worker attempt/completion/currentness around the replacement;
- first new ADD timestamp/hash if one exists;
- any later occurrence of the old hash.

Acceptance requires:

1. natural path owner replacement occurs;
2. stale worker completions remain rejected;
3. visible action order is `OLD ADD -> DELETE -> NEW ADD`, or
   `OLD ADD -> DELETE` when no replacement completes;
4. DELETE does not wait for successful replacement construction/completion;
5. `STALE_MARKER_RETENTION_DURATION <= tube_update_period + measured normal
   timer callback jitter`;
6. no old geometry hash reappears after DELETE;
7. no stale DELETE removes a newer current-owner ADD;
8. `/tube`, `/tube_candidate`, and R3 each satisfy their classified lifecycle
   without changing R3 eligibility;
9. all five goals reach or retain the accepted bounded success classification;
10. command, planner, map, Runtime, authority, and navigation remain healthy;
11. no half-voxel, safety, scheduling, currentness, marker-lifetime, or control
    semantic changes are present.

The run is diagnostic evidence only. Do not add bags, CSVs, logs, or `/tmp`
artifacts to Git.

## 14. Stop conditions

Stop without redesign if:

- the repair requires command-thread ROS publication;
- the timer cannot retire the marker within one period without synchronous
  Tube construction;
- correct serialization would require changing Stage 1A currentness;
- a stale completion must be accepted to obtain the trace;
- R3 eligibility, Pair/Active fallback, Runtime, Recovery, H2, or safety
  semantics must change;
- a fourth implementation/test path is required;
- marker lifetime or marker construction must change;
- a new implementation-caused regression appears;
- production main, user-owned dirty files, or staging would be modified.

## 15. Frozen reporting fields for the future implementation turn

```text
PLAN_MODE=BOUNDED_TUBE_MARKER_LIFECYCLE_REPAIR
PRODUCTION_BASELINE=0fb998966bab78f8e555aca49e1b4189519462be
ROOT_CAUSE=PATH_FRAME_OWNER_REPLACEMENT_WITHOUT_OLD_MARKER_RETIREMENT
STALE_COMPLETION_REJECTION_CORRECT=true
MARKER_LIFETIME_CHANGE_REQUIRED=false
COMMAND_THREAD_DIRECT_DELETE_REQUIRED=false
PUBLICATION_OWNER=gvf_manager::phaseOffsetTubeTimerCallback -> PhaseOffsetMatchedAdapter::scheduleTubeBuild, serialized with finalizeTubeEpoch
OLD_MARKER_RETIREMENT_TRIGGER=ordinary same-task source/path/frame work-identity replacement
OLD_MARKER_RETIREMENT_LATENCY_CONTRACT=one configured tube_update_period plus normal callback scheduling jitter
STALE_COMPLETION_CAN_READD=false
STALE_COMPLETION_CAN_DELETE_NEW_OWNER=false
AFFECTED_TOPICS=/formation_planning/phase_offset_manual/tube_candidate,/formation_planning/phase_offset_manual/tube,/formation_planning/phase_offset_manual/tube_certified_geometry
REPAIR_FILES=phase_offset_matched_adapter.h,phase_offset_matched_adapter.cpp
TEST_FILES=phase_offset_matched_adapter_test.cpp
DETERMINISTIC_TRACE_REQUIRED=A_ADD -> A_DELETE -> B_ADD
HALF_VOXEL_REVERT=false
STAGE1A_CURRENTNESS_CHANGED=false
R3_CONTRACT_CHANGED=false
BATCH_C_D=PAUSED
CODE_MODIFIED=false
COMMIT_PUSH=false
```

## 16. Candidate status

Main source review finds the mechanism implementable within the exact three
file whitelist and the existing publication/runtime lock order.

    MAIN_PLAN_REVIEW=PASS

This candidate is not frozen until exactly one fresh Sol xhigh returns the
required acceptable verdict.
