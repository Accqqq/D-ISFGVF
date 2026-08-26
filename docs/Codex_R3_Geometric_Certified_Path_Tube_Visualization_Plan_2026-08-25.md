# R3 — Geometric Certified Path-Tube Visualization Execution Plan

```text
DOCUMENT_ROLE=FROZEN_EXECUTION_PLAN
DOCUMENT_STATUS=FROZEN / IMPLEMENTATION NOT AUTHORIZED
DATE=2026-08-25
PLAN_OWNER=CURRENT_MAIN_CODEX_FINAL_REVIEW_GATE
IMPLEMENTATION_AGENT=NONE
LUNA_AUTHORIZED=false
BATCH_C_AUTHORIZED=false
COMMIT_AUTHORIZED=false
PUSH_AUTHORIZED=false
MAIN_PLAN_REVIEW=PASS
SOL_PLAN_AUDIT=PLAN_ACCEPTABLE
SOL_AUDITED_PRE_FREEZE_SHA256=9754789f8da3035ada9c1505de9e18130885fc4e8eec5d10c10936b3f000307c
FREEZE_METADATA_ONLY_TRANSITION=true
```

## 0. Governance and repository baseline

Repository:

```text
Accqqq/D-ISFGVF
```

Planning baseline at document creation:

```text
workspace: /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
branch: main
HEAD: 7f162b3d3e019971821db1d209abe0d820a9f4b5
upstream: d_isfgvf/main
checkpoint message: refactor: complete batch B post-acceptance tube repairs
```

Accepted state:

- R1 / D3: **ACCEPTED**
- R2 / D1: **ACCEPTED**
- D2: **PRODUCTION NO CHANGE / REGRESSION PASS**
- Batch C: **NOT AUTHORIZED**
- R3 implementation authorization: **NONE**
- Luna: **NOT AUTHORIZED**

This document was persisted as a plan candidate. The final Main/Codex review
rechecked the actual branch, HEAD, upstream, worktree, staged state and relevant
HEAD source, then returned PASS. One new independent read-only
`gpt-5.6-sol/max` PLAN AUDIT judged the exact pre-freeze bytes and returned
`PLAN_ACCEPTABLE`. The plan is therefore FROZEN. Implementation still requires
separate explicit user authorization.

Existing unrelated worktree contents are user-owned. They must not be staged,
cleaned, restored, deleted, reformatted or attributed to R3. At document
creation, known unrelated paths included `AGENTS.md`, `.codex/`, `Testing/`, a
user architecture brief, logs and `__pycache__/` directories. The future
implementation context must inspect the real worktree rather than assuming
this list remains complete.

## 1. Purpose and paper semantics

R3 adds visualization only for the paper-defined geometric Path-Tube. For path
phase `w`, the admissible transverse interval is

```text
I_i(w) = [delta_lower_i(w), delta_upper_i(w)]
```

and the physical swept set is

```text
T_i = { r_i(w, delta) | delta in I_i(w) }.
```

The geometric Path-Tube means where the PhaseOffset state `delta` is
geometrically allowed to vary according to the already-computed ESDF, safety
margin, regularity, Filter and continuous-validation evidence. It is distinct
from whether PhaseOffset currently owns or qualifies for execution.

R3 adds exactly one relative MANUAL ROS topic:

```text
phase_offset_manual/tube_certified_geometry
```

The typical resolved name under the current node namespace is:

```text
/formation_planning/phase_offset_manual/tube_certified_geometry
```

Its exact meaning is:

```text
latest completed
+ current-task/current-source
+ immutable provenance-safe
+ ESDF/safety-certified
+ filtered geometric Candidate Path-Tube
```

It does **not** mean:

- Runtime NORMAL or SAFETY_PRIORITY;
- Active Tube ownership;
- ExecutionAuthority;
- selected-u or selected control owner;
- matched, projection or execution-output validity;
- PathTubePair ownership;
- Batch C activity;
- the paper's Preview-feasible envelope `K_i(s)`.

The current implementation does not expose a faithful immutable sampled
representation of `K_i(s)`. R3 must not approximate it from the current
reachable bounds and must not add a Preview-envelope topic.

## 2. Existing topic semantics remain frozen

R3 does not redefine any existing topic:

| Topic | Frozen meaning |
|---|---|
| `/phase_offset_manual/base_path` | planner centerline |
| `/phase_offset_manual/active_path` | current-delta reference curve |
| `/phase_offset_manual/tube_candidate` | raw / uncertified Candidate build evidence |
| `/phase_offset_manual/tube` | historical G18 execution-qualified Active Tube |
| `/phase_offset_manual/tube_certified_geometry` | new R3 filtered ESDF-certified geometric Candidate Tube |

The existing `/tube_candidate` same-build exact-anchor behavior and
`ExactPhase()` semantics remain unchanged. The existing `/tube` continues to
use the unchanged G18 publication integrity and `tubeDisplayCertified()`
execution qualification.

## 3. Sole production geometry owner

The sole normal production geometry owner for the new topic is:

```text
latest_candidate_epoch_snapshot_
  -> candidate_profile
  -> candidate_profile.samples
  -> filtered_lower / filtered_upper
```

There is no fallback to:

- `PathTubePair::epoch_snapshot`;
- `pair->active_profile`;
- any `active_profile`;
- exact-live-Pair state;
- `control.output.candidate_profile` or a masked Candidate copy;
- `raw_build_samples`;
- `ControlPublishSnapshot::current_w`;
- Runtime or ExecutionAuthority state;
- selected port or selected owner;
- a second path, ESDF or map query.

If no current provenance-safe authoritative Candidate epoch exists, the new
topic publishes DELETE.

A Candidate slot never establishes its own currentness. Current source
authority comes from the immutable request published in
`latest_build_request_`, and every geometric publication decision must bind the
Candidate to a stable current request identity as specified in Section 7.

## 4. Candidate-only and execution independence

`control.candidate_only` is not a geometric veto. It describes whether an
epoch can cross the current Runtime/Active execution boundary; it is not a
statement that the original immutable Candidate geometry is stale.

An otherwise valid geometric Candidate must not be hidden solely because of:

- `observe_only=true` or `observe_only=false`;
- Runtime being passive or denied;
- no execution owner;
- `selected=false`;
- matched/projection/output being inactive solely because execution is
  unavailable;
- forward-horizon execution qualification;
- tracking execution qualification;
- Active installation/crossing state;
- `control.candidate_only == true`.

The production decision loads the unmasked
`latest_candidate_epoch_snapshot_` independently. R3 reads geometric evidence
only and never participates in navigation or control authority.

## 5. Map sample-and-hold and newer-completion precedence

The new topic means:

```text
latest completed provenance-safe ESDF-certified geometric Candidate
for the current task/source
```

It does not mean geometry already certified against the latest requested map
sequence.

A newer **request** map sequence alone does not invalidate the last completed
same-task/same-source Candidate. That completed immutable Candidate may remain
sample-and-held and visible while a newer build is pending or while the newer
request cannot cross Runtime.

Once a newer Candidate completion for the current task/source is published to
`latest_candidate_epoch_snapshot_`, it becomes the sole authoritative
completion:

- valid `OFFSET_CERTIFIED` completion: ADD the new filtered geometry;
- `ZERO_ONLY_PLANNER_BASELINE`: DELETE;
- incomplete or invalid completion: DELETE;
- never fall back to an older completed Candidate.

For R3, stale map provenance means internal Candidate/epoch/status provenance
is zero, mutable or mutually inconsistent. It does not mean merely that a
newer request map sequence exists.

## 6. Stateless geometric certification predicate

R3 adds a private stateless geometric eligibility/currentness path. It must not
reuse `epochMatchesRequest()`, because that function includes Active/current
cloud execution-crossing conditions.

ADD is allowed only when all of the following hold.

### 6.1 Adapter and source

- adapter mode is MANUAL;
- configured Tube source is ESDF.

### 6.2 Epoch and current task/source

- epoch is active;
- epoch task generation equals the stable current request task generation;
- task generation equals the current adapter task generation;
- epoch source revision is nonzero;
- epoch source revision equals the stable current request source revision;
- `epoch.request_control_sequence <= request.control_sequence`;
- Candidate path source revision equals the epoch/request source revision;
- request semantic path owner and immutable frame owner are present and
  current;
- `FrameMatchesRevision(request.frame_owner, request.source_revision)` remains
  true.

### 6.3 Immutable map provenance

- epoch map observation sequence is nonzero;
- epoch map observation is an immutable snapshot;
- status reports the same immutable snapshot fact;
- status Candidate map sequence equals epoch map sequence;
- profile `snapshot_provenance_is_immutable == true`;
- `profile.map_revision == profile.snapshot_sequence`;
- profile/status/epoch map provenance identifies the same frozen build.

No equality with the latest request map sequence is required.

### 6.4 Profile source, completion and classification

- `profile.source == ESDF`;
- profile source revision equals epoch/status Candidate path source revision;
- profile raw, filtered and final completion fields are all true;
- status Candidate raw, filtered and final completion fields are all true;
- profile/status completion fields are mutually consistent;
- profile classification is `OFFSET_CERTIFIED`;
- status Candidate classification is `OFFSET_CERTIFIED`;
- profile `zero_only == false`;
- status Candidate zero-only is false;
- `profile.obstacle_certified == true`.

No new Candidate obstacle-certified diagnostics/schema field is added. The
existing profile field is the obstacle certificate fact.

### 6.5 Revisions

- `profile.source_revision == epoch.source_revision ==
  status.candidate_path_source_revision != 0`;
- `profile.path_revision == request.frame_owner->pathRevision() ==
  request.source_revision`, nonzero;
- `profile.frame_revision == request.frame_owner->frameRevision()`, nonzero;
- `profile.tube_revision == profile.profile_revision ==
  status.candidate_sequence`, nonzero;
- `profile.map_revision == profile.snapshot_sequence ==
  epoch.map_observation_sequence ==
  status.candidate_map_observation_sequence`, nonzero.

Do not compare `full_path_samples` revisions. Ordinary full-path samples may
retain compatibility revisions that are not the immutable frame revisions.
R3 validates only the displayed profile samples and current request frame
identity. It does not reconstruct or re-evaluate the path.

### 6.6 Displayed samples

- at least two samples exist;
- every displayed sample is complete;
- sample `w`, `p`, `N`, `filtered_lower` and `filtered_upper` are finite;
- constructed lower/upper boundary points are finite;
- `filtered_lower <= filtered_upper`;
- sample path/frame revisions equal the profile path/frame revisions;
- sample `w` values are strictly increasing;
- duplicate or decreasing `w` is rejected.

The displayed geometry comes only from `candidate_profile.samples` and its
`filtered_lower/filtered_upper`. `raw_build_samples` must never supply
certified geometry.

`ZERO_ONLY_PLANNER_BASELINE`, NONE, incomplete, non-ESDF, invalid obstacle
certificate, invalid provenance, nonfinite, unordered, duplicate or
fewer-than-two-sample profiles produce DELETE. The predicate is display-only
and cannot mutate or weaken Tube construction, certification or lifecycle.

## 7. Atomic Candidate/request publication linearization

R3 adopts atomic Candidate/request publication linearization. It does not
restructure the command critical section or add the
`task_publication_mutex_ -> runtime_command_mutex_` lock order around ordinary
source replacement.

### 7.1 Current-source authority

`latest_build_request_` is current-source authority for geometric publication.
The Candidate slot alone is never authoritative.

### 7.2 Timer-owned decision protocol

At each timer-owned certified-geometry publication decision:

1. Atomically load request A from `latest_build_request_`.
2. Atomically load Candidate from `latest_candidate_epoch_snapshot_`.
3. Atomically load request B from `latest_build_request_`.
4. Require immutable shared-pointer identity `A == B`.
5. Require request B active, at the current task generation and current source
   authority.
6. Require Candidate task/source/provenance to match request B.
7. Apply the full Section 6 geometric predicate.
8. Immediately before choosing ADD, atomically reread
   `latest_build_request_`.
9. Require the final request pointer to remain exactly request B.

The successful final immutable request-identity validation is the R3
certified-geometry publication linearization point.

If any request identity, task, source or Candidate provenance check changes,
the result is DELETE. The decision never retries or searches for another
Candidate.

### 7.3 Source-replacement race

During ordinary same-task source replacement, command/update code may clear
the old Candidate exposure and store the replacement immutable request, but it
must not publish ROS markers.

An old timer completion may physically write an old Candidate back into
`latest_candidate_epoch_snapshot_` around that replacement. That storage race
is tolerated. The old Candidate is permanently ineligible for ADD because it
cannot match the stable current replacement request identity/source at the
publication linearization point.

Thus:

```text
stale Candidate slot contents != geometric publication authority
```

No Pair fallback, production mutex, lock-topology change, state machine,
cache, retry, worker, timer, Runtime change or ExecutionAuthority change is
allowed.

## 8. Marker and publication contract

R3 adds:

- one `MatchedAdapterMarkerBundle::tube_certified_geometry` field;
- one MANUAL publisher;
- one stateless filtered-profile marker construction path.

Suggested marker namespace:

```text
phase_offset_manual_tube_certified_geometry
```

Marker contract:

| ID | Type | Meaning |
|---:|---|---|
| 0 | `LINE_STRIP` | lower filtered boundary |
| 1 | `LINE_STRIP` | upper filtered boundary |
| 2 | `TRIANGLE_LIST` | filtered certified ribbon |

Every publication decision produces exactly either three ADD markers or three
DELETE markers. Partial/mixed marker state is forbidden. Marker construction
is stateless and only converts already-computed immutable filtered Candidate
geometry into visualization messages.

The new topic must be integrated into the applicable existing MANUAL
publication/lifecycle paths:

- normal timer-owned publication;
- integrity-mismatch publication;
- no-current-Candidate publication;
- `publishManualDelete()`;
- deactivate lifecycle;
- new-navigation-task reset lifecycle;
- shutdown cleanup.

For ordinary same-task source replacement, the command side performs no ROS
publication. Timer/publication code owns the new ADD/DELETE decision using the
Section 7 protocol. Existing topic publication and delete bookkeeping remain
unchanged unless an isolated change is proven strictly necessary for the new
topic.

Existing reset, explicit deactivate and shutdown publication ownership is
preserved; the source-replacement command-thread prohibition does not redefine
those already-established lifecycle owners.

## 9. Bounded test-only publication observation

Deterministic tests must prove the Section 7 forced interleaving and marker
order. Prefer a pure/private decision helper, direct atomic test setup and
reuse of existing test hooks.

If those mechanisms cannot observe the exact publication boundary, at most
one bounded passive test-only observer may be added within the six-file
whitelist. It must:

- default to absent and never affect production eligibility;
- observe only transaction-local publication events/results;
- never influence Runtime, ExecutionAuthority, selected output or marker
  choice;
- never be ROS-facing production state;
- never become a production cache, FSM, retry or authority;
- be used only to force/record the deterministic test interleaving.

If this cannot be achieved without production behavior/state expansion, stop
for a plan revision rather than broadening R3.

## 10. Exact future implementation whitelist

Only these six files may be modified during a future explicitly authorized R3
implementation:

1. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
2. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
3. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_markers.h`
4. `src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_markers.cpp`
5. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
6. `src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp`

Optional but **not authorized** by this plan candidate:

```text
src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz
```

No CMake change is expected. Everything outside the six-file whitelist remains
read-only unless a new explicit plan revision is authorized.

## 11. Forbidden changes and hard stops

Stop rather than expand scope if implementation requires any of:

- TubeBuilder, TubeFilter, CertifiedTubeBuilder or SurfaceValidator changes;
- ESDF clearance, erosion, safety margin or fail-closed changes;
- Runtime or ExecutionAuthority changes;
- planner, governor, PositionCommand or `gvf_manager` changes;
- Preview or `K_i(s)` changes;
- ContinuousPhaseNormalFrame or preferred-normal/orientation changes;
- Pair fallback or H2/Pair lifecycle expansion;
- Batch C allocator or production nonzero `u_delta`;
- new production persistent state, cache, mutex, lock topology, worker, timer,
  retry or state machine;
- launch, YAML or parameter changes to make acceptance pass;
- `/tube_candidate` or `/tube` semantic changes;
- whitelist expansion;
- safety weakening;
- material baseline/worktree mismatch.

The known vertical/horizontal Tube orientation issue is intentionally deferred
to a separate future stage.

## 12. Deterministic test contract

Future implementation must add deterministic tests proving at least:

1. Current ESDF `OFFSET_CERTIFIED` Candidate produces exactly three ADD.
2. A raw Candidate wider than its filtered profile still produces certified
   geometry from the exact filtered samples only.
3. `ZERO_ONLY_PLANNER_BASELINE` produces exactly three DELETE.
4. NONE, incomplete, nonfinite, unordered, duplicate and fewer-than-two-sample
   profiles produce DELETE.
5. Task/source/path/frame/map/profile/tube provenance mismatch produces
   DELETE.
6. Profile/status classification or completion mismatch produces DELETE.
7. `candidate_only=true` with a current valid Candidate still produces ADD.
8. A newer requested map sequence alone preserves the previous completed
   same-source certified Candidate ADD.
9. A newer completed valid Candidate replaces the older displayed geometry.
10. A newer completed invalid or ZERO_ONLY Candidate produces DELETE and never
    falls back to the older Candidate.
11. Runtime denial, absent authority, `selected=false` and execution-side
    matched/projection/output inactivity do not hide a valid geometric
    Candidate.
12. A forced source-replacement race may repopulate the slot with the old
    Candidate, but after the replacement request linearizes that Candidate can
    never ADD.
13. Publication trace proves replacement request -> old geometry DELETE ->
    replacement build -> new ADD/DELETE.
14. Any test-only observer is passive and cannot alter production state or
    marker choice.
15. R2 `/tube_candidate` exact-build/exact-anchor behavior is unchanged.
16. R2 `ExactPhase()` and near-bit mismatch regressions are unchanged.
17. G18 `/tube` execution-qualified semantics are unchanged.
18. R1 passive-NORMAL/no-Runtime-bootstrap behavior is unchanged.
19. D2 ZERO_ONLY/below-clearance fail-closed behavior is unchanged.
20. Seeded Recovery E2E remains unchanged.
21. Marker construction cannot mutate Runtime, retained delta, selected state,
    Candidate/Active ownership, authority snapshot or control output.

Additional marker tests must verify the new namespace, IDs, marker types,
ribbon vertex count, all-ADD/all-DELETE action contract and a visually
distinct fixed style.

## 13. Stage snapshot and regression contract

Before future implementation, record:

- branch, HEAD and upstream;
- `git status --short`;
- staged path count;
- hashes/diffs of all six whitelist files;
- all pre-existing unrelated user worktree contents.

Compare final R3 changes only against that stage-start snapshot.

At minimum run:

```text
catkin_make -j2

devel/lib/bspline_race/phase_offset_tube_markers_test
devel/lib/bspline_race/phase_offset_matched_adapter_test
devel/lib/bspline_race/phase_offset_tube_epoch_integration_test
devel/lib/bspline_race/phase_offset_tube_epoch_diagnostics_test
devel/lib/bspline_race/gvf_switch_policy_test

devel/lib/phase_offset_navigation/phase_offset_tube_epoch_manager_test
devel/lib/phase_offset_navigation/phase_offset_certified_tube_builder_test
devel/lib/phase_offset_navigation/phase_offset_runtime_test
devel/lib/phase_offset_navigation/phase_offset_preview_feasibility_test
devel/lib/phase_offset_navigation/phase_offset_active_reference_authority_test

git diff --check
```

Explicitly verify:

- Seeded Recovery E2E;
- R1 passive-NORMAL boundary;
- R2 same-build provenance and exact anchor;
- D2 ZERO_ONLY/below-clearance behavior;
- G18 Candidate/Active marker behavior;
- exact six-file whitelist;
- forbidden production hashes/files unchanged.

## 14. Known SurfaceValidator baseline fixture

The known inherited fixture is:

```text
TubeSurfaceValidatorTest.ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel
```

Run all other SurfaceValidator fixtures:

```bash
devel/lib/phase_offset_navigation/phase_offset_tube_surface_validator_test \
  --gtest_filter='*-TubeSurfaceValidatorTest.ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel'
```

They must PASS.

Then separately reproduce the inherited fixture:

```bash
devel/lib/phase_offset_navigation/phase_offset_tube_surface_validator_test \
  --gtest_filter='TubeSurfaceValidatorTest.ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel'
```

Expected unchanged baseline:

```text
Actual: false
Expected: true
exit: 1
```

Record that relevant production/test hashes remain unchanged. This inherited
failure excuses no new R3 failure.

## 15. Real ROS acceptance

The Main context—not Luna—must run two independent fresh split-launch cases.

Terminal 1:

```bash
roslaunch so3_quadrotor_simulator simulator.launch
```

Terminal 2, Run A:

```bash
roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_observe_only:=true \
  phase_offset_manual_tube_source:=esdf \
  phase_offset_tube_cloud_obstacle_set_complete:=true \
  phase_offset_tube_preincluded_map_uncertainty:=0.10
```

Terminal 2, Run B uses the same command except:

```text
phase_offset_manual_observe_only:=false
```

Each run must use its own private `ROS_MASTER_URI`, `ROS_HOME` and task-owned
process set. Task cleanup may stop only task-started processes.

Correct goal topic:

```text
/move_base_simple/goal
```

Fixed goal:

```text
(8.0, 0.0, 1.0)
```

Timeouts and success criterion:

```text
subscriber/readiness timeout: 30 s
goal publication connection timeout: 15 s
total run timeout: 180 s
goal success: 3D distance <= 0.20 m
```

Start recording before relevant publishers when practical. Capture at least:

- base path;
- active path;
- `/tube_candidate`;
- `/tube`;
- `/tube_certified_geometry`;
- epoch diagnostics;
- raw Candidate diagnostics;
- cloud snapshot diagnostics;
- odometry and goal;
- authority/lifecycle logs.

Each A/B run must demonstrate:

- at least one valid `OFFSET_CERTIFIED` build;
- at least one three-ADD certified-geometry publication;
- finite, nonempty marker geometry corresponding to the filtered Candidate
  bounds;
- unchanged `/tube_candidate` semantics;
- historical `/tube` may remain DELETE when execution qualification is absent;
- navigation reaches the fixed goal criterion;
- no persistent HOLD, fatal process death, Runtime bootstrap, passive pending
  activation, routing warning or unauthorized NORMAL/COORDINATION authority
  commit.

ZERO_ONLY is conditional in real ROS. If it occurs naturally, the new topic
must DELETE. Its absence does not fail an otherwise valid run. Mandatory
ZERO_ONLY proof belongs to deterministic R3 tests and the existing D2
regression. Do not change map, safety, margins, launch semantics or Tube
parameters to manufacture ZERO_ONLY.

Same-task source replacement is also conditional in real ROS. The mandatory
race and DELETE/ADD ordering proof belongs to the deterministic concurrency
test. If a natural same-task replacement occurs, verify that the old Candidate
never ADDs after replacement linearization and no old geometry ghosts. Do not
send a second goal to fabricate a same-task replacement.

A newer request map sequence without a newer completion must not be reported
as stale. Once a newer completion becomes authoritative, black-box output must
follow only that completion: valid -> ADD, invalid/ZERO_ONLY -> DELETE, with no
older fallback.

## 16. Expected bounded implementation shape

The production change should remain conceptually limited to:

- one ROS publisher;
- one marker-bundle field;
- one stateless Candidate geometric predicate/currentness path;
- one stateless filtered-profile marker path;
- additions to existing timer/lifecycle publication points;
- focused tests.

R3 may observe already-computed Tube evidence but never participate in
navigation authority. If R3 fails, the worst acceptable result is a missing
new marker—never changed navigation, HOLD or stop behavior.

## 17. Future review and acceptance sequence

Steps 1 through 5 below are complete for the plan-review gate. Steps 6 onward
remain the required future implementation/acceptance sequence:

1. Fresh Main/Codex reads the handoff, this exact plan and the latest Frozen
   baseline.
2. Fresh Main verifies actual repository state.
3. Fresh Main formally reviews this exact persisted plan.
4. Only after Main review PASS, run exactly one new independent read-only
   `gpt-5.6-sol/max` PLAN AUDIT.
5. Only `PLAN_ACCEPTABLE` permits marking this exact plan FROZEN.
6. Freezing does not authorize implementation.
7. User must explicitly authorize R3 implementation.
8. Only then may exactly one Luna Max implement the six-file scope.
9. Luna stops after diff/build/tests/whitelist/diff-check reporting.
10. Main independently reviews actual diff, reruns build/tests and performs
    both real ROS cases.
11. Then run exactly one read-only Sol R3 implementation audit.
12. R3 is ACCEPTED only if all gates pass.
13. R3 acceptance does not authorize Batch C, commit or push.

## 18. Current stop statement

At the time this file was persisted:

```text
R3 plan: FROZEN / IMPLEMENTATION NOT AUTHORIZED
R3 implementation authorization: NONE
Luna: NOT AUTHORIZED
Batch C: NOT AUTHORIZED
formal fresh-context Main review: PASS
fresh exact-plan Sol audit: PLAN_ACCEPTABLE
```

The exact persisted plan received the final Main review PASS and one new
independent read-only `gpt-5.6-sol/max` audit returning `PLAN_ACCEPTABLE`.
Only the governance status metadata changed after that audit; the technical
plan, whitelist, contracts and acceptance criteria remain unchanged. The
implementation authorization remains NONE, so no R3 implementation may begin
from this document alone.
