# D-ISFGVF Batch-B Post-Acceptance D1/D3 Repair Execution Plan

Status: **FROZEN — main-window review PASS; Sol PLAN AUDIT `PLAN_ACCEPTABLE`**

Date: 2026-08-24

Repository: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

Branch: `main`

Historical accepted Batch-B checkpoint and current planning HEAD: `190276628b7c617097476460211656422badb765`

## 0. Authority and Scope

This document is a forward-only execution addendum under the accepted D-ISFGVF Frozen Batch-B architecture. It does not replace or amend the Frozen Plan, does not authorize Batch C, and does not authorize implementation merely by existing.

The addendum converts the accepted D1/D2/D3 diagnosis into two separately reviewable repair stages:

1. **R1 — D3 production passive-NORMAL lifecycle repair**
2. **R2 — D1 Candidate visualization provenance repair**

D2 is a no-change safety invariant and regression requirement, not an implementation stage.

No production or test implementation may begin until:

1. the main window has independently reviewed this document;
2. one read-only Sol PLAN AUDIT has returned `PLAN_ACCEPTABLE`;
3. the plan has been marked frozen; and
4. the user has separately and explicitly authorized the relevant implementation stage.

Plan approval must not automatically authorize Luna. R1 and R2 must not auto-advance. Batch C remains **NOT AUTHORIZED**.

## 1. Baseline and Source of Truth

### 1.1 Repository baseline

- Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`
- Branch: `main`
- Accepted Batch-B checkpoint: `190276628b7c617097476460211656422badb765`
- Planning HEAD: `190276628b7c617097476460211656422badb765`
- Repair direction: forward-only on top of this checkpoint
- Forbidden repository operations: reset, revert, rollback, restore, clean, stash, or any attempt to discard the existing worktree
- Commit/push authorization: none in this plan
- Batch C: **NOT AUTHORIZED**

Existing worktree changes are user-owned and must be preserved. Implementation-stage whitelist checks must compare the implementation agent's new production/test changes against the stage-start snapshot rather than treating unrelated pre-existing paths as stage output.

### 1.2 Source precedence

For this addendum, conflicts are resolved in the following order:

1. `PHASE_OFFSET_REFACTOR_FROZEN_PLAN_V3_1.md` — production architecture and runtime contract
2. the accepted D1/D2/D3 diagnosis and its runtime evidence
3. current repository source at the actual planning/implementation HEAD
4. `Paper/ICRA/PHASEOFFSET_DAMPING_CURRENT_CONTRACT.md` — paper/method semantics only

The paper cannot override the Frozen implementation/runtime contract.

### 1.3 Prior Sol disposition

The accepted D3 repair-boundary audit returned:

`REPAIR_BOUNDARY_ACCEPTABLE`

That verdict established only that the proposed D3 repair can remain inside the existing Batch-B integration boundary. It was not an approval of this detailed implementation plan and did not authorize implementation.

## 2. Frozen Batch-B Boundaries Relevant to This Repair

The following accepted Frozen constraints remain unchanged:

- Batch B is infrastructure-ready and recovery-ready, not production NORMAL/COORDINATION ready.
- Until Batch C connects the sole `PhaseOffsetAllocator`, clean neutral navigation remains `PLANNER_ONLY`.
- Production NORMAL/COORDINATION selected-u ownership is `PhaseOffsetAllocator` only.
- `PhaseOffsetMatchedAdapterRuntime`, PortProjector, the adapter, the governor, diagnostics, and owner labels cannot substitute for `PhaseOffsetAllocator`.
- `PhaseOffsetRecoveryOwner` remains the sole RECOVERY selected-u owner.
- A controlled seeded authoritative-nonzero fixture may exercise accepted Batch-B RECOVERY, but it is not a production bootstrap and cannot create a production NORMAL owner.
- Tube, preview, certification, allocator, or optional PhaseOffset failure cannot invalidate an otherwise planner-valid path or cause a PhaseOffset-only persistent HOLD.
- ZERO_ONLY remains valid planner geometry and does not create nonzero PathTubePair ownership.
- The existing planner, C2 mechanism, governor, PositionCommand publication, and atomic ExecutionAuthority transaction remain protected.

## 3. Complete Accepted Diagnosis

### 3.1 D1 — Candidate visualization provenance

Classification: **IMPLEMENTATION_DEFECT**

#### Dynamic evidence

- Candidate request `control_sequence=32`
- Candidate build anchor `w=1.1819213005227145`
- The exact anchor is present in `raw_build_samples[8]`
- Publication `control_sequence=65`
- Publication `control.current_w=1.8836218625902468`
- Closest raw knot `1.9314122964849596`
- Difference from publication phase `0.047790433894712825`
- No raw knot is bitwise equal to the later publication phase
- Candidate marker actions become `DELETE`

#### Exact source causal chain

```text
build Candidate from TubeBuildRequest::current_path.w
  -> asynchronous TubeEpochSnapshot completion
  -> later command cycle creates ControlPublishSnapshot
  -> ControlPublishSnapshot::current_w is overwritten with later input.path.w
  -> MakeCandidateTubeMarkers
  -> SelectCandidateDisplaySamples
  -> ExactPhase(raw.w, later control.current_w)
  -> anchor == raw.size()
  -> Candidate DELETE
```

Relevant current source:

- `TubeBuildRequest::current_path.w` is the exact immutable build phase.
- `buildTubeEpoch()` copies request identities into `TubeEpochSnapshot`, but currently does not retain the request phase as Candidate publication provenance.
- `makeControlPublishSnapshot()` sets `ControlPublishSnapshot::current_w` from the later command's `input.path.w`.
- Candidate publication can substitute `latest_candidate_epoch_snapshot_` and its Candidate profile while retaining the later command phase.
- `phase_offset_tube_markers.cpp` correctly performs bit-exact `ExactPhase`; its exactness is not the defect.

#### Accepted repair invariant

The Candidate marker must consume the exact phase anchor belonging to the **same immutable Candidate build provenance**:

```text
Candidate profile
+ Candidate epoch
+ request/build anchor
```

Do not replace `ExactPhase` with tolerance matching. Do not use nearest-knot matching. Do not weaken exact phase semantics.

This invariant does not mandate a particular member name. The concrete member/signature naming remains an implementation choice within the R2 mechanism frozen below.

### 3.2 D2 — ESDF certification

Classification: **CORRECT_FAIL_CLOSED**

#### Decisive evidence

```text
actual centreline clearance = 0.38588798379131628
required clearance          = 0.4

0.38588798379131628 < 0.4
```

The physical centreline denial occurs during raw Builder construction before later filter/validator depth or query-limit evidence. The nonzero Candidate therefore correctly collapses to:

`ZERO_ONLY_PLANNER_BASELINE`

#### Frozen no-change result

D2 authorizes no production behavior change. The following remain frozen:

- planner safe clearance
- Tube erosion and robust margins
- zero-connected component semantics
- ZERO_ONLY validity
- Builder/Filter/Validator safety predicates
- fail-closed behavior
- validator query/depth behavior unless a different, independently authorized problem requires it

D2 must not be converted into a “make Tube visible” repair. A valid Candidate marker and a Certified Tube marker are different evidence layers; Certified Tube may legitimately remain `DELETE` when D2 fails.

### 3.3 D3 — production passive-NORMAL lifecycle boundary

Classification: **IMPLEMENTATION_DEFECT / BATCH-BOUNDARY LEAK**

#### Clean split-launch evidence

The diagnosis was reproduced with the checked-in production launch pair and no source, launch, or parameter edits:

```bash
roslaunch so3_quadrotor_simulator simulator.launch

roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_observe_only:=false \
  phase_offset_manual_tube_source:=esdf \
  phase_offset_tube_cloud_obstacle_set_complete:=true \
  phase_offset_tube_preincluded_map_uncertainty:=0.10
```

Runtime evidence:

- first point goal reached at approximately `15.719 s`
- `OFFSET_BOOTSTRAP COMMITTED` at approximately `50.316 s`
- pair generation `1`
- authority session `10`
- source revision `11`
- Tube revision `1`
- map observation sequence `616`
- owner domain `[3.097373908, 9.428950119]`
- captured `w0=6.661867588`
- live `wc=6.741252555`
- future seam `7.153539918`
- command captures the same pair as `pending_activation` at approximately `50.332 s`
- `owner-bound request could not be routed` at approximately `50.519 s`
- real H2 successor staging reports `stage_success=1` at approximately `50.835 s`, with `captured_w=7.200995512`
- the successful staging phase is already beyond the original seam `7.153539918`
- first `36/0/36 all_candidates_path_end_clamped` HOLD at approximately `51.840 s`
- a later H2 attempt fails `TUBE_PROFILE_COVERAGE` at approximately `53.554 s`
- final phase remains near the old path end: approximately `9.405865` versus `9.428950`
- final goal distance remains approximately `10.50 m`
- `GOVERNOR_INVALID_HOLD` persists

#### Correct transaction behavior

Production advertisement clears `allow_test_only_runtime_owner`. A Runtime-selected NORMAL candidate is labeled `PhaseOffsetMatchedAdapterRuntime`; `OwnerAllowed()` correctly rejects it because production NORMAL/COORDINATION permits only `PhaseOffsetAllocator`.

This rejection is **CORRECT_FROZEN_BEHAVIOR**. R1 must not change `OwnerAllowed()`, relabel Runtime, or broaden the owner whitelist.

#### Defective lifecycle behavior

The constructor enables the existing test-only Runtime-owner exception for unadvertised fixtures, and `advertise()` correctly clears it for production. However, the two lifecycle predicates do not consult that same ownership capability:

- `requiresPathTubePairBootstrapLocked()` can still convert pending manual Runtime intent into a production bootstrap request when Runtime is neutral.
- `hasPendingOffsetActivationPairLocked()` can still classify the committed pair as authority-preservation state even though the pending Runtime activation cannot legally commit NORMAL selected-u.

The resulting contradiction is:

```text
cannot legally execute NORMAL selected-u
AND
cannot neutral-retire / permit ordinary planner replacement
```

No executed-nonzero predecessor exists. Therefore `requestRecenter()` correctly returns false and cannot manufacture RecoveryOwner authority. The mailbox warning and final path-end HOLD are downstream consequences.

The paper does not require or justify manufacturing Recovery from a never-executed offset activation. The executed-predecessor requirement belongs to the Frozen runtime contract and is compatible with, but is not explicitly stated by, the paper.

## 4. Repair Staging and Non-Auto-Advance Rule

R1 and R2 are separate implementation/review stages.

```text
frozen plan
  -> explicit user authorization for R1 only
  -> sole implementation agent implements R1
  -> main independent R1 review/build/tests/ROS/whitelist
  -> read-only Sol R1 implementation audit
  -> main declares R1 accepted or requests R1-only repair
  -> explicit user authorization for R2 only
  -> sole implementation agent implements R2
  -> main independent R2 review/build/tests/ROS/whitelist
  -> read-only Sol R2 implementation audit
  -> main declares R2 accepted or requests R2-only repair
  -> stop; Batch C remains unauthorized
```

R2 must not start merely because R1 passes. Each stage requires explicit authorization after the preceding stage is independently accepted.

## 5. R1 — D3 Production Lifecycle Repair

### 5.1 Goal

While Batch C and `PhaseOffsetAllocator` are absent:

- production NORMAL remains passive / `PLANNER_ONLY`;
- Tube construction, certification, diagnostics, and visualization remain available;
- passive Runtime intent cannot obtain production pending-activation ownership;
- a never-executed activation cannot create RECOVERY ownership or recovery-mailbox work;
- neutral planner replacement remains available;
- genuine pre-existing authoritative-nonzero RECOVERY remains unchanged;
- unadvertised deterministic Runtime fixtures remain available only through the existing test-only owner permission.

### 5.2 Existing state and predicates

R1 uses only existing state:

- `advertised_`
- `PhaseOffsetExecutionAuthorityConfig::allow_test_only_runtime_owner`
- `execution_authority_.config()`
- `runtime_->hasPendingOrActiveOffsetIntent()`
- `runtime_->hasExecutedOffsetAuthority()`
- `authoritative_path_tube_pair_`
- `requiresPathTubePairBootstrapLocked()`
- `hasPendingOffsetActivationPairLocked()`
- `requiresAuthoritativeOffsetHandoffLocked()`

No new manager, FSM, authority, state mirror, cache, gate, retry object, worker, timer, ROS message, or parameter is authorized.

### 5.3 Frozen implementation mechanism

Use the existing `execution_authority_.config().allow_test_only_runtime_owner` value as the single capability fact for Runtime-originated NORMAL bootstrap/pending activation.

Modify both adapter lifecycle predicates so Runtime-originated bootstrap/pending activation is eligible only when that existing test-only permission is true:

1. `requiresPathTubePairBootstrapLocked()` must return false before evaluating pending intent when the Runtime owner is not permitted.
2. `hasPendingOffsetActivationPairLocked()` must return false when the Runtime owner is not permitted.

The exact helper/member name, if any, is an implementation choice. Do not add a second boolean. Do not infer permission independently from mode, amplitude, Tube state, or diagnostics. The authority configuration is already the source of truth and is already cleared by `advertise()`.

### 5.4 Required before/after behavior

| Situation | Before R1 | After R1 |
|---|---|---|
| Unadvertised deterministic fixture, test-only Runtime owner allowed, certified nonzero Tube, neutral pending intent | Bootstrap/pending activation allowed | Unchanged |
| Advertised production, Runtime owner disallowed, certified nonzero Tube, neutral pending intent | Bootstrap pair may commit and become pending activation | No bootstrap; no pending activation |
| Advertised production with a never-executed illegal pending pair already present | Pair can block neutral retirement | Pair is not classified as pending activation; neutral retirement may proceed through the existing path |
| Existing valid executed-nonzero authority | Authoritative handoff/recovery required | Unchanged |
| Existing RECOVERY snapshot at exact neutral terminal | Atomic neutral handoff required | Unchanged |
| ZERO_ONLY Candidate with no executed authority | No nonzero bootstrap | Unchanged |

### 5.5 Why no manager production change is required

`gvf_manager` already delegates lifecycle ownership to the adapter predicates:

- timer bootstrap calls `activatePendingOffsetAuthority()` only when `requiresPathTubePairBootstrap()` is true;
- command/replan capture reports `pending_activation` only when `hasPendingOffsetActivationPair()` is true;
- neutral planner replacement is blocked only when the adapter reports a pending activation;
- recovery mailbox staging requires an owner pair and a qualifying denial;
- genuine executed authority is separately reported by `requiresAuthoritativeOffsetHandoff()`.

After the two source predicates are corrected:

- production timer bootstrap is never entered for passive Runtime intent;
- no illegal production pair exists to arm H2 or the current-state recovery mailbox;
- an already present never-executed pair is no longer protected as pending activation and can retire using the existing neutral retirement path;
- genuine executed-nonzero/RECOVERY behavior continues through the independent executed-authority predicates.

Therefore `gvf_manager.h` and `gvf_manager.cpp` remain read-only in R1. A manager production edit is not authorized as “defense in depth.” If implementation evidence contradicts this source analysis, stop and return for plan revision rather than expanding the whitelist.

### 5.6 R1 exact modification whitelist

Only the following files may be modified in R1:

| File | Required reason |
|---|---|
| `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h` | Update the lifecycle contract comments so public/private predicate semantics explicitly distinguish unadvertised test-only Runtime bootstrap from advertised production passive NORMAL. No new public API or stored state is authorized. |
| `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp` | Add the existing Runtime-owner permission predicate to `requiresPathTubePairBootstrapLocked()` and `hasPendingOffsetActivationPairLocked()`; make no authority, Runtime, Tube, recovery, governor, or publication changes. |
| `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp` | Add deterministic adapter regressions for production-disabled bootstrap/pending activation, preserve unadvertised fixture behavior, preserve owner rejection, and add the D2 below-clearance no-change regression. |
| `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp` | Add manager-level integration regression proving passive production intent cannot create H2/pending-activation ownership or block neutral planner replacement; retain and rerun the seeded authoritative-nonzero RECOVERY E2E. |

Everything else is read-only in R1, including:

- `gvf_manager.h/.cpp`
- `active_reference_authority.h/.cpp`
- Runtime/RecoveryOwner/Preview/Handoff sources
- governor and planner sources
- Tube Builder/Filter/Validator sources
- launch files and parameters

### 5.7 R1 deterministic tests

Add or update tests with the following exact assertions.

#### Adapter production-boundary regression

Construct an otherwise bootstrap-eligible manual adapter with:

- certified nonzero active profile;
- zero gate open;
- pending Runtime intent;
- no executed offset authority;
- production owner permission disabled, matching `advertise()`.

Assert:

- `requiresPathTubePairBootstrap()` is false;
- no PathTubePair is committed;
- `hasPendingOffsetActivationPair()` is false for any never-executed fixture pair;
- Runtime retained delta stays exactly zero;
- ExecutionAuthority has no NORMAL/COORDINATION commit;
- `PhaseOffsetMatchedAdapterRuntime` remains rejected for NORMAL/COORDINATION.

#### Unadvertised test-fixture regression

With the existing test-only Runtime-owner permission true, assert the established deterministic bootstrap fixture remains usable. This protects the controlled seeded-authority setup without converting it into production behavior.

#### Manager neutral-replacement regression

At manager integration level, configure production-disabled Runtime ownership and pending manual intent. Assert:

- the timer bootstrap result is `NOT_REQUIRED` / no committed pair;
- replan capture has `executed_authority=false` and `pending_activation=false`;
- neutral planner frontend replacement succeeds through the existing path;
- no current-state recovery mailbox ticket is created or consumed for the never-executed activation;
- no H2 authority-preservation branch is entered for that intent.

#### Genuine RECOVERY regression

Rerun without semantic weakening:

`SeededRecoveryE2E.SeededNonzeroAuthorityUsesSuccessorRecoveryAndAtomicNeutralHandoff`

It must continue to prove:

- controlled pre-existing authoritative nonzero predecessor;
- actual staged successor PathTubePair/revisions/Tube/profile;
- Preview/Handoff and RecoveryContinuationProvider when required;
- exact `PhaseOffsetRecoveryOwner` selected-u;
- immutable executed reference and existing governor;
- local PositionCommand publication before atomic commit;
- repeated RECOVERY ticks;
- validated terminal predicate;
- atomic neutral PlannerOwner handoff.

#### D2 no-change regression

Add a deterministic ESDF/clearance fixture inside the already whitelisted adapter test file with:

- required centreline clearance exactly `0.4`;
- known-free, certified centreline clearance strictly below it, preferably `0.39` to avoid numerical ambiguity;
- otherwise valid path/map evidence.

Assert:

- the raw nonzero cross-section is rejected at the physical clearance comparison;
- Candidate classification is `ZERO_ONLY_PLANNER_BASELINE`;
- the rejection reason is insufficient clearance / empty-after-obstacle-bounds as appropriate to the existing layer;
- no nonzero active PathTubePair/bootstrap is produced;
- no margin, query limit, depth limit, or fail-closed predicate is changed.

### 5.8 R1 build and test gate

Required after implementation:

1. `catkin_make -j2`
2. `devel/lib/bspline_race/phase_offset_matched_adapter_test`
3. `devel/lib/bspline_race/gvf_switch_policy_test`
4. `devel/lib/phase_offset_navigation/phase_offset_active_reference_authority_test`
5. `devel/lib/phase_offset_navigation/phase_offset_recovery_owner_test`
6. `devel/lib/bspline_race/phase_offset_tube_markers_test`
7. relevant existing Tube/epoch suites needed to prove D2 remains unchanged
8. `git diff --check`
9. exact R1 whitelist comparison against the stage-start production/test snapshot

No failing test may be reclassified, disabled, or loosened to accept the repair.

### 5.9 R1 real split-launch acceptance

Use the checked-in production launch pair.

Terminal 1:

```bash
roslaunch so3_quadrotor_simulator simulator.launch
```

Terminal 2:

```bash
roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_observe_only:=false \
  phase_offset_manual_tube_source:=esdf \
  phase_offset_tube_cloud_obstacle_set_complete:=true \
  phase_offset_tube_preincluded_map_uncertainty:=0.10
```

Acceptance requires runtime evidence that:

- planner-valid navigation continues and reaches the goal criterion used by the checked-in launch;
- no Runtime-originated `OFFSET_BOOTSTRAP COMMITTED` occurs in advertised production;
- no production `pending_activation` pair is created from passive Runtime intent;
- no `owner-bound request could not be routed` warning is emitted for a never-executed activation;
- no PhaseOffset-induced persistent `all_candidates_path_end_clamped` HOLD occurs;
- no production NORMAL/COORDINATION ExecutionAuthority commit occurs;
- Tube Candidate/certification/diagnostic evidence remains published according to existing semantics;
- the test does not claim active production PhaseOffset NORMAL behavior;
- genuine RECOVERY remains protected by the seeded deterministic E2E, not manufactured by this launch.

FIXED Tube is not a mandatory R1 acceptance gate.

### 5.10 R1 independent acceptance gate

After R1 implementation, the sole implementation agent stops. The main window independently reviews:

- source diff and state-machine/lifecycle semantics;
- exact whitelist;
- build and deterministic tests;
- seeded RECOVERY E2E;
- active ESDF split-launch evidence;
- absence of Batch C, owner-whitelist, governor, planner, Tube-safety, or second-state changes.

Then one read-only Sol R1 implementation audit is required. R2 remains unauthorized until R1 is independently accepted and the user explicitly authorizes R2.

## 6. R2 — D1 Candidate Provenance Repair

### 6.1 Entry condition

R2 may begin only after R1 is independently accepted by the main window and read-only Sol implementation audit, followed by explicit user authorization for R2.

### 6.2 Goal

Candidate display must use the exact phase anchor from its own immutable build provenance even when the current command phase has advanced asynchronously.

R2 is visualization/evidence-only. It must not alter:

- Runtime selection or state;
- Certified/Active Tube ownership;
- PathTubePair ownership;
- ExecutionAuthority;
- Preview/Handoff/RecoveryOwner;
- Tube construction, filtering, validation, classification, or margins;
- planner, governor, or PositionCommand behavior.

### 6.3 Current provenance structures

The exact build anchor currently exists as:

`TubeBuildRequest::current_path.w`

The immutable timer result is:

`TubeEpochSnapshot`

It already carries request identity, build sequence, source/map identity, Candidate profile, Active profile, and diagnostics, but it does not independently expose the exact build phase for Candidate marker selection.

The publication snapshot is:

`ControlPublishSnapshot`

Its current `current_w` value is the later command-cycle `input.path.w`. Production publication may select or substitute a Candidate profile from `latest_candidate_epoch_snapshot_`; therefore Candidate profile and marker anchor can come from different epochs.

### 6.4 Frozen provenance plumbing mechanism

R2 must implement this data path:

```text
TubeBuildRequest::current_path.w
  -> exact value copy in the immutable TubeEpochSnapshot provenance
  -> Candidate publication selects one exact TubeEpochSnapshot
  -> Candidate profile and build anchor are both taken from that same snapshot
  -> MakeCandidateTubeMarkers(..., exact_same_build_anchor)
```

Requirements:

1. Retain only the exact finite phase value needed for Candidate provenance; do not retain an additional TubeBuildRequest pointer, mutable callback, worker state, or second cache.
2. The copied value must preserve the exact double bits from `TubeBuildRequest::current_path.w`.
3. The implementation may choose the concrete member and private helper/signature names. No particular field name such as `candidate_anchor_w` is mandated.
4. The private marker-construction path must consume one coherent Candidate provenance owner. It must not accept an independently chosen Candidate profile together with `ControlPublishSnapshot::current_w`.
5. When publication substitutes `authoritative_candidate`, both profile and anchor must come from that same `TubeEpochSnapshot`.
6. When exact-live-pair publication selects the authoritative Candidate epoch, the same rule applies.
7. If no exact epoch/profile identity is available in production publication, Candidate marker publication fails closed to `DELETE`; it must not guess an anchor. The existing compatibility/test helper may continue using its caller-supplied exact input phase with a caller-supplied profile, because it does not perform asynchronous epoch substitution and is not the production timer publication path.
8. The command-cycle phase may remain available for command/diagnostic uses, but it must not replace Candidate build provenance.
9. `phase_offset_tube_markers.cpp::ExactPhase` remains unchanged.

### 6.5 Required before/after behavior

| Situation | Before R2 | After R2 |
|---|---|---|
| Candidate build and publication use same exact phase | Candidate ADD when displayable | Unchanged |
| Candidate build completes asynchronously and later command phase advances | Later command phase may cause `anchor == raw.size()` and DELETE | Same-build anchor produces ADD/nonempty markers when Candidate is displayable |
| Candidate profile is substituted from authoritative Candidate epoch | Profile may be paired with unrelated command phase | Profile and anchor both come from authoritative Candidate epoch |
| Near but not bit-equal phase | DELETE | Unchanged DELETE |
| No matching immutable Candidate epoch | DELETE | Unchanged/fail-closed DELETE |
| Candidate raw geometry valid but Certified Tube is ZERO_ONLY/D2 denied | Candidate may be ADD; Certified remains DELETE | Preserved |

### 6.6 R2 exact modification whitelist

Only the following files may be modified in R2:

| File | Required reason |
|---|---|
| `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h` | Extend the immutable Candidate epoch provenance with the exact request/build phase value and adjust only private publication/marker-plumbing declarations/comments as required. No public control API, ROS schema, cache, manager, or authority state may be added. |
| `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp` | Copy the exact build phase into the immutable epoch and make all production Candidate marker branches consume profile plus anchor from the same epoch. Preserve command phase separately and preserve all Runtime/Certified ownership behavior. |
| `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp` | Add asynchronous build/publication provenance tests, authoritative-Candidate substitution tests, exact-bit anchor assertions, and visualization-only noninterference assertions. |

The following files are required read-only regression dependencies and are explicitly **not** in the R2 modification whitelist:

- `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_markers.h`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_markers.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp`

Reason: current marker code already implements the accepted exact `ExactPhase` semantics, and existing marker tests already prove that a near-but-not-bit-equal anchor is rejected. R2 repairs the caller's provenance, not marker geometry or matching semantics.

Everything else remains read-only in R2, including manager, authority, Runtime, Tube, planner, governor, launch, and paper files.

### 6.7 R2 deterministic tests

Add adapter-level tests that create two distinct phases:

- Candidate build phase `w_build`
- later command/publication phase `w_later`, with `w_later != w_build`

The exact Candidate raw sample at `w_build` must be present and displayable. Assert:

- the immutable Candidate epoch preserves the exact bits of `w_build`;
- the later control snapshot preserves `w_later` as command-cycle evidence if still needed;
- Candidate marker generation uses `w_build`, not `w_later`;
- Candidate markers are `ADD` and nonempty;
- marker points include the exact anchored raw sample/component;
- substituting the authoritative Candidate epoch also substitutes its anchor;
- `w_later` cannot silently replace Candidate provenance;
- a near-but-not-bit-equal anchor still produces `DELETE` in the marker unit suite;
- no Candidate marker publication changes `selected`, Runtime delta, authority snapshot, Active profile, planner output, or governor state.

Rerun existing marker tests, especially:

- `TubeMarkersTest.CandidateUsesCurrentEpochRawGeometryWhenFinalProfileIsZero`
- `TubeMarkersTest.CandidateRequiresExactCurrentRawAnchor`
- `TubeMarkersTest.CandidateRejectsSingleCurrentRawSampleOrRemoteGap`
- `TubeMarkersTest.CandidateKeepsBroadRawGeometryWhileCertifiedMarkerUsesNarrowProfile`

The D2 below-clearance regression added in R1 must remain green.

### 6.8 R2 build and test gate

Required after implementation:

1. `catkin_make -j2`
2. `devel/lib/bspline_race/phase_offset_matched_adapter_test`
3. `devel/lib/bspline_race/phase_offset_tube_markers_test`
4. `devel/lib/bspline_race/gvf_switch_policy_test`
5. relevant Tube/epoch suites required to prove D2 remains unchanged
6. `git diff --check`
7. exact R2 whitelist comparison against the R2 stage-start production/test snapshot

### 6.9 R2 real split-launch acceptance

Terminal 1:

```bash
roslaunch so3_quadrotor_simulator simulator.launch
```

Terminal 2:

```bash
roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_observe_only:=true \
  phase_offset_manual_tube_source:=esdf \
  phase_offset_tube_cloud_obstacle_set_complete:=true \
  phase_offset_tube_preincluded_map_uncertainty:=0.10
```

Acceptance requires:

- capture of a valid completed Candidate build and its immutable request/build phase;
- a later command-cycle phase different from the build phase;
- Candidate `ADD`/nonempty markers generated from the same Candidate build provenance;
- no tolerance or nearest-knot behavior;
- Certified Tube may legitimately remain `DELETE`/ZERO_ONLY when D2 clearance fails;
- visualization does not change planner navigation, Runtime selection, or authority;
- no process death, fatal invariant, or visualization-induced HOLD.

FIXED Tube is not a mandatory R2 acceptance gate.

### 6.10 R2 independent acceptance gate

After R2 implementation, the implementation agent stops. The main window independently reviews:

- exact Candidate profile/epoch/anchor provenance in every publication branch;
- absence of tolerance/nearest matching;
- source diff and exact whitelist;
- deterministic tests and observe-only ESDF split-launch evidence;
- unchanged D2 safety behavior;
- unchanged R1 passive production boundary and seeded RECOVERY E2E.

Then one read-only Sol R2 implementation audit is required. Passing R2 does not authorize Batch C.

## 7. D2 No-Change Regression Invariant

D2 is not an implementation batch. Its only authorized change is the deterministic regression added inside an already R1-whitelisted test file.

Acceptance across both R1 and R2 requires:

- a clearance strictly below `0.4` remains rejected;
- `ZERO_ONLY_PLANNER_BASELINE` remains planner-valid;
- a zero-only active profile cannot request nonzero bootstrap authority;
- Candidate visualization can remain available from raw build evidence without claiming Certified Tube validity;
- no change to clearance, erosion, filter, validator, query depth/limit, or fail-closed production code.

## 8. Exact Forbidden Changes

The following are frozen for both stages:

- `OwnerAllowed()` production whitelist
- `PhaseOffsetAllocator` ownership contract
- Batch C packages or semantics
- production NORMAL/COORDINATION activation
- governor implementation or selection semantics
- planner, Kino A*, B-spline, or goal semantics
- base-path C2 architecture and continuation provider contract
- Tube clearance, erosion, margins, connected-component rules, and ZERO_ONLY semantics
- Builder/Filter/Validator fail-closed behavior
- Runtime dynamics, PortProjector mathematics, and RecoveryOwner selector/liveness rules
- second FSM
- second execution authority
- second execution-state mirror or cache
- new lifecycle gate/state bit duplicating the existing authority permission
- new retry manager
- new worker, thread, or timer
- ROS message or diagnostic schema changes
- launch/parameter edits
- paper source changes
- preferred normal orientation / lateral seed work
- public `N_ww`, pointwise `p_www`, strict nonzero `r_ww`, C3, or governor redesign
- whitelist expansion without a new main-window plan revision and Sol plan audit

## 9. Required Real-Launch Baseline

The checked-in production launch pair is mandatory for both stages:

```bash
roslaunch so3_quadrotor_simulator simulator.launch
roslaunch bspline_race test_gvf.launch [stage-specific arguments]
```

The default checked-in `test_gvf.launch` may be run as a planner-navigation baseline, but the stage-specific active ESDF and observe-only ESDF runs above are the required acceptance evidence.

Do not edit launch files or parameters during acceptance. Do not use FIXED Tube as a substitute for the mandatory ESDF evidence. Clean up only ROS processes started by the review task.

## 10. Complexity and Runtime Constraints

R1 adds only constant-time boolean predicate checks on existing state. It must add:

- no loop;
- no allocation;
- no copy of path/Tube data;
- no reconstruction;
- no new lock;
- no 50 Hz query work.

R2 adds one scalar immutable provenance copy and constant-time selection of the same epoch's anchor. It must add:

- no path-wide reconstruction or resampling;
- no per-marker search beyond the existing `ExactPhase` scan;
- no new worker/timer/cache;
- no new command-tick Tube/frame/proof construction;
- no new material allocation beyond existing immutable snapshot construction.

Any proposed operation with plausible new cost above 1 ms in a command or publication tick is outside this plan and requires a plan revision.

## 11. Stage Review and Stop Conditions

Stop and return to the main window if any implementation requires:

- modification outside the exact stage whitelist;
- a manager production change for R1;
- an authority whitelist or owner-label change;
- a new lifecycle state bit or cache;
- Batch C/allocator work;
- Tube safety or validator changes;
- governor/planner/C2 changes;
- tolerance/nearest matching for D1;
- weakening seeded authoritative-nonzero RECOVERY;
- launch or parameter edits;
- a Frozen contract change or paper reconciliation.

Do not work around such a condition.

## 12. Main-Window Plan Review Checklist

Before Sol PLAN AUDIT, the main window must verify:

1. all accepted D1/D2/D3 evidence and classifications are represented;
2. D1 states an invariant without mandating a particular field name;
3. D2 is no-change production behavior with a deterministic regression only;
4. D3 separates correct owner rejection from defective lifecycle arming;
5. R1 uses the existing test-only owner permission and no duplicate gate;
6. manager production sources are excluded with a source-grounded reason;
7. R1 preserves seeded authoritative-nonzero RECOVERY;
8. R2 couples Candidate profile, epoch, and exact request/build anchor;
9. marker `ExactPhase` source remains read-only;
10. R1/R2 whitelists are exact and inside the Frozen Batch-B whitelist;
11. deterministic and ESDF split-launch gates are sufficient;
12. forbidden boundaries and complexity limits are explicit;
13. paper wording does not attribute the Frozen executed-predecessor rule to the paper itself;
14. no implementation or Luna authorization is implied by plan acceptance.

## 13. Required Sol PLAN AUDIT

After the main-window review, run exactly one read-only Sol PLAN AUDIT asking:

1. Does the plan correctly cover all accepted D1/D2/D3 findings?
2. Is D2 correctly frozen as no-change?
3. Are R1 and R2 mechanisms minimal and source-grounded?
4. Does any proposed modification leak into Batch C?
5. Does R1 preserve real authoritative-nonzero RECOVERY?
6. Does R2 preserve exact Candidate provenance without weakening geometry/certification semantics?
7. Are the file whitelists exact and sufficient?
8. Are deterministic and real split-launch acceptance tests sufficient?
9. Does the plan introduce a second FSM/cache/authority/gate/retry/thread?
10. Is the plan compatible with the paper/method contract and Frozen Plan?

Sol must return one of:

### `PLAN_ACCEPTABLE`

The plan is implementation-ready after explicit user authorization for R1 only.

### `PLAN_REVISION_REQUIRED`

Sol must identify exact blocking sections and required plan-only corrections. The main window revises only this document and reruns the read-only PLAN AUDIT.

### `ARCHITECTURE_DECISION_REQUIRED`

Sol may return this only if the plan actually conflicts with the Frozen architecture and must identify the exact conflict.

If Sol returns `PLAN_ACCEPTABLE`, mark this document frozen and stop. Do not authorize Luna in the same step. Do not authorize Batch C.

## 14. Plan Audit Record

Main-window review: **PASS — mechanisms, exact whitelists, tests, Frozen boundaries, and paper wording independently checked against HEAD `190276628b7c617097476460211656422badb765`**

Sol PLAN AUDIT: **PLAN_ACCEPTABLE — all D1/D2/D3 coverage, R1/R2 mechanisms, exact whitelists, deterministic tests, mandatory ESDF split-launch gates, Frozen boundaries, and paper compatibility accepted**

Frozen status: **FROZEN**

Implementation authorization at plan freeze: **NONE** (historical; R1 and R2
were later authorized separately and completed under the frozen stage gates)

Batch C authorization: **NO — NOT AUTHORIZED**

## 15. Post-Acceptance Execution Record

Record date: **2026-08-25**

This section records execution and acceptance only. It does not revise the
Frozen architecture, mechanisms, whitelists, contracts, or stage boundaries.

### R1 / D3

- Status: **ACCEPTED**
- Implementation audit: **R1_IMPLEMENTATION_ACCEPTABLE**
- Real split-launch: planner-valid navigation reached the goal criterion at
  distance `0.186`.
- Production boundary evidence: no Runtime bootstrap, no passive Runtime
  pending activation, no owner-bound routing warning, and no persistent HOLD.

### R2 / D1

- Status: **ACCEPTED**
- Implementation audit: **R2_IMPLEMENTATION_ACCEPTABLE**
- Real observe-only ESDF split-launch evidence:
  - Candidate build phase `w_build = 0.62253585507942055`;
  - later command phase `w_later = 2.542`;
  - `w_later != w_build`;
  - Candidate markers were `ADD` and nonempty from same-build exact provenance;
  - planner navigation reached the goal criterion;
  - no production NORMAL/COORDINATION authority commit.

### D2

- Status: **PRODUCTION NO CHANGE / REGRESSION PASS**
- Clearance, margins, Builder/Filter/Validator, ZERO_ONLY, and fail-closed
  production semantics remain unchanged.

### Authorization Boundary

- Batch C: **NOT AUTHORIZED**
- No Batch C implementation or implementation plan is authorized by R1/R2
  acceptance or by this execution record.
