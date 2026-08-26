# R3 Geometric Certified Path-Tube Visualization — Fresh-Context Handoff

```text
DOCUMENT_ROLE=FRESH_MAIN_CONTEXT_HANDOFF
DOCUMENT_STATUS=CURRENT HANDOFF FOR FROZEN PLAN / IMPLEMENTATION NOT AUTHORIZED
DATE=2026-08-25
R3_IMPLEMENTATION_AUTHORIZATION=NONE
LUNA_AUTHORIZED=false
BATCH_C_AUTHORIZED=false
```

## 1. Purpose of this handoff

This document lets a fresh Main/Codex conversation continue R3 without relying
on the long historical conversation that produced the plan. A later explicit
user correction retained the current Main/Codex context for the final
plan-review gate only. That gate is now complete; this handoff records its
result for the next context.

The exact persisted R3 plan is:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/
Codex_R3_Geometric_Certified_Path_Tube_Visualization_Plan_2026-08-25.md
```

Its status is:

```text
FROZEN / IMPLEMENTATION NOT AUTHORIZED
```

## 2. Repository baseline

Repository:

```text
Accqqq/D-ISFGVF
```

Workspace:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

Baseline recorded when this handoff was written:

```text
branch: main
HEAD: 7f162b3d3e019971821db1d209abe0d820a9f4b5
upstream: d_isfgvf/main
checkpoint message: refactor: complete batch B post-acceptance tube repairs
```

That checkpoint contains the accepted combined R1/D3 and R2/D1
post-acceptance repairs. The fresh context must still recheck the actual HEAD,
branch, upstream, staged state and worktree. It must not assume they remain
unchanged merely because they are recorded here.

At handoff creation, unrelated user-owned worktree contents included a modified
`AGENTS.md` plus untracked `.codex/`, `Testing/`, a user architecture brief and
`__pycache__/` directories. Preserve all actual user worktree contents. Do not
clean, restore, stash, stage or attribute them to R3.

## 3. Accepted prior stages and boundaries

```text
R1 / D3: ACCEPTED
R2 / D1: ACCEPTED
D2: PRODUCTION NO CHANGE / REGRESSION PASS
Batch C: NOT AUTHORIZED
```

R1 prevents unauthorized production Runtime bootstrap, passive pending
activation and ownership/liveness regressions that could create persistent
HOLD-style behavior. Advertised production NORMAL remains passive until the
proper allocator stage.

R2 preserves Candidate visualization provenance. Candidate profile and exact
build anchor come from the same immutable `TubeEpochSnapshot`; a later command
phase cannot replace the same-build anchor, and `ExactPhase()` remains exact
bit matching.

D2 preserves the existing fail-closed safety contract. When nonzero capacity
is not actually certified, `ZERO_ONLY_PLANNER_BASELINE` remains valid planner
baseline evidence and is never promoted to nonzero authority or geometry.

Batch C remains explicitly unauthorized. R3 is visualization-only and cannot
start or scaffold the allocator, coordination or production nonzero
PhaseOffset execution work.

The latest accepted Batch-B repair execution record is:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/
Codex_BatchB_PostAcceptance_D1_D3_Repair_Execution_Plan_2026-08-24.md
```

The latest Frozen architecture baseline is:

```text
/home/cxq/ISF-GVF/New_ISFGVF/
PHASE_OFFSET_REFACTOR_FROZEN_PLAN_V3_1.md
```

The current paper/method contract is:

```text
/home/cxq/ISF-GVF/Paper/ICRA/
PHASEOFFSET_DAMPING_CURRENT_CONTRACT.md
```

## 4. The user's actual Tube requirement

The user wants RViz to show the geometric region where PhaseOffset `delta` is
safe to vary according to the already-computed ESDF and safety certificate.

In plain language:

```text
geometric Path-Tube
=
ESDF/safety-certified offset range
```

This is not the same as:

```text
whether that Tube currently owns or qualifies for execution
```

The historical `/phase_offset_manual/tube` topic is execution-qualified. It
may remain DELETE even when certified geometric Candidate evidence exists.
That semantic mismatch is why R3 exists.

## 5. Paper semantics and the Preview boundary

The geometric transverse interval is:

```text
I_i(w) = [delta_lower_i(w), delta_upper_i(w)]
```

The physical Path-Tube is:

```text
T_i = { r_i(w, delta) | delta in I_i(w) }.
```

The current offset state/reference moves inside this geometric Tube when
permitted by later feasibility and execution layers.

The paper's Preview-feasible envelope `K_i(s)` is a distinct, dynamically
feasible subset of the geometric Tube. Current code exposes only value-level
Preview results and does not retain a faithful immutable sampled `K_i(s)`
envelope. R3 must not visualize, invent or approximate it. Preview requires a
separate future architecture decision.

## 6. Previous read-only visibility diagnosis

The useful previous conclusion was:

- Tube/Candidate construction generally occurred successfully;
- Candidate evidence was visible for most of the observed active-navigation
  interval;
- the historical `/tube` topic was rare or absent because it is coupled to
  execution/publication qualification;
- therefore the immediate problem is primarily visualization semantics and
  publication ownership, not general proof that the ESDF Builder fails.

The diagnostic run used `observe_only=true`. The user's normal workflow may
use `observe_only=false`, so that run alone must not be overclaimed as proof of
every false-mode runtime first-false. R3 therefore requires separate real ROS
acceptance for both modes while keeping geometry independent from execution.

## 7. Current and proposed topic meanings

| Topic | Meaning |
|---|---|
| `/phase_offset_manual/base_path` | planner centerline |
| `/phase_offset_manual/active_path` | current-delta reference curve |
| `/phase_offset_manual/tube_candidate` | raw / uncertified Candidate evidence |
| `/phase_offset_manual/tube` | historical G18 execution-qualified Active Tube |
| `/phase_offset_manual/tube_certified_geometry` | new R3 ESDF-certified filtered geometric Candidate Tube |

No existing topic may be redefined. R3 adds only the final row.

## 8. Sole R3 geometry owner and no Pair fallback

R3 deliberately has one production geometry owner:

```text
latest_candidate_epoch_snapshot_
  -> candidate_profile
  -> candidate_profile.samples
  -> filtered_lower / filtered_upper
```

There is no fallback to:

- PathTubePair;
- Pair epoch Candidate;
- Pair Active profile;
- Runtime;
- ExecutionAuthority;
- selected port;
- `active_profile`;
- raw Candidate geometry.

If a current provenance-safe authoritative Candidate is absent or invalid, the
new topic DELETEs. This keeps a visualization feature from expanding back into
H2/Pair/execution ownership.

`candidate_only` is not a geometric veto. A Candidate may be unable to cross
Runtime/Active while remaining valid certified geometric evidence.

## 9. Map sample-and-hold semantics

The new topic means the latest completed certified Candidate for the current
task/source—not certification against the newest requested map sequence.

A newer request map sequence alone does not erase the last completed same-source
Candidate. That immutable Candidate is sample-and-held until a newer Candidate
completion becomes authoritative.

Once a newer current-task/source completion exists:

- valid `OFFSET_CERTIFIED`: use its new filtered geometry;
- ZERO_ONLY: DELETE;
- incomplete/invalid: DELETE;
- never return to the older Candidate.

Internal zero, mutable or mismatched Candidate map provenance remains
fail-closed DELETE.

## 10. Source-replacement concurrency issue and chosen solution

An old timer Tube build can finish around the same time that command/update
logic switches to a new path/source. The old completion may physically write
its Candidate into the Candidate slot after source replacement.

R3 does not solve this by restructuring production locks. The chosen solution
is atomic Candidate/request publication linearization.

At each timer-owned geometric publication decision:

```text
load request A
load Candidate
load request B
require A == B by immutable shared_ptr identity
require Candidate task/source/provenance matches B
require geometric certification
final exact request-identity reread
only then ADD
```

The successful final immutable request-identity recheck is the R3 publication
linearization point.

If request identity, task, source or Candidate provenance changes, the result
is DELETE. A stale Candidate may exist physically in storage but can never
recover current publication authority.

Command/update source replacement must not publish ROS markers. The timer and
existing lifecycle publication paths own ADD/DELETE decisions. No production
mutex, lock-topology change, state machine, cache, retry, worker, timer or Pair
fallback is allowed.

The persisted plan conditionally permits at most one bounded passive test-only
observer if pure decision tests, direct atomic setup and existing hooks cannot
prove the forced interleaving. It must never affect production eligibility,
Runtime, authority, control output or marker choice.

## 11. Known vertical Tube issue is deferred

Tube surfaces may sometimes appear vertical because of the current normal
orientation. That issue is intentionally outside R3.

Do not modify during R3:

- `ContinuousPhaseNormalFrame`;
- preferred normal direction;
- gravity-aware lateral frame rules;
- `N_z` semantics;
- paper normal-orientation semantics.

The user will decide that issue in a separate future stage.

## 12. Future implementation boundary

The exact future R3 modification whitelist is only:

1. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
2. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
3. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_markers.h`
4. `src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_markers.cpp`
5. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
6. `src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp`

Optional but not authorized:

```text
src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz
```

No CMake change is expected. Protected Tube, safety, Runtime, authority,
planner, governor, Preview, normal frame, launch/parameters and Batch C files
remain read-only.

## 13. Real ROS acceptance facts

Normal split-launch entrypoints:

```bash
roslaunch so3_quadrotor_simulator simulator.launch
```

```bash
roslaunch bspline_race test_gvf.launch
```

Correct navigation goal topic:

```text
/move_base_simple/goal
```

Fixed R3 goal:

```text
(8.0, 0.0, 1.0)
```

R3 requires fresh independent acceptance runs for:

- `phase_offset_manual_observe_only:=true`;
- `phase_offset_manual_observe_only:=false`.

Each run uses a separate private `ROS_MASTER_URI`, `ROS_HOME` and task-owned
process set. The exact ESDF arguments, timeouts, recording topics and success
criteria are frozen in the R3 plan.

Natural ZERO_ONLY and natural same-task source replacement are conditional
black-box observations, not mandatory occurrences. Their strict semantics are
always covered by deterministic tests and D2 regressions. No map, safety,
margin, launch or Tube-parameter tuning may manufacture evidence.

## 14. Known SurfaceValidator fixture

The inherited red fixture remains:

```text
TubeSurfaceValidatorTest.ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel
```

The R3 plan requires all other SurfaceValidator fixtures to PASS and separately
reproduces this known fixture with its existing `Actual false / Expected true /
exit 1` disposition. R3 must not edit it or use it to excuse any new failure.

## 15. Governance and exact next action

The final plan-review gate completed steps 1 through 7 below. Steps 8 onward
remain the future implementation/acceptance sequence:

1. Read this handoff completely.
2. Read the exact persisted R3 plan completely.
3. Read the latest Frozen baseline plan and relevant accepted R1/R2 execution
   record.
4. Inspect actual repository branch, HEAD, upstream, worktree and staged state.
5. Perform formal Main review of the exact persisted R3 plan.
6. Only if formal Main review is PASS, run exactly one new independent
   read-only `gpt-5.6-sol/max` PLAN AUDIT.
7. Only if that exact-plan audit returns `PLAN_ACCEPTABLE` may the exact plan
   be marked FROZEN.
8. Freezing still does not authorize implementation.
9. Implementation requires explicit user authorization.
10. Only after authorization may exactly one Luna Max implement the exact
    six-file plan.
11. Main independently reviews the actual diff, build, deterministic tests and
    both real ROS runs.
12. Then run one read-only Sol R3 implementation audit.
13. Do not automatically start Batch C.
14. Do not commit or push without separate explicit authorization.

Current governance state:

```text
R3 plan: FROZEN / IMPLEMENTATION NOT AUTHORIZED
formal fresh-context Main review: PASS
fresh exact-plan Sol audit: PLAN_ACCEPTABLE
R3 implementation authorization: NONE
Luna: NOT AUTHORIZED
Batch C: NOT AUTHORIZED
```

Final plan-review gate record:

```text
plan path: /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_R3_Geometric_Certified_Path_Tube_Visualization_Plan_2026-08-25.md
repository HEAD: 7f162b3d3e019971821db1d209abe0d820a9f4b5
Main review: PASS
new independent Sol model: gpt-5.6-sol / max
Sol verdict: PLAN_ACCEPTABLE
Sol-audited pre-freeze plan SHA-256: 9754789f8da3035ada9c1505de9e18130885fc4e8eec5d10c10936b3f000307c
final frozen plan SHA-256: 0ee560e70c3329350abbc575506770031bf13f14b5134a235bdc2e4492dcb8f0
```

The Sol audit covered the exact persisted pre-freeze plan bytes. The only
post-audit change to that file was the minimum governance status/stop metadata
needed to record the FROZEN result; no technical content changed. The final
frozen plan file hash is recorded below after that metadata update.

Next action for a fresh Main/Codex context:

```text
Implementation authorization: NONE
Luna: NOT AUTHORIZED
Batch C: NOT AUTHORIZED
Wait for explicit user authorization of R3 implementation, then re-verify
the actual repository state before any implementation action.
```

The fresh context must not infer implementation authorization from this handoff.
