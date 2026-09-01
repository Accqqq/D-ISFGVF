# Repository Agent Rules

Scope: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws` and all subdirectories.

## Project

The final research direction is the distributed multi-UAV PhaseOffsetSwarm
extension of the existing ISF-GVF system.

Development proceeds from the trusted single-UAV system toward the distributed
swarm system. Single-UAV PhaseOffset work validates the per-agent core of the
final swarm method; it is not a separate research direction.

Trusted original single-UAV regression baseline:

- commit: `9a0e975`

Preserve the original planner, map, ISF-GVF, SDF, B-spline, C2 connector,
governor, simulator, and SO3 execution chain.

User-owned settings in
`src/swarm_planner/bspline_traj/launch/test_gvf.launch` must remain:

- `gvf/circle_test/enable=false`
- `gvf/circle_test/auto_start=false`

Detailed architecture and stage behavior belong in `docs/`, not in this file.

## Agent roles

The current primary conversation is the sole planner, architect, and
orchestrator.

- The current conversation writes and finalizes execution specifications,
  architecture decisions, file whitelists, and acceptance criteria.
- Sol Max is read-only. It reviews plans or completed implementations and
  reports omissions, risks, conflicts, and evidence. It does not rewrite plans
  or modify code.
- Luna Max is execution-only. It implements an approved execution specification
  and does not write a new plan, redesign frozen interfaces, expand scope, or
  start another stage.
- If Sol Max or Luna Max finds a material defect or blocker, it stops and
  reports evidence to the current conversation.
- Only the current conversation may revise the plan or authorize further work.

Preferred workflow for nontrivial or control-critical work:

`current conversation plans -> Sol Max reviews -> current conversation freezes
the plan -> Luna Max implements -> current conversation verifies`

Trivial localized edits already covered by an approved specification do not
require the full workflow.

## Authorization

`AUTO_ADVANCE=false`.

The project roadmap does not authorize implementation by itself.

Code changes require an approved current execution specification defining:

- authorized scope;
- allowed files;
- required behavior;
- tests and acceptance criteria.

If no execution specification is provided, perform read-only inspection only.

Do not modify files outside the authorized whitelist. If a required change lies
outside it, stop and report.

A single execution specification may authorize several tightly coupled
implementation batches when their target architecture and interfaces are frozen
before implementation begins.

A newer explicitly approved architecture or execution specification overrides
older prototype planning documents when they conflict.

## Repository safety

Treat all existing worktree changes as user-owned.

Never use:

- `git reset --hard`
- `git checkout -- .`
- `git restore .`
- `git clean`
- `git stash pop` on the prototype stash

Do not commit, create branches/tags, push, or stage unrelated files unless
explicitly requested.

Before and after implementation work, record:

- current branch and HEAD;
- `git status --short`;
- relevant tracked diff.

Preserve the stash:

`deepseek-phaseoffset-tracked-prototype-2026-08-08`

Prototype code is reference material only unless the current execution
specification explicitly authorizes its use.

## Architecture boundaries

Dependency direction:

`Eigen/STL`
-> `phase_offset_core`
-> `phase_offset_navigation`
-> thin `bspline_race` integration
-> original governor / SO3 execution

Later distributed stages add:

`phase_offset_msgs`
-> `phase_offset_swarm`
-> local coordination inputs to the per-agent navigation layer

### phase_offset_core

Pure C++14/Eigen mathematics.

No ROS, ROS time, SDF map, B-spline manager, robot ID, neighbor, or swarm
dependencies.

### phase_offset_navigation

Owns per-agent Path-Tube, preview/runtime feasibility, constrained phase-offset
execution, and matched-port composition.

Use abstract environment-query interfaces rather than storing concrete ROS map
objects inside the algorithm.

Neighbor discovery and swarm organization do not belong here.

### bspline_race

The existing planner remains authoritative for the base path.

Keep integration thin and orchestration-focused. Do not place control
derivations or optimization algorithms inside ROS callbacks or
`gvf_manager.cpp`.

Reuse the existing base-path C2 continuation mechanism.

### phase_offset_swarm

Owns distributed neighbor processing and swarm coordination.

It may provide local world-frame coordination intent and pairwise safety
constraints to the per-agent layer.

It does not own authoritative `w`, `delta`, `u_w`, or `u_delta`, and never
publishes final UAV control commands.

Do not add PhaseOffsetSwarm-specific messages to legacy `common_msgs`.

## Core invariants

- Preserve the original planner + ISF-GVF navigation baseline.
- Base planner path `p` and active reference `r` are distinct.
- Geometric Tube validity, preview feasibility, and control feasibility are
  separate concepts.
- Loss of nonzero transverse Tube capacity must not by itself invalidate a
  planner-valid path.
- `delta` is a continuous state; transverse expansion, compression, recovery,
  and path-update transitions must not introduce reference jumps.
- Use the same final executed `(u_w, u_delta)` in both matched channels.
- Never add swarm velocity directly to the nominal ISF-GVF physical output.
- Preserve non-reversing phase progression and the approved positive physical
  tangential margin whenever feasible.
- CBF is a safety-constraint layer, not the matched-port core or swarm
  organization law.
- A Tube, preview, staging, or offset-authority failure alone must not cause
  persistent HOLD of an otherwise planner-valid navigation task.

Detailed equations and algorithm contracts belong in the approved architecture
or execution specification.

## Editing

Prefer the existing abstraction when it matches the approved architecture.

Do not preserve an incorrect legacy contract merely to minimize diff size.
During an authorized architecture refactor, obsolete gates, fallback stacks,
certificate layers, or compatibility logic may be removed when required by the
frozen design.

Do not:

- perform repository-wide formatting or unrelated cleanup;
- tune planner/safety limits merely to make a test pass;
- mix unrelated feature work into the current stage;
- scaffold future packages before their stage;
- duplicate the same control law across modules.

Keep mathematical code out of ROS callbacks and keep central orchestration files
focused.

## Verification

Every implementation batch must perform, as applicable:

1. build;
2. stage-specific tests;
3. regression tests for completed functionality;
4. dependency-boundary checks;
5. `git diff --check`;
6. final `git status --short`;
7. verification that no unauthorized files changed;
8. written self-audit and explicit stop statement.

PhaseOffset integration must not regress the original single-UAV navigation
path.

For ROS runtime tests, do not attach to or terminate user-owned processes.
Clean up only processes started by the current task.

## Stop

Stop and report rather than expanding scope when:

- the execution specification is missing or inconsistent with the repository;
- a required edit is outside the whitelist;
- branch, HEAD, worktree, or baseline assumptions differ materially;
- a frozen interface must be redesigned;
- an unrelated historical defect blocks validation;
- ROS/runtime resources required by the specification are unavailable;
- acceptance requires work from a later stage.

Do not silently redesign the plan, weaken acceptance criteria, tune around an
architectural defect, or advance to another stage.