# Repository Agent Rules

Scope: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws` and all subdirectories.

## 1. Project direction

The final research direction is the distributed multi-UAV PhaseOffsetSwarm
extension of the existing ISF-GVF system.

Implementation order is fixed:

1. preserve the original single-UAV baseline;
2. validate the single-UAV phase-offset core;
3. validate the complete single-UAV closed loop;
4. add isolated multi-instance simulation;
5. add distributed swarm intent;
6. add pairwise safety and complex scenarios.

Single-UAV stages validate the per-agent core of the final swarm method. They
are not a separate research direction.

## 2. Stage authorization

- The master roadmap describes the whole project but does not authorize all
  stages.
- Code changes require a dedicated current-stage execution specification.
- `AUTO_ADVANCE=false` is the default.
- Complete the authorized stage, test it, self-audit, report, and stop.
- Never start the next stage without a new execution specification.
- If no stage execution specification is provided, perform read-only inspection
  only.
- If a required edit is outside the stage whitelist, stop and report instead of
  expanding scope.

Reference documents:

- `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`
- `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`

## 3. Trusted baseline

Trusted original single-UAV baseline:

- branch: `main`
- commit: `9a0e975`

User-owned retained settings in
`src/swarm_planner/bspline_traj/launch/test_gvf.launch`:

- `gvf/circle_test/enable=false`
- `gvf/circle_test/auto_start=false`

The original single-UAV launch, planner, map, ISF-GVF, SDF, B-spline, C2
connector, governor, simulator, and SO3 chain must remain operational.

A disabled feature flag is not proof of baseline equivalence. Each integration
stage requires explicit regression tests.

## 4. Existing prototype preservation

Tracked prototype modifications are preserved in the named stash:

`deepseek-phaseoffset-tracked-prototype-2026-08-08`

Untracked phase-offset, swarm, CBF, tube, simulator, test, map, config, and
launch files from the previous prototype are reference material only.

- Do not delete, move, rename, modify, or connect them to CMake unless the
  current stage explicitly authorizes the file.
- Do not restore or pop the complete prototype stash into the baseline.
- Migrate useful code only after line-by-line review into the new architecture.

## 5. Git safety

- Treat all existing changes as user-owned.
- Do not use `git reset --hard`, `git checkout -- .`, `git restore .`, or
  `git clean`.
- Do not run `git stash pop` on the prototype stash.
- Do not create commits, branches, tags, or pushes unless explicitly requested.
- Do not stage unrelated files.
- Record `git status --short` and the tracked diff before and after each stage.

## 6. Target architecture

Dependency direction:

1. Eigen/STL;
2. `phase_offset_core`;
3. `phase_offset_navigation`;
4. thin `bspline_race` integration adapter;
5. original governor and SO3 execution.

Multi-UAV additions later use:

1. `phase_offset_msgs`;
2. `phase_offset_swarm`;
3. local `g_swarm` input to the single-UAV adapter.

Target package directory:

`src/swarm_planner/phase_offset/`

Create a package only when its authorized stage begins. Do not scaffold future
packages early.

## 7. Module boundaries

### phase_offset_core

- Pure C++14 and Eigen mathematics.
- No ROS, ROS time, messages, SDF map, B-spline, manager, robot ID, neighbor,
  swarm, tube, or CBF dependencies unless a later approved architecture change
  explicitly revises this boundary.
- Add geometry, matched port, single-agent port projection, and continuation
  only in their authorized stages.

### phase_offset_navigation

- Owns single-UAV tube construction, filtering, and runtime composition.
- Depends on `phase_offset_core`.
- Uses an abstract distance-query interface instead of storing a concrete
  `SDFMap` inside the algorithm.
- Contains no neighbor aggregation or pairwise safety logic.

### bspline_race integration

- Preserve original planning and ISF-GVF behavior.
- Add only a thin adapter from `ContinuousPhasePathState` to the new core.
- Do not place mathematical derivations or optimization algorithms in ROS
  callbacks.
- Keep phase-offset additions to `gvf_manager.cpp` orchestration-only and small;
  target roughly 150 added lines or fewer.
- Do not duplicate the ISF guidance formula in multiple modules.

### phase_offset_msgs and phase_offset_swarm

- Create only in an authorized multi-UAV stage.
- Do not add PhaseOffsetSwarm messages to legacy `common_msgs`.
- The swarm layer outputs local coordination intent `g_swarm` only.
- The swarm layer does not own authoritative `w` or `delta` and does not publish
  final UAV control commands.
- Final `u_w` and `u_delta` are determined by the single-UAV runtime and applied
  through the matched port.
- Visualization nodes are read-only.

## 8. Editing rules

- Modify only files explicitly allowed by the current stage specification.
- Prefer small patches over rewrites.
- Do not perform repository-wide formatting or unrelated cleanup.
- Do not mix refactoring, theory changes, parameter tuning, and scenario
  expansion in one stage.
- One header represents one concept; do not create another mixed global types
  header.
- Keep algorithm implementation files focused, preferably below about 500
  lines.
- ROS callbacks convert data and call interfaces; they do not contain formulas.
- Parse parameters in adapters or nodes, not throughout mathematical classes.

## 9. Theory and control invariants

- Base path `p` and active reference `r` are distinct.
- Active phase-offset tracking error is defined relative to `r`.
- Use the same final executed port in the physical and internal matched
  channels.
- Never add swarm velocity directly to the ISF-GVF physical output.
- Preserve the approved non-reversing phase condition and positive physical
  tangential margin.
- Reuse the existing base-path C2 connector. Do not implement a separate tube
  C2 connector; validate phase-offset and tube continuation around the existing
  path update.
- CBF is a safety filter, not the matched-port core or swarm organization law.

## 10. Parameter and scenario discipline

- Do not change maximum velocity, planner limits, or saturation bounds to make
  a stage pass unless explicitly authorized.
- When authorized, normal speed reduction scales `K1` and `K2` together and
  preserves the existing sign convention.
- Keep the original `pillar.pcd` baseline until another map is authorized.
- Do not add narrow-corridor, circle, figure-eight, seven-agent, ablation, or
  paper-statistics work before their stages.

## 11. Testing and self-audit

Every stage must include:

1. an appropriate clean or incremental build;
2. stage-specific unit tests;
3. regression tests for completed stages;
4. dependency-boundary searches;
5. `git diff --check`;
6. final `git status --short`;
7. verification that no unauthorized files changed;
8. a written self-audit and explicit stop statement.

For authorized ROS tests, check for an existing ROS master first, do not attach
to or terminate user-owned processes, and clean up only processes started by
the current task.

## 12. Stop conditions

Stop and report without expanding scope when:

- a required edit is outside the whitelist;
- branch, HEAD, baseline diff, or stash preconditions differ;
- an unrelated historical defect blocks a workspace-wide test;
- ROS runtime is unavailable;
- acceptance requires a later-stage feature.

Preserve the worktree and report the blocker. Do not fix unrelated problems or
advance to the next stage.
