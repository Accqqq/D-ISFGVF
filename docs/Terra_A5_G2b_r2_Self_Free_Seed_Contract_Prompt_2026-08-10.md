# A5-G2b-r2 Execution Specification: Local Self-Free Seed for Raw Tube Raycasting

Date: 2026-08-10

Execution model: one `gpt-5.6-terra` agent at `reasoning_effort=max`.

`AUTO_ADVANCE=false`.

This document authorizes only A5-G2b-r2. Implement the smallest integration-
layer correction needed to let the raw-occupancy tube raycast start from the
actual UAV footprint when that footprint is still `UNKNOWN` in the raw map.
Build, test, collect isolated ROS evidence, self-audit, report, and stop.

This is not permission to modify SDFMap, change UNKNOWN globally, tune tube
geometry, add planner clearance constraints, alter Runtime/control, or enter
A5-G3, A5.3, A6, A7, or multi-UAV work.

Do not delegate to another agent.

## 1. Established evidence and exact problem

A5-G1, G2a, and G2b already established the proposal-consistent dataflow:

```text
ContinuousPhasePath samples p(w), p_w(w), p_ww(w)
  -> lifted path geometry T(w), N(w), kappa(w)
  -> independent raw occupancy raycasts along +N and -N
  -> subtract r_eff exactly once
  -> intersect obstacle and signed-curvature intervals
  -> candidate TubeProfile
  -> TubeEpochManager candidate/active ownership
  -> Candidate/Certified Marker and 50 Hz Runtime
```

The default raw contract remains:

```text
search_extent       = 3.00 m per side
ray_step            = 0.05 m
boundary_tolerance  = 0.01 m
uav_radius          = 0.25 m
map_uncertainty     = 0.10 m
localization        = 0.05 m
tracking_error      = 0.15 m
r_eff               = 0.55 m
```

A5-G2b-r1 then proved the failed pillar run is not a tube formula, epoch,
Marker, query-injection, or storage-initialization failure. In all 46 due
candidate attempts:

```text
current_cross_section_reason = CENTER_UNKNOWN
current base raw status      = UNKNOWN
actual UAV raw status        = UNKNOWN
p + 0.05 N raw status        = UNKNOWN
p - 0.05 N raw status        = UNKNOWN
raw storage ready            = true
raw query injected           = true
current sample found         = true
current sample complete      = false
```

Evidence directory:

```text
/tmp/a5g2b_r1_raw_candidate_diagnostics_20260810_003406
```

Relevant SHA-256 values:

```text
bag       debeb3e96ad5e5c16397d772f545b9c1d5f7387e2bcba84fb26a6204a452e18b
analyzer  0fb318315451bbcad781765786194afc0f45ebbc654bd429d2ca249540847ef8
report    bf14a71b43070e590a367d071c71ae390a5d90aeca4cca2e172bd58c365c72b9
```

The current raw mapper ray traversal marks traversed voxels free but does not
explicitly mark the ray origin/current camera voxel free. With 0.10 m voxels,
the base point and `p +/- 0.05 N` can remain in the same `UNKNOWN` origin
voxel. The raw tube kernel correctly fails closed at its center, so no
Candidate profile can be built.

The correction in this stage is a local query overlay: the actual UAV's own
known footprint is valid evidence that an `UNKNOWN` query point inside that
footprint is not a static environmental obstacle. This overlay exists only in
the thin raw occupancy bridge used for the due tube attempt. It is not written
into SDFMap and it does not make any point outside the current UAV footprint
free.

## 2. Proposal consistency

Read the proposal's Sections 10.2-10.6, 21, and 22.5 in context. The tube is
still constructed from the planned continuous centerline after its lifted
geometry has provided `p`, `N`, and curvature. This stage does not construct a
tube around the executed trajectory, and it does not replace the centerline
with the UAV position.

The actual position is used for one purpose only: provide a conservative local
starting certificate for raw raycasting in the physical volume already
occupied by this UAV. The queried path point must still lie inside that volume
to receive this evidence. Once a ray leaves the volume, only the real raw map
may certify it.

This preserves the intended meanings:

```text
p(w)              planned continuous base path
r(w,delta)        active lifted reference p + N delta
actual_position   measured UAV position
tube bounds        environment/curvature bounds around p(w)
self-free seed     local evidence source for UNKNOWN only
```

## 3. Mandatory references

Read completely before editing:

1. `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md`
2. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`
3. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`
4. `docs/Terra_A5_G1_Asymmetric_Tube_Cross_Section_Kernel_Prompt_2026-08-09.md`
5. `docs/Terra_A5_G2a_Navigation_Asymmetric_Tube_Integration_Prompt_2026-08-09.md`
6. `docs/Terra_A5_G2b_Raw_Occupancy_ROS_And_Certified_Segment_Prompt_2026-08-09.md`
7. `docs/Terra_A5_G2b_r1_Raw_Candidate_Failure_Diagnostics_Prompt_2026-08-10.md`
8. The complete current implementations and tests of:
   - `phase_offset_raw_occupancy_query`;
   - `PhaseOffsetMatchedAdapter::updateTubeEpoch`;
   - raw candidate diagnostics;
   - `TubeCrossSectionSolver` and raw `TubeBuilder`;
   - `TubeEpochManager`;
   - Candidate/Certified Marker helper.
9. The G2b and G2b-r1 raw pillar bags, reports, analyzers, and logs.

Do not copy from the uncompiled prototype `path_tube_builder.*` or any swarm
visualizer.

## 4. Required precondition audit

Before editing, record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --cached --name-only
git stash list
sha256sum docs/Terra_A5_G2b_r2_Self_Free_Seed_Contract_Prompt_2026-08-10.md
```

Required:

- branch `main`;
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` exists;
- no staged content;
- the dirty/untracked worktree is user-owned and must be preserved.

Stop before editing if any required precondition differs. Never reset, restore,
clean, stash-pop, move, or overwrite user files.

Because relevant integration files are untracked, record pre/post SHA-256 and
mtime manifests for the whitelist and protected files. Before ROS commands,
check for existing ROS masters/processes. Never attach to or terminate the
user's ROS graph, especially port `11311`. Use fresh isolated ports and clean
up only processes created by this stage.

## 5. Exact whitelist

Only these five files may be modified:

1. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_raw_occupancy_query.h`
2. `src/swarm_planner/bspline_traj/src/integration/phase_offset_raw_occupancy_query.cpp`
3. `src/swarm_planner/bspline_traj/test/phase_offset_raw_occupancy_query_test.cpp`
4. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
5. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

All other files are protected, including:

- this specification and all other docs;
- `bspline_traj/CMakeLists.txt`, package.xml, and every launch/RViz file;
- adapter header and all diagnostics headers/sources/tests;
- Marker helper/tests;
- every `phase_offset_core` and `phase_offset_navigation` file;
- TubeCrossSectionSolver, TubeBuilder, TubeFilter, TubeEpochManager, Runtime;
- SDFMap/local sensing/map generator;
- gvf_manager, planner, B-spline optimizer, C2, GVF, governor, SO3;
- messages, simulator, swarm, neighbor, CBF, maps, goals, and parameters.

If a required edit lies outside the whitelist, stop and report. Do not expand
scope. Existing CMake already compiles the helper and both test targets, so no
build-system edit is expected or authorized.

## 6. Self-free seed data contract

Add a small value type in the raw bridge header equivalent to:

```cpp
struct RawOccupancySelfFreeSeed {
  Eigen::Vector3d center;
  double radius;
};
```

The exact naming may differ, but it must remain an integration-layer value
type. A seed is valid only if:

```text
center is finite
radius is finite
radius > 0
```

Containment uses three-dimensional Euclidean distance and an inclusive exact
boundary:

```text
(point - center).squaredNorm() <= radius * radius
```

Do not add an epsilon, new ROS parameter, voxel expansion, localization margin,
tracking margin, map margin, or search-step allowance to the seed radius.

For the live adapter, construct a fresh seed on every due MANUAL+ESDF tube
attempt from:

```text
center = input.position
radius = config_.tube.cross_section.margins.uav_radius
       = existing raw_uav_radius
       = 0.25 m by default
```

Do not persist or union old seed positions. Do not sweep between positions.
Do not seed around `p(w)`, `r(w,delta)`, the whole centerline, preview samples,
or Marker points. Fixed/NONE and ACTIVE-only paths must not use a seed.

## 7. Exact raw classification priority

Refactor the focused bridge so classification follows this strict priority for
every query point:

```text
1. map/raw storage unavailable
     -> DistanceStatus::UNAVAILABLE

2. point nonfinite or outside map
     -> DistanceStatus::OUT_OF_MAP

3. raw map says occupied
     -> DistanceStatus::OCCUPIED

4. raw map says observed free
     -> DistanceStatus::KNOWN_FREE, evidence MAP

5. raw map says unknown AND valid seed contains point
     -> DistanceStatus::KNOWN_FREE, evidence SELF_FREE_SEED

6. raw map says unknown outside/without valid seed
     -> DistanceStatus::UNKNOWN
```

The occupied check must precede the self-free overlay. An occupied voxel inside
the seed stays `OCCUPIED` under all circumstances. OUT_OF_MAP and UNAVAILABLE
also cannot be overlaid. The self-free seed may convert only raw `UNKNOWN` to
the navigation query's existing `KNOWN_FREE` status.

Observed free remains observed free even inside the seed. Seed evidence does
not alter the public `phase_offset_navigation::DistanceStatus` enum and must
not add a navigation dependency on SDFMap.

## 8. Test-visible evidence without schema changes

The implementation must expose a focused testable classification/probe result
that distinguishes at least:

```text
UNAVAILABLE
OUT_OF_MAP
RAW_OCCUPIED
RAW_UNKNOWN
RAW_KNOWN_FREE
KNOWN_FREE_BY_SELF
```

This evidence enum/result belongs only to the raw bridge integration helper.
It must not be added to `phase_offset_navigation`, ROS messages, or existing
diagnostic schemas.

Provide an equivalent of:

```cpp
RawOccupancyProbe makeRawOccupancyProbe(
    SDFMap* map,
    const RawOccupancySelfFreeSeed& seed);

phase_offset_navigation::RawOccupancyQuery makeRawOccupancyQuery(
    SDFMap* map,
    const RawOccupancySelfFreeSeed& seed);
```

The exact API may differ, provided:

- the query is a thin projection of the same probe classification;
- map and seed are captured safely for the synchronous due update;
- the existing no-seed call remains available or all authorized call sites are
  updated without changing no-seed behavior;
- the probe can prove in unit tests whether `KNOWN_FREE` came from the raw map
  or the self seed;
- no ESDF distance, inflated occupancy, planner state, ROS publisher, or tube
  formula enters this helper.

Do not append evidence fields to the existing 83-field manual diagnostics,
49-field epoch diagnostics, or 49-field raw-candidate diagnostics.

## 9. Adapter wiring

Change only the raw-query construction inside the existing due
`PhaseOffsetMatchedAdapter::updateTubeEpoch` flow.

For a due MANUAL+ESDF attempt with initialized raw storage:

1. build the fresh seed from actual `input.position` and existing
   `raw_uav_radius`;
2. create one seeded raw query;
3. pass that exact query object to `epoch_input.raw_occupancy_query`;
4. pass the same query object to the existing raw-candidate diagnostics;
5. do not query or seed at 50 Hz non-due cycles.

The existing raw-storage-missing compatibility behavior is unchanged. Seed
logic must not turn an unavailable map into an injected raw query.

Do not alter:

- update rate/scheduling or `map_observation_sequence`;
- preview sampling;
- candidate/active ownership or installation rules;
- retained `w` or `delta`;
- gate, selected control, failure latch, or fallback;
- Runtime input/output;
- Candidate/Certified Marker decisions or geometry;
- publication rate or any diagnostics field/index;
- raw tube margins/formulas, filter, or certified segment semantics.

## 10. Required deterministic helper tests

Extend `phase_offset_raw_occupancy_query_test.cpp` to cover at least:

1. raw `UNKNOWN` strictly inside a valid seed becomes navigation
   `KNOWN_FREE`, with probe evidence `KNOWN_FREE_BY_SELF`;
2. raw `UNKNOWN` strictly outside the seed remains `UNKNOWN`;
3. raw `OCCUPIED` inside the seed remains `OCCUPIED`;
4. raw observed free inside/outside the seed remains `KNOWN_FREE` with raw-map
   evidence, not self evidence;
5. OUT_OF_MAP cannot be overlaid by a seed centered outside/near a boundary;
6. missing raw storage remains UNAVAILABLE even with a valid seed;
7. nonfinite point is OUT_OF_MAP and cannot be overlaid;
8. nonfinite center disables the seed;
9. nonfinite, zero, and negative radius disable the seed;
10. a point exactly at the radius is included deterministically;
11. a point just beyond the radius remains UNKNOWN;
12. an UNKNOWN path/base point farther than the UAV radius from actual
    position is not certified by the seed;
13. the existing inflated-only raw-free test still proves no ESDF or inflated
    occupancy read is used;
14. the existing no-seed mapping remains byte-for-byte equivalent in status
    semantics.

Poison ESDF/inflated buffers where useful so accidental reads fail the test.

## 11. Required adapter tests

Extend `phase_offset_matched_adapter_test.cpp` with focused cases proving:

1. On a due MANUAL+ESDF attempt, an otherwise valid straight path whose current
   origin voxel and immediate `+/- ray_step` probes are raw UNKNOWN can use the
   actual-position seed to pass the current center and build a candidate when
   raw observed free begins outside the seed.
2. The produced raw candidate still uses the G1/G2 raw geometry and is not
   capped by legacy `max_offset=0.20`; at least one deterministic sample width
   is greater than `0.45 m`.
3. An occupied current voxel inside the seed still yields an incomplete
   candidate/current occupied reason and no unsafe active install.
4. Moving the path/base point beyond `raw_uav_radius` from actual position
   leaves its UNKNOWN center invalid.
5. The exact same seeded query is reflected by raw-candidate diagnostic probes
   on that due cycle: current/actual/immediate probe statuses agree with the
   candidate attempt.
6. Non-due cycles do not create or apply a new seed/query/diagnostic snapshot.
7. FIXED and NONE paths preserve current behavior and do not use raw seed
   semantics.
8. ACTIVE-only baseline behavior remains unchanged.
9. Manual, epoch, and raw-candidate schemas remain exactly 83, 49, and 49
   fields, all finite.
10. Gate-100, matched-port recurrence, candidate/active ownership, and Marker
    certificate tests remain unchanged and pass.

Use only test-map setup in the already authorized test file. Do not modify
production SDFMap or local sensing for tests.

## 12. Build and regression requirements

After edits, run:

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_phase_offset_core
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

Directly run and report at least:

```text
phase_offset_raw_occupancy_query_test
phase_offset_matched_adapter_test
phase_offset_raw_candidate_diagnostics_test
phase_offset_tube_cross_section_test
phase_offset_tube_builder_test
phase_offset_tube_filter_test
phase_offset_tube_epoch_manager_test
phase_offset_tube_epoch_diagnostics_test
phase_offset_tube_markers_test
phase_offset_runtime_test
```

Run tests from clean current results, removing only generated test-result
artifacts if necessary. Do not delete or alter source files. Do not repair an
unrelated historical failure.

## 13. Isolated ROS acceptance

First verify no task-owned ROS process from an earlier stage remains. Record
the user's existing ROS masters/processes and leave them untouched.

Use a fresh isolated ROS port, not `11311`. Run exactly the same scenario as
G2b/G2b-r1:

```text
launch              phase_offset_esdf_tube_single.launch
map                 original pillar.pcd
goal                (8, 0, 1)
mode/source          manual / ESDF raw path
map refresh          3.0 s
search extent        3.0 m
ray step             0.05 m
raw UAV radius       0.25 m
r_eff                0.55 m
all planner/control/map/target/speed parameters unchanged
```

Do not add a manual-click obstacle and do not modify the launch file. Send the
same one-shot goal only after the isolated graph is ready.

Record a new bag under a new `/tmp/a5g2b_r2_*` directory containing:

```text
/formation_planning/phase_offset_manual/diagnostics
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/tube
```

Also retain launch/goal logs, parameters, process manifest, CSV, JSON report,
and analyzer source. Print SHA-256 for bag, analyzer, CSV, and JSON.

Collect enough post-path due attempts to determine the acceptance results; a
15-second stable epoch is explicitly not required.

## 14. ROS analyzer and acceptance criteria

Create the analyzer only under `/tmp`. It must:

- assert every manual row has exactly 83 fields;
- assert every epoch row has exactly 49 fields;
- assert every raw-candidate row has exactly 49 fields;
- assert all fields and Marker points are finite;
- align all diagnostics and Marker actions by rosbag message time;
- reject mixed ADD/DELETE actions among IDs 0/1/2;
- report raw due-attempt rate;
- group current cross-section reason and current/actual/plus/minus raw status;
- report storage-ready/query-injected/sample-found/sample-complete counts;
- report current `r_eff`, bounds, truncation, candidate/active sample counts,
  epoch states/reasons, failure/fallback, and safety-priority counts;
- pair Candidate lower/upper boundary points and compute widths;
- verify exact six ribbon vertices per adjacent sample pair;
- report Candidate/Certified ADD/DELETE counts and diagnostic/Marker mismatch;
- state that Marker width is boundary separation, not proof of symmetry;
- make no coherent raw-map snapshot/revision claim.

Acceptance requires all of:

```text
raw due-attempt rate                    8-12 Hz while due
raw storage ready/query injected        observed true
current/actual raw probe status         KNOWN_FREE in at least one due row
Candidate Marker                        at least one three-ID ADD frame
Candidate width                         at least one > 0.45 m
maximum Candidate width                 <= 2*(3.0-0.55)=4.90 m
                                          plus only documented ray/grid tolerance
current effective radius                0.55 m
manual/epoch/raw schemas                 83/49/49 exactly
Marker/schema/action mismatch            0
mixed action arrays                       0
nonfinite payload/point count             0
```

Candidate may be ADD while Certified is DELETE. Certified ADD is desirable but
is not manufactured by changing gate, tracking, parameters, or timing. Report
the actual epoch and certificate state honestly.

The Candidate width requirement proves the environmental tube is not the old
fixed total width `0.40 m`. It does not prove left/right symmetry.

## 15. Mandatory stop-on-failure behavior

If the self-free seed is active but no Candidate ADD occurs after path/map
readiness:

1. use the existing raw-candidate diagnostics to identify the new exact current
   reason and four probe statuses;
2. retain the bag/report/logs;
3. stop immediately after the required build/tests and self-audit;
4. do not tune radius, margins, search extent, map refresh, planner, speed,
   target, or timing;
5. do not add another overlay or modify SDFMap.

Also stop without scope expansion if:

- occupied is ever converted to known free;
- seed outside-radius UNKNOWN is converted to known free;
- any required edit lies outside the whitelist;
- preconditions/protected hashes differ unexpectedly;
- ROS cannot be isolated safely;
- a schema/Marker mismatch appears;
- an unrelated historical issue blocks a workspace-wide aggregate.

## 16. Forbidden changes

Do not:

- modify SDFMap, raycastProcess, local sensing, occupancy buffers, inflation,
  ESDF, map refresh, or manual-click handling;
- change global UNKNOWN semantics or mark a complete voxel neighborhood free;
- let the seed override OCCUPIED, OUT_OF_MAP, or UNAVAILABLE;
- grow the seed by map/localization/tracking margins or ray step;
- persist, union, trail, or sweep seeds over time;
- seed the base path, active reference, complete preview, or candidate samples;
- call `getDistance`, `getInflateOccupancy`, or read inflated/ESDF storage in
  the raw bridge;
- change G1 tube mathematics, `r_eff`, filter, epoch install logic, Runtime,
  Marker helper, C2, planner, optimizer, control, gate, or failure latch;
- change 83/49/49 schemas or add a new ROS topic;
- change launch, map, goal, speed, planner weights, maximum velocity, or safety
  parameters;
- claim the raw bridge covers manual-click obstacles;
- add swarm, neighbor, CBF, emergency, `U_i^+`, replanning, snapshot, or
  continuation logic;
- commit, stage, tag, branch, or push.

## 17. Dependency and invariant audit

Required final searches must prove:

- SDFMap ownership remains only in the thin integration helper/adapter/test;
- `phase_offset_navigation` remains ROS/SDFMap-free;
- the raw helper has no ESDF/inflated query;
- Runtime still does not own DistanceQuery, TubeBuilder, or TubeFilter;
- gvf_manager remains unchanged;
- no PositionCommand publisher was added;
- no planner, C2, swarm, neighbor, CBF, snapshot, continuation, or new
  diagnostics code was added;
- candidate and active profiles remain distinct;
- Fixed/NONE/ACTIVE baseline and Gate-100 behavior remain intact.

Compare whitelist and protected SHA/mtime manifests before and after. Explain
every changed hash; there must be no unauthorized change.

## 18. Final self-audit and report

Before stopping, run and record:

```bash
git diff --check
git diff --cached --name-only
git status --short
git branch --show-current
git rev-parse HEAD
git stash list
```

Verify all isolated ROS ports are released and all task-owned processes are
gone. Do not touch user processes.

The final report must state:

1. exact files modified;
2. exact self-free seed center/radius/validity/containment contract;
3. exact classification priority and proof OCCUPIED cannot be overlaid;
4. probe/evidence API and proof no diagnostics schema changed;
5. adapter wiring and proof the same due-cycle query feeds builder and raw
   diagnostics;
6. direct and aggregate test counts;
7. ROS bag/CSV/JSON/analyzer/log paths and SHA-256;
8. distributions of current reason and four raw probe statuses;
9. Candidate/Certified action counts, widths, `r_eff`, epoch state, failures,
   and mismatches;
10. whether every Section 14 criterion passed;
11. exact new reason and stop statement if Candidate still never ADDs;
12. dependency/protected-hash/diff/staged/HEAD/stash/process audit;
13. explicit statement that SDFMap, planner, tube theory, margins, Runtime,
    control, Marker, map, goal, speed, and 83/49/49 schemas were unchanged;
14. explicit statement that manual-click obstacles remain outside this raw
    bridge contract;
15. explicit statement that no 15-second stable-epoch rule was used;
16. explicit stop statement.

A5 remains blocked after A5-G2b-r2. A new dedicated execution specification is
required for any further fix, G3 control/hybrid work, A5.3, A6, A7, or
multi-UAV work.
