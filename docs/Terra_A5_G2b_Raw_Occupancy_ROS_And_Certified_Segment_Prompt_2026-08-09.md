# A5-G2b Execution Specification: Raw Occupancy ROS Bridge and Certified Tube Segment

Date: 2026-08-09

Execution model: one `gpt-5.6-terra` agent at `reasoning_effort=max`.

`AUTO_ADVANCE=false`.

This document authorizes only A5-G2b. Complete the implementation, builds,
tests, isolated ROS evidence, dependency audit, and self-audit described here,
then stop. Do not enter A5-G3, A5.3, A6, A7, or multi-UAV work.

## 1. Purpose and current state

A5-G1 proved a pure proposal-consistent asymmetric cross-section kernel.

A5-G2a integrated that kernel into a new `phase_offset_navigation` raw
occupancy path while preserving the current legacy `DistanceQuery` path for
existing ROS callers. G2a also corrected one-sided installation semantics and
removed the TubeFilter global envelope fallback.

The current ROS adapter still supplies only the legacy inflated-ESDF
`DistanceQuery`; therefore current RViz behavior is still legacy behavior:

```text
inflated ESDF
  + legacy max_offset=0.20
  + legacy six-field erosion
  -> legacy candidate/active profiles
```

A5-G2b must switch the ESDF-selected ROS adapter to an explicit raw occupancy
query and prove that Candidate/Certified Markers now read the new environment
tube. It must also prevent one invalid future preview sample from erasing a
currently valid tube by retaining the maximal contiguous certified segment
that contains the current phase sample.

The desired ROS dataflow after this stage is:

```text
SDFMap raw occupancy_buffer
  -> thin RawOccupancyQuery bridge
  -> 10 Hz TubeBuilder::buildRawOccupancy
  -> contiguous certified segment containing current w
  -> TubeFilter
  -> candidate TubeProfile
  -> TubeEpochManager install/preserve active
  -> Candidate Marker reads candidate environment bounds
  -> Certified Marker reads installed active environment bounds
  -> 50 Hz Runtime consumes immutable active profile
```

Planner inflation remains unchanged for the original planner but is not used
by the new tube geometry query.

## 2. Mandatory references

Read completely before editing:

1. `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md`
2. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`
3. Proposal Sections 9, 10, 10.2-10.6, 13.5, 13.6, and 17 in context
4. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`
5. `docs/Terra_A5_G1_Asymmetric_Tube_Cross_Section_Kernel_Prompt_2026-08-09.md`
6. `docs/Terra_A5_G2a_Navigation_Asymmetric_Tube_Integration_Prompt_2026-08-09.md`
7. `docs/DeepSeek_A5_R2_Multirate_Runtime_ROS_Integration_Prompt_2026-08-09.md`
8. Current complete implementations/tests of:
   - `TubeCrossSectionSolver`
   - `TubeBuilder::buildRawOccupancy`
   - `TubeFilter`
   - `TubeEpochManager`
   - `PhaseOffsetMatchedAdapter`
   - tube Marker helper
   - 49-field epoch diagnostics
   - 83-field manual diagnostics
   - relevant launch files

Do not copy from the uncompiled prototype `path_tube_builder.*` or swarm
visualizer.

## 3. Required precondition audit

Before editing, record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --cached --name-only
git stash list
sha256sum docs/Terra_A5_G2b_Raw_Occupancy_ROS_And_Certified_Segment_Prompt_2026-08-09.md
```

Required:

- branch `main`;
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` exists;
- no staged content;
- the dirty worktree, A5-R2, A5-G1, A5-G2a, and unrelated untracked files are
  user-owned and must be preserved.

Stop before editing if these conditions fail. Never reset, restore, clean,
stash-pop, or overwrite the worktree.

Because the relevant packages are untracked directories, capture pre/post
SHA-256 and mtimes for every protected existing file named in Section 5.

Before any ROS command, check for existing ROS masters and record their ports
and processes. Do not attach to or terminate user-owned ROS processes. Use
isolated ports and clean up only processes started by this stage.

## 4. Exact whitelist

Only these paths may be added or modified:

### phase_offset_navigation

1. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h`
2. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h`
3. `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp`
4. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h`
5. `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp`
6. `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp`
7. `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp`
8. `src/swarm_planner/phase_offset/phase_offset_navigation/README.md`

### bspline_race integration

9. `src/swarm_planner/bspline_traj/CMakeLists.txt`
10. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_raw_occupancy_query.h` (new)
11. `src/swarm_planner/bspline_traj/src/integration/phase_offset_raw_occupancy_query.cpp` (new)
12. `src/swarm_planner/bspline_traj/test/phase_offset_raw_occupancy_query_test.cpp` (new)
13. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
14. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
15. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

### launch

16. `src/swarm_planner/bspline_traj/launch/test_gvf.launch`
17. `src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch`

All other paths are protected. In particular, do not modify:

- `TubeCrossSectionSolver` header/source/test;
- TubeFilter header/source/test from G2a;
- TubeEpochManager header;
- Runtime or Runtime tests;
- phase_offset_core;
- Marker helper or Marker tests;
- 49-field diagnostics header/source/test;
- 83-field diagnostics enum/schema;
- gvf_manager, planner, C2, SDFMap, GVF, governor, SO3, simulator, RViz,
  messages, swarm, or CBF;
- fixed/manual/other launch files;
- this specification.

If a required edit lies outside this whitelist, stop and report. Do not expand
scope.

## 5. Raw SDFMap occupancy bridge

Create one focused integration helper. The header may forward-declare `SDFMap`;
only the `.cpp` may include `<plan_env/sdf_map.h>`.

Provide an equivalent of:

```cpp
phase_offset_navigation::RawOccupancyQuery makeRawOccupancyQuery(SDFMap* map);
```

Required translation for each point:

```text
map == nullptr                        -> UNAVAILABLE
point nonfinite or !map->isInMap      -> OUT_OF_MAP
map->isUnknown(point)                 -> UNKNOWN
map->getOccupancy(point) != 0         -> OCCUPIED
otherwise                             -> KNOWN_FREE
```

Order matters: check in-map before `isUnknown`, and unknown before raw
occupancy.

The helper must not call:

```text
getDistance
getInflateOccupancy
occupancy_buffer_inflate
```

This stage intentionally queries the raw occupancy layer. Existing planner
inflation `0.099 m` remains untouched and is not part of the tube query.

The existing public `SDFMap::getOccupancy` does not include the manual-click
buffer. Do not modify SDFMap to expand scope. G2b ROS evidence must use the
original pillar point-cloud map with no manual-click obstacles. State this
limitation explicitly in the final report.

Required helper tests must prove:

- null, nonfinite, out-of-map, unknown, raw occupied, and raw free mapping;
- a voxel that is only inflated but not raw occupied returns `KNOWN_FREE`;
- the helper never reads scalar ESDF distance;
- no ROS node or publisher is needed for the test.

## 6. Explicit raw tube configuration

Add launch/adapter parameters for the raw cross-section contract. Suggested
names and required defaults:

```text
phase_offset/tube/environment_search_extent       3.0
phase_offset/tube/raw_ray_step                    0.05
phase_offset/tube/boundary_tolerance              0.01
phase_offset/tube/raw_regularity_margin           0.10
phase_offset/tube/curvature_epsilon                1e-9
phase_offset/tube/raw_uav_radius                   0.25
phase_offset/tube/raw_map_uncertainty              0.10
phase_offset/tube/raw_localization_uncertainty     0.05
phase_offset/tube/raw_tracking_error_bound         0.15
```

Launch argument names should follow the existing
`phase_offset_tube_*` convention.

This gives the explicit default proposal contract:

```text
r_eff = 0.25 + 0.10 + 0.05 + 0.15 = 0.55 m
```

`raw_map_uncertainty=0.10` includes the intended map/grid discretization
allowance for this stage. Do not additionally add legacy:

```text
map_margin + extra_margin + discretization_margin
```

The old launch parameters remain for the legacy compatibility path and Fixed
regression, but the raw path must not consume them.

`test_gvf.launch` defaults must remain:

```text
phase_offset_mode=disabled
phase_offset_manual_tube_source=none
sdf_map_buffer_refresh_period=3.0
circle_test enable=false
circle_test auto_start=false
```

Do not change speed, acceleration, map, target, saturation, planner weights, or
tracking values to make ROS evidence pass.

## 7. Runtime configuration consistency

For ESDF source after the raw bridge is connected:

- Runtime tracking bound must use
  `cross_section.margins.tracking_error_bound`;
- Runtime regularity margin must use
  `cross_section.regularity_margin`;
- Fixed source retains its current legacy values;
- environment search extent must never become a Runtime/controller offset cap;
- `interior_margin` remains a controller/interior policy fact and does not
  shrink stored environment bounds.

Do not change Runtime code.

## 8. Adapter selection and query ownership

When `tube_source == ESDF`, every due 10 Hz candidate update must set:

```cpp
epoch_input.raw_occupancy_query = makeRawOccupancyQuery(input.sdf_map);
```

The raw query must be the reason G2a selects `buildRawOccupancy`. It must not be
set for FIXED or NONE.

The adapter may continue to construct the legacy `DistanceQuery` only for
compatibility or existing non-raw diagnostics, but G2a raw-path tests guarantee
that TubeBuilder/TubeEpochManager do not call it when the raw query is present.

Candidate and active ownership from R2 remains unchanged:

```text
candidate_profile = latest raw candidate
active_profile = immutable installed active profile
```

Do not move builder/filter ownership back into Runtime. Do not rebuild at 50 Hz.

## 9. Certified segment containing current phase

The G2a raw builder currently requires every requested preview sample to be
valid. G2b must change only the raw path so that a future invalid sample does
not erase a valid current tube.

Extend the raw builder call with the current/anchor phase `current_w`.
`TubeEpochManager` passes `input.current_path.w`.

For the requested ordered preview:

1. Evaluate each raw cross-section using the G1 solver.
2. Locate the sample matching `current_w` within a deterministic tolerance.
3. The current sample itself must be valid. If it is UNKNOWN, OUT_OF_MAP,
   UNAVAILABLE, OCCUPIED, geometrically invalid, or cross-section empty, the
   candidate is incomplete.
4. If current is valid, retain the maximal contiguous run of valid samples
   containing current.
5. Invalid samples before or after that run are not included in the candidate's
   `samples` vector.
6. Preserve the first truncation reason/w in diagnostics.
7. Set `preview_start_w/preview_end_w` to the retained certified segment.
8. Preserve separately the originally requested preview start/end.
9. Mark whether the requested preview was truncated before and/or after the
   current phase.
10. `raw_complete=true` means the retained current-containing segment is
    complete, not that the entire originally requested preview was observed.
11. `obstacle_certified=true` means every retained sample is conservatively
    certified.
12. TubeFilter operates only on the retained segment.

Add focused profile fields equivalent to:

```text
requested_preview_start_w
requested_preview_end_w
certified_segment_start_w
certified_segment_end_w
certified_segment_truncated_before
certified_segment_truncated_after
first_truncated_w
first_truncated_reason
```

Do not append these to the 83- or 49-field ROS schemas in G2b.

If the retained segment contains fewer than two samples, the profile may remain
mathematically valid but current Marker helpers will DELETE because they need a
surface segment. Do not change Marker helper behavior in this stage; test and
report this limitation.

## 10. Hybrid meaning of truncation

Existing manager logic computes:

```text
certified_forward_w = certified_segment_end_w - current_w
```

Required behavior:

- sufficient certified forward horizon and retained-delta containment may
  remain `ROLLING`;
- short certified forward horizon produces `REPLAN_REQUIRED`;
- current valid + future UNKNOWN must not become `CANDIDATE_INCOMPLETE` merely
  because the requested far preview was truncated;
- current invalid remains `WAITING_FOR_CANDIDATE`;
- valid candidate excluding retained delta remains `SAFETY_PRIORITY` and does
  not overwrite active, per G2a;
- valid candidate containing retained delta but excluding zero remains
  `REPLAN_REQUIRED/CENTERLINE_OUTSIDE_CANDIDATE_TUBE`;
- do not implement dynamic reachability, `U_i^+`, new emergency states, or new
  planner callbacks.

## 11. Marker semantics

Do not modify the Marker helper. Prove that its existing correct geometry now
receives the correct profiles:

```text
Candidate topic -> candidate_profile retained environment segment
Certified topic -> installed active_profile retained environment segment
lower point     -> p + N * filtered_lower
upper point     -> p + N * filtered_upper
```

The raw environment profile must not be intersected with legacy
`max_offset=0.20` or a new controller state clamp before Marker creation.

Candidate remains uncertified visualization. Certified ADD still requires the
existing display certificate. Candidate geometry may remain visible when
Certified is DELETE.

No third Marker topic and no RViz modification are authorized.

## 12. Diagnostic schema invariants

Manual diagnostics remain exactly 83 fields. Epoch diagnostics remain exactly
49 fields. Do not move, append, rename, or reinterpret field indices.

Required runtime evidence for the raw path uses existing fields:

- required reference/actual clearance must report the explicit raw contract,
  expected default `0.55 m`, not legacy `0.70/0.55` accounting;
- candidate sample count may shrink when the requested preview is truncated;
- certified forward distance reflects the retained segment;
- `map_observation_is_snapshot` remains `0`;
- never claim coherent map revision/snapshot;
- `map_observation_sequence` remains only an opaque observation token.

Legacy `reference_signed_distance/actual_signed_distance` fields must remain
finite but must not claim raw Euclidean ESDF measurements when none were made.

## 13. Required navigation tests

Keep G1/G2a tests passing and add at least:

1. current sample valid, future center UNKNOWN:
   - candidate retains current-containing segment;
   - `raw_complete=true`;
   - truncated-after=true;
   - preview end equals last valid sample;
   - diagnostics preserve future invalid reason.
2. invalid sample before current but current/future valid:
   - segment truncates before;
   - current candidate remains complete.
3. current sample UNKNOWN/occupied/empty:
   - candidate incomplete.
4. invalid samples on both sides:
   - only maximal valid run containing current retained.
5. short retained forward horizon requests replan without deleting candidate.
6. sufficient retained forward horizon rolls.
7. one-sided G2a install/reject semantics remain unchanged after segment logic.
8. legacy builder path remains byte-for-byte behavior compatible under its
   existing tests.

## 14. Required integration tests

Add tests for the new raw bridge and update adapter tests to prove:

1. ESDF source supplies a nonempty raw query to `TubeEpochManager`.
2. FIXED/NONE do not supply or call raw occupancy.
3. Raw configuration parameters load exactly and give default `r_eff=0.55`.
4. An open raw test map can produce environment width greater than `0.40 m`
   despite legacy `max_offset=0.20` remaining configured.
5. Inflated-only occupancy is ignored by raw tube query.
6. Raw one-sided candidate remains Candidate geometry; active installation
   follows G2a continuation rules.
7. Future unknown truncates sample count/end-w but Candidate remains complete.
8. Current unknown causes Candidate DELETE/incomplete.
9. Candidate and Certified continue to use separate profile pointers.
10. 83- and 49-field payload lengths remain exact.
11. Candidate/Certified Marker action decisions continue matching their
    respective profile/certificate inputs.
12. Adapter update rate remains 10 Hz path/event driven, not 50 Hz rebuild.

Do not add a planner/map simulator to unit tests.

## 15. Build and regression requirements

Run:

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_phase_offset_core
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

Directly run and report counts for:

```text
phase_offset_tube_cross_section_test
phase_offset_tube_builder_test
phase_offset_tube_filter_test
phase_offset_tube_epoch_manager_test
phase_offset_raw_occupancy_query_test
phase_offset_matched_adapter_test
phase_offset_tube_markers_test
phase_offset_tube_epoch_diagnostics_test
```

Do not fix unrelated failures.

## 16. Isolated ROS acceptance

Use fresh isolated ROS ports. Do not use or terminate a user master.

### ROS-0 baseline

Bare `test_gvf.launch` with defaults:

- actual parameters are disabled/none/refresh 3.0;
- manual diagnostics, candidate, certified, and epoch topics have no publisher;
- `/position_cmd` has only `/formation_planning`;
- original point-to-point `(8,0,0)` reaches the existing baseline tolerance;
- no raw tube query is exercised.

### ROS-1 fixed regression

Run the existing fixed launch/config without changing its target or control
parameters.

Acceptance is regression-only, not the obsolete 15-second stable-epoch rule:

- fixed source never queries raw occupancy;
- diagnostics remain 83/49 fields;
- Candidate/Certified actions match profile/certificate;
- no mixed ADD/DELETE within one MarkerArray;
- failure/fallback behavior is reported, not tuned away;
- any previously known tracking/safety-priority behavior must be reported but
  is not repaired in G2b.

### ROS-2 raw occupancy pillar run

Use `phase_offset_esdf_tube_single.launch` with the original `pillar.pcd`,
default raw parameters, `sdf_map_buffer_refresh_period=3.0`, and the existing
stage goal used by the launch/test workflow. Do not change the map, target,
speed, planner, or margins after observing results.

Record raw bag, CSV, and JSON report under a new `/tmp/a5g2b_*` directory.

Required evidence:

- adapter candidate build rate is within `[8,12] Hz` while due;
- diagnostics are exactly 83 and 49 fields on every frame;
- epoch `map_observation_is_snapshot=0`;
- raw required clearance reports `0.55 m` with the default contract;
- at least one Candidate ADD frame has two boundaries and ribbon;
- at least one paired Candidate cross-section has width strictly greater than
  `0.40 m + 0.05 m` tolerance, proving legacy `max_offset=0.20` is not the
  environmental cap;
- no Candidate width exceeds `2*(3.0-0.55)` beyond ray/grid tolerance;
- Candidate geometry contains finite points and exact six ribbon vertices per
  adjacent sample pair;
- Candidate may remain ADD while Certified is DELETE;
- Candidate/Certified actions agree with diagnostics/profile certificate with
  zero mismatches under the analyzer's matching tolerance;
- a retained current-containing truncated segment, if observed, remains
  Candidate ADD and reports reduced certified forward horizon rather than
  candidate incomplete;
- if no natural truncation occurs, the unit/integration tests are the formal
  truncation evidence; do not force map/target changes to manufacture it.

Do not require a 15-second stable active epoch. A rolling local tube may update
with map observations. Report epoch/update statistics without treating normal
rolling updates as failure.

Stop ROS-2 immediately and do not tune parameters if:

- no Candidate ADD occurs after path/map readiness;
- all Candidate widths remain capped near `0.40 m`;
- raw required clearance still reports legacy accounting;
- current sample becomes invalid and safety state is entered;
- a build/schema/Marker inconsistency occurs.

Do not run additional parameter groups to evade a stop condition.

## 17. ROS evidence analyzer

Create any analyzer only under `/tmp`, not in the repository. It must:

- assert every manual row is 83 fields;
- assert every epoch row is 49 fields;
- align Marker arrays and diagnostics by rosbag message time;
- reject mixed actions within IDs 0/1/2;
- compute Candidate update rate;
- pair lower/upper marker points by index and compute finite widths;
- verify ribbon vertex count;
- report Candidate ADD/DELETE and Certified ADD/DELETE counts;
- report required clearance, candidate/active sample counts, certified forward
  horizon, epoch states/reasons, safety-priority and failure counts;
- state that marker width is boundary separation, not proof of centerline
  symmetry/asymmetry without a base-point association;
- output CSV and JSON with exact row lengths;
- print its SHA-256.

Do not infer raw map revision or coherent snapshot from observation sequence.

## 18. Dependency and architecture audit

Required boundaries:

- raw SDFMap ownership remains only in the thin integration helper/adapter;
- `phase_offset_navigation` remains ROS/SDFMap-free;
- Runtime still has no DistanceQuery, TubeBuilder, or TubeFilter ownership;
- gvf_manager remains unchanged;
- no PositionCommand publisher is added;
- no planner, C2, swarm, neighbor, CBF, snapshot, or continuation logic is
  added;
- Marker helper remains read-only geometry.

Search modified files for forbidden dependencies and report results.

## 19. Forbidden changes

Do not:

- modify SDFMap or its inflation;
- use `getInflateOccupancy` or `getDistance` in the raw query helper;
- change planner/B-spline weights;
- change speed, acceleration, map, target, saturation, tracking bound, or UAV
  radius to make evidence pass;
- change Runtime, PortProjector, matched port, phase/delta update, C2, planner,
  governor, or SO3;
- add emergency states, `U_i^+`, dynamic reachability, or new replanning
  callbacks;
- modify Marker helper, RViz, diagnostics schemas, messages, swarm, or CBF;
- claim manual-click obstacles are covered by the raw query;
- claim coherent map snapshot/revision;
- remove the legacy navigation path in G2b;
- commit, stage, tag, branch, or push.

## 20. Stop conditions

Stop and report without expanding scope if:

- a required edit is outside the whitelist;
- branch/HEAD/stash/staged/protected-hash preconditions fail;
- raw occupancy cannot be bridged without modifying SDFMap;
- certified segment semantics require Runtime or planner changes;
- ROS is unavailable;
- an existing user ROS graph cannot be isolated safely;
- the raw pillar run hits any stop condition in Section 16;
- an unrelated historical defect blocks workspace-wide reporting.

Preserve the worktree and evidence. Do not fix unrelated problems.

## 21. Final self-audit and report

Before stopping:

```bash
git diff --check
git diff --cached --name-only
git status --short
git rev-parse HEAD
git stash list
```

Verify pre/post hashes, mtimes, ROS port cleanup, and no unauthorized file
changes.

The final report must state:

1. exact files added/modified;
2. raw occupancy status mapping and proof inflated-only voxels are ignored;
3. exact raw configuration and `r_eff=0.55 m` accounting;
4. certified-segment/truncation semantics and tests;
5. proof ROS ESDF source selects the raw path;
6. proof Candidate Marker is no longer capped at total width `0.40 m`;
7. Candidate/Certified action counts and schema consistency;
8. all direct/aggregate test counts;
9. raw bag/CSV/JSON/analyzer paths and analyzer SHA;
10. ROS-0/1/2 results and any stop condition;
11. dependency, protected-hash, diff, staged, HEAD, stash, and process audit;
12. explicit manual-click-map limitation;
13. explicit statement that no 15-second stable-epoch requirement was used;
14. explicit stop statement.

A5 remains blocked after A5-G2b. A separate execution specification is
required for any G3 control/hybrid extension, A5.3, A6, A7, or multi-UAV work.
