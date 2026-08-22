# A5-G2b-r1 Execution Specification: Raw Candidate Failure Diagnostics

Date: 2026-08-10

Execution model: one `gpt-5.6-terra` agent at `reasoning_effort=max`.

`AUTO_ADVANCE=false`.

This document authorizes only A5-G2b-r1. It is a diagnosis stage, not a tube
geometry, map, planner, Runtime, control, or parameter-fix stage. Add the
focused diagnostic evidence, rerun the same isolated raw-pillar scenario,
identify the exact current-sample failure, self-audit, and stop.

Do not enter a fix stage, A5-G3, A5.3, A6, A7, or multi-UAV work.

## 1. Known evidence and unresolved question

A5-G2b implementation and unit/regression tests passed:

```text
520 tests, 0 failures
raw bridge tests pass
raw configuration resolves search_extent=3.0 and r_eff=0.55 m
```

The isolated raw-pillar ROS run failed with:

```text
candidate update rate       10.89895 Hz
manual schema               216/216 rows have 83 fields
epoch schema                216/216 rows have 49 fields
Candidate ADD               0
Candidate DELETE            216
Certified ADD               0
Certified DELETE            216
epoch state                 WAITING_FOR_CANDIDATE
epoch reason                CANDIDATE_INCOMPLETE
candidate sample count      22-23
```

Evidence directory:

```text
/tmp/a5g2b_ros2_u1brq9
```

The current 83-field manual diagnostics read the legacy active-profile view.
Because no active profile was installed, they do not expose the latest raw
candidate's per-sample cross-section reason. The 49-field epoch diagnostics
only report the aggregate `CANDIDATE_INCOMPLETE` reason.

Therefore the exact cause is not yet proven. Plausible causes include:

- current base-path cell is raw `UNKNOWN`;
- actual-position cell is raw `UNKNOWN`;
- current base-path cell is raw occupied;
- current cross-section is genuinely empty after obstacle bounds;
- current cross-section fails curvature/numerical checks;
- current phase sample was not matched in the requested preview;
- another precise builder failure already stored in the candidate sample.

G2b-r1 must distinguish these with direct evidence. Do not guess and do not
change behavior to make Candidate ADD.

## 2. Mandatory references

Read completely before editing:

1. `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md`
2. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`
3. `docs/Terra_A5_G1_Asymmetric_Tube_Cross_Section_Kernel_Prompt_2026-08-09.md`
4. `docs/Terra_A5_G2a_Navigation_Asymmetric_Tube_Integration_Prompt_2026-08-09.md`
5. `docs/Terra_A5_G2b_Raw_Occupancy_ROS_And_Certified_Segment_Prompt_2026-08-09.md`
6. Current complete implementations/tests of:
   - raw occupancy query helper;
   - `TubeProfile` / `TubeRawSample` raw facts;
   - `TubeBuilder::buildRawOccupancy`;
   - `TubeEpochManager`;
   - `PhaseOffsetMatchedAdapter` update and publication;
   - 83- and 49-field diagnostics.
7. Existing G2b raw-pillar bag, CSV, JSON, analyzer output, and ROS logs.

## 3. Required precondition audit

Before editing, record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --cached --name-only
git stash list
sha256sum docs/Terra_A5_G2b_r1_Raw_Candidate_Failure_Diagnostics_Prompt_2026-08-10.md
```

Required:

- branch `main`;
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- prototype stash exists;
- no staged content;
- dirty/untracked/A5-R2/G1/G2a/G2b content is user-owned and preserved.

Stop before editing if these differ. Never reset, restore, clean, pop, or
overwrite the worktree.

Capture pre/post SHA-256 and mtimes for all protected integration/navigation
files. Check existing ROS masters before starting any ROS command. Use fresh
isolated ports and clean up only this stage's processes.

## 4. Exact whitelist

Only these paths may be added or modified:

1. `src/swarm_planner/bspline_traj/CMakeLists.txt`
2. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_raw_candidate_diagnostics.h` (new)
3. `src/swarm_planner/bspline_traj/src/integration/phase_offset_raw_candidate_diagnostics.cpp` (new)
4. `src/swarm_planner/bspline_traj/test/phase_offset_raw_candidate_diagnostics_test.cpp` (new)
5. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
6. `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
7. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

All other files are protected, including:

- every `phase_offset_navigation` and `phase_offset_core` file;
- raw occupancy bridge helper and tests;
- TubeBuilder, TubeFilter, TubeEpochManager, Runtime;
- Marker helper/RViz;
- 83- and 49-field diagnostics headers/sources/tests;
- launch files;
- SDFMap, gvf_manager, planner, C2, GVF, governor, SO3;
- messages, swarm, CBF, simulator, maps, targets, and parameters;
- this specification.

If a required edit is outside the whitelist, stop and report. Do not expand
scope.

## 5. New independent diagnostic topic

For manual ESDF/raw source only, advertise:

```text
/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
```

Message type:

```text
std_msgs/Float64MultiArray
```

It is a separate diagnostic topic. Do not change or append the existing 83- or
49-field schemas.

Publication rules:

- advertise only when mode is MANUAL and configured tube source is ESDF;
- do not advertise in baseline disabled, ACTIVE-only, FIXED, or NONE modes;
- publish only on a due candidate-update cycle, nominally 10 Hz;
- publish after the `TubeEpochManager` result is available;
- do not publish at 50 Hz between candidate attempts;
- all payload numbers must be finite;
- diagnostic generation must be read-only and must not affect candidate,
  active, Runtime, gate, Marker, or control selection.

## 6. Exact 49-field raw-candidate schema

Define an enum, field-name array, and static assertion for exactly 49 fields:

```text
0  schema_version
1  raw_source_configured
2  raw_storage_ready
3  raw_query_injected
4  tube_update_due
5  candidate_sequence
6  epoch_state
7  epoch_reason
8  candidate_raw_complete
9  candidate_filtered_complete
10 candidate_complete
11 candidate_sample_count
12 candidate_invalid_count
13 candidate_unavailable_count
14 candidate_out_of_map_count
15 candidate_unknown_count
16 candidate_occupied_count
17 candidate_insufficient_clearance_count
18 requested_preview_start_w
19 requested_preview_end_w
20 certified_segment_start_w
21 certified_segment_end_w
22 certified_segment_truncated_before
23 certified_segment_truncated_after
24 first_truncated_w
25 first_truncated_reason
26 current_w
27 current_sample_found
28 current_sample_complete
29 current_cross_section_reason
30 current_positive_ray_termination
31 current_negative_ray_termination
32 current_base_raw_status
33 actual_position_raw_status
34 current_plus_step_raw_status
35 current_minus_step_raw_status
36 current_c_plus_raw
37 current_c_minus_raw
38 current_effective_radius
39 current_obstacle_lower
40 current_obstacle_upper
41 current_curvature_lower
42 current_curvature_upper
43 current_environment_lower
44 current_environment_upper
45 current_environment_width
46 current_contains_zero
47 current_contains_preferred_delta
48 current_base_to_actual_norm
```

Use schema version `1.0`.

For missing enum/status values, use finite sentinel `-1.0`. For unavailable
numeric facts, use `0.0`. Do not use NaN or infinity.

## 7. Diagnostic helper contract

Create one focused helper independent of ROS publication. It may depend on
phase-offset navigation types and the abstract `RawOccupancyQuery`, but not on
SDFMap, planner, manager ownership, or Marker types.

Input should include equivalents of:

```text
configured source/raw booleans
storage-ready/query-injected booleans
tube-update-due
TubeEpochStatus
candidate TubeProfile pointer
current PathDifferentialState/current_w
actual position
RawOccupancyQuery used for this attempt
raw ray step
```

Required behavior:

1. Locate the candidate sample nearest `current_w` within the same deterministic
   matching tolerance used by the raw builder or a documented stricter
   tolerance.
2. Export the exact stored `TubeRawSample` facts without recomputing tube
   formulas.
3. Probe the raw query at:
   - current base path point `p`;
   - actual UAV position;
   - `p + raw_ray_step * N`;
   - `p - raw_ray_step * N`.
4. If the candidate sample has a finite nondegenerate `N`, normalize it for the
   probes. Otherwise plus/minus statuses are `-1`.
5. Probes are diagnostics only. Their results must not be fed back into the
   builder or manager.
6. Export requested/certified segment and first-truncation facts from the
   candidate profile.
7. Export exact candidate diagnostics counts.
8. Keep every output finite.

Do not call SDFMap directly from this helper. The adapter supplies the same
abstract raw query used for the candidate attempt.

## 8. Adapter wiring

On each due ESDF/raw candidate attempt:

1. Determine and store `raw_storage_ready`.
2. Store whether a raw query was injected versus legacy compatibility fallback.
3. Retain the exact `RawOccupancyQuery` object used for the attempt long enough
   to create the diagnostic snapshot after manager update.
4. Generate the 49-field snapshot from the latest candidate profile/result.
5. Store the snapshot until `publishManual` publishes it during that same due
   cycle.

Do not call the query or diagnostics on non-due cycles.

Do not alter:

- `updateTubeEpoch` scheduling;
- candidate/active ownership;
- gate or selected control;
- failure latch;
- Marker decisions;
- manual/epoch diagnostics;
- Runtime input/output.

## 9. Required deterministic tests

Add helper tests covering at least:

1. Complete valid current sample exports all exact cross-section values.
2. Current `CENTER_UNKNOWN` exports:
   - sample found;
   - sample complete false;
   - exact cross-section reason;
   - base raw status UNKNOWN.
3. Current occupied exports occupied status/reason.
4. Current genuinely empty exports obstacle-empty reason and final bounds.
5. Future truncation keeps current complete and exports truncated-after,
   first-truncated w/reason.
6. Missing current sample uses finite sentinels.
7. Invalid/missing normal uses finite plus/minus status sentinels.
8. Actual-position status is independent of current-base status.
9. Every row is exactly 49 fields and every value is finite.

Update adapter tests to prove:

- topic is advertised only for MANUAL+ESDF;
- diagnostic snapshot is generated/published only on due cycles;
- FIXED/NONE do not advertise or query it;
- existing 83/49 schemas remain exact;
- diagnostic generation causes zero changes to candidate, active epoch,
  Marker actions, Runtime execution, gate, or selected control.

## 10. Build and regression requirements

Run:

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_phase_offset_core
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

Directly run and report:

```text
phase_offset_raw_candidate_diagnostics_test
phase_offset_raw_occupancy_query_test
phase_offset_matched_adapter_test
phase_offset_tube_builder_test
phase_offset_tube_epoch_manager_test
phase_offset_tube_markers_test
phase_offset_tube_epoch_diagnostics_test
```

Do not fix unrelated failures.

## 11. Isolated ROS diagnosis

Use a fresh isolated ROS port. Do not attach to user ROS.

Run the same G2b raw-pillar launch, default parameters, original `pillar.pcd`,
refresh `3.0`, and the same point-to-point goal used by the failed G2b evidence.
Do not change map, goal, timing, speed, search extent, margins, or planner.

Record a new bag containing:

```text
manual diagnostics
epoch diagnostics
raw candidate diagnostics
Candidate Marker
Certified Marker
```

Collect at least 50 raw-candidate diagnostic rows after a path exists, or stop
earlier if the exact cause is already invariant and proven.

The analyzer must:

- assert 83, 49, and raw-candidate 49 field lengths exactly;
- group current cross-section reasons and all four raw probe statuses;
- report sample-found/sample-complete counts;
- report raw storage/query-injected counts;
- report requested/certified segment and first-truncation values;
- report `c_plus/c_minus`, obstacle/final bounds, and effective radius when
  available;
- align diagnostics with Candidate/Certified Marker actions;
- output CSV/JSON and print analyzer SHA-256;
- make no map snapshot/revision claim.

## 12. Diagnosis decision table

Report the first matching evidence-backed conclusion:

### A. Current base and actual position are UNKNOWN

```text
current_cross_section_reason = CENTER_UNKNOWN
current_base_raw_status = UNKNOWN
actual_position_raw_status = UNKNOWN
```

Conclusion: raw mapper does not mark the current vehicle/base-path cell as
observed free under the current mapping pipeline. This stage must stop; a later
map-free-seed contract decision is required.

### B. Base UNKNOWN, actual known free

Conclusion: current path/base reference lies outside observed raw free space
even though vehicle position is observed. Diagnose tracking/path-map geometry;
do not change tracking bound or path here.

### C. Center occupied

Conclusion: current base/reference is in raw occupied space. Stop; do not mask
it with a free seed.

### D. Obstacle-empty interval

Conclusion: raw `c_plus/c_minus` and `r_eff` genuinely produce
`lower>upper`. Stop; report exact numeric values.

### E. Curvature/numerical failure

Conclusion: report curvature bounds/reason. Do not change theory here.

### F. Current sample missing

Conclusion: preview/current-w matching is wrong. Stop; a later adapter/builder
contract fix specification is required.

### G. Another exact reason

Report it with raw facts and stop.

Even if diagnostics reveal an obvious fix, do not implement it in G2b-r1.

## 13. Forbidden changes

Do not:

- change raw occupancy semantics;
- mark UNKNOWN as free;
- create a free seed around the vehicle;
- modify SDFMap, occupancy updates, inflation, map refresh, or local sensing;
- change tube formulas, margins, search extent, filter, certified segment,
  EpochManager, Runtime, Marker, planner, C2, control, goal, or map;
- change any existing diagnostics schema;
- add logs at 50 Hz;
- claim coherent map snapshots/revisions;
- commit, stage, tag, branch, or push.

## 14. Stop conditions

Stop and report without expanding scope if:

- any required edit is outside the whitelist;
- preconditions differ;
- diagnostics cannot be produced without modifying navigation/SDFMap;
- ROS cannot be isolated;
- an unrelated failure blocks tests;
- the exact cause is proven.

## 15. Final self-audit and report

Before stopping:

```bash
git diff --check
git diff --cached --name-only
git status --short
git rev-parse HEAD
git stash list
```

Verify hashes/mtimes, exact whitelist, process cleanup, and user ROS untouched.

Final report must state:

1. exact files added/modified;
2. 49-field raw diagnostic schema and topic;
3. tests and aggregate counts;
4. bag/CSV/JSON/analyzer paths and SHA;
5. exact distributions of current reason and four probe statuses;
6. exact current sample/truncation/cross-section numeric facts;
7. one evidence-backed diagnosis from Section 12;
8. explicit statement that no behavior, parameter, map, control, or theory was
   changed;
9. diff/staged/HEAD/stash/process audit;
10. explicit stop statement.

A5 remains blocked after G2b-r1. A new dedicated specification is required for
any fix.
