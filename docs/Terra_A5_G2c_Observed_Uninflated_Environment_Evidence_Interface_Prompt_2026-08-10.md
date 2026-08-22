# A5-G2c Execution Specification: Observed Uninflated Environment Evidence Interface

Date: 2026-08-10

Execution model: one `gpt-5.6-terra` agent at `reasoning_effort=max`.

`AUTO_ADVANCE=false`.

This document authorizes only A5-G2c. Introduce the abstract, testable contract
for environment evidence required by the proposal-consistent phase-offset tube,
add an explicit fail-closed SDFMap capability boundary, and add a separate thin
integration translator. Build, test, self-audit, report, and stop.

This stage deliberately does not add an observed-free producer, an uninflated
distance-field backing, or runtime TubeBuilder selection. With the current
pillar obstacle-cloud producer, the new SDFMap boundary must report
`UNAVAILABLE`. Candidate ADD is not an acceptance criterion for this stage.

Do not enter a producer/coverage stage, runtime integration, A5-G3, A5.3, A6,
A7, or multi-UAV work. Do not delegate to another agent.

## 1. Established diagnosis

The tube construction order is already correct:

```text
planned ContinuousPhasePath
  -> lifted geometry p, T, N, signed curvature
  -> independent +N/-N environment clearance
  -> subtract r_eff exactly once
  -> candidate TubeProfile
  -> candidate/active TubeEpoch ownership
```

The G2b-r3 read-only audit proved the current pillar dataflow is:

```text
pillar.pcd
  -> local_sensing axis-aligned obstacle-point crop
  -> /sim/local_map
  -> SDFMap::cloudCallback
  -> occupancy_buffer_inflate_ and distance_buffer_all_
```

That path does not populate raw observed-free log odds. Therefore the current
raw bridge correctly returns UNKNOWN outside the G2b-r2 self-free UAV sphere.
The measured raw clearance was:

```text
c_plus  = 0.00-0.25 m
c_minus = 0.00-0.30 m
r_eff   = 0.55 m
```

The empty cross-section and zero Candidate ADD are correct fail-closed results.

The current ESDF is not an acceptable replacement:

- it is generated from `occupancy_buffer_inflate_`;
- raw UNKNOWN and raw known free are indistinguishable in that buffer;
- `getDistance` has no observation-validity result;
- `obstacles_inflation=0.099 m` at `resolution=0.1 m` becomes
  `ceil(0.099/0.1)=1`, a 3x3x3 cubical grid dilation;
- adding or subtracting scalar `0.099 m` cannot invert that dilation;
- full `r_eff` subtraction would repeat environment erosion.

The G2b-r3 decision was:

```text
production: observed status + uninflated environmental distance
simulation-only: explicitly isolated globally known uninflated static backend
```

A5-G2c establishes only the first contract and an honest unavailable boundary.

## 2. Mandatory references

Read completely before editing:

1. `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md`
2. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`
3. Proposal Sections 10.2-10.7, 13.5-13.6, 21, and 22.5 in context
4. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`
5. `docs/Terra_A5_G1_Asymmetric_Tube_Cross_Section_Kernel_Prompt_2026-08-09.md`
6. `docs/Terra_A5_G2a_Navigation_Asymmetric_Tube_Integration_Prompt_2026-08-09.md`
7. `docs/Terra_A5_G2b_Raw_Occupancy_ROS_And_Certified_Segment_Prompt_2026-08-09.md`
8. `docs/Terra_A5_G2b_r1_Raw_Candidate_Failure_Diagnostics_Prompt_2026-08-10.md`
9. `docs/Terra_A5_G2b_r2_Self_Free_Seed_Contract_Prompt_2026-08-10.md`
10. `docs/Terra_A5_G2b_r3_Map_Evidence_Contract_Read_Only_Audit_Prompt_2026-08-10.md`
11. `/tmp/a5g2b_r3_map_evidence_audit_JtObm5/map_evidence_trace.md`
12. `/tmp/a5g2b_r3_map_evidence_audit_JtObm5/architecture_recommendation.md`
13. `/tmp/a5g2b_r3_map_evidence_audit_JtObm5/next_stage_boundary.md`
14. Current complete implementations and tests for:
    - `phase_offset_navigation/distance_query.h`;
    - raw TubeCrossSectionSolver/TubeBuilder;
    - SDFMap initialization, buffers, query API, cloud/depth paths;
    - raw occupancy bridge and G2b-r2 self-free seed;
    - existing CMake/test conventions in navigation, plan_env, bspline_race.

Do not use the old uncompiled prototype as authoritative code.

## 3. Required precondition audit

Before editing, record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --cached --name-only
git stash list
sha256sum docs/Terra_A5_G2c_Observed_Uninflated_Environment_Evidence_Interface_Prompt_2026-08-10.md
```

Required:

- branch `main`;
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- prototype stash exists;
- no staged content;
- dirty/untracked worktree and completed A stages are user-owned.

Stop before editing if these differ. Never reset, restore, clean, stash-pop,
stage, commit, branch, tag, or push.

Capture pre/post SHA-256 and mtime manifests for every whitelist and protected
file. Because several relevant package directories are untracked, do not rely
on `git diff` alone to determine what changed.

## 4. Exact whitelist

Only these files may be added or modified.

### phase_offset_navigation

1. `src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt`
2. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/environment_evidence.h` (new)
3. `src/swarm_planner/phase_offset/phase_offset_navigation/src/environment_evidence.cpp` (new)
4. `src/swarm_planner/phase_offset/phase_offset_navigation/test/environment_evidence_test.cpp` (new)

### plan_env

5. `src/swarm_planner/plan_env/CMakeLists.txt`
6. `src/swarm_planner/plan_env/include/plan_env/sdf_map_environment_evidence.h` (new)
7. `src/swarm_planner/plan_env/include/plan_env/sdf_map.h`
8. `src/swarm_planner/plan_env/src/sdf_map_environment_evidence.cpp` (new)
9. `src/swarm_planner/plan_env/test/sdf_map_environment_evidence_test.cpp` (new)

### bspline_race integration

10. `src/swarm_planner/bspline_traj/CMakeLists.txt`
11. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_environment_evidence_query.h` (new)
12. `src/swarm_planner/bspline_traj/src/integration/phase_offset_environment_evidence_query.cpp` (new)
13. `src/swarm_planner/bspline_traj/test/phase_offset_environment_evidence_query_test.cpp` (new)

All other paths are protected, including:

- every proposal, architecture document, and this specification;
- `phase_offset_navigation` TubeCrossSectionSolver, TubeBuilder, TubeFilter,
  TubeEpochManager, Runtime, types, and all existing tests;
- `plan_env/src/sdf_map.cpp`, all existing SDFMap buffers/callbacks/timers,
  raycast, ESDF generation, manual/static logic, and existing tests;
- G2b raw query helper, G2b-r2 self-free seed, matched adapter, diagnostics,
  Marker helper, and their tests;
- gvf_manager, planner, optimizer, C2, GVF, governor, SO3;
- launch, RViz, map PCDs, parameters, messages, simulator, swarm, neighbor, CBF;
- package.xml files.

If a required edit is outside this whitelist, stop and report. Do not expand
scope.

## 5. Navigation-level contract

Create one ROS-free, SDFMap-free header representing the environment evidence
consumed by future tube logic. Reuse the existing
`phase_offset_navigation::DistanceStatus` values from `distance_query.h`; do
not create a second status enum with the same meanings.

Define equivalents of:

```cpp
enum EnvironmentEvidenceLayer : std::uint32_t {
  kEnvironmentLayerNone = 0U,
  kEnvironmentLayerSensorUninflated = 1U << 0,
  kEnvironmentLayerManualUninflated = 1U << 1,
  kEnvironmentLayerStaticUninflated = 1U << 2,
  kEnvironmentLayerGlobalStaticTest = 1U << 3,
};

struct EnvironmentEvidenceResult {
  DistanceStatus status;
  double uninflated_distance;
  bool distance_valid;
  std::uint32_t included_layer_mask;
  std::uint32_t unsupported_active_layer_mask;
};

using EnvironmentEvidenceQuery =
    std::function<EnvironmentEvidenceResult(const Eigen::Vector3d&)>;
```

Exact names may differ, but the information and semantics must not.

`uninflated_distance` means nonnegative Euclidean distance to the nearest
included, uninflated environmental occupied set. It is not planner clearance,
not distance to `occupancy_buffer_inflate_`, not distance after subtracting
UAV radius, and not a signed tracking margin.

The result must always contain finite numeric values. Use `0.0` when distance
is invalid; do not use NaN, infinity, or `10000` as a validity signal.

## 6. Exact consistency rules

Provide focused pure functions equivalent to:

```text
environmentEvidenceConsistent(result)
environmentEvidenceHasUsableDistance(result)
```

Required rules:

### UNAVAILABLE / OUT_OF_MAP / UNKNOWN

```text
distance_valid = false
uninflated_distance = 0.0
hasUsableDistance = false
```

`UNKNOWN` means the query location lacks positive observation evidence. It is
not an obstacle-distance measurement.

### OCCUPIED

```text
distance_valid = true
uninflated_distance = 0.0
hasUsableDistance = true
included_layer_mask != 0
unsupported_active_layer_mask = 0
```

### KNOWN_FREE

```text
distance_valid = true
uninflated_distance is finite and strictly > 0.0
hasUsableDistance = true
included_layer_mask != 0
unsupported_active_layer_mask = 0
```

For every status, any nonzero `unsupported_active_layer_mask` makes the result
unusable. A consistent result must not claim an included and unsupported copy
of the same layer simultaneously.

Reject:

- nonfinite or negative distances;
- `KNOWN_FREE` with zero distance;
- UNKNOWN/UNAVAILABLE/OUT_OF_MAP with a valid distance;
- any sentinel such as 10000 treated as proof of free space;
- distance provenance from an unspecified layer;
- included/unsupported layer-mask overlap.

Do not embed `r_eff` or tube formulas in this type. The consumer later applies
the robust erosion exactly once.

## 7. SDFMap-side contract and capability boundary

Create a plan_env-local result/capability type in
`sdf_map_environment_evidence.h`. It must not include or depend on
`phase_offset_navigation`.

It must contain equivalents of:

```text
status: UNAVAILABLE / OUT_OF_MAP / UNKNOWN / KNOWN_FREE / OCCUPIED
finite uninflated distance
distance-valid flag
included layer mask
unsupported active layer mask
capabilities:
  observation status available
  uninflated distance available
  supported layer mask
```

Add one narrow public SDFMap method declared in `sdf_map.h` and implemented in
the new focused `.cpp`, equivalent to:

```cpp
SDFMapEnvironmentEvidenceResult
queryObservedUninflatedEnvironment(const Eigen::Vector3d& point);
```

Optionally add a separate capabilities method if it keeps responsibilities
clear.

### Required current behavior

The current SDFMap has no backing representation satisfying both observed
status and uninflated environmental distance. Therefore:

```text
nonfinite or outside map -> OUT_OF_MAP, invalid distance
finite point inside map  -> UNAVAILABLE, invalid distance
```

Inside-map `UNAVAILABLE` is mandatory even if:

- raw occupancy says known free;
- raw occupancy says occupied;
- `distance_buffer_all_` is finite;
- `distance_buffer_all_` equals 10000;
- inflated occupancy is occupied or free;
- manual/static preinflated layers are active.

This stage must not claim partial availability by mixing current buffers. The
method must not call/read:

```text
getDistance
distance_buffer_all_
getInflateOccupancy
occupancy_buffer_inflate_
occupancy_buffer_
isUnknown
```

except that tests may deliberately poison those existing buffers to prove the
new method ignores them. Do not add new buffers, producer state, callbacks,
timers, map updates, or ROS topics.

The unsupported layer mask/capabilities must make clear that current sensor,
manual, and static representations are not yet proven uninflated evidence.
Do not mislabel manual or static-preinflated geometry as Contract D.

## 8. Thin integration translator

Create a separate integration helper/library that translates only the new
SDFMap result into the navigation `EnvironmentEvidenceResult`.

Provide equivalents of:

```cpp
phase_offset_navigation::EnvironmentEvidenceResult
translateSDFMapEnvironmentEvidence(
    const plan_env_result_type& result);

phase_offset_navigation::EnvironmentEvidenceQuery
makeEnvironmentEvidenceQuery(SDFMap* map);
```

Required behavior:

- null SDFMap -> UNAVAILABLE with finite invalid-distance result;
- exact one-to-one status translation;
- exact distance-valid and layer-mask translation;
- inconsistent SDFMap results fail closed to UNAVAILABLE;
- all outputs satisfy navigation consistency rules;
- no call to legacy ESDF, inflated occupancy, raw bridge, TubeBuilder, Runtime,
  Marker, planner, or control.

Build it as its own focused library/target. Do not add its header to
`phase_offset_matched_adapter.*`, `gvf_manager.*`, or any production runtime
source. No runtime object may construct or call this query in G2c.

## 9. Required navigation tests

Add deterministic tests covering at least:

1. UNAVAILABLE consistent only with invalid zero distance.
2. OUT_OF_MAP consistent only with invalid zero distance.
3. UNKNOWN consistent only with invalid zero distance.
4. OCCUPIED consistent with valid zero distance and explicit layer.
5. KNOWN_FREE consistent with valid finite positive distance and explicit
   layer.
6. NaN, infinity, negative distance, and 10000 sentinel misuse are rejected.
7. Included/unsupported mask overlap is rejected.
8. Any unsupported active layer makes the result unusable.
9. All supported layer bits can be represented without introducing ROS/SDFMap.
10. A consumer-side demonstration starts from an uninflated distance and
    subtracts a sample `r_eff` exactly once; no planner-inflation compensation
    appears in the contract.
11. Query type accepts deterministic synthetic results for all statuses.

Do not modify TubeCrossSectionSolver or TubeBuilder to perform this
demonstration.

## 10. Required SDFMap tests

Add a focused plan_env test proving:

1. Nonfinite and outside-map points return OUT_OF_MAP.
2. A finite inside-map point returns UNAVAILABLE.
3. Raw known-free plus finite existing ESDF still returns UNAVAILABLE.
4. Raw occupied plus zero existing ESDF still returns UNAVAILABLE.
5. Inflated occupied/free state cannot change the result.
6. A 10000 distance sentinel cannot become KNOWN_FREE.
7. Manual occupancy cannot be mislabeled uninflated evidence.
8. Static-preinflated occupancy cannot be mislabeled uninflated evidence.
9. All fields are finite and the advertised capabilities are false/unsupported
   as required.
10. Existing manual-map tests remain unchanged and pass.

Use test fixture initialization only. Do not call ROS callbacks, write map
files except existing `/tmp` test conventions, or change SDFMap runtime data.

## 11. Required integration tests

Add focused tests covering:

1. Exact translation of every valid plan_env status/result.
2. Null SDFMap returns consistent UNAVAILABLE.
3. Current SDFMap inside-map query translates to UNAVAILABLE.
4. OUT_OF_MAP translation is exact.
5. Inconsistent plan_env result fails closed.
6. Included and unsupported layer masks remain exact when valid.
7. Every translated numeric field is finite.
8. No query invokes legacy ESDF/inflated/raw bridge functions.
9. No production runtime file includes or references the new integration
   helper.

## 12. CMake and dependency boundaries

Make only the build changes required to compile:

- the navigation contract library source/test;
- the plan_env SDFMap evidence source/test;
- the standalone bspline_race integration bridge library/test.

Required dependency direction:

```text
Eigen/STL
  -> phase_offset_navigation contract

plan_env SDFMap-local contract (independent of phase_offset packages)

phase_offset_navigation + plan_env
  -> thin bspline_race environment-evidence translator
```

Forbidden dependencies:

- navigation -> ROS, plan_env, SDFMap, messages, planner;
- plan_env -> phase_offset_navigation/core;
- new integration helper -> gvf_manager, Runtime ownership, Marker, planner,
  C2, control, swarm, CBF;
- any circular package dependency.

Keep new implementation files focused and preferably under 250 lines each.

## 13. Build and regression requirements

Run:

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_phase_offset_core
catkin_make run_tests_plan_env
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

Directly run and report:

```text
phase_offset_environment_evidence_test
sdf_map_environment_evidence_test
phase_offset_environment_evidence_query_test
manual_map_layer_test
phase_offset_raw_occupancy_query_test
phase_offset_matched_adapter_test
phase_offset_tube_cross_section_test
phase_offset_tube_builder_test
phase_offset_tube_epoch_manager_test
phase_offset_runtime_test
```

Do not fix unrelated failures.

## 14. No ROS acceptance in G2c

Do not start ROS. The new query is deliberately not connected to runtime, and
current SDFMap must report UNAVAILABLE. A ROS run cannot provide additional
acceptance evidence for an unused interface.

Prove statically that:

- no production runtime source includes the new integration header;
- matched adapter remains on the current G2 raw bridge;
- Candidate/Certified Marker code is unchanged;
- no publisher/topic/parameter/launch file was added;
- current user ROS processes were untouched.

## 15. Protected behavior and explicit non-goals

Do not:

- implement an uninflated distance buffer or observed-free mask;
- modify `cloudCallback`, depth/raycast, ESDF, inflation, refresh, manual/static
  map update, or visualization;
- populate free space from obstacle-point absence;
- use the global PCD as production evidence;
- connect the new interface to TubeBuilder, EpochManager, Runtime, adapter, or
  gvf_manager;
- change the G2 raw bridge or r2 self-free seed;
- change `r_eff`, margins, search extent, ray step, tube formulas, filter,
  epoch logic, gate, control, planner, speed, map, or target;
- change diagnostics schemas or Marker behavior;
- add producer/coverage configuration;
- add swarm, neighbor, CBF, emergency, snapshot, or continuation work;
- commit, stage, branch, tag, or push.

## 16. Mandatory stop conditions

Stop without expanding scope if:

- any required edit lies outside the whitelist;
- preconditions or protected hashes differ unexpectedly;
- navigation would need a ROS/SDFMap dependency;
- plan_env would need a phase_offset dependency;
- the interface cannot remain unused by runtime;
- tests require creating an actual producer/backing;
- a current SDFMap buffer would need to be presented as uninflated observed
  evidence;
- an unrelated historical failure blocks aggregate reporting.

Preserve all changes/evidence and report the blocker. Do not advance to the
producer stage.

## 17. Final dependency and self-audit

Required searches must prove:

- navigation contract contains no ROS, SDFMap, plan_env, Marker, planner,
  messages, swarm, or CBF dependency;
- plan_env evidence files contain no phase_offset dependency;
- integration bridge contains no getDistance, distance_buffer_all_,
  getInflateOccupancy, occupancy_buffer_inflate_, raw bridge, TubeBuilder,
  Runtime, Marker, planner, C2, control, swarm, or CBF calls;
- new integration header is referenced only by its implementation/test/build
  declarations;
- protected runtime/mapper source hashes are unchanged.

Before stopping:

```bash
git diff --check
git diff --cached --name-only
git status --short
git branch --show-current
git rev-parse HEAD
git stash list
```

Verify exact whitelist, pre/post hashes/mtimes, no staged content, no ROS
process created, and no user process touched.

## 18. Required final report

Report:

1. exact files added/modified;
2. navigation result/query/layer-mask contract;
3. exact consistency and usable-distance rules;
4. SDFMap capability/query behavior and proof current inside-map result is
   UNAVAILABLE;
5. proof no existing buffer is reused or mislabeled;
6. integration translation behavior and proof it is not runtime-connected;
7. all direct and aggregate test counts;
8. dependency and forbidden-symbol searches;
9. protected hash/mtime, diff, staged, HEAD, stash, and process audit;
10. explicit statement that no ROS run occurred;
11. explicit statement that no producer/backing, Runtime selection, tube
    behavior, parameter, or control change occurred;
12. explicit statement that Candidate ADD remains outside this stage;
13. explicit stop statement.

A5 remains blocked after A5-G2c. A separate dedicated producer/backing
execution specification and explicit authorization are required before the
new interface can return usable production evidence.
