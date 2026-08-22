# A5-G2a Execution Specification: Pure Navigation Integration of the Asymmetric Tube Kernel

Date: 2026-08-09

Execution model: one `gpt-5.6-terra` agent at `reasoning_effort=max`.

`AUTO_ADVANCE=false`.

This document authorizes only A5-G2a. Complete the implementation, build,
tests, compatibility regression, dependency audit, and self-audit described
here, then stop. Do not start A5-G2b, A5-G3, A5.3, A6, A7, or any multi-UAV
work.

## 1. Context and stage boundary

A5-G1 added and proved the isolated pure-C++ `TubeCrossSectionSolver`:

```text
raw occupancy along +N/-N
  -> c_plus_raw / c_minus_raw
  -> obstacle interval after exactly one r_eff erosion
  -> signed curvature intersection
  -> valid asymmetric cross-section
```

G1 verified:

```text
open:                  [-2.70,+2.70]
single-sided obstacle: [-2.20,+0.30]
narrow nonempty:       [-0.05,+0.05]
genuinely empty:       [+0.05,-0.05]
legal one-sided:       [-2.00,-0.20]
```

The G1 kernel is not connected to the current `TubeBuilder`, `TubeFilter`,
`TubeEpochManager`, ROS adapter, SDFMap, Marker, or Runtime.

A5-G2a integrates the G1 kernel only inside `phase_offset_navigation` and
proves the navigation semantics with synthetic raw occupancy queries. It does
not change the current ROS adapter or launch path. A temporary compatibility
boundary is required:

- existing callers that provide only the old `DistanceQuery` continue through
  the legacy builder path during G2a;
- new navigation tests provide `RawOccupancyQuery` and exercise the new
  proposal-consistent path;
- A5-G2b will later replace the ROS adapter input with an explicit raw
  occupancy bridge and remove or isolate the legacy ESDF construction path.

Do not claim the displayed ROS tube is fixed after G2a.

## 2. Mandatory references

Read completely before editing:

1. `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md`
2. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`
3. Proposal Sections 9, 10, 10.2-10.6, 13.5, 13.6, and 17 in context
4. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`
5. `docs/Terra_A5_G1_Asymmetric_Tube_Cross_Section_Kernel_Prompt_2026-08-09.md`
6. `docs/DeepSeek_A5_R1_Pure_CPP_Tube_Epoch_Manager_Prompt_2026-08-09.md`
7. `docs/DeepSeek_A5_R2_Multirate_Runtime_ROS_Integration_Prompt_2026-08-09.md`
8. Current complete implementations and tests of:
   - `tube_cross_section.h/.cpp`
   - `tube_types.h`
   - `tube_builder.h/.cpp`
   - `tube_filter.h/.cpp`
   - `tube_epoch_types.h`
   - `tube_epoch_manager.h/.cpp`

Do not use the uncompiled legacy `bspline_race/path_tube_builder.*` as an
implementation source.

## 3. Required precondition audit

Before editing, record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --cached --name-only
git stash list
sha256sum docs/Terra_A5_G2a_Navigation_Asymmetric_Tube_Integration_Prompt_2026-08-09.md
```

Required state:

- branch `main`;
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` exists;
- no staged changes;
- existing dirty worktree, A5-R2 changes, A5-G1 files, and all unrelated
  untracked files are user-owned and must be preserved.

Stop before editing if branch, HEAD, stash, or staged-state conditions fail.
Never reset, restore, clean, pop, or overwrite the dirty worktree.

Because the phase-offset packages are untracked as a directory, record
SHA-256 values for every protected existing file named in Section 5 before and
after the stage. Also record mtimes of all navigation files before editing.

## 4. Exact whitelist

Only these paths may be modified:

1. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h`
2. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h`
3. `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp`
4. `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp`
5. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h`
6. `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp`
7. `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp`
8. `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp`
9. `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp`
10. `src/swarm_planner/phase_offset/phase_offset_navigation/README.md`

All other paths are protected. In particular, do not modify:

- `phase_offset_navigation/CMakeLists.txt`;
- `distance_query.h`;
- `tube_cross_section.h/.cpp/test` from G1;
- `tube_filter.h`;
- `tube_epoch_manager.h`;
- `phase_offset_runtime.h/.cpp/test`;
- any `phase_offset_core` file;
- any `bspline_race` adapter, diagnostics, Marker, launch, RViz, manager,
  planner, C2, GVF, or test file;
- any `plan_env/SDFMap` file;
- any simulator, message, swarm, CBF, or prototype file;
- this execution specification.

If a required edit is outside the whitelist, stop and report. Do not expand
scope.

## 5. Compatibility rule for G2a

The current ROS adapter constructs only the old `DistanceQuery`. It is outside
this stage's whitelist.

Therefore G2a must preserve source and behavior compatibility for existing
legacy callers:

- the existing `TubeBuilder::build(... DistanceQuery ...)` entry point remains
  available and retains its current behavior;
- a new explicit raw-occupancy entry point is added for the new semantics;
- `TubeEpochUpdateInput` gains an optional `RawOccupancyQuery`;
- `TubeEpochManager` selects the raw path only when the raw query is present;
- if the raw query is absent, the existing legacy path remains active;
- Fixed-source behavior remains unchanged and never queries either map source.

Do not add a ROS parameter or feature flag. Query presence is the temporary
C++ compatibility boundary. The final report must state clearly that this
dual path is temporary and G2b must connect the adapter explicitly.

## 6. TubeBuilderConfig and margin contract

Add a focused cross-section configuration to `TubeBuilderConfig`, reusing the
G1 public type rather than duplicating formulas. An acceptable shape is:

```cpp
TubeCrossSectionConfig cross_section;
```

The new raw path uses only:

```text
cross_section.search_extent
cross_section.ray_step
cross_section.boundary_tolerance
cross_section.regularity_margin
cross_section.curvature_epsilon
cross_section.margins = r_uav + e_map + e_loc + e_track
```

The old fields remain only for legacy compatibility in G2a:

```text
max_offset
ray_step
ErosionMargins
```

Do not silently copy the old six-field `0.70 m` erosion into the new four-field
contract. Tests must configure the new contract explicitly.

The new path must not contain a controller offset clamp. Environmental search
extent is not controller `delta_max`.

## 7. TubeRawSample facts

Extend `TubeRawSample` only with focused fields required to preserve the G1
cross-section result for each path sample. Include equivalents of:

```text
c_plus_raw
c_minus_raw
effective_radius
obstacle_lower
obstacle_upper
curvature_lower
curvature_upper
environment_lower
environment_upper
environment_width
environment_interval_nonempty
environment_contains_zero
environment_contains_preferred_delta
cross_section_reason
positive_ray_termination
negative_ray_termination
```

Exact names may be improved. Keep existing legacy fields and indices intact;
do not change any ROS diagnostics schema in G2a.

For a raw-path sample:

- `raw_lower/raw_upper` become the final environment bounds from the G1 solver;
- `filtered_lower/filtered_upper` start from those environment bounds;
- `sample.complete` means the environment interval is geometrically nonempty
  and all required finite geometry/query facts exist;
- `environment_contains_zero=false` does not make the sample incomplete;
- `environment_contains_preferred_delta=false` does not make the sample
  incomplete;
- positive/negative certification must reflect conservative ray termination,
  including valid truncation at the last known-free point before UNKNOWN,
  OUT_OF_MAP, or UNAVAILABLE.

Do not reinterpret `fixed_lower/fixed_upper` as environment bounds for the raw
path. They may remain populated for legacy compatibility, but cannot clamp the
new environment interval.

## 8. New TubeBuilder raw entry point

Add an explicit overload or distinctly named method that receives:

```text
TubeSource
preview path
RawOccupancyQuery
preferred_delta
source revision
tube revision
TubeProfile output
```

Requirements:

1. Fixed source continues through the existing fixed-tube logic.
2. ESDF/raw source requires a valid `RawOccupancyQuery`.
3. Evaluate path geometry once per preview sample and pass `p`, normalized
   horizontal `N`, and signed curvature into `TubeCrossSectionSolver`.
4. Do not query the old `DistanceQuery` in the raw path.
5. Do not perform a separate centerline-clearance gate.
6. Do not intersect with legacy `max_offset`.
7. Preserve valid intervals that do not contain zero.
8. Preserve valid intervals that do not contain `preferred_delta`.
9. The profile can remain all-samples-complete in G2a; partial certified-prefix
   profile semantics are deferred to G2b. However UNKNOWN or map-boundary
   encountered on a side after a certified free prefix must use the G1
   conservative truncated bound and must not automatically make that sample
   incomplete.
10. Current-center UNKNOWN/OUT_OF_MAP/UNAVAILABLE/OCCUPIED remains a precise
    sample failure.
11. Profile obstacle certification for the raw path means every included
    sample has a valid conservatively certified environment cross-section.

Do not duplicate the G1 formulas in TubeBuilder. It must call the solver and
copy its result.

## 9. TubeFilter correction

Remove the global envelope fallback from the active filter implementation:

```cpp
global_lower = max(all raw_lower)
global_upper = min(all raw_upper)
```

Requirements:

- retain the existing conservative local slope-limiting and dense subset
  certification;
- local contractions may move lower inward and upper inward;
- filtered bounds must remain a subset of raw bounds at every dense check;
- a local narrow point must not make every distant sample equal to the global
  minimum interval;
- if the current local algorithm cannot produce a certified C1 segment after
  its bounded local repair attempts, return filter failure instead of replacing
  the entire profile with one global interval;
- do not add partial-prefix filtering or segment splitting in G2a;
- do not loosen the slope or dense-sampling limits to make tests pass.

Add a deterministic regression that would have used the old global fallback
and assert it now fails instead of flattening the full profile. Add another
test showing one local narrow point leaves sufficiently distant open samples
strictly wider than the narrow point when local filtering succeeds.

## 10. TubeEpochUpdateInput raw path

Add an optional:

```cpp
RawOccupancyQuery raw_occupancy_query;
```

The raw query is for candidate geometry only. The legacy `DistanceQuery`
remains in the input solely to preserve old callers during G2a.

Selection:

```text
ESDF + raw query present -> new raw TubeBuilder path
ESDF + raw query absent  -> current legacy DistanceQuery path
FIXED                    -> fixed path; query functions unused
NONE                     -> existing source-none semantics
```

Expose in `TubeEpochStatus` a boolean equivalent of:

```text
raw_cross_section_path_used
```

This field is pure C++ status only. Do not append it to the 49- or 83-field ROS
diagnostics in G2a.

## 11. Raw-path current-state semantics

For the raw path, candidate geometry already erodes the environment by:

```text
r_uav + e_map + e_loc + e_track
```

Therefore do not run the old inflated-ESDF `CheckDistance(...0.70/0.55...)`
again for raw-path current-state validation.

For the raw path:

```text
reference_safe = current bounds valid && retained_delta inside current bounds
actual_safe = reference_safe && tracking_error_norm <= e_track
current_state_admissible = geometry valid && reference_safe && actual_safe
```

Keep the legacy point-distance checks unchanged for the legacy path.

Required status facts for the raw path:

- `required_reference_clearance = cross_section.margins.effectiveRadius()`;
- `required_actual_clearance` may be the same documented robust radius or the
  radius without `tracking_error_bound`, but the choice must be explicit and
  tested; do not reuse the old six-field erosion accidentally;
- legacy signed-distance diagnostic fields remain finite but must not claim a
  raw Euclidean ESDF measurement that was not made;
- `reference_clearance_sufficient` mirrors current interval containment;
- `actual_clearance_sufficient` mirrors interval containment plus tracking;
- `current_safety_status` is SAFE/UNSAFE/INDETERMINATE from these facts.

Do not add a new distance-field implementation.

## 12. One-sided candidate and installation semantics

The key distinction is:

```text
candidate geometric validity
!= candidate contains zero
!= candidate contains retained delta
!= candidate is installable now
```

For the current raw candidate sample, publish status equivalents of:

```text
current_interval_nonempty
current_interval_contains_zero
current_interval_contains_retained_delta
```

Required behavior:

### Candidate contains retained delta

- It may be installed if geometry, tracking, and existing forward-horizon
  conditions pass.
- If it does not contain zero but contains retained delta, install is allowed,
  but final state must be `REPLAN_REQUIRED`, not normal `ROLLING`.
- Add a precise epoch reason such as `CENTERLINE_OUTSIDE_CANDIDATE_TUBE`.

### Candidate does not contain retained delta

- Candidate remains geometrically complete and is returned in
  `candidate_profile`.
- It must not overwrite an existing active profile.
- It must not be installed as a `SAFETY_REPLACEMENT`.
- State is `SAFETY_PRIORITY` with reason `CURRENT_OFFSET_OUTSIDE`.
- If no active profile existed, active remains unavailable.
- Retained delta is never clipped or reset.
- `update()` may return true to indicate a valid candidate/status result, but
  documentation/tests must not interpret true as successful normal install.

This corrects the current behavior where explicit unsafe status can call
`installActive(candidate, ..., safety_replacement=true)` even when the retained
delta lies outside the candidate.

Do not command motion toward the one-sided interval in G2a. Do not change
Runtime, PortProjector, `w_dot`, `u_delta`, or replanning callbacks.

## 13. Forward horizon and equivalence

Retain existing R1 behavior:

- forward containment checks whether the unchanged retained delta remains in
  the profile over the minimum horizon;
- `dynamic_feasibility_evaluated` remains false;
- no `U_i^+`, reachability QP, or port dynamics are added;
- equivalent candidate refresh does not increment active epoch;
- rejected/noninstallable candidate does not partially overwrite active
  profile or provenance.

Update profile equivalence and finite checks to include every new environment
cross-section field and enum. A material change in an environmental boundary
must not be classified equivalent.

## 14. Required TubeBuilder tests

Keep every existing legacy TubeBuilder test passing. Add raw-path tests for:

1. Open 3 m environment with `r_eff=0.30` produces
   `[-2.70,+2.70]`, not legacy `max_offset`.
2. `c_plus=0.60`, `c_minus=2.50`, `r_eff=0.30` produces
   `[-2.20,+0.30]`.
3. `0.35/0.35/0.30` is valid `[-0.05,+0.05]`.
4. `0.25/0.25/0.30` is genuinely empty with the precise reason.
5. `[-2.00,-0.20]` is complete although it does not contain zero.
6. The same interval remains complete when it does not contain preferred delta.
7. Positive/negative curvature only contracts the correct side.
8. UNKNOWN/OUT_OF_MAP after a free prefix truncates only that side and remains
   valid when the resulting interval is nonempty.
9. Center UNKNOWN/OUT_OF_MAP/UNAVAILABLE/OCCUPIED fails precisely.
10. Old `DistanceQuery` entry point still produces its existing deterministic
    results, proving compatibility.

Assert the copied per-sample diagnostic fields equal the G1 solver result.

## 15. Required TubeFilter tests

Keep existing tests and add:

1. A local narrow point does not flatten distant open samples to the same
   interval width.
2. A constructed profile that cannot pass local dense C1 certification now
   returns false and never applies a global envelope.
3. Asymmetric one-sided intervals remain asymmetric through filtering.
4. Dense queries remain inside the raw interval and slope bound.

Do not weaken existing numeric assertions.

## 16. Required TubeEpochManager tests

Keep legacy tests passing unless a test asserts the explicitly corrected
unsafe-install behavior. Add raw-path tests for:

1. Raw query presence selects the new path and sets status accordingly.
2. Open raw candidate installs and rolls when retained delta/horizon are valid.
3. Valid one-sided candidate containing retained delta installs but returns
   `REPLAN_REQUIRED` with the centerline-outside reason.
4. Valid one-sided candidate excluding retained delta remains visible as
   candidate, does not install, and returns `SAFETY_PRIORITY`.
5. The same exclusion preserves an older active profile and epoch exactly.
6. Unknown on one ray after a free prefix does not make the candidate
   incomplete if the interval remains nonempty.
7. Current-center UNKNOWN makes candidate incomplete without overwriting active.
8. Raw path does not call legacy `DistanceQuery` for candidate or current-state
   validation.
9. Tracking beyond the G1 `e_track` bound enters `SAFETY_PRIORITY` without
   installing the unsafe candidate.
10. Material environmental-bound change increments epoch only on install;
    equivalent refresh does not.
11. Source NONE and FIXED semantics remain unchanged.
12. `dynamic_feasibility_evaluated` remains false.

Update any existing test that currently expects an outside-retained-delta
candidate to be installed as a safety replacement. The corrected expected
behavior is preservation of the old active profile.

## 17. Build and regression requirements

Required commands:

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_phase_offset_core
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

Also run directly and report exact test counts for:

```text
phase_offset_tube_cross_section_test
phase_offset_tube_builder_test
phase_offset_tube_filter_test
phase_offset_tube_epoch_manager_test
```

The existing bspline tests must pass with the legacy compatibility path,
proving G2a did not silently switch ROS integration.

Do not run ROS, roslaunch, rosbag, RViz, or simulation.

## 18. Dependency and boundary audit

Modified navigation files must contain no new dependencies or identifiers for:

```text
ros
SDFMap
plan_env
visualization_msgs
Marker
gvf_manager
ContinuousPhasePath
PositionCommand
planner
C2
swarm
neighbor
CBF
```

`TubeEpochManager` and `PhaseOffsetRuntime` names already exist within their
proper navigation files and are not dependency violations by themselves.

No formulas may be added to ROS callbacks because no ROS file is authorized.

## 19. Forbidden changes

Do not:

- edit the G1 solver to make integration easier;
- change the adapter or SDFMap;
- change launch parameters, current `max_offset`, map inflation, planner
  weights, speeds, accelerations, saturation, GVF, governor, or SO3;
- append/change the 83- or 49-field diagnostics schemas;
- add Marker behavior;
- implement partial-profile/certified-prefix ROS behavior;
- add a new epoch state such as EMERGENCY;
- implement `U_i^+`, dynamic reachability, QP, or new replan callbacks;
- change Runtime, matched port, phase, delta update, or C2 continuation;
- remove the legacy builder entry point before G2b;
- restore prototype files;
- commit, stage, tag, branch, or push.

## 20. Stop conditions

Stop and report without expanding scope if:

- any required edit is outside the whitelist;
- branch, HEAD, stash, staged state, or protected hashes differ;
- the G1 solver cannot be integrated without modifying it;
- legacy bspline tests cannot compile without adapter changes;
- current raw-path safety cannot be expressed without adding planner/control
  behavior;
- an unrelated historical failure blocks workspace-wide reporting.

Preserve the worktree and report the blocker. Do not fix unrelated problems.

## 21. Final self-audit and report

Before stopping:

```bash
git diff --check
git diff --cached --name-only
git status --short
git rev-parse HEAD
git stash list
```

Because the package is untracked, verify whitelist compliance with pre/post
SHA-256 and file mtimes, not only `git diff`.

The final report must state:

1. exact modified files;
2. how legacy and raw paths are selected;
3. how old six-field erosion and new four-field `r_eff` remain separated;
4. TubeBuilder outputs for open, asymmetric, narrow, empty, and one-sided cases;
5. proof that one-sided candidate validity is independent of zero/current
   containment;
6. proof that an excluding candidate cannot overwrite active;
7. proof that centerline-outside but current-delta-inside requests replan;
8. proof the global filter fallback no longer exists;
9. direct and aggregate test counts;
10. dependency search, protected hashes, `git diff --check`, staged state, HEAD,
    and stash results;
11. an explicit statement that ROS still uses the legacy path;
12. an explicit stop statement.

A5 remains blocked after A5-G2a. A new A5-G2b execution specification is
required to connect raw SDFMap occupancy, candidate/active visualization, and
certified-prefix preview semantics.
