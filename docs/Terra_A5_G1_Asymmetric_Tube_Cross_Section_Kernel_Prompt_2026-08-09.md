# A5-G1 Execution Specification: Asymmetric Robust Tube Cross-Section Kernel

Date: 2026-08-09

Execution model: one `gpt-5.6-terra` agent at `reasoning_effort=max`.

`AUTO_ADVANCE=false`.

This document authorizes only A5-G1. Complete the implementation, build,
tests, dependency audit, and self-audit described here, then stop. Do not start
A5-G2, A5-G3, A5.3, A6, A7, or any multi-UAV work.

## 1. Why this stage exists

The current A5 integration has a useful candidate/active epoch architecture,
but the active ESDF TubeBuilder does not yet implement the proposal's actual
asymmetric robust cross-section contract:

- `max_offset=0.20 m` is currently used as an environmental tube boundary;
- the queried ESDF is already based on inflated occupancy, while the builder
  applies another full clearance erosion;
- the builder requires the centerline `delta=0` to be safe before tracing;
- therefore a legal one-sided interval such as `[-2.0,-0.2]` cannot exist;
- unknown or out-of-map preview samples can erase the complete profile;
- later filtering has a global-envelope fallback that can propagate a local
  narrow point over the full preview.

A5-G1 deliberately does not patch ROS, Runtime, TubeEpochManager, SDFMap, the
current TubeBuilder, or visualization. It first establishes and proves the
small pure-C++ mathematical kernel that later stages will integrate.

The intended eventual dataflow remains:

```text
accepted ContinuousPhasePath
  -> lifted p(w), T(w), N(w), kappa(w)
  -> raw local-map occupancy query along +N and -N
  -> asymmetric robust obstacle interval
  -> signed curvature intersection
  -> local conservative C1 filtering
  -> candidate TubeProfile
  -> TubeEpochManager hybrid installation
  -> immutable active TubeProfile
  -> 50 Hz PhaseOffsetRuntime
```

Only the isolated cross-section kernel in the middle is authorized now.

## 2. Mandatory references

Read completely before editing:

1. `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md`
2. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`
3. Proposal Sections 9, 10, 10.2-10.6, 13.5, 13.6, and 17 in context
4. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`
5. `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`
6. `docs/DeepSeek_A5_R1_Pure_CPP_Tube_Epoch_Manager_Prompt_2026-08-09.md`
7. `docs/DeepSeek_A5_R2_Multirate_Runtime_ROS_Integration_Prompt_2026-08-09.md`
8. The current implementations of:
   - `phase_offset_navigation/distance_query.h`
   - `phase_offset_navigation/tube_types.h`
   - `phase_offset_navigation/tube_builder.h/.cpp`
   - `phase_offset_navigation/tube_filter.h/.cpp`
   - their existing tests

Do not use the uncompiled legacy `bspline_race/path_tube_builder.*` as the
implementation base. It is reference material only.

## 3. Required precondition audit

Before editing, record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --cached --name-only
git stash list
sha256sum docs/Terra_A5_G1_Asymmetric_Tube_Cross_Section_Kernel_Prompt_2026-08-09.md
```

Required state:

- branch `main`;
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` exists;
- no staged changes;
- the worktree is dirty and all pre-existing changes are user-owned;
- A5-R2 changes may already exist in the dirty worktree and must be preserved.

Stop without editing if branch, HEAD, stash, or staged-state requirements do
not hold. Never reset, restore, clean, pop, or overwrite the dirty worktree.

Capture SHA-256 values for all protected existing files named in Section 5
before and after the stage.

## 4. Stage objective

Add one focused pure-C++14/Eigen component that computes a single path sample's
proposal-consistent asymmetric robust tube cross-section from a raw occupancy
query.

For base point `p`, unit horizontal normal `N`, and search distances measured
from the centerline:

```text
c_plus  = certified raw free distance along p + lambda N
c_minus = certified raw free distance along p - lambda N

r_eff = r_uav + e_map + e_loc + e_track

delta_upper_obs =  c_plus  - r_eff
delta_lower_obs = -c_minus + r_eff
```

Curvature regularity is:

```text
1 - kappa * delta >= mu > 0
```

therefore:

```text
kappa > 0: delta_upper_curv = (1-mu)/kappa
kappa < 0: delta_lower_curv = (1-mu)/kappa
|kappa| near zero: no material curvature contraction
```

The final interval is the obstacle interval intersected with the signed
curvature interval.

The only cross-section emptiness test is:

```text
delta_lower_final > delta_upper_final
```

`contains_zero` and `contains_preferred_delta` are separate output facts. They
must never be folded into geometric validity.

## 5. Exact whitelist

Only these paths may be added or modified by the execution agent:

1. `src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt`
2. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h` (new)
3. `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp` (new)
4. `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp` (new)
5. `src/swarm_planner/phase_offset/phase_offset_navigation/README.md`

All other files are protected, including but not limited to:

- `distance_query.h`;
- `tube_types.h`;
- `tube_builder.h/.cpp`;
- `tube_filter.h/.cpp`;
- `tube_epoch_types.h`;
- `tube_epoch_manager.h/.cpp`;
- `phase_offset_runtime.h/.cpp`;
- all `phase_offset_core` files;
- all `bspline_race` integration, adapter, Marker, launch, RViz, diagnostics,
  manager, planner, C2, and GVF files;
- all `plan_env/SDFMap` files;
- all simulator, message, swarm, and CBF files;
- all existing tests outside the one new test file;
- this execution specification.

If implementation genuinely requires an edit outside this whitelist, stop and
report the exact need. Do not expand scope.

## 6. Required public concept

Create one header representing one concept: an asymmetric robust tube
cross-section solver. Exact type names may be improved, but the public contract
must remain small and explicit.

The header should provide equivalents of:

```cpp
using RawOccupancyQuery =
    std::function<DistanceStatus(const Eigen::Vector3d&)>;

struct RobustTubeMargins {
  double uav_radius;
  double map_uncertainty;
  double localization_uncertainty;
  double tracking_error_bound;

  double effectiveRadius() const;
};

struct TubeCrossSectionConfig {
  double search_extent;
  double ray_step;
  double boundary_tolerance;
  double regularity_margin;
  double curvature_epsilon;
  RobustTubeMargins margins;
};

enum class TubeCrossSectionReason {
  NONE,
  INVALID_CONFIGURATION,
  INVALID_GEOMETRY,
  CENTER_UNAVAILABLE,
  CENTER_OUT_OF_MAP,
  CENTER_UNKNOWN,
  CENTER_OCCUPIED,
  EMPTY_AFTER_OBSTACLE_BOUNDS,
  CURVATURE_NUMERICAL_FAILURE,
  EMPTY_AFTER_CURVATURE_INTERSECTION,
};

enum class TubeRayTermination {
  SEARCH_EXTENT,
  OCCUPIED,
  UNKNOWN,
  OUT_OF_MAP,
  UNAVAILABLE,
};

struct TubeCrossSectionInput {
  Eigen::Vector3d p;
  Eigen::Vector3d N;
  double curvature;
  double preferred_delta;
  RawOccupancyQuery occupancy_query;
};

struct TubeCrossSectionResult {
  double c_plus_raw;
  double c_minus_raw;
  double effective_radius;
  double lower_obstacle;
  double upper_obstacle;
  double lower_curvature;
  double upper_curvature;
  double lower_final;
  double upper_final;
  double width;
  bool contains_zero;
  bool contains_preferred_delta;
  bool valid;
  TubeRayTermination positive_termination;
  TubeRayTermination negative_termination;
  TubeCrossSectionReason reason;
};
```

Do not create a mixed global types header. Do not add ROS types, ROS time,
SDFMap, planner, manager, path version, tube epoch, Marker, or control policy to
this component.

## 7. Raw occupancy semantics

The query contract is deliberately raw occupancy, not the current inflated
ESDF contract.

`DistanceStatus` meanings for this component:

- `KNOWN_FREE`: raw cell is observed and free;
- `OCCUPIED`: raw cell is occupied;
- `UNKNOWN`: cell has not been observed;
- `OUT_OF_MAP`: outside map storage;
- `UNAVAILABLE`: query source cannot answer.

The component must not call or know SDFMap. A later A5-G2 adapter will translate
the existing `SDFMap::isInMap`, `isUnknown`, and `getOccupancy` calls into this
abstract query.

Do not add any map-inflation parameter to this kernel. It consumes raw
occupancy, so the robust erosion is exactly `r_eff` once. Planner inflation is
outside this contract.

## 8. Ray tracing requirements

For each side independently:

1. Require finite `p`, finite nondegenerate `N`, and a valid query.
2. Normalize `N` before interpreting lambda as metres.
3. Query the center cell.
4. A raw-free center is allowed even when an obstacle is less than `r_eff` away.
   This is essential for a valid one-sided interval.
5. A raw-occupied center is a geometric failure. Do not search through an
   occupied center for a disconnected free component.
6. March independently along `+N` and `-N` up to `search_extent`.
7. `ray_step` must be finite, positive, and no larger than `search_extent`.
8. When an occupied transition is bracketed by a known-free point, refine the
   transition by deterministic bisection until `boundary_tolerance` is met.
9. When `UNKNOWN`, `OUT_OF_MAP`, or `UNAVAILABLE` is encountered away from the
   center, terminate that side conservatively at the last certified known-free
   distance. This must not automatically invalidate the opposite side or the
   whole cross-section.
10. If the search reaches `search_extent` without a stop, report
    `SEARCH_EXTENT` and use that extent as the certified finite bound.
11. Never infer free space through unknown cells.

The returned `c_plus_raw` and `c_minus_raw` must be finite, nonnegative, and
independently computed.

## 9. Validity and asymmetry requirements

The solver must support and correctly classify all of these:

```text
[-2.2, +0.3]  valid, asymmetric, contains zero
[-2.0, -0.2]  valid, asymmetric, does not contain zero
[+0.1, +1.4]  valid, asymmetric, does not contain zero
[+0.05,-0.05] invalid/empty
```

Forbidden logic includes:

```cpp
if (lower > 0.0 || upper < 0.0) invalid = true;
if (!(lower <= 0.0 && 0.0 <= upper)) invalid = true;
half_width = min(c_plus, c_minus) - margin;
lower = -half_width;
upper = half_width;
```

The solver must not include a controller offset clamp. `search_extent` is a map
observation extent, not a controller design limit.

## 10. Curvature requirements

Implement exactly the signed intersection stated in Section 4.

- Positive curvature contracts only the upper side.
- Negative curvature contracts only the lower side.
- Do not use `abs(kappa)` to contract both sides.
- A finite `|kappa| <= curvature_epsilon` leaves the obstacle interval
  unchanged.
- Non-finite curvature produces `CURVATURE_NUMERICAL_FAILURE`.
- Distinguish obstacle-empty from curvature-empty in the result reason.

## 11. Required deterministic unit tests

The new test target must cover at least the following cases with explicit
numeric assertions. Use synthetic raw occupancy functions; do not use ROS or
SDFMap.

### Case A: open space

```text
search_extent = 3.0
r_eff = 0.30
kappa = 0
expected interval = [-2.70,+2.70]
width = 5.40
```

This proves no hidden `0.20/0.30` controller cap exists.

### Case B: obstacle only near +N

```text
c_plus = 0.60
c_minus = 2.50
r_eff = 0.30
expected = [-2.20,+0.30]
```

It must not become `[-0.30,+0.30]`.

### Case C1: narrow but nonempty corridor

```text
c_plus = 0.35
c_minus = 0.35
r_eff = 0.30
expected = [-0.05,+0.05]
valid = true
```

### Case C2: genuinely empty corridor

```text
c_plus = 0.25
c_minus = 0.25
r_eff = 0.30
expected lower = +0.05
expected upper = -0.05
valid = false
reason = EMPTY_AFTER_OBSTACLE_BOUNDS
```

### One-sided legal interval

Choose raw distances that produce:

```text
expected = [-2.00,-0.20]
valid = true
contains_zero = false
```

Also verify `contains_preferred_delta` independently for a preferred delta
inside and outside the same interval.

### Signed curvature

- positive curvature changes only upper;
- negative curvature changes only lower;
- near-zero curvature changes neither;
- non-finite curvature fails with the precise reason;
- a curvature intersection that empties the obstacle interval reports
  `EMPTY_AFTER_CURVATURE_INTERSECTION`.

### Unknown and map boundary

- unknown/out-of-map after a certified free prefix conservatively truncates
  only that side and may still return a valid asymmetric interval;
- unknown/out-of-map/unavailable at the center fails with the precise reason;
- an occupied center fails;
- unknown must never be crossed and interpreted as free.

### Configuration and geometry

- zero/non-finite normal rejected;
- non-unit normal is normalized, so distances remain metric;
- invalid step, extent, tolerance, margin, or regularity settings rejected;
- all public numeric outputs are finite on every valid result.

Use deterministic tolerances derived from `boundary_tolerance`; do not weaken
tests with broad arbitrary epsilons.

## 12. Build and regression requirements

Add the new source to the existing `phase_offset_navigation` library and add one
new gtest target. Keep CMake changes minimal.

Required checks:

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_phase_offset_core
catkin_test_results --verbose
```

Run the new test executable directly as well and report its exact count.

Existing failures unrelated to this stage must not be fixed. If a historical
workspace-wide failure appears, report it separately after confirming all
authorized tests pass.

Do not run ROS, roslaunch, rosbag, RViz, or simulation in A5-G1.

## 13. Dependency and boundary audit

The new header/source/test must contain no dependency or identifier associated
with:

```text
ros
SDFMap
plan_env
visualization_msgs
Marker
gvf_manager
ContinuousPhasePath
TubeEpochManager
PhaseOffsetRuntime
PositionCommand
planner
C2
swarm
neighbor
CBF
```

Eigen, STL, `distance_query.h`, and focused phase-offset navigation concepts are
allowed.

Keep the implementation focused and preferably below 300 lines excluding
tests. No formulas in CMake or README.

## 14. Forbidden changes and shortcuts

Do not:

- tune B-spline obstacle weights;
- change map inflation, planner limits, speed, acceleration, saturation, GVF,
  governor, or SO3 parameters;
- change current `max_offset`, `0.70 m` erosion, Runtime, markers, or launch;
- add a feature flag to production integration;
- modify the existing TubeBuilder in this stage;
- create a second SDFMap or raw ESDF implementation;
- claim current ROS tube behavior is fixed;
- claim map-snapshot coherence;
- add structured ROS messages or more diagnostic array fields;
- restore prototype code;
- commit, stage, tag, branch, or push.

The expected A5-G1 result is a proven pure kernel that is not yet wired into
the active ROS path.

## 15. Stop conditions

Stop and report without expanding scope if:

- a required edit is outside the whitelist;
- the trusted branch/HEAD/stash/staged-state precondition fails;
- the proposal cannot be located or read;
- the desired raw-occupancy semantics cannot be expressed without changing
  existing production interfaces;
- an existing unrelated defect prevents authorized tests from running;
- any request would require Runtime, TubeEpochManager, adapter, SDFMap, planner,
  C2, Marker, launch, or ROS changes.

## 16. Final self-audit and report

Before stopping:

```bash
git diff --check
git diff --cached --name-only
git status --short
```

Verify:

- only the five whitelist paths changed;
- all protected-file SHA-256 values are unchanged;
- no staged content exists;
- HEAD and prototype stash are unchanged;
- no ROS processes were started;
- the new test demonstrates every numeric case in Section 11;
- no A5-G2/A5-G3/A5.3/A6/A7 work was started.

The final report must state:

1. exact modified/added files;
2. the final public API and mathematical formula;
3. exact output of Cases A, B, C1, C2, and the one-sided case;
4. signed-curvature and unknown-boundary test results;
5. build and complete test counts;
6. dependency-search results;
7. `git diff --check`, staged state, HEAD, stash, and protected-hash results;
8. an explicit statement that the component is not yet connected to ROS;
9. an explicit stop statement.

A5 remains blocked after A5-G1. A new dedicated execution specification is
required before A5-G2 integration.
