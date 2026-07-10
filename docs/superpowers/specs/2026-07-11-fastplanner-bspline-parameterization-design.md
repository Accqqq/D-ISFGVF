# Fast-Planner-Style B-Spline Parameterization Design

## Objective

Replace the current direct `K -> K+4` control-point construction in `gvf_manager::astaropt()` with a Fast-Planner-style `K -> K+2` least-squares parameterization that uses the actual sampling interval returned by Kino A*. The same path-generation pipeline will serve point-to-point navigation and closed-reference navigation.

The change must preserve the existing Kino A* search, target selection, partial-path acceptance, FSM switching, GVF tracking, ROS topics, and command interfaces.

## Problem Statement

The current pipeline obtains a Kino A* point set and boundary derivatives, but then constructs the B-spline with a separate time scale and treats the interior Kino samples as B-spline control points:

```text
Kino point_set
  -> cps_num = K + 4
  -> first/last three control points from boundary P/V/A
  -> interior samples copied directly into control points
  -> interval derived from planning/dist_p / planning/max_vel
```

A cubic B-spline generally does not interpolate its control points. Therefore a valid, ordered Kino point set can be converted into a curve with extra spans, overshoot, backtracking, or local self-crossing. The returned Kino sampling interval is stored in `astaropt()` but is not used to construct or optimize the spline.

Fast-Planner instead solves one least-squares system containing the sampled positions and the four boundary derivatives. For `K` samples it produces `K+2` control points and uses the same `ts` for parameterization, optimization, derivative evaluation, and feasibility handling.

## Approved Scope

### Included

- Apply one Fast-Planner-style parameterization pipeline to both point-to-point and closed-reference calls to `astaropt()`.
- Read the initial Kino sampling interval from `gvf/kino_sample_ts` and clamp it by `gvf/kino_sample_ts_min`.
- Preserve the actual interval returned by `KinodynamicAstar::getSamples()`.
- Parameterize the complete Kino sample set; do not truncate it before constructing the spline.
- Generate `K+2` cubic B-spline control points by solving the Fast-Planner position/PVA parameterization system.
- Initialize the existing optimizer directly from those parameterized control points and the actual interval.
- Keep the first and last three control points fixed during the existing normal optimization, preserving the parameterized boundary state.
- Correct the no-shot terminal velocity source in `KinodynamicAstar::getSamples()` from the root node to `path_nodes_.back()`.
- Compute a velocity/acceleration feasibility ratio after optimization and uniformly enlarge the spline interval when necessary. Uniform scaling changes timing, not geometry.
- Add focused unit tests for parameterization, boundary derivatives, duration, control-point count, and time scaling.

### Explicitly Excluded

- No self-intersection rejection or post-publication safety gate.
- No new collision-clearance gate in `astaropt()`.
- No rejection of `NEAR_END`, `REACH_HORIZON`, or existing partial paths.
- No change to closed-goal candidate selection or `partial_best_end_dist` behavior.
- No change to FSM acceptance policy, GVF progress logic, trajectory switching, or command generation.
- No guide-path cost or topology optimizer in this iteration.
- No full dependency on the external Fast-Planner catkin packages.
- No replacement of the existing optimizer with Fast-Planner's optimizer.

## Alternatives Considered

### 1. Minimal Fast-Planner Core Transplant — Selected

Copy the mathematical parameterization into the existing `UniformBspline` module, let the current optimizer consume the resulting control points, and use uniform time scaling for feasibility.

Benefits:

- Directly removes the `K+4` geometric mismatch.
- Applies uniformly to point-to-point and closed-reference navigation.
- Keeps all ROS interfaces and GVF behavior unchanged.
- Avoids a new package dependency and limits the regression surface.

### 2. Import Full Fast-Planner `NonUniformBspline` and Optimizer

This would provide per-knot time reallocation and the complete Fast-Planner optimizer, but it would replace the spline representation, derivative handling, optimizer API, and sampling code. The migration is substantially larger than required to address the current geometric distortion.

### 3. Change Only the Fixed `0.2 s` Interval

Changing the interval can reduce speed or acceleration, but it leaves the direct `K+4` control-point construction unchanged. It cannot reliably remove geometric overshoot or backtracking and is therefore insufficient.

## Architecture

### `UniformBspline`

Add a static parameterization function with the following interface:

```cpp
static bool parameterizeToBspline(
    double ts,
    const std::vector<Eigen::Vector3d>& point_set,
    const std::vector<Eigen::Vector3d>& start_end_derivatives,
    Eigen::MatrixXd& control_points);
```

The implementation will reproduce Fast-Planner's cubic least-squares system:

- `K` position equations using `[1, 4, 1] / 6`.
- Initial and terminal velocity equations using `[-1, 0, 1] / (2 ts)`.
- Initial and terminal acceleration equations using `[1, -2, 1] / ts^2`.
- `K+4` equations and `K+2` unknown control points.

Add initialization from already-parameterized control points:

```cpp
bool setControlPointsAndInterval(
    const Eigen::MatrixXd& control_points,
    int order,
    double interval);
```

This method will initialize spline dimensions, knot indices, `beta = 1 / interval`, valid parameter/time ranges, and sampling state without reconstructing boundary control points.

Add timing helpers:

```cpp
double getFeasibilityRatio(double max_vel, double max_acc) const;
bool scaleTime(double ratio);
```

The ratio will be:

```text
max(max_component_velocity / max_vel,
    sqrt(max_component_acceleration / max_acc))
```

When the ratio exceeds `1`, the interval will be multiplied by `ratio * 1.01`. Control points remain unchanged, so the geometric curve remains unchanged.

### `bspline_optimizer`

Add an initialization API for precomputed control points:

```cpp
bool setInitialControlPoints(
    const Eigen::MatrixXd& control_points,
    double interval);
```

It will set `control_points_`, `cps_num_`, `bspline_interval_`, and `beta_`. The existing optimizer will continue to optimize only indices `[p_order_, cps_num_ - p_order_)`, so the parameterized first and last three control points remain fixed.

The existing `set3DPath2()` and `setSplineParam()` APIs remain available for compatibility but will no longer be used by the active `astaropt()` path.

### `gvf_manager`

Add manager parameters:

```cpp
double kino_sample_ts_ = 0.2;
double kino_sample_ts_min_ = 0.05;
```

`planKinoToGoal()` will initialize `samples.ts` from these parameters before calling `getSamples()`. The returned `samples.ts` is authoritative.

After Kino sampling, `astaropt()` will use this data flow:

```text
point_set + actual ts + four boundary derivatives
  -> UniformBspline::parameterizeToBspline
  -> optimizer.setInitialControlPoints
  -> optimizer.optimize
  -> UniformBspline::setControlPointsAndInterval
  -> feasibility ratio and optional uniform time scaling
  -> position/velocity sampling
  -> existing candidate-selection and GVF publication flow
```

Point-to-point and closed-reference branches merge before this pipeline, so both modes receive the same parameterization.

The legacy `num_points_to_take_` cap is not applied inside the new Fast-Planner-style path because truncating positions while retaining derivatives from the untruncated Kino trajectory creates inconsistent terminal constraints. The parameter remains available to legacy code paths.

### `KinodynamicAstar::getSamples`

For a path without a successful shot trajectory, use the terminal search node's velocity:

```cpp
end_vel = path_nodes_.back()->state.tail(3);
```

The current code walks a local pointer back to the root while accumulating duration and then incorrectly reads the root velocity as the terminal velocity. This correction does not reject or otherwise change partial-path acceptance.

## Data and Time Semantics

- `samples.ts` begins with `max(kino_sample_ts_min_, kino_sample_ts_)`.
- `getSamples()` may change `samples.ts` so the total Kino duration is divided into an integer number of segments.
- The returned interval is passed unchanged into parameterization and optimizer initialization.
- For `K` Kino samples, the parameterized cubic spline has `K+2` control points and nominal duration `(K-1) * ts`.
- `K` refers to the complete sample set returned by Kino A*.
- If uniform feasibility scaling is required, the final interval and duration increase by the same factor while the spatial curve remains unchanged.
- `planning/dist_p` will no longer define the active `astaropt()` B-spline interval. It remains available for legacy APIs and unrelated code paths.

## Failure Handling

- Parameterization returns `false` for non-positive `ts`, fewer than two samples, a derivative vector whose size is not four, a non-finite matrix solution, or an unexpected `K+2` result size.
- Optimizer initialization returns `false` for fewer than seven cubic control points, a non-positive interval, or non-finite control points.
- Spline initialization and time scaling reject non-positive or non-finite intervals/ratios.
- Any construction failure causes `astaropt()` to return `false`, using the existing FSM behavior for a failed candidate. This is construction error handling, not a geometric safety gate.

## Testing Strategy

Add `test/fastplanner_parameterization_test.cpp` and register it with `catkin_add_gtest`.

The tests will cover:

1. `K=9` samples produce exactly `K+2=11` control points.
2. Evaluating the parameterized spline at all sample times reconstructs a straight-line point set within numerical tolerance.
3. Initial and terminal velocity/acceleration reconstructed from the first/last three control points match the supplied derivatives.
4. Spline duration before time scaling equals `(K-1) * ts`.
5. Feasibility scaling increases the interval for an intentionally aggressive control-point set.
6. Time scaling leaves sampled positions at equal normalized parameters unchanged.
7. The existing point-to-point-compatible straight-line case remains monotonic after optimization-free parameterization.

Build verification:

```bash
catkin_make --pkg bspline_race
catkin_make run_tests_bspline_race
catkin_test_results build/bspline_race
```

Simulation verification will use the existing `test_gvf.launch` and compare `/particle0/path` before and after the parameterization change. No new launch-mode split is introduced.

## Success Criteria

- Both point-to-point and closed-reference `astaropt()` calls use the same Fast-Planner-style parameterization.
- The complete Kino sample set is parameterized without position-only truncation.
- `K` Kino samples yield `K+2` control points.
- The actual Kino sampling interval is used through parameterization and optimization.
- Optimized spline geometry is preserved when timing is enlarged for feasibility.
- Existing ROS topics, GVF interfaces, partial acceptance, and FSM switching behavior remain unchanged.
- New unit tests and existing `bspline_race` tests pass.
- The package builds successfully with the existing catkin workspace.
