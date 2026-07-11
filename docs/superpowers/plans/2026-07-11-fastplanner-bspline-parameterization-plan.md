# Fast-Planner-Style B-Spline Parameterization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the active `astaropt()` direct `K+4` control-point construction with a shared point-to-point/closed-reference Fast-Planner-style `K+2` parameterization that uses Kino A*'s actual sampling interval.

**Architecture:** Add the Fast-Planner least-squares parameterization and uniform timing helpers to the existing `UniformBspline` class, add a direct-control-point initialization API to the existing optimizer, correct the no-shot Kino terminal velocity, and rewire `astaropt()` to use the new APIs. Keep search status handling, partial acceptance, FSM switching, GVF behavior, ROS topics, and command generation unchanged.

**Tech Stack:** ROS Noetic, catkin, C++14, Eigen3, NLopt, GoogleTest.

## Global Constraints

- Point-to-point and closed-reference navigation must use the same parameterization path.
- Do not add self-intersection, collision-clearance, or trajectory-publication safety gates.
- Do not reject `NEAR_END`, `REACH_HORIZON`, or current partial paths.
- Do not change closed-goal candidate selection, `partial_best_end_dist`, FSM acceptance, GVF progress, or command generation.
- Do not add a dependency on the external Fast-Planner catkin packages.
- Do not replace the current optimizer.
- Parameterize the complete Kino sample set; do not truncate positions before parameterization.
- Preserve existing untracked `.codex/` and `paper/` content.

---

## File Structure

- `src/swarm_planner/bspline_traj/include/bspline_race/UniformBspline_3d.h`: public parameterization, direct initialization, feasibility-ratio, and time-scaling APIs.
- `src/swarm_planner/bspline_traj/src/UniformBspline_3d.cpp`: Fast-Planner matrix solve and timing implementations.
- `src/swarm_planner/bspline_traj/include/bspline_race/bspline_opt_3d.h`: direct optimizer initialization declaration.
- `src/swarm_planner/bspline_traj/src/bspline_opt_3d.cpp`: direct optimizer initialization implementation.
- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`: Kino sampling parameters.
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`: parameter loading and `astaropt()` pipeline integration.
- `src/swarm_planner/path_searching/src/kinodynamic_astar.cpp`: correct partial-path terminal velocity source.
- `src/swarm_planner/bspline_traj/test/fastplanner_parameterization_test.cpp`: parameterization, timing, and optimizer API tests.
- `src/swarm_planner/path_searching/test/kinodynamic_samples_test.cpp`: no-shot terminal velocity regression test.
- `src/swarm_planner/bspline_traj/CMakeLists.txt`: register and link the B-spline tests.
- `src/swarm_planner/path_searching/CMakeLists.txt`: register and link the Kino sampling test.

---

### Task 1: Fast-Planner Parameterization and Uniform Timing

**Files:**
- Modify: `src/swarm_planner/bspline_traj/include/bspline_race/UniformBspline_3d.h`
- Modify: `src/swarm_planner/bspline_traj/src/UniformBspline_3d.cpp`
- Create: `src/swarm_planner/bspline_traj/test/fastplanner_parameterization_test.cpp`
- Modify: `src/swarm_planner/bspline_traj/CMakeLists.txt`

**Interfaces:**
- Consumes: Kino `point_set`, four boundary derivatives ordered as `[v0, vT, a0, aT]`, and actual `ts`.
- Produces:
  - `UniformBspline::parameterizeToBspline(double, const std::vector<Eigen::Vector3d>&, const std::vector<Eigen::Vector3d>&, Eigen::MatrixXd&) -> bool`
  - `UniformBspline::setControlPointsAndInterval(const Eigen::MatrixXd&, int, double) -> bool`
  - `UniformBspline::getFeasibilityRatio(double, double) const -> double`
  - `UniformBspline::scaleTime(double) -> bool`

- [ ] **Step 1: Register the new failing test target**

Add inside `if(CATKIN_ENABLE_TESTING)` in `src/swarm_planner/bspline_traj/CMakeLists.txt`:

```cmake
  catkin_add_gtest(fastplanner_parameterization_test
    test/fastplanner_parameterization_test.cpp)
  if(TARGET fastplanner_parameterization_test)
    target_link_libraries(fastplanner_parameterization_test
      bspline_gvf
      ${catkin_LIBRARIES}
      ${NLOPT_LIBRARIES}
    )
  endif()
```

- [ ] **Step 2: Write the failing parameterization and timing tests**

Create `src/swarm_planner/bspline_traj/test/fastplanner_parameterization_test.cpp`:

```cpp
#include <gtest/gtest.h>

#include <bspline_race/UniformBspline_3d.h>

#include <cmath>
#include <vector>

namespace {

std::vector<Eigen::Vector3d> makeStraightSamples(int count, double ts,
                                                 const Eigen::Vector3d& velocity) {
  std::vector<Eigen::Vector3d> samples;
  samples.reserve(count);
  for (int i = 0; i < count; ++i) {
    samples.push_back(static_cast<double>(i) * ts * velocity);
  }
  return samples;
}

std::vector<Eigen::Vector3d> makeBoundaryDerivatives(
    const Eigen::Vector3d& velocity) {
  return {velocity, velocity, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}

TEST(FastPlannerParameterization, ProducesKPlusTwoControlPointsAndInterpolatesLine) {
  const int k = 9;
  const double ts = 0.2;
  const Eigen::Vector3d velocity(0.5, -0.25, 0.0);
  const auto samples = makeStraightSamples(k, ts, velocity);
  const auto derivatives = makeBoundaryDerivatives(velocity);

  Eigen::MatrixXd control_points;
  ASSERT_TRUE(FLAG_Race::UniformBspline::parameterizeToBspline(
      ts, samples, derivatives, control_points));
  ASSERT_EQ(control_points.rows(), k + 2);
  ASSERT_EQ(control_points.cols(), 3);

  FLAG_Race::UniformBspline spline;
  ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, ts));

  Eigen::VectorXd query(k);
  for (int i = 0; i < k; ++i) query(i) = static_cast<double>(i) * ts;
  const Eigen::MatrixXd reconstructed = spline.getTrajectory(query);

  for (int i = 0; i < k; ++i) {
    EXPECT_NEAR((reconstructed.row(i).transpose() - samples[i]).norm(), 0.0, 1e-8);
  }
  EXPECT_NEAR(spline.t_range(1), static_cast<double>(k - 1) * ts, 1e-12);
}

TEST(FastPlannerParameterization, PreservesBoundaryVelocityAndAcceleration) {
  const int k = 9;
  const double ts = 0.2;
  const Eigen::Vector3d velocity(0.4, 0.2, 0.0);
  const auto samples = makeStraightSamples(k, ts, velocity);
  const auto derivatives = makeBoundaryDerivatives(velocity);

  Eigen::MatrixXd control_points;
  ASSERT_TRUE(FLAG_Race::UniformBspline::parameterizeToBspline(
      ts, samples, derivatives, control_points));

  FLAG_Race::UniformBspline position;
  ASSERT_TRUE(position.setControlPointsAndInterval(control_points, 3, ts));
  auto velocity_spline = position.getDerivative();
  auto acceleration_spline = velocity_spline.getDerivative();

  Eigen::VectorXd boundary_times(2);
  boundary_times << 0.0, static_cast<double>(k - 1) * ts;
  const Eigen::MatrixXd velocities = velocity_spline.getTrajectory(boundary_times);
  const Eigen::MatrixXd accelerations = acceleration_spline.getTrajectory(boundary_times);

  EXPECT_NEAR((velocities.row(0).transpose() - velocity).norm(), 0.0, 1e-8);
  EXPECT_NEAR((velocities.row(1).transpose() - velocity).norm(), 0.0, 1e-8);
  EXPECT_NEAR(accelerations.row(0).norm(), 0.0, 1e-8);
  EXPECT_NEAR(accelerations.row(1).norm(), 0.0, 1e-8);
}

TEST(FastPlannerParameterization, UniformTimeScalingPreservesGeometry) {
  const int k = 9;
  const double ts = 0.1;
  const Eigen::Vector3d velocity(4.0, 0.0, 0.0);
  const auto samples = makeStraightSamples(k, ts, velocity);
  const auto derivatives = makeBoundaryDerivatives(velocity);

  Eigen::MatrixXd control_points;
  ASSERT_TRUE(FLAG_Race::UniformBspline::parameterizeToBspline(
      ts, samples, derivatives, control_points));

  FLAG_Race::UniformBspline spline;
  ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, ts));
  const double ratio = spline.getFeasibilityRatio(2.0, 3.5);
  ASSERT_GT(ratio, 1.0);

  Eigen::VectorXd before_times(3);
  before_times << 0.0, 0.4, 0.8;
  const Eigen::MatrixXd before = spline.getTrajectory(before_times);

  ASSERT_TRUE(spline.scaleTime(ratio * 1.01));
  Eigen::VectorXd after_times = before_times * (ratio * 1.01);
  const Eigen::MatrixXd after = spline.getTrajectory(after_times);

  EXPECT_NEAR((before - after).norm(), 0.0, 1e-8);
  EXPECT_LE(spline.getFeasibilityRatio(2.0, 3.5), 1.0 + 1e-8);
}

TEST(FastPlannerParameterization, RejectsInvalidInputs) {
  Eigen::MatrixXd control_points;
  const std::vector<Eigen::Vector3d> one_point{Eigen::Vector3d::Zero()};
  const std::vector<Eigen::Vector3d> four_derivatives(4, Eigen::Vector3d::Zero());

  EXPECT_FALSE(FLAG_Race::UniformBspline::parameterizeToBspline(
      0.0, one_point, four_derivatives, control_points));
  EXPECT_FALSE(FLAG_Race::UniformBspline::parameterizeToBspline(
      0.2, one_point, four_derivatives, control_points));
}

}  // namespace
```

- [ ] **Step 3: Run the test target and verify RED**

Run:

```bash
catkin_make --pkg bspline_race --make-args fastplanner_parameterization_test
```

Expected: compilation fails because `parameterizeToBspline`, `setControlPointsAndInterval`, `getFeasibilityRatio`, and `scaleTime` do not exist.

- [ ] **Step 4: Declare the new `UniformBspline` APIs**

Add to the public function section of `UniformBspline_3d.h`:

```cpp
static bool parameterizeToBspline(
    double ts,
    const std::vector<Eigen::Vector3d>& point_set,
    const std::vector<Eigen::Vector3d>& start_end_derivatives,
    Eigen::MatrixXd& control_points);

bool setControlPointsAndInterval(const Eigen::MatrixXd& control_points,
                                 int order,
                                 double interval);

double getFeasibilityRatio(double max_vel, double max_acc) const;
bool scaleTime(double ratio);
```

- [ ] **Step 5: Implement the Fast-Planner matrix parameterization**

Add to `UniformBspline_3d.cpp` inside namespace `FLAG_Race`:

```cpp
bool UniformBspline::parameterizeToBspline(
    double ts,
    const std::vector<Eigen::Vector3d>& point_set,
    const std::vector<Eigen::Vector3d>& start_end_derivatives,
    Eigen::MatrixXd& control_points) {
  if (!std::isfinite(ts) || ts <= 0.0 || point_set.size() < 2 ||
      start_end_derivatives.size() != 4) {
    return false;
  }

  const int k = static_cast<int>(point_set.size());
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(k + 4, k + 2);
  const Eigen::RowVector3d position_row(1.0, 4.0, 1.0);
  const Eigen::RowVector3d velocity_row(-1.0, 0.0, 1.0);
  const Eigen::RowVector3d acceleration_row(1.0, -2.0, 1.0);

  for (int i = 0; i < k; ++i) {
    a.block<1, 3>(i, i) = position_row / 6.0;
  }
  a.block<1, 3>(k, 0) = velocity_row / (2.0 * ts);
  a.block<1, 3>(k + 1, k - 1) = velocity_row / (2.0 * ts);
  a.block<1, 3>(k + 2, 0) = acceleration_row / (ts * ts);
  a.block<1, 3>(k + 3, k - 1) = acceleration_row / (ts * ts);

  Eigen::MatrixXd b(k + 4, 3);
  for (int i = 0; i < k; ++i) b.row(i) = point_set[i].transpose();
  for (int i = 0; i < 4; ++i) {
    b.row(k + i) = start_end_derivatives[i].transpose();
  }

  control_points = a.colPivHouseholderQr().solve(b);
  return control_points.rows() == k + 2 && control_points.cols() == 3 &&
         control_points.allFinite();
}
```

- [ ] **Step 6: Implement direct spline initialization and timing helpers**

Add to `UniformBspline_3d.cpp`:

```cpp
bool UniformBspline::setControlPointsAndInterval(
    const Eigen::MatrixXd& control_points, int order, double interval) {
  if (order < 1 || control_points.cols() != 3 ||
      control_points.rows() < order + 1 || !control_points.allFinite() ||
      !std::isfinite(interval) || interval <= 0.0) {
    return false;
  }

  p_ = order;
  D_ = static_cast<int>(control_points.cols());
  n_ = static_cast<int>(control_points.rows()) - 1;
  m_ = p_ + n_ + 1;
  beta_ = 1.0 / interval;
  control_points_ = control_points;
  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i) u_(i) = static_cast<double>(i);
  setIniTerMatrix();
  getAvailableSrange();
  getAvailableTrange();
  getInterval();
  time_.resize(0);
  return true;
}

double UniformBspline::getFeasibilityRatio(double max_vel, double max_acc) const {
  if (!std::isfinite(max_vel) || !std::isfinite(max_acc) ||
      max_vel <= 0.0 || max_acc <= 0.0 || control_points_.rows() < 2) {
    return std::numeric_limits<double>::infinity();
  }

  double velocity_max = 0.0;
  for (int i = 0; i + 1 < control_points_.rows(); ++i) {
    const Eigen::RowVectorXd velocity =
        beta_ * (control_points_.row(i + 1) - control_points_.row(i));
    velocity_max = std::max(velocity_max, velocity.cwiseAbs().maxCoeff());
  }

  double acceleration_max = 0.0;
  for (int i = 0; i + 2 < control_points_.rows(); ++i) {
    const Eigen::RowVectorXd acceleration = beta_ * beta_ *
        (control_points_.row(i + 2) - 2.0 * control_points_.row(i + 1) +
         control_points_.row(i));
    acceleration_max =
        std::max(acceleration_max, acceleration.cwiseAbs().maxCoeff());
  }

  return std::max(velocity_max / max_vel,
                  std::sqrt(acceleration_max / max_acc));
}

bool UniformBspline::scaleTime(double ratio) {
  if (!std::isfinite(ratio) || ratio <= 0.0 ||
      !std::isfinite(beta_) || beta_ <= 0.0) {
    return false;
  }
  beta_ /= ratio;
  setIniTerMatrix();
  getAvailableTrange();
  getInterval();
  time_.resize(0);
  return true;
}
```

Also add `#include <cmath>` and `#include <limits>` to `UniformBspline_3d.cpp`.

- [ ] **Step 7: Run the test and verify GREEN**

Run:

```bash
catkin_make --pkg bspline_race --make-args fastplanner_parameterization_test
devel/lib/bspline_race/fastplanner_parameterization_test
```

Expected: `fastplanner_parameterization_test` builds and all four tests pass.

- [ ] **Step 8: Commit Task 1**

```bash
git add src/swarm_planner/bspline_traj/CMakeLists.txt \
  src/swarm_planner/bspline_traj/include/bspline_race/UniformBspline_3d.h \
  src/swarm_planner/bspline_traj/src/UniformBspline_3d.cpp \
  src/swarm_planner/bspline_traj/test/fastplanner_parameterization_test.cpp
git commit -m "feat: add Fast-Planner spline parameterization"
```

---

### Task 2: Initialize the Existing Optimizer from Parameterized Control Points

**Files:**
- Modify: `src/swarm_planner/bspline_traj/test/fastplanner_parameterization_test.cpp`
- Modify: `src/swarm_planner/bspline_traj/include/bspline_race/bspline_opt_3d.h`
- Modify: `src/swarm_planner/bspline_traj/src/bspline_opt_3d.cpp`

**Interfaces:**
- Consumes: `K+2` parameterized control points and actual `ts` from Task 1.
- Produces: `bspline_optimizer::setInitialControlPoints(const Eigen::MatrixXd&, double) -> bool`.

- [ ] **Step 1: Add the failing optimizer initialization test**

Add this include with the other includes at the top of `fastplanner_parameterization_test.cpp`:

```cpp
#include <bspline_race/bspline_opt_3d.h>
```

Append this test to the file:

```cpp
TEST(FastPlannerParameterization, OptimizerAcceptsParameterizedControlPoints) {
  Eigen::MatrixXd control_points(11, 3);
  for (int i = 0; i < control_points.rows(); ++i) {
    control_points.row(i) << 0.1 * i, -0.05 * i, 1.0;
  }

  FLAG_Race::bspline_optimizer optimizer;
  optimizer.setDimandOrder(3, 3);
  ASSERT_TRUE(optimizer.setInitialControlPoints(control_points, 0.2));
  EXPECT_EQ(optimizer.cps_num_, 11);
  EXPECT_NEAR(optimizer.bspline_interval_, 0.2, 1e-12);
  EXPECT_NEAR(optimizer.beta_, 5.0, 1e-12);
  EXPECT_NEAR((optimizer.control_points_ - control_points).norm(), 0.0, 1e-12);
}
```

- [ ] **Step 2: Run the test target and verify RED**

Run:

```bash
catkin_make --pkg bspline_race --make-args fastplanner_parameterization_test
devel/lib/bspline_race/fastplanner_parameterization_test
```

Expected: compilation fails because `setInitialControlPoints` does not exist.

- [ ] **Step 3: Declare and implement optimizer initialization**

Add to `bspline_opt_3d.h`:

```cpp
bool setInitialControlPoints(const Eigen::MatrixXd& control_points,
                             double interval);
```

Add to `bspline_opt_3d.cpp`:

```cpp
bool bspline_optimizer::setInitialControlPoints(
    const Eigen::MatrixXd& control_points, double interval) {
  if (control_points.cols() != Dim_ ||
      control_points.rows() <= 2 * p_order_ ||
      !control_points.allFinite() ||
      !std::isfinite(interval) || interval <= 0.0) {
    return false;
  }
  control_points_ = control_points;
  cps_num_ = static_cast<int>(control_points.rows());
  bspline_interval_ = interval;
  beta_ = 1.0 / interval;
  return true;
}
```

Also add `#include <cmath>` to `bspline_opt_3d.cpp` if it is not already available through a direct include.

- [ ] **Step 4: Run the test and verify GREEN**

Run:

```bash
catkin_make --pkg bspline_race --make-args fastplanner_parameterization_test
devel/lib/bspline_race/fastplanner_parameterization_test
```

Expected: all `fastplanner_parameterization_test` tests pass.

- [ ] **Step 5: Commit Task 2**

```bash
git add src/swarm_planner/bspline_traj/include/bspline_race/bspline_opt_3d.h \
  src/swarm_planner/bspline_traj/src/bspline_opt_3d.cpp \
  src/swarm_planner/bspline_traj/test/fastplanner_parameterization_test.cpp
git commit -m "feat: initialize optimizer from parameterized spline"
```

---

### Task 3: Correct No-Shot Kino Terminal Velocity

**Files:**
- Create: `src/swarm_planner/path_searching/test/kinodynamic_samples_test.cpp`
- Modify: `src/swarm_planner/path_searching/CMakeLists.txt`
- Modify: `src/swarm_planner/path_searching/src/kinodynamic_astar.cpp`

**Interfaces:**
- Consumes: existing public `PathNode`, `path_nodes_`, and `getSamples()` APIs.
- Produces: boundary derivative index `1` equal to the terminal search node velocity when `is_shot_succ_ == false`.

- [ ] **Step 1: Register the Kino sampling test**

Append to `path_searching/CMakeLists.txt`:

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(kinodynamic_samples_test test/kinodynamic_samples_test.cpp)
  if(TARGET kinodynamic_samples_test)
    target_link_libraries(kinodynamic_samples_test
      path_searching
      ${catkin_LIBRARIES}
    )
  endif()
endif()
```

- [ ] **Step 2: Write the failing regression test**

Create `src/swarm_planner/path_searching/test/kinodynamic_samples_test.cpp`:

```cpp
#include <gtest/gtest.h>

#include <path_searching/kinodynamic_astar.h>

TEST(KinodynamicSamples, NoShotUsesTerminalNodeVelocity) {
  FLAG_Race::PathNode start;
  FLAG_Race::PathNode terminal;

  start.parent = nullptr;
  start.state.setZero();
  start.state.tail<3>() << 0.25, 0.0, 0.0;

  terminal.parent = &start;
  terminal.duration = 1.0;
  terminal.input.setZero();
  terminal.state.setZero();
  terminal.state.head<3>() << 0.5, 0.0, 1.0;
  terminal.state.tail<3>() << 0.75, 0.0, 0.0;

  FLAG_Race::KinodynamicAstar astar;
  astar.phi_.setIdentity();
  astar.is_shot_succ_ = false;
  astar.start_vel_ = start.state.tail<3>();
  astar.path_nodes_ = {&start, &terminal};

  double ts = 0.2;
  std::vector<Eigen::Vector3d> point_set;
  std::vector<Eigen::Vector3d> derivatives;
  astar.getSamples(ts, point_set, derivatives);

  ASSERT_EQ(derivatives.size(), 4u);
  EXPECT_NEAR((derivatives[1] - terminal.state.tail<3>()).norm(), 0.0, 1e-12);
}
```

- [ ] **Step 3: Run the test and verify RED**

Run:

```bash
catkin_make --pkg path_searching --make-args kinodynamic_samples_test
devel/lib/path_searching/kinodynamic_samples_test
```

Expected: test fails because `derivatives[1]` equals the root/start velocity `0.25`, not terminal velocity `0.75`.

- [ ] **Step 4: Fix the terminal velocity source**

In the no-shot branch of `KinodynamicAstar::getSamples()`, replace:

```cpp
end_vel = node->state.tail(3);
```

with:

```cpp
end_vel = path_nodes_.back()->state.tail(3);
```

- [ ] **Step 5: Run the test and verify GREEN**

Run:

```bash
catkin_make --pkg path_searching --make-args kinodynamic_samples_test
devel/lib/path_searching/kinodynamic_samples_test
```

Expected: `kinodynamic_samples_test` passes.

- [ ] **Step 6: Commit Task 3**

```bash
git add src/swarm_planner/path_searching/CMakeLists.txt \
  src/swarm_planner/path_searching/src/kinodynamic_astar.cpp \
  src/swarm_planner/path_searching/test/kinodynamic_samples_test.cpp
git commit -m "fix: use terminal velocity for partial Kino samples"
```

---

### Task 4: Integrate the Parameterized Pipeline into `astaropt()`

**Files:**
- Modify: `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- Modify: `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`

**Interfaces:**
- Consumes: all APIs produced by Tasks 1–3.
- Produces: one shared `astaropt()` B-spline construction path for point-to-point and closed-reference navigation.

- [ ] **Step 1: Add Kino sampling members and load parameters**

Add beside the existing timer/sample configuration in `gvf_manager.h`:

```cpp
double kino_sample_ts_ = 0.2;
double kino_sample_ts_min_ = 0.05;
```

In the `gvf_manager` constructor, load:

```cpp
nh.param("gvf/kino_sample_ts", kino_sample_ts_, 0.2);
nh.param("gvf/kino_sample_ts_min", kino_sample_ts_min_, 0.05);
kino_sample_ts_min_ = std::max(1e-3, kino_sample_ts_min_);
kino_sample_ts_ = std::max(kino_sample_ts_min_, kino_sample_ts_);
```

- [ ] **Step 2: Use configured initial `ts` in both Kino sampling attempts**

In both success paths of `planKinoToGoal()`, replace `samples.ts = 0.2;` with:

```cpp
samples.ts = std::max(kino_sample_ts_min_, kino_sample_ts_);
```

Keep the call to `getSamples()` immediately after it so the returned `samples.ts` becomes authoritative.

- [ ] **Step 3: Replace the active `K+4` construction block in `astaropt()`**

Delete the active position-only truncation, `initial_state`/`terminal_state`, `set3DPath2()`, `setIniandTerandCpsnum()`, and `setSplineParam()` calls. Replace them with:

```cpp
if (point_set.size() < 2 || start_end_derivatives.size() != 4 ||
    !std::isfinite(ts) || ts <= 0.0) {
  ROS_WARN("[gvf kino replan] invalid Kino samples: points=%zu derivatives=%zu ts=%.6f",
           point_set.size(), start_end_derivatives.size(), ts);
  return false;
}

Eigen::MatrixXd initial_control_points;
if (!UniformBspline::parameterizeToBspline(
        ts, point_set, start_end_derivatives, initial_control_points)) {
  ROS_WARN("[gvf kino replan] Fast-Planner parameterization failed");
  return false;
}

if (!pm.bspline_opt_->setInitialControlPoints(initial_control_points, ts)) {
  ROS_WARN("[gvf kino replan] optimizer rejected parameterized control points");
  return false;
}

pm.bspline_opt_->optimize();

if (!pm.spline_->setControlPointsAndInterval(
        pm.bspline_opt_->control_points_, 3, ts)) {
  ROS_WARN("[gvf kino replan] spline initialization failed");
  return false;
}

const double feasibility_ratio = pm.spline_->getFeasibilityRatio(
    pm.bspline_opt_->max_vel_, pm.bspline_opt_->max_acc_);
if (!std::isfinite(feasibility_ratio)) {
  ROS_WARN("[gvf kino replan] invalid feasibility ratio");
  return false;
}
if (feasibility_ratio > 1.0 &&
    !pm.spline_->scaleTime(feasibility_ratio * 1.01)) {
  ROS_WARN("[gvf kino replan] time scaling failed");
  return false;
}

pm.spline_->getT();
```

Leave the existing position/velocity sampling and `new_i0_out` selection below this block unchanged.

- [ ] **Step 4: Add a diagnostic proving the active point/control/time relationship**

After `pm.spline_->getT()`, add:

```cpp
ROS_INFO_THROTTLE(
    1.0,
    "[GVF][BSPLINE_PARAM] kino_points=%zu control_points=%d kino_ts=%.4f final_interval=%.4f ratio=%.3f",
    point_set.size(), static_cast<int>(pm.bspline_opt_->control_points_.rows()),
    ts, pm.spline_->interval_, feasibility_ratio);
```

- [ ] **Step 5: Build both affected packages**

Run:

```bash
catkin_make --pkg path_searching bspline_race
```

Expected: both packages build without errors. Warnings already present in unrelated legacy code do not count as failures.

- [ ] **Step 6: Run focused and existing tests**

Run:

```bash
catkin_make run_tests_path_searching run_tests_bspline_race
catkin_test_results build
```

Expected: new parameterization and Kino sampling tests pass, and existing `bspline_race` tests remain green.

- [ ] **Step 7: Commit Task 4**

```bash
git add src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp
git commit -m "refactor: use Fast-Planner parameterization in astaropt"
```

---

### Task 5: Final Regression Verification

**Files:**
- No production-file changes expected.
- Update this plan's checkboxes only if the execution workflow tracks completion in the plan.

**Interfaces:**
- Consumes: completed Tasks 1–4.
- Produces: evidence that the shared point-to-point/closed-reference pipeline builds and passes tests without adding excluded behavior.

- [ ] **Step 1: Verify the final diff contains no excluded behavior**

Run:

```bash
git diff 073477a..HEAD -- src/swarm_planner/bspline_traj src/swarm_planner/path_searching
```

Confirm there is no self-intersection check, new collision-clearance check, `REACH_HORIZON`/`NEAR_END` rejection, partial-path rejection, or GVF/FSM policy change.

- [ ] **Step 2: Run the full package build**

Run:

```bash
catkin_make --pkg path_searching bspline_race
```

Expected: exit code `0`.

- [ ] **Step 3: Run all registered tests and inspect results**

Run:

```bash
catkin_make run_tests_path_searching run_tests_bspline_race
catkin_test_results build
```

Expected: zero failed tests.

- [ ] **Step 4: Verify repository scope**

Run:

```bash
git status --short
```

Expected: only the user's pre-existing untracked `.codex/` and `paper/` entries remain, unless the execution workflow intentionally leaves the plan document uncommitted.

- [ ] **Step 5: Prepare simulation handoff**

Report that `test_gvf.launch` already provides `gvf/kino_sample_ts=0.15` and `gvf/kino_sample_ts_min=0.05`, which the new code now reads. Recommend simulation comparison of `/particle0/path` before porting the same source changes into the external real-flight workspace.
