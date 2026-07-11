# Closed-Goal Progressive Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace one-shot closed-goal obstacle pushing and target-relative partial fallback with a bounded progressive selector that preserves normal `1.0 m` tracking, avoids repeated `0.15–0.35 m` paths when better partials exist, and prevents TRACK from jumping directly to `3.0 m` goals.

**Architecture:** Add pure scalar helpers for required progress, TRACK lookahead bounds, and deterministic candidate ordering. `selectClosedGoalCandidate()` continues generating KinoA* candidates, but TRACK only considers the progressive window and ranks every valid full/partial result by actual endpoint phase progress, closeness to the unpushed desired lookahead, and Kino geometric length. Existing obstacle scanning remains diagnostic; collision detection, replanning triggers, governor logic, point-goal behavior, and trajectory switching remain unchanged.

**Tech Stack:** ROS1, catkin, C++14, Eigen, GoogleTest

## Global Constraints

- Do not add a reference-path deviation cost.
- Do not reject valid partial KinoA* results.
- Do not modify point-to-point goal selection.
- Do not modify `checkCollision()`, `shouldAcceptCandidate()`, FSM triggers, governor commands, or trajectory switch semantics.
- TRACK uses `goal_prefer_lookahead_w + 0.75 m`, capped by the existing maximum lookahead.
- RECOVER keeps the complete existing candidate range.
- Required progress is `clamp(odom_speed_xy * 1.0 s, 0.6 m, 0.8 m)`.
- Obstacle scanning and bypass fields remain diagnostic and no longer directly push `desired_lookahead`.

---

### Task 1: Add deterministic progressive-selection policy helpers

**Files:**
- Modify: `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- Test: `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`

**Interfaces:**
- Produces: `gvf_manager::ClosedGoalProgressiveCandidate`
- Produces: `closedGoalRequiredProgress(double, double, double, double)`
- Produces: `closedGoalProgressiveMaxLookahead(double, double, double, bool)`
- Produces: `preferClosedGoalProgressiveCandidate(lhs, rhs, required_progress, desired_lookahead)`

- [ ] **Step 1: Write failing policy tests**

Add this helper and tests to `gvf_switch_policy_test.cpp`:

```cpp
FLAG_Race::gvf_manager::ClosedGoalProgressiveCandidate progressiveCandidate(
    int idx, double lookahead, double end_delta_w,
    double kino_path_length, double end_to_goal_dist)
{
  FLAG_Race::gvf_manager::ClosedGoalProgressiveCandidate value;
  value.valid = true;
  value.idx = idx;
  value.lookahead = lookahead;
  value.end_delta_w = end_delta_w;
  value.kino_path_length = kino_path_length;
  value.end_to_goal_dist = end_to_goal_dist;
  return value;
}

TEST(GvfClosedGoalProgressivePolicy, ComputesBoundedRequiredProgress)
{
  EXPECT_NEAR(0.6, FLAG_Race::gvf_manager::closedGoalRequiredProgress(
      0.0, 1.0, 0.6, 0.8), 1e-6);
  EXPECT_NEAR(0.7, FLAG_Race::gvf_manager::closedGoalRequiredProgress(
      0.7, 1.0, 0.6, 0.8), 1e-6);
  EXPECT_NEAR(0.8, FLAG_Race::gvf_manager::closedGoalRequiredProgress(
      1.2, 1.0, 0.6, 0.8), 1e-6);
}

TEST(GvfClosedGoalProgressivePolicy, BoundsTrackButNotRecoverLookahead)
{
  EXPECT_NEAR(1.75, FLAG_Race::gvf_manager::closedGoalProgressiveMaxLookahead(
      1.0, 3.0, 0.75, false), 1e-6);
  EXPECT_NEAR(3.0, FLAG_Race::gvf_manager::closedGoalProgressiveMaxLookahead(
      1.0, 3.0, 0.75, true), 1e-6);
}

TEST(GvfClosedGoalProgressivePolicy, KeepsDesiredLookaheadWhenProgressIsSufficient)
{
  const auto desired = progressiveCandidate(2, 1.0, 1.0, 1.2, 0.0);
  const auto shorter = progressiveCandidate(1, 0.75, 0.75, 0.8, 0.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      desired, shorter, 0.6, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, ChoosesUsefulPartialOverTinyNearGoalPartial)
{
  const auto useful = progressiveCandidate(4, 1.5, 0.82, 1.1, 0.68);
  const auto tiny = progressiveCandidate(0, 0.5, 0.15, 0.2, 0.35);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      useful, tiny, 0.8, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, ChoosesFarthestWhenNoneIsSufficient)
{
  const auto farther = progressiveCandidate(4, 1.5, 0.55, 0.9, 0.95);
  const auto nearer = progressiveCandidate(2, 1.0, 0.20, 0.3, 0.80);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      farther, nearer, 0.8, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, UsesKinoLengthAfterEqualDesiredError)
{
  const auto short_path = progressiveCandidate(1, 0.75, 0.8, 1.0, 0.0);
  const auto long_path = progressiveCandidate(3, 1.25, 0.8, 4.0, 0.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      short_path, long_path, 0.8, 1.0));
}
```

- [ ] **Step 2: Run the policy test target and verify it fails**

Run:

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
```

Expected: compilation fails because the progressive type and helpers do not exist.

- [ ] **Step 3: Implement the minimal pure helpers in `gvf_manager.h`**

```cpp
struct ClosedGoalProgressiveCandidate
{
    bool valid = false;
    int idx = -1;
    double lookahead = 0.0;
    double end_delta_w = 0.0;
    double kino_path_length = std::numeric_limits<double>::infinity();
    double end_to_goal_dist = std::numeric_limits<double>::infinity();
};

static double closedGoalRequiredProgress(double speed_xy,
                                         double progress_time,
                                         double min_progress,
                                         double max_progress)
{
    const double lo = std::max(0.0, std::min(min_progress, max_progress));
    const double hi = std::max(lo, std::max(min_progress, max_progress));
    const double raw = std::max(0.0, speed_xy) * std::max(0.0, progress_time);
    return std::max(lo, std::min(raw, hi));
}

static double closedGoalProgressiveMaxLookahead(double preferred,
                                                double configured_max,
                                                double extra,
                                                bool recover_mode)
{
    const double hi = std::max(0.0, configured_max);
    if (recover_mode) return hi;
    return std::min(hi, std::max(0.0, preferred) + std::max(0.0, extra));
}

static bool preferClosedGoalProgressiveCandidate(
    const ClosedGoalProgressiveCandidate& lhs,
    const ClosedGoalProgressiveCandidate& rhs,
    double required_progress,
    double desired_lookahead)
{
    if (lhs.valid != rhs.valid) return lhs.valid;
    if (!lhs.valid) return false;

    constexpr double eps = 1e-6;
    const bool lhs_sufficient = lhs.end_delta_w + eps >= required_progress;
    const bool rhs_sufficient = rhs.end_delta_w + eps >= required_progress;
    if (lhs_sufficient != rhs_sufficient) return lhs_sufficient;

    if (lhs_sufficient) {
        const double lhs_error = std::abs(lhs.lookahead - desired_lookahead);
        const double rhs_error = std::abs(rhs.lookahead - desired_lookahead);
        if (std::abs(lhs_error - rhs_error) > eps) return lhs_error < rhs_error;
        if (std::abs(lhs.kino_path_length - rhs.kino_path_length) > eps)
            return lhs.kino_path_length < rhs.kino_path_length;
        if (std::abs(lhs.lookahead - rhs.lookahead) > eps)
            return lhs.lookahead < rhs.lookahead;
    } else {
        if (std::abs(lhs.end_delta_w - rhs.end_delta_w) > eps)
            return lhs.end_delta_w > rhs.end_delta_w;
        if (std::abs(lhs.kino_path_length - rhs.kino_path_length) > eps)
            return lhs.kino_path_length < rhs.kino_path_length;
        if (std::abs(lhs.lookahead - rhs.lookahead) > eps)
            return lhs.lookahead < rhs.lookahead;
    }

    if (std::abs(lhs.end_to_goal_dist - rhs.end_to_goal_dist) > eps)
        return lhs.end_to_goal_dist < rhs.end_to_goal_dist;
    return lhs.idx < rhs.idx;
}
```

- [ ] **Step 4: Rebuild and run the tests**

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
./devel/lib/bspline_race/gvf_switch_policy_test
```

Expected: all existing and new policy tests pass.

- [ ] **Step 5: Commit Task 1**

```bash
git add src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
        src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
git commit -m "test: define progressive closed-goal selection policy"
```

### Task 2: Integrate progressive metrics and selection

**Files:**
- Modify: `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- Modify: `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`

**Interfaces:**
- Consumes: Task 1 helpers, existing forward-local projection, Kino samples, odometry velocity, and obstacle diagnostics.
- Produces: bounded TRACK candidate ordering, actual-progress selection, Kino path-length metrics, and expanded `[GVF][CLOSED_GOAL]` logs.

- [ ] **Step 1: Add configuration members and ROS parameters**

Add header defaults:

```cpp
double closed_goal_progressive_lookahead_extra_w_ = 0.75;
double closed_goal_progressive_min_progress_w_ = 0.6;
double closed_goal_progressive_max_progress_w_ = 0.8;
double closed_goal_progressive_progress_time_ = 1.0;
```

Load them beside the existing closed-goal parameters:

```cpp
nh.param("gvf/circle_test/progressive_lookahead_extra_w",
         closed_goal_progressive_lookahead_extra_w_, 0.75);
nh.param("gvf/circle_test/progressive_min_progress_w",
         closed_goal_progressive_min_progress_w_, 0.6);
nh.param("gvf/circle_test/progressive_max_progress_w",
         closed_goal_progressive_max_progress_w_, 0.8);
nh.param("gvf/circle_test/progressive_progress_time",
         closed_goal_progressive_progress_time_, 1.0);
```

- [ ] **Step 2: Stop obstacle scanning from pushing the desired target**

Keep `first_obstacle_delta_w`, `obstacle_end_delta_w`, `bypass_delta_w`, and `bypass_mode` for logs. Remove the assignment through `closedGoalObstaclePushedLookahead()` and set:

```cpp
const bool desired_pushed_by_obstacle = false;
```

`desired_lookahead` must remain the value from `closedGoalDesiredLookahead()` clamped to the configured candidate range.

- [ ] **Step 3: Restrict TRACK candidates to the progressive window**

Before filling `order`, calculate:

```cpp
const double progressive_max_lookahead = closedGoalProgressiveMaxLookahead(
    closed_goal_prefer_lookahead_w_, candidates.back(),
    closed_goal_progressive_lookahead_extra_w_, goal_recover_mode);
const double required_progress_w = closedGoalRequiredProgress(
    odom_v_xy.norm(), closed_goal_progressive_progress_time_,
    closed_goal_progressive_min_progress_w_,
    closed_goal_progressive_max_progress_w_);
```

Only add candidate indices satisfying:

```cpp
goal_recover_mode || candidates[i] <= progressive_max_lookahead + 1e-6
```

With current parameters TRACK considers `0.5–1.75 m`; RECOVER still considers `0.5–3.0 m`.

- [ ] **Step 4: Compute Kino geometric length**

After validating `candidate_samples.point_set`, calculate:

```cpp
double kino_path_length = 0.0;
for (size_t i = 1; i < candidate_samples.point_set.size(); ++i) {
    kino_path_length +=
        (candidate_samples.point_set[i] - candidate_samples.point_set[i - 1]).norm();
}
```

Store it in `ClosedGoalCandidateResult`, populate a `ClosedGoalProgressiveCandidate progressive` member, and append the value to a `tried_kino_path_lengths_ss` diagnostic stream. A non-finite length makes that candidate invalid; finite partials remain accepted.

- [ ] **Step 5: Replace bypass/full/partial selection branching**

Populate the scalar policy input:

```cpp
result.progressive.valid = true;
result.progressive.idx = idx;
result.progressive.lookahead = lookahead;
result.progressive.end_delta_w = end_delta_w;
result.progressive.kino_path_length = kino_path_length;
result.progressive.end_to_goal_dist = end_to_goal_dist;
```

Select across all valid full and partial candidates:

```cpp
int selected_candidate = -1;
for (size_t i = 0; i < valid_candidates.size(); ++i) {
    if (selected_candidate < 0 || preferClosedGoalProgressiveCandidate(
            valid_candidates[i].progressive,
            valid_candidates[selected_candidate].progressive,
            required_progress_w, desired_lookahead)) {
        selected_candidate = static_cast<int>(i);
    }
}
```

After selection use:

```cpp
const bool selected_progress_sufficient =
    selected_end_delta_w + 1e-6 >= required_progress_w;
selected_reason = selected_progress_sufficient
    ? "progressive_sufficient"
    : "progressive_farthest_fallback";
```

Keep `accepted_full_goal`, `accepted_partial_goal`, `passed_obstacle`, pending-goal state, and accepted-goal state for diagnostics and compatibility.

- [ ] **Step 6: Extend diagnostics**

Add these fields to `[GVF][CLOSED_GOAL]` without removing current fields:

```text
selection_mode=progressive
progressive_max_lookahead_w
required_progress_w
selected_kino_path_length
selected_progress_sufficient
tried_kino_path_lengths
```

- [ ] **Step 7: Build and run policy tests**

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
./devel/lib/bspline_race/gvf_switch_policy_test
```

Expected: build succeeds and all tests pass.

- [ ] **Step 8: Commit Task 2**

```bash
git add src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
        src/swarm_planner/bspline_traj/src/gvf_manager.cpp
git commit -m "fix: select closed goals by bounded actual progress"
```

### Task 3: Configure simulation defaults and verify the package

**Files:**
- Modify: `src/swarm_planner/bspline_traj/launch/test_gvf.launch`
- Test: `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`

**Interfaces:**
- Consumes: Task 2 ROS parameters.
- Produces: explicit simulation configuration matching the design.

- [ ] **Step 1: Add the progressive parameters beside existing closed-goal settings**

```xml
<param name="gvf/circle_test/progressive_lookahead_extra_w" value="0.75" />
<param name="gvf/circle_test/progressive_min_progress_w" value="0.6" />
<param name="gvf/circle_test/progressive_max_progress_w" value="0.8" />
<param name="gvf/circle_test/progressive_progress_time" value="1.0" />
```

Do not modify the user's current `v_max`, `a_max`, GVF gains, manual-map settings, collision settings, or KinoA* settings.

- [ ] **Step 2: Run focused tests**

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
./devel/lib/bspline_race/gvf_switch_policy_test
```

Expected: all tests pass.

- [ ] **Step 3: Build the package**

```bash
catkin_make --pkg bspline_race
```

Expected: `bspline_race` compiles successfully without errors.

- [ ] **Step 4: Verify diff scope**

```bash
git diff --check
git status --short
git diff -- src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
             src/swarm_planner/bspline_traj/src/gvf_manager.cpp \
             src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp \
             src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

Expected: point-goal, collision, FSM, governor, and switch functions are unchanged; `.codex`, `paper/`, and the user's pre-existing launch edits remain untouched.

- [ ] **Step 5: Commit only the new launch parameter lines**

`test_gvf.launch` already contains user edits. Inspect the diff and stage only the four new parameter lines with patch-based staging, then commit:

```bash
git commit -m "config: enable progressive closed-goal selection"
```

### Task 4: Review and final verification

**Files:**
- Review: all files modified in Tasks 1–3

**Interfaces:**
- Consumes: completed implementation and tests.
- Produces: reviewed, buildable progressive closed-goal selection.

- [ ] **Step 1: Request a specification-compliance review**

The reviewer checks every design requirement, especially that point-goal, collision, governor, FSM trigger, and switch-policy code did not change.

- [ ] **Step 2: Request a code-quality review**

The reviewer checks comparator transitivity, finite-value handling, stable tie-breakers, log format/argument alignment, and TRACK/RECOVER candidate windows.

- [ ] **Step 3: Apply only in-scope review fixes**

For any behavioral defect, add a focused failing test before changing implementation, then rerun the focused test target.

- [ ] **Step 4: Run final verification**

```bash
./devel/lib/bspline_race/gvf_switch_policy_test
catkin_make --pkg bspline_race
git diff --check
git status --short
```

Expected: tests pass, package compiles, no whitespace errors, and unrelated user files remain untouched.
