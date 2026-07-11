# Closed-Goal Obstacle Bypass Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Select closed-reference KinoA* candidates by their actual forward phase progress around obstacles so short partial paths no longer win merely because their own target is nearby.

**Architecture:** Keep the existing normal TRACK scoring path unchanged. When the inflated-map scan identifies both the start and end of an obstacle interval on the forward closed reference, project every valid Kino endpoint only into the current forward lookahead window and rank candidates by whether that actual endpoint passed the obstacle. Keep all logic inside closed-goal candidate generation; do not modify replanning triggers, collision checks, governor behavior, or trajectory switch acceptance.

**Tech Stack:** ROS1, catkin, C++14, Eigen, GoogleTest

## Global Constraints

- Do not add a reference-path deviation cost to KinoA*.
- Do not reject partial KinoA* paths.
- Do not modify collision detection, timed replanning, halfway forced acceptance, governor behavior, or new/old trajectory switching.
- Normal TRACK selection continues to prefer the configured short lookahead.
- Endpoint projection is forward-local only, covering `[closed_ref_w_, closed_ref_w_ + max_lookahead]`.
- An obstacle ends only after three in-map free samples following its first occupied sample.
- If the scan does not find an obstacle end, bypass ranking is disabled for that replan.

---

### Task 1: Add a deterministic bypass candidate comparator

**Files:**
- Modify: `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- Test: `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`

**Interfaces:**
- Consumes: scalar validity, obstacle-pass status, actual endpoint phase advance, endpoint-to-target distance, and target lookahead.
- Produces: `gvf_manager::ClosedGoalCandidateProgress` and `gvf_manager::preferClosedGoalBypassCandidate(lhs, rhs, bypass_mode)`.

- [ ] **Step 1: Write failing comparator tests**

Append tests that construct candidates through a local helper:

```cpp
namespace {
FLAG_Race::gvf_manager::ClosedGoalCandidateProgress candidate(
    bool passed, double end_delta_w, double end_to_goal_dist, double lookahead)
{
  FLAG_Race::gvf_manager::ClosedGoalCandidateProgress value;
  value.valid = true;
  value.passed_obstacle = passed;
  value.end_delta_w = end_delta_w;
  value.end_to_goal_dist = end_to_goal_dist;
  value.lookahead = lookahead;
  return value;
}
}

TEST(GvfClosedGoalBypassPolicy, PassedCandidateBeatsUnpassedCandidate)
{
  const auto passed = candidate(true, 1.2, 0.4, 1.5);
  const auto unpassed = candidate(false, 1.1, 0.01, 1.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalBypassCandidate(
      passed, unpassed, true));
}

TEST(GvfClosedGoalBypassPolicy, ChoosesLeastOvershootWhenBothPassed)
{
  const auto just_passed = candidate(true, 1.2, 0.3, 1.5);
  const auto far_passed = candidate(true, 2.4, 0.01, 3.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalBypassCandidate(
      just_passed, far_passed, true));
}

TEST(GvfClosedGoalBypassPolicy, ChoosesMostProgressWhenNeitherPassed)
{
  const auto farther = candidate(false, 0.9, 0.5, 1.5);
  const auto nearer = candidate(false, 0.4, 0.01, 0.5);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalBypassCandidate(
      farther, nearer, true));
}

TEST(GvfClosedGoalBypassPolicy, UsesGoalErrorThenLookaheadAsTieBreakers)
{
  const auto lower_error = candidate(false, 0.9, 0.1, 1.5);
  const auto higher_error = candidate(false, 0.9, 0.2, 1.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalBypassCandidate(
      lower_error, higher_error, true));

  const auto shorter_target = candidate(false, 0.9, 0.1, 1.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalBypassCandidate(
      shorter_target, lower_error, true));
}

TEST(GvfClosedGoalBypassPolicy, DisabledModeDoesNotOverrideNormalSelection)
{
  const auto lhs = candidate(true, 1.2, 0.1, 1.5);
  const auto rhs = candidate(false, 0.4, 0.1, 0.5);
  EXPECT_FALSE(FLAG_Race::gvf_manager::preferClosedGoalBypassCandidate(
      lhs, rhs, false));
}
```

- [ ] **Step 2: Build the test target and verify it fails**

Run:

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
```

Expected: compilation fails because `ClosedGoalCandidateProgress` and `preferClosedGoalBypassCandidate` do not exist.

- [ ] **Step 3: Add the minimal public scalar helper**

Add beside the existing closed-goal static helpers:

```cpp
struct ClosedGoalCandidateProgress
{
    bool valid = false;
    bool passed_obstacle = false;
    double end_delta_w = 0.0;
    double end_to_goal_dist = std::numeric_limits<double>::infinity();
    double lookahead = 0.0;
};

static bool preferClosedGoalBypassCandidate(
    const ClosedGoalCandidateProgress& lhs,
    const ClosedGoalCandidateProgress& rhs,
    bool bypass_mode)
{
    if (!bypass_mode) {
        return false;
    }
    if (lhs.valid != rhs.valid) {
        return lhs.valid;
    }
    if (!lhs.valid) {
        return false;
    }
    if (lhs.passed_obstacle != rhs.passed_obstacle) {
        return lhs.passed_obstacle;
    }

    constexpr double eps = 1e-6;
    if (std::abs(lhs.end_delta_w - rhs.end_delta_w) > eps) {
        return lhs.passed_obstacle ? lhs.end_delta_w < rhs.end_delta_w
                                   : lhs.end_delta_w > rhs.end_delta_w;
    }
    if (std::abs(lhs.end_to_goal_dist - rhs.end_to_goal_dist) > eps) {
        return lhs.end_to_goal_dist < rhs.end_to_goal_dist;
    }
    return lhs.lookahead < rhs.lookahead - eps;
}
```

- [ ] **Step 4: Rebuild and run the comparator tests**

Run:

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
./devel/lib/bspline_race/gvf_switch_policy_test
```

Expected: all `GvfClosedGoalBypassPolicy` tests pass, along with the existing switch and point-goal policy tests.

- [ ] **Step 5: Commit the comparator and tests**

```bash
git add src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
        src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
git commit -m "test: define closed-goal bypass candidate ordering"
```

### Task 2: Detect the complete obstacle interval and rank actual Kino endpoints

**Files:**
- Modify: `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- Modify: `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- Test: `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`

**Interfaces:**
- Consumes: `pointFromClosedW`, `projectClosedLocal`, the inflated occupancy map, Kino samples, and Task 1's comparator.
- Produces: obstacle start/end/bypass phase values, per-candidate actual endpoint progress, bypass selection, and extended `[GVF][CLOSED_GOAL]` diagnostics.

- [ ] **Step 1: Add a failing obstacle-interval test helper contract**

Add a public scalar helper declaration and tests for occupancy samples encoded as `-1=outside map`, `0=free`, and nonzero=`occupied`:

```cpp
TEST(GvfClosedGoalBypassPolicy, FindsObstacleEndAfterThreeFreeSamples)
{
  const std::vector<int> occupancy{0, 1, 1, 0, 0, 0, 0};
  const auto interval = FLAG_Race::gvf_manager::detectClosedGoalObstacleInterval(
      occupancy, 0.1, 3);
  ASSERT_TRUE(interval.found_start);
  ASSERT_TRUE(interval.found_end);
  EXPECT_NEAR(0.2, interval.start_delta_w, 1e-6);
  EXPECT_NEAR(0.4, interval.end_delta_w, 1e-6);
}

TEST(GvfClosedGoalBypassPolicy, OutsideMapDoesNotProveObstacleExit)
{
  const std::vector<int> occupancy{1, 0, -1, 0, -1};
  const auto interval = FLAG_Race::gvf_manager::detectClosedGoalObstacleInterval(
      occupancy, 0.1, 3);
  EXPECT_TRUE(interval.found_start);
  EXPECT_FALSE(interval.found_end);
}
```

Run:

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
```

Expected: compilation fails because the interval type and helper do not exist.

- [ ] **Step 2: Implement the obstacle-interval scalar helper**

Add:

```cpp
struct ClosedGoalObstacleInterval
{
    bool found_start = false;
    bool found_end = false;
    double start_delta_w = std::numeric_limits<double>::infinity();
    double end_delta_w = std::numeric_limits<double>::infinity();
};

static ClosedGoalObstacleInterval detectClosedGoalObstacleInterval(
    const std::vector<int>& occupancy,
    double step_w,
    int required_free_samples = 3)
{
    ClosedGoalObstacleInterval result;
    if (!std::isfinite(step_w) || step_w <= 0.0 || required_free_samples <= 0) {
        return result;
    }

    int free_count = 0;
    double free_run_start = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < occupancy.size(); ++i) {
        const double delta_w = (static_cast<double>(i) + 1.0) * step_w;
        if (occupancy[i] < 0) {
            continue;
        }
        if (!result.found_start) {
            if (occupancy[i] != 0) {
                result.found_start = true;
                result.start_delta_w = delta_w;
            }
            continue;
        }
        if (occupancy[i] != 0) {
            free_count = 0;
            free_run_start = std::numeric_limits<double>::infinity();
            continue;
        }
        if (free_count == 0) {
            free_run_start = delta_w;
        }
        ++free_count;
        if (free_count >= required_free_samples) {
            result.found_end = true;
            result.end_delta_w = free_run_start;
            break;
        }
    }
    return result;
}
```

Rebuild and run the test. Expected: both obstacle interval tests pass.

- [ ] **Step 3: Replace first-hit-only scanning with complete interval detection**

In `selectClosedGoalCandidate()`, collect one occupancy entry for each forward reference sample up to `candidates.back()`. Use `-1` for samples outside the map and call `detectClosedGoalObstacleInterval(occupancy, check_step, 3)`.

Set:

```cpp
first_obstacle_delta_w = interval.start_delta_w;
obstacle_end_delta_w = interval.end_delta_w;
if (interval.found_end) {
    bypass_delta_w = std::min(max_check_w,
        obstacle_end_delta_w + std::max(0.0, closed_goal_obstacle_pass_margin_w_));
    desired_lookahead = closedGoalObstaclePushedLookahead(
        desired_lookahead, obstacle_end_delta_w,
        candidates.front(), candidates.back(),
        closed_goal_obstacle_pass_margin_w_);
}
const bool bypass_mode = interval.found_end;
```

Do not push the desired lookahead when only an obstacle start is visible.

- [ ] **Step 4: Store and project every valid Kino result**

Inside `selectClosedGoalCandidate()`, store each valid result in a local structure containing the existing index, lookahead, goal, goal phase, samples, full-success flag, and full-success score plus:

```cpp
const double projected_end_w = projectClosedLocal(
    candidate_samples.point_set.back(), closed_ref_w_, 0.0, candidates.back());
const double end_delta_w = std::max(0.0, projected_end_w - closed_ref_w_);
const bool passed_obstacle = bypass_mode &&
    end_delta_w >= bypass_delta_w - std::max(1e-3, closed_goal_obstacle_check_step_w_);
```

Populate `ClosedGoalCandidateProgress` from those values.

- [ ] **Step 5: Select by bypass progress only when bypass mode is active**

When `bypass_mode` is true and at least one valid candidate exists, reduce the candidate list using `preferClosedGoalBypassCandidate`. The selected candidate may be full or partial. Assign all existing output fields from that candidate and use:

```cpp
selected_reason = selected.passed_obstacle
    ? "bypass_passed_actual_end"
    : "bypass_farthest_partial_progress";
```

When `bypass_mode` is false, preserve the existing policy exactly: choose the lowest-score full success; if none exists, choose the partial with the smallest `end_to_goal_dist`.

- [ ] **Step 6: Extend diagnostics without changing launch parameters**

Add to `[GVF][CLOSED_GOAL]`:

```text
obstacle_end_delta_w
bypass_delta_w
bypass_mode
selected_end_delta_w
selected_passed_obstacle
tried_end_delta_ws
```

Use `-1.0` for unavailable scalar values. Keep all existing log fields so old bag-analysis scripts remain usable.

- [ ] **Step 7: Run the focused and package tests**

Run:

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
./devel/lib/bspline_race/gvf_switch_policy_test
catkin_make --pkg bspline_race --make-args run_tests_bspline_race
catkin_test_results build/test_results/bspline_race
```

Expected: compilation succeeds; all existing and new tests pass with zero errors and failures.

- [ ] **Step 8: Inspect scope and commit implementation**

Run:

```bash
git diff --check
git diff -- src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
             src/swarm_planner/bspline_traj/src/gvf_manager.cpp \
             src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
```

Verify there are no changes to collision checking, replanning triggers, governor code, `shouldAcceptCandidate()`, or launch parameters. Then commit:

```bash
git add src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
        src/swarm_planner/bspline_traj/src/gvf_manager.cpp \
        src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
git commit -m "fix: rank closed-goal bypass paths by actual progress"
```

### Task 3: Final regression verification and handoff

**Files:**
- Verify only; no source modification is expected.

**Interfaces:**
- Consumes: committed Task 1 and Task 2 implementation.
- Produces: fresh build/test evidence and a concise simulation checklist.

- [ ] **Step 1: Run a clean package verification**

```bash
catkin_make --pkg bspline_race
catkin_make --pkg bspline_race --make-args run_tests_bspline_race
catkin_test_results build/test_results/bspline_race
```

Expected: build succeeds and test results report zero errors and zero failures.

- [ ] **Step 2: Confirm the worktree scope**

```bash
git status --short
git log -3 --oneline
```

Expected: only the user's pre-existing `test_gvf.launch`, `.codex`, and `paper/` changes remain outside the implementation commits; no unrelated file is added to either commit.

- [ ] **Step 3: Report the flight-relevant behavior**

Document that simulation should check `bypass_mode=1`, `selected_end_delta_w`, and `selected_passed_obstacle` in `[GVF][CLOSED_GOAL]`, and explicitly state that no collision, replan, governor, switch-policy, or launch-parameter behavior changed.
