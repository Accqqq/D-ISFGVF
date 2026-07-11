# Point-to-Point Short-Trajectory Exhaustion Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep normal point-to-point replanning active until the vehicle is within the final `0.2 m` completion tolerance so the accepted trajectory cannot silently exhaust inside the legacy `2.0 m` goal neighborhood.

**Architecture:** Add one pure static goal-completion policy helper to `gvf_manager`, cover its boundary and mode semantics in the existing switch-policy gtest, and use it to replace the nested `goal_reach_radius_` early return in `EXEC_TRAJ`. All existing collision, switch, governor, B-spline, closed-reference, ROS, and launch behavior remains unchanged.

**Tech Stack:** ROS Noetic, catkin, C++14, GoogleTest.

## Global Constraints

- Only point-to-point near-goal FSM behavior may change.
- Point-to-point completion remains a strict `dist_xy < 0.2 m` condition.
- Point-to-point execution at `dist_xy >= 0.2 m` must fall through to the existing collision and `planInterval` checks.
- Closed-reference/circle/figure-eight behavior must remain unchanged.
- Do not change `checkCollision()`, collision parameters, `accept_collision&timout`, `shouldAcceptCandidate()`, switch scoring, or anchor/reparameterization behavior.
- Do not change `GOVERNOR_INVALID_HOLD`, governor candidate generation, lead limits, command motion limits, B-spline parameterization, optimizer, timing, ROS topics, or launch parameters.
- Preserve the user's existing uncommitted `src/swarm_planner/bspline_traj/launch/test_gvf.launch` changes and untracked `.codex/` and `paper/` content.

---

## File Structure

- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`: define the pure point-goal completion policy helper alongside the existing public static replan-policy helpers.
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`: use the policy helper in the active point-to-point `EXEC_TRAJ` branch and remove the `0.2 m ~ goal_reach_radius_` replan suppression.
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`: cover point-to-point threshold, closed-reference, and invalid-input semantics.

### Task 1: Keep Point-to-Point Replanning Active Until Final Completion

**Files:**
- Modify: `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- Modify: `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- Modify: `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`

**Interfaces:**
- Consumes: `circle_mode_active`, current XY goal distance, and the final completion tolerance.
- Produces: `gvf_manager::shouldDeclarePointGoalReached(bool, double, double) -> bool`.

- [ ] **Step 1: Add failing point-goal policy tests**

Add `#include <limits>` with the standard includes in `gvf_switch_policy_test.cpp`, then append these tests before `main()`:

```cpp
TEST(GvfPointGoalPolicy, ContinuesReplanningInsideLegacyReachRadius)
{
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldDeclarePointGoalReached(
      false, 1.0, 0.2));
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldDeclarePointGoalReached(
      false, 0.2, 0.2));
}

TEST(GvfPointGoalPolicy, DeclaresCompletionOnlyBelowFinalTolerance)
{
  EXPECT_TRUE(FLAG_Race::gvf_manager::shouldDeclarePointGoalReached(
      false, 0.19, 0.2));
}

TEST(GvfPointGoalPolicy, ClosedReferenceNeverDeclaresPointGoalCompletion)
{
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldDeclarePointGoalReached(
      true, 0.05, 0.2));
}

TEST(GvfPointGoalPolicy, RejectsInvalidDistancesAndTolerance)
{
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldDeclarePointGoalReached(
      false, -0.1, 0.2));
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldDeclarePointGoalReached(
      false, std::numeric_limits<double>::infinity(), 0.2));
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldDeclarePointGoalReached(
      false, 0.1, 0.0));
}
```

- [ ] **Step 2: Run the focused test target and verify RED**

Run:

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
```

Expected: compilation fails because `gvf_manager::shouldDeclarePointGoalReached` does not exist. The existing switch and closed-goal tests must still compile up to that missing-symbol error.

- [ ] **Step 3: Add the minimal pure policy helper**

Add beside `shouldForceAcceptForGovernorPathShort()` in the public static replan-helper section of `gvf_manager.h`:

```cpp
static bool shouldDeclarePointGoalReached(bool circle_mode_active,
                                          double dist_xy,
                                          double final_tolerance = 0.2)
{
    if (circle_mode_active || !std::isfinite(dist_xy) ||
        !std::isfinite(final_tolerance) || dist_xy < 0.0 ||
        final_tolerance <= 0.0) {
        return false;
    }
    return dist_xy < final_tolerance;
}
```

- [ ] **Step 4: Run the focused test and verify the policy is GREEN**

Run:

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
devel/lib/bspline_race/gvf_switch_policy_test
```

Expected: all eight tests in `gvf_switch_policy_test` pass: the four existing tests and the four new point-goal policy tests.

- [ ] **Step 5: Replace the active FSM early-return block**

In the `EXEC_TRAJ` case of `gvf_manager::FSMCallback()`, replace:

```cpp
const bool circle_mode_active = enable_circle_reference_test_ && circle_reference_ready_;
if (!circle_mode_active) {
    const double dist_xy = (pm.goal_pt.head<2>() - current_pos.head<2>()).norm();
    if (dist_xy < goal_reach_radius_) {
        if(dist_xy < 0.2){
            changeFSMExecState(WAIT_TARGET, "reach_goal");
            pm.receive_goal = false;
            pm.is_first_goal = false;
        }
        return;
    }
}
```

with:

```cpp
const bool circle_mode_active = enable_circle_reference_test_ && circle_reference_ready_;
const double dist_xy = (pm.goal_pt.head<2>() - current_pos.head<2>()).norm();
if (shouldDeclarePointGoalReached(circle_mode_active, dist_xy, 0.2)) {
    changeFSMExecState(WAIT_TARGET, "reach_goal");
    pm.receive_goal = false;
    pm.is_first_goal = false;
    return;
}
```

Leave the immediately following `checkCollision()` and `planInterval` branches byte-for-byte unchanged.

- [ ] **Step 6: Run focused and package regression verification**

Run:

```bash
catkin_make --pkg bspline_race
catkin_make run_tests_bspline_race
catkin_test_results build
```

Expected:

- `bspline_race` builds successfully.
- `gvf_switch_policy_test` reports eight passing tests.
- Existing Fast-Planner parameterization, reference join, lifted visualization, and switch-policy tests remain green.
- `catkin_test_results build` reports zero errors and zero failures.

- [ ] **Step 7: Verify scope and commit**

Run:

```bash
git diff --check
git diff -- src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp \
  src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
git status --short
```

Confirm:

- no `checkCollision()`, collision parameter, switch-policy, governor, B-spline, launch, or closed-reference change is present;
- the user's pre-existing `test_gvf.launch` modification remains unstaged and unchanged by this task.

Commit only the three task files:

```bash
git add src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp \
  src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
git commit -m "fix: keep replanning until point goal reached"
```

