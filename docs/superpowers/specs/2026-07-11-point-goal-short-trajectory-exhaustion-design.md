# Point-to-Point Short-Trajectory Exhaustion Fix Design

## Objective

Fix the point-to-point navigation case where replanning stops too early near the goal, the accepted trajectory is consumed to its end, and the governor repeatedly enters `GOVERNOR_INVALID_HOLD` with `all_candidates_path_end_clamped`.

This change is intentionally limited to point-to-point near-goal FSM behavior. It must not change collision detection, collision-triggered replanning, candidate collision handling, trajectory switch scoring, closed-reference tracking, B-spline parameterization, or governor candidate generation.

## Observed Failure

The active point-to-point `EXEC_TRAJ` branch currently uses two nested distance thresholds:

```cpp
if (dist_xy < goal_reach_radius_) {
  if (dist_xy < 0.2) {
    // declare goal reached
  }
  return;
}
```

`goal_reach_radius_` defaults to `2.0 m`. Therefore the FSM returns early for every point-to-point distance in the range:

```text
0.2 m <= dist_xy < 2.0 m
```

The early return skips both `checkCollision()` and the normal `planInterval` transition to `REPLAN_TRAJ`. The command timer continues consuming the last accepted path. Once lifted progress reaches the end of that path, even the zero-lookahead governor candidate queries past `path_w_end`, so all candidates are rejected as end-clamped:

```text
candidate_count=36
valid_count=0
path_end_clamped_count=36
fallback_reason=all_candidates_path_end_clamped
```

The failure is not caused by Fast-Planner B-spline parameterization. The affected logs show valid finite `kino_points`, `control_points`, `kino_ts`, `final_interval`, and feasibility ratio values.

## Approved Scope

### Included

- Change point-to-point goal-completion policy so only the final `0.2 m` tolerance declares the goal reached and stops replanning.
- Continue normal collision and `planInterval` evaluation while `dist_xy >= 0.2 m`, including the current `0.2 m ~ 2.0 m` range.
- Preserve the existing `cmdCallback()` goal-position override inside `stop_radius` (currently `0.3 m`).
- Add a focused pure-policy unit test covering point-to-point and closed-reference semantics.
- Keep `goal_reach_radius_` loaded for configuration compatibility, but stop using it as a no-replan radius in the active point-to-point FSM.

### Explicitly Excluded

- No change to `checkCollision()`, `collision_threshold_`, collision horizon, or consecutive-hit policy.
- No change to `accept_collision&timout`, `shouldAcceptCandidate()`, switch scoring, or anchor/reparameterization behavior.
- No candidate collision gate and no rejection of `NEAR_END`, `REACH_HORIZON`, or partial Kino paths.
- No change to `GOVERNOR_INVALID_HOLD`, endpoint-clamped candidate handling, governor lead limits, or command motion limits.
- No change to closed-reference/circle/figure-eight behavior.
- No change to B-spline parameterization, optimizer, timing, GVF guidance, ROS topics, or launch parameters.

## Selected Design

### Goal-Completion Policy Helper

Add a public static pure helper to `gvf_manager` so the FSM policy is testable without constructing ROS timers, maps, or planners:

```cpp
static bool shouldDeclarePointGoalReached(bool circle_mode_active,
                                          double dist_xy,
                                          double final_tolerance = 0.2);
```

The helper returns `true` only when all conditions hold:

- closed-reference mode is inactive;
- `dist_xy` is finite and non-negative;
- `final_tolerance` is finite and positive;
- `dist_xy < final_tolerance`.

Closed-reference mode always returns `false` because a closed trajectory has no terminal point at which the point-to-point FSM should enter `WAIT_TARGET`.

### FSM Integration

Replace the current nested `goal_reach_radius_` block with:

```cpp
const double dist_xy = (pm.goal_pt.head<2>() - current_pos.head<2>()).norm();
if (shouldDeclarePointGoalReached(circle_mode_active, dist_xy, 0.2)) {
  changeFSMExecState(WAIT_TARGET, "reach_goal");
  pm.receive_goal = false;
  pm.is_first_goal = false;
  return;
}
```

When the point-to-point distance is `0.2 m` or greater, execution falls through to the existing code unchanged:

```text
checkCollision()
  -> collision-triggered REPLAN_TRAJ, if applicable
else planInterval elapsed
  -> time-triggered REPLAN_TRAJ
```

No new replan trigger is introduced. The change only removes the old suppression of existing triggers.

### Final Approach Behavior

The command layer already contains:

```cpp
force_goal_position = !circle_mode_active && real_dis_to_goal < stop_radius;
```

With the current `stop_radius=0.3 m`, the intended final approach becomes:

```text
dist_xy >= 0.3 m
  normal GVF/governor command and normal replanning

0.2 m <= dist_xy < 0.3 m
  command position is overridden to the exact goal
  FSM still permits collision/planInterval replanning

dist_xy < 0.2 m
  declare point goal reached
  enter WAIT_TARGET and stop publishing the active goal
```

This retains the existing command behavior while preventing the trajectory from being silently consumed for as much as `1.8 m` without replenishment.

## Alternatives Considered

### Use the Trajectory Endpoint When All Candidates Are Clamped

The governor could publish the path endpoint instead of entering invalid hold. This masks path exhaustion rather than fixing why replanning stopped. It is also unsafe for partial paths whose endpoint is not the requested goal.

### Dynamically Reduce Governor Lookahead Near the End

Reducing `governor_l_max` can delay end clamping, but once `progress_w` exceeds `path_w_end`, even a zero-lookahead candidate remains clamped. The underlying FSM suppression would remain.

### Reject Short Candidate Trajectories During Switching

Requiring a minimum new-path length would modify switch policy and could leave the system on an equally short old path. It is outside the approved scope and does not address the near-goal early return.

## Testing Strategy

Extend `gvf_switch_policy_test.cpp` with focused policy tests:

1. Point-to-point at `1.0 m` does not declare completion, proving the old `goal_reach_radius_=2.0 m` suppression is gone.
2. Point-to-point below `0.2 m` declares completion.
3. Closed-reference mode never declares point-goal completion, even below `0.2 m`.
4. Non-finite or invalid inputs do not declare completion.

Build and regression verification:

```bash
catkin_make --pkg bspline_race
catkin_make run_tests_bspline_race
catkin_test_results build
```

Runtime verification with point-to-point navigation:

- Enter the `2.0 m` goal neighborhood and confirm `[GVF][REPLAN_REASON] reason=plan_interval` continues while distance is above `0.2 m`.
- Confirm new trajectories continue to be accepted/rejected by the existing switch policy.
- Confirm `fallback_reason=all_candidates_path_end_clamped` does not appear solely because the vehicle entered the old `goal_reach_radius_` range.
- Confirm the existing goal-position override acts below `stop_radius` and the FSM transitions to `WAIT_TARGET` only below `0.2 m`.

## Success Criteria

- Point-to-point replanning is no longer suppressed in the `0.2 m ~ 2.0 m` goal-distance range.
- Point-to-point goal completion remains at the existing `0.2 m` final tolerance.
- Closed-reference behavior is unchanged.
- Collision detection and trajectory switch policy are unchanged.
- The existing governor and B-spline implementations are unchanged.
- New focused tests and existing `bspline_race` tests pass.

