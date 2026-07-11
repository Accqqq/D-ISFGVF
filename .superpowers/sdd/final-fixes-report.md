# Progressive Closed-Goal Final Fixes Report

Date: 2026-07-12

## Scope

- Production changes are limited to `gvf_manager.h` and `gvf_manager.cpp`.
- Regression coverage is limited to `gvf_switch_policy_test.cpp`.
- No launch, point-goal, collision, FSM, governor, or trajectory-switch behavior was changed.

## RED

1. Exact progress boundary
   - Added `UsesExactRequiredProgressBoundary` before production code.
   - `catkin_make --pkg bspline_race --make-args gvf_switch_policy_test -j2` failed because `closedGoalProgressSufficient` did not exist.
   - The test distinguishes exact equality from a value only `5e-7` below the requirement.

2. Strict candidate-attempt ordering
   - Added `MaintainsStrictOrderAcrossCloseDesiredErrors` and `ProducesSameOrderAcrossInputPermutations` before production code.
   - The focused target failed because `closedGoalAttemptComesBefore` did not exist.
   - The cases use three desired-lookahead errors separated by `0.75e-6`, exposing the old epsilon comparator's non-strict ordering, and exercise all six input permutations.

3. Candidate-window diagnostic reason
   - Added `DescribesTrackAndRecoverCandidateWindows` before production code.
   - The focused target failed because `closedGoalCandidateOrderReason` did not exist.

## GREEN

- Added shared `closedGoalProgressSufficient(end_delta_w, required_progress)` with exact `>=` semantics and finite-input validation.
- Progressive candidate comparison and selected-candidate reason/diagnostic now call the same progress helper.
- Added pure strict lexicographic attempt comparator using `(desired error, lookahead, idx)` and replaced the epsilon-based `std::sort` lambda.
- Candidate-order reasons now report `track_progressive_window` in TRACK and `recover_full_window` in RECOVER.
- Both `[GVF][CLOSED_GOAL]` log forms now emit `considered_candidate_count`, derived from `order.size()`, in the same field position.

Verification:

- `devel/lib/bspline_race/gvf_switch_policy_test`: 40/40 passed.
- `catkin_make --pkg bspline_race -j2`: completed successfully.
- Scoped `git diff --check` for the three implementation/test files: clean.

## Commit

- Intended commit message: `fix: align progressive closed-goal diagnostics`
- Intended committed scope: the three specified implementation/test files plus this required report.

## Concerns

- The package build retains two pre-existing warnings in `gvf_manager.cpp`: signed/unsigned comparison near line 1882 and an unhandled `INIT` enum warning near line 3882. Neither is in this fix scope.
- The worktree already contained an unrelated modified `launch/test_gvf.launch` with trailing whitespace and untracked `.codex` and `paper/` paths. They were left untouched and excluded from the commit.
