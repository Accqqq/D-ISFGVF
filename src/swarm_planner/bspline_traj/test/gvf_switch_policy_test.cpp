#include <gtest/gtest.h>

#include <initializer_list>
#include <limits>

#include <bspline_race/gvf_manager.h>

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

int selectProgressiveCandidateIndex(
    std::initializer_list<FLAG_Race::gvf_manager::ClosedGoalProgressiveCandidate> candidates,
    double required_progress, double desired_lookahead)
{
  FLAG_Race::gvf_manager::ClosedGoalProgressiveCandidate best;
  for (const auto& value : candidates) {
    if (FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
            value, best, required_progress, desired_lookahead)) {
      best = value;
    }
  }
  return best.idx;
}
}

TEST(GvfSwitchPolicy, ForcesAcceptWhenAcceptedPathCannotSupportGovernorLookahead)
{
  EXPECT_TRUE(FLAG_Race::gvf_manager::shouldForceAcceptForGovernorPathShort(
      28.95, 29.40, 1.6, 0.2));
}

TEST(GvfSwitchPolicy, KeepsScoreBasedSwitchWhenAcceptedPathHasEnoughGovernorLookahead)
{
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldForceAcceptForGovernorPathShort(
      27.40, 29.40, 1.6, 0.2));
}

TEST(GvfClosedGoalPolicy, PrefersDesiredLookaheadOverFarthestFullSuccess)
{
  const double near_desired_score =
      FLAG_Race::gvf_manager::closedGoalCandidateScore(1.5, 1.5, 0.05, 5.0, 20.0);
  const double far_score =
      FLAG_Race::gvf_manager::closedGoalCandidateScore(4.0, 1.5, 0.05, 5.0, 20.0);

  EXPECT_LT(near_desired_score, far_score);
}

TEST(GvfClosedGoalPolicy, PushesDesiredLookaheadPastDetectedObstacle)
{
  const double desired = FLAG_Race::gvf_manager::closedGoalObstaclePushedLookahead(
      1.5, 1.4, 1.2, 2.5, 0.8);

  EXPECT_NEAR(2.2, desired, 1e-6);
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

TEST(GvfClosedGoalProgressivePolicy, UsesExactDesiredError)
{
  const auto closer = progressiveCandidate(9, 1.0000005, 0.8, 1.0, 0.0);
  const auto farther = progressiveCandidate(1, 1.0000009, 0.8, 1.0, 0.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      closer, farther, 0.8, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, BetterCandidateIsNotPreferredInReverse)
{
  const auto closer = progressiveCandidate(9, 1.0000005, 0.8, 1.0, 0.0);
  const auto farther = progressiveCandidate(1, 1.0000009, 0.8, 1.0, 0.0);
  EXPECT_FALSE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      farther, closer, 0.8, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, MaintainsTransitivityAcrossThreeCloseErrors)
{
  const auto a = progressiveCandidate(0, 1.0000015, 0.8, 1.0, 0.0);
  const auto b = progressiveCandidate(1, 1.00000075, 0.8, 1.0, 0.0);
  const auto c = progressiveCandidate(2, 1.0, 0.8, 1.0, 0.0);
  const bool a_before_b =
      FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(a, b, 0.8, 1.0);
  const bool b_before_c =
      FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(b, c, 0.8, 1.0);
  const bool a_before_c =
      FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(a, c, 0.8, 1.0);
  EXPECT_FALSE(a_before_b && b_before_c && !a_before_c);
}

TEST(GvfClosedGoalProgressivePolicy, SelectsSameWinnerAcrossInputPermutations)
{
  const auto a = progressiveCandidate(0, 1.0000015, 0.8, 1.0, 0.0);
  const auto b = progressiveCandidate(1, 1.00000075, 0.8, 1.0, 0.0);
  const auto c = progressiveCandidate(2, 1.0, 0.8, 1.0, 0.0);
  EXPECT_EQ(2, selectProgressiveCandidateIndex({a, b, c}, 0.8, 1.0));
  EXPECT_EQ(2, selectProgressiveCandidateIndex({a, c, b}, 0.8, 1.0));
  EXPECT_EQ(2, selectProgressiveCandidateIndex({b, a, c}, 0.8, 1.0));
  EXPECT_EQ(2, selectProgressiveCandidateIndex({b, c, a}, 0.8, 1.0));
  EXPECT_EQ(2, selectProgressiveCandidateIndex({c, a, b}, 0.8, 1.0));
  EXPECT_EQ(2, selectProgressiveCandidateIndex({c, b, a}, 0.8, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, ValidCandidateBeatsInvalidCandidate)
{
  const auto valid = progressiveCandidate(2, 1.0, 0.8, 1.0, 0.0);
  auto invalid = progressiveCandidate(1, 1.0, 0.8, 1.0, 0.0);
  invalid.valid = false;
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      valid, invalid, 0.8, 1.0));
  EXPECT_FALSE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      invalid, valid, 0.8, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, UsesIndexForExactTie)
{
  const auto lower_idx = progressiveCandidate(1, 1.0, 0.8, 1.0, 0.0);
  const auto higher_idx = progressiveCandidate(2, 1.0, 0.8, 1.0, 0.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      lower_idx, higher_idx, 0.8, 1.0));
  EXPECT_FALSE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      higher_idx, lower_idx, 0.8, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, RejectsNonFiniteCandidateMetrics)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity_value = std::numeric_limits<double>::infinity();
  const auto good = progressiveCandidate(10, 1.0, 0.8, 1.0, 0.0);

  auto nonfinite_lookahead = progressiveCandidate(0, 1.0, 0.8, 1.0, 0.0);
  nonfinite_lookahead.lookahead = nan;
  auto nonfinite_progress = progressiveCandidate(0, 1.0, 0.8, 1.0, 0.0);
  nonfinite_progress.end_delta_w = infinity_value;
  auto nonfinite_length = progressiveCandidate(0, 1.0, 0.8, 1.0, 0.0);
  nonfinite_length.kino_path_length = nan;
  auto nonfinite_goal_error = progressiveCandidate(0, 1.0, 0.8, 1.0, 0.0);
  nonfinite_goal_error.end_to_goal_dist = nan;

  for (const auto& bad : {nonfinite_lookahead, nonfinite_progress,
                          nonfinite_length, nonfinite_goal_error}) {
    EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
        good, bad, 0.8, 1.0));
    EXPECT_FALSE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
        bad, good, 0.8, 1.0));
  }
}

TEST(GvfClosedGoalProgressivePolicy, TreatsNonFiniteRequiredProgressAsUnsatisfied)
{
  const double negative_infinity = -std::numeric_limits<double>::infinity();
  const auto farther = progressiveCandidate(2, 2.0, 0.9, 1.0, 0.0);
  const auto near_desired = progressiveCandidate(1, 1.0, 0.2, 1.0, 0.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      farther, near_desired, negative_infinity, 1.0));
}

TEST(GvfClosedGoalProgressivePolicy, UsesZeroDesiredLookaheadWhenDesiredIsNonFinite)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto near_zero = progressiveCandidate(2, 1.0, 0.8, 1.0, 0.0);
  const auto negative_far = progressiveCandidate(1, -2.0, 0.8, 1.0, 0.0);
  EXPECT_TRUE(FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
      near_zero, negative_far, 0.8, nan));
}

TEST(GvfClosedGoalPolicy, NormalAcceptedLookaheadRemainsPreferred)
{
  EXPECT_DOUBLE_EQ(3.0, FLAG_Race::gvf_manager::closedGoalDesiredLookahead(
      2.0, true, 3.0, false));
}

TEST(GvfClosedGoalPolicy, BypassAcceptedLookaheadRestoresConfiguredPreference)
{
  EXPECT_DOUBLE_EQ(2.0, FLAG_Race::gvf_manager::closedGoalDesiredLookahead(
      2.0, true, 3.0, true));
}

TEST(GvfClosedGoalPolicy, SelectsConfiguredDefaultLookaheadIndex)
{
  const std::vector<double> candidates{0.5, 1.0, 1.5};

  EXPECT_EQ(1, FLAG_Race::gvf_manager::selectDefaultClosedLookaheadIndex(
      candidates, 1.0));
}

TEST(GvfClosedGoalPolicy, ClampsDefaultLookaheadOutsideCandidateRange)
{
  const std::vector<double> candidates{0.5, 1.0, 1.5};

  EXPECT_EQ(0, FLAG_Race::gvf_manager::selectDefaultClosedLookaheadIndex(
      candidates, -1.0));
  EXPECT_EQ(2, FLAG_Race::gvf_manager::selectDefaultClosedLookaheadIndex(
      candidates, 3.0));
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

TEST(GvfClosedGoalBypassPolicy, RejectsNonFiniteActualCandidateMetrics)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity_value = std::numeric_limits<double>::infinity();

  EXPECT_TRUE(FLAG_Race::gvf_manager::isFiniteClosedGoalCandidate(
      1.0, 2.0, 3.0, 0.2, 1.1));
  EXPECT_FALSE(FLAG_Race::gvf_manager::isFiniteClosedGoalCandidate(
      infinity_value, 2.0, 3.0, 0.2, 1.1));
  EXPECT_FALSE(FLAG_Race::gvf_manager::isFiniteClosedGoalCandidate(
      1.0, 2.0, 3.0, nan, 1.1));
  EXPECT_FALSE(FLAG_Race::gvf_manager::isFiniteClosedGoalCandidate(
      1.0, 2.0, 3.0, 0.2, nan));
}

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

TEST(GvfClosedGoalBypassPolicy, OutsideMapPreservesAccumulatedFreeSamples)
{
  const std::vector<int> occupancy{1, 0, -1, 0, -1, 0};
  const auto interval = FLAG_Race::gvf_manager::detectClosedGoalObstacleInterval(
      occupancy, 0.1, 3);
  ASSERT_TRUE(interval.found_start);
  ASSERT_TRUE(interval.found_end);
  EXPECT_NEAR(0.2, interval.end_delta_w, 1e-6);
}

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
