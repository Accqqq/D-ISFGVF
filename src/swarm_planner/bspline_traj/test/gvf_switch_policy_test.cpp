#include <gtest/gtest.h>

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
