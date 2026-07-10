#include <gtest/gtest.h>

#include <bspline_race/gvf_manager.h>

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
