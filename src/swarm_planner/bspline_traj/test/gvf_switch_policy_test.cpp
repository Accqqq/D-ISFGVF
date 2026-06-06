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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
