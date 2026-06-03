#include <gtest/gtest.h>

#include <bspline_race/reference_join.h>

namespace {

Eigen::MatrixXd makeCircle(int n)
{
  Eigen::MatrixXd traj(n, 3);
  for (int i = 0; i < n; ++i) {
    const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
    traj.row(i) << std::cos(theta), std::sin(theta), 1.0;
  }
  return traj;
}

}  // namespace

TEST(ReferenceJoin, WrapsLookaheadOnClosedReference)
{
  const Eigen::MatrixXd traj = makeCircle(8);
  const Eigen::Vector3d curr_pos = traj.row(7).transpose();
  FLAG_Race::ReferenceJoinState state;

  const auto selection = FLAG_Race::selectReferenceJoinGoal(
      traj, curr_pos, 2, 3, 0.5, 2, false, state);

  EXPECT_EQ(selection.best_idx, 7);
  EXPECT_EQ(selection.goal_idx, 1);
  EXPECT_TRUE(state.active);
}

TEST(ReferenceJoin, ExitsAfterStableCloseSelections)
{
  const Eigen::MatrixXd traj = makeCircle(8);
  const Eigen::Vector3d curr_pos = traj.row(0).transpose();
  FLAG_Race::ReferenceJoinState state;

  auto selection = FLAG_Race::selectReferenceJoinGoal(
      traj, curr_pos, 1, 3, 0.5, 2, true, state);
  EXPECT_FALSE(selection.exited);
  EXPECT_TRUE(state.active);
  EXPECT_EQ(state.stable_count, 1);

  selection = FLAG_Race::selectReferenceJoinGoal(
      traj, curr_pos, 1, 3, 0.5, 2, true, state);
  EXPECT_TRUE(selection.exited);
  EXPECT_FALSE(state.active);
  EXPECT_EQ(state.stable_count, 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
