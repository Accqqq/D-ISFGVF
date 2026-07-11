#include <gtest/gtest.h>

#include <path_searching/kinodynamic_astar.h>

TEST(KinodynamicSamples, NoShotUsesTerminalNodeVelocity) {
  FLAG_Race::PathNode start;
  FLAG_Race::PathNode terminal;

  start.parent = nullptr;
  start.state.setZero();
  start.state.tail<3>() << 0.25, 0.0, 0.0;

  terminal.parent = &start;
  terminal.duration = 1.0;
  terminal.input.setZero();
  terminal.state.setZero();
  terminal.state.head<3>() << 0.5, 0.0, 1.0;
  terminal.state.tail<3>() << 0.75, 0.0, 0.0;

  FLAG_Race::KinodynamicAstar astar;
  astar.phi_.setIdentity();
  astar.is_shot_succ_ = false;
  astar.start_vel_ = start.state.tail<3>();
  astar.path_nodes_ = {&start, &terminal};

  double ts = 0.2;
  std::vector<Eigen::Vector3d> point_set;
  std::vector<Eigen::Vector3d> derivatives;
  astar.getSamples(ts, point_set, derivatives);

  ASSERT_EQ(derivatives.size(), 4u);
  EXPECT_NEAR((derivatives[1] - terminal.state.tail<3>()).norm(), 0.0, 1e-12);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
