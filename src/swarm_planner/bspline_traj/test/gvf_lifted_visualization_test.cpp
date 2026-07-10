#include <gtest/gtest.h>

#include <bspline_race/gvf.h>

namespace {

nav_msgs::Path::ConstPtr makeCrossingPath()
{
  nav_msgs::Path::Ptr path(new nav_msgs::Path);
  path->header.frame_id = "world";

  const std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(-1.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 1.0, 1.0),
  };

  for (const auto& p : points) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.w = 1.0;
    path->poses.push_back(pose);
  }

  return path;
}

void initLiftedGvfForTest(FLAG_Race::gvf& field)
{
  field.gvf_.K1_ = 2.0;
  field.gvf_.K2_ = -2.2;
  field.gvf_.convergence_bandwidth_ = 0.1;
  field.progress_window_ = 0.6;
  field.progress_rho0_ = 0.5;
  field.progress_delta_ = 0.3;
  field.alpha_min_ = 0.05;
  field.buildReparamTableFromPathMsg(makeCrossingPath());
}

}  // namespace

TEST(GvfLiftedVisualization, UsesStoredLiftedPhaseForBranchSelection)
{
  FLAG_Race::gvf field;
  initLiftedGvfForTest(field);

  Eigen::Vector3d vec;

  field.setVisualizationProgressW(1.0);
  ASSERT_TRUE(field.calcLiftedVisualizationVector(Eigen::Vector3d(0.0, 0.0, 1.0), vec));
  EXPECT_GT(vec.x(), 0.5);
  EXPECT_NEAR(vec.y(), 0.0, 1e-6);

  field.setVisualizationProgressW(3.0);
  ASSERT_TRUE(field.calcLiftedVisualizationVector(Eigen::Vector3d(0.0, 0.0, 1.0), vec));
  EXPECT_LT(vec.x(), -0.5);
  EXPECT_GT(vec.y(), 0.5);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
