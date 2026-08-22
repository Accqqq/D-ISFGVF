#include <gtest/gtest.h>

#include <cmath>

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

nav_msgs::Path::ConstPtr makeScaledPhasePath()
{
  nav_msgs::Path::Ptr path(new nav_msgs::Path);
  path->header.frame_id = "world";
  for (double x : {0.0, 2.0, 4.0}) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 1.0;
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

TEST(GvfLiftedPhase, PreservesExplicitGlobalPhaseSamples)
{
  FLAG_Race::gvf field;
  field.setNextPathWSamples({10.0, 11.0, 12.0});
  field.buildReparamTableFromPathMsg(makeScaledPhasePath());

  ASSERT_TRUE(field.reparam_ready_);
  ASSERT_EQ(3u, field.sample_w_.size());
  EXPECT_DOUBLE_EQ(10.0, field.sample_w_[0]);
  EXPECT_DOUBLE_EQ(11.0, field.sample_w_[1]);
  EXPECT_DOUBLE_EQ(12.0, field.sample_w_[2]);
}

TEST(GvfLiftedVisualization, ProjectsEachGridPointToItsOwnLocalPhase)
{
  FLAG_Race::gvf field;
  field.setNextPathWSamples({10.0, 11.0, 12.0});
  field.buildReparamTableFromPathMsg(makeScaledPhasePath());

  const double near_start = field.projectToPathLocalForVisualization(
      Eigen::Vector3d(0.2, 0.5, 1.0), 11.0, 2.0);
  const double near_end = field.projectToPathLocalForVisualization(
      Eigen::Vector3d(3.8, -0.5, 1.0), 11.0, 2.0);

  EXPECT_NEAR(10.1, near_start, 1e-9);
  EXPECT_NEAR(11.9, near_end, 1e-9);
  EXPECT_LT(near_start, near_end);
}

TEST(GvfLiftedVisualization, TerminalFieldConvergesDirectlyToPointGoal)
{
  FLAG_Race::gvf field;
  field.gvf_.K1_ = 2.0;
  field.gvf_.convergence_bandwidth_ = 0.2;
  field.setTerminalGoalVisualization(Eigen::Vector3d(1.0, 2.0, 1.0));

  Eigen::Vector3d from_right;
  Eigen::Vector3d from_below;
  ASSERT_TRUE(field.calcLiftedVisualizationVector(
      Eigen::Vector3d(3.0, 2.0, 1.0), from_right));
  ASSERT_TRUE(field.calcLiftedVisualizationVector(
      Eigen::Vector3d(1.0, 0.0, 1.0), from_below));

  EXPECT_LT(from_right.x(), 0.0);
  EXPECT_NEAR(0.0, from_right.y(), 1e-12);
  EXPECT_GT(from_below.y(), 0.0);
  EXPECT_NEAR(0.0, from_below.x(), 1e-12);

  field.clearTerminalGoalVisualization();
  Eigen::Vector3d cleared;
  EXPECT_FALSE(field.calcLiftedVisualizationVector(
      Eigen::Vector3d(3.0, 2.0, 1.0), cleared));
}

TEST(GvfLiftedPhase, DirectEvaluationDoesNotProjectAwayFromAuthoritativePhase)
{
  FLAG_Race::gvf field;
  field.gvf_.K1_ = 2.0;
  field.gvf_.K2_ = -2.2;
  field.gvf_.convergence_bandwidth_ = 0.1;
  field.progress_rho0_ = 0.5;
  field.progress_delta_ = 0.3;
  field.alpha_min_ = 0.05;
  field.setNextPathWSamples({10.0, 11.0, 12.0});
  field.buildReparamTableFromPathMsg(makeScaledPhasePath());

  const auto at_start = field.calcLiftedGuidanceAtPhase(
      Eigen::Vector3d(4.0, 0.0, 1.0), 10.0);
  ASSERT_TRUE(at_start.valid);
  EXPECT_DOUBLE_EQ(10.0, at_start.w_proj);
  EXPECT_NEAR(0.0, at_start.ref_pt.x(), 1e-9);

  const auto on_path = field.calcLiftedGuidanceAtPhase(
      Eigen::Vector3d(1.0, 0.0, 1.0), 10.5);
  ASSERT_TRUE(on_path.valid);
  EXPECT_NEAR(1.0, on_path.ref_pt.x(), 1e-9);
  EXPECT_NEAR(1.0, on_path.w_dot, 1e-6);
}

TEST(GvfLiftedPhase, AuthoritativeGuidanceKeepsHardCodedRepresentativeOutput)
{
  FLAG_Race::gvf field;
  field.gvf_.K1_ = 2.0;
  field.gvf_.K2_ = -2.2;
  field.gvf_.convergence_bandwidth_ = 0.1;
  field.progress_rho0_ = 0.5;
  field.progress_delta_ = 0.3;
  field.alpha_min_ = 0.05;
  field.setNextPathWSamples({10.0, 11.0, 12.0});
  field.buildReparamTableFromPathMsg(makeScaledPhasePath());

  const auto guidance = field.calcLiftedGuidanceAtPhase(
      Eigen::Vector3d(1.0, 0.2, 1.0), 10.5);
  ASSERT_TRUE(guidance.valid);
  EXPECT_NEAR(1.7379310344827585, guidance.v_cmd.x(), 1e-12);
  EXPECT_NEAR(-2.1208606761667970, guidance.v_cmd.y(), 1e-12);
  EXPECT_NEAR(0.0, guidance.v_cmd.z(), 1e-12);
  EXPECT_NEAR(0.8689655172413793, guidance.w_dot, 1e-12);
  EXPECT_NEAR(0.0, guidance.e_parallel, 1e-12);
  EXPECT_NEAR(0.2, guidance.e_perp.y(), 1e-12);
  EXPECT_NEAR(1.0, guidance.ref_pt.x(), 1e-12);
  EXPECT_NEAR(1.0, guidance.tangent.x(), 1e-12);
}

TEST(GvfLiftedVisualization, AuthoritativeSquareUsesExactDisplayedPhase)
{
  FLAG_Race::gvf field;
  field.gvf_.K1_ = 2.0;
  field.gvf_.K2_ = -2.2;
  field.gvf_.convergence_bandwidth_ = 0.1;
  field.progress_rho0_ = 0.5;
  field.progress_delta_ = 0.3;
  field.alpha_min_ = 0.05;
  field.setNextPathWSamples({10.0, 11.0, 12.0});
  field.buildReparamTableFromPathMsg(makeScaledPhasePath());
  field.setAuthoritativePhaseMode(true);

  const double phase_w = 10.35;
  const Eigen::Vector3d sample_pos(0.7, 1.2, 1.0);
  field.setVisualizationProgressW(phase_w);

  Eigen::Vector3d displayed;
  ASSERT_TRUE(field.calcLiftedVisualizationVector(sample_pos, displayed));
  const auto expected = field.calcLiftedGuidanceAtPhase(sample_pos, phase_w);
  ASSERT_TRUE(expected.valid);
  EXPECT_NEAR(0.0, (displayed - expected.v_cmd).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(phase_w, expected.w_proj);
}

TEST(GvfLiftedPhase, AuthoritativeModeEvaluatesContinuousPathNotDisplaySamples)
{
  FLAG_Race::gvf field;
  field.gvf_.K1_ = 2.0;
  field.gvf_.K2_ = -2.2;
  field.gvf_.convergence_bandwidth_ = 0.1;
  field.progress_rho0_ = 0.5;
  field.progress_delta_ = 0.3;
  field.alpha_min_ = 0.05;
  field.setNextPathWSamples({10.0, 11.0, 12.0});
  field.buildReparamTableFromPathMsg(makeScaledPhasePath());

  auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  const auto circle = FLAG_Race::ContinuousPhasePath::makePeriodicCircle(
      Eigen::Vector3d(0.0, 0.0, 1.0), 2.0, 4.0 * M_PI, 1.0);
  ASSERT_TRUE(path->appendSegment(10.0, 12.0, "circle", circle));
  field.setContinuousPhasePath(path);
  field.setAuthoritativePhaseMode(true);

  const double w = 10.5;
  FLAG_Race::ContinuousPhasePathState expected;
  ASSERT_TRUE(path->evaluate(w, expected, false));
  EXPECT_NEAR(0.0, (field.evalPathByW(w) - expected.p).norm(), 1e-12);
  EXPECT_NEAR(0.0, (field.evalDpDwByW(w) - expected.dp_dw).norm(), 1e-12);
  EXPECT_NEAR(0.0, (field.evalD2pDw2ByW(w) - expected.d2p_dw2).norm(), 1e-12);

  const auto guidance = field.calcLiftedGuidanceAtPhase(expected.p, w);
  ASSERT_TRUE(guidance.valid);
  EXPECT_NEAR(0.0, (guidance.ref_pt - expected.p).norm(), 1e-12);
  EXPECT_NEAR(0.0, guidance.e_perp.norm(), 1e-12);
}

TEST(GvfLiftedPhase, AuthoritativeModeUsesFullThreeDimensionalContinuousTangent)
{
  FLAG_Race::gvf field;
  field.gvf_.K1_ = 2.0;
  field.gvf_.K2_ = -2.2;
  field.gvf_.convergence_bandwidth_ = 0.1;
  field.progress_rho0_ = 0.5;
  field.progress_delta_ = 0.3;
  field.alpha_min_ = 0.05;
  field.setNextPathWSamples({10.0, 11.0, 12.0});
  field.buildReparamTableFromPathMsg(makeScaledPhasePath());

  auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  const auto height_varying = [](double w,
                                 FLAG_Race::ContinuousPhasePathState& state) {
    const double s = w - 10.0;
    state.p = Eigen::Vector3d(s, 0.5 * std::sin(s), 1.0 + 0.2 * s);
    state.dp_dw = Eigen::Vector3d(1.0, 0.5 * std::cos(s), 0.2);
    state.d2p_dw2 = Eigen::Vector3d(0.0, -0.5 * std::sin(s), 0.0);
    state.vel = state.dp_dw;
    state.valid = true;
    return true;
  };
  ASSERT_TRUE(path->appendSegment(10.0, 12.0, "height_varying", height_varying));
  field.setContinuousPhasePath(path);
  field.setAuthoritativePhaseMode(true);

  const double w = 10.7;
  FLAG_Race::ContinuousPhasePathState expected;
  ASSERT_TRUE(path->evaluate(w, expected, false));
  const auto guidance = field.calcLiftedGuidanceAtPhase(expected.p, w);
  ASSERT_TRUE(guidance.valid);
  EXPECT_NEAR(0.0, (guidance.ref_pt - expected.p).norm(), 1e-12);
  EXPECT_NEAR(0.0, (guidance.tangent -
                     expected.dp_dw / expected.dp_dw.norm()).norm(), 1e-12);
  EXPECT_GT(std::abs(guidance.tangent.z()), 1e-3);
  EXPECT_NEAR(0.0, guidance.e_perp.norm(), 1e-12);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
