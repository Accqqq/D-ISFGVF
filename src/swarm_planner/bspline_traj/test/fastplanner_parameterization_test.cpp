#include <gtest/gtest.h>

#include <bspline_race/UniformBspline_3d.h>
#include <bspline_race/bspline_opt_3d.h>

#include <cmath>
#include <vector>

namespace {

std::vector<Eigen::Vector3d> makeStraightSamples(int count, double ts,
                                                 const Eigen::Vector3d& velocity) {
  std::vector<Eigen::Vector3d> samples;
  samples.reserve(count);
  for (int i = 0; i < count; ++i) {
    samples.push_back(static_cast<double>(i) * ts * velocity);
  }
  return samples;
}

std::vector<Eigen::Vector3d> makeBoundaryDerivatives(
    const Eigen::Vector3d& velocity) {
  return {velocity, velocity, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}

TEST(FastPlannerParameterization, ProducesKPlusTwoControlPointsAndInterpolatesLine) {
  const int k = 9;
  const double ts = 0.2;
  const Eigen::Vector3d velocity(0.5, -0.25, 0.0);
  const auto samples = makeStraightSamples(k, ts, velocity);
  const auto derivatives = makeBoundaryDerivatives(velocity);

  Eigen::MatrixXd control_points;
  ASSERT_TRUE(FLAG_Race::UniformBspline::parameterizeToBspline(
      ts, samples, derivatives, control_points));
  ASSERT_EQ(control_points.rows(), k + 2);
  ASSERT_EQ(control_points.cols(), 3);

  FLAG_Race::UniformBspline spline;
  ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, ts));

  Eigen::VectorXd query(k);
  for (int i = 0; i < k; ++i) query(i) = static_cast<double>(i) * ts;
  const Eigen::MatrixXd reconstructed = spline.getTrajectory(query);

  for (int i = 0; i < k; ++i) {
    EXPECT_NEAR((reconstructed.row(i).transpose() - samples[i]).norm(), 0.0, 1e-8);
  }
  EXPECT_NEAR(spline.t_range(1), static_cast<double>(k - 1) * ts, 1e-12);
}

TEST(FastPlannerParameterization, PreservesBoundaryVelocityAndAcceleration) {
  const int k = 9;
  const double ts = 0.2;
  const Eigen::Vector3d velocity(0.4, 0.2, 0.0);
  const auto samples = makeStraightSamples(k, ts, velocity);
  const auto derivatives = makeBoundaryDerivatives(velocity);

  Eigen::MatrixXd control_points;
  ASSERT_TRUE(FLAG_Race::UniformBspline::parameterizeToBspline(
      ts, samples, derivatives, control_points));

  FLAG_Race::UniformBspline position;
  ASSERT_TRUE(position.setControlPointsAndInterval(control_points, 3, ts));
  auto velocity_spline = position.getDerivative();
  auto acceleration_spline = velocity_spline.getDerivative();

  Eigen::VectorXd boundary_times(2);
  boundary_times << 0.0, static_cast<double>(k - 1) * ts;
  const Eigen::MatrixXd velocities = velocity_spline.getTrajectory(boundary_times);
  const Eigen::MatrixXd accelerations = acceleration_spline.getTrajectory(boundary_times);

  EXPECT_NEAR((velocities.row(0).transpose() - velocity).norm(), 0.0, 1e-8);
  EXPECT_NEAR((velocities.row(1).transpose() - velocity).norm(), 0.0, 1e-8);
  EXPECT_NEAR(accelerations.row(0).norm(), 0.0, 1e-8);
  EXPECT_NEAR(accelerations.row(1).norm(), 0.0, 1e-8);
}

TEST(FastPlannerParameterization, UniformTimeScalingPreservesGeometry) {
  const int k = 9;
  const double ts = 0.1;
  const Eigen::Vector3d velocity(4.0, 0.0, 0.0);
  const auto samples = makeStraightSamples(k, ts, velocity);
  const auto derivatives = makeBoundaryDerivatives(velocity);

  Eigen::MatrixXd control_points;
  ASSERT_TRUE(FLAG_Race::UniformBspline::parameterizeToBspline(
      ts, samples, derivatives, control_points));

  FLAG_Race::UniformBspline spline;
  ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, ts));
  const double ratio = spline.getFeasibilityRatio(2.0, 3.5);
  ASSERT_GT(ratio, 1.0);

  Eigen::VectorXd before_times(3);
  before_times << 0.0, 0.4, 0.8;
  const Eigen::MatrixXd before = spline.getTrajectory(before_times);

  ASSERT_TRUE(spline.scaleTime(ratio * 1.01));
  Eigen::VectorXd after_times = before_times * (ratio * 1.01);
  const Eigen::MatrixXd after = spline.getTrajectory(after_times);

  EXPECT_NEAR((before - after).norm(), 0.0, 1e-8);
  EXPECT_LE(spline.getFeasibilityRatio(2.0, 3.5), 1.0 + 1e-8);
}

TEST(FastPlannerParameterization, RejectsInvalidInputs) {
  Eigen::MatrixXd control_points;
  const std::vector<Eigen::Vector3d> one_point{Eigen::Vector3d::Zero()};
  const std::vector<Eigen::Vector3d> four_derivatives(4, Eigen::Vector3d::Zero());

  EXPECT_FALSE(FLAG_Race::UniformBspline::parameterizeToBspline(
      0.0, one_point, four_derivatives, control_points));
  EXPECT_FALSE(FLAG_Race::UniformBspline::parameterizeToBspline(
      0.2, one_point, four_derivatives, control_points));
}

TEST(FastPlannerParameterization, OptimizerAcceptsParameterizedControlPoints) {
  Eigen::MatrixXd control_points(11, 3);
  for (int i = 0; i < control_points.rows(); ++i) {
    control_points.row(i) << 0.1 * i, -0.05 * i, 1.0;
  }

  FLAG_Race::bspline_optimizer optimizer;
  optimizer.setDimandOrder(3, 3);
  ASSERT_TRUE(optimizer.setInitialControlPoints(control_points, 0.2));
  EXPECT_EQ(optimizer.cps_num_, 11);
  EXPECT_NEAR(optimizer.bspline_interval_, 0.2, 1e-12);
  EXPECT_NEAR(optimizer.beta_, 5.0, 1e-12);
  EXPECT_NEAR((optimizer.control_points_ - control_points).norm(), 0.0, 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
